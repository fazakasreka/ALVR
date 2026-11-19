//! An implicit OpenXR API layer that gives OpenXR applications the QR codes an ALVR headset sees.
//!
//! On the PC, ALVR is a SteamVR driver, and SteamVR's OpenXR runtime does not implement the
//! spatial entity extensions. This layer, loaded by the OpenXR loader into every application,
//! implements XR_EXT_spatial_entity and XR_EXT_spatial_marker_tracking (QR codes) on top of the
//! runtime, plus XR_EXT_future, which they depend on, when the runtime lacks it.
//!
//! ```text
//! app → OpenXR loader → this layer → SteamVR OpenXR runtime → vrserver ← ALVR driver
//!                           ↑                                                │
//!                           └──────────── named pipe (alvr_marker_ipc) ──────┘
//! ```
//!
//! The ALVR driver owns the markers: it receives them from the headset and publishes their poses
//! in the space it submits the headset pose to SteamVR in, the runtime's STAGE space, with the
//! time they are for on the performance counter clock. The layer maps that state onto the
//! extensions: it locates STAGE in the application's spaces, answers for the requested times with
//! XR_KHR_win32_convert_performance_counter_time, and tells the driver which tracker options the
//! application's spatial contexts ask for.
//!
//! ALVR also recenters: it moves the space it submits poses to SteamVR in, which SteamVR does not
//! see, so every application's reference spaces move without the event a runtime must queue. The
//! layer queues XrEventDataReferenceSpaceChangePending for those moves, for every application.
//!
//! Otherwise the layer stays out of the way of applications that do not enable these extensions,
//! and of runtimes that implement them themselves: then their calls go straight to the runtime.

mod history;
mod installation;
mod log;
mod math;
mod session;
mod spatial;
#[cfg(test)]
mod test_util;
#[cfg(test)]
mod tests;

use alvr_marker_ipc::{self as ipc, Capabilities, Client};
use history::MarkerHistory;
use log::log;
use openxr_sys::{self as xr, Handle, loader, pfn};
use session::OriginChange;
use std::{
    collections::{HashMap, HashSet},
    ffi::{CStr, CString, c_char},
    mem, ptr, slice,
    sync::{LazyLock, Mutex, MutexGuard},
};

const LAYER_NAME: &CStr = c"XR_APILAYER_ALVR_marker_tracking";

const SPATIAL_ENTITY_EXTENSION: &CStr = c"XR_EXT_spatial_entity";
const MARKER_TRACKING_EXTENSION: &CStr = c"XR_EXT_spatial_marker_tracking";
const SPATIAL_EXTENSIONS: [&CStr; 2] = [SPATIAL_ENTITY_EXTENSION, MARKER_TRACKING_EXTENSION];
const FUTURE_EXTENSION: &CStr = c"XR_EXT_future";
const TIME_EXTENSION: &CStr = c"XR_KHR_win32_convert_performance_counter_time";

macro_rules! void_fn {
    ($f:expr, $ty:ty) => {
        unsafe { ::std::mem::transmute::<$ty, pfn::VoidFunction>($f as $ty) }
    };
}
pub(crate) use void_fn;

/// A function of the runtime (or the next layer), if it has it.
unsafe fn load_fn<T>(
    gipa: pfn::GetInstanceProcAddr,
    instance: xr::Instance,
    name: &CStr,
) -> Option<T> {
    let mut function = None;
    if unsafe { gipa(instance, name.as_ptr(), &mut function) } != xr::Result::SUCCESS {
        return None;
    }

    function.map(|function| unsafe { mem::transmute_copy(&function) })
}

/// Runtime functions the layer calls itself.
#[derive(Clone, Copy)]
struct Downstream {
    get_instance_proc_addr: pfn::GetInstanceProcAddr,
    destroy_instance: pfn::DestroyInstance,
    get_system: pfn::GetSystem,
    poll_event: pfn::PollEvent,
    create_session: pfn::CreateSession,
    destroy_session: pfn::DestroySession,
    create_reference_space: pfn::CreateReferenceSpace,
    destroy_space: pfn::DestroySpace,
    locate_space: pfn::LocateSpace,
    enumerate_reference_spaces: pfn::EnumerateReferenceSpaces,
    /// Only when the runtime implements XR_EXT_future.
    poll_future: Option<pfn::PollFutureEXT>,
    /// Only when the runtime implements XR_EXT_future.
    cancel_future: Option<pfn::CancelFutureEXT>,
    /// Only when XR_KHR_win32_convert_performance_counter_time is enabled.
    convert_time: Option<pfn::ConvertTimeToWin32PerformanceCounterKHR>,
    /// Only when XR_KHR_win32_convert_performance_counter_time is enabled.
    convert_counter: Option<pfn::ConvertWin32PerformanceCounterToTimeKHR>,
}

impl Downstream {
    unsafe fn load(
        gipa: pfn::GetInstanceProcAddr,
        instance: xr::Instance,
        has_time_extension: bool,
    ) -> Option<Self> {
        unsafe {
            Some(Self {
                get_instance_proc_addr: gipa,
                destroy_instance: load_fn(gipa, instance, c"xrDestroyInstance")?,
                get_system: load_fn(gipa, instance, c"xrGetSystem")?,
                poll_event: load_fn(gipa, instance, c"xrPollEvent")?,
                create_session: load_fn(gipa, instance, c"xrCreateSession")?,
                destroy_session: load_fn(gipa, instance, c"xrDestroySession")?,
                create_reference_space: load_fn(gipa, instance, c"xrCreateReferenceSpace")?,
                destroy_space: load_fn(gipa, instance, c"xrDestroySpace")?,
                locate_space: load_fn(gipa, instance, c"xrLocateSpace")?,
                enumerate_reference_spaces: load_fn(gipa, instance, c"xrEnumerateReferenceSpaces")?,
                poll_future: load_fn(gipa, instance, c"xrPollFutureEXT"),
                cancel_future: load_fn(gipa, instance, c"xrCancelFutureEXT"),
                convert_time: has_time_extension
                    .then(|| load_fn(gipa, instance, c"xrConvertTimeToWin32PerformanceCounterKHR"))
                    .flatten(),
                convert_counter: has_time_extension
                    .then(|| load_fn(gipa, instance, c"xrConvertWin32PerformanceCounterToTimeKHR"))
                    .flatten(),
            })
        }
    }
}

/// The functions the layer implements for an instance.
#[derive(Clone, Copy, Default)]
struct Hooks {
    /// Of XR_EXT_spatial_entity and XR_EXT_spatial_marker_tracking.
    spatial: bool,
    /// Of XR_EXT_future, for the layer's futures, also when the runtime implements it.
    future: bool,
    /// xrCreateSession, xrDestroySession and xrPollEvent, which report recentering and recommend
    /// spatial discoveries.
    session: bool,
}

struct InstanceData {
    downstream: Downstream,
    hooks: Hooks,
    /// The application enabled XR_EXT_spatial_marker_tracking, and the ALVR copy of the layer is a
    /// registered SteamVR driver.
    qr_codes: bool,
    /// The systems xrGetSystem returned, the ones the spatial functions accept.
    systems: HashSet<u64>,
}

/// The markers sent by the ALVR driver.
struct Markers {
    client: Client,
    was_connected: bool,
    history: MarkerHistory,
    /// The origin the driver sent last.
    origin: Option<ipc::OriginChange>,
}

impl Default for Markers {
    fn default() -> Self {
        Self {
            client: Client::new_named(&pipe_name()),
            was_connected: false,
            history: MarkerHistory::default(),
            origin: None,
        }
    }
}

impl Markers {
    /// Takes in what the driver sent. Returns what the headset connected last can do, or `None`
    /// while the driver is not running, and how the origin changed since the last call.
    fn refresh(&mut self) -> (Option<Capabilities>, Vec<OriginChange>) {
        let capabilities = self.client.read().map(|snapshot| snapshot.capabilities);
        let mut changes = Vec::new();
        for received in self.client.take_received() {
            // Before the snapshot's poses, which are in the new space already
            if let Some(previous) = self.origin.replace(received.origin)
                && let Some(change) = OriginChange::between(previous, received.origin)
            {
                if change.moved {
                    self.history.move_origin(change.pose_in_previous);
                }
                changes.push(change);
            }
            self.history.add(&received);
        }
        if capabilities.is_none() {
            self.history.clear();
        }

        let is_connected = self.client.is_connected();
        if is_connected != self.was_connected {
            self.was_connected = is_connected;
            log!(
                "{}",
                if is_connected {
                    "Connected to the ALVR driver"
                } else {
                    "Disconnected from the ALVR driver"
                }
            );
        }

        (capabilities, changes)
    }
}

#[cfg(not(test))]
fn pipe_name() -> String {
    alvr_marker_ipc::PIPE_NAME.to_owned()
}
#[cfg(test)]
use tests::pipe_name;

#[derive(Default)]
struct State {
    instances: HashMap<u64, InstanceData>,
    sessions: HashMap<u64, session::SessionData>,
    contexts: HashMap<u64, spatial::ContextData>,
    snapshots: HashMap<u64, spatial::SnapshotData>,
    entities: HashMap<u64, spatial::EntityData>,
    futures: HashMap<u64, spatial::FutureData>,
    markers: Markers,
    /// For calls on an instance the layer did not see created, which should not happen.
    last_get_instance_proc_addr: Option<pfn::GetInstanceProcAddr>,
    next_handle: u64,
}

/// In every handle and future of the layer: recognisable in a debugger, and tells the layer's
/// futures apart from the runtime's.
const HANDLE_PREFIX: u64 = 0x414C_5652_0000_0000;

/// Futures and handles of the layer have this in their upper half.
fn is_layer_handle(handle: u64) -> bool {
    handle >> 32 == HANDLE_PREFIX >> 32
}

impl State {
    /// A value for a handle or future the layer creates. Never 0, and never reused.
    fn new_handle(&mut self) -> u64 {
        self.next_handle += 1;
        HANDLE_PREFIX | self.next_handle
    }

    fn downstream(&self, instance: u64) -> Option<Downstream> {
        self.instances.get(&instance).map(|data| data.downstream)
    }

    /// The runtime's xrDestroyInstance.
    fn remove_instance(&mut self, instance: u64) -> Option<pfn::DestroyInstance> {
        let data = self.instances.remove(&instance)?;

        let sessions = self
            .sessions
            .iter()
            .filter(|(_, session)| session.instance == instance)
            .map(|(&handle, _)| handle)
            .collect::<Vec<_>>();
        for session in sessions {
            self.remove_session(session);
        }
        self.futures.retain(|_, future| future.instance != instance);

        Some(data.downstream.destroy_instance)
    }
}

static STATE: LazyLock<Mutex<State>> = LazyLock::new(Mutex::default);

fn state() -> MutexGuard<'static, State> {
    STATE.lock().unwrap_or_else(|e| e.into_inner())
}

/// # Safety
/// Called by the OpenXR loader, with pointers that follow the loader interface.
#[unsafe(no_mangle)]
pub unsafe extern "system" fn xrNegotiateLoaderApiLayerInterface(
    loader_info: *const loader::XrNegotiateLoaderInfo,
    layer_name: *const c_char,
    request: *mut loader::XrNegotiateApiLayerRequest,
) -> xr::Result {
    if loader_info.is_null() || layer_name.is_null() || request.is_null() {
        return xr::Result::ERROR_INITIALIZATION_FAILED;
    }
    let (loader_info, request) = unsafe { (&*loader_info, &mut *request) };

    if unsafe { CStr::from_ptr(layer_name) } != LAYER_NAME
        || loader_info.ty != loader::XrNegotiateLoaderInfo::TYPE
        || loader_info.struct_version != loader::XrNegotiateLoaderInfo::VERSION
        || loader_info.struct_size != mem::size_of::<loader::XrNegotiateLoaderInfo>()
        || request.ty != loader::XrNegotiateApiLayerRequest::TYPE
        || request.struct_version != loader::XrNegotiateApiLayerRequest::VERSION
        || request.struct_size != mem::size_of::<loader::XrNegotiateApiLayerRequest>()
        || loader_info.min_interface_version > loader::CURRENT_LOADER_API_LAYER_VERSION
        || loader_info.max_interface_version < loader::CURRENT_LOADER_API_LAYER_VERSION
    {
        return xr::Result::ERROR_INITIALIZATION_FAILED;
    }

    request.layer_interface_version = loader::CURRENT_LOADER_API_LAYER_VERSION;
    request.layer_api_version = xr::CURRENT_API_VERSION.min(loader_info.max_api_version);
    request.get_instance_proc_addr = Some(get_instance_proc_addr);
    request.create_api_layer_instance = Some(create_api_layer_instance);

    xr::Result::SUCCESS
}

/// What the layer implements for an instance, and whether it enables the time conversion on the
/// runtime for it.
#[derive(Clone, Copy)]
struct Plan {
    provides_spatial: bool,
    provides_future: bool,
    add_time: bool,
}

impl Plan {
    /// What the runtime implements itself is left to it.
    fn new(
        wants: impl Fn(&CStr) -> bool,
        alvr_registered: bool,
        runtime_has: impl Fn(&CStr) -> bool,
    ) -> Self {
        let provides_spatial = SPATIAL_EXTENSIONS.into_iter().any(&wants)
            && !SPATIAL_EXTENSIONS.into_iter().all(&runtime_has);
        let provides_future = wants(FUTURE_EXTENSION) && !runtime_has(FUTURE_EXTENSION);
        // Without it, markers are located at their latest pose, and recentering is not reported
        let add_time = (provides_spatial || alvr_registered)
            && !wants(TIME_EXTENSION)
            && runtime_has(TIME_EXTENSION);

        Self {
            provides_spatial,
            provides_future,
            add_time,
        }
    }

    /// The extensions to enable on the runtime, from the ones the application asked for.
    fn extensions_for_runtime(&self, requested: &[*const c_char]) -> Vec<*const c_char> {
        let provided = |name: &CStr| {
            (self.provides_spatial && SPATIAL_EXTENSIONS.contains(&name))
                || (self.provides_future && name == FUTURE_EXTENSION)
        };
        let mut names = requested
            .iter()
            .copied()
            .filter(|&name| !provided(unsafe { CStr::from_ptr(name) }))
            .collect::<Vec<_>>();
        if self.add_time {
            names.push(TIME_EXTENSION.as_ptr());
        }

        names
    }
}

unsafe extern "system" fn create_api_layer_instance(
    info: *const xr::InstanceCreateInfo,
    layer_info: *const loader::ApiLayerCreateInfo,
    instance: *mut xr::Instance,
) -> xr::Result {
    if info.is_null() || layer_info.is_null() || instance.is_null() {
        return xr::Result::ERROR_VALIDATION_FAILURE;
    }
    let (info, layer_info) = unsafe { (&*info, &*layer_info) };

    let next_info = layer_info.next_info;
    if next_info.is_null() {
        return xr::Result::ERROR_INITIALIZATION_FAILED;
    }
    let next_info = unsafe { &*next_info };
    if next_info.ty != loader::XrApiLayerNextInfo::TYPE
        || unsafe { CStr::from_ptr(next_info.layer_name.as_ptr()) } != LAYER_NAME
    {
        return xr::Result::ERROR_INITIALIZATION_FAILED;
    }
    let next_gipa = next_info.next_get_instance_proc_addr;
    let next_create = next_info.next_create_api_layer_instance;
    // The next layer (or the runtime) gets the rest of the chain
    let mut next_layer_info = *layer_info;
    next_layer_info.next_info = next_info.next;

    let requested = if info.enabled_extension_count == 0 || info.enabled_extension_names.is_null() {
        Vec::new()
    } else {
        unsafe {
            slice::from_raw_parts(
                info.enabled_extension_names,
                info.enabled_extension_count as usize,
            )
        }
        .to_vec()
    };
    let wants = |extension: &CStr| {
        requested
            .iter()
            .any(|&name| unsafe { CStr::from_ptr(name) } == extension)
    };
    let alvr_registered = installation::alvr_is_registered();
    if !alvr_registered {
        log!("ALVR is not registered with SteamVR: no QR codes, and recentering is not reported");
    }

    let create = |names: &[*const c_char]| {
        let mut info = *info;
        info.enabled_extension_count = names.len() as u32;
        info.enabled_extension_names = if names.is_empty() {
            ptr::null()
        } else {
            names.as_ptr()
        };

        unsafe { next_create(&info, &next_layer_info, instance) }
    };

    // The runtime is asked what it implements rather than tried with the extensions: SteamVR
    // asserts when an instance is created again after a failed attempt, which stops applications
    // in a debugger.
    let (plan, result) = match unsafe { runtime_extensions(next_gipa) } {
        Some(available) => {
            let plan = Plan::new(wants, alvr_registered, |name| {
                available.iter().any(|ext| ext.as_c_str() == name)
            });
            (plan, create(&plan.extensions_for_runtime(&requested)))
        }
        // Found out by trying, then: each refusal is put down to the first extension in this
        // order that the runtime may lack
        None => {
            let mut lacking = Vec::new();
            loop {
                let plan = Plan::new(wants, alvr_registered, |name| !lacking.contains(&name));
                let names = plan.extensions_for_runtime(&requested);
                let result = create(&names);
                let suspect = [
                    SPATIAL_ENTITY_EXTENSION,
                    MARKER_TRACKING_EXTENSION,
                    FUTURE_EXTENSION,
                    TIME_EXTENSION,
                ]
                .into_iter()
                .find(|&extension| {
                    !lacking.contains(&extension)
                        && names
                            .iter()
                            .any(|&name| unsafe { CStr::from_ptr(name) } == extension)
                });
                match suspect {
                    Some(extension) if result == xr::Result::ERROR_EXTENSION_NOT_PRESENT => {
                        lacking.push(extension);
                    }
                    _ => break (plan, result),
                }
            }
        }
    };
    if result != xr::Result::SUCCESS {
        return result;
    }

    let instance = unsafe { *instance };
    let Some(downstream) =
        (unsafe { Downstream::load(next_gipa, instance, wants(TIME_EXTENSION) || plan.add_time) })
    else {
        log!("The runtime lacks core functions");
        if let Some(destroy) =
            unsafe { load_fn::<pfn::DestroyInstance>(next_gipa, instance, c"xrDestroyInstance") }
        {
            unsafe { destroy(instance) };
        }
        return xr::Result::ERROR_RUNTIME_FAILURE;
    };

    let wants_spatial = SPATIAL_EXTENSIONS.into_iter().any(wants);
    if plan.provides_spatial {
        log!(
            "Providing XR_EXT_spatial_entity and XR_EXT_spatial_marker_tracking{}",
            if plan.provides_future {
                ", and XR_EXT_future"
            } else {
                ""
            }
        );
    } else if wants_spatial {
        log!("The runtime implements the spatial entity extensions itself");
    }

    let mut state = state();
    state.last_get_instance_proc_addr = Some(next_gipa);
    state.instances.insert(
        instance.into_raw(),
        InstanceData {
            downstream,
            hooks: Hooks {
                spatial: plan.provides_spatial,
                future: (plan.provides_spatial || plan.provides_future) && wants(FUTURE_EXTENSION),
                session: plan.provides_spatial || alvr_registered,
            },
            qr_codes: wants(MARKER_TRACKING_EXTENSION) && alvr_registered,
            systems: HashSet::new(),
        },
    );

    xr::Result::SUCCESS
}

/// The instance extensions of the runtime and the layers after this one, or `None` if they cannot
/// be listed.
unsafe fn runtime_extensions(next_gipa: pfn::GetInstanceProcAddr) -> Option<Vec<CString>> {
    let enumerate: pfn::EnumerateInstanceExtensionProperties = unsafe {
        load_fn(
            next_gipa,
            xr::Instance::NULL,
            c"xrEnumerateInstanceExtensionProperties",
        )
    }?;

    let mut count = 0;
    let result = unsafe { enumerate(ptr::null(), 0, &mut count, ptr::null_mut()) };
    if result != xr::Result::SUCCESS {
        return None;
    }
    let mut properties = vec![
        xr::ExtensionProperties {
            ty: xr::ExtensionProperties::TYPE,
            next: ptr::null_mut(),
            extension_name: [0; xr::MAX_EXTENSION_NAME_SIZE],
            extension_version: 0,
        };
        count as usize
    ];
    let result = unsafe { enumerate(ptr::null(), count, &mut count, properties.as_mut_ptr()) };
    if result != xr::Result::SUCCESS {
        return None;
    }

    Some(
        properties
            .iter()
            .take(count as usize)
            .map(|ext| unsafe { CStr::from_ptr(ext.extension_name.as_ptr()) }.to_owned())
            .collect(),
    )
}

unsafe extern "system" fn get_instance_proc_addr(
    instance: xr::Instance,
    name: *const c_char,
    function: *mut Option<pfn::VoidFunction>,
) -> xr::Result {
    if name.is_null() || function.is_null() {
        return xr::Result::ERROR_VALIDATION_FAILURE;
    }
    let name_str = unsafe { CStr::from_ptr(name) };

    let (next, hooks) = {
        let state = state();
        match state.instances.get(&instance.into_raw()) {
            Some(data) => (data.downstream.get_instance_proc_addr, data.hooks),
            None => match state.last_get_instance_proc_addr {
                Some(next) => (next, Hooks::default()),
                None => return xr::Result::ERROR_HANDLE_INVALID,
            },
        }
    };

    let hooked: [(bool, fn(&CStr) -> Option<pfn::VoidFunction>); 3] = [
        (hooks.spatial, spatial::function),
        (hooks.future, spatial::future_function),
        (hooks.session, session::function),
    ];
    let ours = match name_str.to_bytes() {
        b"xrGetInstanceProcAddr" => {
            Some(void_fn!(get_instance_proc_addr, pfn::GetInstanceProcAddr))
        }
        b"xrDestroyInstance" => Some(void_fn!(destroy_instance, pfn::DestroyInstance)),
        _ => hooked
            .into_iter()
            .find_map(|(hooked, function)| hooked.then(|| function(name_str)).flatten()),
    };
    if let Some(ours) = ours {
        unsafe { *function = Some(ours) };
        return xr::Result::SUCCESS;
    }

    unsafe { next(instance, name, function) }
}

unsafe extern "system" fn destroy_instance(instance: xr::Instance) -> xr::Result {
    let destroy = {
        let mut state = state();
        let Some(destroy) = state.remove_instance(instance.into_raw()) else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };
        state.sync_requests();
        destroy
    };

    // The runtime destroys the reference spaces the layer created along with the instance
    unsafe { destroy(instance) }
}
