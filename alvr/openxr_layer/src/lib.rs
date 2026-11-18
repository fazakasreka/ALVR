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
//! The layer stays out of the way of applications that do not enable these extensions, and of
//! runtimes that implement them themselves: then every call goes straight to the runtime.

mod history;
mod log;
mod math;
mod spatial;
#[cfg(test)]
mod tests;

use alvr_marker_ipc::{Capabilities, Client};
use history::MarkerHistory;
use log::log;
use openxr_sys::{self as xr, Handle, loader, pfn};
use std::{
    collections::HashMap,
    ffi::{CStr, CString, c_char},
    mem, ptr, slice,
    sync::{LazyLock, Mutex, MutexGuard},
};

const LAYER_NAME: &CStr = c"XR_APILAYER_ALVR_marker_tracking";

const SPATIAL_EXTENSIONS: [&CStr; 2] =
    [c"XR_EXT_spatial_entity", c"XR_EXT_spatial_marker_tracking"];
const FUTURE_EXTENSION: &CStr = c"XR_EXT_future";
/// Enabled on the runtime too when the layer provides the spatial extensions, to answer for the
/// times applications ask for.
const TIME_EXTENSION: &CStr = c"XR_KHR_win32_convert_performance_counter_time";

macro_rules! void_fn {
    ($f:expr, $ty:ty) => {
        unsafe { mem::transmute::<$ty, pfn::VoidFunction>($f as $ty) }
    };
}
pub(crate) use void_fn;

/// Runtime functions the layer calls itself.
#[derive(Clone, Copy)]
struct Downstream {
    get_instance_proc_addr: pfn::GetInstanceProcAddr,
    destroy_instance: pfn::DestroyInstance,
    poll_event: pfn::PollEvent,
    create_session: pfn::CreateSession,
    destroy_session: pfn::DestroySession,
    create_reference_space: pfn::CreateReferenceSpace,
    destroy_space: pfn::DestroySpace,
    locate_space: pfn::LocateSpace,
    // Only when the runtime implements XR_EXT_future
    poll_future: Option<pfn::PollFutureEXT>,
    cancel_future: Option<pfn::CancelFutureEXT>,
    // Only when XR_KHR_win32_convert_performance_counter_time is enabled
    convert_time: Option<pfn::ConvertTimeToWin32PerformanceCounterKHR>,
}

impl Downstream {
    unsafe fn load(
        gipa: pfn::GetInstanceProcAddr,
        instance: xr::Instance,
        has_time_extension: bool,
    ) -> Option<Self> {
        unsafe fn get<T: Copy>(
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

        unsafe {
            Some(Self {
                get_instance_proc_addr: gipa,
                destroy_instance: get(gipa, instance, c"xrDestroyInstance")?,
                poll_event: get(gipa, instance, c"xrPollEvent")?,
                create_session: get(gipa, instance, c"xrCreateSession")?,
                destroy_session: get(gipa, instance, c"xrDestroySession")?,
                create_reference_space: get(gipa, instance, c"xrCreateReferenceSpace")?,
                destroy_space: get(gipa, instance, c"xrDestroySpace")?,
                locate_space: get(gipa, instance, c"xrLocateSpace")?,
                poll_future: get(gipa, instance, c"xrPollFutureEXT"),
                cancel_future: get(gipa, instance, c"xrCancelFutureEXT"),
                convert_time: if has_time_extension {
                    get(gipa, instance, c"xrConvertTimeToWin32PerformanceCounterKHR")
                } else {
                    None
                },
            })
        }
    }
}

struct InstanceData {
    downstream: Downstream,
    /// XR_EXT_spatial_entity and XR_EXT_spatial_marker_tracking are implemented by the layer.
    provides_spatial: bool,
    /// XR_EXT_future is implemented by the layer.
    provides_future: bool,
}

/// The markers sent by the ALVR driver.
struct Markers {
    client: Client,
    was_connected: bool,
    history: MarkerHistory,
}

impl Default for Markers {
    fn default() -> Self {
        Self {
            client: Client::new_named(&pipe_name()),
            was_connected: false,
            history: MarkerHistory::default(),
        }
    }
}

impl Markers {
    /// Takes in what the driver sent. Returns what the headset connected last can do, or `None`
    /// while the driver is not running.
    fn refresh(&mut self) -> Option<Capabilities> {
        let snapshot = self.client.read();
        for received in self.client.take_received() {
            self.history.add(&received);
        }
        if snapshot.is_none() {
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

        snapshot.map(|snapshot| snapshot.capabilities)
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
    sessions: HashMap<u64, spatial::SessionData>,
    contexts: HashMap<u64, spatial::ContextData>,
    snapshots: HashMap<u64, spatial::SnapshotData>,
    entities: HashMap<u64, spatial::EntityData>,
    futures: HashMap<u64, spatial::FutureData>,
    markers: Markers,
    /// For calls on an instance the layer did not see created, which should not happen.
    last_get_instance_proc_addr: Option<pfn::GetInstanceProcAddr>,
    next_handle: u64,
}

impl State {
    /// A value for a handle or future the layer creates. Never 0, and never reused.
    fn new_handle(&mut self) -> u64 {
        self.next_handle += 1;
        // A recognisable prefix, to tell the layer's handles apart in a debugger
        0x414C_5652_0000_0000 | self.next_handle
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
    let is_spatial =
        |name: *const c_char| SPATIAL_EXTENSIONS.contains(&unsafe { CStr::from_ptr(name) });
    let is_future = |name: *const c_char| unsafe { CStr::from_ptr(name) } == FUTURE_EXTENSION;
    let wants_spatial = requested.iter().any(|&name| is_spatial(name));
    let wants_future = requested.iter().any(|&name| is_future(name));
    let wants_time = requested
        .iter()
        .any(|&name| unsafe { CStr::from_ptr(name) } == TIME_EXTENSION);

    let create = |strip_spatial: bool, strip_future: bool, add_time: bool| {
        let mut names = requested
            .iter()
            .copied()
            .filter(|&name| {
                !(strip_spatial && is_spatial(name)) && !(strip_future && is_future(name))
            })
            .collect::<Vec<_>>();
        if add_time {
            names.push(TIME_EXTENSION.as_ptr());
        }
        let mut info = *info;
        info.enabled_extension_count = names.len() as u32;
        info.enabled_extension_names = if names.is_empty() {
            ptr::null()
        } else {
            names.as_ptr()
        };

        unsafe { next_create(&info, &next_layer_info, instance) }
    };

    // What the runtime implements itself is left to it. The runtime is asked what it implements
    // rather than tried with the extensions: SteamVR asserts when an instance is created again
    // after a failed attempt, which stops applications in a debugger.
    let (provides_spatial, provides_future, add_time, result) =
        match unsafe { runtime_extensions(next_gipa) } {
            Some(available) => {
                let has = |name: &CStr| available.iter().any(|ext| ext.as_c_str() == name);
                let provides_spatial =
                    wants_spatial && !SPATIAL_EXTENSIONS.iter().all(|&name| has(name));
                let provides_future = wants_future && !has(FUTURE_EXTENSION);
                // Without it, markers are located at their latest pose
                let add_time = provides_spatial && !wants_time && has(TIME_EXTENSION);

                let result = create(provides_spatial, provides_future, add_time);
                (provides_spatial, provides_future, add_time, result)
            }
            // Found out by trying, then
            None => {
                let mut provides_spatial = false;
                let mut provides_future = false;
                let mut add_time = false;
                let mut result = create(false, false, false);
                if result == xr::Result::ERROR_EXTENSION_NOT_PRESENT && wants_spatial {
                    provides_spatial = true;
                    add_time = !wants_time;
                    result = create(true, false, add_time);
                }
                if result == xr::Result::ERROR_EXTENSION_NOT_PRESENT && wants_future {
                    provides_future = true;
                    result = create(provides_spatial, true, add_time);
                }
                if result == xr::Result::ERROR_EXTENSION_NOT_PRESENT && add_time {
                    add_time = false;
                    result = create(provides_spatial, provides_future, false);
                }
                (provides_spatial, provides_future, add_time, result)
            }
        };
    if result != xr::Result::SUCCESS {
        return result;
    }

    let instance = unsafe { *instance };
    let Some(downstream) =
        (unsafe { Downstream::load(next_gipa, instance, wants_time || add_time) })
    else {
        log!("The runtime lacks core functions");
        let mut destroy = None;
        unsafe { next_gipa(instance, c"xrDestroyInstance".as_ptr(), &mut destroy) };
        if let Some(destroy) = destroy {
            let destroy: pfn::DestroyInstance = unsafe { mem::transmute(destroy) };
            unsafe { destroy(instance) };
        }
        return xr::Result::ERROR_RUNTIME_FAILURE;
    };

    if provides_spatial {
        log!(
            "Providing XR_EXT_spatial_entity and XR_EXT_spatial_marker_tracking{}",
            if provides_future {
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
            provides_spatial,
            provides_future,
        },
    );

    xr::Result::SUCCESS
}

/// The instance extensions of the runtime and the layers after this one, or `None` if they cannot
/// be listed.
unsafe fn runtime_extensions(next_gipa: pfn::GetInstanceProcAddr) -> Option<Vec<CString>> {
    let mut function = None;
    let result = unsafe {
        next_gipa(
            xr::Instance::NULL,
            c"xrEnumerateInstanceExtensionProperties".as_ptr(),
            &mut function,
        )
    };
    if result != xr::Result::SUCCESS {
        return None;
    }
    let enumerate: pfn::EnumerateInstanceExtensionProperties = unsafe { mem::transmute(function?) };

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

    let (next, provides_spatial, provides_future) = {
        let state = state();
        match state.instances.get(&instance.into_raw()) {
            Some(data) => (
                data.downstream.get_instance_proc_addr,
                data.provides_spatial,
                data.provides_future,
            ),
            None => match state.last_get_instance_proc_addr {
                Some(next) => (next, false, false),
                None => return xr::Result::ERROR_HANDLE_INVALID,
            },
        }
    };

    let ours = match name_str.to_bytes() {
        b"xrGetInstanceProcAddr" => {
            Some(void_fn!(get_instance_proc_addr, pfn::GetInstanceProcAddr))
        }
        b"xrDestroyInstance" => Some(void_fn!(destroy_instance, pfn::DestroyInstance)),
        _ if provides_spatial => spatial::function(name_str),
        _ if provides_future => spatial::future_function(name_str),
        _ => None,
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
        let Some((destroy, provided_spatial)) = state.remove_instance(instance.into_raw()) else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };
        // Applications that did not use the layer never talk to the driver
        if provided_spatial {
            state.sync_requests();
        }
        destroy
    };

    // The runtime destroys the stage spaces the layer created along with the instance
    unsafe { destroy(instance) }
}

impl State {
    /// The runtime's xrDestroyInstance, and whether the layer provided the spatial extensions.
    fn remove_instance(&mut self, instance: u64) -> Option<(pfn::DestroyInstance, bool)> {
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

        Some((data.downstream.destroy_instance, data.provides_spatial))
    }
}
