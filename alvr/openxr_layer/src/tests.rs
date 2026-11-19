//! The layer driven the way the OpenXR loader and an application drive it, on top of a fake
//! runtime that, like SteamVR, implements none of the extensions but the time conversion.

use crate::{LAYER_NAME, TIME_EXTENSION, xrNegotiateLoaderApiLayerInterface};
use alvr_marker_ipc::{
    Capabilities, FLAG_CLIENT_CONNECTED, Marker, Origin, Payload, Pose, Server, Stream,
    TrackingConfig, now_ns, ns_to_performance_counter, performance_counter_to_ns,
};
use openxr_sys::{self as xr, Handle, loader, pfn};
use std::{
    ffi::{CStr, c_char},
    mem, ptr,
    sync::{
        Mutex, MutexGuard,
        atomic::{AtomicBool, AtomicUsize, Ordering},
        mpsc::{self, Receiver},
    },
    thread,
    time::{Duration, Instant},
};

pub fn pipe_name() -> String {
    format!("ALVR_Markers_layer_test_{}", std::process::id())
}

// The layer's state is global, so tests run one at a time
static SERIAL: Mutex<()> = Mutex::new(());

fn serial() -> MutexGuard<'static, ()> {
    SERIAL.lock().unwrap_or_else(|e| e.into_inner())
}

const INSTANCE: u64 = 0x1000;
const SESSION: u64 = 0x2000;
const STAGE: u64 = 0x3000;
/// The application's base space. The fake runtime puts the stage 1.5 m below it.
const LOCAL: u64 = 0x4000;
const STAGE_IN_LOCAL: xr::Posef = xr::Posef {
    orientation: xr::Quaternionf::IDENTITY,
    position: xr::Vector3f {
        x: 0.0,
        y: -1.5,
        z: 0.0,
    },
};

#[derive(Default)]
struct Runtime {
    enabled_extensions: Vec<String>,
    created_spaces: Vec<xr::ReferenceSpaceType>,
    destroyed_spaces: Vec<u64>,
}

static RUNTIME: Mutex<Option<Runtime>> = Mutex::new(None);

fn runtime() -> MutexGuard<'static, Option<Runtime>> {
    RUNTIME.lock().unwrap_or_else(|e| e.into_inner())
}

const RUNTIME_EXTENSIONS: [&CStr; 1] = [TIME_EXTENSION];
/// xrCreateInstance calls on the runtime, failed ones too.
static CREATE_ATTEMPTS: AtomicUsize = AtomicUsize::new(0);
/// Whether the runtime lists its extensions, as SteamVR does.
static LISTS_EXTENSIONS: AtomicBool = AtomicBool::new(true);

unsafe extern "system" fn runtime_enumerate_extensions(
    layer: *const c_char,
    capacity: u32,
    count: *mut u32,
    properties: *mut xr::ExtensionProperties,
) -> xr::Result {
    assert!(layer.is_null());
    unsafe { *count = RUNTIME_EXTENSIONS.len() as u32 };
    if capacity == 0 {
        return xr::Result::SUCCESS;
    }
    for (i, name) in RUNTIME_EXTENSIONS.iter().enumerate() {
        let property = unsafe { &mut *properties.add(i) };
        for (j, byte) in name.to_bytes_with_nul().iter().enumerate() {
            property.extension_name[j] = *byte as c_char;
        }
        property.extension_version = 1;
    }
    xr::Result::SUCCESS
}

unsafe extern "system" fn runtime_create_instance(
    info: *const xr::InstanceCreateInfo,
    _layer_info: *const loader::ApiLayerCreateInfo,
    instance: *mut xr::Instance,
) -> xr::Result {
    CREATE_ATTEMPTS.fetch_add(1, Ordering::SeqCst);
    let info = unsafe { &*info };
    let names = (0..info.enabled_extension_count as usize)
        .map(|i| unsafe { CStr::from_ptr(*info.enabled_extension_names.add(i)) })
        .collect::<Vec<_>>();
    if names.iter().any(|name| !RUNTIME_EXTENSIONS.contains(name)) {
        return xr::Result::ERROR_EXTENSION_NOT_PRESENT;
    }

    *runtime() = Some(Runtime {
        enabled_extensions: names
            .iter()
            .map(|name| name.to_string_lossy().into())
            .collect(),
        ..Default::default()
    });
    unsafe { *instance = xr::Instance::from_raw(INSTANCE) };

    xr::Result::SUCCESS
}

unsafe extern "system" fn runtime_destroy_instance(_: xr::Instance) -> xr::Result {
    xr::Result::SUCCESS
}

unsafe extern "system" fn runtime_get_system(
    _: xr::Instance,
    _: *const xr::SystemGetInfo,
    system: *mut xr::SystemId,
) -> xr::Result {
    unsafe { *system = SYSTEM };
    xr::Result::SUCCESS
}

unsafe extern "system" fn runtime_poll_event(
    _: xr::Instance,
    _: *mut xr::EventDataBuffer,
) -> xr::Result {
    xr::Result::EVENT_UNAVAILABLE
}

unsafe extern "system" fn runtime_create_session(
    _: xr::Instance,
    _: *const xr::SessionCreateInfo,
    session: *mut xr::Session,
) -> xr::Result {
    unsafe { *session = xr::Session::from_raw(SESSION) };
    xr::Result::SUCCESS
}

unsafe extern "system" fn runtime_destroy_session(_: xr::Session) -> xr::Result {
    xr::Result::SUCCESS
}

unsafe extern "system" fn runtime_create_reference_space(
    _: xr::Session,
    info: *const xr::ReferenceSpaceCreateInfo,
    space: *mut xr::Space,
) -> xr::Result {
    runtime()
        .as_mut()
        .unwrap()
        .created_spaces
        .push(unsafe { (*info).reference_space_type });
    let ty = unsafe { (*info).reference_space_type };
    unsafe {
        *space = xr::Space::from_raw(if ty == xr::ReferenceSpaceType::LOCAL {
            LOCAL
        } else {
            STAGE
        })
    };
    xr::Result::SUCCESS
}

unsafe extern "system" fn runtime_enumerate_reference_spaces(
    _: xr::Session,
    capacity: u32,
    count: *mut u32,
    spaces: *mut xr::ReferenceSpaceType,
) -> xr::Result {
    let types = [
        xr::ReferenceSpaceType::VIEW,
        xr::ReferenceSpaceType::LOCAL,
        xr::ReferenceSpaceType::STAGE,
    ];
    unsafe { *count = types.len() as u32 };
    if capacity >= types.len() as u32 {
        for (i, ty) in types.into_iter().enumerate() {
            unsafe { *spaces.add(i) = ty };
        }
    }
    xr::Result::SUCCESS
}

unsafe extern "system" fn runtime_destroy_space(space: xr::Space) -> xr::Result {
    runtime()
        .as_mut()
        .unwrap()
        .destroyed_spaces
        .push(space.into_raw());
    xr::Result::SUCCESS
}

unsafe extern "system" fn runtime_locate_space(
    space: xr::Space,
    base: xr::Space,
    _: xr::Time,
    location: *mut xr::SpaceLocation,
) -> xr::Result {
    let pose = match (space.into_raw(), base.into_raw()) {
        (STAGE, LOCAL) => STAGE_IN_LOCAL,
        (LOCAL, STAGE) => crate::math::inverse(STAGE_IN_LOCAL),
        _ => return xr::Result::ERROR_HANDLE_INVALID,
    };
    unsafe {
        (*location).location_flags =
            xr::SpaceLocationFlags::ORIENTATION_VALID | xr::SpaceLocationFlags::POSITION_VALID;
        (*location).pose = pose;
    }
    xr::Result::SUCCESS
}

unsafe extern "system" fn runtime_convert_counter(
    _: xr::Instance,
    counter: *const i64,
    time: *mut xr::Time,
) -> xr::Result {
    unsafe { *time = xr::Time::from_nanos(performance_counter_to_ns(*counter)) };
    xr::Result::SUCCESS
}

/// OpenXR times are performance counter nanoseconds here.
unsafe extern "system" fn runtime_convert_time(
    _: xr::Instance,
    time: xr::Time,
    counter: *mut i64,
) -> xr::Result {
    unsafe { *counter = ns_to_performance_counter(time.as_nanos()) };
    xr::Result::SUCCESS
}

unsafe extern "system" fn runtime_get_instance_proc_addr(
    _: xr::Instance,
    name: *const c_char,
    function: *mut Option<pfn::VoidFunction>,
) -> xr::Result {
    macro_rules! f {
        ($f:expr, $ty:ty) => {
            unsafe { mem::transmute::<$ty, pfn::VoidFunction>($f as $ty) }
        };
    }
    let found = match unsafe { CStr::from_ptr(name) }.to_bytes() {
        b"xrDestroyInstance" => f!(runtime_destroy_instance, pfn::DestroyInstance),
        b"xrGetSystem" => f!(runtime_get_system, pfn::GetSystem),
        b"xrPollEvent" => f!(runtime_poll_event, pfn::PollEvent),
        b"xrCreateSession" => f!(runtime_create_session, pfn::CreateSession),
        b"xrDestroySession" => f!(runtime_destroy_session, pfn::DestroySession),
        b"xrCreateReferenceSpace" => f!(runtime_create_reference_space, pfn::CreateReferenceSpace),
        b"xrDestroySpace" => f!(runtime_destroy_space, pfn::DestroySpace),
        b"xrLocateSpace" => f!(runtime_locate_space, pfn::LocateSpace),
        b"xrEnumerateReferenceSpaces" => f!(
            runtime_enumerate_reference_spaces,
            pfn::EnumerateReferenceSpaces
        ),
        b"xrConvertWin32PerformanceCounterToTimeKHR" => f!(
            runtime_convert_counter,
            pfn::ConvertWin32PerformanceCounterToTimeKHR
        ),
        b"xrEnumerateInstanceExtensionProperties" if LISTS_EXTENSIONS.load(Ordering::SeqCst) => f!(
            runtime_enumerate_extensions,
            pfn::EnumerateInstanceExtensionProperties
        ),
        b"xrConvertTimeToWin32PerformanceCounterKHR" => f!(
            runtime_convert_time,
            pfn::ConvertTimeToWin32PerformanceCounterKHR
        ),
        _ => {
            unsafe { *function = None };
            return xr::Result::ERROR_FUNCTION_UNSUPPORTED;
        }
    };
    unsafe { *function = Some(found) };

    xr::Result::SUCCESS
}

/// What the loader does: negotiates, then creates the instance through the layer.
fn create_instance(extensions: &[&CStr]) -> (xr::Result, xr::Instance, pfn::GetInstanceProcAddr) {
    let loader_info = loader::XrNegotiateLoaderInfo {
        ty: loader::XrNegotiateLoaderInfo::TYPE,
        struct_version: loader::XrNegotiateLoaderInfo::VERSION,
        struct_size: mem::size_of::<loader::XrNegotiateLoaderInfo>(),
        min_interface_version: 1,
        max_interface_version: 1,
        min_api_version: xr::Version::new(1, 0, 0),
        max_api_version: xr::Version::new(1, 1, 0),
    };
    let mut request = loader::XrNegotiateApiLayerRequest {
        ty: loader::XrNegotiateApiLayerRequest::TYPE,
        struct_version: loader::XrNegotiateApiLayerRequest::VERSION,
        struct_size: mem::size_of::<loader::XrNegotiateApiLayerRequest>(),
        layer_interface_version: 0,
        layer_api_version: xr::Version::new(0, 0, 0),
        get_instance_proc_addr: None,
        create_api_layer_instance: None,
    };
    let result = unsafe {
        xrNegotiateLoaderApiLayerInterface(&loader_info, LAYER_NAME.as_ptr(), &mut request)
    };
    assert_eq!(result, xr::Result::SUCCESS);
    assert_eq!(request.layer_api_version, xr::Version::new(1, 1, 0));

    let mut next_info = loader::XrApiLayerNextInfo {
        ty: loader::XrApiLayerNextInfo::TYPE,
        struct_version: loader::XrApiLayerNextInfo::VERSION,
        struct_size: mem::size_of::<loader::XrApiLayerNextInfo>(),
        layer_name: [0; xr::MAX_API_LAYER_NAME_SIZE],
        next_get_instance_proc_addr: runtime_get_instance_proc_addr,
        next_create_api_layer_instance: runtime_create_instance,
        next: ptr::null_mut(),
    };
    for (i, byte) in LAYER_NAME.to_bytes().iter().enumerate() {
        next_info.layer_name[i] = *byte as c_char;
    }
    let layer_info = loader::ApiLayerCreateInfo {
        ty: loader::ApiLayerCreateInfo::TYPE,
        struct_version: loader::ApiLayerCreateInfo::VERSION,
        struct_size: mem::size_of::<loader::ApiLayerCreateInfo>(),
        loader_instance: ptr::null(),
        settings_file_location: [0; loader::API_LAYER_MAX_SETTINGS_PATH_SIZE],
        next_info: &mut next_info,
    };

    let names = extensions
        .iter()
        .map(|name| name.as_ptr())
        .collect::<Vec<_>>();
    let info = xr::InstanceCreateInfo {
        ty: xr::InstanceCreateInfo::TYPE,
        next: ptr::null(),
        create_flags: xr::InstanceCreateFlags::EMPTY,
        application_info: unsafe { mem::zeroed() },
        enabled_api_layer_count: 0,
        enabled_api_layer_names: ptr::null(),
        enabled_extension_count: names.len() as u32,
        enabled_extension_names: names.as_ptr(),
    };

    let mut instance = xr::Instance::NULL;
    let result =
        unsafe { (request.create_api_layer_instance.unwrap())(&info, &layer_info, &mut instance) };

    (result, instance, request.get_instance_proc_addr.unwrap())
}

/// Looks up a function through the layer, as the loader does for the application.
fn get<T>(gipa: pfn::GetInstanceProcAddr, instance: xr::Instance, name: &CStr) -> T {
    let mut function = None;
    let result = unsafe { gipa(instance, name.as_ptr(), &mut function) };
    assert_eq!(result, xr::Result::SUCCESS, "{name:?}");
    unsafe { mem::transmute_copy(&function.unwrap()) }
}

fn pose(x: f32, y: f32, z: f32) -> Pose {
    Pose {
        orientation: [0.0, 0.0, 0.0, 1.0],
        position: [x, y, z],
    }
}

const MOVING: TrackingConfig = TrackingConfig {
    static_markers: false,
    marker_size: None,
};
const STATIC: TrackingConfig = TrackingConfig {
    static_markers: true,
    marker_size: None,
};
const STATIC_SIZED: TrackingConfig = TrackingConfig {
    static_markers: true,
    marker_size: Some(0.5),
};
const HEADSET: Capabilities = Capabilities {
    qr_codes: true,
    fixed_size_markers: true,
    static_markers: true,
};

/// The driver's end, and what the layers ask it for.
fn start_driver() -> (Server, Receiver<Vec<TrackingConfig>>) {
    let (sender, receiver) = mpsc::channel();
    let server = Server::start_named(&pipe_name(), move |configs| {
        sender.send(configs).ok();
    })
    .unwrap();

    (server, receiver)
}

/// Waits until the layers ask for `expected`.
fn expect_requests(requests: &Receiver<Vec<TrackingConfig>>, expected: &[TrackingConfig]) {
    let start = Instant::now();
    loop {
        let configs = requests
            .recv_timeout(Duration::from_secs(5))
            .unwrap_or_else(|_| panic!("no request for {expected:?}"));
        if configs.len() == expected.len() && expected.iter().all(|c| configs.contains(c)) {
            return;
        }
        assert!(start.elapsed() < Duration::from_secs(5), "{configs:?}");
    }
}

/// Sends what every tracker of the headset sees now, as the driver does, and waits until the
/// layer has it. Returns the time the poses are for.
fn publish(server: &mut Server, markers: &[(&str, Pose, f32)]) -> xr::Time {
    let markers = markers
        .iter()
        .map(|&(text, pose, size)| Marker::new(text, pose, size))
        .collect::<Vec<_>>();
    publish_markers(server, &markers)
}

fn publish_markers(server: &mut Server, markers: &[Marker]) -> xr::Time {
    // Connected first: a client that tried while no driver ran waits a while before trying again,
    // and the markers would be stale by the time they arrived
    let start = Instant::now();
    loop {
        let mut state = crate::state();
        state.refresh_markers();
        if state.markers.client.is_connected() {
            break;
        }
        drop(state);
        assert!(start.elapsed() < Duration::from_secs(5));
        thread::sleep(Duration::from_millis(1));
    }

    let time_ns = now_ns();
    let streams = [MOVING, STATIC, STATIC_SIZED]
        .into_iter()
        .map(|config| Stream {
            config,
            time_ns,
            markers: markers.to_vec(),
        })
        .collect();
    server.publish(FLAG_CLIENT_CONNECTED, HEADSET, streams);

    let generation = server.generation();
    let start = Instant::now();
    loop {
        let mut state = crate::state();
        state.refresh_markers();
        if state
            .markers
            .client
            .read()
            .is_some_and(|snapshot| snapshot.generation == generation)
        {
            // Taking in what that read received too
            state.refresh_markers();
            break;
        }
        drop(state);
        assert!(start.elapsed() < Duration::from_secs(5));
        thread::sleep(Duration::from_millis(1));
    }

    xr::Time::from_nanos(time_ns)
}

const ALL_EXTENSIONS: [&CStr; 3] = [
    c"XR_EXT_future",
    c"XR_EXT_spatial_entity",
    c"XR_EXT_spatial_marker_tracking",
];

const SYSTEM: xr::SystemId = unsafe { mem::transmute(1_u64) };

struct App {
    gipa: pfn::GetInstanceProcAddr,
    instance: xr::Instance,
}

impl App {
    /// Gets the system, as applications do before anything else, which makes it known to the
    /// layer.
    fn new(gipa: pfn::GetInstanceProcAddr, instance: xr::Instance) -> Self {
        let app = Self { gipa, instance };
        let get_system: pfn::GetSystem = app.get(c"xrGetSystem");
        let mut system = xr::SystemId::NULL;
        assert_eq!(
            unsafe { get_system(instance, ptr::null(), &mut system) },
            xr::Result::SUCCESS
        );
        assert_eq!(system, SYSTEM);

        app
    }

    fn get<T>(&self, name: &CStr) -> T {
        get(self.gipa, self.instance, name)
    }

    fn create_session(&self) -> xr::Session {
        let create_session: pfn::CreateSession = self.get(c"xrCreateSession");
        let mut session = xr::Session::NULL;
        let result = unsafe { create_session(self.instance, ptr::null(), &mut session) };
        assert_eq!(result, xr::Result::SUCCESS);
        session
    }

    fn destroy(self) {
        let destroy_instance: pfn::DestroyInstance = self.get(c"xrDestroyInstance");
        assert_eq!(
            unsafe { destroy_instance(self.instance) },
            xr::Result::SUCCESS
        );
    }

    /// What xrCreateSpatialContextAsyncEXT says to these configurations.
    fn try_create_context(
        &self,
        session: xr::Session,
        configs: &[*const xr::SpatialCapabilityConfigurationBaseHeaderEXT],
    ) -> xr::Result {
        let create: pfn::CreateSpatialContextAsyncEXT = self.get(c"xrCreateSpatialContextAsyncEXT");
        let info = xr::SpatialContextCreateInfoEXT {
            ty: xr::SpatialContextCreateInfoEXT::TYPE,
            next: ptr::null(),
            capability_config_count: configs.len() as u32,
            capability_configs: if configs.is_empty() {
                ptr::null()
            } else {
                configs.as_ptr()
            },
        };
        let mut future = xr::FutureEXT::from_raw(0);
        unsafe { create(session, &info, &mut future) }
    }

    fn capabilities(&self) -> Vec<xr::SpatialCapabilityEXT> {
        let enumerate: pfn::EnumerateSpatialCapabilitiesEXT =
            self.get(c"xrEnumerateSpatialCapabilitiesEXT");
        let mut count = 0;
        let result = unsafe { enumerate(self.instance, SYSTEM, 0, &mut count, ptr::null_mut()) };
        assert_eq!(result, xr::Result::SUCCESS);
        let mut capabilities = vec![xr::SpatialCapabilityEXT::from_raw(0); count as usize];
        let result = unsafe {
            enumerate(
                self.instance,
                SYSTEM,
                count,
                &mut count,
                capabilities.as_mut_ptr(),
            )
        };
        assert_eq!(result, xr::Result::SUCCESS);

        capabilities
    }

    fn create_context(
        &self,
        session: xr::Session,
        capability: xr::SpatialCapabilityEXT,
        ty: xr::StructureType,
        components: &[xr::SpatialComponentTypeEXT],
        config: TrackingConfig,
    ) -> Result<xr::SpatialContextEXT, xr::Result> {
        let marker_size = config.marker_size;
        let create: pfn::CreateSpatialContextAsyncEXT = self.get(c"xrCreateSpatialContextAsyncEXT");
        let complete: pfn::CreateSpatialContextCompleteEXT =
            self.get(c"xrCreateSpatialContextCompleteEXT");
        let poll: pfn::PollFutureEXT = self.get(c"xrPollFutureEXT");

        let size = xr::SpatialMarkerSizeEXT {
            ty: xr::SpatialMarkerSizeEXT::TYPE,
            next: ptr::null(),
            marker_side_length: marker_size.unwrap_or(0.0),
        };
        let static_markers = xr::SpatialMarkerStaticOptimizationEXT {
            ty: xr::SpatialMarkerStaticOptimizationEXT::TYPE,
            next: if marker_size.is_some() {
                (&raw const size).cast()
            } else {
                ptr::null()
            },
            optimize_for_static_marker: config.static_markers.into(),
        };
        let config = xr::SpatialCapabilityConfigurationBaseHeaderEXT {
            ty,
            next: (&raw const static_markers).cast(),
            capability,
            enabled_component_count: components.len() as u32,
            enabled_components: components.as_ptr(),
        };
        let configs = [&raw const config];
        let info = xr::SpatialContextCreateInfoEXT {
            ty: xr::SpatialContextCreateInfoEXT::TYPE,
            next: ptr::null(),
            capability_config_count: 1,
            capability_configs: configs.as_ptr(),
        };

        let mut future = xr::FutureEXT::from_raw(0);
        let result = unsafe { create(session, &info, &mut future) };
        if result != xr::Result::SUCCESS {
            return Err(result);
        }

        let poll_info = xr::FuturePollInfoEXT {
            ty: xr::FuturePollInfoEXT::TYPE,
            next: ptr::null(),
            future,
        };
        let mut poll_result = xr::FuturePollResultEXT {
            ty: xr::FuturePollResultEXT::TYPE,
            next: ptr::null_mut(),
            state: xr::FutureStateEXT::PENDING,
        };
        assert_eq!(
            unsafe { poll(self.instance, &poll_info, &mut poll_result) },
            xr::Result::SUCCESS
        );
        assert_eq!(poll_result.state, xr::FutureStateEXT::READY);

        let mut completion = xr::CreateSpatialContextCompletionEXT {
            ty: xr::CreateSpatialContextCompletionEXT::TYPE,
            next: ptr::null_mut(),
            future_result: xr::Result::ERROR_RUNTIME_FAILURE,
            spatial_context: xr::SpatialContextEXT::NULL,
        };
        assert_eq!(
            unsafe { complete(session, future, &mut completion) },
            xr::Result::SUCCESS
        );
        assert_eq!(completion.future_result, xr::Result::SUCCESS);

        // A future is gone once completed
        assert_eq!(
            unsafe { complete(session, future, &mut completion) },
            xr::Result::ERROR_FUTURE_INVALID_EXT
        );

        Ok(completion.spatial_context)
    }

    fn discover(&self, context: xr::SpatialContextEXT, time: xr::Time) -> xr::SpatialSnapshotEXT {
        self.discover_with(context, time, &[], None)
    }

    /// A discovery snapshot of the given components, and only of entities in the given state.
    fn discover_with(
        &self,
        context: xr::SpatialContextEXT,
        time: xr::Time,
        components: &[xr::SpatialComponentTypeEXT],
        tracking_state: Option<xr::SpatialEntityTrackingStateEXT>,
    ) -> xr::SpatialSnapshotEXT {
        let future = self.start_discovery(context, components, tracking_state);
        let complete: pfn::CreateSpatialDiscoverySnapshotCompleteEXT =
            self.get(c"xrCreateSpatialDiscoverySnapshotCompleteEXT");

        let completion_info = xr::CreateSpatialDiscoverySnapshotCompletionInfoEXT {
            ty: xr::CreateSpatialDiscoverySnapshotCompletionInfoEXT::TYPE,
            next: ptr::null(),
            base_space: xr::Space::from_raw(LOCAL),
            time,
            future,
        };
        let mut completion = xr::CreateSpatialDiscoverySnapshotCompletionEXT {
            ty: xr::CreateSpatialDiscoverySnapshotCompletionEXT::TYPE,
            next: ptr::null_mut(),
            future_result: xr::Result::ERROR_RUNTIME_FAILURE,
            snapshot: xr::SpatialSnapshotEXT::NULL,
        };
        assert_eq!(
            unsafe { complete(context, &completion_info, &mut completion) },
            xr::Result::SUCCESS
        );
        assert_eq!(completion.future_result, xr::Result::SUCCESS);

        completion.snapshot
    }

    fn start_discovery(
        &self,
        context: xr::SpatialContextEXT,
        components: &[xr::SpatialComponentTypeEXT],
        tracking_state: Option<xr::SpatialEntityTrackingStateEXT>,
    ) -> xr::FutureEXT {
        let create: pfn::CreateSpatialDiscoverySnapshotAsyncEXT =
            self.get(c"xrCreateSpatialDiscoverySnapshotAsyncEXT");

        let filter = xr::SpatialFilterTrackingStateEXT {
            ty: xr::SpatialFilterTrackingStateEXT::TYPE,
            next: ptr::null(),
            tracking_state: tracking_state.unwrap_or(xr::SpatialEntityTrackingStateEXT::TRACKING),
        };
        let info = xr::SpatialDiscoverySnapshotCreateInfoEXT {
            ty: xr::SpatialDiscoverySnapshotCreateInfoEXT::TYPE,
            next: if tracking_state.is_some() {
                (&raw const filter).cast()
            } else {
                ptr::null()
            },
            component_type_count: components.len() as u32,
            component_types: components.as_ptr(),
        };
        let mut future = xr::FutureEXT::from_raw(0);
        assert_eq!(
            unsafe { create(context, &info, &mut future) },
            xr::Result::SUCCESS
        );

        future
    }

    /// The ids of the entities in the snapshot that have the components.
    fn ids(
        &self,
        snapshot: xr::SpatialSnapshotEXT,
        components: &[xr::SpatialComponentTypeEXT],
    ) -> Vec<u64> {
        let query: pfn::QuerySpatialComponentDataEXT = self.get(c"xrQuerySpatialComponentDataEXT");
        let condition = xr::SpatialComponentDataQueryConditionEXT {
            ty: xr::SpatialComponentDataQueryConditionEXT::TYPE,
            next: ptr::null(),
            component_type_count: components.len() as u32,
            component_types: components.as_ptr(),
        };
        let mut ids = [xr::SpatialEntityIdEXT::from_raw(0); 8];
        let mut result = xr::SpatialComponentDataQueryResultEXT {
            ty: xr::SpatialComponentDataQueryResultEXT::TYPE,
            next: ptr::null_mut(),
            entity_id_capacity_input: ids.len() as u32,
            entity_id_count_output: 0,
            entity_ids: ids.as_mut_ptr(),
            entity_state_capacity_input: 0,
            entity_state_count_output: 0,
            entity_states: ptr::null_mut(),
        };
        assert_eq!(
            unsafe { query(snapshot, &condition, &mut result) },
            xr::Result::SUCCESS
        );

        ids[..result.entity_id_count_output as usize]
            .iter()
            .map(|id| id.into_raw())
            .collect()
    }

    /// Ids, states, 2D bounds and marker data of every entity in the snapshot.
    fn query(
        &self,
        snapshot: xr::SpatialSnapshotEXT,
    ) -> Vec<(
        u64,
        xr::SpatialEntityTrackingStateEXT,
        xr::SpatialBounded2DDataEXT,
        xr::SpatialMarkerDataEXT,
    )> {
        let query: pfn::QuerySpatialComponentDataEXT = self.get(c"xrQuerySpatialComponentDataEXT");

        let components = [
            xr::SpatialComponentTypeEXT::BOUNDED_2D,
            xr::SpatialComponentTypeEXT::MARKER,
        ];
        let condition = xr::SpatialComponentDataQueryConditionEXT {
            ty: xr::SpatialComponentDataQueryConditionEXT::TYPE,
            next: ptr::null(),
            component_type_count: components.len() as u32,
            component_types: components.as_ptr(),
        };
        let mut result = xr::SpatialComponentDataQueryResultEXT {
            ty: xr::SpatialComponentDataQueryResultEXT::TYPE,
            next: ptr::null_mut(),
            entity_id_capacity_input: 0,
            entity_id_count_output: 0,
            entity_ids: ptr::null_mut(),
            entity_state_capacity_input: 0,
            entity_state_count_output: 0,
            entity_states: ptr::null_mut(),
        };
        assert_eq!(
            unsafe { query(snapshot, &condition, &mut result) },
            xr::Result::SUCCESS
        );
        let count = result.entity_id_count_output as usize;

        let mut ids = vec![xr::SpatialEntityIdEXT::from_raw(0); count];
        let mut states = vec![xr::SpatialEntityTrackingStateEXT::STOPPED; count];
        let mut bounds = vec![xr::SpatialBounded2DDataEXT::default(); count];
        let mut markers = vec![unsafe { mem::zeroed::<xr::SpatialMarkerDataEXT>() }; count];
        let mut marker_list = xr::SpatialComponentMarkerListEXT {
            ty: xr::SpatialComponentMarkerListEXT::TYPE,
            next: ptr::null_mut(),
            marker_count: count as u32,
            markers: markers.as_mut_ptr(),
        };
        let mut bounds_list = xr::SpatialComponentBounded2DListEXT {
            ty: xr::SpatialComponentBounded2DListEXT::TYPE,
            next: (&raw mut marker_list).cast(),
            bound_count: count as u32,
            bounds: bounds.as_mut_ptr(),
        };
        result.next = (&raw mut bounds_list).cast();
        result.entity_id_capacity_input = count as u32;
        result.entity_ids = ids.as_mut_ptr();
        result.entity_state_capacity_input = count as u32;
        result.entity_states = states.as_mut_ptr();
        if count > 0 {
            assert_eq!(
                unsafe { query(snapshot, &condition, &mut result) },
                xr::Result::SUCCESS
            );
        }

        (0..count)
            .map(|i| (ids[i].into_raw(), states[i], bounds[i], markers[i]))
            .collect()
    }

    fn buffer_string(
        &self,
        snapshot: xr::SpatialSnapshotEXT,
        buffer: xr::SpatialBufferIdEXT,
    ) -> Result<String, xr::Result> {
        let get_string: pfn::GetSpatialBufferStringEXT = self.get(c"xrGetSpatialBufferStringEXT");
        let info = xr::SpatialBufferGetInfoEXT {
            ty: xr::SpatialBufferGetInfoEXT::TYPE,
            next: ptr::null(),
            buffer_id: buffer,
        };
        let mut count = 0;
        let result = unsafe { get_string(snapshot, &info, 0, &mut count, ptr::null_mut()) };
        if result != xr::Result::SUCCESS {
            return Err(result);
        }
        let mut text = vec![0 as c_char; count as usize];
        let result = unsafe { get_string(snapshot, &info, count, &mut count, text.as_mut_ptr()) };
        assert_eq!(result, xr::Result::SUCCESS);

        Ok(unsafe { CStr::from_ptr(text.as_ptr()) }
            .to_string_lossy()
            .into())
    }

    fn buffer_bytes(
        &self,
        snapshot: xr::SpatialSnapshotEXT,
        buffer: xr::SpatialBufferIdEXT,
    ) -> Result<Vec<u8>, xr::Result> {
        let get_bytes: pfn::GetSpatialBufferUint8EXT = self.get(c"xrGetSpatialBufferUint8EXT");
        let info = xr::SpatialBufferGetInfoEXT {
            ty: xr::SpatialBufferGetInfoEXT::TYPE,
            next: ptr::null(),
            buffer_id: buffer,
        };
        let mut count = 0;
        let result = unsafe { get_bytes(snapshot, &info, 0, &mut count, ptr::null_mut()) };
        if result != xr::Result::SUCCESS {
            return Err(result);
        }
        let mut bytes = vec![0; count as usize];
        let result = unsafe { get_bytes(snapshot, &info, count, &mut count, bytes.as_mut_ptr()) };
        assert_eq!(result, xr::Result::SUCCESS);

        Ok(bytes)
    }

    fn poll_event(&self) -> Option<xr::SpatialContextEXT> {
        let poll: pfn::PollEvent = self.get(c"xrPollEvent");
        let mut event = xr::EventDataBuffer {
            ty: xr::StructureType::EVENT_DATA_BUFFER,
            next: ptr::null(),
            varying: [0; 4000],
        };
        match unsafe { poll(self.instance, &mut event) } {
            xr::Result::EVENT_UNAVAILABLE => None,
            xr::Result::SUCCESS => {
                assert_eq!(event.ty, xr::EventDataSpatialDiscoveryRecommendedEXT::TYPE);
                let event = unsafe {
                    &*(&raw const event).cast::<xr::EventDataSpatialDiscoveryRecommendedEXT>()
                };
                Some(event.spatial_context)
            }
            result => panic!("{result:?}"),
        }
    }
}

fn close(a: xr::Vector3f, b: [f32; 3]) -> bool {
    (a.x - b[0]).abs() < 1e-5 && (a.y - b[1]).abs() < 1e-5 && (a.z - b[2]).abs() < 1e-5
}

#[test]
fn markers_reach_the_application() {
    let _serial = serial();
    let (mut writer, requests) = start_driver();
    let mut time = publish(
        &mut writer,
        &[
            ("dog", pose(1.0, 0.0, -2.0), 0.2),
            ("cat", pose(-1.0, 1.0, -3.0), 0.1),
        ],
    );

    CREATE_ATTEMPTS.store(0, Ordering::SeqCst);
    let (result, instance, gipa) = create_instance(&ALL_EXTENSIONS);
    assert_eq!(result, xr::Result::SUCCESS);
    // SteamVR would have refused them; the layer converts times with the runtime
    assert_eq!(
        runtime().as_ref().unwrap().enabled_extensions,
        [TIME_EXTENSION.to_str().unwrap()]
    );
    // In one go: SteamVR asserts when created again after a failed attempt
    assert_eq!(CREATE_ATTEMPTS.load(Ordering::SeqCst), 1);
    let app = App::new(gipa, instance);

    assert_eq!(
        app.capabilities(),
        [xr::SpatialCapabilityEXT::MARKER_TRACKING_QR_CODE]
    );

    let enumerate_components: pfn::EnumerateSpatialCapabilityComponentTypesEXT =
        app.get(c"xrEnumerateSpatialCapabilityComponentTypesEXT");
    let mut components = [xr::SpatialComponentTypeEXT::from_raw(0); 4];
    let mut component_types = xr::SpatialCapabilityComponentTypesEXT {
        ty: xr::SpatialCapabilityComponentTypesEXT::TYPE,
        next: ptr::null_mut(),
        component_type_capacity_input: 4,
        component_type_count_output: 0,
        component_types: components.as_mut_ptr(),
    };
    let result = unsafe {
        enumerate_components(
            instance,
            SYSTEM,
            xr::SpatialCapabilityEXT::MARKER_TRACKING_QR_CODE,
            &mut component_types,
        )
    };
    assert_eq!(result, xr::Result::SUCCESS);
    assert_eq!(
        components[..component_types.component_type_count_output as usize],
        [
            xr::SpatialComponentTypeEXT::MARKER,
            xr::SpatialComponentTypeEXT::BOUNDED_2D
        ]
    );
    let result = unsafe {
        enumerate_components(
            instance,
            SYSTEM,
            xr::SpatialCapabilityEXT::MARKER_TRACKING_ARUCO_MARKER,
            &mut component_types,
        )
    };
    assert_eq!(result, xr::Result::ERROR_SPATIAL_CAPABILITY_UNSUPPORTED_EXT);

    let session = app.create_session();

    let qr = xr::SpatialCapabilityEXT::MARKER_TRACKING_QR_CODE;
    let qr_config = xr::SpatialCapabilityConfigurationQrCodeEXT::TYPE;
    let both = [
        xr::SpatialComponentTypeEXT::MARKER,
        xr::SpatialComponentTypeEXT::BOUNDED_2D,
    ];

    // Invalid configurations
    assert_eq!(
        app.create_context(
            session,
            xr::SpatialCapabilityEXT::MARKER_TRACKING_ARUCO_MARKER,
            xr::SpatialCapabilityConfigurationArucoMarkerEXT::TYPE,
            &both,
            MOVING
        ),
        Err(xr::Result::ERROR_SPATIAL_CAPABILITY_UNSUPPORTED_EXT)
    );
    assert_eq!(
        app.create_context(
            session,
            qr,
            xr::SpatialCapabilityConfigurationAprilTagEXT::TYPE,
            &both,
            MOVING
        ),
        Err(xr::Result::ERROR_SPATIAL_CAPABILITY_CONFIGURATION_INVALID_EXT)
    );
    assert_eq!(
        app.create_context(
            session,
            qr,
            qr_config,
            &[xr::SpatialComponentTypeEXT::MESH_3D],
            MOVING
        ),
        Err(xr::Result::ERROR_SPATIAL_COMPONENT_UNSUPPORTED_FOR_CAPABILITY_EXT)
    );

    // For codes that move
    let context = app
        .create_context(session, qr, qr_config, &both, MOVING)
        .unwrap();
    // The headset is asked to track them that way
    expect_requests(&requests, &[MOVING]);

    // A new context gets a discovery recommendation, once
    assert_eq!(app.poll_event(), Some(context));
    assert_eq!(app.poll_event(), None);

    let snapshot = app.discover(context, time);
    let mut entities = app.query(snapshot);
    entities.sort_by_key(|entity| entity.2.center.position.x as i32);
    assert_eq!(entities.len(), 2);
    let (cat, dog) = (entities[0], entities[1]);
    assert_ne!(dog.0, 0);
    assert_ne!(dog.0, cat.0);
    assert_eq!(dog.1, xr::SpatialEntityTrackingStateEXT::TRACKING);
    // In the application's space, the stage is 1.5 m lower
    assert!(close(dog.2.center.position, [1.0, -1.5, -2.0]));
    assert!(close(cat.2.center.position, [-1.0, -0.5, -3.0]));
    assert_eq!(dog.2.extents.width, 0.2);
    assert_eq!(dog.3.capability, qr);
    assert_eq!(dog.3.data.buffer_type, xr::SpatialBufferTypeEXT::STRING);
    assert_eq!(
        app.buffer_string(snapshot, dog.3.data.buffer_id).unwrap(),
        "dog"
    );
    assert_eq!(
        app.buffer_string(snapshot, cat.3.data.buffer_id).unwrap(),
        "cat"
    );
    assert_eq!(
        app.buffer_string(snapshot, xr::SpatialBufferIdEXT::from_raw(12345)),
        Err(xr::Result::ERROR_SPATIAL_BUFFER_ID_INVALID_EXT)
    );
    // The stage was created once, from the runtime
    assert_eq!(
        runtime().as_ref().unwrap().created_spaces,
        [xr::ReferenceSpaceType::STAGE]
    );

    // Too small a component list
    let query: pfn::QuerySpatialComponentDataEXT = app.get(c"xrQuerySpatialComponentDataEXT");
    let mut ids = [xr::SpatialEntityIdEXT::from_raw(0); 2];
    let mut bounds = [xr::SpatialBounded2DDataEXT::default(); 1];
    let mut bounds_list = xr::SpatialComponentBounded2DListEXT {
        ty: xr::SpatialComponentBounded2DListEXT::TYPE,
        next: ptr::null_mut(),
        bound_count: 1,
        bounds: bounds.as_mut_ptr(),
    };
    let condition = xr::SpatialComponentDataQueryConditionEXT {
        ty: xr::SpatialComponentDataQueryConditionEXT::TYPE,
        next: ptr::null(),
        component_type_count: 1,
        component_types: &xr::SpatialComponentTypeEXT::BOUNDED_2D,
    };
    let mut query_result = xr::SpatialComponentDataQueryResultEXT {
        ty: xr::SpatialComponentDataQueryResultEXT::TYPE,
        next: (&raw mut bounds_list).cast(),
        entity_id_capacity_input: 2,
        entity_id_count_output: 0,
        entity_ids: ids.as_mut_ptr(),
        entity_state_capacity_input: 0,
        entity_state_count_output: 0,
        entity_states: ptr::null_mut(),
    };
    assert_eq!(
        unsafe { query(snapshot, &condition, &mut query_result) },
        xr::Result::ERROR_SIZE_INSUFFICIENT
    );

    // A code keeps its id in the context's later snapshots
    let again = app.query(app.discover(context, time));
    assert!(again.iter().any(|entity| entity.0 == dog.0));

    // Following one entity
    let create_entity: pfn::CreateSpatialEntityFromIdEXT =
        app.get(c"xrCreateSpatialEntityFromIdEXT");
    let mut entity_info = xr::SpatialEntityFromIdCreateInfoEXT {
        ty: xr::SpatialEntityFromIdCreateInfoEXT::TYPE,
        next: ptr::null(),
        entity_id: xr::SpatialEntityIdEXT::from_raw(12345),
    };
    let mut entity = xr::SpatialEntityEXT::NULL;
    assert_eq!(
        unsafe { create_entity(context, &entity_info, &mut entity) },
        xr::Result::ERROR_SPATIAL_ENTITY_ID_INVALID_EXT
    );
    entity_info.entity_id = xr::SpatialEntityIdEXT::from_raw(dog.0);
    assert_eq!(
        unsafe { create_entity(context, &entity_info, &mut entity) },
        xr::Result::SUCCESS
    );

    let update: pfn::CreateSpatialUpdateSnapshotEXT = app.get(c"xrCreateSpatialUpdateSnapshotEXT");
    let update_snapshot = |entity: xr::SpatialEntityEXT, time: xr::Time| {
        let info = xr::SpatialUpdateSnapshotCreateInfoEXT {
            ty: xr::SpatialUpdateSnapshotCreateInfoEXT::TYPE,
            next: ptr::null(),
            entity_count: 1,
            entities: &entity,
            component_type_count: 0,
            component_types: ptr::null(),
            base_space: xr::Space::from_raw(LOCAL),
            time,
        };
        let mut snapshot = xr::SpatialSnapshotEXT::NULL;
        assert_eq!(
            unsafe { update(context, &info, &mut snapshot) },
            xr::Result::SUCCESS
        );
        snapshot
    };

    // The dog moves
    let before = time;
    time = publish(
        &mut writer,
        &[
            ("dog", pose(2.0, 0.0, -2.0), 0.2),
            ("cat", pose(-1.0, 1.0, -3.0), 0.1),
        ],
    );
    let updated = app.query(update_snapshot(entity, time));
    assert_eq!(updated.len(), 1);
    assert_eq!(updated[0].1, xr::SpatialEntityTrackingStateEXT::TRACKING);
    assert!(close(updated[0].2.center.position, [2.0, -1.5, -2.0]));
    // Where it was at the time asked for, half way between its two poses
    let middle = xr::Time::from_nanos((before.as_nanos() + time.as_nanos()) / 2);
    let updated = app.query(update_snapshot(entity, middle));
    // To the performance counter's tick
    assert!((updated[0].2.center.position.x - 1.5).abs() < 1e-3);
    // Same markers, no recommendation
    assert_eq!(app.poll_event(), None);

    // The headset loses sight of it
    time = publish(&mut writer, &[("cat", pose(-1.0, 1.0, -3.0), 0.1)]);
    let updated = app.query(update_snapshot(entity, time));
    assert_eq!(updated[0].1, xr::SpatialEntityTrackingStateEXT::PAUSED);
    assert_eq!(app.poll_event(), Some(context));

    // In a context that only enables markers, no entity has bounds
    let markers_only = app
        .create_context(
            session,
            qr,
            qr_config,
            &[xr::SpatialComponentTypeEXT::MARKER],
            STATIC_SIZED,
        )
        .unwrap();
    expect_requests(&requests, &[MOVING, STATIC_SIZED]);
    let markers_only_snapshot = app.discover(markers_only, time);
    assert_eq!(
        unsafe { query(markers_only_snapshot, &condition, &mut query_result) },
        xr::Result::SUCCESS
    );
    assert_eq!(query_result.entity_id_count_output, 0);
    assert_eq!(
        app.ids(
            markers_only_snapshot,
            &[xr::SpatialComponentTypeEXT::MARKER]
        )
        .len(),
        1
    );

    // A known marker size wins over the detected one
    let destroy_context: pfn::DestroySpatialContextEXT = app.get(c"xrDestroySpatialContextEXT");
    assert_eq!(
        unsafe { destroy_context(markers_only) },
        xr::Result::SUCCESS
    );
    expect_requests(&requests, &[MOVING]);
    let sized = app
        .create_context(session, qr, qr_config, &both, STATIC_SIZED)
        .unwrap();
    let sized_entities = app.query(app.discover(sized, time));
    assert_eq!(sized_entities[0].2.extents.width, 0.5);

    // Static codes are averaged over their latest poses
    let static_context = app
        .create_context(session, qr, qr_config, &both, STATIC)
        .unwrap();
    expect_requests(&requests, &[MOVING, STATIC_SIZED, STATIC]);
    publish(&mut writer, &[("cat", pose(-1.0, 1.0, -3.0), 0.1)]);
    time = publish(&mut writer, &[("cat", pose(-1.2, 1.0, -3.0), 0.1)]);
    let averaged = app.query(app.discover(static_context, time));
    // Over every pose of the last second: mostly at -1, once at -1.2
    let x = averaged[0].2.center.position.x;
    assert!(-1.2 < x && x < -1.0, "{x}");
    // Destroying the context destroyed its snapshots
    let destroy_snapshot: pfn::DestroySpatialSnapshotEXT = app.get(c"xrDestroySpatialSnapshotEXT");
    assert_eq!(
        unsafe { destroy_snapshot(markers_only_snapshot) },
        xr::Result::ERROR_HANDLE_INVALID
    );
    assert_eq!(unsafe { destroy_snapshot(snapshot) }, xr::Result::SUCCESS);

    // The driver stops
    drop(writer);
    assert!(app.capabilities().is_empty());
    let updated = app.query(update_snapshot(entity, time));
    assert_eq!(updated[0].1, xr::SpatialEntityTrackingStateEXT::PAUSED);

    let destroy_session: pfn::DestroySession = app.get(c"xrDestroySession");
    assert_eq!(unsafe { destroy_session(session) }, xr::Result::SUCCESS);
    assert_eq!(runtime().as_ref().unwrap().destroyed_spaces, [STAGE]);
    // Its contexts went with it
    assert_eq!(
        unsafe { destroy_context(context) },
        xr::Result::ERROR_HANDLE_INVALID
    );

    let destroy_instance: pfn::DestroyInstance = app.get(c"xrDestroyInstance");
    assert_eq!(unsafe { destroy_instance(instance) }, xr::Result::SUCCESS);
}

#[test]
fn a_runtime_that_does_not_list_its_extensions_is_tried() {
    let _serial = serial();
    LISTS_EXTENSIONS.store(false, Ordering::SeqCst);
    let (result, instance, gipa) = create_instance(&ALL_EXTENSIONS);
    LISTS_EXTENSIONS.store(true, Ordering::SeqCst);

    assert_eq!(result, xr::Result::SUCCESS);
    assert_eq!(
        runtime().as_ref().unwrap().enabled_extensions,
        [TIME_EXTENSION.to_str().unwrap()]
    );
    let destroy_instance: pfn::DestroyInstance = get(gipa, instance, c"xrDestroyInstance");
    assert_eq!(unsafe { destroy_instance(instance) }, xr::Result::SUCCESS);
}

#[test]
fn applications_without_the_extensions_go_straight_to_the_runtime() {
    let _serial = serial();
    let (result, instance, gipa) = create_instance(&[]);
    assert_eq!(result, xr::Result::SUCCESS);

    // The runtime's own functions, but those the layer reports recentering with
    let locate: pfn::LocateSpace = get(gipa, instance, c"xrLocateSpace");
    assert_eq!(
        locate as usize,
        runtime_locate_space as pfn::LocateSpace as usize
    );
    let poll: pfn::PollEvent = get(gipa, instance, c"xrPollEvent");
    assert_ne!(poll as usize, runtime_poll_event as pfn::PollEvent as usize);

    let mut function = None;
    let result = unsafe {
        gipa(
            instance,
            c"xrCreateSpatialContextAsyncEXT".as_ptr(),
            &mut function,
        )
    };
    assert_eq!(result, xr::Result::ERROR_FUNCTION_UNSUPPORTED);

    let destroy_instance: pfn::DestroyInstance = get(gipa, instance, c"xrDestroyInstance");
    assert_eq!(unsafe { destroy_instance(instance) }, xr::Result::SUCCESS);
}

#[test]
fn xr_ext_future_alone_is_provided_too() {
    let _serial = serial();
    let (result, instance, gipa) = create_instance(&[c"XR_EXT_future"]);
    assert_eq!(result, xr::Result::SUCCESS);

    let poll: pfn::PollFutureEXT = get(gipa, instance, c"xrPollFutureEXT");
    let info = xr::FuturePollInfoEXT {
        ty: xr::FuturePollInfoEXT::TYPE,
        next: ptr::null(),
        future: xr::FutureEXT::from_raw(1),
    };
    let mut poll_result = xr::FuturePollResultEXT {
        ty: xr::FuturePollResultEXT::TYPE,
        next: ptr::null_mut(),
        state: xr::FutureStateEXT::PENDING,
    };
    assert_eq!(
        unsafe { poll(instance, &info, &mut poll_result) },
        xr::Result::ERROR_FUTURE_INVALID_EXT
    );
    // Spatial entities were not asked for
    let mut function = None;
    let result = unsafe {
        gipa(
            instance,
            c"xrCreateSpatialContextAsyncEXT".as_ptr(),
            &mut function,
        )
    };
    assert_eq!(result, xr::Result::ERROR_FUNCTION_UNSUPPORTED);

    let destroy_instance: pfn::DestroyInstance = get(gipa, instance, c"xrDestroyInstance");
    assert_eq!(unsafe { destroy_instance(instance) }, xr::Result::SUCCESS);
}

/// The rules of XR_EXT_spatial_entity and XR_EXT_spatial_marker_tracking that a conformance test
/// checks, and the ones it does not.
#[test]
fn the_spatial_entity_rules_are_followed() {
    let _serial = serial();
    let (mut writer, _requests) = start_driver();
    publish(&mut writer, &[("dog", pose(1.0, 0.0, -2.0), 0.2)]);

    let qr = xr::SpatialCapabilityEXT::MARKER_TRACKING_QR_CODE;
    let qr_config = xr::SpatialCapabilityConfigurationQrCodeEXT::TYPE;
    let both = [
        xr::SpatialComponentTypeEXT::MARKER,
        xr::SpatialComponentTypeEXT::BOUNDED_2D,
    ];

    // Without XR_EXT_spatial_marker_tracking there are no QR codes, nor XR_EXT_future functions
    // without XR_EXT_future
    {
        let (result, instance, gipa) = create_instance(&[c"XR_EXT_spatial_entity"]);
        assert_eq!(result, xr::Result::SUCCESS);
        let app = App::new(gipa, instance);
        assert!(app.capabilities().is_empty());

        let enumerate_features: pfn::EnumerateSpatialCapabilityFeaturesEXT =
            app.get(c"xrEnumerateSpatialCapabilityFeaturesEXT");
        let mut count = 0;
        assert_eq!(
            unsafe { enumerate_features(instance, SYSTEM, qr, 0, &mut count, ptr::null_mut()) },
            xr::Result::ERROR_SPATIAL_CAPABILITY_UNSUPPORTED_EXT
        );
        let session = app.create_session();
        let valid = xr::SpatialCapabilityConfigurationBaseHeaderEXT {
            ty: qr_config,
            next: ptr::null(),
            capability: qr,
            enabled_component_count: both.len() as u32,
            enabled_components: both.as_ptr(),
        };
        assert_eq!(
            app.try_create_context(session, &[&raw const valid]),
            xr::Result::ERROR_SPATIAL_CAPABILITY_UNSUPPORTED_EXT
        );

        let mut function = None;
        let result = unsafe { gipa(instance, c"xrPollFutureEXT".as_ptr(), &mut function) };
        assert_eq!(result, xr::Result::ERROR_FUNCTION_UNSUPPORTED);
        app.destroy();
    }

    let (result, instance, gipa) = create_instance(&ALL_EXTENSIONS);
    assert_eq!(result, xr::Result::SUCCESS);
    let app = App::new(gipa, instance);

    // Only the system the runtime gave
    let enumerate: pfn::EnumerateSpatialCapabilitiesEXT =
        app.get(c"xrEnumerateSpatialCapabilitiesEXT");
    let mut count = 0;
    let other_system = xr::SystemId::from_raw(2);
    assert_eq!(
        unsafe { enumerate(instance, other_system, 0, &mut count, ptr::null_mut()) },
        xr::Result::ERROR_SYSTEM_INVALID
    );

    let session = app.create_session();

    // Invalid context configurations
    let config = |components: &[xr::SpatialComponentTypeEXT]| {
        xr::SpatialCapabilityConfigurationBaseHeaderEXT {
            ty: qr_config,
            next: ptr::null(),
            capability: qr,
            enabled_component_count: components.len() as u32,
            enabled_components: if components.is_empty() {
                ptr::null()
            } else {
                components.as_ptr()
            },
        }
    };
    let unknown = [xr::SpatialComponentTypeEXT::from_raw(0x7fff_0000)];
    let valid = config(&both);
    let no_components = config(&[]);
    let unknown_component = config(&unknown);
    assert_eq!(
        app.try_create_context(session, &[]),
        xr::Result::ERROR_SPATIAL_CAPABILITY_CONFIGURATION_INVALID_EXT
    );
    assert_eq!(
        app.try_create_context(session, &[&raw const no_components]),
        xr::Result::ERROR_SPATIAL_CAPABILITY_CONFIGURATION_INVALID_EXT
    );
    assert_eq!(
        app.try_create_context(session, &[&raw const valid, &raw const valid]),
        xr::Result::ERROR_SPATIAL_CAPABILITY_CONFIGURATION_INVALID_EXT
    );
    assert_eq!(
        app.try_create_context(session, &[&raw const unknown_component]),
        xr::Result::ERROR_VALIDATION_FAILURE
    );

    let first = app
        .create_context(session, qr, qr_config, &both, MOVING)
        .unwrap();
    let second = app
        .create_context(session, qr, qr_config, &both, MOVING)
        .unwrap();
    let time = publish(&mut writer, &[("dog", pose(1.0, 0.0, -2.0), 0.2)]);

    // The same code is a different entity in each context, and an id is only good in its own
    let first_snapshot = app.discover(first, time);
    let second_snapshot = app.discover(second, time);
    let first_ids = app.ids(first_snapshot, &[]);
    let second_ids = app.ids(second_snapshot, &[]);
    assert_eq!(first_ids.len(), 1);
    assert_eq!(second_ids.len(), 1);
    assert_ne!(first_ids, second_ids);
    let create_entity: pfn::CreateSpatialEntityFromIdEXT =
        app.get(c"xrCreateSpatialEntityFromIdEXT");
    let entity_info = |id: u64| xr::SpatialEntityFromIdCreateInfoEXT {
        ty: xr::SpatialEntityFromIdCreateInfoEXT::TYPE,
        next: ptr::null(),
        entity_id: xr::SpatialEntityIdEXT::from_raw(id),
    };
    let mut entity = xr::SpatialEntityEXT::NULL;
    assert_eq!(
        unsafe { create_entity(second, &entity_info(first_ids[0]), &mut entity) },
        xr::Result::ERROR_SPATIAL_ENTITY_ID_INVALID_EXT
    );

    // A discovery is completed on the context it was started on
    let future = app.start_discovery(first, &[], None);
    let complete: pfn::CreateSpatialDiscoverySnapshotCompleteEXT =
        app.get(c"xrCreateSpatialDiscoverySnapshotCompleteEXT");
    let completion_info = xr::CreateSpatialDiscoverySnapshotCompletionInfoEXT {
        ty: xr::CreateSpatialDiscoverySnapshotCompletionInfoEXT::TYPE,
        next: ptr::null(),
        base_space: xr::Space::from_raw(LOCAL),
        time,
        future,
    };
    let mut completion = xr::CreateSpatialDiscoverySnapshotCompletionEXT {
        ty: xr::CreateSpatialDiscoverySnapshotCompletionEXT::TYPE,
        next: ptr::null_mut(),
        future_result: xr::Result::ERROR_RUNTIME_FAILURE,
        snapshot: xr::SpatialSnapshotEXT::NULL,
    };
    assert_eq!(
        unsafe { complete(second, &completion_info, &mut completion) },
        xr::Result::ERROR_HANDLE_INVALID
    );
    // Cancelled, it is gone: the layer answers for its own futures
    let cancel: pfn::CancelFutureEXT = app.get(c"xrCancelFutureEXT");
    let cancel_info = xr::FutureCancelInfoEXT {
        ty: xr::FutureCancelInfoEXT::TYPE,
        next: ptr::null(),
        future,
    };
    assert_eq!(
        unsafe { cancel(instance, &cancel_info) },
        xr::Result::SUCCESS
    );
    let poll: pfn::PollFutureEXT = app.get(c"xrPollFutureEXT");
    let poll_info = xr::FuturePollInfoEXT {
        ty: xr::FuturePollInfoEXT::TYPE,
        next: ptr::null(),
        future,
    };
    let mut poll_result = xr::FuturePollResultEXT {
        ty: xr::FuturePollResultEXT::TYPE,
        next: ptr::null_mut(),
        state: xr::FutureStateEXT::PENDING,
    };
    assert_eq!(
        unsafe { poll(instance, &poll_info, &mut poll_result) },
        xr::Result::ERROR_FUTURE_INVALID_EXT
    );
    assert_eq!(
        unsafe { complete(first, &completion_info, &mut completion) },
        xr::Result::ERROR_FUTURE_INVALID_EXT
    );

    // A discovery of some components has only their data
    let bounds_only = app.discover_with(
        first,
        time,
        &[xr::SpatialComponentTypeEXT::BOUNDED_2D],
        None,
    );
    assert_eq!(app.ids(bounds_only, &[]), first_ids);
    assert!(
        app.ids(bounds_only, &[xr::SpatialComponentTypeEXT::MARKER])
            .is_empty()
    );
    let full = app.query(first_snapshot);
    assert_eq!(
        app.buffer_string(bounds_only, full[0].3.data.buffer_id),
        Err(xr::Result::ERROR_SPATIAL_BUFFER_ID_INVALID_EXT)
    );

    // Discoveries filter by tracking state
    let paused_only = app.discover_with(
        first,
        time,
        &[],
        Some(xr::SpatialEntityTrackingStateEXT::PAUSED),
    );
    assert!(app.ids(paused_only, &[]).is_empty());

    // Component lists only for the components asked for; larger ones than needed are fine
    let query: pfn::QuerySpatialComponentDataEXT = app.get(c"xrQuerySpatialComponentDataEXT");
    let mut ids = [xr::SpatialEntityIdEXT::from_raw(0); 4];
    let mut bounds = [xr::SpatialBounded2DDataEXT::default(); 4];
    let mut bounds_list = xr::SpatialComponentBounded2DListEXT {
        ty: xr::SpatialComponentBounded2DListEXT::TYPE,
        next: ptr::null_mut(),
        bound_count: 4,
        bounds: bounds.as_mut_ptr(),
    };
    let mut query_result = xr::SpatialComponentDataQueryResultEXT {
        ty: xr::SpatialComponentDataQueryResultEXT::TYPE,
        next: (&raw mut bounds_list).cast(),
        entity_id_capacity_input: 4,
        entity_id_count_output: 0,
        entity_ids: ids.as_mut_ptr(),
        entity_state_capacity_input: 0,
        entity_state_count_output: 0,
        entity_states: ptr::null_mut(),
    };
    let marker_only = [xr::SpatialComponentTypeEXT::MARKER];
    let bounds_condition = [xr::SpatialComponentTypeEXT::BOUNDED_2D];
    let condition =
        |components: &[xr::SpatialComponentTypeEXT]| xr::SpatialComponentDataQueryConditionEXT {
            ty: xr::SpatialComponentDataQueryConditionEXT::TYPE,
            next: ptr::null(),
            component_type_count: components.len() as u32,
            component_types: components.as_ptr(),
        };
    assert_eq!(
        unsafe { query(first_snapshot, &condition(&marker_only), &mut query_result) },
        xr::Result::ERROR_VALIDATION_FAILURE
    );
    assert_eq!(
        unsafe {
            query(
                first_snapshot,
                &condition(&bounds_condition),
                &mut query_result,
            )
        },
        xr::Result::SUCCESS
    );
    assert_eq!(query_result.entity_id_count_output, 1);
    assert!(close(bounds[0].center.position, [1.0, -1.5, -2.0]));

    // One entity twice in an update snapshot is the same both times
    let mut dog = xr::SpatialEntityEXT::NULL;
    let mut dog_again = xr::SpatialEntityEXT::NULL;
    for entity in [&mut dog, &mut dog_again] {
        assert_eq!(
            unsafe { create_entity(first, &entity_info(first_ids[0]), entity) },
            xr::Result::SUCCESS
        );
    }
    let update: pfn::CreateSpatialUpdateSnapshotEXT = app.get(c"xrCreateSpatialUpdateSnapshotEXT");
    let entities = [dog, dog_again];
    let update_info = xr::SpatialUpdateSnapshotCreateInfoEXT {
        ty: xr::SpatialUpdateSnapshotCreateInfoEXT::TYPE,
        next: ptr::null(),
        entity_count: 2,
        entities: entities.as_ptr(),
        component_type_count: 0,
        component_types: ptr::null(),
        base_space: xr::Space::from_raw(LOCAL),
        time,
    };
    let mut update_snapshot = xr::SpatialSnapshotEXT::NULL;
    assert_eq!(
        unsafe { update(first, &update_info, &mut update_snapshot) },
        xr::Result::SUCCESS
    );
    let updated = app.query(update_snapshot);
    assert_eq!(updated.len(), 2);
    assert!(updated.iter().all(|entity| entity.0 == first_ids[0]
        && entity.1 == xr::SpatialEntityTrackingStateEXT::TRACKING));

    // Codes holding bytes rather than text have byte buffers
    let time = publish_markers(
        &mut writer,
        &[Marker::with_payload(
            Payload::Bytes(vec![0, 1, 0xff]),
            pose(0.0, 0.0, -1.0),
            0.1,
        )],
    );
    let snapshot = app.discover(first, time);
    let entities = app.query(snapshot);
    let bytes = entities
        .iter()
        .find(|entity| entity.1 == xr::SpatialEntityTrackingStateEXT::TRACKING)
        .unwrap();
    assert_eq!(bytes.3.data.buffer_type, xr::SpatialBufferTypeEXT::UINT8);
    assert_eq!(
        app.buffer_bytes(snapshot, bytes.3.data.buffer_id),
        Ok(vec![0, 1, 0xff])
    );
    assert_eq!(
        app.buffer_string(snapshot, bytes.3.data.buffer_id),
        Err(xr::Result::ERROR_VALIDATION_FAILURE)
    );
    // And text codes have no byte buffer
    let time = publish(&mut writer, &[("dog", pose(1.0, 0.0, -2.0), 0.2)]);
    let snapshot = app.discover(first, time);
    let dog_buffer = app
        .query(snapshot)
        .iter()
        .find(|entity| entity.1 == xr::SpatialEntityTrackingStateEXT::TRACKING)
        .unwrap()
        .3
        .data
        .buffer_id;
    assert_eq!(
        app.buffer_bytes(snapshot, dog_buffer),
        Err(xr::Result::ERROR_VALIDATION_FAILURE)
    );
    assert_eq!(app.buffer_string(snapshot, dog_buffer).unwrap(), "dog");

    app.destroy();
}

/// ALVR recentering moves every reference space but VIEW, in every application.
#[test]
fn recentering_is_reported() {
    let _serial = serial();
    let (mut writer, _requests) = start_driver();

    // An application without the marker extensions
    let (result, instance, gipa) = create_instance(&[]);
    assert_eq!(result, xr::Result::SUCCESS);
    // To report the change time
    assert_eq!(
        runtime().as_ref().unwrap().enabled_extensions,
        [TIME_EXTENSION.to_str().unwrap()]
    );
    let app = App { gipa, instance };

    // Changes before the session are not its own
    writer.set_origin(Origin {
        changes: 3,
        changed_ns: now_ns(),
        pose_in_previous: None,
        moved: true,
    });
    publish(&mut writer, &[]);
    let session = app.create_session();
    let poll: pfn::PollEvent = app.get(c"xrPollEvent");
    let poll_space_event = || {
        let mut event = xr::EventDataBuffer {
            ty: xr::StructureType::EVENT_DATA_BUFFER,
            next: ptr::null(),
            varying: [0; 4000],
        };
        match unsafe { poll(instance, &mut event) } {
            xr::Result::EVENT_UNAVAILABLE => None,
            xr::Result::SUCCESS => {
                assert_eq!(event.ty, xr::EventDataReferenceSpaceChangePending::TYPE);
                Some(unsafe {
                    *(&raw const event).cast::<xr::EventDataReferenceSpaceChangePending>()
                })
            }
            result => panic!("{result:?}"),
        }
    };

    assert!(poll_space_event().is_none());

    // The origin moves 1 m along +X
    let changed_ns = now_ns();
    writer.set_origin(Origin {
        changes: 4,
        changed_ns,
        pose_in_previous: Some(pose(1.0, 0.0, 0.0)),
        moved: true,
    });
    publish(&mut writer, &[]);
    let mut events = (0..2)
        .map(|_| poll_space_event().unwrap())
        .collect::<Vec<_>>();
    assert!(poll_space_event().is_none());
    events.sort_by_key(|event| event.reference_space_type.into_raw());
    let [local, stage] = events[..] else {
        unreachable!()
    };
    assert_eq!(local.reference_space_type, xr::ReferenceSpaceType::LOCAL);
    assert_eq!(stage.reference_space_type, xr::ReferenceSpaceType::STAGE);
    for event in [local, stage] {
        assert_eq!(event.session, session);
        assert_eq!(event.pose_valid, xr::TRUE);
        // To the performance counter's tick
        assert!((event.change_time.as_nanos() - changed_ns).abs() < 1000);
        assert!(close(
            event.pose_in_previous_space.position,
            [1.0, 0.0, 0.0]
        ));
    }

    // A change missed, or a driver restarted: where the origin went is not known
    writer.set_origin(Origin {
        changes: 6,
        changed_ns: now_ns(),
        pose_in_previous: Some(pose(1.0, 0.0, 0.0)),
        moved: true,
    });
    publish(&mut writer, &[]);
    for _ in 0..2 {
        assert_eq!(poll_space_event().unwrap().pose_valid, xr::FALSE);
    }
    assert!(poll_space_event().is_none());

    // Two moves before the application polls are one to it
    for (changes, x) in [(7, 1.0), (8, 0.5)] {
        writer.set_origin(Origin {
            changes,
            changed_ns: now_ns(),
            pose_in_previous: Some(pose(x, 0.0, 0.0)),
            moved: true,
        });
        publish(&mut writer, &[]);
    }
    for _ in 0..2 {
        let event = poll_space_event().unwrap();
        assert_eq!(event.pose_valid, xr::TRUE);
        assert!(close(
            event.pose_in_previous_space.position,
            [1.5, 0.0, 0.0]
        ));
    }
    assert!(poll_space_event().is_none());

    // Only the bounds changed: STAGE alone, where it was
    writer.set_origin(Origin {
        changes: 9,
        changed_ns: now_ns(),
        pose_in_previous: Some(pose(0.0, 0.0, 0.0)),
        moved: false,
    });
    publish(&mut writer, &[]);
    let event = poll_space_event().unwrap();
    assert_eq!(event.reference_space_type, xr::ReferenceSpaceType::STAGE);
    assert_eq!(event.pose_valid, xr::TRUE);
    assert!(close(
        event.pose_in_previous_space.position,
        [0.0, 0.0, 0.0]
    ));
    assert!(poll_space_event().is_none());

    let destroy_session: pfn::DestroySession = app.get(c"xrDestroySession");
    assert_eq!(unsafe { destroy_session(session) }, xr::Result::SUCCESS);
    app.destroy();
}
