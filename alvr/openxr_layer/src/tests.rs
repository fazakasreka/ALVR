//! The layer driven the way the OpenXR loader and an application drive it, on top of a fake
//! runtime that, like SteamVR, implements none of the extensions.

use crate::{LAYER_NAME, xrNegotiateLoaderApiLayerInterface};
use alvr_marker_shm::{FLAG_CLIENT_CONNECTED, Pose, Writer};
use openxr_sys::{self as xr, Handle, loader, pfn};
use std::{
    ffi::{CStr, c_char},
    mem, ptr,
    sync::{Mutex, MutexGuard},
};

pub fn shared_memory_name() -> String {
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

const RUNTIME_EXTENSIONS: [&CStr; 0] = [];

unsafe extern "system" fn runtime_create_instance(
    info: *const xr::InstanceCreateInfo,
    _layer_info: *const loader::ApiLayerCreateInfo,
    instance: *mut xr::Instance,
) -> xr::Result {
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
    unsafe { *space = xr::Space::from_raw(STAGE) };
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
    if space.into_raw() != STAGE || base.into_raw() != LOCAL {
        return xr::Result::ERROR_HANDLE_INVALID;
    }
    unsafe {
        (*location).location_flags =
            xr::SpaceLocationFlags::ORIENTATION_VALID | xr::SpaceLocationFlags::POSITION_VALID;
        (*location).pose = STAGE_IN_LOCAL;
    }
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
        b"xrPollEvent" => f!(runtime_poll_event, pfn::PollEvent),
        b"xrCreateSession" => f!(runtime_create_session, pfn::CreateSession),
        b"xrDestroySession" => f!(runtime_destroy_session, pfn::DestroySession),
        b"xrCreateReferenceSpace" => f!(runtime_create_reference_space, pfn::CreateReferenceSpace),
        b"xrDestroySpace" => f!(runtime_destroy_space, pfn::DestroySpace),
        b"xrLocateSpace" => f!(runtime_locate_space, pfn::LocateSpace),
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

const ALL_EXTENSIONS: [&CStr; 3] = [
    c"XR_EXT_future",
    c"XR_EXT_spatial_entity",
    c"XR_EXT_spatial_marker_tracking",
];

const SYSTEM: xr::SystemId = unsafe { mem::transmute(1_u64) };
const TIME: xr::Time = unsafe { mem::transmute(1_000_000_i64) };

struct App {
    gipa: pfn::GetInstanceProcAddr,
    instance: xr::Instance,
}

impl App {
    fn get<T>(&self, name: &CStr) -> T {
        get(self.gipa, self.instance, name)
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
        marker_size: Option<f32>,
    ) -> Result<xr::SpatialContextEXT, xr::Result> {
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
            optimize_for_static_marker: xr::TRUE,
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

    fn discover(&self, context: xr::SpatialContextEXT) -> xr::SpatialSnapshotEXT {
        let create: pfn::CreateSpatialDiscoverySnapshotAsyncEXT =
            self.get(c"xrCreateSpatialDiscoverySnapshotAsyncEXT");
        let complete: pfn::CreateSpatialDiscoverySnapshotCompleteEXT =
            self.get(c"xrCreateSpatialDiscoverySnapshotCompleteEXT");

        let info = xr::SpatialDiscoverySnapshotCreateInfoEXT {
            ty: xr::SpatialDiscoverySnapshotCreateInfoEXT::TYPE,
            next: ptr::null(),
            component_type_count: 0,
            component_types: ptr::null(),
        };
        let mut future = xr::FutureEXT::from_raw(0);
        assert_eq!(
            unsafe { create(context, &info, &mut future) },
            xr::Result::SUCCESS
        );

        let completion_info = xr::CreateSpatialDiscoverySnapshotCompletionInfoEXT {
            ty: xr::CreateSpatialDiscoverySnapshotCompletionInfoEXT::TYPE,
            next: ptr::null(),
            base_space: xr::Space::from_raw(LOCAL),
            time: TIME,
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
    let writer = Writer::create_named(&shared_memory_name()).unwrap();
    writer.publish(
        FLAG_CLIENT_CONNECTED,
        [
            ("dog", pose(1.0, 0.0, -2.0), 0.2),
            ("cat", pose(-1.0, 1.0, -3.0), 0.1),
        ],
    );

    let (result, instance, gipa) = create_instance(&ALL_EXTENSIONS);
    assert_eq!(result, xr::Result::SUCCESS);
    // SteamVR would have refused them
    assert!(runtime().as_ref().unwrap().enabled_extensions.is_empty());
    let app = App { gipa, instance };

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

    let create_session: pfn::CreateSession = app.get(c"xrCreateSession");
    let mut session = xr::Session::NULL;
    let result = unsafe { create_session(instance, ptr::null(), &mut session) };
    assert_eq!(result, xr::Result::SUCCESS);

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
            None
        ),
        Err(xr::Result::ERROR_SPATIAL_CAPABILITY_UNSUPPORTED_EXT)
    );
    assert_eq!(
        app.create_context(
            session,
            qr,
            xr::SpatialCapabilityConfigurationAprilTagEXT::TYPE,
            &both,
            None
        ),
        Err(xr::Result::ERROR_SPATIAL_CAPABILITY_CONFIGURATION_INVALID_EXT)
    );
    assert_eq!(
        app.create_context(
            session,
            qr,
            qr_config,
            &[xr::SpatialComponentTypeEXT::MESH_3D],
            None
        ),
        Err(xr::Result::ERROR_SPATIAL_COMPONENT_UNSUPPORTED_FOR_CAPABILITY_EXT)
    );

    let context = app
        .create_context(session, qr, qr_config, &both, None)
        .unwrap();

    // A new context gets a discovery recommendation, once
    assert_eq!(app.poll_event(), Some(context));
    assert_eq!(app.poll_event(), None);

    let snapshot = app.discover(context);
    let mut entities = app.query(snapshot);
    entities.sort_by_key(|entity| entity.2.center.position.x as i32);
    assert_eq!(entities.len(), 2);
    let (cat, dog) = (entities[0], entities[1]);
    assert_eq!(dog.0, alvr_marker_shm::marker_id("dog"));
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
        xr::Result::ERROR_VALIDATION_FAILURE
    );

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
    let update_snapshot = |entity: xr::SpatialEntityEXT| {
        let info = xr::SpatialUpdateSnapshotCreateInfoEXT {
            ty: xr::SpatialUpdateSnapshotCreateInfoEXT::TYPE,
            next: ptr::null(),
            entity_count: 1,
            entities: &entity,
            component_type_count: 0,
            component_types: ptr::null(),
            base_space: xr::Space::from_raw(LOCAL),
            time: TIME,
        };
        let mut snapshot = xr::SpatialSnapshotEXT::NULL;
        assert_eq!(
            unsafe { update(context, &info, &mut snapshot) },
            xr::Result::SUCCESS
        );
        snapshot
    };

    // The dog moves
    writer.publish(
        FLAG_CLIENT_CONNECTED,
        [
            ("dog", pose(2.0, 0.0, -2.0), 0.2),
            ("cat", pose(-1.0, 1.0, -3.0), 0.1),
        ],
    );
    let updated = app.query(update_snapshot(entity));
    assert_eq!(updated.len(), 1);
    assert_eq!(updated[0].1, xr::SpatialEntityTrackingStateEXT::TRACKING);
    assert!(close(updated[0].2.center.position, [2.0, -1.5, -2.0]));
    // Same markers, no recommendation
    assert_eq!(app.poll_event(), None);

    // The headset loses sight of it
    writer.publish(FLAG_CLIENT_CONNECTED, [("cat", pose(-1.0, 1.0, -3.0), 0.1)]);
    let updated = app.query(update_snapshot(entity));
    assert_eq!(updated[0].1, xr::SpatialEntityTrackingStateEXT::PAUSED);
    assert_eq!(app.poll_event(), Some(context));

    // A context that only enables markers cannot be asked for bounds
    let markers_only = app
        .create_context(
            session,
            qr,
            qr_config,
            &[xr::SpatialComponentTypeEXT::MARKER],
            Some(0.5),
        )
        .unwrap();
    let markers_only_snapshot = app.discover(markers_only);
    assert_eq!(
        unsafe { query(markers_only_snapshot, &condition, &mut query_result) },
        xr::Result::ERROR_SPATIAL_COMPONENT_NOT_ENABLED_EXT
    );

    // A known marker size wins over the detected one
    let destroy_context: pfn::DestroySpatialContextEXT = app.get(c"xrDestroySpatialContextEXT");
    assert_eq!(
        unsafe { destroy_context(markers_only) },
        xr::Result::SUCCESS
    );
    let sized = app
        .create_context(session, qr, qr_config, &both, Some(0.5))
        .unwrap();
    let sized_entities = app.query(app.discover(sized));
    assert_eq!(sized_entities[0].2.extents.width, 0.5);
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
    let updated = app.query(update_snapshot(entity));
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
fn applications_without_the_extensions_go_straight_to_the_runtime() {
    let _serial = serial();
    let (result, instance, gipa) = create_instance(&[]);
    assert_eq!(result, xr::Result::SUCCESS);

    // The runtime's own function, not the layer's
    let poll: pfn::PollEvent = get(gipa, instance, c"xrPollEvent");
    assert_eq!(poll as usize, runtime_poll_event as pfn::PollEvent as usize);

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
