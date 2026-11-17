//! XR_EXT_spatial_entity with the QR code capability of XR_EXT_spatial_marker_tracking, and the
//! parts of XR_EXT_future they use.
//!
//! Every QR code the driver publishes is an entity with the marker and 2D bounds components. Its
//! entity id comes from its text, so it stays the same across snapshots, contexts and sessions.
//! Asynchronous operations complete immediately: their futures are ready as soon as they exist.

use crate::{Downstream, State, log, math, state, void_fn};
use alvr_marker_shm::Snapshot as Markers;
use openxr_sys::{self as xr, Handle, pfn};
use std::{
    collections::HashSet,
    ffi::{CStr, c_char, c_void},
    mem, ptr, slice,
};

const QR_COMPONENTS: [xr::SpatialComponentTypeEXT; 2] = [
    xr::SpatialComponentTypeEXT::MARKER,
    xr::SpatialComponentTypeEXT::BOUNDED_2D,
];
const QR_FEATURES: [xr::SpatialCapabilityFeatureEXT; 2] = [
    xr::SpatialCapabilityFeatureEXT::MARKER_TRACKING_FIXED_SIZE_MARKERS,
    xr::SpatialCapabilityFeatureEXT::MARKER_TRACKING_STATIC_MARKERS,
];

pub(crate) struct SessionData {
    pub instance: u64,
    /// Created the first time markers are located for the session.
    stage: Option<xr::Space>,
}

#[derive(Clone)]
pub(crate) struct ContextData {
    session: u64,
    components: Vec<xr::SpatialComponentTypeEXT>,
    /// From XrSpatialMarkerSizeEXT: the application knows the size of its markers.
    marker_size: Option<f32>,
    /// Entities seen in this context's discovery snapshots, which may be created from their ids.
    discovered: HashSet<u64>,
    /// The marker ids the last discovery recommendation was sent for.
    announced: Option<Vec<u64>>,
}

pub(crate) struct SnapshotData {
    context: u64,
    entities: Vec<SnapshotEntity>,
}

struct SnapshotEntity {
    id: u64,
    state: xr::SpatialEntityTrackingStateEXT,
    bounds: xr::SpatialBounded2DDataEXT,
    text: String,
}

pub(crate) struct EntityData {
    context: u64,
    id: u64,
}

pub(crate) struct FutureData {
    pub instance: u64,
    kind: FutureKind,
}

enum FutureKind {
    Context { session: u64, context: ContextData },
    Discovery { context: u64 },
}

pub(crate) fn function(name: &CStr) -> Option<pfn::VoidFunction> {
    Some(match name.to_bytes() {
        b"xrCreateSession" => void_fn!(create_session, pfn::CreateSession),
        b"xrDestroySession" => void_fn!(destroy_session, pfn::DestroySession),
        b"xrPollEvent" => void_fn!(poll_event, pfn::PollEvent),
        b"xrEnumerateSpatialCapabilitiesEXT" => {
            void_fn!(enumerate_capabilities, pfn::EnumerateSpatialCapabilitiesEXT)
        }
        b"xrEnumerateSpatialCapabilityComponentTypesEXT" => void_fn!(
            enumerate_capability_component_types,
            pfn::EnumerateSpatialCapabilityComponentTypesEXT
        ),
        b"xrEnumerateSpatialCapabilityFeaturesEXT" => void_fn!(
            enumerate_capability_features,
            pfn::EnumerateSpatialCapabilityFeaturesEXT
        ),
        b"xrCreateSpatialContextAsyncEXT" => {
            void_fn!(create_context_async, pfn::CreateSpatialContextAsyncEXT)
        }
        b"xrCreateSpatialContextCompleteEXT" => {
            void_fn!(
                create_context_complete,
                pfn::CreateSpatialContextCompleteEXT
            )
        }
        b"xrDestroySpatialContextEXT" => void_fn!(destroy_context, pfn::DestroySpatialContextEXT),
        b"xrCreateSpatialDiscoverySnapshotAsyncEXT" => void_fn!(
            create_discovery_snapshot_async,
            pfn::CreateSpatialDiscoverySnapshotAsyncEXT
        ),
        b"xrCreateSpatialDiscoverySnapshotCompleteEXT" => void_fn!(
            create_discovery_snapshot_complete,
            pfn::CreateSpatialDiscoverySnapshotCompleteEXT
        ),
        b"xrQuerySpatialComponentDataEXT" => {
            void_fn!(query_component_data, pfn::QuerySpatialComponentDataEXT)
        }
        b"xrDestroySpatialSnapshotEXT" => {
            void_fn!(destroy_snapshot, pfn::DestroySpatialSnapshotEXT)
        }
        b"xrCreateSpatialEntityFromIdEXT" => {
            void_fn!(create_entity_from_id, pfn::CreateSpatialEntityFromIdEXT)
        }
        b"xrDestroySpatialEntityEXT" => void_fn!(destroy_entity, pfn::DestroySpatialEntityEXT),
        b"xrCreateSpatialUpdateSnapshotEXT" => {
            void_fn!(create_update_snapshot, pfn::CreateSpatialUpdateSnapshotEXT)
        }
        b"xrGetSpatialBufferStringEXT" => {
            void_fn!(get_buffer_string, pfn::GetSpatialBufferStringEXT)
        }
        b"xrGetSpatialBufferUint8EXT" => {
            void_fn!(get_buffer_uint8, pfn::GetSpatialBufferUint8EXT)
        }
        b"xrGetSpatialBufferUint16EXT" => {
            void_fn!(get_buffer_uint16, pfn::GetSpatialBufferUint16EXT)
        }
        b"xrGetSpatialBufferUint32EXT" => {
            void_fn!(get_buffer_uint32, pfn::GetSpatialBufferUint32EXT)
        }
        b"xrGetSpatialBufferFloatEXT" => {
            void_fn!(get_buffer_float, pfn::GetSpatialBufferFloatEXT)
        }
        b"xrGetSpatialBufferVector2fEXT" => {
            void_fn!(get_buffer_vector2f, pfn::GetSpatialBufferVector2fEXT)
        }
        b"xrGetSpatialBufferVector3fEXT" => {
            void_fn!(get_buffer_vector3f, pfn::GetSpatialBufferVector3fEXT)
        }
        _ => return future_function(name),
    })
}

/// XR_EXT_future, for the layer's own futures. Others go to the runtime when it has the extension.
pub(crate) fn future_function(name: &CStr) -> Option<pfn::VoidFunction> {
    Some(match name.to_bytes() {
        b"xrPollFutureEXT" => void_fn!(poll_future, pfn::PollFutureEXT),
        b"xrCancelFutureEXT" => void_fn!(cancel_future, pfn::CancelFutureEXT),
        _ => return None,
    })
}

/// The two-call idiom: reports the count, and fills `output` when its capacity allows.
unsafe fn write_array<T: Copy>(
    items: &[T],
    capacity: u32,
    count_output: *mut u32,
    output: *mut T,
) -> xr::Result {
    if count_output.is_null() {
        return xr::Result::ERROR_VALIDATION_FAILURE;
    }
    unsafe { *count_output = items.len() as u32 };

    if capacity == 0 {
        return xr::Result::SUCCESS;
    }
    if (capacity as usize) < items.len() {
        return xr::Result::ERROR_SIZE_INSUFFICIENT;
    }
    if output.is_null() {
        return xr::Result::ERROR_VALIDATION_FAILURE;
    }
    unsafe { ptr::copy_nonoverlapping(items.as_ptr(), output, items.len()) };

    xr::Result::SUCCESS
}

unsafe fn read_array<'a, T>(items: *const T, count: u32) -> &'a [T] {
    if count == 0 || items.is_null() {
        &[]
    } else {
        unsafe { slice::from_raw_parts(items, count as usize) }
    }
}

/// The structures chained to `next`, as (type, pointer) pairs.
unsafe fn chain(mut next: *const c_void) -> Vec<(xr::StructureType, *const c_void)> {
    let mut structs = Vec::new();
    while !next.is_null() {
        let header = unsafe { &*next.cast::<xr::BaseInStructure>() };
        structs.push((header.ty, next));
        next = header.next.cast();
    }

    structs
}

impl State {
    fn downstream(&self, instance: u64) -> Option<Downstream> {
        self.instances.get(&instance).map(|data| data.downstream)
    }

    /// Whether the driver is running, so that QR codes can be offered.
    fn qr_codes_available(&mut self) -> bool {
        self.markers.read().is_some()
    }

    fn check_instance(&self, instance: xr::Instance, system: xr::SystemId) -> xr::Result {
        if !self
            .instances
            .get(&instance.into_raw())
            .is_some_and(|data| data.provides_spatial)
        {
            xr::Result::ERROR_HANDLE_INVALID
        } else if system == xr::SystemId::NULL {
            xr::Result::ERROR_SYSTEM_INVALID
        } else {
            xr::Result::SUCCESS
        }
    }

    pub(crate) fn remove_session(&mut self, session: u64) -> Option<SessionData> {
        let data = self.sessions.remove(&session)?;

        let contexts = self
            .contexts
            .iter()
            .filter(|(_, context)| context.session == session)
            .map(|(&handle, _)| handle)
            .collect::<Vec<_>>();
        for context in contexts {
            self.remove_context(context);
        }
        self.futures.retain(|_, future| {
            !matches!(future.kind, FutureKind::Context { session: s, .. } if s == session)
        });

        Some(data)
    }

    fn remove_context(&mut self, context: u64) -> bool {
        if self.contexts.remove(&context).is_none() {
            return false;
        }

        self.snapshots
            .retain(|_, snapshot| snapshot.context != context);
        self.entities.retain(|_, entity| entity.context != context);
        self.futures.retain(|_, future| {
            !matches!(future.kind, FutureKind::Discovery { context: c } if c == context)
        });

        true
    }

    /// The pose of the runtime's STAGE space, which marker poses are in, in `base_space`.
    /// `Ok(None)` when it cannot be located at `time`.
    fn locate_stage(
        &mut self,
        session: u64,
        base_space: xr::Space,
        time: xr::Time,
    ) -> Result<Option<xr::Posef>, xr::Result> {
        let instance = self.sessions[&session].instance;
        let downstream = self
            .downstream(instance)
            .ok_or(xr::Result::ERROR_HANDLE_INVALID)?;

        let stage = match self.sessions[&session].stage {
            Some(stage) => stage,
            None => {
                let info = xr::ReferenceSpaceCreateInfo {
                    ty: xr::ReferenceSpaceCreateInfo::TYPE,
                    next: ptr::null(),
                    reference_space_type: xr::ReferenceSpaceType::STAGE,
                    pose_in_reference_space: xr::Posef::IDENTITY,
                };
                let mut stage = xr::Space::NULL;
                let result = unsafe {
                    (downstream.create_reference_space)(
                        xr::Session::from_raw(session),
                        &info,
                        &mut stage,
                    )
                };
                if result != xr::Result::SUCCESS {
                    log!("Cannot create the stage space: {result:?}");
                    return Ok(None);
                }
                self.sessions.get_mut(&session).unwrap().stage = Some(stage);

                stage
            }
        };

        let mut location = xr::SpaceLocation {
            ty: xr::SpaceLocation::TYPE,
            next: ptr::null_mut(),
            location_flags: xr::SpaceLocationFlags::EMPTY,
            pose: xr::Posef::IDENTITY,
        };
        let result = unsafe { (downstream.locate_space)(stage, base_space, time, &mut location) };
        if result != xr::Result::SUCCESS {
            return Err(result);
        }

        let valid =
            xr::SpaceLocationFlags::ORIENTATION_VALID | xr::SpaceLocationFlags::POSITION_VALID;
        Ok(location
            .location_flags
            .contains(valid)
            .then_some(location.pose))
    }

    fn snapshot_entity(
        context: &ContextData,
        marker: &alvr_marker_shm::Marker,
        stage: Option<xr::Posef>,
    ) -> SnapshotEntity {
        let Some(stage) = stage else {
            return SnapshotEntity {
                id: marker.id,
                state: xr::SpatialEntityTrackingStateEXT::PAUSED,
                bounds: xr::SpatialBounded2DDataEXT::default(),
                text: marker.text.clone(),
            };
        };

        let [x, y, z, w] = marker.pose.orientation;
        let [px, py, pz] = marker.pose.position;
        let pose = xr::Posef {
            orientation: xr::Quaternionf { x, y, z, w },
            position: xr::Vector3f {
                x: px,
                y: py,
                z: pz,
            },
        };
        let size = context.marker_size.unwrap_or(marker.size);

        SnapshotEntity {
            id: marker.id,
            state: xr::SpatialEntityTrackingStateEXT::TRACKING,
            bounds: xr::SpatialBounded2DDataEXT {
                center: math::compose(stage, pose),
                extents: xr::Extent2Df {
                    width: size,
                    height: size,
                },
            },
            text: marker.text.clone(),
        }
    }

    /// A context of `instance` to recommend a discovery for, when the markers have changed since
    /// its last recommendation.
    fn take_discovery_recommendation(&mut self, instance: u64) -> Option<u64> {
        let mut ids: Vec<u64> = self
            .markers
            .read()
            .map(|markers: Markers| markers.markers.iter().map(|marker| marker.id).collect())
            .unwrap_or_default();
        ids.sort_unstable();

        let sessions = &self.sessions;
        let (&handle, context) = self.contexts.iter_mut().find(|(_, context)| {
            sessions
                .get(&context.session)
                .is_some_and(|session| session.instance == instance)
                && context.announced.as_ref() != Some(&ids)
        })?;
        context.announced = Some(ids);

        Some(handle)
    }
}

unsafe extern "system" fn create_session(
    instance: xr::Instance,
    info: *const xr::SessionCreateInfo,
    session: *mut xr::Session,
) -> xr::Result {
    let Some(downstream) = state().downstream(instance.into_raw()) else {
        return xr::Result::ERROR_HANDLE_INVALID;
    };

    let result = unsafe { (downstream.create_session)(instance, info, session) };
    if result.into_raw() >= 0 {
        state().sessions.insert(
            unsafe { *session }.into_raw(),
            SessionData {
                instance: instance.into_raw(),
                stage: None,
            },
        );
    }

    result
}

unsafe extern "system" fn destroy_session(session: xr::Session) -> xr::Result {
    let (data, downstream) = {
        let mut state = state();
        let Some(data) = state.remove_session(session.into_raw()) else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };
        let Some(downstream) = state.downstream(data.instance) else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };
        (data, downstream)
    };

    if let Some(stage) = data.stage {
        unsafe { (downstream.destroy_space)(stage) };
    }

    unsafe { (downstream.destroy_session)(session) }
}

unsafe extern "system" fn poll_event(
    instance: xr::Instance,
    event: *mut xr::EventDataBuffer,
) -> xr::Result {
    let Some(downstream) = state().downstream(instance.into_raw()) else {
        return xr::Result::ERROR_HANDLE_INVALID;
    };

    let result = unsafe { (downstream.poll_event)(instance, event) };
    if result != xr::Result::EVENT_UNAVAILABLE || event.is_null() {
        return result;
    }

    // The runtime's events come first, so the layer's never delay them
    let Some(context) = state().take_discovery_recommendation(instance.into_raw()) else {
        return result;
    };
    unsafe {
        event
            .cast::<xr::EventDataSpatialDiscoveryRecommendedEXT>()
            .write(xr::EventDataSpatialDiscoveryRecommendedEXT {
                ty: xr::EventDataSpatialDiscoveryRecommendedEXT::TYPE,
                next: ptr::null(),
                spatial_context: xr::SpatialContextEXT::from_raw(context),
            })
    };

    xr::Result::SUCCESS
}

unsafe extern "system" fn poll_future(
    instance: xr::Instance,
    info: *const xr::FuturePollInfoEXT,
    result: *mut xr::FuturePollResultEXT,
) -> xr::Result {
    if info.is_null() || result.is_null() {
        return xr::Result::ERROR_VALIDATION_FAILURE;
    }
    let future = unsafe { (*info).future }.into_raw();

    let state = state();
    if state
        .futures
        .get(&future)
        .is_some_and(|data| data.instance == instance.into_raw())
    {
        unsafe { (*result).state = xr::FutureStateEXT::READY };
        return xr::Result::SUCCESS;
    }

    match state
        .downstream(instance.into_raw())
        .and_then(|downstream| downstream.poll_future)
    {
        Some(poll_future) => {
            drop(state);
            unsafe { poll_future(instance, info, result) }
        }
        None => xr::Result::ERROR_FUTURE_INVALID_EXT,
    }
}

unsafe extern "system" fn cancel_future(
    instance: xr::Instance,
    info: *const xr::FutureCancelInfoEXT,
) -> xr::Result {
    if info.is_null() {
        return xr::Result::ERROR_VALIDATION_FAILURE;
    }
    let future = unsafe { (*info).future }.into_raw();

    let mut state = state();
    if state
        .futures
        .get(&future)
        .is_some_and(|data| data.instance == instance.into_raw())
    {
        state.futures.remove(&future);
        return xr::Result::SUCCESS;
    }

    match state
        .downstream(instance.into_raw())
        .and_then(|downstream| downstream.cancel_future)
    {
        Some(cancel_future) => {
            drop(state);
            unsafe { cancel_future(instance, info) }
        }
        None => xr::Result::ERROR_FUTURE_INVALID_EXT,
    }
}

unsafe extern "system" fn enumerate_capabilities(
    instance: xr::Instance,
    system: xr::SystemId,
    capacity: u32,
    count_output: *mut u32,
    capabilities: *mut xr::SpatialCapabilityEXT,
) -> xr::Result {
    let mut state = state();
    let result = state.check_instance(instance, system);
    if result != xr::Result::SUCCESS {
        return result;
    }

    let available: &[_] = if state.qr_codes_available() {
        &[xr::SpatialCapabilityEXT::MARKER_TRACKING_QR_CODE]
    } else {
        &[]
    };

    unsafe { write_array(available, capacity, count_output, capabilities) }
}

unsafe extern "system" fn enumerate_capability_component_types(
    instance: xr::Instance,
    system: xr::SystemId,
    capability: xr::SpatialCapabilityEXT,
    components: *mut xr::SpatialCapabilityComponentTypesEXT,
) -> xr::Result {
    let mut state = state();
    let result = state.check_instance(instance, system);
    if result != xr::Result::SUCCESS {
        return result;
    }
    if components.is_null()
        || unsafe { (*components).ty } != xr::SpatialCapabilityComponentTypesEXT::TYPE
    {
        return xr::Result::ERROR_VALIDATION_FAILURE;
    }
    if capability != xr::SpatialCapabilityEXT::MARKER_TRACKING_QR_CODE
        || !state.qr_codes_available()
    {
        return xr::Result::ERROR_SPATIAL_CAPABILITY_UNSUPPORTED_EXT;
    }

    let components = unsafe { &mut *components };
    unsafe {
        write_array(
            &QR_COMPONENTS,
            components.component_type_capacity_input,
            &mut components.component_type_count_output,
            components.component_types,
        )
    }
}

unsafe extern "system" fn enumerate_capability_features(
    instance: xr::Instance,
    system: xr::SystemId,
    capability: xr::SpatialCapabilityEXT,
    capacity: u32,
    count_output: *mut u32,
    features: *mut xr::SpatialCapabilityFeatureEXT,
) -> xr::Result {
    let mut state = state();
    let result = state.check_instance(instance, system);
    if result != xr::Result::SUCCESS {
        return result;
    }
    if capability != xr::SpatialCapabilityEXT::MARKER_TRACKING_QR_CODE
        || !state.qr_codes_available()
    {
        return xr::Result::ERROR_SPATIAL_CAPABILITY_UNSUPPORTED_EXT;
    }

    unsafe { write_array(&QR_FEATURES, capacity, count_output, features) }
}

/// Validates the capability configurations of a context.
unsafe fn context_config(
    session: u64,
    info: &xr::SpatialContextCreateInfoEXT,
) -> Result<ContextData, xr::Result> {
    let configs = unsafe { read_array(info.capability_configs, info.capability_config_count) };
    if configs.is_empty() {
        return Err(xr::Result::ERROR_VALIDATION_FAILURE);
    }

    let mut context = ContextData {
        session,
        components: Vec::new(),
        marker_size: None,
        discovered: HashSet::new(),
        announced: None,
    };
    for &config in configs {
        if config.is_null() {
            return Err(xr::Result::ERROR_VALIDATION_FAILURE);
        }
        let config = unsafe { &*config };

        if config.capability != xr::SpatialCapabilityEXT::MARKER_TRACKING_QR_CODE {
            return Err(xr::Result::ERROR_SPATIAL_CAPABILITY_UNSUPPORTED_EXT);
        }
        if config.ty != xr::SpatialCapabilityConfigurationQrCodeEXT::TYPE {
            return Err(xr::Result::ERROR_SPATIAL_CAPABILITY_CONFIGURATION_INVALID_EXT);
        }

        let components =
            unsafe { read_array(config.enabled_components, config.enabled_component_count) };
        if components.is_empty() {
            return Err(xr::Result::ERROR_VALIDATION_FAILURE);
        }
        for component in components {
            if !QR_COMPONENTS.contains(component) {
                return Err(xr::Result::ERROR_SPATIAL_COMPONENT_UNSUPPORTED_FOR_CAPABILITY_EXT);
            }
            if !context.components.contains(component) {
                context.components.push(*component);
            }
        }

        for (ty, structure) in unsafe { chain(config.next) } {
            if ty == xr::SpatialMarkerSizeEXT::TYPE {
                let size =
                    unsafe { (*structure.cast::<xr::SpatialMarkerSizeEXT>()).marker_side_length };
                if size.is_nan() || size <= 0.0 {
                    return Err(xr::Result::ERROR_VALIDATION_FAILURE);
                }
                context.marker_size = Some(size);
            }
            // XrSpatialMarkerStaticOptimizationEXT needs nothing: the headset already treats
            // every QR code as static
        }
    }

    Ok(context)
}

unsafe extern "system" fn create_context_async(
    session: xr::Session,
    info: *const xr::SpatialContextCreateInfoEXT,
    future: *mut xr::FutureEXT,
) -> xr::Result {
    if info.is_null()
        || future.is_null()
        || unsafe { (*info).ty } != xr::SpatialContextCreateInfoEXT::TYPE
    {
        return xr::Result::ERROR_VALIDATION_FAILURE;
    }

    let mut state = state();
    let Some(instance) = state
        .sessions
        .get(&session.into_raw())
        .map(|data| data.instance)
    else {
        return xr::Result::ERROR_HANDLE_INVALID;
    };
    if !state.qr_codes_available() {
        return xr::Result::ERROR_SPATIAL_CAPABILITY_UNSUPPORTED_EXT;
    }
    let context = match unsafe { context_config(session.into_raw(), &*info) } {
        Ok(context) => context,
        Err(result) => return result,
    };

    let handle = state.new_handle();
    state.futures.insert(
        handle,
        FutureData {
            instance,
            kind: FutureKind::Context {
                session: session.into_raw(),
                context,
            },
        },
    );
    unsafe { *future = xr::FutureEXT::from_raw(handle) };

    xr::Result::SUCCESS
}

unsafe extern "system" fn create_context_complete(
    session: xr::Session,
    future: xr::FutureEXT,
    completion: *mut xr::CreateSpatialContextCompletionEXT,
) -> xr::Result {
    if completion.is_null()
        || unsafe { (*completion).ty } != xr::CreateSpatialContextCompletionEXT::TYPE
    {
        return xr::Result::ERROR_VALIDATION_FAILURE;
    }

    let mut state = state();
    if !state.sessions.contains_key(&session.into_raw()) {
        return xr::Result::ERROR_HANDLE_INVALID;
    }
    let context = match state.futures.get(&future.into_raw()) {
        Some(FutureData {
            kind: FutureKind::Context { session: s, .. },
            ..
        }) if *s == session.into_raw() => {
            let Some(FutureData {
                kind: FutureKind::Context { context, .. },
                ..
            }) = state.futures.remove(&future.into_raw())
            else {
                unreachable!()
            };
            context
        }
        _ => return xr::Result::ERROR_FUTURE_INVALID_EXT,
    };

    let handle = state.new_handle();
    log!("Created spatial context {handle:#x} for QR codes");
    state.contexts.insert(handle, context);

    unsafe {
        (*completion).future_result = xr::Result::SUCCESS;
        (*completion).spatial_context = xr::SpatialContextEXT::from_raw(handle);
    }

    xr::Result::SUCCESS
}

unsafe extern "system" fn destroy_context(context: xr::SpatialContextEXT) -> xr::Result {
    if state().remove_context(context.into_raw()) {
        xr::Result::SUCCESS
    } else {
        xr::Result::ERROR_HANDLE_INVALID
    }
}

/// Checks that the context has every requested component enabled.
fn check_components(
    context: &ContextData,
    components: &[xr::SpatialComponentTypeEXT],
) -> xr::Result {
    if components
        .iter()
        .all(|component| context.components.contains(component))
    {
        xr::Result::SUCCESS
    } else {
        xr::Result::ERROR_SPATIAL_COMPONENT_NOT_ENABLED_EXT
    }
}

unsafe extern "system" fn create_discovery_snapshot_async(
    context: xr::SpatialContextEXT,
    info: *const xr::SpatialDiscoverySnapshotCreateInfoEXT,
    future: *mut xr::FutureEXT,
) -> xr::Result {
    if info.is_null()
        || future.is_null()
        || unsafe { (*info).ty } != xr::SpatialDiscoverySnapshotCreateInfoEXT::TYPE
    {
        return xr::Result::ERROR_VALIDATION_FAILURE;
    }

    let mut state = state();
    let Some(context_data) = state.contexts.get(&context.into_raw()) else {
        return xr::Result::ERROR_HANDLE_INVALID;
    };
    // Every QR code has every component, so the requested ones do not filter entities
    let components = unsafe { read_array((*info).component_types, (*info).component_type_count) };
    let result = check_components(context_data, components);
    if result != xr::Result::SUCCESS {
        return result;
    }
    let instance = state.sessions[&context_data.session].instance;

    let handle = state.new_handle();
    state.futures.insert(
        handle,
        FutureData {
            instance,
            kind: FutureKind::Discovery {
                context: context.into_raw(),
            },
        },
    );
    unsafe { *future = xr::FutureEXT::from_raw(handle) };

    xr::Result::SUCCESS
}

unsafe extern "system" fn create_discovery_snapshot_complete(
    context: xr::SpatialContextEXT,
    info: *const xr::CreateSpatialDiscoverySnapshotCompletionInfoEXT,
    completion: *mut xr::CreateSpatialDiscoverySnapshotCompletionEXT,
) -> xr::Result {
    if info.is_null()
        || completion.is_null()
        || unsafe { (*info).ty } != xr::CreateSpatialDiscoverySnapshotCompletionInfoEXT::TYPE
        || unsafe { (*completion).ty } != xr::CreateSpatialDiscoverySnapshotCompletionEXT::TYPE
    {
        return xr::Result::ERROR_VALIDATION_FAILURE;
    }
    let info = unsafe { &*info };
    if info.time.as_nanos() <= 0 {
        return xr::Result::ERROR_TIME_INVALID;
    }

    let mut state = state();
    let Some(session) = state
        .contexts
        .get(&context.into_raw())
        .map(|data| data.session)
    else {
        return xr::Result::ERROR_HANDLE_INVALID;
    };
    if !matches!(
        state.futures.get(&info.future.into_raw()),
        Some(FutureData { kind: FutureKind::Discovery { context: c }, .. }) if *c == context.into_raw()
    ) {
        return xr::Result::ERROR_FUTURE_INVALID_EXT;
    }

    let stage = match state.locate_stage(session, info.base_space, info.time) {
        Ok(stage) => stage,
        Err(result) => return result,
    };
    state.futures.remove(&info.future.into_raw());

    let markers = state
        .markers
        .read()
        .map(|markers| markers.markers)
        .unwrap_or_default();
    let context_data = state.contexts.get_mut(&context.into_raw()).unwrap();
    let entities = markers
        .iter()
        .map(|marker| {
            context_data.discovered.insert(marker.id);
            State::snapshot_entity(context_data, marker, stage)
        })
        .collect();

    let handle = state.new_handle();
    state.snapshots.insert(
        handle,
        SnapshotData {
            context: context.into_raw(),
            entities,
        },
    );

    unsafe {
        (*completion).future_result = xr::Result::SUCCESS;
        (*completion).snapshot = xr::SpatialSnapshotEXT::from_raw(handle);
    }

    xr::Result::SUCCESS
}

unsafe extern "system" fn create_update_snapshot(
    context: xr::SpatialContextEXT,
    info: *const xr::SpatialUpdateSnapshotCreateInfoEXT,
    snapshot: *mut xr::SpatialSnapshotEXT,
) -> xr::Result {
    if info.is_null()
        || snapshot.is_null()
        || unsafe { (*info).ty } != xr::SpatialUpdateSnapshotCreateInfoEXT::TYPE
    {
        return xr::Result::ERROR_VALIDATION_FAILURE;
    }
    let info = unsafe { &*info };
    if info.time.as_nanos() <= 0 {
        return xr::Result::ERROR_TIME_INVALID;
    }

    let mut state = state();
    let Some(context_data) = state.contexts.get(&context.into_raw()) else {
        return xr::Result::ERROR_HANDLE_INVALID;
    };
    let components = unsafe { read_array(info.component_types, info.component_type_count) };
    let result = check_components(context_data, components);
    if result != xr::Result::SUCCESS {
        return result;
    }
    let session = context_data.session;

    let entity_handles = unsafe { read_array(info.entities, info.entity_count) };
    if entity_handles.is_empty() {
        return xr::Result::ERROR_VALIDATION_FAILURE;
    }
    let mut ids = Vec::with_capacity(entity_handles.len());
    for entity in entity_handles {
        match state.entities.get(&entity.into_raw()) {
            Some(data) if data.context == context.into_raw() => ids.push(data.id),
            _ => return xr::Result::ERROR_HANDLE_INVALID,
        }
    }

    let stage = match state.locate_stage(session, info.base_space, info.time) {
        Ok(stage) => stage,
        Err(result) => return result,
    };

    let markers = state
        .markers
        .read()
        .map(|markers| markers.markers)
        .unwrap_or_default();
    let context_data = &state.contexts[&context.into_raw()];
    let entities = ids
        .iter()
        .map(|&id| match markers.iter().find(|marker| marker.id == id) {
            Some(marker) => State::snapshot_entity(context_data, marker, stage),
            // Out of the headset's sight, or the headset is not connected: it may come back
            None => SnapshotEntity {
                id,
                state: xr::SpatialEntityTrackingStateEXT::PAUSED,
                bounds: xr::SpatialBounded2DDataEXT::default(),
                text: String::new(),
            },
        })
        .collect();

    let handle = state.new_handle();
    state.snapshots.insert(
        handle,
        SnapshotData {
            context: context.into_raw(),
            entities,
        },
    );
    unsafe { *snapshot = xr::SpatialSnapshotEXT::from_raw(handle) };

    xr::Result::SUCCESS
}

unsafe extern "system" fn query_component_data(
    snapshot: xr::SpatialSnapshotEXT,
    condition: *const xr::SpatialComponentDataQueryConditionEXT,
    result: *mut xr::SpatialComponentDataQueryResultEXT,
) -> xr::Result {
    if condition.is_null()
        || result.is_null()
        || unsafe { (*condition).ty } != xr::SpatialComponentDataQueryConditionEXT::TYPE
        || unsafe { (*result).ty } != xr::SpatialComponentDataQueryResultEXT::TYPE
    {
        return xr::Result::ERROR_VALIDATION_FAILURE;
    }
    let (condition, result) = unsafe { (&*condition, &mut *result) };

    let state = state();
    let Some(snapshot_data) = state.snapshots.get(&snapshot.into_raw()) else {
        return xr::Result::ERROR_HANDLE_INVALID;
    };
    let context = &state.contexts[&snapshot_data.context];
    let components =
        unsafe { read_array(condition.component_types, condition.component_type_count) };
    let check = check_components(context, components);
    if check != xr::Result::SUCCESS {
        return check;
    }

    let mut tracking_state = None;
    for (ty, structure) in unsafe { chain(condition.next) } {
        if ty == xr::SpatialFilterTrackingStateEXT::TYPE {
            tracking_state = Some(unsafe {
                (*structure.cast::<xr::SpatialFilterTrackingStateEXT>()).tracking_state
            });
        }
    }
    let entities = snapshot_data
        .entities
        .iter()
        .filter(|entity| tracking_state.is_none_or(|state| entity.state == state))
        .collect::<Vec<_>>();
    let count = entities.len() as u32;

    result.entity_id_count_output = count;
    result.entity_state_count_output = count;
    if result.entity_id_capacity_input == 0 {
        return xr::Result::SUCCESS;
    }
    if result.entity_id_capacity_input < count
        || (result.entity_state_capacity_input != 0 && result.entity_state_capacity_input < count)
    {
        return xr::Result::ERROR_SIZE_INSUFFICIENT;
    }
    if result.entity_ids.is_null()
        || (result.entity_state_capacity_input != 0 && result.entity_states.is_null())
    {
        return xr::Result::ERROR_VALIDATION_FAILURE;
    }

    // Check every component list before writing anything
    let lists = unsafe { chain(result.next) };
    for &(ty, structure) in &lists {
        let (component, list_count, list_ptr) = if ty == xr::SpatialComponentBounded2DListEXT::TYPE
        {
            let list = unsafe { &*structure.cast::<xr::SpatialComponentBounded2DListEXT>() };
            (
                xr::SpatialComponentTypeEXT::BOUNDED_2D,
                list.bound_count,
                list.bounds.cast::<c_void>(),
            )
        } else if ty == xr::SpatialComponentMarkerListEXT::TYPE {
            let list = unsafe { &*structure.cast::<xr::SpatialComponentMarkerListEXT>() };
            (
                xr::SpatialComponentTypeEXT::MARKER,
                list.marker_count,
                list.markers.cast::<c_void>(),
            )
        } else {
            continue;
        };
        if !context.components.contains(&component) {
            return xr::Result::ERROR_SPATIAL_COMPONENT_NOT_ENABLED_EXT;
        }
        if list_count != count || (count > 0 && list_ptr.is_null()) {
            return xr::Result::ERROR_VALIDATION_FAILURE;
        }
    }

    for (i, entity) in entities.iter().enumerate() {
        unsafe {
            *result.entity_ids.add(i) = xr::SpatialEntityIdEXT::from_raw(entity.id);
            if result.entity_state_capacity_input != 0 {
                *result.entity_states.add(i) = entity.state;
            }
        }
    }
    for (ty, structure) in lists {
        for (i, entity) in entities.iter().enumerate() {
            // Component data is only valid for tracked entities
            if entity.state != xr::SpatialEntityTrackingStateEXT::TRACKING {
                continue;
            }

            if ty == xr::SpatialComponentBounded2DListEXT::TYPE {
                let list = unsafe { &*structure.cast::<xr::SpatialComponentBounded2DListEXT>() };
                unsafe { *list.bounds.add(i) = entity.bounds };
            } else if ty == xr::SpatialComponentMarkerListEXT::TYPE {
                let list = unsafe { &*structure.cast::<xr::SpatialComponentMarkerListEXT>() };
                unsafe {
                    *list.markers.add(i) = xr::SpatialMarkerDataEXT {
                        capability: xr::SpatialCapabilityEXT::MARKER_TRACKING_QR_CODE,
                        // Only ArUco markers and AprilTags have a numeric id
                        marker_id: 0,
                        data: xr::SpatialBufferEXT {
                            // The buffer of a QR code is its text, identified by its entity
                            buffer_id: xr::SpatialBufferIdEXT::from_raw(entity.id),
                            buffer_type: xr::SpatialBufferTypeEXT::STRING,
                        },
                    }
                };
            }
        }
    }

    xr::Result::SUCCESS
}

unsafe extern "system" fn destroy_snapshot(snapshot: xr::SpatialSnapshotEXT) -> xr::Result {
    if state().snapshots.remove(&snapshot.into_raw()).is_some() {
        xr::Result::SUCCESS
    } else {
        xr::Result::ERROR_HANDLE_INVALID
    }
}

unsafe extern "system" fn create_entity_from_id(
    context: xr::SpatialContextEXT,
    info: *const xr::SpatialEntityFromIdCreateInfoEXT,
    entity: *mut xr::SpatialEntityEXT,
) -> xr::Result {
    if info.is_null()
        || entity.is_null()
        || unsafe { (*info).ty } != xr::SpatialEntityFromIdCreateInfoEXT::TYPE
    {
        return xr::Result::ERROR_VALIDATION_FAILURE;
    }
    let id = unsafe { (*info).entity_id }.into_raw();

    let mut state = state();
    let Some(context_data) = state.contexts.get(&context.into_raw()) else {
        return xr::Result::ERROR_HANDLE_INVALID;
    };
    if !context_data.discovered.contains(&id) {
        return xr::Result::ERROR_SPATIAL_ENTITY_ID_INVALID_EXT;
    }

    let handle = state.new_handle();
    state.entities.insert(
        handle,
        EntityData {
            context: context.into_raw(),
            id,
        },
    );
    unsafe { *entity = xr::SpatialEntityEXT::from_raw(handle) };

    xr::Result::SUCCESS
}

unsafe extern "system" fn destroy_entity(entity: xr::SpatialEntityEXT) -> xr::Result {
    if state().entities.remove(&entity.into_raw()).is_some() {
        xr::Result::SUCCESS
    } else {
        xr::Result::ERROR_HANDLE_INVALID
    }
}

/// The text of the QR code whose buffer `info` names.
fn buffer_text(
    state: &State,
    snapshot: xr::SpatialSnapshotEXT,
    info: *const xr::SpatialBufferGetInfoEXT,
) -> Result<String, xr::Result> {
    if info.is_null() || unsafe { (*info).ty } != xr::SpatialBufferGetInfoEXT::TYPE {
        return Err(xr::Result::ERROR_VALIDATION_FAILURE);
    }
    let buffer = unsafe { (*info).buffer_id }.into_raw();

    let snapshot = state
        .snapshots
        .get(&snapshot.into_raw())
        .ok_or(xr::Result::ERROR_HANDLE_INVALID)?;
    snapshot
        .entities
        .iter()
        .find(|entity| {
            entity.id == buffer && entity.state == xr::SpatialEntityTrackingStateEXT::TRACKING
        })
        .map(|entity| entity.text.clone())
        .ok_or(xr::Result::ERROR_SPATIAL_BUFFER_ID_INVALID_EXT)
}

unsafe extern "system" fn get_buffer_string(
    snapshot: xr::SpatialSnapshotEXT,
    info: *const xr::SpatialBufferGetInfoEXT,
    capacity: u32,
    count_output: *mut u32,
    buffer: *mut c_char,
) -> xr::Result {
    let text = match buffer_text(&state(), snapshot, info) {
        Ok(text) => text,
        Err(result) => return result,
    };

    // The count includes the terminating null
    let bytes = text
        .bytes()
        .chain([0])
        .map(|byte| byte as c_char)
        .collect::<Vec<_>>();
    unsafe { write_array(&bytes, capacity, count_output, buffer) }
}

/// The buffers of QR codes are strings, so getting one as anything else is an error.
macro_rules! typed_buffer_getter {
    ($name:ident, $ty:ty) => {
        unsafe extern "system" fn $name(
            snapshot: xr::SpatialSnapshotEXT,
            info: *const xr::SpatialBufferGetInfoEXT,
            _capacity: u32,
            _count_output: *mut u32,
            _buffer: *mut $ty,
        ) -> xr::Result {
            match buffer_text(&state(), snapshot, info) {
                Ok(_) => xr::Result::ERROR_VALIDATION_FAILURE,
                Err(result) => result,
            }
        }
    };
}
typed_buffer_getter!(get_buffer_uint8, u8);
typed_buffer_getter!(get_buffer_uint16, u16);
typed_buffer_getter!(get_buffer_uint32, u32);
typed_buffer_getter!(get_buffer_float, f32);
typed_buffer_getter!(get_buffer_vector2f, xr::Vector2f);
typed_buffer_getter!(get_buffer_vector3f, xr::Vector3f);
