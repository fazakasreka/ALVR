//! XR_EXT_spatial_entity with the QR code capability of XR_EXT_spatial_marker_tracking, and the
//! parts of XR_EXT_future they use.
//!
//! Every QR code the headset sees is an entity with the marker and 2D bounds components. A context
//! gives a code an entity id the first time it discovers it, and keeps it for the code's data, so
//! it stays the same across the context's snapshots; as the extension requires, no two contexts
//! share an id. Buffer ids come from the code's data, so they are the same in every snapshot.
//! Codes the headset has not decoded yet are not reported.
//! Asynchronous operations complete immediately: their futures are ready as soon as they exist.
//!
//! Each spatial context gets its codes from the headset tracker with its marker options, as far as
//! the headset supports them. Both options are honoured either way: a context for static markers
//! gets their poses averaged, and one with a marker size gets that size.

use crate::{
    State,
    history::{At, Located, MarkerHistory},
    is_layer_handle, log, math,
    session::locate,
    state, void_fn,
};
use alvr_marker_ipc::{
    Capabilities, Payload, Pose, TrackingConfig, now_ns, performance_counter_to_ns,
};
use openxr_sys::{self as xr, Handle, pfn};
use std::{
    collections::HashMap,
    ffi::{CStr, c_char, c_void},
    ptr, slice,
};

const QR_COMPONENTS: [xr::SpatialComponentTypeEXT; 2] = [
    xr::SpatialComponentTypeEXT::MARKER,
    xr::SpatialComponentTypeEXT::BOUNDED_2D,
];
/// The component types defined by the extensions the layer implements. Others are invalid values.
const KNOWN_COMPONENTS: [xr::SpatialComponentTypeEXT; 5] = [
    xr::SpatialComponentTypeEXT::BOUNDED_2D,
    xr::SpatialComponentTypeEXT::BOUNDED_3D,
    xr::SpatialComponentTypeEXT::PARENT,
    xr::SpatialComponentTypeEXT::MESH_3D,
    xr::SpatialComponentTypeEXT::MARKER,
];
const QR_FEATURES: [xr::SpatialCapabilityFeatureEXT; 2] = [
    xr::SpatialCapabilityFeatureEXT::MARKER_TRACKING_FIXED_SIZE_MARKERS,
    xr::SpatialCapabilityFeatureEXT::MARKER_TRACKING_STATIC_MARKERS,
];

pub(crate) struct ContextData {
    session: u64,
    components: Vec<xr::SpatialComponentTypeEXT>,
    /// The marker options the application asked for.
    config: TrackingConfig,
    /// The entity ids of the codes this context discovered, by marker id: the ones entities may be
    /// created from.
    entity_ids: HashMap<u64, u64>,
    /// The marker ids the last discovery recommendation was sent for.
    announced: Option<Vec<u64>>,
}

pub(crate) struct SnapshotData {
    context: u64,
    /// The components whose data the snapshot has, which every entity in it has.
    components: Vec<xr::SpatialComponentTypeEXT>,
    entities: Vec<SnapshotEntity>,
}

struct SnapshotEntity {
    id: u64,
    /// The marker id, also the id of the buffer with the code's data.
    marker: u64,
    state: xr::SpatialEntityTrackingStateEXT,
    bounds: xr::SpatialBounded2DDataEXT,
    payload: Option<Payload>,
}

pub(crate) struct EntityData {
    context: u64,
    marker: u64,
}

pub(crate) struct FutureData {
    pub instance: u64,
    kind: FutureKind,
}

enum FutureKind {
    Context {
        session: u64,
        context: ContextData,
    },
    Discovery {
        context: u64,
        components: Vec<xr::SpatialComponentTypeEXT>,
        tracking_state: Option<xr::SpatialEntityTrackingStateEXT>,
    },
}

pub(crate) fn function(name: &CStr) -> Option<pfn::VoidFunction> {
    Some(match name.to_bytes() {
        b"xrGetSystem" => void_fn!(get_system, pfn::GetSystem),
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
        _ => return None,
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

unsafe fn read_array<'a, T>(items: *const T, count: u32) -> Result<&'a [T], xr::Result> {
    if count == 0 {
        Ok(&[])
    } else if items.is_null() {
        Err(xr::Result::ERROR_VALIDATION_FAILURE)
    } else {
        Ok(unsafe { slice::from_raw_parts(items, count as usize) })
    }
}

/// Fails for component types the extensions do not define.
fn check_known(components: &[xr::SpatialComponentTypeEXT]) -> Result<(), xr::Result> {
    if components
        .iter()
        .all(|component| KNOWN_COMPONENTS.contains(component))
    {
        Ok(())
    } else {
        Err(xr::Result::ERROR_VALIDATION_FAILURE)
    }
}

/// The components whose data a snapshot of the context is to have: the ones of the filter, valid
/// values all enabled in the context, or all of the context's for an empty filter. Every QR code
/// has every component, so none is left out for having none of these.
unsafe fn read_component_filter(
    context: &ContextData,
    items: *const xr::SpatialComponentTypeEXT,
    count: u32,
) -> Result<Vec<xr::SpatialComponentTypeEXT>, xr::Result> {
    let components = unsafe { read_array(items, count) }?;
    check_known(components)?;
    if !components
        .iter()
        .all(|component| context.components.contains(component))
    {
        return Err(xr::Result::ERROR_SPATIAL_COMPONENT_NOT_ENABLED_EXT);
    }

    Ok(if components.is_empty() {
        context.components.clone()
    } else {
        components.to_vec()
    })
}

/// The tracking state an `XrSpatialFilterTrackingStateEXT` in the chain filters by.
unsafe fn tracking_state_filter(
    next: *const c_void,
) -> Result<Option<xr::SpatialEntityTrackingStateEXT>, xr::Result> {
    let mut filter = None;
    for (ty, structure) in unsafe { chain(next) } {
        if ty == xr::SpatialFilterTrackingStateEXT::TYPE {
            let state =
                unsafe { (*structure.cast::<xr::SpatialFilterTrackingStateEXT>()).tracking_state };
            if ![
                xr::SpatialEntityTrackingStateEXT::STOPPED,
                xr::SpatialEntityTrackingStateEXT::TRACKING,
                xr::SpatialEntityTrackingStateEXT::PAUSED,
            ]
            .contains(&state)
            {
                return Err(xr::Result::ERROR_VALIDATION_FAILURE);
            }
            filter = Some(state);
        }
    }

    Ok(filter)
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

impl ContextData {
    /// The tracker the headset runs for the context's options, and the codes it sees: their
    /// marker ids and data.
    fn seen(
        &self,
        history: &MarkerHistory,
        capabilities: &Capabilities,
        now: i64,
    ) -> (TrackingConfig, Vec<(u64, Payload)>) {
        let tracker = self.config.supported_by(capabilities);
        (tracker, history.seen(tracker, now))
    }

    /// A code the context discovered, in a snapshot whose base space has STAGE at `stage`.
    fn snapshot_entity(
        &self,
        marker: u64,
        payload: Payload,
        located: Option<Located>,
        stage: Option<Pose>,
    ) -> SnapshotEntity {
        let id = self.entity_ids[&marker];
        let (Some(located), Some(stage)) = (located, stage) else {
            return SnapshotEntity::paused(id, marker);
        };

        // As the extension requires: every code has the size the application gave
        let size = self.config.marker_size.unwrap_or(located.size);

        SnapshotEntity {
            id,
            marker,
            state: xr::SpatialEntityTrackingStateEXT::TRACKING,
            bounds: xr::SpatialBounded2DDataEXT {
                center: math::to_xr(math::compose(stage, located.pose)),
                extents: xr::Extent2Df {
                    width: size,
                    height: size,
                },
            },
            payload: Some(payload),
        }
    }
}

impl SnapshotEntity {
    /// Out of the headset's sight, or the headset is not connected: it may come back.
    fn paused(id: u64, marker: u64) -> Self {
        Self {
            id,
            marker,
            state: xr::SpatialEntityTrackingStateEXT::PAUSED,
            bounds: xr::SpatialBounded2DDataEXT::default(),
            payload: None,
        }
    }
}

impl FutureKind {
    /// The session a context is being created for.
    fn session(&self) -> Option<u64> {
        match self {
            Self::Context { session, .. } => Some(*session),
            Self::Discovery { .. } => None,
        }
    }

    /// The context a discovery snapshot is being created for.
    fn context(&self) -> Option<u64> {
        match self {
            Self::Context { .. } => None,
            Self::Discovery { context, .. } => Some(*context),
        }
    }
}

impl State {
    /// Whether the application enabled QR codes on the instance and the headset tracks them, as
    /// far as the driver knows.
    fn qr_codes_available(&mut self, instance: u64) -> bool {
        self.instances
            .get(&instance)
            .is_some_and(|data| data.qr_codes)
            && self
                .refresh_markers()
                .is_some_and(|capabilities| capabilities.qr_codes)
    }

    /// Tells the driver the marker options of the application's spatial contexts, also the ones
    /// being created.
    pub(crate) fn sync_requests(&mut self) {
        let configs = self
            .contexts
            .values()
            .map(|context| context.config)
            .chain(
                self.futures
                    .values()
                    .filter_map(|future| match &future.kind {
                        FutureKind::Context { context, .. } => Some(context.config),
                        FutureKind::Discovery { .. } => None,
                    }),
            )
            .collect::<Vec<_>>();
        self.markers.client.set_requests(&configs);
        self.refresh_markers();
    }

    /// The codes the context's tracker sees: their marker ids and data, and where they are at
    /// `time`.
    fn context_markers(
        &mut self,
        context: u64,
        time: xr::Time,
    ) -> Vec<(u64, Payload, Option<Located>)> {
        let Some(capabilities) = self.refresh_markers() else {
            return Vec::new();
        };
        let context = &self.contexts[&context];
        let instance = self.sessions[&context.session].instance;

        let at = if context.config.static_markers {
            At::Static
        } else {
            self.performance_counter_time(instance, time)
                .map_or(At::Latest, At::Time)
        };
        let now = now_ns();
        let history = &self.markers.history;
        let (tracker, seen) = context.seen(history, &capabilities, now);
        seen.into_iter()
            .map(|(id, payload)| {
                let located = history.locate(tracker, id, at, now);
                (id, payload, located)
            })
            .collect()
    }

    /// `time` on the clock of the marker times.
    fn performance_counter_time(&self, instance: u64, time: xr::Time) -> Option<i64> {
        let convert = self.downstream(instance)?.convert_time?;
        let mut counter = 0;
        let result = unsafe { convert(xr::Instance::from_raw(instance), time, &mut counter) };

        (result == xr::Result::SUCCESS).then(|| performance_counter_to_ns(counter))
    }

    fn check_instance(&self, instance: xr::Instance, system: xr::SystemId) -> xr::Result {
        match self.instances.get(&instance.into_raw()) {
            Some(data) if data.hooks.spatial => {
                if data.systems.contains(&system.into_raw()) {
                    xr::Result::SUCCESS
                } else {
                    xr::Result::ERROR_SYSTEM_INVALID
                }
            }
            _ => xr::Result::ERROR_HANDLE_INVALID,
        }
    }

    /// Removes the contexts of a session, and the ones being created for it.
    pub(crate) fn remove_session_contexts(&mut self, session: u64) {
        let contexts = self
            .contexts
            .iter()
            .filter(|(_, context)| context.session == session)
            .map(|(&handle, _)| handle)
            .collect::<Vec<_>>();
        for context in contexts {
            self.remove_context(context);
        }
        self.futures
            .retain(|_, future| future.kind.session() != Some(session));
    }

    fn remove_context(&mut self, context: u64) -> bool {
        if self.contexts.remove(&context).is_none() {
            return false;
        }

        self.snapshots
            .retain(|_, snapshot| snapshot.context != context);
        self.entities.retain(|_, entity| entity.context != context);
        self.futures
            .retain(|_, future| future.kind.context() != Some(context));

        true
    }

    /// The layer's answer for a future of its own: whether it is pending for `instance`. `None`
    /// for the runtime's futures.
    fn layer_future(&self, instance: u64, future: u64) -> Option<Result<(), xr::Result>> {
        is_layer_handle(future).then(|| match self.futures.get(&future) {
            Some(data) if data.instance == instance => Ok(()),
            // Completed, cancelled, or of another instance
            _ => Err(xr::Result::ERROR_FUTURE_INVALID_EXT),
        })
    }

    /// Checks that `future` is pending, of the kind `started_on` gives the handle the operation
    /// was started on for, and that the operation is completed with that handle, `owner`.
    fn check_future(
        &self,
        future: u64,
        owner: u64,
        started_on: fn(&FutureKind) -> Option<u64>,
    ) -> Result<(), xr::Result> {
        match self
            .futures
            .get(&future)
            .and_then(|data| started_on(&data.kind))
        {
            Some(started_on) if started_on == owner => Ok(()),
            Some(_) => Err(xr::Result::ERROR_HANDLE_INVALID),
            None => Err(xr::Result::ERROR_FUTURE_INVALID_EXT),
        }
    }

    /// Removes the future of a context being created for `session`.
    fn take_context_future(
        &mut self,
        future: u64,
        session: u64,
    ) -> Result<ContextData, xr::Result> {
        self.check_future(future, session, FutureKind::session)?;
        match self.futures.remove(&future).map(|data| data.kind) {
            Some(FutureKind::Context { context, .. }) => Ok(context),
            _ => unreachable!(),
        }
    }

    /// Removes the future of a discovery that `check_future` accepted: the components and the
    /// tracking state the snapshot is filtered by.
    fn take_discovery_future(
        &mut self,
        future: u64,
    ) -> (
        Vec<xr::SpatialComponentTypeEXT>,
        Option<xr::SpatialEntityTrackingStateEXT>,
    ) {
        match self.futures.remove(&future).map(|data| data.kind) {
            Some(FutureKind::Discovery {
                components,
                tracking_state,
                ..
            }) => (components, tracking_state),
            _ => unreachable!(),
        }
    }

    /// The pose of the runtime's STAGE space, which marker poses are in, in `base_space`.
    /// `Ok(None)` when it cannot be located at `time`.
    fn locate_stage(
        &mut self,
        session: u64,
        base_space: xr::Space,
        time: xr::Time,
    ) -> Result<Option<Pose>, xr::Result> {
        let instance = self.sessions[&session].instance;
        let downstream = self
            .downstream(instance)
            .ok_or(xr::Result::ERROR_HANDLE_INVALID)?;

        let Some(stage) = self.reference_space(session, xr::ReferenceSpaceType::STAGE) else {
            return Ok(None);
        };

        Ok(unsafe { locate(&downstream, stage, base_space, time) }?.map(math::from_xr))
    }

    /// A context of `instance` to recommend a discovery for, when the codes its tracker sees have
    /// changed since its last recommendation.
    pub(crate) fn take_discovery_recommendation(&mut self, instance: u64) -> Option<u64> {
        let capabilities = self.refresh_markers().unwrap_or_default();
        let now = now_ns();

        for (&handle, context) in &mut self.contexts {
            if self
                .sessions
                .get(&context.session)
                .is_none_or(|session| session.instance != instance)
            {
                continue;
            }

            let (_, seen) = context.seen(&self.markers.history, &capabilities, now);
            let mut ids = seen.into_iter().map(|(id, _)| id).collect::<Vec<_>>();
            ids.sort_unstable();

            if context.announced.as_ref() != Some(&ids) {
                context.announced = Some(ids);
                return Some(handle);
            }
        }

        None
    }
}

/// Remembers the systems of the instance, which the spatial functions are called with.
unsafe extern "system" fn get_system(
    instance: xr::Instance,
    info: *const xr::SystemGetInfo,
    system: *mut xr::SystemId,
) -> xr::Result {
    let Some(downstream) = state().downstream(instance.into_raw()) else {
        return xr::Result::ERROR_HANDLE_INVALID;
    };

    let result = unsafe { (downstream.get_system)(instance, info, system) };
    if result == xr::Result::SUCCESS
        && let Some(data) = state().instances.get_mut(&instance.into_raw())
    {
        data.systems.insert(unsafe { *system }.into_raw());
    }

    result
}

unsafe extern "system" fn poll_future(
    instance: xr::Instance,
    info: *const xr::FuturePollInfoEXT,
    result: *mut xr::FuturePollResultEXT,
) -> xr::Result {
    if info.is_null()
        || result.is_null()
        || unsafe { (*info).ty } != xr::FuturePollInfoEXT::TYPE
        || unsafe { (*result).ty } != xr::FuturePollResultEXT::TYPE
    {
        return xr::Result::ERROR_VALIDATION_FAILURE;
    }
    let future = unsafe { (*info).future }.into_raw();

    let state = state();
    let Some(downstream) = state.downstream(instance.into_raw()) else {
        return xr::Result::ERROR_HANDLE_INVALID;
    };
    match state.layer_future(instance.into_raw(), future) {
        Some(Ok(())) => {
            unsafe { (*result).state = xr::FutureStateEXT::READY };
            xr::Result::SUCCESS
        }
        Some(Err(error)) => error,
        None => {
            drop(state);
            match downstream.poll_future {
                Some(poll_future) => unsafe { poll_future(instance, info, result) },
                None => xr::Result::ERROR_FUTURE_INVALID_EXT,
            }
        }
    }
}

unsafe extern "system" fn cancel_future(
    instance: xr::Instance,
    info: *const xr::FutureCancelInfoEXT,
) -> xr::Result {
    if info.is_null() || unsafe { (*info).ty } != xr::FutureCancelInfoEXT::TYPE {
        return xr::Result::ERROR_VALIDATION_FAILURE;
    }
    let future = unsafe { (*info).future }.into_raw();

    let mut state = state();
    let Some(downstream) = state.downstream(instance.into_raw()) else {
        return xr::Result::ERROR_HANDLE_INVALID;
    };
    match state.layer_future(instance.into_raw(), future) {
        Some(Ok(())) => {
            state.futures.remove(&future);
            state.sync_requests();
            xr::Result::SUCCESS
        }
        Some(Err(error)) => error,
        None => {
            drop(state);
            match downstream.cancel_future {
                Some(cancel_future) => unsafe { cancel_future(instance, info) },
                None => xr::Result::ERROR_FUTURE_INVALID_EXT,
            }
        }
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

    let available: &[_] = if state.qr_codes_available(instance.into_raw()) {
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
        || !state.qr_codes_available(instance.into_raw())
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
        || !state.qr_codes_available(instance.into_raw())
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
    let configs = unsafe { read_array(info.capability_configs, info.capability_config_count) }?;
    // A context needs at least one capability
    if configs.is_empty() {
        return Err(xr::Result::ERROR_SPATIAL_CAPABILITY_CONFIGURATION_INVALID_EXT);
    }

    let mut context = ContextData {
        session,
        components: Vec::new(),
        config: TrackingConfig::default(),
        entity_ids: HashMap::new(),
        announced: None,
    };
    let mut capabilities = Vec::new();
    for &config in configs {
        if config.is_null() {
            return Err(xr::Result::ERROR_VALIDATION_FAILURE);
        }
        let config = unsafe { &*config };

        if capabilities.contains(&config.capability) {
            return Err(xr::Result::ERROR_SPATIAL_CAPABILITY_CONFIGURATION_INVALID_EXT);
        }
        capabilities.push(config.capability);

        if config.capability != xr::SpatialCapabilityEXT::MARKER_TRACKING_QR_CODE {
            return Err(xr::Result::ERROR_SPATIAL_CAPABILITY_UNSUPPORTED_EXT);
        }
        if config.ty != xr::SpatialCapabilityConfigurationQrCodeEXT::TYPE {
            return Err(xr::Result::ERROR_SPATIAL_CAPABILITY_CONFIGURATION_INVALID_EXT);
        }

        let components =
            unsafe { read_array(config.enabled_components, config.enabled_component_count) }?;
        // A capability configuration needs at least one component
        if components.is_empty() {
            return Err(xr::Result::ERROR_SPATIAL_CAPABILITY_CONFIGURATION_INVALID_EXT);
        }
        for component in components {
            if !KNOWN_COMPONENTS.contains(component) {
                return Err(xr::Result::ERROR_VALIDATION_FAILURE);
            }
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
                context.config.marker_size = Some(size);
            } else if ty == xr::SpatialMarkerStaticOptimizationEXT::TYPE {
                context.config.static_markers = unsafe {
                    (*structure.cast::<xr::SpatialMarkerStaticOptimizationEXT>())
                        .optimize_for_static_marker
                } == xr::TRUE;
            }
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
    let context = match unsafe { context_config(session.into_raw(), &*info) } {
        Ok(context) => context,
        Err(result) => return result,
    };
    if !state.qr_codes_available(instance) {
        return xr::Result::ERROR_SPATIAL_CAPABILITY_UNSUPPORTED_EXT;
    }

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
    // The headset starts the tracker while the application waits for the context
    state.sync_requests();

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
    let context = match state.take_context_future(future.into_raw(), session.into_raw()) {
        Ok(context) => context,
        Err(result) => return result,
    };

    let handle = state.new_handle();
    log!(
        "Created spatial context {handle:#x} for QR codes, {:?}",
        context.config
    );
    state.contexts.insert(handle, context);

    unsafe {
        (*completion).future_result = xr::Result::SUCCESS;
        (*completion).spatial_context = xr::SpatialContextEXT::from_raw(handle);
    }

    xr::Result::SUCCESS
}

unsafe extern "system" fn destroy_context(context: xr::SpatialContextEXT) -> xr::Result {
    let mut state = state();
    if state.remove_context(context.into_raw()) {
        state.sync_requests();
        xr::Result::SUCCESS
    } else {
        xr::Result::ERROR_HANDLE_INVALID
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
    let info = unsafe { &*info };
    let components = match unsafe {
        read_component_filter(
            context_data,
            info.component_types,
            info.component_type_count,
        )
    } {
        Ok(components) => components,
        Err(result) => return result,
    };
    let tracking_state = match unsafe { tracking_state_filter(info.next) } {
        Ok(filter) => filter,
        Err(result) => return result,
    };
    let instance = state.sessions[&context_data.session].instance;

    let handle = state.new_handle();
    state.futures.insert(
        handle,
        FutureData {
            instance,
            kind: FutureKind::Discovery {
                context: context.into_raw(),
                components,
                tracking_state,
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
    let future = info.future.into_raw();
    if let Err(result) = state.check_future(future, context.into_raw(), FutureKind::context) {
        return result;
    }

    let stage = match state.locate_stage(session, info.base_space, info.time) {
        Ok(stage) => stage,
        Err(result) => return result,
    };
    let (components, tracking_state) = state.take_discovery_future(future);

    let markers = state.context_markers(context.into_raw(), info.time);
    // Codes discovered for the first time get their entity ids
    for (marker, ..) in &markers {
        if !state.contexts[&context.into_raw()]
            .entity_ids
            .contains_key(marker)
        {
            let id = state.new_handle();
            state
                .contexts
                .get_mut(&context.into_raw())
                .unwrap()
                .entity_ids
                .insert(*marker, id);
        }
    }
    let context_data = &state.contexts[&context.into_raw()];
    let entities = markers
        .into_iter()
        .map(|(marker, payload, located)| {
            context_data.snapshot_entity(marker, payload, located, stage)
        })
        .filter(|entity| tracking_state.is_none_or(|state| entity.state == state))
        .collect();

    let handle = state.new_handle();
    state.snapshots.insert(
        handle,
        SnapshotData {
            context: context.into_raw(),
            components,
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
    let components = match unsafe {
        read_component_filter(
            context_data,
            info.component_types,
            info.component_type_count,
        )
    } {
        Ok(components) => components,
        Err(result) => return result,
    };
    let session = context_data.session;

    let entity_handles = match unsafe { read_array(info.entities, info.entity_count) } {
        Ok([]) | Err(_) => return xr::Result::ERROR_VALIDATION_FAILURE,
        Ok(handles) => handles,
    };
    let mut markers = Vec::with_capacity(entity_handles.len());
    for entity in entity_handles {
        match state.entities.get(&entity.into_raw()) {
            Some(data) if data.context == context.into_raw() => markers.push(data.marker),
            _ => return xr::Result::ERROR_HANDLE_INVALID,
        }
    }

    let stage = match state.locate_stage(session, info.base_space, info.time) {
        Ok(stage) => stage,
        Err(result) => return result,
    };

    let seen = state
        .context_markers(context.into_raw(), info.time)
        .into_iter()
        .map(|(marker, payload, located)| (marker, (payload, located)))
        .collect::<HashMap<_, _>>();
    let context_data = &state.contexts[&context.into_raw()];
    let entities = markers
        .iter()
        .map(|&marker| match seen.get(&marker) {
            Some((payload, located)) => {
                context_data.snapshot_entity(marker, payload.clone(), *located, stage)
            }
            None => SnapshotEntity::paused(context_data.entity_ids[&marker], marker),
        })
        .collect();

    let handle = state.new_handle();
    state.snapshots.insert(
        handle,
        SnapshotData {
            context: context.into_raw(),
            components,
            entities,
        },
    );
    unsafe { *snapshot = xr::SpatialSnapshotEXT::from_raw(handle) };

    xr::Result::SUCCESS
}

/// A component list in the chain of a query result.
struct ComponentList {
    component: xr::SpatialComponentTypeEXT,
    count: u32,
    elements: Elements,
}

#[derive(Clone, Copy)]
enum Elements {
    Bounded2D(*mut xr::SpatialBounded2DDataEXT),
    Marker(*mut xr::SpatialMarkerDataEXT),
    /// Of a component no QR code has. Such lists are only allowed when no entity has the
    /// component, so nothing is written to them.
    Other(*const c_void),
}

impl Elements {
    fn is_null(&self) -> bool {
        match *self {
            Self::Bounded2D(elements) => elements.is_null(),
            Self::Marker(elements) => elements.is_null(),
            Self::Other(elements) => elements.is_null(),
        }
    }
}

/// The component list `structure` is, if it is one.
unsafe fn component_list(ty: xr::StructureType, structure: *const c_void) -> Option<ComponentList> {
    use xr::SpatialComponentTypeEXT as Component;

    let (component, count, elements) = if ty == xr::SpatialComponentBounded2DListEXT::TYPE {
        let list = unsafe { &*structure.cast::<xr::SpatialComponentBounded2DListEXT>() };
        (
            Component::BOUNDED_2D,
            list.bound_count,
            Elements::Bounded2D(list.bounds),
        )
    } else if ty == xr::SpatialComponentMarkerListEXT::TYPE {
        let list = unsafe { &*structure.cast::<xr::SpatialComponentMarkerListEXT>() };
        (
            Component::MARKER,
            list.marker_count,
            Elements::Marker(list.markers),
        )
    } else if ty == xr::SpatialComponentBounded3DListEXT::TYPE {
        let list = unsafe { &*structure.cast::<xr::SpatialComponentBounded3DListEXT>() };
        (
            Component::BOUNDED_3D,
            list.bound_count,
            Elements::Other(list.bounds.cast()),
        )
    } else if ty == xr::SpatialComponentParentListEXT::TYPE {
        let list = unsafe { &*structure.cast::<xr::SpatialComponentParentListEXT>() };
        (
            Component::PARENT,
            list.parent_count,
            Elements::Other(list.parents.cast()),
        )
    } else if ty == xr::SpatialComponentMesh3DListEXT::TYPE {
        let list = unsafe { &*structure.cast::<xr::SpatialComponentMesh3DListEXT>() };
        (
            Component::MESH_3D,
            list.mesh_count,
            Elements::Other(list.meshes.cast()),
        )
    } else {
        return None;
    };

    Some(ComponentList {
        component,
        count,
        elements,
    })
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
    let components =
        match unsafe { read_array(condition.component_types, condition.component_type_count) } {
            Ok(components) => components,
            Err(result) => return result,
        };
    if let Err(result) = check_known(components) {
        return result;
    }
    let tracking_state = match unsafe { tracking_state_filter(condition.next) } {
        Ok(filter) => filter,
        Err(result) => return result,
    };

    // Lists only for the components of the condition. Without any in the condition, for the ones
    // the snapshot has: the entities it returns have those.
    let lists = unsafe { chain(result.next) }
        .into_iter()
        .filter_map(|(ty, structure)| unsafe { component_list(ty, structure) })
        .collect::<Vec<_>>();
    let allowed = if components.is_empty() {
        &snapshot_data.components
    } else {
        components
    };
    if lists.iter().any(|list| !allowed.contains(&list.component)) {
        return xr::Result::ERROR_VALIDATION_FAILURE;
    }

    // Every entity in the snapshot has the snapshot's components, and only those
    let has_components = components
        .iter()
        .all(|component| snapshot_data.components.contains(component));
    let entities = snapshot_data
        .entities
        .iter()
        .filter(|entity| has_components && tracking_state.is_none_or(|state| entity.state == state))
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
    for list in &lists {
        if list.count < count {
            return xr::Result::ERROR_SIZE_INSUFFICIENT;
        }
        if count > 0 && list.elements.is_null() {
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
    for list in lists {
        for (i, entity) in entities.iter().enumerate() {
            // Component data is only valid for tracked entities
            let Some(payload) = entity
                .payload
                .as_ref()
                .filter(|_| entity.state == xr::SpatialEntityTrackingStateEXT::TRACKING)
            else {
                continue;
            };

            match list.elements {
                Elements::Bounded2D(bounds) => unsafe { *bounds.add(i) = entity.bounds },
                Elements::Marker(markers) => unsafe {
                    *markers.add(i) = xr::SpatialMarkerDataEXT {
                        capability: xr::SpatialCapabilityEXT::MARKER_TRACKING_QR_CODE,
                        // Only ArUco markers and AprilTags have a numeric id
                        marker_id: 0,
                        data: xr::SpatialBufferEXT {
                            buffer_id: xr::SpatialBufferIdEXT::from_raw(entity.marker),
                            buffer_type: match payload {
                                Payload::Text(_) => xr::SpatialBufferTypeEXT::STRING,
                                Payload::Bytes(_) => xr::SpatialBufferTypeEXT::UINT8,
                            },
                        },
                    }
                },
                Elements::Other(_) => (),
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
    // Only ids this context discovered
    let Some(marker) = context_data
        .entity_ids
        .iter()
        .find(|&(_, &entity_id)| entity_id == id)
        .map(|(&marker, _)| marker)
    else {
        return xr::Result::ERROR_SPATIAL_ENTITY_ID_INVALID_EXT;
    };

    let handle = state.new_handle();
    state.entities.insert(
        handle,
        EntityData {
            context: context.into_raw(),
            marker,
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

/// The data of the QR code whose buffer `info` names.
fn buffer_payload(
    state: &State,
    snapshot: xr::SpatialSnapshotEXT,
    info: *const xr::SpatialBufferGetInfoEXT,
) -> Result<Payload, xr::Result> {
    if info.is_null() || unsafe { (*info).ty } != xr::SpatialBufferGetInfoEXT::TYPE {
        return Err(xr::Result::ERROR_VALIDATION_FAILURE);
    }
    let buffer = unsafe { (*info).buffer_id }.into_raw();

    let snapshot = state
        .snapshots
        .get(&snapshot.into_raw())
        .ok_or(xr::Result::ERROR_HANDLE_INVALID)?;
    // Only snapshots with the marker component have the codes' data
    if !snapshot
        .components
        .contains(&xr::SpatialComponentTypeEXT::MARKER)
    {
        return Err(xr::Result::ERROR_SPATIAL_BUFFER_ID_INVALID_EXT);
    }
    snapshot
        .entities
        .iter()
        .find(|entity| {
            entity.marker == buffer && entity.state == xr::SpatialEntityTrackingStateEXT::TRACKING
        })
        .and_then(|entity| entity.payload.clone())
        .ok_or(xr::Result::ERROR_SPATIAL_BUFFER_ID_INVALID_EXT)
}

unsafe extern "system" fn get_buffer_string(
    snapshot: xr::SpatialSnapshotEXT,
    info: *const xr::SpatialBufferGetInfoEXT,
    capacity: u32,
    count_output: *mut u32,
    buffer: *mut c_char,
) -> xr::Result {
    let text = match buffer_payload(&state(), snapshot, info) {
        Ok(Payload::Text(text)) => text,
        Ok(Payload::Bytes(_)) => return xr::Result::ERROR_VALIDATION_FAILURE,
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

unsafe extern "system" fn get_buffer_uint8(
    snapshot: xr::SpatialSnapshotEXT,
    info: *const xr::SpatialBufferGetInfoEXT,
    capacity: u32,
    count_output: *mut u32,
    buffer: *mut u8,
) -> xr::Result {
    let bytes = match buffer_payload(&state(), snapshot, info) {
        Ok(Payload::Bytes(bytes)) => bytes,
        Ok(Payload::Text(_)) => return xr::Result::ERROR_VALIDATION_FAILURE,
        Err(result) => return result,
    };

    unsafe { write_array(&bytes, capacity, count_output, buffer) }
}

/// The buffers of QR codes are strings or bytes, so getting one as anything else is an error.
macro_rules! typed_buffer_getter {
    ($name:ident, $ty:ty) => {
        unsafe extern "system" fn $name(
            snapshot: xr::SpatialSnapshotEXT,
            info: *const xr::SpatialBufferGetInfoEXT,
            _capacity: u32,
            _count_output: *mut u32,
            _buffer: *mut $ty,
        ) -> xr::Result {
            match buffer_payload(&state(), snapshot, info) {
                Ok(_) => xr::Result::ERROR_VALIDATION_FAILURE,
                Err(result) => result,
            }
        }
    };
}
typed_buffer_getter!(get_buffer_uint16, u16);
typed_buffer_getter!(get_buffer_uint32, u32);
typed_buffer_getter!(get_buffer_float, f32);
typed_buffer_getter!(get_buffer_vector2f, xr::Vector2f);
typed_buffer_getter!(get_buffer_vector3f, xr::Vector3f);
