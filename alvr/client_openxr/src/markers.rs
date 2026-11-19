//! QR codes seen by the headset, through XR_EXT_spatial_marker_tracking.
//!
//! The server says which trackers to run: one spatial context per configuration, each with the
//! marker options OpenXR applications on the PC asked for. A tracker finds codes with a discovery
//! every [`DISCOVERY_INTERVAL`], and follows the codes it found with an update snapshot every
//! frame, so that moving codes are reported at the frame rate, for the frame's display time.

use alvr_common::{error, info, warn};
use alvr_packets::{MarkerCapabilities, MarkerData, MarkerPayload, MarkerTrackingConfig};
use openxr::{self as xr, sys, sys::Handle};
use std::{
    collections::HashMap,
    ffi::CStr,
    mem, ptr,
    time::{Duration, Instant},
};

const DISCOVERY_INTERVAL: Duration = Duration::from_millis(500);
/// Tracking of a static code drops out every so often while it is in view, so one is still
/// reported for this long after it was last tracked.
const STATIC_MARKER_TIMEOUT: Duration = Duration::from_secs(1);
/// A code not tracked for this long is forgotten until a discovery finds it again.
const ENTITY_TIMEOUT: Duration = Duration::from_secs(30);
const COMPONENTS: [sys::SpatialComponentTypeEXT; 2] = [
    sys::SpatialComponentTypeEXT::MARKER,
    sys::SpatialComponentTypeEXT::BOUNDED_2D,
];
const POSE_COMPONENT: [sys::SpatialComponentTypeEXT; 1] =
    [sys::SpatialComponentTypeEXT::BOUNDED_2D];
// Each vendor names its own spatial data permission; a headset ignores the others
#[cfg(target_os = "android")]
const SPATIAL_DATA_PERMISSIONS: [&str; 3] = [
    "com.oculus.permission.USE_SCENE",
    "android.permission.SCENE_UNDERSTANDING_COARSE",
    "com.picovr.permission.SPATIAL_DATA",
];

const PERMISSION_CHECK_INTERVAL: Duration = Duration::from_secs(1);
// Android drops a permission request while another dialog (e.g. the microphone one) is open, so
// the request is repeated until answered
const PERMISSION_REQUEST_INTERVAL: Duration = Duration::from_secs(5);
const PERMISSION_REQUEST_COUNT: u32 = 12;

struct PermissionWait {
    last_check: Instant,
    last_request: Instant,
    requests: u32,
}

enum ContextState {
    NotStarted,
    Creating(sys::FutureEXT),
    Ready(sys::SpatialContextEXT),
    Failed,
}

/// A code a tracker found.
struct Entity {
    handle: sys::SpatialEntityEXT,
    payload: MarkerPayload,
    /// The last tracked pose, and when it was tracked.
    last_tracked: Option<(MarkerData, Instant)>,
    found_at: Instant,
}

/// One spatial context.
struct Tracker {
    config: MarkerTrackingConfig,
    context: ContextState,
    discovery: Option<sys::FutureEXT>,
    last_discovery: Option<Instant>,
    entities: HashMap<sys::SpatialEntityIdEXT, Entity>,
    /// Whether the last report had codes, so that an empty one follows when they are all gone.
    reported_markers: bool,
}

pub struct MarkerTracker {
    instance: xr::Instance,
    session: xr::Session<xr::OpenGlEs>,
    entity_fns: xr::raw::SpatialEntityEXT,
    future_fns: xr::raw::FutureEXT,
    // Stage space, which the stream's tracking is reported in
    space: xr::Space,
    capabilities: MarkerCapabilities,
    permission: Option<PermissionWait>,
    trackers: Vec<Tracker>,
}

impl MarkerTracker {
    pub fn new(session: &xr::Session<xr::OpenGlEs>, system: xr::SystemId) -> Option<Self> {
        let instance = session.instance().clone();
        let exts = instance.exts();
        let (Some(entity_fns), Some(future_fns), Some(_)) = (
            exts.ext_spatial_entity,
            exts.ext_future,
            exts.ext_spatial_marker_tracking,
        ) else {
            info!("QR code tracking is not available");
            return None;
        };

        let capabilities = unsafe {
            two_call(|capacity, count, items| {
                (entity_fns.enumerate_spatial_capabilities)(
                    instance.as_raw(),
                    system,
                    capacity,
                    count,
                    items,
                )
            })
        };
        info!("Spatial capabilities: {capabilities:?}");
        if !capabilities.contains(&sys::SpatialCapabilityEXT::MARKER_TRACKING_QR_CODE) {
            info!("QR code tracking is not supported");
            return None;
        }

        let features = unsafe {
            two_call(|capacity, count, items| {
                (entity_fns.enumerate_spatial_capability_features)(
                    instance.as_raw(),
                    system,
                    sys::SpatialCapabilityEXT::MARKER_TRACKING_QR_CODE,
                    capacity,
                    count,
                    items,
                )
            })
        };
        info!("QR code tracking features: {features:?}");

        let space = session
            .create_reference_space(xr::ReferenceSpaceType::STAGE, xr::Posef::IDENTITY)
            .ok()?;

        let permission = (!has_spatial_data_permission()).then(|| {
            request_spatial_data_permission();
            info!("Waiting for the spatial data permission");
            PermissionWait {
                last_check: Instant::now(),
                last_request: Instant::now(),
                requests: 1,
            }
        });

        Some(Self {
            instance,
            session: session.clone(),
            entity_fns,
            future_fns,
            space,
            capabilities: MarkerCapabilities {
                qr_codes: true,
                fixed_size_markers: features.contains(
                    &sys::SpatialCapabilityFeatureEXT::MARKER_TRACKING_FIXED_SIZE_MARKERS,
                ),
                static_markers: features
                    .contains(&sys::SpatialCapabilityFeatureEXT::MARKER_TRACKING_STATIC_MARKERS),
            },
            permission,
            trackers: Vec::new(),
        })
    }

    pub fn capabilities(&self) -> MarkerCapabilities {
        self.capabilities
    }

    /// Runs one tracker per configuration, stopping the others.
    pub fn set_configs(&mut self, configs: &[MarkerTrackingConfig]) {
        let (kept, stopped) = mem::take(&mut self.trackers)
            .into_iter()
            .partition::<Vec<_>, _>(|tracker| configs.contains(&tracker.config));
        for tracker in stopped {
            self.destroy(tracker);
        }
        self.trackers = kept;

        for config in configs {
            if !self
                .trackers
                .iter()
                .any(|tracker| tracker.config == *config)
            {
                self.trackers.push(Tracker {
                    config: *config,
                    context: ContextState::NotStarted,
                    discovery: None,
                    last_discovery: None,
                    entities: HashMap::new(),
                    reported_markers: false,
                });
            }
        }
    }

    /// The codes each tracker sees at `time`, the frame's display time. A tracker is left out
    /// while it has seen none since its last report.
    pub fn update(&mut self, time: xr::Time) -> Vec<(MarkerTrackingConfig, Vec<MarkerData>)> {
        if !self.wait_for_permission() {
            return Vec::new();
        }

        let mut trackers = mem::take(&mut self.trackers);
        let reports = trackers
            .iter_mut()
            .filter_map(|tracker| self.update_tracker(tracker, time))
            .collect();
        self.trackers = trackers;

        reports
    }

    /// Whether the spatial data permission is granted.
    fn wait_for_permission(&mut self) -> bool {
        let Some(wait) = &mut self.permission else {
            return true;
        };
        if wait.last_check.elapsed() < PERMISSION_CHECK_INTERVAL {
            return false;
        }
        wait.last_check = Instant::now();

        if has_spatial_data_permission() {
            info!("Spatial data permission granted");
            self.permission = None;
            return true;
        }
        if wait.requests < PERMISSION_REQUEST_COUNT
            && wait.last_request.elapsed() >= PERMISSION_REQUEST_INTERVAL
            && !spatial_data_permission_denied()
        {
            request_spatial_data_permission();
            wait.last_request = Instant::now();
            wait.requests += 1;
        }

        false
    }

    fn update_tracker(
        &self,
        tracker: &mut Tracker,
        time: xr::Time,
    ) -> Option<(MarkerTrackingConfig, Vec<MarkerData>)> {
        let context = match tracker.context {
            ContextState::NotStarted => {
                tracker.context = self.create_context(tracker.config);
                return None;
            }
            ContextState::Creating(future) => {
                if self.is_ready(future) {
                    tracker.context = self.complete_context(future, tracker.config);
                }
                return None;
            }
            ContextState::Ready(context) => context,
            ContextState::Failed => return None,
        };

        match tracker.discovery {
            Some(future) => {
                if self.is_ready(future) {
                    tracker.discovery = None;
                    self.complete_discovery(tracker, context, future, time);
                }
            }
            None => {
                if tracker
                    .last_discovery
                    .is_none_or(|last| last.elapsed() >= DISCOVERY_INTERVAL)
                {
                    tracker.last_discovery = Some(Instant::now());
                    tracker.discovery = self.start_discovery(context);
                }
            }
        }

        let markers = self.locate(tracker, context, time);
        if markers.is_empty() && !tracker.reported_markers {
            return None;
        }
        tracker.reported_markers = !markers.is_empty();

        Some((tracker.config, markers))
    }

    fn create_context(&self, config: MarkerTrackingConfig) -> ContextState {
        let static_markers = sys::SpatialMarkerStaticOptimizationEXT {
            ty: sys::SpatialMarkerStaticOptimizationEXT::TYPE,
            next: ptr::null(),
            optimize_for_static_marker: config.static_markers.into(),
        };
        let marker_size = config.marker_size.map(|size| sys::SpatialMarkerSizeEXT {
            ty: sys::SpatialMarkerSizeEXT::TYPE,
            next: (&raw const static_markers).cast(),
            marker_side_length: size,
        });
        let qr_config = sys::SpatialCapabilityConfigurationQrCodeEXT {
            ty: sys::SpatialCapabilityConfigurationQrCodeEXT::TYPE,
            // Only the options the headset reported: the server sends nothing else
            next: match &marker_size {
                Some(marker_size) => (marker_size as *const sys::SpatialMarkerSizeEXT).cast(),
                None if config.static_markers => (&raw const static_markers).cast(),
                None => ptr::null(),
            },
            capability: sys::SpatialCapabilityEXT::MARKER_TRACKING_QR_CODE,
            enabled_component_count: COMPONENTS.len() as u32,
            enabled_components: COMPONENTS.as_ptr(),
        };
        let configs =
            [(&raw const qr_config).cast::<sys::SpatialCapabilityConfigurationBaseHeaderEXT>()];
        let create_info = sys::SpatialContextCreateInfoEXT {
            ty: sys::SpatialContextCreateInfoEXT::TYPE,
            next: ptr::null(),
            capability_config_count: configs.len() as u32,
            capability_configs: configs.as_ptr(),
        };

        let mut future = sys::FutureEXT::default();
        let result = unsafe {
            (self.entity_fns.create_spatial_context_async)(
                self.session.as_raw(),
                &create_info,
                &mut future,
            )
        };
        if result != sys::Result::SUCCESS {
            error!("Cannot start QR code tracking {config:?}: {result:?}");
            return ContextState::Failed;
        }

        ContextState::Creating(future)
    }

    fn is_ready(&self, future: sys::FutureEXT) -> bool {
        let info = sys::FuturePollInfoEXT {
            ty: sys::FuturePollInfoEXT::TYPE,
            next: ptr::null(),
            future,
        };
        let mut result = sys::FuturePollResultEXT {
            ty: sys::FuturePollResultEXT::TYPE,
            next: ptr::null_mut(),
            state: sys::FutureStateEXT::PENDING,
        };
        let res =
            unsafe { (self.future_fns.poll_future)(self.instance.as_raw(), &info, &mut result) };

        res == sys::Result::SUCCESS && result.state == sys::FutureStateEXT::READY
    }

    fn complete_context(
        &self,
        future: sys::FutureEXT,
        config: MarkerTrackingConfig,
    ) -> ContextState {
        let mut completion = sys::CreateSpatialContextCompletionEXT {
            ty: sys::CreateSpatialContextCompletionEXT::TYPE,
            next: ptr::null_mut(),
            future_result: sys::Result::SUCCESS,
            spatial_context: sys::SpatialContextEXT::NULL,
        };
        let result = unsafe {
            (self.entity_fns.create_spatial_context_complete)(
                self.session.as_raw(),
                future,
                &mut completion,
            )
        };

        if result == sys::Result::SUCCESS && completion.future_result == sys::Result::SUCCESS {
            info!("QR code tracking {config:?} started");
            ContextState::Ready(completion.spatial_context)
        } else {
            error!(
                "Cannot start QR code tracking {config:?}: {result:?}, {:?}",
                completion.future_result
            );
            ContextState::Failed
        }
    }

    fn start_discovery(&self, context: sys::SpatialContextEXT) -> Option<sys::FutureEXT> {
        let create_info = sys::SpatialDiscoverySnapshotCreateInfoEXT {
            ty: sys::SpatialDiscoverySnapshotCreateInfoEXT::TYPE,
            next: ptr::null(),
            component_type_count: COMPONENTS.len() as u32,
            component_types: COMPONENTS.as_ptr(),
        };
        let mut future = sys::FutureEXT::default();
        let result = unsafe {
            (self.entity_fns.create_spatial_discovery_snapshot_async)(
                context,
                &create_info,
                &mut future,
            )
        };

        if result == sys::Result::SUCCESS {
            Some(future)
        } else {
            warn!("QR code discovery failed: {result:?}");
            None
        }
    }

    /// Adds the codes the discovery found to the ones the tracker follows.
    fn complete_discovery(
        &self,
        tracker: &mut Tracker,
        context: sys::SpatialContextEXT,
        future: sys::FutureEXT,
        time: xr::Time,
    ) {
        let info = sys::CreateSpatialDiscoverySnapshotCompletionInfoEXT {
            ty: sys::CreateSpatialDiscoverySnapshotCompletionInfoEXT::TYPE,
            next: ptr::null(),
            base_space: self.space.as_raw(),
            time,
            future,
        };
        let mut completion = sys::CreateSpatialDiscoverySnapshotCompletionEXT {
            ty: sys::CreateSpatialDiscoverySnapshotCompletionEXT::TYPE,
            next: ptr::null_mut(),
            future_result: sys::Result::SUCCESS,
            snapshot: sys::SpatialSnapshotEXT::NULL,
        };
        let result = unsafe {
            (self.entity_fns.create_spatial_discovery_snapshot_complete)(
                context,
                &info,
                &mut completion,
            )
        };
        if result != sys::Result::SUCCESS || completion.future_result != sys::Result::SUCCESS {
            warn!(
                "QR code discovery failed: {result:?}, {:?}",
                completion.future_result
            );
            return;
        }

        let snapshot = completion.snapshot;
        let entities = self.query(snapshot, &COMPONENTS, true);
        for found in entities {
            if found.state != sys::SpatialEntityTrackingStateEXT::TRACKING
                || tracker.entities.contains_key(&found.id)
            {
                continue;
            }
            // A code not decoded yet is left for a later discovery
            let Some(payload) = found
                .marker
                .and_then(|marker| self.read_payload(snapshot, marker.data))
            else {
                continue;
            };

            let create_info = sys::SpatialEntityFromIdCreateInfoEXT {
                ty: sys::SpatialEntityFromIdCreateInfoEXT::TYPE,
                next: ptr::null(),
                entity_id: found.id,
            };
            let mut handle = sys::SpatialEntityEXT::NULL;
            let result = unsafe {
                (self.entity_fns.create_spatial_entity_from_id)(context, &create_info, &mut handle)
            };
            if result != sys::Result::SUCCESS {
                warn!("Cannot follow QR code {payload}: {result:?}");
                continue;
            }

            info!(
                "QR code {payload} at {:?}, size {} ({:?})",
                found.bounds.center.position, found.bounds.extents.width, tracker.config
            );
            tracker.entities.insert(
                found.id,
                Entity {
                    handle,
                    payload,
                    last_tracked: None,
                    found_at: Instant::now(),
                },
            );
        }

        unsafe { (self.entity_fns.destroy_spatial_snapshot)(snapshot) };
    }

    /// Where the codes the tracker follows are at `time`.
    fn locate(
        &self,
        tracker: &mut Tracker,
        context: sys::SpatialContextEXT,
        time: xr::Time,
    ) -> Vec<MarkerData> {
        if tracker.entities.is_empty() {
            return Vec::new();
        }

        let handles = tracker
            .entities
            .values()
            .map(|entity| entity.handle)
            .collect::<Vec<_>>();
        let create_info = sys::SpatialUpdateSnapshotCreateInfoEXT {
            ty: sys::SpatialUpdateSnapshotCreateInfoEXT::TYPE,
            next: ptr::null(),
            entity_count: handles.len() as u32,
            entities: handles.as_ptr(),
            component_type_count: POSE_COMPONENT.len() as u32,
            component_types: POSE_COMPONENT.as_ptr(),
            base_space: self.space.as_raw(),
            time,
        };
        let mut snapshot = sys::SpatialSnapshotEXT::NULL;
        let result = unsafe {
            (self.entity_fns.create_spatial_update_snapshot)(context, &create_info, &mut snapshot)
        };
        if result != sys::Result::SUCCESS {
            warn!("Cannot locate QR codes: {result:?}");
            return Vec::new();
        }
        let located = self.query(snapshot, &POSE_COMPONENT, false);
        unsafe { (self.entity_fns.destroy_spatial_snapshot)(snapshot) };

        let now = Instant::now();
        let mut stopped = Vec::new();
        for found in located {
            let Some(entity) = tracker.entities.get_mut(&found.id) else {
                continue;
            };
            match found.state {
                sys::SpatialEntityTrackingStateEXT::TRACKING => {
                    let marker = MarkerData {
                        payload: entity.payload.clone(),
                        pose: crate::from_xr_pose(found.bounds.center),
                        size: found.bounds.extents.width,
                    };
                    entity.last_tracked = Some((marker, now));
                }
                sys::SpatialEntityTrackingStateEXT::STOPPED => {
                    info!("QR code {} lost", entity.payload);
                    stopped.push(found.id);
                }
                _ => (),
            }
        }

        // Forget codes the headset stopped tracking, or did not see for long, so that the list
        // does not grow forever
        let (forgotten, kept) = mem::take(&mut tracker.entities)
            .into_iter()
            .partition::<HashMap<_, _>, _>(|(id, entity)| {
                let last = entity
                    .last_tracked
                    .as_ref()
                    .map_or(entity.found_at, |t| t.1);
                stopped.contains(id) || now.duration_since(last) >= ENTITY_TIMEOUT
            });
        for entity in forgotten.into_values() {
            unsafe { (self.entity_fns.destroy_spatial_entity)(entity.handle) };
        }
        tracker.entities = kept;

        // Static codes stay put while tracking drops out briefly; moving ones may not
        let timeout = if tracker.config.static_markers {
            STATIC_MARKER_TIMEOUT
        } else {
            Duration::ZERO
        };
        tracker
            .entities
            .values()
            .filter_map(|entity| {
                let (marker, tracked_at) = entity.last_tracked.as_ref()?;
                (*tracked_at == now || now.duration_since(*tracked_at) < timeout)
                    .then(|| marker.clone())
            })
            .collect()
    }

    /// The entities of a snapshot, with their bounds, and their marker data if `with_markers`.
    fn query(
        &self,
        snapshot: sys::SpatialSnapshotEXT,
        components: &[sys::SpatialComponentTypeEXT],
        with_markers: bool,
    ) -> Vec<FoundEntity> {
        let condition = sys::SpatialComponentDataQueryConditionEXT {
            ty: sys::SpatialComponentDataQueryConditionEXT::TYPE,
            next: ptr::null(),
            component_type_count: components.len() as u32,
            component_types: components.as_ptr(),
        };
        let mut query = sys::SpatialComponentDataQueryResultEXT {
            ty: sys::SpatialComponentDataQueryResultEXT::TYPE,
            next: ptr::null_mut(),
            entity_id_capacity_input: 0,
            entity_id_count_output: 0,
            entity_ids: ptr::null_mut(),
            entity_state_capacity_input: 0,
            entity_state_count_output: 0,
            entity_states: ptr::null_mut(),
        };
        let result = unsafe {
            (self.entity_fns.query_spatial_component_data)(snapshot, &condition, &mut query)
        };
        let count = query.entity_id_count_output as usize;
        if result != sys::Result::SUCCESS || count == 0 {
            return Vec::new();
        }

        let mut ids = vec![sys::SpatialEntityIdEXT::default(); count];
        let mut states = vec![sys::SpatialEntityTrackingStateEXT::STOPPED; count];
        let mut marker_data = vec![unsafe { mem::zeroed::<sys::SpatialMarkerDataEXT>() }; count];
        let mut bounds = vec![sys::SpatialBounded2DDataEXT::default(); count];

        let mut marker_list = sys::SpatialComponentMarkerListEXT {
            ty: sys::SpatialComponentMarkerListEXT::TYPE,
            next: ptr::null_mut(),
            marker_count: count as u32,
            markers: marker_data.as_mut_ptr(),
        };
        let mut bounds_list = sys::SpatialComponentBounded2DListEXT {
            ty: sys::SpatialComponentBounded2DListEXT::TYPE,
            next: if with_markers {
                (&raw mut marker_list).cast()
            } else {
                ptr::null_mut()
            },
            bound_count: count as u32,
            bounds: bounds.as_mut_ptr(),
        };
        query.next = (&raw mut bounds_list).cast();
        query.entity_id_capacity_input = count as u32;
        query.entity_ids = ids.as_mut_ptr();
        query.entity_state_capacity_input = count as u32;
        query.entity_states = states.as_mut_ptr();

        let result = unsafe {
            (self.entity_fns.query_spatial_component_data)(snapshot, &condition, &mut query)
        };
        if result != sys::Result::SUCCESS {
            warn!("Cannot read QR codes: {result:?}");
            return Vec::new();
        }

        (0..count)
            .map(|i| FoundEntity {
                id: ids[i],
                state: states[i],
                bounds: bounds[i],
                marker: with_markers.then_some(marker_data[i]),
            })
            .collect()
    }

    /// The data of a QR code, `None` if it is not decoded yet.
    fn read_payload(
        &self,
        snapshot: sys::SpatialSnapshotEXT,
        buffer: sys::SpatialBufferEXT,
    ) -> Option<MarkerPayload> {
        if buffer.buffer_id.into_raw() == 0 {
            return None;
        }
        let info = sys::SpatialBufferGetInfoEXT {
            ty: sys::SpatialBufferGetInfoEXT::TYPE,
            next: ptr::null(),
            buffer_id: buffer.buffer_id,
        };

        match buffer.buffer_type {
            sys::SpatialBufferTypeEXT::STRING => {
                let text = self
                    .read_buffer(self.entity_fns.get_spatial_buffer_string, snapshot, &info)?
                    .into_iter()
                    .map(|c| c as u8)
                    .collect::<Vec<_>>();
                // The count includes the terminating null
                let text = CStr::from_bytes_until_nul(&text)
                    .ok()?
                    .to_string_lossy()
                    .into_owned();
                Some(MarkerPayload::Text(text))
            }
            sys::SpatialBufferTypeEXT::UINT8 => self
                .read_buffer(self.entity_fns.get_spatial_buffer_uint8, snapshot, &info)
                .map(MarkerPayload::Bytes),
            _ => None,
        }
    }

    /// A spatial buffer, read with the two-call idiom.
    fn read_buffer<T: Copy + Default>(
        &self,
        get: unsafe extern "system" fn(
            sys::SpatialSnapshotEXT,
            *const sys::SpatialBufferGetInfoEXT,
            u32,
            *mut u32,
            *mut T,
        ) -> sys::Result,
        snapshot: sys::SpatialSnapshotEXT,
        info: &sys::SpatialBufferGetInfoEXT,
    ) -> Option<Vec<T>> {
        let mut count = 0;
        if unsafe { get(snapshot, info, 0, &mut count, ptr::null_mut()) } != sys::Result::SUCCESS {
            return None;
        }
        let mut buffer = vec![T::default(); count as usize];
        let result = unsafe { get(snapshot, info, count, &mut count, buffer.as_mut_ptr()) };
        buffer.truncate(count as usize);

        (result == sys::Result::SUCCESS).then_some(buffer)
    }

    fn destroy(&self, tracker: Tracker) {
        for entity in tracker.entities.into_values() {
            unsafe { (self.entity_fns.destroy_spatial_entity)(entity.handle) };
        }
        match tracker.context {
            ContextState::Ready(context) => {
                unsafe { (self.entity_fns.destroy_spatial_context)(context) };
                info!("QR code tracking {:?} stopped", tracker.config);
            }
            // The context of a pending future is never handed over, so cancelling it is enough
            ContextState::Creating(future) => {
                let info = sys::FutureCancelInfoEXT {
                    ty: sys::FutureCancelInfoEXT::TYPE,
                    next: ptr::null(),
                    future,
                };
                unsafe { (self.future_fns.cancel_future)(self.instance.as_raw(), &info) };
            }
            ContextState::NotStarted | ContextState::Failed => (),
        }
    }
}

impl Drop for MarkerTracker {
    fn drop(&mut self) {
        for tracker in mem::take(&mut self.trackers) {
            self.destroy(tracker);
        }
    }
}

struct FoundEntity {
    id: sys::SpatialEntityIdEXT,
    state: sys::SpatialEntityTrackingStateEXT,
    bounds: sys::SpatialBounded2DDataEXT,
    marker: Option<sys::SpatialMarkerDataEXT>,
}

/// The two-call idiom, for an array of plain values.
unsafe fn two_call<T: Copy>(mut call: impl FnMut(u32, &mut u32, *mut T) -> sys::Result) -> Vec<T> {
    let mut count = 0;
    if call(0, &mut count, ptr::null_mut()) != sys::Result::SUCCESS || count == 0 {
        return Vec::new();
    }
    let mut items = Vec::with_capacity(count as usize);
    if call(count, &mut count, items.as_mut_ptr()) != sys::Result::SUCCESS {
        return Vec::new();
    }
    unsafe { items.set_len(count as usize) };

    items
}

fn request_spatial_data_permission() {
    #[cfg(target_os = "android")]
    alvr_system_info::try_get_permissions(&SPATIAL_DATA_PERMISSIONS);
}

fn spatial_data_permission_denied() -> bool {
    #[cfg(target_os = "android")]
    return SPATIAL_DATA_PERMISSIONS
        .iter()
        .any(|p| alvr_system_info::was_permission_denied(p));

    #[cfg(not(target_os = "android"))]
    false
}

fn has_spatial_data_permission() -> bool {
    #[cfg(target_os = "android")]
    return SPATIAL_DATA_PERMISSIONS
        .iter()
        .any(|p| alvr_system_info::has_permission(p));

    #[cfg(not(target_os = "android"))]
    true
}
