//! QR codes seen by the headset, through XR_EXT_spatial_marker_tracking.
//!
//! The server says which trackers to run: one spatial context per configuration, each with the
//! marker options OpenXR applications on the PC asked for. A tracker finds codes with a discovery
//! every [`DISCOVERY_INTERVAL`], and follows the codes it found with an update snapshot every
//! frame, so that moving codes are reported at the frame rate, for the frame's display time.

use crate::extra_extensions::{SpatialEntityData, SpatialMarkerTrackingEXT};
use alvr_common::{error, info, warn};
use alvr_packets::{MarkerCapabilities, MarkerData, MarkerPayload, MarkerTrackingConfig};
use openxr::{self as xr, sys};
use std::{
    collections::HashMap,
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
/// Each vendor names its own spatial data permission; a headset ignores the others.
#[cfg(target_os = "android")]
const SPATIAL_DATA_PERMISSIONS: [&str; 3] = [
    "com.oculus.permission.USE_SCENE",
    "android.permission.SCENE_UNDERSTANDING_COARSE",
    "com.picovr.permission.SPATIAL_DATA",
];

const PERMISSION_CHECK_INTERVAL: Duration = Duration::from_secs(1);
/// Android drops a permission request while another dialog (e.g. the microphone one) is open, so
/// the request is repeated until answered.
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
    ext: SpatialMarkerTrackingEXT,
    /// Stage space, which the stream's tracking is reported in.
    space: xr::Space,
    capabilities: MarkerCapabilities,
    permission: Option<PermissionWait>,
    trackers: Vec<Tracker>,
}

impl MarkerTracker {
    pub fn new(session: &xr::Session<xr::OpenGlEs>, system: xr::SystemId) -> Option<Self> {
        let Ok(ext) = SpatialMarkerTrackingEXT::new(session) else {
            info!("QR code tracking is not available");
            return None;
        };

        let capabilities = ext.capabilities(system).unwrap_or_default();
        info!("Spatial capabilities: {capabilities:?}");
        if !capabilities.contains(&sys::SpatialCapabilityEXT::MARKER_TRACKING_QR_CODE) {
            info!("QR code tracking is not supported");
            return None;
        }

        let features = ext
            .capability_features(system, sys::SpatialCapabilityEXT::MARKER_TRACKING_QR_CODE)
            .unwrap_or_default();
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
            ext,
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
        self.trackers.retain_mut(|tracker| {
            let kept = configs.contains(&tracker.config);
            if !kept {
                tracker.destroy(&self.ext);
            }

            kept
        });

        for config in configs {
            if !self
                .trackers
                .iter()
                .any(|tracker| tracker.config == *config)
            {
                self.trackers.push(Tracker::new(*config));
            }
        }
    }

    /// The codes each tracker sees at `time`, the frame's display time. A tracker is left out
    /// while it has seen none since its last report.
    pub fn update(&mut self, time: xr::Time) -> Vec<(MarkerTrackingConfig, Vec<MarkerData>)> {
        if !self.wait_for_permission() {
            return Vec::new();
        }

        self.trackers
            .iter_mut()
            .filter_map(|tracker| tracker.update(&self.ext, &self.space, time))
            .collect()
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
            && !should_show_spatial_data_permission_rationale()
        {
            request_spatial_data_permission();
            wait.last_request = Instant::now();
            wait.requests += 1;
        }

        false
    }
}

impl Drop for MarkerTracker {
    fn drop(&mut self) {
        for tracker in &mut self.trackers {
            tracker.destroy(&self.ext);
        }
    }
}

impl Tracker {
    fn new(config: MarkerTrackingConfig) -> Self {
        Self {
            config,
            context: ContextState::NotStarted,
            discovery: None,
            last_discovery: None,
            entities: HashMap::new(),
            reported_markers: false,
        }
    }

    fn update(
        &mut self,
        ext: &SpatialMarkerTrackingEXT,
        space: &xr::Space,
        time: xr::Time,
    ) -> Option<(MarkerTrackingConfig, Vec<MarkerData>)> {
        let context = match self.context {
            ContextState::NotStarted => {
                self.context = self.create_context(ext);
                return None;
            }
            ContextState::Creating(future) => {
                if ext.is_ready(future) {
                    self.context = self.complete_context(ext, future);
                }
                return None;
            }
            ContextState::Ready(context) => context,
            ContextState::Failed => return None,
        };

        self.discover(ext, space, context, time);

        let markers = self.locate(ext, space, context, time);
        if markers.is_empty() && !self.reported_markers {
            return None;
        }
        self.reported_markers = !markers.is_empty();

        Some((self.config, markers))
    }

    fn create_context(&self, ext: &SpatialMarkerTrackingEXT) -> ContextState {
        // Only the options the headset reported: the server sends nothing else
        match ext.create_qr_code_context_async(
            &COMPONENTS,
            self.config.static_markers,
            self.config.marker_size,
        ) {
            Ok(future) => ContextState::Creating(future),
            Err(e) => {
                error!("Cannot start QR code tracking {:?}: {e:?}", self.config);
                ContextState::Failed
            }
        }
    }

    fn complete_context(
        &self,
        ext: &SpatialMarkerTrackingEXT,
        future: sys::FutureEXT,
    ) -> ContextState {
        match ext.create_context_complete(future) {
            Ok(context) => {
                info!("QR code tracking {:?} started", self.config);
                ContextState::Ready(context)
            }
            Err(e) => {
                error!("Cannot start QR code tracking {:?}: {e:?}", self.config);
                ContextState::Failed
            }
        }
    }

    /// Completes a pending discovery, or starts one every [`DISCOVERY_INTERVAL`].
    fn discover(
        &mut self,
        ext: &SpatialMarkerTrackingEXT,
        space: &xr::Space,
        context: sys::SpatialContextEXT,
        time: xr::Time,
    ) {
        match self.discovery {
            Some(future) => {
                if ext.is_ready(future) {
                    self.discovery = None;
                    self.complete_discovery(ext, space, context, future, time);
                }
            }
            None => {
                if self
                    .last_discovery
                    .is_none_or(|last| last.elapsed() >= DISCOVERY_INTERVAL)
                {
                    self.last_discovery = Some(Instant::now());
                    self.discovery = ext
                        .create_discovery_snapshot_async(context, &COMPONENTS)
                        .inspect_err(|e| warn!("QR code discovery failed: {e:?}"))
                        .ok();
                }
            }
        }
    }

    /// Adds the codes the discovery found to the ones the tracker follows.
    fn complete_discovery(
        &mut self,
        ext: &SpatialMarkerTrackingEXT,
        space: &xr::Space,
        context: sys::SpatialContextEXT,
        future: sys::FutureEXT,
        time: xr::Time,
    ) {
        let snapshot = match ext.create_discovery_snapshot_complete(context, future, space, time) {
            Ok(snapshot) => snapshot,
            Err(e) => {
                warn!("QR code discovery failed: {e:?}");
                return;
            }
        };

        match ext.query(snapshot, &COMPONENTS, true) {
            Ok(found) => {
                for entity in found {
                    self.follow(ext, context, snapshot, entity);
                }
            }
            Err(e) => warn!("Cannot read QR codes: {e:?}"),
        }

        ext.destroy_snapshot(snapshot);
    }

    /// Follows a code the discovery found, unless it is followed already or not decoded yet.
    fn follow(
        &mut self,
        ext: &SpatialMarkerTrackingEXT,
        context: sys::SpatialContextEXT,
        snapshot: sys::SpatialSnapshotEXT,
        found: SpatialEntityData,
    ) {
        if found.state != sys::SpatialEntityTrackingStateEXT::TRACKING
            || self.entities.contains_key(&found.id)
        {
            return;
        }
        let Some(payload) = found
            .marker
            .and_then(|marker| read_payload(ext, snapshot, marker.data))
        else {
            return;
        };

        let handle = match ext.create_entity_from_id(context, found.id) {
            Ok(handle) => handle,
            Err(e) => {
                warn!("Cannot follow QR code {payload}: {e:?}");
                return;
            }
        };

        info!(
            "QR code {payload} at {:?}, size {} ({:?})",
            found.bounds.center.position, found.bounds.extents.width, self.config
        );
        self.entities.insert(
            found.id,
            Entity {
                handle,
                payload,
                last_tracked: None,
                found_at: Instant::now(),
            },
        );
    }

    /// Where the codes the tracker follows are at `time`.
    fn locate(
        &mut self,
        ext: &SpatialMarkerTrackingEXT,
        space: &xr::Space,
        context: sys::SpatialContextEXT,
        time: xr::Time,
    ) -> Vec<MarkerData> {
        if self.entities.is_empty() {
            return Vec::new();
        }
        let Some(located) = self.query_poses(ext, space, context, time) else {
            return Vec::new();
        };

        let now = Instant::now();
        let stopped = self.apply_tracking_states(located, now);
        self.forget_stale(ext, &stopped, now);

        self.markers_to_report(now)
    }

    /// The poses of the followed codes, `None` if they cannot be located.
    fn query_poses(
        &self,
        ext: &SpatialMarkerTrackingEXT,
        space: &xr::Space,
        context: sys::SpatialContextEXT,
        time: xr::Time,
    ) -> Option<Vec<SpatialEntityData>> {
        let handles = self
            .entities
            .values()
            .map(|entity| entity.handle)
            .collect::<Vec<_>>();
        let snapshot =
            match ext.create_update_snapshot(context, &handles, &POSE_COMPONENT, space, time) {
                Ok(snapshot) => snapshot,
                Err(e) => {
                    warn!("Cannot locate QR codes: {e:?}");
                    return None;
                }
            };
        let located = ext.query(snapshot, &POSE_COMPONENT, false);
        ext.destroy_snapshot(snapshot);

        Some(located.unwrap_or_else(|e| {
            warn!("Cannot read QR codes: {e:?}");
            Vec::new()
        }))
    }

    /// Records the tracked poses. Returns the codes the headset stopped tracking.
    fn apply_tracking_states(
        &mut self,
        located: Vec<SpatialEntityData>,
        now: Instant,
    ) -> Vec<sys::SpatialEntityIdEXT> {
        let mut stopped = Vec::new();
        for found in located {
            let Some(entity) = self.entities.get_mut(&found.id) else {
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

        stopped
    }

    /// Forgets codes the headset stopped tracking, or did not see for long, so that the list does
    /// not grow forever.
    fn forget_stale(
        &mut self,
        ext: &SpatialMarkerTrackingEXT,
        stopped: &[sys::SpatialEntityIdEXT],
        now: Instant,
    ) {
        self.entities.retain(|id, entity| {
            let last = entity
                .last_tracked
                .as_ref()
                .map_or(entity.found_at, |(_, tracked_at)| *tracked_at);
            let forget = stopped.contains(id) || now.duration_since(last) >= ENTITY_TIMEOUT;
            if forget {
                ext.destroy_entity(entity.handle);
            }

            !forget
        });
    }

    /// The codes tracked at `now`, and for a static tracker the ones tracked shortly before.
    fn markers_to_report(&self, now: Instant) -> Vec<MarkerData> {
        // Static codes stay put while tracking drops out briefly; moving ones may not
        let timeout = if self.config.static_markers {
            STATIC_MARKER_TIMEOUT
        } else {
            Duration::ZERO
        };

        self.entities
            .values()
            .filter_map(|entity| {
                let (marker, tracked_at) = entity.last_tracked.as_ref()?;
                (now.duration_since(*tracked_at) <= timeout).then(|| marker.clone())
            })
            .collect()
    }

    fn destroy(&mut self, ext: &SpatialMarkerTrackingEXT) {
        for (_, entity) in self.entities.drain() {
            ext.destroy_entity(entity.handle);
        }
        match self.context {
            ContextState::Ready(context) => {
                ext.destroy_context(context);
                info!("QR code tracking {:?} stopped", self.config);
            }
            // The context of a pending future is never handed over, so cancelling it is enough
            ContextState::Creating(future) => ext.cancel(future),
            ContextState::NotStarted | ContextState::Failed => (),
        }
        self.context = ContextState::NotStarted;
    }
}

/// The data of a QR code, `None` if it is not decoded yet.
fn read_payload(
    ext: &SpatialMarkerTrackingEXT,
    snapshot: sys::SpatialSnapshotEXT,
    buffer: sys::SpatialBufferEXT,
) -> Option<MarkerPayload> {
    if buffer.buffer_id.into_raw() == 0 {
        return None;
    }

    match buffer.buffer_type {
        sys::SpatialBufferTypeEXT::STRING => ext
            .buffer_string(snapshot, buffer.buffer_id)
            .ok()
            .map(MarkerPayload::Text),
        sys::SpatialBufferTypeEXT::UINT8 => ext
            .buffer_uint8(snapshot, buffer.buffer_id)
            .ok()
            .map(MarkerPayload::Bytes),
        _ => None,
    }
}

fn request_spatial_data_permission() {
    #[cfg(target_os = "android")]
    alvr_system_info::try_get_permissions(&SPATIAL_DATA_PERMISSIONS);
}

fn should_show_spatial_data_permission_rationale() -> bool {
    #[cfg(target_os = "android")]
    return SPATIAL_DATA_PERMISSIONS
        .iter()
        .any(|p| alvr_system_info::should_show_permission_rationale(p));

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
