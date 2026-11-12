//! QR codes seen by the headset, through XR_EXT_spatial_marker_tracking.

use alvr_common::{error, info, warn};
use alvr_packets::MarkerData;
use openxr::{self as xr, sys, sys::Handle};
use std::{
    collections::HashMap,
    ffi::CStr,
    mem, ptr,
    time::{Duration, Instant},
};

const DISCOVERY_INTERVAL: Duration = Duration::from_millis(500);
// Discoveries miss a code in view every so often, so one is kept for this long after it was seen
const MARKER_TIMEOUT: Duration = Duration::from_secs(1);
const COMPONENTS: [sys::SpatialComponentTypeEXT; 2] = [
    sys::SpatialComponentTypeEXT::MARKER,
    sys::SpatialComponentTypeEXT::BOUNDED_2D,
];
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
    WaitingForPermission(PermissionWait),
    Creating(sys::FutureEXT),
    Ready(sys::SpatialContextEXT),
    Failed,
}

pub struct MarkerTracker {
    instance: xr::Instance,
    session: xr::Session<xr::OpenGlEs>,
    entity_fns: xr::raw::SpatialEntityEXT,
    future_fns: xr::raw::FutureEXT,
    // Stage space, which the stream's tracking is reported in
    space: xr::Space,
    context: ContextState,
    snapshot: Option<sys::FutureEXT>,
    last_discovery: Option<Instant>,
    seen: HashMap<String, (MarkerData, Instant)>,
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
            let mut count = 0;
            (entity_fns.enumerate_spatial_capabilities)(
                instance.as_raw(),
                system,
                0,
                &mut count,
                ptr::null_mut(),
            );
            let mut capabilities = vec![sys::SpatialCapabilityEXT::from_raw(0); count as usize];
            (entity_fns.enumerate_spatial_capabilities)(
                instance.as_raw(),
                system,
                count,
                &mut count,
                capabilities.as_mut_ptr(),
            );
            capabilities
        };
        info!("Spatial capabilities: {capabilities:?}");
        if !capabilities.contains(&sys::SpatialCapabilityEXT::MARKER_TRACKING_QR_CODE) {
            info!("QR code tracking is not supported");
            return None;
        }

        let space = session
            .create_reference_space(xr::ReferenceSpaceType::STAGE, xr::Posef::IDENTITY)
            .ok()?;

        let mut this = Self {
            instance,
            session: session.clone(),
            entity_fns,
            future_fns,
            space,
            context: ContextState::Failed,
            snapshot: None,
            last_discovery: None,
            seen: HashMap::new(),
        };

        this.context = if has_spatial_data_permission() {
            this.create_context()
        } else {
            request_spatial_data_permission();
            info!("Waiting for the spatial data permission");
            ContextState::WaitingForPermission(PermissionWait {
                last_check: Instant::now(),
                last_request: Instant::now(),
                requests: 1,
            })
        };

        Some(this)
    }

    fn create_context(&self) -> ContextState {
        let static_markers = sys::SpatialMarkerStaticOptimizationEXT {
            ty: sys::SpatialMarkerStaticOptimizationEXT::TYPE,
            next: ptr::null(),
            optimize_for_static_marker: sys::TRUE,
        };
        let qr_config = sys::SpatialCapabilityConfigurationQrCodeEXT {
            ty: sys::SpatialCapabilityConfigurationQrCodeEXT::TYPE,
            next: (&raw const static_markers).cast(),
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
            error!("Cannot start QR code tracking: {result:?}");
            return ContextState::Failed;
        }

        ContextState::Creating(future)
    }

    /// Returns the QR codes currently tracked whenever a new discovery completes.
    pub fn update(&mut self, time: xr::Time) -> Option<Vec<MarkerData>> {
        match self.context {
            ContextState::WaitingForPermission(ref mut wait) => {
                if wait.last_check.elapsed() < PERMISSION_CHECK_INTERVAL {
                    return None;
                }
                wait.last_check = Instant::now();

                if has_spatial_data_permission() {
                    info!("Spatial data permission granted");
                    self.context = self.create_context();
                } else if wait.requests < PERMISSION_REQUEST_COUNT
                    && wait.last_request.elapsed() >= PERMISSION_REQUEST_INTERVAL
                    && !spatial_data_permission_denied()
                {
                    request_spatial_data_permission();
                    wait.last_request = Instant::now();
                    wait.requests += 1;
                }
                None
            }
            ContextState::Creating(future) => {
                if self.is_ready(future) {
                    self.context = self.complete_context(future);
                }
                None
            }
            ContextState::Ready(context) => {
                if let Some(future) = self.snapshot {
                    if !self.is_ready(future) {
                        return None;
                    }
                    self.snapshot = None;

                    return self.complete_discovery(context, future, time);
                }

                if self
                    .last_discovery
                    .is_none_or(|last| last.elapsed() >= DISCOVERY_INTERVAL)
                {
                    self.last_discovery = Some(Instant::now());
                    self.snapshot = self.start_discovery(context);
                }
                None
            }
            ContextState::Failed => None,
        }
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

    fn complete_context(&self, future: sys::FutureEXT) -> ContextState {
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
            info!("QR code tracking started");
            ContextState::Ready(completion.spatial_context)
        } else {
            error!(
                "Cannot start QR code tracking: {result:?}, {:?}",
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

    fn complete_discovery(
        &mut self,
        context: sys::SpatialContextEXT,
        future: sys::FutureEXT,
        time: xr::Time,
    ) -> Option<Vec<MarkerData>> {
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
            return None;
        }

        let detected = self.read_snapshot(completion.snapshot);
        unsafe { (self.entity_fns.destroy_spatial_snapshot)(completion.snapshot) };

        let now = Instant::now();
        for marker in detected {
            if !self.seen.contains_key(&marker.id) {
                info!(
                    "QR code \"{}\" at {:?}, {:?}, size {}",
                    marker.id, marker.pose.position, marker.pose.orientation, marker.size
                );
            }
            self.seen.insert(marker.id.clone(), (marker, now));
        }
        self.seen.retain(|id, (_, seen_at)| {
            let kept = now.duration_since(*seen_at) < MARKER_TIMEOUT;
            if !kept {
                info!("QR code \"{id}\" lost");
            }
            kept
        });

        Some(
            self.seen
                .values()
                .map(|(marker, _)| marker.clone())
                .collect(),
        )
    }

    fn read_snapshot(&self, snapshot: sys::SpatialSnapshotEXT) -> Vec<MarkerData> {
        let condition = sys::SpatialComponentDataQueryConditionEXT {
            ty: sys::SpatialComponentDataQueryConditionEXT::TYPE,
            next: ptr::null(),
            component_type_count: COMPONENTS.len() as u32,
            component_types: COMPONENTS.as_ptr(),
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
            next: (&raw mut marker_list).cast(),
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
            .filter(|&i| states[i] == sys::SpatialEntityTrackingStateEXT::TRACKING)
            .map(|i| MarkerData {
                id: self
                    .read_text(snapshot, marker_data[i].data)
                    .unwrap_or_else(|| marker_data[i].marker_id.to_string()),
                pose: crate::from_xr_pose(bounds[i].center),
                size: bounds[i].extents.width,
            })
            .collect()
    }

    fn read_text(
        &self,
        snapshot: sys::SpatialSnapshotEXT,
        buffer: sys::SpatialBufferEXT,
    ) -> Option<String> {
        if buffer.buffer_type != sys::SpatialBufferTypeEXT::STRING {
            return None;
        }

        let info = sys::SpatialBufferGetInfoEXT {
            ty: sys::SpatialBufferGetInfoEXT::TYPE,
            next: ptr::null(),
            buffer_id: buffer.buffer_id,
        };
        let mut count = 0;
        unsafe {
            (self.entity_fns.get_spatial_buffer_string)(
                snapshot,
                &info,
                0,
                &mut count,
                ptr::null_mut(),
            )
        };
        if count == 0 {
            return None;
        }
        let mut text = vec![0; count as usize];
        let result = unsafe {
            (self.entity_fns.get_spatial_buffer_string)(
                snapshot,
                &info,
                count,
                &mut count,
                text.as_mut_ptr(),
            )
        };

        (result == sys::Result::SUCCESS).then(|| {
            unsafe { CStr::from_ptr(text.as_ptr()) }
                .to_string_lossy()
                .into_owned()
        })
    }
}

impl Drop for MarkerTracker {
    fn drop(&mut self) {
        if let ContextState::Ready(context) = self.context {
            unsafe { (self.entity_fns.destroy_spatial_context)(context) };
        }
    }
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
