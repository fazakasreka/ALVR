use openxr::{
    self as xr, raw,
    sys::{self, Handle},
};
use std::{ffi::CStr, mem, ptr};

pub struct SpatialEntityData {
    pub id: sys::SpatialEntityIdEXT,
    pub state: sys::SpatialEntityTrackingStateEXT,
    pub bounds: sys::SpatialBounded2DDataEXT,
    pub marker: Option<sys::SpatialMarkerDataEXT>,
}

pub struct SpatialMarkerTrackingEXT {
    session: xr::Session<xr::OpenGlEs>,
    entity_fns: raw::SpatialEntityEXT,
    future_fns: raw::FutureEXT,
}

impl SpatialMarkerTrackingEXT {
    pub fn new(session: &xr::Session<xr::OpenGlEs>) -> xr::Result<Self> {
        let exts = session.instance().exts();
        let (Some(entity_fns), Some(future_fns), Some(_)) = (
            exts.ext_spatial_entity,
            exts.ext_future,
            exts.ext_spatial_marker_tracking,
        ) else {
            return Err(sys::Result::ERROR_EXTENSION_NOT_PRESENT);
        };

        Ok(Self {
            session: session.clone(),
            entity_fns,
            future_fns,
        })
    }

    pub fn capabilities(&self, system: xr::SystemId) -> xr::Result<Vec<sys::SpatialCapabilityEXT>> {
        let instance = self.session.instance().as_raw();

        unsafe {
            two_call(|capacity, count, items| {
                (self.entity_fns.enumerate_spatial_capabilities)(
                    instance, system, capacity, count, items,
                )
            })
        }
    }

    pub fn capability_features(
        &self,
        system: xr::SystemId,
        capability: sys::SpatialCapabilityEXT,
    ) -> xr::Result<Vec<sys::SpatialCapabilityFeatureEXT>> {
        let instance = self.session.instance().as_raw();

        unsafe {
            two_call(|capacity, count, items| {
                (self.entity_fns.enumerate_spatial_capability_features)(
                    instance, system, capability, capacity, count, items,
                )
            })
        }
    }

    pub fn create_qr_code_context_async(
        &self,
        components: &[sys::SpatialComponentTypeEXT],
        static_markers: bool,
        marker_size: Option<f32>,
    ) -> xr::Result<sys::FutureEXT> {
        let static_markers_info = sys::SpatialMarkerStaticOptimizationEXT {
            ty: sys::SpatialMarkerStaticOptimizationEXT::TYPE,
            next: ptr::null(),
            optimize_for_static_marker: static_markers.into(),
        };
        let marker_size_info = marker_size.map(|size| sys::SpatialMarkerSizeEXT {
            ty: sys::SpatialMarkerSizeEXT::TYPE,
            next: (&raw const static_markers_info).cast(),
            marker_side_length: size,
        });
        let qr_config = sys::SpatialCapabilityConfigurationQrCodeEXT {
            ty: sys::SpatialCapabilityConfigurationQrCodeEXT::TYPE,
            next: match &marker_size_info {
                Some(info) => (info as *const sys::SpatialMarkerSizeEXT).cast(),
                None if static_markers => (&raw const static_markers_info).cast(),
                None => ptr::null(),
            },
            capability: sys::SpatialCapabilityEXT::MARKER_TRACKING_QR_CODE,
            enabled_component_count: components.len() as u32,
            enabled_components: components.as_ptr(),
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
        unsafe {
            super::xr_res((self.entity_fns.create_spatial_context_async)(
                self.session.as_raw(),
                &create_info,
                &mut future,
            ))?
        };

        Ok(future)
    }

    pub fn create_context_complete(
        &self,
        future: sys::FutureEXT,
    ) -> xr::Result<sys::SpatialContextEXT> {
        let mut completion = sys::CreateSpatialContextCompletionEXT {
            ty: sys::CreateSpatialContextCompletionEXT::TYPE,
            next: ptr::null_mut(),
            future_result: sys::Result::SUCCESS,
            spatial_context: sys::SpatialContextEXT::NULL,
        };
        unsafe {
            super::xr_res((self.entity_fns.create_spatial_context_complete)(
                self.session.as_raw(),
                future,
                &mut completion,
            ))?
        };
        super::xr_res(completion.future_result)?;

        Ok(completion.spatial_context)
    }

    pub fn is_ready(&self, future: sys::FutureEXT) -> bool {
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
        let res = unsafe {
            (self.future_fns.poll_future)(self.session.instance().as_raw(), &info, &mut result)
        };

        res == sys::Result::SUCCESS && result.state == sys::FutureStateEXT::READY
    }

    pub fn cancel(&self, future: sys::FutureEXT) {
        let info = sys::FutureCancelInfoEXT {
            ty: sys::FutureCancelInfoEXT::TYPE,
            next: ptr::null(),
            future,
        };
        unsafe { (self.future_fns.cancel_future)(self.session.instance().as_raw(), &info) };
    }

    pub fn create_discovery_snapshot_async(
        &self,
        context: sys::SpatialContextEXT,
        components: &[sys::SpatialComponentTypeEXT],
    ) -> xr::Result<sys::FutureEXT> {
        let create_info = sys::SpatialDiscoverySnapshotCreateInfoEXT {
            ty: sys::SpatialDiscoverySnapshotCreateInfoEXT::TYPE,
            next: ptr::null(),
            component_type_count: components.len() as u32,
            component_types: components.as_ptr(),
        };
        let mut future = sys::FutureEXT::default();
        unsafe {
            super::xr_res((self.entity_fns.create_spatial_discovery_snapshot_async)(
                context,
                &create_info,
                &mut future,
            ))?
        };

        Ok(future)
    }

    pub fn create_discovery_snapshot_complete(
        &self,
        context: sys::SpatialContextEXT,
        future: sys::FutureEXT,
        base: &xr::Space,
        time: xr::Time,
    ) -> xr::Result<sys::SpatialSnapshotEXT> {
        let info = sys::CreateSpatialDiscoverySnapshotCompletionInfoEXT {
            ty: sys::CreateSpatialDiscoverySnapshotCompletionInfoEXT::TYPE,
            next: ptr::null(),
            base_space: base.as_raw(),
            time,
            future,
        };
        let mut completion = sys::CreateSpatialDiscoverySnapshotCompletionEXT {
            ty: sys::CreateSpatialDiscoverySnapshotCompletionEXT::TYPE,
            next: ptr::null_mut(),
            future_result: sys::Result::SUCCESS,
            snapshot: sys::SpatialSnapshotEXT::NULL,
        };
        unsafe {
            super::xr_res(
                (self.entity_fns.create_spatial_discovery_snapshot_complete)(
                    context,
                    &info,
                    &mut completion,
                ),
            )?
        };
        super::xr_res(completion.future_result)?;

        Ok(completion.snapshot)
    }

    pub fn create_update_snapshot(
        &self,
        context: sys::SpatialContextEXT,
        entities: &[sys::SpatialEntityEXT],
        components: &[sys::SpatialComponentTypeEXT],
        base: &xr::Space,
        time: xr::Time,
    ) -> xr::Result<sys::SpatialSnapshotEXT> {
        let create_info = sys::SpatialUpdateSnapshotCreateInfoEXT {
            ty: sys::SpatialUpdateSnapshotCreateInfoEXT::TYPE,
            next: ptr::null(),
            entity_count: entities.len() as u32,
            entities: entities.as_ptr(),
            component_type_count: components.len() as u32,
            component_types: components.as_ptr(),
            base_space: base.as_raw(),
            time,
        };
        let mut snapshot = sys::SpatialSnapshotEXT::NULL;
        unsafe {
            super::xr_res((self.entity_fns.create_spatial_update_snapshot)(
                context,
                &create_info,
                &mut snapshot,
            ))?
        };

        Ok(snapshot)
    }

    // The entities of a snapshot with their bounds, and their marker data if `with_markers`
    pub fn query(
        &self,
        snapshot: sys::SpatialSnapshotEXT,
        components: &[sys::SpatialComponentTypeEXT],
        with_markers: bool,
    ) -> xr::Result<Vec<SpatialEntityData>> {
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
        unsafe {
            super::xr_res((self.entity_fns.query_spatial_component_data)(
                snapshot, &condition, &mut query,
            ))?
        };
        let count = query.entity_id_count_output as usize;
        if count == 0 {
            return Ok(Vec::new());
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

        unsafe {
            super::xr_res((self.entity_fns.query_spatial_component_data)(
                snapshot, &condition, &mut query,
            ))?
        };

        Ok((0..count)
            .map(|i| SpatialEntityData {
                id: ids[i],
                state: states[i],
                bounds: bounds[i],
                marker: with_markers.then_some(marker_data[i]),
            })
            .collect())
    }

    pub fn buffer_string(
        &self,
        snapshot: sys::SpatialSnapshotEXT,
        buffer_id: sys::SpatialBufferIdEXT,
    ) -> xr::Result<String> {
        let info = buffer_info(buffer_id);
        let text = unsafe {
            two_call(|capacity, count, items| {
                (self.entity_fns.get_spatial_buffer_string)(snapshot, &info, capacity, count, items)
            })?
        }
        .into_iter()
        .map(|c| c as u8)
        .collect::<Vec<_>>();

        // The count includes the terminating null
        CStr::from_bytes_until_nul(&text)
            .map(|text| text.to_string_lossy().into_owned())
            .map_err(|_| sys::Result::ERROR_VALIDATION_FAILURE)
    }

    pub fn buffer_uint8(
        &self,
        snapshot: sys::SpatialSnapshotEXT,
        buffer_id: sys::SpatialBufferIdEXT,
    ) -> xr::Result<Vec<u8>> {
        let info = buffer_info(buffer_id);

        unsafe {
            two_call(|capacity, count, items| {
                (self.entity_fns.get_spatial_buffer_uint8)(snapshot, &info, capacity, count, items)
            })
        }
    }

    pub fn create_entity_from_id(
        &self,
        context: sys::SpatialContextEXT,
        entity_id: sys::SpatialEntityIdEXT,
    ) -> xr::Result<sys::SpatialEntityEXT> {
        let create_info = sys::SpatialEntityFromIdCreateInfoEXT {
            ty: sys::SpatialEntityFromIdCreateInfoEXT::TYPE,
            next: ptr::null(),
            entity_id,
        };
        let mut handle = sys::SpatialEntityEXT::NULL;
        unsafe {
            super::xr_res((self.entity_fns.create_spatial_entity_from_id)(
                context,
                &create_info,
                &mut handle,
            ))?
        };

        Ok(handle)
    }

    pub fn destroy_snapshot(&self, snapshot: sys::SpatialSnapshotEXT) {
        unsafe { (self.entity_fns.destroy_spatial_snapshot)(snapshot) };
    }

    pub fn destroy_entity(&self, entity: sys::SpatialEntityEXT) {
        unsafe { (self.entity_fns.destroy_spatial_entity)(entity) };
    }

    pub fn destroy_context(&self, context: sys::SpatialContextEXT) {
        unsafe { (self.entity_fns.destroy_spatial_context)(context) };
    }
}

fn buffer_info(buffer_id: sys::SpatialBufferIdEXT) -> sys::SpatialBufferGetInfoEXT {
    sys::SpatialBufferGetInfoEXT {
        ty: sys::SpatialBufferGetInfoEXT::TYPE,
        next: ptr::null(),
        buffer_id,
    }
}

// The two-call idiom, for an array of plain values
unsafe fn two_call<T: Copy>(
    mut call: impl FnMut(u32, &mut u32, *mut T) -> sys::Result,
) -> xr::Result<Vec<T>> {
    let mut count = 0;
    super::xr_res(call(0, &mut count, ptr::null_mut()))?;
    if count == 0 {
        return Ok(Vec::new());
    }

    let mut items = Vec::with_capacity(count as usize);
    super::xr_res(call(count, &mut count, items.as_mut_ptr()))?;
    unsafe { items.set_len(count as usize) };

    Ok(items)
}
