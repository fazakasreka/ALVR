//! Checks the ALVR OpenXR layer end to end: a minimal OpenXR application on the active runtime
//! (SteamVR) that prints the QR codes it gets through XR_EXT_spatial_marker_tracking, relative to
//! the head (VIEW) and to the STAGE space.
//!
//! `cargo run -p alvr_openxr_layer --example marker_probe -- [seconds] [openxr_loader.dll]`
//!
//! While it runs, the headset shows black: the probe submits empty frames.

use alvr_marker_ipc::Payload;
use openxr::{self as xr, sys, sys::Handle};
use std::{
    ffi::CStr,
    mem,
    path::PathBuf,
    ptr,
    time::{Duration, Instant},
};
use windows::{
    Win32::Graphics::{
        Direct3D::D3D_DRIVER_TYPE_HARDWARE,
        Direct3D11::{
            D3D11_CREATE_DEVICE_BGRA_SUPPORT, D3D11_SDK_VERSION, D3D11CreateDevice, ID3D11Device,
        },
    },
    core::Interface,
};

const DEFAULT_LOADER: &str =
    r"C:\Program Files (x86)\Steam\steamapps\common\SteamVR\bin\win64\openxr_loader.dll";
const DISCOVERY_INTERVAL: Duration = Duration::from_secs(1);
const COMPONENTS: [sys::SpatialComponentTypeEXT; 2] = [
    sys::SpatialComponentTypeEXT::MARKER,
    sys::SpatialComponentTypeEXT::BOUNDED_2D,
];

fn check(result: sys::Result, what: &str) {
    assert_eq!(result, sys::Result::SUCCESS, "{what}");
}

struct Spatial {
    fns: xr::raw::SpatialEntityEXT,
    future_fns: xr::raw::FutureEXT,
    instance: sys::Instance,
}

impl Spatial {
    fn wait(&self, future: sys::FutureEXT) {
        loop {
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
            check(
                unsafe { (self.future_fns.poll_future)(self.instance, &info, &mut result) },
                "xrPollFutureEXT",
            );
            if result.state == sys::FutureStateEXT::READY {
                return;
            }
            std::thread::sleep(Duration::from_millis(5));
        }
    }

    fn create_context(&self, session: sys::Session) -> sys::SpatialContextEXT {
        let config = sys::SpatialCapabilityConfigurationQrCodeEXT {
            ty: sys::SpatialCapabilityConfigurationQrCodeEXT::TYPE,
            next: ptr::null(),
            capability: sys::SpatialCapabilityEXT::MARKER_TRACKING_QR_CODE,
            enabled_component_count: COMPONENTS.len() as u32,
            enabled_components: COMPONENTS.as_ptr(),
        };
        let configs =
            [(&raw const config).cast::<sys::SpatialCapabilityConfigurationBaseHeaderEXT>()];
        let info = sys::SpatialContextCreateInfoEXT {
            ty: sys::SpatialContextCreateInfoEXT::TYPE,
            next: ptr::null(),
            capability_config_count: 1,
            capability_configs: configs.as_ptr(),
        };
        let mut future = sys::FutureEXT::from_raw(0);
        check(
            unsafe { (self.fns.create_spatial_context_async)(session, &info, &mut future) },
            "xrCreateSpatialContextAsyncEXT",
        );
        self.wait(future);

        let mut completion = sys::CreateSpatialContextCompletionEXT {
            ty: sys::CreateSpatialContextCompletionEXT::TYPE,
            next: ptr::null_mut(),
            future_result: sys::Result::ERROR_RUNTIME_FAILURE,
            spatial_context: sys::SpatialContextEXT::from_raw(0),
        };
        check(
            unsafe { (self.fns.create_spatial_context_complete)(session, future, &mut completion) },
            "xrCreateSpatialContextCompleteEXT",
        );
        check(completion.future_result, "spatial context creation");

        completion.spatial_context
    }

    fn discover(
        &self,
        context: sys::SpatialContextEXT,
        base: sys::Space,
        time: sys::Time,
    ) -> Vec<(
        String,
        sys::SpatialEntityTrackingStateEXT,
        sys::SpatialBounded2DDataEXT,
    )> {
        let info = sys::SpatialDiscoverySnapshotCreateInfoEXT {
            ty: sys::SpatialDiscoverySnapshotCreateInfoEXT::TYPE,
            next: ptr::null(),
            component_type_count: COMPONENTS.len() as u32,
            component_types: COMPONENTS.as_ptr(),
        };
        let mut future = sys::FutureEXT::from_raw(0);
        check(
            unsafe {
                (self.fns.create_spatial_discovery_snapshot_async)(context, &info, &mut future)
            },
            "xrCreateSpatialDiscoverySnapshotAsyncEXT",
        );
        self.wait(future);

        let completion_info = sys::CreateSpatialDiscoverySnapshotCompletionInfoEXT {
            ty: sys::CreateSpatialDiscoverySnapshotCompletionInfoEXT::TYPE,
            next: ptr::null(),
            base_space: base,
            time,
            future,
        };
        let mut completion = sys::CreateSpatialDiscoverySnapshotCompletionEXT {
            ty: sys::CreateSpatialDiscoverySnapshotCompletionEXT::TYPE,
            next: ptr::null_mut(),
            future_result: sys::Result::ERROR_RUNTIME_FAILURE,
            snapshot: sys::SpatialSnapshotEXT::from_raw(0),
        };
        check(
            unsafe {
                (self.fns.create_spatial_discovery_snapshot_complete)(
                    context,
                    &completion_info,
                    &mut completion,
                )
            },
            "xrCreateSpatialDiscoverySnapshotCompleteEXT",
        );
        check(completion.future_result, "discovery");
        let snapshot = completion.snapshot;

        let condition = sys::SpatialComponentDataQueryConditionEXT {
            ty: sys::SpatialComponentDataQueryConditionEXT::TYPE,
            next: ptr::null(),
            component_type_count: COMPONENTS.len() as u32,
            component_types: COMPONENTS.as_ptr(),
        };
        let mut result = sys::SpatialComponentDataQueryResultEXT {
            ty: sys::SpatialComponentDataQueryResultEXT::TYPE,
            next: ptr::null_mut(),
            entity_id_capacity_input: 0,
            entity_id_count_output: 0,
            entity_ids: ptr::null_mut(),
            entity_state_capacity_input: 0,
            entity_state_count_output: 0,
            entity_states: ptr::null_mut(),
        };
        check(
            unsafe { (self.fns.query_spatial_component_data)(snapshot, &condition, &mut result) },
            "xrQuerySpatialComponentDataEXT",
        );
        let count = result.entity_id_count_output as usize;

        let mut entities = Vec::new();
        if count > 0 {
            let mut ids = vec![sys::SpatialEntityIdEXT::from_raw(0); count];
            let mut states = vec![sys::SpatialEntityTrackingStateEXT::STOPPED; count];
            let mut markers = vec![unsafe { mem::zeroed::<sys::SpatialMarkerDataEXT>() }; count];
            let mut bounds = vec![sys::SpatialBounded2DDataEXT::default(); count];
            let mut marker_list = sys::SpatialComponentMarkerListEXT {
                ty: sys::SpatialComponentMarkerListEXT::TYPE,
                next: ptr::null_mut(),
                marker_count: count as u32,
                markers: markers.as_mut_ptr(),
            };
            let mut bounds_list = sys::SpatialComponentBounded2DListEXT {
                ty: sys::SpatialComponentBounded2DListEXT::TYPE,
                next: (&raw mut marker_list).cast(),
                bound_count: count as u32,
                bounds: bounds.as_mut_ptr(),
            };
            result.next = (&raw mut bounds_list).cast();
            result.entity_id_capacity_input = count as u32;
            result.entity_ids = ids.as_mut_ptr();
            result.entity_state_capacity_input = count as u32;
            result.entity_states = states.as_mut_ptr();
            check(
                unsafe {
                    (self.fns.query_spatial_component_data)(snapshot, &condition, &mut result)
                },
                "xrQuerySpatialComponentDataEXT",
            );

            for i in 0..count {
                let text = if states[i] == sys::SpatialEntityTrackingStateEXT::TRACKING {
                    self.data(snapshot, markers[i].data)
                } else {
                    String::new()
                };
                entities.push((text, states[i], bounds[i]));
            }
        }

        unsafe { (self.fns.destroy_spatial_snapshot)(snapshot) };

        entities
    }

    /// The data of a QR code: its text, or its bytes in hexadecimal.
    fn data(&self, snapshot: sys::SpatialSnapshotEXT, buffer: sys::SpatialBufferEXT) -> String {
        let info = sys::SpatialBufferGetInfoEXT {
            ty: sys::SpatialBufferGetInfoEXT::TYPE,
            next: ptr::null(),
            buffer_id: buffer.buffer_id,
        };

        let payload = match buffer.buffer_type {
            sys::SpatialBufferTypeEXT::STRING => {
                let text = read_buffer(self.fns.get_spatial_buffer_string, snapshot, &info);
                Payload::Text(
                    unsafe { CStr::from_ptr(text.as_ptr()) }
                        .to_string_lossy()
                        .into(),
                )
            }
            sys::SpatialBufferTypeEXT::UINT8 => Payload::Bytes(read_buffer(
                self.fns.get_spatial_buffer_uint8,
                snapshot,
                &info,
            )),
            other => return format!("(not decoded, {other:?})"),
        };

        payload.to_string()
    }
}

/// A spatial buffer, read with the two-call idiom.
fn read_buffer<T: Copy + Default>(
    get: unsafe extern "system" fn(
        sys::SpatialSnapshotEXT,
        *const sys::SpatialBufferGetInfoEXT,
        u32,
        *mut u32,
        *mut T,
    ) -> sys::Result,
    snapshot: sys::SpatialSnapshotEXT,
    info: &sys::SpatialBufferGetInfoEXT,
) -> Vec<T> {
    let mut count = 0;
    check(
        unsafe { get(snapshot, info, 0, &mut count, ptr::null_mut()) },
        "xrGetSpatialBuffer*EXT",
    );
    let mut buffer = vec![T::default(); count as usize];
    check(
        unsafe { get(snapshot, info, count, &mut count, buffer.as_mut_ptr()) },
        "xrGetSpatialBuffer*EXT",
    );

    buffer
}

fn format_pose(pose: sys::Posef) -> String {
    let (p, o) = (pose.position, pose.orientation);
    format!(
        "position ({:.3}, {:.3}, {:.3}) orientation ({:.3}, {:.3}, {:.3}, {:.3})",
        p.x, p.y, p.z, o.x, o.y, o.z, o.w
    )
}

fn main() {
    let mut args = std::env::args().skip(1);
    let seconds = args.next().map_or(10, |arg| arg.parse().expect("seconds"));
    let loader = args
        .next()
        .map_or_else(|| PathBuf::from(DEFAULT_LOADER), PathBuf::from);

    let entry = unsafe { xr::Entry::load_from(&loader) }.expect("cannot load the OpenXR loader");
    for layer in entry.enumerate_layers().unwrap() {
        println!("API layer: {} ({})", layer.layer_name, layer.description);
    }
    let available = entry.enumerate_extensions().unwrap();
    println!(
        "Extensions: XR_EXT_spatial_entity {}, XR_EXT_spatial_marker_tracking {}, XR_EXT_future {}",
        available.ext_spatial_entity, available.ext_spatial_marker_tracking, available.ext_future
    );

    let mut extensions = xr::ExtensionSet::default();
    extensions.khr_d3d11_enable = true;
    extensions.ext_future = true;
    extensions.ext_spatial_entity = true;
    extensions.ext_spatial_marker_tracking = true;
    let instance = entry
        .create_instance(
            &xr::ApplicationInfo {
                application_name: "ALVR marker probe",
                ..Default::default()
            },
            &extensions,
            &[],
        )
        .expect("cannot create the instance");
    let properties = instance.properties().unwrap();
    println!(
        "Runtime: {} {}",
        properties.runtime_name, properties.runtime_version
    );

    let system = instance
        .system(xr::FormFactor::HEAD_MOUNTED_DISPLAY)
        .expect("no headset");
    let spatial = Spatial {
        fns: instance.exts().ext_spatial_entity.unwrap(),
        future_fns: instance.exts().ext_future.unwrap(),
        instance: instance.as_raw(),
    };

    let mut capabilities = [sys::SpatialCapabilityEXT::from_raw(0); 8];
    let mut count = 0;
    check(
        unsafe {
            (spatial.fns.enumerate_spatial_capabilities)(
                instance.as_raw(),
                system,
                capabilities.len() as u32,
                &mut count,
                capabilities.as_mut_ptr(),
            )
        },
        "xrEnumerateSpatialCapabilitiesEXT",
    );
    println!(
        "Spatial capabilities: {:?}",
        &capabilities[..count as usize]
    );
    if count == 0 {
        println!("No QR codes: is the ALVR driver running?");
        return;
    }

    instance.graphics_requirements::<xr::D3D11>(system).unwrap();
    let mut device: Option<ID3D11Device> = None;
    unsafe {
        D3D11CreateDevice(
            None,
            D3D_DRIVER_TYPE_HARDWARE,
            Default::default(),
            D3D11_CREATE_DEVICE_BGRA_SUPPORT,
            None,
            D3D11_SDK_VERSION,
            Some(&mut device),
            None,
            None,
        )
    }
    .expect("cannot create a D3D11 device");
    let device = device.unwrap();

    let (session, mut frame_waiter, mut frame_stream) = unsafe {
        instance.create_session::<xr::D3D11>(
            system,
            &xr::d3d::SessionCreateInfoD3D11 {
                device: device.as_raw().cast(),
            },
        )
    }
    .expect("cannot create the session");
    let view = session
        .create_reference_space(xr::ReferenceSpaceType::VIEW, xr::Posef::IDENTITY)
        .unwrap();
    let stage = session
        .create_reference_space(xr::ReferenceSpaceType::STAGE, xr::Posef::IDENTITY)
        .unwrap();

    let deadline = Instant::now() + Duration::from_secs(seconds);
    let mut running = false;
    let mut context = None;
    let mut last_discovery: Option<Instant> = None;
    let mut recommendations = 0;
    // Every frame's STAGE poses of each still tracked code, for how much they jitter
    let mut poses = std::collections::BTreeMap::<String, Vec<sys::Posef>>::new();

    while Instant::now() < deadline {
        loop {
            let mut event = sys::EventDataBuffer {
                ty: sys::StructureType::EVENT_DATA_BUFFER,
                next: ptr::null(),
                varying: [0; 4000],
            };
            let result = unsafe { (instance.fp().poll_event)(instance.as_raw(), &mut event) };
            if result != sys::Result::SUCCESS {
                break;
            }

            if event.ty == sys::StructureType::EVENT_DATA_SESSION_STATE_CHANGED {
                let changed =
                    unsafe { &*(&raw const event).cast::<sys::EventDataSessionStateChanged>() };
                println!("Session state {:?}", changed.state);
                match changed.state {
                    sys::SessionState::READY => {
                        session
                            .begin(xr::ViewConfigurationType::PRIMARY_STEREO)
                            .unwrap();
                        running = true;
                    }
                    sys::SessionState::STOPPING => {
                        session.end().unwrap();
                        running = false;
                    }
                    sys::SessionState::EXITING | sys::SessionState::LOSS_PENDING => return,
                    _ => (),
                }
            } else if event.ty == sys::EventDataSpatialDiscoveryRecommendedEXT::TYPE {
                recommendations += 1;
                println!("Discovery recommended ({recommendations})");
                last_discovery = None;
            } else if event.ty == sys::EventDataReferenceSpaceChangePending::TYPE {
                let change = unsafe {
                    &*(&raw const event).cast::<sys::EventDataReferenceSpaceChangePending>()
                };
                println!(
                    "{:?} space changes at {} ns: {}",
                    change.reference_space_type,
                    change.change_time.as_nanos(),
                    if change.pose_valid.into() {
                        format!(
                            "new origin in the previous one: {}",
                            format_pose(change.pose_in_previous_space)
                        )
                    } else {
                        "where to is not known".to_owned()
                    }
                );
                last_discovery = None;
            }
        }

        if !running {
            std::thread::sleep(Duration::from_millis(10));
            continue;
        }

        let frame = frame_waiter.wait().unwrap();
        frame_stream.begin().unwrap();
        let time = frame.predicted_display_time;

        let context = *context.get_or_insert_with(|| spatial.create_context(session.as_raw()));
        if last_discovery.is_none_or(|last| last.elapsed() >= DISCOVERY_INTERVAL) {
            last_discovery = Some(Instant::now());

            let head = view.locate(&stage, time).unwrap();
            println!("Head in STAGE: {}", format_pose(head.pose));
            let in_view = spatial.discover(context, view.as_raw(), time);
            let in_stage = spatial.discover(context, stage.as_raw(), time);
            println!("{} QR code(s)", in_view.len());
            for ((text, state, bounds), (_, _, stage_bounds)) in in_view.iter().zip(&in_stage) {
                println!(
                    "  {text} {state:?}, {:.3} x {:.3} m",
                    bounds.extents.width, bounds.extents.height
                );
                println!("    in VIEW:  {}", format_pose(bounds.center));
                println!("    in STAGE: {}", format_pose(stage_bounds.center));
            }
        }

        for (text, state, bounds) in spatial.discover(context, stage.as_raw(), time) {
            if state == sys::SpatialEntityTrackingStateEXT::TRACKING {
                poses.entry(text).or_default().push(bounds.center);
            }
        }

        frame_stream
            .end(time, xr::EnvironmentBlendMode::OPAQUE, &[])
            .unwrap();
    }

    println!("Done; {recommendations} discovery recommendation(s)");
    println!("Jitter, as the frames saw each code in STAGE (meaningful for still codes):");
    for (text, poses) in &poses {
        let (position_mm, angle_degrees) = jitter(poses);
        println!(
            "  {text}: {} frames, position {position_mm:.2} mm, angle {angle_degrees:.2}° (standard deviations)",
            poses.len()
        );
    }
}

/// The standard deviations of the positions, and of the angles from the mean orientation.
fn jitter(poses: &[sys::Posef]) -> (f32, f32) {
    let count = poses.len() as f32;
    let mean = |f: &dyn Fn(&sys::Posef) -> f32| poses.iter().map(f).sum::<f32>() / count;
    let (x, y, z) = (
        mean(&|p| p.position.x),
        mean(&|p| p.position.y),
        mean(&|p| p.position.z),
    );
    let position = mean(&|p| {
        (p.position.x - x).powi(2) + (p.position.y - y).powi(2) + (p.position.z - z).powi(2)
    })
    .sqrt();

    // The mean orientation, all in one hemisphere
    let first = poses[0].orientation;
    let sign = |q: sys::Quaternionf| {
        if q.x * first.x + q.y * first.y + q.z * first.z + q.w * first.w < 0.0 {
            -1.0
        } else {
            1.0
        }
    };
    let mut q = [
        mean(&|p| sign(p.orientation) * p.orientation.x),
        mean(&|p| sign(p.orientation) * p.orientation.y),
        mean(&|p| sign(p.orientation) * p.orientation.z),
        mean(&|p| sign(p.orientation) * p.orientation.w),
    ];
    let length = q.iter().map(|v| v * v).sum::<f32>().sqrt();
    q = q.map(|v| v / length);
    let angle = mean(&|p| {
        let o = p.orientation;
        let cos = (o.x * q[0] + o.y * q[1] + o.z * q[2] + o.w * q[3])
            .abs()
            .min(1.0);
        (2.0 * cos.acos()).powi(2)
    })
    .sqrt();

    (position * 1000.0, angle.to_degrees())
}
