mod body;
mod face;
mod vmc;

pub use body::*;
pub use face::*;
pub use vmc::*;

use crate::{
    ConnectionContext, SESSION_MANAGER, ServerCoreEvent,
    connection::STREAMING_RECV_TIMEOUT,
    hand_gestures::{self, HAND_GESTURE_BUTTON_SET, HandGestureManager},
    input_mapping::ButtonMappingManager,
};
use alvr_common::{
    ConnectionError, DEVICE_ID_TO_PATH, DeviceMotion, Pose, ViewParams,
    glam::{Quat, Vec3},
    info, inputs as inp,
};
use alvr_events::{EventType, TrackingEvent};
use alvr_packets::{MarkerData, TrackingData};
use alvr_session::{
    BodyTrackingConfig, HeadsetConfig, RecenteringMode, Settings, VMCConfig,
    settings_schema::Switch,
};
use alvr_sockets::StreamReceiver;
use std::{
    cmp::Ordering,
    collections::{HashMap, VecDeque},
    f32::consts::PI,
    time::Duration,
};

const DEG_TO_RAD: f32 = PI / 180.0;

#[derive(Debug)]
pub enum HandType {
    Left = 0,
    Right = 1,
}

const MARKER_RELOCK_DISTANCE: f32 = 0.03; // m
const MARKER_RELOCK_ANGLE: f32 = 2.0 * PI / 180.0;

// Floor point under the code, yaw only. Forward (-Z) is the code's top edge if it lies flat, or
// into the wall if it hangs on one.
fn floor_origin_from_marker(marker: Pose) -> Option<Pose> {
    let normal = marker.orientation * Vec3::Z;
    let forward = if normal.y.abs() > 0.7 {
        marker.orientation * Vec3::Y
    } else {
        -normal
    };
    let forward = Vec3::new(forward.x, 0.0, forward.z).try_normalize()?;

    Some(Pose {
        position: Vec3::new(marker.position.x, 0.0, marker.position.z),
        orientation: Quat::from_rotation_y(f32::atan2(-forward.x, -forward.z)),
    })
}

// todo: Move this struct to Settings and use it for every tracked device
#[derive(Default)]
struct MotionConfig {
    // Position offset applied after rotation offset
    pose_offset: Pose,
    linear_velocity_cutoff: f32,
    angular_velocity_cutoff: f32,
}

pub struct TrackingManager {
    last_head_pose: Pose,             // client's reference space
    inverse_recentering_origin: Pose, // client's reference space
    device_motions_history: HashMap<u64, VecDeque<(Duration, DeviceMotion)>>,
    hand_skeletons_history: [VecDeque<(Duration, [Pose; 26])>; 2],
    max_history_size: usize,
    markers: Vec<MarkerData>,    // client's reference space
    marker_origin: Option<Pose>, // client's reference space, last origin taken from a QR code
    // With no code text configured: the text of the first code that set the origin
    first_code_text: Option<String>,
}

impl TrackingManager {
    pub fn new(max_history_size: usize) -> TrackingManager {
        TrackingManager {
            last_head_pose: Pose::IDENTITY,
            inverse_recentering_origin: Pose::IDENTITY,
            device_motions_history: HashMap::new(),
            hand_skeletons_history: [VecDeque::new(), VecDeque::new()],
            max_history_size,
            markers: Vec::new(),
            marker_origin: None,
            first_code_text: None,
        }
    }

    pub fn set_markers(&mut self, markers: Vec<MarkerData>) {
        self.markers = markers;
    }

    pub fn recentered_markers(&self) -> Vec<MarkerData> {
        self.markers
            .iter()
            .map(|marker| MarkerData {
                pose: self.recenter_pose(marker.pose),
                ..marker.clone()
            })
            .collect()
    }

    // In Marker mode, moves the origin to the QR code when it moved noticeably (jitter is ignored,
    // a recenter of the client's space is followed). Returns true if the origin changed.
    pub fn update_marker_origin(&mut self, recentering_mode: &RecenteringMode) -> bool {
        let RecenteringMode::Marker { code_text } = recentering_mode else {
            self.first_code_text = None;
            return false;
        };
        if !code_text.is_empty() {
            self.first_code_text = None;
        }
        // Configured text, else the first code used, else any code
        let wanted = (!code_text.is_empty())
            .then_some(code_text)
            .or(self.first_code_text.as_ref());

        let Some((id, origin)) = self
            .markers
            .iter()
            .find(|m| wanted.is_none_or(|text| m.id == *text))
            .and_then(|m| Some((m.id.clone(), floor_origin_from_marker(m.pose)?)))
        else {
            return false;
        };
        if code_text.is_empty() && self.first_code_text.is_none() {
            info!("Using QR code \"{id}\" as origin");
            self.first_code_text = Some(id);
        }

        if let Some(current) = self.marker_origin
            && current.position.distance(origin.position) < MARKER_RELOCK_DISTANCE
            && current.orientation.angle_between(origin.orientation) < MARKER_RELOCK_ANGLE
        {
            return false;
        }

        self.marker_origin = Some(origin);
        self.inverse_recentering_origin = origin.inverse();

        true
    }

    pub fn recenter(&mut self, recentering_mode: &RecenteringMode) {
        let recentering_mode = match recentering_mode {
            RecenteringMode::Marker { .. } => {
                self.update_marker_origin(recentering_mode);
                if let Some(origin) = self.marker_origin {
                    self.inverse_recentering_origin = origin.inverse();
                    return;
                }
                &RecenteringMode::LocalFloor
            }
            mode => mode,
        };

        let position = match recentering_mode {
            RecenteringMode::Stage => Vec3::ZERO,
            RecenteringMode::LocalFloor => {
                let mut pos = self.last_head_pose.position;
                pos.y = 0.0;

                pos
            }
            RecenteringMode::Local { view_height } | RecenteringMode::Tilted { view_height } => {
                self.last_head_pose.position - Vec3::new(0.0, *view_height, 0.0)
            }
            RecenteringMode::Marker { .. } => unreachable!(),
        };

        let orientation = match recentering_mode {
            RecenteringMode::Stage => Quat::IDENTITY,
            RecenteringMode::LocalFloor | RecenteringMode::Local { .. } => {
                let mut rot = self.last_head_pose.orientation;
                // extract yaw rotation
                rot.x = 0.0;
                rot.z = 0.0;
                rot = rot.normalize();

                rot
            }
            RecenteringMode::Tilted { .. } => self.last_head_pose.orientation,
            RecenteringMode::Marker { .. } => unreachable!(),
        };

        self.inverse_recentering_origin = Pose {
            position,
            orientation,
        }
        .inverse();
    }

    pub fn recenter_pose(&self, pose: Pose) -> Pose {
        self.inverse_recentering_origin * pose
    }

    pub fn recenter_motion(&self, motion: DeviceMotion) -> DeviceMotion {
        self.inverse_recentering_origin * motion
    }

    // Performs all kinds of tracking transformations, driven by settings.
    pub fn report_device_motions(
        &mut self,
        headset_config: &HeadsetConfig,
        timestamp: Duration,
        device_motions: &[(u64, DeviceMotion)],
    ) {
        let mut device_motion_configs = HashMap::new();
        device_motion_configs.insert(*inp::HEAD_ID, MotionConfig::default());
        device_motion_configs.extend([
            (*inp::BODY_CHEST_ID, MotionConfig::default()),
            (*inp::BODY_HIPS_ID, MotionConfig::default()),
            (*inp::BODY_LEFT_ELBOW_ID, MotionConfig::default()),
            (*inp::BODY_RIGHT_ELBOW_ID, MotionConfig::default()),
            (*inp::BODY_LEFT_KNEE_ID, MotionConfig::default()),
            (*inp::BODY_LEFT_FOOT_ID, MotionConfig::default()),
            (*inp::BODY_RIGHT_KNEE_ID, MotionConfig::default()),
            (*inp::BODY_RIGHT_FOOT_ID, MotionConfig::default()),
        ]);

        if let Switch::Enabled(controllers) = &headset_config.controllers {
            device_motion_configs.insert(
                *inp::HAND_LEFT_ID,
                MotionConfig {
                    pose_offset: Pose::IDENTITY,
                    linear_velocity_cutoff: controllers.linear_velocity_cutoff,
                    angular_velocity_cutoff: controllers.angular_velocity_cutoff * DEG_TO_RAD,
                },
            );

            device_motion_configs.insert(
                *inp::HAND_RIGHT_ID,
                MotionConfig {
                    pose_offset: Pose::IDENTITY,
                    linear_velocity_cutoff: controllers.linear_velocity_cutoff,
                    angular_velocity_cutoff: controllers.angular_velocity_cutoff * DEG_TO_RAD,
                },
            );
        }

        for &(device_id, mut motion) in device_motions {
            if device_id == *inp::HEAD_ID {
                self.last_head_pose = motion.pose;
            }

            if let Some(config) = device_motion_configs.get(&device_id) {
                motion = self.recenter_motion(motion);

                motion.pose = motion.pose * config.pose_offset;

                fn cutoff(v: Vec3, threshold: f32) -> Vec3 {
                    if v.length_squared() > threshold * threshold {
                        v
                    } else {
                        Vec3::ZERO
                    }
                }

                motion.linear_velocity =
                    cutoff(motion.linear_velocity, config.linear_velocity_cutoff);
                motion.angular_velocity =
                    cutoff(motion.angular_velocity, config.angular_velocity_cutoff);
            }

            if let Some(motions) = self.device_motions_history.get_mut(&device_id) {
                motions.push_front((timestamp, motion));

                if motions.len() > self.max_history_size {
                    motions.pop_back();
                }
            } else {
                self.device_motions_history
                    .insert(device_id, VecDeque::from(vec![(timestamp, motion)]));
            }
        }
    }

    // If the exact sample_timestamp is not found, use the closest one if it's not older. This makes
    // sure that we return None if there is no newer sample and always return Some otherwise.
    pub fn get_device_motion(
        &self,
        device_id: u64,
        sample_timestamp: Duration,
    ) -> Option<DeviceMotion> {
        self.device_motions_history
            .get(&device_id)
            .and_then(|motions| {
                // Get first element to initialize a valid motion reference
                if let Some((_, motion)) = motions.front() {
                    let mut best_timestamp_diff = Duration::MAX;
                    let mut best_motion_ref = motion;

                    // Note: we are iterating from most recent to oldest
                    for (ts, m) in motions {
                        match ts.cmp(&sample_timestamp) {
                            Ordering::Equal => return Some(*m),
                            Ordering::Greater => {
                                let diff = ts.saturating_sub(sample_timestamp);
                                if diff < best_timestamp_diff {
                                    best_timestamp_diff = diff;
                                    best_motion_ref = m;
                                }
                            }
                            Ordering::Less => continue,
                        }
                    }

                    (best_timestamp_diff != Duration::MAX).then_some(*best_motion_ref)
                } else {
                    None
                }
            })
    }

    pub fn report_hand_skeleton(
        &mut self,
        hand_type: HandType,
        timestamp: Duration,
        mut skeleton: [Pose; 26],
    ) {
        for pose in &mut skeleton {
            *pose = self.recenter_pose(*pose);
        }

        let skeleton_history = &mut self.hand_skeletons_history[hand_type as usize];

        skeleton_history.push_back((timestamp, skeleton));

        if skeleton_history.len() > self.max_history_size {
            skeleton_history.pop_front();
        }
    }

    pub fn get_hand_skeleton(
        &self,
        hand_type: HandType,
        sample_timestamp: Duration,
    ) -> Option<&[Pose; 26]> {
        self.hand_skeletons_history[hand_type as usize]
            .iter()
            .find(|(timestamp, _)| *timestamp == sample_timestamp)
            .map(|(_, skeleton)| skeleton)
    }

    pub fn unrecenter_view_params(&self, view_params: &mut [ViewParams; 2]) {
        for params in view_params {
            params.pose = self.inverse_recentering_origin.inverse() * params.pose;
        }
    }
}

pub fn tracking_loop(
    ctx: &ConnectionContext,
    initial_settings: Settings,
    mut tracking_receiver: StreamReceiver<TrackingData>,
    is_streaming: impl Fn() -> bool,
) {
    let mut hand_gesture_manager = HandGestureManager::new();

    let mut gestures_button_mapping_manager =
        initial_settings
            .headset
            .controllers
            .as_option()
            .map(|config| {
                ButtonMappingManager::new_automatic(
                    &HAND_GESTURE_BUTTON_SET,
                    &config.emulation_mode,
                    &config.button_mapping_config,
                )
            });

    let mut face_tracking_sink = initial_settings
        .headset
        .face_tracking
        .into_option()
        .and_then(|config| {
            FaceTrackingSink::new(config.sink, initial_settings.connection.osc_local_port).ok()
        });

    let mut body_tracking_sink = initial_settings
        .headset
        .body_tracking
        .into_option()
        .and_then(|config| {
            BodyTrackingSink::new(config.sink, initial_settings.connection.osc_local_port).ok()
        });

    let mut vmc_sink = initial_settings
        .headset
        .vmc
        .into_option()
        .and_then(|config| VMCSink::new(config).ok());

    while is_streaming() {
        let data = match tracking_receiver.recv(STREAMING_RECV_TIMEOUT) {
            Ok(tracking) => tracking,
            Err(ConnectionError::TryAgain(_)) => continue,
            Err(ConnectionError::Other(_)) => return,
        };
        let Ok(mut tracking) = data.get_header() else {
            return;
        };

        let timestamp = tracking.poll_timestamp;

        if let Some(stats) = &mut *ctx.statistics_manager.write() {
            stats.report_tracking_received(timestamp);
        }

        let controllers_config = {
            let data_lock = SESSION_MANAGER.read();
            data_lock
                .settings()
                .headset
                .controllers
                .clone()
                .into_option()
        };

        let device_motion_keys;
        {
            let mut tracking_manager_lock = ctx.tracking_manager.write();
            let session_manager_lock = SESSION_MANAGER.read();
            let headset_config = &session_manager_lock.settings().headset;

            tracking.device_motions.extend_from_slice(
                &body::get_default_body_trackers_from_detached_controllers(
                    &tracking.device_motions,
                ),
            );
            tracking.device_motions.extend_from_slice(
                &body::get_default_body_trackers_from_motion_trackers_bd(&tracking.device_motions),
            );
            if let Some(skeleton) = &tracking.body {
                tracking
                    .device_motions
                    .extend_from_slice(&body::extract_default_trackers(skeleton));
            }

            device_motion_keys = tracking
                .device_motions
                .iter()
                .map(|(id, _)| *id)
                .collect::<Vec<_>>();

            let velocity_multiplier = session_manager_lock.settings().extra.velocities_multiplier;
            tracking.device_motions.iter_mut().for_each(|(_, motion)| {
                motion.linear_velocity *= velocity_multiplier;
                motion.angular_velocity *= velocity_multiplier;
            });

            tracking_manager_lock.report_device_motions(
                headset_config,
                timestamp,
                &tracking.device_motions,
            );

            if let Some(skeleton) = tracking.hand_skeletons[0] {
                tracking_manager_lock.report_hand_skeleton(HandType::Left, timestamp, skeleton);
            }
            if let Some(skeleton) = tracking.hand_skeletons[1] {
                tracking_manager_lock.report_hand_skeleton(HandType::Right, timestamp, skeleton);
            }

            if let Some(sink) = &mut face_tracking_sink {
                sink.send_tracking(&tracking.face);
            }

            if session_manager_lock.settings().extra.logging.log_tracking {
                let device_motions = device_motion_keys
                    .iter()
                    .filter_map(move |id| {
                        Some((
                            (*DEVICE_ID_TO_PATH.get(id)?).into(),
                            tracking_manager_lock
                                .get_device_motion(*id, timestamp)
                                .unwrap(),
                        ))
                    })
                    .collect::<Vec<(String, DeviceMotion)>>();

                alvr_events::send_event(EventType::Tracking(Box::new(TrackingEvent {
                    device_motions,
                    hand_skeletons: tracking.hand_skeletons,
                    face: tracking.face,
                })))
            }
        };

        // Handle hand gestures
        if let (Some(gestures_config), Some(gestures_button_mapping_manager)) = (
            controllers_config
                .as_ref()
                .and_then(|c| c.hand_tracking_interaction.as_option()),
            &mut gestures_button_mapping_manager,
        ) {
            if !device_motion_keys.contains(&*inp::HAND_LEFT_ID)
                && let Some(hand_skeleton) = tracking.hand_skeletons[0]
            {
                ctx.events_sender
                    .send(ServerCoreEvent::Buttons(
                        hand_gestures::trigger_hand_gesture_actions(
                            gestures_button_mapping_manager,
                            *inp::HAND_LEFT_ID,
                            &hand_gesture_manager.get_active_gestures(
                                &hand_skeleton,
                                gestures_config,
                                *inp::HAND_LEFT_ID,
                            ),
                            gestures_config.only_touch,
                        ),
                    ))
                    .ok();
            }
            if !device_motion_keys.contains(&*inp::HAND_RIGHT_ID)
                && let Some(hand_skeleton) = tracking.hand_skeletons[1]
            {
                ctx.events_sender
                    .send(ServerCoreEvent::Buttons(
                        hand_gestures::trigger_hand_gesture_actions(
                            gestures_button_mapping_manager,
                            *inp::HAND_RIGHT_ID,
                            &hand_gesture_manager.get_active_gestures(
                                &hand_skeleton,
                                gestures_config,
                                *inp::HAND_RIGHT_ID,
                            ),
                            gestures_config.only_touch,
                        ),
                    ))
                    .ok();
            }
        }

        ctx.events_sender
            .send(ServerCoreEvent::Tracking {
                poll_timestamp: tracking.poll_timestamp,
            })
            .ok();

        let publish_vmc = matches!(
            SESSION_MANAGER.read().settings().headset.vmc,
            Switch::Enabled(VMCConfig { publish: true, .. })
        );
        if publish_vmc {
            let orientation_correction = matches!(
                SESSION_MANAGER.read().settings().headset.vmc,
                Switch::Enabled(VMCConfig {
                    orientation_correction: true,
                    ..
                })
            );

            if let Some(sink) = &mut vmc_sink {
                let tracking_manager_lock = ctx.tracking_manager.read();
                let device_motions = device_motion_keys
                    .iter()
                    .map(move |id| {
                        (
                            *id,
                            tracking_manager_lock
                                .get_device_motion(*id, timestamp)
                                .unwrap(),
                        )
                    })
                    .collect::<Vec<(u64, DeviceMotion)>>();

                if let Some(skeleton) = tracking.hand_skeletons[0] {
                    sink.send_hand_tracking(HandType::Left, &skeleton, orientation_correction);
                }
                if let Some(skeleton) = tracking.hand_skeletons[1] {
                    sink.send_hand_tracking(HandType::Right, &skeleton, orientation_correction);
                }
                sink.send_tracking(&device_motions, orientation_correction);
            }
        }

        let track_body = matches!(
            SESSION_MANAGER.read().settings().headset.body_tracking,
            Switch::Enabled(BodyTrackingConfig { tracked: true, .. })
        );
        if track_body && let Some(sink) = &mut body_tracking_sink {
            let tracking_manager_lock = ctx.tracking_manager.read();
            let device_motions = device_motion_keys
                .iter()
                .map(move |id| {
                    (
                        *id,
                        tracking_manager_lock
                            .get_device_motion(*id, timestamp)
                            .unwrap(),
                    )
                })
                .collect::<Vec<_>>();
            sink.send_tracking(&device_motions);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_origin(marker: Pose, position: Vec3, forward: Vec3) {
        let origin = floor_origin_from_marker(marker).unwrap();
        assert!(origin.position.distance(position) < 1e-5);
        assert!((origin.orientation * -Vec3::Z).distance(forward) < 1e-5);
    }

    #[test]
    fn marker_origin() {
        // Flat on the floor, face up, top edge towards +X
        let flat = Pose {
            position: Vec3::new(1.0, 0.01, 2.0),
            orientation: Quat::from_rotation_y(-PI / 2.0) * Quat::from_rotation_x(-PI / 2.0),
        };
        assert_origin(flat, Vec3::new(1.0, 0.0, 2.0), Vec3::X);

        // On a wall at z = -3, facing the room (+Z)
        let wall = Pose {
            position: Vec3::new(0.5, 1.4, -3.0),
            orientation: Quat::IDENTITY,
        };
        assert_origin(wall, Vec3::new(0.5, 0.0, -3.0), -Vec3::Z);
    }

    fn code(id: &str, x: f32) -> MarkerData {
        MarkerData {
            id: id.into(),
            pose: Pose {
                position: Vec3::new(x, 1.0, -2.0),
                orientation: Quat::IDENTITY,
            },
            size: 0.1,
        }
    }

    #[test]
    fn first_code_keeps_origin() {
        let mode = RecenteringMode::Marker {
            code_text: String::new(),
        };
        let mut manager = TrackingManager::new(10);

        manager.set_markers(vec![code("dog", 0.0)]);
        assert!(manager.update_marker_origin(&mode));

        // A different code, alone or next to the first one, is ignored
        manager.set_markers(vec![code("cat", 1.0)]);
        assert!(!manager.update_marker_origin(&mode));
        manager.set_markers(vec![code("cat", 1.0), code("dog", 0.0)]);
        assert!(!manager.update_marker_origin(&mode));

        // The first code moving still moves the origin
        manager.set_markers(vec![code("dog", 0.5)]);
        assert!(manager.update_marker_origin(&mode));
    }
}
