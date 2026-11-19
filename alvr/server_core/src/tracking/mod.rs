mod body;
mod face;
mod vmc;

pub use body::*;
pub use face::*;
pub use vmc::*;

use crate::{
    ConnectionContext, MarkerStream, SESSION_MANAGER, ServerCoreEvent,
    connection::STREAMING_RECV_TIMEOUT,
    hand_gestures::{self, HAND_GESTURE_BUTTON_SET, HandGestureManager},
    input_mapping::ButtonMappingManager,
};
use alvr_common::{
    ConnectionError, DEVICE_ID_TO_PATH, DeviceMotion, Pose, ViewParams,
    glam::{Quat, Vec2, Vec3},
    inputs as inp,
};
use alvr_events::{EventType, TrackingEvent};
use alvr_packets::{MarkerTrackingConfig, TrackingData};
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

// todo: Move this struct to Settings and use it for every tracked device
#[derive(Default)]
struct MotionConfig {
    // Position offset applied after rotation offset
    pose_offset: Pose,
    linear_velocity_cutoff: f32,
    angular_velocity_cutoff: f32,
}

/// A change of the space poses are submitted to SteamVR in. Applications see their reference
/// spaces move: SteamVR's do not change, the poses in them do.
#[derive(Clone, Copy, Debug, Default)]
pub struct OriginChange {
    /// Changes so far.
    pub count: u64,
    /// The new origin in the space before the latest change, when known.
    pub pose_in_previous: Option<Pose>,
    /// False when only the bounds of STAGE changed.
    pub moved: bool,
}

pub struct TrackingManager {
    last_head_pose: Pose,             // client's reference space
    inverse_recentering_origin: Pose, // client's reference space
    origin_change: OriginChange,
    /// Of the client's STAGE, as given to SteamVR.
    stage_area: Option<Vec2>,
    device_motions_history: HashMap<u64, VecDeque<(Duration, DeviceMotion)>>,
    hand_skeletons_history: [VecDeque<(Duration, [Pose; 26])>; 2],
    max_history_size: usize,
    marker_streams: Vec<MarkerStream>, // client's reference space, one per tracker
}

impl TrackingManager {
    pub fn new(max_history_size: usize) -> TrackingManager {
        TrackingManager {
            last_head_pose: Pose::IDENTITY,
            inverse_recentering_origin: Pose::IDENTITY,
            origin_change: OriginChange::default(),
            stage_area: None,
            device_motions_history: HashMap::new(),
            hand_skeletons_history: [VecDeque::new(), VecDeque::new()],
            max_history_size,
            marker_streams: Vec::new(),
        }
    }

    /// Replaces the markers of the stream's tracker.
    pub fn set_marker_stream(&mut self, stream: MarkerStream) {
        match self
            .marker_streams
            .iter_mut()
            .find(|existing| existing.config == stream.config)
        {
            Some(existing) => *existing = stream,
            None => self.marker_streams.push(stream),
        }
    }

    /// Forgets the trackers the headset no longer runs.
    pub fn retain_marker_streams(&mut self, configs: &[MarkerTrackingConfig]) {
        self.marker_streams
            .retain(|stream| configs.contains(&stream.config));
    }

    pub fn recentered_marker_streams(&self) -> Vec<MarkerStream> {
        self.marker_streams
            .iter()
            .map(|stream| MarkerStream {
                markers: stream
                    .markers
                    .iter()
                    .map(|marker| alvr_packets::MarkerData {
                        pose: self.recenter_pose(marker.pose),
                        ..marker.clone()
                    })
                    .collect(),
                ..stream.clone()
            })
            .collect()
    }

    /// `client_change` is where the client's new origin is in its previous one, if known.
    fn change_origin(
        &mut self,
        inverse_origin: Pose,
        client_change: Option<Pose>,
        bounds_changed: bool,
    ) {
        let previous = self.inverse_recentering_origin;
        self.inverse_recentering_origin = inverse_origin;

        // A point at c in the client's new space is at client_change * c in its previous one, so
        // at previous * client_change * c then, and at inverse_origin * c now
        let pose_in_previous =
            client_change.map(|change| previous * change * inverse_origin.inverse());
        let moved = pose_in_previous.is_none_or(|pose| {
            pose.position.length() >= 1e-4 || pose.orientation.angle_between(Quat::IDENTITY) >= 1e-4
        });
        // Nothing changed: not a change for applications
        if !moved && !bounds_changed {
            return;
        }

        self.origin_change = OriginChange {
            count: self.origin_change.count + 1,
            pose_in_previous,
            moved,
        };
    }

    /// Takes the size of the client's STAGE. Returns whether it changed.
    pub fn set_stage_area(&mut self, area: Vec2) -> bool {
        self.stage_area.replace(area) != Some(area)
    }

    pub fn origin_change(&self) -> OriginChange {
        self.origin_change
    }

    /// For a new connection: its origin is a change from the last one, to somewhere not known.
    pub fn after_origin_changes(mut self, count: u64) -> Self {
        self.origin_change = OriginChange {
            count: count + 1,
            pose_in_previous: None,
            moved: true,
        };
        self
    }

    /// Moves the origin as `recentering_mode` puts it. `client_change` is where the client's new
    /// origin is in its previous one: the identity when its space stayed, `None` when not known.
    pub fn recenter(
        &mut self,
        recentering_mode: &RecenteringMode,
        client_change: Option<Pose>,
        bounds_changed: bool,
    ) {
        // The last head pose is still in the client's previous space: where it is in the new one,
        // when known, for the modes that recenter on the head
        if let Some(change) = client_change {
            self.last_head_pose = change.inverse() * self.last_head_pose;
        }

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
        };

        self.change_origin(
            Pose {
                position,
                orientation,
            }
            .inverse(),
            client_change,
            bounds_changed,
        );
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

    const KEPT: Option<Pose> = Some(Pose::IDENTITY);

    fn point() -> Pose {
        Pose {
            position: Vec3::new(0.3, 1.7, -0.4),
            orientation: Quat::from_rotation_y(0.3),
        }
    }

    /// The change says where the new origin is: anything fixed in the room, at `before` in the
    /// previous space, is that pose composed with where it is in the new one.
    fn assert_change(manager: &TrackingManager, before: Pose, after: Pose) {
        let in_previous = manager.origin_change().pose_in_previous.unwrap() * after;
        assert!(in_previous.position.distance(before.position) < 1e-5);
        assert!(in_previous.orientation.angle_between(before.orientation) < 1e-5);
    }

    #[test]
    fn recentering_says_where_the_new_origin_is() {
        let mut manager = TrackingManager::new(10).after_origin_changes(4);
        assert_eq!(manager.origin_change().count, 5);
        assert!(manager.origin_change().pose_in_previous.is_none());

        // Recentered on the head, which moved and turned; the client's space stays
        manager.last_head_pose = Pose {
            position: Vec3::new(1.0, 1.6, -2.0),
            orientation: Quat::from_rotation_y(0.5),
        };
        let before = manager.recenter_pose(point());
        manager.recenter(&RecenteringMode::LocalFloor, KEPT, false);
        assert_eq!(manager.origin_change().count, 6);
        assert!(manager.origin_change().moved);
        assert_change(&manager, before, manager.recenter_pose(point()));

        // The same origin again is no change
        manager.recenter(&RecenteringMode::LocalFloor, KEPT, false);
        assert_eq!(manager.origin_change().count, 6);

        // Another mode, in the same client space
        let before = manager.recenter_pose(point());
        manager.recenter(&RecenteringMode::Stage, KEPT, false);
        assert_change(&manager, before, manager.recenter_pose(point()));
        assert_eq!(manager.recenter_pose(point()).position, point().position);
    }

    #[test]
    fn the_client_space_moving_is_followed() {
        let mut manager = TrackingManager::new(10);
        manager.recenter(&RecenteringMode::Stage, KEPT, false);

        // The client's STAGE moves 1 m along X and turns: the same point in the room is somewhere
        // else in the client's new space
        let client_change = Pose {
            position: Vec3::new(1.0, 0.0, 0.0),
            orientation: Quat::from_rotation_y(0.7),
        };
        let before = manager.recenter_pose(point());
        let point_in_new_client_space = client_change.inverse() * point();
        manager.recenter(&RecenteringMode::Stage, Some(client_change), false);
        assert_change(
            &manager,
            before,
            manager.recenter_pose(point_in_new_client_space),
        );

        // The same with recentering on the head, which ALVR does whenever the client's space
        // changes in the other modes
        let head = Pose {
            position: Vec3::new(-0.5, 1.6, 0.2),
            orientation: Quat::from_rotation_y(-0.4),
        };
        manager.last_head_pose = head;
        let before = manager.recenter_pose(point_in_new_client_space);
        let point_in_newer_client_space = client_change.inverse() * point_in_new_client_space;
        manager.recenter(&RecenteringMode::LocalFloor, Some(client_change), false);
        assert_change(
            &manager,
            before,
            manager.recenter_pose(point_in_newer_client_space),
        );
        // The origin is under the head, where it is in the client's new space
        let head_now = manager.recenter_pose(client_change.inverse() * head);
        assert!(head_now.position.x.abs() < 1e-5 && head_now.position.z.abs() < 1e-5);
        assert!((head_now.position.y - 1.6).abs() < 1e-5);

        // Not known how the client's space moved
        manager.recenter(&RecenteringMode::LocalFloor, None, false);
        assert!(manager.origin_change().pose_in_previous.is_none());
        assert!(manager.origin_change().moved);
    }

    #[test]
    fn new_bounds_are_a_change_of_their_own() {
        let mut manager = TrackingManager::new(10);
        assert!(manager.set_stage_area(Vec2::new(2.0, 3.0)));
        assert!(!manager.set_stage_area(Vec2::new(2.0, 3.0)));

        manager.recenter(&RecenteringMode::Stage, KEPT, true);
        let change = manager.origin_change();
        assert_eq!(change.count, 1);
        assert!(!change.moved);
        assert_eq!(
            change.pose_in_previous.map(|pose| pose.position),
            Some(Vec3::ZERO)
        );

        // Neither moved nor resized
        manager.recenter(&RecenteringMode::Stage, KEPT, false);
        assert_eq!(manager.origin_change().count, 1);
    }

    #[test]
    fn marker_streams_follow_the_trackers() {
        let config = MarkerTrackingConfig::default();
        let mut manager = TrackingManager::new(10);
        manager.set_marker_stream(MarkerStream {
            config,
            timestamp: Duration::ZERO,
            markers: Vec::new(),
        });
        assert_eq!(manager.recentered_marker_streams().len(), 1);

        manager.retain_marker_streams(&[]);
        assert!(manager.recentered_marker_streams().is_empty());
    }
}
