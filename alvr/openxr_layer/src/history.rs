//! The marker poses the driver sent, per tracker of the headset, to answer for any time.
//!
//! Detections jitter by millimetres and fractions of a degree, and now and then one is off by
//! centimetres. Applications ask for the display time, tens of milliseconds past the latest
//! detection, so whatever is predicted from the noise is magnified by that much.
//!
//! - **Moving markers** go through a One Euro filter, on position and on orientation: smoothing
//!   strongly while they stand still and little while they move fast, so a still code holds still
//!   and a moving one is followed closely. The poses are interpolated between the filtered ones
//!   around the requested time, and predicted past the latest one with the filtered velocity, which
//!   fades out near standstill so that no noise is extrapolated.
//! - **Static markers** (the application asked for XrSpatialMarkerStaticOptimizationEXT) are the
//!   average of their poses over the latest second, without the ones far from the rest.
//! - A detection far from where the marker was predicted to be is held back: it is taken only when
//!   the next one confirms it, so a single wild detection is ignored and a real jump is followed one
//!   detection late.

use crate::math;
#[cfg(test)]
use alvr_marker_ipc::marker_id;
use alvr_marker_ipc::{Payload, Pose, Snapshot, TrackingConfig};
use openxr_sys::Posef;
use std::{
    collections::{HashMap, VecDeque},
    f32::consts::PI,
};

const SECOND_NS: i64 = 1_000_000_000;
/// How far back poses are kept.
const HISTORY_NS: i64 = SECOND_NS;
/// A tracker that sent nothing for this long sees nothing any more, as the headset sends what it
/// sees every frame.
const STALE_AFTER_NS: i64 = SECOND_NS / 2;
/// Moving markers are predicted at most this far past their latest pose.
const MAX_EXTRAPOLATION_NS: i64 = SECOND_NS / 10;
/// Static markers are the average of their poses over this window.
const STATIC_WINDOW_NS: i64 = SECOND_NS;

/// One Euro filter of positions: cutoff frequency at standstill, its increase per m/s, and the
/// cutoff of the velocity.
const POSITION_MIN_CUTOFF_HZ: f32 = 1.0;
const POSITION_BETA: f32 = 10.0;
/// The same for orientations, per rad/s.
const ORIENTATION_MIN_CUTOFF_HZ: f32 = 1.0;
const ORIENTATION_BETA: f32 = 2.0;
/// Velocities are the least squares slope of the detections this far back.
const VELOCITY_WINDOW_NS: i64 = 200_000_000;
/// Prediction fades in between these speeds, below which the velocity is mostly noise.
const PREDICTED_SPEED: [f32; 2] = [0.02, 0.06]; // m/s
const PREDICTED_ANGULAR_SPEED: [f32; 2] = [0.1, 0.3]; // rad/s
/// A detection this far from the prediction is an outlier unless the next one agrees.
const OUTLIER_DISTANCE: f32 = 0.02;
const OUTLIER_ANGLE: f32 = 8.0 * PI / 180.0;
/// Static markers leave out poses further from the median than this many median deviations, or
/// these, whichever is more.
const STATIC_OUTLIER_DEVIATIONS: f32 = 3.0;
const STATIC_MIN_OUTLIER_DISTANCE: f32 = 0.003;
const STATIC_MIN_OUTLIER_ANGLE: f32 = 1.0 * PI / 180.0;

type Vec3 = [f32; 3];
type Quat = [f32; 4];

#[derive(Clone, Copy, Debug)]
struct Sample {
    time_ns: i64,
    /// As detected.
    pose: Pose,
    size: f32,
    /// After the One Euro filter.
    filtered: Pose,
    /// Smoothed, in m/s and rad/s, in the space of the poses.
    velocity: Vec3,
    angular_velocity: Vec3,
    /// How far behind the filtered pose trails a steady motion: the time constants of the filters.
    lag: [f32; 2],
    /// The filtered pose moved on by the lag: where the marker is now.
    estimate: Pose,
}

#[derive(Default)]
struct MarkerSamples {
    samples: VecDeque<Sample>,
    /// A detection far from the prediction, waiting for the next one to confirm it.
    held: Option<(i64, Pose, f32)>,
}

struct TrackerHistory {
    config: TrackingConfig,
    latest_time_ns: i64,
    /// The markers of the latest update: id and text.
    seen: Vec<(u64, Payload)>,
    markers: HashMap<u64, MarkerSamples>,
}

/// When to locate a marker.
#[derive(Clone, Copy, Debug)]
pub enum At {
    /// The average of its latest poses, for a static marker.
    Static,
    /// At this time, on the performance counter clock, for a moving marker.
    Time(i64),
    /// Its latest pose, when the time cannot be converted.
    Latest,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Located {
    pub pose: Pose,
    pub size: f32,
}

#[derive(Default)]
pub struct MarkerHistory {
    trackers: Vec<TrackerHistory>,
}

impl MarkerHistory {
    pub fn clear(&mut self) {
        self.trackers.clear();
    }

    /// The space the poses are in moved: the new origin is at `new_in_previous` in the old space.
    /// The poses so far are moved into the new space, or forgotten if where it is is not known.
    pub fn move_origin(&mut self, new_in_previous: Option<Posef>) {
        let Some(new_in_previous) = new_in_previous else {
            self.clear();
            return;
        };

        let previous_in_new = math::inverse(new_in_previous);
        let pose = |pose: Pose| math::to_ipc(math::compose(previous_in_new, math::from_ipc(pose)));
        let rotation = previous_in_new.orientation;
        let vector = |v: Vec3| {
            let o = rotation;
            rotate([o.x, o.y, o.z, o.w], v)
        };
        for marker in self
            .trackers
            .iter_mut()
            .flat_map(|tracker| tracker.markers.values_mut())
        {
            for sample in &mut marker.samples {
                sample.pose = pose(sample.pose);
                sample.filtered = pose(sample.filtered);
                sample.estimate = pose(sample.estimate);
                sample.velocity = vector(sample.velocity);
                sample.angular_velocity = vector(sample.angular_velocity);
            }
            if let Some((_, held, _)) = &mut marker.held {
                *held = pose(*held);
            }
        }
    }

    /// Takes in a snapshot from the driver. Snapshots repeat the trackers that did not change.
    pub fn add(&mut self, snapshot: &Snapshot) {
        // The trackers the headset stopped
        self.trackers.retain(|tracker| {
            snapshot
                .streams
                .iter()
                .any(|stream| stream.config == tracker.config)
        });

        for stream in &snapshot.streams {
            let index = match self
                .trackers
                .iter()
                .position(|tracker| tracker.config == stream.config)
            {
                Some(index) => index,
                None => {
                    self.trackers.push(TrackerHistory {
                        config: stream.config,
                        latest_time_ns: i64::MIN,
                        seen: Vec::new(),
                        markers: HashMap::new(),
                    });
                    self.trackers.len() - 1
                }
            };
            let tracker = &mut self.trackers[index];
            if stream.time_ns <= tracker.latest_time_ns {
                continue;
            }
            tracker.latest_time_ns = stream.time_ns;
            tracker.seen = stream
                .markers
                .iter()
                .map(|marker| (marker.id, marker.payload.clone()))
                .collect();

            for marker in &stream.markers {
                tracker.markers.entry(marker.id).or_default().add(
                    stream.time_ns,
                    marker.pose,
                    marker.size,
                );
            }
            let oldest = stream.time_ns - HISTORY_NS;
            tracker.markers.retain(|_, marker| {
                let samples = &mut marker.samples;
                // Two at least, to interpolate between
                while samples.len() > 2 && samples[0].time_ns < oldest {
                    samples.pop_front();
                }
                samples
                    .back()
                    .is_some_and(|sample| sample.time_ns >= oldest)
            });
        }
    }

    fn tracker(&self, config: TrackingConfig, now_ns: i64) -> Option<&TrackerHistory> {
        self.trackers.iter().find(|tracker| {
            tracker.config == config
                && now_ns.saturating_sub(tracker.latest_time_ns) < STALE_AFTER_NS
        })
    }

    /// The markers the tracker of `config` currently sees: id and text.
    pub fn seen(&self, config: TrackingConfig, now_ns: i64) -> Vec<(u64, Payload)> {
        self.tracker(config, now_ns)
            .map(|tracker| tracker.seen.clone())
            .unwrap_or_default()
    }

    /// Where the marker `id` of the tracker of `config` is, if the tracker currently sees it.
    pub fn locate(&self, config: TrackingConfig, id: u64, at: At, now_ns: i64) -> Option<Located> {
        let tracker = self.tracker(config, now_ns)?;
        if !tracker.seen.iter().any(|(seen, _)| *seen == id) {
            return None;
        }
        let samples = &tracker.markers.get(&id)?.samples;
        let latest = samples.back()?;

        Some(match at {
            At::Static => robust_average(
                samples
                    .iter()
                    .filter(|sample| sample.time_ns >= latest.time_ns - STATIC_WINDOW_NS),
            ),
            At::Time(time_ns) => interpolate(samples, time_ns),
            At::Latest => Located {
                pose: latest.estimate,
                size: latest.size,
            },
        })
    }
}

impl MarkerSamples {
    fn add(&mut self, time_ns: i64, pose: Pose, size: f32) {
        let Some(&previous) = self.samples.back() else {
            self.samples.push_back(Sample {
                time_ns,
                pose,
                size,
                filtered: pose,
                velocity: [0.0; 3],
                angular_velocity: [0.0; 3],
                lag: [0.0; 2],
                estimate: pose,
            });
            return;
        };
        if time_ns <= previous.time_ns {
            return;
        }

        // Far from where it should be: wait for the next detection to agree
        let predicted = extrapolate(&previous, time_ns);
        let far = distance(pose.position, predicted.position) > OUTLIER_DISTANCE
            || angle(pose.orientation, predicted.orientation) > OUTLIER_ANGLE;
        match self.held.take() {
            None if far => {
                self.held = Some((time_ns, pose, size));
                return;
            }
            // Confirmed: a real jump, which the filter would only crawl after. Start again from
            // the two detections.
            Some((held_time, held_pose, _))
                if far
                    && distance(pose.position, held_pose.position) <= OUTLIER_DISTANCE
                    && angle(pose.orientation, held_pose.orientation) <= OUTLIER_ANGLE =>
            {
                let seconds = (time_ns - held_time).max(1) as f32 / 1e9;
                // The detections before were of another place
                self.samples.clear();
                self.samples.push_back(Sample {
                    time_ns,
                    pose,
                    size,
                    filtered: pose,
                    velocity: std::array::from_fn(|i| {
                        (pose.position[i] - held_pose.position[i]) / seconds
                    }),
                    angular_velocity: rotation_vector(quat_mul(
                        pose.orientation,
                        conjugate(held_pose.orientation),
                    ))
                    .map(|v| v / seconds),
                    lag: [0.0; 2],
                    estimate: pose,
                });
                return;
            }
            _ => (),
        }

        let seconds = (time_ns - previous.time_ns) as f32 / 1e9;

        // One Euro filter of the position, in the space of the poses
        // Of the detections: from the filtered poses, they would include the filter's lag
        let (velocity, angular_velocity) = self.velocities(time_ns, pose);
        let position_cutoff = POSITION_MIN_CUTOFF_HZ + POSITION_BETA * length(velocity);
        let position = lerp3(
            previous.filtered.position,
            pose.position,
            smoothing(seconds, position_cutoff),
        );

        // And of the orientation, along the rotation from the filtered one to the detected one
        let orientation_cutoff =
            ORIENTATION_MIN_CUTOFF_HZ + ORIENTATION_BETA * length(angular_velocity);
        let orientation = slerp(
            previous.filtered.orientation,
            pose.orientation,
            smoothing(seconds, orientation_cutoff),
        );

        let mut sample = Sample {
            time_ns,
            pose,
            size,
            filtered: Pose {
                orientation,
                position,
            },
            velocity,
            angular_velocity,
            lag: [
                time_constant(position_cutoff),
                time_constant(orientation_cutoff),
            ],
            estimate: pose,
        };
        sample.estimate = moved_on(&sample, sample.lag);
        self.samples.push_back(sample);
    }
}

fn time_constant(cutoff_hz: f32) -> f32 {
    1.0 / (2.0 * PI * cutoff_hz)
}

impl MarkerSamples {
    /// The velocities of the detections of the latest window, ending with `pose` at `time_ns`:
    /// the least squares slopes of the positions, and of the rotations from `pose`.
    fn velocities(&self, time_ns: i64, pose: Pose) -> (Vec3, Vec3) {
        let window = self
            .samples
            .iter()
            .rev()
            .take_while(|sample| sample.time_ns >= time_ns - VELOCITY_WINDOW_NS)
            .map(|sample| {
                (
                    (sample.time_ns - time_ns) as f32 / 1e9,
                    sample.pose.position,
                    rotation_vector(quat_mul(
                        sample.pose.orientation,
                        conjugate(pose.orientation),
                    )),
                )
            })
            .chain([(0.0, pose.position, [0.0; 3])])
            .collect::<Vec<_>>();
        if window.len() < 2 {
            return ([0.0; 3], [0.0; 3]);
        }

        let count = window.len() as f32;
        let mean_time = window.iter().map(|(t, ..)| t).sum::<f32>() / count;
        let spread = window
            .iter()
            .map(|(t, ..)| (t - mean_time) * (t - mean_time))
            .sum::<f32>();
        if spread <= 0.0 {
            return ([0.0; 3], [0.0; 3]);
        }
        let slope = |value: &dyn Fn(&(f32, Vec3, Vec3)) -> f32| {
            let mean = window.iter().map(value).sum::<f32>() / count;
            window
                .iter()
                .map(|entry| (entry.0 - mean_time) * (value(entry) - mean))
                .sum::<f32>()
                / spread
        };

        (
            std::array::from_fn(|i| slope(&|entry| entry.1[i])),
            std::array::from_fn(|i| slope(&|entry| entry.2[i])),
        )
    }
}

/// The weight of a new value in a low-pass filter of this cutoff, `seconds` after the last one.
fn smoothing(seconds: f32, cutoff_hz: f32) -> f32 {
    1.0 / (1.0 + time_constant(cutoff_hz) / seconds)
}

/// The filtered pose of `sample` moved on with its velocities for these position and orientation
/// times, as far as the velocities are not noise.
fn moved_on(sample: &Sample, [position_seconds, orientation_seconds]: [f32; 2]) -> Pose {
    let speed = fade_in(length(sample.velocity), PREDICTED_SPEED);
    let angular_speed = fade_in(length(sample.angular_velocity), PREDICTED_ANGULAR_SPEED);

    Pose {
        position: std::array::from_fn(|i| {
            sample.filtered.position[i] + sample.velocity[i] * speed * position_seconds
        }),
        orientation: normalize(quat_mul(
            from_rotation_vector(
                sample
                    .angular_velocity
                    .map(|v| v * angular_speed * orientation_seconds),
            ),
            sample.filtered.orientation,
        )),
    }
}

/// Where `sample` predicts the marker to be at `time_ns`.
fn extrapolate(sample: &Sample, time_ns: i64) -> Pose {
    let seconds = (time_ns - sample.time_ns).clamp(0, MAX_EXTRAPOLATION_NS) as f32 / 1e9;
    moved_on(sample, sample.lag.map(|lag| lag + seconds))
}

/// 0 below the range, 1 above it, smoothly in between.
fn fade_in(value: f32, [from, to]: [f32; 2]) -> f32 {
    let t = ((value - from) / (to - from)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// Between the filtered poses around `time_ns`, or predicted past the latest one.
fn interpolate(samples: &VecDeque<Sample>, time_ns: i64) -> Located {
    let latest = samples[samples.len() - 1];
    let (a, b) = match samples.iter().position(|sample| sample.time_ns >= time_ns) {
        Some(0) => (samples[0], samples[0]),
        Some(i) => (samples[i - 1], samples[i]),
        None => {
            return Located {
                pose: extrapolate(&latest, time_ns),
                size: latest.size,
            };
        }
    };
    if b.time_ns <= a.time_ns {
        return Located {
            pose: b.estimate,
            size: b.size,
        };
    }

    let t = ((time_ns - a.time_ns) as f64 / (b.time_ns - a.time_ns) as f64) as f32;
    Located {
        pose: Pose {
            orientation: slerp(a.estimate.orientation, b.estimate.orientation, t),
            position: lerp3(a.estimate.position, b.estimate.position, t),
        },
        size: a.size + (b.size - a.size) * t,
    }
}

/// The mean of the poses, without the ones far from the rest.
fn robust_average<'a>(samples: impl Iterator<Item = &'a Sample>) -> Located {
    let samples = samples.collect::<Vec<_>>();

    // Far from the median position, in median deviations
    let median_position: Vec3 = std::array::from_fn(|i| {
        median(
            samples
                .iter()
                .map(|sample| sample.pose.position[i])
                .collect(),
        )
    });
    let distances = samples
        .iter()
        .map(|sample| distance(sample.pose.position, median_position))
        .collect::<Vec<_>>();
    let distance_limit =
        (STATIC_OUTLIER_DEVIATIONS * median(distances.clone())).max(STATIC_MIN_OUTLIER_DISTANCE);

    // And from the mean orientation of the rest
    let near = samples
        .iter()
        .zip(&distances)
        .filter(|&(_, &distance)| distance <= distance_limit)
        .map(|(sample, _)| *sample)
        .collect::<Vec<_>>();
    let near_orientation = mean_orientation(near.iter().map(|sample| sample.pose.orientation));
    let angles = near
        .iter()
        .map(|sample| angle(sample.pose.orientation, near_orientation))
        .collect::<Vec<_>>();
    let angle_limit =
        (STATIC_OUTLIER_DEVIATIONS * median(angles.clone())).max(STATIC_MIN_OUTLIER_ANGLE);
    let kept = near
        .iter()
        .zip(&angles)
        .filter(|&(_, &angle)| angle <= angle_limit)
        .map(|(sample, _)| *sample)
        .collect::<Vec<_>>();
    // Never all of them
    let kept = if kept.is_empty() { samples } else { kept };

    let count = kept.len() as f32;
    Located {
        pose: Pose {
            orientation: mean_orientation(kept.iter().map(|sample| sample.pose.orientation)),
            position: std::array::from_fn(|i| {
                kept.iter()
                    .map(|sample| sample.pose.position[i])
                    .sum::<f32>()
                    / count
            }),
        },
        size: kept.iter().map(|sample| sample.size).sum::<f32>() / count,
    }
}

fn median(mut values: Vec<f32>) -> f32 {
    if values.is_empty() {
        return 0.0;
    }
    values.sort_by(f32::total_cmp);
    let middle = values.len() / 2;
    if values.len() % 2 == 0 {
        (values[middle - 1] + values[middle]) / 2.0
    } else {
        values[middle]
    }
}

fn mean_orientation(orientations: impl Iterator<Item = Quat>) -> Quat {
    let mut sum = [0.0; 4];
    let mut reference = None;
    for q in orientations {
        // q and -q are the same rotation; add them up in one hemisphere
        let reference = *reference.get_or_insert(q);
        let sign = if dot(q, reference) < 0.0 { -1.0 } else { 1.0 };
        for (sum, value) in sum.iter_mut().zip(q) {
            *sum += sign * value;
        }
    }

    normalize(sum)
}

fn lerp3(a: Vec3, b: Vec3, t: f32) -> Vec3 {
    std::array::from_fn(|i| a[i] + (b[i] - a[i]) * t)
}

fn length(v: Vec3) -> f32 {
    (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
}

fn distance(a: Vec3, b: Vec3) -> f32 {
    length(std::array::from_fn(|i| a[i] - b[i]))
}

fn dot(a: Quat, b: Quat) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]
}

fn normalize(q: Quat) -> Quat {
    let length = dot(q, q).sqrt();
    if length > 0.0 {
        q.map(|value| value / length)
    } else {
        [0.0, 0.0, 0.0, 1.0]
    }
}

/// The angle of the rotation between two orientations, in radians.
fn angle(a: Quat, b: Quat) -> f32 {
    2.0 * dot(a, b).abs().min(1.0).acos()
}

fn conjugate(q: Quat) -> Quat {
    [-q[0], -q[1], -q[2], q[3]]
}

fn quat_mul(a: Quat, b: Quat) -> Quat {
    [
        a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
        a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
        a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
        a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2],
    ]
}

fn rotate(q: Quat, v: Vec3) -> Vec3 {
    let rotated = quat_mul(quat_mul(q, [v[0], v[1], v[2], 0.0]), conjugate(q));
    [rotated[0], rotated[1], rotated[2]]
}

/// Axis times angle, along the shorter way round.
fn rotation_vector(q: Quat) -> Vec3 {
    let q = if q[3] < 0.0 { q.map(|v| -v) } else { q };
    let sin = length([q[0], q[1], q[2]]);
    if sin < 1e-7 {
        return [2.0 * q[0], 2.0 * q[1], 2.0 * q[2]];
    }
    let angle = 2.0 * sin.atan2(q[3]);
    [q[0], q[1], q[2]].map(|v| v / sin * angle)
}

fn from_rotation_vector(v: Vec3) -> Quat {
    let angle = length(v);
    if angle < 1e-7 {
        return normalize([v[0] / 2.0, v[1] / 2.0, v[2] / 2.0, 1.0]);
    }
    let sin = (angle / 2.0).sin() / angle;
    [v[0] * sin, v[1] * sin, v[2] * sin, (angle / 2.0).cos()]
}

/// Spherical interpolation, along the shorter arc.
fn slerp(a: Quat, b: Quat, t: f32) -> Quat {
    let mut cos = dot(a, b);
    let b = if cos < 0.0 {
        cos = -cos;
        b.map(|value| -value)
    } else {
        b
    };

    // Nearly the same rotation: linear is precise enough, and avoids dividing by ~0
    if cos > 0.9995 {
        return normalize(std::array::from_fn(|i| a[i] + (b[i] - a[i]) * t));
    }

    let angle = cos.acos();
    let sin = angle.sin();
    let wa = ((1.0 - t) * angle).sin() / sin;
    let wb = (t * angle).sin() / sin;

    normalize(std::array::from_fn(|i| a[i] * wa + b[i] * wb))
}

#[cfg(test)]
mod tests {
    use super::*;
    use alvr_marker_ipc::{Marker, Stream};

    const MOVING: TrackingConfig = TrackingConfig {
        static_markers: false,
        marker_size: None,
    };
    const MS: i64 = 1_000_000;
    /// When the dog of [`history`] was seen last.
    const END: i64 = 1000 * MS;

    fn yaw(degrees: f32) -> [f32; 4] {
        let half = degrees.to_radians() / 2.0;
        [0.0, half.sin(), 0.0, half.cos()]
    }

    /// Text, x and yaw in degrees of each marker.
    type Seen<'a> = &'a [(&'a str, f32, f32)];

    fn snapshot(streams: &[(i64, Seen)]) -> Snapshot {
        Snapshot {
            streams: streams
                .iter()
                .map(|&(time_ns, markers)| Stream {
                    config: MOVING,
                    time_ns,
                    markers: markers
                        .iter()
                        .map(|&(text, x, degrees)| {
                            Marker::new(
                                text,
                                Pose {
                                    orientation: yaw(degrees),
                                    position: [x, 0.0, 0.0],
                                },
                                0.2,
                            )
                        })
                        .collect(),
                })
                .collect(),
            ..Default::default()
        }
    }

    fn dog() -> u64 {
        alvr_marker_ipc::marker_id("dog")
    }

    fn near(a: f32, b: f32, tolerance: f32) -> bool {
        (a - b).abs() < tolerance
    }

    /// Where the dog of [`history`] is at `time_ns`: running along +X at 0.5 m/s, turning 30°/s.
    fn dog_at(time_ns: i64) -> (f32, f32) {
        let seconds = time_ns as f32 / 1e9;
        (0.5 * seconds, 30.0 * seconds)
    }

    /// The dog, seen every 10 ms for a second, exactly.
    fn history() -> MarkerHistory {
        let mut history = MarkerHistory::default();
        for step in 0..=100 {
            let time = step * 10 * MS;
            let (x, degrees) = dog_at(time);
            history.add(&snapshot(&[(time, &[("dog", x, degrees)])]));
        }
        history
    }

    fn at(history: &MarkerHistory, time_ns: i64) -> Pose {
        history
            .locate(MOVING, dog(), At::Time(time_ns), END)
            .unwrap()
            .pose
    }

    #[test]
    fn moving_markers_are_followed() {
        let history = history();

        // Between two detections, on one, and the latest
        for time in [505 * MS, 700 * MS, END] {
            let (x, degrees) = dog_at(time);
            let pose = at(&history, time);
            assert!(near(pose.position[0], x, 0.002), "{time}: {pose:?}");
            assert!(angle(pose.orientation, yaw(degrees)).to_degrees() < 0.5);
        }
    }

    #[test]
    fn moving_markers_are_predicted_a_little() {
        let history = history();

        // 40 ms past the latest detection
        let (x, degrees) = dog_at(END + 40 * MS);
        let pose = at(&history, END + 40 * MS);
        assert!(near(pose.position[0], x, 0.002));
        assert!(angle(pose.orientation, yaw(degrees)).to_degrees() < 0.5);

        // No further than 100 ms
        let furthest = at(&history, END + MAX_EXTRAPOLATION_NS);
        assert_eq!(at(&history, END + 500 * MS), furthest);
    }

    #[test]
    fn static_markers_are_averaged_without_outliers() {
        let mut history = MarkerHistory::default();
        // Jittering by 2 mm, and once off by 5 cm
        for step in 0..60 {
            let x = if step == 30 {
                0.05
            } else if step % 2 == 0 {
                0.0
            } else {
                0.002
            };
            history.add(&snapshot(&[(step * 16 * MS, &[("dog", x, 10.0)])]));
        }

        let located = history
            .locate(MOVING, dog(), At::Static, 59 * 16 * MS)
            .unwrap();
        assert!(near(located.pose.position[0], 0.001, 0.0001));
        assert!(angle(located.pose.orientation, yaw(10.0)) < 1e-4);
        assert!(near(located.size, 0.2, 1e-6));
    }

    #[test]
    fn single_wild_detections_are_ignored_and_jumps_followed() {
        let mut history = MarkerHistory::default();
        let mut time = 0;
        let mut see = |history: &mut MarkerHistory, x: f32| {
            time += 16 * MS;
            history.add(&snapshot(&[(time, &[("dog", x, 0.0)])]));
            history
                .locate(MOVING, dog(), At::Latest, time)
                .unwrap()
                .pose
                .position[0]
        };
        for _ in 0..30 {
            see(&mut history, 0.0);
        }

        // Once off by 10 cm: ignored
        assert!(near(see(&mut history, 0.1), 0.0, 0.001));
        assert!(near(see(&mut history, 0.0), 0.0, 0.001));

        // Moved for real: followed from the second detection
        assert!(near(see(&mut history, 0.2), 0.0, 0.001));
        assert!(near(see(&mut history, 0.2), 0.2, 0.001));
        assert!(near(see(&mut history, 0.2), 0.2, 0.01));
    }

    #[test]
    fn poses_follow_the_origin() {
        let mut history = history();
        let before = at(&history, END);
        let middle = at(&history, 505 * MS);

        // The new origin is 1 m along +X: everything is 1 m nearer along X
        history.move_origin(Some(openxr_sys::Posef {
            orientation: openxr_sys::Quaternionf::IDENTITY,
            position: openxr_sys::Vector3f {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
        }));
        assert!(near(
            at(&history, END).position[0],
            before.position[0] - 1.0,
            1e-4
        ));
        assert!(near(
            at(&history, 505 * MS).position[0],
            middle.position[0] - 1.0,
            1e-4
        ));

        // Not known where to: the poses are of no use
        history.move_origin(None);
        assert!(history.seen(MOVING, END).is_empty());
    }

    #[test]
    fn markers_out_of_sight_are_not_located() {
        let mut history = history();
        history.add(&snapshot(&[(END + 10 * MS, &[("cat", 1.0, 0.0)])]));
        let now = END + 10 * MS;

        assert_eq!(history.seen(MOVING, now).len(), 1);
        assert!(history.locate(MOVING, dog(), At::Latest, now).is_none());
        assert!(
            history
                .locate(MOVING, alvr_marker_ipc::marker_id("cat"), At::Latest, now)
                .is_some()
        );
    }

    #[test]
    fn a_silent_tracker_sees_nothing() {
        let history = history();
        assert_eq!(history.seen(MOVING, END).len(), 1);
        assert!(history.seen(MOVING, END + STALE_AFTER_NS).is_empty());
        assert!(
            history
                .locate(MOVING, dog(), At::Latest, END + STALE_AFTER_NS)
                .is_none()
        );
    }

    #[test]
    fn repeated_and_stopped_trackers() {
        let mut history = history();
        // The driver repeats a tracker that did not change
        history.add(&snapshot(&[(END, &[])]));
        assert_eq!(history.seen(MOVING, END).len(), 1);

        // The headset stops it
        history.add(&Snapshot::default());
        assert!(history.seen(MOVING, END).is_empty());
    }

    #[test]
    fn slerp_takes_the_shorter_arc() {
        let a = yaw(10.0);
        let b = yaw(30.0).map(|value| -value);
        let middle = slerp(a, b, 0.5);
        assert!(angle(middle, yaw(20.0)) < 1e-4);
    }

    #[test]
    fn rotation_vectors_round_trip() {
        let q = normalize([0.1, 0.7, -0.2, 0.6]);
        let back = from_rotation_vector(rotation_vector(q));
        assert!(angle(q, back) < 1e-4);
    }
}

/// Noisy detections of codes moving in known ways, and how far from the truth an application
/// asking for the display time is told they are.
#[cfg(test)]
mod jitter {
    use super::*;
    use alvr_marker_ipc::{Marker, Stream};

    const MOVING: TrackingConfig = TrackingConfig {
        static_markers: false,
        marker_size: None,
    };
    const MS: i64 = 1_000_000;
    /// The headset's detections.
    const DETECTION_INTERVAL_NS: i64 = 1_000_000_000 / 60;
    const TIMING_JITTER_NS: i64 = 3 * MS;
    const POSITION_NOISE: f32 = 0.002;
    const ANGLE_NOISE: f32 = 0.5;
    const OUTLIER_PROBABILITY: f32 = 0.01;
    const OUTLIER_DISTANCE: f32 = 0.03;
    /// How far ahead of the latest detection the application asks, about where the display time is.
    const AHEAD_NS: i64 = 40 * MS;

    struct Random(u64);

    impl Random {
        fn uniform(&mut self) -> f32 {
            self.0 ^= self.0 << 13;
            self.0 ^= self.0 >> 7;
            self.0 ^= self.0 << 17;
            (self.0 >> 40) as f32 / (1u64 << 24) as f32
        }

        fn gaussian(&mut self) -> f32 {
            let u = self.uniform().max(1e-7);
            let v = self.uniform();
            (-2.0 * u.ln()).sqrt() * (std::f32::consts::TAU * v).cos()
        }
    }

    fn small_rotation(v: [f32; 3]) -> [f32; 4] {
        let angle = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
        if angle < 1e-9 {
            return [0.0, 0.0, 0.0, 1.0];
        }
        let s = (angle / 2.0).sin() / angle;
        [v[0] * s, v[1] * s, v[2] * s, (angle / 2.0).cos()]
    }

    fn angle_between(a: [f32; 4], b: [f32; 4]) -> f32 {
        angle(a, b).to_degrees()
    }

    #[derive(Debug)]
    pub struct Errors {
        pub position_rms_mm: f32,
        pub position_p95_mm: f32,
        pub angle_rms_degrees: f32,
    }

    /// Runs `seconds` of detections of a code whose true pose is `truth(seconds)`, located as for
    /// a moving or a static context.
    pub fn measure(
        seed: u64,
        seconds: f32,
        truth: impl Fn(f32) -> Pose,
        static_context: bool,
    ) -> Errors {
        // Scrambled (splitmix64): xorshift's first numbers from similar seeds are not random
        let mut z = seed.wrapping_add(0x9e37_79b9_7f4a_7c15);
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        let mut random = Random((z ^ (z >> 31)) | 1);
        let mut history = MarkerHistory::default();
        let mut position_errors = Vec::new();
        let mut angle_errors = Vec::new();

        let mut time = 0;
        while (time as f32) < seconds * 1e9 {
            let jitter = ((random.uniform() * 2.0 - 1.0) * TIMING_JITTER_NS as f32) as i64;
            let detected_at = time + jitter;
            let true_pose = truth(detected_at as f32 / 1e9);

            let mut position =
                std::array::from_fn(|i| true_pose.position[i] + random.gaussian() * POSITION_NOISE);
            if random.uniform() < OUTLIER_PROBABILITY {
                position[0] += OUTLIER_DISTANCE;
            }
            let noise = std::array::from_fn(|_| (random.gaussian() * ANGLE_NOISE).to_radians());
            let orientation = quat_mul(true_pose.orientation, small_rotation(noise));

            history.add(&Snapshot {
                streams: vec![Stream {
                    config: MOVING,
                    time_ns: detected_at,
                    markers: vec![Marker::new(
                        "dog",
                        Pose {
                            orientation,
                            position,
                        },
                        0.2,
                    )],
                }],
                ..Default::default()
            });

            // After a settling second
            if time as f32 > 1e9 {
                let asked = detected_at + AHEAD_NS;
                let at = if static_context {
                    At::Static
                } else {
                    At::Time(asked)
                };
                let located = history
                    .locate(MOVING, marker_id("dog"), at, detected_at)
                    .unwrap();
                let truth = truth(asked as f32 / 1e9);
                let error = std::array::from_fn::<f32, 3, _>(|i| {
                    located.pose.position[i] - truth.position[i]
                });
                position_errors
                    .push((error[0] * error[0] + error[1] * error[1] + error[2] * error[2]).sqrt());
                angle_errors.push(angle_between(located.pose.orientation, truth.orientation));
            }

            time += DETECTION_INTERVAL_NS;
        }

        let rms = |values: &[f32]| {
            (values.iter().map(|v| v * v).sum::<f32>() / values.len() as f32).sqrt()
        };
        let mut sorted = position_errors.clone();
        sorted.sort_by(f32::total_cmp);
        Errors {
            position_rms_mm: rms(&position_errors) * 1000.0,
            position_p95_mm: sorted[sorted.len() * 95 / 100] * 1000.0,
            angle_rms_degrees: rms(&angle_errors),
        }
    }

    fn yaw(radians: f32) -> [f32; 4] {
        [0.0, (radians / 2.0).sin(), 0.0, (radians / 2.0).cos()]
    }

    pub fn still(_: f32) -> Pose {
        Pose {
            orientation: yaw(0.3),
            position: [0.5, 1.5, -2.0],
        }
    }

    pub fn walking(t: f32) -> Pose {
        Pose {
            orientation: yaw(0.3),
            position: [0.5 * t, 1.5, -2.0],
        }
    }

    /// The emulator's dog: around a 0.4 m circle every 6 s, turning with it.
    pub fn circling(t: f32) -> Pose {
        let angle = std::f32::consts::TAU * t / 6.0;
        Pose {
            orientation: yaw(angle),
            position: [0.4 * angle.cos(), 1.2, -2.0 - 0.4 * angle.sin()],
        }
    }

    /// Stands for 1.5 s, walks at 0.5 m/s for 1 s, and stands again.
    pub fn start_stop(t: f32) -> Pose {
        Pose {
            orientation: yaw(0.3),
            position: [0.5 * (t - 1.5).clamp(0.0, 1.0), 1.5, -2.0],
        }
    }

    /// The errors of five runs, averaged.
    pub fn errors(truth: fn(f32) -> Pose, static_context: bool) -> Errors {
        let runs = (1..=5)
            .map(|seed| measure(seed * 7919, 4.0, truth, static_context))
            .collect::<Vec<_>>();
        let mean = |f: fn(&Errors) -> f32| runs.iter().map(f).sum::<f32>() / runs.len() as f32;
        Errors {
            position_rms_mm: mean(|e| e.position_rms_mm),
            position_p95_mm: mean(|e| e.position_p95_mm),
            angle_rms_degrees: mean(|e| e.angle_rms_degrees),
        }
    }

    pub const SCENARIOS: [(&str, fn(f32) -> Pose, bool); 5] = [
        ("still", still, false),
        ("walking", walking, false),
        ("circling", circling, false),
        ("start-stop", start_stop, false),
        ("static", still, true),
    ];

    /// What applications are told stays near the truth, by about half again what it is now.
    #[test]
    fn jitter_stays_small() {
        let limits = [
            ("still", 1.5, 0.35),
            ("walking", 4.0, 0.35),
            ("circling", 6.0, 1.0),
            ("start-stop", 11.0, 0.8),
            ("static", 0.8, 0.2),
        ];
        for ((name, truth, static_context), (_, position_mm, angle_degrees)) in
            SCENARIOS.into_iter().zip(limits)
        {
            let errors = errors(truth, static_context);
            assert!(
                errors.position_rms_mm < position_mm && errors.angle_rms_degrees < angle_degrees,
                "{name}: {errors:?}"
            );
        }
    }

    /// `cargo test -p alvr_openxr_layer jitter::report -- --nocapture`
    #[test]
    fn report() {
        for (name, truth, static_context) in SCENARIOS {
            let errors = errors(truth, static_context);
            println!(
                "{name:>10}: position rms {:5.2} mm, p95 {:5.2} mm, angle rms {:4.2}°",
                errors.position_rms_mm, errors.position_p95_mm, errors.angle_rms_degrees,
            );
        }
    }
}
