//! The marker poses the driver sent, per tracker of the headset, to answer for any time.
//!
//! - **Moving markers** are interpolated between the poses around the requested time, and
//!   extrapolated a little past the latest one, as a runtime predicts poses for the display time.
//! - **Static markers** (the application asked for XrSpatialMarkerStaticOptimizationEXT) are
//!   averaged over the latest second, since they do not move and detections jitter.

use alvr_marker_ipc::{Pose, Snapshot, TrackingConfig};
use std::collections::{HashMap, VecDeque};

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

#[derive(Clone, Copy, Debug)]
struct Sample {
    time_ns: i64,
    pose: Pose,
    size: f32,
}

struct TrackerHistory {
    config: TrackingConfig,
    latest_time_ns: i64,
    /// The markers of the latest update: id and text.
    seen: Vec<(u64, String)>,
    samples: HashMap<u64, VecDeque<Sample>>,
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
                        samples: HashMap::new(),
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
                .map(|marker| (marker.id, marker.text.clone()))
                .collect();

            for marker in &stream.markers {
                tracker
                    .samples
                    .entry(marker.id)
                    .or_default()
                    .push_back(Sample {
                        time_ns: stream.time_ns,
                        pose: marker.pose,
                        size: marker.size,
                    });
            }
            let oldest = stream.time_ns - HISTORY_NS;
            tracker.samples.retain(|_, samples| {
                // Two at least, to extrapolate from
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
    pub fn seen(&self, config: TrackingConfig, now_ns: i64) -> Vec<(u64, String)> {
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
        let samples = tracker.samples.get(&id)?;
        let latest = samples.back()?;

        Some(match at {
            At::Static => average(
                samples
                    .iter()
                    .filter(|sample| sample.time_ns >= latest.time_ns - STATIC_WINDOW_NS),
            ),
            At::Time(time_ns) => interpolate(samples, time_ns),
            At::Latest => Located {
                pose: latest.pose,
                size: latest.size,
            },
        })
    }
}

/// Between the poses around `time_ns`, or a little past the latest two.
fn interpolate(samples: &VecDeque<Sample>, time_ns: i64) -> Located {
    let latest = samples[samples.len() - 1];
    let (a, b) = match samples.iter().position(|sample| sample.time_ns >= time_ns) {
        Some(0) => (samples[0], samples[0]),
        Some(i) => (samples[i - 1], samples[i]),
        None if samples.len() >= 2 => (samples[samples.len() - 2], latest),
        None => (latest, latest),
    };
    if b.time_ns <= a.time_ns {
        return Located {
            pose: b.pose,
            size: b.size,
        };
    }

    let span = (b.time_ns - a.time_ns) as f64;
    let max = (b.time_ns - a.time_ns + MAX_EXTRAPOLATION_NS) as f64 / span;
    let t = ((time_ns - a.time_ns) as f64 / span).clamp(0.0, max) as f32;

    Located {
        pose: Pose {
            orientation: slerp(a.pose.orientation, b.pose.orientation, t),
            position: std::array::from_fn(|i| {
                a.pose.position[i] + (b.pose.position[i] - a.pose.position[i]) * t
            }),
        },
        size: a.size + (b.size - a.size) * t.clamp(0.0, 1.0),
    }
}

fn average<'a>(samples: impl Iterator<Item = &'a Sample>) -> Located {
    let mut count = 0.0;
    let mut position = [0.0; 3];
    let mut orientation = [0.0; 4];
    let mut size = 0.0;
    let mut reference = None;
    for sample in samples {
        let q = sample.pose.orientation;
        // q and -q are the same rotation; add them up in one hemisphere
        let reference = *reference.get_or_insert(q);
        let sign = if dot(q, reference) < 0.0 { -1.0 } else { 1.0 };

        for (sum, value) in position.iter_mut().zip(sample.pose.position) {
            *sum += value;
        }
        for (sum, value) in orientation.iter_mut().zip(q) {
            *sum += sign * value;
        }
        size += sample.size;
        count += 1.0;
    }

    Located {
        pose: Pose {
            orientation: normalize(orientation),
            position: position.map(|value| value / count),
        },
        size: size / count,
    }
}

fn dot(a: [f32; 4], b: [f32; 4]) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]
}

fn normalize(q: [f32; 4]) -> [f32; 4] {
    let length = dot(q, q).sqrt();
    if length > 0.0 {
        q.map(|value| value / length)
    } else {
        [0.0, 0.0, 0.0, 1.0]
    }
}

/// Spherical interpolation, along the shorter arc. `t` beyond 1 continues the rotation.
fn slerp(a: [f32; 4], b: [f32; 4], t: f32) -> [f32; 4] {
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

    fn close(a: f32, b: f32) -> bool {
        (a - b).abs() < 1e-4
    }

    fn history() -> MarkerHistory {
        let mut history = MarkerHistory::default();
        // A dog running along +X at 1 m/s, turning 90°/s
        for step in 0..=10 {
            let time = step * 10 * MS;
            history.add(&snapshot(&[(
                time,
                &[("dog", step as f32 * 0.01, step as f32 * 0.9)],
            )]));
        }
        history
    }

    #[test]
    fn moving_markers_are_interpolated() {
        let history = history();
        let now = 100 * MS;

        let at = |time| {
            history
                .locate(MOVING, dog(), At::Time(time), now)
                .unwrap()
                .pose
        };
        // Between two poses
        assert!(close(at(45 * MS).position[0], 0.045));
        assert!(close(at(45 * MS).orientation[1], yaw(4.05)[1]));
        // On one
        assert!(close(at(50 * MS).position[0], 0.05));
        // Before the first
        assert!(close(at(-50 * MS).position[0], 0.0));
    }

    #[test]
    fn moving_markers_are_predicted_a_little() {
        let history = history();
        let now = 100 * MS;
        let at = |time| {
            history
                .locate(MOVING, dog(), At::Time(time), now)
                .unwrap()
                .pose
        };

        // 20 ms past the latest pose
        assert!(close(at(120 * MS).position[0], 0.12));
        assert!(close(at(120 * MS).orientation[1], yaw(10.8)[1]));
        // No further than 100 ms
        assert!(close(at(500 * MS).position[0], 0.2));
    }

    #[test]
    fn static_markers_are_averaged() {
        let history = history();
        let located = history.locate(MOVING, dog(), At::Static, 100 * MS).unwrap();
        assert!(close(located.pose.position[0], 0.05));
        assert!(close(located.pose.orientation[1], yaw(4.5)[1]));
        assert!(close(located.size, 0.2));
    }

    #[test]
    fn markers_out_of_sight_are_not_located() {
        let mut history = history();
        history.add(&snapshot(&[(110 * MS, &[("cat", 1.0, 0.0)])]));
        let now = 110 * MS;

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
        assert_eq!(history.seen(MOVING, 100 * MS).len(), 1);
        assert!(history.seen(MOVING, 100 * MS + STALE_AFTER_NS).is_empty());
        assert!(
            history
                .locate(MOVING, dog(), At::Latest, 100 * MS + STALE_AFTER_NS)
                .is_none()
        );
    }

    #[test]
    fn repeated_and_stopped_trackers() {
        let mut history = history();
        // The driver repeats a tracker that did not change
        history.add(&snapshot(&[(100 * MS, &[])]));
        assert_eq!(history.seen(MOVING, 100 * MS).len(), 1);

        // The headset stops it
        history.add(&Snapshot::default());
        assert!(history.seen(MOVING, 100 * MS).is_empty());
    }

    #[test]
    fn slerp_takes_the_shorter_arc() {
        let a = yaw(10.0);
        let b = yaw(30.0).map(|value| -value);
        let middle = slerp(a, b, 0.5);
        let expected = yaw(20.0);
        assert!(close(dot(middle, expected).abs(), 1.0));
    }
}
