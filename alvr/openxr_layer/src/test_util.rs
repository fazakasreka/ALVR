//! What the tests of several modules share.

use crate::math::{Quat, Vec3};
use alvr_marker_ipc::TrackingConfig;

/// The tracker of the default options, for markers that move.
pub const MOVING: TrackingConfig = TrackingConfig {
    static_markers: false,
    marker_size: None,
};
pub const MS: i64 = 1_000_000;

/// A rotation around +Y.
pub fn yaw_radians(radians: f32) -> Quat {
    [0.0, (radians / 2.0).sin(), 0.0, (radians / 2.0).cos()]
}

pub fn close(a: Vec3, b: Vec3) -> bool {
    (0..3).all(|i| (a[i] - b[i]).abs() < 1e-5)
}
