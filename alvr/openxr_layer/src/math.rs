//! Poses, positions and orientations as the driver sends them: positions are x, y, z and
//! orientations are quaternions x, y, z, w.

use alvr_marker_ipc::Pose;
use openxr_sys as xr;

pub type Vec3 = [f32; 3];
pub type Quat = [f32; 4];

pub const IDENTITY: Quat = [0.0, 0.0, 0.0, 1.0];

pub fn lerp(a: Vec3, b: Vec3, t: f32) -> Vec3 {
    std::array::from_fn(|i| a[i] + (b[i] - a[i]) * t)
}

pub fn length(v: Vec3) -> f32 {
    (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
}

pub fn distance(a: Vec3, b: Vec3) -> f32 {
    length(std::array::from_fn(|i| a[i] - b[i]))
}

pub fn dot(a: Quat, b: Quat) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]
}

pub fn normalize(q: Quat) -> Quat {
    let length = dot(q, q).sqrt();
    if length > 0.0 {
        q.map(|value| value / length)
    } else {
        IDENTITY
    }
}

/// The angle of the rotation between two orientations, in radians.
pub fn angle(a: Quat, b: Quat) -> f32 {
    2.0 * dot(a, b).abs().min(1.0).acos()
}

pub fn conjugate(q: Quat) -> Quat {
    [-q[0], -q[1], -q[2], q[3]]
}

pub fn quat_mul(a: Quat, b: Quat) -> Quat {
    [
        a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
        a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
        a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
        a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2],
    ]
}

pub fn rotate(q: Quat, v: Vec3) -> Vec3 {
    // v + 2w(u × v) + 2u × (u × v), with u the vector part of q
    let cross = |a: Vec3, b: Vec3| {
        [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ]
    };
    let u = [q[0], q[1], q[2]];
    let t = cross(u, v).map(|value| 2.0 * value);
    let c = cross(u, t);

    std::array::from_fn(|i| v[i] + q[3] * t[i] + c[i])
}

/// Axis times angle, along the shorter way round.
pub fn rotation_vector(q: Quat) -> Vec3 {
    let q = if q[3] < 0.0 { q.map(|v| -v) } else { q };
    let sin = length([q[0], q[1], q[2]]);
    if sin < 1e-7 {
        return [2.0 * q[0], 2.0 * q[1], 2.0 * q[2]];
    }
    let angle = 2.0 * sin.atan2(q[3]);
    [q[0], q[1], q[2]].map(|v| v / sin * angle)
}

pub fn from_rotation_vector(v: Vec3) -> Quat {
    let angle = length(v);
    if angle < 1e-7 {
        return normalize([v[0] / 2.0, v[1] / 2.0, v[2] / 2.0, 1.0]);
    }
    let sin = (angle / 2.0).sin() / angle;
    [v[0] * sin, v[1] * sin, v[2] * sin, (angle / 2.0).cos()]
}

/// Spherical interpolation, along the shorter arc.
pub fn slerp(a: Quat, b: Quat, t: f32) -> Quat {
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

/// The pose `b`, given relative to `a`, relative to what `a` is relative to.
pub fn compose(a: Pose, b: Pose) -> Pose {
    let offset = rotate(a.orientation, b.position);

    Pose {
        orientation: quat_mul(a.orientation, b.orientation),
        position: std::array::from_fn(|i| a.position[i] + offset[i]),
    }
}

/// The pose that undoes `a`: composed with it, either way round, it gives the identity.
pub fn inverse(a: Pose) -> Pose {
    let orientation = conjugate(a.orientation);

    Pose {
        orientation,
        position: rotate(orientation, a.position).map(|value| -value),
    }
}

pub fn to_xr(pose: Pose) -> xr::Posef {
    let [x, y, z, w] = pose.orientation;
    let [px, py, pz] = pose.position;
    xr::Posef {
        orientation: xr::Quaternionf { x, y, z, w },
        position: xr::Vector3f {
            x: px,
            y: py,
            z: pz,
        },
    }
}

pub fn from_xr(pose: xr::Posef) -> Pose {
    let (o, p) = (pose.orientation, pose.position);
    Pose {
        orientation: [o.x, o.y, o.z, o.w],
        position: [p.x, p.y, p.z],
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_util::{close, yaw_radians};
    use std::f32::consts::FRAC_PI_2;

    #[test]
    fn inverse_undoes() {
        let a = Pose {
            orientation: yaw_radians(0.785_398_2),
            position: [1.0, 2.0, -3.0],
        };
        let point = Pose {
            orientation: IDENTITY,
            position: [0.5, -1.0, 2.0],
        };
        assert!(close(
            compose(inverse(a), compose(a, point)).position,
            point.position
        ));
        assert!(close(
            compose(a, compose(inverse(a), point)).position,
            point.position
        ));
    }

    #[test]
    fn compose_rotates_then_translates() {
        // 90 degrees around +Y, at (1, 0, 0)
        let a = Pose {
            orientation: yaw_radians(FRAC_PI_2),
            position: [1.0, 0.0, 0.0],
        };
        // One metre ahead (-Z) of a
        let b = Pose {
            orientation: IDENTITY,
            position: [0.0, 0.0, -1.0],
        };

        let c = compose(a, b);
        // Turning left by 90 degrees makes ahead point to -X
        assert!(close(c.position, [0.0, 0.0, 0.0]));
        assert!((0..4).all(|i| (c.orientation[i] - a.orientation[i]).abs() < 1e-5));
    }

    #[test]
    fn slerp_takes_the_shorter_arc() {
        let a = yaw_radians(10f32.to_radians());
        let b = yaw_radians(30f32.to_radians()).map(|value| -value);
        let middle = slerp(a, b, 0.5);
        assert!(angle(middle, yaw_radians(20f32.to_radians())) < 1e-4);
    }

    #[test]
    fn rotation_vectors_round_trip() {
        let q = normalize([0.1, 0.7, -0.2, 0.6]);
        let back = from_rotation_vector(rotation_vector(q));
        assert!(angle(q, back) < 1e-4);
    }
}
