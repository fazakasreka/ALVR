use openxr_sys::{Posef, Quaternionf, Vector3f};

fn multiply(a: Quaternionf, b: Quaternionf) -> Quaternionf {
    Quaternionf {
        x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    }
}

fn rotate(q: Quaternionf, v: Vector3f) -> Vector3f {
    // v + 2w(u × v) + 2u × (u × v), with u the vector part of q
    let cross = |a: Vector3f, b: Vector3f| Vector3f {
        x: a.y * b.z - a.z * b.y,
        y: a.z * b.x - a.x * b.z,
        z: a.x * b.y - a.y * b.x,
    };
    let u = Vector3f {
        x: q.x,
        y: q.y,
        z: q.z,
    };
    let t = cross(u, v);
    let t = Vector3f {
        x: 2.0 * t.x,
        y: 2.0 * t.y,
        z: 2.0 * t.z,
    };
    let c = cross(u, t);

    Vector3f {
        x: v.x + q.w * t.x + c.x,
        y: v.y + q.w * t.y + c.y,
        z: v.z + q.w * t.z + c.z,
    }
}

/// The pose `b`, given relative to `a`, relative to what `a` is relative to.
pub fn compose(a: Posef, b: Posef) -> Posef {
    let offset = rotate(a.orientation, b.position);

    Posef {
        orientation: multiply(a.orientation, b.orientation),
        position: Vector3f {
            x: a.position.x + offset.x,
            y: a.position.y + offset.y,
            z: a.position.z + offset.z,
        },
    }
}

/// The pose that undoes `a`: composed with it, either way round, it gives the identity.
pub fn inverse(a: Posef) -> Posef {
    let orientation = Quaternionf {
        x: -a.orientation.x,
        y: -a.orientation.y,
        z: -a.orientation.z,
        w: a.orientation.w,
    };
    let position = rotate(orientation, a.position);

    Posef {
        orientation,
        position: Vector3f {
            x: -position.x,
            y: -position.y,
            z: -position.z,
        },
    }
}

pub fn from_ipc(pose: alvr_marker_ipc::Pose) -> Posef {
    let [x, y, z, w] = pose.orientation;
    let [px, py, pz] = pose.position;
    Posef {
        orientation: Quaternionf { x, y, z, w },
        position: Vector3f {
            x: px,
            y: py,
            z: pz,
        },
    }
}

pub fn to_ipc(pose: Posef) -> alvr_marker_ipc::Pose {
    let (o, p) = (pose.orientation, pose.position);
    alvr_marker_ipc::Pose {
        orientation: [o.x, o.y, o.z, o.w],
        position: [p.x, p.y, p.z],
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn inverse_undoes() {
        let a = Posef {
            orientation: Quaternionf {
                x: 0.0,
                y: 0.38268343,
                z: 0.0,
                w: 0.9238795,
            },
            position: Vector3f {
                x: 1.0,
                y: 2.0,
                z: -3.0,
            },
        };
        let point = Posef {
            orientation: Quaternionf::IDENTITY,
            position: Vector3f {
                x: 0.5,
                y: -1.0,
                z: 2.0,
            },
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

    fn close(a: Vector3f, b: Vector3f) -> bool {
        (a.x - b.x).abs() < 1e-5 && (a.y - b.y).abs() < 1e-5 && (a.z - b.z).abs() < 1e-5
    }

    #[test]
    fn compose_rotates_then_translates() {
        let half = std::f32::consts::FRAC_1_SQRT_2;
        // 90 degrees around +Y, at (1, 0, 0)
        let a = Posef {
            orientation: Quaternionf {
                x: 0.0,
                y: half,
                z: 0.0,
                w: half,
            },
            position: Vector3f {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
        };
        // One metre ahead (-Z) of a
        let b = Posef {
            orientation: Quaternionf {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 1.0,
            },
            position: Vector3f {
                x: 0.0,
                y: 0.0,
                z: -1.0,
            },
        };

        let c = compose(a, b);
        // Turning left by 90 degrees makes ahead point to -X
        assert!(close(
            c.position,
            Vector3f {
                x: 0.0,
                y: 0.0,
                z: 0.0
            }
        ));
        assert!((c.orientation.y - half).abs() < 1e-5);
        assert!((c.orientation.w - half).abs() < 1e-5);
    }
}
