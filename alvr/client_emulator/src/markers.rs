//! QR codes in the room, from `markers.json`, or replaced through `/api/markers`.
//!
//! The emulated headset reports a code while it could read it: in the view, facing the camera and
//! near enough for its size. `always_seen` reports every code all the time instead.
//!
//! A code with a `motion` circles its position, like one on a dog running around, to test
//! applications that follow moving markers. The room shows it where it is.

use crate::{
    camera::Camera,
    scene::{Primitive, Scene, Texture, Vertex},
};
use alvr_common::{
    Pose,
    anyhow::{Context, Result, bail},
    glam::{EulerRot, Mat4, Quat, Vec3},
    hash_string, info, warn,
};
use alvr_packets::{MarkerData, MarkerPayload};
use serde::{Deserialize, Serialize};
use std::{f32::consts::TAU, path::Path, time::Duration};

pub const SETTINGS_FILE_NAME: &str = "markers.json";

/// Modules along one side of the pattern: the size of a version 1 QR code.
const MODULES: usize = 21;
/// Light modules around the pattern, as the QR specification asks for.
const QUIET_ZONE: usize = 4;
/// Texels per module. Large enough to stay crisp up close under linear filtering.
const TEXELS_PER_MODULE: usize = 8;

/// Furthest a code is read from, in its edge lengths: about where a headset camera still resolves
/// its modules.
const MAX_DISTANCE_IN_SIZES: f32 = 20.0;
/// A code turned further than this from the camera is not read.
const MAX_VIEWING_ANGLE: f32 = 70.0_f32.to_radians();

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct MarkerSettings {
    /// The text the code holds, unless it holds `bytes_hex`. Names the code either way.
    pub id: String,
    /// Bytes the code holds instead of text, as hexadecimal digits.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub bytes_hex: Option<String>,
    pub position: [f32; 3],
    /// Yaw, pitch and roll. All zero is an upright code facing +Z.
    pub rotation_degrees: [f32; 3],
    /// Edge length without the quiet zone, in metres.
    pub size: f32,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub motion: Option<MarkerMotion>,
    /// How much its detections jitter, as a headset camera's do.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub noise: Option<MarkerNoise>,
}

/// Detection noise: normally distributed per axis, and now and then a detection far off.
#[derive(Serialize, Deserialize, Clone, Copy, Debug)]
pub struct MarkerNoise {
    /// Standard deviation of the position, per axis.
    #[serde(default)]
    pub position_mm: f32,
    /// Standard deviation of the orientation, per axis.
    #[serde(default)]
    pub angle_degrees: f32,
    /// Share of the detections that are off by `outlier_mm`, in a random direction.
    #[serde(default)]
    pub outlier_probability: f32,
    #[serde(default)]
    pub outlier_mm: f32,
}

impl MarkerNoise {
    /// `pose`, as detected once. The same `seed` gives the same detection.
    fn apply(&self, pose: Pose, seed: u64) -> Pose {
        let mut random = Random(scramble(seed));

        let mut position = pose.position
            + Vec3::new(random.gaussian(), random.gaussian(), random.gaussian())
                * (self.position_mm / 1000.0);
        if random.uniform() < self.outlier_probability {
            let direction = Vec3::new(random.gaussian(), random.gaussian(), random.gaussian())
                .normalize_or(Vec3::X);
            position += direction * (self.outlier_mm / 1000.0);
        }
        let rotation = Vec3::new(random.gaussian(), random.gaussian(), random.gaussian())
            * self.angle_degrees.to_radians();

        Pose {
            orientation: pose.orientation * Quat::from_scaled_axis(rotation),
            position,
        }
    }
}

/// The seed scrambled (splitmix64): xorshift's first numbers from similar seeds are not random.
fn scramble(seed: u64) -> u64 {
    let mut z = seed.wrapping_add(0x9e37_79b9_7f4a_7c15);
    z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
    (z ^ (z >> 31)) | 1
}

/// xorshift64, enough for detection noise.
struct Random(u64);

impl Random {
    /// In [0, 1).
    fn uniform(&mut self) -> f32 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        (self.0 >> 40) as f32 / (1u64 << 24) as f32
    }

    /// Standard normal, by Box-Muller.
    fn gaussian(&mut self) -> f32 {
        let u = self.uniform().max(1e-7);
        let v = self.uniform();
        (-2.0 * u.ln()).sqrt() * (TAU * v).cos()
    }
}

/// A horizontal circle around the code's position, turning the code along with it.
#[derive(Serialize, Deserialize, Clone, Copy, Debug)]
pub struct MarkerMotion {
    pub radius: f32,
    /// For one turn. Negative turns clockwise seen from above.
    pub period_seconds: f32,
}

impl MarkerSettings {
    /// Where the code is at rest.
    pub fn pose(&self) -> Pose {
        let [yaw, pitch, roll] = self.rotation_degrees.map(f32::to_radians);

        Pose {
            orientation: Quat::from_euler(EulerRot::YXZ, yaw, pitch, roll),
            position: Vec3::from_array(self.position),
        }
    }

    /// Where the code is `time` after the tracking clock started.
    pub fn pose_at(&self, time: Duration) -> Pose {
        let pose = self.pose();
        let Some(motion) = self.motion.filter(|motion| motion.period_seconds != 0.0) else {
            return pose;
        };

        let angle = TAU * time.as_secs_f32() / motion.period_seconds;
        let turn = Quat::from_rotation_y(angle);
        Pose {
            orientation: turn * pose.orientation,
            position: pose.position + turn * Vec3::new(motion.radius, 0.0, 0.0),
        }
    }

    pub fn payload(&self) -> Result<MarkerPayload> {
        let Some(hex) = &self.bytes_hex else {
            return Ok(MarkerPayload::Text(self.id.clone()));
        };

        let digits = hex.trim();
        if digits.len() % 2 != 0 || !digits.is_ascii() {
            bail!("bytes_hex of \"{}\" is not pairs of hex digits", self.id);
        }
        (0..digits.len())
            .step_by(2)
            .map(|i| {
                u8::from_str_radix(&digits[i..i + 2], 16)
                    .with_context(|| format!("bytes_hex of \"{}\" is not hexadecimal", self.id))
            })
            .collect::<Result<Vec<_>>>()
            .map(MarkerPayload::Bytes)
    }

    /// Whether a headset at `head` could read the code at `pose`.
    pub fn is_seen_from(&self, pose: Pose, head: Pose) -> bool {
        let to_code = pose.position - head.position;
        let distance = to_code.length();
        if distance <= f32::EPSILON || distance > MAX_DISTANCE_IN_SIZES * self.size {
            return false;
        }

        // In the camera's view, which looks along -Z
        let local = head.orientation.inverse() * to_code;
        if local.z >= 0.0 {
            return false;
        }
        let fov = Camera::fov();
        let horizontal = local.x.atan2(-local.z);
        let vertical = local.y.atan2(-local.z);
        if !(fov.left..=fov.right).contains(&horizontal) || !(fov.down..=fov.up).contains(&vertical)
        {
            return false;
        }

        // Its printed face towards the camera
        let normal = pose.orientation * Vec3::Z;
        normal.dot(-to_code / distance) >= MAX_VIEWING_ANGLE.cos()
    }
}

/// Every code in the room, and how they are reported.
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct MarkerSettingsFile {
    /// Report every code all the time, wherever the headset looks.
    #[serde(default)]
    pub always_seen: bool,
    pub markers: Vec<MarkerSettings>,
}

impl MarkerSettingsFile {
    pub fn validate(&self) -> Result<()> {
        for marker in &self.markers {
            marker.payload()?;
            if !(marker.size > 0.0 && marker.size.is_finite()) {
                bail!("The size of \"{}\" is not positive", marker.id);
            }
            if marker.noise.is_some_and(|noise| {
                ![
                    noise.position_mm,
                    noise.angle_degrees,
                    noise.outlier_probability,
                    noise.outlier_mm,
                ]
                .iter()
                .all(|value| value.is_finite() && *value >= 0.0)
            }) {
                bail!("The noise of \"{}\" is not finite and positive", marker.id);
            }
            if marker.motion.is_some_and(|motion| {
                !motion.radius.is_finite() || !motion.period_seconds.is_finite()
            }) {
                bail!("The motion of \"{}\" is not finite", marker.id);
            }
        }

        Ok(())
    }

    /// The codes a headset at `head` reads `time` after the tracking clock started, and where.
    pub fn seen(&self, head: Pose, time: Duration) -> Vec<MarkerData> {
        self.markers
            .iter()
            .filter_map(|marker| {
                let pose = marker.pose_at(time);
                if !self.always_seen && !marker.is_seen_from(pose, head) {
                    return None;
                }

                let pose = match &marker.noise {
                    Some(noise) => {
                        noise.apply(pose, hash_string(&marker.id) ^ time.as_nanos() as u64)
                    }
                    None => pose,
                };

                Some(MarkerData {
                    // Validated when loaded
                    payload: marker.payload().ok()?,
                    pose,
                    size: marker.size,
                })
            })
            .collect()
    }
}

/// Loads the markers in `directory`, writing a default file first if none exists: one code lying
/// face up on the centre of the bundled room's rug.
pub fn load_or_create(directory: &Path) -> MarkerSettingsFile {
    let path = directory.join(SETTINGS_FILE_NAME);

    if !path.exists() {
        let file = MarkerSettingsFile {
            always_seen: false,
            markers: vec![MarkerSettings {
                id: "ALVR-QR-1".into(),
                bytes_hex: None,
                position: [0.0, 0.003, 0.0],
                rotation_degrees: [0.0, -90.0, 0.0],
                size: 0.5,
                motion: None,
                noise: None,
            }],
        };

        match serde_json::to_string_pretty(&file) {
            Ok(json) => {
                if let Err(e) = std::fs::write(&path, json + "\n") {
                    warn!("Cannot write default marker settings: {e}");
                } else {
                    info!("Wrote default marker settings to {}", path.display());
                }
            }
            Err(e) => warn!("Cannot serialise default marker settings: {e}"),
        }
    }

    match load_file(&path) {
        Ok(file) => file,
        Err(e) => {
            warn!(
                "Cannot load {}: {e:#}. No markers in the room.",
                path.display()
            );
            MarkerSettingsFile::default()
        }
    }
}

fn load_file(path: &Path) -> Result<MarkerSettingsFile> {
    let text = std::fs::read_to_string(path).context("Cannot read marker settings")?;
    let file: MarkerSettingsFile =
        serde_json::from_str(&text).context("Cannot parse marker settings")?;
    file.validate()?;

    Ok(file)
}

/// The code's model, around its centre: +X to its right, +Y to its top edge, its face towards +Z.
pub fn marker_scene(marker: &MarkerSettings) -> Scene {
    let half = marker.size / 2.0 * (MODULES + 2 * QUIET_ZONE) as f32 / MODULES as f32;
    // Off the surface it lies on, to avoid z-fighting
    let lift = 0.001;

    let corners = [
        ([-half, -half, lift], [0.0, 1.0]),
        ([half, -half, lift], [1.0, 1.0]),
        ([half, half, lift], [1.0, 0.0]),
        ([-half, half, lift], [0.0, 0.0]),
    ];

    Scene {
        vertices: corners
            .into_iter()
            .map(|(position, uv)| Vertex {
                position,
                uv,
                color: [1.0; 3],
            })
            .collect(),
        primitives: vec![Primitive {
            indices: vec![0, 1, 2, 0, 2, 3],
            texture: Some(pattern_texture(&marker.id)),
        }],
    }
}

pub fn model_matrix(pose: Pose) -> Mat4 {
    Mat4::from_rotation_translation(pose.orientation, pose.position)
}

/// Looks like a QR code but does not decode: the data modules are noise seeded by the text.
fn pattern_texture(id: &str) -> Texture {
    let mut dark = [[false; MODULES]; MODULES];

    // Data modules first, then the fixed patterns on top.
    let mut state = hash_string(id) | 1;
    for row in dark.iter_mut() {
        for module in row.iter_mut() {
            // xorshift64
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            *module = state & 1 == 1;
        }
    }

    for i in 0..MODULES {
        dark[6][i] = i % 2 == 0;
        dark[i][6] = i % 2 == 0;
    }

    for (top, left) in [(0, 0), (0, MODULES - 7), (MODULES - 7, 0)] {
        // Finder pattern plus its light separator
        for row in top.saturating_sub(1)..(top + 8).min(MODULES) {
            for column in left.saturating_sub(1)..(left + 8).min(MODULES) {
                let y = row as isize - top as isize;
                let x = column as isize - left as isize;
                let ring = y.min(x).min(6 - y).min(6 - x);

                dark[row][column] =
                    (0..7).contains(&y) && (0..7).contains(&x) && (ring == 0 || ring >= 2);
            }
        }
    }

    let side = (MODULES + 2 * QUIET_ZONE) * TEXELS_PER_MODULE;
    let mut pixels = Vec::with_capacity(side * side * 4);

    for y in 0..side {
        for x in 0..side {
            let row = (y / TEXELS_PER_MODULE).wrapping_sub(QUIET_ZONE);
            let column = (x / TEXELS_PER_MODULE).wrapping_sub(QUIET_ZONE);
            let is_dark = row < MODULES && column < MODULES && dark[row][column];
            let value = if is_dark { 16 } else { 245 };

            pixels.extend_from_slice(&[value, value, value, 255]);
        }
    }

    Texture {
        width: side as u32,
        height: side as u32,
        pixels,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn upright(position: [f32; 3], size: f32) -> MarkerSettings {
        MarkerSettings {
            id: "dog".into(),
            bytes_hex: None,
            position,
            rotation_degrees: [0.0; 3],
            size,
            motion: None,
            noise: None,
        }
    }

    const HEAD: Pose = Pose {
        orientation: Quat::IDENTITY,
        position: Vec3::new(0.0, 1.6, 0.0),
    };

    #[test]
    fn codes_are_seen_in_view_facing_and_near() {
        // Ahead, its face (+Z) towards the head
        let ahead = upright([0.0, 1.6, -1.0], 0.2);
        assert!(ahead.is_seen_from(ahead.pose(), HEAD));

        let behind = upright([0.0, 1.6, 1.0], 0.2);
        assert!(!behind.is_seen_from(behind.pose(), HEAD));

        let mut turned_away = upright([0.0, 1.6, -1.0], 0.2);
        turned_away.rotation_degrees = [180.0, 0.0, 0.0];
        assert!(!turned_away.is_seen_from(turned_away.pose(), HEAD));

        let too_far = upright([0.0, 1.6, -5.0], 0.2);
        assert!(!too_far.is_seen_from(too_far.pose(), HEAD));

        // Outside the 90° view
        let aside = upright([-2.0, 1.6, -1.0], 0.2);
        assert!(!aside.is_seen_from(aside.pose(), HEAD));
    }

    #[test]
    fn the_default_code_is_seen_when_looking_down_at_it() {
        let file = MarkerSettingsFile {
            always_seen: false,
            markers: vec![MarkerSettings {
                rotation_degrees: [0.0, -90.0, 0.0],
                ..upright([0.0, 0.003, 0.0], 0.5)
            }],
        };
        assert!(file.seen(HEAD, Duration::ZERO).is_empty());

        let looking_down = Pose {
            orientation: Quat::from_rotation_x(-80.0_f32.to_radians()),
            ..HEAD
        };
        assert_eq!(file.seen(looking_down, Duration::ZERO).len(), 1);

        let always = MarkerSettingsFile {
            always_seen: true,
            ..file
        };
        assert_eq!(always.seen(HEAD, Duration::ZERO).len(), 1);
    }

    #[test]
    fn noise_jitters_detections() {
        let noise = MarkerNoise {
            position_mm: 2.0,
            angle_degrees: 0.5,
            outlier_probability: 0.0,
            outlier_mm: 0.0,
        };
        let pose = upright([0.0, 1.5, -1.0], 0.2).pose();
        let detections = (0..1000)
            .map(|seed| noise.apply(pose, seed * 7919))
            .collect::<Vec<_>>();
        let deviation = (detections
            .iter()
            .map(|detection| (detection.position.x - pose.position.x).powi(2))
            .sum::<f32>()
            / detections.len() as f32)
            .sqrt();
        assert!((deviation - 0.002).abs() < 0.0003, "{deviation}");
        // The same seed, the same detection
        assert_eq!(noise.apply(pose, 3).position, noise.apply(pose, 3).position);
    }

    #[test]
    fn bytes_are_parsed() {
        let mut marker = upright([0.0; 3], 0.1);
        marker.bytes_hex = Some("00ff10".into());
        assert_eq!(
            marker.payload().unwrap(),
            MarkerPayload::Bytes(vec![0, 0xff, 0x10])
        );

        marker.bytes_hex = Some("0".into());
        assert!(marker.payload().is_err());
        marker.bytes_hex = Some("zz".into());
        assert!(marker.payload().is_err());
    }
}
