//! QR codes in the room, from `markers.json`. The emulated headset reports all of them as seen.

use crate::scene::{Primitive, Scene, Texture, Vertex};
use alvr_common::{
    Pose,
    anyhow::{Context, Result},
    glam::{EulerRot, Quat, Vec3},
    hash_string, info, warn,
};
use alvr_packets::MarkerData;
use serde::{Deserialize, Serialize};
use std::path::Path;

pub const SETTINGS_FILE_NAME: &str = "markers.json";

/// Modules along one side of the pattern: the size of a version 1 QR code.
const MODULES: usize = 21;
/// Light modules around the pattern, as the QR specification asks for.
const QUIET_ZONE: usize = 4;
/// Texels per module. Large enough to stay crisp up close under linear filtering.
const TEXELS_PER_MODULE: usize = 8;
#[derive(Serialize, Deserialize)]
pub struct MarkerSettings {
    pub id: String,
    pub position: [f32; 3],
    /// Yaw, pitch and roll. All zero is an upright code facing +Z.
    pub rotation_degrees: [f32; 3],
    /// Edge length without the quiet zone, in metres.
    pub size: f32,
}

impl MarkerSettings {
    pub fn pose(&self) -> Pose {
        let [yaw, pitch, roll] = self.rotation_degrees.map(f32::to_radians);

        Pose {
            orientation: Quat::from_euler(EulerRot::YXZ, yaw, pitch, roll),
            position: Vec3::from_array(self.position),
        }
    }

    pub fn data(&self) -> MarkerData {
        MarkerData {
            id: self.id.clone(),
            pose: self.pose(),
            size: self.size,
        }
    }
}

#[derive(Serialize, Deserialize, Default)]
struct SettingsFile {
    markers: Vec<MarkerSettings>,
}

/// Loads the markers in `directory`, writing a default file first if none exists: one code lying
/// face up on the centre of the bundled room's rug.
pub fn load_or_create(directory: &Path) -> Vec<MarkerSettings> {
    let path = directory.join(SETTINGS_FILE_NAME);

    if !path.exists() {
        let file = SettingsFile {
            markers: vec![MarkerSettings {
                id: "ALVR-QR-1".into(),
                position: [0.0, 0.003, 0.0],
                rotation_degrees: [0.0, -90.0, 0.0],
                size: 0.5,
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
        Ok(file) => file.markers,
        Err(e) => {
            warn!(
                "Cannot load {}: {e:#}. No markers in the room.",
                path.display()
            );
            Vec::new()
        }
    }
}

fn load_file(path: &Path) -> Result<SettingsFile> {
    let text = std::fs::read_to_string(path).context("Cannot read marker settings")?;
    serde_json::from_str(&text).context("Cannot parse marker settings")
}

pub fn add_to_scene(scene: &mut Scene, markers: &[MarkerSettings]) {
    for marker in markers {
        let pose = marker.pose();

        let half = marker.size / 2.0 * (MODULES + 2 * QUIET_ZONE) as f32 / MODULES as f32;
        // Off the surface it lies on, to avoid z-fighting
        let lift = Vec3::Z * 0.001;

        let corners = [
            (Vec3::new(-half, -half, 0.0), [0.0, 1.0]),
            (Vec3::new(half, -half, 0.0), [1.0, 1.0]),
            (Vec3::new(half, half, 0.0), [1.0, 0.0]),
            (Vec3::new(-half, half, 0.0), [0.0, 0.0]),
        ];

        let base = scene.vertices.len() as u32;
        for (corner, uv) in corners {
            scene.vertices.push(Vertex {
                position: (pose.position + pose.orientation * (corner + lift)).to_array(),
                uv,
                color: [1.0; 3],
            });
        }

        scene.primitives.push(Primitive {
            indices: vec![base, base + 1, base + 2, base, base + 2, base + 3],
            texture: Some(pattern_texture(&marker.id)),
        });
    }
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
