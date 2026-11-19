//! Prints the markers the ALVR driver currently sends to OpenXR applications.
//!
//! `cargo run -p alvr_marker_ipc --example dump_markers [-- --watch]`

use alvr_marker_ipc::{Client, FLAG_CLIENT_CONNECTED, now_ns};
use std::{process::ExitCode, thread, time::Duration};

fn main() -> ExitCode {
    let watch = std::env::args().any(|arg| arg == "--watch");

    let mut client = Client::new();
    let mut last_generation = None;
    loop {
        match client.read() {
            Some(snapshot) if last_generation != Some(snapshot.generation) => {
                last_generation = Some(snapshot.generation);
                println!(
                    "generation {}, headset {}, {:?}",
                    snapshot.generation,
                    if snapshot.flags & FLAG_CLIENT_CONNECTED != 0 {
                        "connected"
                    } else {
                        "not connected"
                    },
                    snapshot.capabilities,
                );
                let origin = &snapshot.origin;
                println!(
                    "  origin: {} change(s), the latest {:.1} s ago, {}",
                    origin.changes,
                    (now_ns() - origin.changed_ns) as f64 / 1e9,
                    match &origin.pose_in_previous {
                        Some(pose) => format!(
                            "new origin at {:?} {:?} in the previous one",
                            pose.position, pose.orientation
                        ),
                        None => "where to not known".to_owned(),
                    }
                );
                for stream in &snapshot.streams {
                    println!(
                        "  tracker {:?}: {} marker(s), {:.1} ms old",
                        stream.config,
                        stream.markers.len(),
                        (now_ns() - stream.time_ns) as f64 / 1e6
                    );
                    for marker in &stream.markers {
                        println!(
                            "    {:016x} {} position {:?} orientation {:?} size {}",
                            marker.id,
                            marker.payload,
                            marker.pose.position,
                            marker.pose.orientation,
                            marker.size
                        );
                    }
                }
            }
            Some(_) => (),
            None => {
                if last_generation.take().is_some() || !watch {
                    println!(
                        "The driver is not running (is SteamVR running with the ALVR driver?)"
                    );
                }
                if !watch {
                    return ExitCode::FAILURE;
                }
            }
        }

        if !watch {
            return ExitCode::SUCCESS;
        }
        thread::sleep(Duration::from_millis(500));
    }
}
