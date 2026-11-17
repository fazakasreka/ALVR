//! Prints the markers the ALVR driver currently publishes to OpenXR applications.
//!
//! `cargo run -p alvr_marker_shm --example dump_markers [-- --watch]`

use alvr_marker_shm::{FLAG_CLIENT_CONNECTED, Reader};
use std::{process::ExitCode, thread, time::Duration};

fn main() -> ExitCode {
    let watch = std::env::args().any(|arg| arg == "--watch");

    let reader = match Reader::open() {
        Ok(reader) => reader,
        Err(e) => {
            println!("No marker memory (is SteamVR running with the ALVR driver?): {e}");
            return ExitCode::FAILURE;
        }
    };

    let mut last_generation = None;
    loop {
        match reader.read() {
            Some(snapshot) if last_generation != Some(snapshot.generation) => {
                last_generation = Some(snapshot.generation);
                println!(
                    "generation {}, headset {}, {} marker(s)",
                    snapshot.generation,
                    if snapshot.flags & FLAG_CLIENT_CONNECTED != 0 {
                        "connected"
                    } else {
                        "not connected"
                    },
                    snapshot.markers.len()
                );
                for marker in &snapshot.markers {
                    println!(
                        "  {:016x} \"{}\" position {:?} orientation {:?} size {}",
                        marker.id,
                        marker.text,
                        marker.pose.position,
                        marker.pose.orientation,
                        marker.size
                    );
                }
            }
            Some(_) => (),
            None => {
                println!("The driver is not running");
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
