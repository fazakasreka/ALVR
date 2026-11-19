//! Markers for OpenXR applications.
//!
//! SteamVR's OpenXR runtime knows nothing about markers, so ALVR ships an OpenXR API layer
//! (alvr/openxr_layer) that implements XR_EXT_spatial_marker_tracking inside each application.
//! The driver exchanges markers and the applications' tracker configurations with it through a
//! named pipe, and registers the layer with the OpenXR loader when SteamVR starts.
//!
//! Marker times are converted from the headset's clock to the PC's performance counter, the clock
//! OpenXR times convert to, the way the head pose is: the head pose submitted to SteamVR now is
//! the one for the headset time `poll_timestamp + motion-to-photon latency`.

use alvr_common::{Pose, info, parking_lot::Mutex, warn};
use alvr_filesystem as afs;
use alvr_marker_ipc::{
    self as ipc, Capabilities, FLAG_CLIENT_CONNECTED, Marker, Server, Stream, TrackingConfig,
    now_ns,
};
use alvr_server_core::{MarkerStream, OriginChange};
use std::time::Duration;

/// Weight of each new sample of the clock offset. Smooths network jitter over about a second.
const CLOCK_OFFSET_SMOOTHING: f64 = 0.02;

struct State {
    server: Server,
    client_connected: bool,
    /// Of the headset connected last.
    capabilities: Capabilities,
    streams: Vec<MarkerStream>,
    /// Performance counter time minus headset time, in nanoseconds.
    clock_offset_ns: Option<f64>,
}

static STATE: Mutex<Option<State>> = Mutex::new(None);
/// What OpenXR applications ask for, kept for a server context created after they asked.
static REQUESTED: Mutex<Vec<TrackingConfig>> = Mutex::new(Vec::new());

/// `on_requests` gets the tracker configurations OpenXR applications ask for, whenever they
/// change.
pub fn start(
    layout: &afs::Layout,
    enabled: bool,
    on_requests: impl Fn(Vec<TrackingConfig>) + Send + Sync + 'static,
) {
    alvr_server_io::register_openxr_layer(layout, enabled);

    if !enabled {
        return;
    }

    let server = match Server::start(move |configs| {
        info!("OpenXR applications ask for marker trackers {configs:?}");
        *REQUESTED.lock() = configs.clone();
        on_requests(configs);
    }) {
        Ok(server) => server,
        Err(e) => {
            warn!("OpenXR applications will not get markers: {e}");
            return;
        }
    };

    *STATE.lock() = Some(State {
        server,
        client_connected: false,
        capabilities: Capabilities::default(),
        streams: Vec::new(),
        clock_offset_ns: None,
    });
}

/// The tracker configurations OpenXR applications ask for.
pub fn requested_configs() -> Vec<TrackingConfig> {
    REQUESTED.lock().clone()
}

/// Joins the server's threads, so that the driver can be unloaded.
pub fn stop() {
    STATE.lock().take();
}

pub fn set_client_connected(connected: bool) {
    if let Some(state) = &mut *STATE.lock() {
        state.client_connected = connected;
        if !connected {
            state.streams.clear();
            state.clock_offset_ns = None;
        }
        publish(state);
    }
}

pub fn set_capabilities(capabilities: Capabilities) {
    if let Some(state) = &mut *STATE.lock() {
        state.capabilities = capabilities;
        publish(state);
    }
}

/// The head pose being submitted to SteamVR now is for `headset_time`.
pub fn report_head_pose_time(headset_time: Duration) {
    if let Some(state) = &mut *STATE.lock() {
        let offset = (now_ns() as i128 - headset_time.as_nanos() as i128) as f64;
        state.clock_offset_ns = Some(match state.clock_offset_ns {
            Some(smoothed) => smoothed + (offset - smoothed) * CLOCK_OFFSET_SMOOTHING,
            None => offset,
        });
    }
}

/// Every tracker of the headset, with poses recentered: in the space the headset pose is
/// submitted to SteamVR in. `origin` says how recentering moved that space.
pub fn set_markers(streams: Vec<MarkerStream>, origin: OriginChange) {
    if let Some(state) = &mut *STATE.lock() {
        if origin.count != state.server.origin().count {
            state.server.set_origin(ipc::OriginChange {
                count: origin.count,
                // The poses being submitted to SteamVR are already in the new space
                changed_ns: now_ns(),
                pose_in_previous: origin.pose_in_previous.map(to_ipc_pose),
                moved: origin.moved,
            });
        }
        state.streams = streams;
        publish(state);
    }
}

fn to_ipc_pose(pose: Pose) -> ipc::Pose {
    ipc::Pose {
        orientation: pose.orientation.to_array(),
        position: pose.position.to_array(),
    }
}

fn publish(state: &mut State) {
    let flags = if state.client_connected {
        FLAG_CLIENT_CONNECTED
    } else {
        0
    };

    let streams = state
        .streams
        .iter()
        .map(|stream| Stream {
            config: stream.config,
            time_ns: match state.clock_offset_ns {
                Some(offset) => (stream.timestamp.as_nanos() as f64 + offset) as i64,
                // No tracking yet to tell the clocks apart
                None => now_ns(),
            },
            markers: stream
                .markers
                .iter()
                .map(|marker| {
                    Marker::with_payload(
                        marker.payload.clone(),
                        to_ipc_pose(marker.pose),
                        marker.size,
                    )
                })
                .collect(),
        })
        .collect();

    state.server.publish(flags, state.capabilities, streams);
}
