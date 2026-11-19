//! Which marker trackers the headset runs: one per configuration OpenXR applications ask for,
//! each without the options the headset does not support.

use crate::{ServerCoreEvent, tracking::TrackingManager};
use alvr_common::parking_lot::Mutex;
use alvr_packets::{MarkerCapabilities, MarkerData, MarkerTrackingConfig, ServerControlPacket};
use alvr_sockets::ControlSocketSender;
use std::{sync::Arc, time::Duration};

/// The markers one tracker of the headset sees.
#[derive(Clone, Debug)]
pub struct MarkerStream {
    pub config: MarkerTrackingConfig,
    /// When the poses are for, on the clock of the tracking's `poll_timestamp`.
    pub timestamp: Duration,
    pub markers: Vec<MarkerData>,
}

type ControlSender = Arc<Mutex<ControlSocketSender<ServerControlPacket>>>;

#[derive(Default)]
pub struct MarkerTracking {
    /// Of the connected headset, once it reported them.
    capabilities: Option<MarkerCapabilities>,
    /// By OpenXR applications.
    requested: Vec<MarkerTrackingConfig>,
    /// Last sent to the headset.
    sent: Option<Vec<MarkerTrackingConfig>>,
    control_sender: Option<ControlSender>,
    streams: Vec<MarkerStream>, // client's reference space, one per running tracker
}

impl MarkerTracking {
    /// The trackers the headset should run, or `None` until it reported its capabilities.
    pub fn configs(&self) -> Option<Vec<MarkerTrackingConfig>> {
        let capabilities = self.capabilities?;
        if !capabilities.qr_codes {
            return Some(Vec::new());
        }

        let mut configs = Vec::new();
        for config in self
            .requested
            .iter()
            .map(|config| config.supported_by(&capabilities))
        {
            if !configs.contains(&config) {
                configs.push(config);
            }
        }

        Some(configs)
    }

    pub fn connect(&mut self, control_sender: ControlSender) {
        self.disconnect();
        self.control_sender = Some(control_sender);
    }

    pub fn disconnect(&mut self) {
        self.capabilities = None;
        self.sent = None;
        self.control_sender = None;
        self.streams.clear();
    }

    /// Returns whether markers of trackers the headset no longer runs were dropped.
    pub fn set_capabilities(&mut self, capabilities: MarkerCapabilities) -> bool {
        self.capabilities = Some(capabilities);
        self.send_configs()
    }

    /// Replaces what OpenXR applications ask for. Returns whether markers of trackers the headset
    /// no longer runs were dropped.
    pub fn set_requested(&mut self, configs: Vec<MarkerTrackingConfig>) -> bool {
        self.requested = configs;
        self.send_configs()
    }

    /// Replaces the markers of the stream's tracker. Returns false, ignoring them, for a tracker
    /// the headset should not run, such as one it was just told to stop.
    pub fn set_stream(&mut self, stream: MarkerStream) -> bool {
        if !self
            .configs()
            .is_some_and(|configs| configs.contains(&stream.config))
        {
            return false;
        }

        match self
            .streams
            .iter_mut()
            .find(|existing| existing.config == stream.config)
        {
            Some(existing) => *existing = stream,
            None => self.streams.push(stream),
        }

        true
    }

    /// Every tracker's markers, recentered, and how recentering moved the space they are in.
    pub fn event(&self, tracking_manager: &TrackingManager) -> ServerCoreEvent {
        ServerCoreEvent::Markers {
            streams: self
                .streams
                .iter()
                .map(|stream| MarkerStream {
                    markers: stream
                        .markers
                        .iter()
                        .map(|marker| MarkerData {
                            pose: tracking_manager.recenter_pose(marker.pose),
                            ..marker.clone()
                        })
                        .collect(),
                    ..stream.clone()
                })
                .collect(),
            origin: tracking_manager.origin_change(),
        }
    }

    /// Sends the trackers to run if they changed, and drops the markers of the others. Returns
    /// whether any were dropped.
    fn send_configs(&mut self) -> bool {
        let Some(configs) = self.configs() else {
            return false;
        };

        let stream_count = self.streams.len();
        self.streams
            .retain(|stream| configs.contains(&stream.config));

        if let Some(sender) = &self.control_sender
            && self.sent.as_ref() != Some(&configs)
            && sender
                .lock()
                .send(&ServerControlPacket::MarkerTrackingConfigs(configs.clone()))
                .is_ok()
        {
            self.sent = Some(configs);
        }

        self.streams.len() != stream_count
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SIZED: MarkerTrackingConfig = MarkerTrackingConfig {
        static_markers: false,
        marker_size: Some(0.15),
    };
    const ALL: MarkerCapabilities = MarkerCapabilities {
        qr_codes: true,
        fixed_size_markers: true,
        static_markers: true,
    };

    fn stream(config: MarkerTrackingConfig) -> MarkerStream {
        MarkerStream {
            config,
            timestamp: Duration::ZERO,
            markers: Vec::new(),
        }
    }

    #[test]
    fn only_requested_trackers_run() {
        let mut tracking = MarkerTracking::default();
        tracking.set_capabilities(ALL);
        assert_eq!(tracking.configs(), Some(vec![]));

        tracking.set_requested(vec![SIZED]);
        assert_eq!(tracking.configs(), Some(vec![SIZED]));
    }

    #[test]
    fn configs_wait_for_the_capabilities() {
        let mut tracking = MarkerTracking::default();
        tracking.set_requested(vec![SIZED]);
        assert_eq!(tracking.configs(), None);

        tracking.set_capabilities(ALL);
        assert_eq!(tracking.configs(), Some(vec![SIZED]));
    }

    #[test]
    fn unsupported_options_merge_configs() {
        let mut tracking = MarkerTracking::default();
        tracking.set_requested(vec![SIZED, MarkerTrackingConfig::default()]);
        tracking.set_capabilities(MarkerCapabilities {
            qr_codes: true,
            ..Default::default()
        });

        // The headset would run all three the same way
        assert_eq!(
            tracking.configs(),
            Some(vec![MarkerTrackingConfig::default()])
        );
    }

    #[test]
    fn no_trackers_without_qr_codes() {
        let mut tracking = MarkerTracking::default();
        tracking.set_capabilities(MarkerCapabilities::default());
        assert_eq!(tracking.configs(), Some(Vec::new()));
    }

    #[test]
    fn markers_follow_the_trackers() {
        let mut tracking = MarkerTracking::default();
        tracking.set_capabilities(ALL);
        assert!(!tracking.set_stream(stream(SIZED)));

        tracking.set_requested(vec![SIZED]);
        assert!(tracking.set_stream(stream(SIZED)));
        assert!(!tracking.set_requested(vec![SIZED]));
        assert!(tracking.set_requested(Vec::new()));
        assert!(tracking.streams.is_empty());
    }
}
