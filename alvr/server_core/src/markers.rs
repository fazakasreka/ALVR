//! Which marker trackers the headset runs: one per configuration OpenXR applications ask for,
//! each without the options the headset does not support.

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
        self.capabilities = None;
        self.sent = None;
        self.control_sender = Some(control_sender);
    }

    pub fn disconnect(&mut self) {
        self.capabilities = None;
        self.sent = None;
        self.control_sender = None;
    }

    pub fn set_capabilities(&mut self, capabilities: MarkerCapabilities) {
        self.capabilities = Some(capabilities);
        self.send_configs();
    }

    /// Replaces what OpenXR applications ask for.
    pub fn set_requested(&mut self, configs: Vec<MarkerTrackingConfig>) {
        self.requested = configs;
        self.send_configs();
    }

    fn send_configs(&mut self) {
        let (Some(configs), Some(sender)) = (self.configs(), &self.control_sender) else {
            return;
        };
        if self.sent.as_ref() == Some(&configs) {
            return;
        }

        if sender
            .lock()
            .send(&ServerControlPacket::MarkerTrackingConfigs(configs.clone()))
            .is_ok()
        {
            self.sent = Some(configs);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SIZED: MarkerTrackingConfig = MarkerTrackingConfig {
        static_markers: false,
        marker_size: Some(0.15),
    };

    #[test]
    fn only_requested_trackers_run() {
        let mut tracking = MarkerTracking::default();
        tracking.set_capabilities(MarkerCapabilities {
            qr_codes: true,
            fixed_size_markers: true,
            static_markers: true,
        });
        assert_eq!(tracking.configs(), Some(vec![]));

        tracking.set_requested(vec![SIZED]);
        assert_eq!(tracking.configs(), Some(vec![SIZED]));
    }

    #[test]
    fn configs_wait_for_the_capabilities() {
        let mut tracking = MarkerTracking::default();
        tracking.set_requested(vec![SIZED]);
        assert_eq!(tracking.configs(), None);

        tracking.set_capabilities(MarkerCapabilities {
            qr_codes: true,
            fixed_size_markers: true,
            static_markers: true,
        });
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
}
