//! Markers seen by the headset, exchanged between the ALVR driver and the ALVR OpenXR API layer.
//!
//! The driver, running inside vrserver.exe, serves a named pipe; the API layer, loaded into every
//! OpenXR application, connects to it.
//!
//! - **Driver → layer:** when a layer connects, and whenever anything changes, the driver sends
//!   the complete current state, a [`Snapshot`]: what the headset can do, and for each marker
//!   tracker the headset runs, the markers it sees and the time their poses are for. The layer
//!   reads whatever has arrived when it needs markers, and keeps a history of the poses to answer
//!   for any time.
//! - **Layer → driver:** the tracker configurations its applications' spatial contexts ask for,
//!   whenever they change. The driver has the headset run the configurations of all applications.
//!
//! The pipe is in message mode, so each message is exactly one encoded snapshot or request. A
//! broken pipe means the other end is gone.
//!
//! Poses are in the space the driver submits the headset pose to SteamVR in, which is what
//! SteamVR's OpenXR runtime calls the STAGE space. Times are nanoseconds on the performance
//! counter clock, see [`now_ns`].

use std::{
    collections::{HashMap, VecDeque},
    io,
    sync::{Arc, Mutex},
    time::{Duration, Instant},
};

#[cfg(windows)]
mod windows;
#[cfg(windows)]
use windows as sys;

#[cfg(not(windows))]
mod unsupported;
#[cfg(not(windows))]
use unsupported as sys;

pub use sys::{now_ns, ns_to_performance_counter, performance_counter_to_ns};

pub const PIPE_NAME: &str = "ALVR_Markers";

/// Changes whenever the encoding of a message changes. Messages of another version are ignored.
pub const PROTOCOL_VERSION: u32 = 3;
const SNAPSHOT_MAGIC: u32 = u32::from_le_bytes(*b"ALVM");
const REQUEST_MAGIC: u32 = u32::from_le_bytes(*b"ALVQ");

pub const MAX_STREAMS: usize = 8;
pub const MAX_MARKERS: usize = 32;
/// The most a QR code holds: 7089 digits, as text. Longer payloads are cut short, texts at a
/// character boundary.
pub const MAX_PAYLOAD_LEN: usize = 7089;
/// Longer messages are treated as a broken connection.
const MAX_MESSAGE_LEN: usize = 4 * 1024 * 1024;
/// Snapshots kept for [`Client::take_received`] at most; older ones are dropped.
const MAX_RECEIVED: usize = 256;

/// How often a client tries to connect while the driver is not running.
pub const CONNECT_RETRY_INTERVAL: Duration = Duration::from_secs(1);
/// How long a client waits for the snapshot the driver sends when it connects.
const FIRST_SNAPSHOT_TIMEOUT: Duration = Duration::from_millis(500);

/// A headset is connected.
pub const FLAG_CLIENT_CONNECTED: u32 = 1;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose {
    /// x, y, z, w
    pub orientation: [f32; 4],
    pub position: [f32; 3],
}

/// What the headset can do with markers.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Capabilities {
    pub qr_codes: bool,
    /// Honours [`TrackingConfig::marker_size`].
    pub fixed_size_markers: bool,
    /// Honours [`TrackingConfig::static_markers`].
    pub static_markers: bool,
}

/// How the headset tracks markers for a spatial context: its marker options.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct TrackingConfig {
    /// The markers do not move (XrSpatialMarkerStaticOptimizationEXT).
    pub static_markers: bool,
    /// Every marker has this edge length in metres (XrSpatialMarkerSizeEXT).
    pub marker_size: Option<f32>,
}

impl TrackingConfig {
    /// Without the options the headset does not support, which it would ignore.
    pub fn supported_by(self, capabilities: &Capabilities) -> Self {
        Self {
            static_markers: self.static_markers && capabilities.static_markers,
            marker_size: self.marker_size.filter(|_| capabilities.fixed_size_markers),
        }
    }
}

/// What a QR code holds, as the headset decoded it.
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub enum Payload {
    Text(String),
    /// Byte mode data that is not text.
    Bytes(Vec<u8>),
}

impl Payload {
    /// Cut to [`MAX_PAYLOAD_LEN`].
    fn truncated(self) -> Self {
        match self {
            Self::Text(text) => Self::Text(truncate(&text).to_owned()),
            Self::Bytes(mut bytes) => {
                bytes.truncate(MAX_PAYLOAD_LEN);
                Self::Bytes(bytes)
            }
        }
    }

    fn as_bytes(&self) -> &[u8] {
        match self {
            Self::Text(text) => text.as_bytes(),
            Self::Bytes(bytes) => bytes,
        }
    }
}

impl std::fmt::Display for Payload {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Text(text) => write!(f, "{text:?}"),
            Self::Bytes(bytes) => {
                write!(f, "0x")?;
                bytes.iter().try_for_each(|byte| write!(f, "{byte:02x}"))
            }
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Marker {
    /// Derived from the payload, so a marker keeps its id across driver restarts. Never 0.
    pub id: u64,
    /// The decoded content of the QR code.
    pub payload: Payload,
    /// Centre of the marker; +Y towards its top edge, +Z out of its face.
    pub pose: Pose,
    /// Edge length in metres.
    pub size: f32,
}

impl Marker {
    /// A marker for a QR code text, cut to [`MAX_PAYLOAD_LEN`].
    pub fn new(text: &str, pose: Pose, size: f32) -> Self {
        Self::with_payload(Payload::Text(text.to_owned()), pose, size)
    }

    /// A marker for a QR code, its payload cut to [`MAX_PAYLOAD_LEN`].
    pub fn with_payload(payload: Payload, pose: Pose, size: f32) -> Self {
        let payload = payload.truncated();
        Self {
            id: payload_id(&payload),
            payload,
            pose,
            size,
        }
    }
}

/// The markers one tracker of the headset sees.
#[derive(Clone, Debug, PartialEq)]
pub struct Stream {
    pub config: TrackingConfig,
    /// When the poses are for, see [`now_ns`].
    pub time_ns: i64,
    pub markers: Vec<Marker>,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Snapshot {
    pub flags: u32,
    /// Increases with every update from the driver.
    pub generation: u64,
    /// Of the headset connected last.
    pub capabilities: Capabilities,
    /// One per tracker the headset runs, each configuration supported by the headset.
    pub streams: Vec<Stream>,
}

/// A stable id for a marker text: FNV-1a, with 0 (reserved by OpenXR for "no entity") remapped.
pub fn marker_id(text: &str) -> u64 {
    fnv1a(0xcbf2_9ce4_8422_2325, text.as_bytes())
}

/// A stable id for a marker payload. Texts keep the ids of [`marker_id`]; bytes get other ones.
pub fn payload_id(payload: &Payload) -> u64 {
    match payload {
        Payload::Text(text) => marker_id(text),
        Payload::Bytes(bytes) => fnv1a(0x6c62_272e_07bb_0142, bytes),
    }
}

fn fnv1a(basis: u64, bytes: &[u8]) -> u64 {
    let hash = bytes.iter().fold(basis, |hash, &byte| {
        (hash ^ byte as u64).wrapping_mul(0x0100_0000_01b3)
    });

    hash.max(1)
}

fn truncate(text: &str) -> &str {
    if text.len() <= MAX_PAYLOAD_LEN {
        return text;
    }

    let mut end = MAX_PAYLOAD_LEN;
    while !text.is_char_boundary(end) {
        end -= 1;
    }

    &text[..end]
}

const CAPABILITY_QR_CODES: u32 = 1;
const CAPABILITY_FIXED_SIZE_MARKERS: u32 = 2;
const CAPABILITY_STATIC_MARKERS: u32 = 4;

const PAYLOAD_TEXT: u32 = 0;
const PAYLOAD_BYTES: u32 = 1;

const CONFIG_STATIC_MARKERS: u32 = 1;
const CONFIG_MARKER_SIZE: u32 = 2;

#[derive(Default)]
struct MessageWriter(Vec<u8>);

impl MessageWriter {
    fn u32(&mut self, value: u32) {
        self.0.extend(value.to_le_bytes());
    }

    fn u64(&mut self, value: u64) {
        self.0.extend(value.to_le_bytes());
    }

    fn i64(&mut self, value: i64) {
        self.0.extend(value.to_le_bytes());
    }

    fn f32(&mut self, value: f32) {
        self.0.extend(value.to_le_bytes());
    }

    fn config(&mut self, config: &TrackingConfig) {
        let mut flags = 0;
        if config.static_markers {
            flags |= CONFIG_STATIC_MARKERS;
        }
        if config.marker_size.is_some() {
            flags |= CONFIG_MARKER_SIZE;
        }
        self.u32(flags);
        self.f32(config.marker_size.unwrap_or(0.0));
    }
}

/// Reads little endian values off the front of a message.
struct MessageReader<'a>(&'a [u8]);

impl<'a> MessageReader<'a> {
    fn bytes(&mut self, len: usize) -> Option<&'a [u8]> {
        let (taken, rest) = self.0.split_at_checked(len)?;
        self.0 = rest;
        Some(taken)
    }

    fn array<const N: usize>(&mut self) -> Option<[u8; N]> {
        self.bytes(N)?.try_into().ok()
    }

    fn u32(&mut self) -> Option<u32> {
        Some(u32::from_le_bytes(self.array()?))
    }

    fn u64(&mut self) -> Option<u64> {
        Some(u64::from_le_bytes(self.array()?))
    }

    fn i64(&mut self) -> Option<i64> {
        Some(i64::from_le_bytes(self.array()?))
    }

    fn f32(&mut self) -> Option<f32> {
        Some(f32::from_le_bytes(self.array()?))
    }

    fn count(&mut self, max: usize) -> Option<usize> {
        let count = self.u32()? as usize;
        (count <= max).then_some(count)
    }

    fn config(&mut self) -> Option<TrackingConfig> {
        let flags = self.u32()?;
        let size = self.f32()?;
        Some(TrackingConfig {
            static_markers: flags & CONFIG_STATIC_MARKERS != 0,
            marker_size: (flags & CONFIG_MARKER_SIZE != 0).then_some(size),
        })
    }
}

fn encode_snapshot(snapshot: &Snapshot) -> Vec<u8> {
    let mut writer = MessageWriter::default();
    writer.u32(SNAPSHOT_MAGIC);
    writer.u32(PROTOCOL_VERSION);
    writer.u32(snapshot.flags);
    writer.u64(snapshot.generation);

    let capabilities = &snapshot.capabilities;
    writer.u32(
        (u32::from(capabilities.qr_codes) * CAPABILITY_QR_CODES)
            | (u32::from(capabilities.fixed_size_markers) * CAPABILITY_FIXED_SIZE_MARKERS)
            | (u32::from(capabilities.static_markers) * CAPABILITY_STATIC_MARKERS),
    );

    writer.u32(snapshot.streams.len() as u32);
    for stream in &snapshot.streams {
        writer.config(&stream.config);
        writer.i64(stream.time_ns);
        writer.u32(stream.markers.len() as u32);
        for marker in &stream.markers {
            writer.u64(marker.id);
            for &value in marker.pose.orientation.iter().chain(&marker.pose.position) {
                writer.f32(value);
            }
            writer.f32(marker.size);
            let bytes = marker.payload.as_bytes();
            writer.u32(match marker.payload {
                Payload::Text(_) => PAYLOAD_TEXT,
                Payload::Bytes(_) => PAYLOAD_BYTES,
            });
            writer.u32(bytes.len() as u32);
            writer.0.extend(bytes);
        }
    }

    writer.0
}

/// `None` for a message of another protocol version, or one that does not decode.
fn decode_snapshot(message: &[u8]) -> Option<Snapshot> {
    let mut reader = MessageReader(message);

    if reader.u32()? != SNAPSHOT_MAGIC || reader.u32()? != PROTOCOL_VERSION {
        return None;
    }
    let flags = reader.u32()?;
    let generation = reader.u64()?;

    let capability_bits = reader.u32()?;
    let capabilities = Capabilities {
        qr_codes: capability_bits & CAPABILITY_QR_CODES != 0,
        fixed_size_markers: capability_bits & CAPABILITY_FIXED_SIZE_MARKERS != 0,
        static_markers: capability_bits & CAPABILITY_STATIC_MARKERS != 0,
    };

    let stream_count = reader.count(MAX_STREAMS)?;
    let mut streams = Vec::with_capacity(stream_count);
    for _ in 0..stream_count {
        let config = reader.config()?;
        let time_ns = reader.i64()?;

        let marker_count = reader.count(MAX_MARKERS)?;
        let mut markers = Vec::with_capacity(marker_count);
        for _ in 0..marker_count {
            let id = reader.u64()?;
            let orientation = [reader.f32()?, reader.f32()?, reader.f32()?, reader.f32()?];
            let position = [reader.f32()?, reader.f32()?, reader.f32()?];
            let size = reader.f32()?;
            let kind = reader.u32()?;
            let len = reader.count(MAX_PAYLOAD_LEN)?;
            let bytes = reader.bytes(len)?.to_vec();
            let payload = match kind {
                PAYLOAD_TEXT => Payload::Text(String::from_utf8(bytes).ok()?),
                PAYLOAD_BYTES => Payload::Bytes(bytes),
                _ => return None,
            };

            markers.push(Marker {
                id,
                payload,
                pose: Pose {
                    orientation,
                    position,
                },
                size,
            });
        }

        streams.push(Stream {
            config,
            time_ns,
            markers,
        });
    }

    reader.0.is_empty().then_some(Snapshot {
        flags,
        generation,
        capabilities,
        streams,
    })
}

fn encode_request(configs: &[TrackingConfig]) -> Vec<u8> {
    let mut writer = MessageWriter::default();
    writer.u32(REQUEST_MAGIC);
    writer.u32(PROTOCOL_VERSION);
    writer.u32(configs.len() as u32);
    for config in configs {
        writer.config(config);
    }

    writer.0
}

/// `None` for a message of another protocol version, or one that does not decode.
fn decode_request(message: &[u8]) -> Option<Vec<TrackingConfig>> {
    let mut reader = MessageReader(message);

    if reader.u32()? != REQUEST_MAGIC || reader.u32()? != PROTOCOL_VERSION {
        return None;
    }
    let count = reader.count(MAX_STREAMS)?;
    let configs = (0..count)
        .map(|_| reader.config())
        .collect::<Option<Vec<_>>>()?;

    reader.0.is_empty().then_some(configs)
}

/// Adds the configurations of `configs` that `all` does not have yet.
fn merge_configs(all: &mut Vec<TrackingConfig>, configs: &[TrackingConfig]) {
    for config in configs {
        if !all.contains(config) {
            all.push(*config);
        }
    }
}

type RequestsCallback = dyn Fn(Vec<TrackingConfig>) + Send + Sync;

/// The driver's end: sends the markers to every connected layer, and collects what they ask for.
pub struct Server {
    pipe: sys::Server,
    snapshot: Snapshot,
}

impl Server {
    /// `on_requests` gets the tracker configurations all connected layers ask for, whenever they
    /// change, on a thread of the server.
    pub fn start(
        on_requests: impl Fn(Vec<TrackingConfig>) + Send + Sync + 'static,
    ) -> io::Result<Self> {
        Self::start_named(PIPE_NAME, on_requests)
    }

    pub fn start_named(
        name: &str,
        on_requests: impl Fn(Vec<TrackingConfig>) + Send + Sync + 'static,
    ) -> io::Result<Self> {
        let on_requests: Arc<RequestsCallback> = Arc::new(on_requests);
        // Per connected layer
        let requests = Mutex::new(HashMap::<u64, Vec<TrackingConfig>>::new());
        let last_reported = Mutex::new(Vec::<TrackingConfig>::new());

        let on_message = move |client: u64, message: Option<&[u8]>| {
            let mut requests = requests.lock().unwrap_or_else(|e| e.into_inner());
            match message {
                Some(message) => {
                    let Some(configs) = decode_request(message) else {
                        return;
                    };
                    requests.insert(client, configs);
                }
                None => {
                    requests.remove(&client);
                }
            }

            let mut all = Vec::new();
            for configs in requests.values() {
                merge_configs(&mut all, configs);
            }
            drop(requests);

            let mut last_reported = last_reported.lock().unwrap_or_else(|e| e.into_inner());
            let is_same = all.len() == last_reported.len()
                && all.iter().all(|config| last_reported.contains(config));
            if !is_same {
                *last_reported = all.clone();
                drop(last_reported);
                on_requests(all);
            }
        };

        let snapshot = Snapshot::default();
        let pipe = sys::Server::start(name, encode_snapshot(&snapshot), Box::new(on_message))?;

        Ok(Self { pipe, snapshot })
    }

    /// The generation of the latest snapshot sent.
    pub fn generation(&self) -> u64 {
        self.snapshot.generation
    }

    /// Replaces the whole state. Streams beyond [`MAX_STREAMS`] and markers beyond
    /// [`MAX_MARKERS`] are dropped.
    pub fn publish(&mut self, flags: u32, capabilities: Capabilities, mut streams: Vec<Stream>) {
        streams.truncate(MAX_STREAMS);
        for stream in &mut streams {
            stream.markers.truncate(MAX_MARKERS);
        }

        self.snapshot = Snapshot {
            flags,
            generation: self.snapshot.generation + 1,
            capabilities,
            streams,
        };

        self.pipe.send(encode_snapshot(&self.snapshot));
    }
}

/// The layer's end: connects to the driver, keeps the latest state it sent, and tells it what the
/// application asks for.
pub struct Client {
    name: String,
    connection: Option<sys::Connection>,
    last_connect_attempt: Option<Instant>,
    snapshot: Option<Snapshot>,
    /// Every snapshot since the last [`Client::take_received`], oldest first.
    received: VecDeque<Snapshot>,
    requests: Vec<TrackingConfig>,
    /// Whether `requests` reached the driver over the current connection.
    requests_sent: bool,
    /// How often to try connecting while the driver is not running.
    pub connect_retry_interval: Duration,
}

impl Client {
    pub fn new() -> Self {
        Self::new_named(PIPE_NAME)
    }

    pub fn new_named(name: &str) -> Self {
        Self {
            name: name.to_owned(),
            connection: None,
            last_connect_attempt: None,
            snapshot: None,
            received: VecDeque::new(),
            requests: Vec::new(),
            requests_sent: false,
            connect_retry_interval: CONNECT_RETRY_INTERVAL,
        }
    }

    pub fn is_connected(&self) -> bool {
        self.connection.is_some()
    }

    /// The latest state, or `None` while the driver is not running. Never blocks, except for
    /// waiting briefly for the first snapshot right after connecting.
    pub fn read(&mut self) -> Option<Snapshot> {
        if self.connection.is_none() {
            self.try_connect();
        }

        if self.receive().is_err() || self.send_requests().is_err() {
            self.disconnect();
        }

        self.snapshot.clone()
    }

    /// Every snapshot received since the last call, oldest first, so that no pose is missed.
    /// Up to the last few hundred.
    pub fn take_received(&mut self) -> Vec<Snapshot> {
        self.received.drain(..).collect()
    }

    /// The tracker configurations the application's spatial contexts ask for. Sent to the driver
    /// on the next [`Client::read`], and again after reconnecting.
    pub fn set_requests(&mut self, configs: &[TrackingConfig]) {
        let mut unique = Vec::new();
        merge_configs(&mut unique, configs);
        unique.truncate(MAX_STREAMS);

        if unique != self.requests {
            self.requests = unique;
            self.requests_sent = false;
        }
    }

    fn try_connect(&mut self) {
        if self
            .last_connect_attempt
            .is_some_and(|last| last.elapsed() < self.connect_retry_interval)
        {
            return;
        }
        self.last_connect_attempt = Some(Instant::now());

        let Ok(connection) = sys::Connection::connect(&self.name) else {
            return;
        };
        self.connection = Some(connection);
        self.requests_sent = false;

        // The driver sends its state right away; wait for it, so that the first read already
        // tells whether markers are available
        let start = Instant::now();
        while self.snapshot.is_none() && start.elapsed() < FIRST_SNAPSHOT_TIMEOUT {
            if self.receive().is_err() {
                self.disconnect();
                return;
            }
            if self.snapshot.is_none() {
                std::thread::sleep(Duration::from_millis(1));
            }
        }
    }

    fn disconnect(&mut self) {
        self.connection = None;
        self.snapshot = None;
    }

    /// Reads everything that has arrived and keeps the latest snapshot.
    fn receive(&mut self) -> io::Result<()> {
        let Some(connection) = &mut self.connection else {
            return Ok(());
        };

        for message in connection.read_available(MAX_MESSAGE_LEN)? {
            if let Some(snapshot) = decode_snapshot(&message) {
                if self.received.len() == MAX_RECEIVED {
                    self.received.pop_front();
                }
                self.received.push_back(snapshot.clone());
                self.snapshot = Some(snapshot);
            }
        }

        Ok(())
    }

    fn send_requests(&mut self) -> io::Result<()> {
        let Some(connection) = &mut self.connection else {
            return Ok(());
        };
        if self.requests_sent {
            return Ok(());
        }

        connection.write(&encode_request(&self.requests))?;
        self.requests_sent = true;

        Ok(())
    }
}

impl Default for Client {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pose(x: f32) -> Pose {
        Pose {
            orientation: [0.0, 0.0, 0.0, 1.0],
            position: [x, 1.0, -2.0],
        }
    }

    const STATIC: TrackingConfig = TrackingConfig {
        static_markers: true,
        marker_size: None,
    };
    const SIZED: TrackingConfig = TrackingConfig {
        static_markers: false,
        marker_size: Some(0.2),
    };
    const ALL: Capabilities = Capabilities {
        qr_codes: true,
        fixed_size_markers: true,
        static_markers: true,
    };

    fn stream(config: TrackingConfig, time_ns: i64, markers: &[(&str, f32)]) -> Stream {
        Stream {
            config,
            time_ns,
            markers: markers
                .iter()
                .map(|&(text, x)| Marker::new(text, pose(x), 0.2))
                .collect(),
        }
    }

    #[test]
    fn snapshots_round_trip() {
        let snapshot = Snapshot {
            flags: FLAG_CLIENT_CONNECTED,
            generation: 7,
            capabilities: Capabilities {
                qr_codes: true,
                fixed_size_markers: false,
                static_markers: true,
            },
            streams: vec![
                stream(STATIC, 123, &[("dog", 1.0)]),
                stream(SIZED, -5, &[("cat", 2.0), ("cow", 3.0)]),
            ],
        };

        assert_eq!(decode_snapshot(&encode_snapshot(&snapshot)), Some(snapshot));
    }

    #[test]
    fn requests_round_trip() {
        let configs = vec![STATIC, SIZED, TrackingConfig::default()];
        assert_eq!(decode_request(&encode_request(&configs)), Some(configs));
    }

    #[test]
    fn bad_messages_are_rejected() {
        let message = encode_snapshot(&Snapshot {
            streams: vec![stream(STATIC, 1, &[("dog", 1.0)])],
            ..Default::default()
        });

        assert!(decode_snapshot(&message[..message.len() - 1]).is_none());

        let mut extra = message.clone();
        extra.push(0);
        assert!(decode_snapshot(&extra).is_none());

        let mut other_version = message.clone();
        other_version[4] = other_version[4].wrapping_add(1);
        assert!(decode_snapshot(&other_version).is_none());

        // A request is not a snapshot, nor the other way round
        assert!(decode_snapshot(&encode_request(&[STATIC])).is_none());
        assert!(decode_request(&message).is_none());
    }

    #[test]
    fn unsupported_options_are_dropped() {
        let none = Capabilities {
            qr_codes: true,
            ..Default::default()
        };
        assert_eq!(SIZED.supported_by(&ALL), SIZED);
        assert_eq!(SIZED.supported_by(&none), TrackingConfig::default());
        assert_eq!(STATIC.supported_by(&none), TrackingConfig::default());
    }

    #[test]
    fn long_text_is_cut_at_a_character_boundary() {
        let text = "é".repeat(MAX_PAYLOAD_LEN);
        let cut = truncate(&text);
        assert_eq!(cut.len(), MAX_PAYLOAD_LEN - 1);
        assert!(text.starts_with(cut));
        assert_eq!(
            Marker::new(&text, pose(0.0), 0.1).payload,
            Payload::Text(cut.to_owned())
        );

        // The longest QR code fits
        let digits = "7".repeat(MAX_PAYLOAD_LEN);
        assert_eq!(
            Marker::new(&digits, pose(0.0), 0.1).payload,
            Payload::Text(digits)
        );
    }

    #[test]
    fn bytes_round_trip_and_have_their_own_ids() {
        let bytes = Marker::with_payload(Payload::Bytes(b"dog".to_vec()), pose(1.0), 0.2);
        assert_ne!(bytes.id, marker_id("dog"));

        let snapshot = Snapshot {
            streams: vec![Stream {
                config: STATIC,
                time_ns: 1,
                markers: vec![bytes, Marker::new("dog", pose(2.0), 0.2)],
            }],
            ..Default::default()
        };
        assert_eq!(decode_snapshot(&encode_snapshot(&snapshot)), Some(snapshot));
    }

    #[test]
    fn ids_are_stable_and_nonzero() {
        assert_eq!(marker_id("ALVR-QR-1"), marker_id("ALVR-QR-1"));
        assert_ne!(marker_id("ALVR-QR-1"), marker_id("ALVR-QR-2"));
        assert_ne!(marker_id(""), 0);
    }

    #[cfg(windows)]
    #[test]
    fn the_clock_advances_in_nanoseconds() {
        let start = now_ns();
        std::thread::sleep(Duration::from_millis(20));
        let elapsed = now_ns() - start;
        assert!((15_000_000..500_000_000).contains(&elapsed), "{elapsed}");
    }

    #[cfg(windows)]
    mod pipe {
        use super::*;
        use std::{sync::mpsc, thread};

        fn unique_name(test: &str) -> String {
            format!("ALVR_Markers_test_{test}_{}", std::process::id())
        }

        fn start(name: &str) -> Server {
            Server::start_named(name, |_| ()).unwrap()
        }

        fn wait_for(client: &mut Client, condition: impl Fn(&Option<Snapshot>) -> bool) {
            let start = Instant::now();
            loop {
                let snapshot = client.read();
                if condition(&snapshot) {
                    return;
                }
                assert!(start.elapsed() < Duration::from_secs(5), "{snapshot:?}");
                thread::sleep(Duration::from_millis(5));
            }
        }

        fn marker_count(snapshot: &Option<Snapshot>) -> usize {
            snapshot
                .as_ref()
                .map_or(0, |s| s.streams.iter().map(|s| s.markers.len()).sum())
        }

        #[test]
        fn clients_get_the_markers() {
            let name = unique_name("markers");
            let mut server = start(&name);

            // The first read already has the current state
            let mut client = Client::new_named(&name);
            assert_eq!(client.read(), Some(Snapshot::default()));

            server.publish(
                FLAG_CLIENT_CONNECTED,
                ALL,
                vec![
                    stream(STATIC, 10, &[("dog", 1.0)]),
                    stream(SIZED, 20, &[("cat", 2.0)]),
                ],
            );
            wait_for(&mut client, |snapshot| marker_count(snapshot) == 2);
            let snapshot = client.read().unwrap();
            assert_eq!(snapshot.flags, FLAG_CLIENT_CONNECTED);
            assert_eq!(snapshot.capabilities, ALL);
            assert_eq!(snapshot.streams[0].config, STATIC);
            assert_eq!(
                snapshot.streams[0].markers[0].payload,
                Payload::Text("dog".into())
            );
            assert_eq!(snapshot.streams[0].markers[0].id, marker_id("dog"));
            assert_eq!(snapshot.streams[1].time_ns, 20);
            assert_eq!(snapshot.streams[1].markers[0].pose, pose(2.0));

            // A client connecting later gets the same state at once
            let mut late = Client::new_named(&name);
            assert_eq!(late.read(), Some(snapshot));
        }

        #[test]
        fn no_snapshot_is_missed() {
            let name = unique_name("history");
            let mut server = start(&name);

            let mut client = Client::new_named(&name);
            client.read();
            client.take_received();

            for time in 1..=50 {
                server.publish(0, ALL, vec![stream(SIZED, time, &[("dog", 1.0)])]);
            }
            let mut times = Vec::new();
            let start = Instant::now();
            while times.last() != Some(&50) {
                assert!(start.elapsed() < Duration::from_secs(5), "{times:?}");
                client.read();
                times.extend(client.take_received().iter().map(|s| s.streams[0].time_ns));
                thread::sleep(Duration::from_millis(1));
            }

            // In order; a client that keeps up gets every one
            assert!(times.windows(2).all(|pair| pair[0] < pair[1]), "{times:?}");
        }

        #[test]
        fn the_server_gets_what_all_clients_ask_for() {
            let name = unique_name("requests");
            let (sender, receiver) = mpsc::channel();
            let _server = Server::start_named(&name, move |configs| {
                sender.send(configs).ok();
            })
            .unwrap();
            let next = || receiver.recv_timeout(Duration::from_secs(5)).unwrap();

            let mut first = Client::new_named(&name);
            first.set_requests(&[STATIC]);
            first.read();
            assert_eq!(next(), vec![STATIC]);

            let mut second = Client::new_named(&name);
            second.set_requests(&[STATIC, SIZED, SIZED]);
            second.read();
            let all = next();
            assert_eq!(all.len(), 2);
            assert!(all.contains(&STATIC) && all.contains(&SIZED));

            // A client that goes away takes its requests along
            drop(second);
            assert_eq!(next(), vec![STATIC]);

            first.set_requests(&[]);
            first.read();
            assert_eq!(next(), Vec::new());
        }

        #[test]
        fn requests_are_sent_again_after_reconnecting() {
            let name = unique_name("rerequest");
            let (sender, receiver) = mpsc::channel();
            let on_requests = move |configs| {
                sender.send(configs).ok();
            };
            let server = Server::start_named(&name, on_requests.clone()).unwrap();

            let mut client = Client::new_named(&name);
            client.connect_retry_interval = Duration::ZERO;
            client.set_requests(&[SIZED]);
            client.read();
            assert_eq!(
                receiver.recv_timeout(Duration::from_secs(5)).unwrap(),
                vec![SIZED]
            );

            // The stopping server lets go of its client
            drop(server);
            assert_eq!(
                receiver.recv_timeout(Duration::from_secs(5)).unwrap(),
                Vec::new()
            );
            wait_for(&mut client, Option::is_none);

            let _server = Server::start_named(&name, on_requests).unwrap();
            wait_for(&mut client, Option::is_some);
            assert_eq!(
                receiver.recv_timeout(Duration::from_secs(5)).unwrap(),
                vec![SIZED]
            );
        }

        #[test]
        fn clients_see_the_driver_go_and_come_back() {
            let name = unique_name("restart");
            let server = start(&name);

            let mut client = Client::new_named(&name);
            client.connect_retry_interval = Duration::ZERO;
            assert!(client.read().is_some());

            drop(server);
            wait_for(&mut client, Option::is_none);
            assert!(!client.is_connected());

            let mut server = start(&name);
            server.publish(0, ALL, vec![stream(STATIC, 1, &[("dog", 1.0)])]);
            wait_for(&mut client, |snapshot| marker_count(snapshot) == 1);
        }

        #[test]
        fn clients_connecting_at_once_all_get_the_markers() {
            let name = unique_name("crowd");
            let mut server = start(&name);
            server.publish(0, ALL, vec![stream(STATIC, 1, &[("dog", 1.0)])]);

            let clients = (0..16)
                .map(|_| {
                    let name = name.clone();
                    thread::spawn(move || {
                        let mut client = Client::new_named(&name);
                        client.connect_retry_interval = Duration::ZERO;
                        client.set_requests(&[SIZED]);
                        wait_for(&mut client, |snapshot| marker_count(snapshot) == 1);
                    })
                })
                .collect::<Vec<_>>();
            for client in clients {
                client.join().unwrap();
            }
        }

        #[test]
        fn a_second_server_cannot_take_the_name() {
            let name = unique_name("second");
            let _server = start(&name);
            assert!(Server::start_named(&name, |_| ()).is_err());
        }

        #[test]
        fn a_client_that_never_reads_does_not_block_the_server() {
            let name = unique_name("stalled");
            let mut server = start(&name);

            let mut stalled = Client::new_named(&name);
            stalled.read();

            let long_text = "x".repeat(MAX_PAYLOAD_LEN);
            for i in 0..200 {
                let markers = (0..MAX_MARKERS)
                    .map(|_| Marker::new(&long_text, pose(i as f32), 0.1))
                    .collect();
                server.publish(
                    0,
                    ALL,
                    vec![Stream {
                        config: STATIC,
                        time_ns: i,
                        markers,
                    }],
                );
            }

            // Others still get the latest markers, and the server shuts down
            let mut client = Client::new_named(&name);
            wait_for(&mut client, |snapshot| {
                snapshot.as_ref().is_some_and(|s| {
                    s.streams[0].markers.first().map(|m| m.pose) == Some(pose(199.0))
                })
            });
            drop(server);
        }
    }
}
