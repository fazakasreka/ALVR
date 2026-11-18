//! Markers seen by the headset, sent by the ALVR driver to the ALVR OpenXR API layer.
//!
//! The driver, running inside vrserver.exe, serves a named pipe; the API layer, loaded into every
//! OpenXR application, connects to it. Data only flows from the driver to the layer: when a layer
//! connects, and whenever the markers change, the driver sends the complete current state, a
//! [`Snapshot`], as one frame. The layer reads whatever has arrived when it needs markers and keeps
//! the latest snapshot. A broken pipe means the driver is gone.
//!
//! A frame is a little endian `u32` byte length followed by the encoded snapshot.
//!
//! Poses are in the space the driver submits the headset pose to SteamVR in, which is what
//! SteamVR's OpenXR runtime calls the STAGE space.

use std::{
    io,
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

pub const PIPE_NAME: &str = "ALVR_Markers";

/// Changes whenever the encoding of a snapshot changes. Clients ignore snapshots of another
/// version.
pub const PROTOCOL_VERSION: u32 = 1;
const MAGIC: u32 = u32::from_le_bytes(*b"ALVM");

pub const MAX_MARKERS: usize = 32;
/// Longer marker texts are cut short at a character boundary.
pub const MAX_TEXT_LEN: usize = 2048;
/// Frames announcing more are treated as a broken connection.
const MAX_FRAME_LEN: usize = 1024 * 1024;

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

#[derive(Clone, Debug, PartialEq)]
pub struct Marker {
    /// Derived from the text, so a marker keeps its id across driver restarts. Never 0.
    pub id: u64,
    /// The decoded content of the QR code.
    pub text: String,
    /// Centre of the marker; +Y towards its top edge, +Z out of its face.
    pub pose: Pose,
    /// Edge length in metres.
    pub size: f32,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Snapshot {
    pub flags: u32,
    /// Increases with every update from the driver.
    pub generation: u64,
    pub markers: Vec<Marker>,
}

/// A stable id for a marker text: FNV-1a, with 0 (reserved by OpenXR for "no entity") remapped.
pub fn marker_id(text: &str) -> u64 {
    let hash = text.bytes().fold(0xcbf2_9ce4_8422_2325_u64, |hash, byte| {
        (hash ^ byte as u64).wrapping_mul(0x0100_0000_01b3)
    });

    hash.max(1)
}

fn truncate(text: &str) -> &str {
    if text.len() <= MAX_TEXT_LEN {
        return text;
    }

    let mut end = MAX_TEXT_LEN;
    while !text.is_char_boundary(end) {
        end -= 1;
    }

    &text[..end]
}

fn encode_frame(snapshot: &Snapshot) -> Vec<u8> {
    let mut payload = Vec::new();
    payload.extend(MAGIC.to_le_bytes());
    payload.extend(PROTOCOL_VERSION.to_le_bytes());
    payload.extend(snapshot.flags.to_le_bytes());
    payload.extend(snapshot.generation.to_le_bytes());
    payload.extend((snapshot.markers.len() as u32).to_le_bytes());
    for marker in &snapshot.markers {
        payload.extend(marker.id.to_le_bytes());
        for value in marker.pose.orientation.iter().chain(&marker.pose.position) {
            payload.extend(value.to_le_bytes());
        }
        payload.extend(marker.size.to_le_bytes());
        payload.extend((marker.text.len() as u32).to_le_bytes());
        payload.extend(marker.text.as_bytes());
    }

    let mut frame = (payload.len() as u32).to_le_bytes().to_vec();
    frame.extend(payload);

    frame
}

/// Reads little endian values off the front of a payload.
struct PayloadReader<'a>(&'a [u8]);

impl<'a> PayloadReader<'a> {
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

    fn f32(&mut self) -> Option<f32> {
        Some(f32::from_le_bytes(self.array()?))
    }
}

/// `None` for a payload of another protocol version, or one that does not decode.
fn decode_payload(payload: &[u8]) -> Option<Snapshot> {
    let mut reader = PayloadReader(payload);

    if reader.u32()? != MAGIC || reader.u32()? != PROTOCOL_VERSION {
        return None;
    }
    let flags = reader.u32()?;
    let generation = reader.u64()?;
    let count = reader.u32()? as usize;
    if count > MAX_MARKERS {
        return None;
    }

    let mut markers = Vec::with_capacity(count);
    for _ in 0..count {
        let id = reader.u64()?;
        let orientation = [reader.f32()?, reader.f32()?, reader.f32()?, reader.f32()?];
        let position = [reader.f32()?, reader.f32()?, reader.f32()?];
        let size = reader.f32()?;
        let text_len = reader.u32()? as usize;
        if text_len > MAX_TEXT_LEN {
            return None;
        }
        let text = String::from_utf8(reader.bytes(text_len)?.to_vec()).ok()?;

        markers.push(Marker {
            id,
            text,
            pose: Pose {
                orientation,
                position,
            },
            size,
        });
    }

    reader.0.is_empty().then_some(Snapshot {
        flags,
        generation,
        markers,
    })
}

/// The driver's end: sends the markers to every connected layer.
pub struct Server {
    pipe: sys::Server,
    snapshot: Snapshot,
}

impl Server {
    pub fn start() -> io::Result<Self> {
        Self::start_named(PIPE_NAME)
    }

    pub fn start_named(name: &str) -> io::Result<Self> {
        let snapshot = Snapshot::default();
        let pipe = sys::Server::start(name, encode_frame(&snapshot))?;

        Ok(Self { pipe, snapshot })
    }

    /// The generation of the latest snapshot sent.
    pub fn generation(&self) -> u64 {
        self.snapshot.generation
    }

    /// Replaces all markers. Markers beyond [`MAX_MARKERS`] are dropped.
    pub fn publish<'a>(
        &mut self,
        flags: u32,
        markers: impl IntoIterator<Item = (&'a str, Pose, f32)>,
    ) {
        self.snapshot = Snapshot {
            flags,
            generation: self.snapshot.generation + 1,
            markers: markers
                .into_iter()
                .take(MAX_MARKERS)
                .map(|(text, pose, size)| {
                    let text = truncate(text);
                    Marker {
                        id: marker_id(text),
                        text: text.to_owned(),
                        pose,
                        size,
                    }
                })
                .collect(),
        };

        self.pipe.send(encode_frame(&self.snapshot));
    }
}

/// The layer's end: connects to the driver and keeps the latest markers it sent.
pub struct Client {
    name: String,
    connection: Option<sys::Connection>,
    last_connect_attempt: Option<Instant>,
    received: Vec<u8>,
    snapshot: Option<Snapshot>,
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
            received: Vec::new(),
            snapshot: None,
            connect_retry_interval: CONNECT_RETRY_INTERVAL,
        }
    }

    pub fn is_connected(&self) -> bool {
        self.connection.is_some()
    }

    /// The latest markers, or `None` while the driver is not running. Never blocks, except for
    /// waiting briefly for the first snapshot right after connecting.
    pub fn read(&mut self) -> Option<Snapshot> {
        if self.connection.is_none() {
            self.try_connect();
        }

        if let Err(_e) = self.receive() {
            self.disconnect();
        }

        self.snapshot.clone()
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

        // The driver sends its markers right away; wait for them, so that the first read already
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
        self.received.clear();
        self.snapshot = None;
    }

    /// Reads everything that has arrived and keeps the last complete snapshot.
    fn receive(&mut self) -> io::Result<()> {
        let Some(connection) = &mut self.connection else {
            return Ok(());
        };
        connection.read_available(&mut self.received)?;

        loop {
            let Some(header) = self.received.get(..4) else {
                return Ok(());
            };
            let len = u32::from_le_bytes(header.try_into().unwrap()) as usize;
            if len > MAX_FRAME_LEN {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidData,
                    "frame too large",
                ));
            }
            if self.received.len() < 4 + len {
                return Ok(());
            }

            let frame = self.received.drain(..4 + len).collect::<Vec<_>>();
            if let Some(snapshot) = decode_payload(&frame[4..]) {
                self.snapshot = Some(snapshot);
            }
        }
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

    #[test]
    fn frames_round_trip() {
        let snapshot = Snapshot {
            flags: FLAG_CLIENT_CONNECTED,
            generation: 7,
            markers: vec![Marker {
                id: marker_id("dog"),
                text: "dog".into(),
                pose: pose(1.0),
                size: 0.2,
            }],
        };

        let frame = encode_frame(&snapshot);
        assert_eq!(
            u32::from_le_bytes(frame[..4].try_into().unwrap()) as usize,
            frame.len() - 4
        );
        assert_eq!(decode_payload(&frame[4..]), Some(snapshot));
    }

    #[test]
    fn bad_payloads_are_rejected() {
        let frame = encode_frame(&Snapshot::default());
        let payload = &frame[4..];

        assert!(decode_payload(&payload[..payload.len() - 1]).is_none());

        let mut extra = payload.to_vec();
        extra.push(0);
        assert!(decode_payload(&extra).is_none());

        let mut other_version = payload.to_vec();
        other_version[4] = other_version[4].wrapping_add(1);
        assert!(decode_payload(&other_version).is_none());
    }

    #[test]
    fn long_text_is_cut_at_a_character_boundary() {
        let text = "é".repeat(MAX_TEXT_LEN);
        let cut = truncate(&text);
        assert_eq!(cut.len(), MAX_TEXT_LEN);
        assert!(text.starts_with(cut));
    }

    #[test]
    fn ids_are_stable_and_nonzero() {
        assert_eq!(marker_id("ALVR-QR-1"), marker_id("ALVR-QR-1"));
        assert_ne!(marker_id("ALVR-QR-1"), marker_id("ALVR-QR-2"));
        assert_ne!(marker_id(""), 0);
    }

    #[cfg(windows)]
    mod pipe {
        use super::*;
        use std::thread;

        fn unique_name(test: &str) -> String {
            format!("ALVR_Markers_test_{test}_{}", std::process::id())
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

        #[test]
        fn clients_get_the_markers() {
            let name = unique_name("markers");
            let mut server = Server::start_named(&name).unwrap();

            // The first read already has the current state
            let mut client = Client::new_named(&name);
            assert_eq!(client.read(), Some(Snapshot::default()));

            server.publish(
                FLAG_CLIENT_CONNECTED,
                [("dog", pose(1.0), 0.2), ("cat", pose(2.0), 0.1)],
            );
            wait_for(&mut client, |snapshot| {
                snapshot.as_ref().is_some_and(|s| s.markers.len() == 2)
            });
            let snapshot = client.read().unwrap();
            assert_eq!(snapshot.flags, FLAG_CLIENT_CONNECTED);
            assert_eq!(snapshot.markers[0].text, "dog");
            assert_eq!(snapshot.markers[0].id, marker_id("dog"));
            assert_eq!(snapshot.markers[1].pose, pose(2.0));

            // A client connecting later gets the same state at once
            let mut late = Client::new_named(&name);
            assert_eq!(late.read(), Some(snapshot));
        }

        #[test]
        fn clients_see_the_driver_go_and_come_back() {
            let name = unique_name("restart");
            let server = Server::start_named(&name).unwrap();

            let mut client = Client::new_named(&name);
            client.connect_retry_interval = Duration::ZERO;
            assert!(client.read().is_some());

            drop(server);
            wait_for(&mut client, Option::is_none);
            assert!(!client.is_connected());

            let mut server = Server::start_named(&name).unwrap();
            server.publish(0, [("dog", pose(1.0), 0.2)]);
            wait_for(&mut client, |snapshot| {
                snapshot.as_ref().is_some_and(|s| s.markers.len() == 1)
            });
        }

        #[test]
        fn a_second_server_cannot_take_the_name() {
            let name = unique_name("second");
            let _server = Server::start_named(&name).unwrap();
            assert!(Server::start_named(&name).is_err());
        }

        #[test]
        fn a_client_that_never_reads_does_not_block_the_server() {
            let name = unique_name("stalled");
            let mut server = Server::start_named(&name).unwrap();

            let mut stalled = Client::new_named(&name);
            stalled.read();

            let long_text = "x".repeat(MAX_TEXT_LEN);
            for i in 0..200 {
                let markers = (0..MAX_MARKERS)
                    .map(|_| (long_text.as_str(), pose(i as f32), 0.1))
                    .collect::<Vec<_>>();
                server.publish(0, markers);
            }

            // Others still get the latest markers, and the server shuts down
            let mut client = Client::new_named(&name);
            wait_for(&mut client, |snapshot| {
                snapshot
                    .as_ref()
                    .is_some_and(|s| s.markers.first().map(|m| m.pose) == Some(pose(199.0)))
            });
            drop(server);
        }
    }
}
