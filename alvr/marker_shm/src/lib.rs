//! Markers seen by the headset, shared by the ALVR driver with the ALVR OpenXR API layer.
//!
//! The driver, running inside vrserver.exe, is the only writer. The API layer is loaded into every
//! OpenXR application and maps the same memory read only, so reading markers costs no system call.
//!
//! The memory holds one versioned [`SharedState`]. A sequence counter guards it as a seqlock: the
//! writer makes the counter odd while it writes and even again when done, and a reader retries
//! until it copies the state between two equal, even reads of the counter. The writer also updates
//! a heartbeat a few times a second, outside the seqlock, so readers notice a writer that is gone
//! even though the memory stays alive while any reader keeps it mapped.
//!
//! Poses are in the space the driver submits the headset pose to SteamVR in, which is what
//! SteamVR's OpenXR runtime calls the STAGE space.

use std::{
    io, mem, ptr,
    sync::{
        Mutex,
        atomic::{AtomicU32, AtomicU64, Ordering, fence},
    },
    time::Duration,
};

#[cfg(windows)]
mod windows;
#[cfg(windows)]
use windows as sys;

#[cfg(not(windows))]
mod unsupported;
#[cfg(not(windows))]
use unsupported as sys;

pub const SHARED_MEMORY_NAME: &str = "ALVR_Markers";

/// Changes whenever the layout of [`SharedState`] or the meaning of a field changes. Readers
/// ignore memory with another version.
pub const PROTOCOL_VERSION: u32 = 1;
const MAGIC: u32 = u32::from_le_bytes(*b"ALVM");

pub const MAX_MARKERS: usize = 32;
/// Longer marker texts are cut short at a character boundary.
pub const MAX_TEXT_LEN: usize = 2048;

pub const HEARTBEAT_INTERVAL: Duration = Duration::from_millis(250);
/// Readers treat the writer as gone when its heartbeat is older than this.
pub const HEARTBEAT_TIMEOUT: Duration = Duration::from_secs(2);

/// A headset is connected.
pub const FLAG_CLIENT_CONNECTED: u32 = 1;

/// A seqlock read that keeps finding a write in progress gives up after this many attempts. Only
/// a writer that died halfway through a write can cause that.
const READ_ATTEMPTS: usize = 1000;

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

#[derive(Clone, Debug, PartialEq)]
pub struct Snapshot {
    pub flags: u32,
    /// Increases with every update from the writer.
    pub generation: u64,
    pub markers: Vec<Marker>,
}

#[repr(C)]
#[derive(Clone, Copy)]
struct SharedMarker {
    id: u64,
    orientation: [f32; 4],
    position: [f32; 3],
    size: f32,
    text_len: u32,
    reserved: u32,
    text: [u8; MAX_TEXT_LEN],
}

#[repr(C)]
struct SharedState {
    magic: u32,
    version: u32,
    sequence: AtomicU32,
    flags: u32,
    /// Milliseconds on the system's monotonic clock, shared by all processes. 0 once the writer
    /// has shut down.
    heartbeat_ms: AtomicU64,
    generation: u64,
    marker_count: u32,
    reserved: u32,
    markers: [SharedMarker; MAX_MARKERS],
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

pub struct Writer {
    mapping: sys::Mapping,
    // Serializes writers within this process; the seqlock only supports one at a time.
    write_lock: Mutex<()>,
}

// The mapping is only written through the seqlock under `write_lock`, and through atomics.
unsafe impl Send for Writer {}
unsafe impl Sync for Writer {}

impl Writer {
    pub fn create() -> io::Result<Self> {
        Self::create_named(SHARED_MEMORY_NAME)
    }

    pub fn create_named(name: &str) -> io::Result<Self> {
        let mapping = sys::Mapping::create(name, mem::size_of::<SharedState>())?;

        let this = Self {
            mapping,
            write_lock: Mutex::new(()),
        };

        // The memory may outlive a previous writer while readers keep it mapped. Its generation
        // is kept so that readers still see it increase.
        let previous_generation = unsafe {
            let state = this.state();
            if ptr::read_volatile(&raw const (*state).magic) == MAGIC
                && ptr::read_volatile(&raw const (*state).version) == PROTOCOL_VERSION
            {
                ptr::read_volatile(&raw const (*state).generation)
            } else {
                0
            }
        };
        this.write(|state| unsafe {
            ptr::write_volatile(&raw mut (*state).magic, MAGIC);
            ptr::write_volatile(&raw mut (*state).version, PROTOCOL_VERSION);
            ptr::write_volatile(&raw mut (*state).flags, 0);
            ptr::write_volatile(&raw mut (*state).generation, previous_generation + 1);
            ptr::write_volatile(&raw mut (*state).marker_count, 0);
        });
        this.heartbeat();

        Ok(this)
    }

    fn state(&self) -> *mut SharedState {
        self.mapping.ptr().cast()
    }

    fn write(&self, f: impl FnOnce(*mut SharedState)) {
        let _guard = self.write_lock.lock().unwrap_or_else(|e| e.into_inner());
        let state = self.state();
        let sequence = unsafe { &(*state).sequence };

        let start = sequence.load(Ordering::Relaxed);
        // An odd value left by a writer that died mid-write is completed rather than reused
        let start = start + (start & 1);
        sequence.store(start + 1, Ordering::Relaxed);
        fence(Ordering::Release);

        f(state);

        sequence.store(start + 2, Ordering::Release);
    }

    /// Tells readers the writer is alive. Call at least every [`HEARTBEAT_INTERVAL`].
    pub fn heartbeat(&self) {
        let state = self.state();
        unsafe { &(*state).heartbeat_ms }.store(sys::now_ms().max(1), Ordering::Release);
    }

    /// Replaces all markers. Markers beyond [`MAX_MARKERS`] are dropped.
    pub fn publish<'a>(&self, flags: u32, markers: impl IntoIterator<Item = (&'a str, Pose, f32)>) {
        self.write(|state| unsafe {
            let mut count = 0;
            for (text, pose, size) in markers.into_iter().take(MAX_MARKERS) {
                let text = truncate(text);
                let mut shared = SharedMarker {
                    id: marker_id(text),
                    orientation: pose.orientation,
                    position: pose.position,
                    size,
                    text_len: text.len() as u32,
                    reserved: 0,
                    text: [0; MAX_TEXT_LEN],
                };
                shared.text[..text.len()].copy_from_slice(text.as_bytes());

                ptr::write_volatile(&raw mut (*state).markers[count], shared);
                count += 1;
            }

            ptr::write_volatile(&raw mut (*state).flags, flags);
            ptr::write_volatile(&raw mut (*state).marker_count, count as u32);
            let generation = ptr::read_volatile(&raw const (*state).generation);
            ptr::write_volatile(&raw mut (*state).generation, generation + 1);
        });
    }
}

impl Drop for Writer {
    fn drop(&mut self) {
        self.publish(0, []);
        unsafe { &(*self.state()).heartbeat_ms }.store(0, Ordering::Release);
    }
}

pub struct Reader {
    mapping: sys::Mapping,
}

unsafe impl Send for Reader {}
unsafe impl Sync for Reader {}

impl Reader {
    pub fn open() -> io::Result<Self> {
        Self::open_named(SHARED_MEMORY_NAME)
    }

    pub fn open_named(name: &str) -> io::Result<Self> {
        let mapping = sys::Mapping::open(name, mem::size_of::<SharedState>())?;

        Ok(Self { mapping })
    }

    fn state(&self) -> *const SharedState {
        self.mapping.ptr().cast()
    }

    /// Whether the writer is running and speaks this protocol version.
    pub fn is_alive(&self) -> bool {
        let state = self.state();
        let (magic, version) = unsafe {
            (
                ptr::read_volatile(&raw const (*state).magic),
                ptr::read_volatile(&raw const (*state).version),
            )
        };
        if magic != MAGIC || version != PROTOCOL_VERSION {
            return false;
        }

        let heartbeat = unsafe { &(*state).heartbeat_ms }.load(Ordering::Acquire);
        heartbeat != 0
            && sys::now_ms().saturating_sub(heartbeat) < HEARTBEAT_TIMEOUT.as_millis() as u64
    }

    /// The current markers, or `None` while the writer is not alive.
    pub fn read(&self) -> Option<Snapshot> {
        if !self.is_alive() {
            return None;
        }

        let state = self.state();
        let sequence = unsafe { &(*state).sequence };

        for _ in 0..READ_ATTEMPTS {
            let start = sequence.load(Ordering::Acquire);
            if start & 1 != 0 {
                std::hint::spin_loop();
                continue;
            }

            let (flags, generation, count) = unsafe {
                (
                    ptr::read_volatile(&raw const (*state).flags),
                    ptr::read_volatile(&raw const (*state).generation),
                    ptr::read_volatile(&raw const (*state).marker_count) as usize,
                )
            };
            let shared = (0..count.min(MAX_MARKERS))
                .map(|i| unsafe { ptr::read_volatile(&raw const (*state).markers[i]) })
                .collect::<Vec<_>>();

            fence(Ordering::Acquire);
            if sequence.load(Ordering::Relaxed) != start {
                continue;
            }

            let markers = shared
                .iter()
                .map(|marker| {
                    let len = (marker.text_len as usize).min(MAX_TEXT_LEN);
                    Marker {
                        id: marker.id,
                        text: String::from_utf8_lossy(&marker.text[..len]).into_owned(),
                        pose: Pose {
                            orientation: marker.orientation,
                            position: marker.position,
                        },
                        size: marker.size,
                    }
                })
                .collect();

            return Some(Snapshot {
                flags,
                generation,
                markers,
            });
        }

        None
    }
}

#[cfg(all(test, windows))]
mod tests {
    use super::*;

    fn unique_name(test: &str) -> String {
        format!("ALVR_Markers_test_{test}_{}", std::process::id())
    }

    fn pose(x: f32) -> Pose {
        Pose {
            orientation: [0.0, 0.0, 0.0, 1.0],
            position: [x, 1.0, -2.0],
        }
    }

    #[test]
    fn round_trip() {
        let name = unique_name("round_trip");
        let writer = Writer::create_named(&name).unwrap();
        let reader = Reader::open_named(&name).unwrap();

        let empty = reader.read().unwrap();
        assert!(empty.markers.is_empty());

        writer.publish(
            FLAG_CLIENT_CONNECTED,
            [("dog", pose(1.0), 0.2), ("cat", pose(2.0), 0.1)],
        );
        let snapshot = reader.read().unwrap();
        assert_eq!(snapshot.flags, FLAG_CLIENT_CONNECTED);
        assert!(snapshot.generation > empty.generation);
        assert_eq!(snapshot.markers.len(), 2);
        assert_eq!(snapshot.markers[0].text, "dog");
        assert_eq!(snapshot.markers[0].id, marker_id("dog"));
        assert_eq!(snapshot.markers[1].pose, pose(2.0));
        assert_eq!(snapshot.markers[1].size, 0.1);

        drop(writer);
        assert!(!reader.is_alive());
        assert!(reader.read().is_none());
    }

    #[test]
    fn restarted_writer_keeps_generation_increasing() {
        let name = unique_name("restart");
        let writer = Writer::create_named(&name).unwrap();
        let reader = Reader::open_named(&name).unwrap();
        writer.publish(0, [("dog", pose(1.0), 0.2)]);
        let before = reader.read().unwrap().generation;
        drop(writer);

        // The reader keeps the memory alive, so the new writer opens the same memory
        let writer = Writer::create_named(&name).unwrap();
        let after = reader.read().unwrap();
        assert!(after.generation > before);
        assert!(after.markers.is_empty());
        drop(writer);
    }

    #[test]
    fn long_text_is_cut_at_a_character_boundary() {
        let name = unique_name("long_text");
        let writer = Writer::create_named(&name).unwrap();
        let reader = Reader::open_named(&name).unwrap();

        let text = "é".repeat(MAX_TEXT_LEN);
        writer.publish(0, [(text.as_str(), pose(0.0), 0.1)]);
        let read = &reader.read().unwrap().markers[0];
        assert_eq!(read.text.len(), MAX_TEXT_LEN);
        assert!(text.starts_with(&read.text));
    }

    #[test]
    fn concurrent_reads_are_never_torn() {
        let name = unique_name("concurrent");
        let writer = Writer::create_named(&name).unwrap();
        let reader = Reader::open_named(&name).unwrap();

        std::thread::scope(|scope| {
            scope.spawn(|| {
                for i in 0..20_000 {
                    let x = i as f32;
                    let text = format!("code-{i}");
                    writer.publish(0, [(text.as_str(), pose(x), x)]);
                }
            });

            for _ in 0..20_000 {
                if let Some(snapshot) = reader.read()
                    && let Some(marker) = snapshot.markers.first()
                {
                    // Every field of a marker comes from the same write
                    assert_eq!(marker.pose.position[0], marker.size);
                    assert_eq!(marker.text, format!("code-{}", marker.size as u32));
                    assert_eq!(marker.id, marker_id(&marker.text));
                }
            }
        });
    }

    #[test]
    fn ids_are_stable_and_nonzero() {
        assert_eq!(marker_id("ALVR-QR-1"), marker_id("ALVR-QR-1"));
        assert_ne!(marker_id("ALVR-QR-1"), marker_id("ALVR-QR-2"));
        assert_ne!(marker_id(""), 0);
    }
}
