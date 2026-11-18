//! A named pipe, one way from the server to its clients.
//!
//! The server keeps the latest message. Each connected client has a thread of its own which writes
//! the latest message whenever it changes, so a client that stops reading only holds up its own
//! thread, and a client that falls behind skips straight to the latest message.

use std::{
    io, mem,
    os::windows::io::AsRawHandle,
    ptr,
    sync::{
        Arc, Condvar, Mutex, MutexGuard,
        atomic::{AtomicBool, Ordering},
    },
    thread::{self, JoinHandle},
    time::Duration,
};
use windows_sys::Win32::{
    Foundation::{
        CloseHandle, ERROR_PIPE_CONNECTED, GENERIC_READ, GetLastError, HANDLE,
        INVALID_HANDLE_VALUE, LocalFree,
    },
    Security::{
        Authorization::{ConvertStringSecurityDescriptorToSecurityDescriptorW, SDDL_REVISION_1},
        PSECURITY_DESCRIPTOR, SECURITY_ATTRIBUTES,
    },
    Storage::FileSystem::{
        CreateFileW, FILE_FLAG_FIRST_PIPE_INSTANCE, OPEN_EXISTING, PIPE_ACCESS_OUTBOUND, ReadFile,
        WriteFile,
    },
    System::{
        IO::CancelSynchronousIo,
        Pipes::{
            ConnectNamedPipe, CreateNamedPipeW, DisconnectNamedPipe, PIPE_READMODE_BYTE,
            PIPE_REJECT_REMOTE_CLIENTS, PIPE_TYPE_BYTE, PIPE_UNLIMITED_INSTANCES, PIPE_WAIT,
            PeekNamedPipe,
        },
    },
};

// Full access for SYSTEM, administrators and the owner (the user running SteamVR). Read access for
// everyone else and for AppContainers (ALL APPLICATION PACKAGES), as packaged OpenXR applications
// load the layer too. Clients only read, so they need no write access, which a low integrity
// process such as an AppContainer would not get anyway.
const SECURITY_DESCRIPTOR: &str = "D:(A;;GA;;;SY)(A;;GA;;;BA)(A;;GA;;;OW)(A;;FR;;;WD)(A;;FR;;;AC)";

const OUT_BUFFER_SIZE: u32 = 128 * 1024;
/// How long the server waits before trying again to create a pipe instance.
const RETRY_INTERVAL: Duration = Duration::from_secs(1);
/// How often a stopping server cancels the writes of clients that do not read.
const CANCEL_INTERVAL: Duration = Duration::from_millis(10);

fn wide(text: &str) -> Vec<u16> {
    text.encode_utf16().chain([0]).collect()
}

fn pipe_path(name: &str) -> Vec<u16> {
    wide(&format!(r"\\.\pipe\{name}"))
}

fn lock<T>(mutex: &Mutex<T>) -> MutexGuard<'_, T> {
    mutex.lock().unwrap_or_else(|e| e.into_inner())
}

struct OwnedHandle(HANDLE);

// A kernel handle can be used from any thread
unsafe impl Send for OwnedHandle {}

impl Drop for OwnedHandle {
    fn drop(&mut self) {
        unsafe { CloseHandle(self.0) };
    }
}

struct Shared {
    /// The latest message, and a version that increases with each one.
    latest: Mutex<(u64, Arc<Vec<u8>>)>,
    changed: Condvar,
    stopping: AtomicBool,
    writers: Mutex<Vec<JoinHandle<()>>>,
}

pub struct Server {
    shared: Arc<Shared>,
    path: Vec<u16>,
    listener: Option<JoinHandle<()>>,
}

impl Server {
    pub fn start(name: &str, message: Vec<u8>) -> io::Result<Self> {
        let path = pipe_path(name);

        // Created here so that a name already in use fails the start
        let first_instance = create_instance(&path, true)?;

        let shared = Arc::new(Shared {
            latest: Mutex::new((1, Arc::new(message))),
            changed: Condvar::new(),
            stopping: AtomicBool::new(false),
            writers: Mutex::new(Vec::new()),
        });

        let listener = thread::spawn({
            let shared = Arc::clone(&shared);
            let path = path.clone();
            move || listen(&shared, &path, first_instance)
        });

        Ok(Self {
            shared,
            path,
            listener: Some(listener),
        })
    }

    pub fn send(&self, message: Vec<u8>) {
        let mut latest = lock(&self.shared.latest);
        latest.0 += 1;
        latest.1 = Arc::new(message);
        self.shared.changed.notify_all();
    }
}

impl Drop for Server {
    fn drop(&mut self) {
        self.shared.stopping.store(true, Ordering::SeqCst);
        self.shared.changed.notify_all();

        // The listener waits for a client to connect, so connect to wake it
        drop(Connection::connect_path(&self.path));
        if let Some(listener) = self.listener.take() {
            listener.join().ok();
        }

        // A writer whose client does not read waits inside its write until that is cancelled
        let writers = mem::take(&mut *lock(&self.shared.writers));
        for writer in writers {
            while !writer.is_finished() {
                unsafe { CancelSynchronousIo(writer.as_raw_handle()) };
                self.shared.changed.notify_all();
                thread::sleep(CANCEL_INTERVAL);
            }
            writer.join().ok();
        }
    }
}

fn create_instance(path: &[u16], is_first: bool) -> io::Result<OwnedHandle> {
    let mut descriptor: PSECURITY_DESCRIPTOR = ptr::null_mut();
    if unsafe {
        ConvertStringSecurityDescriptorToSecurityDescriptorW(
            wide(SECURITY_DESCRIPTOR).as_ptr(),
            SDDL_REVISION_1,
            &mut descriptor,
            ptr::null_mut(),
        )
    } == 0
    {
        return Err(io::Error::last_os_error());
    }

    let attributes = SECURITY_ATTRIBUTES {
        nLength: mem::size_of::<SECURITY_ATTRIBUTES>() as u32,
        lpSecurityDescriptor: descriptor,
        bInheritHandle: 0,
    };
    // Only the first instance may create the pipe, so another server cannot take over its name
    let open_mode = if is_first {
        PIPE_ACCESS_OUTBOUND | FILE_FLAG_FIRST_PIPE_INSTANCE
    } else {
        PIPE_ACCESS_OUTBOUND
    };
    let handle = unsafe {
        CreateNamedPipeW(
            path.as_ptr(),
            open_mode,
            PIPE_TYPE_BYTE | PIPE_READMODE_BYTE | PIPE_WAIT | PIPE_REJECT_REMOTE_CLIENTS,
            PIPE_UNLIMITED_INSTANCES,
            OUT_BUFFER_SIZE,
            0,
            0,
            &attributes,
        )
    };
    let error = io::Error::last_os_error();
    unsafe { LocalFree(descriptor) };

    if handle == INVALID_HANDLE_VALUE {
        return Err(error);
    }

    Ok(OwnedHandle(handle))
}

/// Waits for clients, one pipe instance at a time, and starts a writer for each.
fn listen(shared: &Arc<Shared>, path: &[u16], first_instance: OwnedHandle) {
    let mut next_instance = Some(first_instance);

    while !shared.stopping.load(Ordering::SeqCst) {
        let instance = match next_instance.take() {
            Some(instance) => instance,
            None => match create_instance(path, false) {
                Ok(instance) => instance,
                Err(_) => {
                    thread::sleep(RETRY_INTERVAL);
                    continue;
                }
            },
        };

        let is_connected = unsafe { ConnectNamedPipe(instance.0, ptr::null_mut()) } != 0
            || unsafe { GetLastError() } == ERROR_PIPE_CONNECTED;
        if shared.stopping.load(Ordering::SeqCst) {
            break;
        }
        if !is_connected {
            continue;
        }

        let writer = thread::spawn({
            let shared = Arc::clone(shared);
            move || write_messages(&shared, instance)
        });

        let mut writers = lock(&shared.writers);
        writers.retain(|writer| !writer.is_finished());
        writers.push(writer);
    }
}

/// Sends the client the latest message, and then each new one, until it disconnects.
fn write_messages(shared: &Shared, pipe: OwnedHandle) {
    let mut sent_version = 0;

    loop {
        let message = {
            let mut latest = lock(&shared.latest);
            while latest.0 == sent_version && !shared.stopping.load(Ordering::SeqCst) {
                latest = shared
                    .changed
                    .wait(latest)
                    .unwrap_or_else(|e| e.into_inner());
            }
            if shared.stopping.load(Ordering::SeqCst) {
                break;
            }

            sent_version = latest.0;
            Arc::clone(&latest.1)
        };

        let mut written = 0;
        let is_written = unsafe {
            WriteFile(
                pipe.0,
                message.as_ptr(),
                message.len() as u32,
                &mut written,
                ptr::null_mut(),
            )
        } != 0;
        if !is_written {
            break;
        }
    }

    unsafe { DisconnectNamedPipe(pipe.0) };
}

pub struct Connection(OwnedHandle);

impl Connection {
    pub fn connect(name: &str) -> io::Result<Self> {
        Self::connect_path(&pipe_path(name))
    }

    fn connect_path(path: &[u16]) -> io::Result<Self> {
        let handle = unsafe {
            CreateFileW(
                path.as_ptr(),
                GENERIC_READ,
                0,
                ptr::null(),
                OPEN_EXISTING,
                0,
                ptr::null_mut(),
            )
        };
        if handle == INVALID_HANDLE_VALUE {
            return Err(io::Error::last_os_error());
        }

        Ok(Self(OwnedHandle(handle)))
    }

    /// Appends whatever has arrived to `buffer` without waiting for more.
    pub fn read_available(&mut self, buffer: &mut Vec<u8>) -> io::Result<()> {
        loop {
            let mut available = 0;
            if unsafe {
                PeekNamedPipe(
                    self.0.0,
                    ptr::null_mut(),
                    0,
                    ptr::null_mut(),
                    &mut available,
                    ptr::null_mut(),
                )
            } == 0
            {
                return Err(io::Error::last_os_error());
            }
            if available == 0 {
                return Ok(());
            }

            let start = buffer.len();
            buffer.resize(start + available as usize, 0);
            let mut read = 0;
            let is_read = unsafe {
                ReadFile(
                    self.0.0,
                    buffer[start..].as_mut_ptr(),
                    available,
                    &mut read,
                    ptr::null_mut(),
                )
            } != 0;
            buffer.truncate(start + read as usize);
            if !is_read {
                return Err(io::Error::last_os_error());
            }
        }
    }
}
