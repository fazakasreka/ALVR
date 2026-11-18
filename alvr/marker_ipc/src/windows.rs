//! A message mode named pipe between a server and its clients.
//!
//! The server keeps the latest message. Each connected client has a writer thread of its own,
//! which writes the latest message whenever it changes, so a client that stops reading only holds
//! up its own thread, and a client that falls behind skips straight to the latest message. A
//! reader thread per client hands what the client sends to the server's callback. The server's
//! waits are overlapped and end when it stops.
//!
//! Only the user running the server can open the pipe, from any process of theirs including
//! packaged (AppContainer) ones, and clients only trust a pipe created by that same user.

use std::{
    io, mem, ptr,
    sync::{
        Arc, Condvar, LazyLock, Mutex, MutexGuard,
        atomic::{AtomicU64, Ordering},
    },
    thread::{self, JoinHandle},
    time::Duration,
};
use windows_sys::Win32::{
    Foundation::{
        CloseHandle, ERROR_IO_PENDING, ERROR_MORE_DATA, ERROR_OPERATION_ABORTED, ERROR_PIPE_BUSY,
        ERROR_PIPE_CONNECTED, ERROR_SUCCESS, GENERIC_READ, GENERIC_WRITE, GetLastError, HANDLE,
        INVALID_HANDLE_VALUE, LocalFree, WAIT_OBJECT_0,
    },
    Security::{
        Authorization::{
            ConvertSidToStringSidW, ConvertStringSecurityDescriptorToSecurityDescriptorW,
            GetSecurityInfo, SDDL_REVISION_1, SE_KERNEL_OBJECT,
        },
        EqualSid, GetTokenInformation, OWNER_SECURITY_INFORMATION, PSECURITY_DESCRIPTOR, PSID,
        SECURITY_ATTRIBUTES, TOKEN_QUERY, TOKEN_USER, TokenUser,
    },
    Storage::FileSystem::{
        CreateFileW, FILE_FLAG_FIRST_PIPE_INSTANCE, FILE_FLAG_OVERLAPPED, OPEN_EXISTING,
        PIPE_ACCESS_DUPLEX, ReadFile, SECURITY_IDENTIFICATION, SECURITY_SQOS_PRESENT, WriteFile,
    },
    System::{
        IO::{CancelIoEx, GetOverlappedResult, OVERLAPPED},
        Performance::{QueryPerformanceCounter, QueryPerformanceFrequency},
        Pipes::{
            ConnectNamedPipe, CreateNamedPipeW, DisconnectNamedPipe, PIPE_READMODE_MESSAGE,
            PIPE_REJECT_REMOTE_CLIENTS, PIPE_TYPE_MESSAGE, PIPE_UNLIMITED_INSTANCES, PIPE_WAIT,
            PeekNamedPipe, SetNamedPipeHandleState, WaitNamedPipeW,
        },
        Threading::{
            CreateEventW, GetCurrentProcess, INFINITE, OpenProcessToken, ResetEvent, SetEvent,
            WaitForMultipleObjects, WaitForSingleObject,
        },
    },
};

const PIPE_BUFFER_SIZE: u32 = 64 * 1024;
/// How long the server waits before trying again to create a pipe instance.
const RETRY_INTERVAL: Duration = Duration::from_secs(1);
/// How long a client waits for a free instance. Every instance is busy while the server is
/// between accepting a client and creating the next instance, which takes moments; clients read
/// on the application's frame loop, so they do not wait longer.
const BUSY_PIPE_TIMEOUT: Duration = Duration::from_millis(100);

/// What a client asks for: reading, writing, and setting its handle to message read mode.
const CLIENT_ACCESS: u32 = GENERIC_READ | GENERIC_WRITE;
/// FILE_GENERIC_READ | FILE_GENERIC_WRITE, granted to AppContainers on top of their user's access,
/// as packaged OpenXR applications load the layer too.
const APP_CONTAINER_ACCESS: &str = "0x12019f";

static PERFORMANCE_FREQUENCY: LazyLock<i64> = LazyLock::new(|| {
    let mut frequency = 0;
    unsafe { QueryPerformanceFrequency(&mut frequency) };
    frequency.max(1)
});

/// Nanoseconds on the performance counter clock, which is what
/// XR_KHR_win32_convert_performance_counter_time converts OpenXR times to.
pub fn now_ns() -> i64 {
    let mut counter = 0;
    unsafe { QueryPerformanceCounter(&mut counter) };
    performance_counter_to_ns(counter)
}

pub fn performance_counter_to_ns(counter: i64) -> i64 {
    (counter as i128 * 1_000_000_000 / *PERFORMANCE_FREQUENCY as i128) as i64
}

pub fn ns_to_performance_counter(ns: i64) -> i64 {
    (ns as i128 * *PERFORMANCE_FREQUENCY as i128 / 1_000_000_000) as i64
}

fn wide(text: &str) -> Vec<u16> {
    text.encode_utf16().chain([0]).collect()
}

fn pipe_path(name: &str) -> Vec<u16> {
    wide(&format!(r"\\.\pipe\{name}"))
}

fn lock<T>(mutex: &Mutex<T>) -> MutexGuard<'_, T> {
    mutex.lock().unwrap_or_else(|e| e.into_inner())
}

fn check(succeeded: i32) -> io::Result<()> {
    if succeeded != 0 {
        Ok(())
    } else {
        Err(io::Error::last_os_error())
    }
}

struct OwnedHandle(HANDLE);

// A kernel handle can be used from any thread
unsafe impl Send for OwnedHandle {}
unsafe impl Sync for OwnedHandle {}

impl Drop for OwnedHandle {
    fn drop(&mut self) {
        unsafe { CloseHandle(self.0) };
    }
}

struct ManualResetEvent(OwnedHandle);

impl ManualResetEvent {
    fn new() -> io::Result<Self> {
        let handle = unsafe { CreateEventW(ptr::null(), 1, 0, ptr::null()) };
        if handle.is_null() {
            return Err(io::Error::last_os_error());
        }

        Ok(Self(OwnedHandle(handle)))
    }

    fn set(&self) {
        unsafe { SetEvent(self.0.0) };
    }

    fn reset(&self) {
        unsafe { ResetEvent(self.0.0) };
    }

    fn is_set(&self) -> bool {
        unsafe { WaitForSingleObject(self.0.0, 0) == WAIT_OBJECT_0 }
    }

    /// Whether the event was set within `timeout`.
    fn wait(&self, timeout: Duration) -> bool {
        unsafe { WaitForSingleObject(self.0.0, timeout.as_millis() as u32) == WAIT_OBJECT_0 }
    }
}

/// One overlapped operation at a time, which a stop event can cancel.
struct OverlappedOperation {
    event: ManualResetEvent,
    // Boxed: the system writes to it until the operation finishes
    overlapped: Box<OVERLAPPED>,
}

impl OverlappedOperation {
    fn new() -> io::Result<Self> {
        Ok(Self {
            event: ManualResetEvent::new()?,
            overlapped: Box::new(unsafe { mem::zeroed() }),
        })
    }

    /// The OVERLAPPED structure for the next operation.
    fn start(&mut self) -> *mut OVERLAPPED {
        self.event.reset();
        *self.overlapped = unsafe { mem::zeroed() };
        self.overlapped.hEvent = self.event.0.0;

        &raw mut *self.overlapped
    }

    /// Waits for the operation `has_started` (the result of the call that started it) on
    /// `handle` to finish, cancelling it if `stop` is set first. Returns the error code and the
    /// number of bytes transferred.
    fn wait(&mut self, handle: HANDLE, has_started: bool, stop: &ManualResetEvent) -> (u32, u32) {
        // Failed immediately, except for an operation which is running
        if !has_started {
            let error = unsafe { GetLastError() };
            if error != ERROR_IO_PENDING && error != ERROR_MORE_DATA {
                return (error, 0);
            }
        }

        let events = [self.event.0.0, stop.0.0];
        if unsafe { WaitForMultipleObjects(2, events.as_ptr(), 0, INFINITE) } != WAIT_OBJECT_0 {
            unsafe { CancelIoEx(handle, &*self.overlapped) };
        }

        // The operation must finish, also when cancelled, before the OVERLAPPED may be reused
        let mut transferred = 0;
        let error = if unsafe {
            GetOverlappedResult(handle, &*self.overlapped, &mut transferred, 1)
        } != 0
        {
            ERROR_SUCCESS
        } else {
            unsafe { GetLastError() }
        };

        (error, transferred)
    }
}

/// The SID of the user running this process. In an AppContainer, that is still the user.
// A TOKEN_USER, in u64s for its alignment
struct CurrentUser(Vec<u64>);

impl CurrentUser {
    fn get() -> io::Result<Self> {
        let mut token = ptr::null_mut();
        check(unsafe { OpenProcessToken(GetCurrentProcess(), TOKEN_QUERY, &mut token) })?;
        let token = OwnedHandle(token);

        let mut size = 0;
        unsafe { GetTokenInformation(token.0, TokenUser, ptr::null_mut(), 0, &mut size) };
        let mut buffer = vec![0u64; (size as usize).div_ceil(size_of::<u64>())];
        check(unsafe {
            GetTokenInformation(
                token.0,
                TokenUser,
                buffer.as_mut_ptr().cast(),
                size,
                &mut size,
            )
        })?;

        Ok(Self(buffer))
    }

    fn sid(&self) -> PSID {
        unsafe { (*self.0.as_ptr().cast::<TOKEN_USER>()).User.Sid }
    }

    fn sid_string(&self) -> io::Result<String> {
        let mut text = ptr::null_mut();
        check(unsafe { ConvertSidToStringSidW(self.sid(), &mut text) })?;

        let len = (0..).take_while(|&i| unsafe { *text.add(i) } != 0).count();
        let result = String::from_utf16_lossy(unsafe { std::slice::from_raw_parts(text, len) });
        unsafe { LocalFree(text.cast()) };

        Ok(result)
    }
}

/// Security attributes for the pipe: owned by and fully accessible to the current user, and
/// nobody else, with inheritance from the parent blocked.
struct UserSecurity {
    descriptor: PSECURITY_DESCRIPTOR,
    attributes: SECURITY_ATTRIBUTES,
}

// Only read after construction
unsafe impl Send for UserSecurity {}
unsafe impl Sync for UserSecurity {}

impl UserSecurity {
    fn new() -> io::Result<Self> {
        let user = CurrentUser::get()?.sid_string()?;
        // The owner is set explicitly, as an elevated process would make the administrators
        // group the owner, which clients would not trust. An AppContainer is only granted what
        // both its user and the AppContainer entry allow, and runs at low integrity, so the low
        // label lets it write, and set its message read mode
        let sddl =
            format!("O:{user}D:P(A;;GA;;;{user})(A;;{APP_CONTAINER_ACCESS};;;AC)S:(ML;;NW;;;LW)");

        let mut descriptor = ptr::null_mut();
        check(unsafe {
            ConvertStringSecurityDescriptorToSecurityDescriptorW(
                wide(&sddl).as_ptr(),
                SDDL_REVISION_1,
                &mut descriptor,
                ptr::null_mut(),
            )
        })?;

        Ok(Self {
            descriptor,
            attributes: SECURITY_ATTRIBUTES {
                nLength: mem::size_of::<SECURITY_ATTRIBUTES>() as u32,
                lpSecurityDescriptor: descriptor,
                bInheritHandle: 0,
            },
        })
    }
}

impl Drop for UserSecurity {
    fn drop(&mut self) {
        unsafe { LocalFree(self.descriptor) };
    }
}

fn is_owned_by_current_user(handle: HANDLE) -> io::Result<bool> {
    let mut owner: PSID = ptr::null_mut();
    let mut descriptor: PSECURITY_DESCRIPTOR = ptr::null_mut();
    let status = unsafe {
        GetSecurityInfo(
            handle,
            SE_KERNEL_OBJECT,
            OWNER_SECURITY_INFORMATION,
            &mut owner,
            ptr::null_mut(),
            ptr::null_mut(),
            ptr::null_mut(),
            &mut descriptor,
        )
    };
    if status != ERROR_SUCCESS {
        return Err(io::Error::from_raw_os_error(status as i32));
    }

    let result = CurrentUser::get().map(|user| unsafe { EqualSid(owner, user.sid()) } != 0);
    unsafe { LocalFree(descriptor) };

    result
}

/// Gets each message a client sends, and `None` when it disconnects.
pub type MessageCallback = Box<dyn Fn(u64, Option<&[u8]>) + Send + Sync>;

struct Shared {
    /// The latest message, and a version that increases with each one.
    latest: Mutex<(u64, Arc<Vec<u8>>)>,
    changed: Condvar,
    stop: ManualResetEvent,
    on_message: MessageCallback,
    next_client: AtomicU64,
    /// The writers and readers of the clients.
    threads: Mutex<Vec<JoinHandle<()>>>,
}

pub struct Server {
    shared: Arc<Shared>,
    listener: Option<JoinHandle<()>>,
}

impl Server {
    pub fn start(name: &str, message: Vec<u8>, on_message: MessageCallback) -> io::Result<Self> {
        let path = pipe_path(name);
        let security = UserSecurity::new()?;

        // Created here so that a name already in use fails the start
        let first_instance = create_instance(&path, &security, true)?;

        let shared = Arc::new(Shared {
            latest: Mutex::new((1, Arc::new(message))),
            changed: Condvar::new(),
            stop: ManualResetEvent::new()?,
            on_message,
            next_client: AtomicU64::new(0),
            threads: Mutex::new(Vec::new()),
        });

        let listener = thread::spawn({
            let shared = Arc::clone(&shared);
            move || accept_clients(&shared, &path, &security, first_instance)
        });

        Ok(Self {
            shared,
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
        // Ends the overlapped waits of the listener and the writers
        self.shared.stop.set();
        {
            // Under the lock, so that no writer misses it between checking and waiting
            let _latest = lock(&self.shared.latest);
            self.shared.changed.notify_all();
        }

        if let Some(listener) = self.listener.take() {
            listener.join().ok();
        }
        for thread in mem::take(&mut *lock(&self.shared.threads)) {
            thread.join().ok();
        }
    }
}

fn create_instance(
    path: &[u16],
    security: &UserSecurity,
    is_first: bool,
) -> io::Result<OwnedHandle> {
    // Only the first instance may create the pipe, so that another process cannot take over its
    // name by starting first
    let first_flag = if is_first {
        FILE_FLAG_FIRST_PIPE_INSTANCE
    } else {
        0
    };
    let handle = unsafe {
        CreateNamedPipeW(
            path.as_ptr(),
            PIPE_ACCESS_DUPLEX | FILE_FLAG_OVERLAPPED | first_flag,
            PIPE_TYPE_MESSAGE | PIPE_READMODE_MESSAGE | PIPE_WAIT | PIPE_REJECT_REMOTE_CLIENTS,
            PIPE_UNLIMITED_INSTANCES,
            PIPE_BUFFER_SIZE,
            PIPE_BUFFER_SIZE,
            0,
            &security.attributes,
        )
    };
    if handle == INVALID_HANDLE_VALUE {
        return Err(io::Error::last_os_error());
    }

    Ok(OwnedHandle(handle))
}

/// Waits for clients, one pipe instance at a time, and starts a writer and a reader for each.
fn accept_clients(
    shared: &Arc<Shared>,
    path: &[u16],
    security: &UserSecurity,
    first_instance: OwnedHandle,
) {
    let Ok(mut operation) = OverlappedOperation::new() else {
        return;
    };
    let mut waiting_instance = Some(first_instance);

    while !shared.stop.is_set() {
        // Create the instance for the next client: the previous one went to its threads, or was
        // dropped after a failure
        let instance = match waiting_instance.take() {
            Some(instance) => instance,
            None => match create_instance(path, security, false) {
                Ok(instance) => instance,
                Err(_) => {
                    shared.stop.wait(RETRY_INTERVAL);
                    continue;
                }
            },
        };

        // Wait for a client on the waiting instance
        let has_started = unsafe { ConnectNamedPipe(instance.0, operation.start()) } != 0;
        let (error, _) = operation.wait(instance.0, has_started, &shared.stop);
        if error == ERROR_OPERATION_ABORTED {
            break;
        }

        // A client which connected before ConnectNamedPipe was called is reported as
        // ERROR_PIPE_CONNECTED
        if error == ERROR_SUCCESS || error == ERROR_PIPE_CONNECTED {
            let pipe = Arc::new(instance);
            let client = shared.next_client.fetch_add(1, Ordering::Relaxed);
            let writer = thread::spawn({
                let shared = Arc::clone(shared);
                let pipe = Arc::clone(&pipe);
                move || send_messages(&shared, &pipe)
            });
            let reader = thread::spawn({
                let shared = Arc::clone(shared);
                move || receive_messages(&shared, &pipe, client)
            });

            let mut threads = lock(&shared.threads);
            threads.retain(|thread| !thread.is_finished());
            threads.extend([writer, reader]);
        }
    }
}

/// Sends the client the latest message, and then each new one, until it disconnects or the
/// server stops.
fn send_messages(shared: &Shared, pipe: &OwnedHandle) {
    let Ok(mut operation) = OverlappedOperation::new() else {
        return;
    };
    let mut sent_version = 0;

    loop {
        let message = {
            let mut latest = lock(&shared.latest);
            while latest.0 == sent_version && !shared.stop.is_set() {
                latest = shared
                    .changed
                    .wait(latest)
                    .unwrap_or_else(|e| e.into_inner());
            }
            if shared.stop.is_set() {
                break;
            }

            sent_version = latest.0;
            Arc::clone(&latest.1)
        };

        let has_started = unsafe {
            WriteFile(
                pipe.0,
                message.as_ptr(),
                message.len() as u32,
                ptr::null_mut(),
                operation.start(),
            )
        } != 0;
        let (error, written) = operation.wait(pipe.0, has_started, &shared.stop);
        if error != ERROR_SUCCESS || written as usize != message.len() {
            break;
        }
    }

    // Also ends the reader's wait
    unsafe { DisconnectNamedPipe(pipe.0) };
}

/// Hands each message the client sends to the callback, until it disconnects or the server
/// stops.
fn receive_messages(shared: &Shared, pipe: &OwnedHandle, client: u64) {
    let Ok(mut operation) = OverlappedOperation::new() else {
        return;
    };
    let mut buffer = vec![0u8; PIPE_BUFFER_SIZE as usize];
    let mut message = Vec::new();

    loop {
        let has_started = unsafe {
            ReadFile(
                pipe.0,
                buffer.as_mut_ptr(),
                buffer.len() as u32,
                ptr::null_mut(),
                operation.start(),
            )
        } != 0;
        let (error, read) = operation.wait(pipe.0, has_started, &shared.stop);
        if error != ERROR_SUCCESS && error != ERROR_MORE_DATA {
            break;
        }

        // A message larger than the buffer arrives in parts, all but the last one with
        // ERROR_MORE_DATA
        message.extend_from_slice(&buffer[..read as usize]);
        if error == ERROR_MORE_DATA {
            continue;
        }

        (shared.on_message)(client, Some(&message));
        message.clear();
    }

    (shared.on_message)(client, None);
    // Also ends the writer's wait at its next message
    unsafe { DisconnectNamedPipe(pipe.0) };
}

/// The client end. Synchronous, and only ever reads what has already arrived, so it never waits
/// on the server.
pub struct Connection(OwnedHandle);

impl Connection {
    pub fn connect(name: &str) -> io::Result<Self> {
        let path = pipe_path(name);

        let mut result = open_pipe(&path);
        // Every instance is busy while the server is between accepting a client and creating
        // the next instance
        if matches!(&result, Err(e) if e.raw_os_error() == Some(ERROR_PIPE_BUSY as i32))
            && unsafe { WaitNamedPipeW(path.as_ptr(), BUSY_PIPE_TIMEOUT.as_millis() as u32) } != 0
        {
            result = open_pipe(&path);
        }
        let pipe = result?;

        // Refuse a pipe another user created under the same name
        if !is_owned_by_current_user(pipe.0)? {
            return Err(io::Error::new(
                io::ErrorKind::PermissionDenied,
                "the pipe was not created by the current user",
            ));
        }

        let mode = PIPE_READMODE_MESSAGE;
        check(unsafe { SetNamedPipeHandleState(pipe.0, &mode, ptr::null(), ptr::null()) })?;

        Ok(Self(pipe))
    }

    /// Sends one message. Returns once the server has room for it, which it normally has.
    pub fn write(&mut self, message: &[u8]) -> io::Result<()> {
        let mut written = 0;
        check(unsafe {
            WriteFile(
                self.0.0,
                message.as_ptr(),
                message.len() as u32,
                &mut written,
                ptr::null_mut(),
            )
        })
    }

    /// The messages that have arrived, without waiting for more. A message longer than
    /// `max_len` is an error.
    pub fn read_available(&mut self, max_len: usize) -> io::Result<Vec<Vec<u8>>> {
        let mut messages = Vec::new();
        loop {
            let mut available = 0;
            let mut message_len = 0;
            check(unsafe {
                PeekNamedPipe(
                    self.0.0,
                    ptr::null_mut(),
                    0,
                    ptr::null_mut(),
                    &mut available,
                    &mut message_len,
                )
            })?;
            if available == 0 {
                return Ok(messages);
            }
            if message_len as usize > max_len {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidData,
                    "message too large",
                ));
            }

            let mut message = vec![0; message_len as usize];
            let mut read = 0;
            check(unsafe {
                ReadFile(
                    self.0.0,
                    message.as_mut_ptr(),
                    message_len,
                    &mut read,
                    ptr::null_mut(),
                )
            })?;
            message.truncate(read as usize);
            messages.push(message);
        }
    }
}

/// The server can identify the client, but not act as it.
fn open_pipe(path: &[u16]) -> io::Result<OwnedHandle> {
    let handle = unsafe {
        CreateFileW(
            path.as_ptr(),
            CLIENT_ACCESS,
            0,
            ptr::null(),
            OPEN_EXISTING,
            SECURITY_SQOS_PRESENT | SECURITY_IDENTIFICATION,
            ptr::null_mut(),
        )
    };
    if handle == INVALID_HANDLE_VALUE {
        return Err(io::Error::last_os_error());
    }

    Ok(OwnedHandle(handle))
}
