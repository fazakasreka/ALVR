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

use alvr_common::{info, parking_lot::Mutex, warn};
use alvr_filesystem as afs;
use alvr_marker_ipc::{
    Capabilities, FLAG_CLIENT_CONNECTED, Marker, Pose, Server, Stream, TrackingConfig, now_ns,
};
use alvr_packets::{MarkerCapabilities, MarkerTrackingConfig};
use alvr_server_core::MarkerStream;
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
static REQUESTED: Mutex<Vec<MarkerTrackingConfig>> = Mutex::new(Vec::new());

/// `on_requests` gets the tracker configurations OpenXR applications ask for, whenever they
/// change.
pub fn start(
    layout: &afs::Layout,
    enabled: bool,
    on_requests: impl Fn(Vec<MarkerTrackingConfig>) + Send + Sync + 'static,
) {
    #[cfg(windows)]
    registration::register(layout, enabled);
    #[cfg(not(windows))]
    let _ = layout;

    if !enabled {
        return;
    }

    let server = match Server::start(move |configs| {
        info!("OpenXR applications ask for marker trackers {configs:?}");
        let configs = configs.into_iter().map(from_ipc_config).collect::<Vec<_>>();
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
pub fn requested_configs() -> Vec<MarkerTrackingConfig> {
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

pub fn set_capabilities(capabilities: MarkerCapabilities) {
    if let Some(state) = &mut *STATE.lock() {
        state.capabilities = Capabilities {
            qr_codes: capabilities.qr_codes,
            fixed_size_markers: capabilities.fixed_size_markers,
            static_markers: capabilities.static_markers,
        };
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
/// submitted to SteamVR in.
pub fn set_markers(streams: Vec<MarkerStream>) {
    if let Some(state) = &mut *STATE.lock() {
        state.streams = streams;
        publish(state);
    }
}

fn from_ipc_config(config: TrackingConfig) -> MarkerTrackingConfig {
    MarkerTrackingConfig {
        static_markers: config.static_markers,
        marker_size: config.marker_size,
    }
}

fn to_ipc_config(config: MarkerTrackingConfig) -> TrackingConfig {
    TrackingConfig {
        static_markers: config.static_markers,
        marker_size: config.marker_size,
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
            config: to_ipc_config(stream.config),
            time_ns: match state.clock_offset_ns {
                Some(offset) => (stream.timestamp.as_nanos() as f64 + offset) as i64,
                // No tracking yet to tell the clocks apart
                None => now_ns(),
            },
            markers: stream
                .markers
                .iter()
                .map(|marker| {
                    Marker::new(
                        &marker.id,
                        Pose {
                            orientation: marker.pose.orientation.to_array(),
                            position: marker.pose.position.to_array(),
                        },
                        marker.size,
                    )
                })
                .collect(),
        })
        .collect();

    state.server.publish(flags, state.capabilities, streams);
}

#[cfg(windows)]
mod registration {
    //! The layer is registered for the current user under
    //! HKCU\Software\Khronos\OpenXR\1\ApiLayers\Implicit, as a value named after the manifest
    //! path: 0 enables it, anything else disables it. Packaged (AppContainer) applications read
    //! neither that key nor the streamer folder unless ALL APPLICATION PACKAGES is granted read
    //! access to them, so that is done too.

    use super::*;
    use std::{ffi::OsStr, os::windows::ffi::OsStrExt, path::Path, ptr};
    use windows_sys::Win32::{
        Foundation::{ERROR_SUCCESS, LocalFree},
        Security::{
            ACL,
            Authorization::{
                ConvertStringSidToSidW, EXPLICIT_ACCESS_W, GRANT_ACCESS, GetNamedSecurityInfoW,
                SE_FILE_OBJECT, SE_REGISTRY_KEY, SetEntriesInAclW, SetNamedSecurityInfoW,
                TRUSTEE_IS_SID, TRUSTEE_IS_WELL_KNOWN_GROUP, TRUSTEE_W,
            },
            CONTAINER_INHERIT_ACE, DACL_SECURITY_INFORMATION, OBJECT_INHERIT_ACE,
            PSECURITY_DESCRIPTOR, PSID,
        },
        Storage::FileSystem::{FILE_GENERIC_EXECUTE, FILE_GENERIC_READ},
        System::Registry::{
            HKEY, HKEY_CURRENT_USER, KEY_READ, KEY_WRITE, REG_DWORD, REG_OPTION_NON_VOLATILE,
            RegCloseKey, RegCreateKeyExW, RegDeleteValueW, RegEnumValueW, RegSetValueExW,
        },
    };

    const IMPLICIT_LAYERS_KEY: &str = r"Software\Khronos\OpenXR\1\ApiLayers\Implicit";
    const ALL_APPLICATION_PACKAGES: &str = "S-1-15-2-1";

    fn wide(text: &OsStr) -> Vec<u16> {
        text.encode_wide().chain([0]).collect()
    }

    pub fn register(layout: &afs::Layout, enabled: bool) {
        let manifest = layout.openxr_layer_manifest();
        if !manifest.exists() {
            info!(
                "No OpenXR layer at {}, OpenXR applications will not get markers",
                manifest.display()
            );
            return;
        }

        if let Err(e) = set_registry_value(&manifest, enabled) {
            warn!("Cannot register the OpenXR marker layer: {e}");
            return;
        }
        if let Err(e) = grant_app_packages_access(
            &layout.openxr_layer_dir(),
            SE_FILE_OBJECT,
            FILE_GENERIC_READ | FILE_GENERIC_EXECUTE,
        ) {
            warn!("Packaged OpenXR applications cannot load the marker layer: {e}");
        }

        info!(
            "OpenXR marker layer {} ({})",
            if enabled { "enabled" } else { "disabled" },
            manifest.display()
        );
    }

    fn set_registry_value(manifest: &Path, enabled: bool) -> Result<(), String> {
        let mut key: HKEY = ptr::null_mut();
        let status = unsafe {
            RegCreateKeyExW(
                HKEY_CURRENT_USER,
                wide(OsStr::new(IMPLICIT_LAYERS_KEY)).as_ptr(),
                0,
                ptr::null(),
                REG_OPTION_NON_VOLATILE,
                KEY_READ | KEY_WRITE,
                ptr::null(),
                &mut key,
                ptr::null_mut(),
            )
        };
        if status != ERROR_SUCCESS {
            return Err(format!("cannot open {IMPLICIT_LAYERS_KEY}: error {status}"));
        }

        let result = (|| {
            // Another ALVR copy (an older install, a development build) registered under another
            // path would load the layer a second time, from a folder that may not match the
            // running driver
            let file_name = manifest.file_name().unwrap_or_default();
            for value in value_names(key) {
                let path = Path::new(&value);
                if path != manifest
                    && path
                        .file_name()
                        .is_some_and(|name| name.eq_ignore_ascii_case(file_name))
                {
                    info!("Unregistering the OpenXR layer at {value}");
                    unsafe { RegDeleteValueW(key, wide(OsStr::new(&value)).as_ptr()) };
                }
            }

            let disabled = u32::from(!enabled);
            let status = unsafe {
                RegSetValueExW(
                    key,
                    wide(manifest.as_os_str()).as_ptr(),
                    0,
                    REG_DWORD,
                    (&raw const disabled).cast(),
                    size_of::<u32>() as u32,
                )
            };
            if status != ERROR_SUCCESS {
                return Err(format!("cannot write the registry value: error {status}"));
            }

            Ok(())
        })();

        unsafe { RegCloseKey(key) };
        result?;

        let key_path = format!(r"CURRENT_USER\{IMPLICIT_LAYERS_KEY}");
        grant_app_packages_access(Path::new(&key_path), SE_REGISTRY_KEY, KEY_READ)
    }

    fn value_names(key: HKEY) -> Vec<String> {
        let mut names = Vec::new();
        let mut buffer = vec![0u16; 32768];
        for index in 0.. {
            let mut length = buffer.len() as u32;
            let status = unsafe {
                RegEnumValueW(
                    key,
                    index,
                    buffer.as_mut_ptr(),
                    &mut length,
                    ptr::null(),
                    ptr::null_mut(),
                    ptr::null_mut(),
                    ptr::null_mut(),
                )
            };
            if status != ERROR_SUCCESS {
                break;
            }
            names.push(String::from_utf16_lossy(&buffer[..length as usize]));
        }

        names
    }

    // Adds an inheritable ACE granting `access` to ALL APPLICATION PACKAGES. Does nothing if
    // the object already grants it.
    fn grant_app_packages_access(
        object: &Path,
        object_type: i32,
        access: u32,
    ) -> Result<(), String> {
        let object = wide(object.as_os_str());

        let mut sid: PSID = ptr::null_mut();
        if unsafe {
            ConvertStringSidToSidW(
                wide(OsStr::new(ALL_APPLICATION_PACKAGES)).as_ptr(),
                &mut sid,
            )
        } == 0
        {
            return Err(std::io::Error::last_os_error().to_string());
        }

        let mut old_acl: *mut ACL = ptr::null_mut();
        let mut descriptor: PSECURITY_DESCRIPTOR = ptr::null_mut();
        let mut new_acl: *mut ACL = ptr::null_mut();

        let result = (|| unsafe {
            let status = GetNamedSecurityInfoW(
                object.as_ptr(),
                object_type,
                DACL_SECURITY_INFORMATION,
                ptr::null_mut(),
                ptr::null_mut(),
                &mut old_acl,
                ptr::null_mut(),
                &mut descriptor,
            );
            if status != ERROR_SUCCESS {
                return Err(format!("cannot read the permissions: error {status}"));
            }

            let entry = EXPLICIT_ACCESS_W {
                grfAccessPermissions: access,
                grfAccessMode: GRANT_ACCESS,
                grfInheritance: CONTAINER_INHERIT_ACE | OBJECT_INHERIT_ACE,
                Trustee: TRUSTEE_W {
                    pMultipleTrustee: ptr::null_mut(),
                    MultipleTrusteeOperation: 0,
                    TrusteeForm: TRUSTEE_IS_SID,
                    TrusteeType: TRUSTEE_IS_WELL_KNOWN_GROUP,
                    ptstrName: sid.cast(),
                },
            };
            let status = SetEntriesInAclW(1, &entry, old_acl, &mut new_acl);
            if status != ERROR_SUCCESS {
                return Err(format!("cannot build the permissions: error {status}"));
            }

            let status = SetNamedSecurityInfoW(
                object.as_ptr(),
                object_type,
                DACL_SECURITY_INFORMATION,
                ptr::null_mut(),
                ptr::null_mut(),
                new_acl,
                ptr::null(),
            );
            if status != ERROR_SUCCESS {
                return Err(format!("cannot write the permissions: error {status}"));
            }

            Ok(())
        })();

        unsafe {
            LocalFree(new_acl.cast());
            LocalFree(descriptor);
            LocalFree(sid);
        }

        result
    }
}
