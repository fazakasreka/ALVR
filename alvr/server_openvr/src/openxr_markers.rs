//! Markers for OpenXR applications.
//!
//! SteamVR's OpenXR runtime knows nothing about markers, so ALVR ships an OpenXR API layer
//! (alvr/openxr_layer) that implements XR_EXT_spatial_marker_tracking inside each application.
//! The driver publishes the markers the headset sees to shared memory for it, and registers the
//! layer with the OpenXR loader when SteamVR starts.

use alvr_common::{debug, info, parking_lot::Mutex, warn};
use alvr_filesystem as afs;
use alvr_marker_shm::{FLAG_CLIENT_CONNECTED, HEARTBEAT_INTERVAL, Pose, Writer};
use alvr_packets::MarkerData;
use std::{sync::Arc, thread};

struct State {
    writer: Arc<Writer>,
    client_connected: bool,
    markers: Vec<MarkerData>,
}

static STATE: Mutex<Option<State>> = Mutex::new(None);
static HEARTBEAT_THREAD: Mutex<Option<thread::JoinHandle<()>>> = Mutex::new(None);

pub fn start(layout: &afs::Layout, enabled: bool) {
    #[cfg(windows)]
    registration::register(layout, enabled);
    #[cfg(not(windows))]
    let _ = layout;

    if !enabled {
        return;
    }

    let writer = match Writer::create() {
        Ok(writer) => Arc::new(writer),
        Err(e) => {
            warn!("OpenXR applications will not get markers: {e}");
            return;
        }
    };

    *STATE.lock() = Some(State {
        writer: Arc::clone(&writer),
        client_connected: false,
        markers: Vec::new(),
    });

    *HEARTBEAT_THREAD.lock() = Some(thread::spawn(move || {
        // Runs until stop() drops the other reference
        while Arc::strong_count(&writer) > 1 {
            writer.heartbeat();
            thread::sleep(HEARTBEAT_INTERVAL);
        }
        // Dropping the writer tells readers it is gone
    }));
}

pub fn stop() {
    STATE.lock().take();
    if let Some(thread) = HEARTBEAT_THREAD.lock().take() {
        thread.join().ok();
    }
}

pub fn set_client_connected(connected: bool) {
    if let Some(state) = &mut *STATE.lock() {
        state.client_connected = connected;
        publish(state);
    }
}

/// `markers` are recentered: in the space the headset pose is submitted to SteamVR in.
pub fn set_markers(markers: Vec<MarkerData>) {
    if let Some(state) = &mut *STATE.lock() {
        state.markers = markers;
        publish(state);
    }
}

fn publish(state: &State) {
    let flags = if state.client_connected {
        FLAG_CLIENT_CONNECTED
    } else {
        0
    };

    state.writer.publish(
        flags,
        state.markers.iter().map(|marker| {
            (
                marker.id.as_str(),
                Pose {
                    orientation: marker.pose.orientation.to_array(),
                    position: marker.pose.position.to_array(),
                },
                marker.size,
            )
        }),
    );
    debug!("Published {} marker(s) to OpenXR", state.markers.len());
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
