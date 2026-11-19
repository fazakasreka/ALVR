//! The OpenXR API layer that gives OpenXR applications the markers the headset sees is registered
//! for the current user under HKCU\Software\Khronos\OpenXR\1\ApiLayers\Implicit, as a value named
//! after the manifest path: 0 enables it, anything else disables it. Packaged (AppContainer)
//! applications read neither that key nor the layer folder unless ALL APPLICATION PACKAGES is
//! granted read access to them, so that is done too.

use alvr_filesystem::Layout;
use std::path::Path;

#[cfg(windows)]
pub use windows::{register_openxr_layer, unregister_openxr_layers};

#[cfg(not(windows))]
pub fn register_openxr_layer(_layout: &Layout, _enabled: bool) {}

/// Removes the registrations of OpenXR layers inside `driver_dir`, which would keep loading into
/// every OpenXR application.
#[cfg(not(windows))]
pub fn unregister_openxr_layers(_driver_dir: &Path) {}

#[cfg(windows)]
mod windows {
    use super::*;
    use alvr_common::{info, warn};
    use std::{ffi::OsStr, os::windows::ffi::OsStrExt, ptr};
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
            RegCloseKey, RegCreateKeyExW, RegDeleteValueW, RegEnumValueW, RegOpenKeyExW,
            RegSetValueExW,
        },
    };

    const IMPLICIT_LAYERS_KEY: &str = r"Software\Khronos\OpenXR\1\ApiLayers\Implicit";
    const ALL_APPLICATION_PACKAGES: &str = "S-1-15-2-1";

    fn wide(text: &OsStr) -> Vec<u16> {
        text.encode_wide().chain([0]).collect()
    }

    pub fn register_openxr_layer(layout: &Layout, enabled: bool) {
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

    /// Removes the registrations of OpenXR layers inside `driver_dir`, which would keep loading
    /// into every OpenXR application.
    pub fn unregister_openxr_layers(driver_dir: &Path) {
        let mut key: HKEY = ptr::null_mut();
        if unsafe {
            RegOpenKeyExW(
                HKEY_CURRENT_USER,
                wide(OsStr::new(IMPLICIT_LAYERS_KEY)).as_ptr(),
                0,
                KEY_READ | KEY_WRITE,
                &mut key,
            )
        } != ERROR_SUCCESS
        {
            return;
        }

        for value in value_names(key) {
            if is_inside(Path::new(&value), driver_dir) {
                info!("Unregistering the OpenXR layer at {value}");
                unsafe { RegDeleteValueW(key, wide(OsStr::new(&value)).as_ptr()) };
            }
        }
        unsafe { RegCloseKey(key) };
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

    /// Adds an inheritable ACE granting `access` to ALL APPLICATION PACKAGES. Does nothing if
    /// the object already grants it.
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

    /// Whether `path` is in `dir`, comparing as Windows does: whatever the case and the
    /// separators.
    pub(super) fn is_inside(path: &Path, dir: &Path) -> bool {
        let normal = |path: &Path| {
            path.to_string_lossy()
                .replace('/', "\\")
                .trim_end_matches('\\')
                .to_lowercase()
        };
        normal(path).starts_with(&format!("{}\\", normal(dir)))
    }
}

#[cfg(all(test, windows))]
mod tests {
    use super::{windows::is_inside, *};

    #[test]
    fn layers_inside_the_driver_folder() {
        let dir = Path::new(r"C:\Users\me\AppData\Local\ALVR\streamer");
        assert!(is_inside(
            Path::new(
                r"c:\users\me\appdata\local\alvr\streamer\openxr_layer\alvr_openxr_layer.json"
            ),
            dir
        ));
        // Not a folder of the same name's prefix
        assert!(!is_inside(
            Path::new(
                r"C:\Users\me\AppData\Local\ALVR\streamer2\openxr_layer\alvr_openxr_layer.json"
            ),
            dir
        ));
    }
}
