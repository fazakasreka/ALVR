//! The ALVR driver registers its OpenXR API layer for the current user under
//! HKCU\Software\Khronos\OpenXR\1\ApiLayers\Implicit, as a value named after the layer manifest's
//! path. A driver unregistered from SteamVR no longer needs it there, or the layer would keep
//! loading into every OpenXR application.

use std::path::Path;

/// Removes the registrations of OpenXR layers inside `driver_dir`. Returns how many there were.
#[cfg(windows)]
pub fn unregister_openxr_layers(driver_dir: &Path) -> usize {
    use std::{ffi::OsStr, os::windows::ffi::OsStrExt, ptr};
    use windows_sys::Win32::{
        Foundation::ERROR_SUCCESS,
        System::Registry::{
            HKEY, HKEY_CURRENT_USER, KEY_READ, KEY_WRITE, RegCloseKey, RegDeleteValueW,
            RegEnumValueW, RegOpenKeyExW,
        },
    };

    const IMPLICIT_LAYERS_KEY: &str = r"Software\Khronos\OpenXR\1\ApiLayers\Implicit";
    let wide = |text: &OsStr| text.encode_wide().chain([0]).collect::<Vec<u16>>();

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
        return 0;
    }

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

    let inside = names
        .into_iter()
        .filter(|name| is_inside(Path::new(name), driver_dir))
        .collect::<Vec<_>>();
    for name in &inside {
        unsafe { RegDeleteValueW(key, wide(OsStr::new(name)).as_ptr()) };
    }
    unsafe { RegCloseKey(key) };

    inside.len()
}

#[cfg(not(windows))]
pub fn unregister_openxr_layers(_driver_dir: &Path) -> usize {
    0
}

/// Whether `path` is in `dir`, comparing as Windows does: whatever the case and the separators.
fn is_inside(path: &Path, dir: &Path) -> bool {
    let normal = |path: &Path| {
        path.to_string_lossy()
            .replace('/', "\\")
            .trim_end_matches('\\')
            .to_lowercase()
    };
    normal(path).starts_with(&format!("{}\\", normal(dir)))
}

#[cfg(test)]
mod tests {
    use super::*;

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
