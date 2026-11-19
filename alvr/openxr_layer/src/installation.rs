//! Whether the ALVR copy the layer belongs to is one SteamVR loads.
//!
//! The ALVR driver registers the layer for every OpenXR application of the user. Uninstalling ALVR
//! without its uninstaller or unregistering its driver from SteamVR can leave that registration
//! behind, and the layer would still load into every application. So the layer checks that its
//! streamer folder, the one above its own, is among SteamVR's drivers, and otherwise stays out of
//! the way.

use std::path::{Path, PathBuf};

/// Whether the layer's streamer folder is registered as a SteamVR driver.
#[cfg(not(test))]
pub fn alvr_is_registered() -> bool {
    let Some(streamer) = own_path().and_then(|dll| Some(dll.parent()?.parent()?.to_owned())) else {
        return false;
    };

    registered_drivers().is_some_and(|drivers| drivers.iter().any(|dir| same_dir(dir, &streamer)))
}

/// The tests drive the layer without an ALVR install.
#[cfg(test)]
pub fn alvr_is_registered() -> bool {
    true
}

/// The external drivers in SteamVR's `openvrpaths.vrpath`.
#[cfg_attr(test, allow(dead_code))]
fn registered_drivers() -> Option<Vec<PathBuf>> {
    let local_app_data = std::env::var_os("LOCALAPPDATA")?;
    let paths = Path::new(&local_app_data).join(r"openvr\openvrpaths.vrpath");
    let text = std::fs::read_to_string(paths).ok()?;
    let json = serde_json::from_str::<serde_json::Value>(&text).ok()?;

    Some(
        json.get("external_drivers")?
            .as_array()?
            .iter()
            .filter_map(|path| path.as_str().map(PathBuf::from))
            .collect(),
    )
}

#[cfg_attr(test, allow(dead_code))]
fn same_dir(a: &Path, b: &Path) -> bool {
    match (a.canonicalize(), b.canonicalize()) {
        (Ok(a), Ok(b)) => a == b,
        _ => {
            let normal = |path: &Path| {
                path.to_string_lossy()
                    .replace('/', "\\")
                    .trim_end_matches('\\')
                    .to_lowercase()
            };
            normal(a) == normal(b)
        }
    }
}

/// The layer DLL's own path.
#[cfg(all(windows, not(test)))]
fn own_path() -> Option<PathBuf> {
    use std::{ffi::OsString, os::windows::ffi::OsStringExt, ptr};
    use windows_sys::Win32::{
        Foundation::HMODULE,
        System::LibraryLoader::{
            GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS, GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
            GetModuleFileNameW, GetModuleHandleExW,
        },
    };

    let mut module: HMODULE = ptr::null_mut();
    let found = unsafe {
        GetModuleHandleExW(
            GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
            (alvr_is_registered as *const ()).cast(),
            &mut module,
        )
    };
    if found == 0 {
        return None;
    }

    let mut buffer = vec![0u16; 32768];
    let length = unsafe { GetModuleFileNameW(module, buffer.as_mut_ptr(), buffer.len() as u32) };
    (length > 0).then(|| PathBuf::from(OsString::from_wide(&buffer[..length as usize])))
}

#[cfg(all(not(windows), not(test)))]
fn own_path() -> Option<PathBuf> {
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn folders_compare_by_their_paths() {
        assert!(same_dir(
            Path::new(r"C:\Missing\ALVR\streamer"),
            Path::new(r"c:/missing/alvr/streamer/")
        ));
        assert!(!same_dir(
            Path::new(r"C:\Missing\ALVR\streamer"),
            Path::new(r"C:\Missing\ALVR\other")
        ));
    }
}
