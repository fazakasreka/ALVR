//! Messages go to the debugger (DebugView shows them too) and, when the ALVR_OPENXR_LAYER_LOG
//! environment variable names a file, to that file as well.

use std::{fs::OpenOptions, io::Write};

pub fn write(message: &str) {
    let line = format!("[ALVR OpenXR layer] {message}\n");

    #[cfg(windows)]
    {
        let wide = line.encode_utf16().chain([0]).collect::<Vec<_>>();
        unsafe {
            windows_sys::Win32::System::Diagnostics::Debug::OutputDebugStringW(wide.as_ptr())
        };
    }

    if let Some(path) = std::env::var_os("ALVR_OPENXR_LAYER_LOG")
        && let Ok(mut file) = OpenOptions::new().create(true).append(true).open(path)
    {
        file.write_all(line.as_bytes()).ok();
    }
}

macro_rules! log {
    ($($arg:tt)*) => {
        $crate::log::write(&format!($($arg)*))
    };
}
pub(crate) use log;
