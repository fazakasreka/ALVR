//! A named file mapping in the session's object namespace.

use std::{io, mem, ptr};
use windows_sys::{
    Wdk::{Foundation::OBJECT_ATTRIBUTES, System::Memory::NtOpenSection},
    Win32::{
        Foundation::{CloseHandle, HANDLE, INVALID_HANDLE_VALUE, LocalFree, UNICODE_STRING},
        Security::{
            Authorization::{
                ConvertStringSecurityDescriptorToSecurityDescriptorW, SDDL_REVISION_1,
            },
            PSECURITY_DESCRIPTOR, SECURITY_ATTRIBUTES,
        },
        System::{
            Memory::{
                CreateFileMappingW, FILE_MAP_READ, FILE_MAP_WRITE, MEMORY_MAPPED_VIEW_ADDRESS,
                MapViewOfFile, OpenFileMappingW, PAGE_READWRITE, UnmapViewOfFile,
            },
            RemoteDesktop::ProcessIdToSessionId,
            SystemInformation::GetTickCount64,
            Threading::GetCurrentProcessId,
        },
    },
};

// Full access for SYSTEM, administrators and the owner (the user running SteamVR). Read access for
// everyone else and for AppContainers (ALL APPLICATION PACKAGES), as packaged OpenXR applications
// load the layer too.
const SECURITY_DESCRIPTOR: &str = "D:(A;;GA;;;SY)(A;;GA;;;BA)(A;;GA;;;OW)(A;;GR;;;WD)(A;;GR;;;AC)";

const SECTION_QUERY: u32 = 0x0001;
const SECTION_MAP_READ: u32 = 0x0004;
const OBJ_CASE_INSENSITIVE: u32 = 0x0040;

fn wide(text: &str) -> Vec<u16> {
    text.encode_utf16().chain([0]).collect()
}

pub fn now_ms() -> u64 {
    unsafe { GetTickCount64() }
}

pub struct Mapping {
    handle: HANDLE,
    view: MEMORY_MAPPED_VIEW_ADDRESS,
}

impl Mapping {
    pub fn create(name: &str, size: usize) -> io::Result<Self> {
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
        let handle = unsafe {
            CreateFileMappingW(
                INVALID_HANDLE_VALUE,
                &attributes,
                PAGE_READWRITE,
                0,
                size as u32,
                wide(&format!("Local\\{name}")).as_ptr(),
            )
        };
        let error = io::Error::last_os_error();
        unsafe { LocalFree(descriptor) };
        if handle.is_null() {
            return Err(error);
        }

        Self::map(handle, FILE_MAP_READ | FILE_MAP_WRITE, size)
    }

    pub fn open(name: &str, size: usize) -> io::Result<Self> {
        let mut handle =
            unsafe { OpenFileMappingW(FILE_MAP_READ, 0, wide(&format!("Local\\{name}")).as_ptr()) };

        // In an AppContainer, Local\ names the container's own namespace rather than the session's
        // one the driver created the memory in, so that is opened by its full path instead.
        if handle.is_null() {
            let local_error = io::Error::last_os_error();
            handle = open_in_session_namespace(name).map_err(|_| local_error)?;
        }

        Self::map(handle, FILE_MAP_READ, size)
    }

    fn map(handle: HANDLE, access: u32, size: usize) -> io::Result<Self> {
        let view = unsafe { MapViewOfFile(handle, access, 0, 0, size) };
        if view.Value.is_null() {
            let error = io::Error::last_os_error();
            unsafe { CloseHandle(handle) };
            return Err(error);
        }

        Ok(Self { handle, view })
    }

    pub fn ptr(&self) -> *mut u8 {
        self.view.Value.cast()
    }
}

impl Drop for Mapping {
    fn drop(&mut self) {
        unsafe {
            UnmapViewOfFile(self.view);
            CloseHandle(self.handle);
        }
    }
}

fn open_in_session_namespace(name: &str) -> io::Result<HANDLE> {
    let mut session = 0;
    if unsafe { ProcessIdToSessionId(GetCurrentProcessId(), &mut session) } == 0 {
        return Err(io::Error::last_os_error());
    }

    let path = if session == 0 {
        format!("\\BaseNamedObjects\\{name}")
    } else {
        format!("\\Sessions\\{session}\\BaseNamedObjects\\{name}")
    };
    let mut path = path.encode_utf16().collect::<Vec<_>>();
    let bytes = (path.len() * 2) as u16;
    let object_name = UNICODE_STRING {
        Length: bytes,
        MaximumLength: bytes,
        Buffer: path.as_mut_ptr(),
    };
    let attributes = OBJECT_ATTRIBUTES {
        Length: mem::size_of::<OBJECT_ATTRIBUTES>() as u32,
        RootDirectory: ptr::null_mut(),
        ObjectName: &object_name,
        Attributes: OBJ_CASE_INSENSITIVE,
        SecurityDescriptor: ptr::null(),
        SecurityQualityOfService: ptr::null(),
    };

    let mut handle = ptr::null_mut();
    let status =
        unsafe { NtOpenSection(&mut handle, SECTION_MAP_READ | SECTION_QUERY, &attributes) };
    if status < 0 {
        return Err(io::Error::other(format!("NtOpenSection: {status:#x}")));
    }

    Ok(handle)
}
