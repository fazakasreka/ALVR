//! Marker sharing is only implemented on Windows so far.

use std::io;

pub fn now_ms() -> u64 {
    0
}

pub struct Mapping;

impl Mapping {
    pub fn create(_name: &str, _size: usize) -> io::Result<Self> {
        Err(io::ErrorKind::Unsupported.into())
    }

    pub fn open(_name: &str, _size: usize) -> io::Result<Self> {
        Err(io::ErrorKind::Unsupported.into())
    }

    pub fn ptr(&self) -> *mut u8 {
        unreachable!()
    }
}
