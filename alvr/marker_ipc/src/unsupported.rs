//! Marker sharing is only implemented on Windows so far.

use std::io;

pub struct Server;

impl Server {
    pub fn start(_name: &str, _message: Vec<u8>) -> io::Result<Self> {
        Err(io::ErrorKind::Unsupported.into())
    }

    pub fn send(&self, _message: Vec<u8>) {}
}

pub struct Connection;

impl Connection {
    pub fn connect(_name: &str) -> io::Result<Self> {
        Err(io::ErrorKind::Unsupported.into())
    }

    pub fn read_available(&mut self, _buffer: &mut Vec<u8>) -> io::Result<()> {
        Ok(())
    }
}
