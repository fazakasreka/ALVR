//! Marker sharing is only implemented on Windows so far.

use std::{io, sync::LazyLock, time::Instant};

static START: LazyLock<Instant> = LazyLock::new(Instant::now);

pub fn now_ns() -> i64 {
    START.elapsed().as_nanos() as i64
}

pub fn performance_counter_to_ns(counter: i64) -> i64 {
    counter
}

pub fn ns_to_performance_counter(ns: i64) -> i64 {
    ns
}

pub type MessageCallback = Box<dyn Fn(u64, Option<&[u8]>) + Send + Sync>;

pub struct Server;

impl Server {
    pub fn start(_name: &str, _message: Vec<u8>, _on_message: MessageCallback) -> io::Result<Self> {
        Err(io::ErrorKind::Unsupported.into())
    }

    pub fn send(&self, _message: Vec<u8>) {}
}

pub struct Connection;

impl Connection {
    pub fn connect(_name: &str) -> io::Result<Self> {
        Err(io::ErrorKind::Unsupported.into())
    }

    pub fn write(&mut self, _message: &[u8]) -> io::Result<()> {
        Ok(())
    }

    pub fn read_available(&mut self, _max_len: usize) -> io::Result<Vec<Vec<u8>>> {
        Ok(Vec::new())
    }
}
