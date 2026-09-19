//! Unix PTY pump: production serial bytes ↔ [`crate::peer::VirtualSerialPeer`].
//!
//! The peer lives on the PTY thread. `VirtualSerialPeer` is not `Send`
//! (`Rc<RefCell<VirtualXl330>>`); the in-process campaign path keeps that type
//! on the governor thread.
#![cfg(unix)]

use std::cell::RefCell;
use std::os::unix::io::{AsRawFd, RawFd};
use std::path::PathBuf;
use std::rc::Rc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread::{self, JoinHandle};
use std::time::Duration;

use nix::pty::openpty;
use nix::unistd::{read as nix_read, write as nix_write};

use crate::device::VirtualXl330;
use crate::faults::FaultSchedule;
use crate::peer::VirtualSerialPeer;

pub struct VirtualXl330Pty {
    slave_path: PathBuf,
    stop: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
    slave_fd: RawFd,
}

impl VirtualXl330Pty {
    pub fn spawn(device: VirtualXl330) -> std::io::Result<Self> {
        Self::spawn_configured(device, true, FaultSchedule::empty())
    }

    pub fn spawn_configured(
        device: VirtualXl330,
        echo: bool,
        transport: FaultSchedule,
    ) -> std::io::Result<Self> {
        let pair =
            openpty(None, None).map_err(|e| std::io::Error::other(format!("openpty: {e}")))?;
        let master_fd = pair.master.as_raw_fd();
        let slave_fd = slave_raw_fd(&pair);
        let slave_path = std::fs::read_link(format!("/proc/self/fd/{slave_fd}"))
            .unwrap_or_else(|_| PathBuf::from("/dev/pts/unknown"));
        let stop = Arc::new(AtomicBool::new(false));
        let stop_t = stop.clone();
        let master = pair.master;
        let thread = thread::spawn(move || {
            let _master = master;
            let mut peer = VirtualSerialPeer::new(Rc::new(RefCell::new(device))).with_echo(echo);
            peer.set_transport_schedule(transport);
            let mut tmp = [0u8; 256];
            while !stop_t.load(Ordering::Relaxed) {
                match nix_read(master_fd, &mut tmp) {
                    Ok(0) => break,
                    Ok(n) => {
                        peer.push(&tmp[..n]);
                        let delay = peer.last_delay_ms();
                        if delay > 0 {
                            thread::sleep(Duration::from_millis(delay));
                        }
                        let out = peer.read(4096);
                        if !out.is_empty() {
                            let _ = nix_write(master_fd, &out);
                        }
                    }
                    Err(nix::errno::Errno::EAGAIN) | Err(nix::errno::Errno::EINTR) => {
                        thread::sleep(Duration::from_millis(2));
                    }
                    Err(_) => {
                        if stop_t.load(Ordering::Relaxed) {
                            break;
                        }
                        thread::sleep(Duration::from_millis(2));
                    }
                }
            }
        });
        Ok(Self {
            slave_path,
            stop,
            thread: Some(thread),
            slave_fd,
        })
    }

    pub fn slave_path(&self) -> &std::path::Path {
        &self.slave_path
    }

    pub fn stop(mut self) {
        self.halt();
    }

    fn halt(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        let _ = nix_write(self.slave_fd, &[0]);
        if let Some(h) = self.thread.take() {
            let _ = h.join();
        }
    }
}

impl Drop for VirtualXl330Pty {
    fn drop(&mut self) {
        self.halt();
    }
}

fn slave_raw_fd(pair: &nix::pty::OpenptyResult) -> RawFd {
    as_raw(&pair.slave)
}

fn as_raw<T: AsRawFdLike>(t: &T) -> RawFd {
    t.as_raw_fd_like()
}

trait AsRawFdLike {
    fn as_raw_fd_like(&self) -> RawFd;
}

impl AsRawFdLike for RawFd {
    fn as_raw_fd_like(&self) -> RawFd {
        *self
    }
}

impl AsRawFdLike for nix::pty::PtyMaster {
    fn as_raw_fd_like(&self) -> RawFd {
        self.as_raw_fd()
    }
}
