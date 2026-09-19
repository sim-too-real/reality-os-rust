//! In-tree Unix integration: production `Xl330Driver` over a PTY loopback to
//! `VirtualXl330`. Not metal evidence. Not the Python echo stand-in.
#![cfg(unix)]

use std::os::unix::io::{AsRawFd, RawFd};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::Duration;

use nix::pty::openpty;
use nix::unistd::{read as nix_read, write as nix_write};
use realityos_metal::config::MetalConfig;
use realityos_metal::protocol::decode_instruction;
use realityos_metal::xl330::Xl330Driver;
use realityos_plant::{ActionParams, HardwareDriverPort};
use realityos_virtual_metal::VirtualXl330;

#[test]
fn xl330_driver_opens_and_holds_against_virtual_metal_pty() {
    let pair = openpty(None, None).expect("openpty");
    let master_fd = pair.master.as_raw_fd();
    let slave_fd: RawFd = slave_raw_fd(&pair);
    let slave_path = std::fs::read_link(format!("/proc/self/fd/{slave_fd}"))
        .unwrap_or_else(|_| format!("/dev/pts/unknown").into());
    let stop = Arc::new(AtomicBool::new(false));
    let stop_t = stop.clone();
    let master = pair.master;
    let h = thread::spawn(move || {
        let _master = master;
        let mut dev = VirtualXl330::xl330_m288();
        let mut acc = Vec::new();
        let mut tmp = [0u8; 256];
        while !stop_t.load(Ordering::Relaxed) {
            match nix_read(master_fd, &mut tmp) {
                Ok(0) => break,
                Ok(n) => acc.extend_from_slice(&tmp[..n]),
                Err(nix::errno::Errno::EAGAIN) | Err(nix::errno::Errno::EINTR) => {
                    thread::sleep(Duration::from_millis(2));
                    continue;
                }
                Err(_) => {
                    if stop_t.load(Ordering::Relaxed) {
                        break;
                    }
                    thread::sleep(Duration::from_millis(2));
                    continue;
                }
            }
            if decode_instruction(&acc).is_ok() {
                let req = acc.clone();
                acc.clear();
                let _ = nix_write(master_fd, &req);
                let status = dev.process(&req);
                if !status.is_empty() {
                    let _ = nix_write(master_fd, &status);
                }
            } else if acc.len() > 512 {
                acc.clear();
            }
        }
    });
    let root =
        std::env::temp_dir().join(format!("realityos-vm-unix-driver-{}", std::process::id()));
    let _ = std::fs::create_dir_all(&root);
    let mut cfg = MetalConfig::example(&slave_path);
    cfg.campaign_hooks = false;
    let opened = Xl330Driver::open(cfg, &root);
    let mut driver = match opened {
        Ok(d) => d,
        Err(e) => {
            stop.store(true, Ordering::Relaxed);
            let _ = nix_write(slave_fd, &[0]);
            let _ = h.join();
            panic!("Xl330Driver::open against VirtualXl330 PTY failed: {e}");
        }
    };
    assert_eq!(driver.measured().model, 1200);
    driver
        .read_sensor(0.0)
        .expect("sensor over Virtual Metal PTY");
    driver
        .write_action(&[0.0], &ActionParams::empty())
        .expect("hold write over Virtual Metal PTY");
    driver.close();
    stop.store(true, Ordering::Relaxed);
    let _ = nix_write(slave_fd, &[0]);
    let _ = h.join();
    let _ = std::fs::remove_dir_all(&root);
    let _ = slave_fd;
}

fn slave_raw_fd(pair: &nix::pty::OpenptyResult) -> RawFd {
    #[allow(unused_imports)]
    use std::os::unix::io::AsRawFd;
    as_raw(&pair.slave)
}

fn as_raw<T: AsRawFdLike>(t: &T) -> RawFd {
    t.as_raw_fd_like()
}

#[allow(dead_code)]
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
