//! In-tree Unix integration: production `Xl330Driver` over a PTY loopback to
//! `VirtualXl330`. Not metal evidence. Not the Python echo stand-in.
#![cfg(unix)]

use realityos_metal::config::MetalConfig;
use realityos_metal::xl330::Xl330Driver;
use realityos_plant::{ActionParams, HardwareDriverPort};
use realityos_virtual_metal::pty::VirtualXl330Pty;
use realityos_virtual_metal::VirtualXl330;

#[test]
fn xl330_driver_opens_and_holds_against_virtual_metal_pty() {
    let device = VirtualXl330::xl330_m288();
    let pty = VirtualXl330Pty::spawn(device).expect("virtual xl330 pty");
    let root =
        std::env::temp_dir().join(format!("realityos-vm-unix-driver-{}", std::process::id()));
    let _ = std::fs::create_dir_all(&root);
    let mut cfg = MetalConfig::example(pty.slave_path());
    cfg.campaign_hooks = false;
    let opened = Xl330Driver::open(cfg, &root);
    let mut driver = match opened {
        Ok(d) => d,
        Err(e) => {
            pty.stop();
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
    pty.stop();
    let _ = std::fs::remove_dir_all(&root);
}
