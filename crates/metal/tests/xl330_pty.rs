//! PTY Protocol 2.0 stand-in. Proves driver identity latch + echo scan.
//! Not a metal proof. Does not write docs/metal_proof.json.

use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::sync::Mutex;
use std::time::Duration;

use realityos_metal::authority::MetalAuthority;
use realityos_metal::config::{MetalConfig, CONFIG_FILE};
use realityos_metal::egress::recorded_writes;
use realityos_metal::identity::is_pty_path;
use realityos_metal::ipc::MetalRequest;
use realityos_metal::xl330::Xl330Driver;
use realityos_plant::{ActionParams, HardwareDriverPort};

/// Parallel start_online + journal fsyncs starve the 100 ms watchdog on GHA.
fn pty_serial() -> std::sync::MutexGuard<'static, ()> {
    static LOCK: Mutex<()> = Mutex::new(());
    LOCK.lock().unwrap_or_else(|e| e.into_inner())
}

fn journal_lines(root: &Path) -> usize {
    std::fs::read_to_string(root.join("driver.jsonl"))
        .map(|s| s.lines().filter(|l| !l.is_empty()).count())
        .unwrap_or(0)
}

fn watchdog_events(root: &Path) -> usize {
    std::fs::read_to_string(root.join("driver.jsonl"))
        .map(|s| {
            s.lines()
                .filter(|l| l.contains("\"watchdog_tick\""))
                .count()
        })
        .unwrap_or(0)
}

struct ChildGuard(std::process::Child);

impl Drop for ChildGuard {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

fn spawn_responder() -> (ChildGuard, String) {
    let script = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/xl330_responder.py");
    assert!(script.is_file(), "missing {}", script.display());
    let mut child = Command::new("python3")
        .arg(&script)
        .stdout(Stdio::piped())
        .stderr(Stdio::inherit())
        .spawn()
        .expect("python3 xl330_responder");
    let mut stdout = child.stdout.take().expect("responder stdout");
    let mut line = String::new();
    {
        use std::io::Read;
        let mut tmp = [0u8; 128];
        let start = std::time::Instant::now();
        while !line.contains('\n') && start.elapsed() < Duration::from_secs(2) {
            match stdout.read(&mut tmp) {
                Ok(0) => break,
                Ok(n) => line.push_str(&String::from_utf8_lossy(&tmp[..n])),
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                    std::thread::sleep(Duration::from_millis(10));
                }
                Err(e) => panic!("read responder tty: {e}"),
            }
        }
    }
    let tty = line.lines().next().unwrap_or("").trim().to_string();
    assert!(tty.starts_with("/dev/"), "responder tty got {tty:?}");
    (ChildGuard(child), tty)
}

#[test]
fn xl330_pty_firmware_survives_sensor_and_echoed_status() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder();
    let root =
        std::env::temp_dir().join(format!("realityos-metal-pty-driver-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&root);
    std::fs::create_dir_all(&root).unwrap();
    let mut cfg = MetalConfig::example(&tty);
    cfg.campaign_hooks = false;
    let mut driver = Xl330Driver::open(cfg, &root).expect("open pty xl330");
    let before = driver.measured();
    assert_eq!(before.firmware_id, "xl330-m288:1190:46");
    assert_eq!(before.model, 1190);
    driver
        .read_sensor(0.0)
        .expect("sensor over echoed PTY status");
    let after = driver.measured();
    assert_eq!(
        after.firmware_id, "xl330-m288:1190:46",
        "sensor samples must not clobber latched firmware"
    );
    driver
        .write_action(&[0.0], &ActionParams::empty())
        .expect("hold write");
    assert_eq!(recorded_writes(root.join("bus")), 1);
    driver.close();
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_start_online_hold_is_not_a_metal_proof() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder();
    assert!(is_pty_path(std::path::Path::new(&tty)));
    let root =
        std::env::temp_dir().join(format!("realityos-metal-pty-online-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&root);
    std::fs::create_dir_all(&root).unwrap();
    let mut cfg = MetalConfig::example(&tty);
    {
        let driver = Xl330Driver::open(cfg.clone(), &root).expect("identify");
        let measured = driver.measured();
        assert!(
            measured.serial.starts_with("tty:"),
            "PTY must use measured tty+rdev, got {}",
            measured.serial
        );
        let id = measured.hardware_identity(&cfg);
        assert!(!id.metal);
        assert_eq!(id.evidence_status, "PTY_STAND_IN_NOT_METAL");
        cfg.expected_serial = measured.serial;
        cfg.expected_firmware = measured.firmware_id;
    }
    cfg.save(root.join(CONFIG_FILE)).unwrap();
    let mut auth = MetalAuthority::start(&root, true).expect("start_online on PTY stand-in");
    let journal_before = journal_lines(&root);
    let watchdog_before = watchdog_events(&root);
    let resp = auth.handle(MetalRequest::propose("pty-hold", "hold"));
    assert!(resp.ok, "hold refused: {resp:?}");
    assert!(!resp.metal, "PTY stand-in must not claim metal: {resp:?}");
    assert_eq!(auth.physical_writes(), 1);
    let added = journal_lines(&root).saturating_sub(journal_before);
    let wd_added = watchdog_events(&root).saturating_sub(watchdog_before);
    assert!(
        added <= 8,
        "first hold must not flood journal+seal fsyncs: added={added}"
    );
    assert!(
        wd_added <= 4,
        "propose ticks watchdog at handle + after-acquire + dispatch + post-write, got {wd_added}"
    );
    assert_eq!(resp.clock, "OsMonotonicClock");
    let repo_proof = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../docs/metal_proof.json");
    assert!(
        !repo_proof.exists(),
        "PTY stand-in must not write docs/metal_proof.json"
    );
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_serve_hold_survives_idle_watchdog() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder();
    let root =
        std::env::temp_dir().join(format!("realityos-metal-pty-idle-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&root);
    std::fs::create_dir_all(&root).unwrap();
    let mut cfg = MetalConfig::example(&tty);
    {
        let driver = Xl330Driver::open(cfg.clone(), &root).expect("identify");
        let measured = driver.measured();
        cfg.expected_serial = measured.serial;
        cfg.expected_firmware = measured.firmware_id;
    }
    cfg.save(root.join(CONFIG_FILE)).unwrap();
    let serve_root = root.clone();
    let handle = std::thread::spawn(move || {
        let _ = realityos_metal::serve_forever(&serve_root, true);
    });
    assert!(
        realityos_metal::ipc::wait_for_ipc(&root, 15_000),
        "serve did not bind ipc.sock: {}",
        std::fs::read_to_string(root.join("serve.err")).unwrap_or_default()
    );
    // Cover idle watchdog (~40 ms) and idle heartbeat (~800 ms). Heartbeat
    // persist in the same serve iteration as handle() was a 100 ms miss.
    std::thread::sleep(Duration::from_millis(900));
    let resp = realityos_metal::ipc::call(&root, &MetalRequest::propose("pty-idle-hold", "hold"))
        .expect("ipc hold after idle");
    assert!(
        resp.ok,
        "first hold after idle watchdog/heartbeat must succeed: {resp:?} serve.err={}",
        std::fs::read_to_string(root.join("serve.err")).unwrap_or_default()
    );
    assert!(!resp.metal);
    let _ = std::fs::write(root.join("stop_serve"), b"1");
    let _ = handle.join();
    let _ = std::fs::remove_dir_all(&root);
}

fn recover_req(id: &str) -> MetalRequest {
    let mut r = MetalRequest::propose(id, "hold");
    r.op = "recover".into();
    r
}

#[test]
fn xl330_pty_firmware_mismatch_kills_session_not_watchdog() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder();
    let root = std::env::temp_dir().join(format!("realityos-metal-pty-fw-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&root);
    std::fs::create_dir_all(&root).unwrap();
    let mut cfg = MetalConfig::example(&tty);
    cfg.campaign_hooks = true;
    {
        let driver = Xl330Driver::open(cfg.clone(), &root).expect("identify");
        let measured = driver.measured();
        cfg.expected_serial = measured.serial;
        cfg.expected_firmware = measured.firmware_id;
    }
    cfg.save(root.join(CONFIG_FILE)).unwrap();
    let mut auth = MetalAuthority::start(&root, true).expect("start_online");
    let hold = auth.handle(MetalRequest::propose("pty-fw-hold", "hold"));
    assert!(hold.ok, "hold refused: {hold:?}");
    let writes = auth.physical_writes();
    std::fs::write(
        root.join("bus/hot_swap.json"),
        r#"{"firmware_id":"xl330-m288:1190:255"}"#,
    )
    .unwrap();
    let fw = auth.handle(MetalRequest::propose("pty-fw", "hold"));
    assert!(!fw.ok, "firmware overlay must refuse: {fw:?}");
    assert!(
        fw.violations
            .iter()
            .any(|v| v.contains("hardware_firmware_mismatch")),
        "identity refuse, not a vacuous miss: {fw:?}"
    );
    assert!(
        !fw.violations
            .iter()
            .any(|v| v.contains("software_watchdog_miss")),
        "identity ESTOP must not be labeled watchdog miss: {fw:?}"
    );
    assert_eq!(auth.physical_writes(), writes);
    let rec = auth.handle(recover_req("pty-fw-rec"));
    assert!(!rec.ok, "recover after identity must refuse: {rec:?}");
    assert!(
        rec.violations
            .iter()
            .any(|v| v.contains("hardware_session_requires_online_restart")),
        "recover must measure the dead hardware session: {rec:?}"
    );
    assert!(
        !rec.violations
            .iter()
            .any(|v| v.contains("software_watchdog_miss")),
        "recover after identity must not be a watchdog short-circuit: {rec:?}"
    );
    let again = auth.handle(MetalRequest::propose("pty-fw-again", "hold"));
    assert!(!again.ok, "{again:?}");
    assert_eq!(auth.physical_writes(), writes);
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_disconnect_overlay_kills_session_via_verify() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder();
    let root =
        std::env::temp_dir().join(format!("realityos-metal-pty-disc-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&root);
    std::fs::create_dir_all(&root).unwrap();
    let mut cfg = MetalConfig::example(&tty);
    cfg.campaign_hooks = true;
    {
        let driver = Xl330Driver::open(cfg.clone(), &root).expect("identify");
        let measured = driver.measured();
        cfg.expected_serial = measured.serial;
        cfg.expected_firmware = measured.firmware_id;
    }
    cfg.save(root.join(CONFIG_FILE)).unwrap();
    let mut auth = MetalAuthority::start(&root, true).expect("start_online");
    let hold = auth.handle(MetalRequest::propose("pty-disc-hold", "hold"));
    assert!(hold.ok, "hold refused: {hold:?}");
    let writes = auth.physical_writes();
    std::fs::write(root.join("bus/force_disconnect"), b"1").unwrap();
    let disc = auth.handle(MetalRequest::propose("pty-disc", "hold"));
    assert!(!disc.ok, "disconnect overlay must refuse: {disc:?}");
    assert!(
        disc.violations
            .iter()
            .any(|v| v.contains("online_hardware_disconnected")),
        "disconnect must reach verify_live_hardware: {disc:?}"
    );
    assert!(
        !disc
            .violations
            .iter()
            .any(|v| v.contains("software_watchdog_miss")),
        "{disc:?}"
    );
    assert_eq!(auth.physical_writes(), writes);
    let rec = auth.handle(recover_req("pty-disc-rec"));
    assert!(!rec.ok, "{rec:?}");
    assert!(
        rec.violations
            .iter()
            .any(|v| v.contains("hardware_session_requires_online_restart")),
        "recover must not resurrect a disconnected instance: {rec:?}"
    );
    std::fs::remove_file(root.join("bus/force_disconnect")).unwrap();
    let again = auth.handle(MetalRequest::propose("pty-disc-again", "hold"));
    assert!(
        !again.ok,
        "clearing the hook must not revive the instance: {again:?}"
    );
    assert_eq!(auth.physical_writes(), writes);
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_bus_timeout_requires_online_restart() {
    let _serial = pty_serial();
    let (guard, tty) = spawn_responder();
    let root = std::env::temp_dir().join(format!(
        "realityos-metal-pty-busloss-{}",
        std::process::id()
    ));
    let _ = std::fs::remove_dir_all(&root);
    std::fs::create_dir_all(&root).unwrap();
    let mut cfg = MetalConfig::example(&tty);
    {
        let driver = Xl330Driver::open(cfg.clone(), &root).expect("identify");
        let measured = driver.measured();
        cfg.expected_serial = measured.serial;
        cfg.expected_firmware = measured.firmware_id;
    }
    cfg.save(root.join(CONFIG_FILE)).unwrap();
    let mut auth = MetalAuthority::start(&root, true).expect("start_online");
    let hold = auth.handle(MetalRequest::propose("pty-bus-hold", "hold"));
    assert!(hold.ok, "hold refused: {hold:?}");
    let writes = auth.physical_writes();
    drop(guard);
    std::thread::sleep(Duration::from_millis(50));
    let lost = auth.handle(MetalRequest::propose("pty-bus-lost", "hold"));
    assert!(!lost.ok, "dead bus must refuse: {lost:?}");
    assert!(
        lost.violations
            .iter()
            .any(|v| v.contains("online_hardware_disconnected")
                || v.contains("dxl_io")
                || v.contains("hardware_session_requires_online_restart")),
        "I/O loss must be measured, not a vacuous miss: {lost:?}"
    );
    assert!(
        !lost
            .violations
            .iter()
            .any(|v| v.contains("software_watchdog_miss")),
        "{lost:?}"
    );
    assert_eq!(auth.physical_writes(), writes);
    let rec = auth.handle(recover_req("pty-bus-rec"));
    assert!(!rec.ok, "recover after bus loss must refuse: {rec:?}");
    assert!(
        rec.violations
            .iter()
            .any(|v| v.contains("hardware_session_requires_online_restart")),
        "recover must not resurrect after live I/O loss: {rec:?}"
    );
    let again = auth.handle(MetalRequest::propose("pty-bus-again", "hold"));
    assert!(!again.ok, "{again:?}");
    assert_eq!(auth.physical_writes(), writes);
    let _ = std::fs::remove_dir_all(&root);
}
