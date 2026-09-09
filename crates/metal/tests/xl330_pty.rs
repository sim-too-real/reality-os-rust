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

/// Journal+seal fsync on the runner disk exceeds 100 ms and labels identity
/// refuse as `software_watchdog_miss`. Prefer tmpfs (`/dev/shm`), same
/// constraint as `scripts/metal-campaign.sh`.
fn metal_test_root(name: &str) -> PathBuf {
    let base = if Path::new("/dev/shm").is_dir() {
        PathBuf::from("/dev/shm")
    } else {
        std::env::temp_dir()
    };
    let dir = base.join(format!("realityos-metal-{name}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    dir
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
    spawn_responder_env(&[])
}

fn spawn_responder_env(vars: &[(&str, &str)]) -> (ChildGuard, String) {
    let script = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/xl330_responder.py");
    assert!(script.is_file(), "missing {}", script.display());
    let mut cmd = Command::new("python3");
    cmd.arg(&script)
        .stdout(Stdio::piped())
        .stderr(Stdio::inherit());
    for (k, v) in vars {
        cmd.env(k, v);
    }
    let mut child = cmd.spawn().expect("python3 xl330_responder");
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
    let root = metal_test_root("pty-driver");
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
fn xl330_pty_alert_bit_is_not_instruction_failure() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[("REALITYOS_METAL_PTY_ALERT", "1")]);
    let root = metal_test_root("pty-alert");
    let cfg = MetalConfig::example(&tty);
    let mut driver = Xl330Driver::open(cfg, &root).expect("open despite Protocol 2.0 ALERT");
    driver
        .read_sensor(0.0)
        .expect("sensor must accept STATUS_ALERT");
    driver
        .write_action(&[0.0], &ActionParams::empty())
        .expect("hold must accept STATUS_ALERT");
    assert_eq!(recorded_writes(root.join("bus")), 1);
    driver.close();
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_status_return_level_zero_can_still_identify() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[("REALITYOS_METAL_PTY_SRL0", "1")]);
    let root = metal_test_root("pty-srl0");
    let cfg = MetalConfig::example(&tty);
    let mut driver = Xl330Driver::open(cfg, &root)
        .expect("Wizard Status Return Level 0 must not block identify (poke 2, then READ)");
    assert_eq!(driver.measured().model, 1190);
    driver
        .write_action(&[0.0], &ActionParams::empty())
        .expect("hold after SRL poke");
    assert_eq!(recorded_writes(root.join("bus")), 1);
    driver.close();
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_nudge_steps_inward_at_wizard_max_limit() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[("REALITYOS_METAL_PTY_AT_MAX", "1")]);
    let root = metal_test_root("pty-at-max");
    let cfg = MetalConfig::example(&tty);
    let mut driver = Xl330Driver::open(cfg, &root).expect("identify at Wizard max");
    driver.read_sensor(0.0).expect("sensor before inward nudge");
    assert_eq!(driver.last_present_position(), 2048);
    driver
        .write_action(&[0.05], &ActionParams::empty())
        .expect("nudge at max must step inward, not NAK");
    assert_eq!(driver.last_goal_position(), Some(2046));
    assert_eq!(recorded_writes(root.join("bus")), 1);
    driver.close();
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_pwm_operating_mode_is_forced_to_position() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[("REALITYOS_METAL_PTY_PWM", "1")]);
    let root = metal_test_root("pty-pwm");
    let cfg = MetalConfig::example(&tty);
    let mut driver = Xl330Driver::open(cfg, &root).expect("PWM EEPROM mode must not block setup");
    driver
        .write_action(&[0.0], &ActionParams::empty())
        .expect("hold after forcing position mode");
    assert_eq!(recorded_writes(root.join("bus")), 1);
    driver.close();
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_raises_wizard_velocity_limit_so_nudge_can_finish() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[("REALITYOS_METAL_PTY_SLOW_VEL", "1")]);
    let root = metal_test_root("pty-slow-vel");
    let cfg = MetalConfig::example(&tty);
    let mut driver = Xl330Driver::open(cfg, &root).expect("raise Velocity Limit 1 to profile 20");
    assert_eq!(driver.applied_velocity_limit(), 20);
    driver.read_sensor(0.0).expect("sensor");
    driver
        .write_action(&[0.05], &ActionParams::empty())
        .expect("nudge after raising velocity limit");
    assert_eq!(recorded_writes(root.join("bus")), 1);
    driver.close();
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_raises_wizard_zero_p_gain_so_nudge_can_track() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[("REALITYOS_METAL_PTY_ZERO_P", "1")]);
    let root = metal_test_root("pty-zero-p");
    let cfg = MetalConfig::example(&tty);
    let mut driver = Xl330Driver::open(cfg, &root).expect("raise Position P Gain 0 to factory 400");
    assert_eq!(driver.applied_position_p_gain(), 400);
    driver.read_sensor(0.0).expect("sensor");
    let before = driver.last_present_position();
    driver
        .write_action(&[0.05], &ActionParams::empty())
        .expect("nudge after restoring P gain");
    driver.read_sensor(0.1).expect("sensor");
    let after = driver.last_present_position();
    assert_ne!(
        after, before,
        "Wizard P=0 must not leave present stuck after setup"
    );
    assert_eq!(recorded_writes(root.join("bus")), 1);
    driver.close();
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_refuses_torque_when_present_cannot_be_read() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[("REALITYOS_METAL_PTY_NO_PRESENT", "1")]);
    let root = metal_test_root("pty-no-present");
    let cfg = MetalConfig::example(&tty);
    let err = match Xl330Driver::open(cfg, &root) {
        Ok(_) => panic!("unreadable present must not torque-on with invented goal 0"),
        Err(e) => e,
    };
    assert!(
        err.to_string()
            .contains("dxl_present_unreadable_before_torque"),
        "got {err}"
    );
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_syncs_stale_goal_before_torque_so_present_does_not_jump() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder();
    let root = metal_test_root("pty-stale-goal");
    let cfg = MetalConfig::example(&tty);
    let mut driver = Xl330Driver::open(cfg, &root).expect("sync goal to present before torque-on");
    driver.read_sensor(0.0).expect("sensor");
    assert_eq!(
        driver.last_present_position(),
        2048,
        "stale Goal Position 0 must not yank present on torque-on"
    );
    assert_eq!(driver.last_goal_position(), Some(2048));
    driver.close();
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_clears_bus_watchdog_error_so_goal_writes_are_live() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[("REALITYOS_METAL_PTY_BUS_WATCHDOG", "1")]);
    let root = metal_test_root("pty-bus-wd");
    let cfg = MetalConfig::example(&tty);
    let mut driver = Xl330Driver::open(cfg, &root).expect("clear Bus Watchdog 0xFF before goal");
    assert_eq!(driver.applied_bus_watchdog(), 0);
    driver.read_sensor(0.0).expect("sensor");
    driver
        .write_action(&[0.05], &ActionParams::empty())
        .expect("goal write after clearing watchdog error");
    assert_eq!(recorded_writes(root.join("bus")), 1);
    driver.close();
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_time_based_drive_mode_is_forced_velocity_based() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[("REALITYOS_METAL_PTY_TIME_BASED", "1")]);
    let root = metal_test_root("pty-time-based");
    let cfg = MetalConfig::example(&tty);
    let mut driver =
        Xl330Driver::open(cfg, &root).expect("time-based Drive Mode must not block setup");
    assert_eq!(driver.applied_drive_mode(), 0);
    driver
        .write_action(&[0.0], &ActionParams::empty())
        .expect("hold after forcing velocity-based drive");
    assert_eq!(recorded_writes(root.join("bus")), 1);
    driver.close();
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_refuses_torque_when_vin_below_wizard_min() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[("REALITYOS_METAL_PTY_HIGH_MINVIN", "1")]);
    let root = metal_test_root("pty-minvin");
    let cfg = MetalConfig::example(&tty);
    let err = match Xl330Driver::open(cfg, &root) {
        Ok(_) => panic!("5.0 V must not torque under min 6.0 V"),
        Err(e) => e,
    };
    assert!(
        err.to_string().contains("dxl_vin_outside_wizard_limits"),
        "got {err}"
    );
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_discover_finds_wizard_id_via_broadcast() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[("REALITYOS_METAL_PTY_ID", "7")]);
    let root = metal_test_root("pty-id7");
    let cfg = MetalConfig::example(&tty);
    assert_eq!(cfg.servo_id, 1);
    let (mut driver, bound) =
        Xl330Driver::open_discovering(cfg, &root).expect("broadcast PING must find Wizard ID 7");
    assert_eq!(bound.servo_id, 7);
    assert_eq!(driver.measured().actuator_id, "xl330:7");
    assert!(
        !driver.torque_is_enabled(),
        "probe/discover must not torque-on"
    );
    driver.close();
    assert_eq!(
        recorded_writes(root.join("bus")),
        0,
        "identify-only close must not write torque-off"
    );
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_start_online_hold_is_not_a_metal_proof() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder();
    assert!(is_pty_path(std::path::Path::new(&tty)));
    let root = metal_test_root("pty-online");
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
    let root = metal_test_root("pty-idle");
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

#[test]
fn xl330_pty_same_command_id_is_not_a_second_write() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder();
    let root = metal_test_root("pty-replay");
    bind_pty_cfg(&root, &tty);
    let mut auth = MetalAuthority::start(&root, true).expect("start_online");
    let hold = auth.handle(MetalRequest::propose("metal-hold", "hold"));
    assert!(hold.ok, "hold: {hold:?}");
    let writes = auth.physical_writes();
    let replay = auth.handle(MetalRequest::propose("metal-hold", "hold"));
    assert!(!replay.ok, "replay must refuse: {replay:?}");
    assert!(
        replay
            .violations
            .iter()
            .any(|v| v.contains("replayed command_id")),
        "ledger must see the first id: {replay:?}"
    );
    assert_eq!(auth.physical_writes(), writes);
    let _ = std::fs::remove_dir_all(&root);
}

fn recover_req(id: &str) -> MetalRequest {
    let mut r = MetalRequest::propose(id, "hold");
    r.op = "recover".into();
    r
}

fn bind_pty_cfg(root: &Path, tty: &str) -> MetalConfig {
    let mut cfg = MetalConfig::example(tty);
    cfg.campaign_hooks = true;
    {
        let driver = Xl330Driver::open(cfg.clone(), root).expect("identify");
        let measured = driver.measured();
        cfg.expected_serial = measured.serial;
        cfg.expected_firmware = measured.firmware_id;
    }
    cfg.save(root.join(CONFIG_FILE)).unwrap();
    cfg
}

/// Campaign order: hold → firmware kill → process --restart → hold →
/// disconnect kill → process --restart → hold. A leftover journal ESTOP or
/// continuity refuse here aborts the XL330 run before crash-replay.
#[test]
fn xl330_pty_campaign_restarts_are_live_after_identity_and_disconnect() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder();
    let root = metal_test_root("pty-campaign-restart");
    bind_pty_cfg(&root, &tty);

    let mut writes;
    {
        let mut auth = MetalAuthority::start(&root, true).expect("first-online");
        let hold = auth.handle(MetalRequest::propose("pty-cr-hold", "hold"));
        assert!(hold.ok, "baseline hold: {hold:?}");
        writes = auth.physical_writes();
        std::fs::write(
            root.join("bus/hot_swap.json"),
            r#"{"firmware_id":"xl330-m288:1190:255"}"#,
        )
        .unwrap();
        let fw = auth.handle(MetalRequest::propose("pty-cr-fw", "hold"));
        assert!(
            !fw.ok
                && fw
                    .violations
                    .iter()
                    .any(|v| v.contains("hardware_firmware_mismatch")),
            "firmware must kill this instance: {fw:?}"
        );
        assert_eq!(auth.physical_writes(), writes);
        let rec = auth.handle(recover_req("pty-cr-fw-rec"));
        assert!(
            !rec.ok
                && rec
                    .violations
                    .iter()
                    .any(|v| v.contains("hardware_session_requires_online_restart")),
            "recover after identity: {rec:?}"
        );
    }
    std::fs::remove_file(root.join("bus/hot_swap.json")).unwrap();

    {
        let mut auth = MetalAuthority::start(&root, false)
            .expect("restart after identity must be a live instance");
        let hold = auth.handle(MetalRequest::propose("pty-cr-hold2", "hold"));
        assert!(
            hold.ok,
            "campaign require_live_session after identity restart: {hold:?}"
        );
        writes = auth.physical_writes();
        std::fs::write(root.join("bus/force_disconnect"), b"1").unwrap();
        let disc = auth.handle(MetalRequest::propose("pty-cr-disc", "hold"));
        assert!(
            !disc.ok
                && disc
                    .violations
                    .iter()
                    .any(|v| v.contains("online_hardware_disconnected")),
            "disconnect must kill this instance: {disc:?}"
        );
        assert_eq!(auth.physical_writes(), writes);
    }
    std::fs::remove_file(root.join("bus/force_disconnect")).unwrap();

    {
        let mut auth = MetalAuthority::start(&root, false)
            .expect("restart after disconnect must be live for crash-replay");
        let hold = auth.handle(MetalRequest::propose("pty-cr-hold3", "hold"));
        assert!(
            hold.ok,
            "crash-replay serve must accept a hold after disconnect restart: {hold:?}"
        );
        assert!(auth.physical_writes() > writes);
    }
    let _ = std::fs::remove_dir_all(&root);
}

/// Continuity counts `replayed` toward `repeated_refuse_n` (default 3).
/// The campaign metal-hold replay plus two crash-replays latch the next
/// `--restart` as `abort_latched:replayed` unless a successful write resets
/// the counter. That is why after_write_before_ack never reached crash_if.
#[test]
fn xl330_pty_third_replay_refuse_latches_restart_unless_reset_hold() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder();
    let root = metal_test_root("pty-replay-latch");
    bind_pty_cfg(&root, &tty);

    {
        let mut auth = MetalAuthority::start(&root, true).expect("first-online");
        assert!(auth.handle(MetalRequest::propose("rl-a", "hold")).ok);
        // Each replay refuse also abort-latches *this* instance. Three
        // journaled refuses (no successful driver_write) latch the next
        // --restart via continuity repeated_refuse_n.
        assert!(!auth.handle(MetalRequest::propose("rl-a", "hold")).ok);
        assert!(!auth.handle(MetalRequest::propose("rl-a", "hold")).ok);
        assert!(!auth.handle(MetalRequest::propose("rl-a", "hold")).ok);
    }
    {
        let mut auth = MetalAuthority::start(&root, false).expect("restart after 3 replays");
        let hold = auth.handle(MetalRequest::propose("rl-d", "hold"));
        assert!(
            !hold.ok
                && hold
                    .violations
                    .iter()
                    .any(|v| v.contains("abort_latched:replayed")),
            "third replay refuse must latch the next restart: {hold:?}"
        );
    }

    let root2 = metal_test_root("pty-replay-reset");
    bind_pty_cfg(&root2, &tty);
    {
        let mut auth = MetalAuthority::start(&root2, true).expect("first-online");
        assert!(auth.handle(MetalRequest::propose("rr-a", "hold")).ok);
        assert!(!auth.handle(MetalRequest::propose("rr-a", "hold")).ok);
        assert!(!auth.handle(MetalRequest::propose("rr-a", "hold")).ok);
        // Same instance is abort-latched; reset hold must be a new process.
    }
    {
        let mut auth =
            MetalAuthority::start(&root2, false).expect("restart while refuse_n is 2 must be live");
        assert!(
            auth.handle(MetalRequest::propose("rr-reset", "hold")).ok,
            "reset hold must succeed while refuse_n is still 2"
        );
        assert!(!auth.handle(MetalRequest::propose("rr-a", "hold")).ok);
    }
    {
        let mut auth =
            MetalAuthority::start(&root2, false).expect("restart after reset hold must be live");
        let hold = auth.handle(MetalRequest::propose("rr-d", "hold"));
        assert!(
            hold.ok,
            "successful write must reset replay refuse count: {hold:?}"
        );
    }
    let _ = std::fs::remove_dir_all(&root);
    let _ = std::fs::remove_dir_all(&root2);
}

#[test]
fn xl330_pty_firmware_mismatch_kills_session_not_watchdog() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder();
    let root = metal_test_root("pty-fw");
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
    let root = metal_test_root("pty-disc");
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
    let root = metal_test_root("pty-busloss");
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
