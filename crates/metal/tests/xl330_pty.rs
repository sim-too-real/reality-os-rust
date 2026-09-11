//! PTY Protocol 2.0 stand-in. Proves driver identity latch + echo scan.
//! Not a metal proof. Does not write docs/metal_proof.json.

use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::sync::Mutex;
use std::time::Duration;

use realityos_metal::authority::MetalAuthority;
use realityos_metal::config::{MetalConfig, CONFIG_FILE};
use realityos_metal::egress::{recorded_serial_tx, recorded_writes};
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
    assert_eq!(before.firmware_id, "xl330-m288:1200:46");
    assert_eq!(before.model, 1200);
    driver
        .read_sensor(0.0)
        .expect("sensor over echoed PTY status");
    let moving = std::fs::read_to_string(root.join("bus").join("moving")).unwrap_or_default();
    assert_eq!(
        moving.trim(),
        "0",
        "sensor must persist XL330 Moving (addr 122) for campaign settle"
    );
    let after = driver.measured();
    assert_eq!(
        after.firmware_id, "xl330-m288:1200:46",
        "sensor samples must not clobber latched firmware"
    );
    driver
        .write_action(&[0.0], &ActionParams::empty())
        .expect("hold write");
    assert_eq!(recorded_writes(root.join("bus")), 1);
    driver
        .write_action(&[0.2], &ActionParams::empty())
        .expect("nudge write");
    driver.read_sensor(0.0).expect("sensor after nudge");
    let moving = std::fs::read_to_string(root.join("bus").join("moving")).unwrap_or_default();
    assert_eq!(
        moving.trim(),
        "1",
        "first motion-block after a goal step must report Moving"
    );
    driver.read_sensor(0.0).expect("sensor after moving");
    let moving = std::fs::read_to_string(root.join("bus").join("moving")).unwrap_or_default();
    assert_eq!(
        moving.trim(),
        "0",
        "next motion-block must clear Moving so campaign settle can finish"
    );
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
    let pkt = driver
        .read_sensor(0.0)
        .expect("sensor after ALERT goal write");
    let hw = pkt
        .samples
        .iter()
        .find(|(k, _)| k == "hw_error")
        .map(|(_, v)| *v);
    assert_eq!(
        hw,
        Some(4.0),
        "live hw_error sample must re-read register 70 after STATUS_ALERT, not keep the setup-time 0"
    );
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
    assert_eq!(driver.measured().model, 1200);
    driver
        .write_action(&[0.0], &ActionParams::empty())
        .expect("hold after SRL poke");
    assert_eq!(recorded_writes(root.join("bus")), 1);
    driver.close();
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_outbound_nudge_at_cage_edge_is_refused() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[("REALITYOS_METAL_PTY_AT_MAX", "1")]);
    let root = metal_test_root("pty-at-max");
    let cfg = MetalConfig::example(&tty);
    let mut driver = Xl330Driver::open(cfg, &root).expect("identify at Wizard max");
    driver
        .read_sensor(0.0)
        .expect("sensor before outbound nudge");
    assert_eq!(driver.last_present_position(), 2048);
    let tx_before = recorded_serial_tx(root.join("bus"));
    let err = driver
        .write_action(&[0.05], &ActionParams::empty())
        .expect_err("outbound goal past the startup cage must refuse, not step inward");
    assert!(
        err.to_string().contains("experiment_cage_violation"),
        "{err}"
    );
    assert_eq!(recorded_serial_tx(root.join("bus")), tx_before);
    driver.close();
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_refuses_setup_when_leftover_window_cannot_host_nudge() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[("REALITYOS_METAL_PTY_TIGHT_WINDOW", "1")]);
    let root = metal_test_root("pty-tight-window");
    let cfg = MetalConfig::example(&tty);
    let err = match Xl330Driver::open(cfg, &root) {
        Ok(_) => panic!("20-tick leftover Wizard window must not torque-on"),
        Err(e) => e,
    };
    assert!(
        err.to_string()
            .contains("metal_experiment_cage_no_inbound_step"),
        "got {err}"
    );
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_refuses_setup_when_edge36_would_abort_latch_after_propose() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[("REALITYOS_METAL_PTY_EDGE36", "1")]);
    let root = metal_test_root("pty-edge36");
    let cfg = MetalConfig::example(&tty);
    let err = match Xl330Driver::open(cfg, &root) {
        Ok(_) => panic!(
            "36-tick leftover edge window must not torque-on; hold+propose hunt abort-latches"
        ),
        Err(e) => e,
    };
    assert!(
        err.to_string()
            .contains("metal_experiment_cage_no_inbound_step"),
        "got {err}"
    );
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_refuses_setup_when_edge_window_is_eaten_by_hold_still() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[("REALITYOS_METAL_PTY_EDGE32", "1")]);
    let root = metal_test_root("pty-edge32");
    let cfg = MetalConfig::example(&tty);
    let err = match Xl330Driver::open(cfg, &root) {
        Ok(_) => panic!("32-tick leftover edge window must not torque-on; hold hunt eats the step"),
        Err(e) => e,
    };
    assert!(
        err.to_string()
            .contains("metal_experiment_cage_no_inbound_step"),
        "got {err}"
    );
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_inbound_nudge_at_wizard_max_window_tracks() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[("REALITYOS_METAL_PTY_AT_MAX", "1")]);
    let root = metal_test_root("pty-at-max-inbound");
    let cfg = MetalConfig::example(&tty);
    let action = realityos_metal::config::pick_inbound_nudge_action(
        2048,
        2000,
        2048,
        cfg.max_position_delta_ticks,
        cfg.tau_max,
    )
    .expect("Wizard leftover max==present still has inbound room");
    assert!(action < 0.0, "plus 32 ticks is outbound of max=2048");
    let mut driver = Xl330Driver::open(cfg, &root).expect("identify at Wizard max");
    driver
        .read_sensor(0.0)
        .expect("sensor before inbound nudge");
    assert_eq!(driver.last_present_position(), 2048);
    let (emin, emax) = driver.experiment_cage();
    assert_eq!((emin, emax), (2000, 2048));
    driver
        .write_action(&[0.2], &ActionParams::empty())
        .expect_err("hardcoded +0.2 must still refuse; do not clamp inward");
    driver
        .write_action(&[action], &ActionParams::empty())
        .expect("inbound -0.2 must write the 32-tick step");
    driver.read_sensor(0.0).expect("sensor after inbound nudge");
    let after = driver.last_present_position();
    assert_eq!(after, 2016, "PTY lands on the inbound 32-tick goal");
    assert!(
        after >= emin && after <= emax,
        "inbound present {after} escaped cage {emin}..{emax}"
    );
    driver.close();
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_at_max_restart_after_inbound_nudge_still_hosts_step() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[("REALITYOS_METAL_PTY_AT_MAX", "1")]);
    let root = metal_test_root("pty-at-max-restart");
    let cfg = MetalConfig::example(&tty);
    let mut first = Xl330Driver::open(cfg.clone(), &root).expect("identify at Wizard max");
    first
        .write_action(&[-0.2], &ActionParams::empty())
        .expect("inbound -0.2");
    first.read_sensor(0.0).expect("sensor after inbound nudge");
    assert_eq!(first.last_present_position(), 2016);
    first.abandon_without_eeprom_restore_for_test();
    drop(first);
    let mut second = Xl330Driver::open(cfg, &root)
        .expect("restart cage around 2016 must flip off +32 that slack-misses leftover max");
    assert_eq!(second.last_present_position(), 2016);
    let (emin, emax) = second.experiment_cage();
    assert_eq!((emin, emax), (1968, 2048));
    second
        .write_action(&[-0.2], &ActionParams::empty())
        .expect("inbound -0.2 must still fit after AT_MAX restart");
    second.close();
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
fn xl330_pty_refuses_when_operating_mode_write_does_not_stick() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[
        ("REALITYOS_METAL_PTY_PWM", "1"),
        ("REALITYOS_METAL_PTY_DROP_OPERATING_MODE", "1"),
    ]);
    let root = metal_test_root("pty-drop-operating-mode");
    let cfg = MetalConfig::example(&tty);
    let err = match Xl330Driver::open(cfg, &root) {
        Ok(_) => panic!("ACK'd-but-dropped position-mode write must not look like mode 3"),
        Err(e) => e,
    };
    assert!(
        err.to_string().contains("dxl_operating_mode_unverified"),
        "got {err}"
    );
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
fn xl330_pty_refuses_when_velocity_limit_write_does_not_stick() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[
        ("REALITYOS_METAL_PTY_SLOW_VEL", "1"),
        ("REALITYOS_METAL_PTY_DROP_VELOCITY_LIMIT", "1"),
    ]);
    let root = metal_test_root("pty-drop-velocity-limit");
    let cfg = MetalConfig::example(&tty);
    let err = match Xl330Driver::open(cfg, &root) {
        Ok(_) => panic!("ACK'd-but-dropped Velocity Limit write must not look like 20"),
        Err(e) => e,
    };
    assert!(
        err.to_string().contains("dxl_velocity_limit_unverified"),
        "got {err}"
    );
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_lowers_wizard_moving_threshold_so_moving_can_assert() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[
        ("REALITYOS_METAL_PTY_HIGH_MOVING_THRESHOLD", "1"),
        ("REALITYOS_METAL_PTY_UNREAD_MOVING_THRESHOLD", "1"),
    ]);
    let root = metal_test_root("pty-move-th");
    let cfg = MetalConfig::example(&tty);
    let mut driver =
        Xl330Driver::open(cfg, &root).expect("lower Moving Threshold 1023 to factory 10");
    assert_eq!(driver.applied_moving_threshold(), 10);
    driver.close();
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_refuses_when_moving_threshold_write_does_not_stick() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[
        ("REALITYOS_METAL_PTY_HIGH_MOVING_THRESHOLD", "1"),
        ("REALITYOS_METAL_PTY_DROP_MOVING_THRESHOLD", "1"),
    ]);
    let root = metal_test_root("pty-drop-moving-threshold");
    let cfg = MetalConfig::example(&tty);
    let err = match Xl330Driver::open(cfg, &root) {
        Ok(_) => panic!("ACK'd-but-dropped Moving Threshold write must not look like 10"),
        Err(e) => e,
    };
    assert!(
        err.to_string().contains("dxl_moving_threshold_unverified"),
        "got {err}"
    );
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_forces_wizard_rc_protocol_type_to_protocol_2() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[("REALITYOS_METAL_PTY_PROTOCOL_RC", "1")]);
    let root = metal_test_root("pty-proto-rc");
    let cfg = MetalConfig::example(&tty);
    let mut driver = Xl330Driver::open(cfg, &root).expect("Wizard RC protocol type must become 2");
    assert_eq!(driver.applied_protocol_type(), 2);
    driver.close();
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_refuses_when_protocol_type_write_does_not_stick() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[
        ("REALITYOS_METAL_PTY_PROTOCOL_RC", "1"),
        ("REALITYOS_METAL_PTY_DROP_PROTOCOL_TYPE", "1"),
    ]);
    let root = metal_test_root("pty-drop-protocol-type");
    let cfg = MetalConfig::example(&tty);
    let err = match Xl330Driver::open(cfg, &root) {
        Ok(_) => panic!("ACK'd-but-dropped Protocol Type write must not look like 2"),
        Err(e) => e,
    };
    assert!(
        err.to_string().contains("dxl_protocol_type_unverified"),
        "got {err}"
    );
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_refuses_when_secondary_id_write_does_not_stick() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[
        ("REALITYOS_METAL_PTY_SECONDARY", "1"),
        ("REALITYOS_METAL_PTY_DROP_SECONDARY_ID", "1"),
    ]);
    let root = metal_test_root("pty-drop-secondary-id");
    let cfg = MetalConfig::example(&tty);
    let err = match Xl330Driver::open(cfg, &root) {
        Ok(_) => panic!("ACK'd-but-dropped Secondary ID write must not look like 255"),
        Err(e) => e,
    };
    assert!(
        err.to_string().contains("dxl_secondary_id_unverified"),
        "got {err}"
    );
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_zeros_wizard_position_id_so_nudge_stays_in_cage() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[
        ("REALITYOS_METAL_PTY_WIZARD_PID", "1"),
        ("REALITYOS_METAL_PTY_UNREAD_PID", "1"),
    ]);
    let root = metal_test_root("pty-pos-id");
    let cfg = MetalConfig::example(&tty);
    let mut driver = Xl330Driver::open(cfg, &root).expect("zero Wizard position I/D");
    assert_eq!(driver.applied_position_i_gain(), 0);
    assert_eq!(driver.applied_position_d_gain(), 0);
    driver.close();
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_restores_factory_pwm_slope_when_wizard_zero() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[("REALITYOS_METAL_PTY_ZERO_PWM_SLOPE", "1")]);
    let root = metal_test_root("pty-pwm-slope");
    let cfg = MetalConfig::example(&tty);
    let mut driver = Xl330Driver::open(cfg, &root).expect("restore factory PWM Slope 140");
    assert_eq!(driver.applied_pwm_slope(), 140);
    driver.close();
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_matches_goal_pwm_to_cap_when_wizard_zero() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[("REALITYOS_METAL_PTY_ZERO_GOAL_PWM", "1")]);
    let root = metal_test_root("pty-zero-goal-pwm");
    let cfg = MetalConfig::example(&tty);
    let mut driver =
        Xl330Driver::open(cfg, &root).expect("Wizard Goal PWM 0 must match the PWM cap");
    assert_eq!(
        driver.applied_goal_pwm(),
        i16::try_from(driver.applied_pwm_limit()).expect("pwm cap fits i16"),
        "Position Mode uses Goal PWM as the live limiter"
    );
    driver.read_sensor(0.0).expect("sensor");
    let before = driver.last_present_position();
    driver
        .write_action(&[0.2], &ActionParams::empty())
        .expect("nudge after restoring Goal PWM");
    driver.read_sensor(0.1).expect("sensor");
    let after = driver.last_present_position();
    assert_ne!(
        after, before,
        "Wizard Goal PWM 0 must not leave present stuck after setup"
    );
    driver.close();
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_matches_goal_pwm_to_cap_when_wizard_too_low() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[("REALITYOS_METAL_PTY_LOW_GOAL_PWM", "1")]);
    let root = metal_test_root("pty-low-goal-pwm");
    let cfg = MetalConfig::example(&tty);
    let mut driver =
        Xl330Driver::open(cfg, &root).expect("Wizard Goal PWM 1 must match the PWM cap");
    assert_eq!(
        driver.applied_goal_pwm(),
        i16::try_from(driver.applied_pwm_limit()).expect("pwm cap fits i16")
    );
    driver.read_sensor(0.0).expect("sensor");
    let before = driver.last_present_position();
    driver
        .write_action(&[0.2], &ActionParams::empty())
        .expect("nudge after raising a legal-but-too-small Goal PWM");
    driver.read_sensor(0.1).expect("sensor");
    assert_ne!(
        driver.last_present_position(),
        before,
        "Wizard Goal PWM 1 must not leave present stuck after setup"
    );
    driver.close();
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_restores_factory_pwm_slope_when_wizard_too_low() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[("REALITYOS_METAL_PTY_LOW_PWM_SLOPE", "1")]);
    let root = metal_test_root("pty-low-pwm-slope");
    let cfg = MetalConfig::example(&tty);
    let mut driver =
        Xl330Driver::open(cfg, &root).expect("raise Wizard PWM Slope 1 to factory 140");
    assert_eq!(driver.applied_pwm_slope(), 140);
    driver.read_sensor(0.0).expect("sensor");
    let before = driver.last_present_position();
    driver
        .write_action(&[0.2], &ActionParams::empty())
        .expect("nudge after restoring a legal-but-too-slow slope");
    driver.read_sensor(0.1).expect("sensor");
    let after = driver.last_present_position();
    assert_ne!(
        after, before,
        "Wizard PWM Slope 1 must not leave present stuck after setup"
    );
    driver.close();
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_refuses_when_pwm_slope_write_does_not_stick() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[
        ("REALITYOS_METAL_PTY_ZERO_PWM_SLOPE", "1"),
        ("REALITYOS_METAL_PTY_DROP_PWM_SLOPE", "1"),
    ]);
    let root = metal_test_root("pty-drop-pwm-slope");
    let cfg = MetalConfig::example(&tty);
    let err = match Xl330Driver::open(cfg, &root) {
        Ok(_) => panic!("ACK'd-but-dropped PWM Slope write must not look like factory 140"),
        Err(e) => e,
    };
    assert!(
        err.to_string().contains("dxl_pwm_slope_unverified"),
        "got {err}"
    );
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_refuses_when_position_p_write_does_not_stick() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[
        ("REALITYOS_METAL_PTY_HIGH_P", "1"),
        ("REALITYOS_METAL_PTY_DROP_POSITION_P", "1"),
    ]);
    let root = metal_test_root("pty-drop-position-p");
    let cfg = MetalConfig::example(&tty);
    let err = match Xl330Driver::open(cfg, &root) {
        Ok(_) => panic!("ACK'd-but-dropped Position P write must not look like factory 400"),
        Err(e) => e,
    };
    assert!(
        err.to_string().contains("dxl_position_p_unverified"),
        "got {err}"
    );
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_refuses_when_profile_write_does_not_stick() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[("REALITYOS_METAL_PTY_DROP_PROFILE", "1")]);
    let root = metal_test_root("pty-drop-profile");
    let cfg = MetalConfig::example(&tty);
    let err = match Xl330Driver::open(cfg, &root) {
        Ok(_) => panic!("ACK'd-but-dropped profile write must not look like vel=20 accel=10"),
        Err(e) => e,
    };
    assert!(
        err.to_string().contains("dxl_profile_unverified"),
        "got {err}"
    );
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_zeros_wizard_feedforward_so_nudge_stays_bounded() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[
        ("REALITYOS_METAL_PTY_FEEDFORWARD", "1"),
        ("REALITYOS_METAL_PTY_UNREAD_FF", "1"),
    ]);
    let root = metal_test_root("pty-ff");
    let cfg = MetalConfig::example(&tty);
    let mut driver = Xl330Driver::open(cfg, &root).expect("zero Wizard feedforward");
    assert_eq!(driver.applied_feedforward_1st(), 0);
    assert_eq!(driver.applied_feedforward_2nd(), 0);
    driver.close();
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_rematches_goal_when_torque_on_resets_present() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[("REALITYOS_METAL_PTY_TORQUE_JUMP_PRESENT", "1")]);
    let root = metal_test_root("pty-tq-jump");
    let cfg = MetalConfig::example(&tty);
    let mut driver =
        Xl330Driver::open(cfg, &root).expect("rematch goal after torque-on present reset");
    assert_eq!(driver.last_present_position(), 2064);
    assert_eq!(driver.last_goal_position(), Some(2064));
    assert_eq!(
        driver.startup_present(),
        2064,
        "register wrap is not certified excursion; cage must follow the parked present"
    );
    let (emin, emax) = driver.experiment_cage();
    assert_eq!(
        (emin, emax),
        (2016, 2112),
        "pre-reset cage 2000..2096 leaves +32+slack past 2096 and abort-latches"
    );
    driver.read_sensor(0.0).expect("sensor");
    assert_eq!(
        driver.last_present_position(),
        2064,
        "goal must already match the post-torque present so the horn does not yank"
    );
    driver
        .write_action(&[0.2], &ActionParams::empty())
        .expect("inbound +0.2 must fit the recentered cage");
    driver.close();
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_recenters_on_live_present_after_torque_off_drift() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[
        ("REALITYOS_METAL_PTY_TORQUE_JUMP_PRESENT", "1"),
        ("REALITYOS_METAL_PTY_DRIFT_ON_TORQUE_OFF", "1"),
    ]);
    let root = metal_test_root("pty-tq-jump-drift");
    let cfg = MetalConfig::example(&tty);
    let mut driver = Xl330Driver::open(cfg, &root)
        .expect("recenter must park on the post-torque-off present, not yank back to 2064");
    assert_eq!(
        driver.last_present_position(),
        2084,
        "torque-off settle +20 from the 2064 wrap must become the park"
    );
    assert_eq!(driver.last_goal_position(), Some(2084));
    assert_eq!(driver.startup_present(), 2084);
    let (emin, emax) = driver.experiment_cage();
    assert_eq!(
        (emin, emax),
        (2036, 2132),
        "cage around stale 2064 (2016..2112) leaves +32 from 2084 past 2112"
    );
    driver
        .write_action(&[0.2], &ActionParams::empty())
        .expect("inbound +0.2 must fit the cage around the live park");
    driver.close();
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_refuses_second_torque_wrap_after_recenter() {
    let _serial = pty_serial();
    let (_guard, tty) =
        spawn_responder_env(&[("REALITYOS_METAL_PTY_TORQUE_JUMP_EVERY_ENABLE", "1")]);
    let root = metal_test_root("pty-tq-jump-every");
    let cfg = MetalConfig::example(&tty);
    let err = match Xl330Driver::open(cfg, &root) {
        Ok(_) => panic!("a wrap on every torque-on must not loop EEPROM rewrites"),
        Err(e) => e,
    };
    assert!(
        err.to_string()
            .contains("dxl_present_jumped_twice_after_torque_recenter"),
        "got {err}"
    );
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_delayed_present_does_not_teleport_on_goal_write() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[("REALITYOS_METAL_PTY_DELAY_MOTION", "1")]);
    let root = metal_test_root("pty-delay-motion");
    let cfg = MetalConfig::example(&tty);
    let mut driver = Xl330Driver::open(cfg, &root).expect("identify");
    driver.read_sensor(0.0).expect("sensor before nudge");
    assert_eq!(driver.last_present_position(), 2048);
    driver
        .write_action(&[0.2], &ActionParams::empty())
        .expect("nudge");
    driver
        .read_sensor(0.0)
        .expect("first live sample must still be the parked present");
    assert_eq!(
        driver.last_present_position(),
        2048,
        "a real XL330 does not teleport present on the goal write"
    );
    for _ in 0..4 {
        driver.read_sensor(0.0).expect("travel sample");
    }
    assert_eq!(
        driver.last_present_position(),
        2080,
        "present must reach the 32-tick goal after the delayed profile"
    );
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
fn xl330_pty_caps_wizard_high_p_gain_so_nudge_stays_in_cage() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[("REALITYOS_METAL_PTY_HIGH_P", "1")]);
    let root = metal_test_root("pty-high-p");
    let cfg = MetalConfig::example(&tty);
    let mut driver =
        Xl330Driver::open(cfg, &root).expect("cap Wizard Position P Gain 8000 to factory 400");
    assert_eq!(driver.applied_position_p_gain(), 400);
    driver.read_sensor(0.0).expect("sensor");
    let before = driver.last_present_position();
    driver
        .write_action(&[0.2], &ActionParams::empty())
        .expect("nudge after capping high P");
    driver.read_sensor(0.1).expect("sensor");
    let after = driver.last_present_position();
    assert_ne!(after, before, "factory P must still track the 32-tick goal");
    let traveled = (i64::from(after) - i64::from(before)).abs();
    assert!(
        traveled <= 48,
        "Wizard P=8000 must not throw present past the 48-tick cage: {before}->{after}"
    );
    driver.close();
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_writes_configured_pwm_cap_never_factory_885() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[("REALITYOS_METAL_PTY_ZERO_PWM", "1")]);
    let root = metal_test_root("pty-zero-pwm");
    let cfg = MetalConfig::example(&tty);
    let mut driver = Xl330Driver::open(cfg, &root).expect("write configured PWM cap, not factory");
    assert_eq!(
        driver.applied_pwm_limit(),
        realityos_metal::protocol::CONSERVATIVE_PWM_LIMIT
    );
    assert_ne!(driver.applied_pwm_limit(), 885);
    let evidence = std::fs::read_to_string(root.join("bus").join("pwm_limit.json")).unwrap();
    assert!(evidence.contains("\"requested\":200"), "{evidence}");
    assert!(evidence.contains("\"measured\":200"), "{evidence}");
    driver.read_sensor(0.0).expect("sensor");
    let before = driver.last_present_position();
    driver
        .write_action(&[0.2], &ActionParams::empty())
        .expect("nudge after writing the configured PWM cap");
    driver.read_sensor(0.1).expect("sensor");
    let after = driver.last_present_position();
    assert_ne!(
        after, before,
        "configured PWM cap must be applied; do not silently restore factory 885"
    );
    assert_eq!(recorded_writes(root.join("bus")), 1);
    driver.close();
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_raises_wizard_zero_velocity_p_so_nudge_can_track() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[("REALITYOS_METAL_PTY_ZERO_VEL_P", "1")]);
    let root = metal_test_root("pty-zero-vel-p");
    let cfg = MetalConfig::example(&tty);
    let mut driver = Xl330Driver::open(cfg, &root).expect("raise Velocity P Gain 0 to factory 100");
    assert_eq!(driver.applied_velocity_p_gain(), 100);
    driver.read_sensor(0.0).expect("sensor");
    let before = driver.last_present_position();
    driver
        .write_action(&[0.2], &ActionParams::empty())
        .expect("nudge after restoring Velocity P");
    driver.read_sensor(0.1).expect("sensor");
    let after = driver.last_present_position();
    assert_ne!(
        after, before,
        "Wizard Velocity P=0 must not leave present stuck after setup"
    );
    assert_eq!(recorded_writes(root.join("bus")), 1);
    driver.close();
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_refuses_when_velocity_p_write_does_not_stick() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[
        ("REALITYOS_METAL_PTY_ZERO_VEL_P", "1"),
        ("REALITYOS_METAL_PTY_DROP_VELOCITY_P", "1"),
    ]);
    let root = metal_test_root("pty-drop-velocity-p");
    let cfg = MetalConfig::example(&tty);
    let err = match Xl330Driver::open(cfg, &root) {
        Ok(_) => panic!("ACK'd-but-dropped Velocity P write must not look like factory 100"),
        Err(e) => e,
    };
    assert!(
        err.to_string().contains("dxl_velocity_p_unverified"),
        "got {err}"
    );
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_raises_wizard_zero_velocity_i_so_profile_can_settle() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[("REALITYOS_METAL_PTY_ZERO_VEL_I", "1")]);
    let root = metal_test_root("pty-zero-vel-i");
    let cfg = MetalConfig::example(&tty);
    let mut driver =
        Xl330Driver::open(cfg, &root).expect("raise Velocity I Gain 0 to factory 1600");
    assert_eq!(driver.applied_velocity_i_gain(), 1600);
    driver
        .write_action(&[0.0], &ActionParams::empty())
        .expect("hold after restoring Velocity I");
    assert_eq!(recorded_writes(root.join("bus")), 1);
    driver.close();
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_refuses_when_velocity_i_write_does_not_stick() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[
        ("REALITYOS_METAL_PTY_ZERO_VEL_I", "1"),
        ("REALITYOS_METAL_PTY_DROP_VELOCITY_I", "1"),
    ]);
    let root = metal_test_root("pty-drop-velocity-i");
    let cfg = MetalConfig::example(&tty);
    let err = match Xl330Driver::open(cfg, &root) {
        Ok(_) => panic!("ACK'd-but-dropped Velocity I write must not look like factory 1600"),
        Err(e) => e,
    };
    assert!(
        err.to_string().contains("dxl_velocity_i_unverified"),
        "got {err}"
    );
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_refuses_when_torque_drops_after_enable() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[("REALITYOS_METAL_PTY_TORQUE_DROP", "1")]);
    let root = metal_test_root("pty-torque-drop");
    let cfg = MetalConfig::example(&tty);
    let err = match Xl330Driver::open(cfg, &root) {
        Ok(_) => panic!("overload drop after torque-on must not look like a live hold"),
        Err(e) => e,
    };
    assert!(
        err.to_string().contains("dxl_torque_dropped_after_enable"),
        "got {err}"
    );
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_clears_wizard_homing_offset_so_present_is_in_window() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[("REALITYOS_METAL_PTY_HOMING", "1")]);
    let root = metal_test_root("pty-homing");
    let cfg = MetalConfig::example(&tty);
    let mut driver =
        Xl330Driver::open(cfg, &root).expect("clear Homing Offset 10000 with torque off");
    assert_eq!(driver.applied_homing_offset(), 0);
    driver.read_sensor(0.0).expect("sensor");
    assert_eq!(
        driver.last_present_position(),
        2048,
        "clearing Homing Offset must not yank the horn; present returns to encoder space"
    );
    driver
        .write_action(&[0.0], &ActionParams::empty())
        .expect("hold after clearing homing offset");
    assert_eq!(recorded_writes(root.join("bus")), 1);
    driver.close();
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_clears_in_window_homing_offset_so_torque_on_does_not_yank() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[("REALITYOS_METAL_PTY_HOMING_IN_WINDOW", "1")]);
    let root = metal_test_root("pty-homing-in-window");
    let cfg = MetalConfig::example(&tty);
    let mut driver = Xl330Driver::open(cfg, &root)
        .expect("in-window Homing Offset 1024 must be cleared before torque-on");
    assert_eq!(driver.applied_homing_offset(), 0);
    driver.read_sensor(0.0).expect("sensor");
    assert_eq!(
        driver.last_present_position(),
        1024,
        "clearing Homing Offset with torque off shifts present in encoder space, not by moving the horn"
    );
    driver
        .write_action(&[0.0], &ActionParams::empty())
        .expect("hold after clearing in-window homing offset");
    assert_eq!(recorded_writes(root.join("bus")), 1);
    driver.close();
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_refuses_when_homing_offset_write_does_not_stick() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[
        ("REALITYOS_METAL_PTY_HOMING_IN_WINDOW", "1"),
        ("REALITYOS_METAL_PTY_DROP_HOMING_OFFSET", "1"),
    ]);
    let root = metal_test_root("pty-drop-homing");
    let cfg = MetalConfig::example(&tty);
    let err = match Xl330Driver::open(cfg, &root) {
        Ok(_) => panic!("ACK'd-but-dropped Homing Offset write must not reach torque-on"),
        Err(e) => e,
    };
    assert!(
        err.to_string().contains("dxl_homing_offset_unverified"),
        "got {err}"
    );
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_torque_off_after_hw_error_reboot_so_eeprom_can_write() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[("REALITYOS_METAL_PTY_HW_ERROR", "1")]);
    let root = metal_test_root("pty-hw-error");
    let cfg = MetalConfig::example(&tty);
    let mut driver = Xl330Driver::open(cfg, &root)
        .expect("reboot then torque-off so PWM-mode EEPROM can become position");
    driver.read_sensor(0.0).expect("sensor");
    assert_eq!(
        driver.last_present_position(),
        2048,
        "stale goal 0 must not yank after Startup Configuration torque-on"
    );
    driver
        .write_action(&[0.0], &ActionParams::empty())
        .expect("hold after hw-error reboot");
    assert_eq!(recorded_writes(root.join("bus")), 1);
    driver.close();
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_refuses_torque_when_present_outside_wizard_limits() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[("REALITYOS_METAL_PTY_PRESENT_OUTSIDE", "1")]);
    let root = metal_test_root("pty-present-outside");
    let cfg = MetalConfig::example(&tty);
    let err = match Xl330Driver::open(cfg, &root) {
        Ok(_) => panic!("present 100 in window 2000..2100 must not yank to the edge"),
        Err(e) => e,
    };
    assert!(
        err.to_string()
            .contains("dxl_present_outside_wizard_limits"),
        "got {err}"
    );
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
fn xl330_pty_refuses_when_goal_match_present_write_does_not_stick() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[("REALITYOS_METAL_PTY_DROP_GOAL_POSITION", "1")]);
    let root = metal_test_root("pty-drop-goal");
    let cfg = MetalConfig::example(&tty);
    let err = match Xl330Driver::open(cfg, &root) {
        Ok(_) => panic!("ACK'd-but-dropped goal=present must not reach torque-on"),
        Err(e) => e,
    };
    assert!(err.to_string().contains("dxl_goal_unverified"), "got {err}");
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_clears_bus_watchdog_error_so_goal_writes_are_live() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[
        ("REALITYOS_METAL_PTY_BUS_WATCHDOG", "1"),
        ("REALITYOS_METAL_PTY_UNREAD_WATCHDOG", "1"),
    ]);
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
fn xl330_pty_refuses_when_drive_mode_write_does_not_stick() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[
        ("REALITYOS_METAL_PTY_TIME_BASED", "1"),
        ("REALITYOS_METAL_PTY_DROP_DRIVE_MODE", "1"),
    ]);
    let root = metal_test_root("pty-drop-drive-mode");
    let cfg = MetalConfig::example(&tty);
    let err = match Xl330Driver::open(cfg, &root) {
        Ok(_) => panic!("ACK'd-but-dropped drive-mode write must not look like velocity-based"),
        Err(e) => e,
    };
    assert!(
        err.to_string().contains("dxl_drive_mode_unverified"),
        "got {err}"
    );
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
fn xl330_pty_refuses_torque_when_vin_cannot_be_read() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[("REALITYOS_METAL_PTY_NO_VIN", "1")]);
    let root = metal_test_root("pty-no-vin");
    let cfg = MetalConfig::example(&tty);
    let err = match Xl330Driver::open(cfg, &root) {
        Ok(_) => panic!("VIN 0 must not skip the supply check and torque-on"),
        Err(e) => e,
    };
    assert!(
        err.to_string().contains("dxl_vin_unreadable_before_torque"),
        "got {err}"
    );
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_refuses_torque_when_present_temperature_at_limit() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[("REALITYOS_METAL_PTY_HOT", "1")]);
    let root = metal_test_root("pty-hot");
    let cfg = MetalConfig::example(&tty);
    let err = match Xl330Driver::open(cfg, &root) {
        Ok(_) => panic!("present temperature at the EEPROM limit must not torque-on"),
        Err(e) => e,
    };
    assert!(
        err.to_string()
            .contains("dxl_present_temperature_at_or_above_limit"),
        "got {err}"
    );
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_refuses_torque_when_temperature_limit_is_zero() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[("REALITYOS_METAL_PTY_ZERO_TEMP_LIMIT", "1")]);
    let root = metal_test_root("pty-zero-tlim");
    let cfg = MetalConfig::example(&tty);
    let err = match Xl330Driver::open(cfg, &root) {
        Ok(_) => panic!("Wizard temperature limit 0 must not torque-on"),
        Err(e) => e,
    };
    assert!(
        err.to_string().contains("dxl_temperature_limit_zero"),
        "got {err}"
    );
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_live_vin_zero_refuses_and_does_not_write() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[("REALITYOS_METAL_PTY_LIVE_LOW_VIN", "1")]);
    let root = metal_test_root("pty-live-vin0");
    let cfg = MetalConfig::example(&tty);
    let mut driver = Xl330Driver::open(cfg, &root).expect("setup VIN 5.0 V is in range");
    assert_eq!(driver.applied_voltage_limits(), (35, 70));
    driver
        .read_sensor(0.0)
        .expect("first live motion sample still has setup VIN");
    let err = driver
        .read_sensor(0.0)
        .expect_err("VIN 0 on a later motion block is not a healthy sample");
    assert!(err.to_string().contains("dxl_vin_unreadable"), "got {err}");
    let tx_before = recorded_serial_tx(root.join("bus"));
    let writes_before = recorded_writes(root.join("bus"));
    driver
        .write_action(&[0.0], &ActionParams::empty())
        .expect_err("certified write must not follow a live VIN fault");
    assert_eq!(recorded_serial_tx(root.join("bus")), tx_before);
    assert_eq!(recorded_writes(root.join("bus")), writes_before);
    driver.close();
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_live_vin_below_wizard_min_refuses_and_does_not_write() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[("REALITYOS_METAL_PTY_LIVE_BROWN_VIN", "1")]);
    let root = metal_test_root("pty-live-brown");
    let cfg = MetalConfig::example(&tty);
    let mut driver = Xl330Driver::open(cfg, &root).expect("setup VIN 5.0 V is in range");
    driver
        .read_sensor(0.0)
        .expect("first live motion sample still has setup VIN");
    let err = driver
        .read_sensor(0.0)
        .expect_err("VIN 2.0 V is below Wizard min 3.5 V");
    assert!(
        err.to_string().contains("dxl_vin_outside_wizard_limits"),
        "got {err}"
    );
    let tx_before = recorded_serial_tx(root.join("bus"));
    driver
        .write_action(&[0.2], &ActionParams::empty())
        .expect_err("cutoff-range VIN must not reach a certified goal write");
    assert_eq!(recorded_serial_tx(root.join("bus")), tx_before);
    driver.close();
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_refuses_torque_when_voltage_limits_cannot_be_read() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[("REALITYOS_METAL_PTY_NO_VLIMIT", "1")]);
    let root = metal_test_root("pty-no-vlimit");
    let cfg = MetalConfig::example(&tty);
    let err = match Xl330Driver::open(cfg, &root) {
        Ok(_) => panic!("unreadable voltage EEPROM must not invent 35/70 and torque-on"),
        Err(e) => e,
    };
    assert!(
        err.to_string()
            .contains("dxl_voltage_limits_unreadable_before_torque"),
        "got {err}"
    );
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_refuses_when_hw_error_unreadable_after_torque() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[("REALITYOS_METAL_PTY_NO_HWERR", "1")]);
    let root = metal_test_root("pty-no-hwerr");
    let cfg = MetalConfig::example(&tty);
    let err = match Xl330Driver::open(cfg, &root) {
        Ok(_) => panic!("unreadable Hardware Error Status must not leave torque on"),
        Err(e) => e,
    };
    assert!(
        err.to_string()
            .contains("dxl_hw_error_unreadable_after_torque"),
        "got {err}"
    );
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_refuses_when_hw_error_latches_after_torque_on() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[("REALITYOS_METAL_PTY_HW_AFTER_TORQUE", "1")]);
    let root = metal_test_root("pty-hw-after-torque");
    let cfg = MetalConfig::example(&tty);
    let err = match Xl330Driver::open(cfg, &root) {
        Ok(_) => panic!("latched Hardware Error after torque-on must not look like a live hold"),
        Err(e) => e,
    };
    assert!(
        err.to_string()
            .contains("dxl_hardware_error_after_torque_on"),
        "got {err}"
    );
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_refuses_inverted_wizard_position_limits() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[("REALITYOS_METAL_PTY_INV_LIMITS", "1")]);
    let root = metal_test_root("pty-inv-limits");
    let cfg = MetalConfig::example(&tty);
    let err = match Xl330Driver::open(cfg, &root) {
        Ok(_) => panic!("inverted min/max must not fall back to invented 0..=4095"),
        Err(e) => e,
    };
    assert!(
        err.to_string().contains("dxl_position_limits_invalid"),
        "got {err}"
    );
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_discover_collapses_wizard_secondary_id_to_one_servo() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[("REALITYOS_METAL_PTY_SECONDARY", "1")]);
    let root = metal_test_root("pty-secondary");
    let cfg = MetalConfig::example(&tty);
    let (mut driver, bound) = Xl330Driver::open_discovering(cfg, &root)
        .expect("one servo with Secondary ID 7 must not look like two actuators");
    assert_eq!(bound.servo_id, 1);
    assert_eq!(driver.measured().actuator_id, "xl330:1");
    driver.close();
    assert_eq!(recorded_writes(root.join("bus")), 0);
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_discover_refuses_two_servo_ids_on_the_bus() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[("REALITYOS_METAL_PTY_MULTI", "1")]);
    let root = metal_test_root("pty-multi");
    let cfg = MetalConfig::example(&tty);
    let err = match Xl330Driver::open_discovering(cfg, &root) {
        Ok(_) => panic!("two status IDs must not pick a servo at random"),
        Err(e) => e,
    };
    assert!(
        err.to_string().contains("dxl_multiple_servos_on_bus"),
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
fn xl330_pty_discover_finds_wizard_id_zero() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[("REALITYOS_METAL_PTY_ID", "0")]);
    let root = metal_test_root("pty-id0");
    let cfg = MetalConfig::example(&tty);
    assert_eq!(cfg.servo_id, 1);
    let (mut driver, bound) =
        Xl330Driver::open_discovering(cfg, &root).expect("broadcast PING must find Wizard ID 0");
    assert_eq!(bound.servo_id, 0);
    assert_eq!(driver.measured().actuator_id, "xl330:0");
    assert!(
        !driver.torque_is_enabled(),
        "probe/discover must not torque-on"
    );
    driver.close();
    assert_eq!(recorded_writes(root.join("bus")), 0);
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_quiesces_startup_torque_during_open_settle() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[
        ("REALITYOS_METAL_PTY_STARTUP_TORQUE", "1"),
        ("REALITYOS_METAL_PTY_STARTUP_YANK", "1"),
        ("REALITYOS_METAL_PTY_UNREAD_STARTUP", "1"),
    ]);
    let root = metal_test_root("pty-startup-yank");
    let cfg = MetalConfig::example(&tty);
    let mut driver = Xl330Driver::open(cfg, &root)
        .expect("Startup Configuration torque-on must not slam present to goal 0");
    assert_eq!(
        driver.applied_startup_configuration(),
        0,
        "serve must clear EEPROM torque-on-boot so the next DTR-RESET does not yank"
    );
    assert_eq!(
        driver.startup_present(),
        2048,
        "stale goal 0 must not yank during the open-settle / identify window"
    );
    driver.read_sensor(0.0).expect("sensor");
    assert_eq!(driver.last_present_position(), 2048);
    assert_eq!(
        recorded_writes(root.join("bus")),
        0,
        "settle / identify torque-off is not command egress"
    );
    driver.close();
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_discover_quiesces_startup_torque_without_command_egress() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[("REALITYOS_METAL_PTY_STARTUP_TORQUE", "1")]);
    let root = metal_test_root("pty-startup-torque");
    let cfg = MetalConfig::example(&tty);
    let (mut driver, _) =
        Xl330Driver::open_discovering(cfg, &root).expect("probe must open with torque already on");
    assert!(
        !driver.torque_is_enabled(),
        "probe must not claim it enabled torque"
    );
    driver.close();
    assert_eq!(
        recorded_writes(root.join("bus")),
        0,
        "probe torque-off is not command egress"
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
    assert_eq!(
        wd_added, 0,
        "successful watchdog pets must not journal, got {wd_added}"
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
            r#"{"firmware_id":"xl330-m288:1200:255"}"#,
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
        r#"{"firmware_id":"xl330-m288:1200:255"}"#,
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
fn xl330_pty_eeprom_identity_change_refuses_write() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[("REALITYOS_METAL_PTY_FLIP_IDENTITY", "1")]);
    let root = metal_test_root("pty-eeprom-flip");
    let mut cfg = MetalConfig::example(&tty);
    cfg.campaign_hooks = false;
    {
        let driver = Xl330Driver::open(cfg.clone(), &root).expect("identify");
        let measured = driver.measured();
        assert_eq!(measured.firmware_id, "xl330-m288:1200:46");
        cfg.expected_serial = measured.serial;
        cfg.expected_firmware = measured.firmware_id;
    }
    cfg.save(root.join(CONFIG_FILE)).unwrap();
    let mut auth = MetalAuthority::start(&root, true).expect("start_online");
    let hold = auth.handle(MetalRequest::propose("pty-eep-hold", "hold"));
    assert!(hold.ok, "first hold before EEPROM flip: {hold:?}");
    let writes = auth.physical_writes();
    let flipped = auth.handle(MetalRequest::propose("pty-eep-flip", "hold"));
    assert!(!flipped.ok, "swapped model/fw must refuse: {flipped:?}");
    assert!(
        flipped
            .violations
            .iter()
            .any(|v| v.contains("hardware_firmware_mismatch")),
        "EEPROM re-read must reach verify_live_hardware: {flipped:?}"
    );
    assert!(
        !flipped
            .violations
            .iter()
            .any(|v| v.contains("software_watchdog_miss")),
        "identity change must not be a watchdog miss: {flipped:?}"
    );
    assert_eq!(auth.physical_writes(), writes);
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_identity_read_failure_keeps_motion() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[("REALITYOS_METAL_PTY_NO_IDENTITY", "1")]);
    let root = metal_test_root("pty-id-crc");
    let mut cfg = MetalConfig::example(&tty);
    cfg.campaign_hooks = false;
    {
        let mut driver = Xl330Driver::open(cfg.clone(), &root).expect("identify");
        let measured = driver.measured();
        assert_eq!(measured.firmware_id, "xl330-m288:1200:46");
        cfg.expected_serial = measured.serial;
        cfg.expected_firmware = measured.firmware_id;
        driver
            .read_sensor(0.0)
            .expect("identity CRC after a good motion sample must not drop the sensor");
        assert!(
            driver.probe_identity().connected,
            "identity CRC must not latch bus_lost after a good motion sample"
        );
        assert_eq!(
            driver.measured().firmware_id,
            "xl330-m288:1200:46",
            "CRC miss must keep the identify-time firmware"
        );
    }
    cfg.save(root.join(CONFIG_FILE)).unwrap();
    let mut auth = MetalAuthority::start(&root, true).expect("start_online");
    let hold = auth.handle(MetalRequest::propose("pty-id-crc-hold", "hold"));
    assert!(
        hold.ok,
        "hold after identity CRC must still write: {hold:?}"
    );
    assert!(
        !hold
            .violations
            .iter()
            .any(|v| v.contains("software_watchdog_miss") || v.contains("bus_lost")),
        "identity CRC must not be bus-lost or watchdog: {hold:?}"
    );
    assert_eq!(auth.physical_writes(), 1);
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

#[test]
fn xl330_pty_vanished_udev_path_is_not_disconnect() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder();
    let root = metal_test_root("pty-udev-vanish");
    let cfg = MetalConfig::example(&tty);
    let mut driver = Xl330Driver::open(cfg, &root).expect("open");
    assert!(driver.is_connected(), "open must leave a live session");
    let before = driver.measured();
    assert!(
        !before.serial.is_empty(),
        "open must latch a measured serial: {before:?}"
    );
    driver.simulate_udev_path_vanished();
    assert!(
        driver.is_connected(),
        "udev rename / dangling by-id must not look like unplug while the exclusive fd is live"
    );
    let id = driver.probe_identity();
    assert!(
        id.connected,
        "probe_identity must not report disconnect after a vanished path: {id:?}"
    );
    driver
        .read_sensor(0.0)
        .expect("first hold-class sensor must work after a vanished udev path");
    let after = driver.measured();
    assert_eq!(
        after.serial, before.serial,
        "sensor refresh must not blank the open-time adapter serial when the udev name vanishes"
    );
    assert!(
        after.connected,
        "latched identity must stay connected after sensor refresh: {after:?}"
    );
    let refreshed = driver.probe_identity();
    assert_eq!(refreshed.serial, before.serial);
    assert!(
        refreshed.connected,
        "write-time probe_identity must keep the bound serial after a vanished udev path: {refreshed:?}"
    );
    driver
        .write_action(&[0.0], &ActionParams::empty())
        .expect("first hold-class write must use the latched serial, not a vanished udev name");
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_low_pwm_cap_is_written_and_read_back() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder();
    let root = metal_test_root("pty-low-pwm");
    let mut cfg = MetalConfig::example(&tty);
    cfg.max_pwm_limit_raw = 50;
    let mut driver = Xl330Driver::open(cfg, &root).expect("apply low PWM cap");
    assert_eq!(driver.pwm_limit_requested(), 50);
    assert_eq!(driver.applied_pwm_limit(), 50);
    assert_ne!(driver.applied_pwm_limit(), 885);
    let raw = std::fs::read_to_string(root.join("bus").join("pwm_limit.json")).unwrap();
    assert!(raw.contains("\"requested\":50"), "{raw}");
    assert!(raw.contains("\"measured\":50"), "{raw}");
    driver.close();
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_repeated_nudges_cannot_escape_startup_cage() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder();
    let root = metal_test_root("pty-cage");
    let cfg = MetalConfig::example(&tty);
    let mut driver = Xl330Driver::open(cfg, &root).expect("open");
    driver.read_sensor(0.0).expect("sensor");
    let (emin, emax) = driver.experiment_cage();
    let startup = driver.startup_present();
    assert!(emax - emin <= 96, "first experiment cage must stay small");
    let mut refused = 0u32;
    for _ in 0..100 {
        driver.read_sensor(0.0).expect("sensor");
        let present = driver.last_present_position();
        assert!(
            present >= emin && present <= emax,
            "present {present} escaped cage {emin}..{emax} startup={startup}"
        );
        let tx_before = recorded_serial_tx(root.join("bus"));
        match driver.write_action(&[0.2], &ActionParams::empty()) {
            Ok(_) => {
                driver.read_sensor(0.1).expect("sensor after nudge");
                let after = driver.last_present_position();
                assert!(
                    after >= emin && after <= emax,
                    "nudge present {after} escaped cage {emin}..{emax}"
                );
            }
            Err(e) => {
                assert!(e.to_string().contains("experiment_cage_violation"), "{e}");
                assert_eq!(recorded_serial_tx(root.join("bus")), tx_before);
                refused += 1;
            }
        }
    }
    assert!(
        refused > 0,
        "100 accumulated +32 nudges must hit the absolute cage"
    );
    driver.close();
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_after_serial_tx_before_status_restart_does_not_retransmit() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder();
    let root = metal_test_root("pty-posttx-crash");
    bind_pty_cfg(&root, &tty);
    let bin = env!("CARGO_BIN_EXE_realityos-metal-smoke");
    let mut crash = std::process::Command::new(bin)
        .args(["--root", &root.to_string_lossy(), "--first-online", "serve"])
        .env("REALITYOS_METAL_ALLOW_PTY", "1")
        .env("REALITYOS_HIL_CRASH", "after_serial_tx_before_status")
        .stdout(Stdio::null())
        .stderr(Stdio::inherit())
        .spawn()
        .expect("crash serve");
    assert!(
        realityos_metal::ipc::wait_for_ipc(&root, 15_000),
        "crash serve did not bind: {}",
        std::fs::read_to_string(root.join("serve.err")).unwrap_or_default()
    );
    let _ = realityos_metal::ipc::call(&root, &MetalRequest::propose("metal-crash-posttx", "hold"));
    let status = crash.wait().expect("wait crash serve");
    assert_eq!(
        status.code(),
        Some(77),
        "expected hil_crash exit 77, got {status:?}"
    );
    let tx_after_first = recorded_serial_tx(root.join("bus"));
    assert!(
        tx_after_first >= 1,
        "first process may have transmitted once, got {tx_after_first}"
    );
    assert!(
        std::path::Path::new(&tty).exists(),
        "PTY stand-in must survive host death after serial TX: {tty}"
    );
    let _ = std::fs::remove_file(root.join("ipc.sock"));
    let mut restart = std::process::Command::new(bin)
        .args(["--root", &root.to_string_lossy(), "--restart", "serve"])
        .env("REALITYOS_METAL_ALLOW_PTY", "1")
        .env("REALITYOS_METAL_DEVICE", &tty)
        .env_remove("REALITYOS_HIL_CRASH")
        .stdout(Stdio::null())
        .stderr(Stdio::inherit())
        .spawn()
        .expect("restart serve");
    assert!(
        realityos_metal::ipc::wait_for_ipc(&root, 15_000),
        "restart serve did not bind: {} tty_exists={}",
        std::fs::read_to_string(root.join("serve.err")).unwrap_or_default(),
        std::path::Path::new(&tty).exists()
    );
    assert!(
        restart.try_wait().ok().flatten().is_none(),
        "restart serve exited early: {}",
        std::fs::read_to_string(root.join("serve.err")).unwrap_or_default()
    );
    let before = recorded_serial_tx(root.join("bus"));
    let replay =
        realityos_metal::ipc::call(&root, &MetalRequest::propose("metal-crash-posttx", "hold"))
            .expect("replay after restart");
    assert!(
        !replay.ok,
        "restart must not re-authorize the same command: {replay:?}"
    );
    assert_eq!(
        recorded_serial_tx(root.join("bus")),
        before,
        "restart must not retransmit a possibly-executed command"
    );
    let _ = std::fs::write(root.join("stop_serve"), b"1");
    let _ = restart.kill();
    let _ = restart.wait();
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_hw_error_refresh_timeout_does_not_latch_disconnect() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[
        ("REALITYOS_METAL_PTY_ALERT", "1"),
        ("REALITYOS_METAL_PTY_HWERR_REFRESH_FAIL", "1"),
    ]);
    let root = metal_test_root("pty-hwerr-refresh");
    let cfg = MetalConfig::example(&tty);
    let mut driver = Xl330Driver::open(cfg, &root).expect("open with ALERT + refresh fail");
    driver
        .write_action(&[0.0], &ActionParams::empty())
        .expect("hold latches STATUS_ALERT");
    driver
        .read_sensor(0.0)
        .expect("good motion sample must survive a failed hw_error refresh");
    assert!(
        driver.is_connected(),
        "diagnostic register 70 timeout must not clear connected"
    );
    driver
        .write_action(&[0.0], &ActionParams::empty())
        .expect("next certified hold must still reach the bus");
    driver.close();
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_reenable_torque_rematches_goal_to_present() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder_env(&[("REALITYOS_METAL_PTY_DRIFT_ON_TORQUE_OFF", "1")]);
    let root = metal_test_root("pty-reenable-match");
    let cfg = MetalConfig::example(&tty);
    let mut driver = Xl330Driver::open(cfg, &root).expect("open");
    driver
        .write_action(&[0.0], &ActionParams::empty())
        .expect("hold");
    driver.read_sensor(0.0).expect("sensor after hold");
    let parked = driver.last_present_position();
    driver.engage_hw_estop("test");
    driver
        .clear_hw_estop(true)
        .expect("clear estop without re-energizing");
    driver
        .write_action(&[0.0], &ActionParams::empty())
        .expect("hold after rematch");
    let after_reenable = driver.last_present_position();
    assert_eq!(
        after_reenable,
        parked + 20,
        "re-enable must match goal to drifted present, not yank back to the stale goal"
    );
    driver.close();
    let _ = std::fs::remove_dir_all(&root);
}

#[test]
fn xl330_pty_crash_restart_restores_wizard_window_before_new_cage() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder();
    let root = metal_test_root("pty-cage-restore");
    let cfg = MetalConfig::example(&tty);
    let mut first = Xl330Driver::open(cfg.clone(), &root).expect("first open");
    let (first_min, first_max) = first.experiment_cage();
    first
        .write_action(&[0.2], &ActionParams::empty())
        .expect("nudge toward the cage edge");
    first.read_sensor(0.0).expect("sensor after nudge");
    first.abandon_without_eeprom_restore_for_test();
    drop(first);
    let mut second = Xl330Driver::open(cfg, &root).expect("reopen after crash-like abandon");
    let (_min, second_max) = second.experiment_cage();
    assert!(
        second_max > first_max,
        "leftover EEPROM cage must not ratchet the next session window: first={first_min}..{first_max} second_max={second_max}"
    );
    second
        .write_action(&[0.2], &ActionParams::empty())
        .expect("post-restart nudge needs the restored Wizard window");
    second.close();
    let _ = std::fs::remove_dir_all(&root);
}
