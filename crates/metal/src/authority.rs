//! Dedicated metal composition root. Not the HIL Authority object.

use std::path::{Path, PathBuf};

use realityos_core::{DecideRequest, Intent, IssuedCommand, PolicyProposal, RealityOs, WorldView};
use realityos_governor::OnlineLocked;
use realityos_plant::{ActionParams, HardwareBackedPlant};
use realityos_session::{RuntimeMode, RuntimeSession, StartArgs};

use crate::config::{
    MetalConfig, CONFIG_FILE, FRESHNESS_FILE, GOAL_FILE, JOURNAL, PRESENT_FILE, SIGNING_KEY_FILE,
};
use crate::egress::{recorded_acks, recorded_egress_attempts, recorded_serial_tx, recorded_writes};
use crate::ipc::{MetalRequest, MetalResponse};
use crate::xl330::Xl330Driver;

pub struct MetalAuthority {
    ros: RealityOs,
    session: RuntimeSession<HardwareBackedPlant<Xl330Driver>, OnlineLocked>,
    root: PathBuf,
    cfg: MetalConfig,
    next_sequence: i64,
    /// Live I/O loss is detected on acquire, before write-time verify.
    /// Latch here so recover cannot resurrect the instance (kernel
    /// `hardware_session_dead` is only set from verify_live_hardware).
    bus_lost: bool,
}

impl MetalAuthority {
    pub fn start(root: impl AsRef<Path>, first_online: bool) -> anyhow::Result<Self> {
        let root = root.as_ref().to_path_buf();
        std::fs::create_dir_all(&root)?;
        let cfg_path = root.join(CONFIG_FILE);
        let mut cfg = MetalConfig::load(&cfg_path)?;
        if !cfg.expected_ready() {
            anyhow::bail!("metal_expected_identity_missing:run_probe_then_bind_measured");
        }
        let json_device = cfg.device.clone();
        // Device only: baud/id in metal.json are the pair probe measured.
        cfg.apply_device_env();
        let live = crate::identity::pick_live_device(
            cfg.device.clone(),
            json_device.clone(),
            &cfg.expected_serial,
            cfg.servo_id,
        );
        if live != json_device {
            cfg.device = live;
            cfg.save(&cfg_path)?;
        } else {
            cfg.device = live;
        }
        if !cfg.device.exists() {
            anyhow::bail!("metal_device_missing:{}", cfg.device.display());
        }
        // Measure the USB-adapter serial before open(). `open` applies
        // bench limits and torque-on; a recycled ttyUSB0 that is a
        // different UART would command the wrong actuator first.
        if !crate::identity::adapter_serial_matches(&cfg.device, &cfg.expected_serial, cfg.servo_id)
        {
            anyhow::bail!(
                "metal_adapter_serial_mismatch:expected={} actual={} device={}",
                cfg.expected_serial,
                crate::identity::adapter_serial_for_tty(&cfg.device, cfg.servo_id),
                cfg.device.display()
            );
        }
        let key = load_or_create_key(&root)?;
        let driver = Xl330Driver::open(cfg.clone(), &root)?;
        // Serve may have rewritten leftover Wizard 9 600 to factory 57 600
        // so live I/O fits the 40 ms deadline. start_online hashes baud;
        // crash-replay must open at the same rate the journal recorded.
        if driver.applied_baud() != cfg.baud {
            cfg.baud = driver.applied_baud();
            cfg.save(&cfg_path)?;
        }
        let measured = driver.measured();
        if measured.serial != cfg.expected_serial {
            anyhow::bail!(
                "metal_serial_mismatch:expected={} actual={}",
                cfg.expected_serial,
                measured.serial
            );
        }
        if measured.firmware_id != cfg.expected_firmware {
            anyhow::bail!(
                "metal_firmware_mismatch:expected={} actual={}",
                cfg.expected_firmware,
                measured.firmware_id
            );
        }
        let plant = HardwareBackedPlant::new(driver, "xl330", 1, cfg.tau_max);
        let args = StartArgs {
            mode: RuntimeMode::Online,
            release_hash: cfg.release_hash.clone(),
            release_class: "MFG_CANDIDATE".into(),
            design_content_hash: cfg.design_content_hash(),
            serial_or_as_built: cfg.expected_serial.clone(),
            firmware_id: cfg.expected_firmware.clone(),
            calibration_id: cfg.calibration_id.clone(),
            require_verified_release: None,
            require_command_signature: None,
            require_driver_envelope: None,
            max_action_abs: cfg.tau_max,
            journal_path: Some(root.join(JOURNAL)),
            signing_key: Some(key),
            first_online,
            actuator_ids: vec![cfg.actuator_id()],
        };
        let session =
            RuntimeSession::<HardwareBackedPlant<Xl330Driver>, OnlineLocked>::start_online(
                args, plant,
            )
            .map_err(|e| anyhow::anyhow!(e.0))?;
        if !cfg.freshness_threshold_s.is_finite() || cfg.freshness_threshold_s <= 0.0 {
            anyhow::bail!(
                "metal_freshness_threshold_invalid:{}",
                cfg.freshness_threshold_s
            );
        }
        // ONLINE sensor_stale_s is locked after start_online. Do not mutate
        // it. Fail closed if metal.json records a different window.
        let enforced = session.governor.config().sensor_stale_s;
        if (cfg.freshness_threshold_s - enforced).abs() > 1e-9 {
            anyhow::bail!(
                "metal_freshness_threshold_mismatch_online_locked:metal={} governor={}",
                cfg.freshness_threshold_s,
                enforced
            );
        }
        let next_sequence = session.governor.ledger().last_sequence().max(0);
        Ok(Self {
            ros: RealityOs::new(),
            session,
            root,
            cfg,
            next_sequence,
            bus_lost: false,
        })
    }

    pub fn physical_writes(&self) -> u64 {
        recorded_writes(self.root.join(crate::config::BUS_DIR))
    }

    pub fn command_egress_attempts(&self) -> u64 {
        recorded_egress_attempts(self.root.join(crate::config::BUS_DIR))
    }

    pub fn serial_tx_completed(&self) -> u64 {
        recorded_serial_tx(self.root.join(crate::config::BUS_DIR))
    }

    pub fn device_acks(&self) -> u64 {
        recorded_acks(self.root.join(crate::config::BUS_DIR))
    }

    pub fn handle(&mut self, req: MetalRequest) -> MetalResponse {
        // Watchdog only: a heartbeat emit is another journal+seal fsync pair.
        // Heartbeat stale is 2 s; idle serve already heartbeats ~every 800 ms.
        if let Err(v) = self.pet_watchdog() {
            return self.refuse("egress", "refuse", v);
        }
        if req.injects_sensor_evidence() {
            return self.refuse(
                "protocol",
                "refuse",
                vec!["autonomy_cannot_refresh_evidence".into()],
            );
        }
        if req.injects_caller_time_or_hil() {
            return self.refuse(
                "protocol",
                "refuse",
                vec!["production_ipc_refuses_caller_time_or_hil_fault".into()],
            );
        }
        if !req.production_ops_only() {
            return self.refuse(
                "protocol",
                "refuse",
                vec![format!("production_ipc_refuses:{}", req.op)],
            );
        }
        match req.op.as_str() {
            "sensor" => self.ingest_sensor(),
            "heartbeat" => {
                let _ = self.session.governor.heartbeat_now();
                self.ok("observe")
            }
            "status" => self.ok("status"),
            "recover" => self.recover(),
            "propose" => self.propose(req),
            other => self.refuse("protocol", "refuse", vec![format!("unknown_op:{other}")]),
        }
    }

    fn ingest_sensor(&mut self) -> MetalResponse {
        if self.bus_lost {
            return self.refuse_bus_lost();
        }
        match self.session.acquire_sensor() {
            Ok(_) => {
                let mut r = self.ok("sensor");
                r.authority_receive_s = Some(self.session.governor.authority_now_s());
                r.device_capture_s = Some(self.session.governor.last_device_capture_s());
                self.persist_freshness(r.device_capture_s, r.authority_receive_s);
                r
            }
            Err(e) => self.note_acquire_err("observe", e),
        }
    }

    fn recover(&mut self) -> MetalResponse {
        if self.bus_lost {
            return self.refuse_bus_lost();
        }
        self.pet_heartbeat();
        let t = self
            .session
            .governor
            .clear_estop_requires_recovery_now(true);
        let mut r = if t.ok {
            self.ok("recover")
        } else {
            self.refuse("authorize", "refuse", t.violations)
        };
        r.present_position = read_i32(self.root.join(crate::config::BUS_DIR).join(PRESENT_FILE));
        r.goal_position = read_i32(self.root.join(crate::config::BUS_DIR).join(GOAL_FILE));
        r
    }

    fn persist_freshness(&self, capture: Option<f64>, receive: Option<f64>) {
        let vin = std::fs::read_to_string(
            self.root
                .join(crate::config::BUS_DIR)
                .join(crate::config::VIN_FILE),
        )
        .ok()
        .and_then(|s| s.trim().parse::<u16>().ok());
        let v = serde_json::json!({
            "sensor_source": "xl330 present_position/velocity/current; capture=Realtime Tick (ms/1000)",
            "device_capture_s": capture,
            "authority_receive_s": receive,
            "freshness_threshold_s": self.cfg.freshness_threshold_s,
            "enforced_sensor_stale_s": self.session.governor.config().sensor_stale_s,
            "clock": "OsMonotonicClock",
            "driver_port": "HardwareDriverPort+Xl330Driver",
            "acquisition": "authority acquire_sensor on propose/sensor; autonomy cannot ingest",
            "vin_0.1v": vin,
            "firmware_id_latched": self.cfg.expected_firmware,
            "max_pwm_limit_raw": self.cfg.max_pwm_limit_raw,
            "max_total_excursion_ticks": self.cfg.max_total_excursion_ticks,
        });
        let _ = std::fs::write(
            self.root.join(crate::config::BUS_DIR).join(FRESHNESS_FILE),
            v.to_string(),
        );
    }

    fn positions(&self) -> (Option<i32>, Option<i32>) {
        let bus = self.root.join(crate::config::BUS_DIR);
        (
            read_i32(bus.join(PRESENT_FILE)),
            read_i32(bus.join(GOAL_FILE)),
        )
    }

    fn propose(&mut self, req: MetalRequest) -> MetalResponse {
        // handle() already ticked the watchdog. Extra heartbeat/watchdog emits
        // here are 4–8 fsyncs and can miss the 100 ms window before the serial
        // write. dispatch_issued ticks again immediately before write_online.
        if self.bus_lost {
            return self.refuse_bus_lost();
        }
        if let Err(e) = self.session.acquire_sensor() {
            return self.note_acquire_err("authorize", e);
        }
        // Sensor I/O sits between handle's tick and dispatch. A 40 ms live
        // read is inside the miss window; refresh so dispatch does not inherit
        // that gap. If acquire itself exceeded 100 ms, this tick latches.
        if let Err(v) = self.pet_watchdog() {
            return self.refuse("egress", "refuse", v);
        }
        self.next_sequence = self.next_sequence.saturating_add(1);
        let now = self.session.governor.authority_now_s();
        let mut dreq = DecideRequest::new(
            Intent::language(&req.verb, &req.verb),
            WorldView {
                tau_max: vec![self.cfg.tau_max],
                ..WorldView::default()
            },
            now,
        );
        dreq.sequence = self.next_sequence;
        dreq.ttl_s = 30.0;
        dreq.command_id = if req.command_id.is_empty() {
            format!("metal-{now}")
        } else {
            req.command_id.clone()
        };
        if let Some(action) = &req.action {
            let mut p = PolicyProposal::operator(action.clone(), "metal-autonomy");
            p.policy_id = "metal".into();
            dreq.proposal = Some(p);
        }
        let decision = self.ros.decide(dreq);
        if !decision.allowed {
            return self.refuse(
                "semantic",
                decision.status.as_str(),
                vec![decision.physical_reason],
            );
        }
        let Some(issued): Option<IssuedCommand> = decision.command else {
            return self.refuse("semantic", "refuse", vec!["no_issued_command".into()]);
        };
        let cid = issued.command_id().to_string();
        let out = self.session.dispatch_issued(issued, &ActionParams::empty());
        // Do not restamp the watchdog after prepare/consume/emit fsync.
        // A real >configured-interval stall must remain visible.
        self.persist_freshness(
            Some(self.session.governor.last_device_capture_s()),
            Some(self.session.governor.authority_now_s()),
        );
        let mut resp = MetalResponse {
            ok: out.ok,
            executed: out.executed,
            stage: if out.ok {
                "write".into()
            } else {
                "egress".into()
            },
            status: if out.ok {
                "allow".into()
            } else {
                "refuse".into()
            },
            violations: out.violations,
            physical_writes: self.physical_writes(),
            command_egress_attempts: self.command_egress_attempts(),
            serial_tx_completed: self.serial_tx_completed(),
            device_acks: self.device_acks(),
            command_id: cid,
            metal: !crate::identity::is_pty_path(&self.cfg.device),
            clock: "OsMonotonicClock".into(),
            present_position: self.positions().0,
            goal_position: self.positions().1,
            device_capture_s: Some(self.session.governor.last_device_capture_s()),
            authority_receive_s: Some(self.session.governor.authority_now_s()),
        };
        if !out.ok
            && resp.stage == "egress"
            && resp.violations.iter().any(|v| v.contains("semantic"))
        {
            resp.stage = "authorize".into();
        }
        if resp.violations.iter().any(|v| {
            v.contains("hardware_") || v.contains("identity") || v.contains("disconnected")
        }) {
            resp.stage = "authorize".into();
        }
        resp
    }

    fn refuse(&self, stage: &str, status: &str, violations: Vec<String>) -> MetalResponse {
        MetalResponse {
            ok: false,
            executed: false,
            stage: stage.into(),
            status: status.into(),
            violations,
            physical_writes: self.physical_writes(),
            command_egress_attempts: self.command_egress_attempts(),
            serial_tx_completed: self.serial_tx_completed(),
            device_acks: self.device_acks(),
            metal: !crate::identity::is_pty_path(&self.cfg.device),
            clock: "OsMonotonicClock".into(),
            ..MetalResponse::default()
        }
    }

    fn ok(&self, stage: &str) -> MetalResponse {
        MetalResponse {
            ok: true,
            executed: false,
            stage: stage.into(),
            status: "ok".into(),
            physical_writes: self.physical_writes(),
            command_egress_attempts: self.command_egress_attempts(),
            serial_tx_completed: self.serial_tx_completed(),
            device_acks: self.device_acks(),
            metal: !crate::identity::is_pty_path(&self.cfg.device),
            clock: "OsMonotonicClock".into(),
            ..MetalResponse::default()
        }
    }

    fn is_bus_loss(err: &str) -> bool {
        let e = err.to_ascii_lowercase();
        e.contains("driver not connected")
            || e.contains("dxl_io")
            || e.contains("dxl_vin_outside_wizard_limits")
            || e.contains("dxl_vin_unreadable")
            || e.contains("metal_live_io_deadline")
            || e.contains("metal_serial_closed")
    }

    fn refuse_bus_lost(&self) -> MetalResponse {
        self.refuse(
            "authorize",
            "refuse",
            vec![
                "hardware_session_requires_online_restart".into(),
                "online_hardware_disconnected".into(),
            ],
        )
    }

    fn note_acquire_err(&mut self, stage: &str, e: String) -> MetalResponse {
        if Self::is_bus_loss(&e) {
            self.bus_lost = true;
            let _ = self.session.governor.engage_estop_now(e.as_str());
            return self.refuse(
                stage,
                "refuse",
                vec![
                    e,
                    "hardware_session_requires_online_restart".into(),
                    "online_hardware_disconnected".into(),
                ],
            );
        }
        self.refuse(stage, "refuse", vec![e])
    }

    /// ONLINE software watchdog is 50 ms (miss at 100 ms). Successful pets
    /// are in-memory only. A gap >100 ms cannot be caught up and is still
    /// durably recorded as ESTOP. Not an independent hardware watchdog.
    ///
    /// Identity/disconnect ESTOP is not a watchdog miss. Folding `estop()`
    /// into this tick made recover-after-identity return
    /// `software_watchdog_miss` and left `hardware_session_requires_online_restart`
    /// unmeasured. A real miss still returns `t.ok == false` and cannot be
    /// caught up.
    fn pet_watchdog(&mut self) -> Result<(), Vec<String>> {
        let t = self.session.governor.watchdog_tick_now();
        if t.ok {
            return Ok(());
        }
        let mut v = t.violations;
        if v.is_empty() {
            v.push("estop_engaged".into());
            v.push("abort_latched:software_watchdog_miss".into());
        }
        Err(v)
    }

    fn pet_heartbeat(&mut self) {
        let _ = self.session.governor.heartbeat_now();
    }
}

pub fn serve_forever(root: &Path, first_online: bool) -> anyhow::Result<()> {
    use std::io::{BufRead, BufReader, ErrorKind};
    use std::time::Duration;

    let mut auth = match MetalAuthority::start(root, first_online) {
        Ok(a) => a,
        Err(e) => {
            let _ = std::fs::write(root.join("serve.err"), format!("{e:#}\n"));
            return Err(e);
        }
    };
    // new_online already ticked; one watchdog pet covers bind without a
    // heartbeat fsync. Schedule the next idle pet from *before* emit so a
    // slow journal fsync cannot push the following Instant gap past 100 ms.
    if let Err(v) = auth.pet_watchdog() {
        let msg = format!("software_watchdog_miss_before_bind:{}", v.join(","));
        let _ = std::fs::write(root.join("serve.err"), format!("{msg}\n"));
        anyhow::bail!("{msg}");
    }
    let listener = crate::ipc::bind_socket(root)?;
    listener.set_nonblocking(true)?;
    // Successful watchdog pets are in-memory only. Heartbeat still fsyncs
    // journal+seal. Do not restamp the watchdog after that persist.
    // Accept waiting IPC before the idle heartbeat (heartbeat+handle in one
    // iteration was software_watchdog_miss with an empty serve.err). Do not
    // pet again on accept: handle() pets.
    let mut next_watchdog = std::time::Instant::now() + Duration::from_millis(40);
    let mut next_heartbeat = std::time::Instant::now() + Duration::from_millis(800);
    loop {
        if root.join("stop_serve").exists() {
            break Ok(());
        }
        let mut stream = match listener.accept() {
            Ok((s, _)) => s,
            Err(e) if e.kind() == ErrorKind::WouldBlock => {
                let now = std::time::Instant::now();
                if now >= next_watchdog {
                    next_watchdog = std::time::Instant::now() + Duration::from_millis(40);
                    if let Err(v) = auth.pet_watchdog() {
                        let _ = std::fs::write(
                            root.join("serve.err"),
                            format!("software_watchdog_miss_idle:{}\n", v.join(",")),
                        );
                    }
                }
                if now >= next_heartbeat {
                    next_heartbeat = std::time::Instant::now() + Duration::from_millis(800);
                    auth.pet_heartbeat();
                    // No post-persist watchdog restamp: heartbeat fsync time
                    // remains visible to the next pet.
                }
                let wait = next_watchdog.saturating_duration_since(std::time::Instant::now());
                std::thread::sleep(wait.min(Duration::from_millis(20)));
                continue;
            }
            Err(_) => continue,
        };
        stream.set_nonblocking(false)?;
        let _ = stream.set_read_timeout(Some(Duration::from_millis(25)));
        let mut line = String::new();
        {
            let mut reader = BufReader::new(&stream);
            loop {
                line.clear();
                match reader.read_line(&mut line) {
                    Ok(0) => break,
                    Ok(_) => break,
                    Err(e)
                        if e.kind() == ErrorKind::TimedOut || e.kind() == ErrorKind::WouldBlock =>
                    {
                        if let Err(v) = auth.pet_watchdog() {
                            let _ = std::fs::write(
                                root.join("serve.err"),
                                format!("software_watchdog_miss_before_handle:{}\n", v.join(",")),
                            );
                            line.clear();
                            break;
                        }
                        next_watchdog = std::time::Instant::now() + Duration::from_millis(40);
                    }
                    Err(_) => {
                        line.clear();
                        break;
                    }
                }
            }
        }
        if line.trim().is_empty() {
            continue;
        }
        let req = match serde_json::from_str::<MetalRequest>(line.trim()) {
            Ok(r) => r,
            Err(e) => {
                crate::ipc::write_response(
                    &mut stream,
                    &MetalResponse {
                        ok: false,
                        stage: "protocol".into(),
                        status: "refuse".into(),
                        violations: vec![format!("bad_request:{e}")],
                        physical_writes: auth.physical_writes(),
                        command_egress_attempts: auth.command_egress_attempts(),
                        serial_tx_completed: auth.serial_tx_completed(),
                        device_acks: auth.device_acks(),
                        metal: !crate::identity::is_pty_path(&auth.cfg.device),
                        clock: "OsMonotonicClock".into(),
                        ..MetalResponse::default()
                    },
                );
                continue;
            }
        };
        if req.op == "shutdown" {
            crate::ipc::write_response(
                &mut stream,
                &auth.refuse(
                    "protocol",
                    "refuse",
                    vec!["production_ipc_refuses:shutdown".into()],
                ),
            );
            continue;
        }
        let resp = auth.handle(req);
        // handle() pets. Push idle Instant out so the next WouldBlock does
        // not add another journal+seal before the following propose.
        next_watchdog = std::time::Instant::now() + Duration::from_millis(40);
        crate::ipc::write_response(&mut stream, &resp);
    }
}

/// Filesystem key. Not a TPM/HSM.
pub fn load_or_create_key(root: impl AsRef<Path>) -> anyhow::Result<Vec<u8>> {
    use std::os::unix::fs::PermissionsExt;
    let path = root.as_ref().join(SIGNING_KEY_FILE);
    match std::fs::read(&path) {
        Ok(bytes) if bytes.len() >= 32 => Ok(bytes),
        Ok(_) => anyhow::bail!("signing_key_too_short"),
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => {
            let mut bytes = vec![0u8; 32];
            fill_random(&mut bytes)?;
            std::fs::write(&path, &bytes)?;
            std::fs::set_permissions(&path, std::fs::Permissions::from_mode(0o600))?;
            Ok(bytes)
        }
        Err(e) => Err(e.into()),
    }
}

fn fill_random(buf: &mut [u8]) -> anyhow::Result<()> {
    let mut f = std::fs::File::open("/dev/urandom")?;
    use std::io::Read;
    f.read_exact(buf)?;
    Ok(())
}

fn read_i32(path: impl AsRef<Path>) -> Option<i32> {
    std::fs::read_to_string(path)
        .ok()
        .and_then(|s| s.trim().parse().ok())
}
