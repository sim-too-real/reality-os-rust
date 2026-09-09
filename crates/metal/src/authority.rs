//! Dedicated metal composition root. Not the HIL Authority object.

use std::path::{Path, PathBuf};

use realityos_core::{DecideRequest, Intent, IssuedCommand, PolicyProposal, RealityOs, WorldView};
use realityos_governor::OnlineLocked;
use realityos_plant::{ActionParams, HardwareBackedPlant};
use realityos_session::{RuntimeMode, RuntimeSession, StartArgs};

use crate::config::{
    MetalConfig, CONFIG_FILE, FRESHNESS_FILE, GOAL_FILE, JOURNAL, PRESENT_FILE, SIGNING_KEY_FILE,
};
use crate::egress::{recorded_acks, recorded_writes};
use crate::ipc::{MetalRequest, MetalResponse};
use crate::xl330::Xl330Driver;

pub struct MetalAuthority {
    ros: RealityOs,
    session: RuntimeSession<HardwareBackedPlant<Xl330Driver>, OnlineLocked>,
    root: PathBuf,
    cfg: MetalConfig,
    next_sequence: i64,
}

impl MetalAuthority {
    pub fn start(root: impl AsRef<Path>, first_online: bool) -> anyhow::Result<Self> {
        let root = root.as_ref().to_path_buf();
        std::fs::create_dir_all(&root)?;
        let cfg = MetalConfig::load(root.join(CONFIG_FILE))?;
        if !cfg.expected_ready() {
            anyhow::bail!("metal_expected_identity_missing:run_probe_then_bind_measured");
        }
        if !cfg.device.exists() {
            anyhow::bail!("metal_device_missing:{}", cfg.device.display());
        }
        let key = load_or_create_key(&root)?;
        let driver = Xl330Driver::open(cfg.clone(), &root)?;
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
        let next_sequence = session.governor.ledger().last_sequence().max(0);
        Ok(Self {
            ros: RealityOs::new(),
            session,
            root,
            cfg,
            next_sequence,
        })
    }

    pub fn physical_writes(&self) -> u64 {
        recorded_writes(self.root.join(crate::config::BUS_DIR))
    }

    pub fn device_acks(&self) -> u64 {
        recorded_acks(self.root.join(crate::config::BUS_DIR))
    }

    pub fn handle(&mut self, req: MetalRequest) -> MetalResponse {
        // Watchdog only: a heartbeat emit is another journal+seal fsync pair.
        // Heartbeat stale is 2 s; idle serve already heartbeats ~every 800 ms.
        if !self.pet_watchdog() {
            return self.refuse(
                "egress",
                "refuse",
                vec![
                    "estop_engaged".into(),
                    "abort_latched:software_watchdog_miss".into(),
                ],
            );
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
        match self.session.acquire_sensor() {
            Ok(_) => {
                let mut r = self.ok("sensor");
                r.authority_receive_s = Some(self.session.governor.authority_now_s());
                r.device_capture_s = Some(self.session.governor.last_device_capture_s());
                self.persist_freshness(r.device_capture_s, r.authority_receive_s);
                r
            }
            Err(e) => self.refuse("observe", "refuse", vec![e]),
        }
    }

    fn recover(&mut self) -> MetalResponse {
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
            "clock": "OsMonotonicClock",
            "acquisition": "authority acquire_sensor on propose/sensor; autonomy cannot ingest",
            "vin_0.1v": vin,
            "firmware_id_latched": self.cfg.expected_firmware,
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
        if let Err(e) = self.session.acquire_sensor() {
            return self.refuse("authorize", "refuse", vec![e]);
        }
        // Sensor I/O sits between handle's tick and dispatch. A 40 ms live
        // read is inside the miss window; refresh so dispatch does not inherit
        // that gap. If acquire itself exceeded 100 ms, this tick latches.
        if !self.pet_watchdog() {
            return self.refuse(
                "egress",
                "refuse",
                vec![
                    "estop_engaged".into(),
                    "abort_latched:software_watchdog_miss".into(),
                ],
            );
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
        // prepare/consume/emit fsync after the dispatch tick. Refresh now so
        // the next propose (nudge) does not inherit that gap. A miss here
        // cannot be caught up.
        let _ = self.pet_watchdog();
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
            device_acks: self.device_acks(),
            metal: !crate::identity::is_pty_path(&self.cfg.device),
            clock: "OsMonotonicClock".into(),
            ..MetalResponse::default()
        }
    }

    /// ONLINE software watchdog is 50 ms (miss at 100 ms). One journal+seal
    /// fsync pair. A gap >100 ms cannot be caught up.
    fn pet_watchdog(&mut self) -> bool {
        let t = self.session.governor.watchdog_tick_now();
        t.ok && !self.session.governor.estop()
    }

    fn watchdog_age_s(&self) -> f64 {
        self.session.governor.watchdog_age_s()
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
    // heartbeat fsync.
    if !auth.pet_watchdog() {
        let msg = "software_watchdog_miss_before_bind";
        let _ = std::fs::write(root.join("serve.err"), format!("{msg}\n"));
        anyhow::bail!("{msg}");
    }
    let listener = crate::ipc::bind_socket(root)?;
    listener.set_nonblocking(true)?;
    // Each watchdog/heartbeat emit fsyncs journal+seal. 10 ms pets of both
    // were ~400 fsyncs/s and can miss the 100 ms watchdog on a bench disk.
    // Idle pets follow the authority-clock gap (stamp is pre-persist). A
    // wall Instant schedule can skip a due tick, then accept+handle miss
    // with an empty serve.err.
    let mut next_heartbeat = std::time::Instant::now() + Duration::from_millis(800);
    loop {
        if root.join("stop_serve").exists() {
            break Ok(());
        }
        // Accept before idle heartbeat. Heartbeat persist in the same
        // iteration as handle() was software_watchdog_miss after ~800 ms
        // idle (serve.err empty; refuse at handle's first pet).
        let mut stream = match listener.accept() {
            Ok((s, _)) => s,
            Err(e) if e.kind() == ErrorKind::WouldBlock => {
                if auth.watchdog_age_s() >= 0.040 && !auth.pet_watchdog() {
                    let _ = std::fs::write(root.join("serve.err"), "software_watchdog_miss_idle\n");
                }
                let now = std::time::Instant::now();
                if now >= next_heartbeat {
                    // Heartbeat persist can consume the 100 ms window if the
                    // last watchdog stamp is already tens of ms old.
                    if auth.watchdog_age_s() >= 0.020 && !auth.pet_watchdog() {
                        let _ =
                            std::fs::write(root.join("serve.err"), "software_watchdog_miss_idle\n");
                    }
                    next_heartbeat = now + Duration::from_millis(800);
                    auth.pet_heartbeat();
                    // Heartbeat is another journal+seal pair. Refresh so
                    // the next accept does not inherit that persist gap.
                    if !auth.pet_watchdog() {
                        let _ = std::fs::write(
                            root.join("serve.err"),
                            "software_watchdog_miss_after_heartbeat\n",
                        );
                    }
                }
                let remain = ((0.040 - auth.watchdog_age_s()) * 1000.0).max(0.0) as u64;
                std::thread::sleep(Duration::from_millis(remain.min(20)));
                continue;
            }
            Err(_) => continue,
        };
        if auth.watchdog_age_s() >= 0.040 && !auth.pet_watchdog() {
            let _ = std::fs::write(
                root.join("serve.err"),
                "software_watchdog_miss_before_handle\n",
            );
            continue;
        }
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
                        if !auth.pet_watchdog() {
                            let _ = std::fs::write(
                                root.join("serve.err"),
                                "software_watchdog_miss_before_handle\n",
                            );
                            line.clear();
                            break;
                        }
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
