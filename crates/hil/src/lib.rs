//! Two-process HIL harness. Untrusted autonomy ≠ execution authority.
//! Does not extend the authority kernel.

mod proof;

use std::io::{BufRead, BufReader, Write};
use std::os::unix::net::{UnixListener, UnixStream};
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::{SystemTime, UNIX_EPOCH};

use realityos_core::{DecideRequest, Intent, IssuedCommand, PolicyProposal, RealityOs, WorldView};
use realityos_governor::{OnlineLocked, RuntimeGovernor, RuntimeIdentity};
use realityos_kernel::{
    AuthorityClock, CalibrationId, DesignContentHash, FakeClock, FirmwareId, ReleaseHash,
    SerialOrAsBuilt,
};
use realityos_plant::{ActionParams, HardwareBackedPlant};
use realityos_vport::{recorded_writes, VirtualSerialPort};
use serde::{Deserialize, Serialize};

pub use proof::{
    aggregates_from_cases, verify_proof_consistency, BlockingLayer, CaseRecord, ProofAggregates,
    ProofExtras, ProofReport, PROOF_SCHEMA,
};

pub const IPC_SOCK: &str = "ipc.sock";
pub const JOURNAL: &str = "driver.jsonl";
pub const SIGNING_KEY: &[u8] = b"hil-authority-signing-key";
pub const AUTHORITY_MAX_TTL_S: f64 = 30.0;

/// Data an untrusted proposer may legitimately own.
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct ProductionProposal {
    #[serde(default)]
    pub verb: String,
    #[serde(default)]
    pub action: Option<Vec<f64>>,
    #[serde(default)]
    pub command_id: String,
    #[serde(default)]
    pub proposer: String,
    #[serde(default)]
    pub intent_metadata: serde_json::Value,
}

/// HIL / test-only time and rail injection. Not accepted as production authority.
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct HilFaultInjectionRequest {
    #[serde(default)]
    pub now_s: Option<f64>,
    #[serde(default)]
    pub write_now_s: Option<f64>,
    #[serde(default)]
    pub ttl_s: Option<f64>,
    #[serde(default)]
    pub sequence: Option<i64>,
    #[serde(default)]
    pub skip_sensor: bool,
    /// Drop previously ingested evidence (missing-evidence attacks).
    #[serde(default)]
    pub drop_sensor: bool,
    #[serde(default)]
    pub skip_heartbeat: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HilRequest {
    pub op: String,
    #[serde(default)]
    pub verb: String,
    #[serde(default)]
    pub command_id: String,
    #[serde(default)]
    pub action: Option<Vec<f64>>,
    #[serde(default)]
    pub proposer: String,
    #[serde(default)]
    pub fault: Option<HilFaultInjectionRequest>,
    /// Legacy fields. Ignored on `op=propose`. Used only when `op=hil_fault`.
    #[serde(default)]
    pub now_s: f64,
    #[serde(default)]
    pub sequence: i64,
    #[serde(default)]
    pub ttl_s: f64,
    #[serde(default)]
    pub skip_sensor: bool,
    #[serde(default)]
    pub write_now_s: Option<f64>,
    #[serde(default)]
    pub ttl_override: Option<f64>,
    #[serde(default)]
    pub skip_heartbeat: bool,
}

impl HilRequest {
    pub fn propose(id: &str, verb: &str) -> Self {
        Self {
            op: "propose".into(),
            verb: verb.into(),
            command_id: id.into(),
            action: None,
            proposer: "autonomy".into(),
            fault: None,
            now_s: 0.0,
            sequence: 0,
            ttl_s: 0.0,
            skip_sensor: false,
            write_now_s: None,
            ttl_override: None,
            skip_heartbeat: false,
        }
    }

    /// Back-compat helper used by older call sites; treated as production propose.
    pub fn propose_legacy(id: &str, verb: &str, _now_s: f64, _sequence: i64) -> Self {
        Self::propose(id, verb)
    }

    pub fn hil_fault(id: &str, verb: &str, fault: HilFaultInjectionRequest) -> Self {
        let mut r = Self::propose(id, verb);
        r.op = "hil_fault".into();
        r.fault = Some(fault);
        r
    }

    pub fn production(&self) -> ProductionProposal {
        ProductionProposal {
            verb: self.verb.clone(),
            action: self.action.clone(),
            command_id: self.command_id.clone(),
            proposer: if self.proposer.is_empty() {
                "autonomy".into()
            } else {
                self.proposer.clone()
            },
            intent_metadata: serde_json::Value::Null,
        }
    }

    pub fn is_fault_surface(&self) -> bool {
        self.op == "hil_fault" || self.fault.is_some()
    }

    fn fault(&self) -> HilFaultInjectionRequest {
        if let Some(f) = &self.fault {
            return f.clone();
        }
        HilFaultInjectionRequest {
            now_s: (self.now_s > 0.0).then_some(self.now_s),
            write_now_s: self.write_now_s,
            ttl_s: self
                .ttl_override
                .or((self.ttl_s > 0.0).then_some(self.ttl_s)),
            sequence: (self.sequence != 0).then_some(self.sequence),
            skip_sensor: self.skip_sensor,
            drop_sensor: false,
            skip_heartbeat: self.skip_heartbeat,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct HilResponse {
    pub ok: bool,
    pub executed: bool,
    pub stage: String,
    pub status: String,
    pub violations: Vec<String>,
    pub driver_writes: u64,
    pub command_id: String,
    pub metal: bool,
}

pub fn hil_identity() -> RuntimeIdentity {
    RuntimeIdentity {
        release_hash: ReleaseHash::new("rel-hil-1").expect("rel"),
        design_content_hash: Some(DesignContentHash::new("hil_design").expect("des")),
        serial_or_as_built: Some(SerialOrAsBuilt::new("SN-HIL-1").expect("sn")),
        firmware_id: Some(FirmwareId::new("HIL-FW-1").expect("fw")),
        calibration_id: Some(CalibrationId::new("HIL-CAL-1").expect("cal")),
    }
}

pub struct Authority {
    ros: RealityOs,
    governor: RuntimeGovernor<HardwareBackedPlant<VirtualSerialPort>, OnlineLocked>,
    root: PathBuf,
    clock: Arc<FakeClock>,
    next_sequence: i64,
}

impl Authority {
    pub fn start(root: impl AsRef<Path>, first_online: bool, now_s: f64) -> anyhow::Result<Self> {
        Self::start_with_identity(root, first_online, now_s, hil_identity())
    }

    pub fn start_with_identity(
        root: impl AsRef<Path>,
        first_online: bool,
        now_s: f64,
        identity: RuntimeIdentity,
    ) -> anyhow::Result<Self> {
        let root = root.as_ref().to_path_buf();
        std::fs::create_dir_all(&root)?;
        let clock = FakeClock::arc(now_s);
        let port = VirtualSerialPort::open(root.join("bus"), "SN-HIL-1")?;
        let plant = HardwareBackedPlant::new(port, "hil", 1, 5.0);
        let journal = root.join(JOURNAL);
        let governor = RuntimeGovernor::new_online(
            identity,
            plant,
            journal,
            SIGNING_KEY.to_vec(),
            first_online,
            vec!["joint-0".into()],
            clock.clone(),
        )
        .map_err(|e| anyhow::anyhow!(e.0))?;
        Ok(Self {
            ros: RealityOs::new(),
            governor,
            root,
            clock,
            next_sequence: 0,
        })
    }

    pub fn driver_writes(&self) -> u64 {
        recorded_writes(self.root.join("bus"))
    }

    pub fn handle(&mut self, req: HilRequest) -> HilResponse {
        match req.op.as_str() {
            "sensor" => self.ingest_production(),
            "heartbeat" => {
                let now = self.clock.monotonic_now().secs();
                let _ = self.governor.heartbeat(now);
                self.ok_status("observe")
            }
            "propose" => self.propose_production(req.production()),
            "hil_fault" => self.propose_fault(req.production(), req.fault()),
            "forge_certificate" | "forge_actuation_command" | "write_online_blob" => HilResponse {
                ok: false,
                executed: false,
                stage: "protocol".into(),
                status: "refuse".into(),
                violations: vec!["untrusted_cannot_submit_authority_objects".into()],
                driver_writes: self.driver_writes(),
                command_id: String::new(),
                metal: false,
            },
            "disconnect_driver" => {
                let _ = std::fs::write(self.root.join("bus").join("force_disconnect"), b"1");
                self.ok_status("disconnect")
            }
            "status" => self.ok_status("status"),
            other => HilResponse {
                ok: false,
                executed: false,
                stage: "protocol".into(),
                status: "refuse".into(),
                violations: vec![format!("unknown_op:{other}")],
                driver_writes: self.driver_writes(),
                command_id: String::new(),
                metal: false,
            },
        }
    }

    fn ingest_production(&mut self) -> HilResponse {
        match self.governor.acquire_sensor() {
            Ok(_) => self.ok_status("sensor"),
            Err(e) => HilResponse {
                ok: false,
                stage: "observe".into(),
                status: "refuse".into(),
                violations: vec![e],
                driver_writes: self.driver_writes(),
                metal: false,
                ..HilResponse::default()
            },
        }
    }

    fn propose_production(&mut self, proposal: ProductionProposal) -> HilResponse {
        let now = self.clock.monotonic_now().secs();
        let _ = self.governor.heartbeat(now);
        if let Err(e) = self.governor.acquire_sensor() {
            return HilResponse {
                ok: false,
                executed: false,
                stage: "authorize".into(),
                status: "refuse".into(),
                violations: vec![e],
                driver_writes: self.driver_writes(),
                metal: false,
                ..HilResponse::default()
            };
        }
        self.next_sequence = self.next_sequence.saturating_add(1);
        let seq = self.next_sequence;
        self.finish_propose(proposal, now, AUTHORITY_MAX_TTL_S, seq, now)
    }

    fn propose_fault(
        &mut self,
        proposal: ProductionProposal,
        fault: HilFaultInjectionRequest,
    ) -> HilResponse {
        if let Some(t) = fault.now_s {
            self.clock.set(t);
        }
        let now = self.clock.monotonic_now().secs();
        if !fault.skip_heartbeat {
            let _ = self.governor.heartbeat(now);
        }
        if fault.drop_sensor {
            self.governor.hil_drop_sensor_evidence();
        } else if !fault.skip_sensor {
            let _ = self.governor.acquire_sensor();
        }
        let ttl = fault.ttl_s.unwrap_or(AUTHORITY_MAX_TTL_S);
        let seq = fault.sequence.unwrap_or_else(|| {
            self.next_sequence = self.next_sequence.saturating_add(1);
            self.next_sequence
        });
        let write_at = fault.write_now_s.unwrap_or(now);
        self.finish_propose(proposal, now, ttl, seq, write_at)
    }

    fn finish_propose(
        &mut self,
        proposal: ProductionProposal,
        now_s: f64,
        ttl_s: f64,
        sequence: i64,
        write_at: f64,
    ) -> HilResponse {
        let mut dreq = DecideRequest::new(
            Intent::language(&proposal.verb, &proposal.verb),
            WorldView {
                tau_max: vec![5.0],
                ..WorldView::default()
            },
            now_s,
        );
        dreq.sequence = sequence;
        dreq.ttl_s = ttl_s;
        dreq.command_id = if proposal.command_id.is_empty() {
            format!("hil-{}", now_s)
        } else {
            proposal.command_id.clone()
        };
        if let Some(action) = &proposal.action {
            let mut p = PolicyProposal::operator(action.clone(), "hil");
            p.policy_id = "hil".into();
            dreq.proposal = Some(p);
        }
        let decision = self.ros.decide(dreq);
        if !decision.allowed {
            return HilResponse {
                ok: false,
                executed: false,
                stage: "semantic".into(),
                status: decision.status.as_str().into(),
                violations: vec![decision.physical_reason],
                driver_writes: self.driver_writes(),
                command_id: String::new(),
                metal: false,
            };
        }
        let Some(issued): Option<IssuedCommand> = decision.command else {
            return HilResponse {
                ok: false,
                stage: "semantic".into(),
                status: "refuse".into(),
                violations: vec!["no_issued_command".into()],
                driver_writes: self.driver_writes(),
                metal: false,
                ..HilResponse::default()
            };
        };
        let cid = issued.command_id().to_string();
        let write = match self.governor.authorize_issued(issued) {
            Ok(w) => w,
            Err(errs) => {
                return HilResponse {
                    ok: false,
                    executed: false,
                    stage: "authorize".into(),
                    status: "refuse".into(),
                    violations: errs,
                    driver_writes: self.driver_writes(),
                    command_id: cid,
                    metal: false,
                };
            }
        };
        let trace = self
            .governor
            .write_online(&write, &ActionParams::empty(), write_at);
        let writes = self.driver_writes();
        HilResponse {
            ok: trace.ok,
            executed: trace.ok,
            stage: if trace.ok {
                "write".into()
            } else {
                "egress".into()
            },
            status: if trace.ok {
                "allow".into()
            } else {
                "refuse".into()
            },
            violations: trace.violations,
            driver_writes: writes,
            command_id: cid,
            metal: false,
        }
    }

    fn ok_status(&self, stage: &str) -> HilResponse {
        HilResponse {
            ok: true,
            executed: false,
            stage: stage.into(),
            status: "ok".into(),
            driver_writes: self.driver_writes(),
            metal: false,
            ..HilResponse::default()
        }
    }
}

pub fn ipc_path(root: impl AsRef<Path>) -> PathBuf {
    root.as_ref().join(IPC_SOCK)
}

pub fn serve_forever(root: &Path, first_online: bool, now_s: f64) -> anyhow::Result<()> {
    let mut auth = Authority::start(root, first_online, now_s)?;
    let sock = ipc_path(root);
    let _ = std::fs::remove_file(&sock);
    let listener = UnixListener::bind(&sock)?;
    for stream in listener.incoming() {
        let mut stream = match stream {
            Ok(s) => s,
            Err(_) => continue,
        };
        let mut line = String::new();
        if BufReader::new(&stream).read_line(&mut line).is_err() {
            continue;
        }
        if line.trim().is_empty() {
            continue;
        }
        let req = match serde_json::from_str::<HilRequest>(line.trim()) {
            Ok(r) => r,
            Err(e) => {
                let resp = HilResponse {
                    ok: false,
                    stage: "protocol".into(),
                    status: "refuse".into(),
                    violations: vec![format!("bad_request:{e}")],
                    driver_writes: auth.driver_writes(),
                    metal: false,
                    ..HilResponse::default()
                };
                let _ = writeln!(
                    stream,
                    "{}",
                    serde_json::to_string(&resp).unwrap_or_default()
                );
                let _ = stream.flush();
                continue;
            }
        };
        if req.op == "shutdown" {
            let resp = auth.ok_status("shutdown");
            writeln!(stream, "{}", serde_json::to_string(&resp)?)?;
            stream.flush()?;
            break;
        }
        let resp = auth.handle(req);
        writeln!(stream, "{}", serde_json::to_string(&resp)?)?;
        stream.flush()?;
    }
    Ok(())
}

pub fn call(root: impl AsRef<Path>, req: &HilRequest) -> anyhow::Result<HilResponse> {
    let mut stream = UnixStream::connect(ipc_path(root))?;
    writeln!(stream, "{}", serde_json::to_string(req)?)?;
    let mut line = String::new();
    BufReader::new(&stream).read_line(&mut line)?;
    Ok(serde_json::from_str(line.trim())?)
}

/// Send a raw JSON line. Used for NaN/Infinity tokens serde cannot emit.
pub fn call_raw(root: impl AsRef<Path>, line: &str) -> anyhow::Result<HilResponse> {
    let mut stream = UnixStream::connect(ipc_path(root))?;
    writeln!(stream, "{line}")?;
    let mut resp = String::new();
    BufReader::new(stream).read_line(&mut resp)?;
    Ok(serde_json::from_str(resp.trim())?)
}

pub fn wait_for_ipc(root: impl AsRef<Path>, timeout_ms: u64) -> bool {
    let p = ipc_path(root);
    let start = SystemTime::now();
    while start.elapsed().map(|d| d.as_millis() as u64).unwrap_or(0) < timeout_ms {
        if p.exists() {
            return true;
        }
        std::thread::sleep(std::time::Duration::from_millis(10));
    }
    false
}

pub fn nanos() -> u128 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_nanos())
        .unwrap_or(0)
}

pub fn layer_from_stage(stage: &str) -> BlockingLayer {
    match stage {
        "protocol" | "ipc" => BlockingLayer::ProtocolBlocked,
        "semantic" | "authorize" => BlockingLayer::AuthorizationBlocked,
        "egress" | "observe" => BlockingLayer::EgressBlocked,
        "os" => BlockingLayer::OsBlocked,
        "write" => BlockingLayer::None,
        other
            if other.starts_with("crash") || other.contains("prepare") || other.contains("ack") =>
        {
            BlockingLayer::CrashRecoveryBlocked
        }
        _ => BlockingLayer::AuthorizationBlocked,
    }
}
