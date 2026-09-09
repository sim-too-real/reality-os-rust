//! Two-process HIL harness. Untrusted autonomy ≠ execution authority.
//! Does not extend the authority kernel.

use std::io::{BufRead, BufReader, Write};
use std::os::unix::net::{UnixListener, UnixStream};
use std::path::{Path, PathBuf};
use std::time::{SystemTime, UNIX_EPOCH};

use realityos_core::{DecideRequest, Intent, IssuedCommand, PolicyProposal, RealityOs, WorldView};
use realityos_governor::{OnlineLocked, RuntimeGovernor, RuntimeIdentity};
use realityos_kernel::{
    CalibrationId, DesignContentHash, FirmwareId, ReleaseHash, SerialOrAsBuilt,
};
use realityos_plant::{ActionParams, HardwareBackedPlant};
use realityos_vport::{recorded_writes, VirtualSerialPort};
use serde::{Deserialize, Serialize};

pub const IPC_SOCK: &str = "ipc.sock";
pub const JOURNAL: &str = "driver.jsonl";
pub const SIGNING_KEY: &[u8] = b"hil-authority-signing-key";

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HilRequest {
    pub op: String,
    #[serde(default)]
    pub verb: String,
    #[serde(default)]
    pub now_s: f64,
    #[serde(default)]
    pub sequence: i64,
    #[serde(default)]
    pub command_id: String,
    #[serde(default)]
    pub action: Option<Vec<f64>>,
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
    pub fn propose(id: &str, verb: &str, now_s: f64, sequence: i64) -> Self {
        Self {
            op: "propose".into(),
            verb: verb.into(),
            now_s,
            sequence,
            command_id: id.into(),
            action: None,
            ttl_s: 30.0,
            skip_sensor: false,
            write_now_s: None,
            ttl_override: None,
            skip_heartbeat: false,
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

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct CaseRecord {
    pub name: String,
    pub proposal: String,
    pub semantic_verdict: String,
    pub authority_transition: String,
    pub driver_write_count: u64,
    pub journal_state: String,
    pub outcome: String,
    pub unauthorized_write: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct ProofReport {
    pub schema: String,
    pub hostile_cases: u64,
    pub refused_before_authorization: u64,
    pub refused_before_driver_egress: u64,
    pub unauthorized_driver_writes: u64,
    pub valid_commands: u64,
    pub valid_driver_writes: u64,
    pub duplicate_writes_after_restart: u64,
    pub direct_device_open_attempts: u64,
    pub direct_device_open_succeeded: u64,
    pub journal_continuity_failures_detected: u64,
    pub unresolved_trust_assumptions: Vec<String>,
    pub cases: Vec<CaseRecord>,
}

impl ProofReport {
    pub fn new() -> Self {
        Self {
            schema: "realityos.hil_proof/1".into(),
            unresolved_trust_assumptions: vec![
                "same-UID chmod or /proc/<pid>/fd can recover a mode-000 log".into(),
                "root can open any endpoint".into(),
                "journal+seal is tamper-evident, not authenticated or WORM".into(),
                "independent STO/SS1 / safety PLC is a named hole".into(),
                "sensor samples are still caller-supplied".into(),
            ],
            ..Self::default()
        }
    }
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
}

impl Authority {
    pub fn start(root: impl AsRef<Path>, first_online: bool, now_s: f64) -> anyhow::Result<Self> {
        let root = root.as_ref().to_path_buf();
        std::fs::create_dir_all(&root)?;
        let port = VirtualSerialPort::open(root.join("bus"), "SN-HIL-1")?;
        let plant = HardwareBackedPlant::new(port, "hil", 1, 5.0);
        let journal = root.join(JOURNAL);
        let governor = RuntimeGovernor::new_online(
            hil_identity(),
            plant,
            journal,
            SIGNING_KEY.to_vec(),
            first_online,
            vec!["joint-0".into()],
            now_s,
        )
        .map_err(|e| anyhow::anyhow!(e.0))?;
        Ok(Self {
            ros: RealityOs::new(),
            governor,
            root,
        })
    }

    pub fn driver_writes(&self) -> u64 {
        recorded_writes(self.root.join("bus"))
    }

    pub fn handle(&mut self, req: HilRequest) -> HilResponse {
        match req.op.as_str() {
            "sensor" => self.ingest(req.now_s),
            "heartbeat" => {
                let _ = self.governor.heartbeat(req.now_s);
                self.ok_status("observe")
            }
            "propose" => self.propose(req),
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

    fn ingest(&mut self, now_s: f64) -> HilResponse {
        match self
            .governor
            .record_sensor(&[("q0".into(), 0.0)], now_s, 1, "hil/sensor", "hil")
        {
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

    fn propose(&mut self, req: HilRequest) -> HilResponse {
        if !req.skip_heartbeat {
            let _ = self.governor.heartbeat(req.now_s);
        }
        if !req.skip_sensor {
            let _ = self.ingest(req.now_s);
        }
        let mut dreq = DecideRequest::new(
            Intent::language(&req.verb, &req.verb),
            WorldView {
                tau_max: vec![5.0],
                ..WorldView::default()
            },
            req.now_s,
        );
        dreq.sequence = if req.sequence == 0 { 1 } else { req.sequence };
        dreq.command_id = if req.command_id.is_empty() {
            format!("hil-{}", req.now_s)
        } else {
            req.command_id.clone()
        };
        if let Some(ttl) = req.ttl_override {
            dreq.ttl_s = ttl;
        } else if req.ttl_s > 0.0 {
            dreq.ttl_s = req.ttl_s;
        }
        if let Some(action) = &req.action {
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
        let write_at = req.write_now_s.unwrap_or(req.now_s);
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
        let mut stream = stream?;
        let mut line = String::new();
        BufReader::new(&stream).read_line(&mut line)?;
        if line.trim().is_empty() {
            continue;
        }
        let req: HilRequest = serde_json::from_str(line.trim())?;
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
    BufReader::new(stream).read_line(&mut line)?;
    Ok(serde_json::from_str(line.trim())?)
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
