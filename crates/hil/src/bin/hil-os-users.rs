//! Run as the autonomy UID. Measures what that UID can and cannot do.
//! Root is outside the threat model; do not invoke this as root and claim success.

use std::env;
use std::fs::OpenOptions;
use std::io::Write;
use std::os::unix::net::UnixStream;
use std::path::{Path, PathBuf};

use realityos_hil::{
    call, call_raw, ipc_path, verify_proof_consistency, CaseRecord, HilRequest, ProofExtras,
    ProofReport, JOURNAL, SIGNING_KEY_FILE,
};
use realityos_vport::try_hostile_open;
use serde::Serialize;

#[derive(Serialize)]
struct Probe {
    ran_as_uid: u32,
    ran_as_root: bool,
    ipc_connect: bool,
    status_reached_authority: bool,
    read_signing_key: bool,
    write_signing_key: bool,
    open_actuator_lock: bool,
    take_actuator_lock: bool,
    open_actuator_log: bool,
    write_actuator_log: bool,
    modify_journal: bool,
    chmod_signing_key: bool,
    proc_fd_device: bool,
}

fn can_read(p: &Path) -> bool {
    std::fs::read(p).is_ok()
}

fn can_write_append(p: &Path) -> bool {
    OpenOptions::new()
        .create(false)
        .append(true)
        .open(p)
        .and_then(|mut f| writeln!(f, "autonomy-tamper"))
        .is_ok()
}

fn can_chmod(p: &Path) -> bool {
    use std::os::unix::fs::PermissionsExt;
    std::fs::set_permissions(p, std::fs::Permissions::from_mode(0o777)).is_ok()
}

fn proc_fd_open(pid: u32, hint: &str) -> bool {
    let dir = format!("/proc/{pid}/fd");
    let Ok(rd) = std::fs::read_dir(&dir) else {
        return false;
    };
    for ent in rd.flatten() {
        let link = std::fs::read_link(ent.path()).unwrap_or_default();
        if link.to_string_lossy().contains(hint) {
            return std::fs::File::options()
                .write(true)
                .open(ent.path())
                .is_ok();
        }
    }
    false
}

fn euid() -> u32 {
    let s = std::fs::read_to_string("/proc/self/status").unwrap_or_default();
    for line in s.lines() {
        let Some(rest) = line.strip_prefix("Uid:") else {
            continue;
        };
        let mut parts = rest.split_whitespace();
        let _real = parts.next();
        if let Some(e) = parts.next().and_then(|x| x.parse().ok()) {
            return e;
        }
    }
    0
}

fn main() -> anyhow::Result<()> {
    let mut root = PathBuf::from("/tmp/realityos-hil-os");
    let mut op = "probe".to_string();
    let mut authority_pid: u32 = 0;
    let mut cases_path = PathBuf::new();
    let mut out_path = PathBuf::new();
    let mut args = env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--root" => root = PathBuf::from(args.next().unwrap_or_default()),
            "--authority-pid" => {
                authority_pid = args.next().unwrap_or_default().parse().unwrap_or(0)
            }
            "--cases" => cases_path = PathBuf::from(args.next().unwrap_or_default()),
            "--out" => out_path = PathBuf::from(args.next().unwrap_or_default()),
            other => op = other.to_string(),
        }
    }

    if euid() == 0 && op != "report" {
        anyhow::bail!("hil-os-users must not run as root; root is outside the threat model");
    }

    match op.as_str() {
        "report" => {
            let raw = std::fs::read_to_string(&cases_path)?;
            let v: serde_json::Value = serde_json::from_str(&raw)?;
            let cases: Vec<CaseRecord> = if v.is_array() {
                serde_json::from_value(v)?
            } else {
                serde_json::from_value(v.get("cases").cloned().unwrap_or(serde_json::json!([])))?
            };
            let extras = ProofExtras {
                unresolved_trust_assumptions: ProofReport::default_assumptions(),
                ..ProofExtras::default()
            };
            let report =
                ProofReport::from_measured_cases(cases, extras).map_err(anyhow::Error::msg)?;
            verify_proof_consistency(&report).map_err(anyhow::Error::msg)?;
            std::fs::write(&out_path, serde_json::to_string_pretty(&report)?)?;
            println!("{}", serde_json::to_string(&report)?);
        }
        "propose" => {
            let resp = call(&root, &HilRequest::propose("os-valid", "hold"))?;
            println!("{}", serde_json::to_string(&resp)?);
        }
        "unsupported" => {
            let resp = call(&root, &HilRequest::propose("os-unsup", "dance"))?;
            println!("{}", serde_json::to_string(&resp)?);
        }
        "oversized" => {
            let mut r = HilRequest::propose("os-big", "hold");
            r.action = Some(vec![1e6]);
            let resp = call(&root, &r)?;
            println!("{}", serde_json::to_string(&resp)?);
        }
        "replay" => {
            let resp = call(&root, &HilRequest::propose("os-valid", "hold"))?;
            println!("{}", serde_json::to_string(&resp)?);
        }
        "hil_fault" => {
            let mut r = HilRequest::propose("os-fault", "hold");
            r.op = "hil_fault".into();
            r.now_s = 1.0e12;
            let resp = call(&root, &r)?;
            println!("{}", serde_json::to_string(&resp)?);
        }
        "raw" => {
            let resp = call_raw(&root, "{not-json")?;
            println!("{}", serde_json::to_string(&resp)?);
        }
        "propose-id" => {
            let id = env::var("HIL_CMD_ID").unwrap_or_else(|_| "os-x".into());
            let resp = call(&root, &HilRequest::propose(&id, "hold"))?;
            println!("{}", serde_json::to_string(&resp)?);
        }
        _ => {
            let key = root.join(SIGNING_KEY_FILE);
            let journal = root.join(JOURNAL);
            let bus = root.join("bus");
            let hostile = try_hostile_open(&bus);
            let ipc_connect = UnixStream::connect(ipc_path(&root)).is_ok();
            let mut status = HilRequest::propose("os-status", "hold");
            status.op = "status".into();
            let status_resp = call(&root, &status);
            let probe = Probe {
                ran_as_uid: euid(),
                ran_as_root: false,
                ipc_connect,
                status_reached_authority: status_resp.as_ref().map(|r| r.ok).unwrap_or(false),
                read_signing_key: can_read(&key),
                write_signing_key: can_write_append(&key),
                open_actuator_lock: hostile.lock_open_ok,
                take_actuator_lock: hostile.lock_exclusive_ok,
                open_actuator_log: hostile.log_open_ok,
                write_actuator_log: hostile.log_write_ok,
                modify_journal: can_write_append(&journal),
                chmod_signing_key: can_chmod(&key),
                proc_fd_device: authority_pid > 0 && proc_fd_open(authority_pid, "actuator.log"),
            };
            println!("{}", serde_json::to_string(&probe)?);
        }
    }
    Ok(())
}
