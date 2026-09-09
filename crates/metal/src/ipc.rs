//! Production IPC only. Autonomy may propose; it may not inject time or HIL faults.

use std::io::{BufRead, BufReader, Write};
use std::os::unix::fs::PermissionsExt;
use std::os::unix::net::{UnixListener, UnixStream};
use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};

use crate::config::{IPC_SOCK, IPC_SOCKET_MODE};

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct MetalRequest {
    pub op: String,
    #[serde(default)]
    pub verb: String,
    #[serde(default)]
    pub command_id: String,
    #[serde(default)]
    pub action: Option<Vec<f64>>,
    #[serde(default)]
    pub proposer: String,
    /// If present on a production propose, the request is refused.
    #[serde(default)]
    pub now_s: Option<f64>,
    #[serde(default)]
    pub fault: Option<serde_json::Value>,
    /// Autonomy-supplied samples. Always refused; authority acquires from the device.
    #[serde(default)]
    pub sensor_samples: Option<serde_json::Value>,
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct MetalResponse {
    pub ok: bool,
    pub executed: bool,
    pub stage: String,
    pub status: String,
    pub violations: Vec<String>,
    pub physical_writes: u64,
    pub device_acks: u64,
    pub command_id: String,
    pub metal: bool,
    pub clock: String,
    #[serde(default)]
    pub present_position: Option<i32>,
    #[serde(default)]
    pub goal_position: Option<i32>,
    #[serde(default)]
    pub device_capture_s: Option<f64>,
    #[serde(default)]
    pub authority_receive_s: Option<f64>,
}

impl MetalRequest {
    pub fn propose(id: &str, verb: &str) -> Self {
        Self {
            op: "propose".into(),
            verb: verb.into(),
            command_id: id.into(),
            action: None,
            proposer: "autonomy".into(),
            now_s: None,
            fault: None,
            sensor_samples: None,
        }
    }

    pub fn production_ops_only(&self) -> bool {
        matches!(
            self.op.as_str(),
            "propose" | "sensor" | "heartbeat" | "status" | "recover"
        )
    }

    pub fn injects_caller_time_or_hil(&self) -> bool {
        self.op == "hil_fault"
            || self.fault.is_some()
            || (self.op == "propose" && self.now_s.is_some())
    }

    pub fn injects_sensor_evidence(&self) -> bool {
        self.sensor_samples.is_some()
    }
}

pub fn ipc_path(root: impl AsRef<Path>) -> PathBuf {
    root.as_ref().join(IPC_SOCK)
}

pub fn bind_socket(root: impl AsRef<Path>) -> anyhow::Result<UnixListener> {
    let sock = ipc_path(root);
    let _ = std::fs::remove_file(&sock);
    let listener = UnixListener::bind(&sock)?;
    std::fs::set_permissions(&sock, std::fs::Permissions::from_mode(IPC_SOCKET_MODE))?;
    Ok(listener)
}

pub fn call(root: impl AsRef<Path>, req: &MetalRequest) -> anyhow::Result<MetalResponse> {
    let mut stream = UnixStream::connect(ipc_path(root))?;
    writeln!(stream, "{}", serde_json::to_string(req)?)?;
    let mut line = String::new();
    BufReader::new(&stream).read_line(&mut line)?;
    Ok(serde_json::from_str(line.trim())?)
}

pub fn call_raw(root: impl AsRef<Path>, line: &str) -> anyhow::Result<MetalResponse> {
    let mut stream = UnixStream::connect(ipc_path(root))?;
    writeln!(stream, "{line}")?;
    let mut resp = String::new();
    BufReader::new(stream).read_line(&mut resp)?;
    Ok(serde_json::from_str(resp.trim())?)
}

pub fn wait_for_ipc(root: impl AsRef<Path>, timeout_ms: u64) -> bool {
    let p = ipc_path(root);
    let start = std::time::Instant::now();
    while (start.elapsed().as_millis() as u64) < timeout_ms {
        if p.exists() {
            return true;
        }
        std::thread::sleep(std::time::Duration::from_millis(10));
    }
    false
}

pub fn write_response(stream: &mut UnixStream, resp: &MetalResponse) {
    let _ = writeln!(
        stream,
        "{}",
        serde_json::to_string(resp).unwrap_or_default()
    );
    let _ = stream.flush();
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn production_propose_rejects_caller_time_and_hil() {
        let mut r = MetalRequest::propose("a", "hold");
        assert!(r.production_ops_only());
        assert!(!r.injects_caller_time_or_hil());
        r.now_s = Some(1.0);
        assert!(r.injects_caller_time_or_hil());
        r.now_s = None;
        r.op = "hil_fault".into();
        assert!(r.injects_caller_time_or_hil());
        assert!(!r.production_ops_only());
        r.op = "propose".into();
        r.sensor_samples = Some(serde_json::json!([{"q0": 1.0}]));
        assert!(r.injects_sensor_evidence());
    }
}
