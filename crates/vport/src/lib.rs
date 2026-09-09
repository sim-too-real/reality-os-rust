//! Exclusive virtual motor endpoint.
//!
//! Transport is a lock file + write log. Not metal. Not a `Plant`.
//! The authority process opens the log, then sets its mode to `000` so a
//! second ordinary process that does not `chmod` cannot `open()` it.

use std::fs::{File, OpenOptions, Permissions};
use std::io::{self, Write};
use std::os::unix::fs::{OpenOptionsExt, PermissionsExt};
use std::path::{Path, PathBuf};

use fs2::FileExt;
use realityos_plant::{
    hil_faults, ActionParams, HardwareDriverPort, HardwareIdentity, PlantError, PlantRealized,
    PlantResult, SensorPacket, HARNESS_EVIDENCE,
};

pub const ENDPOINT_LOCK: &str = "actuator.lock";
pub const ENDPOINT_LOG: &str = "actuator.log";
pub const WRITES_FILE: &str = "writes";

#[derive(Debug)]
pub struct ExclusiveEndpoint {
    pub root: PathBuf,
    pub lock_path: PathBuf,
    pub log_path: PathBuf,
    /// Held so the exclusive flock lives as long as the endpoint.
    #[allow(dead_code)]
    lock: File,
    log: File,
    writes: u64,
    connected: bool,
}

impl ExclusiveEndpoint {
    pub fn create(root: impl AsRef<Path>) -> io::Result<Self> {
        let root = root.as_ref().to_path_buf();
        std::fs::create_dir_all(&root)?;
        let _ = std::fs::set_permissions(&root, Permissions::from_mode(0o700));
        let lock_path = root.join(ENDPOINT_LOCK);
        let log_path = root.join(ENDPOINT_LOG);
        let lock = OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .truncate(false)
            .mode(0o600)
            .open(&lock_path)?;
        lock.try_lock_exclusive()?;
        // Only the exclusive lock holder may restore mode to reopen after restart.
        // Hostile try_hostile_open does not chmod — that is the measured guarantee.
        if log_path.exists() {
            let _ = std::fs::set_permissions(&log_path, Permissions::from_mode(0o600));
        }
        let mut log = OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .truncate(false)
            .mode(0o600)
            .open(&log_path)?;
        std::io::Seek::seek(&mut log, std::io::SeekFrom::End(0))?;
        std::fs::set_permissions(&log_path, Permissions::from_mode(0o000))?;
        let writes = recorded_writes(&root);
        if !root.join(WRITES_FILE).exists() {
            std::fs::write(root.join(WRITES_FILE), b"0")?;
        }
        Ok(Self {
            root,
            lock_path,
            log_path,
            lock,
            log,
            writes,
            connected: true,
        })
    }

    pub fn write_count(&self) -> u64 {
        self.writes
    }

    pub fn persist_writes(&self) -> io::Result<()> {
        std::fs::write(self.root.join(WRITES_FILE), self.writes.to_string())
    }

    pub fn write_frame(&mut self, command_id: &str, action: &[f64]) -> io::Result<()> {
        if !self.connected {
            return Err(io::Error::other("disconnected"));
        }
        writeln!(
            self.log,
            "ACT {command_id} {}",
            action
                .iter()
                .map(|x| format!("{x}"))
                .collect::<Vec<_>>()
                .join(",")
        )?;
        self.log.sync_all()?;
        self.writes += 1;
        self.persist_writes()?;
        hil_faults::crash_if("during_write");
        Ok(())
    }

    pub fn disconnect(&mut self) {
        self.connected = false;
    }

    pub fn reconnect(&mut self) {
        self.connected = true;
    }

    pub fn is_connected(&self) -> bool {
        self.connected
    }
}

/// Result of a hostile same-process-class attempt to take the endpoint.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct HostileOpenResult {
    pub lock_open_ok: bool,
    pub lock_exclusive_ok: bool,
    pub log_open_ok: bool,
    pub log_write_ok: bool,
}

/// Ordinary untrusted attempt: open + exclusive lock + write. Does not chmod.
pub fn try_hostile_open(root: impl AsRef<Path>) -> HostileOpenResult {
    let root = root.as_ref();
    let lock_path = root.join(ENDPOINT_LOCK);
    let log_path = root.join(ENDPOINT_LOG);
    let lock_open = OpenOptions::new().read(true).write(true).open(&lock_path);
    let (lock_open_ok, lock_exclusive_ok) = match lock_open {
        Ok(f) => (true, f.try_lock_exclusive().is_ok()),
        Err(_) => (false, false),
    };
    let log_open = OpenOptions::new().read(true).write(true).open(&log_path);
    let (log_open_ok, log_write_ok) = match log_open {
        Ok(mut f) => (true, writeln!(f, "HOSTILE").is_ok()),
        Err(_) => (false, false),
    };
    HostileOpenResult {
        lock_open_ok,
        lock_exclusive_ok,
        log_open_ok,
        log_write_ok,
    }
}

pub fn recorded_writes(root: impl AsRef<Path>) -> u64 {
    std::fs::read_to_string(root.as_ref().join(WRITES_FILE))
        .ok()
        .and_then(|s| s.trim().parse().ok())
        .unwrap_or(0)
}

pub struct VirtualSerialPort {
    endpoint: ExclusiveEndpoint,
    identity: HardwareIdentity,
    estop: bool,
    last: Vec<(String, f64)>,
}

impl VirtualSerialPort {
    pub fn open(root: impl AsRef<Path>, serial: impl Into<String>) -> io::Result<Self> {
        Ok(Self {
            endpoint: ExclusiveEndpoint::create(root)?,
            identity: HardwareIdentity {
                serial: serial.into(),
                firmware_id: "HIL-FW-1".into(),
                calibration_id: "HIL-CAL-1".into(),
                design_content_hash: "hil_design".into(),
                connected: true,
                metal: false,
                evidence_status: HARNESS_EVIDENCE.into(),
                actuator_ids: vec!["joint-0".into()],
            },
            estop: false,
            last: vec![("q0".into(), 0.0)],
        })
    }

    pub fn endpoint_root(&self) -> &Path {
        &self.endpoint.root
    }

    pub fn driver_writes(&self) -> u64 {
        self.endpoint.write_count()
    }

    pub fn disconnect_bus(&mut self) {
        self.endpoint.disconnect();
    }

    pub fn reconnect_bus(&mut self) {
        self.endpoint.reconnect();
    }

    pub fn replace_identity(&mut self, identity: HardwareIdentity) {
        self.identity = identity;
    }

    fn overlay_identity(&self, mut id: HardwareIdentity) -> HardwareIdentity {
        let swap = self.endpoint.root.join("hot_swap.json");
        if let Ok(raw) = std::fs::read_to_string(&swap) {
            if let Ok(v) = serde_json::from_str::<serde_json::Value>(&raw) {
                if let Some(s) = v.get("serial").and_then(|x| x.as_str()) {
                    id.serial = s.to_string();
                }
                if let Some(s) = v.get("firmware_id").and_then(|x| x.as_str()) {
                    id.firmware_id = s.to_string();
                }
                if let Some(s) = v.get("calibration_id").and_then(|x| x.as_str()) {
                    id.calibration_id = s.to_string();
                }
                if let Some(s) = v.get("design_content_hash").and_then(|x| x.as_str()) {
                    id.design_content_hash = s.to_string();
                }
                if let Some(arr) = v.get("actuator_ids").and_then(|x| x.as_array()) {
                    id.actuator_ids = arr
                        .iter()
                        .filter_map(|x| x.as_str().map(str::to_string))
                        .collect();
                }
            }
        }
        if self.endpoint.root.join("placeholder_identity").exists() {
            id.serial = "SIM_SERIAL".into();
            id.firmware_id = "SIM_FW".into();
            id.calibration_id = "SIM_CAL".into();
        }
        if self.endpoint.root.join("missing_identity").exists() {
            id.serial.clear();
            id.firmware_id.clear();
            id.calibration_id.clear();
            id.design_content_hash.clear();
        }
        id
    }

    fn bus_connected(&self) -> bool {
        !self.estop && !self.endpoint.root.join("force_disconnect").exists()
    }
}

impl HardwareDriverPort for VirtualSerialPort {
    fn probe_identity(&self) -> HardwareIdentity {
        let mut id = self.overlay_identity(self.identity.clone());
        id.metal = false;
        id.connected = self.bus_connected();
        id.evidence_status = HARNESS_EVIDENCE.into();
        id
    }

    fn read_sensor(&mut self, now_s: f64) -> PlantResult<SensorPacket> {
        if !self.endpoint.is_connected() {
            return Err(PlantError::Disconnected);
        }
        Ok(SensorPacket::from_samples(self.last.clone(), now_s))
    }

    fn write_action(
        &mut self,
        action: &[f64],
        _params: &ActionParams,
    ) -> PlantResult<PlantRealized> {
        if self.estop {
            return Err(PlantError::EstopEngaged);
        }
        if !self.bus_connected() {
            return Err(PlantError::Disconnected);
        }
        self.endpoint
            .write_frame("port", action)
            .map_err(|e| PlantError::refused(format!("vport_io:{e}")))?;
        let peak = action.iter().copied().fold(0.0_f64, |a, b| a.max(b.abs()));
        self.last = vec![("q0".into(), peak)];
        Ok(PlantRealized::sim(self.last.clone()))
    }

    fn engage_hw_estop(&mut self, _reason: &str) {
        self.estop = true;
    }

    fn clear_hw_estop(&mut self, operator_ack: bool) -> PlantResult<()> {
        if !operator_ack {
            return Err(PlantError::OperatorAckRequired);
        }
        self.estop = false;
        Ok(())
    }

    fn is_connected(&self) -> bool {
        self.bus_connected()
    }

    fn is_sim_harness(&self) -> bool {
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn exclusive_lock_blocks_second_writer() {
        let dir = std::env::temp_dir().join(format!(
            "vport-{}",
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        let port = VirtualSerialPort::open(&dir, "SN-V").unwrap();
        let hostile = try_hostile_open(&dir);
        assert!(!hostile.lock_exclusive_ok);
        assert!(!hostile.log_open_ok);
        assert!(!hostile.log_write_ok);
        drop(port);
    }
}
