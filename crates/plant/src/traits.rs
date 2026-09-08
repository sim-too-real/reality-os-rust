use crate::caps::{ActionParams, PlantCaps, PlantRealized};
use crate::error::PlantResult;
use sha2::{Digest, Sha256};

/// Plant protocol. ONLINE implementations must refuse `act` outside certified scope.
pub trait Plant {
    fn caps(&self) -> PlantCaps;
    fn is_online(&self) -> bool;
    fn act(&mut self, action: &[f64], params: &ActionParams) -> PlantResult<PlantRealized>;
    fn sense(&self) -> PlantRealized;
    fn engage_estop(&mut self, reason: &str);
    fn clear_estop(&mut self, operator_ack: bool) -> PlantResult<()>;
    fn follow_waypoints(&mut self, waypoints: &[Vec<f64>]) -> PlantResult<PlantRealized> {
        let _ = waypoints;
        Err(crate::error::PlantError::refused(
            "follow_waypoints_not_supported",
        ))
    }
}

/// Robot-side port. Governor depends on [`Plant`], never on this trait.
/// A port never certifies or acknowledges a command.
pub trait HardwareDriverPort {
    fn probe_identity(&self) -> HardwareIdentity;
    fn read_sensor(&mut self, now_s: f64) -> PlantResult<SensorPacket>;
    fn write_action(&mut self, action: &[f64], params: &ActionParams)
        -> PlantResult<PlantRealized>;
    fn engage_hw_estop(&mut self, reason: &str);
    fn clear_hw_estop(&mut self, operator_ack: bool) -> PlantResult<()>;
    fn is_connected(&self) -> bool;
    fn close(&mut self) {}
    fn is_sim_harness(&self) -> bool {
        false
    }
}

pub const HARNESS_EVIDENCE: &str = "SIM_HARDWARE_DRIVER_HARNESS_NOT_METAL";

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct HardwareIdentity {
    pub serial: String,
    pub firmware_id: String,
    pub calibration_id: String,
    pub design_content_hash: String,
    pub connected: bool,
    pub metal: bool,
    pub evidence_status: String,
}

impl HardwareIdentity {
    pub fn harness(serial: impl Into<String>) -> Self {
        Self {
            serial: serial.into(),
            firmware_id: "HARNESS-FW-1.0.0".into(),
            calibration_id: "HARNESS-CAL-001".into(),
            design_content_hash: "harness_design_content".into(),
            connected: true,
            metal: false,
            evidence_status: HARNESS_EVIDENCE.into(),
        }
    }

    pub fn is_placeholder(&self) -> bool {
        [&self.serial, &self.firmware_id, &self.calibration_id]
            .iter()
            .any(|s| {
                let u = s.trim().to_ascii_uppercase();
                u.is_empty()
                    || u.starts_with("SIM_")
                    || matches!(u.as_str(), "SIM" | "UNKNOWN" | "NONE")
            })
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct SensorPacket {
    pub content_hash: String,
    pub timestamp_s: f64,
    pub samples: Vec<(String, f64)>,
    pub frame_id: String,
    pub sequence: u64,
}

impl SensorPacket {
    pub fn empty() -> Self {
        Self {
            content_hash: String::new(),
            timestamp_s: 0.0,
            samples: Vec::new(),
            frame_id: String::new(),
            sequence: 0,
        }
    }

    pub fn from_samples(samples: Vec<(String, f64)>, timestamp_s: f64) -> Self {
        let content_hash = hash_sensor_samples(&samples);
        Self {
            content_hash,
            timestamp_s,
            samples,
            frame_id: String::new(),
            sequence: 0,
        }
    }

    pub fn is_empty(&self) -> bool {
        self.samples.is_empty()
    }
}

/// Content hash of measurement samples only (no timestamp/frame metadata).
pub fn hash_sensor_samples(samples: &[(String, f64)]) -> String {
    let mut pairs = samples.to_vec();
    pairs.sort_by(|a, b| a.0.cmp(&b.0));
    let canon = serde_json::to_string(&pairs).unwrap_or_default();
    hex::encode(Sha256::digest(canon.as_bytes()))
}
