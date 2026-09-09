use crate::caps::{ActionParams, PlantCaps, PlantRealized};
use crate::error::PlantResult;
use sha2::{Digest, Sha256};

mod sealed {
    pub trait Sealed {}
}

/// Plant protocol. ONLINE implementations must refuse `act` outside certified scope.
/// Sealed: only this crate may implement `Plant`. External crates implement
/// [`HardwareDriverPort`] and wrap it in [`crate::HardwareBackedPlant`].
pub trait Plant: sealed::Sealed {
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
    fn probe_identity(&mut self) -> Option<HardwareIdentity> {
        None
    }
    fn write_count(&self) -> u32 {
        0
    }
    /// Irreversible production lock. Default is a no-op (SIM plants that ignore it
    /// still face `force_online_rails` from `RuntimeGovernor<OnlineLocked>`).
    fn lock_production(&mut self, signing_key: &[u8]) {
        let _ = signing_key;
    }
    fn production_locked(&self) -> bool {
        false
    }
    fn production_key_hash(&self) -> Option<&str> {
        None
    }
}

/// Robot-side transport. Governor depends on [`Plant`], never on this trait.
///
/// **Unsealed on purpose.** Vendor crates may implement this port. A port cannot
/// certify, acknowledge, widen, or mint the certified-write token. Calling
/// `write_action` on a port you own is transport ownership — not semantic
/// certification. Machine-wide single-writer is an OS/bus topology property.
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
    pub sensor_id: String,
    pub calibration_hash: String,
}

impl SensorPacket {
    pub fn empty() -> Self {
        Self {
            content_hash: String::new(),
            timestamp_s: 0.0,
            samples: Vec::new(),
            frame_id: String::new(),
            sequence: 0,
            sensor_id: String::new(),
            calibration_hash: String::new(),
        }
    }

    pub fn from_samples(samples: Vec<(String, f64)>, timestamp_s: f64) -> Self {
        let content_hash = hash_sensor_packet(&samples, timestamp_s, "", "", 0);
        Self {
            content_hash,
            timestamp_s,
            samples,
            frame_id: String::new(),
            sequence: 0,
            sensor_id: String::new(),
            calibration_hash: String::new(),
        }
    }

    pub fn is_empty(&self) -> bool {
        self.samples.is_empty()
    }

    pub fn rehash(&mut self) {
        self.content_hash = hash_sensor_packet(
            &self.samples,
            self.timestamp_s,
            &self.frame_id,
            &self.sensor_id,
            self.sequence,
        );
    }
}

/// Content hash of measurement samples only (no timestamp/frame metadata).
pub fn hash_sensor_samples(samples: &[(String, f64)]) -> String {
    let mut pairs = samples.to_vec();
    pairs.sort_by(|a, b| a.0.cmp(&b.0));
    let canon = serde_json::to_string(&pairs).unwrap_or_default();
    hex::encode(Sha256::digest(canon.as_bytes()))
}

/// Bind samples to time, frame, sensor identity, and sequence.
pub fn hash_sensor_packet(
    samples: &[(String, f64)],
    timestamp_s: f64,
    frame_id: &str,
    sensor_id: &str,
    sequence: u64,
) -> String {
    let mut pairs = samples.to_vec();
    pairs.sort_by(|a, b| a.0.cmp(&b.0));
    let canon = serde_json::json!({
        "samples": pairs,
        "timestamp_s": timestamp_s,
        "frame_id": frame_id,
        "sensor_id": sensor_id,
        "sequence": sequence,
    });
    hex::encode(Sha256::digest(
        serde_json::to_string(&canon).unwrap_or_default().as_bytes(),
    ))
}

impl sealed::Sealed for crate::sim::SimPlant {}
impl<P: HardwareDriverPort> sealed::Sealed for crate::backed::HardwareBackedPlant<P> {}
