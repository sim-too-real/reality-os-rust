use crate::caps::{ActionParams, PlantCaps, PlantRealized};
use crate::error::PlantResult;

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
pub trait HardwareDriverPort {
    fn probe_identity(&self) -> HardwareIdentity;
    fn read_sensor(&self) -> SensorPacket;
    fn write_action(&mut self, action: &[f64], params: &ActionParams)
        -> PlantResult<PlantRealized>;
    fn engage_hw_estop(&mut self, reason: &str);
    fn clear_hw_estop(&mut self, operator_ack: bool) -> PlantResult<()>;
    fn is_connected(&self) -> bool;
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct HardwareIdentity {
    pub serial: String,
    pub firmware_id: String,
    pub connected: bool,
}

#[derive(Debug, Clone, PartialEq)]
pub struct SensorPacket {
    pub content_hash: String,
    pub timestamp_s: f64,
    pub samples: Vec<(String, f64)>,
}

impl SensorPacket {
    pub fn empty() -> Self {
        Self {
            content_hash: String::new(),
            timestamp_s: 0.0,
            samples: Vec::new(),
        }
    }

    pub fn is_empty(&self) -> bool {
        self.samples.is_empty()
    }
}
