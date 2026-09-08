use serde::{Deserialize, Serialize};

/// Point-in-time session snapshot for debug (not a certificate).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SessionSnapshot {
    pub mode: String,
    pub release_hash: String,
    pub serial: String,
    pub estop: bool,
    pub last_heartbeat_s: f64,
    pub last_sensor_s: f64,
    pub last_sensor_hash: Option<String>,
    pub writes: u64,
    pub refuses: u64,
    pub metal: bool,
}

impl SessionSnapshot {
    pub fn empty() -> Self {
        Self {
            mode: String::new(),
            release_hash: String::new(),
            serial: String::new(),
            estop: false,
            last_heartbeat_s: 0.0,
            last_sensor_s: 0.0,
            last_sensor_hash: None,
            writes: 0,
            refuses: 0,
            metal: false,
        }
    }
}
