use crate::provenance::Provenanced;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SensorClass {
    RgbCamera,
    DepthCamera,
    Rgbd,
    JointEncoder,
    JointVelocity,
    ActuatorTorque,
    ForceTorque,
    Imu,
    Other,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SensorModel {
    pub sensor_id: String,
    pub class: SensorClass,
    pub parent_frame: String,
    pub calibration_hash: String,
    pub rate_hz: Provenanced<f64>,
    pub latency_s: Provenanced<f64>,
    pub units: String,
    pub shape: String,
}
