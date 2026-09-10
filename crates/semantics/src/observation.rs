use crate::sensor::SensorClass;
use realityos_kernel::{KernelResult, ObservationEvidence};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SensorObservation {
    pub sensor_id: String,
    pub class: SensorClass,
    pub robot_frame: String,
    pub calibration_hash: String,
    pub capture_s: f64,
    pub receive_s: f64,
    pub sequence: u64,
    pub clock_domain: String,
    pub digest: String,
    pub expires_at_s: f64,
}

impl SensorObservation {
    #[allow(clippy::too_many_arguments)]
    pub fn joint_encoder(
        sensor_id: impl Into<String>,
        robot_frame: impl Into<String>,
        calibration_hash: impl Into<String>,
        capture_s: f64,
        receive_s: f64,
        sequence: u64,
        clock_domain: impl Into<String>,
        digest: impl Into<String>,
        expires_at_s: f64,
    ) -> Self {
        Self {
            sensor_id: sensor_id.into(),
            class: SensorClass::JointEncoder,
            robot_frame: robot_frame.into(),
            calibration_hash: calibration_hash.into(),
            capture_s,
            receive_s,
            sequence,
            clock_domain: clock_domain.into(),
            digest: digest.into(),
            expires_at_s,
        }
    }

    pub fn to_kernel_handle(
        &self,
        transform_epoch: &str,
        quality: f64,
        ood: f64,
    ) -> KernelResult<ObservationEvidence> {
        ObservationEvidence::new(
            &self.sensor_id,
            &self.calibration_hash,
            self.capture_s,
            self.receive_s,
            &self.digest,
            transform_epoch,
            quality,
            ood,
            self.expires_at_s,
        )
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ObservationFrame {
    pub frame_id: String,
    pub transform_epoch: String,
    pub observations: Vec<SensorObservation>,
    pub as_of_s: f64,
}

impl ObservationFrame {
    pub fn required_stale(&self, now_s: f64, freshness_s: f64) -> bool {
        self.observations.iter().any(|obs| {
            obs.class == SensorClass::JointEncoder
                && (now_s - obs.receive_s > freshness_s || now_s > obs.expires_at_s)
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stale_joint_observation_is_visible() {
        let obs = SensorObservation::joint_encoder(
            "enc0", "j1", "cal", 1.0, 1.0, 1, "sim", "digest", 1.2,
        );
        let frame = ObservationFrame {
            frame_id: "f1".into(),
            transform_epoch: "e0".into(),
            observations: vec![obs],
            as_of_s: 1.0,
        };
        assert!(frame.required_stale(2.0, 0.25));
        assert!(!frame.required_stale(1.1, 0.25));
    }

    #[test]
    fn kernel_citation_does_not_carry_payload_arrays() {
        let obs =
            SensorObservation::joint_encoder("enc0", "j1", "cal", 1.0, 1.0, 1, "sim", "abc", 3.0);
        let k = obs.to_kernel_handle("e0", 0.9, 0.1).unwrap();
        assert_eq!(k.digest(), "abc");
        assert_eq!(k.transform_epoch(), "e0");
    }
}
