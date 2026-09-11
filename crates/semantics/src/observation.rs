use crate::sensor::SensorClass;
use realityos_kernel::{KernelResult, ObservationEvidence};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

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
pub struct JointStateSample {
    pub joint_name: String,
    pub q: f64,
    pub dq: Option<f64>,
    pub capture_s: f64,
    pub receive_s: f64,
    pub sensor_id: String,
    pub calibration_id: String,
    pub transform_epoch: String,
    pub digest: String,
}

impl JointStateSample {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        joint_name: impl Into<String>,
        q: f64,
        dq: Option<f64>,
        capture_s: f64,
        receive_s: f64,
        sensor_id: impl Into<String>,
        calibration_id: impl Into<String>,
        transform_epoch: impl Into<String>,
    ) -> Self {
        let joint_name = joint_name.into();
        let sensor_id = sensor_id.into();
        let calibration_id = calibration_id.into();
        let transform_epoch = transform_epoch.into();
        let digest = joint_sample_digest(
            &joint_name,
            q,
            &sensor_id,
            &calibration_id,
            &transform_epoch,
        );
        Self {
            joint_name,
            q,
            dq,
            capture_s,
            receive_s,
            sensor_id,
            calibration_id,
            transform_epoch,
            digest,
        }
    }

    pub fn stale(&self, now_s: f64, freshness_s: f64) -> bool {
        now_s - self.receive_s > freshness_s
    }
}

pub fn joint_sample_digest(
    joint_name: &str,
    q: f64,
    sensor_id: &str,
    calibration_id: &str,
    epoch: &str,
) -> String {
    let mut h = Sha256::new();
    h.update(b"realityos.joint_state_sample/1\0");
    h.update(joint_name.as_bytes());
    h.update(q.to_le_bytes());
    h.update(sensor_id.as_bytes());
    h.update(calibration_id.as_bytes());
    h.update(epoch.as_bytes());
    hex::encode(h.finalize())
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ObservationFrame {
    pub frame_id: String,
    pub transform_epoch: String,
    pub observations: Vec<SensorObservation>,
    pub joint_state: Vec<JointStateSample>,
    pub as_of_s: f64,
}

impl ObservationFrame {
    pub fn required_stale(&self, now_s: f64, freshness_s: f64) -> bool {
        let encoders_stale = self.observations.iter().any(|obs| {
            obs.class == SensorClass::JointEncoder
                && (now_s - obs.receive_s > freshness_s || now_s > obs.expires_at_s)
        });
        let joints_stale = self.joint_state.iter().any(|s| s.stale(now_s, freshness_s));
        encoders_stale || joints_stale
    }

    pub fn joint_q(&self, name: &str) -> Option<f64> {
        self.joint_state
            .iter()
            .find(|s| s.joint_name == name)
            .map(|s| s.q)
    }

    pub fn missing_required(&self, names: &[String]) -> bool {
        names.iter().any(|n| self.joint_q(n).is_none())
    }

    pub fn wrong_epoch(&self, epoch: &str) -> bool {
        self.joint_state.iter().any(|s| s.transform_epoch != epoch)
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
            joint_state: vec![],
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
