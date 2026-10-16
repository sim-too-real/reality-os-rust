use crate::sensor::{SensorClass, WrenchSampleOrigin};
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
pub struct WrenchSample {
    pub sensor_id: String,
    pub sensor_frame: String,
    pub transform_epoch: String,
    pub units: String,
    pub wrench: [f64; 6],
    pub error_bounds: [f64; 6],
    pub capture_s: f64,
    pub receive_s: f64,
    pub sequence: u64,
    pub clock_domain: String,
    pub origin: WrenchSampleOrigin,
    pub digest: String,
}

impl WrenchSample {
    pub fn validate(&self) -> Result<(), &'static str> {
        if self.sensor_id.trim().is_empty()
            || self.sensor_frame.trim().is_empty()
            || self.transform_epoch.trim().is_empty()
            || self.clock_domain.trim().is_empty()
        {
            return Err("identity");
        }
        if self.units != "N,N*m" {
            return Err("units");
        }
        if !self.capture_s.is_finite()
            || !self.receive_s.is_finite()
            || self.capture_s < 0.0
            || self.receive_s < self.capture_s
        {
            return Err("timestamps");
        }
        if self.wrench.iter().any(|value| !value.is_finite()) {
            return Err("wrench");
        }
        if self
            .error_bounds
            .iter()
            .any(|bound| !bound.is_finite() || *bound < 0.0)
        {
            return Err("error_bounds");
        }
        if self.digest.len() != 64 || !self.digest.bytes().all(|byte| byte.is_ascii_hexdigit()) {
            return Err("digest_format");
        }
        if self.digest != wrench_sample_digest(self) {
            return Err("digest_mismatch");
        }
        Ok(())
    }
}

pub fn wrench_sample_digest(sample: &WrenchSample) -> String {
    fn string_field(hash: &mut Sha256, value: &str) {
        hash.update((value.len() as u64).to_le_bytes());
        hash.update(value.as_bytes());
    }

    let mut hash = Sha256::new();
    hash.update(b"realityos.wrench_sample/1\0");
    string_field(&mut hash, &sample.sensor_id);
    string_field(&mut hash, &sample.sensor_frame);
    string_field(&mut hash, &sample.transform_epoch);
    string_field(&mut hash, &sample.units);
    string_field(&mut hash, &sample.clock_domain);
    string_field(&mut hash, sample.origin.as_str());
    for value in sample.wrench {
        hash.update(value.to_bits().to_le_bytes());
    }
    for bound in sample.error_bounds {
        hash.update(bound.to_bits().to_le_bytes());
    }
    hash.update(sample.capture_s.to_bits().to_le_bytes());
    hash.update(sample.receive_s.to_bits().to_le_bytes());
    hash.update(sample.sequence.to_le_bytes());
    hex::encode(hash.finalize())
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
    #[serde(default)]
    pub wrench_samples: Vec<WrenchSample>,
    pub as_of_s: f64,
}

impl ObservationFrame {
    pub fn validate_wrench_samples(&self, now_s: f64, max_age_s: f64) -> Result<(), &'static str> {
        if !now_s.is_finite() || now_s < 0.0 || !max_age_s.is_finite() || max_age_s < 0.0 {
            return Err("validation_time");
        }
        let mut previous_by_sensor = std::collections::BTreeMap::<&str, (u64, f64, f64)>::new();
        for sample in &self.wrench_samples {
            sample.validate()?;
            if sample.transform_epoch != self.transform_epoch {
                return Err("transform_epoch");
            }
            if sample.receive_s > now_s || now_s - sample.receive_s > max_age_s {
                return Err("sample_age");
            }
            if !self.as_of_s.is_finite()
                || sample.capture_s > self.as_of_s
                || sample.receive_s > self.as_of_s
            {
                return Err("frame_time");
            }
            if let Some((previous_sequence, previous_capture_s, previous_receive_s)) =
                previous_by_sensor.get(sample.sensor_id.as_str())
            {
                if sample.sequence <= *previous_sequence
                    || sample.capture_s < *previous_capture_s
                    || sample.receive_s < *previous_receive_s
                {
                    return Err("sample_order");
                }
            }
            previous_by_sensor.insert(
                sample.sensor_id.as_str(),
                (sample.sequence, sample.capture_s, sample.receive_s),
            );
        }
        Ok(())
    }

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
            wrench_samples: vec![],
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
