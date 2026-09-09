//! Observation evidence. Caller booleans cannot satisfy see-before-act.

use crate::error::{KernelError, KernelResult};
use crate::units::{require_finite, require_positive};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ObservationEvidence {
    sensor_id: String,
    calibration_hash: String,
    capture_s: f64,
    receive_mono_s: f64,
    digest: String,
    transform_epoch: String,
    quality: f64,
    ood_score: f64,
    expires_at_s: f64,
}

impl ObservationEvidence {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        sensor_id: impl Into<String>,
        calibration_hash: impl Into<String>,
        capture_s: f64,
        receive_mono_s: f64,
        digest: impl Into<String>,
        transform_epoch: impl Into<String>,
        quality: f64,
        ood_score: f64,
        expires_at_s: f64,
    ) -> KernelResult<Self> {
        let sensor_id = sensor_id.into();
        let calibration_hash = calibration_hash.into();
        let digest = digest.into();
        let transform_epoch = transform_epoch.into();
        if sensor_id.trim().is_empty() {
            return Err(KernelError::validation("observation.sensor_id", "empty"));
        }
        if calibration_hash.trim().is_empty() {
            return Err(KernelError::validation(
                "observation.calibration_hash",
                "empty",
            ));
        }
        if digest.trim().is_empty() {
            return Err(KernelError::validation("observation.digest", "empty"));
        }
        if transform_epoch.trim().is_empty() {
            return Err(KernelError::validation(
                "observation.transform_epoch",
                "empty",
            ));
        }
        let capture_s = require_finite(capture_s, "observation.capture_s")?;
        let receive_mono_s = require_finite(receive_mono_s, "observation.receive_mono_s")?;
        let quality = require_finite(quality, "observation.quality")?;
        let ood_score = require_finite(ood_score, "observation.ood_score")?;
        let expires_at_s = require_positive(expires_at_s, "observation.expires_at_s")?;
        if !(0.0..=1.0).contains(&quality) {
            return Err(KernelError::validation(
                "observation.quality",
                "must be in [0, 1]",
            ));
        }
        if !(0.0..=1.0).contains(&ood_score) {
            return Err(KernelError::validation(
                "observation.ood_score",
                "must be in [0, 1]",
            ));
        }
        Ok(Self {
            sensor_id,
            calibration_hash,
            capture_s,
            receive_mono_s,
            digest,
            transform_epoch,
            quality,
            ood_score,
            expires_at_s,
        })
    }

    pub fn sensor_id(&self) -> &str {
        &self.sensor_id
    }
    pub fn calibration_hash(&self) -> &str {
        &self.calibration_hash
    }
    pub fn capture_s(&self) -> f64 {
        self.capture_s
    }
    pub fn receive_mono_s(&self) -> f64 {
        self.receive_mono_s
    }
    pub fn digest(&self) -> &str {
        &self.digest
    }
    pub fn transform_epoch(&self) -> &str {
        &self.transform_epoch
    }
    pub fn quality(&self) -> f64 {
        self.quality
    }
    pub fn ood_score(&self) -> f64 {
        self.ood_score
    }
    pub fn expires_at_s(&self) -> f64 {
        self.expires_at_s
    }

    pub fn is_expired(&self, now_s: f64) -> bool {
        !now_s.is_finite() || now_s > self.expires_at_s
    }

    pub fn is_ood(&self, max_ood: f64) -> bool {
        self.ood_score > max_ood
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ok_ev() -> ObservationEvidence {
        ObservationEvidence::new("cam0", "cal-1", 1.0, 1.0, "digest", "tf-1", 0.9, 0.1, 10.0)
            .unwrap()
    }

    #[test]
    fn rejects_empty_and_non_finite() {
        assert!(ObservationEvidence::new("", "c", 1.0, 1.0, "d", "t", 0.5, 0.1, 2.0).is_err());
        assert!(
            ObservationEvidence::new("s", "c", f64::NAN, 1.0, "d", "t", 0.5, 0.1, 2.0).is_err()
        );
        assert!(ok_ev().is_expired(11.0));
        assert!(!ok_ev().is_expired(9.0));
    }
}
