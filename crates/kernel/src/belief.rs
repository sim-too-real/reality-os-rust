//! Estimator posterior. Controllers consume this, not raw samples.

use crate::error::{KernelError, KernelResult};
use crate::units::require_finite_slice;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SensorHealth {
    Ok,
    Degraded,
    Dropout,
    Unobservable,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BeliefState {
    model_hash: String,
    stamp_s: f64,
    mean: Vec<f64>,
    covariance_diag: Vec<f64>,
    health: SensorHealth,
    innovations: Vec<f64>,
}

impl BeliefState {
    pub fn new(
        model_hash: impl Into<String>,
        stamp_s: f64,
        mean: Vec<f64>,
        covariance_diag: Vec<f64>,
        health: SensorHealth,
        innovations: Vec<f64>,
    ) -> KernelResult<Self> {
        let model_hash = model_hash.into();
        if model_hash.trim().is_empty() {
            return Err(KernelError::validation("belief.model_hash", "empty"));
        }
        if !stamp_s.is_finite() {
            return Err(KernelError::validation("belief.stamp_s", "non-finite"));
        }
        require_finite_slice(&mean, "belief.mean")?;
        require_finite_slice(&covariance_diag, "belief.covariance")?;
        require_finite_slice(&innovations, "belief.innovations")?;
        if mean.len() != covariance_diag.len() {
            return Err(KernelError::validation(
                "belief.covariance",
                "dimension mismatch",
            ));
        }
        if covariance_diag.iter().any(|x| *x < 0.0) {
            return Err(KernelError::validation(
                "belief.covariance",
                "negative variance",
            ));
        }
        Ok(Self {
            model_hash,
            stamp_s,
            mean,
            covariance_diag,
            health,
            innovations,
        })
    }

    pub fn model_hash(&self) -> &str {
        &self.model_hash
    }
    pub fn stamp_s(&self) -> f64 {
        self.stamp_s
    }
    pub fn mean(&self) -> &[f64] {
        &self.mean
    }
    pub fn covariance_diag(&self) -> &[f64] {
        &self.covariance_diag
    }
    pub fn health(&self) -> SensorHealth {
        self.health
    }
    pub fn innovations(&self) -> &[f64] {
        &self.innovations
    }

    pub fn is_usable_for_control(&self) -> bool {
        matches!(self.health, SensorHealth::Ok | SensorHealth::Degraded)
    }
}

pub trait Estimator {
    fn ingest(&mut self, stamp_s: f64, samples: &[(String, f64)]) -> KernelResult<()>;
    fn belief(&self) -> Option<&BeliefState>;
    fn fuse_observation(&mut self, _stamp_s: f64, _digest: &str, _quality: f64) -> KernelResult<()> {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dim_mismatch_and_nan_refuse() {
        assert!(BeliefState::new(
            "m",
            1.0,
            vec![1.0],
            vec![1.0, 2.0],
            SensorHealth::Ok,
            vec![]
        )
        .is_err());
        assert!(BeliefState::new(
            "m",
            1.0,
            vec![f64::NAN],
            vec![1.0],
            SensorHealth::Ok,
            vec![]
        )
        .is_err());
        let b = BeliefState::new("m", 1.0, vec![0.0], vec![0.1], SensorHealth::Ok, vec![]).unwrap();
        assert!(b.is_usable_for_control());
    }
}
