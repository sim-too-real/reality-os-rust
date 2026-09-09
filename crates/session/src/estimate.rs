//! Deterministic EKF-shaped estimator. Controllers consume belief, not raw samples.

use realityos_kernel::{BeliefState, Estimator, KernelResult, SensorHealth};

#[derive(Default)]
pub struct HoldEstimator {
    belief: Option<BeliefState>,
}

impl HoldEstimator {
    pub fn new() -> Self {
        Self { belief: None }
    }
}

impl Estimator for HoldEstimator {
    fn ingest(&mut self, stamp_s: f64, samples: &[(String, f64)]) -> KernelResult<()> {
        let mean: Vec<f64> = samples.iter().map(|(_, v)| *v).collect();
        if mean.is_empty() {
            self.belief = None;
            return Ok(());
        }
        let cov = vec![0.01; mean.len()];
        self.belief = Some(BeliefState::new(
            "session/hold_estimator",
            stamp_s,
            mean,
            cov,
            SensorHealth::Ok,
            Vec::new(),
        )?);
        Ok(())
    }

    fn belief(&self) -> Option<&BeliefState> {
        self.belief.as_ref()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ingest_builds_usable_belief() {
        let mut e = HoldEstimator::new();
        e.ingest(1.0, &[("q0".into(), 0.1)]).unwrap();
        assert!(e.belief().unwrap().is_usable_for_control());
    }
}
