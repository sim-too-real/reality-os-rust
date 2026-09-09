//! Deterministic fused estimator. Controllers consume belief, not raw samples.

use realityos_kernel::{BeliefState, Estimator, KernelResult, ObservationEvidence, SensorHealth};

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

/// Joints + observation evidence. Missing or OOD vision degrades, never invents pose.
#[derive(Default)]
pub struct FuseEstimator {
    joints: Option<(f64, Vec<f64>)>,
    observation: Option<ObservationEvidence>,
    belief: Option<BeliefState>,
}

impl FuseEstimator {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn ingest_observation(&mut self, ev: ObservationEvidence, now_s: f64) -> KernelResult<()> {
        self.observation = Some(ev);
        self.rebuild(now_s)
    }

    fn rebuild(&mut self, now_s: f64) -> KernelResult<()> {
        let Some((stamp, mean)) = &self.joints else {
            self.belief = None;
            return Ok(());
        };
        let mut cov = vec![0.01; mean.len()];
        let mut health = SensorHealth::Ok;
        let mut innovations = Vec::new();
        if let Some(ev) = &self.observation {
            if ev.is_expired(now_s) || ev.is_ood(0.8) || ev.quality() < 0.2 {
                health = SensorHealth::Degraded;
                for c in &mut cov {
                    *c *= 10.0;
                }
            }
            innovations.push((now_s - ev.receive_mono_s()).abs());
        } else {
            health = SensorHealth::Degraded;
            for c in &mut cov {
                *c *= 4.0;
            }
        }
        self.belief = Some(BeliefState::new(
            "session/fuse_estimator",
            *stamp,
            mean.clone(),
            cov,
            health,
            innovations,
        )?);
        Ok(())
    }
}

impl Estimator for FuseEstimator {
    fn ingest(&mut self, stamp_s: f64, samples: &[(String, f64)]) -> KernelResult<()> {
        let mean: Vec<f64> = samples.iter().map(|(_, v)| *v).collect();
        if mean.is_empty() {
            self.joints = None;
            self.belief = None;
            return Ok(());
        }
        self.joints = Some((stamp_s, mean));
        self.rebuild(stamp_s)
    }

    fn belief(&self) -> Option<&BeliefState> {
        self.belief.as_ref()
    }

    fn fuse_observation(&mut self, stamp_s: f64, digest: &str, quality: f64) -> KernelResult<()> {
        let ev = ObservationEvidence::new(
            "fuse/obs",
            "fuse/cal",
            stamp_s,
            stamp_s,
            digest,
            "fuse/tf",
            quality,
            if quality < 0.2 { 0.9 } else { 0.1 },
            stamp_s + 1.0,
        )?;
        self.ingest_observation(ev, stamp_s)
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

    #[test]
    fn fuse_degrades_without_observation_and_on_ood() {
        let mut e = FuseEstimator::new();
        e.ingest(1.0, &[("q0".into(), 0.2)]).unwrap();
        assert_eq!(e.belief().unwrap().health(), SensorHealth::Degraded);
        e.fuse_observation(1.0, "digest-ok", 0.9).unwrap();
        assert_eq!(e.belief().unwrap().health(), SensorHealth::Ok);
        e.fuse_observation(1.0, "digest-bad", 0.05).unwrap();
        assert_eq!(e.belief().unwrap().health(), SensorHealth::Degraded);
        assert!(e.belief().unwrap().covariance_diag()[0] > 0.05);
    }
}
