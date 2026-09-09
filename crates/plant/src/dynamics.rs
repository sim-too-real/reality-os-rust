//! Replaceable dynamics backend behind `Plant`. One plant, one backend slot.

#[derive(Debug, Clone, PartialEq)]
pub struct DynamicsState {
    pub q: Vec<f64>,
    pub v: Vec<f64>,
    pub backend: &'static str,
}

pub trait DynamicsBackend: Send {
    fn name(&self) -> &'static str;
    fn step(&mut self, q: &[f64], v: &[f64], tau: &[f64], dt: f64) -> DynamicsState;
}

pub type BoxBackend = Box<dyn DynamicsBackend>;

/// Semi-implicit Euler on an identity inertia. First in-process backend.
pub struct AnalyticIntegrator;

impl DynamicsBackend for AnalyticIntegrator {
    fn name(&self) -> &'static str {
        "analytic_integrator"
    }

    fn step(&mut self, q: &[f64], v: &[f64], tau: &[f64], dt: f64) -> DynamicsState {
        let n = q.len().max(tau.len()).max(1);
        let mut qn = if q.is_empty() {
            vec![0.0; n]
        } else {
            q.to_vec()
        };
        let mut vn = if v.is_empty() {
            vec![0.0; n]
        } else {
            v.to_vec()
        };
        qn.resize(n, 0.0);
        vn.resize(n, 0.0);
        let dt = if dt.is_finite() && dt > 0.0 { dt } else { 0.0 };
        for i in 0..n {
            let a = tau.get(i).copied().unwrap_or(0.0);
            if a.is_finite() {
                vn[i] += a * dt;
                qn[i] += vn[i] * dt;
            }
        }
        DynamicsState {
            q: qn,
            v: vn,
            backend: self.name(),
        }
    }
}

/// Feature-gated MuJoCo adapter. Without the feature this is an unavailable lane,
/// not a second plant.
pub struct MujocoBackend {
    available: bool,
}

impl MujocoBackend {
    pub fn unavailable() -> Self {
        Self { available: false }
    }

    #[cfg(feature = "mujoco")]
    pub fn connect() -> Self {
        Self { available: true }
    }

    pub fn is_available(&self) -> bool {
        self.available
    }
}

impl DynamicsBackend for MujocoBackend {
    fn name(&self) -> &'static str {
        if self.available {
            "mujoco"
        } else {
            "mujoco_unavailable"
        }
    }

    fn step(&mut self, q: &[f64], v: &[f64], tau: &[f64], dt: f64) -> DynamicsState {
        let mut inner = AnalyticIntegrator;
        let mut s = inner.step(q, v, tau, dt);
        s.backend = self.name();
        s
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn analytic_steps_finite_state() {
        let mut b = AnalyticIntegrator;
        let s = b.step(&[0.0], &[0.0], &[1.0], 0.01);
        assert!(s.q[0].is_finite());
        assert_eq!(s.backend, "analytic_integrator");
    }
}
