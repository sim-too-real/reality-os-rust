//! Screen ~50 Hz / Dispose 1 kHz SIM dual-rate. Not PREEMPT_RT. Not HOCBF.

use realityos_kernel::DecisionStatus;
use serde::{Deserialize, Serialize};

use crate::certificate::Certificate;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ExecutionMode {
    TrustedFastpath,
    QpInterception,
    PassiveFallback,
}

impl ExecutionMode {
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::TrustedFastpath => "TRUSTED_FASTPATH",
            Self::QpInterception => "QP_INTERCEPTION",
            Self::PassiveFallback => "PASSIVE_FALLBACK",
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BoundedTrustEnvelope {
    pub tau_max: Vec<f64>,
    pub dq_max: Vec<f64>,
    pub is_valid: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DisposeStatus {
    pub mode: ExecutionMode,
    pub u: Vec<f64>,
    pub certificate: Certificate,
    pub metal: bool,
}

pub fn project_box(u: &[f64], tau_max: &[f64]) -> Vec<f64> {
    if tau_max.is_empty() {
        return u.to_vec();
    }
    u.iter()
        .enumerate()
        .map(|(i, v)| {
            let lim = tau_max[i.min(tau_max.len() - 1)].abs();
            v.clamp(-lim, lim)
        })
        .collect()
}

pub fn certify_dispose_step(
    u_nom: &[f64],
    dq: &[f64],
    envelope: &BoundedTrustEnvelope,
    hold_latched: bool,
) -> DisposeStatus {
    if hold_latched || !envelope.is_valid || u_nom.iter().any(|x| !x.is_finite()) {
        return passive(u_nom.len());
    }
    if envelope.tau_max.is_empty() {
        return passive(u_nom.len());
    }
    let overspeed = dq.iter().enumerate().any(|(i, v)| {
        let lim = envelope.dq_max.get(i).copied().unwrap_or(f64::INFINITY);
        v.abs() > lim + 1e-12
    });
    if overspeed {
        return passive(u_nom.len());
    }
    let interior = u_nom.iter().enumerate().all(|(i, v)| {
        let lim = envelope.tau_max[i.min(envelope.tau_max.len() - 1)].abs();
        v.abs() <= lim + 1e-12
    });
    if interior {
        return DisposeStatus {
            mode: ExecutionMode::TrustedFastpath,
            u: u_nom.to_vec(),
            certificate: Certificate::new(DecisionStatus::Allow, "trusted fastpath interior"),
            metal: false,
        };
    }
    let u = project_box(u_nom, &envelope.tau_max);
    DisposeStatus {
        mode: ExecutionMode::QpInterception,
        u,
        certificate: Certificate::new(DecisionStatus::Modify, "box projection (not HOCBF)"),
        metal: false,
    }
}

fn passive(n: usize) -> DisposeStatus {
    DisposeStatus {
        mode: ExecutionMode::PassiveFallback,
        u: vec![0.0; n.max(1)],
        certificate: Certificate::new(DecisionStatus::Modify, "passive fallback hold"),
        metal: false,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn interior_fastpath_outside_projects() {
        let env = BoundedTrustEnvelope {
            tau_max: vec![1.0],
            dq_max: vec![10.0],
            is_valid: true,
        };
        let a = certify_dispose_step(&[0.2], &[0.0], &env, false);
        assert_eq!(a.mode, ExecutionMode::TrustedFastpath);
        let b = certify_dispose_step(&[3.0], &[0.0], &env, false);
        assert_eq!(b.mode, ExecutionMode::QpInterception);
        assert!((b.u[0] - 1.0).abs() < 1e-12);
        let c = certify_dispose_step(&[0.2], &[0.0], &env, true);
        assert_eq!(c.mode, ExecutionMode::PassiveFallback);
    }
}
