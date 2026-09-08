//! Physical action limits at the write boundary. ALLOW cert cannot bypass.

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct DriverEnvelopePack {
    pub max_action_abs: f64,
    #[serde(default)]
    pub max_action_per_dim: Vec<f64>,
    #[serde(default)]
    pub action_dim: Option<usize>,
    #[serde(default)]
    pub max_force_n: Option<f64>,
    #[serde(default)]
    pub max_torque_nm: Option<f64>,
    #[serde(default)]
    pub source: String,
    #[serde(default)]
    pub require_for_write: bool,
}

impl DriverEnvelopePack {
    pub fn scalar(max_action_abs: f64, require_for_write: bool) -> Self {
        Self {
            max_action_abs,
            max_action_per_dim: Vec::new(),
            action_dim: None,
            max_force_n: None,
            max_torque_nm: None,
            source: "session_limits".into(),
            require_for_write,
        }
    }

    pub fn from_max_action(max: &[f64], require_for_write: bool) -> Self {
        let peak = max.iter().copied().fold(0.0_f64, f64::max);
        Self {
            max_action_abs: if peak > 0.0 { peak } else { 1.0 },
            max_action_per_dim: max.to_vec(),
            action_dim: Some(max.len()),
            max_force_n: None,
            max_torque_nm: None,
            source: "plant_caps".into(),
            require_for_write,
        }
    }

    pub fn is_complete(&self) -> bool {
        if !self.max_action_abs.is_finite() || self.max_action_abs <= 0.0 {
            return false;
        }
        if self
            .max_action_per_dim
            .iter()
            .any(|x| !x.is_finite() || *x <= 0.0)
        {
            return false;
        }
        true
    }

    /// Empty list means within envelope.
    pub fn check_action(&self, action: &[f64]) -> Vec<String> {
        let mut errs = Vec::new();
        if !self.is_complete() {
            if self.require_for_write {
                errs.push("envelope_pack_incomplete".into());
            }
            return errs;
        }
        if action.is_empty() {
            errs.push("envelope_missing_allowed_action".into());
            return errs;
        }
        if action.iter().any(|a| !a.is_finite()) {
            errs.push("envelope_non_finite_allowed_action".into());
            return errs;
        }
        if let Some(dim) = self.action_dim {
            if dim > 0 && action.len() != dim {
                errs.push(format!(
                    "envelope_action_dim_mismatch:{}!={}",
                    action.len(),
                    dim
                ));
            }
        }
        let peak = action.iter().fold(0.0_f64, |a, b| a.max(b.abs()));
        if peak > self.max_action_abs + 1e-12 {
            errs.push(format!(
                "envelope_action_exceeds_max_abs:{peak}>{}",
                self.max_action_abs
            ));
        }
        if !self.max_action_per_dim.is_empty() {
            for (i, a) in action.iter().enumerate() {
                let j = i.min(self.max_action_per_dim.len() - 1);
                let lim = self.max_action_per_dim[j];
                if a.abs() > lim + 1e-12 {
                    errs.push(format!("envelope_action_exceeds_dim_{i}:{}>{lim}", a.abs()));
                }
            }
        }
        if let Some(mf) = self.max_force_n {
            if action.len() == 1 && mf.is_finite() && mf > 0.0 && action[0].abs() > mf + 1e-12 {
                errs.push(format!(
                    "envelope_force_exceeds_max_n:{}>{mf}",
                    action[0].abs()
                ));
            }
        }
        if let Some(mt) = self.max_torque_nm {
            if action.len() == 1 && mt.is_finite() && mt > 0.0 && action[0].abs() > mt + 1e-12 {
                errs.push(format!(
                    "envelope_torque_exceeds_max_nm:{}>{mt}",
                    action[0].abs()
                ));
            }
        }
        errs
    }
}

/// Per-joint clip: never widen. Used as a PFL *screen*, not measured ISO 15066.
pub fn per_joint_clip(action: &[f64], limits: &[f64]) -> Vec<f64> {
    if limits.is_empty() {
        return action.to_vec();
    }
    action
        .iter()
        .enumerate()
        .map(|(i, a)| {
            let lim = limits[i.min(limits.len() - 1)].abs();
            a.clamp(-lim, lim)
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn allow_cert_would_still_fail_envelope() {
        let env = DriverEnvelopePack::scalar(0.5, true);
        let v = env.check_action(&[0.9]);
        assert!(v
            .iter()
            .any(|s| s.contains("envelope_action_exceeds_max_abs")));
    }

    #[test]
    fn clip_never_widens() {
        let out = per_joint_clip(&[2.0, -3.0], &[1.0, 1.0]);
        assert_eq!(out, vec![1.0, -1.0]);
    }
}
