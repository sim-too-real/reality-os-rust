//! Frame/time/contact-aware reference. Control tracks only this type.

use crate::plan::PhysicalPlan;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TrajectoryReference {
    pub frame_id: String,
    pub units: String,
    pub samples: Vec<Vec<f64>>,
    pub dt_s: f64,
    pub completion: String,
    pub fallback: Vec<f64>,
}

impl TrajectoryReference {
    pub fn from_plan(plan: &PhysicalPlan, dt_s: f64) -> Option<Self> {
        if plan.action.is_empty() || !plan.finite() || !dt_s.is_finite() || dt_s <= 0.0 {
            return None;
        }
        Some(Self {
            frame_id: if plan.frame_id.is_empty() {
                "joint".into()
            } else {
                plan.frame_id.clone()
            },
            units: if plan.units.is_empty() {
                "effort".into()
            } else {
                plan.units.clone()
            },
            samples: vec![plan.action.clone()],
            dt_s,
            completion: plan.completion.clone(),
            fallback: vec![0.0; plan.action.len()],
        })
    }

    pub fn contains(&self, action: &[f64]) -> bool {
        self.samples.iter().any(|s| {
            s.len() == action.len() && s.iter().zip(action).all(|(a, b)| (a - b).abs() < 1e-9)
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_plan_is_not_a_reference() {
        assert!(TrajectoryReference::from_plan(&PhysicalPlan::new("x", vec![]), 0.01).is_none());
    }
}
