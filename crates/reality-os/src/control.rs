//! Controller traits. Component clamp is named as a clamp, not a QP.

use crate::trajectory::TrajectoryReference;
use realityos_kernel::{BeliefState, KernelResult};

#[derive(Debug, Clone, PartialEq)]
pub struct ControlProposal {
    pub action: Vec<f64>,
    pub feasible: bool,
    pub residual: f64,
    pub solver: &'static str,
}

pub trait Controller {
    fn track(
        &mut self,
        reference: &TrajectoryReference,
        belief: &BeliefState,
    ) -> KernelResult<ControlProposal>;
}

/// Named clamp. Not whole-body control.
pub struct ComponentClamp {
    limits: Vec<f64>,
}

impl ComponentClamp {
    pub fn new(limits: Vec<f64>) -> Self {
        Self { limits }
    }
}

impl Controller for ComponentClamp {
    fn track(
        &mut self,
        reference: &TrajectoryReference,
        belief: &BeliefState,
    ) -> KernelResult<ControlProposal> {
        if !belief.is_usable_for_control() {
            return Ok(ControlProposal {
                action: reference.fallback.clone(),
                feasible: false,
                residual: f64::INFINITY,
                solver: "component_clamp",
            });
        }
        let raw = reference.samples.first().cloned().unwrap_or_default();
        let action: Vec<f64> = raw
            .iter()
            .enumerate()
            .map(|(i, a)| {
                let lim = self.limits.get(i).copied().unwrap_or(*a).abs();
                a.clamp(-lim, lim)
            })
            .collect();
        Ok(ControlProposal {
            residual: raw
                .iter()
                .zip(action.iter())
                .map(|(a, b)| (a - b).abs())
                .fold(0.0, f64::max),
            action,
            feasible: true,
            solver: "component_clamp",
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AssuranceAction {
    Continue,
    Hold,
    Zero,
}

pub fn runtime_assurance(proposal: &ControlProposal, deadline_missed: bool) -> AssuranceAction {
    if deadline_missed || !proposal.feasible {
        AssuranceAction::Hold
    } else if !proposal.action.iter().all(|x| x.is_finite()) {
        AssuranceAction::Zero
    } else {
        AssuranceAction::Continue
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::plan::PhysicalPlan;
    use realityos_kernel::SensorHealth;

    #[test]
    fn missed_deadline_holds() {
        let p = ControlProposal {
            action: vec![1.0],
            feasible: true,
            residual: 0.0,
            solver: "x",
        };
        assert_eq!(runtime_assurance(&p, true), AssuranceAction::Hold);
        let belief =
            BeliefState::new("m", 1.0, vec![0.0], vec![0.1], SensorHealth::Ok, vec![]).unwrap();
        let plan = PhysicalPlan::hold("actuator_envelope", 1);
        let r = TrajectoryReference::from_plan(&plan, 0.01).unwrap();
        let mut c = ComponentClamp::new(vec![1.0]);
        let out = c.track(&r, &belief).unwrap();
        assert_eq!(out.solver, "component_clamp");
    }
}
