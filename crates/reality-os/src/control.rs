//! Controller traits. Component clamp is named as a clamp, not a QP.
//! Gravity-comp WBC uses native RNEA. It is not a hierarchical QP.

use crate::trajectory::TrajectoryReference;
use realityos_kernel::{BeliefState, KernelResult};
use realityos_physics::{default_backend, RigidBodyBackend, SerialModel};

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

/// Gravity compensation + clamp. Stance is a support-polygon screen, not balance control.
pub struct GravityCompWbc {
    model: SerialModel,
    limits: Vec<f64>,
    gravity: [f64; 3],
    support_half_m: f64,
}

impl GravityCompWbc {
    pub fn new(model: SerialModel, limits: Vec<f64>, gravity: [f64; 3]) -> Self {
        Self {
            model,
            limits,
            gravity,
            support_half_m: 0.15,
        }
    }

    pub fn with_support_half_m(mut self, half_m: f64) -> Self {
        self.support_half_m = half_m.abs();
        self
    }

    fn stance_ok(&self, q: &[f64]) -> bool {
        let Ok(frames) = default_backend().fk(&self.model, q) else {
            return false;
        };
        let Some(ee) = frames.last() else {
            return true;
        };
        let x = ee[0][3];
        let y = ee[1][3];
        x.abs() <= self.support_half_m && y.abs() <= self.support_half_m
    }
}

impl Controller for GravityCompWbc {
    fn track(
        &mut self,
        reference: &TrajectoryReference,
        belief: &BeliefState,
    ) -> KernelResult<ControlProposal> {
        if !belief.is_usable_for_control() || belief.mean().len() != self.model.joints.len() {
            return Ok(ControlProposal {
                action: reference.fallback.clone(),
                feasible: false,
                residual: f64::INFINITY,
                solver: "gravity_comp_wbc",
            });
        }
        let q = belief.mean();
        if !self.stance_ok(q) {
            return Ok(ControlProposal {
                action: vec![0.0; q.len()],
                feasible: false,
                residual: f64::INFINITY,
                solver: "gravity_comp_wbc",
            });
        }
        let dq = vec![0.0; q.len()];
        let ddq = vec![0.0; q.len()];
        let grav = match default_backend().rnea(&self.model, q, &dq, &ddq, self.gravity) {
            Ok(t) => t,
            Err(_) => {
                return Ok(ControlProposal {
                    action: reference.fallback.clone(),
                    feasible: false,
                    residual: f64::INFINITY,
                    solver: "gravity_comp_wbc",
                });
            }
        };
        let raw = reference.samples.first().cloned().unwrap_or_else(|| grav.clone());
        let action: Vec<f64> = grav
            .iter()
            .enumerate()
            .map(|(i, g)| {
                let add: f64 = raw.get(i).copied().unwrap_or(0.0);
                let lim = self.limits.get(i).copied().unwrap_or(add.abs()).abs();
                (g + add).clamp(-lim, lim)
            })
            .collect();
        Ok(ControlProposal {
            residual: grav
                .iter()
                .zip(action.iter())
                .map(|(a, b)| (a - b).abs())
                .fold(0.0, f64::max),
            feasible: action.iter().all(|x| x.is_finite()),
            action,
            solver: "gravity_comp_wbc",
        })
    }
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

    #[test]
    fn gravity_comp_wbc_is_not_a_qp() {
        let model = realityos_physics::SerialModel {
            joints: vec![realityos_physics::SerialJoint {
                kind: realityos_physics::JointKind::Revolute,
                axis: [0.0, 0.0, 1.0],
                origin: [0.0, 0.0, 0.0],
                mass: 1.0,
                com: [0.05, 0.0, 0.0],
                inertia_diag: [0.0, 0.0, 0.0],
            }],
        };
        let mut wbc = GravityCompWbc::new(model, vec![20.0], [0.0, 0.0, -9.81]);
        let belief =
            BeliefState::new("m", 1.0, vec![0.0], vec![0.1], SensorHealth::Ok, vec![]).unwrap();
        let plan = PhysicalPlan::hold("actuator_envelope", 1);
        let r = TrajectoryReference::from_plan(&plan, 0.01).unwrap();
        let out = wbc.track(&r, &belief).unwrap();
        assert_eq!(out.solver, "gravity_comp_wbc");
        assert!(out.feasible);
        assert!(out.action[0].is_finite());
    }
}
