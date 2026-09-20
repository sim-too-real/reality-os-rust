//! Restricted two-sided pinch HOLD using the same friction-cone / Coulomb primitives.

use realityos_physics::{coulomb_initiation_force_n, friction_cone_membership, ConeMembership};
use serde::{Deserialize, Serialize};

use crate::pair_friction::PairFriction;
use crate::physical_quantity::PhysicalFactProvenance;
use crate::provenance::Provenanced;
use crate::transform::{norm3, scale3};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum HoldFeasibility {
    Feasible,
    Infeasible,
    Unknown,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PinchHoldInput {
    pub mass_kg: Provenanced<f64>,
    pub gravity_m_s2: Provenanced<[f64; 3]>,
    pub finger_a_inward_normal: [f64; 3],
    pub finger_b_inward_normal: [f64; 3],
    pub finger_object_friction: PairFriction,
    pub gripper_force_bound_n: Provenanced<f64>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct HoldFeasibilityWitness {
    pub feasibility: HoldFeasibility,
    pub reason: Option<String>,
    pub required_tangent_n: Option<f64>,
    pub available_tangent_n: Option<f64>,
    pub cone_margin: Option<f64>,
    pub mass_provenance: PhysicalFactProvenance,
    pub friction_provenance: PhysicalFactProvenance,
    pub force_provenance: PhysicalFactProvenance,
}

/// Can two opposing finger contacts resist weight without slipping?
///
/// Not a full grasp-stability proof.
pub fn evaluate_pinch_hold(input: &PinchHoldInput) -> HoldFeasibilityWitness {
    let mass_p = PhysicalFactProvenance::from(input.mass_kg.provenance);
    let friction_p = input.finger_object_friction.sliding_provenance();
    let force_p = PhysicalFactProvenance::from(input.gripper_force_bound_n.provenance);
    let unknown = |reason: &str| HoldFeasibilityWitness {
        feasibility: HoldFeasibility::Unknown,
        reason: Some(reason.into()),
        required_tangent_n: None,
        available_tangent_n: None,
        cone_margin: None,
        mass_provenance: mass_p,
        friction_provenance: friction_p,
        force_provenance: force_p,
    };
    let Some(&mass) = input.mass_kg.known_value() else {
        return unknown("PARAMETER_MISSING:mass");
    };
    if mass <= 0.0 {
        return unknown("NONPOSITIVE_MASS");
    }
    let Some(&g) = input.gravity_m_s2.known_value() else {
        return unknown("PARAMETER_MISSING:gravity");
    };
    let Some(mu) = input.finger_object_friction.sliding_mu_known() else {
        return unknown("PARAMETER_MISSING:finger_friction");
    };
    let Some(&f_grip) = input.gripper_force_bound_n.known_value() else {
        return unknown("PARAMETER_MISSING:gripper_force");
    };
    if f_grip < 0.0 || !f_grip.is_finite() {
        return unknown("PARAMETER_WRONG:gripper_force");
    }
    let weight = mass * norm3(g);
    let required_each = weight / 2.0;
    let available_each = match coulomb_initiation_force_n(mu, f_grip) {
        Ok(v) => v,
        Err(_) => return unknown("PARAMETER_WRONG:finger_friction"),
    };
    let f_a = scale3(input.finger_a_inward_normal, f_grip);
    let (cone, margin) = match friction_cone_membership(f_a, input.finger_a_inward_normal, mu) {
        Ok(v) => v,
        Err(_) => return unknown("PARAMETER_WRONG:cone"),
    };
    let mut w = HoldFeasibilityWitness {
        feasibility: HoldFeasibility::Unknown,
        reason: None,
        required_tangent_n: Some(required_each),
        available_tangent_n: Some(available_each),
        cone_margin: Some(margin),
        mass_provenance: mass_p,
        friction_provenance: friction_p,
        force_provenance: force_p,
    };
    if cone == ConeMembership::Outside {
        w.feasibility = HoldFeasibility::Infeasible;
        w.reason = Some("TOOL_CONTACT_SLIP".into());
        return w;
    }
    if available_each + 1e-12 < required_each {
        w.feasibility = HoldFeasibility::Infeasible;
        w.reason = Some("HOLD_SLIP".into());
        return w;
    }
    w.feasibility = HoldFeasibility::Feasible;
    w.reason = Some("HOLD_FEASIBLE".into());
    w
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pair_friction::PairFriction;
    use crate::provenance::Provenanced;

    fn input(mass: f64, mu: f64, f: f64) -> PinchHoldInput {
        PinchHoldInput {
            mass_kg: Provenanced::declared(mass, "test.mass", 0.0),
            gravity_m_s2: Provenanced::declared([0.0, 0.0, -9.80665], "test.g", 0.0),
            finger_a_inward_normal: [1.0, 0.0, 0.0],
            finger_b_inward_normal: [-1.0, 0.0, 0.0],
            finger_object_friction: PairFriction::coulomb(
                "finger",
                "object",
                Provenanced::declared(mu, "test.mu", 0.0),
            ),
            gripper_force_bound_n: Provenanced::declared(f, "test.f", 0.0),
        }
    }

    #[test]
    fn lowering_friction_or_force_flips_hold() {
        let ok = evaluate_pinch_hold(&input(0.2, 0.8, 5.0));
        let low_mu = evaluate_pinch_hold(&input(0.2, 0.01, 5.0));
        let low_f = evaluate_pinch_hold(&input(0.2, 0.8, 0.05));
        assert_eq!(ok.feasibility, HoldFeasibility::Feasible);
        assert_eq!(low_mu.feasibility, HoldFeasibility::Infeasible);
        assert_eq!(low_f.feasibility, HoldFeasibility::Infeasible);
        assert_eq!(low_mu.reason.as_deref(), Some("HOLD_SLIP"));
        assert_ne!(low_mu.reason.as_deref(), Some("PUSH_FAILED"));
    }

    #[test]
    fn unknown_finger_friction_is_unknown() {
        let mut p = input(0.2, 0.8, 5.0);
        p.finger_object_friction = PairFriction::unknown("finger", "object", "missing");
        let w = evaluate_pinch_hold(&p);
        assert_eq!(w.feasibility, HoldFeasibility::Unknown);
        assert_eq!(w.friction_provenance, PhysicalFactProvenance::Unknown);
    }
}
