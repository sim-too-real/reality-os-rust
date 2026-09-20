//! Planar-push MOTION_INITIATION feasibility. Not displacement completion.

use std::collections::BTreeMap;

use realityos_physics::{
    coulomb_initiation_force_n, friction_cone_membership, max_force_along_direction,
    supported_normal_force_n, ConeMembership, PhysicsError,
};
use serde::{Deserialize, Serialize};

use crate::contact_jacobian::{contact_jacobian_witness, jacobian_3xn_columns};
use crate::effort::{any_link_com_known, chain_physical_effort};
use crate::embodiment::EmbodimentModel;
use crate::mechanics_regime::{
    support_is_horizontal, AssumptionState, PlanarPushAssumptions, RegimeApplicability,
};
use crate::pair_friction::PairFriction;
use crate::physical_quantity::{PhysicalEffort, PhysicalFactProvenance};
use crate::provenance::Provenanced;
use crate::skill::SkillRefuse;
use crate::transform::{cross3, norm3, normalize3, scale3, sub3};

fn dot3(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum EffectFeasibility {
    Feasible,
    Infeasible,
    Unknown,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum EffectClass {
    MotionInitiation,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum EffortBoundKind {
    GrossEffortBound,
    AvailableContactEffortBound,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum OffCenterClass {
    CenteredTranslation,
    TranslationDominated,
    RotationDominated,
    Mixed,
    ToolSlip,
    InsufficientEffort,
    Unknown,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PlanarPushInitiation {
    pub mass_kg: Provenanced<f64>,
    pub object_com_world: Provenanced<[f64; 3]>,
    pub gravity_m_s2: Provenanced<[f64; 3]>,
    pub support_normal: Provenanced<[f64; 3]>,
    pub object_support_friction: PairFriction,
    pub tool_object_friction: PairFriction,
    pub contact_point_world: Provenanced<[f64; 3]>,
    pub contact_normal_world: Provenanced<[f64; 3]>,
    pub push_direction_world: Provenanced<[f64; 3]>,
    pub joint_names: Vec<String>,
    pub translational_jacobian_3xn: Vec<Vec<f64>>,
    pub jacobian_residual: Option<f64>,
    pub joint_effort_abs: Vec<Provenanced<f64>>,
    pub link_com_known: bool,
    pub object_supported: bool,
    pub approximately_planar: bool,
    pub quasi_static: bool,
    pub single_intended_contact: bool,
    pub no_significant_impact: bool,
    pub object_characteristic_length_m: Option<f64>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct EffectFeasibilityWitness {
    pub effect_class: EffectClass,
    pub feasibility: EffectFeasibility,
    pub infeasible_reason: Option<String>,
    pub unknown_reason: Option<String>,
    pub regime: RegimeApplicability,
    pub regime_assumptions: PlanarPushAssumptions,
    pub contact_point: Option<[f64; 3]>,
    pub contact_normal: Option<[f64; 3]>,
    pub push_direction: Option<[f64; 3]>,
    pub object_mass_kg: Option<f64>,
    pub gravity_m_s2: Option<[f64; 3]>,
    pub support_friction_mu: Option<f64>,
    pub tool_object_friction_mu: Option<f64>,
    pub required_force_n: Option<f64>,
    pub required_wrench: Option<[f64; 6]>,
    pub available_lambda: Option<f64>,
    pub available_wrench: Option<[f64; 6]>,
    pub effort_bound_kind: EffortBoundKind,
    pub limiting_joint: Option<String>,
    pub jacobian_residual: Option<f64>,
    pub cone_margin: Option<f64>,
    pub cone_membership: Option<String>,
    pub off_center_class: OffCenterClass,
    pub claims_requested_displacement: bool,
    pub provenance: BTreeMap<String, PhysicalFactProvenance>,
    pub notes: Vec<String>,
}

impl EffectFeasibilityWitness {
    fn blank(assumptions: PlanarPushAssumptions) -> Self {
        Self {
            effect_class: EffectClass::MotionInitiation,
            feasibility: EffectFeasibility::Unknown,
            infeasible_reason: None,
            unknown_reason: None,
            regime: assumptions.applicability(),
            regime_assumptions: assumptions,
            contact_point: None,
            contact_normal: None,
            push_direction: None,
            object_mass_kg: None,
            gravity_m_s2: None,
            support_friction_mu: None,
            tool_object_friction_mu: None,
            required_force_n: None,
            required_wrench: None,
            available_lambda: None,
            available_wrench: None,
            effort_bound_kind: EffortBoundKind::GrossEffortBound,
            limiting_joint: None,
            jacobian_residual: None,
            cone_margin: None,
            cone_membership: None,
            off_center_class: OffCenterClass::Unknown,
            claims_requested_displacement: false,
            provenance: BTreeMap::new(),
            notes: Vec::new(),
        }
    }
}

fn flag(v: bool) -> AssumptionState {
    if v {
        AssumptionState::Satisfied
    } else {
        AssumptionState::Violated
    }
}

fn known_flag(v: bool) -> AssumptionState {
    if v {
        AssumptionState::Satisfied
    } else {
        AssumptionState::Unknown
    }
}

fn line_of_action_offset(contact: [f64; 3], direction: [f64; 3], com: [f64; 3]) -> Option<f64> {
    let d = normalize3(direction)?;
    let r = sub3(com, contact);
    let rej = sub3(r, scale3(d, dot3(r, d)));
    Some(norm3(rej))
}

fn classify_offset(offset: f64, size: Option<f64>) -> OffCenterClass {
    let rel = size.filter(|s| *s > 1e-9).map(|s| offset / s);
    if offset < 1e-3 || rel.is_some_and(|r| r < 0.05) {
        OffCenterClass::CenteredTranslation
    } else if rel.is_some_and(|r| r > 0.5) {
        OffCenterClass::RotationDominated
    } else if rel.is_some_and(|r| r < 0.15) {
        OffCenterClass::TranslationDominated
    } else {
        OffCenterClass::Mixed
    }
}

fn finish_unknown(
    mut w: EffectFeasibilityWitness,
    reason: impl Into<String>,
) -> EffectFeasibilityWitness {
    w.feasibility = EffectFeasibility::Unknown;
    w.unknown_reason = Some(reason.into());
    w.claims_requested_displacement = false;
    w
}

fn finish_infeasible(
    mut w: EffectFeasibilityWitness,
    reason: &str,
    off: OffCenterClass,
) -> EffectFeasibilityWitness {
    w.feasibility = EffectFeasibility::Infeasible;
    w.infeasible_reason = Some(reason.into());
    w.off_center_class = off;
    w.claims_requested_displacement = false;
    w
}

/// MOTION_INITIATION for a restricted planar push. Does not predict stroke completion.
pub fn evaluate_planar_push_initiation(input: &PlanarPushInitiation) -> EffectFeasibilityWitness {
    let mass_known = input.mass_kg.known_value().is_some();
    let friction_known = input.object_support_friction.sliding_mu_known().is_some()
        && input.tool_object_friction.sliding_mu_known().is_some();
    let contact_known = input.contact_point_world.known_value().is_some()
        && input.contact_normal_world.known_value().is_some()
        && input.push_direction_world.known_value().is_some();
    let support_known = input.support_normal.known_value().is_some()
        && input
            .gravity_m_s2
            .known_value()
            .and_then(|g| {
                input
                    .support_normal
                    .known_value()
                    .and_then(|n| support_is_horizontal(*g, *n))
            })
            .unwrap_or(false);

    let assumptions = PlanarPushAssumptions {
        object_supported: flag(input.object_supported),
        approximately_planar: flag(input.approximately_planar),
        quasi_static: flag(input.quasi_static),
        single_intended_contact: flag(input.single_intended_contact),
        support_known: if support_known {
            AssumptionState::Satisfied
        } else if input.support_normal.known_value().is_none()
            || input.gravity_m_s2.known_value().is_none()
        {
            AssumptionState::Unknown
        } else {
            AssumptionState::Violated
        },
        friction_known: known_flag(friction_known),
        mass_known: known_flag(mass_known),
        contact_geometry_known: known_flag(contact_known),
        no_significant_impact: flag(input.no_significant_impact),
    };

    let mut w = EffectFeasibilityWitness::blank(assumptions);
    w.effort_bound_kind = EffortBoundKind::GrossEffortBound;
    if input.link_com_known {
        w.notes
            .push("link_com_known_but_gravity_load_not_subtracted:GROSS_EFFORT_BOUND".into());
    } else {
        w.notes
            .push("self_load_unaccounted:GROSS_EFFORT_BOUND".into());
    }
    w.jacobian_residual = input.jacobian_residual;
    w.object_mass_kg = input.mass_kg.known_value().copied();
    w.gravity_m_s2 = input.gravity_m_s2.known_value().copied();
    w.support_friction_mu = input.object_support_friction.sliding_mu_known();
    w.tool_object_friction_mu = input.tool_object_friction.sliding_mu_known();
    w.contact_point = input.contact_point_world.known_value().copied();
    w.contact_normal = input.contact_normal_world.known_value().copied();
    w.push_direction = input
        .push_direction_world
        .known_value()
        .and_then(|d| normalize3(*d));
    w.provenance.insert(
        "mass".into(),
        PhysicalFactProvenance::from(input.mass_kg.provenance),
    );
    w.provenance.insert(
        "gravity".into(),
        PhysicalFactProvenance::from(input.gravity_m_s2.provenance),
    );
    w.provenance.insert(
        "support_friction".into(),
        input.object_support_friction.sliding_provenance(),
    );
    w.provenance.insert(
        "tool_object_friction".into(),
        input.tool_object_friction.sliding_provenance(),
    );
    w.provenance.insert(
        "contact_point".into(),
        PhysicalFactProvenance::from(input.contact_point_world.provenance),
    );
    w.provenance.insert(
        "contact_normal".into(),
        PhysicalFactProvenance::from(input.contact_normal_world.provenance),
    );
    w.provenance.insert(
        "push_direction".into(),
        PhysicalFactProvenance::from(input.push_direction_world.provenance),
    );

    match w.regime {
        RegimeApplicability::ModelNotApplicable => {
            return finish_unknown(w, "MODEL_NOT_APPLICABLE");
        }
        RegimeApplicability::Unknown => {
            if input.mass_kg.known_value().is_none() {
                return finish_unknown(w, "PARAMETER_MISSING:mass");
            }
            if input.object_support_friction.sliding_mu_known().is_none() {
                return finish_unknown(w, "PARAMETER_MISSING:support_friction");
            }
            if input.tool_object_friction.sliding_mu_known().is_none() {
                return finish_unknown(w, "PARAMETER_MISSING:tool_object_friction");
            }
            return finish_unknown(w, "REGIME_UNKNOWN");
        }
        RegimeApplicability::Applicable => {}
    }

    let Some(&mass) = input.mass_kg.known_value() else {
        return finish_unknown(w, "PARAMETER_MISSING:mass");
    };
    if mass <= 0.0 {
        return finish_unknown(w, "NONPOSITIVE_MASS");
    }
    let Some(&g) = input.gravity_m_s2.known_value() else {
        return finish_unknown(w, "PARAMETER_MISSING:gravity");
    };
    let Some(&n_support) = input.support_normal.known_value() else {
        return finish_unknown(w, "PARAMETER_MISSING:support_normal");
    };
    let Some(mu_s) = input.object_support_friction.sliding_mu_known() else {
        return finish_unknown(w, "PARAMETER_MISSING:support_friction");
    };
    let Some(mu_t) = input.tool_object_friction.sliding_mu_known() else {
        return finish_unknown(w, "PARAMETER_MISSING:tool_object_friction");
    };
    let Some(&contact) = input.contact_point_world.known_value() else {
        return finish_unknown(w, "PARAMETER_MISSING:contact_point");
    };
    let Some(&n_contact) = input.contact_normal_world.known_value() else {
        return finish_unknown(w, "PARAMETER_MISSING:contact_normal");
    };
    let Some(&d_raw) = input.push_direction_world.known_value() else {
        return finish_unknown(w, "PARAMETER_MISSING:push_direction");
    };
    let Some(d) = normalize3(d_raw) else {
        return finish_unknown(w, "PARAMETER_MISSING:push_direction");
    };

    let load = match supported_normal_force_n(mass, g, n_support) {
        Ok(v) => v,
        Err(_) => return finish_unknown(w, "PARAMETER_WRONG:normal_load"),
    };
    let f_req = match coulomb_initiation_force_n(mu_s, load) {
        Ok(v) => v,
        Err(_) => return finish_unknown(w, "PARAMETER_WRONG:support_friction"),
    };
    w.required_force_n = Some(f_req);
    let f_vec = [d[0] * f_req, d[1] * f_req, d[2] * f_req];
    let torque = input
        .object_com_world
        .known_value()
        .map(|com| cross3(sub3(contact, *com), f_vec))
        .unwrap_or([0.0, 0.0, 0.0]);
    w.required_wrench = Some([
        f_vec[0], f_vec[1], f_vec[2], torque[0], torque[1], torque[2],
    ]);

    let off = match input.object_com_world.known_value() {
        None => OffCenterClass::Unknown,
        Some(com) => match line_of_action_offset(contact, d, *com) {
            None => OffCenterClass::Unknown,
            Some(off) => classify_offset(off, input.object_characteristic_length_m),
        },
    };
    w.off_center_class = off;

    let (cone, margin) = match friction_cone_membership(f_vec, n_contact, mu_t) {
        Ok(v) => v,
        Err(_) => return finish_unknown(w, "PARAMETER_WRONG:tool_friction_cone"),
    };
    w.cone_margin = Some(margin);
    w.cone_membership = Some(
        match cone {
            ConeMembership::Inside => "INSIDE",
            ConeMembership::Outside => "OUTSIDE",
            ConeMembership::Unevaluable => "UNEVALUABLE",
        }
        .into(),
    );

    if input.joint_effort_abs.len() != input.joint_names.len()
        || input.translational_jacobian_3xn.first().map(|r| r.len())
            != Some(input.joint_names.len())
    {
        return finish_unknown(w, "JACOBIAN_MISMATCH");
    }
    let mut tau = Vec::with_capacity(input.joint_effort_abs.len());
    for e in &input.joint_effort_abs {
        match e.known_value().copied() {
            Some(v) if v.is_finite() && v >= 0.0 => tau.push(v),
            _ => return finish_unknown(w, "PARAMETER_MISSING:actuator_effort"),
        }
    }
    w.provenance.insert(
        "actuator_effort".into(),
        PhysicalFactProvenance::from(
            input
                .joint_effort_abs
                .first()
                .map(|e| e.provenance)
                .unwrap_or(crate::provenance::Provenance::Unknown),
        ),
    );

    let cols = jacobian_3xn_columns(&input.translational_jacobian_3xn);
    let bound = match max_force_along_direction(&cols, &tau, d) {
        Ok(b) => b,
        Err(PhysicsError::Singular(_)) => {
            return finish_unknown(w, "NEAR_SINGULAR_JACOBIAN");
        }
        Err(_) => return finish_unknown(w, "EFFORT_MAPPING_WRONG"),
    };
    w.available_lambda = Some(bound.lambda_abs_max);
    w.limiting_joint = input.joint_names.get(bound.limiting_index).cloned();
    let a_vec = [
        d[0] * bound.lambda_abs_max,
        d[1] * bound.lambda_abs_max,
        d[2] * bound.lambda_abs_max,
    ];
    w.available_wrench = Some([a_vec[0], a_vec[1], a_vec[2], 0.0, 0.0, 0.0]);

    if off != OffCenterClass::CenteredTranslation {
        if cone == ConeMembership::Outside {
            return finish_infeasible(w, "TOOL_CONTACT_SLIP", OffCenterClass::ToolSlip);
        }
        if bound.lambda_abs_max + 1e-12 < f_req {
            return finish_infeasible(
                w,
                "INSUFFICIENT_ACTUATOR_EFFORT",
                OffCenterClass::InsufficientEffort,
            );
        }
        return finish_unknown(w, "OFF_CENTER_NOT_CENTERED_TRANSLATION");
    }

    if cone == ConeMembership::Outside {
        return finish_infeasible(w, "TOOL_CONTACT_SLIP", OffCenterClass::ToolSlip);
    }
    if bound.lambda_abs_max + 1e-12 < f_req {
        return finish_infeasible(
            w,
            "INSUFFICIENT_ACTUATOR_EFFORT",
            OffCenterClass::InsufficientEffort,
        );
    }

    w.feasibility = EffectFeasibility::Feasible;
    w.notes
        .push("MOTION_INITIATION_FEASIBLE_is_not_displacement_completion".into());
    w
}

/// Drive the shipped FK / contact Jacobian / effort map, then evaluate initiation.
pub fn evaluate_planar_push_at_model(
    model: &EmbodimentModel,
    ee: &str,
    q: &[f64],
    contact_in_ee: [f64; 3],
    mut params: PlanarPushInitiation,
) -> Result<EffectFeasibilityWitness, SkillRefuse> {
    let chain = model.ee_joint_chain(ee).ok_or(SkillRefuse::Unsupported)?;
    let jac = contact_jacobian_witness(model, &chain, ee, q, contact_in_ee, 1e-6)?;
    params.joint_names = jac.joint_names.clone();
    params.translational_jacobian_3xn = jac.analytic_3xn.clone();
    params.jacobian_residual = Some(jac.residual);
    params.contact_point_world = Provenanced::declared(jac.contact_point_world, "fk.contact", 0.0);
    params.link_com_known = any_link_com_known(model);
    match chain_physical_effort(model, &params.joint_names) {
        Ok(tau) => {
            params.joint_effort_abs = tau
                .into_iter()
                .map(|v| Provenanced::declared(v, "joint.effort", 0.0))
                .collect();
        }
        Err(PhysicalEffort::Unknown { reason }) => {
            params.joint_effort_abs = params
                .joint_names
                .iter()
                .map(|_| Provenanced::unknown(reason.clone(), 0.0))
                .collect();
        }
        Err(_) => {
            params.joint_effort_abs = params
                .joint_names
                .iter()
                .map(|_| Provenanced::unknown("effort", 0.0))
                .collect();
        }
    }
    Ok(evaluate_planar_push_initiation(&params))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapter::synth_planar_two_link;
    use crate::pair_friction::PairFriction;
    use crate::provenance::Provenanced;

    fn identity_j() -> Vec<Vec<f64>> {
        vec![
            vec![1.0, 0.0, 0.0],
            vec![0.0, 1.0, 0.0],
            vec![0.0, 0.0, 1.0],
        ]
    }

    fn declared_tau(v: f64) -> Provenanced<f64> {
        Provenanced::declared(v, "test.effort", 0.0)
    }

    fn centered_input(mass: f64, mu_s: f64, mu_t: f64, tau: f64) -> PlanarPushInitiation {
        let contact = [0.2, 0.0, 0.03];
        let com = [0.2, 0.0, 0.03];
        PlanarPushInitiation {
            mass_kg: Provenanced::declared(mass, "test.mass", 0.0),
            object_com_world: Provenanced::declared(com, "test.com", 0.0),
            gravity_m_s2: Provenanced::declared([0.0, 0.0, -9.80665], "test.g", 0.0),
            support_normal: Provenanced::declared([0.0, 0.0, 1.0], "test.n", 0.0),
            object_support_friction: PairFriction::coulomb(
                "object",
                "support",
                Provenanced::declared(mu_s, "test.mu_s", 0.0),
            ),
            tool_object_friction: PairFriction::coulomb(
                "tool",
                "object",
                Provenanced::declared(mu_t, "test.mu_t", 0.0),
            ),
            contact_point_world: Provenanced::declared(contact, "test.contact", 0.0),
            contact_normal_world: Provenanced::declared([1.0, 0.0, 0.0], "test.cn", 0.0),
            push_direction_world: Provenanced::declared([1.0, 0.0, 0.0], "test.d", 0.0),
            joint_names: vec!["j0".into(), "j1".into(), "j2".into()],
            translational_jacobian_3xn: identity_j(),
            jacobian_residual: Some(0.0),
            joint_effort_abs: vec![declared_tau(tau), declared_tau(tau), declared_tau(tau)],
            link_com_known: false,
            object_supported: true,
            approximately_planar: true,
            quasi_static: true,
            single_intended_contact: true,
            no_significant_impact: true,
            object_characteristic_length_m: Some(0.06),
        }
    }

    #[test]
    fn unknown_mass_is_unknown_not_a_default() {
        let mut p = centered_input(0.2, 0.3, 0.8, 20.0);
        p.mass_kg = Provenanced::unknown("mass", 0.0);
        let w = evaluate_planar_push_initiation(&p);
        assert_eq!(w.feasibility, EffectFeasibility::Unknown);
        assert_eq!(w.effect_class, EffectClass::MotionInitiation);
        assert!(!w.claims_requested_displacement);
        assert_eq!(w.unknown_reason.as_deref(), Some("PARAMETER_MISSING:mass"));
        assert_eq!(
            w.provenance.get("mass"),
            Some(&PhysicalFactProvenance::Unknown)
        );
    }

    #[test]
    fn zero_mass_is_not_a_one_kg_default() {
        let w = evaluate_planar_push_initiation(&centered_input(0.0, 0.5, 0.8, 20.0));
        assert_eq!(w.feasibility, EffectFeasibility::Unknown);
        assert_eq!(w.unknown_reason.as_deref(), Some("NONPOSITIVE_MASS"));
        assert_ne!(w.required_force_n, Some(0.5 * 9.80665));
    }

    #[test]
    fn unknown_friction_stays_unknown() {
        let mut p = centered_input(0.2, 0.3, 0.8, 20.0);
        p.object_support_friction = PairFriction::unknown("object", "support", "missing");
        let w = evaluate_planar_push_initiation(&p);
        assert_eq!(w.feasibility, EffectFeasibility::Unknown);
        assert!(w
            .unknown_reason
            .as_deref()
            .unwrap()
            .contains("support_friction"));
    }

    #[test]
    fn zero_vs_high_support_friction_reverses_initiation() {
        let low = evaluate_planar_push_initiation(&centered_input(0.5, 0.0, 1.0, 5.0));
        let high = evaluate_planar_push_initiation(&centered_input(0.5, 8.0, 1.0, 5.0));
        assert_eq!(low.feasibility, EffectFeasibility::Feasible);
        assert_eq!(high.feasibility, EffectFeasibility::Infeasible);
        assert_eq!(
            high.infeasible_reason.as_deref(),
            Some("INSUFFICIENT_ACTUATOR_EFFORT")
        );
        assert_eq!(high.off_center_class, OffCenterClass::InsufficientEffort);
        assert!(low.required_force_n.unwrap() < high.required_force_n.unwrap());
        assert!(!low.claims_requested_displacement);
    }

    #[test]
    fn insufficient_effort_is_not_slip() {
        let w = evaluate_planar_push_initiation(&centered_input(2.0, 1.0, 1.0, 0.05));
        assert_eq!(w.feasibility, EffectFeasibility::Infeasible);
        assert_eq!(
            w.infeasible_reason.as_deref(),
            Some("INSUFFICIENT_ACTUATOR_EFFORT")
        );
        assert_ne!(w.infeasible_reason.as_deref(), Some("TOOL_CONTACT_SLIP"));
        assert_eq!(w.cone_membership.as_deref(), Some("INSIDE"));
    }

    #[test]
    fn unknown_actuator_effort_is_unknown() {
        let mut p = centered_input(0.2, 0.2, 0.8, 20.0);
        p.joint_effort_abs = vec![
            Provenanced::unknown("effort", 0.0),
            declared_tau(20.0),
            declared_tau(20.0),
        ];
        let w = evaluate_planar_push_initiation(&p);
        assert_eq!(w.feasibility, EffectFeasibility::Unknown);
        assert_eq!(
            w.unknown_reason.as_deref(),
            Some("PARAMETER_MISSING:actuator_effort")
        );
    }

    #[test]
    fn force_outside_tool_cone_is_slip_not_effort_ok() {
        let mut p = centered_input(0.2, 0.2, 0.05, 50.0);
        p.push_direction_world = Provenanced::declared([0.2, 1.0, 0.0], "test.d", 0.0);
        p.contact_normal_world = Provenanced::declared([1.0, 0.0, 0.0], "test.cn", 0.0);
        let w = evaluate_planar_push_initiation(&p);
        assert_eq!(w.feasibility, EffectFeasibility::Infeasible);
        assert_eq!(w.infeasible_reason.as_deref(), Some("TOOL_CONTACT_SLIP"));
        assert_eq!(w.cone_membership.as_deref(), Some("OUTSIDE"));
        assert!(w.available_lambda.unwrap() > w.required_force_n.unwrap());
    }

    #[test]
    fn centered_push_is_motion_initiation_not_stroke() {
        let w = evaluate_planar_push_initiation(&centered_input(0.1, 0.2, 0.8, 20.0));
        assert_eq!(w.feasibility, EffectFeasibility::Feasible);
        assert_eq!(w.effect_class, EffectClass::MotionInitiation);
        assert!(!w.claims_requested_displacement);
        assert_eq!(w.off_center_class, OffCenterClass::CenteredTranslation);
        assert_eq!(w.effort_bound_kind, EffortBoundKind::GrossEffortBound);
        assert!(w.notes.iter().any(|n| n.contains("GROSS_EFFORT_BOUND")));
        assert_eq!(w.regime, RegimeApplicability::Applicable);
    }

    #[test]
    fn off_center_is_not_treated_as_centered_translation() {
        let mut p = centered_input(0.1, 0.2, 0.8, 20.0);
        p.contact_point_world = Provenanced::declared([0.2, 0.04, 0.03], "test.contact", 0.0);
        p.object_com_world = Provenanced::declared([0.2, 0.0, 0.03], "test.com", 0.0);
        let w = evaluate_planar_push_initiation(&p);
        assert_ne!(w.off_center_class, OffCenterClass::CenteredTranslation);
        assert_eq!(w.feasibility, EffectFeasibility::Unknown);
        assert_eq!(
            w.unknown_reason.as_deref(),
            Some("OFF_CENTER_NOT_CENTERED_TRANSLATION")
        );
    }

    #[test]
    fn near_singular_jacobian_is_unknown_not_huge_force() {
        let mut p = centered_input(0.1, 0.2, 0.8, 20.0);
        p.translational_jacobian_3xn = vec![
            vec![0.0, 0.0, 0.0],
            vec![1.0, 0.5, 0.2],
            vec![0.0, 0.0, 0.0],
        ];
        let w = evaluate_planar_push_initiation(&p);
        assert_eq!(w.feasibility, EffectFeasibility::Unknown);
        assert_eq!(w.unknown_reason.as_deref(), Some("NEAR_SINGULAR_JACOBIAN"));
        assert!(w.available_lambda.is_none());
    }

    #[test]
    fn high_impact_regime_is_not_applicable() {
        let mut p = centered_input(0.1, 0.2, 0.8, 20.0);
        p.no_significant_impact = false;
        let w = evaluate_planar_push_initiation(&p);
        assert_eq!(w.regime, RegimeApplicability::ModelNotApplicable);
        assert_eq!(w.feasibility, EffectFeasibility::Unknown);
        assert_eq!(w.unknown_reason.as_deref(), Some("MODEL_NOT_APPLICABLE"));
    }

    #[test]
    fn model_path_uses_shipped_jacobian_residual() {
        let mut m = synth_planar_two_link();
        m.joints[0].effort_max = Provenanced::declared(8.0, "test", 0.0);
        m.joints[1].effort_max = Provenanced::declared(8.0, "test", 0.0);
        let params = centered_input(0.05, 0.1, 1.0, 8.0);
        let w =
            evaluate_planar_push_at_model(&m, "ee", &[0.3, -0.4], [0.0, 0.0, 0.0], params).unwrap();
        assert!(w.jacobian_residual.is_some());
        assert!(w.jacobian_residual.unwrap() < 1e-5);
        assert_eq!(w.effect_class, EffectClass::MotionInitiation);
    }

    #[test]
    fn witness_reconstructs_feasible_infeasible_unknown() {
        let f = evaluate_planar_push_initiation(&centered_input(0.05, 0.1, 1.0, 20.0));
        let i = evaluate_planar_push_initiation(&centered_input(3.0, 2.0, 1.0, 0.2));
        let mut u_in = centered_input(0.2, 0.2, 0.8, 20.0);
        u_in.mass_kg = Provenanced::unknown("mass", 0.0);
        let u = evaluate_planar_push_initiation(&u_in);
        assert_eq!(f.feasibility, EffectFeasibility::Feasible);
        assert_eq!(i.feasibility, EffectFeasibility::Infeasible);
        assert_eq!(u.feasibility, EffectFeasibility::Unknown);
        assert!(f.required_force_n.is_some() && f.available_lambda.is_some());
        assert!(f.support_friction_mu.is_some() && f.tool_object_friction_mu.is_some());
        assert_eq!(f.effect_class, EffectClass::MotionInitiation);
        assert!(!f.claims_requested_displacement);
        assert_eq!(f.effort_bound_kind, EffortBoundKind::GrossEffortBound);
        assert_ne!(i.infeasible_reason.as_deref(), Some("TOOL_CONTACT_SLIP"));
    }
}
