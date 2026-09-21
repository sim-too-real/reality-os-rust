//! Planar-push MOTION_INITIATION feasibility. Not displacement completion.

use std::collections::BTreeMap;

use realityos_physics::{
    available_lambda_interval, contact_mode_from_pusher, coulomb_initiation_force_n,
    friction_cone_membership, lambda_to_limit_surface, max_force_along_direction,
    motion_compatibility, project_to_plane, supported_normal_force_n, twist_from_contact_force,
    ConeMembership, ContactMode, MotionCompatibility, PhysicsError, PlanarTwist,
    PressureDistribution, RotationSign, SupportFrictionModel,
};
use serde::{Deserialize, Serialize};

use crate::contact_jacobian::{contact_jacobian_witness, jacobian_3xn_columns};
use crate::effort::{any_link_com_known, chain_physical_effort, chain_physical_effort_signed};
use crate::embodiment::EmbodimentModel;
use crate::mechanics_regime::{
    support_is_horizontal, AssumptionState, PlanarPushAssumptions, RegimeApplicability,
};
use crate::pair_friction::PairFriction;
use crate::physical_quantity::{PhysicalEffort, PhysicalFactProvenance};
use crate::provenance::Provenanced;
use crate::self_load::{gravity_self_load, self_load_provenanced};
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
    PlanarTwistDirection,
    SustainedEffect,
}

/// Four capability levels. A lower level never implies a higher one.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub struct PhysicalEffectLevels {
    pub contact_geometrically_feasible: EffectFeasibility,
    pub motion_initiation: EffectFeasibility,
    pub instantaneous_motion: EffectFeasibility,
    pub sustained_effect: EffectFeasibility,
}

impl PhysicalEffectLevels {
    fn unknown() -> Self {
        Self {
            contact_geometrically_feasible: EffectFeasibility::Unknown,
            motion_initiation: EffectFeasibility::Unknown,
            instantaneous_motion: EffectFeasibility::Unknown,
            sustained_effect: EffectFeasibility::Unknown,
        }
    }

    /// SUSTAINED_EFFECT = FEASIBLE is only honest when every lower level is FEASIBLE.
    pub fn all_prerequisites_of_sustained_are_feasible(self) -> bool {
        self.contact_geometrically_feasible == EffectFeasibility::Feasible
            && self.motion_initiation == EffectFeasibility::Feasible
            && self.instantaneous_motion == EffectFeasibility::Feasible
    }
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
    /// Geometric diagnostic only. Not a motion prediction.
    GeometricNearCenter,
    /// Geometric diagnostic only. Not a motion prediction.
    GeometricFarOffset,
    Mixed,
    ToolSlip,
    InsufficientEffort,
    Unknown,
    /// Legacy names kept so old JSON still deserializes. Not emitted.
    TranslationDominated,
    RotationDominated,
}

fn unknown_vec3() -> Provenanced<[f64; 3]> {
    Provenanced::unknown("pusher_velocity", 0.0)
}

fn unknown_yaw() -> Provenanced<f64> {
    Provenanced::unknown("object_yaw", 0.0)
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
    /// Contact-force direction for initiation. Distinct from pusher velocity.
    #[serde(default = "unknown_vec3")]
    pub contact_force_direction_world: Provenanced<[f64; 3]>,
    /// Commanded pusher velocity at the contact. Not equated with force direction.
    #[serde(default = "unknown_vec3")]
    pub pusher_velocity_world: Provenanced<[f64; 3]>,
    pub joint_names: Vec<String>,
    pub translational_jacobian_3xn: Vec<Vec<f64>>,
    pub jacobian_residual: Option<f64>,
    pub joint_effort_abs: Vec<Provenanced<f64>>,
    #[serde(default)]
    pub joint_effort_min: Vec<Provenanced<f64>>,
    #[serde(default)]
    pub joint_effort_max: Vec<Provenanced<f64>>,
    #[serde(default)]
    pub self_load_torque_nm: Vec<Provenanced<f64>>,
    pub link_com_known: bool,
    pub object_supported: bool,
    pub approximately_planar: bool,
    pub quasi_static: bool,
    pub single_intended_contact: bool,
    pub no_significant_impact: bool,
    pub object_characteristic_length_m: Option<f64>,
    #[serde(default)]
    pub support_friction_model: SupportFrictionModel,
    #[serde(default = "unknown_yaw")]
    pub object_yaw_rad: Provenanced<f64>,
    #[serde(default)]
    pub stale_object_evidence: bool,
    #[serde(default)]
    pub intended_contact_lost: bool,
    #[serde(default = "default_authority_ok")]
    pub authority_ok: bool,
}

fn default_authority_ok() -> bool {
    true
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
    #[serde(default = "PhysicalEffectLevels::unknown")]
    pub physical_levels: PhysicalEffectLevels,
    #[serde(default)]
    pub contact_mode: Option<ContactMode>,
    #[serde(default)]
    pub rotation_sign: Option<RotationSign>,
    #[serde(default)]
    pub planar_twist: Option<PlanarTwist>,
    #[serde(default)]
    pub motion_compatibility: Option<MotionCompatibility>,
    #[serde(default)]
    pub self_load_torque: Option<Vec<f64>>,
    #[serde(default)]
    pub signed_lambda_interval: Option<[f64; 2]>,
    #[serde(default)]
    pub support_model_kind: Option<String>,
    #[serde(default)]
    pub pressure_model_kind: Option<String>,
    #[serde(default)]
    pub pusher_velocity: Option<[f64; 3]>,
    #[serde(default)]
    pub contact_force_direction: Option<[f64; 3]>,
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
            physical_levels: PhysicalEffectLevels::unknown(),
            contact_mode: None,
            rotation_sign: None,
            planar_twist: None,
            motion_compatibility: None,
            self_load_torque: None,
            signed_lambda_interval: None,
            support_model_kind: None,
            pressure_model_kind: None,
            pusher_velocity: None,
            contact_force_direction: None,
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
        OffCenterClass::GeometricFarOffset
    } else if rel.is_some_and(|r| r < 0.15) {
        OffCenterClass::GeometricNearCenter
    } else {
        OffCenterClass::Mixed
    }
}

fn force_direction_of(input: &PlanarPushInitiation) -> Option<[f64; 3]> {
    input
        .contact_force_direction_world
        .known_value()
        .copied()
        .or_else(|| input.push_direction_world.known_value().copied())
        .and_then(normalize3)
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
        && (input.push_direction_world.known_value().is_some()
            || input.contact_force_direction_world.known_value().is_some());
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
    w.support_model_kind = Some(input.support_friction_model.kind_name().into());
    w.pressure_model_kind = Some(input.support_friction_model.pressure_name().into());
    w.pusher_velocity = input.pusher_velocity_world.known_value().copied();
    w.contact_force_direction = force_direction_of(input);
    if input
        .self_load_torque_nm
        .iter()
        .any(|t| t.known_value().is_some())
    {
        w.notes.push("self_load_present".into());
    } else if input.link_com_known {
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
    let Some(d) = force_direction_of(input) else {
        return finish_unknown(w, "PARAMETER_MISSING:push_direction");
    };
    w.contact_force_direction = Some(d);
    w.push_direction = Some(d);

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

    if input.joint_names.len()
        != input
            .translational_jacobian_3xn
            .first()
            .map(|r| r.len())
            .unwrap_or(0)
    {
        return finish_unknown(w, "JACOBIAN_MISMATCH");
    }
    let n = input.joint_names.len();
    let cols = jacobian_3xn_columns(&input.translational_jacobian_3xn);

    let self_load = known_f64_vec(&input.self_load_torque_nm, n);
    let tau_min = known_f64_vec(&input.joint_effort_min, n);
    let tau_max = known_f64_vec(&input.joint_effort_max, n);
    let mut tau_abs = Vec::new();
    if input.joint_effort_abs.len() == n {
        for e in &input.joint_effort_abs {
            match e.known_value().copied() {
                Some(v) if v.is_finite() && v >= 0.0 => tau_abs.push(v),
                _ => {
                    tau_abs.clear();
                    break;
                }
            }
        }
    }
    w.provenance.insert(
        "actuator_effort".into(),
        PhysicalFactProvenance::from(
            input
                .joint_effort_max
                .first()
                .or(input.joint_effort_abs.first())
                .map(|e| e.provenance)
                .unwrap_or(crate::provenance::Provenance::Unknown),
        ),
    );

    let available = if let (Some(ts), Some(tmin), Some(tmax)) = (self_load, tau_min, tau_max) {
        w.effort_bound_kind = EffortBoundKind::AvailableContactEffortBound;
        w.self_load_torque = Some(ts.clone());
        w.notes.retain(|n| !n.contains("GROSS_EFFORT_BOUND"));
        match available_lambda_interval(&cols, d, &ts, &tmin, &tmax) {
            Ok(b) => Some(b),
            Err(PhysicsError::Singular(_)) => {
                return finish_unknown(w, "NEAR_SINGULAR_JACOBIAN");
            }
            Err(PhysicsError::Unevaluable("self_load_exceeds_effort")) => {
                return finish_infeasible(
                    w,
                    "SELF_LOAD_EXCEEDS_EFFORT",
                    OffCenterClass::InsufficientEffort,
                );
            }
            Err(PhysicsError::Unevaluable("lambda_empty")) => {
                return finish_infeasible(
                    w,
                    "INSUFFICIENT_ACTUATOR_EFFORT",
                    OffCenterClass::InsufficientEffort,
                );
            }
            Err(_) => return finish_unknown(w, "EFFORT_MAPPING_WRONG"),
        }
    } else {
        w.effort_bound_kind = EffortBoundKind::GrossEffortBound;
        None
    };

    let (lambda_max, limiting_index) = if let Some(b) = available.as_ref() {
        w.signed_lambda_interval = Some([b.lambda_min, b.lambda_max]);
        (b.lambda_max, b.limiting_index)
    } else {
        if tau_abs.len() != n {
            return finish_unknown(w, "PARAMETER_MISSING:actuator_effort");
        }
        match max_force_along_direction(&cols, &tau_abs, d) {
            Ok(b) => (b.lambda_abs_max, b.limiting_index),
            Err(PhysicsError::Singular(_)) => {
                return finish_unknown(w, "NEAR_SINGULAR_JACOBIAN");
            }
            Err(_) => return finish_unknown(w, "EFFORT_MAPPING_WRONG"),
        }
    };
    w.available_lambda = Some(lambda_max);
    w.limiting_joint = input.joint_names.get(limiting_index).cloned();
    let a_vec = [d[0] * lambda_max, d[1] * lambda_max, d[2] * lambda_max];
    w.available_wrench = Some([a_vec[0], a_vec[1], a_vec[2], 0.0, 0.0, 0.0]);

    let ls_req = match input.support_friction_model {
        SupportFrictionModel::Ellipsoidal {
            f_max,
            tau_max,
            pressure: PressureDistribution::DeclaredUniform,
        } => {
            if let (Some(com), Some(&yaw), Some(&n_s)) = (
                input.object_com_world.known_value(),
                input.object_yaw_rad.known_value(),
                input.support_normal.known_value(),
            ) {
                planar_lambda_to_ls(d, contact, *com, yaw, n_s, f_max, tau_max)
            } else {
                None
            }
        }
        _ => None,
    };
    let required = ls_req.unwrap_or(f_req);
    if ls_req.is_some() {
        w.required_force_n = Some(required);
    }

    if cone == ConeMembership::Outside {
        let mut out = finish_infeasible(w, "TOOL_CONTACT_SLIP", OffCenterClass::ToolSlip);
        fill_twist_fields(&mut out, input, d, contact);
        stamp_levels(&mut out);
        return out;
    }

    let centered = off == OffCenterClass::CenteredTranslation;
    if lambda_max + 1e-12 < required && (centered || ls_req.is_some()) {
        let mut out = finish_infeasible(
            w,
            "INSUFFICIENT_ACTUATOR_EFFORT",
            OffCenterClass::InsufficientEffort,
        );
        fill_twist_fields(&mut out, input, d, contact);
        stamp_levels(&mut out);
        return out;
    }

    if w.effort_bound_kind != EffortBoundKind::AvailableContactEffortBound {
        let mut out = finish_unknown(w, "GROSS_EFFORT_BOUND_NOT_AVAILABLE_CONTACT_EFFORT");
        fill_twist_fields(&mut out, input, d, contact);
        stamp_levels(&mut out);
        return out;
    }

    if !centered && ls_req.is_none() {
        match input.support_friction_model {
            SupportFrictionModel::Unknown => {
                let mut out = finish_unknown(w, "OFF_CENTER_WITHOUT_SUPPORT_MODEL");
                fill_twist_fields(&mut out, input, d, contact);
                stamp_levels(&mut out);
                return out;
            }
            SupportFrictionModel::Ellipsoidal {
                pressure: PressureDistribution::Unknown,
                ..
            } => {
                let mut out = finish_unknown(w, "PRESSURE_MODEL_UNKNOWN");
                fill_twist_fields(&mut out, input, d, contact);
                stamp_levels(&mut out);
                return out;
            }
            _ => {
                let mut out = finish_unknown(w, "OFF_CENTER_WITHOUT_LIMIT_SURFACE");
                fill_twist_fields(&mut out, input, d, contact);
                stamp_levels(&mut out);
                return out;
            }
        }
    }

    if lambda_max + 1e-12 < required {
        let mut out = finish_infeasible(
            w,
            "INSUFFICIENT_ACTUATOR_EFFORT",
            OffCenterClass::InsufficientEffort,
        );
        fill_twist_fields(&mut out, input, d, contact);
        stamp_levels(&mut out);
        return out;
    }

    w.feasibility = EffectFeasibility::Feasible;
    w.notes
        .push("MOTION_INITIATION_FEASIBLE_is_not_displacement_completion".into());
    fill_twist_fields(&mut w, input, d, contact);
    stamp_levels(&mut w);
    w.physical_levels.motion_initiation = EffectFeasibility::Feasible;
    if w.planar_twist.is_some() && w.rotation_sign.is_some() {
        w.physical_levels.instantaneous_motion = EffectFeasibility::Unknown;
    }
    w
}

fn known_f64_vec(v: &[Provenanced<f64>], n: usize) -> Option<Vec<f64>> {
    if v.len() != n || n == 0 {
        return None;
    }
    let mut out = Vec::with_capacity(n);
    for e in v {
        match e.known_value().copied() {
            Some(x) if x.is_finite() => out.push(x),
            _ => return None,
        }
    }
    Some(out)
}

fn planar_lambda_to_ls(
    force_dir: [f64; 3],
    contact: [f64; 3],
    com: [f64; 3],
    yaw: f64,
    n_support: [f64; 3],
    f_max: f64,
    tau_max: f64,
) -> Option<f64> {
    let f_xy = project_to_plane(force_dir, n_support).ok()?;
    let r_xy = project_to_plane(sub3(contact, com), n_support).ok()?;
    let (f_obj, r_obj) = world_xy_to_object(f_xy, r_xy, yaw);
    lambda_to_limit_surface(f_obj, r_obj, f_max, tau_max).ok()
}

fn world_xy_to_object(v: [f64; 2], r: [f64; 2], yaw: f64) -> ([f64; 2], [f64; 2]) {
    let c = yaw.cos();
    let s = yaw.sin();
    let rot = |p: [f64; 2]| [c * p[0] + s * p[1], -s * p[0] + c * p[1]];
    (rot(v), rot(r))
}

fn fill_twist_fields(
    w: &mut EffectFeasibilityWitness,
    input: &PlanarPushInitiation,
    force_dir: [f64; 3],
    contact: [f64; 3],
) {
    w.support_model_kind = Some(input.support_friction_model.kind_name().into());
    w.pressure_model_kind = Some(input.support_friction_model.pressure_name().into());
    w.pusher_velocity = input.pusher_velocity_world.known_value().copied();
    w.contact_force_direction = Some(force_dir);
    let Some(&com) = input.object_com_world.known_value() else {
        return;
    };
    let Some(&yaw) = input.object_yaw_rad.known_value() else {
        return;
    };
    let Some(&n_support) = input.support_normal.known_value() else {
        return;
    };
    let Ok(f_xy) = project_to_plane(force_dir, n_support) else {
        return;
    };
    let Ok(r_xy) = project_to_plane(sub3(contact, com), n_support) else {
        return;
    };
    let (f_obj, r_obj) = world_xy_to_object(f_xy, r_xy, yaw);
    match twist_from_contact_force(f_obj, r_obj, input.support_friction_model) {
        Ok((tw, sign)) => {
            w.planar_twist = Some(tw);
            w.rotation_sign = Some(sign);
        }
        Err(_) => {
            if matches!(input.support_friction_model.pressure_name(), "UNKNOWN") {
                w.rotation_sign = Some(RotationSign::Unknown);
            }
        }
    }
    let Some(vp) = input.pusher_velocity_world.known_value() else {
        return;
    };
    let Some(&n_c) = input.contact_normal_world.known_value() else {
        return;
    };
    let Ok(vp_xy) = project_to_plane(*vp, n_support) else {
        return;
    };
    let Ok(n_xy) = project_to_plane(n_c, n_support) else {
        return;
    };
    let (vp_obj, n_obj) = world_xy_to_object(vp_xy, n_xy, yaw);
    let Some(mu_t) = input.tool_object_friction.sliding_mu_known() else {
        return;
    };
    if let Ok(mode) =
        contact_mode_from_pusher(vp_obj, n_obj, mu_t, r_obj, input.support_friction_model)
    {
        w.contact_mode = Some(mode);
    }
    if let Some(tw) = w.planar_twist {
        if let Ok(comp) = motion_compatibility(tw, r_obj, n_obj, mu_t, input.support_friction_model)
        {
            w.motion_compatibility = Some(comp);
        }
    }
}

fn stamp_levels(w: &mut EffectFeasibilityWitness) {
    w.physical_levels.contact_geometrically_feasible = if w.contact_point.is_some() {
        EffectFeasibility::Feasible
    } else {
        EffectFeasibility::Unknown
    };
    w.physical_levels.motion_initiation = w.feasibility;
    if w.physical_levels.instantaneous_motion == EffectFeasibility::Feasible
        && w.feasibility != EffectFeasibility::Feasible
    {
        w.physical_levels.instantaneous_motion = EffectFeasibility::Unknown;
    }
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
    match chain_physical_effort_signed(model, &params.joint_names) {
        Ok(signed) => {
            params.joint_effort_min = signed
                .iter()
                .map(|s| Provenanced::declared(s.tau_min_nm, "joint.effort_min", 0.0))
                .collect();
            params.joint_effort_max = signed
                .iter()
                .map(|s| Provenanced::declared(s.tau_max_nm, "joint.effort_max", 0.0))
                .collect();
            params.joint_effort_abs = signed
                .iter()
                .map(|s| {
                    Provenanced::declared(
                        s.tau_min_nm.abs().max(s.tau_max_nm.abs()),
                        "joint.effort",
                        0.0,
                    )
                })
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
    if params
        .joint_effort_abs
        .iter()
        .all(|e| e.known_value().is_none())
    {
        if let Ok(tau) = chain_physical_effort(model, &params.joint_names) {
            params.joint_effort_abs = tau
                .into_iter()
                .map(|v| Provenanced::declared(v, "joint.effort", 0.0))
                .collect();
        }
    }
    let mut q_by_joint = BTreeMap::new();
    for (name, qi) in chain.iter().zip(q.iter()) {
        q_by_joint.insert(name.clone(), *qi);
    }
    if let Some(&g) = params.gravity_m_s2.known_value() {
        match gravity_self_load(model, &params.joint_names, &q_by_joint, g) {
            Ok(tau) => {
                params.self_load_torque_nm = self_load_provenanced(&tau, "self_load.gravity");
            }
            Err(crate::self_load::SelfLoadError::Unknown(reason)) => {
                params.self_load_torque_nm = params
                    .joint_names
                    .iter()
                    .map(|_| Provenanced::unknown(reason.clone(), 0.0))
                    .collect();
            }
            Err(_) => {
                params.self_load_torque_nm = params
                    .joint_names
                    .iter()
                    .map(|_| Provenanced::unknown("self_load", 0.0))
                    .collect();
            }
        }
    }
    Ok(evaluate_planar_push_initiation(&params))
}

/// PLANAR_TWIST_DIRECTION. Does not claim a finite-stroke object pose.
pub fn evaluate_planar_twist_direction(input: &PlanarPushInitiation) -> EffectFeasibilityWitness {
    let mut w = evaluate_planar_push_initiation(input);
    w.effect_class = EffectClass::PlanarTwistDirection;
    w.claims_requested_displacement = false;
    if w.planar_twist.is_some()
        && w.rotation_sign.is_some()
        && w.rotation_sign != Some(RotationSign::Unknown)
        && w.feasibility == EffectFeasibility::Feasible
    {
        w.physical_levels.instantaneous_motion = EffectFeasibility::Feasible;
    } else if w.planar_twist.is_none()
        || matches!(w.rotation_sign, Some(RotationSign::Unknown) | None)
    {
        w.physical_levels.instantaneous_motion = EffectFeasibility::Unknown;
        if w.unknown_reason.is_none()
            && w.feasibility != EffectFeasibility::Infeasible
            && matches!(input.support_friction_model, SupportFrictionModel::Unknown)
        {
            w.unknown_reason = Some("SUPPORT_MODEL_UNKNOWN".into());
        }
    }
    w
}

/// Finite stroke as checkpointed local proofs. Not a trajectory pose claim.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SustainedEffectWitness {
    pub effect_class: EffectClass,
    pub feasibility: EffectFeasibility,
    pub first_infeasible_checkpoint: Option<usize>,
    pub first_reason: Option<String>,
    pub checkpoints: Vec<EffectFeasibilityWitness>,
    pub physical_levels: PhysicalEffectLevels,
    pub claims_requested_displacement: bool,
}

pub fn evaluate_sustained_effect(checkpoints: &[PlanarPushInitiation]) -> SustainedEffectWitness {
    let mut levels = PhysicalEffectLevels::unknown();
    let mut out_cp = Vec::new();
    let mut first_bad = None;
    let mut first_reason = None;
    for (i, cp) in checkpoints.iter().enumerate() {
        if !cp.authority_ok {
            let mut w = evaluate_planar_push_initiation(cp);
            w.feasibility = EffectFeasibility::Infeasible;
            w.infeasible_reason = Some("AUTHORITY_REFUSAL".into());
            w.effect_class = EffectClass::SustainedEffect;
            stamp_levels(&mut w);
            out_cp.push(w);
            first_bad = Some(i);
            first_reason = Some("AUTHORITY_REFUSAL".into());
            break;
        }
        if cp.stale_object_evidence {
            let mut w = evaluate_planar_push_initiation(cp);
            w.feasibility = EffectFeasibility::Infeasible;
            w.infeasible_reason = Some("STALE_OBJECT_EVIDENCE".into());
            w.effect_class = EffectClass::SustainedEffect;
            stamp_levels(&mut w);
            out_cp.push(w);
            first_bad = Some(i);
            first_reason = Some("STALE_OBJECT_EVIDENCE".into());
            break;
        }
        if cp.intended_contact_lost {
            let mut w = evaluate_planar_push_initiation(cp);
            w.feasibility = EffectFeasibility::Infeasible;
            w.infeasible_reason = Some("INTENDED_CONTACT_LOST".into());
            w.effect_class = EffectClass::SustainedEffect;
            stamp_levels(&mut w);
            out_cp.push(w);
            first_bad = Some(i);
            first_reason = Some("INTENDED_CONTACT_LOST".into());
            break;
        }
        let local = evaluate_planar_twist_direction(cp);
        let initiation = local.feasibility;
        let instant = local.physical_levels.instantaneous_motion;
        let contact = local.physical_levels.contact_geometrically_feasible;
        let any_infeasible =
            [initiation, instant, contact].contains(&EffectFeasibility::Infeasible);
        let all_feasible = initiation == EffectFeasibility::Feasible
            && instant == EffectFeasibility::Feasible
            && contact == EffectFeasibility::Feasible;
        out_cp.push(local);
        if any_infeasible {
            first_bad = Some(i);
            first_reason = out_cp[i]
                .infeasible_reason
                .clone()
                .or_else(|| out_cp[i].unknown_reason.clone())
                .or_else(|| Some("PREREQUISITE_INFEASIBLE".into()));
            break;
        }
        if !all_feasible {
            first_reason = out_cp[i]
                .unknown_reason
                .clone()
                .or_else(|| Some("PREREQUISITE_NOT_FEASIBLE".into()));
            break;
        }
    }
    if first_bad.is_none()
        && first_reason.is_none()
        && !checkpoints.is_empty()
        && out_cp.first().is_some_and(|w| {
            w.physical_levels
                .all_prerequisites_of_sustained_are_feasible()
        })
    {
        if let Some(first) = out_cp.first() {
            levels = first.physical_levels;
            levels.sustained_effect = EffectFeasibility::Feasible;
        }
    } else {
        if let Some(first) = out_cp.first() {
            levels.contact_geometrically_feasible =
                first.physical_levels.contact_geometrically_feasible;
            levels.motion_initiation = first.physical_levels.motion_initiation;
            levels.instantaneous_motion = first.physical_levels.instantaneous_motion;
        }
        levels.sustained_effect = if first_bad.is_some() {
            EffectFeasibility::Infeasible
        } else {
            EffectFeasibility::Unknown
        };
        if checkpoints.is_empty() {
            levels.sustained_effect = EffectFeasibility::Unknown;
            first_reason = Some("NO_CHECKPOINTS".into());
        }
    }
    let feasibility = levels.sustained_effect;
    SustainedEffectWitness {
        effect_class: EffectClass::SustainedEffect,
        feasibility,
        first_infeasible_checkpoint: first_bad,
        first_reason,
        checkpoints: out_cp,
        physical_levels: levels,
        claims_requested_displacement: false,
    }
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
            contact_force_direction_world: Provenanced::unknown("test.fd", 0.0),
            pusher_velocity_world: Provenanced::unknown("test.vp", 0.0),
            joint_names: vec!["j0".into(), "j1".into(), "j2".into()],
            translational_jacobian_3xn: identity_j(),
            jacobian_residual: Some(0.0),
            joint_effort_abs: vec![declared_tau(tau), declared_tau(tau), declared_tau(tau)],
            joint_effort_min: Vec::new(),
            joint_effort_max: Vec::new(),
            self_load_torque_nm: Vec::new(),
            link_com_known: false,
            object_supported: true,
            approximately_planar: true,
            quasi_static: true,
            single_intended_contact: true,
            no_significant_impact: true,
            object_characteristic_length_m: Some(0.06),
            support_friction_model: SupportFrictionModel::Unknown,
            object_yaw_rad: Provenanced::declared(0.0, "test.yaw", 0.0),
            stale_object_evidence: false,
            intended_contact_lost: false,
            authority_ok: true,
        }
    }

    fn with_available(mut p: PlanarPushInitiation, tau: f64) -> PlanarPushInitiation {
        let n = p.joint_names.len();
        p.joint_effort_min = vec![Provenanced::declared(-tau, "test.tmin", 0.0); n];
        p.joint_effort_max = vec![Provenanced::declared(tau, "test.tmax", 0.0); n];
        p.self_load_torque_nm = vec![Provenanced::declared(0.0, "test.self", 0.0); n];
        p
    }

    fn declared_ellip(mu: f64, mass: f64) -> SupportFrictionModel {
        let n = mass * 9.80665;
        let f_max = mu * n;
        SupportFrictionModel::Ellipsoidal {
            f_max,
            tau_max: f_max * (2.0 / 3.0) * 0.05,
            pressure: PressureDistribution::DeclaredUniform,
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
        let low = evaluate_planar_push_initiation(&with_available(
            centered_input(0.5, 0.0, 1.0, 5.0),
            5.0,
        ));
        let high = evaluate_planar_push_initiation(&centered_input(0.5, 8.0, 1.0, 5.0));
        assert_eq!(low.feasibility, EffectFeasibility::Feasible);
        assert_eq!(
            low.effort_bound_kind,
            EffortBoundKind::AvailableContactEffortBound
        );
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
    fn gross_effort_bound_cannot_yield_feasible() {
        let w = evaluate_planar_push_initiation(&centered_input(0.1, 0.2, 0.8, 20.0));
        assert_eq!(w.effort_bound_kind, EffortBoundKind::GrossEffortBound);
        assert_ne!(w.feasibility, EffectFeasibility::Feasible);
        assert_eq!(w.feasibility, EffectFeasibility::Unknown);
        assert_eq!(
            w.unknown_reason.as_deref(),
            Some("GROSS_EFFORT_BOUND_NOT_AVAILABLE_CONTACT_EFFORT")
        );
    }

    #[test]
    fn required_above_gross_is_still_infeasible() {
        let w = evaluate_planar_push_initiation(&centered_input(3.0, 2.0, 1.0, 0.2));
        assert_eq!(w.feasibility, EffectFeasibility::Infeasible);
        assert_eq!(w.effort_bound_kind, EffortBoundKind::GrossEffortBound);
        assert_eq!(
            w.infeasible_reason.as_deref(),
            Some("INSUFFICIENT_ACTUATOR_EFFORT")
        );
    }

    #[test]
    fn gross_covers_but_self_load_leaves_insufficient_contact_effort() {
        let mut p = with_available(centered_input(0.2, 0.5, 1.0, 5.0), 5.0);
        // required ≈ 0.2 * 9.81 * 0.5 ≈ 0.98 N. Gross λ = 5 N. Self-load 4.9 N along the coupled joint.
        p.self_load_torque_nm = vec![
            Provenanced::declared(4.9, "test.self", 0.0),
            Provenanced::declared(0.0, "test.self", 0.0),
            Provenanced::declared(0.0, "test.self", 0.0),
        ];
        let w = evaluate_planar_push_initiation(&p);
        assert_eq!(
            w.effort_bound_kind,
            EffortBoundKind::AvailableContactEffortBound
        );
        assert_ne!(w.feasibility, EffectFeasibility::Feasible);
        assert_eq!(w.feasibility, EffectFeasibility::Infeasible);
        assert!(
            w.available_lambda.unwrap() < 0.2,
            "{:?}",
            w.available_lambda
        );
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
        let w = evaluate_planar_push_initiation(&with_available(
            centered_input(0.1, 0.2, 0.8, 20.0),
            20.0,
        ));
        assert_eq!(w.feasibility, EffectFeasibility::Feasible);
        assert_eq!(w.effect_class, EffectClass::MotionInitiation);
        assert!(!w.claims_requested_displacement);
        assert_eq!(w.off_center_class, OffCenterClass::CenteredTranslation);
        assert_eq!(
            w.effort_bound_kind,
            EffortBoundKind::AvailableContactEffortBound
        );
        assert_eq!(w.regime, RegimeApplicability::Applicable);
        assert_eq!(
            w.physical_levels.motion_initiation,
            EffectFeasibility::Feasible
        );
        assert_ne!(
            w.physical_levels.instantaneous_motion,
            EffectFeasibility::Feasible
        );
        assert_ne!(
            w.physical_levels.sustained_effect,
            EffectFeasibility::Feasible
        );
    }

    #[test]
    fn off_center_is_not_treated_as_centered_translation() {
        let mut p = with_available(centered_input(0.1, 0.2, 0.8, 20.0), 20.0);
        p.contact_point_world = Provenanced::declared([0.2, 0.04, 0.03], "test.contact", 0.0);
        p.object_com_world = Provenanced::declared([0.2, 0.0, 0.03], "test.com", 0.0);
        let w = evaluate_planar_push_initiation(&p);
        assert_ne!(w.off_center_class, OffCenterClass::CenteredTranslation);
        assert_eq!(w.feasibility, EffectFeasibility::Unknown);
        assert_eq!(
            w.unknown_reason.as_deref(),
            Some("OFF_CENTER_WITHOUT_SUPPORT_MODEL")
        );
        assert_ne!(w.off_center_class, OffCenterClass::TranslationDominated);
        assert_ne!(w.off_center_class, OffCenterClass::RotationDominated);
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
        let f = evaluate_planar_push_initiation(&with_available(
            centered_input(0.05, 0.1, 1.0, 20.0),
            20.0,
        ));
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
        assert_eq!(
            f.effort_bound_kind,
            EffortBoundKind::AvailableContactEffortBound
        );
        assert_ne!(i.infeasible_reason.as_deref(), Some("TOOL_CONTACT_SLIP"));
    }

    #[test]
    fn missing_robot_com_cannot_be_available_feasible() {
        let mut m = synth_planar_two_link();
        m.joints[0].effort_max = Provenanced::declared(20.0, "test", 0.0);
        m.joints[1].effort_max = Provenanced::declared(20.0, "test", 0.0);
        m.bodies[1].mass_kg = Provenanced::declared(1.0, "test", 0.0);
        let params = with_available(centered_input(0.05, 0.1, 1.0, 20.0), 20.0);
        let w =
            evaluate_planar_push_at_model(&m, "ee", &[0.3, -0.4], [0.0, 0.0, 0.0], params).unwrap();
        assert_ne!(w.feasibility, EffectFeasibility::Feasible);
        assert_eq!(w.effort_bound_kind, EffortBoundKind::GrossEffortBound);
    }

    #[test]
    fn asymmetric_and_negative_gear_change_lambda_and_limiting_joint() {
        let mut p = with_available(centered_input(0.05, 0.1, 1.0, 20.0), 20.0);
        p.joint_effort_min = vec![
            Provenanced::declared(-20.0, "t", 0.0),
            Provenanced::declared(-1.0, "t", 0.0),
            Provenanced::declared(-20.0, "t", 0.0),
        ];
        p.joint_effort_max = vec![
            Provenanced::declared(20.0, "t", 0.0),
            Provenanced::declared(2.0, "t", 0.0),
            Provenanced::declared(20.0, "t", 0.0),
        ];
        p.translational_jacobian_3xn = vec![
            vec![1.0, 0.5, 0.0],
            vec![0.0, 0.0, 0.0],
            vec![0.0, 0.0, 1.0],
        ];
        let a = evaluate_planar_push_initiation(&p);
        p.joint_effort_min[1] = Provenanced::declared(-10.0, "t", 0.0);
        p.joint_effort_max[1] = Provenanced::declared(10.0, "t", 0.0);
        let b = evaluate_planar_push_initiation(&p);
        assert_eq!(a.limiting_joint.as_deref(), Some("j1"));
        assert_ne!(a.available_lambda, b.available_lambda);
        let (tmin, tmax) =
            realityos_physics::joint_torque_limits_from_actuator(-1.0, 5.0, -2.0).unwrap();
        p.joint_effort_min = vec![
            Provenanced::declared(tmin, "t", 0.0),
            Provenanced::declared(-20.0, "t", 0.0),
            Provenanced::declared(-20.0, "t", 0.0),
        ];
        p.joint_effort_max = vec![
            Provenanced::declared(tmax, "t", 0.0),
            Provenanced::declared(20.0, "t", 0.0),
            Provenanced::declared(20.0, "t", 0.0),
        ];
        p.translational_jacobian_3xn = identity_j();
        let neg = evaluate_planar_push_initiation(&p);
        assert_eq!(neg.limiting_joint.as_deref(), Some("j0"));
        assert!((neg.available_lambda.unwrap() - 2.0).abs() < 1e-9);
    }

    #[test]
    fn pusher_velocity_is_not_force_direction_and_can_change_mode() {
        let mut p = with_available(centered_input(0.1, 0.2, 0.4, 20.0), 20.0);
        p.support_friction_model = declared_ellip(0.2, 0.1);
        p.contact_force_direction_world = Provenanced::declared([1.0, 0.0, 0.0], "test.fd", 0.0);
        p.push_direction_world = Provenanced::declared([1.0, 0.0, 0.0], "test.d", 0.0);
        p.pusher_velocity_world = Provenanced::declared([1.0, 0.0, 0.0], "test.vp", 0.0);
        let along = evaluate_planar_twist_direction(&p);
        p.pusher_velocity_world = Provenanced::declared([1.0, 3.0, 0.0], "test.vp", 0.0);
        let tan = evaluate_planar_twist_direction(&p);
        assert!(along.pusher_velocity.is_some());
        assert!(along.contact_force_direction.is_some());
        assert_eq!(along.contact_mode, Some(ContactMode::Sticking));
        assert_ne!(tan.contact_mode, Some(ContactMode::Sticking));
        assert_ne!(tan.pusher_velocity, tan.contact_force_direction);
    }

    #[test]
    fn mirrored_offset_opposite_rotation_with_declared_support() {
        let mut p = with_available(centered_input(0.2, 0.3, 0.8, 20.0), 20.0);
        p.support_friction_model = declared_ellip(0.3, 0.2);
        p.object_com_world = Provenanced::declared([0.2, 0.0, 0.03], "test.com", 0.0);
        p.contact_point_world = Provenanced::declared([0.2, 0.03, 0.03], "test.contact", 0.0);
        let a = evaluate_planar_twist_direction(&p);
        p.contact_point_world = Provenanced::declared([0.2, -0.03, 0.03], "test.contact", 0.0);
        let b = evaluate_planar_twist_direction(&p);
        assert_eq!(a.rotation_sign, Some(RotationSign::Clockwise));
        assert_eq!(b.rotation_sign, Some(RotationSign::Counterclockwise));
        assert_ne!(a.rotation_sign, b.rotation_sign);
    }

    #[test]
    fn zero_wrench_does_not_claim_sliding_twist() {
        let mut p = with_available(centered_input(0.1, 0.2, 0.8, 20.0), 20.0);
        p.support_friction_model = declared_ellip(0.2, 0.1);
        p.contact_force_direction_world = Provenanced::declared([0.0, 0.0, 0.0], "test.fd", 0.0);
        p.push_direction_world = Provenanced::declared([0.0, 0.0, 0.0], "test.d", 0.0);
        let w = evaluate_planar_twist_direction(&p);
        assert!(w.planar_twist.is_none());
        assert_ne!(w.feasibility, EffectFeasibility::Feasible);
    }

    #[test]
    fn unknown_pressure_is_not_auto_uniform() {
        let mut p = with_available(centered_input(0.2, 0.3, 0.8, 20.0), 20.0);
        p.support_friction_model = SupportFrictionModel::Ellipsoidal {
            f_max: 1.0,
            tau_max: 0.05,
            pressure: PressureDistribution::Unknown,
        };
        p.contact_point_world = Provenanced::declared([0.2, 0.03, 0.03], "test.contact", 0.0);
        let w = evaluate_planar_twist_direction(&p);
        assert_eq!(w.pressure_model_kind.as_deref(), Some("UNKNOWN"));
        assert_ne!(w.rotation_sign, Some(RotationSign::Clockwise));
        assert_ne!(w.rotation_sign, Some(RotationSign::Counterclockwise));
        assert_ne!(w.feasibility, EffectFeasibility::Feasible);
    }

    #[test]
    fn initiation_feasible_does_not_imply_instantaneous_or_sustained() {
        let p = with_available(centered_input(0.1, 0.2, 0.8, 20.0), 20.0);
        let w = evaluate_planar_push_initiation(&p);
        assert_eq!(w.feasibility, EffectFeasibility::Feasible);
        assert_eq!(
            w.physical_levels.motion_initiation,
            EffectFeasibility::Feasible
        );
        assert_ne!(
            w.physical_levels.instantaneous_motion,
            EffectFeasibility::Feasible
        );
        assert_ne!(
            w.physical_levels.sustained_effect,
            EffectFeasibility::Feasible
        );
        let s = evaluate_sustained_effect(&[]);
        assert_ne!(s.feasibility, EffectFeasibility::Feasible);
        assert!(!s.claims_requested_displacement);
    }

    #[test]
    fn later_checkpoint_infeasible_is_not_sustainably_feasible() {
        let mut q0 = with_available(centered_input(0.05, 0.1, 1.0, 20.0), 20.0);
        q0.support_friction_model = declared_ellip(0.1, 0.05);
        q0.pusher_velocity_world = Provenanced::declared([1.0, 0.0, 0.0], "test.vp", 0.0);
        let mut q1 = q0.clone();
        q1.joint_effort_min = vec![Provenanced::declared(-0.01, "t", 0.0); 3];
        q1.joint_effort_max = vec![Provenanced::declared(0.01, "t", 0.0); 3];
        q1.mass_kg = Provenanced::declared(3.0, "t", 0.0);
        q1.object_support_friction =
            PairFriction::coulomb("object", "support", Provenanced::declared(2.0, "t", 0.0));
        let s = evaluate_sustained_effect(&[q0, q1]);
        assert_ne!(s.feasibility, EffectFeasibility::Feasible);
        assert_eq!(s.first_infeasible_checkpoint, Some(1));
        assert_eq!(s.checkpoints[0].feasibility, EffectFeasibility::Feasible);
        assert_eq!(s.effect_class, EffectClass::SustainedEffect);
        assert!(!s.claims_requested_displacement);
        assert_eq!(
            s.physical_levels.sustained_effect,
            EffectFeasibility::Infeasible
        );
    }

    #[test]
    fn stale_or_lost_contact_stops_sustained_effect() {
        let mut p = with_available(centered_input(0.05, 0.1, 1.0, 20.0), 20.0);
        p.stale_object_evidence = true;
        let s = evaluate_sustained_effect(&[p.clone()]);
        assert_eq!(s.first_reason.as_deref(), Some("STALE_OBJECT_EVIDENCE"));
        p.stale_object_evidence = false;
        p.intended_contact_lost = true;
        let l = evaluate_sustained_effect(&[p]);
        assert_eq!(l.first_reason.as_deref(), Some("INTENDED_CONTACT_LOST"));
    }

    /// P0: initiation FEASIBLE + instantaneous UNKNOWN must not mint SUSTAINED FEASIBLE.
    /// `centered_input` uses SupportFrictionModel::Unknown, so twist stays unevaluable.
    #[test]
    fn instantaneous_unknown_cannot_make_sustained_feasible() {
        let p = with_available(centered_input(0.1, 0.2, 0.8, 20.0), 20.0);
        let local = evaluate_planar_twist_direction(&p);
        assert_eq!(local.feasibility, EffectFeasibility::Feasible);
        assert_eq!(
            local.physical_levels.instantaneous_motion,
            EffectFeasibility::Unknown
        );
        let s = evaluate_sustained_effect(&[p]);
        assert_ne!(s.feasibility, EffectFeasibility::Feasible);
        assert_ne!(
            s.physical_levels.sustained_effect,
            EffectFeasibility::Feasible
        );
        assert_eq!(s.feasibility, EffectFeasibility::Unknown);
        assert_eq!(
            s.physical_levels.instantaneous_motion,
            EffectFeasibility::Unknown
        );
        assert_eq!(s.first_infeasible_checkpoint, None);
    }

    #[test]
    fn sustained_feasible_implies_every_prerequisite_feasible() {
        let mut p = with_available(centered_input(0.1, 0.2, 0.8, 20.0), 20.0);
        p.support_friction_model = declared_ellip(0.2, 0.1);
        p.pusher_velocity_world = Provenanced::declared([1.0, 0.0, 0.0], "test.vp", 0.0);
        let local = evaluate_planar_twist_direction(&p);
        let s = evaluate_sustained_effect(&[p]);
        if s.feasibility == EffectFeasibility::Feasible {
            assert_eq!(
                local.physical_levels.instantaneous_motion,
                EffectFeasibility::Feasible
            );
        }
        assert_sustained_levels_monotonic(&s.physical_levels);
        assert_eq!(
            s.physical_levels.sustained_effect == EffectFeasibility::Feasible,
            s.physical_levels
                .all_prerequisites_of_sustained_are_feasible()
                && s.feasibility == EffectFeasibility::Feasible
        );
        if s.feasibility == EffectFeasibility::Feasible {
            assert_eq!(
                s.physical_levels.contact_geometrically_feasible,
                EffectFeasibility::Feasible
            );
            assert_eq!(
                s.physical_levels.motion_initiation,
                EffectFeasibility::Feasible
            );
            assert_eq!(
                s.physical_levels.instantaneous_motion,
                EffectFeasibility::Feasible
            );
        }
    }

    #[test]
    fn shipped_sustained_effect_does_not_treat_not_infeasible_as_feasible() {
        let src = include_str!("effect_feasibility.rs");
        let hole = ["instantaneous_motion != ", "EffectFeasibility::Infeasible"].concat();
        assert!(
            !src.contains(&hole),
            "SUSTAINED_EFFECT must not treat instantaneous != INFEASIBLE as a pass"
        );
    }

    fn assert_sustained_levels_monotonic(levels: &PhysicalEffectLevels) {
        if levels.sustained_effect == EffectFeasibility::Feasible {
            assert_eq!(
                levels.contact_geometrically_feasible,
                EffectFeasibility::Feasible
            );
            assert_eq!(levels.motion_initiation, EffectFeasibility::Feasible);
            assert_eq!(levels.instantaneous_motion, EffectFeasibility::Feasible);
        }
        if levels.instantaneous_motion == EffectFeasibility::Feasible {
            assert_eq!(levels.motion_initiation, EffectFeasibility::Feasible);
            assert_eq!(
                levels.contact_geometrically_feasible,
                EffectFeasibility::Feasible
            );
        }
        if levels.motion_initiation == EffectFeasibility::Feasible {
            assert_eq!(
                levels.contact_geometrically_feasible,
                EffectFeasibility::Feasible
            );
        }
    }
}
