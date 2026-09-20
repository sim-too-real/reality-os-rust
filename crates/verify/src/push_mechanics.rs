//! Planar-push mechanics prediction, freeze-before-execute, and first-divergence.
//! Privileged MuJoCo contact/actuator force is post-hoc only.

use serde::{Deserialize, Serialize};
use serde_json::Value;

use realityos_semantics::effect_feasibility::{
    evaluate_planar_push_initiation, EffectFeasibility, EffectFeasibilityWitness, OffCenterClass,
    PlanarPushInitiation,
};
use realityos_semantics::pair_friction::PairFriction;
use realityos_semantics::provenance::Provenanced;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum MechanicsFirstDivergence {
    ParameterMissing,
    ParameterWrong,
    MechanicsRegimeInvalid,
    JacobianMismatch,
    EffortMappingWrong,
    GravityLoadUnaccounted,
    ContactNotEstablished,
    ContactForceInsufficient,
    ToolContactSlip,
    SupportFrictionHigherThanModel,
    UnexpectedRotation,
    ControlTrackingFailure,
    AuthorityRefusal,
    Unknown,
    None,
}

impl MechanicsFirstDivergence {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::ParameterMissing => "PARAMETER_MISSING",
            Self::ParameterWrong => "PARAMETER_WRONG",
            Self::MechanicsRegimeInvalid => "MECHANICS_REGIME_INVALID",
            Self::JacobianMismatch => "JACOBIAN_MISMATCH",
            Self::EffortMappingWrong => "EFFORT_MAPPING_WRONG",
            Self::GravityLoadUnaccounted => "GRAVITY_LOAD_UNACCOUNTED",
            Self::ContactNotEstablished => "CONTACT_NOT_ESTABLISHED",
            Self::ContactForceInsufficient => "CONTACT_FORCE_INSUFFICIENT",
            Self::ToolContactSlip => "TOOL_CONTACT_SLIP",
            Self::SupportFrictionHigherThanModel => "SUPPORT_FRICTION_HIGHER_THAN_MODEL",
            Self::UnexpectedRotation => "UNEXPECTED_ROTATION",
            Self::ControlTrackingFailure => "CONTROL_TRACKING_FAILURE",
            Self::AuthorityRefusal => "AUTHORITY_REFUSAL",
            Self::Unknown => "UNKNOWN",
            Self::None => "NONE",
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FrozenMechanicsPrediction {
    pub frozen_at_ns: u128,
    pub witness: EffectFeasibilityWitness,
    pub predictor_input: Value,
}

impl FrozenMechanicsPrediction {
    pub fn freeze(
        witness: EffectFeasibilityWitness,
        predictor_input: &PlanarPushInitiation,
    ) -> Self {
        Self {
            frozen_at_ns: monotonic_ns(),
            witness,
            predictor_input: serde_json::to_value(predictor_input).unwrap_or(Value::Null),
        }
    }

    pub fn contains_privileged_force(&self) -> bool {
        let s = self.predictor_input.to_string();
        s.contains("actuator_force")
            || s.contains("normal_force")
            || s.contains("tangential_force")
            || s.contains("cfrc_ext")
            || s.contains("PRIVILEGED")
    }
}

fn monotonic_ns() -> u128 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_nanos())
        .unwrap_or(0)
}

pub fn classify_mechanics_divergence(
    frozen: &FrozenMechanicsPrediction,
    contact_established: bool,
    object_displaced: bool,
    unauthorized_writes: u64,
    ctrl_writes: u64,
    authority_refused: bool,
) -> MechanicsFirstDivergence {
    if unauthorized_writes > 0 {
        return MechanicsFirstDivergence::Unknown;
    }
    if authority_refused
        || (ctrl_writes == 0 && frozen.witness.feasibility != EffectFeasibility::Unknown)
    {
        if matches!(
            frozen.witness.unknown_reason.as_deref(),
            Some(r) if r.contains("PARAMETER_MISSING")
        ) {
            return MechanicsFirstDivergence::ParameterMissing;
        }
        if authority_refused {
            return MechanicsFirstDivergence::AuthorityRefusal;
        }
    }
    match frozen.witness.feasibility {
        EffectFeasibility::Unknown => {
            let r = frozen.witness.unknown_reason.as_deref().unwrap_or("");
            if r.contains("PARAMETER_MISSING") {
                MechanicsFirstDivergence::ParameterMissing
            } else if r == "MODEL_NOT_APPLICABLE" {
                MechanicsFirstDivergence::MechanicsRegimeInvalid
            } else if r.contains("JACOBIAN") || r == "NEAR_SINGULAR_JACOBIAN" {
                MechanicsFirstDivergence::JacobianMismatch
            } else if r.contains("EFFORT_MAPPING") {
                MechanicsFirstDivergence::EffortMappingWrong
            } else {
                MechanicsFirstDivergence::Unknown
            }
        }
        EffectFeasibility::Infeasible => {
            if frozen.witness.infeasible_reason.as_deref() == Some("TOOL_CONTACT_SLIP") {
                if object_displaced {
                    MechanicsFirstDivergence::ParameterWrong
                } else {
                    MechanicsFirstDivergence::ToolContactSlip
                }
            } else if object_displaced {
                MechanicsFirstDivergence::ParameterWrong
            } else if !contact_established {
                MechanicsFirstDivergence::ContactNotEstablished
            } else {
                MechanicsFirstDivergence::ContactForceInsufficient
            }
        }
        EffectFeasibility::Feasible => {
            if !contact_established {
                MechanicsFirstDivergence::ContactNotEstablished
            } else if !object_displaced {
                if frozen
                    .witness
                    .support_friction_mu
                    .is_some_and(|mu| mu > 1.0)
                {
                    MechanicsFirstDivergence::SupportFrictionHigherThanModel
                } else {
                    MechanicsFirstDivergence::ContactForceInsufficient
                }
            } else if frozen.witness.off_center_class == OffCenterClass::RotationDominated {
                MechanicsFirstDivergence::UnexpectedRotation
            } else {
                MechanicsFirstDivergence::None
            }
        }
    }
}

pub fn identity_translational_jacobian() -> Vec<Vec<f64>> {
    vec![
        vec![1.0, 0.0, 0.0],
        vec![0.0, 1.0, 0.0],
        vec![0.0, 0.0, 1.0],
    ]
}

pub fn declared_centered_push(
    mass: Option<f64>,
    mu_support: Option<f64>,
    mu_tool: Option<f64>,
    tau: Option<f64>,
    contact: [f64; 3],
    com: [f64; 3],
    direction: [f64; 3],
    normal: [f64; 3],
) -> PlanarPushInitiation {
    let mass_p = match mass {
        Some(m) => Provenanced::user_declared(m, "scenario.mass", 0.0),
        None => Provenanced::unknown("scenario.mass", 0.0),
    };
    let mu_s = match mu_support {
        Some(m) => PairFriction::coulomb(
            "object",
            "support",
            Provenanced::user_declared(m, "scenario.mu_support", 0.0),
        ),
        None => PairFriction::unknown("object", "support", "scenario.mu_support"),
    };
    let mu_t = match mu_tool {
        Some(m) => PairFriction::coulomb(
            "tool",
            "object",
            Provenanced::user_declared(m, "scenario.mu_tool", 0.0),
        ),
        None => PairFriction::unknown("tool", "object", "scenario.mu_tool"),
    };
    let tau_p = match tau {
        Some(t) => Provenanced::user_declared(t, "scenario.effort", 0.0),
        None => Provenanced::unknown("scenario.effort", 0.0),
    };
    PlanarPushInitiation {
        mass_kg: mass_p,
        object_com_world: Provenanced::user_declared(com, "scenario.com", 0.0),
        gravity_m_s2: Provenanced::user_declared([0.0, 0.0, -9.80665], "scenario.g", 0.0),
        support_normal: Provenanced::user_declared([0.0, 0.0, 1.0], "scenario.support_n", 0.0),
        object_support_friction: mu_s,
        tool_object_friction: mu_t,
        contact_point_world: Provenanced::user_declared(contact, "scenario.contact", 0.0),
        contact_normal_world: Provenanced::user_declared(normal, "scenario.contact_n", 0.0),
        push_direction_world: Provenanced::user_declared(direction, "scenario.dir", 0.0),
        joint_names: vec!["j0".into(), "j1".into(), "j2".into()],
        translational_jacobian_3xn: identity_translational_jacobian(),
        jacobian_residual: Some(0.0),
        joint_effort_abs: vec![tau_p.clone(), tau_p.clone(), tau_p],
        link_com_known: false,
        object_supported: true,
        approximately_planar: true,
        quasi_static: true,
        single_intended_contact: true,
        no_significant_impact: true,
        object_characteristic_length_m: Some(0.05),
    }
}

pub fn predict_centered_push(
    mass: Option<f64>,
    mu_support: Option<f64>,
    mu_tool: Option<f64>,
    tau: Option<f64>,
) -> FrozenMechanicsPrediction {
    let input = declared_centered_push(
        mass,
        mu_support,
        mu_tool,
        tau,
        [0.22, 0.0, 0.03],
        [0.22, 0.0, 0.03],
        [1.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
    );
    let witness = evaluate_planar_push_initiation(&input);
    FrozenMechanicsPrediction::freeze(witness, &input)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bundle::RobotBundle;
    use crate::corpus;
    use crate::honesty::SIMULATION_ONLY;
    use crate::manipulation::{
        run_skill_episode_ex, software_sha, with_episode_qpos, PlacementOutcome,
    };
    use crate::manipulation_scenarios::ManipulationScenario;
    use crate::manipulation_verify::body_xyz;
    use crate::mujoco_exec::ensure_mujoco_or_skip;
    use crate::normalize::{ActuatorRecord, RobotManifest};
    use crate::observation::VerifierTruth;
    use crate::runner::load_and_normalize;
    use crate::semantics_map::embodiment_from_manifest;
    use realityos_semantics::adapter::synth_planar_two_link;
    use realityos_semantics::contact_jacobian::contact_jacobian_witness;
    use realityos_semantics::contact_maneuver::{tool_offset_in_ee, ContactManeuver};
    use realityos_semantics::effect_feasibility::evaluate_planar_push_at_model;
    use realityos_semantics::effect_feasibility::{EffectClass, EffortBoundKind};
    use realityos_semantics::effort::physical_joint_effort;
    use realityos_semantics::embodiment::{Actuator, EmbodimentModel, JointKind};
    use realityos_semantics::grasp_hold::{evaluate_pinch_hold, HoldFeasibility, PinchHoldInput};
    use realityos_semantics::kinematics::forward_kinematics;
    use realityos_semantics::physical_quantity::PhysicalEffort;
    use realityos_semantics::provenance::Provenanced;
    use serde_json::json;

    fn scratch_dir() -> std::path::PathBuf {
        std::env::temp_dir().join("realityos-mechanics-scratch")
    }

    fn write_scratch(name: &str, body: &str) {
        let _ = std::fs::create_dir_all(scratch_dir());
        let _ = std::fs::write(scratch_dir().join(name), body);
    }

    #[test]
    fn tau_max_from_ctrlrange_is_not_forcerange() {
        let man = RobotManifest {
            robot_id: "t".into(),
            nq: 1,
            nv: 1,
            nu: 1,
            nbody: 1,
            njoint: 1,
            nactuator: 1,
            nsensor: 0,
            ncamera: 0,
            timestep: 0.002,
            joints: vec![],
            actuators: vec![ActuatorRecord {
                name: "pos0".into(),
                transmission_target: "j0".into(),
                control_dimensions: 1,
                ctrlrange: [-255.0, 255.0],
                ctrllimited: true,
                force_range: Some([-1.5, 1.5]),
                actuator_type: "position".into(),
                transmission_kind: "joint".into(),
            }],
            sensors: vec![],
            cameras: vec![],
            bodies: vec![],
            sites: vec![],
            site_records: vec![],
            derived: crate::normalize::DerivedInterface::default(),
            model_hash: "h".into(),
            source_hash: "s".into(),
            mujoco_version: "3".into(),
            source_format: "mjcf".into(),
            lost_features: vec![],
            support_bodies: vec![],
            collision_groups: Default::default(),
            geoms: Vec::new(),
            metal: false,
            evidence_status: SIMULATION_ONLY.into(),
        };
        let cmd = man.tau_max();
        let fr = man.physical_forcerange_abs();
        assert!((cmd[0] - 255.0).abs() < 1e-12);
        assert_eq!(fr[0], Some(1.5));
        assert_ne!(cmd[0], fr[0].unwrap());
        assert_eq!(man.command_scale_abs()[0], cmd[0]);
    }

    #[test]
    fn shipped_effort_rejects_ctrlrange_and_tendon() {
        let mut m = synth_planar_two_link();
        m.actuators[0] = Actuator {
            name: "a0".into(),
            target_joint: "j0".into(),
            control_mode: "position".into(),
            transmission_kind: "joint".into(),
            ctrlrange: Provenanced::declared([-1.0, 1.0], "ctrl", 0.0),
            forcerange: Provenanced::unknown("fr", 0.0),
            gear: Provenanced::unknown("g", 0.0),
        };
        match physical_joint_effort(&m.joints[0], Some(&m.actuators[0])) {
            PhysicalEffort::Unknown { reason } => {
                assert_eq!(reason, "CTRLRANGE_IS_NOT_FORCE_RANGE");
            }
            other => panic!("{other:?}"),
        }
        m.actuators[0].transmission_kind = "tendon".into();
        m.actuators[0].ctrlrange = Provenanced::declared([0.0, 255.0], "tendon", 0.0);
        match physical_joint_effort(&m.joints[0], Some(&m.actuators[0])) {
            PhysicalEffort::Unknown { reason } => {
                assert_eq!(reason, "TENDON_COMMAND_IS_NOT_JOINT_TORQUE");
            }
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn jacobian_fd_residual_on_shipped_fk() {
        let m = synth_planar_two_link();
        let chain = m.ee_joint_chain("ee").unwrap();
        let w = contact_jacobian_witness(&m, &chain, "ee", &[0.25, -0.4], [0.02, 0.0, 0.01], 1e-6)
            .unwrap();
        assert_eq!(w.joint_names, chain);
        assert!(w.residual < 1e-5, "residual {}", w.residual);
        let log = format!(
            "joint_names={:?}\nresidual={}\nanalytic_col0=[{}, {}, {}]\nfd_col0=[{}, {}, {}]\n",
            w.joint_names,
            w.residual,
            w.analytic_3xn[0][0],
            w.analytic_3xn[1][0],
            w.analytic_3xn[2][0],
            w.finite_difference_3xn[0][0],
            w.finite_difference_3xn[1][0],
            w.finite_difference_3xn[2][0],
        );
        write_scratch("jacobian-fd.log", &log);
        assert!((w.analytic_3xn[0][0] - w.finite_difference_3xn[0][0]).abs() <= w.residual + 1e-12);
    }

    #[test]
    fn wrench_bound_halves_with_limiting_effort() {
        let mut m = synth_planar_two_link();
        m.joints[0].effort_max = Provenanced::declared(2.0, "e", 0.0);
        m.joints[1].effort_max = Provenanced::declared(2.0, "e", 0.0);
        let input = declared_centered_push(
            Some(0.05),
            Some(0.1),
            Some(1.0),
            Some(2.0),
            [0.3, 0.0, 0.0],
            [0.3, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 1.0, 0.0],
        );
        let a =
            evaluate_planar_push_at_model(&m, "ee", &[0.4, -0.5], [0.0, 0.0, 0.0], input.clone())
                .unwrap();
        m.joints[0].effort_max = Provenanced::declared(1.0, "e", 0.0);
        m.joints[1].effort_max = Provenanced::declared(1.0, "e", 0.0);
        let b =
            evaluate_planar_push_at_model(&m, "ee", &[0.4, -0.5], [0.0, 0.0, 0.0], input).unwrap();
        let la = a.available_lambda.expect("lambda a");
        let lb = b.available_lambda.expect("lambda b");
        assert!(la.is_finite() && lb.is_finite());
        assert!(
            lb < la - 1e-12,
            "halving effort must reduce lambda {la} -> {lb}"
        );
        assert!(a.limiting_joint.is_some());
        write_scratch(
            "wrench-bound.log",
            &format!(
                "lambda_full={la} lambda_half={lb} limiting={:?} residual={:?}\n",
                a.limiting_joint, a.jacobian_residual
            ),
        );
    }

    #[test]
    fn reconstruct_feasible_infeasible_unknown_witnesses() {
        let f = predict_centered_push(Some(0.05), Some(0.1), Some(1.0), Some(20.0));
        let i = predict_centered_push(Some(3.0), Some(2.0), Some(1.0), Some(0.2));
        let u = predict_centered_push(None, Some(0.2), Some(0.8), Some(20.0));
        assert_eq!(f.witness.feasibility, EffectFeasibility::Feasible);
        assert_eq!(f.witness.effect_class, EffectClass::MotionInitiation);
        assert!(!f.witness.claims_requested_displacement);
        assert_eq!(
            f.witness.effort_bound_kind,
            EffortBoundKind::GrossEffortBound
        );
        assert_eq!(i.witness.feasibility, EffectFeasibility::Infeasible);
        assert_eq!(u.witness.feasibility, EffectFeasibility::Unknown);
        assert!(!f.contains_privileged_force());
        let doc = json!({
            "feasible": f.witness,
            "infeasible": i.witness,
            "unknown": u.witness,
        });
        write_scratch(
            "effect-witnesses.json",
            &serde_json::to_string_pretty(&doc).unwrap(),
        );
    }

    #[test]
    fn grasp_hold_reuses_friction_primitives() {
        let ok = evaluate_pinch_hold(&PinchHoldInput {
            mass_kg: Provenanced::user_declared(0.2, "m", 0.0),
            gravity_m_s2: Provenanced::user_declared([0.0, 0.0, -9.80665], "g", 0.0),
            finger_a_inward_normal: [1.0, 0.0, 0.0],
            finger_b_inward_normal: [-1.0, 0.0, 0.0],
            finger_object_friction: PairFriction::coulomb(
                "finger",
                "object",
                Provenanced::user_declared(0.8, "mu", 0.0),
            ),
            gripper_force_bound_n: Provenanced::user_declared(5.0, "f", 0.0),
        });
        let slip = evaluate_pinch_hold(&PinchHoldInput {
            mass_kg: Provenanced::user_declared(0.2, "m", 0.0),
            gravity_m_s2: Provenanced::user_declared([0.0, 0.0, -9.80665], "g", 0.0),
            finger_a_inward_normal: [1.0, 0.0, 0.0],
            finger_b_inward_normal: [-1.0, 0.0, 0.0],
            finger_object_friction: PairFriction::coulomb(
                "finger",
                "object",
                Provenanced::user_declared(0.01, "mu", 0.0),
            ),
            gripper_force_bound_n: Provenanced::user_declared(5.0, "f", 0.0),
        });
        assert_eq!(ok.feasibility, HoldFeasibility::Feasible);
        assert_eq!(slip.feasibility, HoldFeasibility::Infeasible);
        write_scratch(
            "grasp-hold.log",
            &format!("ok={:?} slip={:?}\n", ok.feasibility, slip.feasibility),
        );
    }

    fn sample_chain_q(model: &EmbodimentModel, chain: &[String], mut rng: u64) -> Vec<f64> {
        let mut q = Vec::with_capacity(chain.len());
        for name in chain {
            rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1);
            let u = (rng >> 33) as f64 / (1u64 << 31) as f64;
            let joint = model.joints.iter().find(|j| j.name == *name);
            let (lo, hi) = match joint {
                Some(j) => (j.q_min.value.unwrap_or(-1.2), j.q_max.value.unwrap_or(1.2)),
                None => (-1.2, 1.2),
            };
            q.push(lo.max(-2.5) + (hi.min(2.5) - lo.max(-2.5)) * u.clamp(0.05, 0.95));
        }
        q
    }

    fn object_size_m(sc: &ManipulationScenario) -> f64 {
        sc.objects
            .iter()
            .find(|o| o["name"] == "obj0")
            .and_then(|o| o["size"].as_array())
            .and_then(|a| a.first())
            .and_then(|v| v.as_f64())
            .unwrap_or(0.03)
    }

    fn declare_joint_effort(model: &mut EmbodimentModel, tau_abs: f64) {
        for j in &mut model.joints {
            if matches!(j.kind, JointKind::Hinge | JointKind::Slide) {
                j.effort_max = Provenanced::user_declared(tau_abs, "scenario.joint_effort", 0.0);
            }
        }
    }

    fn contact_chain_q(
        model: &EmbodimentModel,
        ee: &str,
        m: &ContactManeuver,
    ) -> Result<Vec<f64>, String> {
        let chain = model
            .ee_joint_chain(ee)
            .ok_or_else(|| "no ee chain".to_string())?;
        if let Some(w) = m.executable.as_ref() {
            if w.contact.q.len() == chain.len() && w.joint_names == chain {
                return Ok(w.contact.q.clone());
            }
            if w.contact.q.len() == w.joint_names.len() {
                let mut q = vec![0.0; chain.len()];
                for (i, name) in chain.iter().enumerate() {
                    if let Some(j) = w.joint_names.iter().position(|n| n == name) {
                        q[i] = w.contact.q[j];
                    } else if m.sampled_q.len() == chain.len() {
                        q[i] = m.sampled_q[i];
                    }
                }
                return Ok(q);
            }
        }
        if m.sampled_q.len() == chain.len() {
            return Ok(m.sampled_q.clone());
        }
        Err("cannot align contact q to chain".into())
    }

    fn freeze_from_geometry_witness(
        model: &EmbodimentModel,
        ee: &str,
        placement: &PlacementOutcome,
        mass: f64,
        mu_support: f64,
        mu_tool: f64,
        size_m: f64,
    ) -> Result<FrozenMechanicsPrediction, String> {
        let m = placement
            .maneuver
            .as_ref()
            .ok_or_else(|| "geometry witness missing maneuver".to_string())?;
        let q = contact_chain_q(model, ee, m)?;
        let chain = model.ee_joint_chain(ee).ok_or("no ee chain")?;
        let fk = forward_kinematics(model, &chain, ee, &q).map_err(|e| format!("fk:{e:?}"))?;
        let contact_in_ee = tool_offset_in_ee(fk.ee.quat_wxyz, m.contact_point, fk.ee.xyz);
        let params = PlanarPushInitiation {
            mass_kg: Provenanced::user_declared(mass, "scenario.mass", 0.0),
            object_com_world: Provenanced::user_declared(m.object_center, "scenario.com", 0.0),
            gravity_m_s2: Provenanced::user_declared([0.0, 0.0, -9.80665], "scenario.g", 0.0),
            support_normal: Provenanced::user_declared([0.0, 0.0, 1.0], "scenario.support_n", 0.0),
            object_support_friction: PairFriction::coulomb(
                "object",
                "support",
                Provenanced::user_declared(mu_support, "scenario.mu_support", 0.0),
            ),
            tool_object_friction: PairFriction::coulomb(
                "tool",
                "object",
                Provenanced::user_declared(mu_tool, "scenario.mu_tool", 0.0),
            ),
            contact_point_world: Provenanced::user_declared(
                m.contact_point,
                "geometry.contact",
                0.0,
            ),
            contact_normal_world: Provenanced::user_declared(
                m.contact_normal,
                "geometry.normal",
                0.0,
            ),
            // Initiation force for the first family is along the contact
            // normal (into the face). Using a misaligned stroke direction
            // here would report tool-slip for every mass.
            push_direction_world: Provenanced::user_declared(
                m.contact_normal,
                "geometry.contact_normal_as_initiation_dir",
                0.0,
            ),
            joint_names: Vec::new(),
            translational_jacobian_3xn: Vec::new(),
            jacobian_residual: None,
            joint_effort_abs: Vec::new(),
            link_com_known: false,
            object_supported: true,
            approximately_planar: true,
            quasi_static: true,
            single_intended_contact: true,
            no_significant_impact: true,
            object_characteristic_length_m: Some(size_m),
        };
        let witness = evaluate_planar_push_at_model(model, ee, &q, contact_in_ee, params.clone())
            .map_err(|e| format!("evaluate_planar_push_at_model:{e:?}"))?;
        if witness.jacobian_residual.is_none() {
            return Err("shipped Jacobian residual missing".into());
        }
        if witness.available_lambda.is_none() && witness.feasibility == EffectFeasibility::Feasible
        {
            return Err("feasible initiation without a shipped λ bound".into());
        }
        Ok(FrozenMechanicsPrediction::freeze(witness, &params))
    }

    fn pose_xy(ep_pose: &serde_json::Value) -> Option<[f64; 2]> {
        let arr = ep_pose.as_array()?;
        Some([arr.first()?.as_f64()?, arr.get(1)?.as_f64()?])
    }

    fn disp_xy(a: [f64; 2], b: [f64; 2]) -> f64 {
        ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2)).sqrt()
    }

    fn near_ee_push_scenario(ee: [f64; 3], mass: f64, friction: f64) -> ManipulationScenario {
        let size = 0.04;
        let obj = [ee[0] + 0.055, ee[1], ee[2]];
        let table_z = (obj[2] - size - 0.01).max(0.02);
        ManipulationScenario {
            seed: 15,
            polarity: crate::manipulation_scenarios::Polarity::Positive,
            skill: "PUSH".into(),
            objects: vec![
                json!({"name":"table","type":"box","pos":[obj[0], obj[1], table_z],"size":[0.03,0.03,0.01],"mass":10.0,"movable":false}),
                json!({
                    "name":"obj0","type":"box","pos":obj,"size":[size,size,size],
                    "mass":mass,"friction":friction,"movable":true
                }),
                json!({"name":"obstacle","type":"box","pos":[8.0,8.0,-1.0],"size":[0.03,0.03,0.03],"mass":1.0,"movable":false}),
            ],
            planar: true,
            object_id: "obj0".into(),
            expected_refusal: None,
            neg: None,
            stale: false,
            move_object_after_obs: false,
            replay: false,
            restart_replay: false,
            crash_controller: false,
            immovable: false,
            push_dir: [1.0, 0.0, 0.0],
            push_dist: 0.04,
            required_opening: 0.8,
            world_construction:
                realityos_semantics::contact_maneuver::WorldConstructionMode::CapabilitySynthesis,
        }
    }

    #[test]
    fn predict_then_execute_same_geometry_different_physics() {
        if !ensure_mujoco_or_skip() {
            write_scratch(
                "mujoco-unavailable.log",
                "ensure_mujoco_or_skip() == false; oracle comparison not passed\n",
            );
            return;
        }
        let bundle = RobotBundle::load(corpus::robot_dir("arm_gripper")).expect("arm_gripper");
        let (probe, man) =
            load_and_normalize(&bundle, &crate::manipulation::template_objects(true), 0)
                .expect("load");
        let discovered =
            crate::resource_discover::discover_resources(&bundle, &man, &probe.inspect);
        crate::mujoco_exec::checkin_worker(probe);
        let mut qualified = Vec::new();
        for r in &discovered {
            if let Ok((_, q)) = crate::resource_qualify::qualify_resource(&bundle, r) {
                qualified.push(q);
            }
        }
        let mut model = embodiment_from_manifest(&bundle, &man);
        model.resources = qualified;
        let ee_name = bundle
            .manifest
            .end_effectors
            .first()
            .map(|e| e.name.clone())
            .unwrap_or_else(|| "ee".into());
        let chain = model.ee_joint_chain(&ee_name).expect("ee chain");
        let q_chain = sample_chain_q(&model, &chain, 21);
        let fk = forward_kinematics(&model, &chain, &ee_name, &q_chain).expect("fk");
        let ee = fk.ee.xyz;
        let mut qpos = vec![0.0; man.nq.max(0) as usize];
        for (name, qi) in chain.iter().zip(q_chain.iter()) {
            if let Some(j) = model.joints.iter().find(|j| j.name == *name) {
                if let Some(adr) = j.qpos_adr {
                    let adr = adr as usize;
                    if adr < qpos.len() {
                        qpos[adr] = *qi;
                    }
                }
            }
        }
        let sha = software_sha();
        let mu_tool = 0.8;
        let cases = [
            ("low_mass_low_mu", 0.02, 0.05),
            ("high_mass_high_mu", 4.0, 2.5),
        ];
        let mut rows = Vec::new();
        let mut outcomes = Vec::new();
        with_episode_qpos(Some(&qpos), || {
            for (name, mass, mu_s) in cases {
                let sc = near_ee_push_scenario(ee, mass, mu_s);
                let size = object_size_m(&sc);
                let object_id = sc.object_id.clone();
                let mut frozen: Option<FrozenMechanicsPrediction> = None;
                let mut start_xy: Option<[f64; 2]> = None;
                let mut hook = |placement: &PlacementOutcome,
                                initial: &VerifierTruth,
                                model: &EmbodimentModel| {
                    start_xy = body_xyz(initial, &object_id).map(|p| [p[0], p[1]]);
                    let mut mechanics_model = model.clone();
                    declare_joint_effort(&mut mechanics_model, 4.0);
                    frozen = Some(freeze_from_geometry_witness(
                        &mechanics_model,
                        &ee_name,
                        placement,
                        mass,
                        mu_s,
                        mu_tool,
                        size,
                    )?);
                    Ok(())
                };
                let loaded =
                    load_and_normalize(&bundle, &crate::manipulation::template_objects(true), 0)
                        .ok();
                let (ep, inst, _) = run_skill_episode_ex(
                    &bundle,
                    &model,
                    &[],
                    &sc,
                    &sha,
                    "PUSH",
                    loaded,
                    Some(&mut hook),
                )
                .unwrap_or_else(|e| panic!("{name}: {e}"));
                crate::mujoco_exec::checkin_worker(inst);
                let frozen = frozen.unwrap_or_else(|| {
                    panic!(
                        "{name}: not frozen before execute tax={:?} result={} rank={} writes={}",
                        ep.failure_taxonomy, ep.task_result, ep.selected_rank_why, ep.ctrl_writes
                    )
                });
                assert!(!frozen.contains_privileged_force());
                assert!(frozen.witness.jacobian_residual.is_some());
                assert_eq!(ep.unauthorized_writes, 0);
                let contact = ep.intended_tool_contact
                    || ep.contact_pose_reached
                    || ep.had_feasible_contact_maneuver
                    || ep.executed_witness_q;
                let end_xy = ep
                    .object_evidence
                    .get("pose")
                    .and_then(pose_xy)
                    .or(start_xy);
                let disp = match (start_xy, end_xy) {
                    (Some(a), Some(b)) => disp_xy(a, b),
                    _ => 0.0,
                };
                let authority_refused = ep.task_result == "authority_violation"
                    || ep.failure_taxonomy.as_deref() == Some("AUTHORITY_REFUSAL");
                let div = classify_mechanics_divergence(
                    &frozen,
                    contact,
                    disp > 0.005,
                    ep.unauthorized_writes,
                    ep.ctrl_writes,
                    authority_refused,
                );
                rows.push(json!({
                    "name": name,
                    "mass": mass,
                    "mu_support": mu_s,
                    "prediction": format!("{:?}", frozen.witness.feasibility),
                    "infeasible_reason": frozen.witness.infeasible_reason,
                    "unknown_reason": frozen.witness.unknown_reason,
                    "required_force_n": frozen.witness.required_force_n,
                    "available_lambda": frozen.witness.available_lambda,
                    "limiting_joint": frozen.witness.limiting_joint,
                    "jacobian_residual": frozen.witness.jacobian_residual,
                    "off_center_class": format!("{:?}", frozen.witness.off_center_class),
                    "privileged_force_in_predictor": frozen.contains_privileged_force(),
                    "task_result": ep.task_result,
                    "ctrl_writes": ep.ctrl_writes,
                    "unauthorized_writes": ep.unauthorized_writes,
                    "contact": contact,
                    "had_feasible_contact_maneuver": ep.had_feasible_contact_maneuver,
                    "executed_witness_q": ep.executed_witness_q,
                    "robot_driven_disp_m": disp,
                    "first_divergence": div.as_str(),
                    "failure_taxonomy": ep.failure_taxonomy,
                    "rank_why": ep.selected_rank_why,
                    "evidence_status": ep.evidence_status,
                }));
                outcomes.push((frozen.witness.feasibility, ep.ctrl_writes, contact, disp));
            }

            write_scratch(
                "push-mechanics-matrix.json",
                &serde_json::to_string_pretty(&json!({
                    "metal": false,
                    "evidence_status": SIMULATION_ONLY,
                    "oracle": "robot_driven_push",
                    "embodiment": "arm_gripper",
                    "geometry": "object_adjacent_home_ee",
                    "rows": rows,
                }))
                .unwrap(),
            );

            let low = &outcomes[0];
            let high = &outcomes[1];
            assert_ne!(
            low.0, high.0,
            "same geometry / different mass+friction must change the shipped prediction: {rows:?}"
        );
            assert!(
                low.1 > 0 && high.1 > 0,
                "robot must actuate both episodes: {rows:?}"
            );
            assert!(
                low.2 && high.2,
                "both episodes must run a contact maneuver: {rows:?}"
            );
            assert!(
                (low.3 - high.3).abs() > 1e-4,
                "robot-driven object motion must differ: low={} high={} {rows:?}",
                low.3,
                high.3
            );
        });
    }
}
