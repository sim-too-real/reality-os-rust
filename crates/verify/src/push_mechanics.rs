//! Planar-push mechanics prediction, freeze-before-execute, and first-divergence.
//! Privileged MuJoCo contact/actuator force is post-hoc only.

use serde::{Deserialize, Serialize};
use serde_json::Value;

use realityos_physics::{ContactMode, RotationSign, SupportFrictionModel};
use realityos_semantics::effect_feasibility::{
    evaluate_planar_push_initiation, EffectFeasibility, EffectFeasibilityWitness,
    PlanarPushInitiation,
};
use realityos_semantics::pair_friction::PairFriction;
use realityos_semantics::provenance::Provenanced;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum MechanicsFirstDivergence {
    RobotSelfLoadModelError,
    EffortBoundError,
    JacobianError,
    ContactFrameError,
    ObjectFrameError,
    PressureModelError,
    SupportWrenchModelError,
    PusherFrictionModelError,
    ContactModeError,
    TwistDirectionError,
    QuasiStaticAssumptionBroken,
    ControlTrackingError,
    ContactNotEstablished,
    AuthorityRefusal,
    ParameterUnknown,
    SimulatorModelMismatch,
    Unknown,
    None,
}

impl MechanicsFirstDivergence {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::RobotSelfLoadModelError => "ROBOT_SELF_LOAD_MODEL_ERROR",
            Self::EffortBoundError => "EFFORT_BOUND_ERROR",
            Self::JacobianError => "JACOBIAN_ERROR",
            Self::ContactFrameError => "CONTACT_FRAME_ERROR",
            Self::ObjectFrameError => "OBJECT_FRAME_ERROR",
            Self::PressureModelError => "PRESSURE_MODEL_ERROR",
            Self::SupportWrenchModelError => "SUPPORT_WRENCH_MODEL_ERROR",
            Self::PusherFrictionModelError => "PUSHER_FRICTION_MODEL_ERROR",
            Self::ContactModeError => "CONTACT_MODE_ERROR",
            Self::TwistDirectionError => "TWIST_DIRECTION_ERROR",
            Self::QuasiStaticAssumptionBroken => "QUASI_STATIC_ASSUMPTION_BROKEN",
            Self::ControlTrackingError => "CONTROL_TRACKING_ERROR",
            Self::ContactNotEstablished => "CONTACT_NOT_ESTABLISHED",
            Self::AuthorityRefusal => "AUTHORITY_REFUSAL",
            Self::ParameterUnknown => "PARAMETER_UNKNOWN",
            Self::SimulatorModelMismatch => "SIMULATOR_MODEL_MISMATCH",
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
            || s.contains("qfrc_bias")
            || s.contains("qfrc_gravcomp")
            || s.contains("PRIVILEGED")
    }
}

fn monotonic_ns() -> u128 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_nanos())
        .unwrap_or(0)
}

/// Independent short-horizon observation. Privileged contact force is not used.
#[derive(Debug, Clone, Default)]
pub struct MechanicsExecuteOutcome {
    pub contact_established: bool,
    pub unauthorized_writes: u64,
    pub ctrl_writes: u64,
    pub authority_refused: bool,
    pub translation_xy: Option<[f64; 2]>,
    pub yaw_change: Option<f64>,
    pub contact_mode: Option<ContactMode>,
    pub outcome_at_ns: u128,
    /// When set, overrides displacement detection from `translation_xy` magnitude.
    pub motion_detected: Option<bool>,
    /// Commanded local stroke (meters). Used to detect ballistic flight.
    pub commanded_stroke_m: Option<f64>,
}

/// Wrap an angle onto (−π, π].
pub fn principal_angle(rad: f64) -> f64 {
    if !rad.is_finite() {
        return rad;
    }
    let two_pi = std::f64::consts::PI * 2.0;
    let mut x = rad.rem_euclid(two_pi);
    if x > std::f64::consts::PI {
        x -= two_pi;
    }
    x
}

/// Signed yaw change wrapped onto (−π, π].
pub fn principal_yaw_delta(start: f64, end: f64) -> f64 {
    principal_angle(end - start)
}

/// Halt the local observation window after first contact + a short post-contact
/// interval, or as soon as displacement exceeds the commanded stroke.
pub fn local_horizon_stop(
    saw_contact: bool,
    disp_m: f64,
    stroke_m: f64,
    post_contact_steps: u32,
    max_post: u32,
) -> bool {
    disp_m > stroke_m || (saw_contact && post_contact_steps >= max_post)
}

pub fn observed_rotation_sign(yaw_change: f64, translation_xy: [f64; 2]) -> RotationSign {
    let yaw_change = principal_angle(yaw_change);
    let t = (translation_xy[0] * translation_xy[0] + translation_xy[1] * translation_xy[1]).sqrt();
    if !yaw_change.is_finite() {
        return RotationSign::Unknown;
    }
    if yaw_change.abs() < 1e-4 && t < 1e-4 {
        return RotationSign::Unknown;
    }
    if yaw_change.abs() * 0.05 < 0.2 * t.max(1e-9) {
        return RotationSign::TranslationOnly;
    }
    if yaw_change > 0.0 {
        RotationSign::Counterclockwise
    } else {
        RotationSign::Clockwise
    }
}

fn hypot2(v: [f64; 2]) -> f64 {
    (v[0] * v[0] + v[1] * v[1]).sqrt()
}

fn unit2(v: [f64; 2]) -> Option<[f64; 2]> {
    let n = hypot2(v);
    if n < 1e-12 {
        None
    } else {
        Some([v[0] / n, v[1] / n])
    }
}

/// Angle in radians between frozen planar translation and observed Δxy.
pub fn translation_direction_error(frozen_xy: [f64; 2], observed_xy: [f64; 2]) -> Option<f64> {
    let a = unit2(frozen_xy)?;
    let b = unit2(observed_xy)?;
    let cos = (a[0] * b[0] + a[1] * b[1]).clamp(-1.0, 1.0);
    Some(cos.acos())
}

fn confident_sign(s: RotationSign) -> bool {
    matches!(
        s,
        RotationSign::Clockwise | RotationSign::Counterclockwise | RotationSign::TranslationOnly
    )
}

fn confident_mode(m: ContactMode) -> bool {
    matches!(
        m,
        ContactMode::Sticking
            | ContactMode::SlidingLeft
            | ContactMode::SlidingRight
            | ContactMode::Separating
    )
}

fn frozen_world_translation(frozen: &FrozenMechanicsPrediction) -> Option<[f64; 2]> {
    let tw = frozen.witness.planar_twist?;
    let yaw = frozen
        .predictor_input
        .get("object_yaw_rad")
        .and_then(|v| v.get("value"))
        .and_then(|v| v.as_f64())
        .unwrap_or(0.0);
    let c = yaw.cos();
    let s = yaw.sin();
    Some([c * tw.vx - s * tw.vy, s * tw.vx + c * tw.vy])
}

fn object_displaced(outcome: &MechanicsExecuteOutcome) -> bool {
    if let Some(m) = outcome.motion_detected {
        return m;
    }
    outcome.translation_xy.is_some_and(|d| hypot2(d) > 5e-4)
}

/// Compare a frozen PLANAR_TWIST_DIRECTION claim to independent short-horizon motion.
pub fn classify_mechanics_execute(
    frozen: &FrozenMechanicsPrediction,
    outcome: &MechanicsExecuteOutcome,
) -> MechanicsFirstDivergence {
    if outcome.unauthorized_writes > 0 {
        return MechanicsFirstDivergence::Unknown;
    }
    if outcome.authority_refused
        || (outcome.ctrl_writes == 0 && frozen.witness.feasibility != EffectFeasibility::Unknown)
    {
        if matches!(
            frozen.witness.unknown_reason.as_deref(),
            Some(r) if r.contains("PARAMETER_MISSING")
        ) {
            return MechanicsFirstDivergence::ParameterUnknown;
        }
        if outcome.authority_refused {
            return MechanicsFirstDivergence::AuthorityRefusal;
        }
    }
    match frozen.witness.feasibility {
        EffectFeasibility::Unknown => {
            let r = frozen.witness.unknown_reason.as_deref().unwrap_or("");
            if r.contains("PARAMETER_MISSING") {
                MechanicsFirstDivergence::ParameterUnknown
            } else if r == "MODEL_NOT_APPLICABLE" {
                MechanicsFirstDivergence::QuasiStaticAssumptionBroken
            } else if r.contains("JACOBIAN") || r == "NEAR_SINGULAR_JACOBIAN" {
                MechanicsFirstDivergence::JacobianError
            } else if r.contains("EFFORT") || r.contains("SELF_LOAD") {
                MechanicsFirstDivergence::EffortBoundError
            } else if r.contains("PRESSURE") {
                MechanicsFirstDivergence::PressureModelError
            } else if r.contains("SUPPORT") {
                MechanicsFirstDivergence::SupportWrenchModelError
            } else {
                MechanicsFirstDivergence::Unknown
            }
        }
        EffectFeasibility::Infeasible => {
            if frozen.witness.infeasible_reason.as_deref() == Some("TOOL_CONTACT_SLIP") {
                MechanicsFirstDivergence::PusherFrictionModelError
            } else if object_displaced(outcome) {
                MechanicsFirstDivergence::SimulatorModelMismatch
            } else if !outcome.contact_established {
                MechanicsFirstDivergence::ContactNotEstablished
            } else {
                MechanicsFirstDivergence::EffortBoundError
            }
        }
        EffectFeasibility::Feasible => {
            if !outcome.contact_established {
                return MechanicsFirstDivergence::ContactNotEstablished;
            }
            if !object_displaced(outcome) {
                return if frozen
                    .witness
                    .support_friction_mu
                    .is_some_and(|mu| mu > 1.0)
                {
                    MechanicsFirstDivergence::SupportWrenchModelError
                } else {
                    MechanicsFirstDivergence::EffortBoundError
                };
            }
            if quasi_static_assumption_broken(outcome) {
                return MechanicsFirstDivergence::QuasiStaticAssumptionBroken;
            }
            if let (Some(claimed), Some(obs_mode)) =
                (frozen.witness.contact_mode, outcome.contact_mode)
            {
                if confident_mode(claimed) && confident_mode(obs_mode) && claimed != obs_mode {
                    return MechanicsFirstDivergence::ContactModeError;
                }
            }
            let claimed_sign = frozen.witness.rotation_sign;
            let obs_sign = match (outcome.yaw_change, outcome.translation_xy) {
                (Some(dyaw), Some(xy)) => Some(observed_rotation_sign(dyaw, xy)),
                _ => None,
            };
            if let (Some(claimed), Some(obs)) = (claimed_sign, obs_sign) {
                if confident_sign(claimed) && confident_sign(obs) && claimed != obs {
                    return MechanicsFirstDivergence::TwistDirectionError;
                }
            }
            if let (Some(fxy), Some(oxy)) =
                (frozen_world_translation(frozen), outcome.translation_xy)
            {
                if let Some(err) = translation_direction_error(fxy, oxy) {
                    if err > std::f64::consts::FRAC_PI_3 {
                        return MechanicsFirstDivergence::TwistDirectionError;
                    }
                }
            } else if (claimed_sign.is_some_and(confident_sign)
                || frozen.witness.planar_twist.is_some())
                && outcome.translation_xy.is_none()
                && outcome.yaw_change.is_none()
            {
                return MechanicsFirstDivergence::Unknown;
            }
            MechanicsFirstDivergence::None
        }
    }
}

pub fn classify_mechanics_divergence(
    frozen: &FrozenMechanicsPrediction,
    contact_established: bool,
    object_displaced: bool,
    unauthorized_writes: u64,
    ctrl_writes: u64,
    authority_refused: bool,
) -> MechanicsFirstDivergence {
    classify_mechanics_execute(
        frozen,
        &MechanicsExecuteOutcome {
            contact_established,
            unauthorized_writes,
            ctrl_writes,
            authority_refused,
            translation_xy: None,
            yaw_change: None,
            contact_mode: None,
            outcome_at_ns: 0,
            motion_detected: Some(object_displaced),
            commanded_stroke_m: None,
        },
    )
}

fn quasi_static_assumption_broken(outcome: &MechanicsExecuteOutcome) -> bool {
    let dyaw = outcome.yaw_change.map(principal_angle).unwrap_or(0.0).abs();
    if dyaw > std::f64::consts::FRAC_PI_2 {
        return true;
    }
    match (outcome.commanded_stroke_m, outcome.translation_xy) {
        (Some(stroke), Some(d)) => hypot2(d) > stroke,
        _ => false,
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
    let (tmin, tmax, self_load) = match tau {
        Some(t) => (
            vec![Provenanced::user_declared(-t, "scenario.effort_min", 0.0); 3],
            vec![Provenanced::user_declared(t, "scenario.effort_max", 0.0); 3],
            vec![Provenanced::user_declared(0.0, "scenario.self_load", 0.0); 3],
        ),
        None => (Vec::new(), Vec::new(), Vec::new()),
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
        contact_force_direction_world: Provenanced::user_declared(
            direction,
            "scenario.force_dir",
            0.0,
        ),
        pusher_velocity_world: Provenanced::user_declared(direction, "scenario.vp", 0.0),
        joint_names: vec!["j0".into(), "j1".into(), "j2".into()],
        translational_jacobian_3xn: identity_translational_jacobian(),
        jacobian_residual: Some(0.0),
        joint_effort_abs: vec![tau_p.clone(), tau_p.clone(), tau_p],
        joint_effort_min: tmin,
        joint_effort_max: tmax,
        self_load_torque_nm: self_load,
        link_com_known: false,
        object_supported: true,
        approximately_planar: true,
        quasi_static: true,
        single_intended_contact: true,
        no_significant_impact: true,
        object_characteristic_length_m: Some(0.05),
        support_friction_model: SupportFrictionModel::Unknown,
        object_yaw_rad: Provenanced::user_declared(0.0, "scenario.yaw", 0.0),
        stale_object_evidence: false,
        intended_contact_lost: false,
        authority_ok: true,
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
    use realityos_physics::PressureDistribution;
    use realityos_semantics::adapter::synth_planar_two_link;
    use realityos_semantics::contact_jacobian::contact_jacobian_witness;
    use realityos_semantics::contact_maneuver::{tool_offset_in_ee, ContactManeuver};
    use realityos_semantics::effect_feasibility::evaluate_planar_push_at_model;
    use realityos_semantics::effect_feasibility::{EffectClass, EffortBoundKind};
    use realityos_semantics::effort::physical_joint_effort;
    use realityos_semantics::embodiment::{Actuator, EmbodimentModel, JointKind};
    use realityos_semantics::grasp_hold::{evaluate_pinch_hold, HoldFeasibility, PinchHoldInput};
    use realityos_semantics::kinematics::{forward_kinematics, solve_ik};
    use realityos_semantics::physical_quantity::PhysicalEffort;
    use realityos_semantics::provenance::Provenanced;
    use serde_json::json;

    fn scratch_dir() -> std::path::PathBuf {
        std::path::PathBuf::from(
            r"C:\Users\moram\AppData\Local\Temp\grok-goal-3f834bc45751\implementer",
        )
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
            EffortBoundKind::AvailableContactEffortBound
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

    fn declared_ellipsoidal_push(y_off: f64, pusher_vel: [f64; 3]) -> PlanarPushInitiation {
        let mut p = declared_centered_push(
            Some(0.2),
            Some(0.3),
            Some(0.8),
            Some(20.0),
            [0.22, y_off, 0.03],
            [0.22, 0.0, 0.03],
            [1.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
        );
        p.support_friction_model = SupportFrictionModel::Ellipsoidal {
            f_max: 0.3 * 0.2 * 9.80665,
            tau_max: 0.3 * 0.2 * 9.80665 * (2.0 / 3.0) * 0.05,
            pressure: PressureDistribution::DeclaredUniform,
        };
        p.pusher_velocity_world = Provenanced::user_declared(pusher_vel, "scenario.vp", 0.0);
        p
    }

    #[test]
    fn classifier_twist_sign_disagreement_is_twist_direction_error() {
        use realityos_semantics::effect_feasibility::evaluate_planar_twist_direction;
        let input = declared_ellipsoidal_push(0.03, [1.0, 0.0, 0.0]);
        let w = evaluate_planar_twist_direction(&input);
        assert_eq!(w.rotation_sign, Some(RotationSign::Clockwise));
        let frozen = FrozenMechanicsPrediction::freeze(w, &input);
        let mut outcome = MechanicsExecuteOutcome {
            contact_established: true,
            unauthorized_writes: 0,
            ctrl_writes: 1,
            authority_refused: false,
            translation_xy: Some([0.004, 0.0]),
            yaw_change: Some(0.08),
            contact_mode: None,
            outcome_at_ns: frozen.frozen_at_ns + 1,
            motion_detected: Some(true),
            commanded_stroke_m: Some(0.055),
        };
        assert_eq!(
            classify_mechanics_execute(&frozen, &outcome),
            MechanicsFirstDivergence::TwistDirectionError
        );
        outcome.yaw_change = Some(-0.08);
        assert_eq!(
            classify_mechanics_execute(&frozen, &outcome),
            MechanicsFirstDivergence::None
        );
        assert!(frozen.frozen_at_ns < outcome.outcome_at_ns);
    }

    #[test]
    fn classifier_mode_disagreement_is_contact_mode_error() {
        use realityos_semantics::effect_feasibility::evaluate_planar_twist_direction;
        let input = declared_ellipsoidal_push(0.0, [1.0, 0.0, 0.0]);
        let w = evaluate_planar_twist_direction(&input);
        assert_eq!(w.contact_mode, Some(ContactMode::Sticking));
        let frozen = FrozenMechanicsPrediction::freeze(w, &input);
        let outcome = MechanicsExecuteOutcome {
            contact_established: true,
            unauthorized_writes: 0,
            ctrl_writes: 1,
            authority_refused: false,
            translation_xy: Some([0.005, 0.0]),
            yaw_change: Some(0.0),
            contact_mode: Some(ContactMode::SlidingLeft),
            outcome_at_ns: frozen.frozen_at_ns + 1,
            motion_detected: Some(true),
            commanded_stroke_m: Some(0.055),
        };
        assert_eq!(
            classify_mechanics_execute(&frozen, &outcome),
            MechanicsFirstDivergence::ContactModeError
        );
    }

    #[test]
    fn classifier_opposite_translation_is_twist_direction_error() {
        use realityos_semantics::effect_feasibility::evaluate_planar_twist_direction;
        let input = declared_ellipsoidal_push(0.0, [1.0, 0.0, 0.0]);
        let w = evaluate_planar_twist_direction(&input);
        assert!(w.planar_twist.is_some_and(|t| t.vx > 0.0));
        let frozen = FrozenMechanicsPrediction::freeze(w, &input);
        let outcome = MechanicsExecuteOutcome {
            contact_established: true,
            unauthorized_writes: 0,
            ctrl_writes: 1,
            authority_refused: false,
            translation_xy: Some([-0.006, 0.0]),
            yaw_change: Some(0.0),
            contact_mode: None,
            outcome_at_ns: frozen.frozen_at_ns + 1,
            motion_detected: Some(true),
            commanded_stroke_m: Some(0.055),
        };
        assert_eq!(
            classify_mechanics_execute(&frozen, &outcome),
            MechanicsFirstDivergence::TwistDirectionError
        );
    }

    #[test]
    fn principal_yaw_delta_wraps_near_two_pi() {
        let d = principal_yaw_delta(0.0, 2.0 * std::f64::consts::PI - 0.1);
        assert!(
            (d + 0.1).abs() < 1e-12,
            "0 vs 2π−0.1 must wrap to ≈−0.1, got {d}"
        );
        assert!(d > -std::f64::consts::PI && d <= std::f64::consts::PI);
        assert!((principal_yaw_delta(0.1, 0.1)).abs() < 1e-15);
        assert!((principal_angle(-std::f64::consts::PI) - std::f64::consts::PI).abs() < 1e-12);
    }

    #[test]
    fn local_horizon_stop_after_contact_or_stroke() {
        assert!(!local_horizon_stop(false, 0.01, 0.055, 0, 25));
        assert!(local_horizon_stop(false, 0.06, 0.055, 0, 25));
        assert!(!local_horizon_stop(true, 0.01, 0.055, 24, 25));
        assert!(local_horizon_stop(true, 0.01, 0.055, 25, 25));
        assert!(local_horizon_stop(true, 0.08, 0.055, 1, 25));
    }

    #[test]
    fn ballistic_flight_is_quasi_static_assumption_broken() {
        use realityos_semantics::effect_feasibility::evaluate_planar_twist_direction;
        let input = declared_ellipsoidal_push(0.0, [1.0, 0.0, 0.0]);
        let w = evaluate_planar_twist_direction(&input);
        assert_eq!(w.contact_mode, Some(ContactMode::Sticking));
        let frozen = FrozenMechanicsPrediction::freeze(w, &input);
        let outcome = MechanicsExecuteOutcome {
            contact_established: true,
            unauthorized_writes: 0,
            ctrl_writes: 1,
            authority_refused: false,
            translation_xy: Some([0.4660165018394705, 0.035343042656063584]),
            yaw_change: Some(-5.281851420276709),
            contact_mode: Some(ContactMode::SlidingRight),
            outcome_at_ns: frozen.frozen_at_ns + 1,
            motion_detected: Some(true),
            commanded_stroke_m: Some(0.055),
        };
        let div = classify_mechanics_execute(&frozen, &outcome);
        assert_eq!(div, MechanicsFirstDivergence::QuasiStaticAssumptionBroken);
        assert_ne!(div, MechanicsFirstDivergence::ContactModeError);
        assert_ne!(div, MechanicsFirstDivergence::TwistDirectionError);
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
        pusher_velocity: [f64; 3],
        contact_y_shift: f64,
    ) -> Result<FrozenMechanicsPrediction, String> {
        let m = placement
            .maneuver
            .as_ref()
            .ok_or_else(|| "geometry witness missing maneuver".to_string())?;
        let q = contact_chain_q(model, ee, m)?;
        let chain = model.ee_joint_chain(ee).ok_or("no ee chain")?;
        let fk = forward_kinematics(model, &chain, ee, &q).map_err(|e| format!("fk:{e:?}"))?;
        // Face inward (into the object), not the line through COM — a COM-aimed
        // force has zero moment and cannot claim off-center rotation.
        let mut into_object = m.contact_normal;
        let toward = [
            m.object_center[0] - m.contact_point[0],
            m.object_center[1] - m.contact_point[1],
            0.0,
        ];
        if toward[0] * into_object[0] + toward[1] * into_object[1] < 0.0 {
            into_object = [-into_object[0], -into_object[1], -into_object[2]];
        }
        let n_into = (into_object[0] * into_object[0]
            + into_object[1] * into_object[1]
            + into_object[2] * into_object[2])
            .sqrt();
        if n_into > 1e-9 {
            into_object = [
                into_object[0] / n_into,
                into_object[1] / n_into,
                into_object[2] / n_into,
            ];
        } else {
            into_object = [1.0, 0.0, 0.0];
        }
        let mut t_face = [-into_object[1], into_object[0], 0.0];
        let t_n = (t_face[0] * t_face[0] + t_face[1] * t_face[1]).sqrt();
        if t_n > 1e-9 {
            t_face = [t_face[0] / t_n, t_face[1] / t_n, 0.0];
        }
        let half = size_m.max(1e-4);
        let contact_on_face = [
            m.object_center[0] - half * into_object[0] + contact_y_shift * t_face[0],
            m.object_center[1] - half * into_object[1] + contact_y_shift * t_face[1],
            m.object_center[2],
        ];
        let contact_in_ee = tool_offset_in_ee(fk.ee.quat_wxyz, contact_on_face, fk.ee.xyz);
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
                contact_on_face,
                "geometry.contact",
                0.0,
            ),
            contact_normal_world: Provenanced::user_declared(
                into_object,
                "geometry.normal_into_object",
                0.0,
            ),
            push_direction_world: Provenanced::user_declared(
                into_object,
                "geometry.push_into_object",
                0.0,
            ),
            contact_force_direction_world: Provenanced::user_declared(
                into_object,
                "geometry.force_dir",
                0.0,
            ),
            pusher_velocity_world: Provenanced::user_declared(
                pusher_velocity,
                "geometry.pusher_vel",
                0.0,
            ),
            joint_names: Vec::new(),
            translational_jacobian_3xn: Vec::new(),
            jacobian_residual: None,
            joint_effort_abs: Vec::new(),
            joint_effort_min: Vec::new(),
            joint_effort_max: Vec::new(),
            self_load_torque_nm: Vec::new(),
            link_com_known: false,
            object_supported: true,
            approximately_planar: true,
            quasi_static: true,
            single_intended_contact: true,
            no_significant_impact: true,
            object_characteristic_length_m: Some(size_m),
            support_friction_model: {
                let n_load = mass * 9.80665;
                let f_max = mu_support * n_load;
                SupportFrictionModel::Ellipsoidal {
                    f_max,
                    tau_max: f_max * (2.0 / 3.0) * size_m.max(1e-4),
                    pressure: PressureDistribution::DeclaredUniform,
                }
            },
            object_yaw_rad: Provenanced::user_declared(0.0, "geometry.yaw", 0.0),
            stale_object_evidence: false,
            intended_contact_lost: false,
            authority_ok: true,
        };
        let mut witness =
            evaluate_planar_push_at_model(model, ee, &q, contact_in_ee, params.clone())
                .map_err(|e| format!("evaluate_planar_push_at_model:{e:?}"))?;
        if witness.jacobian_residual.is_none() {
            return Err("shipped Jacobian residual missing".into());
        }
        if witness.available_lambda.is_none() && witness.feasibility == EffectFeasibility::Feasible
        {
            return Err("feasible initiation without a shipped λ bound".into());
        }
        if witness.planar_twist.is_some() || witness.contact_mode.is_some() {
            witness.effect_class =
                realityos_semantics::effect_feasibility::EffectClass::PlanarTwistDirection;
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
        near_ee_push_scenario_offset(ee, mass, friction, 0.0)
    }

    fn near_ee_push_scenario_offset(
        ee: [f64; 3],
        mass: f64,
        friction: f64,
        y_off: f64,
    ) -> ManipulationScenario {
        let size = 0.04;
        let obj = [ee[0] + 0.055, ee[1] + y_off, ee[2]];
        let table_z = (obj[2] - size - 0.01).max(0.02);
        ManipulationScenario {
            seed: 15,
            polarity: crate::manipulation_scenarios::Polarity::Positive,
            skill: "PUSH".into(),
            objects: vec![
                json!({"name":"table","type":"box","pos":[obj[0], obj[1], table_z],"size":[0.12,0.12,0.01],"mass":10.0,"movable":false}),
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
                let mut hook =
                    |placement: &PlacementOutcome,
                     initial: &VerifierTruth,
                     model: &EmbodimentModel,
                     _inst: &mut crate::mujoco_exec::MujocoInstance| {
                        start_xy = body_xyz(initial, &object_id).map(|p| [p[0], p[1]]);
                        let mut mechanics_model = model.clone();
                        declare_joint_effort(&mut mechanics_model, 4.0);
                        let vp = placement
                            .maneuver
                            .as_ref()
                            .map(|m| m.contact_normal)
                            .unwrap_or([1.0, 0.0, 0.0]);
                        frozen = Some(freeze_from_geometry_witness(
                            &mechanics_model,
                            &ee_name,
                            placement,
                            mass,
                            mu_s,
                            mu_tool,
                            size,
                            vp,
                            0.0,
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
                let dxy = match (start_xy, end_xy) {
                    (Some(a), Some(b)) => Some([b[0] - a[0], b[1] - a[1]]),
                    _ => None,
                };
                let div = classify_mechanics_execute(
                    &frozen,
                    &MechanicsExecuteOutcome {
                        contact_established: contact,
                        unauthorized_writes: ep.unauthorized_writes,
                        ctrl_writes: ep.ctrl_writes,
                        authority_refused,
                        translation_xy: dxy,
                        yaw_change: None,
                        contact_mode: None,
                        outcome_at_ns: 0,
                        motion_detected: Some(disp > 0.005),
                        commanded_stroke_m: None,
                    },
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

    #[test]
    fn mechanics_v2_analytic_freeze_and_invariants() {
        use realityos_physics::{ContactMode, RotationSign};
        use realityos_semantics::effect_feasibility::{
            evaluate_planar_twist_direction, evaluate_sustained_effect, EffectClass,
            EffortBoundKind,
        };

        let centered = declared_centered_push(
            Some(0.05),
            Some(0.1),
            Some(1.0),
            Some(20.0),
            [0.22, 0.0, 0.03],
            [0.22, 0.0, 0.03],
            [1.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
        );
        let mut with_ls = centered.clone();
        with_ls.support_friction_model = SupportFrictionModel::Ellipsoidal {
            f_max: 0.1 * 0.05 * 9.80665,
            tau_max: 0.1 * 0.05 * 9.80665 * (2.0 / 3.0) * 0.05,
            pressure: PressureDistribution::DeclaredUniform,
        };
        let f =
            FrozenMechanicsPrediction::freeze(evaluate_planar_twist_direction(&with_ls), &with_ls);
        assert!(!f.contains_privileged_force());
        assert_eq!(
            f.witness.effort_bound_kind,
            EffortBoundKind::AvailableContactEffortBound
        );
        assert_eq!(f.witness.feasibility, EffectFeasibility::Feasible);
        assert_ne!(
            f.witness.physical_levels.sustained_effect,
            EffectFeasibility::Feasible
        );

        let mut tan = with_ls.clone();
        tan.pusher_velocity_world = Provenanced::user_declared([1.0, 3.0, 0.0], "scenario.vp", 0.0);
        let tan_w = evaluate_planar_twist_direction(&tan);
        assert_ne!(tan_w.pusher_velocity, tan_w.contact_force_direction);
        assert_ne!(tan_w.contact_mode, Some(ContactMode::Sticking));

        let mut pos = with_ls.clone();
        pos.contact_point_world =
            Provenanced::user_declared([0.22, 0.03, 0.03], "scenario.contact", 0.0);
        let mut neg = pos.clone();
        neg.contact_point_world =
            Provenanced::user_declared([0.22, -0.03, 0.03], "scenario.contact", 0.0);
        let a = evaluate_planar_twist_direction(&pos);
        let b = evaluate_planar_twist_direction(&neg);
        assert_eq!(a.rotation_sign, Some(RotationSign::Clockwise));
        assert_eq!(b.rotation_sign, Some(RotationSign::Counterclockwise));

        let mut q1 = with_ls.clone();
        q1.mass_kg = Provenanced::user_declared(4.0, "scenario.mass", 0.0);
        q1.object_support_friction = PairFriction::coulomb(
            "object",
            "support",
            Provenanced::user_declared(2.5, "scenario.mu_support", 0.0),
        );
        q1.support_friction_model = SupportFrictionModel::Ellipsoidal {
            f_max: 2.5 * 4.0 * 9.80665,
            tau_max: 2.5 * 4.0 * 9.80665 * (2.0 / 3.0) * 0.05,
            pressure: PressureDistribution::DeclaredUniform,
        };
        let sus = evaluate_sustained_effect(&[with_ls.clone(), q1]);
        assert_ne!(sus.feasibility, EffectFeasibility::Feasible);
        assert_eq!(sus.first_infeasible_checkpoint, Some(1));
        assert_eq!(sus.effect_class, EffectClass::SustainedEffect);

        let mut unk_p = with_ls.clone();
        unk_p.support_friction_model = SupportFrictionModel::Ellipsoidal {
            f_max: 1.0,
            tau_max: 0.05,
            pressure: PressureDistribution::Unknown,
        };
        unk_p.contact_point_world =
            Provenanced::user_declared([0.22, 0.03, 0.03], "scenario.contact", 0.0);
        let unk = evaluate_planar_twist_direction(&unk_p);
        assert_eq!(unk.pressure_model_kind.as_deref(), Some("UNKNOWN"));
        assert_ne!(unk.rotation_sign, Some(RotationSign::Clockwise));

        let fa = FrozenMechanicsPrediction::freeze(a.clone(), &pos);
        let fb = FrozenMechanicsPrediction::freeze(b.clone(), &neg);
        assert!(!fa.contains_privileged_force() && !fb.contains_privileged_force());
        assert!(fa.frozen_at_ns > 0 && fb.frozen_at_ns > 0);

        let doc = json!({
            "metal": false,
            "evidence_status": SIMULATION_ONLY,
            "analytic": {
                "initiation_feasible": format!("{:?}", f.witness.feasibility),
                "effort_bound_kind": format!("{:?}", f.witness.effort_bound_kind),
                "tangential_mode": format!("{:?}", tan_w.contact_mode),
                "force_dir": tan_w.contact_force_direction,
                "pusher_vel": tan_w.pusher_velocity,
                "mirrored_plus_sign": format!("{:?}", a.rotation_sign),
                "mirrored_minus_sign": format!("{:?}", b.rotation_sign),
                "unknown_pressure_sign": format!("{:?}", unk.rotation_sign),
                "sustained_first_infeasible": sus.first_infeasible_checkpoint,
                "privileged_force_in_predictor": f.contains_privileged_force(),
                "unauthorized_writes": 0,
            }
        });
        write_scratch(
            "push-mechanics-v2.json",
            &serde_json::to_string_pretty(&doc).unwrap(),
        );
    }

    fn yaw_wxyz(q: &[f64]) -> Option<f64> {
        if q.len() < 4 {
            return None;
        }
        Some(2.0 * q[3].atan2(q[0]))
    }

    fn truth_now(inst: &mut crate::mujoco_exec::MujocoInstance) -> Result<VerifierTruth, String> {
        let st = inst.step(0).map_err(|e| e.to_string())?;
        Ok(VerifierTruth::from_mujoco_state(
            st.get("state").unwrap_or(&st),
        ))
    }

    fn apply_named_q(
        inst: &mut crate::mujoco_exec::MujocoInstance,
        model: &EmbodimentModel,
        names: &[String],
        q: &[f64],
        current_qpos: &[f64],
    ) -> Result<(), String> {
        use realityos_semantics::adapter::{compile_named_joint_q, lower_named_targets};
        use std::collections::HashMap;
        let compiled = compile_named_joint_q(model, names, q, "skill.push")
            .map_err(|e| format!("compile:{e:?}"))?;
        let mut current = HashMap::new();
        for j in &model.joints {
            if let Some(adr) = j.qpos_adr {
                if let Some(&v) = current_qpos.get(adr as usize) {
                    current.insert(j.name.clone(), v);
                }
            }
        }
        let lowered = lower_named_targets(model, &compiled.targets, &current)
            .map_err(|e| format!("lower:{e:?}"))?;
        let mut ctrl = vec![0.0; model.actuators.len().max(1)];
        for (i, act) in model.actuators.iter().enumerate() {
            if i >= ctrl.len() {
                break;
            }
            if let Some((_, v)) = lowered.iter().find(|(n, _)| n == &act.name) {
                ctrl[i] = *v;
            } else if let Some(&v) = current.get(&act.target_joint) {
                ctrl[i] = v;
            }
        }
        inst.set_ctrl(&ctrl).map_err(|e| e.to_string())?;
        Ok(())
    }

    fn infer_observed_mode(
        pusher_xy: [f64; 2],
        n_xy: [f64; 2],
        object_v_xy: [f64; 2],
        omega: f64,
        r_xy: [f64; 2],
    ) -> Option<ContactMode> {
        let pn = (n_xy[0] * n_xy[0] + n_xy[1] * n_xy[1]).sqrt();
        let vp_n = (pusher_xy[0] * pusher_xy[0] + pusher_xy[1] * pusher_xy[1]).sqrt();
        if pn < 1e-9 || vp_n < 1e-9 {
            return None;
        }
        let n = [n_xy[0] / pn, n_xy[1] / pn];
        let vp_hat = [pusher_xy[0] / vp_n, pusher_xy[1] / vp_n];
        let vc = [
            object_v_xy[0] - omega * r_xy[1],
            object_v_xy[1] + omega * r_xy[0],
        ];
        let into = vp_hat[0] * n[0] + vp_hat[1] * n[1];
        if into < -1e-4 {
            return Some(ContactMode::Separating);
        }
        let vc_n = (vc[0] * vc[0] + vc[1] * vc[1]).sqrt();
        if vc_n < 1e-4 {
            return Some(ContactMode::Sticking);
        }
        let vc_hat = [vc[0] / vc_n, vc[1] / vc_n];
        let rel = [vp_hat[0] - vc_hat[0], vp_hat[1] - vc_hat[1]];
        let t = [-n[1], n[0]];
        let rel_t = rel[0] * t[0] + rel[1] * t[1];
        if rel_t.abs() < 0.35 {
            Some(ContactMode::Sticking)
        } else if rel_t > 0.0 {
            Some(ContactMode::SlidingLeft)
        } else {
            Some(ContactMode::SlidingRight)
        }
    }

    #[test]
    fn predict_then_execute_v2_short_horizon() {
        if !ensure_mujoco_or_skip() {
            write_scratch(
                "mujoco-unavailable.log",
                "ensure_mujoco_or_skip() == false; v2 short-horizon oracle not passed\n",
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
        let mut qpos0 = vec![0.0; man.nq.max(0) as usize];
        for (name, qi) in chain.iter().zip(q_chain.iter()) {
            if let Some(j) = model.joints.iter().find(|j| j.name == *name) {
                if let Some(adr) = j.qpos_adr {
                    if let Some(slot) = qpos0.get_mut(adr as usize) {
                        *slot = *qi;
                    }
                }
            }
        }
        let sha = software_sha();
        let cases: [(&str, f64, [f64; 3], bool); 4] = [
            ("centered_normal", 0.0, [1.0, 0.0, 0.0], false),
            ("tangential_vp", 0.0, [1.0, 3.0, 0.0], true),
            ("offset_plus", 0.02, [1.0, 0.0, 0.0], false),
            ("offset_minus", -0.02, [1.0, 0.0, 0.0], false),
        ];
        let mut rows = Vec::new();
        with_episode_qpos(Some(&qpos0), || {
            for (name, y_off, pusher_vel, tangential) in cases {
                // Same object COM for every case. Offset is contact vs COM, not a scene shift.
                let sc = near_ee_push_scenario(ee, 0.02, 0.08);
                let size = object_size_m(&sc);
                let object_id = sc.object_id.clone();
                let mut frozen: Option<FrozenMechanicsPrediction> = None;
                let mut outcome = MechanicsExecuteOutcome::default();
                let mut disp = 0.0_f64;
                let mut start_xy_rec: Option<[f64; 2]> = None;
                let mut obs_sign: Option<RotationSign> = None;
                let mut dir_err: Option<f64> = None;
                let mut q_cmd: Vec<f64> = Vec::new();
                let mut t0_q: Vec<f64> = Vec::new();
                let mut ee_dxy_rec: Option<[f64; 2]> = None;
                let mut r_xy_rec: Option<[f64; 2]> = None;
                let mut contact_shifted_rec: Option<[f64; 3]> = None;
                let mut t0_contact_rec = false;
                let mut t1_contact_rec = false;
                let mut horizon_s_rec = 0.0_f64;
                let mut hook =
                    |placement: &PlacementOutcome,
                     initial: &VerifierTruth,
                     model: &EmbodimentModel,
                     inst: &mut crate::mujoco_exec::MujocoInstance| {
                        let mut mechanics_model = model.clone();
                        declare_joint_effort(&mut mechanics_model, 4.0);
                        let fr = freeze_from_geometry_witness(
                            &mechanics_model,
                            &ee_name,
                            placement,
                            0.02,
                            0.08,
                            0.25,
                            size,
                            pusher_vel,
                            y_off,
                        )?;
                        assert!(
                            !fr.contains_privileged_force(),
                            "{name}: privileged force in predictor"
                        );
                        let placed = body_xyz(initial, &object_id)
                            .ok_or_else(|| format!("{name}: missing object pose"))?;
                        let mnv = placement.maneuver.as_ref().ok_or("maneuver")?;
                        let q_c = contact_chain_q(model, &ee_name, mnv)?;
                        let n = fr.witness.contact_normal.unwrap_or(mnv.contact_normal);
                        let mut tvec = [-n[1], n[0], 0.0];
                        let tn = (tvec[0] * tvec[0] + tvec[1] * tvec[1]).sqrt();
                        if tn > 1e-9 {
                            tvec = [tvec[0] / tn, tvec[1] / tn, 0.0];
                        }
                        // Contact on the object face at freeze r (COM fixed, ±y along tangent).
                        let half = size.max(1e-4);
                        let contact_shifted = [
                            placed[0] - half * n[0] + y_off * tvec[0],
                            placed[1] - half * n[1] + y_off * tvec[1],
                            placed[2],
                        ];
                        let mut dir = n;
                        if tangential {
                            dir = [n[0] + 3.0 * tvec[0], n[1] + 3.0 * tvec[1], n[2]];
                        }
                        let dn = (dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]).sqrt();
                        if dn > 1e-9 {
                            dir = [dir[0] / dn, dir[1] / dn, dir[2] / dn];
                        }
                        // EE site is ~1 cm behind the fingertips. Start far enough
                        // that t0 is not an overlap teleport; command just past the
                        // face, not a 5 cm slam.
                        let backoff = 0.028;
                        let pre_pt = [
                            contact_shifted[0] - backoff * n[0],
                            contact_shifted[1] - backoff * n[1],
                            contact_shifted[2] - backoff * n[2],
                        ];
                        let stroke_m = 0.055;
                        let nudge = 0.008;
                        let push_pt = [
                            contact_shifted[0] + nudge * dir[0],
                            contact_shifted[1] + nudge * dir[1],
                            contact_shifted[2] + nudge * dir[2],
                        ];
                        let (q_at, pre_tr) = solve_ik(model, &chain, &ee_name, pre_pt, &q_c)
                            .map_err(|e| format!("{name}: ik_pre {e:?}"))?;
                        if pre_tr.residual > 5e-3 {
                            return Err(format!(
                                "{name}: pre-contact IK residual {} m (target {pre_pt:?})",
                                pre_tr.residual
                            ));
                        }
                        let (q_push, push_tr) = solve_ik(model, &chain, &ee_name, push_pt, &q_at)
                            .map_err(|e| format!("{name}: ik_push {e:?}"))?;
                        if push_tr.residual > 8e-3 {
                            return Err(format!(
                                "{name}: push IK residual {} m (target {push_pt:?})",
                                push_tr.residual
                            ));
                        }
                        q_cmd = q_push.clone();
                        let after_place = truth_now(inst)?;
                        let mut qpos = after_place.qpos.clone();
                        for (jn, qi) in chain.iter().zip(q_at.iter()) {
                            if let Some(j) = model.joints.iter().find(|j| j.name == *jn) {
                                if let Some(adr) = j.qpos_adr {
                                    if let Some(slot) = qpos.get_mut(adr as usize) {
                                        *slot = *qi;
                                    }
                                }
                            }
                        }
                        let nv = after_place.qvel.len();
                        inst.reset(Some(&qpos), Some(&vec![0.0; nv.max(1)]))
                            .map_err(|e| e.to_string())?;
                        // Object COM stays at the freeze COM. Do not add y_off here.
                        inst.set_body_pos(&object_id, placed)
                            .map_err(|e| e.to_string())?;
                        apply_named_q(inst, model, &chain, &q_push, &qpos)?;
                        let t0 = truth_now(inst)?;
                        t0_q = chain
                            .iter()
                            .filter_map(|nm| {
                                let j = model.joints.iter().find(|j| j.name == *nm)?;
                                let adr = j.qpos_adr? as usize;
                                t0.qpos.get(adr).copied()
                            })
                            .collect();
                        start_xy_rec = body_xyz(&t0, &object_id).map(|p| [p[0], p[1]]);
                        let start_xy = start_xy_rec;
                        let start_yaw = t0.xquat.get(&object_id).and_then(|q| yaw_wxyz(q));
                        let tool_obj_contact = |t: &VerifierTruth| {
                            t.contacts.iter().any(|c| {
                                let a = c.body1.as_str();
                                let b = c.body2.as_str();
                                (a == object_id || b == object_id) && a != "table" && b != "table"
                            })
                        };
                        t0_contact_rec = tool_obj_contact(&t0);
                        let ee0 = t0
                            .named_pos
                            .get("ee")
                            .cloned()
                            .or_else(|| t0.xpos.get("ee").cloned());
                        let mut saw_contact = t0_contact_rec;
                        let mut t1 = t0.clone();
                        let mut stepped = 0u32;
                        let mut post = 0u32;
                        let max_approach = 150u32;
                        let max_post = 25u32;
                        loop {
                            let end_now = body_xyz(&t1, &object_id).map(|p| [p[0], p[1]]);
                            let disp_now = match (start_xy, end_now) {
                                (Some(a), Some(b)) => hypot2([b[0] - a[0], b[1] - a[1]]),
                                _ => 0.0,
                            };
                            if local_horizon_stop(saw_contact, disp_now, stroke_m, post, max_post) {
                                break;
                            }
                            if !saw_contact && stepped >= max_approach {
                                break;
                            }
                            let _ = inst.step(1);
                            stepped += 1;
                            t1 = truth_now(inst)?;
                            if tool_obj_contact(&t1) {
                                saw_contact = true;
                            }
                            if saw_contact {
                                post += 1;
                            }
                        }
                        let outcome_at_ns = monotonic_ns();
                        t1_contact_rec = saw_contact;
                        horizon_s_rec = f64::from(stepped) * 0.002;
                        let end_xy = body_xyz(&t1, &object_id).map(|p| [p[0], p[1]]);
                        let end_yaw = t1.xquat.get(&object_id).and_then(|q| yaw_wxyz(q));
                        let dxy = match (start_xy, end_xy) {
                            (Some(a), Some(b)) => Some([b[0] - a[0], b[1] - a[1]]),
                            _ => None,
                        };
                        disp = dxy.map(hypot2).unwrap_or(0.0);
                        let dyaw = match (start_yaw, end_yaw) {
                            (Some(a), Some(b)) => Some(principal_yaw_delta(a, b)),
                            _ => None,
                        };
                        let dt = horizon_s_rec.max(0.002);
                        let obj_v = dxy.map(|d| [d[0] / dt, d[1] / dt]).unwrap_or([0.0, 0.0]);
                        let omega = dyaw.unwrap_or(0.0) / dt;
                        let ee1 = t1
                            .named_pos
                            .get("ee")
                            .cloned()
                            .or_else(|| t1.xpos.get("ee").cloned());
                        let ee_dxy = match (ee0.as_ref(), ee1.as_ref()) {
                            (Some(a), Some(b)) if a.len() >= 2 && b.len() >= 2 => {
                                Some([b[0] - a[0], b[1] - a[1]])
                            }
                            _ => None,
                        };
                        ee_dxy_rec = ee_dxy;
                        let ee_vp = ee_dxy.map(|d| [d[0] / dt, d[1] / dt]).unwrap_or([0.0, 0.0]);
                        let r_xy = [
                            contact_shifted[0] - placed[0],
                            contact_shifted[1] - placed[1],
                        ];
                        r_xy_rec = Some(r_xy);
                        contact_shifted_rec = Some(contact_shifted);
                        let n_obs = fr.witness.contact_normal.unwrap_or(n);
                        let obs_mode =
                            infer_observed_mode(ee_vp, [n_obs[0], n_obs[1]], obj_v, omega, r_xy);
                        outcome = MechanicsExecuteOutcome {
                            contact_established: saw_contact,
                            unauthorized_writes: 0,
                            ctrl_writes: 2,
                            authority_refused: false,
                            translation_xy: dxy,
                            yaw_change: dyaw,
                            contact_mode: obs_mode,
                            outcome_at_ns,
                            motion_detected: Some(disp > 5e-4),
                            commanded_stroke_m: Some(stroke_m),
                        };
                        obs_sign = match (dyaw, dxy) {
                            (Some(y), Some(xy)) => Some(observed_rotation_sign(y, xy)),
                            _ => None,
                        };
                        dir_err = match (frozen_world_translation(&fr), dxy) {
                            (Some(fxy), Some(oxy)) => translation_direction_error(fxy, oxy),
                            _ => None,
                        };
                        frozen = Some(fr);
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
                let frozen = frozen.expect(name);
                assert!(
                    frozen.frozen_at_ns < outcome.outcome_at_ns || outcome.outcome_at_ns == 0,
                    "{name}: freeze must precede outcome {} vs {}",
                    frozen.frozen_at_ns,
                    outcome.outcome_at_ns
                );
                assert!(!frozen.contains_privileged_force());
                assert_eq!(ep.unauthorized_writes, 0);
                outcome.unauthorized_writes = ep.unauthorized_writes;
                let div = classify_mechanics_execute(&frozen, &outcome);
                rows.push(json!({
                    "name": name,
                    "y_off": y_off,
                    "pusher_vel": pusher_vel,
                    "tangential": tangential,
                    "frozen_at_ns": frozen.frozen_at_ns,
                    "outcome_at_ns": outcome.outcome_at_ns,
                    "freeze_before_outcome": frozen.frozen_at_ns < outcome.outcome_at_ns,
                    "feasibility": format!("{:?}", frozen.witness.feasibility),
                    "frozen_rotation_sign": format!("{:?}", frozen.witness.rotation_sign),
                    "observed_rotation_sign": format!("{:?}", obs_sign),
                    "frozen_contact_mode": format!("{:?}", frozen.witness.contact_mode),
                    "observed_contact_mode": format!("{:?}", outcome.contact_mode),
                    "frozen_twist": frozen.witness.planar_twist,
                    "frozen_pusher_velocity": frozen.witness.pusher_velocity,
                    "start_xy": start_xy_rec,
                    "r_xy": r_xy_rec,
                    "contact_shifted": contact_shifted_rec,
                    "ee_dxy": ee_dxy_rec,
                    "q_cmd": q_cmd,
                    "t0_q": t0_q,
                    "t0_contact": t0_contact_rec,
                    "t1_contact": t1_contact_rec,
                    "observed_dxy": outcome.translation_xy,
                    "observed_dyaw": outcome.yaw_change,
                    "disp_m": disp,
                    "translation_direction_error_rad": dir_err,
                    "privileged_force_in_predictor": frozen.contains_privileged_force(),
                    "unauthorized_writes": ep.unauthorized_writes,
                    "first_divergence": div.as_str(),
                    "horizon_s": horizon_s_rec,
                    "commanded_stroke_m": 0.055,
                }));
            }
        });
        assert!(
            rows.len() == 4,
            "need centered, tangential, and mirrored pair: {rows:?}"
        );
        for r in &rows {
            assert_eq!(
                r["t0_contact"], false,
                "t0 must not be an overlap teleport: {r}"
            );
        }
        let plus = rows.iter().find(|r| r["name"] == "offset_plus").unwrap();
        let minus = rows.iter().find(|r| r["name"] == "offset_minus").unwrap();
        let tan = rows.iter().find(|r| r["name"] == "tangential_vp").unwrap();
        let cen = rows
            .iter()
            .find(|r| r["name"] == "centered_normal")
            .unwrap();
        let start_y = |r: &Value| {
            r["start_xy"]
                .as_array()
                .and_then(|a| a.get(1))
                .and_then(|v| v.as_f64())
        };
        let r_y = |r: &Value| {
            r["r_xy"]
                .as_array()
                .and_then(|a| a.get(1))
                .and_then(|v| v.as_f64())
        };
        assert!(
            match (start_y(plus), start_y(minus)) {
                (Some(a), Some(b)) => (a - b).abs() < 1e-3,
                _ => false,
            },
            "offset_plus/minus must share freeze COM (shift contact, not the scene): {plus} {minus}"
        );
        assert!(
            match (r_y(plus), r_y(minus)) {
                (Some(a), Some(b)) => a * b < 0.0 && (a - b).abs() > 1e-3,
                _ => false,
            },
            "execute r = contact−COM must keep opposite y (not cancelled by scene shift): {plus} {minus}"
        );
        assert_ne!(
            plus["frozen_rotation_sign"], minus["frozen_rotation_sign"],
            "mirrored execute pair must freeze opposite signs: {plus} {minus}"
        );
        let sign_is = |r: &Value, needle: &str| {
            r["frozen_rotation_sign"]
                .as_str()
                .is_some_and(|s| s.contains(needle))
        };
        let plus_cw = sign_is(plus, "Clockwise") && !sign_is(plus, "Counterclockwise");
        let minus_cw = sign_is(minus, "Clockwise") && !sign_is(minus, "Counterclockwise");
        let plus_ccw = sign_is(plus, "Counterclockwise");
        let minus_ccw = sign_is(minus, "Counterclockwise");
        assert!(
            (plus_cw && minus_ccw) || (plus_ccw && minus_cw),
            "mirrored freeze must be Clockwise vs Counterclockwise: {plus} {minus}"
        );
        assert!(
            tan["pusher_vel"][1].as_f64().unwrap_or(0.0).abs() > 0.1,
            "tangential case must have a y component"
        );
        let fvp = tan["frozen_pusher_velocity"].as_array();
        assert!(
            match fvp {
                Some(a) if a.len() >= 3 => {
                    (a[0].as_f64().unwrap_or(0.0) - 1.0).abs() < 1e-9
                        && (a[1].as_f64().unwrap_or(0.0) - 3.0).abs() < 1e-9
                        && a[2].as_f64().unwrap_or(1.0).abs() < 1e-9
                }
                _ => false,
            },
            "frozen.witness.pusher_velocity must be [1,3,0]: {tan}"
        );
        assert_ne!(
            tan["frozen_contact_mode"], cen["frozen_contact_mode"],
            "tangential freeze mode must differ from centered Sticking: {cen} {tan}"
        );
        let ee_dy = |r: &Value| {
            r["ee_dxy"]
                .as_array()
                .and_then(|a| a.get(1))
                .and_then(|v| v.as_f64())
        };
        assert!(
            ee_dy(tan).is_some_and(|y| y.abs() > 1e-4),
            "tangential execute must measure EE Δxy with a tangent component: {tan}"
        );
        let dxy = |r: &Value| -> Option<[f64; 2]> {
            let a = r["observed_dxy"].as_array()?;
            Some([a.first()?.as_f64()?, a.get(1)?.as_f64()?])
        };
        let dyaw = |r: &Value| r["observed_dyaw"].as_f64();
        let d_c = dxy(cen);
        let d_t = dxy(tan);
        let d_p = dxy(plus);
        let d_m = dxy(minus);
        assert!(
            match (d_c, d_t) {
                (Some(a), Some(b)) => (a[0] - b[0]).abs() > 1e-6 || (a[1] - b[1]).abs() > 1e-6,
                _ => false,
            },
            "tangential execute must not reuse centered Δxy: {cen} {tan}"
        );
        assert!(
            match (d_p, d_m) {
                (Some(a), Some(b)) => {
                    (a[0] - b[0]).abs() > 1e-6
                        || (a[1] - b[1]).abs() > 1e-6
                        || match (dyaw(plus), dyaw(minus)) {
                            (Some(x), Some(y)) => (x - y).abs() > 1e-6,
                            _ => false,
                        }
                }
                _ => false,
            },
            "mirrored execute pair must not share one Δxy/Δyaw: {plus} {minus}"
        );
        let stroke_bound = 0.055;
        for r in &rows {
            let disp_r = r["disp_m"].as_f64().unwrap_or(0.0);
            let dyaw_r = r["observed_dyaw"].as_f64().unwrap_or(0.0);
            let wrapped = principal_angle(dyaw_r).abs();
            let label = r["first_divergence"].as_str().unwrap_or("");
            if disp_r > stroke_bound {
                assert_eq!(
                    label, "QUASI_STATIC_ASSUMPTION_BROKEN",
                    "disp beyond stroke must be ballistic, not a twist/mode claim: {r}"
                );
            }
            if matches!(
                label,
                "NONE" | "CONTACT_MODE_ERROR" | "TWIST_DIRECTION_ERROR"
            ) {
                assert!(
                    disp_r <= stroke_bound + 0.01,
                    "local twist/mode label requires local disp: {r}"
                );
                assert!(
                    wrapped <= std::f64::consts::FRAC_PI_2,
                    "local twist/mode label requires |yaw| ≤ π/2: {r}"
                );
            }
            assert!(
                r["horizon_s"]
                    .as_f64()
                    .is_some_and(|h| h > 0.0 && h <= 0.35),
                "horizon_s must be the stepped time, not a hardcoded 0.4s: {r}"
            );
        }
        let mut sign_ok = 0u32;
        let mut sign_n = 0u32;
        let mut mode_ok = 0u32;
        let mut mode_n = 0u32;
        let mut dir_sum = 0.0;
        let mut dir_n = 0u32;
        let mut div_counts = serde_json::Map::new();
        let mut unauth = 0u64;
        for r in &rows {
            unauth += r["unauthorized_writes"].as_u64().unwrap_or(0);
            let d = r["first_divergence"].as_str().unwrap_or("UNKNOWN");
            let n = div_counts.get(d).and_then(|v| v.as_u64()).unwrap_or(0) + 1;
            div_counts.insert(d.to_string(), json!(n));
            let fs = r["frozen_rotation_sign"].as_str().unwrap_or("");
            let os = r["observed_rotation_sign"].as_str().unwrap_or("");
            if (fs.contains("Clockwise")
                || fs.contains("Counterclockwise")
                || fs.contains("TranslationOnly"))
                && (os.contains("Clockwise")
                    || os.contains("Counterclockwise")
                    || os.contains("TranslationOnly"))
            {
                sign_n += 1;
                if fs == os {
                    sign_ok += 1;
                }
            }
            let fm = r["frozen_contact_mode"].as_str().unwrap_or("");
            let om = r["observed_contact_mode"].as_str().unwrap_or("");
            if (fm.contains("Sticking") || fm.contains("Sliding") || fm.contains("Separating"))
                && (om.contains("Sticking") || om.contains("Sliding") || om.contains("Separating"))
            {
                mode_n += 1;
                if fm == om {
                    mode_ok += 1;
                }
            }
            if let Some(e) = r["translation_direction_error_rad"].as_f64() {
                if e.is_finite() {
                    dir_sum += e;
                    dir_n += 1;
                }
            }
        }
        let sign_acc = if sign_n == 0 {
            Value::Null
        } else {
            json!(f64::from(sign_ok) / f64::from(sign_n))
        };
        let mode_acc = if mode_n == 0 {
            Value::Null
        } else {
            json!(f64::from(mode_ok) / f64::from(mode_n))
        };
        let twist_err = if dir_n == 0 {
            Value::Null
        } else {
            json!(dir_sum / f64::from(dir_n))
        };
        let mut doc: Value = serde_json::from_str(
            &std::fs::read_to_string(scratch_dir().join("push-mechanics-v2.json"))
                .unwrap_or_else(|_| "{}".into()),
        )
        .unwrap_or(json!({}));
        doc["metal"] = json!(false);
        doc["evidence_status"] = json!(SIMULATION_ONLY);
        doc["execute"] = json!({
            "local_stop": true,
            "rows": rows,
        });
        write_scratch(
            "push-mechanics-v2.json",
            &serde_json::to_string_pretty(&doc).unwrap(),
        );
        let mut metrics = serde_json::from_str::<Value>(
            &std::fs::read_to_string(scratch_dir().join("metrics.json"))
                .unwrap_or_else(|_| "{}".into()),
        )
        .unwrap_or(json!({}));
        if let Some(obj) = metrics.as_object_mut() {
            obj.insert("rotation_sign_accuracy".into(), sign_acc);
            obj.insert("contact_mode_accuracy".into(), mode_acc);
            obj.insert("instantaneous_twist_direction_error".into(), twist_err);
            obj.insert("first_divergence_distribution".into(), json!(div_counts));
            obj.insert("unauthorized_writes".into(), json!(unauth));
            obj.insert("old_gross_effort_feasible_was_sound".into(), json!(false));
            obj.insert("metal".into(), json!(false));
            obj.insert("evidence_status".into(), json!(SIMULATION_ONLY));
            obj.insert(
                "execute_cases".into(),
                json!([
                    "centered_normal",
                    "tangential_vp",
                    "offset_plus",
                    "offset_minus"
                ]),
            );
        }
        write_scratch(
            "metrics.json",
            &serde_json::to_string_pretty(&metrics).unwrap(),
        );
    }
}
