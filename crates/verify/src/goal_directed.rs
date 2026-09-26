//! Goal-directed receding-horizon planar loop. MuJoCo is a falsifier only.

use serde::{Deserialize, Serialize};
use serde_json::Value;

use crate::honesty::SIMULATION_ONLY;
use crate::manipulation::ManipulationEpisode;
use crate::push_mechanics::FrozenMechanicsPrediction;
use realityos_semantics::goal_loop::{CausalActionRecord, GoalLoopOutcome, LoopDecision};
use realityos_semantics::physical_interaction::PhysicalInteractionCandidate;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ClosedLoopTrace {
    pub metal: bool,
    pub evidence_status: String,
    pub embodiment: String,
    pub start_xy: [f64; 2],
    pub start_yaw: f64,
    pub goal_xy: Option<[f64; 2]>,
    pub goal_yaw: Option<f64>,
    pub actions: Vec<Value>,
    pub final_outcome: String,
    pub unauthorized_writes: u64,
    pub authority_violations: u64,
    pub selected_faces: Vec<String>,
    pub open_loop_reached: Option<bool>,
    pub closed_loop_reached: Option<bool>,
}

impl ClosedLoopTrace {
    pub fn new(embodiment: impl Into<String>) -> Self {
        Self {
            metal: false,
            evidence_status: SIMULATION_ONLY.into(),
            embodiment: embodiment.into(),
            start_xy: [0.0, 0.0],
            start_yaw: 0.0,
            goal_xy: None,
            goal_yaw: None,
            actions: Vec::new(),
            final_outcome: String::new(),
            unauthorized_writes: 0,
            authority_violations: 0,
            selected_faces: Vec::new(),
            open_loop_reached: None,
            closed_loop_reached: None,
        }
    }
}

pub fn record_action(
    trace: &mut ClosedLoopTrace,
    rec: &CausalActionRecord,
    cand: Option<&PhysicalInteractionCandidate>,
    ep: Option<&ManipulationEpisode>,
    frozen: Option<&FrozenMechanicsPrediction>,
) {
    if let Some(f) = cand.map(|c| c.face_id.clone()) {
        trace.selected_faces.push(f);
    }
    if let Some(ep) = ep {
        trace.unauthorized_writes += ep.unauthorized_writes;
        if ep.task_result == "authority_violation" {
            trace.authority_violations += 1;
        }
    }
    trace.actions.push(serde_json::json!({
        "object_xy": rec.object_xy,
        "object_yaw": rec.object_yaw,
        "candidate_count": rec.candidate_count,
        "rejection_reasons": rec.rejection_reasons,
        "selected_id": rec.selected_id,
        "selected_face": rec.selected_face,
        "selection_rationale": rec.selection_rationale,
        "predicted_contact_mode": rec.predicted_contact_mode,
        "predicted_twist": rec.predicted_twist,
        "predicted_goal_progress": rec.predicted_goal_progress,
        "model_validity": rec.model_validity,
        "authority_decision": rec.authority_decision,
        "goal_error_before": rec.goal_error_before,
        "goal_error_after": rec.goal_error_after,
        "prediction_residual": rec.prediction_residual,
        "first_divergence": rec.first_divergence,
        "decision": rec.decision,
        "outcome": rec.outcome,
        "contact_switch": rec.contact_switch,
        "unauthorized_writes": rec.unauthorized_writes,
        "privileged_force_in_predictor": frozen.map(|f| f.contains_privileged_force()).unwrap_or(false),
        "evidence_status": ep.map(|e| e.evidence_status.clone()).unwrap_or_else(|| SIMULATION_ONLY.into()),
        "ctrl_writes": ep.map(|e| e.ctrl_writes).unwrap_or(0),
        "failure_taxonomy": ep.and_then(|e| e.failure_taxonomy.clone()),
        "had_feasible_contact_maneuver": ep.map(|e| e.had_feasible_contact_maneuver),
        "selected_rank_why": ep.map(|e| e.selected_rank_why.clone()),
        "reasoning": rec.reasoning,
    }));
    trace.final_outcome = format!("{:?}", rec.outcome);
    if rec.outcome == GoalLoopOutcome::GoalReached {
        trace.closed_loop_reached = Some(true);
    }
    if rec.decision == LoopDecision::Halt {
        trace.closed_loop_reached = Some(rec.outcome == GoalLoopOutcome::GoalReached);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bundle::RobotBundle;
    use crate::corpus;
    use crate::manipulation::{
        chain_q_from_qpos, local_ee_poses, run_skill_episode_ex, sample_push_units,
        template_objects, with_episode_qpos, with_injected_push_maneuver,
    };
    use crate::manipulation_scenarios::{ManipulationScenario, Polarity};
    use crate::manipulation_verify::body_xyz;
    use crate::mujoco_exec::{checkin_worker, ensure_mujoco_or_skip};
    use crate::observation::VerifierTruth;
    use crate::runner::load_and_normalize;
    use crate::semantics_map::embodiment_from_manifest;
    use realityos_physics::{PressureDistribution, SupportFrictionModel};
    use realityos_semantics::command_domain::named_joint_limit_margin;
    use realityos_semantics::contact::declared_manipulation_contact_bodies;
    use realityos_semantics::contact_jacobian::contact_jacobian_witness;
    use realityos_semantics::contact_maneuver::{
        object_center_for_sample, select_fixed_world_push_seeded_with_funnel, tool_offset_in_ee,
        BoxObject, ContactInfeasible, ContactManeuver, ContactManeuverSpec, ContactSelectFunnel,
        SampledEePose, SupportPlane,
    };
    use realityos_semantics::discrepancy::{
        apply_probe_observation, hypothesize, prediction_regime, tag_probe_motion, DiscrepancyKind,
        DiscrepancyObservation, Identifiability, Stimulus,
    };
    use realityos_semantics::effect_feasibility::PlanarPushInitiation;
    use realityos_semantics::effort::{any_link_com_known, chain_physical_effort_signed};
    use realityos_semantics::execution_envelope::{
        check_execution_envelope, EnvelopeVerdict, ExecutionEnvelope, RuntimeExecutionObservation,
        QUASI_STATIC_DISPLACEMENT_RATIO,
    };
    use realityos_semantics::geometry::PrimitiveShape;
    use realityos_semantics::goal_loop::{
        goal_status_if_no_admissible_interaction, receding_horizon_step, record_after_with_goal,
        GoalLoopOutcome, LoopDecision, LoopState, ReasoningNote, WorldObservation,
    };
    use realityos_semantics::kinematics::{forward_kinematics, ik_residual_is_precise, solve_ik};
    use realityos_semantics::maneuver_witness::execution_block_reason;
    use realityos_semantics::pair_friction::PairFriction;
    use realityos_semantics::physical_belief::{
        BeliefEpistemicStatus, ParameterBelief, PhysicalParameter, PhysicalParameterBelief,
    };
    use realityos_semantics::physical_experience::{
        authorize_from_experience, initial_decision, ApplicabilityQuery, ExperienceLog,
        PhysicalExperienceRecord,
    };
    use realityos_semantics::physical_interaction::{
        action_key_is_forbidden, evaluate_all, evaluate_candidate, generate_planar_push_candidates,
        initiation_from_candidate, select_interaction, EvaluationContext, FunnelStage,
        SelectionOutcome,
    };
    use realityos_semantics::physical_quantity::PhysicalEffort;
    use realityos_semantics::planar_goal::{
        evaluate_goal_error, yaw_from_quat_wxyz, InteractionFamily, PlanarObjectGoal,
        SafetyConstraints,
    };
    use realityos_semantics::probe_selection::{
        candidates_for_uncertainty, goal_contact_at_contradicted_declared_friction,
        rank_goal_or_probe, BeliefRobustness, DecisionClass,
    };
    use realityos_semantics::provenance::Provenanced;
    use realityos_semantics::recoverability::{
        classify_recoverability, select_recoverable_progress, InteractionRegion,
        RecoverabilityChoice, RecoverabilityClass, RecoverabilityInput,
    };
    use realityos_semantics::self_load::{gravity_self_load, self_load_provenanced};
    use realityos_semantics::transform::{rotate_by_quat, Se3};
    use realityos_semantics::workspace::reachable_ee_poses;
    use serde_json::json;
    use std::path::PathBuf;

    fn scratch_dir() -> PathBuf {
        PathBuf::from(r"C:\Users\moram\AppData\Local\Temp\grok-goal-45646d0dc2cc\implementer")
    }

    fn write_scratch(name: &str, body: &str) {
        let _ = std::fs::create_dir_all(scratch_dir());
        let _ = std::fs::write(scratch_dir().join(name), body);
    }

    fn trans_goal(target: [f64; 2], tol: f64, attempts: u32) -> PlanarObjectGoal {
        PlanarObjectGoal {
            object_id: "obj0".into(),
            world_id: "sim".into(),
            model_id: "sim".into(),
            target_xy: Some(target),
            target_xy_region: None,
            target_yaw: None,
            target_yaw_interval: None,
            translation_tolerance_m: tol,
            orientation_tolerance_rad: 0.2,
            freshness_s: 1.0,
            allowed_interaction_family: InteractionFamily::PlanarPush,
            safety: SafetyConstraints::default(),
            max_bounded_attempts: attempts,
        }
    }

    #[derive(Debug, Clone, Copy, PartialEq)]
    struct PushScenarioPhysics {
        simulator_mass_kg: f64,
        simulator_support_friction: f64,
        reasoner_mass_kg: Option<f64>,
        reasoner_support_friction: Option<f64>,
    }

    const PUSH_SCENARIO_PHYSICS: PushScenarioPhysics = PushScenarioPhysics {
        simulator_mass_kg: 0.05,
        simulator_support_friction: 0.3,
        reasoner_mass_kg: Some(0.05),
        reasoner_support_friction: Some(0.3),
    };

    fn parameter_belief(
        parameter: PhysicalParameter,
        value: Option<f64>,
        source: &str,
    ) -> ParameterBelief {
        match value {
            Some(value) => ParameterBelief {
                parameter,
                status: BeliefEpistemicStatus::DeclaredFact,
                declared: Provenanced::declared(value, source, 0.0),
                empirical_interval: None,
                lineage: Vec::new(),
            },
            None => ParameterBelief {
                parameter,
                status: BeliefEpistemicStatus::Unknown,
                declared: Provenanced::unknown(source, 0.0),
                empirical_interval: None,
                lineage: Vec::new(),
            },
        }
    }

    fn fresh_belief_from_physics(physics: &PushScenarioPhysics) -> PhysicalParameterBelief {
        let mut belief = match physics.reasoner_support_friction {
            Some(mu) => PhysicalParameterBelief::declared_point(
                PhysicalParameter::SupportFriction,
                mu,
                "reasoner.disclosure.support_friction",
            ),
            None => PhysicalParameterBelief {
                parameters: Vec::new(),
            }
            .with_unknown(
                PhysicalParameter::SupportFriction,
                "reasoner.disclosure.support_friction",
            ),
        };
        belief.parameters.push(parameter_belief(
            PhysicalParameter::ObjectMassKg,
            physics.reasoner_mass_kg,
            "reasoner.disclosure.object_mass_kg",
        ));
        belief.parameters.push(parameter_belief(
            PhysicalParameter::QuasiStaticApplicability,
            Some(1.0),
            "declared.quasi_static",
        ));
        belief
    }

    fn physics_from_belief(
        base: PushScenarioPhysics,
        belief: Option<&PhysicalParameterBelief>,
    ) -> PushScenarioPhysics {
        let Some(belief) = belief else {
            return base;
        };
        PushScenarioPhysics {
            reasoner_mass_kg: belief.declared_value(PhysicalParameter::ObjectMassKg),
            reasoner_support_friction: belief.declared_value(PhysicalParameter::SupportFriction),
            ..base
        }
    }

    fn rationale_mu(value: Option<f64>) -> String {
        match value.filter(|mu| mu.is_finite()) {
            Some(mu) => format!("{mu:.3}"),
            None => "UNKNOWN".into(),
        }
    }

    fn pose(xy: [f64; 2], z: f64) -> Se3 {
        pose_xy_yaw(xy, z, 0.0)
    }

    fn pose_xy_yaw(xy: [f64; 2], z: f64, yaw: f64) -> Se3 {
        let rot = Se3::from_axis_angle([0.0, 0.0, 1.0], yaw).unwrap_or_else(|_| Se3::identity());
        Se3 {
            xyz: [xy[0], xy[1], z],
            quat_wxyz: rot.quat_wxyz,
        }
    }

    fn mechanics_template(mass: f64, mu: f64, tau: f64) -> PlanarPushInitiation {
        let n = mass * 9.80665;
        let f_max = mu * n;
        PlanarPushInitiation {
            mass_kg: Provenanced::declared(mass, "scenario.mass", 0.0),
            object_com_world: Provenanced::declared([0.0, 0.0, 0.03], "scenario.com", 0.0),
            gravity_m_s2: Provenanced::declared([0.0, 0.0, -9.80665], "scenario.g", 0.0),
            support_normal: Provenanced::declared([0.0, 0.0, 1.0], "scenario.n", 0.0),
            object_support_friction: PairFriction::coulomb(
                "object",
                "support",
                Provenanced::declared(mu, "scenario.mu_s", 0.0),
            ),
            tool_object_friction: PairFriction::coulomb(
                "tool",
                "object",
                Provenanced::declared(0.8, "scenario.mu_t", 0.0),
            ),
            contact_point_world: Provenanced::unknown("c", 0.0),
            contact_normal_world: Provenanced::unknown("n", 0.0),
            push_direction_world: Provenanced::unknown("d", 0.0),
            contact_force_direction_world: Provenanced::unknown("f", 0.0),
            pusher_velocity_world: Provenanced::unknown("v", 0.0),
            joint_names: vec!["j0".into(), "j1".into(), "j2".into()],
            translational_jacobian_3xn: vec![
                vec![1.0, 0.0, 0.0],
                vec![0.0, 1.0, 0.0],
                vec![0.0, 0.0, 1.0],
            ],
            jacobian_residual: Some(0.0),
            joint_effort_abs: vec![
                Provenanced::declared(tau, "t", 0.0),
                Provenanced::declared(tau, "t", 0.0),
                Provenanced::declared(tau, "t", 0.0),
            ],
            joint_effort_min: vec![
                Provenanced::declared(-tau, "t", 0.0),
                Provenanced::declared(-tau, "t", 0.0),
                Provenanced::declared(-tau, "t", 0.0),
            ],
            joint_effort_max: vec![
                Provenanced::declared(tau, "t", 0.0),
                Provenanced::declared(tau, "t", 0.0),
                Provenanced::declared(tau, "t", 0.0),
            ],
            self_load_torque_nm: vec![
                Provenanced::declared(0.0, "s", 0.0),
                Provenanced::declared(0.0, "s", 0.0),
                Provenanced::declared(0.0, "s", 0.0),
            ],
            link_com_known: true,
            object_supported: true,
            approximately_planar: true,
            quasi_static: true,
            single_intended_contact: true,
            no_significant_impact: true,
            object_characteristic_length_m: Some(0.04),
            support_friction_model: SupportFrictionModel::Ellipsoidal {
                f_max,
                tau_max: f_max * (2.0 / 3.0) * 0.04,
                pressure: PressureDistribution::DeclaredUniform,
            },
            object_yaw_rad: Provenanced::declared(0.0, "yaw", 0.0),
            stale_object_evidence: false,
            intended_contact_lost: false,
            authority_ok: true,
        }
    }

    fn eval_at(
        xy: [f64; 2],
        goal: &PlanarObjectGoal,
        authority_ok: bool,
    ) -> Vec<realityos_semantics::physical_interaction::PhysicalInteractionCandidate> {
        let mut cands = generate_planar_push_candidates(
            "obj0",
            pose(xy, 0.03),
            [0.04, 0.03, 0.03],
            [0.0, 0.0, 1.0],
            0.01,
            0.02,
        )
        .expect("valid fixture support geometry");
        let mut mech = mechanics_template(0.05, 0.2, 20.0);
        mech.authority_ok = authority_ok;
        let ctx = EvaluationContext {
            goal: goal.clone(),
            object_xy: xy,
            object_yaw: 0.0,
            object_com_world: [xy[0], xy[1], 0.03],
            mechanics_template: Some(mech),
            authority_ok,
            robot_provided: false,
            robot_reachable: None,
            collision_admissible: None,
            executable_witness: None,
            robot_reject_reason: None,
        };
        evaluate_all(&mut cands, &ctx);
        cands
    }

    #[test]
    fn push_fixture_truth_and_disclosure_are_separate() {
        let physics = PushScenarioPhysics {
            reasoner_mass_kg: Some(0.08),
            reasoner_support_friction: Some(0.24),
            ..PUSH_SCENARIO_PHYSICS
        };
        let scenario = push_scenario([0.0, 0.0, 0.03], 0.03, physics, [1.0, 0.0, 0.0], 0.05, 7);
        let object = &scenario.objects[1];
        assert_eq!(object["mass"].as_f64(), Some(physics.simulator_mass_kg));
        assert_eq!(
            object["friction"].as_f64(),
            Some(physics.simulator_support_friction)
        );

        let model = realityos_semantics::adapter::synth_planar_two_link();
        let mechanics = fill_mechanics_from_q(&model, "ee", &[], [0.0, 0.0, 0.03], 0.0, &physics)
            .expect("explicitly disclosed physics produces a mechanics template");
        assert_eq!(mechanics.mass_kg.value, physics.reasoner_mass_kg);
        assert_eq!(
            mechanics.mass_kg.provenance,
            realityos_semantics::provenance::Provenance::ModelDeclared
        );
        assert_eq!(
            mechanics.object_support_friction.sliding_mu.value,
            physics.reasoner_support_friction
        );
        assert_eq!(
            mechanics.object_support_friction.sliding_mu.provenance,
            realityos_semantics::provenance::Provenance::ModelDeclared
        );

        let belief = fresh_belief_from_physics(&physics);
        assert_eq!(
            belief.declared_value(PhysicalParameter::SupportFriction),
            physics.reasoner_support_friction
        );
    }

    #[test]
    fn hidden_fixture_friction_refuses_unknown_mechanics() {
        let physics = PushScenarioPhysics {
            reasoner_support_friction: None,
            ..PUSH_SCENARIO_PHYSICS
        };
        let belief = fresh_belief_from_physics(&physics);
        assert_eq!(
            belief
                .entry(PhysicalParameter::SupportFriction)
                .map(|entry| entry.status),
            Some(BeliefEpistemicStatus::Unknown)
        );
        assert_eq!(
            belief.declared_value(PhysicalParameter::SupportFriction),
            None
        );
        assert_eq!(
            rationale_mu(belief.declared_value(PhysicalParameter::SupportFriction)),
            "UNKNOWN"
        );

        let model = realityos_semantics::adapter::synth_planar_two_link();
        let mechanics = fill_mechanics_from_q(&model, "ee", &[], [0.0, 0.0, 0.03], 0.0, &physics);
        assert!(mechanics.is_none());

        let goal = trans_goal([0.20, 0.0], 0.01, 6);
        let mut candidates = generate_planar_push_candidates(
            "obj0",
            pose([0.0, 0.0], 0.03),
            [0.04, 0.03, 0.03],
            [0.0, 0.0, 1.0],
            0.01,
            0.02,
        )
        .expect("valid fixture support geometry");
        let context = EvaluationContext {
            goal,
            object_xy: [0.0, 0.0],
            object_yaw: 0.0,
            object_com_world: [0.0, 0.0, 0.03],
            mechanics_template: mechanics,
            authority_ok: true,
            robot_provided: false,
            robot_reachable: None,
            collision_admissible: None,
            executable_witness: None,
            robot_reject_reason: None,
        };
        evaluate_all(&mut candidates, &context);
        assert!(matches!(
            select_interaction(&candidates, &[]),
            SelectionOutcome::MechanicsUnknown { .. }
        ));
    }

    #[test]
    fn analytic_selection_and_loop_twice() {
        let mut log = String::new();
        for pass in 1..=2 {
            let g = trans_goal([0.20, 0.0], 0.01, 6);
            let xy = [0.0, 0.0];
            let cands = eval_at(xy, &g, true);
            let sel = select_interaction(&cands, &[]);
            let SelectionOutcome::Selected { index, reason } = sel else {
                panic!("pass {pass}: expected selection, got {sel:?}");
            };
            assert_eq!(
                cands[index].goal_progress,
                Some(realityos_semantics::planar_goal::GoalProgressClass::StrictProgress)
            );
            assert!(reason.contains("STRICT_PROGRESS"));
            let frozen = {
                let init = initiation_from_candidate(
                    &mechanics_template(0.05, 0.2, 20.0),
                    &cands[index],
                    [xy[0], xy[1], 0.03],
                    0.0,
                    true,
                );
                FrozenMechanicsPrediction::freeze(
                    cands[index].witness.clone().expect("witness"),
                    &init,
                )
            };
            assert!(!frozen.contains_privileged_force());
            let obs = WorldObservation {
                object_id: "obj0".into(),
                xy,
                yaw: 0.0,
                robot_q: vec![0.0],
                freshness_ok: true,
                intended_contact_face: None,
                authority_ok: true,
                observed_at_s: 1.0,
            };
            let step = receding_horizon_step(&obs, &g, &cands, LoopState::default(), None);
            assert_eq!(step.record.unauthorized_writes, 0);
            let after = WorldObservation {
                object_id: "obj0".into(),
                xy: [0.03, 0.0],
                yaw: 0.0,
                robot_q: vec![0.1],
                freshness_ok: true,
                intended_contact_face: step.record.selected_face.clone(),
                authority_ok: true,
                observed_at_s: 2.0,
            };
            let rec = record_after_with_goal(step, &after, &g);
            assert!(rec.state.last_predicted_pose.is_none());
            log.push_str(&format!(
                "pass={pass} selected={} reason={} outcome={:?} residual={:?}\n",
                rec.record.selected_face.as_deref().unwrap_or("-"),
                rec.record.selection_rationale,
                rec.record.outcome,
                rec.record.prediction_residual
            ));
        }
        write_scratch("selection-counterfactual.log", &log);
        write_scratch("closed-loop-analytic.log", &log);
    }

    fn object_xy_yaw(ep: &ManipulationEpisode, truth: &VerifierTruth) -> ([f64; 2], f64) {
        let xy = body_xyz(truth, "obj0")
            .map(|p| [p[0], p[1]])
            .or_else(|| {
                ep.object_evidence
                    .get("pose")
                    .and_then(|p| p.as_array())
                    .and_then(|a| Some([a.first()?.as_f64()?, a.get(1)?.as_f64()?]))
            })
            .unwrap_or([0.0, 0.0]);
        let yaw = truth
            .xquat
            .get("obj0")
            .and_then(|q| {
                if q.len() >= 4 {
                    Some(yaw_from_quat_wxyz([q[0], q[1], q[2], q[3]]))
                } else {
                    None
                }
            })
            .unwrap_or(0.0);
        (xy, yaw)
    }

    fn push_scenario(
        obj: [f64; 3],
        size: f64,
        physics: PushScenarioPhysics,
        push_dir: [f64; 3],
        push_dist: f64,
        seed: u64,
    ) -> ManipulationScenario {
        let table_z = (obj[2] - size - 0.01).max(0.02);
        ManipulationScenario {
            seed,
            polarity: Polarity::Positive,
            skill: "PUSH".into(),
            objects: vec![
                json!({"name":"table","type":"box","pos":[obj[0], obj[1], table_z],"size":[0.18,0.18,0.01],"mass":10.0,"movable":false}),
                json!({
                    "name":"obj0","type":"box","pos":obj,"size":[size,size,size],
                    "mass":physics.simulator_mass_kg,
                    "friction":physics.simulator_support_friction,"movable":true
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
            push_dir,
            push_dist,
            required_opening: 0.8,
            world_construction:
                realityos_semantics::contact_maneuver::WorldConstructionMode::FixedWorld,
        }
    }

    fn truth_of(inst: &mut crate::mujoco_exec::MujocoInstance) -> Result<VerifierTruth, String> {
        let st = inst.step(0).map_err(|e| e.to_string())?;
        Ok(VerifierTruth::from_mujoco_state(
            st.get("state").unwrap_or(&st),
        ))
    }

    fn flags_from_infeasible(e: ContactInfeasible) -> (Option<bool>, Option<bool>, Option<bool>) {
        match e {
            ContactInfeasible::InvalidSupportPlane | ContactInfeasible::InvalidObjectPose => {
                (None, None, None)
            }
            ContactInfeasible::NoIkSolution
            | ContactInfeasible::ContactPoseUnreachableFromApproach
            | ContactInfeasible::NoFeasibleContactPose => (Some(false), None, None),
            ContactInfeasible::CollisionInadmissible
            | ContactInfeasible::ApproachCollidesBeforeContact
            | ContactInfeasible::SupportPlaneBlocksEe => (Some(true), Some(false), None),
            ContactInfeasible::NoExecutableContactManeuver
            | ContactInfeasible::InsufficientJointMargin
            | ContactInfeasible::OrientationInfeasible
            | ContactInfeasible::InsufficientRemainingStroke
            | ContactInfeasible::WrongContactGeometry => (Some(true), Some(true), Some(false)),
        }
    }

    fn build_ee_cloud(
        model: &realityos_semantics::embodiment::EmbodimentModel,
        ee: &str,
        qpos: &[f64],
        seed: u64,
    ) -> Vec<SampledEePose> {
        let units = sample_push_units(model, ee, seed, 96);
        let mut cloud = reachable_ee_poses(model, ee, &units);
        if let Some(q) = chain_q_from_qpos(model, ee, qpos) {
            if let Some(chain) = model.ee_joint_chain(ee) {
                if let Ok(fk) = forward_kinematics(model, &chain, ee, &q) {
                    cloud.insert(
                        0,
                        SampledEePose {
                            xyz: fk.ee.xyz,
                            quat_wxyz: fk.ee.quat_wxyz,
                            q: q.clone(),
                            joint_names: chain.clone(),
                        },
                    );
                }
                cloud.extend(local_ee_poses(model, ee, &chain, &q, seed, 32));
            }
        }
        cloud
    }

    fn fill_mechanics_from_q(
        model: &realityos_semantics::embodiment::EmbodimentModel,
        ee: &str,
        qpos: &[f64],
        contact_world: [f64; 3],
        yaw: f64,
        physics: &PushScenarioPhysics,
    ) -> Option<PlanarPushInitiation> {
        let mass = physics.reasoner_mass_kg?;
        let mu = physics.reasoner_support_friction?;
        let mut p = mechanics_template(mass, mu, 20.0);
        p.mass_kg = Provenanced::declared(mass, "reasoner.disclosure.object_mass_kg", 0.0);
        p.object_support_friction = PairFriction::coulomb(
            "object",
            "support",
            Provenanced::declared(mu, "reasoner.disclosure.support_friction", 0.0),
        );
        p.object_yaw_rad = Provenanced::declared(yaw, "obs.yaw", 0.0);
        let Some(q) = chain_q_from_qpos(model, ee, qpos) else {
            return Some(p);
        };
        let Some(chain) = model.ee_joint_chain(ee) else {
            return Some(p);
        };
        let Ok(fk) = forward_kinematics(model, &chain, ee, &q) else {
            return Some(p);
        };
        let contact_in_ee = tool_offset_in_ee(fk.ee.quat_wxyz, contact_world, fk.ee.xyz);
        let Ok(jac) = contact_jacobian_witness(model, &chain, ee, &q, contact_in_ee, 1e-6) else {
            return Some(p);
        };
        p.joint_names = jac.joint_names.clone();
        p.translational_jacobian_3xn = jac.analytic_3xn.clone();
        p.jacobian_residual = Some(jac.residual);
        p.link_com_known = any_link_com_known(model);
        match chain_physical_effort_signed(model, &p.joint_names) {
            Ok(signed) => {
                p.joint_effort_min = signed
                    .iter()
                    .map(|s| Provenanced::declared(s.tau_min_nm, "joint.effort_min", 0.0))
                    .collect();
                p.joint_effort_max = signed
                    .iter()
                    .map(|s| Provenanced::declared(s.tau_max_nm, "joint.effort_max", 0.0))
                    .collect();
                p.joint_effort_abs = signed
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
                p.joint_effort_abs = p
                    .joint_names
                    .iter()
                    .map(|_| Provenanced::unknown(reason.clone(), 0.0))
                    .collect();
            }
            Err(_) => {}
        }
        let mut q_by_joint = std::collections::BTreeMap::new();
        for (name, qi) in chain.iter().zip(q.iter()) {
            q_by_joint.insert(name.clone(), *qi);
        }
        if let Ok(tau) = gravity_self_load(model, &p.joint_names, &q_by_joint, [0.0, 0.0, -9.80665])
        {
            p.self_load_torque_nm = self_load_provenanced(&tau, "self_load.gravity");
        }
        Some(p)
    }

    fn prove_face_funnel(
        model: &realityos_semantics::embodiment::EmbodimentModel,
        ee: &str,
        cloud: &[SampledEePose],
        object: BoxObject,
        support: SupportPlane,
        push: [f64; 3],
        stroke: f64,
        ee_xyz: [f64; 3],
        tool_off: [f64; 3],
        short_witness: bool,
    ) -> (
        Result<ContactManeuver, ContactInfeasible>,
        ContactSelectFunnel,
    ) {
        let mut spec = ContactManeuverSpec::table_push(push, stroke, ee_xyz, tool_off);
        if short_witness && stroke.is_finite() && stroke > 1e-4 && stroke < spec.min_stroke {
            // A discriminating probe must end below the quasi-static limit.
            // The table-push floor would otherwise drive at least 0.02 m.
            spec.min_stroke = stroke;
            spec.requested_stroke = stroke;
        }
        spec.object_id = "obj0".into();
        spec.intended_tool_bodies =
            declared_manipulation_contact_bodies(model, model.resources.first(), ee);
        if spec.intended_tool_bodies.is_empty() {
            spec.intended_tool_bodies.push("tool".into());
        }
        let intended = spec.intended_tool_bodies.clone();
        for b in &model.bodies {
            if intended.iter().any(|n| n == &b.name) {
                continue;
            }
            if b.parent.is_none() {
                continue;
            }
            spec.robot_body_volumes
                .push(realityos_semantics::contact_collision::AttachedSphere {
                    body: b.name.clone(),
                    radius: spec.ee_radius,
                    offset: [0.0, 0.0, 0.0],
                });
        }
        let (res, funnel) =
            select_fixed_world_push_seeded_with_funnel(model, ee, cloud, object, support, &spec);
        (
            match res {
                Ok((m, _)) => {
                    if m.executable
                        .as_ref()
                        .is_some_and(|w| execution_block_reason(w).is_none())
                    {
                        Ok(m)
                    } else {
                        Err(ContactInfeasible::NoExecutableContactManeuver)
                    }
                }
                Err(e) => Err(e),
            },
            funnel,
        )
    }

    fn prove_face(
        model: &realityos_semantics::embodiment::EmbodimentModel,
        ee: &str,
        cloud: &[SampledEePose],
        object: BoxObject,
        support: SupportPlane,
        push: [f64; 3],
        stroke: f64,
        ee_xyz: [f64; 3],
        tool_off: [f64; 3],
        short_witness: bool,
    ) -> Result<ContactManeuver, ContactInfeasible> {
        prove_face_funnel(
            model,
            ee,
            cloud,
            object,
            support,
            push,
            stroke,
            ee_xyz,
            tool_off,
            short_witness,
        )
        .0
    }

    /// Contacts proved from the observed pose. The count is the admissible set.
    fn proved_contacts_at(
        model: &realityos_semantics::embodiment::EmbodimentModel,
        ee: &str,
        qpos: &[f64],
        xy: [f64; 2],
        z: f64,
        yaw: f64,
        size: f64,
        stroke: f64,
        tool_off: [f64; 3],
        seed: u64,
        ee_fallback: [f64; 3],
        prefer_push: [f64; 3],
        short_witness: bool,
    ) -> (u32, Option<ContactManeuver>) {
        let object_pose = pose_xy_yaw(xy, z, yaw);
        let cands = generate_planar_push_candidates(
            "obj0",
            object_pose,
            [size, size, size],
            [0.0, 0.0, 1.0],
            0.015,
            stroke,
        )
        .expect("valid fixture support geometry");
        let cloud = build_ee_cloud(model, ee, qpos, seed);
        let ee_xyz = cloud.first().map(|s| s.xyz).unwrap_or(ee_fallback);
        let object = BoxObject {
            center: [xy[0], xy[1], z],
            half_extents: [size, size, size],
            quat_wxyz: object_pose.quat_wxyz,
        };
        let support = SupportPlane {
            origin: [xy[0], xy[1], z - size],
            normal: [0.0, 0.0, 1.0],
        };
        let mut admissible = 0u32;
        let mut best: Option<(f64, ContactManeuver)> = None;
        let mut seen = std::collections::BTreeSet::new();
        for cand in &cands {
            if !seen.insert(cand.face_id.clone()) {
                continue;
            }
            if let Ok(maneuver) = prove_face(
                model,
                ee,
                &cloud,
                object,
                support,
                cand.push_direction_world,
                cand.stroke_m,
                ee_xyz,
                tool_off,
                short_witness,
            ) {
                admissible += 1;
                let dot = maneuver.push_direction[0] * prefer_push[0]
                    + maneuver.push_direction[1] * prefer_push[1]
                    + maneuver.push_direction[2] * prefer_push[2];
                let better = best.as_ref().map(|(score, _)| dot > *score).unwrap_or(true);
                if better {
                    best = Some((dot, maneuver));
                }
            }
        }
        (admissible, best.map(|(_, maneuver)| maneuver))
    }

    /// Stroke length of the witness end pose, not the shorter requested probe.
    fn executed_witness_stroke(maneuver: &ContactManeuver) -> f64 {
        if let Some(witness) = &maneuver.executable {
            let from = witness.contact.pose.xyz;
            let to = witness.end_stroke.pose.xyz;
            let d =
                ((to[0] - from[0]).powi(2) + (to[1] - from[1]).powi(2) + (to[2] - from[2]).powi(2))
                    .sqrt();
            if d.is_finite() && d > 1e-6 {
                return d;
            }
        }
        realityos_semantics::push::effective_push_distance(maneuver.requested_stroke)
    }

    fn proven_witness_stroke(maneuver: &ContactManeuver) -> Option<f64> {
        let witness = maneuver.executable.as_ref()?;
        let from = witness.contact.pose.xyz;
        let to = witness.end_stroke.pose.xyz;
        let d = ((to[0] - from[0]).powi(2) + (to[1] - from[1]).powi(2) + (to[2] - from[2]).powi(2))
            .sqrt();
        (d.is_finite() && d > 1e-6).then_some(d)
    }

    fn install_proven_maneuver(
        candidate: &mut PhysicalInteractionCandidate,
        maneuver: Option<ContactManeuver>,
    ) -> bool {
        if let Some(maneuver) = maneuver.as_ref() {
            let Some(stroke_m) = proven_witness_stroke(maneuver) else {
                candidate.maneuver = None;
                return false;
            };
            candidate.stroke_m = stroke_m;
        }
        candidate.maneuver = maneuver;
        true
    }

    fn selected_witness_stroke(candidate: &PhysicalInteractionCandidate) -> Option<f64> {
        proven_witness_stroke(candidate.maneuver.as_ref()?)
    }

    fn execution_scenario_strokes(full_stroke: f64) -> Vec<(u32, f64)> {
        // A scenario executes every phase of the selected witness. Until the executor can
        // observe and abort within one witness, dispatch that frozen stroke exactly once.
        vec![(0, full_stroke)]
    }

    fn observed_discrepancy_without_residuals(
        displacement_ratio: f64,
        yaw_change_rad: f64,
        predicted_yaw_sign: Option<i8>,
        contact_persisted: bool,
        stroke_m: f64,
        quasi_static_stroke_limit_m: f64,
    ) -> DiscrepancyObservation {
        DiscrepancyObservation {
            displacement_ratio: Some(displacement_ratio),
            yaw_change_rad: Some(yaw_change_rad),
            predicted_yaw_sign,
            observed_yaw_sign: yaw_change_rad.is_finite().then_some({
                if yaw_change_rad > 0.0 {
                    1
                } else if yaw_change_rad < 0.0 {
                    -1
                } else {
                    0
                }
            }),
            contact_persisted: Some(contact_persisted),
            tracking_error_m: None,
            geometry_residual_m: None,
            freshness_ok: true,
            stroke_m,
            quasi_static_stroke_limit_m,
            contradictory: false,
            reachable: true,
        }
    }

    fn reasoning_taxonomy(
        prevention_impossible: bool,
        interactable: bool,
        selected_probe: bool,
        executable_probe_available: bool,
        discrepancy_status: Identifiability,
        abort_at: Option<u32>,
    ) -> &'static str {
        if prevention_impossible {
            "STATE_ALREADY_UNRECOVERABLE"
        } else if interactable && selected_probe && executable_probe_available {
            "SAFE_PROBE_AVAILABLE"
        } else if discrepancy_status == Identifiability::Unknown {
            "INSUFFICIENT_DISCREPANCY_EVIDENCE"
        } else if interactable {
            "SAFE_INTERACTION_AVAILABLE"
        } else if abort_at.is_some() {
            "GOAL_CURRENTLY_UNACHIEVABLE"
        } else {
            "NOT_PROVED"
        }
    }

    fn candidate_stroke_plan(
        belief_state: Option<&PhysicalParameterBelief>,
        live_hypotheses: &[DiscrepancyKind],
        remain: f64,
        quasi_limit_m: f64,
    ) -> (bool, f64, Option<f64>) {
        let belief_short = belief_state.is_some_and(|belief| {
            let long = prediction_regime(belief, live_hypotheses, 0.03, quasi_limit_m);
            !long.quasi_static || long.friction_contradicted
        });
        let long_stroke = remain.clamp(0.02, 0.03);
        let short_stroke = (quasi_limit_m * 0.5).max(0.004);
        let primary = if belief_short {
            short_stroke
        } else {
            long_stroke
        };
        let short_alternative = (belief_state.is_some() && !belief_short).then_some(short_stroke);
        (belief_short, primary, short_alternative)
    }

    fn admissible_override_indices(
        candidates: &[realityos_semantics::physical_interaction::PhysicalInteractionCandidate],
        forbidden_action_keys: &[String],
    ) -> Vec<usize> {
        candidates
            .iter()
            .enumerate()
            .filter_map(|(index, candidate)| {
                let action_key = candidate.action_key();
                let goal_useful = candidate.funnel.stage == FunnelStage::GoalUseful
                    && candidate.goal_progress
                        == Some(
                            realityos_semantics::planar_goal::GoalProgressClass::StrictProgress,
                        );
                (goal_useful
                    && candidate.authority_ok
                    && candidate.executable_for_plant
                    && !action_key_is_forbidden(&action_key, forbidden_action_keys))
                .then_some(index)
            })
            .collect()
    }

    fn select_recoverable_override_index(
        candidates: &[realityos_semantics::physical_interaction::PhysicalInteractionCandidate],
        choices: &[RecoverabilityChoice],
        forbidden_action_keys: &[String],
    ) -> Option<usize> {
        if candidates.len() != choices.len() {
            return None;
        }
        let admissible_indices = admissible_override_indices(candidates, forbidden_action_keys);
        let admissible_choices: Vec<_> = admissible_indices
            .iter()
            .map(|&index| choices[index].clone())
            .collect();
        select_recoverable_progress(&admissible_choices)
            .and_then(|selected| admissible_indices.get(selected).copied())
    }

    fn midreach_aligned_seed(
        model: &realityos_semantics::embodiment::EmbodimentModel,
        ee: &str,
        desired: [f64; 3],
        z: f64,
        tool_off: [f64; 3],
    ) -> Option<SampledEePose> {
        let chain = model.ee_joint_chain(ee)?;
        let n = chain.len();
        if n == 0 {
            return None;
        }
        let target = [0.30, 0.0, z];
        let mut guesses = vec![vec![0.0; n]];
        let mut g1 = vec![0.0; n];
        if n > 1 {
            g1[1] = -0.9;
        }
        if n > 0 {
            g1[0] = 0.45;
        }
        if n > 2 {
            g1[2] = 0.45;
        }
        guesses.push(g1);
        let mut g2 = vec![0.0; n];
        if n > 1 {
            g2[1] = 0.9;
        }
        if n > 0 {
            g2[0] = -0.45;
        }
        if n > 2 {
            g2[2] = -0.45;
        }
        guesses.push(g2);
        let mut g3 = vec![0.0; n];
        if n > 0 {
            g3[0] = 0.7;
        }
        if n > 1 {
            g3[1] = -1.3;
        }
        if n > 2 {
            g3[2] = 0.6;
        }
        guesses.push(g3);
        let mut g4 = vec![0.0; n];
        if n > 0 {
            g4[0] = 1.1;
        }
        if n > 1 {
            g4[1] = 0.9;
        }
        if n > 2 {
            g4[2] = 1.1;
        }
        guesses.push(g4);
        let mut best: Option<(SampledEePose, f64)> = None;
        for seed in guesses {
            let Ok((q, tr)) = solve_ik(model, &chain, ee, target, &seed) else {
                continue;
            };
            if !ik_residual_is_precise(tr.residual) {
                continue;
            }
            let Ok(fk) = forward_kinematics(model, &chain, ee, &q) else {
                continue;
            };
            let s = SampledEePose {
                xyz: fk.ee.xyz,
                quat_wxyz: fk.ee.quat_wxyz,
                q,
                joint_names: chain.clone(),
            };
            let align = planar_tool_axis(s.quat_wxyz, tool_off)
                .map(|ax| ax[0] * desired[0] + ax[1] * desired[1])
                .unwrap_or(-1.0);
            let margin = named_joint_limit_margin(&s.q, &s.joint_names, &model.joints);
            if margin < 0.04 || align < 0.85 {
                continue;
            }
            let better = match &best {
                None => true,
                Some((_, a)) => align > *a,
            };
            if better {
                best = Some((s, align));
            }
        }
        best.map(|(s, _)| s)
    }

    fn ee_from_truth(truth: &VerifierTruth, bundle: &RobotBundle) -> Option<[f64; 3]> {
        let names: Vec<String> = bundle
            .manifest
            .end_effectors
            .iter()
            .map(|e| e.name.clone())
            .collect();
        for k in names.iter().map(String::as_str) {
            if let Some(p) = truth.named_pos.get(k).or_else(|| truth.xpos.get(k)) {
                if p.len() >= 3 {
                    return Some([p[0], p[1], p[2]]);
                }
            }
        }
        None
    }

    fn ee_quat_from_truth(truth: &VerifierTruth, ee_name: &str) -> [f64; 4] {
        truth
            .site_xquat
            .get(ee_name)
            .or_else(|| truth.xquat.get(ee_name))
            .and_then(|q| {
                if q.len() >= 4 {
                    Some([q[0], q[1], q[2], q[3]])
                } else {
                    None
                }
            })
            .unwrap_or([1.0, 0.0, 0.0, 0.0])
    }

    fn geom_local_points(shape: &PrimitiveShape) -> Vec<[f64; 3]> {
        match *shape {
            PrimitiveShape::Box { half_extents } => {
                let h = half_extents;
                let mut v = Vec::with_capacity(8);
                for sx in [-1.0, 1.0] {
                    for sy in [-1.0, 1.0] {
                        for sz in [-1.0, 1.0] {
                            v.push([sx * h[0], sy * h[1], sz * h[2]]);
                        }
                    }
                }
                v
            }
            PrimitiveShape::Sphere { radius } => vec![
                [radius, 0.0, 0.0],
                [-radius, 0.0, 0.0],
                [0.0, radius, 0.0],
                [0.0, -radius, 0.0],
                [0.0, 0.0, radius],
                [0.0, 0.0, -radius],
            ],
            PrimitiveShape::Capsule {
                radius,
                half_length,
            }
            | PrimitiveShape::Cylinder {
                radius,
                half_length,
            } => vec![
                [0.0, 0.0, half_length],
                [0.0, 0.0, -half_length],
                [radius, 0.0, 0.0],
                [-radius, 0.0, 0.0],
            ],
            PrimitiveShape::Plane { .. } | PrimitiveShape::Unsupported { .. } => {
                vec![[0.0, 0.0, 0.0]]
            }
        }
    }

    fn offset_len(o: [f64; 3]) -> f64 {
        (o[0] * o[0] + o[1] * o[1] + o[2] * o[2]).sqrt()
    }

    /// Tool offset in the EE frame from declared intended-tool geometry, not invented +X.
    /// Distal points are those furthest along the live planar reach, averaged so a
    /// two-finger tool yields a centered contact rather than a single corner.
    fn derived_tool_offset_ee(
        model: &realityos_semantics::embodiment::EmbodimentModel,
        ee_name: &str,
        ee_xyz: [f64; 3],
        ee_quat: [f64; 4],
        truth: &VerifierTruth,
    ) -> [f64; 3] {
        let intended =
            declared_manipulation_contact_bodies(model, model.resources.first(), ee_name);
        let reach_n = ee_xyz[0].hypot(ee_xyz[1]);
        let reach = if reach_n > 1e-6 {
            [ee_xyz[0] / reach_n, ee_xyz[1] / reach_n, 0.0]
        } else {
            [1.0, 0.0, 0.0]
        };
        let mut pts: Vec<[f64; 3]> = Vec::new();
        let mut max_along = f64::NEG_INFINITY;
        for g in &model.collision_geoms {
            if !intended.iter().any(|n| n == &g.owner_body) {
                continue;
            }
            let Some(bx) = truth
                .xpos
                .get(&g.owner_body)
                .or_else(|| truth.named_pos.get(&g.owner_body))
            else {
                continue;
            };
            if bx.len() < 3 {
                continue;
            }
            let bq = truth
                .xquat
                .get(&g.owner_body)
                .and_then(|q| {
                    if q.len() >= 4 {
                        Some([q[0], q[1], q[2], q[3]])
                    } else {
                        None
                    }
                })
                .unwrap_or([1.0, 0.0, 0.0, 0.0]);
            let Ok(body) = Se3::try_new([bx[0], bx[1], bx[2]], bq) else {
                continue;
            };
            let gw = body.compose(g.local_pose);
            for local in geom_local_points(&g.shape) {
                let p = gw.transform_point(local);
                if (p[2] - ee_xyz[2]).abs() > 0.012 {
                    continue;
                }
                let along = p[0] * reach[0] + p[1] * reach[1];
                if along > max_along + 1e-4 {
                    max_along = along;
                    pts.clear();
                    pts.push(p);
                } else if (along - max_along).abs() <= 1e-4 {
                    pts.push(p);
                }
            }
        }
        if !pts.is_empty() && max_along.is_finite() {
            let n = pts.len() as f64;
            let avg = [
                pts.iter().map(|p| p[0]).sum::<f64>() / n,
                pts.iter().map(|p| p[1]).sum::<f64>() / n,
                pts.iter().map(|p| p[2]).sum::<f64>() / n,
            ];
            let o = tool_offset_in_ee(ee_quat, avg, ee_xyz);
            if offset_len(o) >= 0.008 {
                return o;
            }
        }
        for b in &intended {
            if let Some(p) = truth.xpos.get(b).or_else(|| truth.named_pos.get(b)) {
                if p.len() >= 3 {
                    let o = tool_offset_in_ee(ee_quat, [p[0], p[1], p[2]], ee_xyz);
                    if offset_len(o) >= 0.008 {
                        return o;
                    }
                }
            }
        }
        [0.0, 0.0, 0.0]
    }

    fn planar_tool_axis(quat: [f64; 4], tool_off: [f64; 3]) -> Option<[f64; 3]> {
        let w = rotate_by_quat(quat, tool_off);
        let n = (w[0] * w[0] + w[1] * w[1]).sqrt();
        if n < 1e-6 {
            return None;
        }
        Some([w[0] / n, w[1] / n, 0.0])
    }

    fn desired_planar_push(goal: &PlanarObjectGoal) -> [f64; 3] {
        let t = goal.target_xy.unwrap_or([0.07, 0.0]);
        let n = t[0].hypot(t[1]);
        if n > 1e-6 {
            [t[0] / n, t[1] / n, 0.0]
        } else {
            [1.0, 0.0, 0.0]
        }
    }

    fn select_seed_aligned<'a>(
        cloud: &'a [SampledEePose],
        tool_off: [f64; 3],
        desired: [f64; 3],
        joints: &[realityos_semantics::embodiment::Joint],
    ) -> Option<&'a SampledEePose> {
        // Leave stroke room inside the planar workspace (~0.41 m max reach).
        const R_LO: f64 = 0.20;
        const R_HI: f64 = 0.34;
        const R_PREF: f64 = 0.28;
        const MIN_MARGIN: f64 = 0.04;
        let mut best: Option<(&'a SampledEePose, f64, f64, f64)> = None;
        for s in cloud {
            let r = s.xyz[0].hypot(s.xyz[1]);
            if !(R_LO..=R_HI).contains(&r) {
                continue;
            }
            let margin = named_joint_limit_margin(&s.q, &s.joint_names, joints);
            if margin < MIN_MARGIN {
                continue;
            }
            let d = match planar_tool_axis(s.quat_wxyz, tool_off) {
                Some(ax) => ax[0] * desired[0] + ax[1] * desired[1],
                None => continue,
            };
            if d < 0.5 {
                continue;
            }
            let r_score = -(r - R_PREF).abs();
            let better = match best {
                None => true,
                Some((_, bd, bm, br)) => {
                    d > bd + 1e-9
                        || ((d - bd).abs() <= 1e-9 && margin > bm + 1e-9)
                        || ((d - bd).abs() <= 1e-9 && (margin - bm).abs() <= 1e-9 && r_score > br)
                }
            };
            if better {
                best = Some((s, d, margin, r_score));
            }
        }
        best.map(|(s, _, _, _)| s)
    }

    fn apply_sample_qpos(
        model: &realityos_semantics::embodiment::EmbodimentModel,
        ee: &str,
        qpos: &mut [f64],
        sample: &SampledEePose,
    ) {
        if let Some(chain) = model.ee_joint_chain(ee) {
            for (name, qi) in chain.iter().zip(sample.q.iter()) {
                if let Some(j) = model.joints.iter().find(|j| j.name == *name) {
                    if let Some(adr) = j.qpos_adr {
                        if let Some(slot) = qpos.get_mut(adr as usize) {
                            *slot = *qi;
                        }
                    }
                }
            }
        }
    }

    struct ExecOptions {
        /// Stop the authorized stroke at the first envelope failure.
        guard: bool,
        /// Extra object translation the development world adds after the
        /// selected witness executes. Decision code never reads this number.
        world_excess_m: f64,
    }

    impl ExecOptions {
        fn nominal() -> Self {
            Self {
                guard: false,
                world_excess_m: 0.0,
            }
        }
    }

    fn closed_loop_on_bundle(
        bundle_id: &str,
        start_xy: [f64; 2],
        goal: &PlanarObjectGoal,
        perturb_after: Option<[f64; 2]>,
        seed: u64,
    ) -> Result<ClosedLoopTrace, String> {
        closed_loop_exec(
            bundle_id,
            start_xy,
            goal,
            perturb_after,
            seed,
            ExecOptions::nominal(),
        )
    }

    fn closed_loop_exec(
        bundle_id: &str,
        start_xy: [f64; 2],
        goal: &PlanarObjectGoal,
        perturb_after: Option<[f64; 2]>,
        seed: u64,
        options: ExecOptions,
    ) -> Result<ClosedLoopTrace, String> {
        let bundle = RobotBundle::load(corpus::robot_dir(bundle_id))
            .map_err(|e| format!("load {bundle_id}: {e}"))?;
        if bundle.manifest.end_effectors.is_empty() {
            return Err("no_end_effector".into());
        }
        let (mut probe, man) = load_and_normalize(&bundle, &template_objects(true), 0)?;
        let t0 = truth_of(&mut probe)?;
        let ee = ee_from_truth(&t0, &bundle).ok_or_else(|| "no_ee_pose".to_string())?;
        let discovered =
            crate::resource_discover::discover_resources(&bundle, &man, &probe.inspect);
        let mut last_loaded = Some((probe, man.clone()));
        let mut qualified = Vec::new();
        for r in &discovered {
            if let Ok((_, q)) = crate::resource_qualify::qualify_resource(&bundle, r) {
                qualified.push(q);
            }
        }
        let mut model = embodiment_from_manifest(&bundle, &man);
        let ee_name = bundle.manifest.end_effectors[0].name.clone();
        let chain_len = model.ee_joint_chain(&ee_name).map(|c| c.len()).unwrap_or(0);
        if chain_len < 3 {
            if let Some((inst, _)) = last_loaded.take() {
                checkin_worker(inst);
            }
            return Err("insufficient_serial_chain".into());
        }
        model.resources = qualified;
        let ee_xyz0 = ee_from_truth(&t0, &bundle).unwrap_or(ee);
        let ee_q0 = ee_quat_from_truth(&t0, &ee_name);
        let tool_off = derived_tool_offset_ee(&model, &ee_name, ee_xyz0, ee_q0, &t0);
        let sha = "goal-directed-loop";
        let size = 0.025;
        let half = [size, size, size];
        let face_gap = 0.015;
        let _ = ee;
        let mut qpos = t0.qpos.clone();
        let mut cloud0 = build_ee_cloud(&model, &ee_name, &qpos, seed);
        let _ = start_xy;
        let desired = desired_planar_push(goal);
        if let Some(s) = midreach_aligned_seed(&model, &ee_name, desired, ee_xyz0[2], tool_off) {
            cloud0.insert(0, s);
        }
        let seed_pose = select_seed_aligned(&cloud0, tool_off, desired, &model.joints)
            .or_else(|| cloud0.first());
        let (mut xy, z) = if let Some(s) = seed_pose {
            apply_sample_qpos(&model, &ee_name, &mut qpos, s);
            let place_push = planar_tool_axis(s.quat_wxyz, tool_off).unwrap_or(desired);
            let center = object_center_for_sample(s, tool_off, place_push, half, face_gap)
                .unwrap_or([s.xyz[0] + size + face_gap, s.xyz[1], s.xyz[2]]);
            ([center[0], center[1]], center[2])
        } else {
            ([ee_xyz0[0] + size + face_gap, ee_xyz0[1]], ee_xyz0[2])
        };
        let mut goal = goal.clone();
        if let Some(t) = goal.target_xy {
            goal.target_xy = Some([xy[0] + t[0], xy[1] + t[1]]);
        }
        let goal = goal;
        let mut yaw = 0.0;
        let mut established_face: Option<String> = None;
        let mut state = LoopState::default();
        let mut trace = ClosedLoopTrace::new(bundle_id);
        trace.start_xy = xy;
        trace.goal_xy = goal.target_xy;
        trace.goal_yaw = goal.target_yaw;
        let mut last_obs: Option<WorldObservation> = None;
        let mut k = 0u32;
        let mut belief_state: Option<PhysicalParameterBelief> = None;
        let mut live_hypotheses: Vec<DiscrepancyKind> = Vec::new();
        let mut quasi_limit_m = 0.015_f64;
        loop {
            let obs = WorldObservation {
                object_id: "obj0".into(),
                xy,
                yaw,
                robot_q: qpos.clone(),
                freshness_ok: true,
                intended_contact_face: established_face.clone(),
                authority_ok: true,
                observed_at_s: k as f64,
            };
            if evaluate_goal_error(xy, yaw, &goal).reached {
                let step = receding_horizon_step(
                    &obs,
                    &goal,
                    &[],
                    state,
                    last_obs.as_ref().map(|o| (o, None)),
                );
                state = step.state.clone();
                record_action(&mut trace, &step.record, None, None, None);
                break;
            }
            let remain = evaluate_goal_error(xy, yaw, &goal).translation_residual_m;
            let (_, stroke, short_candidate_stroke) = candidate_stroke_plan(
                belief_state.as_ref(),
                &live_hypotheses,
                remain,
                quasi_limit_m,
            );
            let object_pose = pose_xy_yaw(xy, z, yaw);
            let mut cands = generate_planar_push_candidates(
                "obj0",
                object_pose,
                [size, size, size],
                [0.0, 0.0, 1.0],
                0.015,
                stroke,
            )
            .expect("valid fixture support geometry");
            if let Some(short_stroke) = short_candidate_stroke {
                let mut short_cands = generate_planar_push_candidates(
                    "obj0",
                    object_pose,
                    [size, size, size],
                    [0.0, 0.0, 1.0],
                    0.015,
                    short_stroke,
                )
                .expect("valid fixture support geometry");
                for cand in &mut short_cands {
                    cand.id = format!("qs:{short_stroke:.4}:{}", cand.id);
                }
                cands.extend(short_cands);
            }
            let object = BoxObject {
                center: [xy[0], xy[1], z],
                half_extents: [size, size, size],
                quat_wxyz: object_pose.quat_wxyz,
            };
            let support = SupportPlane {
                origin: [xy[0], xy[1], z - size],
                normal: [0.0, 0.0, 1.0],
            };
            let cloud = build_ee_cloud(&model, &ee_name, &qpos, seed.wrapping_add(k as u64));
            let ee_xyz = cloud.first().map(|s| s.xyz).unwrap_or(ee);
            let mut proven: std::collections::BTreeMap<
                String,
                Result<ContactManeuver, ContactInfeasible>,
            > = std::collections::BTreeMap::new();
            for c in &cands {
                let proof_key = format!("{}:{:.4}", c.face_id, c.stroke_m);
                if proven.contains_key(&proof_key) {
                    continue;
                }
                proven.insert(
                    proof_key,
                    prove_face(
                        &model,
                        &ee_name,
                        &cloud,
                        object,
                        support,
                        c.push_direction_world,
                        c.stroke_m,
                        ee_xyz,
                        tool_off,
                        c.stroke_m + 1e-9 < 0.02,
                    ),
                );
            }
            for c in &mut cands {
                let proof_key = format!("{}:{:.4}", c.face_id, c.stroke_m);
                let (reachable, collision, mut executable, maneuver, mut why) =
                    match proven.get(&proof_key) {
                        Some(Ok(m)) => {
                            let close = {
                                let d = [
                                    c.contact_point_world[0] - m.contact_point[0],
                                    c.contact_point_world[1] - m.contact_point[1],
                                ];
                                (d[0] * d[0] + d[1] * d[1]).sqrt() < 0.03
                            };
                            if close {
                                c.contact_point_world = m.contact_point;
                                c.contact_normal_world = m.contact_normal;
                                c.push_direction_world = m.push_direction;
                                (Some(true), Some(true), Some(true), Some(m.clone()), None)
                            } else {
                                (
                                    Some(true),
                                    Some(true),
                                    Some(false),
                                    None,
                                    Some("NON_EXECUTABLE".into()),
                                )
                            }
                        }
                        Some(Err(e)) => {
                            let (r, col, ex) = flags_from_infeasible(*e);
                            (r, col, ex, None, Some(e.as_str().to_string()))
                        }
                        None => (Some(false), None, None, None, Some("NO_PROOF".into())),
                    };
                if !install_proven_maneuver(c, maneuver) {
                    executable = Some(false);
                    why = Some("INVALID_EXECUTABLE_WITNESS_STROKE".into());
                }
                let regime = belief_state.as_ref().map(|belief| {
                    prediction_regime(belief, &live_hypotheses, c.stroke_m, quasi_limit_m)
                });
                let mechanics_physics =
                    physics_from_belief(PUSH_SCENARIO_PHYSICS, belief_state.as_ref());
                let mut mechanics_template = fill_mechanics_from_q(
                    &model,
                    &ee_name,
                    &qpos,
                    c.contact_point_world,
                    yaw,
                    &mechanics_physics,
                );
                if let (Some(mech), Some(regime)) = (mechanics_template.as_mut(), regime) {
                    mech.quasi_static = regime.quasi_static;
                }
                let ctx = EvaluationContext {
                    goal: goal.clone(),
                    object_xy: xy,
                    object_yaw: yaw,
                    object_com_world: [xy[0], xy[1], z],
                    mechanics_template,
                    authority_ok: true,
                    robot_provided: true,
                    robot_reachable: reachable,
                    collision_admissible: collision,
                    executable_witness: executable,
                    robot_reject_reason: why,
                };
                evaluate_candidate(c, &ctx);
            }
            let reach = cloud
                .iter()
                .map(|sample| {
                    let d0 = sample.xyz[0] - xy[0];
                    let d1 = sample.xyz[1] - xy[1];
                    (d0 * d0 + d1 * d1).sqrt()
                })
                .fold(0.0_f64, f64::max);
            let recoverability_choices: Vec<RecoverabilityChoice> = cands
                .iter()
                .map(|c| {
                    let push = c.push_direction_world;
                    let nrm = (push[0] * push[0] + push[1] * push[1]).sqrt().max(1e-9);
                    let strict = c.goal_progress
                        == Some(
                            realityos_semantics::planar_goal::GoalProgressClass::StrictProgress,
                        );
                    let regime = belief_state.as_ref().map(|belief| {
                        prediction_regime(belief, &live_hypotheses, c.stroke_m, quasi_limit_m)
                    });
                    let outside_regime = regime.is_some_and(|regime| !regime.quasi_static);
                    let mut class = classify_recoverability(&RecoverabilityInput {
                        physically_feasible: c.executable_for_plant && strict && !outside_regime,
                        makes_progress: strict,
                        current_xy: xy,
                        nominal_dxy: Some([push[0] / nrm * c.stroke_m, push[1] / nrm * c.stroke_m]),
                        uncertainty_radius_m: Some(if outside_regime {
                            reach.max(c.stroke_m) + 0.05
                        } else {
                            0.005
                        }),
                        region: Some(InteractionRegion {
                            center_xy: xy,
                            radius_m: reach.max(0.02),
                        }),
                        next_contact_admissible: Some(c.executable_for_plant && !outside_regime),
                    });
                    if strict && outside_regime {
                        class = RecoverabilityClass::ProgressButCanEnterUnrecoverableState;
                    }
                    RecoverabilityChoice {
                        id: c.id.clone(),
                        class,
                        progress: -c.predicted_error_derivative.unwrap_or(0.0),
                    }
                })
                .collect();
            let mut step = receding_horizon_step(
                &obs,
                &goal,
                &cands,
                state,
                last_obs.as_ref().map(|o| (o, None)),
            );
            let override_indices =
                admissible_override_indices(&cands, &step.state.forbidden_action_keys);
            let recoverable_index = select_recoverable_override_index(
                &cands,
                &recoverability_choices,
                &step.state.forbidden_action_keys,
            );
            if belief_state.is_some() {
                let proved_pick = recoverable_index
                    .map(|index| cands[index].id.clone())
                    .unwrap_or_else(|| "none".into());
                if let Some(prev) = trace.actions.last_mut() {
                    let probed = prev
                        .pointer("/reasoning/probe_displacement_m")
                        .and_then(|value| value.as_f64())
                        .is_some();
                    if probed {
                        if let Some(note) = prev.get_mut("reasoning") {
                            let prior = note
                                .get("ranking_after")
                                .and_then(|value| value.as_str())
                                .unwrap_or("");
                            let experience =
                                prior.split_once('|').map(|(_, rest)| rest).unwrap_or("");
                            let class = recoverable_index
                                .map(|index| format!("{:?}", recoverability_choices[index].class))
                                .unwrap_or_else(|| "none".into());
                            let ranking = if experience.is_empty() {
                                format!("{proved_pick}|class={class}")
                            } else {
                                format!("{proved_pick}|class={class}|{experience}")
                            };
                            note["ranking_after"] = json!(ranking);
                        }
                    }
                }
                let long_regime = belief_state
                    .as_ref()
                    .map(|belief| prediction_regime(belief, &live_hypotheses, 0.03, quasi_limit_m));
                let friction_contradicted = long_regime
                    .map(|regime| regime.friction_contradicted)
                    .unwrap_or(false);
                if friction_contradicted {
                    let mut kinds = live_hypotheses.clone();
                    if !kinds.contains(&DiscrepancyKind::SupportFrictionInconsistent) {
                        kinds.push(DiscrepancyKind::SupportFrictionInconsistent);
                    }
                    let ranked_contacts: Vec<_> = override_indices
                        .iter()
                        .map(|&index| {
                            let cand = &cands[index];
                            let progress = cand.goal_progress.unwrap_or(
                                realityos_semantics::planar_goal::GoalProgressClass::Neutral,
                            );
                            goal_contact_at_contradicted_declared_friction(
                                cand.id.clone(),
                                cand.stroke_m,
                                -cand.predicted_error_derivative.unwrap_or(0.0),
                                progress,
                                recoverability_choices[index].class,
                                cand.executable_for_plant && cand.maneuver.is_some(),
                            )
                        })
                        .collect();
                    let ranking = rank_goal_or_probe(&ranked_contacts, &kinds, quasi_limit_m);
                    let robust_goal = ranking.selected_class == Some(DecisionClass::GoalAction)
                        && ranking.robustness.iter().any(|(id, robust)| {
                            ranking.selected_id.as_ref() == Some(id)
                                && *robust == BeliefRobustness::RobustStrictProgress
                        });
                    if robust_goal {
                        if let Some(id) = ranking.selected_id.clone() {
                            if let Some(index) = override_indices
                                .iter()
                                .copied()
                                .find(|&index| cands[index].id == id)
                            {
                                if step.selected.is_none() {
                                    step.state.attempts = step.state.attempts.saturating_add(1);
                                }
                                let stroke_regime = belief_state.as_ref().map(|belief| {
                                    prediction_regime(
                                        belief,
                                        &live_hypotheses,
                                        cands[index].stroke_m,
                                        quasi_limit_m,
                                    )
                                });
                                step.selected = Some(cands[index].clone());
                                step.record.selected_id = Some(cands[index].id.clone());
                                step.record.selected_face = Some(cands[index].face_id.clone());
                                step.record.selection_rationale = format!(
                                    "ROBUST_STRICT_PROGRESS {} class={:?} stroke_m={:.4} declared_mu={} friction_contradicted={} long_quasi_static={} stroke_quasi_static={}",
                                    cands[index].id,
                                    recoverability_choices[index].class,
                                    cands[index].stroke_m,
                                    rationale_mu(long_regime.map(|r| r.support_friction)),
                                    true,
                                    long_regime.map(|r| r.quasi_static).unwrap_or(true),
                                    stroke_regime.map(|r| r.quasi_static).unwrap_or(true)
                                );
                                step.record.authority_decision = "AUTHORIZE".into();
                                step.record.decision = LoopDecision::Continue;
                                step.record.outcome = GoalLoopOutcome::GoalProgress;
                                step.record.predicted_twist = cands[index].predicted_twist;
                                step.record.predicted_goal_progress = cands[index].goal_progress;
                            }
                        }
                    } else {
                        let picked = recoverable_index.map(|index| cands[index].id.clone());
                        for (id, why) in &ranking.refused {
                            step.record.rejection_reasons.push(format!("{id}:{why}"));
                        }
                        step.selected = None;
                        step.record.selected_id = None;
                        step.record.selected_face = None;
                        step.record.predicted_twist = None;
                        step.record.predicted_goal_progress = None;
                        let picked_reason = picked
                            .as_ref()
                            .and_then(|id| {
                                ranking
                                    .refused
                                    .iter()
                                    .find(|(rid, _)| rid == id)
                                    .map(|(_, why)| why.clone())
                            })
                            .unwrap_or_else(|| "NO_ROBUST_STRICT_PROGRESS".into());
                        step.record.authority_decision = "REFUSE".into();
                        step.record.decision = LoopDecision::Refuse;
                        step.record.outcome = GoalLoopOutcome::InsufficientEvidence;
                        step.record.unauthorized_writes = 0;
                        step.record.first_divergence = Some(picked_reason.clone());
                        step.record.selection_rationale = format!(
                            "BELIEF_SET_REFUSAL declared_mu={} friction_status={:?} quasi_status={:?} long_quasi_static={} recoverable_pick={} robust_goal=false reason={}",
                            rationale_mu(long_regime.map(|r| r.support_friction)),
                            belief_state.as_ref().and_then(|belief| {
                                belief
                                    .entry(PhysicalParameter::SupportFriction)
                                    .map(|entry| entry.status)
                            }),
                            belief_state.as_ref().and_then(|belief| {
                                belief
                                    .entry(PhysicalParameter::QuasiStaticApplicability)
                                    .map(|entry| entry.status)
                            }),
                            long_regime.map(|r| r.quasi_static).unwrap_or(true),
                            picked.as_deref().unwrap_or("none"),
                            picked_reason.clone()
                        );
                        if let Some(belief) = belief_state.as_ref() {
                            step.record.reasoning = Some(ReasoningNote {
                                hypotheses: kinds.iter().map(|kind| format!("{kind:?}")).collect(),
                                hypothesis_status: Some(
                                    match kinds.len() {
                                        0 => "Unknown",
                                        1 => "Identified",
                                        _ => "Underdetermined",
                                    }
                                    .into(),
                                ),
                                belief_before: Some(format!(
                                    "support_friction={:?} quasi_static={:?}",
                                    belief.declared_value(PhysicalParameter::SupportFriction),
                                    belief
                                        .entry(PhysicalParameter::QuasiStaticApplicability)
                                        .map(|entry| entry.status)
                                )),
                                belief_after: Some(format!(
                                    "carried|mu={:?}|friction={:?}|quasi_static={:?}",
                                    belief.declared_value(PhysicalParameter::SupportFriction),
                                    belief
                                        .entry(PhysicalParameter::SupportFriction)
                                        .map(|entry| entry.status),
                                    belief
                                        .entry(PhysicalParameter::QuasiStaticApplicability)
                                        .map(|entry| (entry.status, entry.declared.value))
                                )),
                                ranking_before: picked.clone(),
                                ranking_after: Some(
                                    picked.clone().unwrap_or_else(|| "none".into()),
                                ),
                                selected_kind: None,
                                information_gain: Some(ranking.information_gain),
                                recoverability: Some("NO_ROBUST_STRICT_PROGRESS".into()),
                                taxonomy: Some(picked_reason.clone()),
                                admissible_contact_count: Some(
                                    override_indices
                                        .iter()
                                        .filter(|&&index| cands[index].maneuver.is_some())
                                        .count() as u32,
                                ),
                                interactable_after_abort: Some(
                                    override_indices
                                        .iter()
                                        .any(|&index| cands[index].maneuver.is_some()),
                                ),
                                ..ReasoningNote::default()
                            });
                        }
                    }
                } else if let Some(index) = recoverable_index {
                    if cands[index].executable_for_plant && cands[index].maneuver.is_some() {
                        if step.selected.is_none() {
                            step.state.attempts = step.state.attempts.saturating_add(1);
                        }
                        step.selected = Some(cands[index].clone());
                        step.record.selected_id = Some(cands[index].id.clone());
                        step.record.selected_face = Some(cands[index].face_id.clone());
                        let stroke_regime = belief_state.as_ref().map(|belief| {
                            prediction_regime(
                                belief,
                                &live_hypotheses,
                                cands[index].stroke_m,
                                quasi_limit_m,
                            )
                        });
                        step.record.selection_rationale = format!(
                            "RECOVERABLE_PROGRESS {} class={:?} stroke_m={:.4} declared_mu={} friction_contradicted={} long_quasi_static={} stroke_quasi_static={}",
                            cands[index].id,
                            recoverability_choices[index].class,
                            cands[index].stroke_m,
                            rationale_mu(long_regime.map(|r| r.support_friction)),
                            long_regime.map(|r| r.friction_contradicted).unwrap_or(false),
                            long_regime.map(|r| r.quasi_static).unwrap_or(true),
                            stroke_regime.map(|r| r.quasi_static).unwrap_or(true)
                        );
                        step.record.authority_decision = "AUTHORIZE".into();
                        step.record.decision = LoopDecision::Continue;
                        step.record.outcome = GoalLoopOutcome::GoalProgress;
                        step.record.predicted_twist = cands[index].predicted_twist;
                        step.record.predicted_goal_progress = cands[index].goal_progress;
                    }
                }
            }
            state = step.state.clone();
            if matches!(
                step.record.decision,
                LoopDecision::Halt | LoopDecision::Refuse
            ) || matches!(
                step.record.outcome,
                GoalLoopOutcome::GoalReached
                    | GoalLoopOutcome::GoalCurrentlyUnachievable
                    | GoalLoopOutcome::InsufficientPhysicalEvidence
                    | GoalLoopOutcome::AuthorityRefusal
                    | GoalLoopOutcome::InsufficientEvidence
                    | GoalLoopOutcome::GoalPhysicallyInfeasible
            ) {
                record_action(&mut trace, &step.record, step.selected.as_ref(), None, None);
                break;
            }
            let Some(sel) = step.selected.clone() else {
                record_action(&mut trace, &step.record, None, None, None);
                break;
            };
            if !sel.executable_for_plant || sel.maneuver.is_none() {
                let mut rec = step.record.clone();
                rec.decision = LoopDecision::Refuse;
                rec.outcome = GoalLoopOutcome::GoalCurrentlyUnachievable;
                rec.selection_rationale = "NO_EXECUTABLE_SELECTED_WITNESS".into();
                record_action(&mut trace, &rec, Some(&sel), None, None);
                break;
            }
            let selected_regime = belief_state.as_ref().map(|belief| {
                prediction_regime(belief, &live_hypotheses, sel.stroke_m, quasi_limit_m)
            });
            let selected_physics =
                physics_from_belief(PUSH_SCENARIO_PHYSICS, belief_state.as_ref());
            let Some(mut mech) = fill_mechanics_from_q(
                &model,
                &ee_name,
                &qpos,
                sel.contact_point_world,
                yaw,
                &selected_physics,
            ) else {
                let mut refusal = step.record.clone();
                refusal.decision = LoopDecision::Refuse;
                refusal.outcome = GoalLoopOutcome::InsufficientEvidence;
                refusal.authority_decision = "REFUSE".into();
                refusal.selection_rationale = format!(
                    "MECHANICS_UNKNOWN declared_mu={}",
                    rationale_mu(selected_physics.reasoner_support_friction)
                );
                record_action(&mut trace, &refusal, Some(&sel), None, None);
                break;
            };
            if let Some(regime) = selected_regime {
                mech.quasi_static = regime.quasi_static;
            }
            let init = initiation_from_candidate(&mech, &sel, [xy[0], xy[1], z], yaw, true);
            let frozen = FrozenMechanicsPrediction::freeze(
                sel.witness.clone().unwrap_or_else(|| {
                    realityos_semantics::effect_feasibility::evaluate_planar_twist_direction(&init)
                }),
                &init,
            );
            assert!(
                !frozen.contains_privileged_force(),
                "privileged simulator force leaked into predictor"
            );
            let predicted_yaw_sign = frozen.witness.rotation_sign.and_then(|sign| match sign {
                realityos_physics::RotationSign::Clockwise => Some(-1),
                realityos_physics::RotationSign::Counterclockwise => Some(1),
                realityos_physics::RotationSign::TranslationOnly => Some(0),
                realityos_physics::RotationSign::Ambiguous
                | realityos_physics::RotationSign::Unknown => None,
            });
            let full_stroke = selected_witness_stroke(&sel)
                .expect("selected executable candidate has a finite witness stroke");
            let scenario_strokes = execution_scenario_strokes(full_stroke);
            let action_xy = xy;
            let action_yaw = yaw;
            let mut nxy = xy;
            let mut nyaw = yaw;
            let mut ep_last = None;
            let mut consumed = 0.0;
            let mut abort_at: Option<u32> = None;
            let mut guarded_displacement = 0.0;
            let mut prevention_impossible = false;
            let mut failed_guard = None;
            for (step_i, piece) in scenario_strokes {
                let mut sc = push_scenario(
                    [nxy[0], nxy[1], z],
                    size,
                    PUSH_SCENARIO_PHYSICS,
                    sel.push_direction_world,
                    piece,
                    seed.wrapping_add(k as u64).wrapping_add(u64::from(step_i)),
                );
                sc.world_construction =
                    realityos_semantics::contact_maneuver::WorldConstructionMode::FixedWorld;
                let loaded = last_loaded.take();
                let maneuver = sel.maneuver.clone();
                let qpos_now = qpos.clone();
                let run = with_episode_qpos(Some(&qpos_now), || {
                    with_injected_push_maneuver(maneuver, || {
                        run_skill_episode_ex(&bundle, &model, &[], &sc, sha, "PUSH", loaded, None)
                    })
                });
                let (ep_i, mut inst, man) = run?;
                let mut truth = truth_of(&mut inst).unwrap_or_default();
                if k == 0 && step_i == 0 {
                    if let Some(p) = perturb_after {
                        let cur = body_xyz(&truth, "obj0").unwrap_or([nxy[0], nxy[1], z]);
                        let disp = [cur[0] + p[0], cur[1] + p[1], cur[2]];
                        let _ = inst.set_body_pos("obj0", disp);
                        if let Ok(t) = truth_of(&mut inst) {
                            truth = t;
                        }
                    }
                }
                consumed += piece;
                if options.world_excess_m > 0.0 && piece >= 0.01 {
                    let cur = body_xyz(&truth, "obj0").unwrap_or([nxy[0], nxy[1], z]);
                    let dir = sel.push_direction_world;
                    let nrm = (dir[0] * dir[0] + dir[1] * dir[1]).sqrt().max(1e-9);
                    let moved = [
                        cur[0] + dir[0] / nrm * options.world_excess_m,
                        cur[1] + dir[1] / nrm * options.world_excess_m,
                        cur[2],
                    ];
                    inst.set_body_pos("obj0", moved)
                        .map_err(|e| e.to_string())?;
                    truth = truth_of(&mut inst)?;
                }
                let (oxy, oyaw) = object_xy_yaw(&ep_i, &truth);
                nxy = oxy;
                nyaw = oyaw;
                last_loaded = Some((inst, man));
                qpos = truth.qpos.clone();
                let disp =
                    ((nxy[0] - action_xy[0]).powi(2) + (nxy[1] - action_xy[1]).powi(2)).sqrt();
                guarded_displacement = disp;
                ep_last = Some(ep_i);
                if options.guard {
                    let err_before = evaluate_goal_error(action_xy, action_yaw, &goal);
                    let err_now = evaluate_goal_error(nxy, nyaw, &goal);
                    let sample = RuntimeExecutionObservation {
                        stroke_consumed_m: consumed,
                        commanded_stroke_m: full_stroke,
                        object_displacement_m: disp,
                        yaw_change_rad: realityos_semantics::planar_goal::wrap_pi(
                            nyaw - action_yaw,
                        ),
                        intended_contact_persists: ep_last
                            .as_ref()
                            .is_some_and(|ep| ep.intended_tool_contact),
                        goal_error_before: err_before.combined,
                        goal_error_now: err_now.combined,
                        robot_tracking_error_m: None,
                        reachability_margin_m: 0.25 - disp,
                        quasi_static_applicable: Some(
                            consumed <= 1e-9 || disp / consumed <= QUASI_STATIC_DISPLACEMENT_RATIO,
                        ),
                        authority_ok: ep_last
                            .as_ref()
                            .is_none_or(|ep| ep.unauthorized_writes == 0),
                    };
                    let envelope =
                        ExecutionEnvelope::for_quasi_static_stroke(full_stroke, full_stroke);
                    let check = check_execution_envelope(&envelope, &sample);
                    if check.verdict == EnvelopeVerdict::AbortAndReobserve {
                        abort_at = Some(step_i);
                        prevention_impossible = check.prevention_impossible;
                        failed_guard = check.failed_guard.clone();
                        let _ = check.early;
                        break;
                    }
                }
            }
            let ep = ep_last.ok_or_else(|| "stroke produced no episode".to_string())?;
            let contact_established = ep.intended_tool_contact;
            let after = WorldObservation {
                object_id: "obj0".into(),
                xy: nxy,
                yaw: nyaw,
                robot_q: qpos.clone(),
                freshness_ok: true,
                intended_contact_face: if contact_established {
                    Some(sel.face_id.clone())
                } else {
                    None
                },
                authority_ok: ep.unauthorized_writes == 0,
                observed_at_s: (k + 1) as f64,
            };
            let mut rec = record_after_with_goal(step, &after, &goal);
            let flung = rec
                .record
                .goal_error_after
                .as_ref()
                .zip(rec.record.goal_error_before.as_ref())
                .is_some_and(|(a, b)| a.translation_residual_m > b.translation_residual_m + 0.06);
            let blocked = flung
                || ep.ctrl_writes == 0
                || matches!(
                    ep.failure_taxonomy.as_deref(),
                    Some("COLLISION_INADMISSIBLE") | Some("NO_FEASIBLE_CONTACT_POSE")
                );
            let mut probe_unauthorized = 0u64;
            if options.guard || options.world_excess_m > 0.0 {
                let limit = full_stroke / 4.0;
                let decision_candidates = candidates_for_uncertainty(limit);
                let ratio = if consumed > 1e-9 {
                    guarded_displacement / consumed
                } else {
                    0.0
                };
                let contact_persisted = ep.intended_tool_contact;
                let slip_obs = observed_discrepancy_without_residuals(
                    ratio,
                    realityos_semantics::planar_goal::wrap_pi(nyaw - action_yaw),
                    predicted_yaw_sign,
                    contact_persisted,
                    consumed,
                    limit,
                );
                let report = hypothesize(&slip_obs);
                let ranking_before = rank_goal_or_probe(&decision_candidates, &report.kinds, limit);
                let belief = if let Some(carried) = belief_state.clone() {
                    carried
                } else {
                    fresh_belief_from_physics(&PUSH_SCENARIO_PHYSICS)
                };
                let carried = belief_state.is_some();
                let mut belief_after_text = if carried {
                    format!(
                        "carried|mu={:?}|friction={:?}|quasi_static={:?}",
                        belief.declared_value(PhysicalParameter::SupportFriction),
                        belief
                            .entry(PhysicalParameter::SupportFriction)
                            .map(|entry| entry.status),
                        belief
                            .entry(PhysicalParameter::QuasiStaticApplicability)
                            .map(|entry| (entry.status, entry.declared.value))
                    )
                } else {
                    format!("{:?}", report.status)
                };
                let mut ranking_after_id = String::new();
                let mut ranking_before_id = ranking_before.selected_id.clone();
                let mut selected_kind = ranking_before
                    .selected_class
                    .map(|class| format!("{class:?}"));
                if carried {
                    selected_kind = Some("GoalAction".into());
                    ranking_before_id = rec.record.selected_id.clone();
                    ranking_after_id = rec.record.selected_id.clone().unwrap_or_default();
                }
                let mut probe_displacement_m = None;
                let mut probe_contact_persisted = None;
                let mut admissible_contact_count = None;
                let mut executable_probe_available = false;
                if options.guard && abort_at.is_some() {
                    let (admissible, _) = proved_contacts_at(
                        &model,
                        &ee_name,
                        &qpos,
                        nxy,
                        z,
                        nyaw,
                        size,
                        full_stroke.max(0.02),
                        tool_off,
                        seed.wrapping_add(80_000),
                        ee,
                        sel.push_direction_world,
                        false,
                    );
                    admissible_contact_count = Some(admissible);
                    let interactable_now = admissible > 0;
                    if abort_at == Some(0) && !interactable_now {
                        prevention_impossible = true;
                    }
                    if belief_state.is_none()
                        && ranking_before.selected_class == Some(DecisionClass::PhysicalProbe)
                        && interactable_now
                    {
                        let mut probe_stroke = decision_candidates
                            .iter()
                            .find(|c| Some(&c.id) == ranking_before.selected_id.as_ref())
                            .map(|c| c.stroke_m)
                            .unwrap_or(limit * 0.4)
                            .max(1e-4);
                        let (probe_admissible, probe_maneuver) = proved_contacts_at(
                            &model,
                            &ee_name,
                            &qpos,
                            nxy,
                            z,
                            nyaw,
                            size,
                            probe_stroke,
                            tool_off,
                            seed.wrapping_add(81_000),
                            ee,
                            sel.push_direction_world,
                            true,
                        );
                        admissible_contact_count = Some(admissible.max(probe_admissible));
                        executable_probe_available =
                            probe_admissible > 0 && probe_maneuver.is_some();
                        let probe_origin = nxy;
                        let observed = if let Some(maneuver) = probe_maneuver {
                            let executed = executed_witness_stroke(&maneuver);
                            probe_stroke = executed;
                            let mut probe_sc = push_scenario(
                                [nxy[0], nxy[1], z],
                                size,
                                PUSH_SCENARIO_PHYSICS,
                                maneuver.push_direction,
                                probe_stroke,
                                seed.wrapping_add(90_000),
                            );
                            probe_sc.world_construction =
                                realityos_semantics::contact_maneuver::WorldConstructionMode::FixedWorld;
                            let loaded = last_loaded.take();
                            let qpos_now = qpos.clone();
                            let probe_run = with_episode_qpos(Some(&qpos_now), || {
                                with_injected_push_maneuver(Some(maneuver), || {
                                    run_skill_episode_ex(
                                        &bundle,
                                        &model,
                                        &[],
                                        &probe_sc,
                                        sha,
                                        "PUSH",
                                        loaded,
                                        None,
                                    )
                                })
                            });
                            let (probe_ep, mut probe_inst, probe_man) = probe_run?;
                            probe_unauthorized = probe_ep.unauthorized_writes;
                            let probe_truth = truth_of(&mut probe_inst)?;
                            let (probe_xy, probe_yaw) = object_xy_yaw(&probe_ep, &probe_truth);
                            last_loaded = Some((probe_inst, probe_man));
                            qpos = probe_truth.qpos.clone();
                            let probe_disp = ((probe_xy[0] - probe_origin[0]).powi(2)
                                + (probe_xy[1] - probe_origin[1]).powi(2))
                            .sqrt();
                            let contact = probe_ep.intended_tool_contact;
                            probe_displacement_m = Some(probe_disp);
                            probe_contact_persisted = Some(contact);
                            nxy = probe_xy;
                            nyaw = probe_yaw;
                            tag_probe_motion(probe_disp, probe_stroke, contact)
                        } else {
                            probe_displacement_m = Some(0.0);
                            probe_contact_persisted = Some(false);
                            tag_probe_motion(0.0, probe_stroke, false)
                        };
                        let update = apply_probe_observation(
                            &belief,
                            &report.kinds,
                            Stimulus {
                                stroke_m: probe_stroke,
                                quasi_static_stroke_limit_m: limit,
                            },
                            observed,
                            "probe-observation",
                        );
                        let ranking_after =
                            rank_goal_or_probe(&decision_candidates, &update.remaining, limit);
                        belief_after_text = format!(
                            "{:?}|{}|mu={:?}",
                            update.status,
                            update
                                .remaining
                                .iter()
                                .map(|kind| format!("{kind:?}"))
                                .collect::<Vec<_>>()
                                .join(","),
                            update
                                .belief
                                .declared_value(PhysicalParameter::SupportFriction)
                        );
                        selected_kind = ranking_before
                            .selected_class
                            .map(|class| format!("{class:?}"));
                        let regime = ApplicabilityQuery {
                            world_model_id: goal.world_id.clone(),
                            embodiment_applicability: "serial-planar".into(),
                            object_support_geometry: "box-on-plane".into(),
                        };
                        let stored = PhysicalExperienceRecord {
                            world_model_id: regime.world_model_id.clone(),
                            embodiment_applicability: regime.embodiment_applicability.clone(),
                            object_support_geometry: regime.object_support_geometry.clone(),
                            belief_before: belief.clone(),
                            selected_action: ranking_before.selected_id.clone().unwrap_or_default(),
                            frozen_prediction: format!("{observed:?}"),
                            authority_result: rec.record.authority_decision.clone(),
                            execution_envelope: "ABORT_AND_REOBSERVE".into(),
                            observed_consequence: format!(
                                "{observed:?} disp={probe_displacement_m:?} contact={probe_contact_persisted:?}"
                            ),
                            first_divergence: failed_guard.clone(),
                            hypotheses_before: report.kinds.clone(),
                            hypotheses_after: update.remaining.clone(),
                            belief_after: update.belief.clone(),
                            goal_effect: format!("{:?}", rec.record.outcome),
                            recoverability_result: format!(
                                "admissible={} ranking={:?}",
                                admissible_contact_count.unwrap_or(0),
                                ranking_after.selected_id
                            ),
                            provenance: "development-slip".into(),
                        };
                        let auth = authorize_from_experience(&stored);
                        probe_unauthorized =
                            probe_unauthorized.saturating_add(auth.unauthorized_writes);
                        let mut log = ExperienceLog::default();
                        log.append(stored);
                        let later =
                            initial_decision(&log, &regime, &decision_candidates, &belief, limit);
                        let fresh = initial_decision(
                            &ExperienceLog::default(),
                            &regime,
                            &decision_candidates,
                            &belief,
                            limit,
                        );
                        ranking_after_id = format!(
                            "{}|later={}|fresh={}",
                            ranking_after.selected_id.clone().unwrap_or_default(),
                            later.ranking.selected_id.clone().unwrap_or_default(),
                            fresh.ranking.selected_id.clone().unwrap_or_default()
                        );
                        belief_state = Some(update.belief);
                        live_hypotheses = update.remaining;
                        quasi_limit_m = limit;
                        let _ = (later.used_experience, fresh.used_experience, auth.allow);
                    }
                    rec.record.decision = LoopDecision::Replan;
                    if rec.record.outcome == GoalLoopOutcome::GoalReached {
                        rec.record.outcome = GoalLoopOutcome::GoalProgress;
                    }
                    rec.record.first_divergence = failed_guard
                        .clone()
                        .or_else(|| Some("ABORT_AND_REOBSERVE".into()));
                }
                let admissible = admissible_contact_count.unwrap_or(0);
                let interactable = admissible > 0;
                let pose_label = if interactable {
                    format!("ADMISSIBLE_CONTACTS={admissible}")
                } else if options.guard && abort_at.is_some() {
                    format!(
                        "{:?}|ADMISSIBLE_CONTACTS=0",
                        goal_status_if_no_admissible_interaction(0)
                    )
                } else {
                    "NOT_PROVED".into()
                };
                rec.record.reasoning = Some(ReasoningNote {
                    envelope_verdict: Some(match abort_at {
                        Some(index) => format!("ABORT_AND_REOBSERVE:{index}"),
                        None => "CONTINUE".into(),
                    }),
                    failed_guard: failed_guard.clone(),
                    prevention_impossible,
                    hypotheses: report
                        .kinds
                        .iter()
                        .map(|kind| format!("{kind:?}"))
                        .collect(),
                    hypothesis_status: Some(format!("{:?}", report.status)),
                    belief_before: Some(format!(
                        "support_friction={:?} quasi_static={:?}",
                        belief.declared_value(PhysicalParameter::SupportFriction),
                        belief
                            .entry(PhysicalParameter::QuasiStaticApplicability)
                            .map(|e| e.status)
                    )),
                    belief_after: Some(belief_after_text),
                    ranking_before: ranking_before_id,
                    ranking_after: Some(ranking_after_id),
                    selected_kind,
                    information_gain: Some(ranking_before.information_gain),
                    recoverability: Some(pose_label),
                    taxonomy: Some(
                        reasoning_taxonomy(
                            prevention_impossible,
                            interactable,
                            ranking_before.selected_class == Some(DecisionClass::PhysicalProbe),
                            executable_probe_available,
                            report.status,
                            abort_at,
                        )
                        .into(),
                    ),
                    displacement_m: Some(guarded_displacement),
                    interactable_after_abort: Some(interactable),
                    probe_displacement_m,
                    probe_contact_persisted,
                    admissible_contact_count,
                });
            }
            if blocked && abort_at.is_none() {
                let key = sel.action_key();
                if !rec.state.forbidden_action_keys.contains(&key) {
                    rec.state.forbidden_action_keys.push(key);
                }
                rec.record.decision = LoopDecision::Recover;
            }
            record_action(
                &mut trace,
                &rec.record,
                Some(&sel),
                Some(&ep),
                Some(&frozen),
            );
            trace.unauthorized_writes =
                trace.unauthorized_writes.saturating_add(probe_unauthorized);
            if ep.unauthorized_writes > 0 || probe_unauthorized > 0 {
                checkin_worker(last_loaded.take().unwrap().0);
                return Ok(trace);
            }
            xy = nxy;
            yaw = nyaw;
            established_face = after.intended_contact_face.clone();
            last_obs = Some(after);
            state = rec.state;
            k += 1;
            if rec.record.outcome == GoalLoopOutcome::GoalReached {
                break;
            }
        }
        let terminal = [
            "GoalReached",
            "GoalCurrentlyUnachievable",
            "InsufficientPhysicalEvidence",
            "AuthorityRefusal",
            "InsufficientEvidence",
            "GoalPhysicallyInfeasible",
            "ModelNotApplicable",
        ]
        .contains(&trace.final_outcome.as_str());
        if !terminal {
            let obs = WorldObservation {
                object_id: "obj0".into(),
                xy,
                yaw,
                robot_q: qpos,
                freshness_ok: true,
                intended_contact_face: established_face,
                authority_ok: true,
                observed_at_s: k as f64,
            };
            state.attempts = goal.max_bounded_attempts;
            let step = receding_horizon_step(
                &obs,
                &goal,
                &[],
                state,
                last_obs.as_ref().map(|o| (o, None)),
            );
            record_action(&mut trace, &step.record, None, None, None);
        }
        if let Some((inst, _)) = last_loaded {
            checkin_worker(inst);
        }
        Ok(trace)
    }

    #[test]
    fn mode_b_executable_witness_from_observed_q() {
        if !ensure_mujoco_or_skip() {
            write_scratch(
                "mode-b-witness.log",
                "ensure_mujoco_or_skip() == false; MuJoCo worker not started\n",
            );
            return;
        }
        let bundle = RobotBundle::load(corpus::robot_dir("arm_gripper")).expect("bundle");
        let (mut probe, man) =
            load_and_normalize(&bundle, &template_objects(true), 0).expect("load");
        let t0 = truth_of(&mut probe).expect("truth");
        let discovered =
            crate::resource_discover::discover_resources(&bundle, &man, &probe.inspect);
        let mut qualified = Vec::new();
        for r in &discovered {
            if let Ok((_, q)) = crate::resource_qualify::qualify_resource(&bundle, r) {
                qualified.push(q);
            }
        }
        let mut model = embodiment_from_manifest(&bundle, &man);
        model.resources = qualified;
        let ee_name = bundle.manifest.end_effectors[0].name.clone();
        let ee_xyz = ee_from_truth(&t0, &bundle).expect("ee");
        let ee_q = ee_quat_from_truth(&t0, &ee_name);
        let tool_off = derived_tool_offset_ee(&model, &ee_name, ee_xyz, ee_q, &t0);
        let qpos = t0.qpos.clone();
        let mut cloud = build_ee_cloud(&model, &ee_name, &qpos, 21);
        let desired = [1.0, 0.0, 0.0];
        if let Some(s) = midreach_aligned_seed(&model, &ee_name, desired, ee_xyz[2], tool_off) {
            cloud.insert(0, s);
        }
        let half = [0.025, 0.025, 0.025];
        let sample = midreach_aligned_seed(&model, &ee_name, desired, ee_xyz[2], tool_off)
            .or_else(|| select_seed_aligned(&cloud, tool_off, desired, &model.joints).cloned())
            .expect("aligned seed");
        let place_push = planar_tool_axis(sample.quat_wxyz, tool_off).unwrap_or(desired);
        let center = object_center_for_sample(&sample, tool_off, place_push, half, 0.015)
            .expect("object_center_for_sample");
        let object = BoxObject {
            center,
            half_extents: half,
            quat_wxyz: [1.0, 0.0, 0.0, 0.0],
        };
        let support = SupportPlane {
            origin: [center[0], center[1], center[2] - half[2]],
            normal: [0.0, 0.0, 1.0],
        };
        let (proved, funnel) = prove_face_funnel(
            &model,
            &ee_name,
            std::slice::from_ref(&sample),
            object,
            support,
            place_push,
            0.03,
            sample.xyz,
            tool_off,
            false,
        );
        write_scratch(
            "mode-b-witness.log",
            &format!(
                "ee_xyz={ee_xyz:?} tool_off={tool_off:?} seed={:?} object={center:?} place_push={place_push:?} funnel={funnel:?} proved={}\n",
                sample.xyz,
                match &proved {
                    Ok(m) => format!("Ok(contact={:?})", m.contact_point),
                    Err(e) => format!("Err({e:?})"),
                }
            ),
        );
        checkin_worker(probe);
        let m = proved.expect("Mode B must produce an executable contact from observed q onto the observed object without moving the world");
        assert_eq!(
            m.object_center, center,
            "FixedWorld: object must stay where observation placed it"
        );
        assert!(
            (m.object_center[2] - sample.xyz[2]).abs() < 0.04,
            "object must sit on the live EE plane, object_z={} ee_z={}",
            m.object_center[2],
            sample.xyz[2]
        );
        let w = m.executable.as_ref().expect("executable witness");
        assert!(
            execution_block_reason(w).is_none(),
            "witness blocked: {:?}",
            execution_block_reason(w)
        );
    }

    fn assert_trace_honest(t: &ClosedLoopTrace, pass: i32) {
        assert_eq!(t.unauthorized_writes, 0, "pass {pass}");
        assert_eq!(t.authority_violations, 0, "pass {pass}");
        assert_eq!(t.evidence_status, SIMULATION_ONLY);
        for act in &t.actions {
            assert_eq!(
                act.get("privileged_force_in_predictor")
                    .and_then(|v| v.as_bool()),
                Some(false)
            );
        }
        let o = t.final_outcome.as_str();
        assert!(
            matches!(
                o,
                "GoalReached"
                    | "GoalCurrentlyUnachievable"
                    | "InsufficientPhysicalEvidence"
                    | "AuthorityRefusal"
                    | "InsufficientEvidence"
                    | "GoalPhysicallyInfeasible"
                    | "ModelNotApplicable"
            ),
            "pass {pass}: must halt with GOAL_REACHED or an honest refuse, got {o}"
        );
    }

    fn n_bounded_executes(t: &ClosedLoopTrace) -> usize {
        t.actions
            .iter()
            .filter(|a| a.get("ctrl_writes").and_then(|v| v.as_u64()).unwrap_or(0) > 0)
            .count()
    }

    fn has_contact_switch(t: &ClosedLoopTrace) -> bool {
        t.actions
            .iter()
            .any(|a| a.get("contact_switch").is_some_and(|v| v.is_object()))
    }

    fn assert_one_push_insufficient(t: &ClosedLoopTrace, pass: i32) {
        if t.final_outcome != "GoalReached" {
            return;
        }
        assert!(
            n_bounded_executes(t) >= 2 || has_contact_switch(t),
            "pass {pass}: GOAL_REACHED must need ≥2 bounded executes or a contact switch, faces={:?} n_exec={}",
            t.selected_faces,
            n_bounded_executes(t)
        );
    }

    fn assert_no_fling(t: &ClosedLoopTrace, pass: i32) {
        let mut prev: Option<f64> = None;
        for (i, a) in t.actions.iter().enumerate() {
            let writes = a.get("ctrl_writes").and_then(|v| v.as_u64()).unwrap_or(0);
            let after = a
                .get("goal_error_after")
                .and_then(|e| e.get("translation_residual_m"))
                .and_then(|v| v.as_f64());
            if writes > 0 {
                if let (Some(p), Some(n)) = (prev, after) {
                    assert!(
                        n < p + 0.08,
                        "pass {pass} action {i}: execute must not fling, residual {p} → {n}"
                    );
                }
            }
            if let Some(n) = after {
                prev = Some(n);
            }
        }
    }

    #[test]
    fn generated_contacts_use_observed_yaw() {
        let yaw = std::f64::consts::FRAC_PI_2;
        let cands = generate_planar_push_candidates(
            "obj0",
            pose_xy_yaw([0.0, 0.0], 0.03, yaw),
            [0.04, 0.03, 0.03],
            [0.0, 0.0, 1.0],
            0.01,
            0.02,
        )
        .expect("valid fixture support geometry");
        let plus_x = cands
            .iter()
            .find(|c| c.face_id == "+x" && c.contact_offset_u.abs() < 1e-9)
            .expect("+x face");
        assert!(
            plus_x.push_direction_world[1].abs() > 0.9,
            "object-local +x at yaw=π/2 must push in world ±Y, got {:?}",
            plus_x.push_direction_world
        );
        let ident = generate_planar_push_candidates(
            "obj0",
            pose([0.0, 0.0], 0.03),
            [0.04, 0.03, 0.03],
            [0.0, 0.0, 1.0],
            0.01,
            0.02,
        )
        .expect("valid fixture support geometry");
        let ident_x = ident
            .iter()
            .find(|c| c.face_id == "+x" && c.contact_offset_u.abs() < 1e-9)
            .expect("identity +x");
        assert!(
            ident_x.push_direction_world[0].abs() > 0.9,
            "identity +x must push in world ±X, got {:?}",
            ident_x.push_direction_world
        );
    }

    fn goal_scratch() -> PathBuf {
        std::env::var_os("REALITYOS_GOAL_SCRATCH")
            .map(PathBuf::from)
            .unwrap_or_else(|| std::env::temp_dir().join("realityos-goal-cfa27dcadf88/implementer"))
    }

    fn reasoning_of(trace: &ClosedLoopTrace) -> Value {
        trace
            .actions
            .iter()
            .find_map(|action| {
                action.get("reasoning").and_then(|note| {
                    if note.is_null() {
                        None
                    } else {
                        Some(note.clone())
                    }
                })
            })
            .unwrap_or(Value::Null)
    }

    #[test]
    fn development_slip_guard_repeats_without_inventing_tracking_evidence() {
        let scratch = goal_scratch();
        std::fs::create_dir_all(&scratch).expect("scratch");
        if !ensure_mujoco_or_skip() {
            std::fs::write(
                scratch.join("mujoco-launch.txt"),
                "ensure_mujoco_or_skip() == false; MuJoCo worker not started\n",
            )
            .expect("mujoco-launch");
            return;
        }
        let goal = trans_goal([0.18, 0.0], 0.025, 2);
        let unguarded = closed_loop_exec(
            "arm_gripper",
            [0.0, 0.0],
            &goal,
            None,
            21,
            ExecOptions {
                guard: false,
                world_excess_m: 0.03,
            },
        );
        let unguarded = match unguarded {
            Ok(trace) => trace,
            Err(err) => {
                std::fs::write(
                    scratch.join("mujoco-launch.txt"),
                    format!("MuJoCo worker started but the unguarded stroke failed: {err}\n"),
                )
                .ok();
                panic!("unguarded development stroke failed: {err}");
            }
        };
        std::fs::write(
            scratch.join("slip-unguarded.json"),
            serde_json::to_string_pretty(&unguarded).unwrap(),
        )
        .unwrap();
        let mut passes = Vec::new();
        for pass in 1..=2 {
            let trace = closed_loop_exec(
                "arm_gripper",
                [0.0, 0.0],
                &goal,
                None,
                21,
                ExecOptions {
                    guard: true,
                    world_excess_m: 0.03,
                },
            )
            .unwrap_or_else(|err| panic!("guarded pass {pass} failed: {err}"));
            std::fs::write(
                scratch.join(format!("slip-guard-pass{pass}.json")),
                serde_json::to_string_pretty(&trace).unwrap(),
            )
            .unwrap();
            passes.push(trace);
        }
        let note_a = reasoning_of(&passes[0]);
        let note_b = reasoning_of(&passes[1]);
        let note_u = reasoning_of(&unguarded);
        assert!(
            note_a["envelope_verdict"]
                .as_str()
                .unwrap_or("")
                .starts_with("ABORT_AND_REOBSERVE"),
            "guard did not abort: {note_a}"
        );
        assert_eq!(note_a["envelope_verdict"], note_b["envelope_verdict"]);
        assert_eq!(note_a["ranking_before"], note_b["ranking_before"]);
        assert!(
            note_a["ranking_before"]
                .as_str()
                .unwrap_or("")
                .contains("probe"),
            "selected probe missing: {note_a}"
        );
        assert_eq!(note_a["belief_after"], note_b["belief_after"]);
        assert_eq!(passes[0].unauthorized_writes, 0);
        assert_eq!(passes[1].unauthorized_writes, 0);
        assert_eq!(passes[0].evidence_status, SIMULATION_ONLY);
        assert_eq!(passes[1].evidence_status, SIMULATION_ONLY);
        let guarded_disp = note_a["displacement_m"].as_f64().expect("disp");
        let unguarded_disp = note_u["displacement_m"].as_f64().expect("unguarded disp");
        assert!(
            (guarded_disp - unguarded_disp).abs() <= 1e-9,
            "this harness observes only after the selected witness completes; it must not claim an early stop: guarded={guarded_disp}, unguarded={unguarded_disp}"
        );
        let hypotheses = note_a["hypotheses"].as_array().expect("hypotheses");
        assert_eq!(
            hypotheses,
            &vec![
                json!("StaleOrInsufficientObservation"),
                json!("ToolContactFrictionInconsistent")
            ],
            "missing tracking/geometry stay insufficient while measured contact loss remains usable"
        );
        assert_eq!(note_a["hypothesis_status"], "Underdetermined");
        assert_eq!(note_a["information_gain"], 1);
        assert_eq!(note_a["taxonomy"], "SAFE_PROBE_AVAILABLE");
        let action = passes[0]
            .actions
            .iter()
            .find(|action| {
                action
                    .get("reasoning")
                    .and_then(|note| note.get("envelope_verdict"))
                    .and_then(|v| v.as_str())
                    .unwrap_or("")
                    .starts_with("ABORT_AND_REOBSERVE")
            })
            .expect("abort action");
        assert!(action.get("goal_error_before").is_some());
        assert!(action.get("goal_error_after").is_some());
        assert_eq!(action["authority_decision"], "AUTHORIZE");
        assert_eq!(action["unauthorized_writes"], 0);
        assert_eq!(action["privileged_force_in_predictor"], false);
        assert_eq!(action["evidence_status"], SIMULATION_ONLY);
        assert_ne!(action["decision"], "RECOVER");
        assert_ne!(
            action["outcome"], "GOAL_REACHED",
            "an envelope abort must not be reported as goal reached"
        );
        let count = note_a["admissible_contact_count"]
            .as_u64()
            .expect("prove_face admissible count");
        let interactable = note_a["interactable_after_abort"]
            .as_bool()
            .unwrap_or(false);
        assert_eq!(interactable, count > 0, "{note_a}");
        let prevention_impossible = note_a["prevention_impossible"].as_bool().unwrap_or(false);
        assert!(
            interactable || prevention_impossible,
            "post-abort state must stay interactable or record that prevention was impossible: {note_a}"
        );
        let probe_contact = note_a["probe_contact_persisted"].as_bool();
        let probe_disp = note_a["probe_displacement_m"].as_f64().unwrap_or(0.0);
        if probe_contact == Some(false) || probe_disp < 1e-3 {
            let belief = note_a["belief_after"].as_str().unwrap_or("");
            assert!(
                belief.contains("Underdetermined") || belief.contains("Unknown"),
                "a missed probe must not identify a cause: {belief}"
            );
        }
        let probe_belief = note_a["belief_after"].as_str().unwrap_or("");
        assert!(
            probe_belief.starts_with("Underdetermined|") || probe_belief.starts_with("Unknown|"),
            "this probe's observation does not distinguish the live causes, so it cannot identify one: {probe_belief}"
        );
        assert_ne!(passes[0].final_outcome, "GoalReached");
        assert_ne!(passes[1].final_outcome, "GoalReached");
        assert!(passes[0]
            .actions
            .iter()
            .all(|action| action["outcome"] != "GOAL_REACHED"));
        assert!(note_a["ranking_after"]
            .as_str()
            .unwrap_or("")
            .contains("later="));
        assert_ne!(
            note_a["ranking_before"].as_str().unwrap_or(""),
            note_a["belief_after"].as_str().unwrap_or("")
        );
        println!(
            "abort={} probe={} belief={} disp={} unguarded_disp={} interactable={} prevention_impossible={}",
            note_a["envelope_verdict"],
            note_a["ranking_before"],
            note_a["belief_after"],
            guarded_disp,
            unguarded_disp,
            interactable,
            prevention_impossible
        );
    }

    #[test]
    fn closed_loop_mujoco_goal_directed() {
        if !ensure_mujoco_or_skip() {
            write_scratch(
                "mujoco-unavailable.log",
                "ensure_mujoco_or_skip() == false; MuJoCo worker not started\n",
            );
            return;
        }
        let mut traces = Vec::new();
        let mut last_err = None;
        for pass in 1..=2 {
            let g_plus = trans_goal([0.07, 0.0], 0.025, 6);
            let g_minus = trans_goal([-0.07, 0.0], 0.025, 6);
            let a = closed_loop_on_bundle("arm_gripper", [0.0, 0.0], &g_plus, None, 21);
            let b = closed_loop_on_bundle("arm_gripper", [0.0, 0.0], &g_minus, None, 22);
            match (a, b) {
                (Ok(ta), Ok(tb)) => {
                    assert_trace_honest(&ta, pass);
                    assert_trace_honest(&tb, pass);
                    assert_eq!(
                        ta.final_outcome, "GoalReached",
                        "pass {pass}: 7 cm +X goal must GOAL_REACHED after multiple bounded executes"
                    );
                    assert_one_push_insufficient(&ta, pass);
                    assert_one_push_insufficient(&tb, pass);
                    assert_no_fling(&ta, pass);
                    assert_no_fling(&tb, pass);
                    if !ta.selected_faces.is_empty() && !tb.selected_faces.is_empty() {
                        assert_ne!(
                            ta.selected_faces, tb.selected_faces,
                            "different initial states must discover different sequences: A={:?} B={:?}",
                            ta.selected_faces, tb.selected_faces
                        );
                    }
                    let mut row = json!({
                        "pass": pass,
                        "from_neg_x": ta,
                        "from_pos_x": tb,
                    });
                    if pass == 1 {
                        let mut g_yaw = g_plus.clone();
                        g_yaw.target_yaw = Some(0.35);
                        g_yaw.orientation_tolerance_rad = 0.15;
                        if let Ok(tc) = closed_loop_on_bundle(
                            "arm_gripper",
                            [0.0, 0.0],
                            &g_plus,
                            Some([0.0, 0.03]),
                            21,
                        ) {
                            assert_trace_honest(&tc, pass);
                            assert_no_fling(&tc, pass);
                            row["perturbed"] = json!(tc);
                        }
                        if let Ok(td) =
                            closed_loop_on_bundle("arm_gripper", [0.0, 0.0], &g_yaw, None, 24)
                        {
                            assert_trace_honest(&td, pass);
                            assert_one_push_insufficient(&td, pass);
                            row["yaw_and_translation"] = json!(td);
                        }
                        let mut emb = serde_json::Map::new();
                        for id in corpus::corpus_ids() {
                            if let Ok(t) = closed_loop_on_bundle(id, [0.0, 0.0], &g_plus, None, 25)
                            {
                                assert_trace_honest(&t, pass);
                                emb.insert(id.to_string(), json!(t.selected_faces));
                            }
                        }
                        row["embodiment_sequences"] = json!(emb);
                    }
                    traces.push(row);
                }
                (ea, eb) => {
                    last_err = Some(format!("{ea:?} {eb:?}"));
                }
            }
        }
        if traces.is_empty() {
            write_scratch(
                "closed-loop-mujoco.json",
                &format!("{{\"error\":{}}}", json!(last_err)),
            );
            if let Some(e) = last_err {
                panic!("closed-loop mujoco failed: {e}");
            }
        } else {
            write_scratch(
                "closed-loop-mujoco.json",
                &serde_json::to_string_pretty(&json!({
                    "metal": false,
                    "evidence_status": SIMULATION_ONLY,
                    "passes": traces,
                }))
                .unwrap(),
            );
            let any_reached = traces.iter().any(|row| {
                [
                    "from_neg_x",
                    "from_pos_x",
                    "perturbed",
                    "yaw_and_translation",
                ]
                .iter()
                .any(|k| {
                    row.get(*k).and_then(|v| v.get("final_outcome")) == Some(&json!("GoalReached"))
                })
            });
            assert!(
                any_reached,
                "closed loop must reach PlanarObjectGoal at least once; traces={}",
                serde_json::to_string(&traces).unwrap_or_default()
            );
        }
    }

    #[test]
    fn contradicted_belief_generates_short_candidates_first() {
        let mut belief = fresh_belief_from_physics(&PUSH_SCENARIO_PHYSICS);
        belief.contradict_declared(
            PhysicalParameter::QuasiStaticApplicability,
            "obs:quasi-static-contradiction",
        );
        let (belief_short, stroke, short_alternative) =
            candidate_stroke_plan(Some(&belief), &[], 0.03, 0.015);
        let candidates = generate_planar_push_candidates(
            "obj0",
            pose([0.0, 0.0], 0.03),
            [0.025; 3],
            [0.0, 0.0, 1.0],
            0.015,
            stroke,
        )
        .expect("valid fixture support geometry");

        assert!(belief_short);
        assert_eq!(stroke, 0.0075);
        assert_eq!(candidates[0].stroke_m, 0.0075);
        assert!(short_alternative.is_none());
    }

    #[test]
    fn recoverability_override_does_not_resurrect_a_blacklisted_action() {
        let goal = trans_goal([0.20, 0.0], 0.01, 6);
        let mut candidates = eval_at([0.0, 0.0], &goal, true);
        let SelectionOutcome::Selected {
            index: forbidden_index,
            ..
        } = select_interaction(&candidates, &[])
        else {
            panic!("expected a canonical candidate");
        };
        let forbidden_key = candidates[forbidden_index].action_key();
        for candidate in &mut candidates {
            if candidate.funnel.stage == FunnelStage::GoalUseful
                && candidate.goal_progress
                    == Some(realityos_semantics::planar_goal::GoalProgressClass::StrictProgress)
            {
                candidate.executable_for_plant = true;
            }
        }
        let admissible =
            admissible_override_indices(&candidates, std::slice::from_ref(&forbidden_key));
        assert!(
            !admissible.contains(&forbidden_index),
            "belief/recoverability overrides must use the canonical action-history exclusion"
        );
        assert!(
            !admissible.is_empty(),
            "another executable strict-progress candidate should remain available"
        );
        let choices: Vec<_> = candidates
            .iter()
            .enumerate()
            .map(|(index, candidate)| RecoverabilityChoice {
                id: candidate.id.clone(),
                class: if candidate.goal_progress
                    == Some(realityos_semantics::planar_goal::GoalProgressClass::StrictProgress)
                {
                    RecoverabilityClass::ProgressAndRecoverable
                } else {
                    RecoverabilityClass::NoProgress
                },
                progress: if index == forbidden_index { 1.0 } else { 0.5 },
            })
            .collect();

        let override_index = select_recoverable_override_index(
            &candidates,
            &choices,
            std::slice::from_ref(&forbidden_key),
        )
        .expect("recoverable choice");

        assert_ne!(
            candidates[override_index].action_key(),
            forbidden_key,
            "the verifier's recoverability override must not restore an action rejected by the canonical selector"
        );
    }

    #[test]
    fn belief_override_cannot_turn_authority_refusal_into_authorization() {
        let goal = trans_goal([0.20, 0.0], 0.01, 6);
        let mut candidates = eval_at([0.0, 0.0], &goal, false);
        for candidate in &mut candidates {
            if candidate.funnel.stage == FunnelStage::GoalUseful
                && candidate.goal_progress
                    == Some(realityos_semantics::planar_goal::GoalProgressClass::StrictProgress)
            {
                candidate.executable_for_plant = true;
            }
        }
        assert!(matches!(
            select_interaction(&candidates, &[]),
            SelectionOutcome::AuthorityRefusal { .. }
        ));
        let choices: Vec<_> = candidates
            .iter()
            .map(|candidate| RecoverabilityChoice {
                id: candidate.id.clone(),
                class: if candidate.goal_progress
                    == Some(realityos_semantics::planar_goal::GoalProgressClass::StrictProgress)
                {
                    RecoverabilityClass::ProgressAndRecoverable
                } else {
                    RecoverabilityClass::NoProgress
                },
                progress: 1.0,
            })
            .collect();

        assert_eq!(
            select_recoverable_override_index(&candidates, &choices, &[]),
            None,
            "belief and recoverability rankings may not turn an authority refusal into AUTHORIZE"
        );
    }

    #[test]
    fn selected_short_witness_is_not_promoted_to_long_execution_stroke() {
        use realityos_semantics::maneuver_witness::{
            ExecutableContactManeuver, ManeuverPhase, PhaseTransition, TransitionKind,
        };

        let selected_stroke = 0.0075;
        let pose_at = |x| pose_xy_yaw([x, 0.0], 0.03, 0.0);
        let phase = |x| ManeuverPhase::positional(pose_at(x), vec![0.0], vec![0.0], 0.0, 0.0);
        let transition = |kind| PhaseTransition::feasible(kind, 1, 0.5);
        let mut candidates = generate_planar_push_candidates(
            "obj0",
            pose([0.0, 0.0], 0.03),
            [0.025; 3],
            [0.0, 0.0, 1.0],
            0.015,
            0.03,
        )
        .expect("valid fixture support geometry");
        let mut selected = candidates.remove(0);
        assert!(selected_witness_stroke(&selected).is_none());
        let maneuver = ContactManeuver {
            contact_pose: pose_at(0.0),
            approach_pose: pose_at(-0.05),
            contact_point: [0.0, 0.0, 0.03],
            contact_normal: [-1.0, 0.0, 0.0],
            push_direction: [1.0, 0.0, 0.0],
            requested_stroke: selected_stroke,
            available_stroke: 0.1,
            support_clearance: 0.01,
            joint_margin: 0.5,
            orientation_error: 0.0,
            object_center: [0.0, 0.0, 0.03],
            support_top_z: 0.005,
            sampled_q: vec![0.0],
            executable: Some(ExecutableContactManeuver {
                joint_names: vec!["joint".into()],
                start_q: vec![0.0],
                approach: phase(-0.05),
                contact: phase(0.0),
                mid_stroke: phase(selected_stroke * 0.5),
                end_stroke: phase(selected_stroke),
                current_to_approach: transition(TransitionKind::CurrentToApproach),
                approach_to_contact: transition(TransitionKind::ApproachToContact),
                contact_to_mid: transition(TransitionKind::ContactToMidStroke),
                mid_to_end: transition(TransitionKind::MidToEndStroke),
            }),
        };
        let mut missing = selected.clone();
        let mut missing_witness = maneuver.clone();
        missing_witness.executable = None;
        assert!(!install_proven_maneuver(
            &mut missing,
            Some(missing_witness)
        ));
        assert!(missing.maneuver.is_none());

        let mut zero_length = selected.clone();
        let mut malformed_witness = maneuver.clone();
        let contact_xyz = malformed_witness
            .executable
            .as_ref()
            .unwrap()
            .contact
            .pose
            .xyz;
        malformed_witness
            .executable
            .as_mut()
            .unwrap()
            .end_stroke
            .pose
            .xyz = contact_xyz;
        assert!(!install_proven_maneuver(
            &mut zero_length,
            Some(malformed_witness)
        ));
        assert!(zero_length.maneuver.is_none());

        assert!(install_proven_maneuver(&mut selected, Some(maneuver)));
        let witness_stroke = selected_witness_stroke(&selected).unwrap();

        assert_eq!(selected.stroke_m, witness_stroke);
        assert_eq!(witness_stroke, selected_stroke);
    }

    #[test]
    fn selected_witness_is_not_split_across_replayed_scenarios() {
        assert_eq!(
            execution_scenario_strokes(0.03),
            vec![(0, 0.03)],
            "the runner executes every witness phase per scenario; splitting would replay the full witness"
        );
    }

    #[test]
    fn absent_tracking_and_geometry_residuals_cannot_identify_slip() {
        let observation =
            observed_discrepancy_without_residuals(3.0, 0.0, Some(0), true, 0.04, 0.015);

        let report = hypothesize(&observation);

        assert_eq!(report.status, Identifiability::Unknown);
        assert_eq!(
            report.kinds,
            vec![DiscrepancyKind::StaleOrInsufficientObservation]
        );
    }

    #[test]
    fn successful_contact_probe_does_not_identify_a_cause_absent_from_its_predictions() {
        let live = vec![
            DiscrepancyKind::StaleOrInsufficientObservation,
            DiscrepancyKind::ToolContactFrictionInconsistent,
        ];
        let update = apply_probe_observation(
            &PhysicalParameterBelief::declared_point(
                PhysicalParameter::SupportFriction,
                0.3,
                "scene.support_friction",
            ),
            &live,
            Stimulus {
                stroke_m: 0.008,
                quasi_static_stroke_limit_m: 0.015,
            },
            realityos_semantics::discrepancy::ObservationTag::HighDisplacementRatio,
            "probe-success",
        );

        assert_eq!(update.status, Identifiability::Unknown);
        assert!(update.remaining.is_empty());
        assert_eq!(update.eliminated, live);
    }

    #[test]
    fn missing_executable_probe_witness_is_not_taxonomized_as_a_safe_probe() {
        assert_eq!(
            reasoning_taxonomy(
                false,
                true,
                true,
                false,
                Identifiability::Underdetermined,
                Some(0),
            ),
            "SAFE_INTERACTION_AVAILABLE"
        );
        assert_eq!(
            reasoning_taxonomy(
                false,
                true,
                true,
                true,
                Identifiability::Underdetermined,
                Some(0),
            ),
            "SAFE_PROBE_AVAILABLE"
        );
    }

    #[test]
    fn predictor_inputs_exclude_privileged_fields() {
        let g = trans_goal([0.2, 0.0], 0.01, 4);
        let cands = eval_at([0.0, 0.0], &g, true);
        let sel = select_interaction(&cands, &[]).clone();
        let SelectionOutcome::Selected { index, .. } = sel else {
            panic!("{sel:?}");
        };
        let init = initiation_from_candidate(
            &mechanics_template(0.05, 0.2, 20.0),
            &cands[index],
            [0.0, 0.0, 0.03],
            0.0,
            true,
        );
        let s = serde_json::to_string(&init).unwrap();
        for needle in [
            "actuator_force",
            "normal_force",
            "tangential_force",
            "cfrc_ext",
            "PRIVILEGED",
        ] {
            assert!(!s.contains(needle), "predictor contains {needle}");
        }
    }
}
