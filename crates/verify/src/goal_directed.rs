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
        chain_q_from_qpos, local_ee_poses, run_skill_episode_ex_supervised, sample_push_units,
        template_objects, with_episode_qpos, with_injected_push_maneuver, WitnessPhaseDisturbance,
    };
    use crate::manipulation_scenarios::{ManipulationScenario, Polarity};
    use crate::manipulation_verify::body_xyz;
    use crate::mujoco_exec::{checkin_worker, ensure_mujoco_or_skip};
    use crate::observation::{PolicyObservation, VerifierTruth};
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
        apply_probe_observation, hypothesize, prediction_regime, DiscrepancyKind,
        DiscrepancyObservation, Identifiability, Stimulus,
    };
    use realityos_semantics::effect_feasibility::PlanarPushInitiation;
    use realityos_semantics::effort::{any_link_com_known, chain_physical_effort_signed};
    use realityos_semantics::embodiment::EmbodimentModel;
    use realityos_semantics::execution_envelope::{
        supervise_execution, ExecutionEnvelope, ExecutionProgress, FrozenAction,
        ObservationContract, ObservationField, RuntimePolicyObservation, SupervisorDecision,
        QUASI_STATIC_DISPLACEMENT_RATIO,
    };
    use realityos_semantics::geometry::PrimitiveShape;
    use realityos_semantics::goal_loop::{
        goal_status_if_no_admissible_interaction, receding_horizon_step, record_after_with_goal,
        DecisionWorkCounters, GoalLoopOutcome, LoopDecision, LoopState, ReasoningNote,
        WorldObservation,
    };
    use realityos_semantics::kinematics::{forward_kinematics, ik_residual_is_precise, solve_ik};
    use realityos_semantics::maneuver_witness::execution_block_reason;
    use realityos_semantics::pair_friction::PairFriction;
    use realityos_semantics::physical_belief::{
        BeliefEpistemicStatus, ParameterBelief, PhysicalParameter, PhysicalParameterBelief,
    };
    use realityos_semantics::physical_consequence::{
        assess_consequence, FrozenPrediction, ObservedConsequence,
    };
    use realityos_semantics::physical_decision::{
        decide_physical_action, CandidateEvidence, CandidateRejection, CandidateRole,
        DecisionContext, DecisionKind, LexicographicPreference, PredictedPhysicalEffect,
        ProbeEvidence, ProbeRecoverabilityAssessment,
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
        crate::test_support::attach_test_execution_proofs(&mut cands);
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
    fn hidden_fixture_friction_reports_insufficient_physical_evidence() {
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
            SelectionOutcome::InsufficientEvidence { .. }
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
            let (index, reason) = match sel {
                SelectionOutcome::Selected { index, reason }
                | SelectionOutcome::ContactTransition { index, reason, .. } => (index, reason),
                other => panic!("pass {pass}: expected selection, got {other:?}"),
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

    fn build_push_candidate_cloud(
        model: &realityos_semantics::embodiment::EmbodimentModel,
        ee: &str,
        qpos: &[f64],
        seed: u64,
        preferred_push: [f64; 3],
        object_z: f64,
        tool_off: [f64; 3],
    ) -> Vec<SampledEePose> {
        let mut cloud = build_ee_cloud(model, ee, qpos, seed);
        if let Some(aligned) = midreach_aligned_seed(model, ee, preferred_push, object_z, tool_off)
        {
            cloud.insert(0, aligned);
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
        // The development PUSH scenario must establish policy-visible contact;
        // the old 15 mm stand-off was executable geometry but never touched.
        spec.face_gap = 0.0;
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
            0.0,
            stroke,
        )
        .expect("valid fixture support geometry");
        let cloud = build_push_candidate_cloud(model, ee, qpos, seed, prefer_push, z, tool_off);
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
        // The executor supervises one frozen witness internally at its four meaningful phases.
        vec![(0, full_stroke)]
    }

    fn policy_object_xy_yaw(obs: &PolicyObservation, object_id: &str) -> Option<([f64; 2], f64)> {
        let detection = obs.detections.iter().find(|d| d.name == object_id)?;
        let xyz = detection.pose.get(..3)?;
        let quat = detection.orientation_wxyz.as_ref()?.get(..4)?;
        if xyz.iter().chain(quat.iter()).any(|v| !v.is_finite()) {
            return None;
        }
        Some((
            [xyz[0], xyz[1]],
            yaw_from_quat_wxyz([quat[0], quat[1], quat[2], quat[3]]),
        ))
    }

    fn physical_observation_contract(model: &EmbodimentModel) -> ObservationContract {
        ObservationContract {
            source: "perfect_perception_from_sim_truth".into(),
            model_epoch: model.model_hash.clone(),
            calibration_epoch: model.calibration_epoch.clone(),
            max_age_s: 0.5,
            required_units: [
                ("stroke_consumed_m".into(), "m".into()),
                ("object_displacement_m".into(), "m".into()),
                ("yaw_change_rad".into(), "rad".into()),
                ("goal_error".into(), "normalized".into()),
                ("robot_tracking_error_m".into(), "m".into()),
                ("reachability_margin_m".into(), "m".into()),
            ]
            .into_iter()
            .collect(),
            required_fields: vec![
                ObservationField::StrokeConsumed,
                ObservationField::Displacement,
                ObservationField::Yaw,
                ObservationField::Contact,
                ObservationField::GoalError,
                ObservationField::Tracking,
                ObservationField::Reachability,
                ObservationField::QuasiStaticApplicability,
            ],
        }
    }

    fn runtime_policy_observation(
        obs: &PolicyObservation,
        action_id: &str,
        witness_id: &str,
        quantum: u32,
        consumed_m: f64,
        origin_xy: [f64; 2],
        origin_yaw: f64,
        goal: &PlanarObjectGoal,
        candidate: &PhysicalInteractionCandidate,
        model: &EmbodimentModel,
        ee_name: &str,
        authority_ok: bool,
    ) -> RuntimePolicyObservation {
        let visible_pose = policy_object_xy_yaw(obs, &goal.object_id);
        let (displacement, yaw, goal_error) = visible_pose
            .map(|(xy, object_yaw)| {
                let displacement =
                    ((xy[0] - origin_xy[0]).powi(2) + (xy[1] - origin_xy[1]).powi(2)).sqrt();
                let yaw_change = realityos_semantics::planar_goal::wrap_pi(object_yaw - origin_yaw);
                let before = evaluate_goal_error(origin_xy, origin_yaw, goal);
                let current = evaluate_goal_error(xy, object_yaw, goal);
                (
                    Some(displacement),
                    Some(yaw_change),
                    Some((before.combined, current.combined)),
                )
            })
            .unwrap_or((None, None, None));
        // Use the same qualified end-effector resource that produced the
        // executable contact proof. Falling back to frame children here drops
        // declared finger bodies and can report a false contact loss.
        let intended_bodies =
            declared_manipulation_contact_bodies(model, model.resources.first(), ee_name);
        let contact = (obs.mode == crate::observation::VisionMode::PerfectPerception).then(|| {
            obs.contact_pairs.iter().any(|pair| {
                (pair.body1 == goal.object_id && intended_bodies.contains(&pair.body2))
                    || (pair.body2 == goal.object_id && intended_bodies.contains(&pair.body1))
            })
        });

        let tracking_error = candidate.maneuver.as_ref().and_then(|maneuver| {
            let executable = maneuver.executable.as_ref()?;
            let phase = match quantum {
                0 | 1 => &executable.approach,
                2 => &executable.contact,
                3 => &executable.mid_stroke,
                _ => &executable.end_stroke,
            };
            let actual: Option<Vec<f64>> = executable
                .joint_names
                .iter()
                .map(|name| {
                    let joint = model.joints.iter().find(|joint| &joint.name == name)?;
                    let q = *obs.qpos.get(joint.qpos_adr? as usize)?;
                    q.is_finite().then_some(q)
                })
                .collect();
            let actual = actual?;
            let fk = forward_kinematics(model, &executable.joint_names, ee_name, &actual).ok()?;
            let target = phase.pose.xyz;
            Some(
                ((fk.ee.xyz[0] - target[0]).powi(2)
                    + (fk.ee.xyz[1] - target[1]).powi(2)
                    + (fk.ee.xyz[2] - target[2]).powi(2))
                .sqrt(),
            )
        });
        let reachability = candidate
            .maneuver
            .as_ref()
            .zip(displacement)
            .map(|(maneuver, displacement)| maneuver.support_clearance - displacement);
        let quasi_static = displacement.map(|displacement| {
            if consumed_m <= 1e-9 {
                displacement <= 0.01
            } else {
                displacement / consumed_m <= QUASI_STATIC_DISPLACEMENT_RATIO
            }
        });
        let mut units = std::collections::BTreeMap::new();
        units.insert("stroke_consumed_m".into(), "m".into());
        units.insert("object_displacement_m".into(), "m".into());
        units.insert("yaw_change_rad".into(), "rad".into());
        units.insert("goal_error".into(), "normalized".into());
        units.insert("robot_tracking_error_m".into(), "m".into());
        units.insert("reachability_margin_m".into(), "m".into());
        RuntimePolicyObservation {
            action_id: action_id.into(),
            witness_id: witness_id.into(),
            observation_id: obs.observation_id.clone(),
            source: "perfect_perception_from_sim_truth".into(),
            timestamp_s: obs.timestamp_s,
            model_epoch: obs.model_hash.clone(),
            calibration_epoch: model.calibration_epoch.clone(),
            units,
            stroke_consumed_m: Some(consumed_m),
            object_displacement_m: displacement,
            yaw_change_rad: yaw,
            intended_contact_persists: contact,
            goal_error_before: goal_error.map(|errors| errors.0),
            goal_error_now: goal_error.map(|errors| errors.1),
            robot_tracking_error_m: tracking_error,
            reachability_margin_m: reachability,
            quasi_static_applicable: quasi_static,
            authority_ok: Some(authority_ok),
        }
    }

    fn observed_discrepancy_without_residuals(
        displacement_ratio: Option<f64>,
        yaw_change_rad: Option<f64>,
        predicted_yaw_sign: Option<i8>,
        contact_persisted: Option<bool>,
        tracking_error_m: Option<f64>,
        freshness_ok: bool,
        stroke_m: f64,
        quasi_static_stroke_limit_m: f64,
    ) -> DiscrepancyObservation {
        DiscrepancyObservation {
            displacement_ratio,
            yaw_change_rad,
            predicted_yaw_sign,
            observed_yaw_sign: yaw_change_rad.and_then(|yaw| {
                yaw.is_finite().then_some({
                    if yaw > 0.0 {
                        1
                    } else if yaw < 0.0 {
                        -1
                    } else {
                        0
                    }
                })
            }),
            contact_persisted,
            tracking_error_m,
            geometry_residual_m: None,
            freshness_ok,
            stroke_m,
            quasi_static_stroke_limit_m,
            contradictory: false,
            reachable: true,
        }
    }

    fn physical_probe_information(
        kinds: &[DiscrepancyKind],
        stroke_m: f64,
        quasi_static_limit_m: f64,
    ) -> ProbeEvidence {
        let tags: Vec<_> = kinds
            .iter()
            .map(|kind| {
                realityos_semantics::discrepancy::predicted_tag(
                    *kind,
                    Stimulus {
                        stroke_m,
                        quasi_static_stroke_limit_m: quasi_static_limit_m,
                    },
                )
            })
            .collect();
        let observable = |tag| {
            matches!(
                tag,
                realityos_semantics::discrepancy::ObservationTag::HighDisplacementRatio
                    | realityos_semantics::discrepancy::ObservationTag::NominalDisplacementRatio
                    | realityos_semantics::discrepancy::ObservationTag::YawSignFlip
                    | realityos_semantics::discrepancy::ObservationTag::ContactLost
            )
        };
        let mut decision_relevant_distinctions = 0_u32;
        let mut observable_distinctions = 0_u32;
        for i in 0..tags.len() {
            for j in (i + 1)..tags.len() {
                if tags[i] == tags[j]
                    || tags[i] == realityos_semantics::discrepancy::ObservationTag::Insufficient
                    || tags[j] == realityos_semantics::discrepancy::ObservationTag::Insufficient
                {
                    continue;
                }
                decision_relevant_distinctions = decision_relevant_distinctions.saturating_add(1);
                if observable(tags[i]) && observable(tags[j]) {
                    observable_distinctions = observable_distinctions.saturating_add(1);
                }
            }
        }
        ProbeEvidence {
            decision_relevant_distinctions,
            observable_distinctions,
            // The contact witness proves this candidate can be approached. It
            // does not bound the post-probe state enough to claim future access.
            future_interaction: ProbeRecoverabilityAssessment::Unknown,
        }
    }

    fn physical_probe_candidate_evidence(
        source_candidate_id: &str,
        maneuver: &ContactManeuver,
        stroke_m: f64,
        authority_ok: bool,
        probe: ProbeEvidence,
    ) -> CandidateEvidence {
        use sha2::Digest;

        let witness_contents = maneuver
            .executable
            .as_ref()
            .and_then(|witness| serde_json::to_string(witness).ok());
        let witness_digest = witness_contents
            .as_ref()
            .map(|contents| format!("{:x}", sha2::Sha256::digest(contents.as_bytes())));
        let contact_id = format!(
            "probe-contact:{:.5}:{:.5}:{:.5}",
            maneuver.contact_point[0], maneuver.contact_point[1], maneuver.contact_point[2]
        );
        let candidate_id = format!("{source_candidate_id}:{contact_id}:{stroke_m:.5}");
        let action_key = format!(
            "{candidate_id}:{:.5}:{:.5}:{:.5}",
            maneuver.push_direction[0], maneuver.push_direction[1], maneuver.push_direction[2]
        );
        let candidate_contents = serde_json::json!({
            "candidate_id": &candidate_id,
            "contact_id": &contact_id,
            "contact_normal": maneuver.contact_normal,
            "push_direction": maneuver.push_direction,
            "requested_stroke_m": stroke_m,
            "support_clearance_m": maneuver.support_clearance,
            "joint_margin": maneuver.joint_margin,
            "witness_contents": &witness_contents,
        })
        .to_string();
        let mut hard_rejections = Vec::new();
        if witness_contents.is_none() {
            hard_rejections.push(CandidateRejection::WitnessUnavailable);
        }
        CandidateEvidence {
            candidate_id: candidate_id.clone(),
            candidate_contents,
            action_key,
            contact_id,
            role: CandidateRole::PhysicalProbe,
            strict_goal_progress: false,
            authority_ok,
            executable_witness_id: witness_contents
                .as_ref()
                .map(|_| format!("{candidate_id}:witness")),
            witness_digest,
            witness_contents,
            robustness: BeliefRobustness::Ambiguous,
            recoverability: RecoverabilityClass::NoProgress,
            predicted_effect: PredictedPhysicalEffect {
                object_translation_world_m: None,
                yaw_change_rad: None,
                contact_persists: None,
                goal_error_derivative: None,
                goal_progress: None,
            },
            probe,
            preference: LexicographicPreference {
                error_derivative: None,
                angular_rate_abs: None,
                contact_offset_abs_m: None,
                stroke_m,
            },
            hard_rejections,
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
        } else if selected_probe && !executable_probe_available {
            "PROBE_NOT_EXECUTABLE"
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
        /// Development-only translation injected after witness phase three.
        /// Decision and supervision consume only the resulting policy observation.
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
        let face_gap = 0.0;
        let _ = ee;
        let mut qpos = t0.qpos.clone();
        let desired = desired_planar_push(goal);
        let cloud0 = build_push_candidate_cloud(
            &model, &ee_name, &qpos, seed, desired, ee_xyz0[2], tool_off,
        );
        let _ = start_xy;
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
        let belief_state: Option<PhysicalParameterBelief> = None;
        let live_hypotheses: Vec<DiscrepancyKind> = Vec::new();
        let quasi_limit_m = 0.015_f64;
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
                face_gap,
                stroke,
            )
            .expect("valid fixture support geometry");
            if let Some(short_stroke) = short_candidate_stroke {
                let mut short_cands = generate_planar_push_candidates(
                    "obj0",
                    object_pose,
                    [size, size, size],
                    [0.0, 0.0, 1.0],
                    face_gap,
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
            let mut decision_hypotheses = live_hypotheses.clone();
            let decision_regime = belief_state
                .as_ref()
                .map(|belief| prediction_regime(belief, &decision_hypotheses, 0.03, quasi_limit_m));
            if decision_regime.is_some_and(|regime| regime.friction_contradicted)
                && !decision_hypotheses.contains(&DiscrepancyKind::SupportFrictionInconsistent)
            {
                decision_hypotheses.push(DiscrepancyKind::SupportFrictionInconsistent);
            }
            for (index, candidate) in cands.iter_mut().enumerate() {
                let progress = candidate
                    .goal_progress
                    .unwrap_or(realityos_semantics::planar_goal::GoalProgressClass::Ambiguous);
                let assessment_candidate = goal_contact_at_contradicted_declared_friction(
                    candidate.id.clone(),
                    candidate.stroke_m,
                    -candidate.predicted_error_derivative.unwrap_or(0.0),
                    progress,
                    recoverability_choices[index].class,
                    candidate.executable_for_plant
                        && candidate
                            .maneuver
                            .as_ref()
                            .and_then(|maneuver| maneuver.executable.as_ref())
                            .is_some_and(|witness| witness.is_executable()),
                );
                let assessment = rank_goal_or_probe(
                    &[assessment_candidate],
                    &decision_hypotheses,
                    quasi_limit_m,
                );
                candidate.decision_robustness = assessment
                    .robustness
                    .first()
                    .map(|(_, robustness)| *robustness);
                candidate.decision_recoverability = Some(recoverability_choices[index].class);
            }
            let decision_started = std::time::Instant::now();
            let mut step = receding_horizon_step(
                &obs,
                &goal,
                &cands,
                state,
                last_obs.as_ref().map(|o| (o, None)),
            );
            let decision_wall_time_ns = decision_started.elapsed().as_nanos() as u64;
            let override_indices =
                admissible_override_indices(&cands, &step.state.forbidden_action_keys);
            let recoverable_index = step.selected.as_ref().and_then(|selected| {
                cands
                    .iter()
                    .position(|candidate| candidate.id == selected.id)
            });
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
                    let robust_goal = recoverable_index.is_some_and(|index| {
                        cands[index].decision_robustness
                            == Some(BeliefRobustness::RobustStrictProgress)
                    });
                    if robust_goal {
                        if let Some(index) = recoverable_index {
                            let stroke_regime = belief_state.as_ref().map(|belief| {
                                prediction_regime(
                                    belief,
                                    &live_hypotheses,
                                    cands[index].stroke_m,
                                    quasi_limit_m,
                                )
                            });
                            debug_assert_eq!(
                                step.selected
                                    .as_ref()
                                    .map(|candidate| candidate.id.as_str()),
                                Some(cands[index].id.as_str()),
                                "the canonical selector owns the selected identity"
                            );
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
                    } else {
                        let picked = recoverable_index.map(|index| cands[index].id.clone());
                        for (id, why) in &ranking.refused {
                            step.record.rejection_reasons.push(format!("{id}:{why}"));
                        }
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
                        debug_assert_eq!(
                            step.selected
                                .as_ref()
                                .map(|candidate| candidate.id.as_str()),
                            Some(cands[index].id.as_str()),
                            "the canonical selector owns the selected identity"
                        );
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
            let action_id = format!("goal-action:{k}:{}", sel.id);
            let witness_id = format!("{}:{}", sel.id, sel.face_id);
            let witness_contents = sel
                .maneuver
                .as_ref()
                .and_then(|maneuver| maneuver.executable.as_ref())
                .and_then(|witness| serde_json::to_string(witness).ok())
                .ok_or_else(|| "selected action lost executable witness contents".to_string())?;
            let frozen_prediction = FrozenPrediction {
                action_id: action_id.clone(),
                witness_id: witness_id.clone(),
                stroke_m: full_stroke,
                // The witness currently supports a direction/regime claim, not a
                // calibrated object-displacement magnitude.
                predicted_displacement_m: None,
                predicted_yaw_change_rad: None,
                predicted_contact_persists: true,
                quasi_static_stroke_limit_m: quasi_limit_m,
            };
            let recoverability = recoverability_choices
                .iter()
                .find(|choice| choice.id == sel.id)
                .map(|choice| choice.class)
                .unwrap_or(RecoverabilityClass::ProgressButRecoverabilityUnknown);
            let belief_snapshot = belief_state
                .clone()
                .unwrap_or_else(|| fresh_belief_from_physics(&PUSH_SCENARIO_PHYSICS));
            let frozen_action = FrozenAction {
                action_id: action_id.clone(),
                action_key: sel.action_key(),
                candidate_id: sel.id.clone(),
                contact_id: sel.face_id.clone(),
                witness_id: witness_id.clone(),
                witness_contents,
                requested_stroke_m: full_stroke,
                prediction: frozen_prediction.clone(),
                belief_snapshot,
                recoverability,
                envelope: ExecutionEnvelope::for_quasi_static_stroke(full_stroke, full_stroke),
                observation_contract: physical_observation_contract(&model),
                authority_granted: sel.authority_ok,
            };
            let mut nxy = xy;
            let mut nyaw = yaw;
            let mut ep_last = None;
            let mut consumed = 0.0;
            let mut abort_at: Option<u32> = None;
            let mut guarded_displacement = 0.0;
            let mut prevention_impossible = false;
            let mut failed_guard = None;
            let mut supervisor_decisions = Vec::new();
            let mut supervisor_events = Vec::new();
            let mut last_policy_observation: Option<PolicyObservation> = None;
            let mut last_runtime_observation: Option<RuntimePolicyObservation> = None;
            let mut executed_quanta = 0;
            let mut probe_evaluations = 0_u64;
            let mut belief_domain_evaluations = 0_u64;
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
                let direction_norm = (sel.push_direction_world[0].powi(2)
                    + sel.push_direction_world[1].powi(2))
                .sqrt()
                .max(1e-9);
                let phase_disturbance =
                    (options.world_excess_m > 0.0).then_some(WitnessPhaseDisturbance {
                        after_quantum: 2,
                        delta_xy_m: [
                            sel.push_direction_world[0] / direction_norm * options.world_excess_m,
                            sel.push_direction_world[1] / direction_norm * options.world_excess_m,
                        ],
                    });
                let mut after_phase = |quantum: u32, policy_observation: &PolicyObservation| {
                    let consumed_for_phase = match quantum {
                        0..=2 => 0.0,
                        3 => full_stroke * 0.5,
                        _ => full_stroke,
                    };
                    let runtime = runtime_policy_observation(
                        policy_observation,
                        &action_id,
                        &witness_id,
                        quantum,
                        consumed_for_phase,
                        action_xy,
                        action_yaw,
                        &goal,
                        &sel,
                        &model,
                        &ee_name,
                        true,
                    );
                    if let Some(displacement) = runtime.object_displacement_m {
                        guarded_displacement = displacement;
                    }
                    last_policy_observation = Some(policy_observation.clone());
                    last_runtime_observation = Some(runtime.clone());
                    executed_quanta = quantum;
                    consumed = consumed_for_phase;
                    let progress = ExecutionProgress {
                        action_id: action_id.clone(),
                        witness_id: witness_id.clone(),
                        completed_quanta: quantum,
                        total_quanta: 4,
                        stroke_consumed_m: Some(consumed_for_phase),
                        contact_guard_active: quantum >= 2,
                        now_s: policy_observation.timestamp_s,
                        remainder_invalidated: false,
                    };
                    let decision = supervise_execution(&frozen_action, &progress, &runtime);
                    supervisor_decisions.push(format!("{decision:?}"));
                    supervisor_events.push(decision.clone());
                    match decision {
                        SupervisorDecision::Continue { .. } => true,
                        SupervisorDecision::Completed { .. } => false,
                        SupervisorDecision::AbortAndReobserve {
                            failed_guard: guard,
                            ..
                        } => {
                            abort_at = Some(quantum.saturating_sub(1));
                            failed_guard = Some(guard);
                            false
                        }
                        SupervisorDecision::EvidenceUnavailable { reason, .. } => {
                            abort_at = Some(quantum.saturating_sub(1));
                            failed_guard = Some(format!("EVIDENCE_UNAVAILABLE:{reason}"));
                            false
                        }
                        SupervisorDecision::AuthorityLost { .. } => {
                            abort_at = Some(quantum.saturating_sub(1));
                            failed_guard = Some("AUTHORITY_LOST".into());
                            false
                        }
                    }
                };
                let phase_hook = options.guard.then_some(&mut after_phase as _);
                let run = with_episode_qpos(Some(&qpos_now), || {
                    with_injected_push_maneuver(maneuver, || {
                        run_skill_episode_ex_supervised(
                            &bundle,
                            &model,
                            &[],
                            &sc,
                            sha,
                            "PUSH",
                            loaded,
                            None,
                            phase_hook,
                            phase_disturbance,
                        )
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
                if !options.guard {
                    consumed = piece;
                    executed_quanta = 4;
                }
                let policy_observation = last_policy_observation.clone().unwrap_or_else(|| {
                    crate::observation::policy_observation(
                        &man,
                        &format!("goal-action-{k}"),
                        &format!("post-action-{step_i}"),
                        &crate::task::TaskSpec::Hold { duration_s: 0.1 },
                        crate::observation::VisionMode::PerfectPerception,
                        &truth,
                        0.0,
                        false,
                    )
                });
                let visible_pose = policy_object_xy_yaw(&policy_observation, &goal.object_id);
                let pose_fresh = visible_pose.is_some();
                let (oxy, oyaw) = visible_pose.unwrap_or((action_xy, action_yaw));
                nxy = oxy;
                nyaw = oyaw;
                qpos = policy_observation.qpos.clone();
                if last_runtime_observation.is_none() {
                    last_runtime_observation = Some(runtime_policy_observation(
                        &policy_observation,
                        &action_id,
                        &witness_id,
                        4,
                        consumed,
                        action_xy,
                        action_yaw,
                        &goal,
                        &sel,
                        &model,
                        &ee_name,
                        ep_i.unauthorized_writes == 0 && ep_i.task_result != "authority_violation",
                    ));
                }
                last_loaded = Some((inst, man));
                let disp =
                    ((nxy[0] - action_xy[0]).powi(2) + (nxy[1] - action_xy[1]).powi(2)).sqrt();
                if pose_fresh {
                    guarded_displacement = disp;
                }
                ep_last = Some(ep_i);
            }
            let ep = ep_last.ok_or_else(|| "stroke produced no episode".to_string())?;
            let intended_bodies =
                declared_manipulation_contact_bodies(&model, model.resources.first(), &ee_name);
            let contact_established = last_policy_observation.as_ref().is_some_and(|observation| {
                observation.contact_pairs.iter().any(|pair| {
                    (pair.body1 == goal.object_id && intended_bodies.contains(&pair.body2))
                        || (pair.body2 == goal.object_id && intended_bodies.contains(&pair.body1))
                })
            });
            let after = WorldObservation {
                object_id: "obj0".into(),
                xy: nxy,
                yaw: nyaw,
                robot_q: qpos.clone(),
                freshness_ok: last_policy_observation
                    .as_ref()
                    .and_then(|observation| policy_object_xy_yaw(observation, &goal.object_id))
                    .is_some(),
                intended_contact_face: if contact_established {
                    Some(sel.face_id.clone())
                } else {
                    None
                },
                authority_ok: ep.unauthorized_writes == 0,
                observed_at_s: (k + 1) as f64,
            };
            let probe_remaining_attempts = goal
                .max_bounded_attempts
                .saturating_sub(step.state.attempts);
            let probe_forbidden_action_keys = step.state.forbidden_action_keys.clone();
            let mut rec = record_after_with_goal(step, &after, &goal);
            let runtime_consequence = last_runtime_observation.clone();
            let consequence = assess_consequence(
                &frozen_prediction,
                &ObservedConsequence {
                    action_id: action_id.clone(),
                    witness_id: witness_id.clone(),
                    observation_id: runtime_consequence
                        .as_ref()
                        .map(|observation| observation.observation_id.clone())
                        .unwrap_or_else(|| "observation-unavailable".into()),
                    freshness_ok: after.freshness_ok,
                    execution_aborted: abort_at.is_some(),
                    stroke_consumed_m: Some(consumed),
                    displacement_m: runtime_consequence
                        .as_ref()
                        .and_then(|observation| observation.object_displacement_m),
                    yaw_change_rad: runtime_consequence
                        .as_ref()
                        .and_then(|observation| observation.yaw_change_rad),
                    contact_persisted: runtime_consequence
                        .as_ref()
                        .and_then(|observation| observation.intended_contact_persists),
                    tracking_error_m: runtime_consequence
                        .as_ref()
                        .and_then(|observation| observation.robot_tracking_error_m),
                    // Geometry residual has no independent policy sensor in this
                    // SIMULATION_ONLY slice, so it remains UNKNOWN.
                    geometry_residual_m: None,
                    quasi_static_applicable: runtime_consequence
                        .as_ref()
                        .and_then(|observation| observation.quasi_static_applicable),
                },
            );
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
            {
                let limit = full_stroke / 4.0;
                let decision_candidates = candidates_for_uncertainty(limit);
                probe_evaluations =
                    probe_evaluations.saturating_add(decision_candidates.len() as u64);
                let observed_displacement = runtime_consequence
                    .as_ref()
                    .and_then(|observation| observation.object_displacement_m);
                let ratio = (consumed > 1e-9)
                    .then(|| observed_displacement.map(|displacement| displacement / consumed))
                    .flatten();
                let observed_yaw_change = runtime_consequence
                    .as_ref()
                    .and_then(|observation| observation.yaw_change_rad);
                let contact_persisted = runtime_consequence
                    .as_ref()
                    .and_then(|observation| observation.intended_contact_persists);
                let slip_obs = observed_discrepancy_without_residuals(
                    ratio,
                    observed_yaw_change,
                    predicted_yaw_sign,
                    contact_persisted,
                    runtime_consequence
                        .as_ref()
                        .and_then(|observation| observation.robot_tracking_error_m),
                    after.freshness_ok,
                    consumed,
                    limit,
                );
                let report = hypothesize(&slip_obs);
                // The discrepancy detector can report an insufficient motion
                // ratio while the shared consequence assessment still has
                // independent physical alternatives from the frozen action
                // (for example, contact loss plus an uncalibrated yaw effect).
                // Keep those alternatives in probe analysis; never treat the
                // operational abort label itself as a physical hypothesis.
                let mut decision_hypotheses = report.kinds.clone();
                for kind in &consequence.hypotheses {
                    if !matches!(
                        kind,
                        DiscrepancyKind::StaleOrInsufficientObservation
                            | DiscrepancyKind::ExecutionTrackingDivergence
                    ) && !decision_hypotheses.contains(kind)
                    {
                        decision_hypotheses.push(*kind);
                    }
                }
                belief_domain_evaluations = belief_domain_evaluations.saturating_add(
                    (decision_candidates.len() as u64)
                        .saturating_mul(decision_hypotheses.len() as u64),
                );
                let ranking_before =
                    rank_goal_or_probe(&decision_candidates, &decision_hypotheses, limit);
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
                    format!(
                        "unchanged|status={:?}|support_friction={:?}|quasi_static={:?}",
                        report.status,
                        belief.declared_value(PhysicalParameter::SupportFriction),
                        belief
                            .entry(PhysicalParameter::QuasiStaticApplicability)
                            .map(|entry| (entry.status, entry.declared.value))
                    )
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
                let probe_displacement_m = None;
                let probe_contact_persisted = None;
                let mut admissible_contact_count = None;
                let executable_probe_available = false;
                let mut probe_decision = None;
                if options.guard && abort_at.is_some() {
                    let probe_requested = belief_state.is_none()
                        && ranking_before.selected_class == Some(DecisionClass::PhysicalProbe);
                    let probe_stroke = if probe_requested {
                        decision_candidates
                            .iter()
                            .find(|candidate| {
                                Some(&candidate.id) == ranking_before.selected_id.as_ref()
                            })
                            .map(|candidate| candidate.stroke_m)
                            .unwrap_or(limit * 0.4)
                            .max(1e-4)
                    } else {
                        full_stroke.max(0.02)
                    };
                    let (admissible, probe_maneuver) = proved_contacts_at(
                        &model,
                        &ee_name,
                        &qpos,
                        nxy,
                        z,
                        nyaw,
                        size,
                        probe_stroke,
                        tool_off,
                        seed.wrapping_add(if probe_requested { 81_000 } else { 80_000 }),
                        ee,
                        sel.push_direction_world,
                        probe_requested,
                    );
                    admissible_contact_count = Some(admissible);
                    let interactable_now = admissible > 0;
                    if abort_at == Some(0) && !interactable_now {
                        prevention_impossible = true;
                    }
                    if probe_requested && interactable_now {
                        let probe_information =
                            physical_probe_information(&decision_hypotheses, probe_stroke, limit);
                        let probe_candidate = probe_maneuver.as_ref().map(|maneuver| {
                            physical_probe_candidate_evidence(
                                ranking_before.selected_id.as_deref().unwrap_or("probe"),
                                maneuver,
                                probe_stroke,
                                // The preceding goal command's grant is not a scoped
                                // grant for a new physical probe.
                                false,
                                probe_information,
                            )
                        });
                        let candidates = probe_candidate.into_iter().collect();
                        let decision = decide_physical_action(&DecisionContext {
                            goal_id: format!("{}:{}", goal.world_id, goal.object_id),
                            goal_reached: rec.record.outcome == GoalLoopOutcome::GoalReached,
                            evidence_fresh: after.freshness_ok,
                            remaining_attempts: probe_remaining_attempts,
                            current_contact_id: Some(sel.face_id.clone()),
                            forbidden_action_keys: probe_forbidden_action_keys.clone(),
                            candidates,
                        });
                        ranking_after_id = format!("PHYSICAL_DECISION:{decision:?}");
                        selected_kind = Some("CanonicalPhysicalDecision".into());
                        belief_after_text = format!(
                            "NO_PROBE_OBSERVATION|{:?}|live={}",
                            report.status,
                            report
                                .kinds
                                .iter()
                                .map(|kind| format!("{kind:?}"))
                                .collect::<Vec<_>>()
                                .join(",")
                        );
                        // This branch has no scoped authority for the probe and no
                        // post-probe recoverability proof. A currently reachable
                        // approach is not evidence that a second unbounded action is
                        // safe, so never run, observe, learn from, or store it here.
                        probe_decision = Some(decision);
                    }
                    if abort_at.is_some() {
                        rec.record.decision = LoopDecision::Replan;
                        if rec.record.outcome == GoalLoopOutcome::GoalReached {
                            rec.record.outcome = GoalLoopOutcome::GoalProgress;
                        }
                        rec.record.first_divergence = failed_guard
                            .clone()
                            .or_else(|| Some("ABORT_AND_REOBSERVE".into()));
                    }
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
                    hypotheses: decision_hypotheses
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
                        match probe_decision.as_ref() {
                            Some(DecisionKind::Refuse { reason })
                                if reason.contains("AUTHORITY") =>
                            {
                                "PROBE_REFUSED_NO_SCOPED_AUTHORITY"
                            }
                            Some(DecisionKind::InsufficientEvidence { reason })
                                if reason == "PROBE_FUTURE_INTERACTION_UNPROVEN" =>
                            {
                                "PROBE_WITHHELD_RECOVERABILITY_UNKNOWN"
                            }
                            Some(DecisionKind::ContactTransition { .. }) => {
                                "PROBE_CONTACT_TRANSITION_REQUIRED"
                            }
                            Some(DecisionKind::PhysicalProbe { .. }) => {
                                "PROBE_SELECTED_REQUIRES_SUPERVISED_EXECUTOR"
                            }
                            Some(DecisionKind::CurrentlyUnachievable { reason })
                                if reason == "PHYSICAL_INTERACTION_BUDGET_EXHAUSTED" =>
                            {
                                "PROBE_BUDGET_EXHAUSTED"
                            }
                            _ => reasoning_taxonomy(
                                prevention_impossible,
                                interactable,
                                ranking_before.selected_class == Some(DecisionClass::PhysicalProbe),
                                executable_probe_available,
                                report.status,
                                abort_at,
                            ),
                        }
                        .into(),
                    ),
                    displacement_m: Some(guarded_displacement),
                    interactable_after_abort: Some(interactable),
                    probe_displacement_m,
                    probe_contact_persisted,
                    probe_decision,
                    admissible_contact_count,
                    consequence_assessment: Some(consequence.clone()),
                    supervisor_decisions: supervisor_decisions.clone(),
                    supervisor_events: supervisor_events.clone(),
                    executed_quanta,
                    ..ReasoningNote::default()
                });
            }
            let note = rec
                .record
                .reasoning
                .get_or_insert_with(ReasoningNote::default);
            note.consequence_assessment = Some(consequence);
            note.supervisor_decisions = supervisor_decisions;
            note.supervisor_events = supervisor_events;
            note.executed_quanta = executed_quanta;
            note.frozen_action_id = Some(action_id);
            note.frozen_witness_id = Some(witness_id);
            note.policy_observation_source = Some("perfect_perception_from_sim_truth".into());
            note.work_counters = Some(DecisionWorkCounters {
                candidates: cands.len() as u64,
                candidate_evaluations: cands.len() as u64,
                ik_attempts: None,
                fk_evaluations: None,
                collision_evaluations: None,
                jacobian_evaluations: None,
                mechanics_evaluations: cands.len() as u64,
                recoverability_evaluations: recoverability_choices.len() as u64,
                belief_domain_evaluations,
                probe_evaluations,
                decision_wall_time_ns,
                execution_quanta: executed_quanta,
                observations: if options.guard { executed_quanta } else { 1 },
                replans: u32::from(abort_at.is_some()),
                aborts: u32::from(abort_at.is_some()),
            });
            if abort_at.is_some() {
                let key = sel.action_key();
                rec.state.forbid_action_key(key);
            }
            if blocked && abort_at.is_none() {
                let key = sel.action_key();
                rec.state.forbid_action_key(key);
                rec.record.decision = LoopDecision::Recover;
            }
            record_action(
                &mut trace,
                &rec.record,
                Some(&sel),
                Some(&ep),
                Some(&frozen),
            );
            if ep.unauthorized_writes > 0 {
                checkin_worker(last_loaded.take().unwrap().0);
                return Ok(trace);
            }
            xy = nxy;
            yaw = nyaw;
            established_face = after.intended_contact_face.clone();
            last_obs = Some(after);
            state = rec.state;
            k += 1;
            if abort_at.is_some() {
                // No ordinary goal selector may run after the frozen witness was
                // aborted. A post-abort action requires its own canonical decision
                // and executor; until that path accepts and runs a probe, stop.
                break;
            }
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
    fn development_contact_loss_aborts_and_withholds_unproved_probe() {
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
        let goal = trans_goal([0.18, 0.0], 0.025, 3);
        let unguarded_goal = trans_goal([0.18, 0.0], 0.025, 1);
        let unguarded = closed_loop_exec(
            "arm_gripper",
            [0.0, 0.0],
            &unguarded_goal,
            None,
            21,
            ExecOptions {
                guard: false,
                world_excess_m: 0.005,
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
            scratch.join("contact-loss-unguarded.json"),
            serde_json::to_string_pretty(&unguarded).unwrap(),
        )
        .unwrap();
        let guarded = closed_loop_exec(
            "arm_gripper",
            [0.0, 0.0],
            &goal,
            None,
            21,
            ExecOptions {
                guard: true,
                world_excess_m: 0.005,
            },
        )
        .unwrap_or_else(|err| panic!("guarded pass failed: {err}"));
        assert!(
            guarded.actions.iter().skip(1).all(|action| {
                action["authority_decision"] == "REFUSE" && action["selected_id"].is_null()
            }),
            "after abort, a refused canonical probe must not fall through to another selector: {:?}",
            guarded.actions
        );
        std::fs::write(
            scratch.join("contact-loss-guarded.json"),
            serde_json::to_string_pretty(&guarded).unwrap(),
        )
        .unwrap();
        let note_a = reasoning_of(&guarded);
        let note_u = reasoning_of(&unguarded);
        assert!(
            note_a["envelope_verdict"]
                .as_str()
                .unwrap_or("")
                .starts_with("ABORT_AND_REOBSERVE"),
            "guard did not abort: {note_a}"
        );
        assert_eq!(note_a["frozen_action_id"], note_u["frozen_action_id"]);
        assert_eq!(note_a["frozen_witness_id"], note_u["frozen_witness_id"]);
        assert_eq!(
            note_a["policy_observation_source"],
            "perfect_perception_from_sim_truth"
        );
        assert!(note_a["executed_quanta"].as_u64().unwrap_or(4) < 4);
        let stop_quantum = note_a["executed_quanta"].as_u64().unwrap_or(4);
        assert!(
            stop_quantum == 2 || stop_quantum == 3,
            "stop must follow the contact or mid-stroke observation, before the final witness phase: {note_a}"
        );
        assert!(note_a["supervisor_events"]
            .as_array()
            .is_some_and(|decisions| decisions
                .iter()
                .any(|decision| { decision.get("AbortAndReobserve").is_some() })));
        assert!(
            note_a["ranking_before"]
                .as_str()
                .unwrap_or("")
                .contains("probe"),
            "the post-abort physical decision should record its probe candidate: {note_a}"
        );
        assert_eq!(guarded.unauthorized_writes, 0);
        assert_eq!(guarded.evidence_status, SIMULATION_ONLY);
        let guarded_disp = note_a["displacement_m"].as_f64().expect("disp");
        let unguarded_disp = note_u["displacement_m"].as_f64().expect("unguarded disp");
        assert!(
            guarded_disp > 0.01,
            "the guarded world must expose the injected disturbance before abort, observed only {guarded_disp} m"
        );
        assert!(
            guarded_disp + 1e-9 < unguarded_disp,
            "bounded supervision must stop the frozen witness before its remainder: guarded={guarded_disp}, unguarded={unguarded_disp}"
        );
        let hypotheses = note_a["hypotheses"].as_array().expect("hypotheses");
        assert!(hypotheses
            .iter()
            .any(|hypothesis| hypothesis == "ContactModeModelInconsistent"));
        assert!(hypotheses
            .iter()
            .any(|hypothesis| hypothesis == "ContactGeometryDisagreement"));
        assert_eq!(note_a["hypothesis_status"], "Underdetermined");
        assert!(note_a["information_gain"].as_u64().unwrap_or(0) > 0);
        let action = guarded
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
        assert!(
            count > 0,
            "the post-abort candidate generator should retain at least one proved physical interaction for the safe informative probe: {note_a}"
        );
        let interactable = note_a["interactable_after_abort"]
            .as_bool()
            .unwrap_or(false);
        assert_eq!(interactable, count > 0, "{note_a}");
        let prevention_impossible = note_a["prevention_impossible"].as_bool().unwrap_or(false);
        assert!(
            interactable || prevention_impossible,
            "post-abort state must stay interactable or record that prevention was impossible: {note_a}"
        );
        assert!(note_a["probe_contact_persisted"].is_null());
        assert!(note_a["probe_displacement_m"].is_null());
        if note_a["admissible_contact_count"].as_u64() == Some(0) {
            assert!(note_a["probe_decision"].is_null());
            assert_eq!(note_a["taxonomy"], "PROBE_NOT_EXECUTABLE");
        } else {
            assert!(note_a["probe_decision"].get("Refuse").is_some());
            assert_eq!(note_a["taxonomy"], "PROBE_REFUSED_NO_SCOPED_AUTHORITY");
        }
        assert!(note_a["belief_after"]
            .as_str()
            .unwrap_or("")
            .starts_with("NO_PROBE_OBSERVATION|"));
        assert_eq!(note_a["failed_guard"], "REACHABILITY_MARGIN");
        assert_eq!(
            note_a["consequence_assessment"]["status"],
            "EXECUTION_DIVERGED"
        );
        assert_eq!(
            note_a["consequence_assessment"]["observation"]["contact_persisted"],
            false
        );
        assert_ne!(guarded.final_outcome, "GoalReached");
        assert!(guarded
            .actions
            .iter()
            .all(|action| action["outcome"] != "GOAL_REACHED"));
        assert!(note_a["ranking_after"]
            .as_str()
            .unwrap_or("")
            .starts_with("PHYSICAL_DECISION:Refuse"));
        assert_ne!(
            note_a["ranking_before"].as_str().unwrap_or(""),
            note_a["belief_after"].as_str().unwrap_or("")
        );
        println!(
            "abort={} selected={} hypotheses={} disp={} unguarded_disp={} interactable={} prevention_impossible={}",
            note_a["envelope_verdict"],
            note_a["ranking_before"],
            note_a["hypotheses"],
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
        let forbidden_index = match select_interaction(&candidates, &[]) {
            SelectionOutcome::Selected { index, .. }
            | SelectionOutcome::ContactTransition { index, .. } => index,
            other => panic!("expected a canonical candidate, got {other:?}"),
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
        let observation = observed_discrepancy_without_residuals(
            Some(3.0),
            Some(0.0),
            Some(0),
            Some(true),
            None,
            true,
            0.04,
            0.015,
        );

        let report = hypothesize(&observation);

        assert_eq!(report.status, Identifiability::Unknown);
        assert_eq!(
            report.kinds,
            vec![DiscrepancyKind::StaleOrInsufficientObservation]
        );
    }

    #[test]
    fn contact_loss_and_unmodeled_yaw_leave_an_observable_probe_distinction() {
        let live = [
            DiscrepancyKind::StaleOrInsufficientObservation,
            DiscrepancyKind::ToolContactFrictionInconsistent,
            DiscrepancyKind::ContactModeModelInconsistent,
            DiscrepancyKind::ContactGeometryDisagreement,
        ];

        let evidence = physical_probe_information(&live, 0.008, 0.015);

        assert!(evidence.decision_relevant_distinctions > 0);
        assert!(evidence.observable_distinctions > 0);
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
            "PROBE_NOT_EXECUTABLE"
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
        let index = match sel {
            SelectionOutcome::Selected { index, .. }
            | SelectionOutcome::ContactTransition { index, .. } => index,
            other => panic!("{other:?}"),
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
