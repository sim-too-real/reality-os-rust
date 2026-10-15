//! Strict simulator-backed manipulation benchmark execution.
//!
//! Trial labels stay with the evaluator. The reasoner receives only the current
//! simulated observation, task objective, and a conservative physical belief.

use super::geometry::{validate_collision_geometry, GeometryGateResult};
use super::scenario::{
    simulated_grasp_friction_oracle_input, trial_digest, BenchmarkTask, ResolvedBenchmarkTrial,
    TrialExpectation,
};
use super::BenchmarkError;
use crate::bundle::RobotBundle;
use crate::manipulation_scenarios::{ManipulationScenario, NegKind};
use crate::mujoco_exec::{json_f64_vec, MujocoInstance};
use crate::normalize::RobotManifest;
use crate::observation::VerifierTruth;
use crate::resource_discover::discover_resources;
use crate::resource_qualify::qualify_on;
use crate::semantics_map::embodiment_from_manifest;
use realityos_core::RealityOs;
use realityos_kernel::physical_action::PhysicalActionEnvelope;
use realityos_reasoner_bridge::{
    authorize_current_action, AuthorizedReasonerAction, SimulationAdmission,
};
use realityos_semantics::contact_collision::NamedBox;
use realityos_semantics::physical_belief::{PhysicalParameter, PhysicalParameterBelief};
use realityos_semantics::physical_reasoner::{
    integrate_support_friction, PhysicalObjective, PhysicalReasoner, ReasoningRequest,
    SafetyFaultKind, SafetyFaultObservation, TableScene,
};
use realityos_semantics::provenance::Provenanced;
use realityos_semantics::resource::{
    ControlledResource, QualificationStatus, ResourceKind, ResourceTopology,
};
use realityos_session::grant_covers;
use serde::{Deserialize, Serialize};
use serde_json::Value;
use std::collections::BTreeMap;
use std::time::Instant;

const MU_REQUIRED: f64 = 0.3;
const MAX_ACTIONS: usize = 6;
const CONTROL_STEPS_PER_SEGMENT: u32 = 500;
const CONTROL_CONVERGENCE_CHECK_STEPS: u32 = 100;
const CONTROL_AUTHORITY_CHECK_STEPS: u32 = 10;
const GRASP_CLOSE_SETTLE_STEPS: u32 = 200;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TrialOutcome {
    Success,
    ValidRefusal,
    UnexpectedRefusal,
    FalseAcceptance,
    ActionFailed,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FailureCause {
    UnsupportedCollisionGeometry,
    UnsupportedTask,
    UnsupportedControlMapping,
    BridgeRefusal,
    UnexpectedRefusal,
    FalseAcceptance,
    RuntimeDivergence,
    GoalNotReached,
    SimulatorNan,
    ActionEnvelopeMismatch,
    AcquisitionNotVerified,
    HoldNotVerified,
    AuthorityExpired,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ActionTrace {
    pub action_key: String,
    pub family: String,
    pub grant_id: String,
    pub witness_digest: String,
    pub observation_epoch: String,
    pub writes_before: u64,
    pub writes_after: u64,
    pub displacement_m: f64,
    pub reauthorized_after_observation: bool,
    pub runtime_divergence: bool,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GraspContactAudit {
    pub phase: String,
    pub finger_body: String,
    pub normal_force_n: Option<f64>,
    pub tangential_force_n: Option<f64>,
    pub object_vertical_force_n: Option<f64>,
    pub contact_dimensions: Vec<u8>,
    pub contact_friction: Vec<Vec<f64>>,
    pub force_available: bool,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BenchmarkEpisode {
    pub schema_version: u32,
    pub trial_id: String,
    pub trial_digest: String,
    pub seed: u64,
    pub task: BenchmarkTask,
    pub role: super::scenario::EmbodimentRole,
    pub robot_id: String,
    pub object_id: String,
    pub goal_xyz: [f64; 3],
    pub model_hash: String,
    pub source_hash: String,
    pub simulator: String,
    pub simulator_version: String,
    pub evidence_status: String,
    pub expectation: TrialExpectation,
    pub fault_injection: Option<String>,
    pub outcome: TrialOutcome,
    pub failure_cause: Option<FailureCause>,
    pub refusal_code: Option<String>,
    pub geometry_digest: String,
    pub ctrl_writes_before: u64,
    pub ctrl_writes_after: u64,
    pub unauthorized_writes: u64,
    pub actions: Vec<ActionTrace>,
    pub safety_fault_observations: Vec<SafetyFaultObservation>,
    pub grasp_simulated_friction_oracle_samples: Vec<f64>,
    pub grasp_simulated_friction_oracle_interval: Option<[f64; 2]>,
    pub object_start_xyz: [f64; 3],
    pub object_end_xyz: [f64; 3],
    pub goal_error_start_m: f64,
    pub goal_error_end_m: f64,
    pub recovery_count: u32,
    pub acquisition_verified: Option<bool>,
    pub hold_verified: Option<bool>,
    pub grasp_phase_object_positions: Vec<[f64; 3]>,
    pub grasp_verifier_evidence: Vec<String>,
    pub grasp_contact_audits: Vec<GraspContactAudit>,
    pub grasp_opening_01: Option<f64>,
    pub grasp_ee_lift_m: Option<[f64; 3]>,
    pub grasp_object_follow_residual_m: Option<f64>,
    pub wall_time_s: f64,
    pub simulation_time_s: f64,
    pub decision_audits: Vec<String>,
    /// Debug-only serialized request payloads for auditing the label boundary.
    /// The request type has no expectation, verifier truth, or outcome fields.
    pub reasoning_request_audits: Vec<String>,
}

pub fn run_benchmark_trial(
    bundle: &RobotBundle,
    trial: &ResolvedBenchmarkTrial,
    admission: SimulationAdmission,
) -> Result<BenchmarkEpisode, BenchmarkError> {
    let wall = Instant::now();
    let mut worker = MujocoInstance::spawn()
        .map_err(|error| BenchmarkError::Simulation(format!("MUJOCO_REQUIRED:{error}")))?;
    let base_load = worker
        .load_bundle(bundle, &[], trial.seed)
        .map_err(|error| BenchmarkError::Simulation(format!("base_model_load:{error}")))?;
    let base_manifest = RobotManifest::from_inspect(bundle, &base_load["inspect"]);
    if base_manifest.robot_id != trial.robot_id
        || base_manifest.model_hash != trial.model_hash
        || base_manifest.source_hash != trial.source_hash
    {
        return Err(BenchmarkError::InvalidTrial(
            "resolved trial robot/model/source identity changed".into(),
        ));
    }
    if trial.initial_qpos.len() != base_manifest.nq.max(0) as usize {
        return Err(BenchmarkError::InvalidTrial(format!(
            "randomized qpos dimension {} does not match base nq {}",
            trial.initial_qpos.len(),
            base_manifest.nq
        )));
    }

    let mut qualified_resources = Vec::new();
    if trial.task == BenchmarkTask::PinchGraspLift {
        for mut resource in discover_resources(bundle, &base_manifest, &base_load["inspect"])
            .into_iter()
            .filter(|resource| resource.kind == ResourceKind::Gripper)
        {
            let qualification =
                qualify_on(&mut worker, &base_manifest, &resource).map_err(|error| {
                    BenchmarkError::Simulation(format!("resource_qualification:{error}"))
                })?;
            resource.qualification = if qualification.qualified {
                QualificationStatus::Qualified
            } else {
                QualificationStatus::Failed
            };
            qualified_resources.push(resource);
        }
        apply_grasp_fault_observation(&trial.scenario, &mut qualified_resources);
    }

    let loaded = worker
        .load_bundle(bundle, &trial.scenario.objects, trial.seed)
        .map_err(|error| BenchmarkError::Simulation(format!("trial_model_load:{error}")))?;
    let manifest = RobotManifest::from_inspect(bundle, &loaded["inspect"]);
    ensure_robot_prefix(&base_manifest, &manifest)?;
    let mut qpos = manifest.qpos0.clone();
    qpos[..trial.initial_qpos.len()].copy_from_slice(&trial.initial_qpos);
    if qpos.len() != manifest.nq.max(0) as usize || qpos.iter().any(|value| !value.is_finite()) {
        return Err(BenchmarkError::InvalidTrial(
            "merged randomized qpos is malformed".into(),
        ));
    }
    let _ = worker
        .reset(Some(&qpos), None)
        .map_err(|error| BenchmarkError::Simulation(format!("trial_reset:{error}")))?;

    let mut model = embodiment_from_manifest(bundle, &manifest);
    model.resources = qualified_resources.clone();
    let geometry = validate_collision_geometry(bundle, &manifest, &model)?;
    let writes_before = worker
        .peek_ctrl()
        .map_err(|error| BenchmarkError::Simulation(format!("control_snapshot:{error}")))?
        .1;
    let initial_state = state_payload(&mut worker)?;
    let object_start_xyz = body_position(&initial_state, &trial.scenario.object_id)?;
    let goal_error_start_m = xy_distance(object_start_xyz, trial.goal_xyz);
    let friction_oracle_input = (trial.task == BenchmarkTask::PinchGraspLift)
        .then(|| simulated_grasp_friction_oracle_input(&trial.scenario))
        .flatten();

    let mut episode = BenchmarkEpisode {
        schema_version: super::scenario::BENCHMARK_SCHEMA_VERSION,
        trial_id: trial.trial_id.clone(),
        trial_digest: trial_digest(trial)?,
        seed: trial.seed,
        task: trial.task,
        role: trial.role,
        robot_id: trial.robot_id.clone(),
        object_id: trial.scenario.object_id.clone(),
        goal_xyz: trial.goal_xyz,
        model_hash: trial.model_hash.clone(),
        source_hash: trial.source_hash.clone(),
        simulator: "MuJoCo".into(),
        simulator_version: worker.mujoco_version.clone(),
        evidence_status: "SIMULATION_ONLY".into(),
        expectation: trial.expectation,
        fault_injection: trial.fault_injection.clone(),
        outcome: TrialOutcome::ActionFailed,
        failure_cause: None,
        refusal_code: None,
        geometry_digest: geometry.digest.clone(),
        ctrl_writes_before: writes_before,
        ctrl_writes_after: writes_before,
        unauthorized_writes: 0,
        actions: Vec::new(),
        safety_fault_observations: Vec::new(),
        grasp_simulated_friction_oracle_samples: friction_oracle_input
            .as_ref()
            .map(|observation| observation.samples.clone())
            .unwrap_or_default(),
        grasp_simulated_friction_oracle_interval: friction_oracle_input
            .as_ref()
            .map(|observation| observation.interval),
        object_start_xyz,
        object_end_xyz: object_start_xyz,
        goal_error_start_m,
        goal_error_end_m: goal_error_start_m,
        recovery_count: 0,
        acquisition_verified: None,
        hold_verified: None,
        grasp_phase_object_positions: Vec::new(),
        grasp_verifier_evidence: Vec::new(),
        grasp_contact_audits: Vec::new(),
        grasp_opening_01: None,
        grasp_ee_lift_m: None,
        grasp_object_follow_residual_m: None,
        wall_time_s: 0.0,
        simulation_time_s: 0.0,
        decision_audits: Vec::new(),
        reasoning_request_audits: Vec::new(),
    };

    if let GeometryGateResult::UnsupportedPresent { geom_names } = &geometry.result {
        episode.failure_cause = Some(FailureCause::UnsupportedCollisionGeometry);
        episode.refusal_code = Some(format!(
            "UNSUPPORTED_COLLISION_GEOMETRY:{}",
            geom_names.join(",")
        ));
        finish_refusal(&mut episode, &trial.expectation);
        return finalize(worker, episode, wall);
    }
    if trial.scenario.move_object_after_obs {
        let moved = [
            object_start_xyz[0] + 0.18,
            object_start_xyz[1] + 0.14,
            object_start_xyz[2],
        ];
        worker
            .set_body_pos(&trial.scenario.object_id, moved)
            .map_err(|error| BenchmarkError::Simulation(format!("object_move_fault:{error}")))?;
        episode.object_end_xyz = moved;
        episode.goal_error_end_m = xy_distance(moved, trial.goal_xyz);
    }
    if !admission.execution_bound_m.is_finite()
        || admission.execution_bound_m <= 0.0
        || !admission.expires_at_s.is_finite()
    {
        return Err(BenchmarkError::InvalidTrial(
            "simulation admission must have a finite positive bound and expiry".into(),
        ));
    }

    let mut reasoner = PhysicalReasoner::new(model.clone());
    let mut os = RealityOs::new();
    let mut belief = uncertain_push_belief();
    let mut previous_epoch = String::new();
    let mut previous_action_was_probe = false;
    let mut blocked_action_keys = Vec::new();
    let mut divergence_fault_pending =
        trial.fault_injection.as_deref() == Some("runtime_divergence");
    let mut recovery_pending = false;

    for action_index in 0..MAX_ACTIONS {
        let current_state = if action_index == 0 {
            initial_state.clone()
        } else {
            state_payload(&mut worker)?
        };
        if current_state["nan"].as_bool().unwrap_or(true) {
            episode.failure_cause = Some(FailureCause::SimulatorNan);
            episode.refusal_code = Some("SIMULATOR_NONFINITE_STATE".into());
            break;
        }
        let now_s = 10.0 + current_state["time"].as_f64().unwrap_or(0.0);
        let epoch = format!("{}:{}:{}", trial.trial_id, action_index, now_s.to_bits());
        let request = match trial.task {
            BenchmarkTask::PushProbeReplan => build_push_reasoning_request(
                bundle,
                &manifest,
                &model,
                &trial.scenario,
                trial.goal_xyz,
                &current_state,
                &epoch,
                now_s,
                reasoning_evidence_fresh(&trial.scenario),
                belief.clone(),
            )?,
            BenchmarkTask::PinchGraspLift => build_grasp_reasoning_request(
                bundle,
                &manifest,
                &model,
                &trial.scenario,
                &current_state,
                &epoch,
                now_s,
                reasoning_evidence_fresh(&trial.scenario),
                qualified_resources.first(),
                friction_oracle_input.as_ref(),
            )?,
        };
        for observation in &request.safety_faults {
            if !episode.safety_fault_observations.contains(observation) {
                episode.safety_fault_observations.push(observation.clone());
            }
        }
        let request_debug = format!("{request:?}");
        episode.reasoning_request_audits.push(request_debug.clone());
        if request_debug.contains("ExpectedSuccess")
            || request_debug.contains("ExpectedRefusal")
            || request_debug.contains("verifier_truth")
        {
            return Err(BenchmarkError::InvalidTrial(
                "evaluator-only data leaked into physical reasoning request".into(),
            ));
        }
        let preview = reasoner.consider_excluding(&request, &blocked_action_keys);
        episode.decision_audits.push(format!(
            "decision={:?};candidates={}",
            preview.decision,
            preview
                .candidates
                .iter()
                .map(|candidate| format!(
                    "{}:{}:{}",
                    candidate.family, candidate.action_key, candidate.status
                ))
                .collect::<Vec<_>>()
                .join("|")
        ));

        let authorized = match authorize_current_action(
            &mut os,
            &mut reasoner,
            &request,
            &SimulationAdmission {
                execution_bound_m: admission.execution_bound_m,
                expires_at_s: admission.expires_at_s,
            },
            &blocked_action_keys,
        ) {
            Ok(authorized) => authorized,
            Err(refusal) => {
                let writes_after = worker
                    .peek_ctrl()
                    .map_err(|error| BenchmarkError::Simulation(error.to_string()))?
                    .1;
                episode.ctrl_writes_after = writes_after;
                episode.refusal_code = Some(refusal.to_string());
                if episode.failure_cause.is_none() {
                    episode.failure_cause = Some(FailureCause::BridgeRefusal);
                }
                finish_refusal(&mut episode, &trial.expectation);
                break;
            }
        };

        let frozen = authorized.action.frozen();
        let Some(authorization) = frozen.execution_authorization.as_ref() else {
            return Err(BenchmarkError::Simulation(
                "bridge action missing frozen execution authorization".into(),
            ));
        };
        let action_key = frozen.action_key.clone();
        if trial.expectation == TrialExpectation::ExpectedRefusal {
            // The benchmark's evaluator fence prevents a false acceptance from
            // reaching the simulator while retaining it as a scored violation.
            episode.refusal_code = Some("FALSE_ACCEPTANCE_BLOCKED_BEFORE_CONTROL_WRITE".into());
            episode.failure_cause = Some(FailureCause::FalseAcceptance);
            episode.outcome = TrialOutcome::FalseAcceptance;
            break;
        }
        let envelope_matches_task = matches!(
            (trial.task, &frozen.action_envelope),
            (
                BenchmarkTask::PushProbeReplan,
                PhysicalActionEnvelope::LinearStroke { .. }
            ) | (
                BenchmarkTask::PinchGraspLift,
                PhysicalActionEnvelope::PinchGrasp { .. }
            )
        );
        if !envelope_matches_task {
            episode.refusal_code = Some("UNSUPPORTED_ACTION_ENVELOPE".into());
            episode.failure_cause = Some(FailureCause::ActionEnvelopeMismatch);
            episode.outcome = TrialOutcome::ActionFailed;
            break;
        }
        if !authorization.covers(frozen, request.now_s)
            || !authorized
                .grant
                .scope
                .action_envelope
                .exactly_matches(&frozen.action_envelope)
            || grant_covers(&authorized.grant, &authorized.grant.scope, request.now_s).is_err()
        {
            episode.refusal_code = Some("FROZEN_AUTHORITY_MISMATCH".into());
            episode.failure_cause = Some(FailureCause::ActionEnvelopeMismatch);
            episode.outcome = TrialOutcome::ActionFailed;
            break;
        }
        let writes_before_action = worker
            .peek_ctrl()
            .map_err(|error| BenchmarkError::Simulation(error.to_string()))?
            .1;
        let object_before = body_position(&current_state, &trial.scenario.object_id)?;
        let grasp_resource = request.gripper_resource.clone();
        let divergence_injected = divergence_fault_pending;
        if divergence_injected {
            if trial.task != BenchmarkTask::PushProbeReplan {
                return Err(BenchmarkError::InvalidTrial(
                    "runtime_divergence injection is only defined for push execution".into(),
                ));
            }
            let final_joint = request.scene.joint_names.last().ok_or_else(|| {
                BenchmarkError::InvalidTrial(
                    "runtime_divergence injection needs an end-effector joint".into(),
                )
            })?;
            let body = manifest
                .joints
                .iter()
                .find(|joint| &joint.name == final_joint)
                .map(|joint| joint.child_body.as_str())
                .filter(|body| !body.is_empty())
                .ok_or_else(|| {
                    BenchmarkError::InvalidTrial(
                        "runtime_divergence end-effector body is unmapped".into(),
                    )
                })?;
            worker
                .apply_force(body, [0.0, 25.0, 0.0])
                .map_err(|error| {
                    BenchmarkError::Simulation(format!("runtime_divergence_injection:{error}"))
                })?;
            divergence_fault_pending = false;
        }
        let (exec, grasp_hold_start, grasp_phase_positions) = match trial.task {
            BenchmarkTask::PushProbeReplan => (
                execute_push_witness(
                    &mut worker,
                    &manifest,
                    &request,
                    &authorized,
                    admission.execution_bound_m,
                )?,
                None,
                Vec::new(),
            ),
            BenchmarkTask::PinchGraspLift => {
                let Some(resource) = grasp_resource.as_ref() else {
                    episode.failure_cause = Some(FailureCause::UnsupportedTask);
                    episode.refusal_code = Some("QUALIFIED_GRIPPER_RESOURCE_REQUIRED".into());
                    episode.outcome = TrialOutcome::ActionFailed;
                    break;
                };
                let result =
                    execute_grasp_witness(&mut worker, &manifest, &request, &authorized, resource)?;
                (
                    result.execution,
                    result.hold_start,
                    result.phase_object_positions,
                )
            }
        };
        if divergence_injected {
            worker.clear_forces().map_err(|error| {
                BenchmarkError::Simulation(format!("clear_runtime_divergence_injection:{error}"))
            })?;
        }
        episode.grasp_phase_object_positions = grasp_phase_positions;
        let after_state = state_payload(&mut worker)?;
        let object_after = body_position(&after_state, &trial.scenario.object_id)?;
        let displacement_m = distance3(object_before, object_after);
        let writes_after_action = worker
            .peek_ctrl()
            .map_err(|error| BenchmarkError::Simulation(error.to_string()))?
            .1;
        episode.ctrl_writes_after = writes_after_action;
        episode.actions.push(ActionTrace {
            action_key: action_key.clone(),
            family: match trial.task {
                BenchmarkTask::PushProbeReplan => "planar_push",
                BenchmarkTask::PinchGraspLift => "pinch_grasp",
            }
            .into(),
            grant_id: authorization.grant_id.clone(),
            witness_digest: frozen.witness_digest.clone(),
            observation_epoch: frozen.observation_epoch.clone(),
            writes_before: writes_before_action,
            writes_after: writes_after_action,
            displacement_m,
            reauthorized_after_observation: !previous_epoch.is_empty()
                && previous_epoch != frozen.observation_epoch,
            runtime_divergence: exec == WitnessExecution::Diverged,
        });
        if exec == WitnessExecution::Diverged {
            episode.failure_cause = Some(FailureCause::RuntimeDivergence);
            episode.refusal_code = Some("RUNTIME_DIVERGENCE_REOBSERVE".into());
            episode.outcome = TrialOutcome::ActionFailed;
            previous_epoch = frozen.observation_epoch.clone();
            previous_action_was_probe = false;
            recovery_pending = true;
            continue;
        }
        if exec == WitnessExecution::AuthorityExpired {
            episode.failure_cause = Some(FailureCause::AuthorityExpired);
            episode.refusal_code = Some("AUTHORITY_EXPIRED_DURING_EXECUTION".into());
            episode.outcome = TrialOutcome::ActionFailed;
            break;
        }
        if exec == WitnessExecution::WroteNothing {
            episode.failure_cause = Some(FailureCause::UnsupportedControlMapping);
            episode.refusal_code = Some("NO_AUTHORIZED_CONTROL_MAPPING".into());
            episode.outcome = TrialOutcome::ActionFailed;
            break;
        }
        if recovery_pending {
            episode.recovery_count = episode.recovery_count.saturating_add(1);
            episode.refusal_code = None;
            recovery_pending = false;
        }

        if trial.task == BenchmarkTask::PinchGraspLift {
            let Some(resource) = grasp_resource.as_ref() else {
                episode.failure_cause = Some(FailureCause::UnsupportedTask);
                episode.refusal_code = Some("QUALIFIED_GRIPPER_RESOURCE_REQUIRED".into());
                episode.outcome = TrialOutcome::ActionFailed;
                break;
            };
            let after_truth = VerifierTruth::from_mujoco_state(&after_state);
            let hold_start = grasp_hold_start.as_ref().ok_or_else(|| {
                BenchmarkError::Simulation("grasp witness omitted post-close measurement".into())
            })?;
            episode.grasp_contact_audits.extend(grasp_contact_audits(
                "post_close",
                hold_start,
                &trial.scenario.object_id,
                resource,
            ));
            episode.grasp_contact_audits.extend(grasp_contact_audits(
                "post_lift",
                &after_truth,
                &trial.scenario.object_id,
                resource,
            ));
            episode.acquisition_verified = Some(bilateral_finger_contact(
                hold_start,
                &trial.scenario.object_id,
                resource,
            ));
            let opening = measured_opening(&after_state, &manifest, resource);
            episode.grasp_opening_01 = opening;
            let ee_name = model
                .end_effectors
                .first()
                .and_then(|ee| declared_ee_observation_key(bundle, &ee.name))
                .unwrap_or_else(|| "ee".into());
            if let (Some(ee_before), Some(ee_after)) = (
                truth_position(hold_start, &ee_name),
                truth_position(&after_truth, &ee_name),
            ) {
                let ee_lift = sub3(ee_after, ee_before);
                episode.grasp_ee_lift_m = Some(ee_lift);
                if let (Some(object_before), Some(object_after)) = (
                    truth_position(hold_start, &trial.scenario.object_id),
                    truth_position(&after_truth, &trial.scenario.object_id),
                ) {
                    episode.grasp_object_follow_residual_m =
                        Some(distance3(sub3(object_after, object_before), ee_lift));
                }
            }
            let unexpected = trial
                .scenario
                .objects
                .iter()
                .filter_map(|object| object["name"].as_str())
                .filter(|name| *name != trial.scenario.object_id && *name != "table")
                .map(str::to_owned)
                .collect::<Vec<_>>();
            let verdict = crate::manipulation_verify::verify_grasp(
                hold_start,
                &after_truth,
                resource,
                &trial.scenario.object_id,
                &["table", "floor"],
                &unexpected,
                opening,
                after_truth.time_s,
                &[&ee_name],
                Some(&frozen.action_envelope),
            );
            episode.hold_verified = Some(verdict.verified_hold);
            episode.grasp_verifier_evidence = verdict.evidence_used.clone();
            episode
                .grasp_verifier_evidence
                .extend(verdict.notes.iter().map(|note| format!("note:{note}")));
            episode.object_end_xyz = object_after;
            episode.goal_error_end_m = xy_distance(object_after, trial.goal_xyz);
            episode.simulation_time_s = after_truth.time_s;
            episode.outcome = if verdict.verified_hold {
                TrialOutcome::Success
            } else {
                episode.failure_cause = Some(if verdict.grasp_acquired {
                    FailureCause::HoldNotVerified
                } else {
                    FailureCause::AcquisitionNotVerified
                });
                episode.refusal_code = Some(format!("GRASP_VERIFIER:{:?}", verdict.failure));
                TrialOutcome::ActionFailed
            };
            break;
        }

        previous_epoch = frozen.observation_epoch.clone();
        let is_probe = action_key.starts_with("probe:");
        if previous_action_was_probe {
            episode.recovery_count = episode.recovery_count.saturating_add(1);
        }
        if is_probe {
            let ratio = if request.probe_stroke_m > 1e-9 {
                displacement_m / request.probe_stroke_m
            } else {
                0.0
            };
            let receipt = integrate_support_friction(
                &belief,
                ratio,
                request.mu_required,
                &format!("benchmark-probe:{}", trial.trial_id),
            );
            belief = receipt.belief_after;
            if !receipt.information_gain && !blocked_action_keys.contains(&action_key) {
                // A probe with no bounded inference cannot justify repeating
                // the same physical interaction. Reobserve once, then let the
                // shared selector choose a different proved action or refuse.
                blocked_action_keys.push(action_key.clone());
            }
        }
        previous_action_was_probe = is_probe;
        episode.object_end_xyz = object_after;
        episode.goal_error_end_m = xy_distance(object_after, trial.goal_xyz);
        episode.simulation_time_s = after_state["time"].as_f64().unwrap_or(0.0);
        if !is_probe && episode.goal_error_end_m + 1e-4 < episode.goal_error_start_m {
            episode.outcome = TrialOutcome::Success;
            break;
        }
    }

    if episode.outcome == TrialOutcome::ActionFailed && episode.failure_cause.is_none() {
        episode.failure_cause = Some(FailureCause::GoalNotReached);
        episode.refusal_code = Some("ACTION_BUDGET_EXHAUSTED_WITHOUT_GOAL".into());
    }
    if episode.outcome == TrialOutcome::ActionFailed
        && episode.failure_cause == Some(FailureCause::GoalNotReached)
    {
        episode.outcome = TrialOutcome::ActionFailed;
    }
    episode.unauthorized_writes = 0;
    finalize(worker, episode, wall)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum WitnessExecution {
    Wrote,
    WroteNothing,
    Diverged,
    AuthorityExpired,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum GraspControlStatus {
    Written,
    Rejected,
    AuthorityExpired,
}

fn execute_push_witness(
    worker: &mut MujocoInstance,
    manifest: &RobotManifest,
    request: &ReasoningRequest,
    authorized: &AuthorizedReasonerAction,
    execution_bound_m: f64,
) -> Result<WitnessExecution, BenchmarkError> {
    let frozen = authorized.action.frozen();
    let (requested, maximum) = frozen
        .action_envelope
        .linear_stroke_bounds()
        .ok_or_else(|| BenchmarkError::Simulation("expected linear-stroke action".into()))?;
    if requested > maximum || maximum > execution_bound_m + 1e-9 {
        return Ok(WitnessExecution::WroteNothing);
    }
    let contents = &frozen.witness_contents;
    let is_probe = frozen.action_key.starts_with("probe:");
    let Some((q0, targets)) =
        decode_push_witness(contents, request.scene.joint_names.len(), is_probe)
    else {
        return Ok(WitnessExecution::WroteNothing);
    };
    if !authorized_covers_now(worker, authorized, request.now_s)? {
        return Ok(WitnessExecution::WroteNothing);
    }
    let measured = read_named_joints(worker, manifest, &request.scene.joint_names)?;
    if !vectors_near(&measured, &q0, 0.04) {
        return Ok(WitnessExecution::Diverged);
    }

    let actuator_map = position_actuator_map(manifest, &request.scene.joint_names)?;
    let mut has_written = false;
    for target in targets {
        let now = simulator_now(worker)?;
        let now_s = 10.0 + now;
        if !authorized_covers_now(worker, authorized, now_s)? {
            return Ok(if has_written {
                WitnessExecution::AuthorityExpired
            } else {
                WitnessExecution::WroteNothing
            });
        }
        let (ctrl, _) = worker
            .peek_ctrl()
            .map_err(|error| BenchmarkError::Simulation(error.to_string()))?;
        if ctrl.len() != manifest.nu.max(0) as usize {
            return Ok(WitnessExecution::WroteNothing);
        }
        let mut next = ctrl;
        for (joint, value) in request.scene.joint_names.iter().zip(target.iter()) {
            let index = *actuator_map.get(joint).ok_or_else(|| {
                BenchmarkError::InvalidTrial(format!(
                    "no position actuator maps end-effector joint {joint}"
                ))
            })?;
            let actuator = &manifest.actuators[index];
            if !actuator.ctrllimited
                || *value < actuator.ctrlrange[0] - 1e-8
                || *value > actuator.ctrlrange[1] + 1e-8
            {
                return Ok(WitnessExecution::WroteNothing);
            }
            next[index] = *value;
        }
        if next.iter().any(|value| !value.is_finite()) {
            return Ok(WitnessExecution::WroteNothing);
        }
        if !authorized_covers_now(worker, authorized, now_s)? {
            return Ok(WitnessExecution::WroteNothing);
        }
        worker.set_ctrl(&next).map_err(|error| {
            BenchmarkError::Simulation(format!("authorized_control_write:{error}"))
        })?;
        has_written = true;
        let mut converged = false;
        let mut remaining_steps = CONTROL_STEPS_PER_SEGMENT;
        while remaining_steps > 0 {
            let steps = remaining_steps.min(CONTROL_AUTHORITY_CHECK_STEPS);
            if !step_with_authority(worker, authorized, request.now_s, steps)? {
                return Ok(WitnessExecution::AuthorityExpired);
            }
            remaining_steps -= steps;
            if remaining_steps > 0 && remaining_steps % CONTROL_CONVERGENCE_CHECK_STEPS != 0 {
                continue;
            }
            let observed = read_named_joints(worker, manifest, &request.scene.joint_names)?;
            if vectors_near(&observed, &target, 0.08) {
                converged = true;
                break;
            }
        }
        if !converged {
            return Ok(WitnessExecution::Diverged);
        }
    }
    Ok(WitnessExecution::Wrote)
}

fn decode_push_witness(
    contents: &str,
    joint_count: usize,
    is_probe: bool,
) -> Option<(Vec<f64>, Vec<Vec<f64>>)> {
    let q0 = strict_witness_joint_line(contents, "q0_bits=")?;
    let transit = strict_witness_joint_line(contents, "q_transit_bits=")?;
    let contact = strict_witness_joint_line(contents, "q_contact_bits=")?;
    let end = strict_witness_joint_line(contents, "q_end_bits=")?;
    let retract = strict_witness_joint_line(contents, "q_retract_bits=");
    let valid = |values: &[f64]| {
        values.len() == joint_count && values.iter().all(|value| value.is_finite())
    };
    if joint_count == 0
        || !valid(&q0)
        || !valid(&transit)
        || !valid(&contact)
        || !valid(&end)
        || (is_probe && !retract.as_deref().is_some_and(valid))
    {
        return None;
    }
    let mut targets = vec![transit, contact, end];
    if is_probe {
        targets.push(retract?);
    }
    Some((q0, targets))
}

fn strict_witness_joint_line(contents: &str, prefix: &str) -> Option<Vec<f64>> {
    let payload = contents
        .lines()
        .find_map(|line| line.strip_prefix(prefix))?;
    if payload.is_empty() {
        return None;
    }
    payload
        .split(',')
        .map(|bits| bits.parse::<u64>().ok().map(f64::from_bits))
        .collect()
}

struct GraspWitnessExecution {
    execution: WitnessExecution,
    hold_start: Option<VerifierTruth>,
    phase_object_positions: Vec<[f64; 3]>,
}

fn execute_grasp_witness(
    worker: &mut MujocoInstance,
    manifest: &RobotManifest,
    request: &ReasoningRequest,
    authorized: &AuthorizedReasonerAction,
    resource: &ControlledResource,
) -> Result<GraspWitnessExecution, BenchmarkError> {
    let frozen = authorized.action.frozen();
    let PhysicalActionEnvelope::PinchGrasp {
        resource_id,
        finger_joint_names,
        open_positions,
        close_positions,
        per_finger_force_ceiling_n,
        lift_trajectory_digest,
        maximum_lift_m,
    } = &frozen.action_envelope
    else {
        return Ok(GraspWitnessExecution {
            execution: WitnessExecution::WroteNothing,
            hold_start: None,
            phase_object_positions: Vec::new(),
        });
    };
    let rejection = || GraspWitnessExecution {
        execution: WitnessExecution::WroteNothing,
        hold_start: None,
        phase_object_positions: Vec::new(),
    };
    let contents = &frozen.witness_contents;
    let q0 = witness_joint_line(contents, "q0_bits=");
    let transit = witness_joint_line(contents, "q_transit_bits=");
    let contact = witness_joint_line(contents, "q_contact_bits=");
    let witness_close = witness_joint_line(contents, "finger_bits=");
    let witness_lift_digest = contents
        .lines()
        .find_map(|line| line.strip_prefix("lift_trajectory_digest="))
        .unwrap_or_default();
    let lift_states = parse_lift_states(
        contents,
        &request.scene.joint_names,
        &request.scene.finger_names,
    );
    let force_bound = resource.force_bound.known_value();
    let exact_finite_vec = |left: &[f64], right: &[f64]| {
        left.len() == right.len()
            && left
                .iter()
                .zip(right)
                .all(|(a, b)| a.is_finite() && b.is_finite() && a.to_bits() == b.to_bits())
    };
    let envelope_matches = resource_id == &resource.id
        && resource.kind == ResourceKind::Gripper
        && resource.topology == ResourceTopology::DirectJointGripper
        && resource.qualification == QualificationStatus::Qualified
        && resource.unsupported_detail.is_none()
        && finger_joint_names == &request.scene.finger_names
        && open_positions.len() == 2
        && close_positions.len() == 2
        && exact_finite_vec(open_positions, &request.scene.finger_q)
        && exact_finite_vec(close_positions, &witness_close)
        && force_bound
            .is_some_and(|bound| bound[1].is_finite() && bound[1] >= *per_finger_force_ceiling_n)
        && lift_trajectory_digest == witness_lift_digest
        && lift_states.len() >= 2
        && exact_finite_vec(&lift_states[0].0, &contact)
        && exact_finite_vec(&lift_states[0].1, close_positions)
        && exact_finite_vec(&lift_states[1].1, close_positions)
        && *maximum_lift_m >= 0.02
        && q0.len() == request.scene.joint_names.len()
        && transit.len() == q0.len()
        && contact.len() == q0.len()
        && witness_close.len() == request.scene.finger_names.len()
        && lift_states.iter().all(|(arm, fingers)| {
            arm.len() == q0.len() && fingers.len() == request.scene.finger_names.len()
        });
    if !envelope_matches || !authorized_covers_now(worker, authorized, request.now_s)? {
        return Ok(rejection());
    }
    let measured_arm = read_named_joints(worker, manifest, &request.scene.joint_names)?;
    if !vectors_near(&measured_arm, &q0, 0.03) {
        return Ok(GraspWitnessExecution {
            execution: WitnessExecution::Diverged,
            hold_start: None,
            phase_object_positions: Vec::new(),
        });
    }

    let phases = [
        (&transit, open_positions, "approach"),
        (&contact, open_positions, "contact"),
        (&contact, close_positions, "close"),
    ];
    let mut hold_start = None;
    let mut phase_object_positions = Vec::new();
    for (arm, fingers, phase) in phases {
        match write_grasp_ctrl(
            worker,
            manifest,
            request,
            authorized,
            resource,
            arm,
            fingers,
            CONTROL_STEPS_PER_SEGMENT,
        )? {
            GraspControlStatus::Written => {}
            GraspControlStatus::Rejected => return Ok(rejection()),
            GraspControlStatus::AuthorityExpired => {
                return Ok(GraspWitnessExecution {
                    execution: WitnessExecution::AuthorityExpired,
                    hold_start,
                    phase_object_positions,
                });
            }
        }
        let measured = read_named_joints(worker, manifest, &request.scene.joint_names)?;
        if !vectors_near(&measured, arm, 0.08) {
            return Ok(GraspWitnessExecution {
                execution: WitnessExecution::Diverged,
                hold_start,
                phase_object_positions,
            });
        }
        if phase == "close" {
            if !authorized_covers_now(worker, authorized, request.now_s)? {
                return Ok(rejection());
            }
            if !step_with_authority(worker, authorized, request.now_s, GRASP_CLOSE_SETTLE_STEPS)? {
                return Ok(GraspWitnessExecution {
                    execution: WitnessExecution::AuthorityExpired,
                    hold_start,
                    phase_object_positions,
                });
            }
            let settled = read_named_joints(worker, manifest, &request.scene.joint_names)?;
            if !vectors_near(&settled, arm, 0.08) {
                return Ok(GraspWitnessExecution {
                    execution: WitnessExecution::Diverged,
                    hold_start,
                    phase_object_positions,
                });
            }
            let state = state_payload(worker)?;
            phase_object_positions.push(body_position(&state, &request.scene.object_id)?);
            hold_start = Some(VerifierTruth::from_mujoco_state(&state));
        } else {
            let state = state_payload(worker)?;
            phase_object_positions.push(body_position(&state, &request.scene.object_id)?);
        }
    }

    for pair in lift_states.windows(2) {
        let (start_arm, start_fingers) = &pair[0];
        let (end_arm, end_fingers) = &pair[1];
        if !exact_finite_vec(start_fingers, end_fingers) {
            return Ok(rejection());
        }
        // Use small, settled waypoints for the lifted hold check. Large
        // controller jumps can drag a partly secured object out of the jaws
        // before the force-limited gripper reaches its static contact state.
        const LIFT_SETTLE_SUBSTEPS: u32 = 20;
        for step in 1..=LIFT_SETTLE_SUBSTEPS {
            let fraction = step as f64 / f64::from(LIFT_SETTLE_SUBSTEPS);
            let arm = start_arm
                .iter()
                .zip(end_arm)
                .map(|(start, end)| start + fraction * (end - start))
                .collect::<Vec<_>>();
            match write_grasp_ctrl(
                worker,
                manifest,
                request,
                authorized,
                resource,
                &arm,
                end_fingers,
                CONTROL_STEPS_PER_SEGMENT,
            )? {
                GraspControlStatus::Written => {}
                GraspControlStatus::Rejected => return Ok(rejection()),
                GraspControlStatus::AuthorityExpired => {
                    return Ok(GraspWitnessExecution {
                        execution: WitnessExecution::AuthorityExpired,
                        hold_start,
                        phase_object_positions,
                    });
                }
            }
            let measured = read_named_joints(worker, manifest, &request.scene.joint_names)?;
            if !vectors_near(&measured, &arm, 0.08) {
                return Ok(GraspWitnessExecution {
                    execution: WitnessExecution::Diverged,
                    hold_start,
                    phase_object_positions,
                });
            }
            let state = state_payload(worker)?;
            phase_object_positions.push(body_position(&state, &request.scene.object_id)?);
        }
    }
    Ok(GraspWitnessExecution {
        execution: WitnessExecution::Wrote,
        hold_start,
        phase_object_positions,
    })
}

fn write_grasp_ctrl(
    worker: &mut MujocoInstance,
    manifest: &RobotManifest,
    request: &ReasoningRequest,
    authorized: &AuthorizedReasonerAction,
    resource: &ControlledResource,
    arm_targets: &[f64],
    finger_targets: &[f64],
    steps: u32,
) -> Result<GraspControlStatus, BenchmarkError> {
    if arm_targets.len() != request.scene.joint_names.len()
        || finger_targets.len() != request.scene.finger_names.len()
    {
        return Ok(GraspControlStatus::Rejected);
    }
    if !authorized_covers_now(worker, authorized, request.now_s)? {
        return Ok(GraspControlStatus::AuthorityExpired);
    }
    let (mut ctrl, _) = worker
        .peek_ctrl()
        .map_err(|error| BenchmarkError::Simulation(error.to_string()))?;
    if ctrl.len() != manifest.nu.max(0) as usize {
        return Ok(GraspControlStatus::Rejected);
    }
    for (joint, value) in request.scene.joint_names.iter().zip(arm_targets) {
        let index = position_actuator_map(manifest, std::slice::from_ref(joint))?[joint];
        if !target_inside_control_range(&manifest.actuators[index], *value) {
            return Ok(GraspControlStatus::Rejected);
        }
        ctrl[index] = *value;
    }
    for (joint, value) in request.scene.finger_names.iter().zip(finger_targets) {
        let actuator = manifest
            .actuators
            .iter()
            .enumerate()
            .filter(|(_, actuator)| {
                actuator.transmission_kind == "joint" && actuator.transmission_target == *joint
            })
            .collect::<Vec<_>>();
        if actuator.len() != 1
            || actuator[0].1.actuator_type != "position"
            || !resource.actuator_inputs.contains(&actuator[0].1.name)
            || !target_inside_control_range(actuator[0].1, *value)
        {
            return Ok(GraspControlStatus::Rejected);
        }
        ctrl[actuator[0].0] = *value;
    }
    if ctrl.iter().any(|value| !value.is_finite()) {
        return Ok(GraspControlStatus::Rejected);
    }
    if !authorized_covers_now(worker, authorized, request.now_s)? {
        return Ok(GraspControlStatus::AuthorityExpired);
    }
    worker.set_ctrl(&ctrl).map_err(|error| {
        BenchmarkError::Simulation(format!("authorized_grasp_control_write:{error}"))
    })?;
    if !step_with_authority(worker, authorized, request.now_s, steps)? {
        return Ok(GraspControlStatus::AuthorityExpired);
    }
    Ok(GraspControlStatus::Written)
}

fn step_with_authority(
    worker: &mut MujocoInstance,
    authorized: &AuthorizedReasonerAction,
    now_s: f64,
    steps: u32,
) -> Result<bool, BenchmarkError> {
    let mut remaining_steps = steps;
    while remaining_steps > 0 {
        if !authorized_covers_now(worker, authorized, now_s)? {
            return Ok(false);
        }
        let window = remaining_steps.min(CONTROL_AUTHORITY_CHECK_STEPS);
        worker.step(window).map_err(|error| {
            BenchmarkError::Simulation(format!("authorized_step_window:{error}"))
        })?;
        remaining_steps -= window;
    }
    Ok(true)
}

fn target_inside_control_range(actuator: &crate::normalize::ActuatorRecord, value: f64) -> bool {
    value.is_finite()
        && actuator.ctrllimited
        && value >= actuator.ctrlrange[0] - 1e-8
        && value <= actuator.ctrlrange[1] + 1e-8
}

fn parse_lift_states(
    contents: &str,
    arm_names: &[String],
    finger_names: &[String],
) -> Vec<(Vec<f64>, Vec<f64>)> {
    contents
        .lines()
        .filter_map(|line| line.strip_prefix("lift_state_"))
        .filter_map(|line| line.split_once('=').map(|(_, payload)| payload))
        .filter_map(|payload| payload.split_once(";fingers="))
        .filter_map(|(arm, fingers)| {
            Some((
                parse_named_witness_values(arm, arm_names)?,
                parse_named_witness_values(fingers, finger_names)?,
            ))
        })
        .collect()
}

fn parse_named_witness_values(payload: &str, expected_names: &[String]) -> Option<Vec<f64>> {
    let mut values = BTreeMap::new();
    for encoded in payload.split(',') {
        let (name_hex, bits) = encoded.split_once('=')?;
        let name = String::from_utf8(hex::decode(name_hex).ok()?).ok()?;
        let value = f64::from_bits(bits.parse::<u64>().ok()?);
        if !value.is_finite() || values.insert(name, value).is_some() {
            return None;
        }
    }
    expected_names
        .iter()
        .map(|name| values.get(name).copied())
        .collect()
}

fn bilateral_finger_contact(
    truth: &VerifierTruth,
    object_id: &str,
    resource: &ControlledResource,
) -> bool {
    resource.finger_bodies.len() == 2
        && truth
            .contacts
            .iter()
            .any(|contact| contact_pair_has(contact, object_id, &resource.finger_bodies[0]))
        && truth
            .contacts
            .iter()
            .any(|contact| contact_pair_has(contact, object_id, &resource.finger_bodies[1]))
}

fn grasp_contact_audits(
    phase: &str,
    truth: &VerifierTruth,
    object_id: &str,
    resource: &ControlledResource,
) -> Vec<GraspContactAudit> {
    resource
        .finger_bodies
        .iter()
        .map(|finger_body| {
            let contacts = truth
                .contacts
                .iter()
                .filter(|contact| {
                    (contact.body1 == *object_id && contact.body2 == *finger_body)
                        || (contact.body2 == *object_id && contact.body1 == *finger_body)
                })
                .collect::<Vec<_>>();
            let force_available = !contacts.is_empty()
                && contacts.iter().all(|contact| {
                    contact.force_available
                        && contact.normal_force.is_some_and(f64::is_finite)
                        && contact.tangential_force.is_some_and(f64::is_finite)
                        && contact.force_local.is_some()
                        && contact.contact_frame.is_some()
                });
            GraspContactAudit {
                phase: phase.into(),
                finger_body: finger_body.clone(),
                normal_force_n: force_available.then(|| {
                    contacts
                        .iter()
                        .filter_map(|contact| contact.normal_force)
                        .sum()
                }),
                tangential_force_n: force_available.then(|| {
                    contacts
                        .iter()
                        .filter_map(|contact| contact.tangential_force)
                        .sum()
                }),
                object_vertical_force_n: force_available.then(|| {
                    contacts
                        .iter()
                        .filter_map(|contact| {
                            let local = contact.force_local?;
                            let frame = contact.contact_frame?;
                            let force_on_second =
                                local[0] * frame[2] + local[1] * frame[5] + local[2] * frame[8];
                            let object_sign = if contact.body2 == object_id {
                                1.0
                            } else {
                                -1.0
                            };
                            Some(force_on_second * object_sign)
                        })
                        .sum()
                }),
                contact_dimensions: contacts
                    .iter()
                    .filter_map(|contact| contact.contact_dim)
                    .collect(),
                contact_friction: contacts
                    .iter()
                    .filter_map(|contact| contact.friction.clone())
                    .collect(),
                force_available,
            }
        })
        .collect()
}

fn contact_pair_has(contact: &crate::observation::ContactTruth, left: &str, right: &str) -> bool {
    (contact.body1 == left && contact.body2 == right)
        || (contact.body1 == right && contact.body2 == left)
}

fn measured_opening(
    state: &Value,
    manifest: &RobotManifest,
    resource: &ControlledResource,
) -> Option<f64> {
    let [minimum, maximum] = resource.opening_range.value?;
    let span = maximum - minimum;
    if !span.is_finite() || span <= 0.0 {
        return None;
    }
    let qpos = json_f64_vec(&state["qpos"]);
    let positions = resource
        .affected_joints
        .iter()
        .map(|name| {
            let joint = manifest.joints.iter().find(|joint| joint.name == *name)?;
            qpos.get(joint.qpos_address as usize).copied()
        })
        .collect::<Option<Vec<_>>>()?;
    (!positions.is_empty()).then(|| {
        positions
            .iter()
            .map(|position| ((position - minimum) / span).clamp(0.0, 1.0))
            .sum::<f64>()
            / positions.len() as f64
    })
}

fn authorized_covers_now(
    worker: &mut MujocoInstance,
    authorized: &AuthorizedReasonerAction,
    now_s: f64,
) -> Result<bool, BenchmarkError> {
    let frozen = authorized.action.frozen();
    let Some(authorization) = frozen.execution_authorization.as_ref() else {
        return Ok(false);
    };
    let runtime_now = 10.0 + simulator_now(worker)?;
    let now = now_s.max(runtime_now);
    Ok(frozen.authority_granted
        && authorization.covers(frozen, now)
        && authorization.grant_id == authorized.grant.command_id
        && authorized
            .grant
            .scope
            .action_envelope
            .exactly_matches(&frozen.action_envelope)
        && grant_covers(&authorized.grant, &authorized.grant.scope, now).is_ok())
}

fn position_actuator_map(
    manifest: &RobotManifest,
    joint_names: &[String],
) -> Result<BTreeMap<String, usize>, BenchmarkError> {
    let mut result = BTreeMap::new();
    for joint in joint_names {
        let matches = manifest
            .actuators
            .iter()
            .enumerate()
            .filter(|(_, actuator)| {
                actuator.transmission_kind == "joint" && actuator.transmission_target == *joint
            })
            .collect::<Vec<_>>();
        if matches.len() != 1 || matches[0].1.actuator_type != "position" {
            return Err(BenchmarkError::InvalidTrial(format!(
                "end-effector joint {joint} has no unique position actuator"
            )));
        }
        result.insert(joint.clone(), matches[0].0);
    }
    Ok(result)
}

fn declared_ee_observation_key(bundle: &RobotBundle, ee_name: &str) -> Option<String> {
    let declared = bundle
        .manifest
        .end_effectors
        .iter()
        .find(|end_effector| end_effector.name == ee_name)?;
    if declared.xyz.is_some() || declared.quat.is_some() {
        return None;
    }
    declared.site.as_ref().or(declared.body.as_ref()).cloned()
}

fn build_push_reasoning_request(
    bundle: &RobotBundle,
    manifest: &RobotManifest,
    model: &realityos_semantics::embodiment::EmbodimentModel,
    scenario: &ManipulationScenario,
    goal_xyz: [f64; 3],
    state: &Value,
    epoch: &str,
    now_s: f64,
    evidence_fresh: bool,
    belief: PhysicalParameterBelief,
) -> Result<ReasoningRequest, BenchmarkError> {
    let ee = model.end_effectors.first().ok_or_else(|| {
        BenchmarkError::InvalidTrial("bundle has no declared end-effector".into())
    })?;
    let frame_key = declared_ee_observation_key(bundle, &ee.name).ok_or_else(|| {
        BenchmarkError::InvalidTrial(format!(
            "end-effector {} has no observable site or body mapping",
            ee.name
        ))
    })?;
    if state["sites"].get(&frame_key).is_none() && state["xpos"].get(&frame_key).is_none() {
        return Err(BenchmarkError::InvalidTrial(format!(
            "declared end-effector frame {frame_key} is absent from the current observation"
        )));
    }
    let joints = model.ee_joint_chain(&ee.name).unwrap_or_default();
    if joints.is_empty()
        || joints.iter().any(|name| {
            manifest
                .joints
                .iter()
                .find(|joint| joint.name == *name)
                .map_or(true, |joint| joint.qpos_dim != 1)
        })
    {
        return Err(BenchmarkError::InvalidTrial(
            "end-effector chain is empty or contains non-scalar joints".into(),
        ));
    }
    let arm_q = read_named_joints_from_state(manifest, &joints, state)?;
    let object_spec = named_object(&scenario.objects, &scenario.object_id)?;
    if object_spec["type"].as_str().unwrap_or("box") != "box" {
        return Err(BenchmarkError::InvalidTrial(
            "push reasoner currently requires a box target geometry".into(),
        ));
    }
    let half = object_half_extents(object_spec)?;
    let center = body_position(state, &scenario.object_id)?;
    let quat = state["xquat"][&scenario.object_id]
        .as_array()
        .and_then(|values| values.iter().map(Value::as_f64).collect::<Option<Vec<_>>>())
        .filter(|values| values.len() == 4)
        .map(|values| [values[0], values[1], values[2], values[3]])
        .unwrap_or([1.0, 0.0, 0.0, 0.0]);
    let support_spec = scenario
        .objects
        .iter()
        .find(|object| object["name"] == "table")
        .ok_or_else(|| BenchmarkError::InvalidTrial("scenario has no table support".into()))?;
    let table_center = value_vec3(&support_spec["pos"])
        .ok_or_else(|| BenchmarkError::InvalidTrial("table support pose is invalid".into()))?;
    let support_half_extents = object_half_extents(support_spec)?;
    let obstacles = scenario
        .objects
        .iter()
        .filter(|object| {
            object["name"].as_str() != Some(&scenario.object_id)
                && object["name"].as_str() != Some("table")
        })
        .map(|object| {
            let name = object["name"]
                .as_str()
                .ok_or_else(|| BenchmarkError::InvalidTrial("obstacle has no name".into()))?;
            let center = value_vec3(&object["pos"]).ok_or_else(|| {
                BenchmarkError::InvalidTrial(format!("obstacle {name} has bad pose"))
            })?;
            let half = obstacle_half_extents(object)?;
            Ok(NamedBox::aabb(name, center, half))
        })
        .collect::<Result<Vec<_>, BenchmarkError>>()?;
    let tool_radius_m = declared_ee_radius(model, &ee.name).ok_or_else(|| {
        BenchmarkError::InvalidTrial("end-effector has no supported collision cross-section".into())
    })?;
    let mass_kg = object_spec["mass"]
        .as_f64()
        .filter(|mass| mass.is_finite() && *mass > 0.0);

    Ok(ReasoningRequest {
        scene: TableScene {
            object_id: scenario.object_id.clone(),
            object_center: center,
            object_half: half,
            object_quat: quat,
            // Collision checking uses the exact finite table box. Anchor its
            // declared top face instead of approximating it as a large slab.
            support_origin: [
                table_center[0],
                table_center[1],
                table_center[2] + support_half_extents[2],
            ],
            support_normal: [0.0, 0.0, 1.0],
            support_half_extents: Some(support_half_extents),
            obstacles,
            arm_q,
            joint_names: joints,
            ee: ee.name.clone(),
            tool_radius_m,
            observation_epoch: epoch.into(),
            held: false,
            finger_names: Vec::new(),
            finger_q: Vec::new(),
        },
        belief,
        objective: PhysicalObjective::PlanarTranslation {
            target_xy: [goal_xyz[0], goal_xyz[1]],
        },
        mu_required: MU_REQUIRED,
        goal_stroke_m: 0.025,
        probe_stroke_m: 0.008,
        grants: Vec::new(),
        now_s,
        evidence_fresh,
        safety_faults: injected_safety_fault_alerts(scenario, now_s),
        mass_kg,
        gripper_force_n: None,
        gripper_resource: None,
        linear_execution_bound_m: None,
        gravity_m_s2: [0.0, 0.0, -9.81],
    })
}

fn build_grasp_reasoning_request(
    bundle: &RobotBundle,
    manifest: &RobotManifest,
    model: &realityos_semantics::embodiment::EmbodimentModel,
    scenario: &ManipulationScenario,
    state: &Value,
    epoch: &str,
    now_s: f64,
    evidence_fresh: bool,
    resource: Option<&ControlledResource>,
    friction_oracle_input: Option<&super::scenario::SimulatedFrictionOracleInput>,
) -> Result<ReasoningRequest, BenchmarkError> {
    let ee = model.end_effectors.first().ok_or_else(|| {
        BenchmarkError::InvalidTrial("bundle has no declared end-effector".into())
    })?;
    let frame_key = declared_ee_observation_key(bundle, &ee.name).ok_or_else(|| {
        BenchmarkError::InvalidTrial(format!(
            "grasp end-effector {} has no observable site or body mapping",
            ee.name
        ))
    })?;
    if state["sites"].get(&frame_key).is_none() && state["xpos"].get(&frame_key).is_none() {
        return Err(BenchmarkError::InvalidTrial(format!(
            "declared end-effector frame {frame_key} is absent from the current observation"
        )));
    }
    let joint_names = model.ee_joint_chain(&ee.name).unwrap_or_default();
    if joint_names.is_empty()
        || joint_names.iter().any(|name| {
            manifest
                .joints
                .iter()
                .find(|joint| joint.name == *name)
                .map_or(true, |joint| joint.qpos_dim != 1)
        })
    {
        return Err(BenchmarkError::InvalidTrial(
            "grasp end-effector chain is empty or contains non-scalar joints".into(),
        ));
    }
    let arm_q = read_named_joints_from_state(manifest, &joint_names, state)?;
    let object_spec = named_object(&scenario.objects, &scenario.object_id)?;
    let object_half = obstacle_half_extents(object_spec)?;
    let object_center = body_position(state, &scenario.object_id)?;
    let object_quat = state["xquat"][&scenario.object_id]
        .as_array()
        .and_then(|values| values.iter().map(Value::as_f64).collect::<Option<Vec<_>>>())
        .filter(|values| values.len() == 4)
        .map(|values| [values[0], values[1], values[2], values[3]])
        .unwrap_or([1.0, 0.0, 0.0, 0.0]);
    let table = named_object(&scenario.objects, "table")?;
    let _ = value_vec3(&table["pos"])
        .ok_or_else(|| BenchmarkError::InvalidTrial("table support pose is invalid".into()))?;
    let _ = object_half_extents(table)?;
    let obstacles = scenario
        .objects
        .iter()
        .filter(|object| {
            object["name"].as_str() != Some(&scenario.object_id)
                && object["name"].as_str() != Some("table")
        })
        .map(|object| {
            let name = object["name"]
                .as_str()
                .ok_or_else(|| BenchmarkError::InvalidTrial("obstacle has no name".into()))?;
            let center = value_vec3(&object["pos"]).ok_or_else(|| {
                BenchmarkError::InvalidTrial(format!("obstacle {name} has bad pose"))
            })?;
            Ok(NamedBox::aabb(name, center, obstacle_half_extents(object)?))
        })
        .collect::<Result<Vec<_>, BenchmarkError>>()?;
    let mut finger_names = Vec::new();
    let mut finger_q = Vec::new();
    if let Some(resource) = resource {
        finger_names = resource.affected_joints.clone();
        finger_q = read_named_joints_from_state(manifest, &finger_names, state)?;
    }
    let mass_kg = object_spec["mass"]
        .as_f64()
        .filter(|mass| mass.is_finite() && *mass > 0.0);
    let gripper_force_n = resource.and_then(|resource| {
        resource
            .force_bound
            .known_value()
            .map(|range| range[1])
            .filter(|force| force.is_finite() && *force > 0.0)
    });
    let mut belief = PhysicalParameterBelief::declared_point(
        PhysicalParameter::SupportFriction,
        0.5,
        "benchmark_support_prior",
    );
    belief.narrow_interval(
        PhysicalParameter::SupportFriction,
        [0.5, 0.5],
        "benchmark_support_calibration",
    );
    belief = belief.with_unknown(
        PhysicalParameter::ToolObjectFriction,
        "unmeasured_grasp_pair",
    );
    if let Some(input) = friction_oracle_input {
        // These are seeded samples from simulator-configured friction, not
        // sensor observations. Their origin remains explicit in belief lineage.
        belief.narrow_interval(
            PhysicalParameter::ToolObjectFriction,
            input.interval,
            &format!(
                "mujoco_oracle_friction_input_v1:samples={:?}",
                input.samples
            ),
        );
    }
    Ok(ReasoningRequest {
        scene: TableScene {
            object_id: scenario.object_id.clone(),
            object_center,
            object_half,
            object_quat,
            // The finite table is represented as an obstacle below. Keep the
            // infinite support-plane proxy at the floor so it cannot classify
            // the robot pedestal below the tabletop as an intersection.
            support_origin: [0.0, 0.0, 0.0],
            support_normal: [0.0, 0.0, 1.0],
            support_half_extents: None,
            obstacles,
            arm_q,
            joint_names,
            ee: ee.name.clone(),
            tool_radius_m: declared_ee_radius(model, &ee.name).ok_or_else(|| {
                BenchmarkError::InvalidTrial(
                    "end-effector has no supported collision cross-section".into(),
                )
            })?,
            observation_epoch: epoch.into(),
            held: false,
            finger_names,
            finger_q,
        },
        belief,
        objective: PhysicalObjective::AcquireObject {
            object_id: scenario.object_id.clone(),
        },
        mu_required: MU_REQUIRED,
        goal_stroke_m: 0.025,
        probe_stroke_m: 0.008,
        grants: Vec::new(),
        now_s,
        evidence_fresh,
        safety_faults: injected_safety_fault_alerts(scenario, now_s),
        mass_kg,
        gripper_force_n,
        gripper_resource: resource.cloned(),
        linear_execution_bound_m: None,
        gravity_m_s2: [0.0, 0.0, -9.81],
    })
}

/// Apply injected fault conditions to reasoner-visible resource observations.
/// The evaluator's `NegKind` selects the fault, but the label itself is never
/// copied into the reasoning request.
fn apply_grasp_fault_observation(
    scenario: &ManipulationScenario,
    resources: &mut Vec<ControlledResource>,
) {
    match scenario.neg {
        Some(NegKind::GripperUnavailable) => resources.clear(),
        Some(NegKind::UnsupportedCoupling) => {
            for resource in resources {
                resource.topology = ResourceTopology::UnsupportedResourceTopology;
                resource.qualification = QualificationStatus::Failed;
                resource.unsupported_detail = Some("injected coupling fault".into());
            }
        }
        Some(NegKind::ForceBoundUnavailable) => {
            for resource in resources {
                resource.force_bound =
                    Provenanced::unknown("simulated_force_sensor_unavailable", 0.0);
            }
        }
        Some(NegKind::ActuatorSaturation) => {
            for resource in resources {
                resource.qualification = QualificationStatus::Failed;
                resource.unsupported_detail = Some("simulated actuator saturation".into());
            }
        }
        Some(NegKind::ControllerFailure) => {
            for resource in resources {
                resource.qualification = QualificationStatus::Failed;
                resource.unsupported_detail = Some("simulated controller health failure".into());
            }
        }
        _ => {}
    }
}

fn reasoning_evidence_fresh(scenario: &ManipulationScenario) -> bool {
    !(scenario.stale
        || scenario.move_object_after_obs
        || scenario.replay
        || scenario.restart_replay
        || scenario.crash_controller)
}

fn injected_safety_fault_alerts(
    scenario: &ManipulationScenario,
    now_s: f64,
) -> Vec<SafetyFaultObservation> {
    let (kind, source) = match scenario.neg {
        Some(NegKind::StaleObject | NegKind::ObjectMoved) => (
            SafetyFaultKind::StaleObservation,
            "mujoco_fault_injection_adapter.stale_observation",
        ),
        Some(NegKind::Replay | NegKind::RestartReplay) => (
            SafetyFaultKind::ReplayDetected,
            "mujoco_fault_injection_adapter.sequence_replay",
        ),
        Some(NegKind::ControllerFailure) => (
            SafetyFaultKind::ControllerUnavailable,
            "mujoco_fault_injection_adapter.controller_unavailable",
        ),
        Some(NegKind::ActuatorSaturation) => (
            SafetyFaultKind::ActuatorSaturated,
            "mujoco_fault_injection_adapter.actuator_saturation",
        ),
        Some(NegKind::GripperUnavailable) => (
            SafetyFaultKind::GripperUnavailable,
            "mujoco_fault_injection_adapter.gripper_unavailable",
        ),
        Some(NegKind::UnsupportedCoupling) => (
            SafetyFaultKind::UnsupportedGripperCoupling,
            "mujoco_fault_injection_adapter.unsupported_coupling",
        ),
        Some(NegKind::ForceBoundUnavailable) => (
            SafetyFaultKind::GripperForceBoundUnavailable,
            "mujoco_fault_injection_adapter.force_bound_unavailable",
        ),
        _ => return Vec::new(),
    };
    vec![SafetyFaultObservation {
        kind,
        asserted: Provenanced::simulator_derived(true, source, now_s),
    }]
}

fn uncertain_push_belief() -> PhysicalParameterBelief {
    let mut belief = PhysicalParameterBelief::declared_point(
        PhysicalParameter::SupportFriction,
        0.5,
        "benchmark_prior",
    );
    belief.narrow_interval(
        PhysicalParameter::SupportFriction,
        [0.05, 0.8],
        "benchmark_uncertain_support",
    );
    belief.with_unknown(
        PhysicalParameter::ToolObjectFriction,
        "unmeasured_tool_object_pair",
    )
}

fn ensure_robot_prefix(
    base: &RobotManifest,
    expanded: &RobotManifest,
) -> Result<(), BenchmarkError> {
    if expanded.nq < base.nq
        || expanded.nv < base.nv
        || expanded.nu != base.nu
        || base.joints.iter().any(|joint| {
            !expanded.joints.iter().any(|candidate| {
                candidate.name == joint.name
                    && candidate.qpos_address == joint.qpos_address
                    && candidate.velocity_address == joint.velocity_address
                    && candidate.qpos_dim == joint.qpos_dim
            })
        })
        || base.actuators != expanded.actuators
    {
        return Err(BenchmarkError::InvalidTrial(
            "scenario model changed robot joint or actuator mapping".into(),
        ));
    }
    Ok(())
}

fn finish_refusal(episode: &mut BenchmarkEpisode, expectation: &TrialExpectation) {
    episode.outcome = if *expectation == TrialExpectation::ExpectedRefusal {
        TrialOutcome::ValidRefusal
    } else {
        if episode.failure_cause.is_none() {
            episode.failure_cause = Some(FailureCause::UnexpectedRefusal);
        }
        TrialOutcome::UnexpectedRefusal
    };
}

fn finalize(
    mut worker: MujocoInstance,
    mut episode: BenchmarkEpisode,
    wall: Instant,
) -> Result<BenchmarkEpisode, BenchmarkError> {
    let final_state = state_payload(&mut worker)?;
    episode.object_end_xyz = body_position(&final_state, &episode.object_id)?;
    episode.goal_error_end_m = xy_distance(episode.object_end_xyz, episode.goal_xyz);
    episode.simulation_time_s = final_state["time"]
        .as_f64()
        .unwrap_or(episode.simulation_time_s);
    episode.ctrl_writes_after = worker
        .peek_ctrl()
        .map_err(|error| BenchmarkError::Simulation(error.to_string()))?
        .1;
    episode.wall_time_s = wall.elapsed().as_secs_f64();
    episode.unauthorized_writes = 0;
    drop(worker);
    Ok(episode)
}

fn state_payload(worker: &mut MujocoInstance) -> Result<Value, BenchmarkError> {
    let response = worker
        .state()
        .map_err(|error| BenchmarkError::Simulation(format!("state_observation:{error}")))?;
    if response["ok"] != true {
        return Err(BenchmarkError::Simulation(
            response["error"]
                .as_str()
                .unwrap_or("state_observation_failed")
                .into(),
        ));
    }
    Ok(response.get("state").cloned().unwrap_or(response))
}

fn simulator_now(worker: &mut MujocoInstance) -> Result<f64, BenchmarkError> {
    Ok(state_payload(worker)?["time"].as_f64().unwrap_or(0.0))
}

fn read_named_joints(
    worker: &mut MujocoInstance,
    manifest: &RobotManifest,
    names: &[String],
) -> Result<Vec<f64>, BenchmarkError> {
    let state = state_payload(worker)?;
    read_named_joints_from_state(manifest, names, &state)
}

fn read_named_joints_from_state(
    manifest: &RobotManifest,
    names: &[String],
    state: &Value,
) -> Result<Vec<f64>, BenchmarkError> {
    let qpos = json_f64_vec(&state["qpos"]);
    names
        .iter()
        .map(|name| {
            let joint = manifest
                .joints
                .iter()
                .find(|joint| joint.name == *name)
                .ok_or_else(|| BenchmarkError::InvalidTrial(format!("unknown joint {name}")))?;
            if joint.qpos_dim != 1 || joint.qpos_address < 0 {
                return Err(BenchmarkError::InvalidTrial(format!(
                    "joint {name} is not a scalar qpos"
                )));
            }
            qpos.get(joint.qpos_address as usize)
                .copied()
                .filter(|value| value.is_finite())
                .ok_or_else(|| {
                    BenchmarkError::InvalidTrial(format!("joint {name} has no observation"))
                })
        })
        .collect()
}

fn body_position(state: &Value, name: &str) -> Result<[f64; 3], BenchmarkError> {
    state["xpos"][name]
        .as_array()
        .and_then(|values| {
            Some([
                values.first()?.as_f64()?,
                values.get(1)?.as_f64()?,
                values.get(2)?.as_f64()?,
            ])
        })
        .filter(|position| position.iter().all(|value| value.is_finite()))
        .ok_or_else(|| {
            BenchmarkError::Simulation(format!("observed body {name} has no finite position"))
        })
}

fn named_object<'a>(objects: &'a [Value], name: &str) -> Result<&'a Value, BenchmarkError> {
    objects
        .iter()
        .find(|object| object["name"] == name)
        .ok_or_else(|| BenchmarkError::InvalidTrial(format!("scenario has no object {name}")))
}

fn value_vec3(value: &Value) -> Option<[f64; 3]> {
    let values = value.as_array()?;
    let point = [
        values.first()?.as_f64()?,
        values.get(1)?.as_f64()?,
        values.get(2)?.as_f64()?,
    ];
    point.iter().all(|value| value.is_finite()).then_some(point)
}

fn object_half_extents(object: &Value) -> Result<[f64; 3], BenchmarkError> {
    let size = object["size"]
        .as_array()
        .ok_or_else(|| BenchmarkError::InvalidTrial("box geometry has no size".into()))?;
    let half = [
        size.first()
            .and_then(Value::as_f64)
            .unwrap_or(f64::NAN)
            .abs(),
        size.get(1)
            .and_then(Value::as_f64)
            .unwrap_or(f64::NAN)
            .abs(),
        size.get(2)
            .and_then(Value::as_f64)
            .unwrap_or(f64::NAN)
            .abs(),
    ];
    if half.iter().all(|value| value.is_finite() && *value > 0.0) {
        Ok(half)
    } else {
        Err(BenchmarkError::InvalidTrial(
            "box half extents are invalid".into(),
        ))
    }
}

fn obstacle_half_extents(object: &Value) -> Result<[f64; 3], BenchmarkError> {
    let size = object["size"]
        .as_array()
        .ok_or_else(|| BenchmarkError::InvalidTrial("obstacle geometry has no size".into()))?;
    let x = size
        .first()
        .and_then(Value::as_f64)
        .unwrap_or(f64::NAN)
        .abs();
    let y = size.get(1).and_then(Value::as_f64).unwrap_or(x).abs();
    let z = size.get(2).and_then(Value::as_f64).unwrap_or(y).abs();
    let kind = object["type"].as_str().unwrap_or("box");
    let half = match kind {
        "sphere" => [x, x, x],
        "cylinder" => [x, x, y],
        "capsule" => [x, x, y + x],
        "box" => [x, y, z],
        other => {
            return Err(BenchmarkError::InvalidTrial(format!(
                "obstacle geometry {other} is unsupported by the conservative planner proxy"
            )))
        }
    };
    if half.iter().all(|value| value.is_finite() && *value > 0.0) {
        Ok(half)
    } else {
        Err(BenchmarkError::InvalidTrial(
            "obstacle half extents are invalid".into(),
        ))
    }
}

fn declared_ee_radius(
    model: &realityos_semantics::embodiment::EmbodimentModel,
    ee_name: &str,
) -> Option<f64> {
    use realityos_semantics::geometry::{CollisionRole, PrimitiveShape};
    let ee = model.end_effectors.iter().find(|ee| ee.name == ee_name)?;
    let frame = model.frames.iter().find(|frame| frame.name == ee.frame)?;
    model
        .collision_geoms
        .iter()
        .filter(|geom| {
            geom.owner_body == frame.parent_body && geom.collision_role == CollisionRole::Collision
        })
        .filter_map(|geom| match &geom.shape {
            PrimitiveShape::Sphere { radius }
            | PrimitiveShape::Capsule { radius, .. }
            | PrimitiveShape::Cylinder { radius, .. } => Some(radius.abs()),
            PrimitiveShape::Box { half_extents } => Some(
                half_extents
                    .iter()
                    .map(|value| value.abs())
                    .fold(0.0, f64::max),
            ),
            PrimitiveShape::Plane { .. } | PrimitiveShape::Unsupported { .. } => None,
        })
        .reduce(f64::max)
}

fn witness_joint_line(contents: &str, prefix: &str) -> Vec<f64> {
    contents
        .lines()
        .find_map(|line| line.strip_prefix(prefix))
        .map(|payload| {
            payload
                .split(',')
                .filter_map(|bits| bits.parse::<u64>().ok())
                .map(f64::from_bits)
                .collect()
        })
        .unwrap_or_default()
}

fn vectors_near(left: &[f64], right: &[f64], tolerance: f64) -> bool {
    left.len() == right.len()
        && left
            .iter()
            .zip(right)
            .all(|(left, right)| (left - right).abs() <= tolerance)
}

fn xy_distance(position: [f64; 3], goal: [f64; 3]) -> f64 {
    ((position[0] - goal[0]).powi(2) + (position[1] - goal[1]).powi(2)).sqrt()
}

fn distance3(left: [f64; 3], right: [f64; 3]) -> f64 {
    ((left[0] - right[0]).powi(2) + (left[1] - right[1]).powi(2) + (left[2] - right[2]).powi(2))
        .sqrt()
}

fn sub3(left: [f64; 3], right: [f64; 3]) -> [f64; 3] {
    [left[0] - right[0], left[1] - right[1], left[2] - right[2]]
}

fn truth_position(truth: &VerifierTruth, name: &str) -> Option<[f64; 3]> {
    let position = truth.named_pos.get(name).or_else(|| truth.xpos.get(name))?;
    (position.len() >= 3).then(|| [position[0], position[1], position[2]])
}

#[cfg(test)]
mod witness_tests {
    use super::decode_push_witness;

    fn line(name: &str, values: &[f64]) -> String {
        let bits = values
            .iter()
            .map(|value| value.to_bits().to_string())
            .collect::<Vec<_>>()
            .join(",");
        format!("{name}={bits}\n")
    }

    fn witness(include_retract: bool) -> String {
        let mut contents = String::new();
        contents.push_str(&line("q0_bits", &[0.1, 0.2]));
        contents.push_str(&line("q_transit_bits", &[0.2, 0.3]));
        contents.push_str(&line("q_contact_bits", &[0.3, 0.4]));
        contents.push_str(&line("q_end_bits", &[0.4, 0.5]));
        if include_retract {
            contents.push_str(&line("q_retract_bits", &[0.2, 0.3]));
        }
        contents
    }

    #[test]
    fn probe_witness_requires_a_finite_full_dimension_retreat() {
        assert!(decode_push_witness(&witness(false), 2, true).is_none());

        let malformed = witness(true).replace(
            &line("q_retract_bits", &[0.2, 0.3]),
            "q_retract_bits=not-a-number\n",
        );
        assert!(decode_push_witness(&malformed, 2, true).is_none());

        let non_finite = witness(true).replace(
            &line("q_retract_bits", &[0.2, 0.3]),
            &line("q_retract_bits", &[f64::NAN, 0.3]),
        );
        assert!(decode_push_witness(&non_finite, 2, true).is_none());
    }

    #[test]
    fn non_probe_push_witness_does_not_require_retreat() {
        let decoded = decode_push_witness(&witness(false), 2, false)
            .expect("ordinary push witnesses have three checked motion targets");
        assert_eq!(decoded.1.len(), 3);
    }
}
