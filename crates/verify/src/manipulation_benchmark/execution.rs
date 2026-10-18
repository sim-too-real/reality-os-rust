//! Strict simulator-backed manipulation benchmark execution.
//!
//! Trial labels stay with the evaluator. The reasoner receives only the current
//! simulated observation, task objective, and a conservative physical belief.

use super::geometry::{validate_collision_geometry, GeometryGateResult};
use super::scenario::{
    paired_trial_key, simulated_grasp_friction_oracle_input, trial_digest, BenchmarkTask,
    ResolvedBenchmarkTrial, TrialExpectation, BENCHMARK_SCHEMA_VERSION,
};
use super::wrench_sensor::{
    SimulationWrenchSensorProfile, SimulationWrenchWindow, WrenchAbortEvent, WrenchAbortReason,
};
use super::BenchmarkError;
use crate::bundle::RobotBundle;
use crate::manipulation_scenarios::{ManipulationScenario, NegKind};
use crate::mujoco_exec::{json_f64_vec, MonitoredControlRamp, MujocoInstance};
use crate::normalize::RobotManifest;
use crate::observation::VerifierTruth;
use crate::resource_discover::discover_resources;
use crate::resource_qualify::qualify_on;
use crate::semantics_map::embodiment_from_manifest;
use realityos_core::RealityOs;
use realityos_kernel::physical_action::PhysicalActionEnvelope;
use realityos_kernel::physical_action::WrenchLimitPolicy;
use realityos_reasoner_bridge::{
    authorize_current_action, AuthorizedReasonerAction, SimulationAdmission,
};
use realityos_semantics::contact_collision::NamedBox;
use realityos_semantics::observation::WrenchSample;
use realityos_semantics::physical_belief::{PhysicalParameter, PhysicalParameterBelief};
use realityos_semantics::physical_reasoner::{
    PhysicalObjective, PhysicalReasoner, ReasoningRequest, SafetyFaultKind, SafetyFaultObservation,
    TableScene,
};
use realityos_semantics::provenance::Provenanced;
use realityos_semantics::push_probe::{
    infer_support_friction, FrictionInferenceResult, IntendedContactEvidence, ProbeMotionWindow,
    PushProbeMeasurement, WrenchTransform,
};
use realityos_semantics::resource::{
    ControlledResource, QualificationStatus, ResourceKind, ResourceTopology,
};
use realityos_semantics::sensor::WrenchSampleOrigin;
use realityos_session::grant_covers;
use serde::{Deserialize, Serialize};
use serde_json::Value;
use std::collections::BTreeMap;
use std::time::Instant;

const MU_REQUIRED: f64 = 0.3;
const MAX_ACTIONS: usize = 6;
const CONTROL_STEPS_PER_SEGMENT: u32 = 500;
const PUSH_STEPS_PER_WAYPOINT: u32 = 500;
const MAX_PROBE_CONTROL_RAMP_STEPS: u32 = 6_000;
const MIN_PROBE_CONTROL_RAMP_STEPS: u32 = 100;
const PROBE_JOINT_RAMP_RATE_RAD_S: f64 = 0.1;
const PROBE_TRANSIT_TRACKING_SETTLE_STEPS: u32 = 2_000;
const PROBE_PRECONTACT_SETTLE_STEPS: u32 = 1_000;
const PROBE_CONTROL_UPDATE_STEPS: u32 = 100;
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
    ForceMonitorAbort,
    ForceBaselineUnavailable,
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
    pub lateral_displacement_m: Option<f64>,
    pub reauthorized_after_observation: bool,
    pub runtime_divergence: bool,
    #[serde(default)]
    pub control_tracking_correction_bound_rad: Option<f64>,
    #[serde(default)]
    pub control_tracking_correction_gain: Option<f64>,
    #[serde(default)]
    pub control_tracking_correction_max_rad: Option<f64>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WrenchGravityReference {
    pub sample: WrenchSample,
    pub sensor_position: [f64; 3],
    pub sensor_orientation: [f64; 4],
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
    #[serde(default)]
    pub paired_trial_key: Option<String>,
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
    #[serde(default)]
    pub wrench_sensor_profile_digest: Option<String>,
    #[serde(default)]
    pub wrench_compensation_digest: Option<String>,
    #[serde(default)]
    pub wrench_inference_compensation_digest: Option<String>,
    #[serde(default)]
    pub wrench_inference_measurement_digest: Option<String>,
    #[serde(default)]
    pub wrench_precontact_baseline: Option<WrenchSample>,
    #[serde(default)]
    pub wrench_precontact_baseline_status: Option<String>,
    #[serde(default)]
    pub wrench_contact_pairs: Vec<String>,
    #[serde(default)]
    pub wrench_monitor_gravity_reference: Option<WrenchSample>,
    #[serde(default)]
    pub wrench_monitor_gravity_reference_status: Option<String>,
    #[serde(default)]
    pub wrench_monitor_gravity_reference_history: Vec<WrenchGravityReference>,
    #[serde(default)]
    pub wrench_monitor_gravity_reference_position: Option<[f64; 3]>,
    #[serde(default)]
    pub wrench_monitor_gravity_reference_orientation: Option<[f64; 4]>,
    #[serde(default)]
    pub wrench_sample_count: u64,
    #[serde(default)]
    pub wrench_max_sample_age_s: Option<f64>,
    #[serde(default)]
    pub wrench_max_inter_sample_gap_s: Option<f64>,
    #[serde(default)]
    pub wrench_max_object_speed_m_s: Option<f64>,
    #[serde(default)]
    pub wrench_peak_conservative_bounds: Option<[f64; 6]>,
    #[serde(default)]
    pub wrench_caps: Option<[f64; 6]>,
    #[serde(default)]
    pub wrench_minimum_margin: Option<f64>,
    #[serde(default)]
    pub wrench_abort: Option<WrenchAbortEvent>,
    #[serde(default)]
    pub wrench_abort_axis: Option<usize>,
    #[serde(default)]
    pub wrench_abort_result: Option<String>,
    #[serde(default)]
    pub wrench_inference_interval: Option<[f64; 2]>,
    #[serde(default)]
    pub wrench_inference_reason: Option<String>,
    #[serde(default)]
    pub prior_support_friction_interval: Option<[f64; 2]>,
    #[serde(default)]
    pub posterior_support_friction_interval: Option<[f64; 2]>,
}

pub fn run_benchmark_trial(
    bundle: &RobotBundle,
    trial: &ResolvedBenchmarkTrial,
    admission: SimulationAdmission,
    wrench_profile: Option<&SimulationWrenchSensorProfile>,
) -> Result<BenchmarkEpisode, BenchmarkError> {
    let wall = Instant::now();
    if trial.schema_version != BENCHMARK_SCHEMA_VERSION {
        return Err(BenchmarkError::InvalidTrial(format!(
            "schema-v{} trial cannot be executed by schema-v{}",
            trial.schema_version, BENCHMARK_SCHEMA_VERSION
        )));
    }
    let computed_paired_key = paired_trial_key(trial)?;
    if trial.paired_trial_key.as_deref() != Some(computed_paired_key.as_str()) {
        return Err(BenchmarkError::InvalidTrial(
            "schema-v6 trial is missing or has a mismatched paired_trial_key".into(),
        ));
    }
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
    match (trial.task, wrench_profile) {
        (BenchmarkTask::PushProbeReplan, Some(profile)) => {
            profile
                .validate_for_timestep(base_manifest.timestep)
                .map_err(|error| BenchmarkError::InvalidTrial(error.to_string()))?;
            admission
                .wrench_limit_policy
                .validate()
                .map_err(|error| BenchmarkError::InvalidTrial(error.to_string()))?;
            if admission.wrench_limit_policy != profile.limit_policy
                || admission.wrench_limit_policy.digest() != profile.limit_policy.digest()
            {
                return Err(BenchmarkError::InvalidTrial(
                    "simulation admission wrench policy must exactly match the registered profile"
                        .into(),
                ));
            }
        }
        (BenchmarkTask::PushProbeReplan, None) => {
            return Err(BenchmarkError::InvalidTrial(
                "push/probe execution requires an explicit registered simulation wrench profile"
                    .into(),
            ));
        }
        (BenchmarkTask::PinchGraspLift, Some(_)) => {
            return Err(BenchmarkError::InvalidTrial(
                "grasp-only execution must not receive a push wrench profile".into(),
            ));
        }
        (BenchmarkTask::PinchGraspLift, None) => {}
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

    let loaded = match wrench_profile {
        Some(profile) => worker.load_bundle_with_wrench_profile(
            bundle,
            &trial.scenario.objects,
            trial.seed,
            profile,
        ),
        None => worker.load_bundle(bundle, &trial.scenario.objects, trial.seed),
    }
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
        paired_trial_key: trial.paired_trial_key.clone(),
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
        wrench_sensor_profile_digest: wrench_profile
            .map(SimulationWrenchSensorProfile::digest)
            .transpose()
            .map_err(|error| BenchmarkError::InvalidTrial(error.to_string()))?,
        wrench_compensation_digest: wrench_profile
            .map(|profile| profile.limit_policy.compensation_digest.clone()),
        wrench_inference_compensation_digest: None,
        wrench_inference_measurement_digest: None,
        wrench_precontact_baseline: None,
        wrench_precontact_baseline_status: None,
        wrench_contact_pairs: Vec::new(),
        wrench_monitor_gravity_reference: None,
        wrench_monitor_gravity_reference_status: None,
        wrench_monitor_gravity_reference_history: Vec::new(),
        wrench_monitor_gravity_reference_position: None,
        wrench_monitor_gravity_reference_orientation: None,
        wrench_sample_count: 0,
        wrench_max_sample_age_s: None,
        wrench_max_inter_sample_gap_s: None,
        wrench_max_object_speed_m_s: None,
        wrench_peak_conservative_bounds: None,
        wrench_caps: wrench_profile.map(|profile| profile.limit_policy.scope.caps),
        wrench_minimum_margin: None,
        wrench_abort: None,
        wrench_abort_axis: None,
        wrench_abort_result: None,
        wrench_inference_interval: None,
        wrench_inference_reason: None,
        prior_support_friction_interval: None,
        posterior_support_friction_interval: None,
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
            "decision={:?};support_friction_interval={:?};candidates={}",
            preview.decision,
            belief
                .entry(PhysicalParameter::SupportFriction)
                .and_then(|entry| entry.empirical_interval),
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
            &admission,
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
            divergence_fault_pending = false;
        }
        let mut push_witness_audit = None;
        let (exec, grasp_hold_start, grasp_phase_positions) = match trial.task {
            BenchmarkTask::PushProbeReplan => {
                let profile = wrench_profile.expect("push profile was validated above");
                let result = execute_push_witness(
                    &mut worker,
                    &manifest,
                    &request,
                    &authorized,
                    admission.execution_bound_m,
                    profile,
                    &admission.wrench_limit_policy,
                    divergence_injected,
                )?;
                let execution = result.execution;
                push_witness_audit = Some(result);
                (execution, None, Vec::new())
            }
            BenchmarkTask::PinchGraspLift => {
                let Some(resource) = grasp_resource.as_ref() else {
                    episode.failure_cause = Some(FailureCause::UnsupportedTask);
                    episode.refusal_code = Some("QUALIFIED_GRIPPER_RESOURCE_REQUIRED".into());
                    episode.outcome = TrialOutcome::ActionFailed;
                    break;
                };
                let result =
                    execute_grasp_witness(&mut worker, &manifest, &request, &authorized, resource)?;
                episode
                    .grasp_contact_audits
                    .extend(result.lift_contact_audits.iter().cloned());
                (
                    result.execution,
                    result.hold_start,
                    result.phase_object_positions,
                )
            }
        };
        episode.grasp_phase_object_positions = grasp_phase_positions;
        let after_state = state_payload(&mut worker)?;
        let object_after = body_position(&after_state, &trial.scenario.object_id)?;
        let displacement_m = distance3(object_before, object_after);
        let object_delta = sub3(object_after, object_before);
        let lateral_displacement_m = action_direction(&action_key).map(|direction| {
            (object_delta[0] * direction[1] - object_delta[1] * direction[0]).abs()
        });
        let writes_after_action = worker
            .peek_ctrl()
            .map_err(|error| BenchmarkError::Simulation(error.to_string()))?
            .1;
        if let Some(audit) = push_witness_audit.as_ref() {
            update_wrench_episode_audit(&mut episode, audit, wrench_profile.expect("profile"));
        }
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
            lateral_displacement_m,
            reauthorized_after_observation: !previous_epoch.is_empty()
                && previous_epoch != frozen.observation_epoch,
            runtime_divergence: exec == WitnessExecution::Diverged,
            control_tracking_correction_bound_rad: push_witness_audit.as_ref().map(|_| {
                admission
                    .wrench_limit_policy
                    .max_position_tracking_correction_rad
            }),
            control_tracking_correction_gain: push_witness_audit.as_ref().map(|_| {
                admission
                    .wrench_limit_policy
                    .position_tracking_correction_gain
            }),
            control_tracking_correction_max_rad: push_witness_audit
                .as_ref()
                .map(|audit| audit.control_correction_max_rad),
        });
        if exec == WitnessExecution::Diverged {
            episode.failure_cause = Some(FailureCause::RuntimeDivergence);
            episode.refusal_code = Some("RUNTIME_DIVERGENCE_REOBSERVE".into());
            episode.outcome = TrialOutcome::ActionFailed;
            previous_epoch = frozen.observation_epoch.clone();
            previous_action_was_probe = false;
            if !blocked_action_keys.contains(&action_key) {
                blocked_action_keys.push(action_key.clone());
            }
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
        if exec == WitnessExecution::BaselineUnavailable {
            episode.failure_cause = Some(FailureCause::ForceBaselineUnavailable);
            episode.refusal_code = Some("PRECONTACT_FORCE_BASELINE_UNAVAILABLE".into());
            episode.outcome = TrialOutcome::ActionFailed;
            break;
        }
        if exec == WitnessExecution::WrenchAborted {
            episode.failure_cause = Some(FailureCause::ForceMonitorAbort);
            episode.refusal_code = Some("WRENCH_MONITOR_ABORTED_ACTION".into());
            episode.wrench_abort_result = Some(
                if episode
                    .wrench_abort
                    .as_ref()
                    .is_some_and(|event| event.stop_verified)
                {
                    "preauthorized_hold_verified"
                } else {
                    "preauthorized_hold_unverified_fault_latched"
                }
                .into(),
            );
            episode.outcome = TrialOutcome::ActionFailed;
            let state = state_payload(&mut worker)?;
            let object = body_position(&state, &trial.scenario.object_id)?;
            episode.object_end_xyz = object;
            episode.goal_error_end_m = xy_distance(object, trial.goal_xyz);
            episode.simulation_time_s = state["time"].as_f64().unwrap_or(0.0);
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
            let profile = wrench_profile.expect("push profile was validated above");
            let audit = push_witness_audit.as_ref().ok_or_else(|| {
                BenchmarkError::Simulation("push execution omitted wrench audit".into())
            })?;
            let prior = belief
                .entry(PhysicalParameter::SupportFriction)
                .and_then(|entry| entry.empirical_interval)
                .unwrap_or([0.05, 0.8]);
            episode.prior_support_friction_interval = Some(prior);
            let measurement = push_probe_measurement(
                &manifest,
                &request,
                profile,
                audit,
                &geometry.digest,
                &after_state,
            )?;
            episode.wrench_precontact_baseline = measurement.precontact_baseline.clone();
            episode.wrench_inference_compensation_digest =
                Some(inference_compensation_digest(&measurement));
            episode.wrench_inference_measurement_digest = Some(measurement.digest());
            match infer_support_friction(&measurement) {
                FrictionInferenceResult::Updated { interval, mode } => {
                    episode.wrench_inference_interval = Some(interval);
                    episode.wrench_inference_reason = Some(format!("updated:{mode:?}"));
                    match belief.intersect_support_friction_with_probe(interval, &measurement) {
                        Ok(posterior) => {
                            episode.posterior_support_friction_interval = Some(posterior);
                            if posterior == prior && !blocked_action_keys.contains(&action_key) {
                                blocked_action_keys.push(action_key.clone());
                            }
                        }
                        Err(error) => {
                            episode.wrench_inference_reason =
                                Some(format!("belief_update_rejected:{error:?}"));
                            if !blocked_action_keys.contains(&action_key) {
                                blocked_action_keys.push(action_key.clone());
                            }
                        }
                    }
                }
                FrictionInferenceResult::NoInference(reason) => {
                    episode.wrench_inference_reason = Some(format!("no_inference:{reason:?}"));
                    episode.posterior_support_friction_interval = Some(prior);
                    if !blocked_action_keys.contains(&action_key) {
                        // Without a strict evidence-derived narrowing, this
                        // interaction cannot justify repeating the same probe.
                        blocked_action_keys.push(action_key.clone());
                    }
                }
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
    WrenchAborted,
    BaselineUnavailable,
    AuthorityExpired,
}

struct PushMotionAudit {
    started: bool,
    start_s: f64,
    end_s: f64,
    start_position: [f64; 3],
    end_position: [f64; 3],
    max_speed_m_s: f64,
    first_motion_s: Option<f64>,
}

impl Default for PushMotionAudit {
    fn default() -> Self {
        Self {
            started: false,
            start_s: 0.0,
            end_s: 0.0,
            start_position: [0.0; 3],
            end_position: [0.0; 3],
            max_speed_m_s: 0.0,
            first_motion_s: None,
        }
    }
}

struct PushWitnessAudit {
    execution: WitnessExecution,
    samples: Vec<WrenchSample>,
    sensor_positions: Vec<[f64; 3]>,
    sensor_orientations: Vec<[f64; 4]>,
    precontact_baseline: Option<WrenchSample>,
    precontact_baseline_status: Option<String>,
    precontact_baseline_position: Option<[f64; 3]>,
    precontact_baseline_orientation: Option<[f64; 4]>,
    wrench_contact_pairs: Vec<String>,
    monitor_gravity_reference: Option<WrenchSample>,
    monitor_gravity_reference_status: Option<String>,
    monitor_gravity_reference_position: Option<[f64; 3]>,
    monitor_gravity_reference_orientation: Option<[f64; 4]>,
    monitor_gravity_references: Vec<WrenchGravityReference>,
    sample_gravity_reference_indices: Vec<Option<usize>>,
    abort: Option<WrenchAbortEvent>,
    motion: PushMotionAudit,
    control_correction_max_rad: f64,
}

fn record_push_wrench_window(audit: &mut PushWitnessAudit, window: SimulationWrenchWindow) {
    if let Some(status) = window.monitor_gravity_reference_status.as_ref() {
        audit.monitor_gravity_reference_status = Some(status.clone());
    }
    let reference_index = if let (Some(sample), Some(sensor_position), Some(sensor_orientation)) = (
        window.monitor_gravity_reference.as_ref(),
        window.monitor_gravity_reference_position,
        window.monitor_gravity_reference_orientation,
    ) {
        let index = audit
            .monitor_gravity_references
            .iter()
            .position(|reference| reference.sample.digest == sample.digest)
            .unwrap_or_else(|| {
                audit
                    .monitor_gravity_references
                    .push(WrenchGravityReference {
                        sample: sample.clone(),
                        sensor_position,
                        sensor_orientation,
                    });
                audit.monitor_gravity_references.len() - 1
            });
        audit.monitor_gravity_reference = Some(sample.clone());
        audit.monitor_gravity_reference_position = Some(sensor_position);
        audit.monitor_gravity_reference_orientation = Some(sensor_orientation);
        Some(index)
    } else {
        None
    };
    audit
        .sample_gravity_reference_indices
        .extend(std::iter::repeat(reference_index).take(window.samples.len()));
    audit.samples.extend(window.samples);
    audit.sensor_positions.extend(window.sensor_positions);
    audit.sensor_orientations.extend(window.sensor_orientations);
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
    profile: &SimulationWrenchSensorProfile,
    policy: &WrenchLimitPolicy,
    inject_runtime_divergence: bool,
) -> Result<PushWitnessAudit, BenchmarkError> {
    let mut divergence_injection_pending = inject_runtime_divergence;
    let mut audit = PushWitnessAudit {
        execution: WitnessExecution::WroteNothing,
        samples: Vec::new(),
        sensor_positions: Vec::new(),
        sensor_orientations: Vec::new(),
        precontact_baseline: None,
        precontact_baseline_status: None,
        precontact_baseline_position: None,
        precontact_baseline_orientation: None,
        wrench_contact_pairs: Vec::new(),
        monitor_gravity_reference: None,
        monitor_gravity_reference_status: None,
        monitor_gravity_reference_position: None,
        monitor_gravity_reference_orientation: None,
        monitor_gravity_references: Vec::new(),
        sample_gravity_reference_indices: Vec::new(),
        abort: None,
        motion: PushMotionAudit::default(),
        control_correction_max_rad: 0.0,
    };
    let frozen = authorized.action.frozen();
    let (requested, maximum) = frozen
        .action_envelope
        .linear_stroke_bounds()
        .ok_or_else(|| BenchmarkError::Simulation("expected linear-stroke action".into()))?;
    if requested > maximum || maximum > execution_bound_m + 1e-9 {
        return Ok(audit);
    }
    let contents = &frozen.witness_contents;
    let is_probe = frozen.action_key.starts_with("probe:");
    let Some((q0, transit_count, targets)) =
        decode_push_witness(contents, request.scene.joint_names.len(), is_probe)
    else {
        return Ok(audit);
    };
    let last_target_index = targets.len().saturating_sub(1);
    if !authorized_covers_now(worker, authorized, request.now_s)? {
        return Ok(audit);
    }
    let measured = read_named_joints(worker, manifest, &request.scene.joint_names)?;
    if !vectors_near(&measured, &q0, 0.04) {
        audit.execution = WitnessExecution::Diverged;
        return Ok(audit);
    }

    if is_probe && audit.monitor_gravity_reference.is_none() {
        let mut settle_remaining = PROBE_PRECONTACT_SETTLE_STEPS;
        while settle_remaining > 0 {
            let runtime_now_s = 10.0 + simulator_now(worker)?;
            if !authorization_covers_at(authorized, runtime_now_s) {
                audit.execution = WitnessExecution::AuthorityExpired;
                return Ok(audit);
            }
            let steps_until_expiry = ((authorized.grant.scope.expires_at_s - runtime_now_s + 1e-9)
                / manifest.timestep)
                .floor()
                .max(0.0) as u32;
            if steps_until_expiry == 0 {
                audit.execution = WitnessExecution::AuthorityExpired;
                return Ok(audit);
            }
            let steps = settle_remaining.min(steps_until_expiry);
            let monitor = worker
                .step_monitored(steps, profile, policy, &authorized.grant, false, true, None)
                .map_err(|error| {
                    BenchmarkError::Simulation(format!("wrench_initial_settle:{error}"))
                })?;
            audit.control_correction_max_rad = audit
                .control_correction_max_rad
                .max(monitor.control_correction_max_rad);
            record_wrench_contacts(&mut audit, &monitor.state);
            record_push_wrench_window(&mut audit, monitor.wrench_window);
            if let Some(abort) = monitor.abort {
                audit.execution = WitnessExecution::WrenchAborted;
                audit.abort = Some(abort);
                return Ok(audit);
            }
            if audit.monitor_gravity_reference_status.as_deref() == Some("contact_present") {
                audit.execution = WitnessExecution::BaselineUnavailable;
                return Ok(audit);
            }
            if audit.monitor_gravity_reference.is_some() {
                break;
            }
            settle_remaining -= steps;
        }
        if audit.monitor_gravity_reference.is_none() {
            audit.execution = WitnessExecution::BaselineUnavailable;
            return Ok(audit);
        }
        let settled = read_named_joints(worker, manifest, &request.scene.joint_names)?;
        if !vectors_near(&settled, &q0, 0.04) {
            audit.execution = WitnessExecution::Diverged;
            return Ok(audit);
        }
    }

    let actuator_map = position_actuator_map(manifest, &request.scene.joint_names)?;
    let mut has_written = false;
    let direction = action_direction(&frozen.action_key);
    let mut stroke_completed_by_observation = false;
    let observed_completion_distance = if is_probe {
        requested.min(0.0045)
    } else {
        requested
    };
    for (target_index, target) in targets.into_iter().enumerate() {
        let transit_target = target_index < transit_count;
        let motion_phase =
            target_index >= transit_count && (!is_probe || target_index < last_target_index);
        if stroke_completed_by_observation && target_index != last_target_index {
            continue;
        }
        let motion_reference = if motion_phase && !audit.motion.started {
            let state = state_payload(worker)?;
            Some((
                state["time"].as_f64().unwrap_or(0.0),
                body_position(&state, &request.scene.object_id)?,
            ))
        } else {
            None
        };
        let now = simulator_now(worker)?;
        let now_s = 10.0 + now;
        if !authorized_covers_now(worker, authorized, now_s)? {
            audit.execution = if has_written {
                WitnessExecution::AuthorityExpired
            } else {
                WitnessExecution::WroteNothing
            };
            return Ok(audit);
        }
        let (ctrl, _) = worker
            .peek_ctrl()
            .map_err(|error| BenchmarkError::Simulation(error.to_string()))?;
        if ctrl.len() != manifest.nu.max(0) as usize {
            return Ok(audit);
        }
        let mut next = ctrl.clone();
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
                return Ok(audit);
            }
            next[index] = *value;
        }
        if next.iter().any(|value| !value.is_finite()) {
            return Ok(audit);
        }
        if !authorized_covers_now(worker, authorized, now_s)? {
            return Ok(audit);
        }
        let probe_control_ramp =
            is_probe && (motion_phase || transit_target || target_index == last_target_index);
        let ramp_target = next.clone();
        if !probe_control_ramp {
            worker.set_ctrl(&next).map_err(|error| {
                BenchmarkError::Simulation(format!("authorized_control_write:{error}"))
            })?;
            has_written = true;
        }
        let mut converged = false;
        let total_steps = if probe_control_ramp {
            probe_control_ramp_steps(
                &ctrl,
                &ramp_target,
                &actuator_map,
                &request.scene.joint_names,
                manifest.timestep,
            )
        } else {
            PUSH_STEPS_PER_WAYPOINT
        };
        let mut remaining_steps = total_steps;
        let mut elapsed_steps = 0_u32;
        while remaining_steps > 0 {
            // Sample contact motion every simulator step so a small bounded
            // probe cannot overshoot between observations. Transit remains
            // batched because it must be object-motion-free.
            let check_steps = if target_index == 0 && is_probe {
                PROBE_CONTROL_UPDATE_STEPS
            } else if target_index == 0 {
                CONTROL_AUTHORITY_CHECK_STEPS
            } else if probe_control_ramp {
                PROBE_CONTROL_UPDATE_STEPS
            } else {
                1
            };
            let runtime_now_s = 10.0 + simulator_now(worker)?;
            if !authorization_covers_at(authorized, runtime_now_s) {
                audit.execution = WitnessExecution::AuthorityExpired;
                return Ok(audit);
            }
            // Never batch physics across the grant's expiry boundary. A check
            // only before a 10-step batch could otherwise let the simulation
            // keep executing an already expired action for several frames.
            let steps_until_expiry = ((authorized.grant.scope.expires_at_s - runtime_now_s + 1e-9)
                / manifest.timestep)
                .floor()
                .max(0.0) as u32;
            if steps_until_expiry == 0 {
                audit.execution = WitnessExecution::AuthorityExpired;
                return Ok(audit);
            }
            let steps = remaining_steps.min(check_steps).min(steps_until_expiry);
            let control_ramp = probe_control_ramp.then(|| {
                has_written = true;
                MonitoredControlRamp {
                    start_ctrl: &ctrl,
                    target_ctrl: &ramp_target,
                    elapsed_steps,
                    total_steps,
                }
            });
            let monitor = worker
                .step_monitored(
                    steps,
                    profile,
                    policy,
                    &authorized.grant,
                    is_probe && motion_phase && elapsed_steps == 0,
                    false,
                    control_ramp,
                )
                .map_err(|error| {
                    BenchmarkError::Simulation(format!("wrench_monitored_step:{error}"))
                })?;
            audit.control_correction_max_rad = audit
                .control_correction_max_rad
                .max(monitor.control_correction_max_rad);
            record_wrench_contacts(&mut audit, &monitor.state);
            if audit.precontact_baseline_status.is_none() {
                audit.precontact_baseline_status =
                    monitor.wrench_window.precontact_baseline_status.clone();
            }
            if audit.precontact_baseline.is_none() {
                audit.precontact_baseline = monitor.wrench_window.precontact_baseline.clone();
                audit.precontact_baseline_position =
                    monitor.wrench_window.precontact_baseline_position;
                audit.precontact_baseline_orientation =
                    monitor.wrench_window.precontact_baseline_orientation;
            }
            record_push_wrench_window(&mut audit, monitor.wrench_window);
            remaining_steps -= steps;
            elapsed_steps += steps;
            let mut inject_now = false;
            if divergence_injection_pending && elapsed_steps == steps {
                let direction = direction.ok_or_else(|| {
                    BenchmarkError::InvalidTrial(
                        "runtime_divergence injection needs a planar action direction".into(),
                    )
                })?;
                let mut position = body_position(&monitor.state, &request.scene.object_id)?;
                position[0] += -direction[1] * 0.02;
                position[1] += direction[0] * 0.02;
                worker
                    .set_body_pos(&request.scene.object_id, position)
                    .map_err(|error| {
                        BenchmarkError::Simulation(format!("runtime_divergence_injection:{error}"))
                    })?;
                inject_now = true;
                divergence_injection_pending = false;
            }
            let state = if inject_now {
                state_payload(worker)?
            } else {
                monitor.state
            };
            let time_s = state["time"].as_f64().unwrap_or(0.0);
            if motion_phase {
                if !audit.motion.started {
                    let (start_s, start_position) = motion_reference.unwrap_or((
                        time_s - f64::from(steps) * manifest.timestep,
                        request.scene.object_center,
                    ));
                    audit.motion.started = true;
                    audit.motion.start_s = start_s;
                    audit.motion.end_s = audit.motion.start_s;
                    audit.motion.start_position = start_position;
                    audit.motion.end_position = start_position;
                }
                let prior_position = audit.motion.end_position;
                let current_position = body_position(&state, &request.scene.object_id)?;
                let step_delta = sub3(current_position, prior_position);
                let start_delta = sub3(current_position, audit.motion.start_position);
                if audit.motion.first_motion_s.is_none()
                    && start_delta[0].hypot(start_delta[1]) > 0.0001
                {
                    audit.motion.first_motion_s = Some(time_s);
                }
                let elapsed_s = f64::from(steps) * manifest.timestep;
                if elapsed_s > 0.0 {
                    audit.motion.max_speed_m_s = audit
                        .motion
                        .max_speed_m_s
                        .max(step_delta[0].hypot(step_delta[1]) / elapsed_s);
                }
                audit.motion.end_s = time_s;
                audit.motion.end_position = current_position;
            }
            if let Some(abort) = monitor.abort {
                audit.execution = WitnessExecution::WrenchAborted;
                audit.abort = Some(abort);
                return Ok(audit);
            }
            let displacement = sub3(
                body_position(&state, &request.scene.object_id)?,
                request.scene.object_center,
            );
            let planar_displacement = displacement[0].hypot(displacement[1]);
            // Transit must not move the object. The bounded contact phase can
            // begin the authorized stroke before the final stroke waypoint,
            // so allow only the requested displacement plus a small model
            // tolerance once contact has been intentionally established.
            let permitted_planar_displacement = if transit_target {
                0.005
            } else {
                requested + 0.005
            };
            let permitted_along_displacement = if transit_target {
                0.005
            } else {
                requested + 0.003
            };
            let along = direction.map(|axis| displacement[0] * axis[0] + displacement[1] * axis[1]);
            let lateral =
                direction.map(|axis| (displacement[0] * axis[1] - displacement[1] * axis[0]).abs());
            if planar_displacement > permitted_planar_displacement
                || displacement[2].abs() > 0.01
                || along.is_some_and(|value| value < -0.005 || value > permitted_along_displacement)
                || lateral.is_some_and(|value| value > 0.006)
            {
                audit.execution = WitnessExecution::Diverged;
                return Ok(audit);
            }
            if !stroke_completed_by_observation
                && target_index > 0
                && along.is_some_and(|value| value >= observed_completion_distance)
            {
                if is_probe {
                    // The measured effect completed the authorized probe.
                    // Skip its remaining stroke waypoints, but execute the
                    // already-authorized retreat before reobserving.
                    stroke_completed_by_observation = true;
                    converged = true;
                    break;
                }
                // The measured object effect completed the authorized goal
                // stroke. Stop issuing the frozen remainder and let the outer
                // loop reobserve and obtain fresh authority.
                audit.execution = WitnessExecution::Wrote;
                return Ok(audit);
            }
            if remaining_steps > 0
                && (probe_control_ramp || remaining_steps % CONTROL_CONVERGENCE_CHECK_STEPS != 0)
            {
                continue;
            }
            let observed = read_named_joints(worker, manifest, &request.scene.joint_names)?;
            if vectors_near(&observed, &target, 0.02) {
                converged = true;
                break;
            }
        }
        if !converged && is_probe && transit_target {
            // The probe transit ramp is deliberately slew-limited. Give the
            // arm a short monitored hold at its already-authorized endpoint
            // to settle tracking lag before treating a small joint error as
            // divergence. No new control target or authority is introduced.
            let mut settle_remaining = PROBE_TRANSIT_TRACKING_SETTLE_STEPS;
            while settle_remaining > 0 {
                let runtime_now_s = 10.0 + simulator_now(worker)?;
                if !authorization_covers_at(authorized, runtime_now_s) {
                    audit.execution = WitnessExecution::AuthorityExpired;
                    return Ok(audit);
                }
                let steps_until_expiry = ((authorized.grant.scope.expires_at_s - runtime_now_s
                    + 1e-9)
                    / manifest.timestep)
                    .floor()
                    .max(0.0) as u32;
                if steps_until_expiry == 0 {
                    audit.execution = WitnessExecution::AuthorityExpired;
                    return Ok(audit);
                }
                let steps = settle_remaining
                    .min(PROBE_CONTROL_UPDATE_STEPS)
                    .min(steps_until_expiry);
                let monitor = worker
                    .step_monitored(
                        steps,
                        profile,
                        policy,
                        &authorized.grant,
                        false,
                        false,
                        None,
                    )
                    .map_err(|error| {
                        BenchmarkError::Simulation(format!(
                            "wrench_transit_tracking_settle:{error}"
                        ))
                    })?;
                audit.control_correction_max_rad = audit
                    .control_correction_max_rad
                    .max(monitor.control_correction_max_rad);
                record_wrench_contacts(&mut audit, &monitor.state);
                record_push_wrench_window(&mut audit, monitor.wrench_window);
                settle_remaining -= steps;
                if let Some(abort) = monitor.abort {
                    audit.execution = WitnessExecution::WrenchAborted;
                    audit.abort = Some(abort);
                    return Ok(audit);
                }
                let displacement = sub3(
                    body_position(&monitor.state, &request.scene.object_id)?,
                    request.scene.object_center,
                );
                let planar_displacement = displacement[0].hypot(displacement[1]);
                let along =
                    direction.map(|axis| displacement[0] * axis[0] + displacement[1] * axis[1]);
                let lateral = direction
                    .map(|axis| (displacement[0] * axis[1] - displacement[1] * axis[0]).abs());
                if planar_displacement > 0.005
                    || displacement[2].abs() > 0.01
                    || along.is_some_and(|value| value < -0.005 || value > 0.005)
                    || lateral.is_some_and(|value| value > 0.006)
                {
                    audit.execution = WitnessExecution::Diverged;
                    return Ok(audit);
                }
                let observed = read_named_joints_from_state(
                    manifest,
                    &request.scene.joint_names,
                    &monitor.state,
                )?;
                if vectors_near(&observed, &target, 0.02) {
                    converged = true;
                    break;
                }
            }
        }
        if !converged {
            audit.execution = WitnessExecution::Diverged;
            return Ok(audit);
        }
        if is_probe && target_index + 1 == transit_count && audit.precontact_baseline.is_none() {
            let mut settle_remaining = PROBE_PRECONTACT_SETTLE_STEPS;
            while settle_remaining > 0 {
                let runtime_now_s = 10.0 + simulator_now(worker)?;
                if !authorization_covers_at(authorized, runtime_now_s) {
                    audit.execution = WitnessExecution::AuthorityExpired;
                    return Ok(audit);
                }
                let steps_until_expiry = ((authorized.grant.scope.expires_at_s - runtime_now_s
                    + 1e-9)
                    / manifest.timestep)
                    .floor()
                    .max(0.0) as u32;
                if steps_until_expiry == 0 {
                    audit.execution = WitnessExecution::AuthorityExpired;
                    return Ok(audit);
                }
                let steps = settle_remaining.min(steps_until_expiry);
                let monitor = worker
                    .step_monitored(steps, profile, policy, &authorized.grant, true, true, None)
                    .map_err(|error| {
                        BenchmarkError::Simulation(format!("wrench_precontact_settle:{error}"))
                    })?;
                audit.control_correction_max_rad = audit
                    .control_correction_max_rad
                    .max(monitor.control_correction_max_rad);
                record_wrench_contacts(&mut audit, &monitor.state);
                let baseline_status = monitor.wrench_window.precontact_baseline_status.clone();
                let baseline = monitor.wrench_window.precontact_baseline.clone();
                let baseline_position = monitor.wrench_window.precontact_baseline_position;
                let baseline_orientation = monitor.wrench_window.precontact_baseline_orientation;
                record_push_wrench_window(&mut audit, monitor.wrench_window);
                if let Some(abort) = monitor.abort {
                    audit.execution = WitnessExecution::WrenchAborted;
                    audit.abort = Some(abort);
                    return Ok(audit);
                }
                if let Some(status) = baseline_status.as_ref() {
                    audit.precontact_baseline_status = Some(status.clone());
                }
                if baseline_status.as_deref() == Some("contact_present") {
                    audit.execution = WitnessExecution::BaselineUnavailable;
                    return Ok(audit);
                }
                if let Some(baseline) = baseline {
                    audit.precontact_baseline = Some(baseline);
                    audit.precontact_baseline_position = baseline_position;
                    audit.precontact_baseline_orientation = baseline_orientation;
                    break;
                }
                settle_remaining -= steps;
            }
            if audit.precontact_baseline.is_none() {
                audit.execution = WitnessExecution::BaselineUnavailable;
                return Ok(audit);
            }
        }
    }
    if is_probe {
        let state = state_payload(worker)?;
        let object = body_position(&state, &request.scene.object_id)?;
        let displacement = sub3(object, request.scene.object_center);
        let planar_displacement = displacement[0].hypot(displacement[1]);
        let along = direction.map(|axis| displacement[0] * axis[0] + displacement[1] * axis[1]);
        let lateral =
            direction.map(|axis| (displacement[0] * axis[1] - displacement[1] * axis[0]).abs());
        if planar_displacement > requested + 0.005
            || along.is_some_and(|value| value > requested + 0.003)
            || lateral.is_some_and(|value| value > 0.006)
            || displacement[2].abs() > 0.01
        {
            audit.execution = WitnessExecution::Diverged;
            return Ok(audit);
        }
    }
    audit.execution = WitnessExecution::Wrote;
    Ok(audit)
}

fn update_wrench_episode_audit(
    episode: &mut BenchmarkEpisode,
    audit: &PushWitnessAudit,
    profile: &SimulationWrenchSensorProfile,
) {
    episode.wrench_max_object_speed_m_s = Some(
        episode
            .wrench_max_object_speed_m_s
            .unwrap_or(0.0)
            .max(audit.motion.max_speed_m_s),
    );
    let Ok(profile_digest) = profile.digest() else {
        episode.wrench_abort_result = Some("profile_digest_invalid".into());
        return;
    };
    episode.wrench_sensor_profile_digest = Some(profile_digest);
    episode.wrench_compensation_digest = Some(profile.limit_policy.compensation_digest.clone());
    episode.wrench_monitor_gravity_reference = audit.monitor_gravity_reference.clone();
    episode.wrench_monitor_gravity_reference_status =
        audit.monitor_gravity_reference_status.clone();
    episode.wrench_monitor_gravity_reference_history = audit.monitor_gravity_references.clone();
    episode.wrench_precontact_baseline_status = audit.precontact_baseline_status.clone();
    episode.wrench_precontact_baseline = audit.precontact_baseline.clone();
    episode.wrench_contact_pairs = audit.wrench_contact_pairs.clone();
    episode.wrench_monitor_gravity_reference_position = audit.monitor_gravity_reference_position;
    episode.wrench_monitor_gravity_reference_orientation =
        audit.monitor_gravity_reference_orientation;
    episode.wrench_sample_count = episode
        .wrench_sample_count
        .saturating_add(audit.samples.len() as u64);
    if !audit.samples.is_empty() {
        let mut peaks = [0.0_f64; 6];
        let mut minimum_margin = f64::INFINITY;
        let mut max_age_s = 0.0_f64;
        let mut max_gap_s = 0.0_f64;
        let mut previous_capture_s = None;
        for (index, sample) in audit.samples.iter().enumerate() {
            max_age_s = max_age_s.max((sample.receive_s - sample.capture_s).max(0.0));
            if let Some(previous) = previous_capture_s {
                max_gap_s = max_gap_s.max(sample.capture_s - previous);
            }
            previous_capture_s = Some(sample.capture_s);
            let age_at_receive = (sample.receive_s - sample.capture_s).max(0.0);
            let sample_position = audit
                .sensor_positions
                .get(index)
                .copied()
                .unwrap_or([0.0; 3]);
            let sample_orientation = audit
                .sensor_orientations
                .get(index)
                .copied()
                .unwrap_or([1.0, 0.0, 0.0, 0.0]);
            let reference = audit
                .sample_gravity_reference_indices
                .get(index)
                .copied()
                .flatten()
                .and_then(|reference_index| audit.monitor_gravity_references.get(reference_index))
                .map(|reference| {
                    (
                        &reference.sample,
                        reference.sensor_position,
                        reference.sensor_orientation,
                    )
                })
                .or_else(|| {
                    audit
                        .monitor_gravity_references
                        .is_empty()
                        .then(|| {
                            audit
                                .monitor_gravity_reference
                                .as_ref()
                                .zip(audit.monitor_gravity_reference_position)
                                .zip(audit.monitor_gravity_reference_orientation)
                                .map(|((sample, position), orientation)| {
                                    (sample, position, orientation)
                                })
                        })
                        .flatten()
                });
            let (compensated, compensated_error) =
                match super::wrench_sensor::gravity_compensated_wrench(
                    sample,
                    sample_position,
                    sample_orientation,
                    profile.bias,
                    &profile.model_id,
                    reference,
                ) {
                    Ok(compensated) => compensated,
                    Err(_) => {
                        episode.wrench_abort_result =
                            Some("monitor_compensation_audit_invalid".into());
                        continue;
                    }
                };
            for (axis, peak) in peaks.iter_mut().enumerate() {
                let upper = compensated[axis].abs()
                    + compensated_error[axis]
                    + profile.limit_policy.max_rise_rate[axis]
                        * (age_at_receive
                            + profile.limit_policy.max_sensing_latency_s
                            + profile.limit_policy.max_abort_latency_s);
                *peak = peak.max(upper);
                minimum_margin = minimum_margin.min(profile.limit_policy.scope.caps[axis] - upper);
            }
        }
        episode.wrench_max_sample_age_s = Some(max_age_s);
        episode.wrench_max_inter_sample_gap_s = Some(max_gap_s);
        episode.wrench_peak_conservative_bounds = Some(peaks);
        episode.wrench_minimum_margin = Some(minimum_margin);
    }
    if let Some(abort) = &audit.abort {
        episode.wrench_abort_axis = match abort.reason {
            WrenchAbortReason::Limit { axis, .. } => Some(axis),
            _ => None,
        };
        episode.wrench_abort_result = Some(if abort.stop_verified {
            "preauthorized_hold_verified".into()
        } else {
            "preauthorized_hold_unverified_fault_latched".into()
        });
        episode.wrench_abort = Some(abort.clone());
    }
}

fn record_wrench_contacts(audit: &mut PushWitnessAudit, state: &Value) {
    let Some(contacts) = state["contacts"].as_array() else {
        return;
    };
    for contact in contacts {
        let body1 = contact["body1"].as_str().unwrap_or("unknown");
        let body2 = contact["body2"].as_str().unwrap_or("unknown");
        let geom1 = contact["geom1_name"].as_str().unwrap_or("unknown_geom");
        let geom2 = contact["geom2_name"].as_str().unwrap_or("unknown_geom");
        let record = format!("{body1}/{geom1}<->{body2}/{geom2}");
        if audit.wrench_contact_pairs.len() < 64 && !audit.wrench_contact_pairs.contains(&record) {
            audit.wrench_contact_pairs.push(record);
        }
    }
}

fn push_probe_measurement(
    manifest: &RobotManifest,
    request: &ReasoningRequest,
    profile: &SimulationWrenchSensorProfile,
    audit: &PushWitnessAudit,
    geometry_digest: &str,
    state: &Value,
) -> Result<PushProbeMeasurement, BenchmarkError> {
    let frame = profile.sensor_frame.as_str();
    let site = manifest.site_records.iter().find(|site| site.name == frame);
    let body_id = site
        .map(|site| site.body.as_str())
        .or_else(|| {
            manifest
                .bodies
                .iter()
                .find(|body| body.name == frame)
                .map(|body| body.name.as_str())
        })
        .ok_or_else(|| {
            BenchmarkError::InvalidTrial(format!(
                "wrench sensor frame {frame} has no mapped tool body"
            ))
        })?;
    let (sensor_position, sensor_quaternion) = if site.is_some() {
        (
            value_vec3(&state["sites"][frame]).ok_or_else(|| {
                BenchmarkError::InvalidTrial(format!("wrench site pose {frame} is missing"))
            })?,
            value_quaternion(&state["site_xquat"][frame]).ok_or_else(|| {
                BenchmarkError::InvalidTrial(format!("wrench site orientation {frame} is missing"))
            })?,
        )
    } else {
        (
            value_vec3(&state["xpos"][frame]).ok_or_else(|| {
                BenchmarkError::InvalidTrial(format!("wrench body pose {frame} is missing"))
            })?,
            value_quaternion(&state["xquat"][frame]).ok_or_else(|| {
                BenchmarkError::InvalidTrial(format!("wrench body orientation {frame} is missing"))
            })?,
        )
    };
    let rotation_to_support = quaternion_rotation(sensor_quaternion);
    let translation_to_support_m = [
        sensor_position[0] - request.scene.support_origin[0],
        sensor_position[1] - request.scene.support_origin[1],
        sensor_position[2] - request.scene.support_origin[2],
    ];
    let start_s = audit.motion.start_s;
    let end_s = audit.motion.end_s.max(start_s + f64::EPSILON);
    if audit.samples.len() != audit.sensor_positions.len()
        || audit.samples.len() != audit.sensor_orientations.len()
    {
        return Err(BenchmarkError::Simulation(
            "wrench_sample_pose_alignment_mismatch".into(),
        ));
    }
    let mut aligned_samples = audit
        .samples
        .iter()
        .zip(&audit.sensor_positions)
        .zip(&audit.sensor_orientations)
        .filter(|((sample, _), _)| {
            sample.capture_s >= start_s
                && sample.capture_s <= end_s
                && audit
                    .precontact_baseline
                    .as_ref()
                    .map_or(true, |baseline| sample.sequence != baseline.sequence)
        })
        .map(|((sample, position), quaternion)| {
            let transform =
                wrench_transform_for_pose(*position, *quaternion, request.scene.support_origin);
            (sample.clone(), transform)
        })
        .collect::<Vec<_>>();
    aligned_samples.sort_by(|left, right| {
        left.0
            .capture_s
            .total_cmp(&right.0.capture_s)
            .then_with(|| left.0.sequence.cmp(&right.0.sequence))
    });
    let samples = aligned_samples
        .iter()
        .map(|(sample, _)| sample.clone())
        .collect::<Vec<_>>();
    let sample_sensor_transforms = aligned_samples
        .iter()
        .map(|(_, transform)| transform.clone())
        .collect::<Vec<_>>();
    let evidence_ready_s = samples
        .iter()
        .map(|sample| sample.receive_s)
        .fold(end_s, f64::max);
    let mass_kg = request
        .mass_kg
        .map(|mass| [mass, mass])
        .unwrap_or([0.0, 0.0]);
    let mut compensation = profile.compensation();
    compensation.gravity_wrench = [0.0; 6];
    compensation.residual_error = audit
        .precontact_baseline
        .as_ref()
        .map(|baseline| baseline.error_bounds)
        .unwrap_or(profile.error_bounds);
    compensation.digest = compensation.digest();
    let sample_gravity_wrenches = audit
        .precontact_baseline
        .as_ref()
        .map(|baseline| {
            let baseline_position = audit.precontact_baseline_position.ok_or_else(|| {
                BenchmarkError::Simulation("wrench_baseline_position_missing".into())
            })?;
            let baseline_orientation = audit.precontact_baseline_orientation.ok_or_else(|| {
                BenchmarkError::Simulation("wrench_baseline_orientation_missing".into())
            })?;
            let baseline_transform = wrench_transform_for_pose(
                baseline_position,
                baseline_orientation,
                request.scene.support_origin,
            );
            let unbiased_baseline =
                std::array::from_fn(|axis| baseline.wrench[axis] - profile.bias[axis]);
            let support_baseline = wrench_to_support_axes(&unbiased_baseline, &baseline_transform);
            Ok::<Vec<[f64; 6]>, BenchmarkError>(
                sample_sensor_transforms
                    .iter()
                    .map(|transform| wrench_from_support_axes(&support_baseline, transform))
                    .collect(),
            )
        })
        .transpose()?
        .unwrap_or_default();
    Ok(PushProbeMeasurement {
        samples,
        precontact_baseline: audit.precontact_baseline.clone(),
        sample_gravity_wrenches,
        sample_sensor_transforms,
        sensor_to_support: WrenchTransform {
            rotation_to_support,
            translation_to_support_m,
        },
        compensation,
        mass_kg,
        support_normal_world: request.scene.support_normal,
        motion: ProbeMotionWindow {
            start_s,
            end_s,
            displacement_m: sub3(audit.motion.end_position, audit.motion.start_position),
            max_speed_m_s: audit.motion.max_speed_m_s,
            max_quasi_static_speed_m_s: 0.02,
            pose_resolution_m: 0.0001,
            first_motion_s: audit.motion.first_motion_s,
        },
        prior_support_friction: request
            .belief
            .entry(PhysicalParameter::SupportFriction)
            .and_then(|entry| entry.empirical_interval)
            .unwrap_or([0.05, 0.8]),
        current_observation_epoch: request.scene.observation_epoch.clone(),
        current_collision_model_digest: geometry_digest.into(),
        now_s: evidence_ready_s,
        max_sample_age_s: profile.limit_policy.max_sample_age_s,
        max_inter_sample_gap_s: profile.limit_policy.max_inter_sample_gap_s,
        max_sample_motion_skew_s: profile.limit_policy.max_inter_sample_gap_s,
        max_wrench_rise_rate: profile.limit_policy.max_rise_rate,
        intended_contact: IntendedContactEvidence {
            observation_epoch: request.scene.observation_epoch.clone(),
            collision_model_digest: geometry_digest.into(),
            tool_body_id: body_id.into(),
            target_body_id: request.scene.object_id.clone(),
            // This pair is derived from the frozen collision-checked push
            // witness; raw MuJoCo contact identities never enter inference.
            plausible_contact_sets: vec![vec![body_id.into(), request.scene.object_id.clone()]],
        },
        saturated: false,
        origin: WrenchSampleOrigin::Simulation,
    })
}

fn wrench_transform_for_pose(
    sensor_position: [f64; 3],
    sensor_quaternion: [f64; 4],
    support_origin: [f64; 3],
) -> WrenchTransform {
    WrenchTransform {
        rotation_to_support: quaternion_rotation(sensor_quaternion),
        translation_to_support_m: [
            sensor_position[0] - support_origin[0],
            sensor_position[1] - support_origin[1],
            sensor_position[2] - support_origin[2],
        ],
    }
}

fn wrench_to_support_axes(wrench: &[f64; 6], transform: &WrenchTransform) -> [f64; 6] {
    let mut output = [0.0; 6];
    for axis in 0..3 {
        output[axis] = (0..3)
            .map(|source| transform.rotation_to_support[axis][source] * wrench[source])
            .sum();
        output[axis + 3] = (0..3)
            .map(|source| transform.rotation_to_support[axis][source] * wrench[source + 3])
            .sum();
    }
    let moment_arm = cross3(
        transform.translation_to_support_m,
        [output[0], output[1], output[2]],
    );
    for axis in 0..3 {
        output[axis + 3] += moment_arm[axis];
    }
    output
}

fn wrench_from_support_axes(wrench: &[f64; 6], transform: &WrenchTransform) -> [f64; 6] {
    let mut output = [0.0; 6];
    let moment_arm = cross3(
        transform.translation_to_support_m,
        [wrench[0], wrench[1], wrench[2]],
    );
    let torque_at_sensor = [
        wrench[3] - moment_arm[0],
        wrench[4] - moment_arm[1],
        wrench[5] - moment_arm[2],
    ];
    for axis in 0..3 {
        output[axis] = (0..3)
            .map(|source| transform.rotation_to_support[source][axis] * wrench[source])
            .sum();
        output[axis + 3] = (0..3)
            .map(|source| transform.rotation_to_support[source][axis] * torque_at_sensor[source])
            .sum();
    }
    output
}

fn cross3(left: [f64; 3], right: [f64; 3]) -> [f64; 3] {
    [
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    ]
}

fn inference_compensation_digest(measurement: &PushProbeMeasurement) -> String {
    use sha2::{Digest, Sha256};
    let mut digest = Sha256::new();
    digest.update(b"realityos.simulated_probe_compensation/1\0");
    digest.update(measurement.compensation.digest.as_bytes());
    if let Some(baseline) = &measurement.precontact_baseline {
        digest.update(baseline.digest.as_bytes());
    }
    for wrench in &measurement.sample_gravity_wrenches {
        for value in wrench {
            digest.update(value.to_bits().to_le_bytes());
        }
    }
    hex::encode(digest.finalize())
}

fn value_quaternion(value: &Value) -> Option<[f64; 4]> {
    let values = value.as_array()?;
    if values.len() != 4 {
        return None;
    }
    Some([
        values[0].as_f64()?,
        values[1].as_f64()?,
        values[2].as_f64()?,
        values[3].as_f64()?,
    ])
}

fn quaternion_rotation([w, x, y, z]: [f64; 4]) -> [[f64; 3]; 3] {
    [
        [
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - z * w),
            2.0 * (x * z + y * w),
        ],
        [
            2.0 * (x * y + z * w),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - x * w),
        ],
        [
            2.0 * (x * z - y * w),
            2.0 * (y * z + x * w),
            1.0 - 2.0 * (x * x + y * y),
        ],
    ]
}

fn decode_push_witness(
    contents: &str,
    joint_count: usize,
    is_probe: bool,
) -> Option<(Vec<f64>, usize, Vec<Vec<f64>>)> {
    let q0 = strict_witness_joint_line(contents, "q0_bits=")?;
    let transit = strict_witness_joint_line(contents, "q_transit_bits=")?;
    let transit_count_payload = contents
        .lines()
        .find_map(|line| line.strip_prefix("transit_state_count="));
    let has_transit_count = transit_count_payload.is_some();
    let transit_count = match transit_count_payload {
        Some(value) => value.parse::<usize>().ok()?,
        None => 1,
    };
    let approach_count = match contents
        .lines()
        .find_map(|line| line.strip_prefix("approach_state_count="))
    {
        Some(value) => value.parse::<usize>().ok()?,
        None => 0,
    };
    let contact = strict_witness_joint_line(contents, "q_contact_bits=")?;
    let end = strict_witness_joint_line(contents, "q_end_bits=")?;
    let stroke_count = contents
        .lines()
        .find_map(|line| line.strip_prefix("stroke_state_count="))?
        .parse::<usize>()
        .ok()?;
    if stroke_count == 0
        || stroke_count > 64
        || transit_count == 0
        || transit_count > 64
        || approach_count > 64
    {
        return None;
    }
    let transit_states = if has_transit_count {
        (0..transit_count)
            .map(|index| strict_witness_joint_line(contents, &format!("transit_state_{index}=")))
            .collect::<Option<Vec<_>>>()?
    } else {
        vec![transit.clone()]
    };
    let approach_states = (0..approach_count)
        .map(|index| strict_witness_joint_line(contents, &format!("approach_state_{index}=")))
        .collect::<Option<Vec<_>>>()?;
    let stroke_states = (0..stroke_count)
        .map(|index| strict_witness_joint_line(contents, &format!("stroke_state_{index}=")))
        .collect::<Option<Vec<_>>>()?;
    let retract = strict_witness_joint_line(contents, "q_retract_bits=");
    let valid = |values: &[f64]| {
        values.len() == joint_count && values.iter().all(|value| value.is_finite())
    };
    if joint_count == 0
        || !valid(&q0)
        || !valid(&transit)
        || transit_states.iter().any(|state| !valid(state))
        || approach_states.iter().any(|state| !valid(state))
        || !valid(&contact)
        || !valid(&end)
        || stroke_states.iter().any(|state| !valid(state))
        || (is_probe && !retract.as_deref().is_some_and(valid))
    {
        return None;
    }
    if !transit_states
        .last()?
        .iter()
        .zip(&transit)
        .all(|(state, transit)| state.to_bits() == transit.to_bits())
    {
        return None;
    }
    let final_stroke_state = stroke_states.last()?;
    if !final_stroke_state
        .iter()
        .zip(&end)
        .all(|(state, end)| state.to_bits() == end.to_bits())
    {
        return None;
    }
    let mut targets = transit_states;
    targets.extend(approach_states);
    targets.push(contact);
    targets.extend(stroke_states);
    if is_probe {
        targets.push(retract?);
    }
    Some((q0, transit_count, targets))
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
    lift_contact_audits: Vec<GraspContactAudit>,
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
            lift_contact_audits: Vec::new(),
        });
    };
    let rejection = || GraspWitnessExecution {
        execution: WitnessExecution::WroteNothing,
        hold_start: None,
        phase_object_positions: Vec::new(),
        lift_contact_audits: Vec::new(),
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
            lift_contact_audits: Vec::new(),
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
                    lift_contact_audits: Vec::new(),
                });
            }
        }
        let measured = read_named_joints(worker, manifest, &request.scene.joint_names)?;
        if !vectors_near(&measured, arm, 0.08) {
            return Ok(GraspWitnessExecution {
                execution: WitnessExecution::Diverged,
                hold_start,
                phase_object_positions,
                lift_contact_audits: Vec::new(),
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
                    lift_contact_audits: Vec::new(),
                });
            }
            let settled = read_named_joints(worker, manifest, &request.scene.joint_names)?;
            if !vectors_near(&settled, arm, 0.08) {
                return Ok(GraspWitnessExecution {
                    execution: WitnessExecution::Diverged,
                    hold_start,
                    phase_object_positions,
                    lift_contact_audits: Vec::new(),
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

    let mut lift_contact_audits = Vec::new();
    for pair in lift_states.windows(2) {
        let (start_arm, start_fingers) = &pair[0];
        let (end_arm, end_fingers) = &pair[1];
        if !exact_finite_vec(start_fingers, end_fingers) {
            return Ok(rejection());
        }
        // Execute the same joint-space samples used by the collision proof,
        // observing finger contact, force, and object motion at each waypoint.
        const LIFT_STEPS_PER_WAYPOINT: u32 = 50;
        if !exact_finite_vec(start_arm, &contact) || end_arm.len() != start_arm.len() {
            return Ok(rejection());
        }
        let sample_count =
            realityos_semantics::maneuver_witness::interpolation_count(start_arm, end_arm);
        let hold_truth = hold_start.as_ref().ok_or_else(|| {
            BenchmarkError::Simulation("post_close_measurement_missing_before_lift".into())
        })?;
        let start_object =
            truth_position(hold_truth, &request.scene.object_id).ok_or_else(|| {
                BenchmarkError::Simulation("post_close_object_pose_unavailable".into())
            })?;
        let start_ee = truth_position(hold_truth, &request.scene.ee).ok_or_else(|| {
            BenchmarkError::Simulation("post_close_end_effector_pose_unavailable".into())
        })?;
        let start_relative = sub3(start_object, start_ee);
        for waypoint in 1..sample_count {
            let fraction = waypoint as f64 / (sample_count - 1) as f64;
            let target = start_arm
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
                &target,
                end_fingers,
                LIFT_STEPS_PER_WAYPOINT,
            )? {
                GraspControlStatus::Written => {}
                GraspControlStatus::Rejected => return Ok(rejection()),
                GraspControlStatus::AuthorityExpired => {
                    return Ok(GraspWitnessExecution {
                        execution: WitnessExecution::AuthorityExpired,
                        hold_start,
                        phase_object_positions,
                        lift_contact_audits,
                    });
                }
            }
            let measured = read_named_joints(worker, manifest, &request.scene.joint_names)?;
            if !vectors_near(&measured, &target, 0.08) {
                return Ok(GraspWitnessExecution {
                    execution: WitnessExecution::Diverged,
                    hold_start,
                    phase_object_positions,
                    lift_contact_audits,
                });
            }
            let state = state_payload(worker)?;
            let object_position = body_position(&state, &request.scene.object_id)?;
            phase_object_positions.push(object_position);
            let truth = VerifierTruth::from_mujoco_state(&state);
            let audits = grasp_contact_audits(
                &format!("lift_waypoint_{waypoint}"),
                &truth,
                &request.scene.object_id,
                resource,
            );
            let force_safe = audits.len() == resource.finger_bodies.len()
                && audits.iter().all(|audit| {
                    audit.force_available
                        && audit.normal_force_n.is_some_and(|force| {
                            force.is_finite() && force <= *per_finger_force_ceiling_n + 1e-6
                        })
                });
            lift_contact_audits.extend(audits);
            let object_hits_obstacle = request.scene.obstacles.iter().any(|obstacle| {
                truth.contacts.iter().any(|contact| {
                    contact_pair_has(contact, &request.scene.object_id, &obstacle.name)
                })
            });
            let relative_pose_error = truth_position(&truth, &request.scene.ee)
                .map(|ee| distance3(sub3(object_position, ee), start_relative))
                .unwrap_or(f64::INFINITY);
            if !bilateral_finger_contact(&truth, &request.scene.object_id, resource)
                || !force_safe
                || object_hits_obstacle
                || !relative_pose_error.is_finite()
                || relative_pose_error > 0.02
            {
                return Ok(GraspWitnessExecution {
                    execution: WitnessExecution::Diverged,
                    hold_start,
                    phase_object_positions,
                    lift_contact_audits,
                });
            }
        }
    }
    Ok(GraspWitnessExecution {
        execution: WitnessExecution::Wrote,
        hold_start,
        phase_object_positions,
        lift_contact_audits,
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
    let runtime_now = 10.0 + simulator_now(worker)?;
    Ok(authorization_covers_at(authorized, now_s.max(runtime_now)))
}

fn authorization_covers_at(authorized: &AuthorizedReasonerAction, now: f64) -> bool {
    let frozen = authorized.action.frozen();
    let Some(authorization) = frozen.execution_authorization.as_ref() else {
        return false;
    };
    frozen.authority_granted
        && authorization.covers(frozen, now)
        && authorization.grant_id == authorized.grant.command_id
        && authorized
            .grant
            .scope
            .action_envelope
            .exactly_matches(&frozen.action_envelope)
        && grant_covers(&authorized.grant, &authorized.grant.scope, now).is_ok()
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

fn probe_control_ramp_steps(
    start_ctrl: &[f64],
    target_ctrl: &[f64],
    actuator_map: &BTreeMap<String, usize>,
    joint_names: &[String],
    timestep_s: f64,
) -> u32 {
    if !timestep_s.is_finite() || timestep_s <= 0.0 || start_ctrl.len() != target_ctrl.len() {
        return MAX_PROBE_CONTROL_RAMP_STEPS;
    }
    let mut maximum_delta_rad = 0.0_f64;
    for joint in joint_names {
        let Some(index) = actuator_map.get(joint).copied() else {
            return MAX_PROBE_CONTROL_RAMP_STEPS;
        };
        let (Some(start), Some(target)) = (start_ctrl.get(index), target_ctrl.get(index)) else {
            return MAX_PROBE_CONTROL_RAMP_STEPS;
        };
        let delta = (target - start).abs();
        if !delta.is_finite() {
            return MAX_PROBE_CONTROL_RAMP_STEPS;
        }
        maximum_delta_rad = maximum_delta_rad.max(delta);
    }
    let requested_steps = (maximum_delta_rad / PROBE_JOINT_RAMP_RATE_RAD_S / timestep_s).ceil();
    if !requested_steps.is_finite() {
        return MAX_PROBE_CONTROL_RAMP_STEPS;
    }
    (requested_steps as u32)
        .max(MIN_PROBE_CONTROL_RAMP_STEPS)
        .min(MAX_PROBE_CONTROL_RAMP_STEPS)
}

pub(super) fn declared_ee_observation_key(bundle: &RobotBundle, ee_name: &str) -> Option<String> {
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

pub(super) fn build_push_reasoning_request(
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
        linear_wrench_limit: None,
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
        linear_wrench_limit: None,
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
        Some(NegKind::UnexpectedContact) => (
            SafetyFaultKind::UnexpectedContact,
            "mujoco_fault_injection_adapter.unexpected_contact",
        ),
        Some(NegKind::Slip) => (
            SafetyFaultKind::SlipDetected,
            "mujoco_fault_injection_adapter.slip_detected",
        ),
        _ => return Vec::new(),
    };
    vec![SafetyFaultObservation {
        kind,
        asserted: Provenanced::simulator_derived(true, source, now_s),
    }]
}

pub(super) fn uncertain_push_belief() -> PhysicalParameterBelief {
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

pub(super) fn object_half_extents(object: &Value) -> Result<[f64; 3], BenchmarkError> {
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

pub(super) fn declared_ee_radius(
    model: &realityos_semantics::embodiment::EmbodimentModel,
    ee_name: &str,
) -> Option<f64> {
    use realityos_semantics::geometry::{CollisionRole, PrimitiveShape};
    let ee = model.end_effectors.iter().find(|ee| ee.name == ee_name)?;
    let frame = model.frames.iter().find(|frame| frame.name == ee.frame)?;
    let frame_pose = frame.pose()?;
    let body_from_ee = frame_pose.inverse();
    let mut found_collision_geom = false;
    let radius = model
        .collision_geoms
        .iter()
        .filter(|geom| geom.owner_body == frame.parent_body)
        .try_fold(0.0_f64, |maximum, geom| {
            match geom.collision_role {
                CollisionRole::Visual => return Some(maximum),
                CollisionRole::Collision => {}
                CollisionRole::Unknown => return None,
            }
            found_collision_geom = true;
            let ee_from_geom = body_from_ee.compose(geom.local_pose);
            let shape_radius = match &geom.shape {
                PrimitiveShape::Sphere { radius } => radius.abs(),
                PrimitiveShape::Capsule {
                    radius,
                    half_length,
                } => radius.abs() + half_length.abs(),
                PrimitiveShape::Cylinder {
                    radius,
                    half_length,
                } => radius.abs().hypot(half_length.abs()),
                PrimitiveShape::Box { half_extents } => half_extents
                    .iter()
                    .map(|value| value.abs().powi(2))
                    .sum::<f64>()
                    .sqrt(),
                PrimitiveShape::ConvexMesh { vertices } => vertices
                    .iter()
                    .map(|vertex| vertex[0].hypot(vertex[1]).hypot(vertex[2]))
                    .reduce(f64::max)?,
                PrimitiveShape::Plane { .. } | PrimitiveShape::Unsupported { .. } => return None,
            };
            let bound = ee_from_geom.xyz[0]
                .hypot(ee_from_geom.xyz[1])
                .hypot(ee_from_geom.xyz[2])
                + shape_radius;
            bound.is_finite().then_some(maximum.max(bound))
        })?;
    found_collision_geom.then_some(radius)
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

fn action_direction(action_key: &str) -> Option<[f64; 2]> {
    match action_key.rsplit(':').next()? {
        "+x" => Some([1.0, 0.0]),
        "-x" => Some([-1.0, 0.0]),
        "+y" => Some([0.0, 1.0]),
        "-y" => Some([0.0, -1.0]),
        _ => None,
    }
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
    use super::{
        declared_ee_radius, decode_push_witness, injected_safety_fault_alerts,
        probe_control_ramp_steps,
    };
    use crate::manipulation_scenarios::{push_scenario, NegKind};
    use realityos_semantics::embodiment::{EmbodimentModel, EndEffector, FrameKind, ModelFrame};
    use realityos_semantics::geometry::{
        CollisionRole, PrimitiveShape, RigidGeometry, SemanticRole,
    };
    use realityos_semantics::physical_reasoner::SafetyFaultKind;
    use realityos_semantics::provenance::Provenance;
    use realityos_semantics::provenance::Provenanced;
    use realityos_semantics::transform::Se3;

    fn line(name: &str, values: &[f64]) -> String {
        let bits = values
            .iter()
            .map(|value| value.to_bits().to_string())
            .collect::<Vec<_>>()
            .join(",");
        format!("{name}={bits}\n")
    }

    #[test]
    fn declared_ee_radius_is_conservative_and_fails_closed() {
        let mut model = EmbodimentModel::new("test", "bundle", "model", "epoch", "version");
        model.end_effectors.push(EndEffector {
            name: "tool".into(),
            frame: "tool".into(),
            joint_chain: Vec::new(),
        });
        model.frames.push(ModelFrame {
            name: "tool".into(),
            kind: FrameKind::Ee,
            parent_body: "wrist".into(),
            translation: Provenanced::declared([0.02, 0.0, 0.0], "test", 0.0),
            rotation: Provenanced::declared([1.0, 0.0, 0.0, 0.0], "test", 0.0),
        });
        model.collision_geoms.push(RigidGeometry::declared(
            "offset_tool_collision",
            "wrist",
            Se3::translation([0.0, 0.03, 0.0]).expect("finite test pose"),
            PrimitiveShape::Sphere { radius: 0.01 },
            CollisionRole::Collision,
            SemanticRole::Tool,
            "test",
        ));

        let radius = declared_ee_radius(&model, "tool").expect("supported collision geometry");
        assert!((radius - (0.02_f64.hypot(0.03) + 0.01)).abs() < 1e-12);

        model.collision_geoms.push(RigidGeometry::declared(
            "unknown_tool_collision",
            "wrist",
            Se3::identity(),
            PrimitiveShape::Unsupported {
                kind: "unknown_mesh".into(),
            },
            CollisionRole::Collision,
            SemanticRole::Tool,
            "test",
        ));
        assert!(declared_ee_radius(&model, "tool").is_none());

        model.collision_geoms.pop();
        model.collision_geoms.push(RigidGeometry::declared(
            "unknown_role_tool_geom",
            "wrist",
            Se3::identity(),
            PrimitiveShape::Sphere { radius: 0.01 },
            CollisionRole::Unknown,
            SemanticRole::Tool,
            "test",
        ));
        assert!(declared_ee_radius(&model, "tool").is_none());
    }

    #[test]
    fn probe_ramp_duration_scales_with_joint_segment_and_keeps_existing_cap() {
        let joints = vec!["joint1".to_string(), "joint2".to_string()];
        let actuator_map =
            std::collections::BTreeMap::from([(joints[0].clone(), 0), (joints[1].clone(), 1)]);

        assert_eq!(
            probe_control_ramp_steps(&[0.0, 0.0], &[0.02, 0.20], &actuator_map, &joints, 0.002,),
            1_000,
            "a 0.2 rad segment should use the registered 0.1 rad/s ramp"
        );
        assert_eq!(
            probe_control_ramp_steps(&[0.0, 0.0], &[0.02, 2.0], &actuator_map, &joints, 0.002,),
            6_000,
            "long segments keep the existing 6,000-step ceiling"
        );
        assert_eq!(
            probe_control_ramp_steps(&[0.0, 0.0], &[0.0, 0.0], &actuator_map, &joints, 0.002,),
            100,
            "stationary targets retain a minimum monitored control window"
        );
    }

    fn witness(include_retract: bool) -> String {
        let mut contents = String::new();
        contents.push_str(&line("q0_bits", &[0.1, 0.2]));
        contents.push_str(&line("q_transit_bits", &[0.2, 0.3]));
        contents.push_str("transit_state_count=1\n");
        contents.push_str(&line("transit_state_0", &[0.2, 0.3]));
        contents.push_str(&line("q_contact_bits", &[0.3, 0.4]));
        contents.push_str(&line("q_end_bits", &[0.4, 0.5]));
        contents.push_str("stroke_state_count=2\n");
        contents.push_str(&line("stroke_state_0", &[0.35, 0.45]));
        contents.push_str(&line("stroke_state_1", &[0.4, 0.5]));
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
            .expect("ordinary push witnesses have checked stroke waypoints");
        assert_eq!(decoded.1, 1);
        assert_eq!(decoded.2.len(), 4);
        assert_eq!(decoded.2[2], vec![0.35, 0.45]);
        assert_eq!(decoded.2[3], vec![0.4, 0.5]);
    }

    #[test]
    fn push_witness_requires_transit_path_to_end_at_certified_standoff() {
        let multi_waypoint = witness(false)
            .replace("transit_state_count=1", "transit_state_count=2")
            .replace(
                &line("transit_state_0", &[0.2, 0.3]),
                &line("transit_state_0", &[0.15, 0.25]),
            )
            + &line("transit_state_1", &[0.2, 0.3]);
        let decoded = decode_push_witness(&multi_waypoint, 2, false)
            .expect("every certified transit waypoint is executable in order");
        assert_eq!(decoded.1, 2);
        assert_eq!(decoded.2[0], vec![0.15, 0.25]);
        assert_eq!(decoded.2[1], vec![0.2, 0.3]);
        assert_eq!(decoded.2[2], vec![0.3, 0.4]);

        let mut malformed = witness(false);
        malformed.push_str(&line("transit_state_1", &[0.25, 0.35]));
        malformed = malformed.replace("transit_state_count=1", "transit_state_count=2");
        assert!(decode_push_witness(&malformed, 2, false).is_none());
    }

    #[test]
    fn push_witness_executes_certified_approach_waypoints_before_contact() {
        let contents = witness(false).replace(
            &line("q_contact_bits", &[0.3, 0.4]),
            &format!(
                "approach_state_count=2\n{}{}{}",
                line("approach_state_0", &[0.24, 0.34]),
                line("approach_state_1", &[0.27, 0.37]),
                line("q_contact_bits", &[0.3, 0.4]),
            ),
        );
        let decoded = decode_push_witness(&contents, 2, false)
            .expect("all force-monitored approach states are part of the executed witness");
        assert_eq!(decoded.1, 1, "approach waypoints follow the transit phase");
        assert_eq!(decoded.2.len(), 6);
        assert_eq!(decoded.2[0], vec![0.2, 0.3]);
        assert_eq!(decoded.2[1], vec![0.24, 0.34]);
        assert_eq!(decoded.2[2], vec![0.27, 0.37]);
        assert_eq!(decoded.2[3], vec![0.3, 0.4]);
    }

    #[test]
    fn push_witness_rejects_end_state_not_matching_last_checked_waypoint() {
        let malformed = witness(false).replace(
            &line("stroke_state_1", &[0.4, 0.5]),
            &line("stroke_state_1", &[0.39, 0.5]),
        );
        assert!(decode_push_witness(&malformed, 2, false).is_none());
    }

    #[test]
    fn push_witness_rejects_invalid_stroke_waypoint_count() {
        let malformed = witness(false).replace("stroke_state_count=2", "stroke_state_count=65");
        assert!(decode_push_witness(&malformed, 2, false).is_none());
    }

    #[test]
    fn injected_contact_and_slip_faults_are_typed_simulator_observations() {
        for (index, expected_neg, expected_kind, expected_source) in [
            (
                3,
                NegKind::UnexpectedContact,
                SafetyFaultKind::UnexpectedContact,
                "mujoco_fault_injection_adapter.unexpected_contact",
            ),
            (
                4,
                NegKind::Slip,
                SafetyFaultKind::SlipDetected,
                "mujoco_fault_injection_adapter.slip_detected",
            ),
        ] {
            let scenario = push_scenario(42, false, index);
            assert_eq!(scenario.neg, Some(expected_neg));
            let alerts = injected_safety_fault_alerts(&scenario, 1.0);
            assert_eq!(alerts.len(), 1);
            assert_eq!(alerts[0].kind, expected_kind);
            assert_eq!(alerts[0].asserted.source, expected_source);
            assert_eq!(alerts[0].asserted.provenance, Provenance::SimulatorDerived);
            assert_eq!(alerts[0].asserted.value, Some(true));
        }
    }
}
