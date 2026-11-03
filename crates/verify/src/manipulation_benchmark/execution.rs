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

const OBJECT_MOTION_RESOLUTION_M: f64 = 0.0001;
// Do not interpret small contact-compliance drift as support friction onset.
// The development MuJoCo fixture exhibited ~0.24 mm of sub-threshold motion.
const PROBE_SLIP_DISPLACEMENT_M: f64 = 0.001;
const MIN_PROBE_CONTACT_FORCE_N: f64 = 0.5;
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
    infer_support_friction, FrictionInferenceResult, IntendedContactEvidence, ProbeContactPair,
    ProbeMotionWindow, ProbeObjectPoseSample, ProbeObjectPoseTrace, PushProbeMeasurement,
    WrenchTransform,
};
use realityos_semantics::resource::{
    ControlledResource, QualificationStatus, ResourceKind, ResourceTopology,
};
use realityos_semantics::sensor::WrenchSampleOrigin;
use realityos_session::grant_covers;
use serde::{Deserialize, Serialize};
use serde_json::Value;
use sha2::{Digest, Sha256};
use std::collections::BTreeMap;
use std::fs::{self, OpenOptions};
use std::io::{BufWriter, Write};
use std::path::Path;
use std::time::Instant;

const MU_REQUIRED: f64 = 0.3;
const MAX_ACTIONS: usize = 6;
const CONTROL_STEPS_PER_SEGMENT: u32 = 500;
const MAX_JOINT_CONTROL_RAMP_STEPS: u32 = 6_000;
const MIN_JOINT_CONTROL_RAMP_STEPS: u32 = 100;
/// All authorized push trajectories use bounded setpoint ramps. Transit stays
/// slow enough to limit acceleration-driven sensor loads.
const PUSH_TRANSIT_JOINT_RAMP_RATE_RAD_S: f64 = 0.04;
/// Contact segments move more slowly so the wrench monitor observes gradual
/// loading before the action approaches a force limit.
const PUSH_CONTACT_JOINT_RAMP_RATE_RAD_S: f64 = 0.005;
const PROBE_TRANSIT_TRACKING_SETTLE_STEPS: u32 = 2_000;
const PROBE_PRECONTACT_SETTLE_STEPS: u32 = 1_000;
const PROBE_CONTROL_UPDATE_STEPS: u32 = 100;
const CONTROL_AUTHORITY_CHECK_STEPS: u32 = 10;
const GRASP_CLOSE_SETTLE_STEPS: u32 = 200;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TrialOutcome {
    Success,
    ValidRefusal,
    UnexpectedRefusal,
    FalseAcceptance,
    /// Stratum label said refuse, but the scenario carried no refusal cause
    /// and the authorized action was only the missing-grant probe.
    EvaluationLabelMismatch,
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
    EvaluationLabelMismatch,
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
    #[serde(default)]
    pub sample_time_s: Option<f64>,
    pub finger_body: String,
    pub normal_force_n: Option<f64>,
    pub tangential_force_n: Option<f64>,
    pub object_vertical_force_n: Option<f64>,
    pub contact_dimensions: Vec<u8>,
    pub contact_friction: Vec<Vec<f64>>,
    pub force_available: bool,
    #[serde(default)]
    pub object_com_world_m: Option<[f64; 3]>,
    #[serde(default)]
    pub ctrl: Vec<f64>,
    #[serde(default)]
    pub actuator_force: Vec<f64>,
    #[serde(default)]
    pub contact_wrenches: Vec<GraspContactWrench>,
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct GraspContactWrench {
    pub body1: String,
    pub body2: String,
    pub position_world_m: Option<[f64; 3]>,
    pub force_local: Option<[f64; 3]>,
    pub torque_local: Option<[f64; 3]>,
    pub contact_frame: Option<[f64; 9]>,
    pub normal_force_n: Option<f64>,
    pub tangential_force_n: Option<f64>,
    pub contact_dimension: Option<u8>,
    pub friction: Option<Vec<f64>>,
    pub force_available: bool,
    pub force_world_on_object_n: Option<[f64; 3]>,
    pub torque_world_on_object_nm: Option<[f64; 3]>,
    pub moment_world_about_object_com_nm: Option<[f64; 3]>,
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
    /// Raw initial replay observation and its recomputable digest. Legacy episodes
    /// deserialize without this optional field but cannot satisfy preflight binding.
    #[serde(default)]
    pub initial_observation_evidence: Option<ReplayInitialObservationEvidenceV1>,
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
    /// True when any post-run compensation audit could not validate its inputs.
    #[serde(default)]
    pub wrench_compensation_audit_invalid: bool,
    #[serde(default)]
    pub wrench_compensation_audit_invalid_samples: u64,
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

/// Complete, opt-in audit data for probe measurements. This is written to a
/// separate file so the default benchmark episode remains compact.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ProbeMeasurementSidecarV1 {
    pub format: String,
    pub schema_version: u32,
    pub trial_id: String,
    pub trial_digest: String,
    pub paired_trial_key: Option<String>,
    pub measurements: Vec<ProbeMeasurementRecordV1>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ProbeMeasurementRecordV1 {
    pub action_key: String,
    pub measurement_digest: String,
    pub prior_support_friction_interval: [f64; 2],
    pub inference: FrictionInferenceResult,
    pub posterior_support_friction_interval: Option<[f64; 2]>,
    pub measurement: PushProbeMeasurement,
}

/// Replay-side record for independently checking which observation preceded
/// the first request and at what simulator time it was captured.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ReplayInitialObservationEvidenceV1 {
    pub format: String,
    pub schema_version: u32,
    /// JSON text; the digest is computed over these exact UTF-8 bytes.
    pub observation_json: String,
    pub observation_sha256: String,
    pub simulator_time_s: f64,
}

impl ReplayInitialObservationEvidenceV1 {
    fn capture(observation: &Value) -> Result<Self, BenchmarkError> {
        let simulator_time_s = observation
            .get("time")
            .and_then(Value::as_f64)
            .filter(|time| time.is_finite() && *time >= 0.0)
            .ok_or_else(|| {
                BenchmarkError::InvalidTrial(
                    "initial replay observation is missing a valid simulator time".into(),
                )
            })?;
        let observation_json = serde_json::to_string(observation)?;
        Ok(Self {
            format: "realityos.replay_initial_observation_evidence".into(),
            schema_version: 1,
            observation_sha256: sha256_hex(observation_json.as_bytes()),
            observation_json,
            simulator_time_s,
        })
    }

    fn parsed_observation(&self) -> Result<Value, String> {
        serde_json::from_str(&self.observation_json)
            .map_err(|error| format!("replay initial-observation JSON is invalid: {error}"))
    }

    /// Recompute the stored-byte digest and verify its simulator-time field.
    pub fn verify(&self) -> Result<(), String> {
        if self.format != "realityos.replay_initial_observation_evidence"
            || self.schema_version != 1
        {
            return Err("invalid replay initial-observation evidence identity".into());
        }
        let observation = self.parsed_observation()?;
        if !observation.is_object() {
            return Err("replay initial observation is not an object".into());
        }
        if self.observation_sha256 != sha256_hex(self.observation_json.as_bytes()) {
            return Err("replay initial-observation digest mismatch".into());
        }
        let observation_time_s = observation
            .get("time")
            .and_then(Value::as_f64)
            .filter(|time| time.is_finite() && *time >= 0.0)
            .ok_or_else(|| "replay initial observation has no valid simulator time".to_owned())?;
        if !self.simulator_time_s.is_finite()
            || self.simulator_time_s < 0.0
            || self.simulator_time_s.to_bits() != observation_time_s.to_bits()
        {
            return Err("replay initial-observation simulator time mismatch".into());
        }
        Ok(())
    }
}

/// Evidence for the first action selected by the normal benchmark runtime,
/// captured before any simulator control write or physics step.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FirstActionPreflightV1 {
    pub format: String,
    pub schema_version: u32,
    pub evidence_status: String,
    pub trial_id: String,
    pub trial_digest: String,
    pub paired_trial_key: Option<String>,
    pub initial_observation: Value,
    pub initial_observation_sha256: String,
    pub reasoning_request_audit: String,
    pub decision_audit: String,
    pub pregrant_action_source: String,
    pub pregrant_action_key: Option<String>,
    pub authorized_action_key: Option<String>,
    pub authorized_grant_id: Option<String>,
    pub authorized_witness_digest: Option<String>,
    pub authorized_observation_epoch: Option<String>,
    pub refusal_code: Option<String>,
    pub control_writes_before: u64,
    pub control_writes_after: u64,
    pub simulation_time_before_s: f64,
    pub simulation_time_after_s: f64,
    pub simulation_timestep_s: f64,
    pub simulator_steps: u64,
}

impl FirstActionPreflightV1 {
    fn new(trial: &ResolvedBenchmarkTrial) -> Result<Self, BenchmarkError> {
        Ok(Self {
            format: "realityos.first_action_preflight".into(),
            schema_version: 1,
            evidence_status: "SIMULATION_ONLY".into(),
            trial_id: trial.trial_id.clone(),
            trial_digest: trial_digest(trial)?,
            paired_trial_key: trial.paired_trial_key.clone(),
            initial_observation: Value::Null,
            initial_observation_sha256: String::new(),
            reasoning_request_audit: String::new(),
            decision_audit: String::new(),
            pregrant_action_source: "none".into(),
            pregrant_action_key: None,
            authorized_action_key: None,
            authorized_grant_id: None,
            authorized_witness_digest: None,
            authorized_observation_epoch: None,
            refusal_code: None,
            control_writes_before: 0,
            control_writes_after: 0,
            simulation_time_before_s: 0.0,
            simulation_time_after_s: 0.0,
            simulation_timestep_s: 0.0,
            simulator_steps: 0,
        })
    }

    /// Verify the trial/observation binding and the zero-write, zero-step claim.
    pub fn verify(&self) -> Result<(), String> {
        if self.format != "realityos.first_action_preflight"
            || self.schema_version != 1
            || self.evidence_status != "SIMULATION_ONLY"
            || self.trial_id.trim().is_empty()
            || self.trial_digest.len() != 64
            || !self
                .trial_digest
                .bytes()
                .all(|byte| byte.is_ascii_hexdigit())
            || self
                .paired_trial_key
                .as_deref()
                .unwrap_or_default()
                .trim()
                .is_empty()
        {
            return Err("invalid first-action preflight identity".into());
        }
        if !self.initial_observation.is_object()
            || self.initial_observation_sha256
                != json_sha256(&self.initial_observation).map_err(|error| error.to_string())?
        {
            return Err("first-action preflight observation digest mismatch".into());
        }
        let observation_time_s = self
            .initial_observation
            .get("time")
            .and_then(Value::as_f64)
            .filter(|time| time.is_finite() && *time >= 0.0)
            .ok_or_else(|| {
                "first-action preflight observation has no valid simulator time".to_owned()
            })?;
        if !self.simulation_time_before_s.is_finite()
            || self.simulation_time_before_s < 0.0
            || self.simulation_time_before_s.to_bits() != observation_time_s.to_bits()
        {
            return Err(
                "first-action preflight simulator time does not match its observation".into(),
            );
        }
        if self.reasoning_request_audit.is_empty() || self.decision_audit.is_empty() {
            return Err("first-action preflight is missing its request or decision audit".into());
        }
        if !self.simulation_timestep_s.is_finite() || self.simulation_timestep_s <= 0.0 {
            return Err("first-action preflight has an invalid simulator timestep".into());
        }
        if self.control_writes_before != 0
            || self.control_writes_after != 0
            || !self.simulation_time_before_s.is_finite()
            || !self.simulation_time_after_s.is_finite()
            || self.simulation_time_before_s < 0.0
            || self.simulation_time_after_s < 0.0
            || self.simulation_time_before_s.to_bits() != self.simulation_time_after_s.to_bits()
            || self.simulator_steps != 0
        {
            return Err("first-action preflight advanced the simulator or wrote controls".into());
        }
        if self.authorized_action_key.is_some() == self.refusal_code.is_some() {
            return Err("first-action preflight must contain one action or one refusal".into());
        }
        if self
            .authorized_action_key
            .as_deref()
            .map_or(false, |action| action.trim().is_empty())
            || self
                .refusal_code
                .as_deref()
                .map_or(false, |refusal| refusal.trim().is_empty())
        {
            return Err("first-action preflight action or refusal is empty".into());
        }
        if self.authorized_action_key.is_some()
            && (self.authorized_grant_id.is_none()
                || self.authorized_witness_digest.is_none()
                || self.authorized_observation_epoch.is_none()
                || self
                    .authorized_grant_id
                    .as_deref()
                    .map_or(true, |value| value.trim().is_empty())
                || self
                    .authorized_witness_digest
                    .as_deref()
                    .map_or(true, |value| value.trim().is_empty())
                || self
                    .authorized_observation_epoch
                    .as_deref()
                    .map_or(true, |value| value.trim().is_empty()))
        {
            return Err("authorized first action is missing its authority binding".into());
        }
        Ok(())
    }

    /// Verify that the receipt was generated for this resolved trial.
    pub fn verify_for_trial(&self, trial: &ResolvedBenchmarkTrial) -> Result<(), String> {
        self.verify()?;
        let expected_digest = trial_digest(trial).map_err(|error| error.to_string())?;
        if self.trial_id != trial.trial_id
            || self.trial_digest != expected_digest
            || self.paired_trial_key != trial.paired_trial_key
        {
            return Err("first-action preflight does not match the resolved trial".into());
        }
        Ok(())
    }

    /// Verify a later episode's initial observation, request, decision, and authority.
    pub fn verify_matches_episode(&self, episode: &BenchmarkEpisode) -> Result<(), String> {
        self.verify()?;
        if self.trial_id != episode.trial_id
            || self.trial_digest != episode.trial_digest
            || self.paired_trial_key != episode.paired_trial_key
        {
            return Err("first-action preflight does not match the replay trial".into());
        }
        let replay_observation = episode
            .initial_observation_evidence
            .as_ref()
            .ok_or_else(|| "replay is missing its initial-observation evidence".to_owned())?;
        replay_observation.verify()?;
        if replay_observation.parsed_observation()? != self.initial_observation
            || replay_observation.observation_sha256 != self.initial_observation_sha256
            || replay_observation.simulator_time_s.to_bits()
                != self.simulation_time_before_s.to_bits()
        {
            return Err(
                "first-action preflight initial observation does not match the replay".into(),
            );
        }
        if episode.reasoning_request_audits.first() != Some(&self.reasoning_request_audit) {
            return Err("first-action preflight request does not match the replay".into());
        }
        let decision_audit = episode
            .decision_audits
            .first()
            .ok_or_else(|| "replay is missing its first decision audit".to_owned())?;
        if !decision_audit.starts_with(&self.decision_audit) {
            return Err("first-action preflight decision does not match the replay".into());
        }
        match (
            self.authorized_action_key.as_deref(),
            episode.actions.first(),
        ) {
            (Some(action_key), Some(action)) => {
                if action.action_key != action_key
                    || self.authorized_grant_id.as_deref() != Some(action.grant_id.as_str())
                    || self.authorized_witness_digest.as_deref()
                        != Some(action.witness_digest.as_str())
                    || self.authorized_observation_epoch.as_deref()
                        != Some(action.observation_epoch.as_str())
                    || !decision_audit.contains(&format!(";authorized_action_key={action_key}"))
                {
                    return Err("first-action preflight authority does not match the replay".into());
                }
            }
            (Some(_), None) => {
                return Err("replay is missing the preflight-authorized first action".into());
            }
            (None, Some(_)) => {
                return Err("replay acted after a preflight refusal".into());
            }
            (None, None) => {
                if episode.refusal_code.as_deref() != self.refusal_code.as_deref() {
                    return Err("first-action preflight refusal does not match the replay".into());
                }
            }
        }
        Ok(())
    }
}

fn json_sha256(value: &Value) -> Result<String, serde_json::Error> {
    let bytes = serde_json::to_vec(value)?;
    Ok(sha256_hex(&bytes))
}

fn sha256_hex(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    digest.iter().map(|byte| format!("{byte:02x}")).collect()
}

impl ProbeMeasurementSidecarV1 {
    /// Check each sample, the full measurement binding, and the inferred interval.
    pub fn verify(&self) -> Result<(), String> {
        if self.format != "realityos.probe_measurement_sidecar"
            || self.schema_version != 1
            || self.trial_id.trim().is_empty()
            || self.trial_digest.len() != 64
            || !self
                .trial_digest
                .bytes()
                .all(|byte| byte.is_ascii_hexdigit())
        {
            return Err("invalid probe measurement sidecar identity".into());
        }
        if self.measurements.is_empty() {
            return Err("probe measurement sidecar has no measurement records".into());
        }
        for (index, record) in self.measurements.iter().enumerate() {
            if record.action_key.trim().is_empty() {
                return Err(format!("measurement record {index} has no action key"));
            }
            if record.prior_support_friction_interval != record.measurement.prior_support_friction {
                return Err(format!(
                    "measurement record {index} prior interval mismatch"
                ));
            }
            if record.measurement.samples.is_empty() {
                return Err(format!("measurement record {index} has no samples"));
            }
            for (sample_index, sample) in record.measurement.samples.iter().enumerate() {
                sample.validate().map_err(|error| {
                    format!("measurement record {index} sample {sample_index}: {error}")
                })?;
            }
            if record.measurement.digest() != record.measurement_digest {
                return Err(format!("measurement record {index} digest mismatch"));
            }
            if infer_support_friction(&record.measurement) != record.inference {
                return Err(format!("measurement record {index} inference mismatch"));
            }
        }
        Ok(())
    }
}

pub fn run_benchmark_trial(
    bundle: &RobotBundle,
    trial: &ResolvedBenchmarkTrial,
    admission: SimulationAdmission,
    wrench_profile: Option<&SimulationWrenchSensorProfile>,
) -> Result<BenchmarkEpisode, BenchmarkError> {
    run_benchmark_trial_inner(bundle, trial, admission, wrench_profile, None, None)
}

/// Run the benchmark's real initial observation, ranking, and authorization
/// path, then return before any control write or simulator step.
pub fn preflight_benchmark_trial_first_action(
    bundle: &RobotBundle,
    trial: &ResolvedBenchmarkTrial,
    admission: SimulationAdmission,
    wrench_profile: Option<&SimulationWrenchSensorProfile>,
) -> Result<FirstActionPreflightV1, BenchmarkError> {
    if trial.task != BenchmarkTask::PushProbeReplan
        || trial.expectation != TrialExpectation::ExpectedSuccess
        || trial.scenario.move_object_after_obs
        || trial.fault_injection.is_some()
    {
        return Err(BenchmarkError::InvalidTrial(
            "first-action preflight supports only expected-success un-faulted push/probe trials"
                .into(),
        ));
    }
    let mut preflight = FirstActionPreflightV1::new(trial)?;
    let episode = run_benchmark_trial_inner(
        bundle,
        trial,
        admission,
        wrench_profile,
        None,
        Some(&mut preflight),
    )?;
    preflight.control_writes_after = episode.ctrl_writes_after;
    preflight.simulation_time_after_s = episode.simulation_time_s;
    let elapsed_steps = (preflight.simulation_time_after_s - preflight.simulation_time_before_s)
        / preflight.simulation_timestep_s;
    if !elapsed_steps.is_finite() || elapsed_steps < 0.0 {
        return Err(BenchmarkError::InvalidTrial(
            "first-action preflight produced an invalid simulation-time delta".into(),
        ));
    }
    preflight.simulator_steps = elapsed_steps.round() as u64;
    preflight.verify().map_err(BenchmarkError::InvalidTrial)?;
    Ok(preflight)
}

/// Run a benchmark trial and write complete probe measurement data to a new,
/// separately named JSON sidecar. The regular `BenchmarkEpisode` stays compact.
pub fn run_benchmark_trial_with_probe_measurement_sidecar(
    bundle: &RobotBundle,
    trial: &ResolvedBenchmarkTrial,
    admission: SimulationAdmission,
    wrench_profile: Option<&SimulationWrenchSensorProfile>,
    sidecar_path: &Path,
) -> Result<BenchmarkEpisode, BenchmarkError> {
    run_benchmark_trial_inner(
        bundle,
        trial,
        admission,
        wrench_profile,
        Some(sidecar_path),
        None,
    )
}

fn scenario_objects_with_grasp_friction(
    scenario_objects: &[Value],
    object_id: &str,
    gripper: Option<&ControlledResource>,
) -> Result<Vec<Value>, String> {
    let mut objects = scenario_objects.to_vec();
    let targets = objects
        .iter()
        .filter(|object| object.get("name").and_then(Value::as_str) == Some(object_id))
        .count();
    if targets != 1 {
        return Err(format!(
            "grasp friction target object must resolve exactly once: {object_id}"
        ));
    }
    let target_index = objects
        .iter()
        .position(|object| object.get("name").and_then(Value::as_str) == Some(object_id))
        .expect("target object was counted above");
    let Some(friction_value) = objects[target_index].get("friction") else {
        return Ok(objects);
    };
    friction_value
        .as_f64()
        .filter(|value| value.is_finite() && *value >= 0.0)
        .ok_or_else(|| "grasp friction target coefficient is invalid".to_string())?;
    let gripper = gripper
        .ok_or_else(|| "grasp friction target has no selected gripper resource".to_string())?;
    let finger_bodies = gripper.finger_bodies.clone();
    let mut seen_finger_bodies = std::collections::BTreeSet::new();
    if finger_bodies.is_empty()
        || finger_bodies
            .iter()
            .any(|body| body.is_empty() || !seen_finger_bodies.insert(body))
    {
        return Err("grasp friction gripper finger-body mapping is invalid".into());
    }
    objects[target_index]["tool_object_friction_bodies"] = serde_json::json!(finger_bodies);
    Ok(objects)
}

fn run_benchmark_trial_inner(
    bundle: &RobotBundle,
    trial: &ResolvedBenchmarkTrial,
    admission: SimulationAdmission,
    wrench_profile: Option<&SimulationWrenchSensorProfile>,
    sidecar_path: Option<&Path>,
    mut preflight: Option<&mut FirstActionPreflightV1>,
) -> Result<BenchmarkEpisode, BenchmarkError> {
    let preflight_only = preflight.is_some();
    let wall = Instant::now();
    let mut probe_measurement_records = sidecar_path.map(|_| Vec::new());
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

    let worker_objects = if trial.task == BenchmarkTask::PinchGraspLift {
        scenario_objects_with_grasp_friction(
            &trial.scenario.objects,
            &trial.scenario.object_id,
            qualified_resources.first(),
        )
        .map_err(BenchmarkError::InvalidTrial)?
    } else {
        trial.scenario.objects.clone()
    };

    let loaded = match wrench_profile {
        Some(profile) => {
            worker.load_bundle_with_wrench_profile(bundle, &worker_objects, trial.seed, profile)
        }
        None => worker.load_bundle(bundle, &worker_objects, trial.seed),
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
    let initial_observation_evidence = ReplayInitialObservationEvidenceV1::capture(&initial_state)?;
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
        initial_observation_evidence: Some(initial_observation_evidence),
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
        wrench_compensation_audit_invalid: false,
        wrench_compensation_audit_invalid_samples: 0,
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
        let (pregrant_action_source, pregrant_action_key) = pregrant_action_metadata(
            preview
                .executable
                .as_ref()
                .map(|step| step.action_key.as_str()),
            preview
                .authority_request
                .as_ref()
                .map(|step| step.action_key.as_str()),
        );
        let decision_audit = format!(
            "decision={:?};pregrant_action_source={};pregrant_action_key={};support_friction_interval={:?};candidates={}",
            preview.decision,
            pregrant_action_source,
            pregrant_action_key.unwrap_or("none"),
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
        );
        episode.decision_audits.push(decision_audit.clone());
        if let Some(preflight) = preflight.as_deref_mut() {
            preflight.initial_observation = current_state.clone();
            preflight.initial_observation_sha256 = json_sha256(&current_state)?;
            preflight.reasoning_request_audit = request_debug.clone();
            preflight.decision_audit = decision_audit;
            preflight.pregrant_action_source = pregrant_action_source.into();
            preflight.pregrant_action_key = pregrant_action_key.map(str::to_owned);
            preflight.control_writes_before = episode.ctrl_writes_before;
            preflight.simulation_time_before_s =
                current_state["time"].as_f64().ok_or_else(|| {
                    BenchmarkError::InvalidTrial(
                        "initial observation is missing simulator time".into(),
                    )
                })?;
            preflight.simulation_timestep_s = manifest.timestep;
        }

        let authorized = match authorize_current_action(
            &mut os,
            &mut reasoner,
            &request,
            &admission,
            &blocked_action_keys,
        ) {
            Ok(authorized) => authorized,
            Err(refusal) => {
                if let Some(preflight) = preflight.as_deref_mut() {
                    preflight.refusal_code = Some(refusal.to_string());
                }
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
        if let Some(preflight) = preflight.as_deref_mut() {
            preflight.authorized_action_key = Some(frozen.action_key.clone());
            preflight.authorized_grant_id = Some(authorized.grant.command_id.clone());
            preflight.authorized_witness_digest = Some(frozen.witness_digest.clone());
            preflight.authorized_observation_epoch = Some(frozen.observation_epoch.clone());
        }
        if preflight_only {
            return finalize(worker, episode, wall);
        }
        let Some(authorization) = frozen.execution_authorization.as_ref() else {
            return Err(BenchmarkError::Simulation(
                "bridge action missing frozen execution authorization".into(),
            ));
        };
        let action_key = frozen.action_key.clone();
        if let Some(audit) = episode.decision_audits.last_mut() {
            audit.push_str(&format!(";authorized_action_key={action_key}"));
        }
        if trial.expectation == TrialExpectation::ExpectedRefusal {
            // The evaluator fence keeps the action out of the simulator.
            // A probe on an unlabeled positive scenario is a label mismatch.
            // A goal push, a faulted scenario, or any unauthorized write stays
            // a scored false acceptance.
            let preview = episode
                .decision_audits
                .last()
                .map(String::as_str)
                .unwrap_or("");
            if evaluation_label_mismatch(
                &action_key,
                &trial.scenario,
                episode.unauthorized_writes,
                preview,
            ) {
                episode.refusal_code = Some("EVALUATION_LABEL_MISMATCH".into());
                episode.failure_cause = Some(FailureCause::EvaluationLabelMismatch);
                episode.outcome = TrialOutcome::EvaluationLabelMismatch;
                break;
            }
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
        let (exec, grasp_hold_start, grasp_phase_positions, grasp_divergence_diagnostic) =
            match trial.task {
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
                    (execution, None, Vec::new(), None)
                }
                BenchmarkTask::PinchGraspLift => {
                    let Some(resource) = grasp_resource.as_ref() else {
                        episode.failure_cause = Some(FailureCause::UnsupportedTask);
                        episode.refusal_code = Some("QUALIFIED_GRIPPER_RESOURCE_REQUIRED".into());
                        episode.outcome = TrialOutcome::ActionFailed;
                        break;
                    };
                    let result = execute_grasp_witness(
                        &mut worker,
                        &manifest,
                        &request,
                        &authorized,
                        resource,
                    )?;
                    episode
                        .grasp_contact_audits
                        .extend(result.lift_contact_audits.iter().cloned());
                    (
                        result.execution,
                        result.hold_start,
                        result.phase_object_positions,
                        result.divergence_reason,
                    )
                }
            };
        episode.grasp_phase_object_positions = grasp_phase_positions;
        if let Some(diagnostic) = grasp_divergence_diagnostic {
            episode
                .decision_audits
                .push(format!("grasp_lift_guard_failure={diagnostic}"));
        }
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
            let measurement_digest = measurement.digest();
            episode.wrench_inference_measurement_digest = Some(measurement_digest.clone());
            let inference = infer_support_friction(&measurement);
            match inference {
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
            if let Some(records) = probe_measurement_records.as_mut() {
                records.push(ProbeMeasurementRecordV1 {
                    action_key: action_key.clone(),
                    measurement_digest,
                    prior_support_friction_interval: prior,
                    inference,
                    posterior_support_friction_interval: episode
                        .posterior_support_friction_interval,
                    measurement,
                });
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
    let episode = finalize(worker, episode, wall)?;
    if let Some(path) = sidecar_path {
        let sidecar = ProbeMeasurementSidecarV1 {
            format: "realityos.probe_measurement_sidecar".into(),
            schema_version: 1,
            trial_id: trial.trial_id.clone(),
            trial_digest: trial_digest(trial)?,
            paired_trial_key: trial.paired_trial_key.clone(),
            measurements: probe_measurement_records.unwrap_or_default(),
        };
        sidecar.verify().map_err(BenchmarkError::InvalidTrial)?;
        write_probe_measurement_sidecar(path, &sidecar)?;
    }
    Ok(episode)
}

fn write_probe_measurement_sidecar(
    path: &Path,
    sidecar: &ProbeMeasurementSidecarV1,
) -> Result<(), BenchmarkError> {
    if sidecar.measurements.is_empty() {
        return Err(BenchmarkError::InvalidTrial(
            "probe measurement sidecar requested but no probe measurement was produced".into(),
        ));
    }
    let file = OpenOptions::new().write(true).create_new(true).open(path)?;
    let mut writer = BufWriter::new(file);
    let result = (|| {
        serde_json::to_writer_pretty(&mut writer, sidecar)?;
        writer.write_all(b"\n")?;
        writer.flush()?;
        Ok::<(), BenchmarkError>(())
    })();
    if result.is_err() {
        drop(writer);
        let _ = fs::remove_file(path);
    }
    result
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
    start_orientation_world_wxyz: [f64; 4],
    start_contact_pairs: Option<Vec<ProbeContactPair>>,
    end_position: [f64; 3],
    max_speed_m_s: f64,
    first_motion_s: Option<f64>,
    unresolved_motion_before_force: bool,
}

impl Default for PushMotionAudit {
    fn default() -> Self {
        Self {
            started: false,
            start_s: 0.0,
            end_s: 0.0,
            start_position: [0.0; 3],
            start_orientation_world_wxyz: [1.0, 0.0, 0.0, 0.0],
            start_contact_pairs: None,
            end_position: [0.0; 3],
            max_speed_m_s: 0.0,
            first_motion_s: None,
            unresolved_motion_before_force: false,
        }
    }
}

struct PushWitnessAudit {
    execution: WitnessExecution,
    samples: Vec<WrenchSample>,
    sample_free_space_wrenches: Vec<[f64; 6]>,
    sample_free_space_wrench_error_bounds: Vec<[f64; 6]>,
    sensor_positions: Vec<[f64; 3]>,
    sensor_orientations: Vec<[f64; 4]>,
    object_pose_trace: Vec<ProbeObjectPoseSample>,
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
        .extend(window.samples.iter().map(|sample| {
            reference_index.filter(|_| {
                window
                    .monitor_gravity_reference
                    .as_ref()
                    .is_some_and(|reference| {
                        reference.sequence <= sample.sequence
                            && reference.capture_s <= sample.capture_s
                            && reference.receive_s <= sample.receive_s
                    })
            })
        }));
    audit
        .sample_free_space_wrenches
        .extend(window.sample_free_space_wrenches);
    audit
        .sample_free_space_wrench_error_bounds
        .extend(window.sample_free_space_wrench_error_bounds);
    audit.samples.extend(window.samples);
    audit.sensor_positions.extend(window.sensor_positions);
    audit.sensor_orientations.extend(window.sensor_orientations);
    audit.object_pose_trace.extend(window.object_pose_trace);
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
        sample_free_space_wrenches: Vec::new(),
        sample_free_space_wrench_error_bounds: Vec::new(),
        sensor_positions: Vec::new(),
        sensor_orientations: Vec::new(),
        object_pose_trace: Vec::new(),
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
                .step_monitored(
                    steps,
                    profile,
                    policy,
                    &authorized.grant,
                    false,
                    true,
                    authorized.action.frozen().witness_contents.as_str(),
                    false,
                    None,
                )
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
        let mut motion_reference = if motion_phase && !audit.motion.started {
            let state = state_payload(worker)?;
            let orientation = value_quaternion(&state["xquat"][request.scene.object_id.as_str()])
                .ok_or_else(|| {
                BenchmarkError::Simulation(format!(
                    "observed body {} has no finite orientation",
                    request.scene.object_id
                ))
            })?;
            Some((
                state["time"].as_f64().unwrap_or(0.0),
                body_position(&state, &request.scene.object_id)?,
                orientation,
                probe_contact_pairs_from_state(&state)?,
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
        let ramp_target = next.clone();
        // A direct target write can create a large contact transient before
        // the wrench monitor receives its next sample. Apply all authorized
        // push trajectories through a bounded per-step setpoint ramp.
        let mut converged = false;
        let ramp_rate = if motion_phase {
            PUSH_CONTACT_JOINT_RAMP_RATE_RAD_S
        } else {
            PUSH_TRANSIT_JOINT_RAMP_RATE_RAD_S
        };
        let total_steps = joint_control_ramp_steps(
            &ctrl,
            &ramp_target,
            &actuator_map,
            &request.scene.joint_names,
            manifest.timestep,
            ramp_rate,
        );
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
            } else {
                PROBE_CONTROL_UPDATE_STEPS
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
            has_written = true;
            let control_ramp = MonitoredControlRamp {
                start_ctrl: &ctrl,
                target_ctrl: &ramp_target,
                elapsed_steps,
                total_steps,
            };
            let monitor = worker
                .step_monitored(
                    steps,
                    profile,
                    policy,
                    &authorized.grant,
                    is_probe && motion_phase && elapsed_steps == 0,
                    false,
                    authorized.action.frozen().witness_contents.as_str(),
                    target_index >= transit_count,
                    Some(control_ramp),
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
                    let (start_s, start_position, start_orientation, start_contact_pairs) =
                        motion_reference.take().ok_or_else(|| {
                            BenchmarkError::Simulation("probe_motion_reference_missing".into())
                        })?;
                    audit.motion.started = true;
                    audit.motion.start_s = start_s;
                    audit.motion.end_s = audit.motion.start_s;
                    audit.motion.start_position = start_position;
                    audit.motion.start_orientation_world_wxyz = start_orientation;
                    audit.motion.start_contact_pairs = Some(start_contact_pairs);
                    audit.motion.end_position = start_position;
                }
                let prior_position = audit.motion.end_position;
                let current_position = body_position(&state, &request.scene.object_id)?;
                let step_delta = sub3(current_position, prior_position);
                let start_delta = sub3(current_position, audit.motion.start_position);
                if is_probe && audit.motion.first_motion_s.is_none() {
                    let force_lower_bound =
                        latest_probe_tangential_force_lower_bound(&audit, profile, time_s);
                    observe_probe_motion_sample(
                        &mut audit.motion,
                        current_position,
                        time_s,
                        force_lower_bound,
                    );
                } else if !is_probe
                    && audit.motion.first_motion_s.is_none()
                    && start_delta[0].hypot(start_delta[1]) > OBJECT_MOTION_RESOLUTION_M
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
            if remaining_steps > 0 {
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
                        authorized.action.frozen().witness_contents.as_str(),
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
                    .step_monitored(
                        steps,
                        profile,
                        policy,
                        &authorized.grant,
                        false,
                        false,
                        authorized.action.frozen().witness_contents.as_str(),
                        false,
                        None,
                    )
                    .map_err(|error| {
                        BenchmarkError::Simulation(format!("wrench_precontact_settle:{error}"))
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
                settle_remaining -= steps;
            }
            if audit.precontact_baseline.is_none() {
                // Capture at the settled pre-contact pose. Taking the baseline
                // at the start of the long hold above makes it stale before the
                // first in-motion sample, despite a freshly settled robot.
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
                let sample_steps =
                    profile
                        .sample_interval_steps(manifest.timestep)
                        .map_err(|error| {
                            BenchmarkError::Simulation(format!(
                                "wrench_precontact_sample_interval:{error}"
                            ))
                        })?;
                if steps_until_expiry < sample_steps {
                    audit.execution = WitnessExecution::AuthorityExpired;
                    return Ok(audit);
                }
                let monitor = worker
                    .step_monitored(
                        sample_steps,
                        profile,
                        policy,
                        &authorized.grant,
                        true,
                        true,
                        authorized.action.frozen().witness_contents.as_str(),
                        false,
                        None,
                    )
                    .map_err(|error| {
                        BenchmarkError::Simulation(format!("wrench_precontact_capture:{error}"))
                    })?;
                audit.control_correction_max_rad = audit
                    .control_correction_max_rad
                    .max(monitor.control_correction_max_rad);
                record_wrench_contacts(&mut audit, &monitor.state);
                audit.precontact_baseline_status =
                    monitor.wrench_window.precontact_baseline_status.clone();
                audit.precontact_baseline = monitor.wrench_window.precontact_baseline.clone();
                audit.precontact_baseline_position =
                    monitor.wrench_window.precontact_baseline_position;
                audit.precontact_baseline_orientation =
                    monitor.wrench_window.precontact_baseline_orientation;
                record_push_wrench_window(&mut audit, monitor.wrench_window);
                if let Some(abort) = monitor.abort {
                    audit.execution = WitnessExecution::WrenchAborted;
                    audit.abort = Some(abort);
                    return Ok(audit);
                }
                if audit.precontact_baseline_status.as_deref() == Some("contact_present")
                    || audit.precontact_baseline.is_none()
                {
                    audit.execution = WitnessExecution::BaselineUnavailable;
                    return Ok(audit);
                }
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
        episode.wrench_compensation_audit_invalid = true;
        episode.wrench_abort_result = Some("profile_digest_invalid".into());
        episode.wrench_peak_conservative_bounds = None;
        episode.wrench_minimum_margin = None;
        return;
    };
    episode.wrench_sensor_profile_digest = Some(profile_digest);
    episode.wrench_compensation_digest = Some(profile.limit_policy.compensation_digest.clone());
    if audit.monitor_gravity_reference.is_some() {
        episode.wrench_monitor_gravity_reference = audit.monitor_gravity_reference.clone();
        episode.wrench_monitor_gravity_reference_position =
            audit.monitor_gravity_reference_position;
        episode.wrench_monitor_gravity_reference_orientation =
            audit.monitor_gravity_reference_orientation;
    }
    if let Some(status) = audit.monitor_gravity_reference_status.as_ref() {
        episode.wrench_monitor_gravity_reference_status = Some(status.clone());
    }
    for reference in &audit.monitor_gravity_references {
        if !episode
            .wrench_monitor_gravity_reference_history
            .iter()
            .any(|existing| existing.sample.digest == reference.sample.digest)
        {
            episode
                .wrench_monitor_gravity_reference_history
                .push(reference.clone());
        }
    }
    if let Some(status) = audit.precontact_baseline_status.as_ref() {
        episode.wrench_precontact_baseline_status = Some(status.clone());
    }
    if audit.precontact_baseline.is_some() {
        episode.wrench_precontact_baseline = audit.precontact_baseline.clone();
    }
    for contact_pair in &audit.wrench_contact_pairs {
        if !episode.wrench_contact_pairs.contains(contact_pair) {
            episode.wrench_contact_pairs.push(contact_pair.clone());
        }
    }
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
                        episode.wrench_compensation_audit_invalid = true;
                        episode.wrench_compensation_audit_invalid_samples = episode
                            .wrench_compensation_audit_invalid_samples
                            .saturating_add(1);
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
        episode.wrench_max_sample_age_s = Some(
            episode
                .wrench_max_sample_age_s
                .map_or(max_age_s, |previous| previous.max(max_age_s)),
        );
        episode.wrench_max_inter_sample_gap_s = Some(
            episode
                .wrench_max_inter_sample_gap_s
                .map_or(max_gap_s, |previous| previous.max(max_gap_s)),
        );
        if episode.wrench_compensation_audit_invalid {
            episode.wrench_peak_conservative_bounds = None;
            episode.wrench_minimum_margin = None;
        } else {
            episode.wrench_peak_conservative_bounds = Some(
                episode
                    .wrench_peak_conservative_bounds
                    .map_or(peaks, |previous| {
                        std::array::from_fn(|axis| previous[axis].max(peaks[axis]))
                    }),
            );
            episode.wrench_minimum_margin = Some(
                episode
                    .wrench_minimum_margin
                    .map_or(minimum_margin, |previous| previous.min(minimum_margin)),
            );
        }
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

fn probe_contact_pairs_from_state(state: &Value) -> Result<Vec<ProbeContactPair>, BenchmarkError> {
    let contacts = state["contacts"]
        .as_array()
        .ok_or_else(|| BenchmarkError::Simulation("probe_start_contact_evidence_missing".into()))?;
    contacts
        .iter()
        .map(|contact| {
            let field = |name| {
                contact[name]
                    .as_str()
                    .filter(|value| !value.trim().is_empty())
                    .map(str::to_owned)
                    .ok_or_else(|| {
                        BenchmarkError::Simulation("probe_start_contact_identity_missing".into())
                    })
            };
            Ok(ProbeContactPair {
                body1: field("body1")?,
                geom1_name: field("geom1_name")?,
                body2: field("body2")?,
                geom2_name: field("geom2_name")?,
            })
        })
        .collect()
}

fn sample_is_after_precontact_baseline(
    sample: &WrenchSample,
    baseline: Option<&WrenchSample>,
) -> bool {
    baseline.map_or(true, |baseline| {
        sample.sequence > baseline.sequence && sample.capture_s > baseline.capture_s
    })
}

fn latest_probe_tangential_force_lower_bound(
    audit: &PushWitnessAudit,
    profile: &SimulationWrenchSensorProfile,
    now_s: f64,
) -> Option<f64> {
    let index = audit.samples.len().checked_sub(1)?;
    if audit.samples.len() != audit.sample_free_space_wrenches.len()
        || audit.samples.len() != audit.sample_free_space_wrench_error_bounds.len()
        || audit.samples.len() != audit.sensor_orientations.len()
    {
        return None;
    }
    let sample = audit.samples.get(index)?;
    let age_s = now_s - sample.capture_s;
    if !age_s.is_finite() || age_s < 0.0 || age_s > profile.limit_policy.max_sample_age_s {
        return None;
    }
    probe_sample_tangential_force_lower_bound(
        sample,
        audit.sample_free_space_wrenches[index],
        audit.sensor_orientations[index],
        profile.bias,
        std::array::from_fn(|axis| {
            profile.error_bounds[axis] + profile.free_space_model_error_bounds[axis]
        }),
        profile.limit_policy.max_rise_rate,
        age_s,
    )
}

fn probe_slip_onset_resolved(displacement: [f64; 3], force_lower_bound: Option<f64>) -> bool {
    displacement[0].hypot(displacement[1]) > PROBE_SLIP_DISPLACEMENT_M
        && force_lower_bound
            .is_some_and(|force| force.is_finite() && force >= MIN_PROBE_CONTACT_FORCE_N)
}

fn observe_probe_motion_sample(
    motion: &mut PushMotionAudit,
    current_position: [f64; 3],
    time_s: f64,
    force_lower_bound: Option<f64>,
) {
    if motion.first_motion_s.is_none() && !motion.unresolved_motion_before_force {
        let displacement = sub3(current_position, motion.start_position);
        if displacement[0].hypot(displacement[1]) > PROBE_SLIP_DISPLACEMENT_M {
            if probe_slip_onset_resolved(displacement, force_lower_bound) {
                motion.first_motion_s = Some(time_s);
            } else {
                // Keep the original motion origin and mark the evidence
                // unresolved. Repeated small steps must not erase cumulative
                // object motion when the force lower bound is too weak.
                motion.unresolved_motion_before_force = true;
            }
        }
    }
    motion.end_s = time_s;
    motion.end_position = current_position;
}

fn probe_sample_tangential_force_lower_bound(
    sample: &WrenchSample,
    free_space_wrench: [f64; 6],
    sensor_orientation: [f64; 4],
    bias: [f64; 6],
    compensation_error: [f64; 6],
    max_rise_rate: [f64; 6],
    sample_age_s: f64,
) -> Option<f64> {
    if !sample_age_s.is_finite()
        || sample_age_s < 0.0
        || sample
            .wrench
            .iter()
            .chain(free_space_wrench.iter())
            .chain(bias.iter())
            .chain(compensation_error.iter())
            .chain(max_rise_rate.iter())
            .any(|value| !value.is_finite())
        || compensation_error.iter().any(|value| *value < 0.0)
        || max_rise_rate.iter().any(|value| *value < 0.0)
    {
        return None;
    }
    let rotation = quaternion_rotation(sensor_orientation);
    let sensor_force = std::array::from_fn::<_, 3, _>(|axis| {
        sample.wrench[axis] - bias[axis] - free_space_wrench[axis]
    });
    let sensor_error = std::array::from_fn::<_, 3, _>(|axis| {
        sample.error_bounds[axis] + compensation_error[axis] + max_rise_rate[axis] * sample_age_s
    });
    let support_force = std::array::from_fn::<_, 2, _>(|axis| {
        (0..3)
            .map(|source| rotation[axis][source] * sensor_force[source])
            .sum::<f64>()
    });
    let support_error = std::array::from_fn::<_, 2, _>(|axis| {
        (0..3)
            .map(|source| rotation[axis][source].abs() * sensor_error[source])
            .sum::<f64>()
    });
    let lower_magnitude = |center: f64, error: f64| {
        let lower = center - error;
        let upper = center + error;
        if lower <= 0.0 && upper >= 0.0 {
            0.0
        } else {
            lower.abs().min(upper.abs())
        }
    };
    Some(
        lower_magnitude(support_force[0], support_error[0])
            .hypot(lower_magnitude(support_force[1], support_error[1])),
    )
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
    if audit.samples.len() != audit.sample_free_space_wrenches.len()
        || audit.samples.len() != audit.sensor_positions.len()
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
        .zip(&audit.sample_free_space_wrenches)
        .zip(&audit.sample_free_space_wrench_error_bounds)
        .filter(|((((sample, _), _), _), _)| {
            sample.capture_s >= start_s
                && sample.capture_s <= end_s
                && sample_is_after_precontact_baseline(sample, audit.precontact_baseline.as_ref())
        })
        .map(
            |((((sample, position), quaternion), gravity_wrench), model_error_bound)| {
                let transform =
                    wrench_transform_for_pose(*position, *quaternion, request.scene.support_origin);
                (
                    sample.clone(),
                    transform,
                    *gravity_wrench,
                    *model_error_bound,
                )
            },
        )
        .collect::<Vec<_>>();
    aligned_samples.sort_by(|left, right| {
        left.0
            .capture_s
            .total_cmp(&right.0.capture_s)
            .then_with(|| left.0.sequence.cmp(&right.0.sequence))
    });
    let samples = aligned_samples
        .iter()
        .map(|(sample, _, _, _)| sample.clone())
        .collect::<Vec<_>>();
    let sample_sensor_transforms = aligned_samples
        .iter()
        .map(|(_, transform, _, _)| transform.clone())
        .collect::<Vec<_>>();
    let sample_free_space_wrenches = aligned_samples
        .iter()
        .map(|(_, _, gravity_wrench, _)| *gravity_wrench)
        .collect::<Vec<_>>();
    let sample_free_space_wrench_error_bounds = aligned_samples
        .iter()
        .map(|(_, _, _, model_error_bound)| *model_error_bound)
        .collect::<Vec<_>>();
    let motion_trace = if audit.motion.started {
        let samples = audit
            .object_pose_trace
            .iter()
            .filter(|sample| {
                sample.object_id == request.scene.object_id
                    && sample.capture_s >= start_s
                    && sample.capture_s <= end_s
            })
            .cloned()
            .collect::<Vec<_>>();
        let clock_domain = samples
            .first()
            .map(|sample| sample.clock_domain.clone())
            .ok_or_else(|| BenchmarkError::Simulation("probe_object_pose_trace_missing".into()))?;
        let bootstrap_wrench_sequences = aligned_samples
            .iter()
            .filter(|(sample, _, _, _)| {
                sample.capture_s == start_s
                    && !samples.iter().any(|pose| {
                        pose.wrench_sequence == Some(sample.sequence)
                            && pose.capture_s == sample.capture_s
                    })
            })
            .map(|(sample, _, _, _)| sample.sequence)
            .collect::<Vec<_>>();
        if bootstrap_wrench_sequences.len() > 1 {
            return Err(BenchmarkError::Simulation(
                "probe_object_pose_trace_ambiguous_start_wrench".into(),
            ));
        }
        let mut trace = ProbeObjectPoseTrace {
            object_id: request.scene.object_id.clone(),
            clock_domain,
            start_s,
            start_position_world_m: audit.motion.start_position,
            start_orientation_world_wxyz: audit.motion.start_orientation_world_wxyz,
            start_wrench_sequence: bootstrap_wrench_sequences.first().copied(),
            start_contact_pairs: audit.motion.start_contact_pairs.clone(),
            samples,
            digest: String::new(),
        };
        trace.digest = trace.digest();
        trace.validate().map_err(|error| {
            BenchmarkError::Simulation(format!("probe_object_pose_trace_invalid:{error}"))
        })?;
        Some(trace)
    } else {
        None
    };
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
    Ok(PushProbeMeasurement {
        samples,
        precontact_baseline: audit.precontact_baseline.clone(),
        sample_free_space_wrenches,
        sample_free_space_wrench_error_bounds,
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
            pose_resolution_m: PROBE_SLIP_DISPLACEMENT_M,
            first_motion_s: audit.motion.first_motion_s,
            unresolved_motion_before_force: audit.motion.unresolved_motion_before_force,
        },
        motion_trace,
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
        runtime_contact_ambiguous: audit
            .abort
            .as_ref()
            .is_some_and(|abort| matches!(&abort.reason, WrenchAbortReason::UnexpectedContact)),
        saturated: audit.abort.as_ref().is_some_and(|abort| {
            matches!(&abort.reason, WrenchAbortReason::ActuatorSaturation { .. })
        }),
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

fn inference_compensation_digest(measurement: &PushProbeMeasurement) -> String {
    use sha2::{Digest, Sha256};
    let mut digest = Sha256::new();
    digest.update(b"realityos.simulated_probe_compensation/3\0");
    digest.update(measurement.compensation.digest.as_bytes());
    if let Some(baseline) = &measurement.precontact_baseline {
        digest.update(baseline.digest.as_bytes());
    }
    for wrench in &measurement.sample_free_space_wrenches {
        for value in wrench {
            digest.update(value.to_bits().to_le_bytes());
        }
    }
    for error_bound in &measurement.sample_free_space_wrench_error_bounds {
        for value in error_bound {
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
    divergence_reason: Option<String>,
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
            divergence_reason: None,
            hold_start: None,
            phase_object_positions: Vec::new(),
            lift_contact_audits: Vec::new(),
        });
    };
    let rejection = || GraspWitnessExecution {
        execution: WitnessExecution::WroteNothing,
        divergence_reason: None,
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
        && grasp_lift_states_match(&lift_states, &contact, close_positions)
        && *maximum_lift_m >= 0.02
        && q0.len() == request.scene.joint_names.len()
        && transit.len() == q0.len()
        && contact.len() == q0.len()
        && witness_close.len() == request.scene.finger_names.len();
    if !envelope_matches || !authorized_covers_now(worker, authorized, request.now_s)? {
        return Ok(rejection());
    }
    let measured_arm = read_named_joints(worker, manifest, &request.scene.joint_names)?;
    if !vectors_near(&measured_arm, &q0, 0.03) {
        let max_abs_error_rad = measured_arm
            .iter()
            .zip(&q0)
            .map(|(measured, target)| (measured - target).abs())
            .fold(0.0_f64, f64::max);
        return Ok(GraspWitnessExecution {
            execution: WitnessExecution::Diverged,
            divergence_reason: Some(format!(
                "joint_tracking_error_exceeded_0.03_rad;phase=start;max_abs_error_rad={max_abs_error_rad:.6}"
            )),
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
    let mut lift_contact_audits = Vec::new();
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
                    divergence_reason: None,
                    hold_start,
                    phase_object_positions,
                    lift_contact_audits: Vec::new(),
                });
            }
        }
        let measured = read_named_joints(worker, manifest, &request.scene.joint_names)?;
        if !vectors_near(&measured, arm, 0.08) {
            let max_abs_error_rad = measured
                .iter()
                .zip(arm)
                .map(|(measured, target)| (measured - target).abs())
                .fold(0.0_f64, f64::max);
            return Ok(GraspWitnessExecution {
                execution: WitnessExecution::Diverged,
                divergence_reason: Some(format!(
                    "joint_tracking_error_exceeded_0.08_rad;phase={phase};max_abs_error_rad={max_abs_error_rad:.6}"
                )),
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
                    divergence_reason: None,
                    hold_start,
                    phase_object_positions,
                    lift_contact_audits: Vec::new(),
                });
            }
            let settled = read_named_joints(worker, manifest, &request.scene.joint_names)?;
            if !vectors_near(&settled, arm, 0.08) {
                let max_abs_error_rad = settled
                    .iter()
                    .zip(arm)
                    .map(|(measured, target)| (measured - target).abs())
                    .fold(0.0_f64, f64::max);
                return Ok(GraspWitnessExecution {
                    execution: WitnessExecution::Diverged,
                    divergence_reason: Some(format!(
                        "joint_tracking_error_exceeded_0.08_rad;phase=close_settle;max_abs_error_rad={max_abs_error_rad:.6}"
                    )),
                    hold_start,
                    phase_object_positions,
                    lift_contact_audits: Vec::new(),
                });
            }
            let state = state_payload(worker)?;
            phase_object_positions.push(body_position(&state, &request.scene.object_id)?);
            let truth = VerifierTruth::from_mujoco_state(&state);
            lift_contact_audits.extend(grasp_contact_audits(
                "post_close",
                &truth,
                &request.scene.object_id,
                resource,
            ));
            hold_start = Some(truth);
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
        // Execute the same joint-space samples used by the collision proof,
        // observing finger contact, force, and object motion at each waypoint.
        const LIFT_STEPS_PER_WAYPOINT: u32 = 50;
        if start_arm.len() != contact.len() || end_arm.len() != contact.len() {
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
                        divergence_reason: None,
                        hold_start,
                        phase_object_positions,
                        lift_contact_audits,
                    });
                }
            }
            let measured = read_named_joints(worker, manifest, &request.scene.joint_names)?;
            if !vectors_near(&measured, &target, 0.08) {
                let max_abs_error_rad = measured
                    .iter()
                    .zip(&target)
                    .map(|(measured, target)| (measured - target).abs())
                    .fold(0.0_f64, f64::max);
                return Ok(GraspWitnessExecution {
                    execution: WitnessExecution::Diverged,
                    divergence_reason: Some(format!(
                        "joint_tracking_error_exceeded_0.08_rad;phase=lift_waypoint_{waypoint};max_abs_error_rad={max_abs_error_rad:.6}"
                    )),
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
            let force_failure = if audits.len() != resource.finger_bodies.len() {
                Some("finger_contact_audit_incomplete".to_owned())
            } else if let Some((audit, force)) = audits.iter().find_map(|audit| {
                audit.normal_force_n.and_then(|force| {
                    (force.is_finite() && force > *per_finger_force_ceiling_n + 1e-6)
                        .then_some((audit, force))
                })
            }) {
                Some(format!(
                    "per_finger_force_ceiling_exceeded;finger={};force_n={force:.6};limit_n={:.6}",
                    audit.finger_body, per_finger_force_ceiling_n
                ))
            } else if audits.iter().any(|audit| {
                !audit.force_available || !audit.normal_force_n.is_some_and(f64::is_finite)
            }) {
                Some("finger_force_observation_unavailable".to_owned())
            } else {
                None
            };
            lift_contact_audits.extend(audits);
            let object_hits_obstacle = request.scene.obstacles.iter().any(|obstacle| {
                truth.contacts.iter().any(|contact| {
                    contact_pair_has(contact, &request.scene.object_id, &obstacle.name)
                })
            });
            let relative_pose_error = truth_position(&truth, &request.scene.ee)
                .map(|ee| distance3(sub3(object_position, ee), start_relative))
                .unwrap_or(f64::INFINITY);
            let divergence_reason = if !bilateral_finger_contact(
                &truth,
                &request.scene.object_id,
                resource,
            ) {
                Some("bilateral_finger_contact_lost".to_owned())
            } else if let Some(force_failure) = force_failure {
                Some(force_failure)
            } else if object_hits_obstacle {
                Some("object_contact_with_obstacle".to_owned())
            } else if !relative_pose_error.is_finite() {
                Some("object_tool_relative_pose_unavailable".to_owned())
            } else if relative_pose_error > 0.02 {
                Some(format!(
                    "object_tool_relative_pose_exceeded_20mm;relative_pose_error_m={relative_pose_error:.6}"
                ))
            } else {
                None
            };
            if let Some(reason) = divergence_reason {
                return Ok(GraspWitnessExecution {
                    execution: WitnessExecution::Diverged,
                    divergence_reason: Some(format!("{reason};waypoint={waypoint}")),
                    hold_start,
                    phase_object_positions,
                    lift_contact_audits,
                });
            }
        }
    }
    Ok(GraspWitnessExecution {
        execution: WitnessExecution::Wrote,
        divergence_reason: None,
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

fn grasp_lift_states_match(
    lift_states: &[(Vec<f64>, Vec<f64>)],
    contact: &[f64],
    close_positions: &[f64],
) -> bool {
    lift_states.len() >= 2
        && lift_states.first().is_some_and(|(arm, fingers)| {
            finite_values_match(arm, contact) && finite_values_match(fingers, close_positions)
        })
        && lift_states.iter().all(|(arm, fingers)| {
            arm.len() == contact.len()
                && arm.iter().all(|value| value.is_finite())
                && finite_values_match(fingers, close_positions)
        })
}

fn finite_values_match(left: &[f64], right: &[f64]) -> bool {
    left.len() == right.len()
        && left
            .iter()
            .zip(right)
            .all(|(a, b)| a.is_finite() && b.is_finite() && a.to_bits() == b.to_bits())
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
    let object_com_world_m = truth
        .xipos
        .get(object_id)
        .and_then(|position| finite_vec3(position));
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
            let contact_wrenches = contacts
                .iter()
                .map(|contact| grasp_contact_wrench(contact, object_id, object_com_world_m))
                .collect();
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
                sample_time_s: truth.sample_time_s,
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
                object_com_world_m,
                ctrl: truth.ctrl.clone(),
                actuator_force: truth.actuator_force.clone(),
                contact_wrenches,
            }
        })
        .collect()
}

fn grasp_contact_wrench(
    contact: &crate::observation::ContactTruth,
    object_id: &str,
    object_com_world_m: Option<[f64; 3]>,
) -> GraspContactWrench {
    let sign_on_object = if contact.body1 == object_id {
        Some(-1.0)
    } else if contact.body2 == object_id {
        Some(1.0)
    } else {
        None
    };
    let force_world_on_object_n = if contact.force_available {
        contact
            .force_local
            .zip(contact.contact_frame)
            .zip(sign_on_object)
            .and_then(|((force_local, frame), sign)| {
                rotate_contact_vector(force_local, frame).map(|mut force| {
                    for component in &mut force {
                        *component *= sign;
                    }
                    force
                })
            })
    } else {
        None
    };
    let torque_world_on_object_nm = if contact.force_available {
        contact
            .torque_local
            .zip(contact.contact_frame)
            .zip(sign_on_object)
            .and_then(|((torque_local, frame), sign)| {
                rotate_contact_vector(torque_local, frame).map(|mut torque| {
                    for component in &mut torque {
                        *component *= sign;
                    }
                    torque
                })
            })
    } else {
        None
    };
    let moment_world_about_object_com_nm = force_world_on_object_n
        .zip(torque_world_on_object_nm)
        .zip(contact.position)
        .zip(object_com_world_m)
        .and_then(|(((force, torque), position), center)| {
            let position = finite_vec3(&position)?;
            let arm = sub3(position, center);
            let mut moment = [
                arm[1] * force[2] - arm[2] * force[1],
                arm[2] * force[0] - arm[0] * force[2],
                arm[0] * force[1] - arm[1] * force[0],
            ];
            for axis in 0..3 {
                moment[axis] += torque[axis];
            }
            moment
                .iter()
                .all(|value| value.is_finite())
                .then_some(moment)
        });

    GraspContactWrench {
        body1: contact.body1.clone(),
        body2: contact.body2.clone(),
        position_world_m: contact.position,
        force_local: contact.force_local,
        torque_local: contact.torque_local,
        contact_frame: contact.contact_frame,
        normal_force_n: contact.normal_force,
        tangential_force_n: contact.tangential_force,
        contact_dimension: contact.contact_dim,
        friction: contact.friction.clone(),
        force_available: contact.force_available,
        force_world_on_object_n,
        torque_world_on_object_nm,
        moment_world_about_object_com_nm,
    }
}

fn rotate_contact_vector(vector: [f64; 3], frame: [f64; 9]) -> Option<[f64; 3]> {
    if vector
        .iter()
        .chain(frame.iter())
        .any(|component| !component.is_finite())
    {
        return None;
    }
    let world = [
        vector[0] * frame[0] + vector[1] * frame[3] + vector[2] * frame[6],
        vector[0] * frame[1] + vector[1] * frame[4] + vector[2] * frame[7],
        vector[0] * frame[2] + vector[1] * frame[5] + vector[2] * frame[8],
    ];
    world
        .iter()
        .all(|component| component.is_finite())
        .then_some(world)
}

fn finite_vec3(values: &[f64]) -> Option<[f64; 3]> {
    let vector: [f64; 3] = values.try_into().ok()?;
    vector
        .iter()
        .all(|value| value.is_finite())
        .then_some(vector)
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

fn joint_control_ramp_steps(
    start_ctrl: &[f64],
    target_ctrl: &[f64],
    actuator_map: &BTreeMap<String, usize>,
    joint_names: &[String],
    timestep_s: f64,
    rate_rad_s: f64,
) -> u32 {
    if !timestep_s.is_finite()
        || timestep_s <= 0.0
        || !rate_rad_s.is_finite()
        || rate_rad_s <= 0.0
        || start_ctrl.len() != target_ctrl.len()
    {
        return MAX_JOINT_CONTROL_RAMP_STEPS;
    }
    let mut maximum_delta_rad = 0.0_f64;
    for joint in joint_names {
        let Some(index) = actuator_map.get(joint).copied() else {
            return MAX_JOINT_CONTROL_RAMP_STEPS;
        };
        let (Some(start), Some(target)) = (start_ctrl.get(index), target_ctrl.get(index)) else {
            return MAX_JOINT_CONTROL_RAMP_STEPS;
        };
        let delta = (target - start).abs();
        if !delta.is_finite() {
            return MAX_JOINT_CONTROL_RAMP_STEPS;
        }
        maximum_delta_rad = maximum_delta_rad.max(delta);
    }
    let requested_steps = (maximum_delta_rad / rate_rad_s / timestep_s).ceil();
    if !requested_steps.is_finite() {
        return MAX_JOINT_CONTROL_RAMP_STEPS;
    }
    (requested_steps as u32).clamp(MIN_JOINT_CONTROL_RAMP_STEPS, MAX_JOINT_CONTROL_RAMP_STEPS)
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

fn observed_frame_quaternion(state: &Value, frame_key: &str) -> Option<[f64; 4]> {
    let values = state["site_xquat"]
        .get(frame_key)
        .or_else(|| state["xquat"].get(frame_key))?
        .as_array()?;
    if values.len() != 4 {
        return None;
    }
    let quaternion = std::array::from_fn(|axis| values[axis].as_f64().unwrap_or(f64::NAN));
    realityos_semantics::transform::Se3::try_new([0.0; 3], quaternion)
        .ok()
        .map(|pose| pose.quat_wxyz)
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
    let tool_contact_extents_m = observed_frame_quaternion(state, &frame_key)
        .and_then(|orientation| {
            declared_ee_push_contact_extents(model, &ee.name, orientation, quat)
        })
        .unwrap_or([tool_radius_m; 4]);
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
            tool_contact_extents_m,
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
            tool_contact_extents_m: [declared_ee_radius(model, &ee.name).unwrap_or(0.0); 4],
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

fn pregrant_action_metadata<'a>(
    executable_action_key: Option<&'a str>,
    authority_request_action_key: Option<&'a str>,
) -> (&'static str, Option<&'a str>) {
    if let Some(action_key) = executable_action_key {
        ("executable", Some(action_key))
    } else if let Some(action_key) = authority_request_action_key {
        ("authority_request", Some(action_key))
    } else {
        ("none", None)
    }
}

/// Probe authorization on a positive scenario that the stratum still marked
/// `ExpectedRefusal`. This is an evaluation-label mismatch, not a physical
/// write and not a proved goal push.
pub fn evaluation_label_mismatch(
    action_key: &str,
    scenario: &ManipulationScenario,
    unauthorized_writes: u64,
    preview_decision: &str,
) -> bool {
    let unlabeled_positive = scenario.polarity == crate::manipulation_scenarios::Polarity::Positive
        && scenario.expected_refusal.is_none()
        && scenario.neg.is_none()
        && !scenario.stale
        && !scenario.move_object_after_obs
        && !scenario.replay
        && !scenario.restart_replay
        && !scenario.crash_controller
        && !scenario.immovable;
    action_key.starts_with("probe:")
        && unlabeled_positive
        && unauthorized_writes == 0
        && preview_decision.contains("SCOPED_GRANT_DOES_NOT_COVER_PROBE")
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

/// Resolve collision support along a world-space push direction using the
/// observed end-effector orientation. `tool_radius_m` remains the conservative
/// clearance bound; this directional support determines intended face contact.
pub(super) fn declared_ee_contact_extent(
    model: &realityos_semantics::embodiment::EmbodimentModel,
    ee_name: &str,
    ee_orientation_world: [f64; 4],
    direction_world: [f64; 3],
) -> Option<f64> {
    realityos_semantics::physical_reasoner::directed_tool_contact_extent(
        model,
        ee_name,
        ee_orientation_world,
        direction_world,
    )
}

fn declared_ee_push_contact_extents(
    model: &realityos_semantics::embodiment::EmbodimentModel,
    ee_name: &str,
    ee_orientation_world: [f64; 4],
    object_quaternion: [f64; 4],
) -> Option<[f64; 4]> {
    let object_rotation =
        realityos_semantics::transform::Se3::try_new([0.0; 3], object_quaternion).ok()?;
    let planar_axis = |local_axis| {
        let world = object_rotation.rotate(local_axis);
        let norm = world[0].hypot(world[1]);
        (norm.is_finite() && norm > 1.0e-9).then_some([world[0] / norm, world[1] / norm, 0.0])
    };
    let axis_x = planar_axis([1.0, 0.0, 0.0])?;
    let axis_y = planar_axis([0.0, 1.0, 0.0])?;
    [
        axis_x,
        [-axis_x[0], -axis_x[1], 0.0],
        axis_y,
        [-axis_y[0], -axis_y[1], 0.0],
    ]
    .map(|direction| declared_ee_contact_extent(model, ee_name, ee_orientation_world, direction))
    .into_iter()
    .collect::<Option<Vec<_>>>()?
    .try_into()
    .ok()
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
        declared_ee_contact_extent, declared_ee_observation_key, declared_ee_push_contact_extents,
        declared_ee_radius, decode_push_witness, injected_safety_fault_alerts,
        joint_control_ramp_steps, observe_probe_motion_sample, observed_frame_quaternion,
        probe_contact_pairs_from_state, probe_sample_tangential_force_lower_bound,
        probe_slip_onset_resolved, sample_is_after_precontact_baseline, PushMotionAudit,
    };
    use crate::bundle::RobotBundle;
    use crate::manipulation_scenarios::{push_scenario, NegKind};
    use crate::semantics_map::embodiment_from_manifest;
    use realityos_semantics::embodiment::{EmbodimentModel, EndEffector, FrameKind, ModelFrame};
    use realityos_semantics::geometry::{
        CollisionRole, PrimitiveShape, RigidGeometry, SemanticRole,
    };
    use realityos_semantics::physical_reasoner::SafetyFaultKind;
    use realityos_semantics::provenance::Provenance;
    use realityos_semantics::provenance::Provenanced;
    use realityos_semantics::transform::Se3;
    use realityos_semantics::{observation::WrenchSample, sensor::WrenchSampleOrigin};

    fn wrench_sample(sequence: u64, capture_s: f64) -> WrenchSample {
        WrenchSample {
            sensor_id: "sim-ft-test".into(),
            sensor_frame: "tool0".into(),
            transform_epoch: "epoch:test".into(),
            units: "N,N*m".into(),
            wrench: [0.0; 6],
            error_bounds: [0.0; 6],
            capture_s,
            receive_s: capture_s,
            sequence,
            clock_domain: "sim:test".into(),
            origin: WrenchSampleOrigin::Simulation,
            digest: String::new(),
        }
    }

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
    }

    #[test]
    fn contact_extent_stops_at_the_flange_while_the_bounding_radius_includes_the_wrist() {
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
            translation: Provenanced::declared([0.0, 0.10, 0.0], "test", 0.0),
            rotation: Provenanced::declared([1.0, 0.0, 0.0, 0.0], "test", 0.0),
        });
        let turn = std::f64::consts::FRAC_1_SQRT_2;
        model.collision_geoms.push(RigidGeometry::declared(
            "flange",
            "wrist",
            Se3::try_new([0.0, 0.08, 0.0], [turn, turn, 0.0, 0.0]).expect("cylinder pose"),
            PrimitiveShape::Cylinder {
                radius: 0.04,
                half_length: 0.02,
            },
            CollisionRole::Collision,
            SemanticRole::Tool,
            "test",
        ));

        let radius = declared_ee_radius(&model, "tool").expect("cylinder bound");
        let aligned_extent =
            declared_ee_contact_extent(&model, "tool", [1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0])
                .expect("cylinder face along the frame axis");
        let orthogonal_extent =
            declared_ee_contact_extent(&model, "tool", [1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0])
                .expect("cylinder cross-section orthogonal to the frame axis");
        assert!(radius > 0.06, "{radius}");
        assert!(aligned_extent < 1.0e-9, "{aligned_extent}");
        assert!(
            (orthogonal_extent - 0.04).abs() < 1.0e-9,
            "{orthogonal_extent}"
        );

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
        assert!(
            declared_ee_contact_extent(&model, "tool", [1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0])
                .is_none()
        );

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
    fn development_embodiment_contact_support_uses_aligned_and_orthogonal_push_directions() {
        use crate::mujoco_exec::{ensure_mujoco_or_skip, MujocoInstance};
        use crate::normalize::RobotManifest;

        if !ensure_mujoco_or_skip() {
            return;
        }
        for (robot_id, bundle_dir) in [("menagerie_ur5e", "ur5e"), ("menagerie_iiwa14", "iiwa14")] {
            let bundle_path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
                .join("../../robots/external")
                .join(bundle_dir);
            let bundle = RobotBundle::load(bundle_path).expect("development robot bundle");
            let mut worker = MujocoInstance::spawn().expect("MuJoCo worker");
            let loaded = worker
                .load_bundle(&bundle, &[], 44)
                .expect("load development embodiment");
            let manifest = RobotManifest::from_inspect(&bundle, &loaded["inspect"]);
            let model = embodiment_from_manifest(&bundle, &manifest);
            let ee = model.end_effectors.first().expect("declared end-effector");
            let frame_key = declared_ee_observation_key(&bundle, &ee.name)
                .expect("observable end-effector frame");
            let state = worker.state().expect("read development embodiment state");
            let state = &state["state"];
            let orientation = observed_frame_quaternion(state, &frame_key)
                .expect("observed end-effector orientation");
            let extents = declared_ee_push_contact_extents(
                &model,
                &ee.name,
                orientation,
                [1.0, 0.0, 0.0, 0.0],
            )
            .expect("all four directed supports");
            let expected = [
                [1.0, 0.0, 0.0],
                [-1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, -1.0, 0.0],
            ]
            .map(|direction| {
                declared_ee_contact_extent(&model, &ee.name, orientation, direction)
                    .expect("directional support")
            });
            for axis in 0..4 {
                assert!(
                    (extents[axis] - expected[axis]).abs() < 1e-12,
                    "{robot_id} push extent {axis} must match its resolved world direction: {extents:?} vs {expected:?}"
                );
            }

            let ee_world = Se3::try_new([0.0; 3], orientation).expect("observed pose");
            let aligned_world = ee_world.rotate([0.0, 0.0, 1.0]);
            let orthogonal_world = ee_world.rotate([1.0, 0.0, 0.0]);
            let aligned_extent =
                declared_ee_contact_extent(&model, &ee.name, orientation, aligned_world)
                    .expect("support along attachment axis");
            let orthogonal_extent =
                declared_ee_contact_extent(&model, &ee.name, orientation, orthogonal_world)
                    .expect("support across attachment axis");
            assert!(
                (aligned_extent - orthogonal_extent).abs() > 1e-4,
                "{robot_id} fixture must prove aligned and orthogonal tool support differ; aligned={aligned_extent}, orthogonal={orthogonal_extent}"
            );
        }
    }

    #[test]
    fn probe_ramp_duration_scales_with_joint_segment_and_keeps_existing_cap() {
        let joints = vec!["joint1".to_string(), "joint2".to_string()];
        let actuator_map =
            std::collections::BTreeMap::from([(joints[0].clone(), 0), (joints[1].clone(), 1)]);

        assert_eq!(
            joint_control_ramp_steps(
                &[0.0, 0.0],
                &[0.02, 0.20],
                &actuator_map,
                &joints,
                0.002,
                super::PUSH_TRANSIT_JOINT_RAMP_RATE_RAD_S,
            ),
            2_500,
            "a 0.2 rad transit segment should use the 0.04 rad/s nominal ramp"
        );
        assert_eq!(
            joint_control_ramp_steps(
                &[0.0, 0.0],
                &[0.02, 0.08],
                &actuator_map,
                &joints,
                0.002,
                super::PUSH_CONTACT_JOINT_RAMP_RATE_RAD_S,
            ),
            6_000,
            "contact motion uses the slower 0.005 rad/s nominal joint ramp and existing step ceiling"
        );
        assert_eq!(
            joint_control_ramp_steps(
                &[0.0, 0.0],
                &[0.02, 0.20],
                &actuator_map,
                &joints,
                0.002,
                super::PUSH_CONTACT_JOINT_RAMP_RATE_RAD_S,
            ),
            6_000,
            "long contact segments keep the existing 6,000-step ceiling"
        );
        assert_eq!(
            joint_control_ramp_steps(
                &[0.0, 0.0],
                &[0.02, 2.0],
                &actuator_map,
                &joints,
                0.002,
                super::PUSH_TRANSIT_JOINT_RAMP_RATE_RAD_S,
            ),
            6_000,
            "long segments keep the existing 6,000-step ceiling"
        );
        assert_eq!(
            joint_control_ramp_steps(
                &[0.0, 0.0],
                &[0.0, 0.0],
                &actuator_map,
                &joints,
                0.002,
                super::PUSH_TRANSIT_JOINT_RAMP_RATE_RAD_S,
            ),
            100,
            "stationary targets retain a minimum monitored control window"
        );
    }

    #[test]
    fn probe_inference_window_excludes_samples_at_or_before_its_baseline() {
        let baseline = wrench_sample(100, 1.0);
        assert!(!sample_is_after_precontact_baseline(
            &wrench_sample(99, 0.992),
            Some(&baseline)
        ));
        assert!(!sample_is_after_precontact_baseline(
            &wrench_sample(100, 1.0),
            Some(&baseline)
        ));
        assert!(!sample_is_after_precontact_baseline(
            &wrench_sample(101, 0.999),
            Some(&baseline)
        ));
        assert!(sample_is_after_precontact_baseline(
            &wrench_sample(101, 1.008),
            Some(&baseline)
        ));
        assert!(sample_is_after_precontact_baseline(
            &wrench_sample(99, 0.992),
            None
        ));
    }

    #[test]
    fn probe_motion_requires_a_sensor_resolved_planar_contact_load() {
        let mut sample = wrench_sample(101, 1.0);
        sample.wrench = [2.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        sample.error_bounds = [0.05; 6];
        let force = probe_sample_tangential_force_lower_bound(
            &sample,
            [0.0; 6],
            [1.0, 0.0, 0.0, 0.0],
            [0.0; 6],
            [0.05; 6],
            [250.0; 6],
            0.001,
        )
        .expect("fresh sensor-supported load");
        assert!((force - 1.65).abs() < 1e-12);

        sample.wrench[0] = 0.5;
        let weak = probe_sample_tangential_force_lower_bound(
            &sample,
            [0.0; 6],
            [1.0, 0.0, 0.0, 0.0],
            [0.0; 6],
            [0.05; 6],
            [250.0; 6],
            0.002,
        )
        .expect("fresh but weak sensor load");
        assert_eq!(weak, 0.0, "uncertainty must cover an unconfirmed force");
    }

    #[test]
    fn probe_motion_ignores_submillimeter_contact_compliance() {
        assert!(!probe_slip_onset_resolved([0.0009, 0.0, 0.0], Some(3.0)));
        assert!(!probe_slip_onset_resolved([0.0011, 0.0, 0.0], Some(0.49)));
        assert!(probe_slip_onset_resolved([0.0011, 0.0, 0.0], Some(0.5)));
        assert!(!probe_slip_onset_resolved([0.01, 0.0, 0.0], Some(f64::NAN)));
    }

    #[test]
    fn gradual_motion_below_force_gate_is_retained_as_unresolved() {
        let mut motion = PushMotionAudit::default();
        motion.started = true;
        observe_probe_motion_sample(&mut motion, [0.0006, 0.0, 0.0], 1.002, Some(0.49));
        observe_probe_motion_sample(&mut motion, [0.0012, 0.0, 0.0], 1.004, Some(0.49));
        observe_probe_motion_sample(&mut motion, [0.0018, 0.0, 0.0], 1.006, Some(0.49));
        observe_probe_motion_sample(&mut motion, [0.0024, 0.0, 0.0], 1.008, Some(0.8));

        assert!(motion.unresolved_motion_before_force);
        assert_eq!(motion.start_position, [0.0; 3]);
        assert_eq!(motion.end_position, [0.0024, 0.0, 0.0]);
        assert_eq!(motion.first_motion_s, None);
    }

    #[test]
    fn probe_start_contacts_preserve_body_and_geometry_identities() {
        let contacts = probe_contact_pairs_from_state(&serde_json::json!({
            "contacts": [{
                "body1": "obj0",
                "geom1_name": "object_geom",
                "body2": "table",
                "geom2_name": "table_geom"
            }]
        }))
        .expect("complete start-state contact identities");

        assert_eq!(contacts.len(), 1);
        assert!(contacts[0].contains_bodies("table", "obj0"));
        assert_eq!(contacts[0].geom1_name, "object_geom");
    }

    #[test]
    fn force_resolved_motion_records_onset_without_unresolved_flag() {
        let mut motion = PushMotionAudit::default();
        motion.started = true;
        observe_probe_motion_sample(&mut motion, [0.0011, 0.0, 0.0], 2.0, Some(0.5));

        assert_eq!(motion.first_motion_s, Some(2.0));
        assert!(!motion.unresolved_motion_before_force);
        assert_eq!(motion.start_position, [0.0; 3]);
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

#[cfg(test)]
mod audit_summary_tests {
    use super::{
        record_push_wrench_window, update_wrench_episode_audit, BenchmarkEpisode, PushMotionAudit,
        PushWitnessAudit, WitnessExecution,
    };
    use crate::manipulation_benchmark::{
        scenario::EmbodimentRegistry, wrench_sensor::SimulationWrenchWindow,
    };
    use realityos_semantics::observation::{wrench_sample_digest, WrenchSample};
    use realityos_semantics::sensor::WrenchSampleOrigin;
    use std::path::PathBuf;

    fn iiwa_profile() -> super::super::wrench_sensor::SimulationWrenchSensorProfile {
        let registry_path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../robots/benchmarks/manipulation/embodiments.json");
        EmbodimentRegistry::load(&registry_path)
            .expect("checked-in embodiment registry")
            .registrations
            .into_iter()
            .find(|registration| registration.robot_id == "menagerie_iiwa14")
            .and_then(|registration| registration.simulation_wrench_profile)
            .expect("registered IIWA wrench profile")
    }

    fn sample(
        profile: &super::super::wrench_sensor::SimulationWrenchSensorProfile,
        sequence: u64,
        capture_s: f64,
        receive_delay_s: f64,
        force_x_n: f64,
    ) -> WrenchSample {
        let mut sample = WrenchSample {
            sensor_id: profile.sensor_id.clone(),
            sensor_frame: profile.sensor_frame.clone(),
            transform_epoch: "audit-test-epoch".into(),
            units: "N,N*m".into(),
            wrench: [force_x_n, 0.0, 0.0, 0.0, 0.0, 0.0],
            error_bounds: profile.error_bounds,
            capture_s,
            receive_s: capture_s + receive_delay_s,
            sequence,
            clock_domain: "mujoco_simulation_clock".into(),
            origin: WrenchSampleOrigin::Simulation,
            digest: String::new(),
        };
        sample.digest = wrench_sample_digest(&sample);
        sample
    }

    fn audit(samples: Vec<WrenchSample>) -> PushWitnessAudit {
        let count = samples.len();
        PushWitnessAudit {
            execution: WitnessExecution::Wrote,
            samples,
            sample_free_space_wrenches: Vec::new(),
            sample_free_space_wrench_error_bounds: Vec::new(),
            sensor_positions: vec![[0.0; 3]; count],
            sensor_orientations: vec![[1.0, 0.0, 0.0, 0.0]; count],
            object_pose_trace: Vec::new(),
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
            sample_gravity_reference_indices: vec![None; count],
            abort: None,
            motion: PushMotionAudit::default(),
            control_correction_max_rad: 0.0,
        }
    }

    #[test]
    fn window_reference_only_applies_after_sample_captured_and_received_it() {
        let profile = iiwa_profile();
        let reference = sample(&profile, 145, 0.292, 0.008, 0.0);
        let samples = vec![
            sample(&profile, 143, 0.288, 0.002, 1.0),
            sample(&profile, 144, 0.290, 0.002, 1.0),
            reference.clone(),
            sample(&profile, 146, 0.294, 0.002, 1.0),
            sample(&profile, 147, 0.296, 0.002, 1.0),
            sample(&profile, 149, 0.300, 0.002, 1.0),
        ];
        let window = SimulationWrenchWindow {
            samples,
            sample_free_space_wrenches: Vec::new(),
            sample_free_space_wrench_error_bounds: Vec::new(),
            sensor_positions: vec![[0.0; 3]; 6],
            sensor_orientations: vec![[1.0, 0.0, 0.0, 0.0]; 6],
            object_pose_trace: Vec::new(),
            precontact_baseline: None,
            precontact_baseline_status: None,
            precontact_baseline_position: None,
            precontact_baseline_orientation: None,
            triggering_sample: None,
            monitor_gravity_reference: Some(reference),
            monitor_gravity_reference_status: Some("captured".into()),
            monitor_gravity_reference_position: Some([0.0; 3]),
            monitor_gravity_reference_orientation: Some([1.0, 0.0, 0.0, 0.0]),
            max_gap_s: 0.002,
            compensation_digest: profile.limit_policy.compensation_digest,
        };
        let mut audit = audit(Vec::new());

        record_push_wrench_window(&mut audit, window);

        assert_eq!(
            audit.sample_gravity_reference_indices,
            vec![None, None, Some(0), None, None, Some(0)]
        );
    }

    #[test]
    fn episode_wrench_summary_accumulates_worst_values_across_actions() {
        let record: serde_json::Value = serde_json::from_slice(include_bytes!(
            "../../../../docs/superpowers/evidence/build-profile-replay-ac09b54/optimized/trials/v7_development_menagerie_iiwa14_push_probe_replan_21000_0.json"
        ))
        .expect("preserved replay record");
        let mut episode: BenchmarkEpisode =
            serde_json::from_value(record["episode"].clone()).expect("preserved episode");
        episode.wrench_sample_count = 0;
        episode.wrench_max_sample_age_s = None;
        episode.wrench_max_inter_sample_gap_s = None;
        episode.wrench_peak_conservative_bounds = None;
        episode.wrench_minimum_margin = None;
        episode.wrench_abort_result = None;

        let profile = iiwa_profile();
        let first = audit(vec![
            sample(&profile, 100, 0.200, 0.020, 15.0),
            sample(&profile, 101, 0.220, 0.000, 2.0),
        ]);
        update_wrench_episode_audit(&mut episode, &first, &profile);
        let first_peak = episode.wrench_peak_conservative_bounds.expect("first peak")[0];
        let first_margin = episode.wrench_minimum_margin.expect("first margin");
        let first_age = episode.wrench_max_sample_age_s.expect("first age");
        let first_gap = episode.wrench_max_inter_sample_gap_s.expect("first gap");

        let second = audit(vec![
            sample(&profile, 102, 0.240, 0.004, 1.0),
            sample(&profile, 103, 0.241, 0.000, 0.5),
        ]);
        update_wrench_episode_audit(&mut episode, &second, &profile);

        assert_eq!(episode.wrench_sample_count, 4);
        assert!(
            episode
                .wrench_peak_conservative_bounds
                .expect("episode peak")[0]
                >= first_peak
        );
        assert!(episode.wrench_minimum_margin.expect("episode margin") <= first_margin);
        assert!(episode.wrench_max_sample_age_s.expect("episode age") >= first_age);
        assert!(episode.wrench_max_inter_sample_gap_s.expect("episode gap") >= first_gap);
    }
}

#[cfg(test)]
mod pregrant_action_audit_tests {
    use super::pregrant_action_metadata;

    #[test]
    fn executable_step_is_recorded_before_a_pending_authority_request() {
        assert_eq!(
            pregrant_action_metadata(Some("push:+x"), Some("probe:+x")),
            ("executable", Some("push:+x"))
        );
    }

    #[test]
    fn pending_probe_is_recorded_as_an_authority_request() {
        assert_eq!(
            pregrant_action_metadata(None, Some("probe:+x")),
            ("authority_request", Some("probe:+x"))
        );
    }

    #[test]
    fn missing_pregrant_action_is_recorded_explicitly() {
        assert_eq!(pregrant_action_metadata(None, None), ("none", None));
    }
}

#[cfg(test)]
mod replay_initial_observation_evidence_tests {
    use super::ReplayInitialObservationEvidenceV1;
    use serde_json::json;

    #[test]
    fn replay_observation_record_hashes_exact_json_bytes_and_binds_time() {
        let observation = json!({
            "time": 0.125,
            "nan": false,
            "qpos": [0.1, -0.2],
        });
        let evidence = ReplayInitialObservationEvidenceV1::capture(&observation)
            .expect("capture valid observation");
        evidence.verify().expect("evidence verifies");
        assert_eq!(
            evidence.observation_json,
            serde_json::to_string(&observation).expect("compact JSON")
        );

        let mut wrong_time = evidence.clone();
        wrong_time.simulator_time_s = 0.25;
        assert!(wrong_time.verify().is_err());

        let mut changed_json_bytes = evidence;
        changed_json_bytes.observation_json.push(' ');
        assert!(changed_json_bytes.verify().is_err());
    }
}

#[cfg(test)]
mod grasp_lift_witness_path_tests {
    use super::grasp_lift_states_match;

    #[test]
    fn accepts_all_collision_checked_waypoints_after_the_contact_start() {
        let fingers = vec![0.0, 0.0];
        let path = vec![
            (vec![0.0, 0.0], fingers.clone()),
            (vec![0.001, 0.0], fingers.clone()),
            (vec![0.002, 0.0], fingers.clone()),
        ];

        assert!(grasp_lift_states_match(&path, &[0.0, 0.0], &fingers));

        let mut wrong_start = path.clone();
        wrong_start[0].0[0] = 0.0001;
        assert!(!grasp_lift_states_match(
            &wrong_start,
            &[0.0, 0.0],
            &fingers
        ));

        let mut opened_during_lift = path;
        opened_during_lift[1].1[0] = 0.001;
        assert!(!grasp_lift_states_match(
            &opened_during_lift,
            &[0.0, 0.0],
            &fingers
        ));
    }
}

#[cfg(test)]
mod grasp_contact_wrench_tests {
    use super::{grasp_contact_audits, scenario_objects_with_grasp_friction, GraspContactAudit};
    use crate::observation::{ContactTruth, VerifierTruth};
    use realityos_semantics::provenance::Provenanced;
    use realityos_semantics::resource::{
        ClosingDirection, ControlledResource, CouplingModel, QualificationStatus, ResourceKind,
        ResourceTopology,
    };
    use serde_json::{json, Value};

    fn fixture_resource() -> ControlledResource {
        ControlledResource {
            id: "test_gripper".into(),
            kind: ResourceKind::Gripper,
            topology: ResourceTopology::DirectJointGripper,
            actuator_inputs: Vec::new(),
            affected_joints: Vec::new(),
            finger_bodies: vec!["finger_l".into(), "finger_r".into()],
            coupling: CouplingModel::none(),
            command_coordinate: "opening".into(),
            opening_range: Provenanced::declared([0.0, 1.0], "test", 0.0),
            command_range: Provenanced::declared([0.0, 1.0], "test", 0.0),
            closing_direction: ClosingDirection::TowardMin,
            force_bound: Provenanced::unknown("test", 0.0),
            qualification: QualificationStatus::Qualified,
            unsupported_detail: None,
        }
    }

    #[test]
    fn worker_friction_mapping_marks_only_the_target_for_the_selected_gripper() {
        let objects = vec![
            json!({"name":"table", "friction":0.8}),
            json!({"name":"obj0", "friction":0.4}),
        ];
        let prepared =
            scenario_objects_with_grasp_friction(&objects, "obj0", Some(&fixture_resource()))
                .expect("target friction mapping");

        assert!(prepared[0].get("tool_object_friction_bodies").is_none());
        assert_eq!(
            prepared[1]["tool_object_friction_bodies"],
            json!(["finger_l", "finger_r"])
        );
        assert_eq!(objects[1].get("tool_object_friction_bodies"), None);
    }

    #[test]
    fn worker_friction_mapping_fails_closed_without_a_selected_gripper() {
        let objects = vec![json!({"name":"obj0", "friction":0.4})];
        assert_eq!(
            scenario_objects_with_grasp_friction(&objects, "obj0", None)
                .expect_err("friction pair needs finger bodies"),
            "grasp friction target has no selected gripper resource"
        );
    }

    fn audit_json(state: &Value) -> Vec<Value> {
        let truth = VerifierTruth::from_mujoco_state(state);
        grasp_contact_audits("post_close", &truth, "obj0", &fixture_resource())
            .into_iter()
            .map(|audit| serde_json::to_value(audit).expect("audit serializes"))
            .collect()
    }

    #[test]
    fn grasp_contact_audits_preserve_sample_time_on_each_finger_row() {
        let audits = audit_json(&json!({
            "time": 12.5,
            "time_observed_s": 12.5,
            "xipos": {"obj0": [0.0, 0.0, 0.0]}
        }));

        assert_eq!(audits.len(), 2);
        assert_eq!(audits[0]["sample_time_s"], json!(12.5));
        assert_eq!(audits[1]["sample_time_s"], json!(12.5));
    }

    #[test]
    fn grasp_contact_audits_do_not_report_fallback_time_as_observed() {
        for state in [
            json!({"xipos": {"obj0": [0.0, 0.0, 0.0]}}),
            json!({"time": null, "xipos": {"obj0": [0.0, 0.0, 0.0]}}),
            json!({"time": "12.5", "xipos": {"obj0": [0.0, 0.0, 0.0]}}),
            json!({
                "time": 12.5,
                "time_observed_s": "12.5",
                "xipos": {"obj0": [0.0, 0.0, 0.0]}
            }),
        ] {
            let audits = audit_json(&state);
            assert_eq!(audits.len(), 2);
            assert!(audits.iter().all(|audit| audit["sample_time_s"].is_null()));
        }

        let zero_time = audit_json(&json!({
            "time": 0.0,
            "time_observed_s": 0.0,
            "xipos": {"obj0": [0.0, 0.0, 0.0]}
        }));
        assert_eq!(zero_time[0]["sample_time_s"], json!(0.0));

        let unavailable_source_time = audit_json(&json!({
            "time": 0.0,
            "time_observed_s": null,
            "xipos": {"obj0": [0.0, 0.0, 0.0]}
        }));
        assert!(unavailable_source_time
            .iter()
            .all(|audit| audit["sample_time_s"].is_null()));
    }

    #[test]
    fn grasp_contact_audits_keep_signed_world_wrench() {
        let object_first = audit_json(&json!({
            "xipos": {"obj0": [0.0, 0.0, 0.0]},
            "ctrl": [0.3, -0.4],
            "actuator_force": [1.2, -2.4],
            "contacts": [{
                "body1": "obj0",
                "body2": "finger_l",
                "pos": [0.0, 1.0, 0.0],
                "dim": 6,
                "friction": [0.5, 0.6, 0.01, 0.02, 0.03],
                "force_local": [2.0, 0.4, -0.2],
                "torque_local": [0.1, 0.2, 0.3],
                "frame": [0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0],
                "normal_force": 2.0,
                "tangential_force": 0.4472135954999579,
                "force_available": true
            }]
        }));
        let left = object_first
            .iter()
            .find(|audit| audit["finger_body"] == "finger_l")
            .expect("left finger audit exists");
        assert_eq!(left["object_com_world_m"], json!([0.0, 0.0, 0.0]));
        assert_eq!(left["ctrl"], json!([0.3, -0.4]));
        assert_eq!(left["actuator_force"], json!([1.2, -2.4]));
        assert_eq!(left["contact_wrenches"].as_array().map(Vec::len), Some(1));
        assert_eq!(
            left["contact_wrenches"][0]["force_world_on_object_n"],
            json!([0.4, -2.0, 0.2])
        );
        assert_eq!(
            left["contact_wrenches"][0]["moment_world_about_object_com_nm"],
            json!([0.4, -0.1, -0.7])
        );

        let object_second = audit_json(&json!({
            "xipos": {"obj0": [0.0, 0.0, 0.0]},
            "contacts": [{
                "body1": "finger_l",
                "body2": "obj0",
                "pos": [0.0, 1.0, 0.0],
                "dim": 3,
                "friction": [0.5, 0.6, 0.0, 0.0, 0.0],
                "force_local": [2.0, 0.0, 0.0],
                "torque_local": [0.0, 0.0, 0.0],
                "frame": [-1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0],
                "normal_force": 2.0,
                "tangential_force": 0.0,
                "force_available": true
            }]
        }));
        let left = object_second
            .iter()
            .find(|audit| audit["finger_body"] == "finger_l")
            .expect("left finger audit exists");
        assert_eq!(
            left["contact_wrenches"][0]["force_world_on_object_n"],
            json!([-2.0, 0.0, 0.0])
        );
    }

    #[test]
    fn grasp_contact_audits_omit_derived_wrench_without_finite_inputs() {
        let truth = VerifierTruth {
            xipos: [("obj0".into(), vec![0.0, 0.0, 0.0])].into(),
            contacts: vec![
                ContactTruth {
                    body1: "obj0".into(),
                    body2: "finger_l".into(),
                    position: None,
                    force_local: Some([1.0, 0.0, 0.0]),
                    torque_local: Some([0.0, 0.0, 0.0]),
                    contact_frame: Some([1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]),
                    force_available: true,
                    ..ContactTruth::default()
                },
                ContactTruth {
                    body1: "obj0".into(),
                    body2: "finger_l".into(),
                    position: Some([0.0, 0.0, 0.0]),
                    force_local: Some([f64::NAN, 0.0, 0.0]),
                    torque_local: Some([0.0, 0.0, 0.0]),
                    contact_frame: Some([1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]),
                    force_available: true,
                    ..ContactTruth::default()
                },
            ],
            ..VerifierTruth::default()
        };
        let audits = grasp_contact_audits("post_close", &truth, "obj0", &fixture_resource());
        let left = audits
            .iter()
            .find(|audit| audit.finger_body == "finger_l")
            .expect("left finger audit exists");
        let left = serde_json::to_value(left).expect("audit serializes");
        assert_eq!(left["contact_wrenches"].as_array().map(Vec::len), Some(2));
        assert_eq!(
            left["contact_wrenches"][0]["force_world_on_object_n"],
            json!([-1.0, 0.0, 0.0])
        );
        assert!(left["contact_wrenches"][0]["moment_world_about_object_com_nm"].is_null());
        assert!(left["contact_wrenches"][1]["force_world_on_object_n"].is_null());
        assert!(left["contact_wrenches"][1]["moment_world_about_object_com_nm"].is_null());
    }

    #[test]
    fn legacy_grasp_contact_audit_deserializes_with_empty_trace_defaults() {
        let legacy = json!({
            "phase": "lift_waypoint_1",
            "finger_body": "finger_l",
            "normal_force_n": 1.0,
            "tangential_force_n": 0.25,
            "object_vertical_force_n": -0.5,
            "contact_dimensions": [3],
            "contact_friction": [[0.5, 0.6, 0.0, 0.0, 0.0]],
            "force_available": true
        });
        let audit: GraspContactAudit =
            serde_json::from_value(legacy).expect("legacy audit remains readable");
        assert_eq!(audit.sample_time_s, None);
        let audit = serde_json::to_value(audit).expect("legacy audit serializes");
        assert!(audit["sample_time_s"].is_null());
        assert_eq!(audit["contact_wrenches"], json!([]));
        assert_eq!(audit["ctrl"], json!([]));
        assert_eq!(audit["actuator_force"], json!([]));
        assert!(audit["object_com_world_m"].is_null());
    }
}
