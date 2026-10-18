use super::wrench_sensor::SimulationWrenchSensorProfile;
use super::BenchmarkError;
use crate::bundle::RobotBundle;
use crate::manipulation_scenarios::{
    arm_is_planar, grasp_scenario, push_scenario, ManipulationScenario, NegKind, Polarity,
};
use crate::normalize::RobotManifest;
use crate::semantics_map::embodiment_from_manifest;
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};
use realityos_semantics::allowed_contact::ContactPhase;
use realityos_semantics::contact_collision::{
    policy_from_scene_model, scene_from_collision_world, CollisionWorld, NamedBox,
};
use realityos_semantics::physical_reasoner::PhysicalReasoner;
use realityos_semantics::transition_validity::{validate_transition, GeometryVerdict};
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};
use sha2::{Digest, Sha256};

pub const BENCHMARK_SCHEMA_VERSION: u32 = 7;
const LEGACY_PAIRED_SCHEMA_VERSION: u32 = 6;
const EMBODIMENT_REGISTRY_SCHEMA_VERSION: u32 = 5;
const MAX_SAFE_INITIAL_QPOS_ATTEMPTS: usize = 64;
const MIN_POSITIVE_JOINT_MARGIN_RAD: f64 = 0.1;
pub const PILOT_SEEDS: std::ops::Range<u64> = 19000..19020;
pub const PUBLIC_SEEDS: std::ops::Range<u64> = 20000..20100;
pub const HOLDOUT_SEEDS: std::ops::Range<u64> = 10000..10100;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PushTrialStratum {
    ExpectedSuccess,
    ExpectedRefusal,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PositivePushRandomization {
    pub object_x_m: [f64; 2],
    pub object_y_m: [f64; 2],
    pub half_size_m: [f64; 2],
    pub half_height_m: [f64; 2],
    pub mass_kg: [f64; 2],
    pub friction: [f64; 2],
    pub push_distance_m: [f64; 2],
    pub obstacle_count: [u32; 2],
    pub obstacle_forward_m: [f64; 2],
    pub obstacle_lateral_m: [f64; 2],
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct UnidentifiablePushRandomization {
    pub mass_kg: [f64; 2],
    pub friction: [f64; 2],
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ScenarioSeedRange {
    pub name: String,
    pub start_inclusive: u64,
    pub end_exclusive: u64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ScenarioSuiteManifest {
    pub schema_version: u32,
    pub suite_id: String,
    pub suite_version: u32,
    pub pilot_seeds: Vec<u64>,
    pub public_seeds: Vec<u64>,
    pub historical_seed_ranges: Vec<ScenarioSeedRange>,
    pub stratum_period: u32,
    pub expected_success_residues: Vec<u32>,
    pub push_approach_gap_m: f64,
    pub push_probe_stroke_m: f64,
    pub minimum_post_contact_stroke_m: f64,
    pub max_positive_resamples: u32,
    pub positive_push: PositivePushRandomization,
    #[serde(default)]
    pub unidentifiable_push: Option<UnidentifiablePushRandomization>,
    pub refusal_cycle: Vec<String>,
}

impl ScenarioSuiteManifest {
    pub fn load(path: &std::path::Path) -> Result<Self, BenchmarkError> {
        let bytes = std::fs::read(path).map_err(|error| {
            BenchmarkError::InvalidManifest(format!(
                "read scenario suite {}: {error}",
                path.display()
            ))
        })?;
        let suite: Self = serde_json::from_slice(&bytes)?;
        suite.validate()?;
        Ok(suite)
    }

    pub fn validate(&self) -> Result<(), BenchmarkError> {
        let invalid = |message: &str| BenchmarkError::InvalidManifest(message.into());
        if self.schema_version != 1
            || self.suite_id.trim().is_empty()
            || !matches!(self.suite_version, 2 | 3)
        {
            return Err(invalid("unsupported or empty scenario suite identity"));
        }
        if self.pilot_seeds.len() != 20 || self.public_seeds.len() != 100 {
            return Err(invalid(
                "scenario suite requires exactly 20 pilot and 100 public seeds",
            ));
        }
        let mut scheduled = std::collections::BTreeSet::new();
        for seed in self.pilot_seeds.iter().chain(&self.public_seeds) {
            if !scheduled.insert(*seed) {
                return Err(invalid(
                    "pilot and public seed lists overlap or contain duplicates",
                ));
            }
        }
        for range in &self.historical_seed_ranges {
            if range.name.trim().is_empty() || range.start_inclusive >= range.end_exclusive {
                return Err(invalid("historical seed range is empty or invalid"));
            }
            if scheduled
                .iter()
                .any(|seed| *seed >= range.start_inclusive && *seed < range.end_exclusive)
            {
                return Err(invalid(
                    "new scenario suite seeds overlap a historical seed range",
                ));
            }
        }
        if self.stratum_period != 5
            || self.expected_success_residues.len() != 3
            || self
                .expected_success_residues
                .iter()
                .any(|residue| *residue >= self.stratum_period)
            || self
                .expected_success_residues
                .iter()
                .collect::<std::collections::BTreeSet<_>>()
                .len()
                != self.expected_success_residues.len()
        {
            return Err(invalid(
                "scenario suite must declare a unique 3-in-5 success schedule",
            ));
        }
        if !self.push_approach_gap_m.is_finite()
            || !self.push_probe_stroke_m.is_finite()
            || !self.minimum_post_contact_stroke_m.is_finite()
            || self.push_approach_gap_m <= 0.0
            || self.minimum_post_contact_stroke_m <= 0.0
            || self.push_approach_gap_m + self.minimum_post_contact_stroke_m
                > self.push_probe_stroke_m + 1e-12
            || (self.push_approach_gap_m
                - realityos_semantics::physical_reasoner::PUSH_PROBE_APPROACH_GAP_M)
                .abs()
                > 1e-12
        {
            return Err(invalid(
                "push probe gap must match the shared planner constant and retain the declared post-contact stroke",
            ));
        }
        if self.max_positive_resamples == 0 || self.refusal_cycle.is_empty() {
            return Err(invalid(
                "scenario suite requires resampling attempts and refusal strata",
            ));
        }
        let positive = &self.positive_push;
        for range in [
            positive.object_x_m,
            positive.object_y_m,
            positive.half_size_m,
            positive.half_height_m,
            positive.mass_kg,
            positive.friction,
            positive.push_distance_m,
            positive.obstacle_forward_m,
            positive.obstacle_lateral_m,
        ] {
            if range.iter().any(|value| !value.is_finite()) || range[0] >= range[1] {
                return Err(invalid("positive scenario range is non-finite or empty"));
            }
        }
        if positive.half_size_m[0] <= 0.0
            || positive.half_height_m[0] <= 0.0
            || positive.mass_kg[0] <= 0.0
            || positive.friction[0] < 0.0
            || positive.push_distance_m[0] <= 0.0
            || positive.obstacle_count[0] > positive.obstacle_count[1]
            || positive.obstacle_lateral_m[0] <= 0.0
        {
            return Err(invalid("positive scenario physical bounds are invalid"));
        }
        if self.suite_version == 3
            && self
                .refusal_cycle
                .iter()
                .any(|label| label == "unidentifiable_friction")
            && self.unidentifiable_push.is_none()
        {
            return Err(invalid(
                "v3 unidentifiable-friction refusal stratum needs explicit randomized bounds",
            ));
        }
        if let Some(unidentifiable) = &self.unidentifiable_push {
            for range in [unidentifiable.mass_kg, unidentifiable.friction] {
                if range.iter().any(|value| !value.is_finite()) || range[0] >= range[1] {
                    return Err(invalid(
                        "unidentifiable-refusal physical range is non-finite or empty",
                    ));
                }
            }
            if unidentifiable.mass_kg[0] <= 0.0 || unidentifiable.friction[0] < 0.0 {
                return Err(invalid(
                    "unidentifiable-refusal physical bounds are invalid",
                ));
            }
        }
        Ok(())
    }

    pub fn stratum_for_index(&self, index: u32) -> PushTrialStratum {
        let residue = index % self.stratum_period;
        if self.expected_success_residues.contains(&residue) {
            PushTrialStratum::ExpectedSuccess
        } else {
            PushTrialStratum::ExpectedRefusal
        }
    }

    pub fn digest(&self) -> Result<String, BenchmarkError> {
        let bytes = serde_json::to_vec(self)?;
        Ok(hex::encode(Sha256::digest(bytes)))
    }
}

#[derive(Debug, Clone, PartialEq)]
pub(super) struct SimulatedFrictionOracleInput {
    pub samples: Vec<f64>,
    pub interval: [f64; 2],
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BenchmarkTask {
    PushProbeReplan,
    PinchGraspLift,
}

impl BenchmarkTask {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::PushProbeReplan => "push_probe_replan",
            Self::PinchGraspLift => "pinch_grasp_lift",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TrialExpectation {
    ExpectedSuccess,
    ExpectedRefusal,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EmbodimentRole {
    Development,
    Holdout,
    GraspFixture,
}

impl EmbodimentRole {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Development => "development",
            Self::Holdout => "holdout",
            Self::GraspFixture => "grasp_fixture",
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BenchmarkConfig {
    pub schema_version: u32,
    pub pilot_seeds: Vec<u64>,
    pub public_seeds: Vec<u64>,
    pub holdout_seeds: Vec<u64>,
    pub target_ci_half_width: f64,
    /// Full scheduled denominator. Missing records are reported as unaccounted.
    #[serde(default)]
    pub scheduled_trial_ids: Vec<String>,
    #[serde(default)]
    pub embodiment_registry_sha256: Option<String>,
    #[serde(default)]
    pub scenario_suite_id: Option<String>,
    #[serde(default)]
    pub scenario_suite_version: Option<u32>,
    #[serde(default)]
    pub scenario_suite_sha256: Option<String>,
}

impl Default for BenchmarkConfig {
    fn default() -> Self {
        Self {
            schema_version: BENCHMARK_SCHEMA_VERSION,
            pilot_seeds: PILOT_SEEDS.collect(),
            public_seeds: PUBLIC_SEEDS.collect(),
            holdout_seeds: HOLDOUT_SEEDS.collect(),
            target_ci_half_width: 0.10,
            scheduled_trial_ids: Vec::new(),
            embodiment_registry_sha256: None,
            scenario_suite_id: None,
            scenario_suite_version: None,
            scenario_suite_sha256: None,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct EmbodimentRegistration {
    pub robot_id: String,
    pub bundle_path: String,
    pub role: EmbodimentRole,
    pub tasks: Vec<BenchmarkTask>,
    #[serde(default)]
    pub simulation_wrench_profile: Option<SimulationWrenchSensorProfile>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct EmbodimentRegistry {
    pub schema_version: u32,
    pub registrations: Vec<EmbodimentRegistration>,
}

impl EmbodimentRegistry {
    pub fn load(path: &std::path::Path) -> Result<Self, BenchmarkError> {
        let bytes = std::fs::read(path).map_err(|error| {
            BenchmarkError::InvalidManifest(format!(
                "read embodiment registry {}: {error}",
                path.display()
            ))
        })?;
        let registry: Self = serde_json::from_slice(&bytes)?;
        registry.validate()?;
        Ok(registry)
    }

    pub fn validate(&self) -> Result<(), BenchmarkError> {
        if self.schema_version != EMBODIMENT_REGISTRY_SCHEMA_VERSION {
            return Err(BenchmarkError::InvalidManifest(format!(
                "unsupported embodiment registry schema {}",
                self.schema_version
            )));
        }
        let mut robot_ids = std::collections::BTreeSet::new();
        for entry in &self.registrations {
            if entry.robot_id.trim().is_empty()
                || entry.bundle_path.trim().is_empty()
                || !robot_ids.insert(entry.robot_id.as_str())
                || entry.tasks.is_empty()
            {
                return Err(BenchmarkError::InvalidManifest(
                    "embodiment registry has an empty or duplicate registration".into(),
                ));
            }
            let mut tasks = std::collections::BTreeSet::new();
            for task in &entry.tasks {
                if !tasks.insert(task.as_str()) {
                    return Err(BenchmarkError::InvalidManifest(format!(
                        "duplicate task {} for {}",
                        task.as_str(),
                        entry.robot_id
                    )));
                }
            }
            let supported_task = match entry.role {
                EmbodimentRole::Development | EmbodimentRole::Holdout => {
                    entry.tasks.contains(&BenchmarkTask::PushProbeReplan)
                }
                EmbodimentRole::GraspFixture => {
                    entry.tasks.contains(&BenchmarkTask::PinchGraspLift)
                }
            };
            if !supported_task {
                return Err(BenchmarkError::InvalidManifest(format!(
                    "role {} has no registered benchmark task for {}",
                    entry.role.as_str(),
                    entry.robot_id
                )));
            }
            let has_push_probe = entry.tasks.contains(&BenchmarkTask::PushProbeReplan);
            match (&entry.simulation_wrench_profile, has_push_probe) {
                (Some(profile), true) => profile.validate().map_err(|error| {
                    BenchmarkError::InvalidManifest(format!(
                        "invalid simulation wrench profile for {}: {error}",
                        entry.robot_id
                    ))
                })?,
                (None, true) => {
                    return Err(BenchmarkError::InvalidManifest(format!(
                    "push/probe embodiment {} is missing its explicit simulation wrench profile",
                    entry.robot_id
                )))
                }
                (Some(_), false) => {
                    return Err(BenchmarkError::InvalidManifest(format!(
                        "non-push embodiment {} must not register a push/probe wrench profile",
                        entry.robot_id
                    )))
                }
                (None, false) => {}
            }
        }
        let development = self
            .registrations
            .iter()
            .filter(|entry| entry.role == EmbodimentRole::Development)
            .count();
        let holdout = self
            .registrations
            .iter()
            .filter(|entry| entry.role == EmbodimentRole::Holdout)
            .count();
        if development < 2 || holdout != 1 {
            return Err(BenchmarkError::InvalidManifest(format!(
                "registry requires at least two development robots and exactly one holdout, found {development} and {holdout}"
            )));
        }
        Ok(())
    }

    pub fn digest(&self) -> Result<String, BenchmarkError> {
        let bytes = serde_json::to_vec(self)?;
        Ok(hex::encode(Sha256::digest(bytes)))
    }

    pub fn selected(&self, holdout_only: bool) -> Vec<&EmbodimentRegistration> {
        self.registrations
            .iter()
            .filter(|entry| {
                if holdout_only {
                    entry.role == EmbodimentRole::Holdout
                } else {
                    entry.role != EmbodimentRole::Holdout
                }
            })
            .collect()
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ResolvedBenchmarkTrial {
    pub schema_version: u32,
    /// Persisted schema-independent key used to pair randomized candidate runs.
    #[serde(default)]
    pub paired_trial_key: Option<String>,
    #[serde(default)]
    pub scenario_suite_id: Option<String>,
    #[serde(default)]
    pub scenario_suite_version: Option<u32>,
    #[serde(default)]
    pub stratum: Option<PushTrialStratum>,
    pub trial_id: String,
    pub seed: u64,
    pub index: u32,
    pub task: BenchmarkTask,
    pub role: EmbodimentRole,
    pub robot_id: String,
    pub model_hash: String,
    pub source_hash: String,
    pub expectation: TrialExpectation,
    pub scenario: ManipulationScenario,
    pub initial_qpos: Vec<f64>,
    pub goal_xyz: [f64; 3],
    pub fault_injection: Option<String>,
}

pub fn resolve_benchmark_trial(
    seed: u64,
    index: u32,
    task: BenchmarkTask,
    role: EmbodimentRole,
    bundle: &RobotBundle,
    manifest: &RobotManifest,
) -> Result<ResolvedBenchmarkTrial, BenchmarkError> {
    resolve_benchmark_trial_impl(seed, index, task, role, bundle, manifest, None)
}

pub fn resolve_benchmark_trial_with_suite(
    seed: u64,
    index: u32,
    task: BenchmarkTask,
    role: EmbodimentRole,
    bundle: &RobotBundle,
    manifest: &RobotManifest,
    suite: &ScenarioSuiteManifest,
) -> Result<ResolvedBenchmarkTrial, BenchmarkError> {
    suite.validate()?;
    if task != BenchmarkTask::PushProbeReplan {
        let mut trial =
            resolve_benchmark_trial_impl(seed, index, task, role, bundle, manifest, None)?;
        trial.scenario_suite_id = Some(suite.suite_id.clone());
        trial.scenario_suite_version = Some(suite.suite_version);
        trial.paired_trial_key = Some(paired_trial_key(&trial)?);
        return Ok(trial);
    }

    let stratum = suite.stratum_for_index(index);
    let attempts = if stratum == PushTrialStratum::ExpectedSuccess {
        suite.max_positive_resamples
    } else {
        1
    };
    let mut last_infeasibility = None;
    for attempt in 0..attempts {
        let attempt_seed = seed
            .wrapping_add(u64::from(attempt).wrapping_mul(0x9e37_79b9_7f4a_7c15))
            ^ 0x6670_726f_6265_7632;
        let trace = std::env::var_os("REALITYOS_BENCHMARK_TRACE").is_some();
        let attempt_started = std::time::Instant::now();
        if trace {
            eprintln!(
                "suite-v2 resolve start robot={} task={} seed={} index={} attempt={}/{}",
                bundle.manifest.robot_id,
                task.as_str(),
                seed,
                index,
                attempt + 1,
                attempts
            );
        }
        let result = resolve_benchmark_trial_impl(
            attempt_seed,
            index,
            task,
            role,
            bundle,
            manifest,
            Some(suite),
        );
        if trace {
            eprintln!(
                "suite-v2 resolve end robot={} task={} seed={} index={} attempt={}/{} elapsed_s={:.3} result={}",
                bundle.manifest.robot_id,
                task.as_str(),
                seed,
                index,
                attempt + 1,
                attempts,
                attempt_started.elapsed().as_secs_f64(),
                match &result {
                    Ok(_) => "resolved".to_owned(),
                    Err(error) => error.to_string(),
                }
            );
        }
        match result {
            Ok(mut trial) => {
                trial.seed = seed;
                trial.scenario.seed = seed;
                trial.scenario_suite_id = Some(suite.suite_id.clone());
                trial.scenario_suite_version = Some(suite.suite_version);
                trial.stratum = Some(stratum);
                trial.trial_id = format!(
                    "v{}:{}:{}:{}:{}:{}",
                    BENCHMARK_SCHEMA_VERSION,
                    trial.role.as_str(),
                    trial.robot_id,
                    trial.task.as_str(),
                    seed,
                    index
                );
                trial.paired_trial_key = Some(paired_trial_key(&trial)?);
                return Ok(trial);
            }
            Err(BenchmarkError::InvalidTrial(message))
                if stratum == PushTrialStratum::ExpectedSuccess
                    && message.starts_with("suite_v2_positive_infeasible:") =>
            {
                last_infeasibility = Some(message);
                continue;
            }
            Err(error) => return Err(error),
        }
    }
    Err(BenchmarkError::InvalidTrial(format!(
        "suite_v2_positive_infeasible: no valid positive scenario after {attempts} deterministic samples; last_reason={}",
        last_infeasibility.as_deref().unwrap_or("no_diagnostic")
    )))
}

fn resolve_benchmark_trial_impl(
    seed: u64,
    index: u32,
    task: BenchmarkTask,
    role: EmbodimentRole,
    bundle: &RobotBundle,
    manifest: &RobotManifest,
    scenario_suite: Option<&ScenarioSuiteManifest>,
) -> Result<ResolvedBenchmarkTrial, BenchmarkError> {
    if bundle.manifest.robot_id != manifest.robot_id {
        return Err(BenchmarkError::InvalidManifest(format!(
            "bundle robot {} does not match inspected robot {}",
            bundle.manifest.robot_id, manifest.robot_id
        )));
    }
    let nq = usize::try_from(manifest.nq)
        .map_err(|_| BenchmarkError::InvalidManifest("negative nq".into()))?;
    if manifest.qpos0.len() != nq || manifest.qpos0.iter().any(|value| !value.is_finite()) {
        return Err(BenchmarkError::InvalidManifest(format!(
            "qpos0 dimension or value invalid: nq={nq}, qpos0={}",
            manifest.qpos0.len()
        )));
    }
    let axes = end_effector_axes(bundle, manifest);
    let planar = arm_is_planar(axes.iter());
    let scenario_index = index as usize;
    let (mut scenario, suite_stratum) = match (scenario_suite, task) {
        (Some(suite), BenchmarkTask::PushProbeReplan) => {
            let stratum = suite.stratum_for_index(index);
            (
                suite_push_scenario(seed, planar, scenario_index, suite, stratum)?,
                Some(stratum),
            )
        }
        (_, BenchmarkTask::PushProbeReplan) => (push_scenario(seed, planar, scenario_index), None),
        (_, BenchmarkTask::PinchGraspLift) => (grasp_scenario(seed, planar, scenario_index), None),
    };
    if suite_stratum == Some(PushTrialStratum::ExpectedSuccess)
        && !manifest.geometry_inspection_complete
    {
        return Err(BenchmarkError::InvalidTrial(
            "suite_v2_positive_infeasible: complete collision geometry is not available".into(),
        ));
    }

    if planar && task == BenchmarkTask::PushProbeReplan {
        // The planar-arm embodiment moves its tool in a fixed horizontal
        // plane at z=0.12. Keep the support footprint outside the fixed base
        // and place its top 40mm below the interaction plane so the declared
        // tool sphere clears the support while overlapping the block's upper side.
        let table = scenario
            .objects
            .iter_mut()
            .find(|object| object["name"] == "table")
            .ok_or_else(|| BenchmarkError::InvalidTrial("planar scenario has no table".into()))?;
        table["pos"] = json!([0.4, 0.0, 0.07]);
        table["size"] = json!([0.2, 0.22, 0.01]);
        let table_center = vec3(&table["pos"]).ok_or_else(|| {
            BenchmarkError::InvalidTrial("planar table has no finite pose".into())
        })?;
        let table_half_z = table["size"]
            .as_array()
            .and_then(|size| size.get(2))
            .and_then(serde_json::Value::as_f64)
            .ok_or_else(|| BenchmarkError::InvalidTrial("planar table has no height".into()))?;
        let target = scenario
            .objects
            .iter_mut()
            .find(|object| object["name"] == scenario.object_id)
            .ok_or_else(|| BenchmarkError::InvalidTrial("missing planar target".into()))?;
        let size = target["size"]
            .as_array()
            .cloned()
            .ok_or_else(|| BenchmarkError::InvalidTrial("planar target has no size".into()))?;
        let half_x = size
            .first()
            .and_then(serde_json::Value::as_f64)
            .unwrap_or(f64::NAN);
        let half_y = size
            .get(1)
            .and_then(serde_json::Value::as_f64)
            .unwrap_or(half_x);
        let half_z = if suite_stratum == Some(PushTrialStratum::ExpectedSuccess) {
            size.get(2)
                .and_then(serde_json::Value::as_f64)
                .filter(|value| value.is_finite() && *value > 0.0)
                .unwrap_or(0.02)
        } else {
            0.02
        };
        let position = target["pos"]
            .as_array_mut()
            .ok_or_else(|| BenchmarkError::InvalidTrial("planar target has no position".into()))?;
        position[2] = json!(table_center[2] + table_half_z + half_z);
        target["size"] = json!([half_x, half_y, half_z]);
    } else if planar && task == BenchmarkTask::PinchGraspLift {
        // This fixture's arm runs just above z=0.1. A compact table footprint
        // keeps the base clear and gives the pinch object a real support face
        // inside the arm's reachable region.
        let table = scenario
            .objects
            .iter_mut()
            .find(|object| object["name"] == "table")
            .ok_or_else(|| {
                BenchmarkError::InvalidTrial("planar grasp scenario has no table".into())
            })?;
        table["pos"] = json!([0.25, 0.0, 0.08]);
        table["size"] = json!([0.15, 0.22, 0.01]);
        let target = scenario
            .objects
            .iter_mut()
            .find(|object| object["name"] == scenario.object_id)
            .ok_or_else(|| BenchmarkError::InvalidTrial("missing planar grasp target".into()))?;
        let shape = target["type"].as_str().unwrap_or("box").to_string();
        let target_size = target["size"].as_array_mut().ok_or_else(|| {
            BenchmarkError::InvalidTrial("planar grasp target has no size".into())
        })?;
        let source_y = target_size
            .get(1)
            .and_then(serde_json::Value::as_f64)
            .ok_or_else(|| {
                BenchmarkError::InvalidTrial("planar grasp target size is malformed".into())
            })?;
        let fit_fraction = ((source_y - 0.010) / 0.006).clamp(0.0, 1.0);
        let pinch_half_width = 0.008 + fit_fraction * 0.002;
        let half_z = 0.025;
        let normalized_size = if shape == "cylinder" {
            let _source_radius = target_size[0].as_f64().ok_or_else(|| {
                BenchmarkError::InvalidTrial("planar grasp target radius is malformed".into())
            })?;
            vec![pinch_half_width, half_z]
        } else {
            vec![0.01, pinch_half_width, half_z]
        };
        *target_size = normalized_size
            .into_iter()
            .map(|size| json!(size))
            .collect();
        let target_pos = target["pos"].as_array_mut().ok_or_else(|| {
            BenchmarkError::InvalidTrial("planar grasp target has no position".into())
        })?;
        target_pos[2] = json!(0.09 + half_z);
    }

    let mut rng = StdRng::seed_from_u64(
        seed.wrapping_add((index as u64).wrapping_mul(0x9e37_79b9)) ^ 0x6d61_6e69_7075_6c61,
    );
    if task == BenchmarkTask::PushProbeReplan && scenario_suite.is_none() {
        add_randomized_obstacles(&mut scenario, &mut rng)?;
    }
    scenario.objects.sort_by(|left, right| {
        left["name"]
            .as_str()
            .unwrap_or("")
            .cmp(right["name"].as_str().unwrap_or(""))
    });
    let object = object_named(&scenario.objects, &scenario.object_id)?;
    let center = vec3(&object["pos"]).ok_or_else(|| {
        BenchmarkError::InvalidTrial("target object has no finite 3D pose".into())
    })?;
    let goal_xyz = match task {
        BenchmarkTask::PushProbeReplan => [
            center[0] + scenario.push_dir[0] * scenario.push_dist,
            center[1] + scenario.push_dir[1] * scenario.push_dist,
            center[2] + scenario.push_dir[2] * scenario.push_dist,
        ],
        BenchmarkTask::PinchGraspLift => center,
    };
    let unsupported_gripper =
        task == BenchmarkTask::PinchGraspLift && bundle.manifest.grippers.is_empty();
    if unsupported_gripper && scenario.expected_refusal.is_none() {
        scenario.expected_refusal = Some("UNSUPPORTED_GRIPPER_TOPOLOGY".into());
    }
    if task == BenchmarkTask::PinchGraspLift
        && scenario.polarity == crate::manipulation_scenarios::Polarity::Positive
        && scenario.expected_refusal.is_none()
        && !grasp_hold_is_conservatively_provable(bundle, manifest, &scenario)
    {
        scenario.expected_refusal = Some("HOLD_MECHANICS_UNPROVEN".into());
    }
    let probe_capable_ambiguous_push = task == BenchmarkTask::PushProbeReplan
        && scenario.polarity == crate::manipulation_scenarios::Polarity::Ambiguous;
    let mut expectation = if let Some(stratum) = suite_stratum {
        match stratum {
            PushTrialStratum::ExpectedSuccess => TrialExpectation::ExpectedSuccess,
            PushTrialStratum::ExpectedRefusal => TrialExpectation::ExpectedRefusal,
        }
    } else if (scenario.polarity == crate::manipulation_scenarios::Polarity::Positive
        || probe_capable_ambiguous_push)
        && scenario.expected_refusal.is_none()
    {
        TrialExpectation::ExpectedSuccess
    } else {
        TrialExpectation::ExpectedRefusal
    };
    let stable_random_start = scenario_suite.is_some()
        && task == BenchmarkTask::PushProbeReplan
        && expectation == TrialExpectation::ExpectedSuccess;
    let mut initial_qpos = if stable_random_start {
        sample_positive_start_qpos(manifest, &mut rng)?
    } else {
        sample_initial_qpos(manifest, &mut rng)?
    };
    if expectation == TrialExpectation::ExpectedSuccess {
        let mut found_standoff = false;
        let mut safe_initial_qpos = None;
        // In the v2 positive suite, the randomized standoff is a witness for
        // the scenario's fixed target pose. It does not depend on which safe
        // randomized home pose is selected below, so solve it once instead of
        // repeating the full IK search for every initial-state candidate.
        if stable_random_start && manifest.geometry_inspection_complete {
            let mut standoff_witness = initial_qpos.clone();
            found_standoff = set_randomized_push_standoff(
                bundle,
                manifest,
                &scenario,
                &mut standoff_witness,
                &mut rng,
            )?;
        }
        let attempts = if stable_random_start && !found_standoff {
            0
        } else if manifest.geometry_inspection_complete {
            MAX_SAFE_INITIAL_QPOS_ATTEMPTS
        } else {
            1
        };
        for _ in 0..attempts {
            let mut candidate = if stable_random_start {
                sample_positive_start_qpos(manifest, &mut rng)?
            } else {
                sample_initial_qpos(manifest, &mut rng)?
            };
            if manifest.geometry_inspection_complete {
                let reachable = match task {
                    BenchmarkTask::PushProbeReplan if stable_random_start => {
                        // Keep the randomized, collision-checked start pose as
                        // the trial initial state. The standoff IK is a
                        // feasibility witness only; using it as qpos would
                        // start the robot beside/inside the support edge and
                        // bypass the required transit proof. Its scenario pose
                        // is fixed across the initial-state search, so reuse
                        // the already-computed witness.
                        found_standoff
                    }
                    BenchmarkTask::PushProbeReplan => set_randomized_push_standoff(
                        bundle,
                        manifest,
                        &scenario,
                        &mut candidate,
                        &mut rng,
                    )?,
                    BenchmarkTask::PinchGraspLift => set_randomized_grasp_standoff(
                        bundle,
                        manifest,
                        &scenario,
                        &mut candidate,
                        &mut rng,
                    )?,
                };
                if !reachable {
                    initial_qpos = candidate;
                    continue;
                }
            } else if task == BenchmarkTask::PinchGraspLift
                && !set_randomized_grasp_standoff(
                    bundle,
                    manifest,
                    &scenario,
                    &mut candidate,
                    &mut rng,
                )?
            {
                initial_qpos = candidate;
                continue;
            }
            found_standoff = true;
            initial_qpos = candidate.clone();
            let collision_free = !manifest.geometry_inspection_complete
                || initial_configuration_is_clear(bundle, manifest, &scenario, &candidate)?;
            if collision_free {
                safe_initial_qpos = Some(candidate);
                break;
            }
        }
        if let Some(candidate) = safe_initial_qpos {
            initial_qpos = candidate;
        } else {
            if suite_stratum == Some(PushTrialStratum::ExpectedSuccess) {
                return Err(BenchmarkError::InvalidTrial(format!(
                    "suite_v2_positive_infeasible: {}",
                    if !found_standoff {
                        "no complete-geometry standoff IK solution"
                    } else {
                        "no collision-free initial configuration"
                    }
                )));
            }
            scenario.expected_refusal = Some(
                if !found_standoff {
                    match task {
                        BenchmarkTask::PushProbeReplan => "PUSH_STANDOFF_UNREACHABLE",
                        BenchmarkTask::PinchGraspLift => "GRASP_STANDOFF_UNREACHABLE",
                    }
                } else {
                    "NO_SAFE_INITIAL_CONFIGURATION"
                }
                .into(),
            );
            expectation = TrialExpectation::ExpectedRefusal;
        }
    }
    if suite_stratum == Some(PushTrialStratum::ExpectedSuccess)
        && task == BenchmarkTask::PushProbeReplan
        && manifest.geometry_inspection_complete
    {
        let probe_status = positive_probe_candidate_status(
            bundle,
            manifest,
            &scenario,
            goal_xyz,
            &initial_qpos,
            seed,
            index,
        )?;
        if probe_status != "PROVED" {
            return Err(BenchmarkError::InvalidTrial(format!(
                "suite_v2_positive_infeasible: complete probe-path proof returned {probe_status}"
            )));
        }
    }
    let task_name = task.as_str();
    let role_name = role.as_str();
    let trial_id = format!(
        "v{BENCHMARK_SCHEMA_VERSION}:{role_name}:{}:{task_name}:{seed}:{index}",
        manifest.robot_id
    );
    let fault_injection = if scenario.neg.is_some() {
        scenario.neg.map(|kind| kind.as_str().to_owned())
    } else if task == BenchmarkTask::PushProbeReplan
        && expectation == TrialExpectation::ExpectedSuccess
        && index % 31 == 30
    {
        Some("runtime_divergence".into())
    } else {
        None
    };

    let mut resolved = ResolvedBenchmarkTrial {
        schema_version: BENCHMARK_SCHEMA_VERSION,
        paired_trial_key: None,
        scenario_suite_id: scenario_suite.map(|suite| suite.suite_id.clone()),
        scenario_suite_version: scenario_suite.map(|suite| suite.suite_version),
        stratum: suite_stratum,
        trial_id,
        seed,
        index,
        task,
        role,
        robot_id: manifest.robot_id.clone(),
        model_hash: manifest.model_hash.clone(),
        source_hash: manifest.source_hash.clone(),
        expectation,
        scenario,
        initial_qpos,
        goal_xyz,
        fault_injection,
    };
    resolved.paired_trial_key = Some(paired_trial_key(&resolved)?);
    Ok(resolved)
}

fn positive_probe_candidate_status(
    bundle: &RobotBundle,
    manifest: &RobotManifest,
    scenario: &ManipulationScenario,
    goal_xyz: [f64; 3],
    initial_qpos: &[f64],
    seed: u64,
    index: u32,
) -> Result<String, BenchmarkError> {
    let model = embodiment_from_manifest(bundle, manifest);
    let ee_name = model
        .end_effectors
        .first()
        .map(|ee| ee.name.as_str())
        .ok_or_else(|| BenchmarkError::InvalidTrial("positive trial has no end-effector".into()))?;
    let frame_key =
        super::execution::declared_ee_observation_key(bundle, ee_name).ok_or_else(|| {
            BenchmarkError::InvalidTrial("positive trial has no observable tool frame".into())
        })?;
    let object = object_named(&scenario.objects, &scenario.object_id)?;
    let center = vec3(&object["pos"])
        .ok_or_else(|| BenchmarkError::InvalidTrial("positive target pose is invalid".into()))?;
    let mut state = json!({
        "qpos": initial_qpos,
        "xpos": {},
        "sites": {},
        "xquat": {}
    });
    state["xpos"][&scenario.object_id] = json!(center);
    state["xpos"][frame_key.as_str()] = json!([0.0, 0.0, 0.0]);
    state["sites"][frame_key.as_str()] = json!([0.0, 0.0, 0.0]);
    state["xquat"][&scenario.object_id] = json!(scenario_quaternion(object));
    let epoch = format!("suite-v2-feasibility:{seed}:{index}");
    let request = super::execution::build_push_reasoning_request(
        bundle,
        manifest,
        &model,
        scenario,
        goal_xyz,
        &state,
        &epoch,
        10.0,
        true,
        super::execution::uncertain_push_belief(),
    )?;
    let before = realityos_semantics::work_counters::work_snapshot();
    let started = std::time::Instant::now();
    let status = PhysicalReasoner::new(model)
        .preview_candidate(&request, "probe:+x")
        .status;
    if std::env::var_os("REALITYOS_BENCHMARK_TRACE").is_some() {
        let after = realityos_semantics::work_counters::work_snapshot();
        eprintln!(
            "suite-v2 probe-proof robot={} seed={} index={} status={} elapsed_s={:.3} ik={} fk={} jacobian={} collision={}",
            bundle.manifest.robot_id,
            seed,
            index,
            status,
            started.elapsed().as_secs_f64(),
            after.ik_attempts.saturating_sub(before.ik_attempts),
            after.fk_evaluations.saturating_sub(before.fk_evaluations),
            after.jacobian_evaluations.saturating_sub(before.jacobian_evaluations),
            after.collision_queries.saturating_sub(before.collision_queries),
        );
    }
    Ok(status)
}

fn suite_push_scenario(
    seed: u64,
    planar: bool,
    index: usize,
    suite: &ScenarioSuiteManifest,
    stratum: PushTrialStratum,
) -> Result<ManipulationScenario, BenchmarkError> {
    let mut rng = StdRng::seed_from_u64(
        seed.wrapping_add((index as u64).wrapping_mul(0x9e37_79b9)) ^ 0x7375_6974_655f_7632,
    );
    match stratum {
        PushTrialStratum::ExpectedRefusal => {
            let label = &suite.refusal_cycle[index % suite.refusal_cycle.len()];
            if label == "unidentifiable_friction" {
                let mut scenario = push_scenario(seed, planar, 10);
                scenario.seed = seed;
                scenario.polarity = Polarity::Positive;
                scenario.expected_refusal = None;
                scenario.neg = None;
                scenario.stale = false;
                scenario.move_object_after_obs = false;
                scenario.replay = false;
                scenario.restart_replay = false;
                scenario.crash_controller = false;
                scenario.immovable = false;
                scenario.planar = planar;
                scenario.push_dir = [1.0, 0.0, 0.0];
                let bounds = suite.unidentifiable_push.as_ref().ok_or_else(|| {
                    BenchmarkError::InvalidManifest(
                        "unidentifiable-friction stratum has no randomized bounds".into(),
                    )
                })?;
                let mass = rng.gen_range(bounds.mass_kg[0]..=bounds.mass_kg[1]);
                let friction = rng.gen_range(bounds.friction[0]..=bounds.friction[1]);
                let target = scenario
                    .objects
                    .iter_mut()
                    .find(|object| object["name"] == scenario.object_id)
                    .ok_or_else(|| {
                        BenchmarkError::InvalidTrial(
                            "unidentifiable-friction scenario has no target".into(),
                        )
                    })?;
                target["mass"] = json!(mass);
                target["friction"] = json!(friction);
                target["movable"] = json!(true);
                let table = scenario
                    .objects
                    .iter_mut()
                    .find(|object| object["name"] == "table")
                    .ok_or_else(|| {
                        BenchmarkError::InvalidTrial(
                            "unidentifiable-friction scenario has no support table".into(),
                        )
                    })?;
                table["friction"] = json!(friction);
                return Ok(scenario);
            }
            let legacy_index = match label.as_str() {
                "unreachable" => 0,
                "stale_object" => 1,
                "object_moved" => 2,
                "unexpected_contact" => 3,
                "slip" => 4,
                "replay" => 5,
                "restart_replay" => 6,
                "controller_failure" => 7,
                "force_bound_unavailable" => 8,
                _ => {
                    return Err(BenchmarkError::InvalidManifest(format!(
                        "unknown refusal stratum {label}"
                    )))
                }
            };
            Ok(push_scenario(seed, planar, legacy_index))
        }
        PushTrialStratum::ExpectedSuccess => {
            let mut scenario = push_scenario(seed, planar, 10);
            if !planar {
                let table = scenario
                    .objects
                    .iter_mut()
                    .find(|object| object["name"] == "table")
                    .ok_or_else(|| {
                        BenchmarkError::InvalidTrial("spatial suite scenario has no table".into())
                    })?;
                // Put the shared positive fixture along the near edge of its
                // support, facing the arm. This leaves a collision-cleared
                // approach corridor without changing the target/tool model.
                table["pos"] = json!([0.90, 0.0, 0.40]);
                table["size"] = json!([0.40, 0.25, 0.02]);
            }
            scenario.seed = seed;
            scenario.polarity = Polarity::Positive;
            scenario.expected_refusal = None;
            scenario.neg = None;
            scenario.stale = false;
            scenario.move_object_after_obs = false;
            scenario.replay = false;
            scenario.restart_replay = false;
            scenario.crash_controller = false;
            scenario.immovable = false;
            scenario.planar = planar;
            scenario.push_dir = [1.0, 0.0, 0.0];

            let [x_min, x_max] = suite.positive_push.object_x_m;
            let [y_min, y_max] = suite.positive_push.object_y_m;
            let [size_min, size_max] = suite.positive_push.half_size_m;
            let [height_min, height_max] = suite.positive_push.half_height_m;
            let [mass_min, mass_max] = suite.positive_push.mass_kg;
            let [friction_min, friction_max] = suite.positive_push.friction;
            let [distance_min, distance_max] = suite.positive_push.push_distance_m;
            let x = rng.gen_range(x_min..=x_max);
            let y = rng.gen_range(y_min..=y_max);
            let half_size = rng.gen_range(size_min..=size_max);
            let half_height = rng.gen_range(height_min..=height_max);
            let mass = rng.gen_range(mass_min..=mass_max);
            let friction = rng.gen_range(friction_min..=friction_max);
            scenario.push_dist = rng.gen_range(distance_min..=distance_max);

            let target = scenario
                .objects
                .iter_mut()
                .find(|object| object["name"] == scenario.object_id)
                .ok_or_else(|| BenchmarkError::InvalidTrial("missing suite push target".into()))?;
            target["pos"] = json!([x, y, 0.42 + half_height]);
            target["size"] = json!([half_size, half_size, half_height]);
            target["mass"] = json!(mass);
            target["friction"] = json!(friction);
            target["movable"] = json!(true);
            let table = scenario
                .objects
                .iter_mut()
                .find(|object| object["name"] == "table")
                .ok_or_else(|| {
                    BenchmarkError::InvalidTrial("suite push scenario has no support table".into())
                })?;
            table["friction"] = json!(friction);
            scenario
                .objects
                .retain(|object| object["name"] == scenario.object_id || object["name"] == "table");

            let [count_min, count_max] = suite.positive_push.obstacle_count;
            let obstacle_count = rng.gen_range(count_min..=count_max);
            let [forward_min, forward_max] = suite.positive_push.obstacle_forward_m;
            let [lateral_min, lateral_max] = suite.positive_push.obstacle_lateral_m;
            for obstacle_index in 0..obstacle_count {
                let side = rng.gen_range(size_min..=size_max);
                let forward = rng.gen_range(forward_min..=forward_max);
                let lateral = rng.gen_range(lateral_min..=lateral_max)
                    * if obstacle_index % 2 == 0 { 1.0 } else { -1.0 };
                let geom = if rng.gen_bool(0.25) {
                    "cylinder"
                } else {
                    "box"
                };
                scenario.objects.push(json!({
                    "name": format!("benchmark_obstacle_{obstacle_index}"),
                    "type": geom,
                    "pos": [x + forward, y + lateral, 0.42 + half_size],
                    "size": if geom == "cylinder" { json!([side, side]) } else { json!([side, side, side]) },
                    "mass": 1.0,
                    "friction": 0.8,
                    "movable": false
                }));
            }
            Ok(scenario)
        }
    }
}

#[cfg(test)]
mod suite_v3_tests {
    use super::*;

    #[test]
    fn v3_unidentifiable_refusal_randomizes_low_load_without_fault_injection() {
        let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../robots/benchmarks/manipulation/scenario_suite_v3.json");
        let suite = ScenarioSuiteManifest::load(&path).expect("v3 scenario suite");
        let expected = suite.unidentifiable_push.as_ref().unwrap();

        let first =
            suite_push_scenario(21_003, false, 3, &suite, PushTrialStratum::ExpectedRefusal)
                .expect("unidentifiable friction trial");
        let second =
            suite_push_scenario(21_003, false, 3, &suite, PushTrialStratum::ExpectedRefusal)
                .expect("deterministic replay");

        let target = first
            .objects
            .iter()
            .find(|object| object["name"] == first.object_id)
            .expect("target object");
        let mass = target["mass"].as_f64().expect("randomized mass");
        let friction = target["friction"].as_f64().expect("randomized friction");
        let table = first
            .objects
            .iter()
            .find(|object| object["name"] == "table")
            .expect("support table");
        assert!((expected.mass_kg[0]..=expected.mass_kg[1]).contains(&mass));
        assert!((expected.friction[0]..=expected.friction[1]).contains(&friction));
        assert_eq!(table["friction"], target["friction"]);
        assert_eq!(target["movable"], true);
        assert_eq!(first.polarity, Polarity::Positive);
        assert_eq!(first.neg, None);
        assert_eq!(first.expected_refusal, None);
        assert_eq!(first, second);
    }

    #[test]
    fn v3_positive_stratum_randomizes_effective_support_friction() {
        let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../robots/benchmarks/manipulation/scenario_suite_v3.json");
        let suite = ScenarioSuiteManifest::load(&path).expect("v3 scenario suite");
        let scenario =
            suite_push_scenario(21_000, false, 0, &suite, PushTrialStratum::ExpectedSuccess)
                .expect("force-observable positive trial");
        let target = scenario
            .objects
            .iter()
            .find(|object| object["name"] == scenario.object_id)
            .expect("target object");
        let table = scenario
            .objects
            .iter()
            .find(|object| object["name"] == "table")
            .expect("support table");
        let friction = target["friction"].as_f64().expect("sampled friction");

        assert!(
            (suite.positive_push.friction[0]..=suite.positive_push.friction[1]).contains(&friction)
        );
        assert_eq!(table["friction"].as_f64(), Some(friction));
    }
}

fn grasp_hold_is_conservatively_provable(
    bundle: &RobotBundle,
    manifest: &RobotManifest,
    scenario: &ManipulationScenario,
) -> bool {
    let Some(friction_oracle_input) = simulated_grasp_friction_oracle_input(scenario) else {
        return false;
    };
    let Some(max_provable_mass_kg) =
        conservative_grasp_mass_limit(bundle, manifest, friction_oracle_input.interval[0])
    else {
        return false;
    };
    let Some(mass) = scenario
        .objects
        .iter()
        .find(|object| object["name"] == scenario.object_id)
        .and_then(|object| object["mass"].as_f64())
        .filter(|mass| mass.is_finite() && *mass > 0.0)
    else {
        return false;
    };

    mass <= max_provable_mass_kg + 1e-12
}

fn conservative_grasp_mass_limit(
    bundle: &RobotBundle,
    manifest: &RobotManifest,
    friction_lower_bound: f64,
) -> Option<f64> {
    if !friction_lower_bound.is_finite() || friction_lower_bound <= 0.0 {
        return None;
    }
    let finger_joints = bundle
        .manifest
        .joint_aliases
        .iter()
        .filter(|(semantic, _)| semantic.to_ascii_lowercase().contains("finger"))
        .map(|(_, physical)| physical.as_str())
        .collect::<std::collections::BTreeSet<_>>();
    if finger_joints.len() != 2 {
        return None;
    }
    let force_ceiling = finger_joints
        .iter()
        .map(|joint| {
            manifest
                .actuators
                .iter()
                .find(|actuator| actuator.transmission_target == **joint)
                .and_then(|actuator| actuator.force_range)
                .map(|range| range[0].abs().max(range[1].abs()))
                .filter(|force| force.is_finite() && *force > 0.0)
        })
        .collect::<Option<Vec<_>>>()
        .and_then(|forces| forces.into_iter().reduce(f64::min))?;
    let max_mass_kg = 2.0 * friction_lower_bound * force_ceiling / 9.81;
    max_mass_kg.is_finite().then_some(max_mass_kg)
}

/// Deterministic simulated friction-oracle input for strict MuJoCo trials.
/// It samples the scenario's configured simulator friction with seeded noise;
/// it is not a tactile sensor measurement or evidence of autonomous detection.
/// Physical trials must use the separate calibrated sensor provenance adapter.
pub(super) fn simulated_grasp_friction_oracle_input(
    scenario: &ManipulationScenario,
) -> Option<SimulatedFrictionOracleInput> {
    const SIMULATED_INPUT_NOISE_BOUND: f64 = 0.01;
    const FROZEN_INTERVAL_ERROR: f64 = 0.02;
    const SAMPLE_COUNT: usize = 5;
    // Ambiguous trials withhold the simulator friction input, so the selector
    // must refuse with unknown friction. A slip case injects a low-friction
    // oracle value to test safe response; it does not test slip detection.
    if scenario.polarity == Polarity::Ambiguous && scenario.neg != Some(NegKind::Slip) {
        return None;
    }
    let friction = scenario
        .objects
        .iter()
        .find(|object| object["name"] == scenario.object_id)?["friction"]
        .as_f64()
        .filter(|value| value.is_finite() && *value >= 0.0 && *value <= 1.2)?;
    let mut rng = StdRng::seed_from_u64(
        scenario.seed
            ^ 0x006f_7261_636c_655f
            ^ scenario.object_id.len() as u64
            ^ friction.to_bits(),
    );
    let samples = (0..SAMPLE_COUNT)
        .map(|_| {
            (friction + rng.gen_range(-SIMULATED_INPUT_NOISE_BOUND..=SIMULATED_INPUT_NOISE_BOUND))
                .clamp(0.0, 1.2)
        })
        .collect::<Vec<_>>();
    let estimate = samples.iter().sum::<f64>() / samples.len() as f64;
    Some(SimulatedFrictionOracleInput {
        samples,
        interval: [
            (estimate - FROZEN_INTERVAL_ERROR).max(0.0),
            (estimate + FROZEN_INTERVAL_ERROR).min(1.2),
        ],
    })
}

pub fn trial_digest(trial: &ResolvedBenchmarkTrial) -> Result<String, BenchmarkError> {
    let mut canonical = trial.clone();
    canonical.scenario.objects.sort_by(|left, right| {
        left["name"]
            .as_str()
            .unwrap_or("")
            .cmp(right["name"].as_str().unwrap_or(""))
    });
    let bytes = serde_json::to_vec(&canonical)?;
    Ok(hex::encode(Sha256::digest(bytes)))
}

/// Hash only resolved scenario inputs so the same randomized trial pairs across
/// schema versions, source revisions, sensor traces, action traces, and outcomes.
pub fn paired_trial_key(trial: &ResolvedBenchmarkTrial) -> Result<String, BenchmarkError> {
    let mut scenario = serde_json::to_value(&trial.scenario)?;
    if let Some(objects) = scenario.get_mut("objects").and_then(Value::as_array_mut) {
        objects.sort_by(|left, right| {
            left.get("name")
                .and_then(Value::as_str)
                .unwrap_or("")
                .cmp(right.get("name").and_then(Value::as_str).unwrap_or(""))
        });
    }
    let mut canonical = json!({
        "role": trial.role.as_str(),
        "robot_id": trial.robot_id,
        "model_hash": trial.model_hash,
        "task": trial.task.as_str(),
        "seed": trial.seed,
        "scenario_index": trial.index,
        "expectation": trial.expectation,
        "scenario": scenario,
        "fault_injection": trial.fault_injection,
        "goal_xyz_bits": trial.goal_xyz.map(f64::to_bits),
        "initial_qpos_bits": trial.initial_qpos.iter().map(|value| value.to_bits()).collect::<Vec<_>>(),
    });
    let suite_bound = trial.scenario_suite_id.is_some()
        || trial.scenario_suite_version.is_some()
        || trial.stratum.is_some();
    if suite_bound {
        canonical["scenario_suite_id"] = serde_json::to_value(&trial.scenario_suite_id)?;
        canonical["scenario_suite_version"] = serde_json::to_value(trial.scenario_suite_version)?;
        canonical["stratum"] = serde_json::to_value(trial.stratum)?;
    }
    let mut bytes = if suite_bound {
        b"realityos.manipulation.paired_trial_key/3\0".to_vec()
    } else {
        b"realityos.manipulation.paired_trial_key/2\0".to_vec()
    };
    push_canonical_json(&canonical, &mut bytes)?;
    Ok(hex::encode(Sha256::digest(bytes)))
}

/// Create schema-v6 execution metadata around an immutable schema-v5 input.
/// Every randomized field is preserved byte-for-byte so the candidate pilot
/// executes the exact saved baseline scenario rather than re-sampling it.
pub fn upgrade_v5_trial_for_v6(
    legacy: &ResolvedBenchmarkTrial,
) -> Result<ResolvedBenchmarkTrial, BenchmarkError> {
    if legacy.schema_version != 5 {
        return Err(BenchmarkError::InvalidTrial(format!(
            "only schema-v5 historical inputs can be paired into schema-v{} execution",
            BENCHMARK_SCHEMA_VERSION
        )));
    }
    let mut trial = legacy.clone();
    trial.schema_version = LEGACY_PAIRED_SCHEMA_VERSION;
    trial.trial_id = format!(
        "v{}:{}:{}:{}:{}:{}",
        LEGACY_PAIRED_SCHEMA_VERSION,
        trial.role.as_str(),
        trial.robot_id,
        trial.task.as_str(),
        trial.seed,
        trial.index
    );
    trial.paired_trial_key = None;
    trial.paired_trial_key = Some(paired_trial_key(&trial)?);
    Ok(trial)
}

fn push_canonical_json(value: &Value, bytes: &mut Vec<u8>) -> Result<(), BenchmarkError> {
    match value {
        Value::Null => bytes.push(b'n'),
        Value::Bool(value) => bytes.push(if *value { b't' } else { b'f' }),
        Value::Number(value) => {
            bytes.push(b'#');
            push_key_bytes(bytes, value.to_string().as_bytes());
        }
        Value::String(value) => {
            bytes.push(b's');
            push_key_bytes(bytes, value.as_bytes());
        }
        Value::Array(values) => {
            bytes.push(b'[');
            bytes.extend_from_slice(&(values.len() as u64).to_le_bytes());
            for value in values {
                push_canonical_json(value, bytes)?;
            }
            bytes.push(b']');
        }
        Value::Object(values) => {
            bytes.push(b'{');
            bytes.extend_from_slice(&(values.len() as u64).to_le_bytes());
            let mut entries = values.iter().collect::<Vec<_>>();
            entries.sort_by(|left, right| left.0.cmp(right.0));
            for (key, value) in entries {
                push_key_bytes(bytes, key.as_bytes());
                push_canonical_json(value, bytes)?;
            }
            bytes.push(b'}');
        }
    }
    Ok(())
}

fn push_key_bytes(bytes: &mut Vec<u8>, value: &[u8]) {
    bytes.extend_from_slice(&(value.len() as u64).to_le_bytes());
    bytes.extend_from_slice(value);
}

fn end_effector_axes(bundle: &RobotBundle, manifest: &RobotManifest) -> Vec<[f64; 3]> {
    // MJCF joint axes are local to their parent bodies. Classifying an arm from
    // those raw axes mistakes spatial chains whose joints each declare local Z
    // axes for planar arms. Evaluate the chain at its inspected reference pose
    // and classify the actual world-space axes instead. Unknown kinematics stay
    // spatial so the scenario generator never lowers a table on incomplete data.
    let model = embodiment_from_manifest(bundle, manifest);
    let Some(ee) = model.end_effectors.first() else {
        return Vec::new();
    };
    let Some(chain) = model.ee_joint_chain(&ee.name) else {
        return Vec::new();
    };
    let mut q = Vec::with_capacity(chain.len());
    for name in &chain {
        let Some(joint) = manifest.joints.iter().find(|joint| joint.name == *name) else {
            return Vec::new();
        };
        let Ok(index) = usize::try_from(joint.qpos_address) else {
            return Vec::new();
        };
        let Some(value) = manifest.qpos0.get(index) else {
            return Vec::new();
        };
        q.push(*value);
    }
    realityos_semantics::kinematics::forward_kinematics(&model, &chain, &ee.name, &q)
        .map(|state| state.axes_world)
        .unwrap_or_default()
}

fn sample_initial_qpos(
    manifest: &RobotManifest,
    rng: &mut StdRng,
) -> Result<Vec<f64>, BenchmarkError> {
    let expected_nq = usize::try_from(manifest.nq)
        .map_err(|_| BenchmarkError::InvalidManifest("negative nq".into()))?;
    if manifest.qpos0.len() != expected_nq || manifest.qpos0.iter().any(|value| !value.is_finite())
    {
        return Err(BenchmarkError::InvalidManifest(format!(
            "qpos0 dimension or value invalid: nq={expected_nq}, qpos={}",
            manifest.qpos0.len()
        )));
    }
    let mut qpos = manifest.qpos0.clone();
    for joint in &manifest.joints {
        if !joint.limited || joint.qpos_dim != 1 {
            continue;
        }
        let index = usize::try_from(joint.qpos_address).map_err(|_| {
            BenchmarkError::InvalidManifest(format!("negative qpos address for {}", joint.name))
        })?;
        let Some(value) = qpos.get_mut(index) else {
            return Err(BenchmarkError::InvalidManifest(format!(
                "qpos address outside qpos0 for {}",
                joint.name
            )));
        };
        let [lower, upper] = joint.range;
        if !lower.is_finite() || !upper.is_finite() || lower > upper {
            return Err(BenchmarkError::InvalidManifest(format!(
                "invalid range for {}: [{lower}, {upper}]",
                joint.name
            )));
        }
        *value = if lower >= upper {
            lower
        } else {
            rng.gen_range(lower..=upper)
        };
    }
    Ok(qpos)
}

fn sample_positive_start_qpos(
    manifest: &RobotManifest,
    rng: &mut StdRng,
) -> Result<Vec<f64>, BenchmarkError> {
    const HOME_RANDOMIZATION_RAD: f64 = 0.08;
    let expected_nq = usize::try_from(manifest.nq)
        .map_err(|_| BenchmarkError::InvalidManifest("negative nq".into()))?;
    if manifest.qpos0.len() != expected_nq || manifest.qpos0.iter().any(|value| !value.is_finite())
    {
        return Err(BenchmarkError::InvalidManifest(format!(
            "qpos0 dimension or value invalid: nq={expected_nq}, qpos0={}",
            manifest.qpos0.len()
        )));
    }
    let mut qpos = manifest.qpos0.clone();
    for joint in &manifest.joints {
        if !joint.limited || joint.qpos_dim != 1 {
            continue;
        }
        let index = usize::try_from(joint.qpos_address).map_err(|_| {
            BenchmarkError::InvalidManifest(format!("negative qpos address for {}", joint.name))
        })?;
        let Some(current) = qpos.get_mut(index) else {
            return Err(BenchmarkError::InvalidManifest(format!(
                "qpos address outside qpos0 for {}",
                joint.name
            )));
        };
        let margin = MIN_POSITIVE_JOINT_MARGIN_RAD;
        let mut lower = joint.range[0] + margin;
        let mut upper = joint.range[1] - margin;
        if let Some(actuator) = manifest
            .actuators
            .iter()
            .find(|actuator| actuator.transmission_target == joint.name)
        {
            if actuator.ctrllimited {
                lower = lower.max(actuator.ctrlrange[0] + margin);
                upper = upper.min(actuator.ctrlrange[1] - margin);
            }
        }
        if !lower.is_finite() || !upper.is_finite() || lower > upper {
            return Err(BenchmarkError::InvalidTrial(format!(
                "suite_v2_positive_infeasible: {} has no controller-safe start range",
                joint.name
            )));
        }
        let center = current.clamp(lower, upper);
        let half_width = HOME_RANDOMIZATION_RAD
            .min(center - lower)
            .min(upper - center)
            .max(0.0);
        *current = if half_width > 1e-9 {
            rng.gen_range(center - half_width..=center + half_width)
        } else {
            center
        };
    }
    Ok(qpos)
}

fn initial_configuration_is_clear(
    bundle: &RobotBundle,
    manifest: &RobotManifest,
    scenario: &ManipulationScenario,
    qpos: &[f64],
) -> Result<bool, BenchmarkError> {
    let model = embodiment_from_manifest(bundle, manifest);
    let Some(ee) = model.end_effectors.first() else {
        return Ok(false);
    };
    if model.collision_geoms.is_empty() {
        return Ok(false);
    }
    if manifest.joints.iter().any(|joint| joint.qpos_dim != 1) {
        return Ok(false);
    }
    let joint_names = manifest
        .joints
        .iter()
        .map(|joint| joint.name.clone())
        .collect::<Vec<_>>();
    if joint_names.is_empty() {
        return Ok(false);
    }
    let mut arm_q = Vec::with_capacity(joint_names.len());
    for name in &joint_names {
        let joint = manifest
            .joints
            .iter()
            .find(|joint| joint.name == *name)
            .ok_or_else(|| {
                BenchmarkError::InvalidManifest(format!(
                    "end-effector chain references uninspected joint {name}"
                ))
            })?;
        let index = usize::try_from(joint.qpos_address).map_err(|_| {
            BenchmarkError::InvalidManifest(format!("negative qpos address for {name}"))
        })?;
        arm_q.push(*qpos.get(index).ok_or_else(|| {
            BenchmarkError::InvalidManifest(format!("qpos address outside state for {name}"))
        })?);
    }

    let object = object_named(&scenario.objects, &scenario.object_id)?;
    let object_center = vec3(&object["pos"])
        .ok_or_else(|| BenchmarkError::InvalidTrial("target object pose is invalid".into()))?;
    let object_half = scenario_half_extents(object)?;
    let object_quat = scenario_quaternion(object);
    let table = object_named(&scenario.objects, "table")?;
    let table_center = vec3(&table["pos"])
        .ok_or_else(|| BenchmarkError::InvalidTrial("table support pose is invalid".into()))?;
    let table_half = scenario_half_extents(table)?;
    let obstacles = scenario
        .objects
        .iter()
        .filter(|candidate| {
            candidate["name"].as_str() != Some(&scenario.object_id)
                && candidate["name"].as_str() != Some("table")
        })
        .map(|candidate| {
            let name = candidate["name"]
                .as_str()
                .ok_or_else(|| BenchmarkError::InvalidTrial("obstacle has no name".into()))?;
            let center = vec3(&candidate["pos"]).ok_or_else(|| {
                BenchmarkError::InvalidTrial(format!("obstacle {name} pose is invalid"))
            })?;
            Ok(NamedBox::aabb(
                name,
                center,
                scenario_half_extents(candidate)?,
            ))
        })
        .collect::<Result<Vec<_>, BenchmarkError>>()?;
    let world = CollisionWorld {
        object_id: scenario.object_id.clone(),
        object_center,
        object_half,
        object_quat,
        support_id: "table".into(),
        support_origin: [
            table_center[0],
            table_center[1],
            table_center[2] + table_half[2],
        ],
        support_normal: [0.0, 0.0, 1.0],
        support_half_extents: Some(table_half),
        intended_tool_bodies: Vec::new(),
        robot_volumes: Vec::new(),
        obstacles,
        ee_radius: 0.01,
        object_probe_radius: object_half.into_iter().fold(0.0, f64::max),
        tool_offset_ee: [0.0; 3],
        declared_geoms: model.collision_geoms.clone(),
    };
    let scene = match scene_from_collision_world(&model, &ee.name, &world) {
        Ok(scene) => scene,
        Err(_) => return Ok(false),
    };
    let policy = policy_from_scene_model(Some(&model), &scene);
    let report = validate_transition(
        &model,
        &joint_names,
        &arm_q,
        &arm_q,
        &scene,
        &policy,
        ContactPhase::CurrentToApproach,
    );
    Ok(report.execution_refusal().is_none()
        && matches!(
            report.verdict,
            GeometryVerdict::ClearUnderCompleteDeclared
                | GeometryVerdict::NoCollisionInProxy
                | GeometryVerdict::AllowedPhaseContact
        ))
}

fn scenario_half_extents(object: &Value) -> Result<[f64; 3], BenchmarkError> {
    let size = object["size"]
        .as_array()
        .ok_or_else(|| BenchmarkError::InvalidTrial("scenario geometry has no size".into()))?;
    let x = size
        .first()
        .and_then(Value::as_f64)
        .unwrap_or(f64::NAN)
        .abs();
    let y = size.get(1).and_then(Value::as_f64).unwrap_or(x).abs();
    let z = size.get(2).and_then(Value::as_f64).unwrap_or(y).abs();
    let half = match object["type"].as_str().unwrap_or("box") {
        "sphere" => [x, x, x],
        "cylinder" => [x, x, y],
        "capsule" => [x, x, y + x],
        "box" => [x, y, z],
        kind => {
            return Err(BenchmarkError::InvalidTrial(format!(
                "unsupported scenario collision geometry {kind}"
            )))
        }
    };
    if half.iter().all(|value| value.is_finite() && *value > 0.0) {
        Ok(half)
    } else {
        Err(BenchmarkError::InvalidTrial(
            "scenario collision geometry has invalid extents".into(),
        ))
    }
}

fn scenario_quaternion(object: &Value) -> [f64; 4] {
    object["quat"]
        .as_array()
        .and_then(|values| values.iter().map(Value::as_f64).collect::<Option<Vec<_>>>())
        .filter(|values| values.len() == 4 && values.iter().all(|value| value.is_finite()))
        .map(|values| [values[0], values[1], values[2], values[3]])
        .unwrap_or([1.0, 0.0, 0.0, 0.0])
}

fn set_randomized_grasp_standoff(
    bundle: &RobotBundle,
    manifest: &RobotManifest,
    scenario: &ManipulationScenario,
    qpos: &mut [f64],
    rng: &mut StdRng,
) -> Result<bool, BenchmarkError> {
    let model = embodiment_from_manifest(bundle, manifest);
    let Some(ee) = model
        .end_effectors
        .first()
        .map(|end_effector| end_effector.name.as_str())
    else {
        return Ok(false);
    };
    let Some(chain) = model.ee_joint_chain(ee) else {
        return Ok(false);
    };
    let object = object_named(&scenario.objects, &scenario.object_id)?;
    let mut target = vec3(&object["pos"])
        .ok_or_else(|| BenchmarkError::InvalidTrial("grasp target has no finite 3D pose".into()))?;
    target[2] += 0.05;
    let mut current_q = Vec::with_capacity(chain.len());
    for name in &chain {
        let joint = manifest
            .joints
            .iter()
            .find(|joint| joint.name == *name)
            .ok_or_else(|| {
                BenchmarkError::InvalidManifest(format!(
                    "end-effector chain references uninspected joint {name}"
                ))
            })?;
        let index = usize::try_from(joint.qpos_address).map_err(|_| {
            BenchmarkError::InvalidManifest(format!("negative qpos address for {name}"))
        })?;
        current_q.push(*qpos.get(index).ok_or_else(|| {
            BenchmarkError::InvalidManifest(format!("qpos address outside state for {name}"))
        })?);
    }
    let randomized_seed = current_q
        .iter()
        .map(|value| value + rng.gen_range(-0.08_f64..=0.08_f64))
        .collect::<Vec<_>>();
    let solved = realityos_semantics::kinematics::with_ik_q_seed(Some(&randomized_seed), || {
        realityos_semantics::kinematics::solve_ik(&model, &chain, ee, target, &current_q)
    });
    let Ok((arm_q, trace)) = solved else {
        return Ok(false);
    };
    if !realityos_semantics::kinematics::ik_residual_is_precise(trace.residual) {
        return Ok(false);
    }
    for (name, value) in chain.iter().zip(arm_q) {
        let joint = manifest
            .joints
            .iter()
            .find(|joint| joint.name == *name)
            .expect("chain joint validated above");
        let index =
            usize::try_from(joint.qpos_address).expect("non-negative qpos address validated above");
        qpos[index] = value;
    }

    let finger_joints = bundle
        .manifest
        .joint_aliases
        .iter()
        .filter(|(semantic, _)| semantic.to_ascii_lowercase().contains("finger"))
        .map(|(_, physical)| physical.as_str())
        .collect::<std::collections::BTreeSet<_>>();
    if finger_joints.len() != 2 {
        return Ok(false);
    }
    for name in finger_joints {
        let joint = manifest
            .joints
            .iter()
            .find(|joint| joint.name == name)
            .ok_or_else(|| {
                BenchmarkError::InvalidManifest(format!(
                    "gripper alias references uninspected joint {name}"
                ))
            })?;
        let index = usize::try_from(joint.qpos_address).map_err(|_| {
            BenchmarkError::InvalidManifest(format!("negative gripper qpos address for {name}"))
        })?;
        let value = qpos.get_mut(index).ok_or_else(|| {
            BenchmarkError::InvalidManifest(format!("qpos address outside state for {name}"))
        })?;
        *value = if joint.range[0].abs() > joint.range[1].abs() {
            joint.range[0]
        } else {
            joint.range[1]
        };
    }
    Ok(true)
}

fn set_randomized_push_standoff(
    bundle: &RobotBundle,
    manifest: &RobotManifest,
    scenario: &ManipulationScenario,
    qpos: &mut [f64],
    rng: &mut StdRng,
) -> Result<bool, BenchmarkError> {
    let model = embodiment_from_manifest(bundle, manifest);
    let Some(ee) = model
        .end_effectors
        .first()
        .map(|end_effector| end_effector.name.as_str())
    else {
        return Ok(false);
    };
    let Some(chain) = model.ee_joint_chain(ee) else {
        return Ok(false);
    };
    if chain.is_empty() {
        return Ok(false);
    }
    let object = object_named(&scenario.objects, &scenario.object_id)?;
    let center = vec3(&object["pos"])
        .ok_or_else(|| BenchmarkError::InvalidTrial("push target has no finite pose".into()))?;
    let half = super::execution::object_half_extents(object)?;
    let push_norm = scenario.push_dir[0].hypot(scenario.push_dir[1]);
    if !push_norm.is_finite() || push_norm <= 1e-9 {
        return Ok(false);
    }
    let push = [
        scenario.push_dir[0] / push_norm,
        scenario.push_dir[1] / push_norm,
    ];
    let direction = if push[0].abs() >= push[1].abs() {
        [push[0].signum(), 0.0, 0.0]
    } else {
        [0.0, push[1].signum(), 0.0]
    };
    let table = object_named(&scenario.objects, "table")?;
    let table_center = vec3(&table["pos"])
        .ok_or_else(|| BenchmarkError::InvalidTrial("push support has no finite pose".into()))?;
    let table_half = super::execution::object_half_extents(table)?;
    let support_top = table_center[2] + table_half[2];
    let Some(tool_radius) = super::execution::declared_ee_radius(&model, ee)
        .filter(|radius| radius.is_finite() && *radius > 0.0)
    else {
        return Ok(false);
    };

    // Match the shared reasoner's support-cleared side-contact construction,
    // then solve a randomized nearby seed to a collision-checked approach pose.
    let extent = if direction[0].abs() > 0.0 {
        half[0]
    } else {
        half[1]
    };
    const SUPPORT_CLEARANCE_MARGIN_M: f64 = 0.025;
    let target_z = center[2].max(support_top + tool_radius + SUPPORT_CLEARANCE_MARGIN_M);
    let vertical_excess = ((target_z - center[2]).abs() - half[2]).max(0.0);
    let cross_section = (tool_radius * tool_radius - vertical_excess * vertical_excess)
        .max(0.0)
        .sqrt();
    let mut target = [
        center[0] - direction[0] * (extent + cross_section),
        center[1] - direction[1] * (extent + cross_section),
        target_z,
    ];
    target[0] -= direction[0] * realityos_semantics::physical_reasoner::PUSH_PROBE_APPROACH_GAP_M;
    target[1] -= direction[1] * realityos_semantics::physical_reasoner::PUSH_PROBE_APPROACH_GAP_M;

    let mut current_q = Vec::with_capacity(chain.len());
    for name in &chain {
        let joint = manifest
            .joints
            .iter()
            .find(|joint| joint.name == *name)
            .ok_or_else(|| {
                BenchmarkError::InvalidManifest(format!(
                    "end-effector chain references uninspected joint {name}"
                ))
            })?;
        if joint.qpos_dim != 1 {
            return Ok(false);
        }
        let index = usize::try_from(joint.qpos_address).map_err(|_| {
            BenchmarkError::InvalidManifest(format!("negative qpos address for {name}"))
        })?;
        current_q.push(*qpos.get(index).ok_or_else(|| {
            BenchmarkError::InvalidManifest(format!("qpos address outside state for {name}"))
        })?);
    }
    let randomized_seed = current_q
        .iter()
        .map(|value| value + rng.gen_range(-0.08_f64..=0.08_f64))
        .collect::<Vec<_>>();
    let solution_has_safe_margins = |arm_q: &[f64]| {
        arm_q.len() == chain.len()
            && chain.iter().zip(arm_q).all(|(name, value)| {
                let Some(joint) = manifest.joints.iter().find(|joint| joint.name == *name) else {
                    return false;
                };
                if !value.is_finite()
                    || !joint.limited
                    || *value <= joint.range[0] + MIN_POSITIVE_JOINT_MARGIN_RAD
                    || *value >= joint.range[1] - MIN_POSITIVE_JOINT_MARGIN_RAD
                {
                    return false;
                }
                manifest
                    .actuators
                    .iter()
                    .find(|actuator| actuator.transmission_target == *name)
                    .is_some_and(|actuator| {
                        actuator.ctrllimited
                            && *value > actuator.ctrlrange[0] + MIN_POSITIVE_JOINT_MARGIN_RAD
                            && *value < actuator.ctrlrange[1] - MIN_POSITIVE_JOINT_MARGIN_RAD
                    })
            })
    };
    let quick_solution = realityos_semantics::kinematics::solve_ik_from_seed(
        &model,
        &chain,
        ee,
        target,
        &randomized_seed,
        &current_q,
    )
    .ok()
    .filter(|(arm_q, trace)| {
        realityos_semantics::kinematics::ik_residual_is_precise(trace.residual)
            && solution_has_safe_margins(arm_q)
    })
    .map(|(arm_q, _)| arm_q);
    let selected = if quick_solution.is_some() {
        quick_solution
    } else {
        let candidates =
            realityos_semantics::kinematics::with_ik_q_seed(Some(&randomized_seed), || {
                realityos_semantics::kinematics::solve_ik_candidates(
                    &model, &chain, ee, target, &current_q, 64,
                )
            });
        let Ok(candidates) = candidates else {
            return Ok(false);
        };
        candidates.into_iter().find_map(|(arm_q, trace)| {
            if !realityos_semantics::kinematics::ik_residual_is_precise(trace.residual)
                || arm_q.len() != chain.len()
            {
                return None;
            }
            solution_has_safe_margins(&arm_q).then_some(arm_q)
        })
    };
    let Some(arm_q) = selected else {
        return Ok(false);
    };
    for (name, value) in chain.iter().zip(arm_q) {
        let joint = manifest
            .joints
            .iter()
            .find(|joint| joint.name == *name)
            .expect("chain joint validated above");
        let index =
            usize::try_from(joint.qpos_address).expect("non-negative qpos address validated above");
        qpos[index] = value;
    }
    Ok(true)
}

fn add_randomized_obstacles(
    scenario: &mut ManipulationScenario,
    rng: &mut StdRng,
) -> Result<(), BenchmarkError> {
    let object = object_named(&scenario.objects, &scenario.object_id)?;
    let center = vec3(&object["pos"]).ok_or_else(|| {
        BenchmarkError::InvalidTrial("target object has no finite 3D pose".into())
    })?;
    let count = rng.gen_range(1..=3);
    for index in 0..count {
        let side = rng.gen_range(0.012..=0.025);
        let forward = rng.gen_range(-0.03..=0.08);
        let lateral = rng.gen_range(0.07..=0.14) * if index % 2 == 0 { 1.0 } else { -1.0 };
        let shape = if rng.gen_bool(0.25) {
            "cylinder"
        } else {
            "box"
        };
        let pos = [
            center[0] + scenario.push_dir[0] * forward - scenario.push_dir[1] * lateral,
            center[1] + scenario.push_dir[1] * forward + scenario.push_dir[0] * lateral,
            center[2],
        ];
        let size = if shape == "cylinder" {
            json!([side, side])
        } else {
            json!([side, side, side])
        };
        scenario.objects.push(json!({
            "name": format!("benchmark_obstacle_{index}"),
            "type": shape,
            "pos": pos,
            "size": size,
            "mass": 1.0,
            "friction": 0.8,
            "movable": false,
        }));
    }
    Ok(())
}

fn object_named<'a>(objects: &'a [Value], name: &str) -> Result<&'a Value, BenchmarkError> {
    objects
        .iter()
        .find(|object| object["name"] == name)
        .ok_or_else(|| BenchmarkError::InvalidTrial(format!("missing target object {name}")))
}

fn vec3(value: &Value) -> Option<[f64; 3]> {
    let values = value.as_array()?;
    let result = [
        values.first()?.as_f64()?,
        values.get(1)?.as_f64()?,
        values.get(2)?.as_f64()?,
    ];
    result
        .iter()
        .all(|number| number.is_finite())
        .then_some(result)
}
