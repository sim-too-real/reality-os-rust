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
use realityos_semantics::transition_validity::{validate_transition, GeometryVerdict};
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};
use sha2::{Digest, Sha256};

pub const BENCHMARK_SCHEMA_VERSION: u32 = 4;
const MAX_SAFE_INITIAL_QPOS_ATTEMPTS: usize = 64;
pub const PILOT_SEEDS: std::ops::Range<u64> = 9000..9020;
pub const PUBLIC_SEEDS: std::ops::Range<u64> = 0..100;
pub const HOLDOUT_SEEDS: std::ops::Range<u64> = 10000..10100;

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
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct EmbodimentRegistration {
    pub robot_id: String,
    pub bundle_path: String,
    pub role: EmbodimentRole,
    pub tasks: Vec<BenchmarkTask>,
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
        if self.schema_version != BENCHMARK_SCHEMA_VERSION {
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

    let axes = end_effector_axes(manifest);
    let planar = arm_is_planar(axes.iter());
    let scenario_index = index as usize;
    let mut scenario = match task {
        BenchmarkTask::PushProbeReplan => push_scenario(seed, planar, scenario_index),
        BenchmarkTask::PinchGraspLift => grasp_scenario(seed, planar, scenario_index),
    };

    if planar && task == BenchmarkTask::PushProbeReplan {
        // The planar-arm embodiment moves its tool in the table plane. Keep
        // the randomized block's bottom on the support and its center at that
        // interaction height so the benchmark tests push reasoning rather
        // than an impossible vertical reach.
        let table = scenario
            .objects
            .iter_mut()
            .find(|object| object["name"] == "table")
            .ok_or_else(|| BenchmarkError::InvalidTrial("planar scenario has no table".into()))?;
        let table_pos = table["pos"]
            .as_array_mut()
            .ok_or_else(|| BenchmarkError::InvalidTrial("planar table has no position".into()))?;
        table_pos[2] = json!(0.09);
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
        let half_z = 0.02;
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
        let (normalized_size, half_z) = if shape == "cylinder" {
            let source_radius = target_size[0].as_f64().ok_or_else(|| {
                BenchmarkError::InvalidTrial("planar grasp target radius is malformed".into())
            })?;
            (
                vec![source_radius + 0.005, source_y + 0.005],
                source_y + 0.005,
            )
        } else {
            (vec![0.01, source_y + 0.01, 0.02], 0.02)
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
    if task == BenchmarkTask::PushProbeReplan {
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
    let mut expectation = if (scenario.polarity
        == crate::manipulation_scenarios::Polarity::Positive
        || probe_capable_ambiguous_push)
        && scenario.expected_refusal.is_none()
    {
        TrialExpectation::ExpectedSuccess
    } else {
        TrialExpectation::ExpectedRefusal
    };
    let mut initial_qpos = sample_initial_qpos(manifest, &mut rng)?;
    if expectation == TrialExpectation::ExpectedSuccess {
        let mut found_standoff = task != BenchmarkTask::PinchGraspLift;
        let mut safe_initial_qpos = None;
        let attempts = if manifest.geometry_inspection_complete {
            MAX_SAFE_INITIAL_QPOS_ATTEMPTS
        } else {
            1
        };
        for _ in 0..attempts {
            let mut candidate = sample_initial_qpos(manifest, &mut rng)?;
            if task == BenchmarkTask::PinchGraspLift
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
            if !manifest.geometry_inspection_complete
                || initial_configuration_is_clear(bundle, manifest, &scenario, &candidate)?
            {
                safe_initial_qpos = Some(candidate);
                break;
            }
        }
        if let Some(candidate) = safe_initial_qpos {
            initial_qpos = candidate;
        } else {
            scenario.expected_refusal = Some(
                if !found_standoff {
                    "GRASP_STANDOFF_UNREACHABLE"
                } else {
                    "NO_SAFE_INITIAL_CONFIGURATION"
                }
                .into(),
            );
            expectation = TrialExpectation::ExpectedRefusal;
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

    Ok(ResolvedBenchmarkTrial {
        schema_version: BENCHMARK_SCHEMA_VERSION,
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
    })
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

fn end_effector_axes(manifest: &RobotManifest) -> Vec<[f64; 3]> {
    let Some(chain) = manifest.derived.end_effector_joint_chains.first() else {
        return manifest
            .joints
            .iter()
            .filter(|joint| matches!(joint.joint_type.as_str(), "hinge" | "slide"))
            .filter_map(|joint| joint.axis)
            .collect();
    };
    chain
        .iter()
        .filter_map(|name| manifest.joints.iter().find(|joint| &joint.name == name))
        .filter_map(|joint| joint.axis)
        .collect()
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
