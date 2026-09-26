//! POLICY OBSERVATION is separated from VERIFIER GROUND TRUTH.

use crate::normalize::RobotManifest;
use crate::task::TaskSpec;
use serde::{Deserialize, Serialize};
use serde_json::Value;
use std::collections::BTreeMap;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum VisionMode {
    State,
    PerfectPerception,
    Camera,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Detection {
    pub name: String,
    pub pose: Vec<f64>,
    #[serde(default)]
    pub orientation_wxyz: Option<Vec<f64>>,
    pub source: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PolicyObservation {
    pub robot_id: String,
    pub model_hash: String,
    pub episode_id: String,
    pub observation_id: String,
    pub timestamp_s: f64,
    pub task_id: String,
    pub task_instruction: String,
    pub mode: VisionMode,
    pub qpos: Vec<f64>,
    pub qvel: Vec<f64>,
    pub action_dim: usize,
    pub detections: Vec<Detection>,
    /// Contact-sensor pairs only; privileged force magnitudes are not exposed.
    #[serde(default)]
    pub contact_pairs: Vec<PolicyContactPair>,
    pub rgb: Option<Vec<u8>>,
    pub depth: Option<Vec<f32>>,
    #[serde(default)]
    pub camera_status: Option<String>,
    pub goal_xyz: Option<[f64; 3]>,
    pub goal_q: Option<Vec<f64>>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PolicyContactPair {
    pub body1: String,
    pub body2: String,
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct VerifierTruth {
    pub time_s: f64,
    pub qpos: Vec<f64>,
    pub qvel: Vec<f64>,
    pub qacc: Vec<f64>,
    pub ctrl: Vec<f64>,
    pub actuator_force: Vec<f64>,
    pub xpos: BTreeMap<String, Vec<f64>>,
    #[serde(default)]
    pub xquat: BTreeMap<String, Vec<f64>>,
    pub named_pos: BTreeMap<String, Vec<f64>>,
    #[serde(default)]
    pub site_xquat: BTreeMap<String, Vec<f64>>,
    pub contacts: Vec<ContactTruth>,
    pub kinetic_energy: f64,
    pub com: Vec<f64>,
    pub nan: bool,
    pub ncon: i64,
    pub interval_max_speed: f64,
    pub interval_max_force: f64,
    pub zone_entries: Vec<String>,
    pub grasped: Vec<String>,
    pub placed: Vec<(String, String)>,
    #[serde(default)]
    pub effort_bound_unavailable: bool,
    #[serde(default)]
    pub last_ee_pos: Option<Vec<f64>>,
    #[serde(default)]
    pub last_ee_time: Option<f64>,
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct ContactTruth {
    pub body1: String,
    pub body2: String,
    pub dist: f64,
    pub force: f64,
    #[serde(default)]
    pub group1: i32,
    #[serde(default)]
    pub group2: i32,
    #[serde(default)]
    pub normal_force: Option<f64>,
    #[serde(default)]
    pub tangential_force: Option<f64>,
    #[serde(default)]
    pub force_available: bool,
}

impl VerifierTruth {
    pub fn from_mujoco_state(state: &Value) -> Self {
        let mut xpos = BTreeMap::new();
        if let Some(obj) = state.get("xpos").and_then(|v| v.as_object()) {
            for (k, v) in obj {
                xpos.insert(k.clone(), crate::mujoco_exec::json_f64_vec(v));
            }
        }
        let mut xquat = BTreeMap::new();
        if let Some(obj) = state.get("xquat").and_then(|v| v.as_object()) {
            for (k, v) in obj {
                xquat.insert(k.clone(), crate::mujoco_exec::json_f64_vec(v));
            }
        }
        let mut named = BTreeMap::new();
        let mut site_xquat = BTreeMap::new();
        if let Some(obj) = state.get("site_xquat").and_then(|v| v.as_object()) {
            for (k, v) in obj {
                site_xquat.insert(k.clone(), crate::mujoco_exec::json_f64_vec(v));
            }
        }
        if let Some(obj) = state.get("sites").and_then(|v| v.as_object()) {
            for (k, v) in obj {
                named.insert(k.clone(), crate::mujoco_exec::json_f64_vec(v));
            }
        }
        if let Some(obj) = state.get("named_pos").and_then(|v| v.as_object()) {
            for (k, v) in obj {
                named.insert(k.clone(), crate::mujoco_exec::json_f64_vec(v));
            }
        }
        let forces = crate::mujoco_exec::json_f64_vec(&state["contact_forces"]);
        let contacts = state
            .get("contacts")
            .and_then(|v| v.as_array())
            .map(|arr| {
                arr.iter()
                    .enumerate()
                    .map(|(i, c)| ContactTruth {
                        body1: c["body1"].as_str().unwrap_or("").into(),
                        body2: c["body2"].as_str().unwrap_or("").into(),
                        dist: c["dist"].as_f64().unwrap_or(0.0),
                        force: forces.get(i).copied().unwrap_or(0.0),
                        group1: c["group1"].as_i64().unwrap_or(-1) as i32,
                        group2: c["group2"].as_i64().unwrap_or(-1) as i32,
                        normal_force: c.get("normal_force").and_then(|v| v.as_f64()),
                        tangential_force: c.get("tangential_force").and_then(|v| v.as_f64()),
                        force_available: c["force_available"].as_bool().unwrap_or(false),
                    })
                    .collect()
            })
            .unwrap_or_default();
        Self {
            time_s: state["time"].as_f64().unwrap_or(0.0),
            qpos: crate::mujoco_exec::json_f64_vec(&state["qpos"]),
            qvel: crate::mujoco_exec::json_f64_vec(&state["qvel"]),
            qacc: crate::mujoco_exec::json_f64_vec(&state["qacc"]),
            ctrl: crate::mujoco_exec::json_f64_vec(&state["ctrl"]),
            actuator_force: crate::mujoco_exec::json_f64_vec(&state["actuator_force"]),
            xpos,
            xquat,
            named_pos: named,
            site_xquat,
            contacts,
            kinetic_energy: state["kinetic_energy"].as_f64().unwrap_or(0.0),
            com: crate::mujoco_exec::json_f64_vec(&state["subtree_com"]),
            nan: state["nan"].as_bool().unwrap_or(false),
            ncon: state["ncon"].as_i64().unwrap_or(0),
            interval_max_speed: state["interval_max_speed"].as_f64().unwrap_or(0.0),
            interval_max_force: state["interval_max_force"].as_f64().unwrap_or(0.0),
            zone_entries: Vec::new(),
            grasped: Vec::new(),
            placed: Vec::new(),
            effort_bound_unavailable: false,
            last_ee_pos: None,
            last_ee_time: None,
        }
    }

    pub fn ee_pos(&self) -> Option<Vec<f64>> {
        self.named_pos
            .get("ee")
            .cloned()
            .or_else(|| self.named_pos.get("tip").cloned())
            .or_else(|| self.named_pos.values().next().cloned())
    }
}

pub fn policy_observation(
    manifest: &RobotManifest,
    episode_id: &str,
    obs_id: &str,
    task: &TaskSpec,
    mode: VisionMode,
    truth: &VerifierTruth,
    delay_s: f64,
    dropout: bool,
) -> PolicyObservation {
    let mut qpos = truth.qpos.clone();
    let mut qvel = truth.qvel.clone();
    let mut detections = Vec::new();
    let mut contact_pairs = Vec::new();
    if dropout {
        qpos.clear();
        qvel.clear();
    }
    if mode == VisionMode::PerfectPerception {
        for (name, pose) in &truth.xpos {
            if name != "world" {
                detections.push(Detection {
                    name: name.clone(),
                    pose: pose.clone(),
                    orientation_wxyz: truth.xquat.get(name).cloned(),
                    source: "perfect_perception_from_sim_truth".into(),
                });
            }
        }
        contact_pairs.extend(
            truth
                .contacts
                .iter()
                .filter(|contact| contact.dist.is_finite() && contact.dist <= 1e-3)
                .map(|contact| PolicyContactPair {
                    body1: contact.body1.clone(),
                    body2: contact.body2.clone(),
                }),
        );
    }
    let camera_status = if mode == VisionMode::Camera {
        Some("NOT_IMPLEMENTED_IN_VERIFY_V1".into())
    } else {
        None
    };
    PolicyObservation {
        robot_id: manifest.robot_id.clone(),
        model_hash: manifest.model_hash.clone(),
        episode_id: episode_id.into(),
        observation_id: obs_id.into(),
        timestamp_s: truth.time_s - delay_s,
        task_id: task.id(),
        task_instruction: task.id(),
        mode,
        qpos,
        qvel,
        action_dim: manifest.nu.max(1) as usize,
        detections,
        contact_pairs,
        rgb: None,
        depth: None,
        camera_status,
        goal_xyz: match task {
            TaskSpec::Reach { target, .. } => Some(*target),
            _ => None,
        },
        goal_q: match task {
            TaskSpec::JointTrack { target, .. } => Some(target.clone()),
            _ => None,
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dropout_does_not_clear_verifier_truth() {
        let truth = VerifierTruth {
            qpos: vec![0.2],
            nan: false,
            ..VerifierTruth::default()
        };
        assert_eq!(truth.qpos[0], 0.2);
        let m = dummy_manifest();
        let task = TaskSpec::Hold { duration_s: 0.1 };
        let obs = policy_observation(&m, "e", "o", &task, VisionMode::State, &truth, 0.0, true);
        assert!(obs.qpos.is_empty());
        assert!(!truth.qpos.is_empty());
    }

    fn dummy_manifest() -> RobotManifest {
        RobotManifest {
            robot_id: "x".into(),
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
            actuators: vec![],
            sensors: vec![],
            cameras: vec![],
            bodies: vec![],
            sites: vec![],
            site_records: vec![],
            derived: crate::normalize::DerivedInterface {
                base_type: crate::bundle::BaseType::Fixed,
                actuated_dofs: vec![],
                passive_dofs: vec![],
                end_effector_chains: vec![],
                end_effector_joint_chains: vec![],
                actuator_coverage: 0.0,
                potentially_uncontrollable_joints: vec![],
            },
            model_hash: "h".into(),
            source_hash: "s".into(),
            mujoco_version: "3".into(),
            source_format: "mjcf".into(),
            lost_features: vec![],
            support_bodies: vec![],
            collision_groups: Default::default(),
            geoms: Vec::new(),
            metal: false,
            evidence_status: crate::honesty::SIMULATION_ONLY.into(),
        }
    }
}
