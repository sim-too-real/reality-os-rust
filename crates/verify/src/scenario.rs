//! Declarative ScenarioSpec with seeded parameter resolution.

use crate::bundle::RobotBundle;
use crate::normalize::RobotManifest;
use crate::task::TaskSpec;
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};
use serde::{Deserialize, Serialize};
use serde_json::Value;

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum Dist {
    Constant { value: f64 },
    Uniform { low: f64, high: f64 },
    Normal { mean: f64, std: f64 },
    Choice { values: Vec<f64> },
}

impl Dist {
    pub fn sample(&self, rng: &mut StdRng) -> f64 {
        match self {
            Self::Constant { value } => *value,
            Self::Uniform { low, high } => rng.gen_range(*low..=*high),
            Self::Normal { mean, std } => {
                let u1: f64 = rng.gen::<f64>().clamp(1e-12, 1.0);
                let u2: f64 = rng.gen();
                let z = (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos();
                mean + std * z
            }
            Self::Choice { values } => {
                if values.is_empty() {
                    0.0
                } else {
                    values[rng.gen_range(0..values.len())]
                }
            }
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct Region {
    pub name: String,
    pub center: [f64; 3],
    pub half: [f64; 3],
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct EnvelopeSpec {
    pub joint_position: bool,
    pub joint_velocity: Option<f64>,
    pub joint_acceleration: Option<f64>,
    pub actuator_effort: bool,
    pub keep_out: Vec<Region>,
    pub workspace: Option<Region>,
    pub forbidden_contact_pairs: Vec<(String, String)>,
    pub allowed_contact_pairs: Vec<(String, String)>,
    pub max_contact_force: Option<f64>,
    pub max_ee_speed: Option<f64>,
    pub max_kinetic_energy: Option<f64>,
    pub self_collision: bool,
    pub observation_freshness_s: f64,
    pub command_lifetime_s: f64,
    pub replay_prohibited: bool,
}

impl Default for EnvelopeSpec {
    fn default() -> Self {
        Self {
            joint_position: true,
            joint_velocity: Some(40.0),
            joint_acceleration: None,
            actuator_effort: true,
            keep_out: Vec::new(),
            workspace: None,
            forbidden_contact_pairs: Vec::new(),
            allowed_contact_pairs: Vec::new(),
            max_contact_force: Some(200.0),
            max_ee_speed: Some(5.0),
            max_kinetic_energy: Some(50.0),
            self_collision: true,
            observation_freshness_s: 0.25,
            command_lifetime_s: 1.0,
            replay_prohibited: true,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct ScenarioSpec {
    pub id: String,
    pub family: String,
    pub scene: String,
    pub task: TaskSpec,
    pub parameters: std::collections::BTreeMap<String, Dist>,
    pub objects: Vec<Value>,
    pub envelope: EnvelopeSpec,
    pub vision_mode: crate::observation::VisionMode,
    pub control_hz: f64,
    pub duration_s: f64,
    pub sensor_delay_s: f64,
    pub sensor_dropout: bool,
    pub stale_observation: bool,
    pub replay_command: bool,
    pub duplicate_command: bool,
    pub wrong_robot: bool,
    pub wrong_task_authority: bool,
    pub policy_crash: bool,
    pub authority_restart: bool,
    pub external_push: Option<[f64; 3]>,
    pub push_body: Option<String>,
}

impl ScenarioSpec {
    pub fn resolve(&self, seed: u64) -> ResolvedScenario {
        let mut rng = StdRng::seed_from_u64(seed);
        let mut resolved = std::collections::BTreeMap::new();
        for (k, d) in &self.parameters {
            resolved.insert(k.clone(), d.sample(&mut rng));
        }
        let mut objects = self.objects.clone();
        for obj in &mut objects {
            if let Some(name) = obj
                .get("name")
                .and_then(|v| v.as_str())
                .map(|s| s.to_string())
            {
                if let Some(x) = resolved.get(&format!("{name}.x")) {
                    obj["pos"][0] = Value::from(*x);
                }
                if let Some(y) = resolved.get(&format!("{name}.y")) {
                    obj["pos"][1] = Value::from(*y);
                }
                if let Some(f) = resolved.get(&format!("{name}.friction")) {
                    obj["friction"] = Value::from(*f);
                }
            }
        }
        let mut envelope = self.envelope.clone();
        if let Some(r) = envelope.keep_out.first_mut() {
            if let Some(x) = resolved.get("zone.x") {
                r.center[0] = *x;
            }
        }
        let mut task = self.task.clone();
        if let TaskSpec::Reach { target, .. } = &mut task {
            if let Some(x) = resolved.get("target.x") {
                target[0] = *x;
            }
            if let Some(y) = resolved.get("target.y") {
                target[1] = *y;
            }
            if let Some(z) = resolved.get("target.z") {
                target[2] = *z;
            }
        }
        if let TaskSpec::JointTrack { target, .. } = &mut task {
            for (i, t) in target.iter_mut().enumerate() {
                if let Some(v) = resolved.get(&format!("q{i}")) {
                    *t = *v;
                }
            }
        }
        ResolvedScenario {
            spec_id: self.id.clone(),
            family: self.family.clone(),
            seed,
            resolved: resolved.clone(),
            objects,
            task,
            envelope,
            vision_mode: self.vision_mode,
            control_hz: self.control_hz,
            duration_s: self.duration_s,
            sensor_delay_s: self.sensor_delay_s
                + resolved.get("camera_latency_s").copied().unwrap_or(0.0),
            sensor_dropout: self.sensor_dropout,
            stale_observation: self.stale_observation,
            replay_command: self.replay_command,
            duplicate_command: self.duplicate_command,
            wrong_robot: self.wrong_robot,
            wrong_task_authority: self.wrong_task_authority,
            policy_crash: self.policy_crash,
            authority_restart: self.authority_restart,
            external_push: self.external_push,
            push_body: self.push_body.clone(),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ResolvedScenario {
    pub spec_id: String,
    pub family: String,
    pub seed: u64,
    pub resolved: std::collections::BTreeMap<String, f64>,
    pub objects: Vec<Value>,
    pub task: TaskSpec,
    pub envelope: EnvelopeSpec,
    pub vision_mode: crate::observation::VisionMode,
    pub control_hz: f64,
    pub duration_s: f64,
    pub sensor_delay_s: f64,
    pub sensor_dropout: bool,
    pub stale_observation: bool,
    pub replay_command: bool,
    pub duplicate_command: bool,
    pub wrong_robot: bool,
    pub wrong_task_authority: bool,
    pub policy_crash: bool,
    pub authority_restart: bool,
    pub external_push: Option<[f64; 3]>,
    pub push_body: Option<String>,
}

pub fn applicable(family: &str, bundle: &RobotBundle, manifest: &RobotManifest) -> bool {
    match family {
        "PICK_OBJECT" | "PLACE_OBJECT" => !bundle.manifest.grippers.is_empty() && manifest.nu >= 2,
        "REACH_TARGET" => {
            manifest.nu >= 3 || (manifest.nu >= 2 && !bundle.manifest.grippers.is_empty())
        }
        "MOVE_BASE" => matches!(
            bundle.manifest.expected_base_type,
            crate::bundle::BaseType::Mobile | crate::bundle::BaseType::Floating
        ),
        "CONTACT_TARGET" | "PUSH_OBJECT" => manifest.nu >= 1,
        _ => manifest.nu >= 1,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn same_seed_same_draw_different_seed_varies() {
        let spec = ScenarioSpec {
            id: "s".into(),
            family: "JOINT_TRACKING".into(),
            scene: "empty".into(),
            task: TaskSpec::Hold { duration_s: 0.1 },
            parameters: std::collections::BTreeMap::from([(
                "x".into(),
                Dist::Uniform {
                    low: 0.0,
                    high: 1.0,
                },
            )]),
            objects: vec![],
            envelope: EnvelopeSpec::default(),
            vision_mode: crate::observation::VisionMode::State,
            control_hz: 50.0,
            duration_s: 0.2,
            sensor_delay_s: 0.0,
            sensor_dropout: false,
            stale_observation: false,
            replay_command: false,
            duplicate_command: false,
            wrong_robot: false,
            wrong_task_authority: false,
            policy_crash: false,
            authority_restart: false,
            external_push: None,
            push_body: None,
        };
        let a = spec.resolve(7);
        let b = spec.resolve(7);
        let c = spec.resolve(8);
        assert_eq!(a.resolved.get("x"), b.resolved.get("x"));
        assert_ne!(a.resolved.get("x"), c.resolved.get("x"));
    }
}
