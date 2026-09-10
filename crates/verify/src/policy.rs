//! Observation -> ActionProposal. All policy kinds produce the same proposal type.

use serde::{Deserialize, Serialize};
use thiserror::Error;

#[derive(Debug, Error, Clone, PartialEq, Eq)]
pub enum PolicyError {
    #[error("policy_crash:{0}")]
    Crash(String),
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct ActionProposal {
    pub robot_id: String,
    pub model_hash: String,
    pub episode_id: String,
    pub observation_id: String,
    pub observation_timestamp: f64,
    pub task_id: String,
    pub action: Vec<f64>,
    pub control_mode: String,
    pub requested_horizon_s: f64,
    pub confidence: Option<f64>,
    pub policy_id: String,
    pub policy_version: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PolicyKind {
    Scripted,
    Pd,
    Ik,
    Trajectory,
    Rl,
    Vla,
    Remote,
    Teleop,
}

pub trait Policy: Send {
    fn identity(&self) -> (&str, &str);
    fn kind(&self) -> PolicyKind;
    fn propose(
        &mut self,
        obs: &crate::observation::PolicyObservation,
    ) -> Result<ActionProposal, PolicyError>;
}

pub struct PdPolicy {
    pub id: String,
    pub version: String,
    pub kp: f64,
    pub kd: f64,
    pub target: Vec<f64>,
    pub ranges: Vec<[f64; 2]>,
    pub position_mask: Vec<bool>,
}

impl PdPolicy {
    pub fn new(target: Vec<f64>, ranges: Vec<[f64; 2]>, position_mask: Vec<bool>) -> Self {
        Self {
            id: "pd".into(),
            version: "1".into(),
            kp: 4.0,
            kd: 0.4,
            target,
            ranges,
            position_mask,
        }
    }
}

impl Policy for PdPolicy {
    fn identity(&self) -> (&str, &str) {
        (&self.id, &self.version)
    }
    fn kind(&self) -> PolicyKind {
        PolicyKind::Pd
    }
    fn propose(
        &mut self,
        obs: &crate::observation::PolicyObservation,
    ) -> Result<ActionProposal, PolicyError> {
        let n = self.ranges.len().max(1);
        let mut action = vec![0.0; n];
        for (i, slot) in action.iter_mut().enumerate() {
            let q = obs.qpos.get(i).copied().unwrap_or(0.0);
            let v = obs.qvel.get(i).copied().unwrap_or(0.0);
            let t = self.target.get(i).copied().unwrap_or(0.0);
            let [lo, hi] = self.ranges.get(i).copied().unwrap_or([-1.0, 1.0]);
            let position = self.position_mask.get(i).copied().unwrap_or(false);
            *slot = if position {
                t.clamp(lo, hi)
            } else {
                ((t - q) * self.kp - v * self.kd).clamp(lo, hi)
            };
        }
        Ok(proposal_from_obs(
            obs,
            action,
            "effort",
            self.id.as_str(),
            self.version.as_str(),
        ))
    }
}

pub struct ScriptedHold {
    pub id: String,
}

impl Default for ScriptedHold {
    fn default() -> Self {
        Self {
            id: "scripted_hold".into(),
        }
    }
}

impl Policy for ScriptedHold {
    fn identity(&self) -> (&str, &str) {
        (&self.id, "1")
    }
    fn kind(&self) -> PolicyKind {
        PolicyKind::Scripted
    }
    fn propose(
        &mut self,
        obs: &crate::observation::PolicyObservation,
    ) -> Result<ActionProposal, PolicyError> {
        let n = obs.action_dim.max(1);
        Ok(proposal_from_obs(obs, vec![0.0; n], "hold", &self.id, "1"))
    }
}

pub struct ScriptedSetpoint {
    pub id: String,
    pub action: Vec<f64>,
}

impl Policy for ScriptedSetpoint {
    fn identity(&self) -> (&str, &str) {
        (&self.id, "1")
    }
    fn kind(&self) -> PolicyKind {
        PolicyKind::Scripted
    }
    fn propose(
        &mut self,
        obs: &crate::observation::PolicyObservation,
    ) -> Result<ActionProposal, PolicyError> {
        Ok(proposal_from_obs(
            obs,
            self.action.clone(),
            "setpoint",
            &self.id,
            "1",
        ))
    }
}

pub struct CrashingPolicy;

impl Policy for CrashingPolicy {
    fn identity(&self) -> (&str, &str) {
        ("crash", "1")
    }
    fn kind(&self) -> PolicyKind {
        PolicyKind::Scripted
    }
    fn propose(
        &mut self,
        _obs: &crate::observation::PolicyObservation,
    ) -> Result<ActionProposal, PolicyError> {
        Err(PolicyError::Crash("intentional_policy_crash".into()))
    }
}

pub fn proposal_from_obs(
    obs: &crate::observation::PolicyObservation,
    action: Vec<f64>,
    mode: &str,
    policy_id: &str,
    version: &str,
) -> ActionProposal {
    ActionProposal {
        robot_id: obs.robot_id.clone(),
        model_hash: obs.model_hash.clone(),
        episode_id: obs.episode_id.clone(),
        observation_id: obs.observation_id.clone(),
        observation_timestamp: obs.timestamp_s,
        task_id: obs.task_id.clone(),
        action,
        control_mode: mode.into(),
        requested_horizon_s: 0.02,
        confidence: Some(1.0),
        policy_id: policy_id.into(),
        policy_version: version.into(),
    }
}
