//! World belief, scenario, and operating envelope contracts.

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SurfaceBelief {
    pub frame_id: String,
    pub mu: Option<f64>,
    pub mu_assumed: bool,
    pub slope_rad: Option<f64>,
    pub provenance: String,
    pub stamp_s: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct WorldBelief {
    pub transform_epoch: String,
    pub surfaces: Vec<SurfaceBelief>,
    pub objects: Vec<String>,
    pub stamp_s: f64,
}

impl WorldBelief {
    pub fn empty(epoch: impl Into<String>, stamp_s: f64) -> Self {
        Self {
            transform_epoch: epoch.into(),
            surfaces: Vec::new(),
            objects: Vec::new(),
            stamp_s,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ScenarioSpec {
    pub id: String,
    pub embodiment_ids: Vec<String>,
    pub task: String,
    pub invariants: Vec<String>,
    pub success: String,
    pub failure: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OperatingEnvelope {
    pub embodiment_hash: String,
    pub skills: Vec<String>,
    pub max_speed: Option<f64>,
    pub human_proximity: bool,
}

impl OperatingEnvelope {
    pub fn contains_skill(&self, skill: &str) -> bool {
        self.skills.iter().any(|s| s == skill)
    }
}
