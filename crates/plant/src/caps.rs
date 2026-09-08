use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PlantCaps {
    pub plant_id: String,
    pub kind: String,
    pub action_dim: usize,
    pub max_action: Vec<f64>,
    pub online: bool,
    pub has_driver: bool,
    pub sim_backend: String,
}

impl PlantCaps {
    pub fn sim(plant_id: impl Into<String>, action_dim: usize, max_action: f64) -> Self {
        Self {
            plant_id: plant_id.into(),
            kind: "sim".into(),
            action_dim,
            max_action: vec![max_action; action_dim.max(1)],
            online: false,
            has_driver: true,
            sim_backend: "analytic".into(),
        }
    }

    pub fn hardware_stub(plant_id: impl Into<String>, action_dim: usize) -> Self {
        let mut c = Self::sim(plant_id, action_dim, 1.0);
        c.kind = "hardware_stub".into();
        c.has_driver = false;
        c
    }
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct ActionParams {
    pub values: Vec<(String, f64)>,
}

impl ActionParams {
    pub fn empty() -> Self {
        Self { values: Vec::new() }
    }

    pub fn get(&self, key: &str) -> Option<f64> {
        self.values.iter().find(|(k, _)| k == key).map(|(_, v)| *v)
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PlantRealized {
    pub ok: bool,
    pub values: Vec<(String, f64)>,
    pub metal: bool,
}

impl PlantRealized {
    pub fn sim(pairs: impl IntoIterator<Item = (String, f64)>) -> Self {
        Self {
            ok: true,
            values: pairs.into_iter().collect(),
            metal: false,
        }
    }
}
