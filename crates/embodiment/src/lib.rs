//! Robot-agnostic bodies + environment-agnostic worlds.
//! Joint limits from robot files. Meshes/metal not imported.

use realityos_physics::{in_limits, G0};
use serde::{Deserialize, Serialize};
use thiserror::Error;

#[derive(Debug, Error, Clone, PartialEq, Eq)]
pub enum EmbodimentError {
    #[error("unknown robot {0}")]
    UnknownRobot(String),
    #[error("parse: {0}")]
    Parse(String),
    #[error("dof mismatch")]
    DofMismatch,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct JointSpec {
    pub name: String,
    pub q_min: f64,
    pub q_max: f64,
    pub tau_max: f64,
    pub dq_max: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RobotModel {
    pub id: String,
    pub kind: String,
    pub source: String,
    pub joints: Vec<JointSpec>,
}

impl RobotModel {
    pub fn dof(&self) -> usize {
        self.joints.len()
    }

    pub fn tau_max(&self) -> Vec<f64> {
        self.joints.iter().map(|j| j.tau_max).collect()
    }

    pub fn dq_max(&self) -> Vec<f64> {
        self.joints.iter().map(|j| j.dq_max).collect()
    }

    pub fn q_min(&self) -> Vec<f64> {
        self.joints.iter().map(|j| j.q_min).collect()
    }

    pub fn q_max(&self) -> Vec<f64> {
        self.joints.iter().map(|j| j.q_max).collect()
    }

    pub fn check_q(&self, q: &[f64]) -> Result<(), EmbodimentError> {
        if q.len() != self.dof() {
            return Err(EmbodimentError::DofMismatch);
        }
        for (j, qi) in self.joints.iter().zip(q) {
            if !in_limits(*qi, j.q_min, j.q_max).unwrap_or(false) {
                return Err(EmbodimentError::Parse(format!(
                    "joint {} out of range",
                    j.name
                )));
            }
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Environment {
    pub id: String,
    pub g_m_s2: f64,
    pub mu: f64,
    pub air_density_kg_m3: f64,
    pub note: String,
}

impl Environment {
    pub fn earth() -> Self {
        Self {
            id: "earth_indoor".into(),
            g_m_s2: G0,
            mu: 0.6,
            air_density_kg_m3: 1.225,
            note: "standard g, dry indoor friction screen".into(),
        }
    }

    pub fn moon() -> Self {
        Self {
            id: "moon".into(),
            g_m_s2: 1.62,
            mu: 0.4,
            air_density_kg_m3: 0.0,
            note: "lunar g; vacuum density; SIM screen".into(),
        }
    }

    pub fn ice() -> Self {
        Self {
            id: "ice".into(),
            g_m_s2: G0,
            mu: 0.05,
            air_density_kg_m3: 1.225,
            note: "low Coulomb μ screen".into(),
        }
    }

    pub fn high_g() -> Self {
        Self {
            id: "high_g".into(),
            g_m_s2: 20.0,
            mu: 0.6,
            air_density_kg_m3: 1.225,
            note: "elevated g screen (not a planet claim)".into(),
        }
    }

    pub fn catalog() -> Vec<Self> {
        vec![Self::earth(), Self::moon(), Self::ice(), Self::high_g()]
    }
}

pub fn parse_robot(json: &str) -> Result<RobotModel, EmbodimentError> {
    serde_json::from_str(json).map_err(|e| EmbodimentError::Parse(e.to_string()))
}

pub fn load_embedded(id: &str) -> Result<RobotModel, EmbodimentError> {
    let raw = match id {
        "uniaxial" => include_str!("../../../robots/uniaxial.json"),
        "arm6" => include_str!("../../../robots/arm6.json"),
        "wheeled" => include_str!("../../../robots/wheeled.json"),
        "unitree_h1" => include_str!("../../../robots/unitree_h1.json"),
        other => return Err(EmbodimentError::UnknownRobot(other.into())),
    };
    parse_robot(raw)
}

pub fn catalog() -> Vec<RobotModel> {
    ["uniaxial", "arm6", "wheeled", "unitree_h1"]
        .iter()
        .map(|id| load_embedded(id).expect("embedded robot json"))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn h1_has_nineteen_joints_from_xml_ranges() {
        let r = load_embedded("unitree_h1").unwrap();
        assert_eq!(r.dof(), 19);
        assert!(r.joints.iter().any(|j| j.name == "left_knee"));
    }

    #[test]
    fn environments_change_g() {
        assert!(Environment::moon().g_m_s2 < Environment::earth().g_m_s2);
    }
}
