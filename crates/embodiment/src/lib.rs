//! Robot-agnostic bodies + environment-agnostic worlds.
//! `EmbodimentGraph` is the canonical body. Robot product names are fixture IDs only.

use realityos_kernel::{capabilities_for_kind, Capability};
use realityos_physics::{in_limits, G0};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use thiserror::Error;

mod import;
mod world;
pub use import::{import_mjcf, import_urdf};
pub use world::{OperatingEnvelope, ScenarioSpec, SurfaceBelief, WorldBelief};

#[derive(Debug, Error, Clone, PartialEq, Eq)]
pub enum EmbodimentError {
    #[error("unknown robot {0}")]
    UnknownRobot(String),
    #[error("parse: {0}")]
    Parse(String),
    #[error("dof mismatch")]
    DofMismatch,
}

fn default_axis() -> [f64; 3] {
    [0.0, 0.0, 1.0]
}

fn default_joint_type() -> String {
    "revolute".into()
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct JointSpec {
    pub name: String,
    pub q_min: f64,
    pub q_max: f64,
    pub tau_max: f64,
    pub dq_max: f64,
    #[serde(default = "default_axis")]
    pub axis: [f64; 3],
    #[serde(default = "default_joint_type")]
    pub joint_type: String,
    #[serde(default)]
    pub origin_xyz: [f64; 3],
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LinkSpec {
    pub name: String,
    pub parent_joint: Option<String>,
    pub mass_kg: Option<f64>,
    #[serde(default)]
    pub com_m: Option<[f64; 3]>,
    #[serde(default)]
    pub inertia_diag: Option<[f64; 3]>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FrameSpec {
    pub name: String,
    pub parent: String,
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct CapabilityManifest {
    pub items: Vec<Capability>,
}

impl CapabilityManifest {
    pub fn from_kind(kind: &str) -> Self {
        Self {
            items: capabilities_for_kind(kind),
        }
    }

    pub fn has(&self, cap: Capability) -> bool {
        self.items.contains(&cap)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EmbodimentGraph {
    pub id: String,
    pub kind: String,
    pub source: String,
    pub source_hash: String,
    pub joints: Vec<JointSpec>,
    pub links: Vec<LinkSpec>,
    pub frames: Vec<FrameSpec>,
    pub capabilities: CapabilityManifest,
    #[serde(default)]
    pub diagnostics: Vec<String>,
}

impl EmbodimentGraph {
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

    pub fn requires(&self, cap: Capability) -> bool {
        self.capabilities.has(cap)
    }

    pub fn serial_model(&self) -> realityos_physics::SerialModel {
        realityos_physics::SerialModel {
            joints: self
                .joints
                .iter()
                .map(|j| {
                    let link = self
                        .links
                        .iter()
                        .find(|l| l.parent_joint.as_deref() == Some(j.name.as_str()));
                    realityos_physics::SerialJoint {
                        kind: if j.joint_type == "prismatic" {
                            realityos_physics::JointKind::Prismatic
                        } else {
                            realityos_physics::JointKind::Revolute
                        },
                        axis: j.axis,
                        origin: j.origin_xyz,
                        mass: link.and_then(|l| l.mass_kg).unwrap_or(0.0),
                        com: link.and_then(|l| l.com_m).unwrap_or([0.0, 0.0, 0.0]),
                        inertia_diag: link
                            .and_then(|l| l.inertia_diag)
                            .unwrap_or([0.0, 0.0, 0.0]),
                    }
                })
                .collect(),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Environment {
    pub id: String,
    pub g_m_s2: f64,
    pub mu: Option<f64>,
    pub air_density_kg_m3: f64,
    pub note: String,
    pub mu_assumed: bool,
}

impl Environment {
    fn with_assumed_mu(id: &str, g: f64, mu: f64, rho: f64, note: &str) -> Self {
        Self {
            id: id.into(),
            g_m_s2: g,
            mu: Some(mu),
            air_density_kg_m3: rho,
            note: format!("{note} [assumed mu={mu}, not an authority default]"),
            mu_assumed: true,
        }
    }

    pub fn earth() -> Self {
        Self::with_assumed_mu(
            "earth_indoor",
            G0,
            0.6,
            1.225,
            "standard g, dry indoor friction screen",
        )
    }

    pub fn moon() -> Self {
        Self::with_assumed_mu(
            "moon",
            1.62,
            0.4,
            0.0,
            "lunar g; vacuum density; SIM screen",
        )
    }

    pub fn ice() -> Self {
        Self::with_assumed_mu("ice", G0, 0.05, 1.225, "low Coulomb μ screen")
    }

    pub fn high_g() -> Self {
        Self::with_assumed_mu(
            "high_g",
            20.0,
            0.6,
            1.225,
            "elevated g screen (not a planet claim)",
        )
    }

    pub fn catalog() -> Vec<Self> {
        vec![Self::earth(), Self::moon(), Self::ice(), Self::high_g()]
    }
}

fn graph_from_model_json(raw: &str) -> Result<EmbodimentGraph, EmbodimentError> {
    #[derive(Deserialize)]
    struct Wire {
        id: String,
        kind: String,
        source: String,
        joints: Vec<JointSpec>,
    }
    let w: Wire = serde_json::from_str(raw).map_err(|e| EmbodimentError::Parse(e.to_string()))?;
    if w.joints.is_empty() {
        return Err(EmbodimentError::Parse("no joints".into()));
    }
    for j in &w.joints {
        if !j.q_min.is_finite() || !j.q_max.is_finite() || j.q_min > j.q_max {
            return Err(EmbodimentError::Parse(format!("invalid limits {}", j.name)));
        }
    }
    let source_hash = hex::encode(Sha256::digest(raw.as_bytes()));
    let mut links = Vec::new();
    let mut frames = vec![FrameSpec {
        name: "world".into(),
        parent: String::new(),
    }];
    let mut parent = "world".to_string();
    for j in &w.joints {
        let link = format!("link_{}", j.name);
        links.push(LinkSpec {
            name: link.clone(),
            parent_joint: Some(j.name.clone()),
            mass_kg: None,
            com_m: None,
            inertia_diag: None,
        });
        frames.push(FrameSpec {
            name: j.name.clone(),
            parent: parent.clone(),
        });
        parent = link;
    }
    let mut diagnostics = Vec::new();
    if links.iter().any(|l| l.mass_kg.is_none()) {
        diagnostics.push("inertia_unspecified".into());
    }
    Ok(EmbodimentGraph {
        capabilities: CapabilityManifest::from_kind(&w.kind),
        id: w.id,
        kind: w.kind,
        source: w.source,
        source_hash,
        joints: w.joints,
        links,
        frames,
        diagnostics,
    })
}

pub fn parse_robot(json: &str) -> Result<EmbodimentGraph, EmbodimentError> {
    graph_from_model_json(json)
}

pub fn load_embedded(id: &str) -> Result<EmbodimentGraph, EmbodimentError> {
    let raw = match id {
        "uniaxial" => include_str!("../../../robots/uniaxial.json"),
        "arm6" => include_str!("../../../robots/arm6.json"),
        "wheeled" => include_str!("../../../robots/wheeled.json"),
        "unitree_h1" => include_str!("../../../robots/unitree_h1.json"),
        "quadruped" => include_str!("../../../robots/quadruped.json"),
        "aerial" => include_str!("../../../robots/aerial.json"),
        other => return Err(EmbodimentError::UnknownRobot(other.into())),
    };
    graph_from_model_json(raw)
}

pub fn catalog() -> Vec<EmbodimentGraph> {
    [
        "uniaxial",
        "arm6",
        "wheeled",
        "unitree_h1",
        "quadruped",
        "aerial",
    ]
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
        assert!(r.requires(Capability::FloatingBase));
        assert!(!r.id.is_empty());
    }

    #[test]
    fn capability_queries_do_not_branch_on_fixture_id() {
        let h1 = load_embedded("unitree_h1").unwrap();
        let arm = load_embedded("arm6").unwrap();
        let quad = load_embedded("quadruped").unwrap();
        let air = load_embedded("aerial").unwrap();
        assert!(h1.requires(Capability::FloatingBase));
        assert!(arm.requires(Capability::SerialArm));
        assert!(!arm.requires(Capability::FloatingBase));
        assert!(quad.requires(Capability::FloatingBase));
        assert!(air.requires(Capability::FloatingBase));
        assert_eq!(quad.kind, "quadruped");
        assert_eq!(air.kind, "aerial");
    }

    #[test]
    fn environments_change_g_and_mark_assumed_mu() {
        assert!(Environment::moon().g_m_s2 < Environment::earth().g_m_s2);
        assert!(Environment::earth().mu_assumed);
    }
}
