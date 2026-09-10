use crate::provenance::Provenanced;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum JointKind {
    Hinge,
    Slide,
    Ball,
    Free,
    Fixed,
    Other,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BaseKind {
    Fixed,
    Floating,
    Mobile,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FrameKind {
    Ee,
    Tool,
    Camera,
    Sensor,
    Task,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Body {
    pub name: String,
    pub parent: Option<String>,
    pub mass_kg: Provenanced<f64>,
    pub com: Provenanced<[f64; 3]>,
    pub inertia: Provenanced<[f64; 6]>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Joint {
    pub name: String,
    pub kind: JointKind,
    pub axis: Provenanced<[f64; 3]>,
    pub qpos_dim: usize,
    pub dof_dim: usize,
    pub parent_body: String,
    pub child_body: String,
    pub q_min: Provenanced<f64>,
    pub q_max: Provenanced<f64>,
    pub dq_max: Provenanced<f64>,
    pub effort_max: Provenanced<f64>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Actuator {
    pub name: String,
    pub target_joint: String,
    pub control_mode: String,
    pub ctrlrange: Provenanced<[f64; 2]>,
    pub forcerange: Provenanced<[f64; 2]>,
    pub gear: Provenanced<f64>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ModelFrame {
    pub name: String,
    pub kind: FrameKind,
    pub parent_body: String,
    pub translation: Provenanced<[f64; 3]>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct EndEffector {
    pub name: String,
    pub frame: String,
    pub joint_chain: Vec<String>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Gripper {
    pub name: String,
    pub actuator: String,
    pub opening_range: Provenanced<[f64; 2]>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Transmission {
    pub name: String,
    pub source: String,
    pub target_joint: String,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ModelDiagnostic {
    pub code: String,
    pub detail: String,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct EmbodimentModel {
    pub robot_id: String,
    pub source_bundle_hash: String,
    pub model_hash: String,
    pub calibration_epoch: String,
    pub model_version: String,
    pub base: BaseKind,
    pub bodies: Vec<Body>,
    pub joints: Vec<Joint>,
    pub actuators: Vec<Actuator>,
    pub frames: Vec<ModelFrame>,
    pub end_effectors: Vec<EndEffector>,
    pub grippers: Vec<Gripper>,
    pub transmissions: Vec<Transmission>,
    pub diagnostics: Vec<ModelDiagnostic>,
    pub metal: bool,
}

impl EmbodimentModel {
    pub fn new(
        robot_id: impl Into<String>,
        source_bundle_hash: impl Into<String>,
        model_hash: impl Into<String>,
        calibration_epoch: impl Into<String>,
        model_version: impl Into<String>,
    ) -> Self {
        Self {
            robot_id: robot_id.into(),
            source_bundle_hash: source_bundle_hash.into(),
            model_hash: model_hash.into(),
            calibration_epoch: calibration_epoch.into(),
            model_version: model_version.into(),
            base: BaseKind::Fixed,
            bodies: Vec::new(),
            joints: Vec::new(),
            actuators: Vec::new(),
            frames: Vec::new(),
            end_effectors: Vec::new(),
            grippers: Vec::new(),
            transmissions: Vec::new(),
            diagnostics: Vec::new(),
            metal: false,
        }
    }

    pub fn ee_joint_chain(&self, ee: &str) -> Option<Vec<String>> {
        self.end_effectors
            .iter()
            .find(|e| e.name == ee)
            .and_then(|e| {
                if e.joint_chain.is_empty() {
                    None
                } else {
                    Some(e.joint_chain.clone())
                }
            })
    }

    pub fn position_actuators(&self) -> impl Iterator<Item = &Actuator> {
        self.actuators
            .iter()
            .filter(|a| a.control_mode == "position")
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::provenance::Provenanced;

    #[test]
    fn tree_accepts_ball_and_fixed_without_inventing_mass() {
        let mut m = EmbodimentModel::new("synth", "src", "hash", "epoch0", "1");
        m.bodies.push(Body {
            name: "link".into(),
            parent: Some("base".into()),
            mass_kg: Provenanced::unknown("bundle", 0.0),
            com: Provenanced::unknown("bundle", 0.0),
            inertia: Provenanced::unknown("bundle", 0.0),
        });
        m.joints.push(Joint {
            name: "j_ball".into(),
            kind: JointKind::Ball,
            axis: Provenanced::unknown("bundle", 0.0),
            qpos_dim: 4,
            dof_dim: 3,
            parent_body: "base".into(),
            child_body: "link".into(),
            q_min: Provenanced::unknown("bundle", 0.0),
            q_max: Provenanced::unknown("bundle", 0.0),
            dq_max: Provenanced::unknown("bundle", 0.0),
            effort_max: Provenanced::unknown("bundle", 0.0),
        });
        m.diagnostics.push(ModelDiagnostic {
            code: "unsupported_joint".into(),
            detail: "ball not used by REACH adapter".into(),
        });
        assert!(m.bodies[0].mass_kg.value.is_none());
        assert!(!m.metal);
        assert_eq!(m.joints[0].kind, JointKind::Ball);
    }

    #[test]
    fn new_model_is_never_metal() {
        let m = EmbodimentModel::new("x", "s", "h", "e", "1");
        assert!(!m.metal);
    }
}
