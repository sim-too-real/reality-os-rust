use crate::provenance::Provenanced;
use crate::transform::Se3;
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
    pub local_pose: Provenanced<Se3>,
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
    pub origin_in_child: Provenanced<[f64; 3]>,
    pub parent_to_joint: Provenanced<Se3>,
    pub joint_to_child: Provenanced<Se3>,
    pub qpos_adr: Option<i32>,
    pub dof_adr: Option<i32>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Actuator {
    pub name: String,
    pub target_joint: String,
    pub control_mode: String,
    #[serde(default)]
    pub transmission_kind: String,
    pub ctrlrange: Provenanced<[f64; 2]>,
    pub forcerange: Provenanced<[f64; 2]>,
    pub gear: Provenanced<f64>,
}

impl Actuator {
    pub fn targets_joint(&self) -> bool {
        self.transmission_kind.is_empty() || self.transmission_kind == "joint"
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ModelFrame {
    pub name: String,
    pub kind: FrameKind,
    pub parent_body: String,
    pub translation: Provenanced<[f64; 3]>,
    pub rotation: Provenanced<[f64; 4]>,
}

impl ModelFrame {
    pub fn pose(&self) -> Option<Se3> {
        let xyz = self.translation.value?;
        let quat = self.rotation.value?;
        Se3::try_new(xyz, quat).ok()
    }
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
    #[serde(default)]
    pub resources: Vec<crate::resource::ControlledResource>,
    #[serde(default)]
    pub collision_geoms: Vec<crate::geometry::RigidGeometry>,
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
            resources: Vec::new(),
            collision_geoms: Vec::new(),
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

    pub fn actuator_for_joint(&self, joint: &str) -> Option<&Actuator> {
        self.actuators.iter().find(|a| a.target_joint == joint)
    }

    pub fn validate_transforms(&self) -> Vec<ModelDiagnostic> {
        let mut out = Vec::new();
        for body in &self.bodies {
            if let Some(pose) = body.local_pose.value {
                if Se3::try_new(pose.xyz, pose.quat_wxyz).is_err() {
                    out.push(ModelDiagnostic {
                        code: "invalid_body_transform".into(),
                        detail: body.name.clone(),
                    });
                }
            }
        }
        for joint in &self.joints {
            if let Some(axis) = joint.axis.value {
                if axis.iter().all(|v| *v == 0.0) || !axis.iter().all(|v| v.is_finite()) {
                    out.push(ModelDiagnostic {
                        code: "invalid_joint_axis".into(),
                        detail: joint.name.clone(),
                    });
                }
            }
            if let Some(origin) = joint.origin_in_child.value {
                if !origin.iter().all(|v| v.is_finite()) {
                    out.push(ModelDiagnostic {
                        code: "invalid_joint_origin".into(),
                        detail: joint.name.clone(),
                    });
                }
            }
        }
        for frame in &self.frames {
            if let (Some(xyz), Some(quat)) = (frame.translation.value, frame.rotation.value) {
                if Se3::try_new(xyz, quat).is_err() {
                    out.push(ModelDiagnostic {
                        code: "invalid_frame_transform".into(),
                        detail: frame.name.clone(),
                    });
                }
            }
        }
        let mut seen = std::collections::BTreeSet::new();
        for body in &self.bodies {
            let mut cur = body.parent.clone();
            let mut guard = 0;
            while let Some(name) = cur {
                if name == body.name || guard > self.bodies.len() + 2 {
                    out.push(ModelDiagnostic {
                        code: "invalid_transform_topology".into(),
                        detail: body.name.clone(),
                    });
                    break;
                }
                if !seen.insert(format!("{}:{}", body.name, name)) {
                    break;
                }
                cur = self
                    .bodies
                    .iter()
                    .find(|b| b.name == name)
                    .and_then(|b| b.parent.clone());
                guard += 1;
            }
        }
        out
    }
}

pub fn unknown_se3(source: &str) -> Provenanced<Se3> {
    Provenanced::unknown(source, 0.0)
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
            local_pose: unknown_se3("bundle"),
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
            origin_in_child: Provenanced::unknown("bundle", 0.0),
            parent_to_joint: unknown_se3("bundle"),
            joint_to_child: unknown_se3("bundle"),
            qpos_adr: None,
            dof_adr: None,
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
