use crate::command::{ActuatorCommand, ActuatorCommandSet, HoldSemantics};
use crate::embodiment::EmbodimentModel;
use crate::provenance::Provenanced;
use crate::skill::SkillRefuse;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum ResourceTopology {
    DirectJointGripper,
    CoupledJointGripper,
    TendonDrivenGripper,
    UnsupportedResourceTopology,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum ResourceKind {
    Gripper,
    ContactEndEffector,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum ClosingDirection {
    TowardMin,
    TowardMax,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum QualificationStatus {
    Unverified,
    Qualified,
    Failed,
    NotApplicable,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TendonWrap {
    pub tendon: String,
    pub joints: Vec<(String, f64)>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct JointEquality {
    pub joint_a: String,
    pub joint_b: String,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CouplingModel {
    pub tendon: Option<TendonWrap>,
    pub equalities: Vec<JointEquality>,
    pub declared_only: bool,
}

impl CouplingModel {
    pub fn none() -> Self {
        Self {
            tendon: None,
            equalities: Vec::new(),
            declared_only: true,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ControlledResource {
    pub id: String,
    pub kind: ResourceKind,
    pub topology: ResourceTopology,
    pub actuator_inputs: Vec<String>,
    pub affected_joints: Vec<String>,
    pub finger_bodies: Vec<String>,
    pub coupling: CouplingModel,
    pub command_coordinate: String,
    pub opening_range: Provenanced<[f64; 2]>,
    pub command_range: Provenanced<[f64; 2]>,
    pub closing_direction: ClosingDirection,
    pub force_bound: Provenanced<[f64; 2]>,
    pub qualification: QualificationStatus,
    pub unsupported_detail: Option<String>,
}

impl ControlledResource {
    pub fn command_from_opening(&self, opening_01: f64) -> Result<f64, SkillRefuse> {
        if !opening_01.is_finite() {
            return Err(SkillRefuse::Unsupported);
        }
        let [lo, hi] = self
            .command_range
            .value
            .ok_or(SkillRefuse::ModelFeatureUnsupported)?;
        let t = opening_01.clamp(0.0, 1.0);
        let cmd = match self.closing_direction {
            ClosingDirection::TowardMin => lo + t * (hi - lo),
            ClosingDirection::TowardMax => hi - t * (hi - lo),
        };
        Ok(cmd)
    }

    pub fn opening_from_command(&self, command: f64) -> Option<f64> {
        let [lo, hi] = self.command_range.value?;
        let span = hi - lo;
        if span.abs() < 1e-12 {
            return None;
        }
        let t = match self.closing_direction {
            ClosingDirection::TowardMin => (command - lo) / span,
            ClosingDirection::TowardMax => (hi - command) / span,
        };
        Some(t.clamp(0.0, 1.0))
    }

    pub fn is_supported(&self) -> bool {
        !matches!(self.topology, ResourceTopology::UnsupportedResourceTopology)
            && self.unsupported_detail.is_none()
    }
}

pub fn lower_resource_opening(
    model: &EmbodimentModel,
    resource: &ControlledResource,
    opening_01: f64,
    skill_id: &str,
    hold_outside: HoldSemantics,
) -> Result<ActuatorCommandSet, SkillRefuse> {
    if !resource.is_supported() {
        return Err(SkillRefuse::ResourceUnsupported);
    }
    if resource.qualification == QualificationStatus::Unverified {
        return Err(SkillRefuse::ResourceUnsupported);
    }
    let cmd = resource.command_from_opening(opening_01)?;
    let mut commands = Vec::new();
    for name in &resource.actuator_inputs {
        if !model.actuators.iter().any(|a| a.name == *name) {
            return Err(SkillRefuse::MissingActuator);
        }
        commands.push(ActuatorCommand {
            actuator_name: name.clone(),
            value: cmd,
            control_mode: "position".into(),
            skill_id: skill_id.into(),
        });
    }
    if commands.is_empty() {
        return Err(SkillRefuse::MissingActuator);
    }
    let _ = model;
    Ok(ActuatorCommandSet {
        commands,
        hold_outside,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::provenance::Provenanced;

    fn direct() -> ControlledResource {
        ControlledResource {
            id: "g0".into(),
            kind: ResourceKind::Gripper,
            topology: ResourceTopology::DirectJointGripper,
            actuator_inputs: vec!["grip_l".into(), "grip_r".into()],
            affected_joints: vec!["finger_l".into(), "finger_r".into()],
            finger_bodies: vec!["finger_l".into(), "finger_r".into()],
            coupling: CouplingModel::none(),
            command_coordinate: "opening".into(),
            opening_range: Provenanced::declared([0.0, 0.03], "test", 0.0),
            command_range: Provenanced::declared([0.0, 0.03], "test", 0.0),
            closing_direction: ClosingDirection::TowardMin,
            force_bound: Provenanced::unknown("test", 0.0),
            qualification: QualificationStatus::Qualified,
            unsupported_detail: None,
        }
    }

    fn tendon() -> ControlledResource {
        ControlledResource {
            id: "g1".into(),
            kind: ResourceKind::Gripper,
            topology: ResourceTopology::TendonDrivenGripper,
            actuator_inputs: vec!["hand_act".into()],
            affected_joints: vec!["finger_a".into(), "finger_b".into()],
            finger_bodies: vec!["finger_a".into(), "finger_b".into()],
            coupling: CouplingModel {
                tendon: Some(TendonWrap {
                    tendon: "split".into(),
                    joints: vec![("finger_a".into(), 0.5), ("finger_b".into(), 0.5)],
                }),
                equalities: vec![JointEquality {
                    joint_a: "finger_a".into(),
                    joint_b: "finger_b".into(),
                }],
                declared_only: true,
            },
            command_coordinate: "tendon_ctrl".into(),
            opening_range: Provenanced::declared([0.0, 0.04], "test", 0.0),
            command_range: Provenanced::declared([0.0, 255.0], "test", 0.0),
            closing_direction: ClosingDirection::TowardMin,
            force_bound: Provenanced::unknown("test", 0.0),
            qualification: QualificationStatus::Qualified,
            unsupported_detail: None,
        }
    }

    #[test]
    fn opening_maps_through_command_range_not_joint_flatten() {
        let t = tendon();
        assert_eq!(t.command_from_opening(0.0).unwrap(), 0.0);
        assert!((t.command_from_opening(1.0).unwrap() - 255.0).abs() < 1e-9);
        assert_eq!(t.coupling.tendon.as_ref().unwrap().tendon, "split");
        assert_eq!(t.actuator_inputs, vec!["hand_act".to_string()]);
    }

    #[test]
    fn unsupported_topology_refuses() {
        let mut r = direct();
        r.topology = ResourceTopology::UnsupportedResourceTopology;
        r.unsupported_detail = Some("cyclic_tendon_network".into());
        assert!(!r.is_supported());
    }

    #[test]
    fn unknown_force_bound_stays_unknown() {
        assert!(direct().force_bound.value.is_none());
    }
}
