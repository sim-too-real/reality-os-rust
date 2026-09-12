use crate::embodiment::{Actuator, BaseKind, EmbodimentModel, Joint, JointKind};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CapName {
    JointPositionControl,
    JointVelocityControl,
    JointEffortControl,
    CartesianPositionControl,
    FixedBaseManipulation,
    MobileBase,
    FloatingBase,
    Grasping,
    ParallelGripper,
    GripperOpenClose,
    ContactManipulation,
    Pushing,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CapStatus {
    Proven,
    Supported,
    PartiallySupported,
    Unverified,
    NotApplicable,
    Unsupported,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CapNode {
    pub name: CapName,
    pub status: CapStatus,
    #[serde(default)]
    pub resource: Option<String>,
    pub evidence: Vec<String>,
    pub confidence: Option<f64>,
    pub dependencies: Vec<CapName>,
    pub unsupported_reason: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CapabilityGraph {
    nodes: Vec<CapNode>,
}

impl CapabilityGraph {
    pub fn nodes(&self) -> &[CapNode] {
        &self.nodes
    }

    pub fn get_opt(&self, name: CapName) -> Option<&CapNode> {
        self.nodes.iter().find(|n| n.name == name)
    }

    pub fn get(&self, name: CapName) -> &CapNode {
        self.get_opt(name).expect("capability node missing")
    }
}

fn joint_for_actuator<'a>(model: &'a EmbodimentModel, actuator: &Actuator) -> Option<&'a Joint> {
    model
        .joints
        .iter()
        .find(|j| j.name == actuator.target_joint)
}

fn is_1dof_hinge_or_slide(joint: &Joint) -> bool {
    joint.dof_dim == 1 && matches!(joint.kind, JointKind::Hinge | JointKind::Slide)
}

fn actuator_on_1dof_hinge_or_slide(model: &EmbodimentModel, actuator: &Actuator) -> bool {
    joint_for_actuator(model, actuator)
        .map(is_1dof_hinge_or_slide)
        .unwrap_or(false)
}

fn node(
    name: CapName,
    status: CapStatus,
    evidence: Vec<String>,
    dependencies: Vec<CapName>,
    unsupported_reason: Option<String>,
) -> CapNode {
    scoped_node(
        name,
        None,
        status,
        evidence,
        dependencies,
        unsupported_reason,
    )
}

fn scoped_node(
    name: CapName,
    resource: Option<String>,
    status: CapStatus,
    evidence: Vec<String>,
    dependencies: Vec<CapName>,
    unsupported_reason: Option<String>,
) -> CapNode {
    CapNode {
        name,
        status,
        resource,
        evidence,
        confidence: None,
        dependencies,
        unsupported_reason,
    }
}

fn chain_position_coverage(model: &EmbodimentModel, chain: &[String]) -> (usize, usize) {
    let mut required = 0;
    let mut covered = 0;
    for name in chain {
        let Some(joint) = model.joints.iter().find(|j| j.name == *name) else {
            continue;
        };
        if !is_1dof_hinge_or_slide(joint) {
            continue;
        }
        required += 1;
        if model.actuators.iter().any(|a| {
            a.target_joint == *name
                && a.control_mode == "position"
                && actuator_on_1dof_hinge_or_slide(model, a)
        }) {
            covered += 1;
        }
    }
    (covered, required)
}

pub fn derive_capabilities(model: &EmbodimentModel, qualify_ok: Option<bool>) -> CapabilityGraph {
    let mut chain_required = 0;
    let mut chain_covered = 0;
    let mut scoped = Vec::new();
    for ee in &model.end_effectors {
        let (covered, required) = chain_position_coverage(model, &ee.joint_chain);
        chain_required += required;
        chain_covered += covered;
        let status = if required == 0 {
            CapStatus::Unsupported
        } else if covered == required {
            CapStatus::Supported
        } else if covered > 0 {
            CapStatus::PartiallySupported
        } else {
            CapStatus::Unsupported
        };
        scoped.push(scoped_node(
            CapName::JointPositionControl,
            Some(format!("chain:{}", ee.name)),
            status,
            vec![format!("covered:{covered}/{required}")],
            vec![],
            None,
        ));
    }

    let joint_position_status = if chain_required == 0 {
        let has_any = model
            .actuators
            .iter()
            .any(|a| a.control_mode == "position" && actuator_on_1dof_hinge_or_slide(model, a));
        if has_any {
            CapStatus::PartiallySupported
        } else {
            CapStatus::Unsupported
        }
    } else if chain_covered == chain_required {
        if qualify_ok == Some(true) {
            CapStatus::Proven
        } else {
            CapStatus::Supported
        }
    } else if chain_covered > 0 {
        CapStatus::PartiallySupported
    } else {
        CapStatus::Unsupported
    };

    let joint_position_evidence =
        if qualify_ok == Some(true) && chain_covered == chain_required && chain_required > 0 {
            vec!["sim_qualify".into()]
        } else {
            vec![]
        };

    let has_velocity = model.actuators.iter().any(|a| {
        (a.control_mode == "velocity" || a.control_mode == "motor")
            && actuator_on_1dof_hinge_or_slide(model, a)
    });

    let effort_actuators: Vec<&Actuator> = model
        .actuators
        .iter()
        .filter(|a| {
            (a.control_mode == "motor" || a.control_mode == "effort")
                && actuator_on_1dof_hinge_or_slide(model, a)
        })
        .collect();

    let (joint_effort_status, joint_effort_reason) = if effort_actuators.is_empty() {
        (CapStatus::Unsupported, None)
    } else if effort_actuators.iter().any(|a| {
        joint_for_actuator(model, a)
            .and_then(|j| j.effort_max.value)
            .is_some()
    }) {
        (CapStatus::Supported, None)
    } else {
        (CapStatus::Unverified, Some("effort_bound_unknown".into()))
    };

    let has_ee = !model.end_effectors.is_empty();
    let cartesian_status = if !has_ee {
        CapStatus::Unsupported
    } else if matches!(
        joint_position_status,
        CapStatus::Supported | CapStatus::Proven | CapStatus::PartiallySupported
    ) {
        CapStatus::PartiallySupported
    } else {
        CapStatus::Unsupported
    };

    let fixed_base_status = match model.base {
        BaseKind::Fixed if has_ee => CapStatus::Supported,
        BaseKind::Fixed => CapStatus::Unsupported,
        BaseKind::Floating | BaseKind::Mobile => CapStatus::NotApplicable,
    };

    let has_free_joint = model.joints.iter().any(|j| j.kind == JointKind::Free);
    let floating_status = if model.base == BaseKind::Floating || has_free_joint {
        CapStatus::Supported
    } else {
        CapStatus::NotApplicable
    };

    let mobile_status = if model.base == BaseKind::Mobile {
        CapStatus::Supported
    } else {
        CapStatus::NotApplicable
    };

    let has_named_gripper = model.grippers.iter().any(|g| !g.actuator.is_empty());
    let coupled_gripper = model.diagnostics.iter().any(|d| {
        matches!(
            d.code.as_str(),
            "TENDON_PRESENT"
                | "ACTUATOR_TARGETS_TENDON"
                | "JOINT_EQUALITY_CONSTRAINT"
                | "COUPLED_JOINTS"
        )
    });
    let (gripper_status, gripper_reason) = if !has_named_gripper {
        (CapStatus::Unsupported, None)
    } else if coupled_gripper {
        (
            CapStatus::PartiallySupported,
            Some("COUPLED_GRIPPER_NOT_QUALIFIED".into()),
        )
    } else {
        (CapStatus::PartiallySupported, None)
    };

    let mut nodes = vec![
        node(
            CapName::JointPositionControl,
            joint_position_status,
            joint_position_evidence,
            vec![],
            None,
        ),
        node(
            CapName::JointVelocityControl,
            if has_velocity {
                CapStatus::Supported
            } else {
                CapStatus::Unsupported
            },
            vec![],
            vec![],
            None,
        ),
        node(
            CapName::JointEffortControl,
            joint_effort_status,
            vec![],
            vec![],
            joint_effort_reason,
        ),
        node(
            CapName::CartesianPositionControl,
            cartesian_status,
            vec![],
            vec![CapName::JointPositionControl],
            None,
        ),
        node(
            CapName::FixedBaseManipulation,
            fixed_base_status,
            vec![],
            vec![],
            None,
        ),
        node(CapName::MobileBase, mobile_status, vec![], vec![], None),
        node(CapName::FloatingBase, floating_status, vec![], vec![], None),
        node(
            CapName::Grasping,
            gripper_status,
            vec![],
            vec![],
            gripper_reason.clone(),
        ),
        node(
            CapName::ParallelGripper,
            gripper_status,
            vec![],
            vec![],
            gripper_reason.clone(),
        ),
        node(
            CapName::GripperOpenClose,
            gripper_status,
            vec![],
            vec![],
            gripper_reason,
        ),
        node(
            CapName::ContactManipulation,
            if has_ee {
                CapStatus::PartiallySupported
            } else {
                CapStatus::Unsupported
            },
            vec![],
            vec![],
            None,
        ),
        node(
            CapName::Pushing,
            if has_ee {
                CapStatus::PartiallySupported
            } else {
                CapStatus::NotApplicable
            },
            vec![],
            vec![],
            None,
        ),
    ];
    nodes.extend(scoped);

    CapabilityGraph { nodes }
}

pub fn apply_resource_qualification(
    mut graph: CapabilityGraph,
    gripper_open_close_proven: bool,
    grasping_supported: bool,
    pushing_supported: bool,
) -> CapabilityGraph {
    for n in &mut graph.nodes {
        match n.name {
            CapName::GripperOpenClose if gripper_open_close_proven => {
                n.status = CapStatus::Proven;
                n.evidence.push("resource_qualification".into());
                n.unsupported_reason = None;
            }
            CapName::ParallelGripper if gripper_open_close_proven => {
                if n.status != CapStatus::Unsupported && n.status != CapStatus::NotApplicable {
                    n.status = CapStatus::Supported;
                    n.evidence.push("resource_qualification".into());
                }
            }
            CapName::Grasping if grasping_supported => {
                if n.status != CapStatus::Unsupported && n.status != CapStatus::NotApplicable {
                    n.status = CapStatus::Supported;
                    n.evidence.push("resource_qualification".into());
                }
            }
            CapName::Pushing | CapName::ContactManipulation if pushing_supported => {
                if n.status != CapStatus::Unsupported {
                    n.status = CapStatus::Supported;
                    n.evidence.push("contact_ee_present".into());
                }
            }
            _ => {}
        }
        if n.name == CapName::Grasping && n.status == CapStatus::Proven {
            n.status = CapStatus::Supported;
        }
    }
    graph
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::embodiment::{EndEffector, Gripper};
    use crate::provenance::Provenanced;

    fn synth_joint(name: &str) -> Joint {
        Joint {
            name: name.into(),
            kind: JointKind::Hinge,
            axis: Provenanced::unknown("test", 0.0),
            qpos_dim: 1,
            dof_dim: 1,
            parent_body: "base".into(),
            child_body: "link".into(),
            q_min: Provenanced::unknown("test", 0.0),
            q_max: Provenanced::unknown("test", 0.0),
            dq_max: Provenanced::unknown("test", 0.0),
            effort_max: Provenanced::unknown("test", 0.0),
            origin_in_child: Provenanced::unknown("test", 0.0),
            parent_to_joint: crate::embodiment::unknown_se3("test"),
            joint_to_child: crate::embodiment::unknown_se3("test"),
            qpos_adr: None,
            dof_adr: None,
        }
    }

    fn synth_actuator(name: &str, joint: &str, mode: &str) -> Actuator {
        Actuator {
            name: name.into(),
            target_joint: joint.into(),
            control_mode: mode.into(),
            transmission_kind: "joint".into(),
            ctrlrange: Provenanced::unknown("test", 0.0),
            forcerange: Provenanced::unknown("test", 0.0),
            gear: Provenanced::unknown("test", 0.0),
        }
    }

    fn synth_fixed_position_arm() -> EmbodimentModel {
        let mut m = EmbodimentModel::new("synth", "src", "hash", "epoch0", "1");
        m.base = BaseKind::Fixed;
        m.joints.push(synth_joint("j0"));
        m.actuators.push(synth_actuator("a0", "j0", "position"));
        m.end_effectors.push(EndEffector {
            name: "ee".into(),
            frame: "ee_frame".into(),
            joint_chain: vec!["j0".into()],
        });
        m
    }

    fn synth_motor_no_effort() -> EmbodimentModel {
        let mut m = EmbodimentModel::new("synth", "src", "hash", "epoch0", "1");
        m.joints.push(synth_joint("j0"));
        m.actuators.push(synth_actuator("a0", "j0", "motor"));
        m
    }

    #[test]
    fn fixed_position_arm_gets_cartesian_partial_not_by_robot_name() {
        let m = synth_fixed_position_arm();
        let g = derive_capabilities(&m, None);
        assert_eq!(
            g.get(CapName::JointPositionControl).status,
            CapStatus::Supported
        );
        assert_eq!(
            g.get(CapName::CartesianPositionControl).status,
            CapStatus::PartiallySupported
        );
        assert_eq!(
            g.get(CapName::FloatingBase).status,
            CapStatus::NotApplicable
        );
        assert_eq!(g.get(CapName::Grasping).status, CapStatus::Unsupported);
    }

    #[test]
    fn motor_without_effort_bound_stays_unverified() {
        let m = synth_motor_no_effort();
        let g = derive_capabilities(&m, None);
        let n = g.get(CapName::JointEffortControl);
        assert_eq!(n.status, CapStatus::Unverified);
        assert_eq!(
            n.unsupported_reason.as_deref(),
            Some("effort_bound_unknown")
        );
    }

    #[test]
    fn derive_does_not_read_robot_id() {
        let mut a = synth_fixed_position_arm();
        let mut b = a.clone();
        a.robot_id = "alice".into();
        b.robot_id = "bob".into();
        assert_eq!(derive_capabilities(&a, None), derive_capabilities(&b, None));
    }

    #[test]
    fn qualify_ok_promotes_joint_position_not_grasping() {
        let m = synth_fixed_position_arm();
        let g = derive_capabilities(&m, Some(true));
        assert_eq!(
            g.get(CapName::JointPositionControl).status,
            CapStatus::Proven
        );
        assert_eq!(
            g.get(CapName::JointPositionControl).evidence,
            vec!["sim_qualify".to_string()]
        );
        assert_eq!(g.get(CapName::Grasping).status, CapStatus::Unsupported);
    }

    #[test]
    fn qualify_ok_false_keeps_supported() {
        let m = synth_fixed_position_arm();
        let g = derive_capabilities(&m, Some(false));
        assert_eq!(
            g.get(CapName::JointPositionControl).status,
            CapStatus::Supported
        );
    }

    #[test]
    fn gripper_yaml_is_not_grasp_proven_or_supported() {
        let mut m = synth_fixed_position_arm();
        m.grippers.push(Gripper {
            name: "g0".into(),
            actuator: "grip_a".into(),
            opening_range: Provenanced::declared([0.0, 0.08], "test", 0.0),
        });
        let g = derive_capabilities(&m, Some(true));
        assert_eq!(
            g.get(CapName::Grasping).status,
            CapStatus::PartiallySupported
        );
        assert_eq!(
            g.get(CapName::ParallelGripper).status,
            CapStatus::PartiallySupported
        );
        assert_ne!(g.get(CapName::Grasping).status, CapStatus::Proven);
        assert_ne!(g.get(CapName::Grasping).status, CapStatus::Supported);
        let promoted = apply_resource_qualification(g, true, true, true);
        assert_eq!(
            promoted.get(CapName::GripperOpenClose).status,
            CapStatus::Proven
        );
        assert_eq!(promoted.get(CapName::Grasping).status, CapStatus::Supported);
        assert_ne!(promoted.get(CapName::Grasping).status, CapStatus::Proven);
    }

    #[test]
    fn coupled_gripper_stays_unqualified() {
        let mut m = synth_fixed_position_arm();
        m.grippers.push(Gripper {
            name: "g0".into(),
            actuator: "grip_a".into(),
            opening_range: Provenanced::declared([0.0, 0.08], "test", 0.0),
        });
        m.diagnostics.push(crate::embodiment::ModelDiagnostic {
            code: "ACTUATOR_TARGETS_TENDON".into(),
            detail: "grip_a".into(),
        });
        let g = derive_capabilities(&m, Some(true));
        assert_eq!(
            g.get(CapName::Grasping).status,
            CapStatus::PartiallySupported
        );
        assert_eq!(
            g.get(CapName::Grasping).unsupported_reason.as_deref(),
            Some("COUPLED_GRIPPER_NOT_QUALIFIED")
        );
        assert_ne!(g.get(CapName::Grasping).status, CapStatus::Supported);
        assert_ne!(g.get(CapName::Grasping).status, CapStatus::Proven);
    }

    #[test]
    fn one_position_actuator_does_not_support_a_two_joint_chain() {
        let mut m = synth_fixed_position_arm();
        m.joints.push(synth_joint("j1"));
        m.end_effectors[0].joint_chain = vec!["j0".into(), "j1".into()];
        let g = derive_capabilities(&m, None);
        assert_eq!(
            g.get(CapName::JointPositionControl).status,
            CapStatus::PartiallySupported
        );
    }
}
