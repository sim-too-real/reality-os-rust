use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum HoldSemantics {
    KeepCurrent,
    ExplicitSafe,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct JointTarget {
    pub joint_name: String,
    pub actuator_name: Option<String>,
    pub value: f64,
    pub control_mode: String,
    pub skill_id: String,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct JointTargetSet {
    pub targets: Vec<JointTarget>,
    pub hold_outside: HoldSemantics,
}

impl JointTargetSet {
    pub fn named_value(&self, joint: &str) -> Option<f64> {
        self.targets
            .iter()
            .find(|t| t.joint_name == joint)
            .map(|t| t.value)
    }

    pub fn contains_joint(&self, joint: &str) -> bool {
        self.targets.iter().any(|t| t.joint_name == joint)
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ActuatorCommand {
    pub actuator_name: String,
    pub value: f64,
    pub control_mode: String,
    pub skill_id: String,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ActuatorCommandSet {
    pub commands: Vec<ActuatorCommand>,
    pub hold_outside: HoldSemantics,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct IkTrace {
    pub initial_q: Vec<f64>,
    pub target_q: Vec<f64>,
    pub joint_delta_norm: f64,
    pub max_joint_move: f64,
    pub residual: f64,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn named_lookup_does_not_use_index() {
        let set = JointTargetSet {
            targets: vec![
                JointTarget {
                    joint_name: "j2".into(),
                    actuator_name: Some("a2".into()),
                    value: 0.4,
                    control_mode: "position".into(),
                    skill_id: "skill.reach".into(),
                },
                JointTarget {
                    joint_name: "j0".into(),
                    actuator_name: Some("a0".into()),
                    value: 0.1,
                    control_mode: "position".into(),
                    skill_id: "skill.reach".into(),
                },
            ],
            hold_outside: HoldSemantics::KeepCurrent,
        };
        assert_eq!(set.named_value("j0"), Some(0.1));
        assert_eq!(set.named_value("j2"), Some(0.4));
        assert!(!set.contains_joint("gripper"));
    }
}
