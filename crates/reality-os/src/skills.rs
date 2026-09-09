//! GoalIR / SkillIR. Free text never becomes motor authority.
//! The catalog is the sole verb admission source.

use serde::{Deserialize, Serialize};

use crate::plan::Intent;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GoalIR {
    pub id: String,
    pub outcome: String,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SkillIR {
    pub id: String,
    pub version: String,
    pub verb: String,
    pub required_capabilities: Vec<String>,
    pub preconditions: Vec<String>,
    pub effects: Vec<String>,
    pub authority_ceiling: String,
    pub timeout_s: f64,
    pub recovery: Vec<String>,
}

impl SkillIR {
    pub fn hold() -> Self {
        Self::entry(
            "hold",
            &[],
            &[],
            &["zero_effort"],
            &["stop"],
            30.0,
        )
    }

    fn entry(
        verb: &str,
        caps: &[&str],
        pre: &[&str],
        effects: &[&str],
        recovery: &[&str],
        timeout_s: f64,
    ) -> Self {
        Self {
            id: format!("skill.{verb}"),
            version: "1".into(),
            verb: verb.into(),
            required_capabilities: caps.iter().map(|s| (*s).into()).collect(),
            preconditions: pre.iter().map(|s| (*s).into()).collect(),
            effects: effects.iter().map(|s| (*s).into()).collect(),
            authority_ceiling: "allow".into(),
            timeout_s,
            recovery: recovery.iter().map(|s| (*s).into()).collect(),
        }
    }

    /// Sole admitted skill set. Unknown verbs are not in this list.
    pub fn catalog() -> Vec<Self> {
        vec![
            Self::entry(
                "grasp",
                &["serial_arm"],
                &["observation_fresh"],
                &["contact"],
                &["hold", "stop"],
                20.0,
            ),
            Self::entry(
                "pick",
                &["serial_arm"],
                &["observation_fresh", "grasp"],
                &["object_in_hand"],
                &["hold", "place"],
                20.0,
            ),
            Self::entry("lift", &["serial_arm"], &["grasp"], &["raised"], &["hold"], 15.0),
            Self::hold(),
            Self::entry(
                "place",
                &["serial_arm"],
                &["observation_fresh"],
                &["object_on_target"],
                &["hold", "stop"],
                20.0,
            ),
            Self::entry("slide", &["serial_arm"], &[], &["translated"], &["hold"], 15.0),
            Self::entry("push", &["serial_arm"], &[], &["contact_force"], &["hold"], 10.0),
            Self::entry(
                "insert",
                &["serial_arm", "cartesian_impedance"],
                &["observation_fresh"],
                &["seated"],
                &["hold", "stop"],
                20.0,
            ),
            Self::entry("press", &["single_dof"], &[], &["force_applied"], &["hold"], 10.0),
            Self::entry("carry", &["serial_arm"], &["grasp"], &["transported"], &["hold"], 30.0),
            Self::entry(
                "walk",
                &["floating_base"],
                &["stance_stable"],
                &["translated"],
                &["hold", "stop"],
                30.0,
            ),
            Self::entry("stop", &[], &[], &["zero_effort"], &["hold"], 5.0),
            Self::entry(
                "reach",
                &["serial_arm"],
                &[],
                &["near_target"],
                &["hold"],
                15.0,
            ),
        ]
    }

    pub fn admit(verb: &str) -> Option<Self> {
        Self::catalog().into_iter().find(|s| s.verb == verb)
    }

    pub fn to_intent(&self) -> Intent {
        Intent::language(&self.id, &self.verb)
    }

    pub fn admits_text_as_motion(&self) -> bool {
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn skill_is_not_a_torque() {
        let s = SkillIR::hold();
        assert!(!s.admits_text_as_motion());
        assert_eq!(s.to_intent().verb, "hold");
    }

    #[test]
    fn catalog_is_the_only_admission_list() {
        assert!(SkillIR::admit("hold").is_some());
        assert!(SkillIR::admit("walk").is_some());
        assert!(SkillIR::admit("dance").is_none());
        assert!(SkillIR::admit("walk")
            .unwrap()
            .required_capabilities
            .contains(&"floating_base".into()));
    }
}
