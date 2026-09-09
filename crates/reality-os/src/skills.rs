//! GoalIR / SkillIR. Free text never becomes motor authority.

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
        Self {
            id: "skill.hold".into(),
            version: "1".into(),
            verb: "hold".into(),
            required_capabilities: Vec::new(),
            preconditions: Vec::new(),
            effects: vec!["zero_effort".into()],
            authority_ceiling: "allow".into(),
            timeout_s: 30.0,
            recovery: vec!["stop".into()],
        }
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
}
