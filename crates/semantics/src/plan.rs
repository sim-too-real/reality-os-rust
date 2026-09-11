use crate::command::ActuatorCommandSet;
use crate::skill::SkillName;
use crate::world::PoseEvidence;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct VerifyMotion {
    pub displacement_xyz: [f64; 3],
    pub bound_m: f64,
    pub frame: String,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum SkillStep {
    Reach {
        end_effector: String,
        target: PoseEvidence,
        success_radius: f64,
    },
    ResourceCommand {
        resource_id: String,
        opening_01: f64,
        commands: ActuatorCommandSet,
        expires_at_s: f64,
        safe_on_expiry: String,
    },
    VerifyMotion(VerifyMotion),
    OpenThenRetract,
    Stop,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SkillPlan {
    pub skill: SkillName,
    pub contract_id: String,
    pub resource_id: Option<String>,
    pub topology: Option<String>,
    pub steps: Vec<SkillStep>,
    pub success_evidence: Vec<String>,
    pub failure_evidence: Vec<String>,
}

impl SkillPlan {
    pub fn empty(skill: SkillName, contract_id: impl Into<String>) -> Self {
        Self {
            skill,
            contract_id: contract_id.into(),
            resource_id: None,
            topology: None,
            steps: Vec::new(),
            success_evidence: Vec::new(),
            failure_evidence: Vec::new(),
        }
    }
}
