use crate::capability::CapName;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SkillName {
    Observe,
    LookAt,
    Reach,
    Grasp,
    Release,
    Push,
    Pull,
    MoveBase,
    Hold,
    Place,
    Press,
    Turn,
    Insert,
    Retract,
    VerifyState,
}

impl SkillName {
    pub fn parse(raw: &str) -> Result<Self, SkillRefuse> {
        match raw {
            "observe" => Ok(Self::Observe),
            "look_at" => Ok(Self::LookAt),
            "reach" => Ok(Self::Reach),
            "grasp" => Ok(Self::Grasp),
            "release" => Ok(Self::Release),
            "push" => Ok(Self::Push),
            "pull" => Ok(Self::Pull),
            "move_base" => Ok(Self::MoveBase),
            "hold" => Ok(Self::Hold),
            "place" => Ok(Self::Place),
            "press" => Ok(Self::Press),
            "turn" => Ok(Self::Turn),
            "insert" => Ok(Self::Insert),
            "retract" => Ok(Self::Retract),
            "verify_state" => Ok(Self::VerifyState),
            _ => Err(SkillRefuse::NotInIr),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SkillRefuse {
    Probe,
    Refuse,
    Unreachable,
    Unsupported,
    NotInIr,
    WrongModelHash,
    StaleEvidence,
    EpochMismatch,
    MissingActuator,
    MissingTarget,
}

impl SkillRefuse {
    pub fn writes_allowed(&self) -> bool {
        false
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct SkillContract {
    pub id: String,
    pub name: SkillName,
    pub required: Vec<CapName>,
    pub required_world: Vec<String>,
    pub success_evidence: Vec<String>,
    pub failure_evidence: Vec<String>,
    pub authority_ceiling: String,
    pub exploration_allowance: bool,
}

impl SkillContract {
    pub fn reach() -> Self {
        Self {
            id: "skill.reach".into(),
            name: SkillName::Reach,
            required: vec![CapName::CartesianPositionControl],
            required_world: vec!["target_xyz".into(), "transform_epoch".into()],
            success_evidence: vec!["privileged_ee_within_radius".into()],
            failure_evidence: vec![
                "UNREACHABLE".into(),
                "STALE_OBJECT".into(),
                "WRONG_ROBOT_HASH".into(),
                "MISSING_ACTUATOR".into(),
                "STALE_EVIDENCE".into(),
            ],
            authority_ceiling: "allow".into(),
            exploration_allowance: false,
        }
    }

    pub fn named(name: SkillName) -> Self {
        Self {
            id: format!("skill.{}", skill_id_suffix(name)),
            name,
            required: Vec::new(),
            required_world: Vec::new(),
            success_evidence: Vec::new(),
            failure_evidence: Vec::new(),
            authority_ceiling: String::new(),
            exploration_allowance: false,
        }
    }

    pub fn is_qualified(&self) -> bool {
        self.name == SkillName::Reach && !self.required.is_empty()
    }
}

fn skill_id_suffix(name: SkillName) -> &'static str {
    match name {
        SkillName::Observe => "observe",
        SkillName::LookAt => "look_at",
        SkillName::Reach => "reach",
        SkillName::Grasp => "grasp",
        SkillName::Release => "release",
        SkillName::Push => "push",
        SkillName::Pull => "pull",
        SkillName::MoveBase => "move_base",
        SkillName::Hold => "hold",
        SkillName::Place => "place",
        SkillName::Press => "press",
        SkillName::Turn => "turn",
        SkillName::Insert => "insert",
        SkillName::Retract => "retract",
        SkillName::VerifyState => "verify_state",
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn llm_string_is_not_a_skill() {
        assert!(matches!(SkillName::parse("do_a_flip"), Err(SkillRefuse::NotInIr)));
    }

    #[test]
    fn only_reach_is_qualified() {
        assert!(SkillContract::reach().is_qualified());
        assert!(!SkillContract::named(SkillName::Grasp).is_qualified());
    }

    #[test]
    fn refuse_never_authorizes_writes() {
        for r in [
            SkillRefuse::Probe,
            SkillRefuse::Refuse,
            SkillRefuse::Unreachable,
            SkillRefuse::Unsupported,
            SkillRefuse::WrongModelHash,
            SkillRefuse::StaleEvidence,
            SkillRefuse::EpochMismatch,
            SkillRefuse::MissingActuator,
            SkillRefuse::MissingTarget,
            SkillRefuse::NotInIr,
        ] {
            assert!(!r.writes_allowed());
        }
    }
}
