use crate::skill::SkillRefuse;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum ManipulationFailure {
    Miss,
    Unreachable,
    ObjectMoved,
    StaleObject,
    BlockedApproach,
    GripperEmptyClose,
    Slip,
    ExcessForce,
    UnexpectedContact,
    ResourceUnsupported,
    CouplingUnsupported,
    StaleGripperState,
    Blocked,
    ActuatorSaturation,
    ForceBoundUnavailable,
    ObjectNotMovable,
    SlipAroundObject,
    RobotBlocked,
    TargetStale,
    Replay,
    ControllerFailure,
    ModelFeatureUnsupported,
}

impl ManipulationFailure {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Miss => "MISS",
            Self::Unreachable => "UNREACHABLE",
            Self::ObjectMoved => "OBJECT_MOVED",
            Self::StaleObject => "STALE_OBJECT",
            Self::BlockedApproach => "BLOCKED_APPROACH",
            Self::GripperEmptyClose => "GRIPPER_EMPTY_CLOSE",
            Self::Slip => "SLIP",
            Self::ExcessForce => "EXCESS_FORCE",
            Self::UnexpectedContact => "UNEXPECTED_CONTACT",
            Self::ResourceUnsupported => "RESOURCE_UNSUPPORTED",
            Self::CouplingUnsupported => "COUPLING_UNSUPPORTED",
            Self::StaleGripperState => "STALE_GRIPPER_STATE",
            Self::Blocked => "BLOCKED",
            Self::ActuatorSaturation => "ACTUATOR_SATURATION",
            Self::ForceBoundUnavailable => "FORCE_BOUND_UNAVAILABLE",
            Self::ObjectNotMovable => "OBJECT_NOT_MOVABLE",
            Self::SlipAroundObject => "SLIP_AROUND_OBJECT",
            Self::RobotBlocked => "ROBOT_BLOCKED",
            Self::TargetStale => "TARGET_STALE",
            Self::Replay => "REPLAY",
            Self::ControllerFailure => "CONTROLLER_FAILURE",
            Self::ModelFeatureUnsupported => "MODEL_FEATURE_UNSUPPORTED",
        }
    }

    pub fn writes_allowed(self) -> bool {
        false
    }

    pub fn from_refuse(r: SkillRefuse) -> Self {
        match r {
            SkillRefuse::Unreachable => Self::Unreachable,
            SkillRefuse::ResourceUnsupported => Self::ResourceUnsupported,
            SkillRefuse::StaleGripperState => Self::StaleGripperState,
            SkillRefuse::StaleObject => Self::StaleObject,
            SkillRefuse::Blocked => Self::Blocked,
            SkillRefuse::UnexpectedContact => Self::UnexpectedContact,
            SkillRefuse::ModelFeatureUnsupported => Self::ModelFeatureUnsupported,
            SkillRefuse::StaleEvidence => Self::TargetStale,
            SkillRefuse::ForceBoundUnavailable => Self::ForceBoundUnavailable,
            _ => Self::ResourceUnsupported,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_failure_blocks_further_writes() {
        for f in [
            ManipulationFailure::Miss,
            ManipulationFailure::Unreachable,
            ManipulationFailure::ObjectMoved,
            ManipulationFailure::StaleObject,
            ManipulationFailure::BlockedApproach,
            ManipulationFailure::GripperEmptyClose,
            ManipulationFailure::Slip,
            ManipulationFailure::ExcessForce,
            ManipulationFailure::UnexpectedContact,
            ManipulationFailure::ResourceUnsupported,
            ManipulationFailure::CouplingUnsupported,
            ManipulationFailure::Replay,
            ManipulationFailure::ControllerFailure,
        ] {
            assert!(!f.writes_allowed());
        }
    }
}
