//! Allowed vs forbidden contact is policy over pair + phase + declared
//! topology. Geometry only answers intersection. No robot-identity branches.

use crate::contact::{classify_contact_pair, ContactClassContext, ContactEvidenceClass};
use crate::embodiment::EmbodimentModel;
use crate::maneuver_witness::TransitionKind;

/// Task-phase for allowed-contact. Names are physical, not robot-specific.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ContactPhase {
    CurrentToApproach,
    ApproachToContact,
    ContactStroke,
    GraspApproach,
    GraspClose,
    FreeSpace,
}

impl ContactPhase {
    pub fn from_transition(kind: TransitionKind) -> Self {
        match kind {
            TransitionKind::CurrentToApproach => Self::CurrentToApproach,
            TransitionKind::ApproachToContact => Self::ApproachToContact,
            TransitionKind::ContactToMidStroke | TransitionKind::MidToEndStroke => {
                Self::ContactStroke
            }
        }
    }

    /// Intended tool↔object is permitted at this sample.
    /// Early approach samples are forbidden (forearm/tool strike).
    pub fn intended_tool_object_ok(self, sample_i: usize, n_samples: usize) -> bool {
        match self {
            Self::ContactStroke | Self::GraspClose => true,
            Self::ApproachToContact | Self::GraspApproach => {
                n_samples > 0 && sample_i + 1 == n_samples
            }
            // Start may already sit near the face; retracting is not a strike.
            Self::CurrentToApproach | Self::FreeSpace => true,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PairPermission {
    Allowed,
    Forbidden,
    Unknown,
}

#[derive(Debug, Clone, PartialEq)]
pub struct AllowedContactPolicy {
    pub object_id: String,
    pub intended_tool_bodies: Vec<String>,
    pub robot_bodies: Vec<String>,
    pub support_bodies: Vec<String>,
    pub obstacle_bodies: Vec<String>,
    pub adjacent_body_pairs: Vec<(String, String)>,
}

impl AllowedContactPolicy {
    pub fn from_names(
        object_id: impl Into<String>,
        intended: Vec<String>,
        robot: Vec<String>,
        support: Vec<String>,
        obstacles: Vec<String>,
        adjacent: Vec<(String, String)>,
    ) -> Self {
        Self {
            object_id: object_id.into(),
            intended_tool_bodies: intended,
            robot_bodies: robot,
            support_bodies: support,
            obstacle_bodies: obstacles,
            adjacent_body_pairs: adjacent,
        }
    }

    pub fn class_of(&self, a: &str, b: &str) -> Option<ContactEvidenceClass> {
        classify_contact_pair(
            a,
            b,
            &ContactClassContext {
                object_id: &self.object_id,
                intended: &self.intended_tool_bodies,
                robot_bodies: &self.robot_bodies,
                support_bodies: &self.support_bodies,
                obstacle_bodies: &self.obstacle_bodies,
            },
        )
    }

    pub fn adjacent(&self, a: &str, b: &str) -> bool {
        self.adjacent_body_pairs
            .iter()
            .any(|(x, y)| (x == a && y == b) || (x == b && y == a))
    }

    pub fn permit(
        &self,
        a: &str,
        b: &str,
        phase: ContactPhase,
        sample_i: usize,
        n_samples: usize,
    ) -> PairPermission {
        let Some(class) = self.class_of(a, b) else {
            return PairPermission::Unknown;
        };
        match class {
            ContactEvidenceClass::IntendedToolContact => {
                if phase.intended_tool_object_ok(sample_i, n_samples) {
                    PairPermission::Allowed
                } else {
                    PairPermission::Forbidden
                }
            }
            ContactEvidenceClass::SelfCollision => {
                if self.adjacent(a, b) {
                    PairPermission::Allowed
                } else {
                    PairPermission::Forbidden
                }
            }
            ContactEvidenceClass::SupportContact
            | ContactEvidenceClass::ObstacleContact
            | ContactEvidenceClass::UnintendedRobotContact => PairPermission::Forbidden,
        }
    }
}

/// Parent↔child body pairs from the kinematic tree. Not robot-name exceptions.
pub fn adjacent_body_pairs(model: &EmbodimentModel) -> Vec<(String, String)> {
    let mut out = Vec::new();
    for b in &model.bodies {
        if let Some(p) = &b.parent {
            if !p.is_empty() && p != "world" {
                out.push((p.clone(), b.name.clone()));
            }
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapter::synth_planar_two_link;

    fn policy() -> AllowedContactPolicy {
        AllowedContactPolicy::from_names(
            "object",
            vec!["tool".into(), "finger".into()],
            vec![
                "link1".into(),
                "link2".into(),
                "tool".into(),
                "finger".into(),
            ],
            vec!["table".into()],
            vec!["obstacle".into()],
            vec![("link1".into(), "link2".into())],
        )
    }

    #[test]
    fn adjacent_self_overlap_is_allowed_by_topology() {
        let p = policy();
        assert_eq!(
            p.permit("link1", "link2", ContactPhase::FreeSpace, 0, 1),
            PairPermission::Allowed
        );
    }

    #[test]
    fn non_adjacent_self_collision_is_forbidden() {
        let p = policy();
        assert_eq!(
            p.permit("link1", "finger", ContactPhase::FreeSpace, 0, 1),
            PairPermission::Forbidden
        );
    }

    #[test]
    fn early_tool_object_on_approach_is_forbidden() {
        let p = policy();
        assert_eq!(
            p.permit("tool", "object", ContactPhase::ApproachToContact, 0, 8),
            PairPermission::Forbidden
        );
        assert_eq!(
            p.permit("tool", "object", ContactPhase::ApproachToContact, 7, 8),
            PairPermission::Allowed
        );
    }

    #[test]
    fn forearm_object_is_unintended_and_forbidden() {
        let p = policy();
        assert_eq!(
            p.permit("link1", "object", ContactPhase::ContactStroke, 0, 1),
            PairPermission::Forbidden
        );
        assert_eq!(
            p.class_of("link1", "object"),
            Some(ContactEvidenceClass::UnintendedRobotContact)
        );
    }

    #[test]
    fn adjacent_pairs_come_from_tree_not_robot_id() {
        let model = synth_planar_two_link();
        let pairs = adjacent_body_pairs(&model);
        assert!(pairs.iter().any(|(a, b)| a == "link1" && b == "link2"));
        assert!(!pairs.iter().any(|(a, b)| a == "base" && b == "link2"));
    }

    #[test]
    fn finger_object_is_intended_during_grasp_close() {
        let p = policy();
        assert_eq!(
            p.permit("finger", "object", ContactPhase::GraspClose, 0, 1),
            PairPermission::Allowed
        );
    }
}
