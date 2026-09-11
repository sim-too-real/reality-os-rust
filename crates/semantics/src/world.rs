use crate::provenance::{Provenance, Provenanced};
use crate::transform::Se3;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ObjectHypothesis {
    pub id: String,
    pub pose: Provenanced<Se3>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PoseEvidence {
    pub frame_id: String,
    pub pose: Provenanced<Se3>,
    pub expires_at_s: f64,
}

impl PoseEvidence {
    pub fn xyz(&self) -> Option<[f64; 3]> {
        self.pose.value.map(|p| p.xyz)
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ReachGoal {
    pub end_effector: String,
    pub target: PoseEvidence,
    pub success_radius: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WorldState {
    pub transform_epoch: String,
    pub as_of_s: f64,
    pub goal: ReachGoal,
    pub objects: Vec<ObjectHypothesis>,
}

impl WorldState {
    pub fn empty(transform_epoch: impl Into<String>, as_of_s: f64) -> Self {
        Self {
            transform_epoch: transform_epoch.into(),
            as_of_s,
            goal: ReachGoal {
                end_effector: String::new(),
                target: PoseEvidence {
                    frame_id: String::new(),
                    pose: Provenanced::unknown("world.target", as_of_s),
                    expires_at_s: as_of_s,
                },
                success_radius: 0.05,
            },
            objects: Vec::new(),
        }
    }

    pub fn end_effector(&self) -> &str {
        &self.goal.end_effector
    }

    pub fn target_frame(&self) -> &str {
        &self.goal.target.frame_id
    }

    pub fn target_xyz(&self) -> Provenanced<[f64; 3]> {
        match self.goal.target.pose.value {
            Some(p) => Provenanced {
                value: Some(p.xyz),
                provenance: self.goal.target.pose.provenance,
                source: self.goal.target.pose.source.clone(),
                as_of_s: self.goal.target.pose.as_of_s,
                uncertainty: None,
            },
            None => Provenanced::unknown("world.target", self.as_of_s),
        }
    }

    pub fn with_target(
        self,
        end_effector: impl Into<String>,
        xyz: [f64; 3],
        expires: f64,
        epoch: impl Into<String>,
        now: f64,
        provenance: Provenance,
    ) -> Self {
        self.with_target_in_frame(end_effector, "world", xyz, expires, epoch, now, provenance)
    }

    #[allow(clippy::too_many_arguments)]
    pub fn with_target_in_frame(
        mut self,
        end_effector: impl Into<String>,
        frame_id: impl Into<String>,
        xyz: [f64; 3],
        expires: f64,
        epoch: impl Into<String>,
        now: f64,
        provenance: Provenance,
    ) -> Self {
        self.transform_epoch = epoch.into();
        self.as_of_s = now;
        let pose = Se3::try_new(xyz, [1.0, 0.0, 0.0, 0.0]).ok();
        self.goal = ReachGoal {
            end_effector: end_effector.into(),
            target: PoseEvidence {
                frame_id: frame_id.into(),
                pose: Provenanced {
                    value: pose,
                    provenance,
                    source: "world.target".into(),
                    as_of_s: now,
                    uncertainty: None,
                },
                expires_at_s: expires,
            },
            success_radius: 0.05,
        };
        self
    }

    pub fn with_success_radius(mut self, radius: f64) -> Self {
        self.goal.success_radius = radius;
        self
    }

    pub fn target_fresh(&self, now: f64, freshness: f64) -> bool {
        let Some(pose) = self.goal.target.pose.value else {
            return false;
        };
        if now > self.goal.target.expires_at_s {
            return false;
        }
        if now - self.as_of_s > freshness {
            return false;
        }
        pose.xyz.iter().all(|v| v.is_finite())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::provenance::Provenance;

    #[test]
    fn missing_target_is_unknown_not_origin() {
        let w = WorldState::empty("e0", 1.0);
        assert!(w.target_xyz().value.is_none());
        assert_eq!(w.target_xyz().provenance, Provenance::Unknown);
        assert!(!w.target_fresh(1.0, 0.25));
        assert!(w.end_effector().is_empty());
        assert!(w.target_frame().is_empty());
    }

    #[test]
    fn end_effector_is_not_the_target_frame() {
        let w = WorldState::empty("e0", 1.0).with_target_in_frame(
            "tool0",
            "world",
            [0.4, 0.0, 0.3],
            5.0,
            "e0",
            1.0,
            Provenance::UserDeclared,
        );
        assert_eq!(w.end_effector(), "tool0");
        assert_eq!(w.target_frame(), "world");
        assert_ne!(w.end_effector(), w.target_frame());
    }
}
