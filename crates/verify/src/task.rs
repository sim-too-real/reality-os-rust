//! TaskSpec is a desired outcome, not a hardcoded trajectory.

use crate::normalize::RobotManifest;
use crate::observation::VerifierTruth;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum TaskSpec {
    Reach {
        end_effector: String,
        target: [f64; 3],
        radius: f64,
    },
    Pick {
        object: String,
    },
    Place {
        object: String,
        target_region: String,
    },
    MoveBase {
        target_pose: [f64; 3],
    },
    Hold {
        duration_s: f64,
    },
    JointTrack {
        target: Vec<f64>,
        tolerance: f64,
    },
    KeepOut {
        name: String,
    },
    AuthorityNegative {
        expect: String,
    },
}

impl TaskSpec {
    pub fn id(&self) -> String {
        match self {
            Self::Reach { end_effector, .. } => format!("reach:{end_effector}"),
            Self::Pick { object } => format!("pick:{object}"),
            Self::Place { object, .. } => format!("place:{object}"),
            Self::MoveBase { .. } => "move_base".into(),
            Self::Hold { .. } => "hold".into(),
            Self::JointTrack { .. } => "joint_track".into(),
            Self::KeepOut { name } => format!("keep_out:{name}"),
            Self::AuthorityNegative { expect } => format!("authority:{expect}"),
        }
    }

    pub fn verb(&self) -> &'static str {
        match self {
            Self::Reach { .. } => "drive",
            Self::Pick { .. } => "hold",
            Self::Place { .. } => "place",
            Self::MoveBase { .. } => "drive",
            Self::Hold { .. } => "hold",
            Self::JointTrack { .. } => "drive",
            Self::KeepOut { .. } => "hold",
            Self::AuthorityNegative { expect } if expect.contains("place") => "place",
            Self::AuthorityNegative { .. } => "drive",
        }
    }

    pub fn requires_gripper(&self) -> bool {
        matches!(self, Self::Pick { .. } | Self::Place { .. })
    }

    pub fn requires_mobile_base(&self) -> bool {
        matches!(self, Self::MoveBase { .. })
    }

    pub fn evaluate(&self, truth: &VerifierTruth, manifest: &RobotManifest) -> bool {
        match self {
            Self::Hold { .. } => !truth.nan && truth.qvel.iter().all(|v| v.abs() < 25.0),
            Self::JointTrack { target, tolerance } => {
                if truth.qpos.len() < target.len() {
                    return false;
                }
                target
                    .iter()
                    .zip(truth.qpos.iter())
                    .all(|(t, q)| (t - q).abs() <= *tolerance)
            }
            Self::Reach {
                end_effector,
                target,
                radius,
            } => {
                let p = truth
                    .named_pos
                    .get(end_effector)
                    .or_else(|| truth.xpos.get(end_effector))
                    .or_else(|| {
                        manifest
                            .end_effector_name()
                            .and_then(|n| truth.named_pos.get(&n).or_else(|| truth.xpos.get(&n)))
                    })
                    .cloned()
                    .or_else(|| truth.ee_pos());
                match p {
                    Some(pos) if pos.len() >= 3 => {
                        let d = (0..3)
                            .map(|i| pos[i] - target[i])
                            .map(|x| x * x)
                            .sum::<f64>()
                            .sqrt();
                        d <= *radius
                    }
                    _ => false,
                }
            }
            Self::KeepOut { name } => !truth.zone_entries.iter().any(|z| z == name),
            Self::AuthorityNegative { .. } => false,
            Self::Pick { object } => truth.grasped.iter().any(|o| o == object),
            Self::Place {
                object,
                target_region,
            } => truth
                .placed
                .iter()
                .any(|(o, r)| o == object && r == target_region),
            Self::MoveBase { target_pose } => truth
                .named_pos
                .get("base")
                .map(|p| {
                    p.len() >= 2 && (p[0] - target_pose[0]).hypot(p[1] - target_pose[1]) < 0.15
                })
                .unwrap_or(false),
        }
    }
}
