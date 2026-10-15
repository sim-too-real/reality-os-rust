//! Exact action authority shared by reasoning, grants, and execution.
//!
//! These values describe a bounded transaction. They do not create authority
//! on their own; an issued grant must bind the exact value.

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum PhysicalActionEnvelope {
    LinearStroke {
        requested_m: f64,
        maximum_m: f64,
    },
    PinchGrasp {
        resource_id: String,
        finger_joint_names: Vec<String>,
        open_positions: Vec<f64>,
        close_positions: Vec<f64>,
        per_finger_force_ceiling_n: f64,
        lift_trajectory_digest: String,
        maximum_lift_m: f64,
    },
}

impl PhysicalActionEnvelope {
    /// Checks the envelope's structure and numeric bounds independent of a model.
    pub fn validate(&self) -> Result<(), &'static str> {
        match self {
            Self::LinearStroke {
                requested_m,
                maximum_m,
            } => {
                if !requested_m.is_finite() || *requested_m <= 0.0 {
                    return Err("requested_stroke");
                }
                if !maximum_m.is_finite() || *maximum_m < *requested_m {
                    return Err("maximum_stroke");
                }
            }
            Self::PinchGrasp {
                resource_id,
                finger_joint_names,
                open_positions,
                close_positions,
                per_finger_force_ceiling_n,
                lift_trajectory_digest,
                maximum_lift_m,
            } => {
                if resource_id.trim().is_empty() {
                    return Err("resource_id");
                }
                if finger_joint_names.len() != 2
                    || finger_joint_names.iter().any(|name| name.trim().is_empty())
                    || finger_joint_names[0] == finger_joint_names[1]
                {
                    return Err("finger_joint_names");
                }
                if open_positions.len() != finger_joint_names.len()
                    || close_positions.len() != finger_joint_names.len()
                {
                    return Err("finger_position_dimensions");
                }
                for (&open, &close) in open_positions.iter().zip(close_positions) {
                    if !open.is_finite() || !close.is_finite() || !(close - open).is_finite() {
                        return Err("finger_positions");
                    }
                }
                if !per_finger_force_ceiling_n.is_finite() || *per_finger_force_ceiling_n <= 0.0 {
                    return Err("force_ceiling");
                }
                if lift_trajectory_digest.len() != 64
                    || !lift_trajectory_digest
                        .bytes()
                        .all(|byte| byte.is_ascii_hexdigit())
                {
                    return Err("lift_trajectory_digest");
                }
                if !maximum_lift_m.is_finite() || *maximum_lift_m < 0.02 {
                    return Err("maximum_lift");
                }
            }
        }
        Ok(())
    }

    /// Returns the command and maximum for a linear action, if this is one.
    pub fn linear_stroke_bounds(&self) -> Option<(f64, f64)> {
        match self {
            Self::LinearStroke {
                requested_m,
                maximum_m,
            } => Some((*requested_m, *maximum_m)),
            Self::PinchGrasp { .. } => None,
        }
    }

    /// Compares the exact structured authority, including each float bit pattern.
    pub fn exactly_matches(&self, other: &Self) -> bool {
        match (self, other) {
            (
                Self::LinearStroke {
                    requested_m: left_requested,
                    maximum_m: left_maximum,
                },
                Self::LinearStroke {
                    requested_m: right_requested,
                    maximum_m: right_maximum,
                },
            ) => {
                left_requested.to_bits() == right_requested.to_bits()
                    && left_maximum.to_bits() == right_maximum.to_bits()
            }
            (
                Self::PinchGrasp {
                    resource_id: left_resource,
                    finger_joint_names: left_names,
                    open_positions: left_open,
                    close_positions: left_close,
                    per_finger_force_ceiling_n: left_force,
                    lift_trajectory_digest: left_lift_digest,
                    maximum_lift_m: left_lift,
                },
                Self::PinchGrasp {
                    resource_id: right_resource,
                    finger_joint_names: right_names,
                    open_positions: right_open,
                    close_positions: right_close,
                    per_finger_force_ceiling_n: right_force,
                    lift_trajectory_digest: right_lift_digest,
                    maximum_lift_m: right_lift,
                },
            ) => {
                left_resource == right_resource
                    && left_names == right_names
                    && exact_float_vec(left_open, right_open)
                    && exact_float_vec(left_close, right_close)
                    && left_force.to_bits() == right_force.to_bits()
                    && left_lift_digest == right_lift_digest
                    && left_lift.to_bits() == right_lift.to_bits()
            }
            _ => false,
        }
    }
}

fn exact_float_vec(left: &[f64], right: &[f64]) -> bool {
    left.len() == right.len()
        && left
            .iter()
            .zip(right)
            .all(|(left, right)| left.to_bits() == right.to_bits())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn exact_match_distinguishes_signed_zero_in_position_vectors() {
        let left = PhysicalActionEnvelope::PinchGrasp {
            resource_id: "gripper".into(),
            finger_joint_names: vec!["left".into(), "right".into()],
            open_positions: vec![-0.0, 0.0],
            close_positions: vec![0.01, 0.01],
            per_finger_force_ceiling_n: 2.0,
            lift_trajectory_digest: "a".repeat(64),
            maximum_lift_m: 0.02,
        };
        let mut right = left.clone();
        if let PhysicalActionEnvelope::PinchGrasp { open_positions, .. } = &mut right {
            open_positions[0] = 0.0;
        }
        assert!(!left.exactly_matches(&right));
    }
}
