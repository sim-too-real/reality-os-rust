//! Joint / workspace margins. Geometry, not FEA.

use crate::error::{finite, PhysicsResult};

/// Signed interior margin. Negative ⇒ outside.
pub fn joint_limit_margin(q: f64, q_min: f64, q_max: f64) -> PhysicsResult<f64> {
    let q = finite(q, "q")?;
    let lo = finite(q_min, "q_min")?;
    let hi = finite(q_max, "q_max")?;
    if hi < lo {
        return Err(crate::error::PhysicsError::NonPositive("q_max_lt_q_min"));
    }
    Ok((q - lo).min(hi - q))
}

pub fn in_limits(q: f64, q_min: f64, q_max: f64) -> PhysicsResult<bool> {
    Ok(joint_limit_margin(q, q_min, q_max)? >= -1e-12)
}
