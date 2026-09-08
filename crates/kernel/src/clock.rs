//! Deterministic time: callers inject `now_s`. Wall clock is opt-in for CLI.

use std::time::{SystemTime, UNIX_EPOCH};

/// Seconds since Unix epoch. Tests pass an explicit value; never sleep on the gate.
#[inline]
pub fn unix_now_s() -> f64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0.0, |d| d.as_secs_f64())
}

#[inline]
pub fn finite_or_err(value: f64, field: &'static str) -> crate::error::KernelResult<f64> {
    if value.is_finite() {
        Ok(value)
    } else {
        Err(crate::error::KernelError::validation(
            field,
            "must be finite",
        ))
    }
}
