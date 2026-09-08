//! Discrete-time freshness. Heartbeat / dispose / Nyquist.

use crate::error::{positive, PhysicsResult};

pub fn period_s(hz: f64) -> PhysicsResult<f64> {
    Ok(1.0 / positive(hz, "hz")?)
}

/// Nyquist frequency: f_s / 2
pub fn nyquist_hz(sample_hz: f64) -> PhysicsResult<f64> {
    Ok(positive(sample_hz, "sample_hz")? / 2.0)
}

/// Stale if age > k · period (Governor uses k=2 for heartbeat).
pub fn is_stale(age_s: f64, period_s: f64, multiples: f64) -> PhysicsResult<bool> {
    let age = crate::error::finite(age_s, "age")?;
    let p = positive(period_s, "period")?;
    let k = positive(multiples, "multiples")?;
    Ok(age > k * p)
}

pub fn dispose_period_s() -> f64 {
    1.0 / crate::si::DISPOSE_HZ
}

pub fn screen_period_s() -> f64 {
    1.0 / crate::si::SCREEN_HZ
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn one_khz_dispose_is_one_ms() {
        assert!((dispose_period_s() - 0.001).abs() < 1e-15);
        assert_eq!(nyquist_hz(1000.0).unwrap(), 500.0);
    }

    #[test]
    fn heartbeat_stale_at_two_periods() {
        assert!(!is_stale(1.9, 1.0, 2.0).unwrap());
        assert!(is_stale(2.1, 1.0, 2.0).unwrap());
    }
}
