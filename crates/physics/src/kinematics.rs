//! Constant-acceleration kinematics. Last-gate uses these for stop distance.

use crate::error::{positive, PhysicsResult};

/// v = v0 + a t
pub fn velocity(v0: f64, a: f64, t: f64) -> f64 {
    v0 + a * t
}

/// s = v0 t + ½ a t²
pub fn displacement(v0: f64, a: f64, t: f64) -> f64 {
    v0 * t + 0.5 * a * t * t
}

/// v² = v0² + 2 a s
pub fn speed_squared_after(v0: f64, a: f64, s: f64) -> f64 {
    v0 * v0 + 2.0 * a * s
}

/// Stopping distance under constant opposing deceleration: s = v² / (2 a).
/// Category-0 analog: dump kinetic energy at max declared decel (SIM envelope).
pub fn stop_distance_m(speed_m_s: f64, decel_m_s2: f64) -> PhysicsResult<f64> {
    let v = crate::error::nonneg(speed_m_s, "speed")?;
    let a = positive(decel_m_s2, "decel")?;
    Ok(v * v / (2.0 * a))
}

/// Time to stop: t = v / a
pub fn stop_time_s(speed_m_s: f64, decel_m_s2: f64) -> PhysicsResult<f64> {
    let v = crate::error::nonneg(speed_m_s, "speed")?;
    let a = positive(decel_m_s2, "decel")?;
    Ok(v / a)
}

/// Horizontal Coulomb decel ceiling a ≤ μ g (SIM μ, not MEASURED).
pub fn coulomb_decel_m_s2(mu: f64, g: f64) -> PhysicsResult<f64> {
    let mu = crate::error::nonneg(mu, "mu")?;
    let g = positive(g, "g")?;
    Ok(mu * g)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stop_distance_inverts_v2_eq() {
        let v = 4.0;
        let a = 2.0;
        let s = stop_distance_m(v, a).unwrap();
        assert!((speed_squared_after(v, -a, s) - 0.0).abs() < 1e-12);
        assert!((s - v * v / (2.0 * a)).abs() < 1e-12);
    }

    #[test]
    fn zero_speed_zero_distance() {
        assert_eq!(stop_distance_m(0.0, 1.0).unwrap(), 0.0);
    }

    #[test]
    fn nonpositive_decel_refuses() {
        assert!(stop_distance_m(1.0, 0.0).is_err());
        assert!(stop_distance_m(1.0, f64::NAN).is_err());
    }
}
