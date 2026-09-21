//! Signed actuator effort and available contact-force magnitude λ.
//!
//! τ_min_i ≤ τ_self_i + (Jᵀ d)_i λ ≤ τ_max_i,  λ ≥ 0.
//! Near-zero coupling is Singular (UNKNOWN), not infinite force.

use crate::contact::DIRECTION_COUPLING_EPS;
use crate::error::{finite, PhysicsError, PhysicsResult};

fn dot3(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn unit3(v: [f64; 3], name: &'static str) -> PhysicsResult<[f64; 3]> {
    let v = [
        finite(v[0], name)?,
        finite(v[1], name)?,
        finite(v[2], name)?,
    ];
    let n = dot3(v, v).sqrt();
    if n <= 0.0 {
        return Err(PhysicsError::NonPositive(name));
    }
    Ok([v[0] / n, v[1] / n, v[2] / n])
}

/// Joint torque interval from actuator force range and transmission gear (signed).
pub fn joint_torque_limits_from_actuator(
    force_min: f64,
    force_max: f64,
    gear: f64,
) -> PhysicsResult<(f64, f64)> {
    let lo = finite(force_min, "force_min")?;
    let hi = finite(force_max, "force_max")?;
    let g = finite(gear, "gear")?;
    if g.abs() < 1e-12 {
        return Err(PhysicsError::Unevaluable("gear"));
    }
    let a = lo * g;
    let b = hi * g;
    Ok((a.min(b), a.max(b)))
}

/// Available λ ≥ 0 along a unit force direction under signed effort and self-load.
#[derive(Debug, Clone, PartialEq)]
pub struct AvailableLambda {
    pub lambda_min: f64,
    pub lambda_max: f64,
    pub limiting_index: usize,
    pub coupling: Vec<f64>,
}

/// Intersect per-joint intervals. Empty intersection is Unevaluable, not a huge λ.
pub fn available_lambda_interval(
    jacobian_columns: &[[f64; 3]],
    direction: [f64; 3],
    tau_self: &[f64],
    tau_min: &[f64],
    tau_max: &[f64],
) -> PhysicsResult<AvailableLambda> {
    let n = jacobian_columns.len();
    if n == 0 || n != tau_self.len() || n != tau_min.len() || n != tau_max.len() {
        return Err(PhysicsError::Unevaluable("dof_mismatch"));
    }
    let d = unit3(direction, "direction")?;
    let mut coupling = Vec::with_capacity(n);
    let mut max_abs_coupling = 0.0_f64;
    for col in jacobian_columns {
        let c = dot3(*col, d);
        max_abs_coupling = max_abs_coupling.max(c.abs());
        coupling.push(c);
    }
    if max_abs_coupling < DIRECTION_COUPLING_EPS {
        return Err(PhysicsError::Singular("direction_uncoupled"));
    }

    let mut lam_lo = 0.0_f64;
    let mut lam_hi = f64::INFINITY;
    let mut limiting_index = 0usize;
    for i in 0..n {
        let c = coupling[i];
        let ts = finite(tau_self[i], "tau_self")?;
        let tmin = finite(tau_min[i], "tau_min")?;
        let tmax = finite(tau_max[i], "tau_max")?;
        if tmin > tmax + 1e-15 {
            return Err(PhysicsError::Unevaluable("tau_bounds"));
        }
        if c.abs() < DIRECTION_COUPLING_EPS {
            if ts < tmin - 1e-12 || ts > tmax + 1e-12 {
                return Err(PhysicsError::Unevaluable("self_load_exceeds_effort"));
            }
            continue;
        }
        let (lo, hi) = if c > 0.0 {
            ((tmin - ts) / c, (tmax - ts) / c)
        } else {
            ((tmax - ts) / c, (tmin - ts) / c)
        };
        if hi < lam_hi {
            lam_hi = hi;
            limiting_index = i;
        }
        if lo > lam_lo {
            lam_lo = lo;
        }
    }
    if !lam_hi.is_finite() {
        return Err(PhysicsError::Singular("direction_uncoupled"));
    }
    if lam_hi + 1e-12 < lam_lo {
        return Err(PhysicsError::Unevaluable("lambda_empty"));
    }
    Ok(AvailableLambda {
        lambda_min: lam_lo,
        lambda_max: lam_hi.max(lam_lo),
        limiting_index,
        coupling,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn negative_gear_swaps_asymmetric_bounds() {
        let (lo, hi) = joint_torque_limits_from_actuator(-1.0, 5.0, -2.0).unwrap();
        assert!((lo + 10.0).abs() < 1e-12);
        assert!((hi - 2.0).abs() < 1e-12);
    }

    #[test]
    fn identity_jacobian_lambda_is_remaining_effort() {
        let cols = vec![[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
        let bound = available_lambda_interval(
            &cols,
            [1.0, 0.0, 0.0],
            &[2.0, 0.0],
            &[-10.0, -10.0],
            &[10.0, 10.0],
        )
        .unwrap();
        assert!((bound.lambda_max - 8.0).abs() < 1e-12);
        assert_eq!(bound.limiting_index, 0);
        assert!(bound.lambda_min <= 0.0 + 1e-12);
    }

    #[test]
    fn self_load_can_zero_available_lambda_while_gross_still_covers() {
        let cols = vec![[1.0, 0.0, 0.0]];
        let d = [1.0, 0.0, 0.0];
        let gross = available_lambda_interval(&cols, d, &[0.0], &[-5.0], &[5.0]).unwrap();
        let avail = available_lambda_interval(&cols, d, &[4.5], &[-5.0], &[5.0]).unwrap();
        assert!(gross.lambda_max > 4.0);
        assert!(avail.lambda_max < 1.0);
        assert!(avail.lambda_max < gross.lambda_max - 1.0);
    }

    #[test]
    fn asymmetric_bounds_change_limiting_joint() {
        let cols = vec![[1.0, 0.0, 0.0], [0.5, 0.0, 0.0]];
        let d = [1.0, 0.0, 0.0];
        let a = available_lambda_interval(&cols, d, &[0.0, 0.0], &[-20.0, -20.0], &[20.0, 20.0])
            .unwrap();
        let b =
            available_lambda_interval(&cols, d, &[0.0, 0.0], &[-20.0, -1.0], &[20.0, 2.0]).unwrap();
        assert_eq!(a.limiting_index, 0);
        assert_eq!(b.limiting_index, 1);
        assert!(b.lambda_max < a.lambda_max - 1.0);
        assert!((b.lambda_max - 4.0).abs() < 1e-12);
    }

    #[test]
    fn negative_gear_interval_is_not_abs_collapse() {
        let cols = vec![[1.0, 0.0, 0.0]];
        let (tmin, tmax) = joint_torque_limits_from_actuator(-1.0, 5.0, -2.0).unwrap();
        let along_pos = available_lambda_interval(&cols, [1.0, 0.0, 0.0], &[0.0], &[tmin], &[tmax]);
        let along_neg =
            available_lambda_interval(&cols, [-1.0, 0.0, 0.0], &[0.0], &[tmin], &[tmax]);
        let pos = along_pos.unwrap();
        let neg = along_neg.unwrap();
        assert!((pos.lambda_max - 2.0).abs() < 1e-12);
        assert!((neg.lambda_max - 10.0).abs() < 1e-12);
        assert!((pos.lambda_max - neg.lambda_max).abs() > 1.0);
    }

    #[test]
    fn near_zero_coupling_is_singular_not_huge_lambda() {
        let cols = vec![[0.0, 1e-12, 0.0], [0.0, 0.0, 0.0]];
        let err = available_lambda_interval(
            &cols,
            [1.0, 0.0, 0.0],
            &[0.0, 0.0],
            &[-5.0, -5.0],
            &[5.0, 5.0],
        )
        .unwrap_err();
        assert_eq!(err, PhysicsError::Singular("direction_uncoupled"));
    }

    #[test]
    fn self_load_outside_uncoupled_joint_is_empty() {
        let cols = vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]];
        let err = available_lambda_interval(
            &cols,
            [1.0, 0.0, 0.0],
            &[9.0, 0.0],
            &[-1.0, -10.0],
            &[1.0, 10.0],
        )
        .unwrap_err();
        assert_eq!(err, PhysicsError::Unevaluable("self_load_exceeds_effort"));
    }

    #[test]
    fn unknown_gear_is_unevaluable() {
        assert!(joint_torque_limits_from_actuator(-1.0, 1.0, 0.0).is_err());
    }
}
