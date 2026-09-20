//! Contact SI identities. Caller supplies every physical parameter.
//!
//! No robot identity. No MuJoCo. No invented μ / mass / effort.

use crate::error::{finite, nonneg, PhysicsError, PhysicsResult};

/// Joint motion used by the translational contact Jacobian.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JointMotionKind {
    Revolute,
    Prismatic,
    Fixed,
}

/// Point-contact Coulomb cone membership.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConeMembership {
    Inside,
    Outside,
    Unevaluable,
}

/// Maximum contact force magnitude along a requested direction under |τ| ≤ τ_abs.
#[derive(Debug, Clone, PartialEq)]
pub struct DirectionForceBound {
    pub lambda_abs_max: f64,
    pub limiting_index: usize,
    pub coupling: Vec<f64>,
}

/// Minimum |J column · d| treated as producing force along `d`.
pub const DIRECTION_COUPLING_EPS: f64 = 1e-8;

fn dot3(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross3(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn sub3(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn norm3(v: [f64; 3]) -> f64 {
    dot3(v, v).sqrt()
}

fn unit3(v: [f64; 3], name: &'static str) -> PhysicsResult<[f64; 3]> {
    let v = [
        finite(v[0], name)?,
        finite(v[1], name)?,
        finite(v[2], name)?,
    ];
    let n = norm3(v);
    if n <= 0.0 {
        return Err(PhysicsError::NonPositive(name));
    }
    Ok([v[0] / n, v[1] / n, v[2] / n])
}

/// Coulomb sliding initiation threshold F = μ N.
pub fn coulomb_initiation_force_n(mu: f64, normal_force_n: f64) -> PhysicsResult<f64> {
    Ok(nonneg(mu, "mu")? * nonneg(normal_force_n, "normal_force")?)
}

/// Supported-object normal load N = m |g · n̂|.
pub fn supported_normal_force_n(
    mass_kg: f64,
    gravity_m_s2: [f64; 3],
    support_normal: [f64; 3],
) -> PhysicsResult<f64> {
    let mass = nonneg(mass_kg, "mass")?;
    let n = unit3(support_normal, "support_normal")?;
    let g = [
        finite(gravity_m_s2[0], "gravity")?,
        finite(gravity_m_s2[1], "gravity")?,
        finite(gravity_m_s2[2], "gravity")?,
    ];
    Ok(mass * dot3(g, n).abs())
}

/// Unilateral friction cone: f_n ≥ 0 and |f_t| ≤ μ f_n.
///
/// Returns membership and signed margin μ f_n − |f_t| (negative ⇒ outside).
pub fn friction_cone_membership(
    force: [f64; 3],
    contact_normal: [f64; 3],
    mu: f64,
) -> PhysicsResult<(ConeMembership, f64)> {
    let mu = nonneg(mu, "mu")?;
    let n = unit3(contact_normal, "contact_normal")?;
    let f = [
        finite(force[0], "force")?,
        finite(force[1], "force")?,
        finite(force[2], "force")?,
    ];
    let f_n = dot3(f, n);
    let f_t = sub3(f, [n[0] * f_n, n[1] * f_n, n[2] * f_n]);
    let t = norm3(f_t);
    if f_n < 0.0 {
        return Ok((ConeMembership::Outside, mu * f_n - t));
    }
    let margin = mu * f_n - t;
    let membership = if margin >= -1e-12 {
        ConeMembership::Inside
    } else {
        ConeMembership::Outside
    };
    Ok((membership, margin))
}

/// Translational Jacobian column at a contact point (not necessarily the EE origin).
pub fn translational_jacobian_column(
    kind: JointMotionKind,
    axis_world: [f64; 3],
    origin_world: [f64; 3],
    contact_world: [f64; 3],
) -> PhysicsResult<[f64; 3]> {
    let axis = [
        finite(axis_world[0], "axis")?,
        finite(axis_world[1], "axis")?,
        finite(axis_world[2], "axis")?,
    ];
    let origin = [
        finite(origin_world[0], "origin")?,
        finite(origin_world[1], "origin")?,
        finite(origin_world[2], "origin")?,
    ];
    let contact = [
        finite(contact_world[0], "contact")?,
        finite(contact_world[1], "contact")?,
        finite(contact_world[2], "contact")?,
    ];
    Ok(match kind {
        JointMotionKind::Prismatic => axis,
        JointMotionKind::Revolute => cross3(axis, sub3(contact, origin)),
        JointMotionKind::Fixed => [0.0, 0.0, 0.0],
    })
}

///  n translational Jacobian columns at `contact_world`.
pub fn translational_jacobian_at_point(
    kinds: &[JointMotionKind],
    origins_world: &[[f64; 3]],
    axes_world: &[[f64; 3]],
    contact_world: [f64; 3],
) -> PhysicsResult<Vec<[f64; 3]>> {
    if kinds.len() != origins_world.len() || kinds.len() != axes_world.len() {
        return Err(PhysicsError::Unevaluable("jacobian_chain_len"));
    }
    let mut cols = Vec::with_capacity(kinds.len());
    for i in 0..kinds.len() {
        cols.push(translational_jacobian_column(
            kinds[i],
            axes_world[i],
            origins_world[i],
            contact_world,
        )?);
    }
    Ok(cols)
}

/// Max |λ| such that τ = Jᵀ (λ d̂) satisfies |τ_i| ≤ τ_abs[i] for all i.
///
/// Does not invert J. Near-zero coupling along `d` is Singular, not a huge force.
pub fn max_force_along_direction(
    jacobian_columns: &[[f64; 3]],
    tau_abs: &[f64],
    direction: [f64; 3],
) -> PhysicsResult<DirectionForceBound> {
    if jacobian_columns.len() != tau_abs.len() {
        return Err(PhysicsError::Unevaluable("dof_mismatch"));
    }
    if jacobian_columns.is_empty() {
        return Err(PhysicsError::Singular("empty_jacobian"));
    }
    let d = unit3(direction, "direction")?;
    let mut coupling = Vec::with_capacity(jacobian_columns.len());
    let mut max_abs_coupling = 0.0_f64;
    for col in jacobian_columns {
        let c = dot3(*col, d);
        max_abs_coupling = max_abs_coupling.max(c.abs());
        coupling.push(c);
    }
    if max_abs_coupling < DIRECTION_COUPLING_EPS {
        return Err(PhysicsError::Singular("direction_uncoupled"));
    }
    let mut lambda_abs_max = f64::INFINITY;
    let mut limiting_index = 0usize;
    for (i, (c, tau)) in coupling.iter().zip(tau_abs.iter()).enumerate() {
        let tau = nonneg(*tau, "tau_abs")?;
        let ac = c.abs();
        if ac < DIRECTION_COUPLING_EPS {
            continue;
        }
        let lim = tau / ac;
        if lim < lambda_abs_max {
            lambda_abs_max = lim;
            limiting_index = i;
        }
    }
    if !lambda_abs_max.is_finite() {
        return Err(PhysicsError::Singular("direction_uncoupled"));
    }
    Ok(DirectionForceBound {
        lambda_abs_max,
        limiting_index,
        coupling,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn coulomb_initiation_is_mu_times_normal() {
        let mu = 0.4;
        let n = 12.0;
        let f = coulomb_initiation_force_n(mu, n).unwrap();
        assert!((f - mu * n).abs() < 1e-12);
        assert_eq!(coulomb_initiation_force_n(0.0, n).unwrap(), 0.0);
    }

    #[test]
    fn supported_normal_is_mass_times_projected_g() {
        let mass = 2.0;
        let g = [0.0, 0.0, -9.80665];
        let n = [0.0, 0.0, 1.0];
        let load = supported_normal_force_n(mass, g, n).unwrap();
        assert!((load - mass * 9.80665).abs() < 1e-12);
    }

    #[test]
    fn zero_friction_cone_rejects_any_tangent() {
        let n = [1.0, 0.0, 0.0];
        let (inside, _) = friction_cone_membership([2.0, 0.0, 0.0], n, 0.0).unwrap();
        assert_eq!(inside, ConeMembership::Inside);
        let (out, margin) = friction_cone_membership([2.0, 0.1, 0.0], n, 0.0).unwrap();
        assert_eq!(out, ConeMembership::Outside);
        assert!(margin < 0.0);
    }

    #[test]
    fn pulling_normal_is_outside_unilateral_cone() {
        let (m, _) = friction_cone_membership([-1.0, 0.0, 0.0], [1.0, 0.0, 0.0], 1.0).unwrap();
        assert_eq!(m, ConeMembership::Outside);
    }

    #[test]
    fn jacobian_offset_uses_contact_not_origin() {
        let axis = [0.0, 0.0, 1.0];
        let origin = [0.0, 0.0, 0.0];
        let ee = [0.3, 0.0, 0.0];
        let contact = [0.3, 0.05, 0.0];
        let at_ee =
            translational_jacobian_column(JointMotionKind::Revolute, axis, origin, ee).unwrap();
        let at_c = translational_jacobian_column(JointMotionKind::Revolute, axis, origin, contact)
            .unwrap();
        let expected_ee = [0.0, 0.3, 0.0];
        let expected_c = [-0.05, 0.3, 0.0];
        for i in 0..3 {
            assert!((at_ee[i] - expected_ee[i]).abs() < 1e-12);
            assert!((at_c[i] - expected_c[i]).abs() < 1e-12);
        }
        assert!((at_ee[0] - at_c[0]).abs() > 1e-9);
    }

    #[test]
    fn prismatic_column_is_the_axis() {
        let col = translational_jacobian_column(
            JointMotionKind::Prismatic,
            [0.0, 1.0, 0.0],
            [1.0, 2.0, 3.0],
            [9.0, 8.0, 7.0],
        )
        .unwrap();
        assert_eq!(col, [0.0, 1.0, 0.0]);
    }

    #[test]
    fn max_lambda_is_min_ratio_not_an_inverse() {
        let cols = vec![[1.0, 0.0, 0.0], [0.0, 0.5, 0.0], [0.0, 0.0, 2.0]];
        let tau = [2.0, 10.0, 10.0];
        let bound = max_force_along_direction(&cols, &tau, [1.0, 0.0, 0.0]).unwrap();
        let expected = tau[0] / 1.0;
        assert!((bound.lambda_abs_max - expected).abs() < 1e-12);
        assert_eq!(bound.limiting_index, 0);
    }

    #[test]
    fn halving_limiting_effort_strictly_reduces_lambda() {
        let cols = vec![[0.0, 0.3, 0.0], [0.0, 0.15, 0.0]];
        let tau = [1.2, 1.2];
        let d = [0.0, 1.0, 0.0];
        let a = max_force_along_direction(&cols, &tau, d).unwrap();
        let mut tau_half = tau;
        tau_half[a.limiting_index] *= 0.5;
        let b = max_force_along_direction(&cols, &tau_half, d).unwrap();
        assert_eq!(b.limiting_index, a.limiting_index);
        assert!(b.lambda_abs_max < a.lambda_abs_max - 1e-12);
        assert!((b.lambda_abs_max * 2.0 - a.lambda_abs_max).abs() < 1e-12);
    }

    #[test]
    fn uncoupled_direction_is_singular_not_a_huge_force() {
        let cols = vec![[0.0, 0.4, 0.0], [0.0, 0.2, 0.0]];
        let tau = [5.0, 5.0];
        let err = max_force_along_direction(&cols, &tau, [1.0, 0.0, 0.0]).unwrap_err();
        assert_eq!(err, PhysicsError::Singular("direction_uncoupled"));
    }

    #[test]
    fn negative_mu_is_refused() {
        assert!(coulomb_initiation_force_n(-0.1, 1.0).is_err());
        assert!(friction_cone_membership([1.0, 0.0, 0.0], [1.0, 0.0, 0.0], -0.2).is_err());
    }
}
