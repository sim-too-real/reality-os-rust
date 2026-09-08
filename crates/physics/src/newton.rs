//! Newton / Euler. F = m a, τ = I α.

use crate::error::{nonneg, PhysicsResult};

pub fn force_n(mass_kg: f64, accel_m_s2: f64) -> PhysicsResult<f64> {
    Ok(nonneg(mass_kg, "mass")? * crate::error::finite(accel_m_s2, "accel")?)
}

pub fn torque_nm(inertia_kg_m2: f64, alpha_rad_s2: f64) -> PhysicsResult<f64> {
    Ok(nonneg(inertia_kg_m2, "inertia")? * crate::error::finite(alpha_rad_s2, "alpha")?)
}

pub fn accel_from_force(force_n: f64, mass_kg: f64) -> PhysicsResult<f64> {
    Ok(crate::error::finite(force_n, "force")? / crate::error::positive(mass_kg, "mass")?)
}
