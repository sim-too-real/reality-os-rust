//! Energy and power. Gate uses these as envelope bounds, not MEASURED dyno.

use crate::error::{finite, nonneg, PhysicsResult};

/// Translational KE = ½ m v²
pub fn translational_ke_j(mass_kg: f64, speed_m_s: f64) -> PhysicsResult<f64> {
    let m = nonneg(mass_kg, "mass")?;
    let v = finite(speed_m_s, "speed")?;
    Ok(0.5 * m * v * v)
}

/// Rotational KE = ½ I ω²
pub fn rotational_ke_j(inertia_kg_m2: f64, omega_rad_s: f64) -> PhysicsResult<f64> {
    let i = nonneg(inertia_kg_m2, "inertia")?;
    let w = finite(omega_rad_s, "omega")?;
    Ok(0.5 * i * w * w)
}

/// Mechanical power P = τ ω = F v
pub fn mechanical_power_w(effort: f64, rate: f64) -> PhysicsResult<f64> {
    Ok(finite(effort, "effort")? * finite(rate, "rate")?)
}

/// Transient contact energy for PFL screen: ½ m v_close²
pub fn contact_energy_j(mass_kg: f64, closing_speed_m_s: f64) -> PhysicsResult<f64> {
    translational_ke_j(mass_kg, closing_speed_m_s)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ke_scales_v_squared() {
        let a = translational_ke_j(2.0, 3.0).unwrap();
        let b = translational_ke_j(2.0, 6.0).unwrap();
        assert!((b / a - 4.0).abs() < 1e-12);
    }
}
