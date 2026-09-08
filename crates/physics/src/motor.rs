//! Steady motor screen: τ = η N Kt I, P_j = I² R. Not a datasheet MEASURED.

use crate::error::{finite, nonneg, positive, PhysicsResult};

/// τ = η · N · Kt · I  (Kt in N·m/A, N gear ratio, η in (0,1])
pub fn motor_torque_nm(
    kt_nm_per_a: f64,
    current_a: f64,
    efficiency: f64,
    gear_ratio: f64,
) -> PhysicsResult<f64> {
    let kt = positive(kt_nm_per_a, "kt")?;
    let i = finite(current_a, "current")?;
    let eta = positive(efficiency, "efficiency")?;
    let n = positive(gear_ratio, "gear_ratio")?;
    if eta > 1.0 {
        return Err(crate::error::PhysicsError::NonPositive("efficiency_gt_1"));
    }
    Ok(eta * n * kt * i)
}

/// Joule heating P = I² R
pub fn joule_w(current_a: f64, resistance_ohm: f64) -> PhysicsResult<f64> {
    let i = finite(current_a, "current")?;
    let r = nonneg(resistance_ohm, "r")?;
    Ok(i * i * r)
}

/// Linear back-EMF V = I R + Ke ω  (SI Ke ≈ Kt)
pub fn terminal_voltage_v(
    current_a: f64,
    resistance_ohm: f64,
    ke_v_s_per_rad: f64,
    omega_rad_s: f64,
) -> PhysicsResult<f64> {
    Ok(finite(current_a, "current")? * nonneg(resistance_ohm, "r")?
        + finite(ke_v_s_per_rad, "ke")? * finite(omega_rad_s, "omega")?)
}

/// Derate torque linearly above T_rated. Clamp at 0. SIM screen.
pub fn thermal_derate(
    tau_max: f64,
    temp_c: f64,
    t_rated_c: f64,
    k_per_c: f64,
) -> PhysicsResult<f64> {
    let tau = positive(tau_max, "tau_max")?;
    let t = finite(temp_c, "temp")?;
    let tr = finite(t_rated_c, "t_rated")?;
    let k = nonneg(k_per_c, "k")?;
    let scale = (1.0 - k * (t - tr)).max(0.0);
    Ok(tau * scale)
}
