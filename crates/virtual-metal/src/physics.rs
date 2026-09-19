//! Envelope-respecting XL330 surrogate. Not exact unmeasured fidelity.

use crate::truth_pack::Xl330TruthPack;

#[derive(Debug, Clone)]
pub struct PhysicsState {
    pub theta_rad: f64,
    pub omega_rad_s: f64,
    pub voltage_v: f64,
    pub temperature_c: f64,
    pub load_torque_nm: f64,
}

impl Default for PhysicsState {
    fn default() -> Self {
        Self {
            theta_rad: 0.0,
            omega_rad_s: 0.0,
            voltage_v: 5.0,
            temperature_c: 25.0,
            load_torque_nm: 0.0,
        }
    }
}

fn lerp(x0: f64, y0: f64, x1: f64, y1: f64, x: f64) -> f64 {
    if (x1 - x0).abs() < f64::EPSILON {
        return y0;
    }
    y0 + (y1 - y0) * (x - x0) / (x1 - x0)
}

/// Piecewise-linear manufacturer envelope. Not a calibrated motor model.
pub fn stall_torque_nm_at(pack: &Xl330TruthPack, voltage_v: f64) -> Option<f64> {
    let t37 = pack.stall_torque_3v7_nm.value?;
    let t50 = pack.stall_torque_5v_nm.value?;
    let t60 = pack.stall_torque_6v_nm.value?;
    Some(if voltage_v <= 5.0 {
        lerp(3.7, t37, 5.0, t50, voltage_v.clamp(3.7, 5.0))
    } else {
        lerp(5.0, t50, 6.0, t60, voltage_v.clamp(5.0, 6.0))
    })
}

pub fn no_load_speed_rpm_at(pack: &Xl330TruthPack, voltage_v: f64) -> Option<f64> {
    let n37 = pack.no_load_speed_3v7_rpm.value?;
    let n50 = pack.no_load_speed_5v_rpm.value?;
    let n60 = pack.no_load_speed_6v_rpm.value?;
    Some(if voltage_v <= 5.0 {
        lerp(3.7, n37, 5.0, n50, voltage_v.clamp(3.7, 5.0))
    } else {
        lerp(5.0, n50, 6.0, n60, voltage_v.clamp(5.0, 6.0))
    })
}

pub fn ticks_to_rad(ticks: i32, pulses: u16) -> f64 {
    (ticks as f64) * std::f64::consts::TAU / f64::from(pulses)
}

pub fn rad_to_ticks(theta: f64, pulses: u16) -> i32 {
    let t = theta / std::f64::consts::TAU * f64::from(pulses);
    t.round() as i32
}

impl PhysicsState {
    pub fn from_ticks(ticks: i32, pulses: u16, voltage_v: f64) -> Self {
        Self {
            theta_rad: ticks_to_rad(ticks, pulses),
            omega_rad_s: 0.0,
            voltage_v,
            temperature_c: 25.0,
            load_torque_nm: 0.0,
        }
    }

    pub fn present_ticks(&self, pulses: u16) -> i32 {
        rad_to_ticks(self.theta_rad, pulses)
    }
}
