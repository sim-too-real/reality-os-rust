//! Envelope-respecting XL330 surrogate. Not exact unmeasured fidelity.
//!
//! Goal writes store a target. Present evolves only through [`PhysicsState::advance`].

use crate::truth_pack::Xl330TruthPack;

/// Manufacturer no-load speed unit: rev/min → rad/s.
const RPM_TO_RAD_S: f64 = std::f64::consts::TAU / 60.0;

#[derive(Debug, Clone)]
pub struct PhysicsState {
    pub theta_rad: f64,
    pub omega_rad_s: f64,
    pub target_rad: f64,
    pub voltage_v: f64,
    pub temperature_c: f64,
    pub load_torque_nm: f64,
    /// 1.0 means "use manufacturer envelope as published" (output-side).
    /// Campaign realizations may sample the estimated gearbox-efficiency range
    /// and scale the envelope; that is a search parameter, not a calibrated Kt.
    pub gearbox_efficiency: f64,
}

impl Default for PhysicsState {
    fn default() -> Self {
        Self {
            theta_rad: 0.0,
            omega_rad_s: 0.0,
            target_rad: 0.0,
            voltage_v: 5.0,
            temperature_c: 25.0,
            load_torque_nm: 0.0,
            gearbox_efficiency: 1.0,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct MotionLimits {
    pub torque_on: bool,
    pub pwm_frac: f64,
    pub current_frac: f64,
    pub pos_min_rad: f64,
    pub pos_max_rad: f64,
    pub profile_velocity_rad_s: f64,
    pub profile_accel_rad_s2: f64,
}

impl Default for MotionLimits {
    fn default() -> Self {
        Self {
            torque_on: true,
            pwm_frac: 1.0,
            current_frac: 1.0,
            pos_min_rad: 0.0,
            pos_max_rad: std::f64::consts::TAU * 4095.0 / 4096.0,
            profile_velocity_rad_s: 0.0,
            profile_accel_rad_s2: 0.0,
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

pub fn rpm_to_rad_s(rpm: f64) -> f64 {
    rpm * RPM_TO_RAD_S
}

impl PhysicsState {
    pub fn from_ticks(ticks: i32, pulses: u16, voltage_v: f64) -> Self {
        let theta = ticks_to_rad(ticks, pulses);
        Self {
            theta_rad: theta,
            omega_rad_s: 0.0,
            target_rad: theta,
            voltage_v,
            temperature_c: 25.0,
            load_torque_nm: 0.0,
            gearbox_efficiency: 1.0,
        }
    }

    pub fn present_ticks(&self, pulses: u16) -> i32 {
        rad_to_ticks(self.theta_rad, pulses)
    }

    pub fn target_ticks(&self, pulses: u16) -> i32 {
        rad_to_ticks(self.target_rad, pulses)
    }

    /// Store a goal. Does not teleport present.
    pub fn set_target_ticks(&mut self, ticks: i32, pulses: u16) {
        self.target_rad = ticks_to_rad(ticks, pulses);
    }

    /// Evolve present toward the stored target under published envelopes.
    ///
    /// Never exceeds no-load speed (voltage-interpolated, PWM/current scaled,
    /// gearbox-efficiency scaled), stall-torque vs load, or position limits.
    pub fn advance(&mut self, dt: f64, pack: &Xl330TruthPack, limits: &MotionLimits) {
        let dt = dt.max(0.0);
        if dt == 0.0 {
            return;
        }
        if !limits.torque_on {
            self.omega_rad_s = 0.0;
            return;
        }
        let pwm = limits.pwm_frac.clamp(0.0, 1.0);
        let current = limits.current_frac.clamp(0.0, 1.0);
        let drive = (pwm * current).clamp(0.0, 1.0);
        if drive <= 0.0 {
            self.omega_rad_s = 0.0;
            return;
        }
        let eff = self.gearbox_efficiency.clamp(0.0, 1.0);
        let Some(noload_rpm) = no_load_speed_rpm_at(pack, self.voltage_v) else {
            return;
        };
        let Some(stall_nm) = stall_torque_nm_at(pack, self.voltage_v) else {
            return;
        };
        let stall_eff = stall_nm * eff * drive;
        if stall_eff + f64::EPSILON < self.load_torque_nm.abs() {
            self.omega_rad_s = 0.0;
            return;
        }
        let mut vmax = rpm_to_rad_s(noload_rpm) * drive * eff;
        if limits.profile_velocity_rad_s > 0.0 {
            vmax = vmax.min(limits.profile_velocity_rad_s);
        }
        if vmax <= 0.0 {
            self.omega_rad_s = 0.0;
            return;
        }
        let err = self.target_rad - self.theta_rad;
        if err.abs() < 1e-12 {
            self.theta_rad = self.target_rad;
            self.omega_rad_s = 0.0;
            self.clamp_position(limits);
            return;
        }
        let dir = err.signum();
        let mut omega = dir * vmax;
        if limits.profile_accel_rad_s2 > 0.0 {
            let max_step = limits.profile_accel_rad_s2 * dt;
            let desired = omega;
            let delta = (desired - self.omega_rad_s).clamp(-max_step, max_step);
            omega = self.omega_rad_s + delta;
            if omega.abs() > vmax {
                omega = vmax.copysign(omega);
            }
        }
        let step = omega * dt;
        if step.abs() >= err.abs() {
            self.theta_rad = self.target_rad;
            self.omega_rad_s = 0.0;
        } else {
            self.theta_rad += step;
            self.omega_rad_s = omega;
        }
        self.clamp_position(limits);
    }

    fn clamp_position(&mut self, limits: &MotionLimits) {
        let lo = limits.pos_min_rad.min(limits.pos_max_rad);
        let hi = limits.pos_min_rad.max(limits.pos_max_rad);
        if self.theta_rad < lo {
            self.theta_rad = lo;
            self.omega_rad_s = 0.0;
        } else if self.theta_rad > hi {
            self.theta_rad = hi;
            self.omega_rad_s = 0.0;
        }
    }
}
