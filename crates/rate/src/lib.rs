//! Multi-rate control. Screen ~50 Hz proposes; dispose 1 kHz certifies.
//! Budgets are recorded SIM values, not lab-measured latency.

use realityos_core::{certify_dispose_step, BoundedTrustEnvelope, DisposeStatus, ExecutionMode};
use realityos_physics::{period_s, DISPOSE_HZ, SCREEN_HZ};
use serde::{Deserialize, Serialize};

pub const BUDGET_FASTPATH_US: f64 = 15.0;
pub const BUDGET_QP_US: f64 = 250.0;
pub const BUDGET_PASSIVE_US: f64 = 5.0;
pub const BUDGET_TICK_US: f64 = 1000.0;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RateBand {
    pub name: String,
    pub hz: f64,
}

impl RateBand {
    pub fn period_s(&self) -> f64 {
        period_s(self.hz).unwrap_or(1.0)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MultiRateSpec {
    pub screen: RateBand,
    pub dispose: RateBand,
    pub telemetry: RateBand,
}

impl Default for MultiRateSpec {
    fn default() -> Self {
        Self {
            screen: RateBand {
                name: "screen".into(),
                hz: SCREEN_HZ,
            },
            dispose: RateBand {
                name: "dispose".into(),
                hz: DISPOSE_HZ,
            },
            telemetry: RateBand {
                name: "telemetry".into(),
                hz: 100.0,
            },
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TickRecord {
    pub i: u32,
    pub t_s: f64,
    pub hz: f64,
    pub mode: String,
    pub deadline_miss: bool,
    pub work_us: f64,
    pub budget_us: f64,
    pub metal: bool,
}

/// Autoware Safety Island skips the control tick on missing input.
/// This gate does the opposite: missing sensor this tick → PassiveFallback.
pub fn missing_input_forces_passive(sensor_this_tick: bool) -> bool {
    !sensor_this_tick
}

pub fn certify_tick(
    u_nom: &[f64],
    dq: &[f64],
    envelope: &BoundedTrustEnvelope,
    hold: bool,
    sensor_this_tick: bool,
) -> DisposeStatus {
    if missing_input_forces_passive(sensor_this_tick) {
        return certify_dispose_step(u_nom, dq, envelope, true);
    }
    certify_dispose_step(u_nom, dq, envelope, hold)
}

pub fn budget_us(mode: ExecutionMode) -> f64 {
    match mode {
        ExecutionMode::TrustedFastpath => BUDGET_FASTPATH_US,
        ExecutionMode::QpInterception => BUDGET_QP_US,
        ExecutionMode::PassiveFallback => BUDGET_PASSIVE_US,
    }
}

/// Run `n` dispose ticks. `work_us` is injected compute (tests), not a wall clock.
pub fn run_dispose_ticks(
    n: u32,
    hz: f64,
    u_nom: &[f64],
    dq: &[f64],
    envelope: &BoundedTrustEnvelope,
    hold: bool,
    work_us: f64,
) -> Vec<TickRecord> {
    let dt = period_s(hz).unwrap_or(0.001);
    (0..n)
        .map(|i| {
            let st: DisposeStatus = certify_dispose_step(u_nom, dq, envelope, hold);
            let budget = budget_us(st.mode).min(BUDGET_TICK_US);
            TickRecord {
                i,
                t_s: f64::from(i) * dt,
                hz,
                mode: st.mode.as_str().into(),
                deadline_miss: work_us > budget,
                work_us,
                budget_us: budget,
                metal: false,
            }
        })
        .collect()
}

pub fn bands() -> [RateBand; 6] {
    [
        RateBand {
            name: "vision".into(),
            hz: 30.0,
        },
        RateBand {
            name: "screen".into(),
            hz: 50.0,
        },
        RateBand {
            name: "telemetry".into(),
            hz: 100.0,
        },
        RateBand {
            name: "governor_legacy".into(),
            hz: 120.0,
        },
        RateBand {
            name: "control".into(),
            hz: 500.0,
        },
        RateBand {
            name: "dispose".into(),
            hz: 1000.0,
        },
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn one_khz_deadline_miss_when_work_exceeds_budget() {
        let env = BoundedTrustEnvelope {
            tau_max: vec![1.0],
            dq_max: vec![10.0],
            is_valid: true,
        };
        let ticks = run_dispose_ticks(8, 1000.0, &[0.1], &[0.0], &env, false, 20.0);
        assert!(ticks.iter().all(|t| t.deadline_miss));
        assert!(ticks.iter().all(|t| t.mode == "TRUSTED_FASTPATH"));
        let ok = run_dispose_ticks(8, 1000.0, &[0.1], &[0.0], &env, false, 5.0);
        assert!(ok.iter().all(|t| !t.deadline_miss));
    }

    #[test]
    fn missing_sensor_this_tick_is_passive_not_skip() {
        let env = BoundedTrustEnvelope {
            tau_max: vec![1.0],
            dq_max: vec![10.0],
            is_valid: true,
        };
        let st = certify_tick(&[0.2], &[0.0], &env, false, false);
        assert_eq!(st.mode, ExecutionMode::PassiveFallback);
        let st2 = certify_tick(&[0.2], &[0.0], &env, false, true);
        assert_eq!(st2.mode, ExecutionMode::TrustedFastpath);
    }
}
