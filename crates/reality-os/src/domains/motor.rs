//! Motor torque screen τ = η N Kt I vs τ_max. Not MEASURED Kt.

use crate::certificate::Certificate;
use crate::domains::{DomainPlugin, WorldView};
use crate::plan::{Intent, PhysicalPlan};
use realityos_kernel::DecisionStatus;
use realityos_physics::motor_torque_nm;

pub struct MotorTorque;

impl DomainPlugin for MotorTorque {
    fn kind(&self) -> &'static str {
        "motor_torque"
    }

    fn plan(&self, _intent: &Intent, world: &WorldView) -> PhysicalPlan {
        PhysicalPlan::new("motor_torque", vec![world.current_a.unwrap_or(0.0)])
    }

    fn certify(&self, plan: &PhysicalPlan, world: &WorldView) -> Certificate {
        let Some(kt) = world.kt_nm_per_a else {
            return Certificate::new(DecisionStatus::Refuse, "kt_undeclared")
                .with_reasons(["actuator_undeclared"]);
        };
        let i = plan.action.first().copied().unwrap_or(0.0);
        let eta = world.motor_eta.unwrap_or(1.0);
        let n = world.gear_ratio.unwrap_or(1.0);
        let tau = match motor_torque_nm(kt, i, eta, n) {
            Ok(t) => t,
            Err(e) => {
                return Certificate::new(DecisionStatus::Abort, e.to_string())
                    .with_reasons(["non_finite"]);
            }
        };
        let lim = world.tau_max.first().copied().unwrap_or(0.0).abs();
        if lim <= 0.0 {
            return Certificate::new(DecisionStatus::Refuse, "actuator_undeclared")
                .with_reasons(["actuator_undeclared"]);
        }
        if tau.abs() > lim + 1e-12 {
            return Certificate::new(
                DecisionStatus::Refuse,
                format!("motor τ {tau} N·m exceeds {lim} (ηNKtI SIM)"),
            )
            .with_reasons(["actuator_envelope"])
            .with_margin(lim - tau.abs());
        }
        Certificate::new(DecisionStatus::Allow, "motor torque inside envelope")
            .with_margin(lim - tau.abs())
    }
}
