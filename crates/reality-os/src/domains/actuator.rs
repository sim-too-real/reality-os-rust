use crate::certificate::Certificate;
use crate::domains::{DomainPlugin, WorldView};
use crate::plan::{Intent, PhysicalPlan};
use realityos_kernel::DecisionStatus;

/// Torque/effort vs declared envelope. Screen, not MEASURED PFL.
pub struct ActuatorEnvelope;

impl DomainPlugin for ActuatorEnvelope {
    fn kind(&self) -> &'static str {
        "actuator_envelope"
    }

    fn plan(&self, _intent: &Intent, world: &WorldView) -> PhysicalPlan {
        let action = if world.tau_max.is_empty() {
            vec![0.0]
        } else {
            world.tau_max.iter().map(|t| t * 0.5).collect()
        };
        PhysicalPlan::new("actuator_envelope", action)
    }

    fn certify(&self, plan: &PhysicalPlan, world: &WorldView) -> Certificate {
        if !plan.finite() {
            return Certificate::new(DecisionStatus::Abort, "non_finite_plan_action")
                .with_reasons(["non_finite_plan_action"]);
        }
        if world.tau_max.is_empty() {
            return Certificate::new(DecisionStatus::Refuse, "actuator_undeclared")
                .with_reasons(["actuator_undeclared"]);
        }
        for (i, a) in plan.action.iter().enumerate() {
            let lim = world.tau_max[i.min(world.tau_max.len() - 1)].abs();
            if a.abs() > lim + 1e-12 {
                return Certificate::new(
                    DecisionStatus::Refuse,
                    format!("actuator_envelope_exceeded dim {i}: {} > {lim}", a.abs()),
                )
                .with_reasons(["actuator_envelope"])
                .with_margin(lim - a.abs());
            }
        }
        let peak = plan.action.iter().fold(0.0_f64, |a, b| a.max(b.abs()));
        let lim = world.tau_max.iter().fold(0.0_f64, |a, b| a.max(b.abs()));
        Certificate::new(DecisionStatus::Allow, "within actuator envelope").with_margin(lim - peak)
    }
}
