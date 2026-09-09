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

    fn plan(&self, intent: &Intent, world: &WorldView) -> PhysicalPlan {
        if intent.verb == "hold" && !world.tau_max.is_empty() {
            return PhysicalPlan::hold("actuator_envelope", world.tau_max.len());
        }
        PhysicalPlan::new("actuator_envelope", vec![])
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
        if plan.action.is_empty() {
            return Certificate::new(DecisionStatus::Refuse, "plan_lacks_explicit_target")
                .with_reasons(["plan_lacks_explicit_target"]);
        }
        if plan.action.len() != world.tau_max.len() {
            return Certificate::new(DecisionStatus::Refuse, "actuator_dimension_mismatch")
                .with_reasons(["dimension_mismatch"]);
        }
        for (i, a) in plan.action.iter().enumerate() {
            let lim = world.tau_max[i].abs();
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
