//! Kinetic-energy envelope. ½ m v² / ½ I ω² vs declared limit.

use crate::certificate::Certificate;
use crate::domains::{DomainPlugin, WorldView};
use crate::plan::{Intent, PhysicalPlan};
use realityos_kernel::DecisionStatus;
use realityos_physics::{rotational_ke_j, translational_ke_j};

pub struct EnergyEnvelope;

impl DomainPlugin for EnergyEnvelope {
    fn kind(&self) -> &'static str {
        "energy_envelope"
    }

    fn plan(&self, _intent: &Intent, world: &WorldView) -> PhysicalPlan {
        PhysicalPlan::new(
            "energy_envelope",
            vec![world.speed_m_s.or(world.omega_rad_s).unwrap_or(0.0)],
        )
    }

    fn certify(&self, plan: &PhysicalPlan, world: &WorldView) -> Certificate {
        let Some(limit) = world.ke_limit_j else {
            return Certificate::new(DecisionStatus::Refuse, "ke_limit_undeclared")
                .with_reasons(["physics_bound"]);
        };
        let ke = if let Some(m) = world.mass_kg {
            let v = plan.action.first().copied().unwrap_or(0.0);
            translational_ke_j(m, v)
        } else if let Some(i) = world.inertia_kg_m2 {
            let w = plan.action.first().copied().unwrap_or(0.0);
            rotational_ke_j(i, w)
        } else {
            return Certificate::new(DecisionStatus::Refuse, "mass_or_inertia_undeclared")
                .with_reasons(["physics_bound"]);
        };
        let ke = match ke {
            Ok(v) => v,
            Err(e) => {
                return Certificate::new(DecisionStatus::Abort, e.to_string())
                    .with_reasons(["non_finite"]);
            }
        };
        if ke > limit + 1e-12 {
            return Certificate::new(
                DecisionStatus::Refuse,
                format!("KE {ke} J exceeds envelope {limit} J"),
            )
            .with_reasons(["physics_bound"])
            .with_margin(limit - ke);
        }
        Certificate::new(DecisionStatus::Allow, "kinetic energy inside envelope")
            .with_margin(limit - ke)
    }
}
