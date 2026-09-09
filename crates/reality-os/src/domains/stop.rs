//! Stop-distance screen: s = v² / (2 a). SIM envelope, not ISO 13850.

use crate::certificate::Certificate;
use crate::domains::{DomainPlugin, WorldView};
use crate::plan::{Intent, PhysicalPlan};
use realityos_kernel::DecisionStatus;
use realityos_physics::{stop_distance_m, G0};

pub struct StopDistance;

impl DomainPlugin for StopDistance {
    fn kind(&self) -> &'static str {
        "stop_distance"
    }

    fn plan(&self, _intent: &Intent, world: &WorldView) -> PhysicalPlan {
        PhysicalPlan::new("stop_distance", vec![world.speed_m_s.unwrap_or(0.0)])
    }

    fn certify(&self, plan: &PhysicalPlan, world: &WorldView) -> Certificate {
        let v = plan.action.first().copied().unwrap_or(0.0);
        let declared = world.decel_m_s2.unwrap_or(G0);
        let a = match (world.mu, world.g_m_s2.or(Some(G0))) {
            (Some(mu), Some(g)) => match realityos_physics::coulomb_decel_m_s2(mu, g) {
                Ok(ceil) => declared.min(ceil),
                Err(e) => {
                    return Certificate::new(DecisionStatus::Abort, e.to_string())
                        .with_reasons(["non_finite"]);
                }
            },
            _ => declared,
        };
        let s = match stop_distance_m(v, a) {
            Ok(s) => s,
            Err(e) => {
                return Certificate::new(DecisionStatus::Abort, e.to_string())
                    .with_reasons(["non_finite"]);
            }
        };
        let limit = match world.max_stop_m {
            Some(l) => l,
            None => {
                return Certificate::new(DecisionStatus::Refuse, "stop_envelope_undeclared")
                    .with_reasons(["physics_bound"]);
            }
        };
        if s > limit + 1e-12 {
            return Certificate::new(
                DecisionStatus::Refuse,
                format!("stop_distance {s} m > envelope {limit} m (v²/2a, SIM)"),
            )
            .with_reasons(["physics_bound"])
            .with_margin(limit - s);
        }
        Certificate::new(DecisionStatus::Allow, "stop distance inside envelope")
            .with_margin(limit - s)
    }
}
