//! Locomotion is a named hole until a stance/contact certifier exists.
//! Capability (floating_base) is not a walk plan.

use crate::certificate::Certificate;
use crate::domains::{DomainPlugin, WorldView};
use crate::plan::{Intent, PhysicalPlan};
use realityos_kernel::DecisionStatus;

pub struct LocomotionScreen;

impl DomainPlugin for LocomotionScreen {
    fn kind(&self) -> &'static str {
        "locomotion"
    }

    fn plan(&self, _intent: &Intent, _world: &WorldView) -> PhysicalPlan {
        let mut p = PhysicalPlan::new("locomotion", vec![0.0]);
        p.provenance = "locomotion_named_hole".into();
        p.completion = "refuse_until_stance_model".into();
        p
    }

    fn certify(&self, _plan: &PhysicalPlan, _world: &WorldView) -> Certificate {
        Certificate::new(
            DecisionStatus::Refuse,
            "locomotion_not_certified; no stance/contact/balance model",
        )
        .with_reasons(["locomotion_certifier_absent", "named_hole"])
    }
}
