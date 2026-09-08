use crate::certificate::Certificate;
use crate::domains::{DomainPlugin, WorldView};
use crate::plan::{Intent, PhysicalPlan};
use realityos_kernel::DecisionStatus;
use realityos_physics::joint_limit_margin;

pub struct WorkspaceBoundary;

impl DomainPlugin for WorkspaceBoundary {
    fn kind(&self) -> &'static str {
        "workspace_boundary"
    }

    fn plan(&self, _intent: &Intent, world: &WorldView) -> PhysicalPlan {
        PhysicalPlan::new(
            "workspace_boundary",
            if world.q.is_empty() {
                vec![0.0]
            } else {
                world.q.clone()
            },
        )
    }

    fn certify(&self, plan: &PhysicalPlan, world: &WorldView) -> Certificate {
        if world.q_min.is_empty() || world.q_max.is_empty() {
            return Certificate::new(DecisionStatus::Refuse, "workspace_undeclared")
                .with_reasons(["workspace_undeclared"]);
        }
        for (i, a) in plan.action.iter().enumerate() {
            let lo = world.q_min[i.min(world.q_min.len() - 1)];
            let hi = world.q_max[i.min(world.q_max.len() - 1)];
            match joint_limit_margin(*a, lo, hi) {
                Ok(m) if m >= -1e-12 => {}
                Ok(m) => {
                    return Certificate::new(
                        DecisionStatus::Refuse,
                        format!("workspace_boundary dim {i}: {a} not in [{lo},{hi}] margin={m}"),
                    )
                    .with_reasons(["workspace_boundary"]);
                }
                Err(e) => {
                    return Certificate::new(DecisionStatus::Abort, e.to_string())
                        .with_reasons(["non_finite"]);
                }
            }
        }
        Certificate::new(DecisionStatus::Allow, "inside workspace")
    }
}
