//! Power-and-force-limiting *screen*. Not ISO 10218 / TS 15066 certified.
//! Unknown body region refuses. Limits are caller-configured SIM screens.

use std::collections::BTreeMap;

use crate::certificate::Certificate;
use crate::domains::{DomainPlugin, WorldView};
use crate::plan::{Intent, PhysicalPlan};
use realityos_kernel::DecisionStatus;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BodyRegion {
    Hand,
    Arm,
    Chest,
    Face,
    Unknown,
}

impl BodyRegion {
    pub fn parse(raw: &str) -> Self {
        match raw.trim().to_ascii_lowercase().as_str() {
            "hand" | "finger" | "wrist" => Self::Hand,
            "arm" | "forearm" | "upper_arm" => Self::Arm,
            "chest" | "torso" => Self::Chest,
            "face" | "head" | "skull" => Self::Face,
            _ => Self::Unknown,
        }
    }
}

/// Quasi-static contact force screens (newtons). Not MEASURED. Not a cobot certificate.
#[derive(Debug, Clone)]
pub struct PflScreen {
    limits_n: BTreeMap<BodyRegion, f64>,
}

impl PflScreen {
    pub fn sim_defaults() -> Self {
        // Conservative software screens, labeled SIM. Do not treat as ISO tables.
        let mut limits_n = BTreeMap::new();
        limits_n.insert(BodyRegion::Hand, 140.0);
        limits_n.insert(BodyRegion::Arm, 150.0);
        limits_n.insert(BodyRegion::Chest, 140.0);
        limits_n.insert(BodyRegion::Face, 65.0);
        Self { limits_n }
    }
}

impl DomainPlugin for PflScreen {
    fn kind(&self) -> &'static str {
        "pfl_contact"
    }

    fn plan(&self, _intent: &Intent, world: &WorldView) -> PhysicalPlan {
        PhysicalPlan::new("pfl_contact", vec![world.contact_force_n.unwrap_or(0.0)])
    }

    fn certify(&self, plan: &PhysicalPlan, world: &WorldView) -> Certificate {
        let region = world.body_region.unwrap_or(BodyRegion::Unknown);
        if region == BodyRegion::Unknown {
            return Certificate::new(DecisionStatus::Refuse, "pfl_body_region_unknown")
                .with_reasons(["pfl_unknown_region"]);
        }
        let Some(&lim) = self.limits_n.get(&region) else {
            return Certificate::new(DecisionStatus::Refuse, "pfl_limit_not_configured")
                .with_reasons(["pfl_limit_missing"]);
        };
        let f = plan.action.first().copied().unwrap_or(0.0);
        if !f.is_finite() {
            return Certificate::new(DecisionStatus::Abort, "non_finite_force")
                .with_reasons(["non_finite_plan_action"]);
        }
        if f.abs() > lim + 1e-12 {
            return Certificate::new(
                DecisionStatus::Refuse,
                format!("pfl_force {f} N exceeds SIM screen {lim} N for {region:?}"),
            )
            .with_reasons(["pfl_exceeded"])
            .with_margin(lim - f.abs());
        }
        Certificate::new(
            DecisionStatus::Allow,
            format!("pfl SIM screen {region:?} {f} <= {lim} N (not MEASURED)"),
        )
        .with_margin(lim - f.abs())
    }
}
