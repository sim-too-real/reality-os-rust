//! Domain plan/certify plugins. Growth path is register, not an if-ladder.
//! Physics formulas stay screens. Unknown kind refuses.

mod actuator;
mod pfl;
mod workspace;

use std::collections::BTreeMap;

use crate::certificate::Certificate;
use crate::plan::{Intent, PhysicalPlan};
use realityos_kernel::DecisionStatus;

pub use actuator::ActuatorEnvelope;
pub use pfl::{BodyRegion, PflScreen};
pub use workspace::WorkspaceBoundary;

pub trait DomainPlugin: Send + Sync {
    fn kind(&self) -> &'static str;
    fn plan(&self, intent: &Intent, world: &WorldView) -> PhysicalPlan;
    fn certify(&self, plan: &PhysicalPlan, world: &WorldView) -> Certificate;
}

#[derive(Debug, Clone, Default)]
pub struct WorldView {
    pub tau_max: Vec<f64>,
    pub q: Vec<f64>,
    pub q_min: Vec<f64>,
    pub q_max: Vec<f64>,
    pub contact_force_n: Option<f64>,
    pub body_region: Option<BodyRegion>,
    pub pixels_present: bool,
    pub scene_compiled: bool,
    pub pose_std_m: Option<f64>,
}

#[derive(Default)]
pub struct DomainRegistry {
    plugins: BTreeMap<&'static str, Box<dyn DomainPlugin>>,
}

impl DomainRegistry {
    pub fn product_defaults() -> Self {
        let mut r = Self::default();
        r.register(Box::new(ActuatorEnvelope));
        r.register(Box::new(WorkspaceBoundary));
        r.register(Box::new(PflScreen::sim_defaults()));
        r
    }

    pub fn register(&mut self, plugin: Box<dyn DomainPlugin>) {
        self.plugins.insert(plugin.kind(), plugin);
    }

    pub fn kinds(&self) -> Vec<&'static str> {
        self.plugins.keys().copied().collect()
    }

    pub fn plan(&self, kind: &str, intent: &Intent, world: &WorldView) -> Option<PhysicalPlan> {
        self.plugins.get(kind).map(|p| p.plan(intent, world))
    }

    pub fn certify(&self, plan: &PhysicalPlan, world: &WorldView) -> Certificate {
        match self.plugins.get(plan.kind.as_str()) {
            Some(p) => p.certify(plan, world),
            None => Certificate::new(
                DecisionStatus::Refuse,
                format!("no physical certifier registered for {}", plan.kind),
            )
            .with_reasons(["unsupported_plan_kind"]),
        }
    }

    pub fn infer_kind(&self, intent: &Intent) -> Option<&'static str> {
        let v = intent.verb.as_str();
        if let Some(k) = self.plugins.keys().copied().find(|k| *k == v) {
            return Some(k);
        }
        let mapped = match v {
            "drive" | "torque" | "hold" => "actuator_envelope",
            "move" | "reach" => "workspace_boundary",
            "contact" | "push" | "pfl" => "pfl_contact",
            "place" | "pick" | "precision_place" | "grasp" | "insert" => "workspace_boundary",
            _ => "actuator_envelope",
        };
        self.plugins.contains_key(mapped).then_some(mapped)
    }
}
