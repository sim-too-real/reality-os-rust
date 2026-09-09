//! Domain plan/certify plugins. Growth path is register, not an if-ladder.
//! Physics formulas stay screens. Unknown kind refuses.

mod actuator;
mod energy;
mod locomotion;
mod motor;
mod pfl;
mod stop;
mod workspace;

use std::collections::BTreeMap;

use crate::certificate::Certificate;
use crate::plan::{Intent, PhysicalPlan};
use realityos_kernel::DecisionStatus;

pub use actuator::ActuatorEnvelope;
pub use energy::EnergyEnvelope;
pub use motor::MotorTorque;
pub use pfl::{BodyRegion, PflScreen};
pub use stop::StopDistance;
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
    pub observation: Option<realityos_kernel::ObservationEvidence>,
    pub pose_std_m: Option<f64>,
    pub mass_kg: Option<f64>,
    pub speed_m_s: Option<f64>,
    pub decel_m_s2: Option<f64>,
    pub max_stop_m: Option<f64>,
    pub ke_limit_j: Option<f64>,
    pub omega_rad_s: Option<f64>,
    pub inertia_kg_m2: Option<f64>,
    pub kt_nm_per_a: Option<f64>,
    pub current_a: Option<f64>,
    pub gear_ratio: Option<f64>,
    pub motor_eta: Option<f64>,
    pub capabilities: Vec<String>,
    pub mu: Option<f64>,
    pub g_m_s2: Option<f64>,
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
        r.register(Box::new(StopDistance));
        r.register(Box::new(EnergyEnvelope));
        r.register(Box::new(MotorTorque));
        r.register(Box::new(locomotion::LocomotionScreen));
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
            "stop" | "halt" | "estop_distance" => "stop_distance",
            "energy" | "ke" => "energy_envelope",
            "motor" | "current" => "motor_torque",
            "walk" | "stand" => "locomotion",
            _ => return None,
        };
        self.plugins.contains_key(mapped).then_some(mapped)
    }
}
