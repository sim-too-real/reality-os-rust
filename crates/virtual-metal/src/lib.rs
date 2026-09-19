//! Virtual Metal V1 for XL330. Simulation surrogate. Not MEASURED metal.

pub mod campaign;
pub mod device;
pub mod evidence;
pub mod faults;
pub mod physics;
pub mod port;
pub mod truth_pack;

pub use campaign::{decide_hold, decide_nudge, run_campaign, start_gov, start_gov_opts};
pub use device::VirtualXl330;
pub use evidence::{
    honesty, refuse_hardware_present_true, refuse_measured_token, refuse_metal_proof_install,
    CampaignRecord, CAMPAIGN_SCHEMA, EVIDENCE_STATUS, VERDICT_PASS,
};
pub use faults::{FaultKind, FaultSchedule};
pub use port::VirtualMetalPort;
pub use truth_pack::{Provenance, Provenanced, Xl330TruthPack, TRUTH_PACK_SCHEMA};

pub const SCHEMA: &str = CAMPAIGN_SCHEMA;
