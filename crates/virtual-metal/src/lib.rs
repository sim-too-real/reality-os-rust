//! Virtual Metal V1 for XL330. Simulation surrogate. Not MEASURED metal.

pub mod campaign;
pub mod device;
pub mod evidence;
pub mod faults;
pub mod peer;
pub mod physics;
pub mod port;
#[cfg(unix)]
pub mod pty;
pub mod truth_pack;

pub use campaign::{
    decide_hold, decide_nudge, run_campaign, run_tier2_smoke, start_gov, start_gov_opts,
};
pub use device::VirtualXl330;
pub use evidence::{
    honesty, refuse_hardware_present_true, refuse_measured_token, refuse_metal_proof_install,
    CampaignRecord, CAMPAIGN_SCHEMA, EVIDENCE_STATUS, FAULT_MODEL_VERSION,
    SCENARIO_GENERATOR_VERSION, VERDICT_PASS,
};
pub use faults::{
    corrupt_crc_bytes, FaultKind, FaultSchedule, LifecycleLoss, WriteLifecycleBoundary,
};
pub use peer::VirtualSerialPeer;
pub use port::{LifecycleInject, VirtualMetalPort};
pub use truth_pack::{Provenance, Provenanced, Xl330TruthPack, TRUTH_PACK_SCHEMA};

pub const SCHEMA: &str = CAMPAIGN_SCHEMA;

pub fn git_sha() -> String {
    std::process::Command::new("git")
        .args(["rev-parse", "HEAD"])
        .output()
        .ok()
        .and_then(|o| String::from_utf8(o.stdout).ok())
        .map(|s| s.trim().to_string())
        .filter(|s| !s.is_empty())
        .unwrap_or_else(|| "unknown".into())
}
