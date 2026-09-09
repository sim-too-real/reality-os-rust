//! First low-energy metal smoke. Not a kernel crate. Not HIL.

pub mod authority;
pub mod config;
pub mod egress;
pub mod identity;
pub mod ipc;
pub mod proof;
pub mod protocol;
pub mod xl330;

pub use authority::{load_or_create_key, serve_forever, MetalAuthority};
pub use config::MetalConfig;
pub use identity::{IdentitySource, MeasuredIdentity};
pub use ipc::{call, call_raw, wait_for_ipc, MetalRequest, MetalResponse};
pub use proof::{aggregates_from_cases, CaseRecord, MetalProof, ProofMeta, PROOF_SCHEMA};
pub use xl330::Xl330Driver;
