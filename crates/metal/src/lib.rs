//! First low-energy metal smoke. Not a kernel crate. Not HIL.
//!
//! Protocol and proof types compile on every host. The XL330 driver, IPC, and
//! authority process are Unix-only (`nix`, PTY, serial).

pub mod proof;
pub mod protocol;

#[cfg(unix)]
pub mod authority;
#[cfg(unix)]
pub mod config;
#[cfg(unix)]
pub mod egress;
#[cfg(unix)]
pub mod identity;
#[cfg(unix)]
pub mod ipc;
#[cfg(unix)]
pub mod xl330;

#[cfg(unix)]
pub use authority::{load_or_create_key, serve_forever, MetalAuthority};
#[cfg(unix)]
pub use config::MetalConfig;
#[cfg(unix)]
pub use identity::{IdentitySource, MeasuredIdentity};
#[cfg(unix)]
pub use ipc::{call, call_raw, wait_for_ipc, MetalRequest, MetalResponse};
pub use proof::{
    aggregates_from_cases, hold_still, nudge_moved, present_position_delta, CaseRecord, MetalProof,
    ProofMeta, HOLD_STILL_MAX_ABS_TICKS, PROOF_SCHEMA,
};
#[cfg(unix)]
pub use xl330::Xl330Driver;
