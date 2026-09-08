//! Shared kernel for Reality OS + Governor.
//!
//! No plant I/O. No invent. No motor write. Types here are the honesty floor.

pub mod clock;
pub mod decision;
pub mod error;
pub mod evidence;
pub mod honesty;
pub mod ids;
pub mod rail;

pub use clock::{finite_or_err, unix_now_s};
pub use decision::{DecisionStatus, UnifiedDecision};
pub use error::{KernelError, KernelResult};
pub use evidence::{
    EvidenceTier, BOUNDED_TRUST_EVIDENCE, GATE_EVIDENCE, KERNEL_EVIDENCE, LEDGER_EVIDENCE,
    PFL_EVIDENCE,
};
pub use honesty::HonestyStamp;
pub use ids::{
    CalibrationId, CommandId, DesignContentHash, FirmwareId, ReleaseHash, SerialOrAsBuilt,
    SessionId,
};
pub use rail::{to_rail, to_rail_word, RailStatus};

/// Schema family. Bump when a public type changes meaning.
pub const SCHEMA_FAMILY: &str = "realityos.kernel/1";
