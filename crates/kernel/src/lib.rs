//! Shared kernel for Reality OS + Governor.
//!
//! No plant I/O. No invent. No motor write. Types here are the honesty floor.

pub mod belief;
pub mod capability;
pub mod clock;
pub mod correlation;
pub mod decision;
pub mod error;
pub mod evidence;
pub mod evidence_contract;
pub mod fault;
pub mod honesty;
pub mod ids;
pub mod layer;
pub mod observation;
pub mod rail;
pub mod time;
pub mod units;
pub mod violation;

pub use belief::{BeliefState, Estimator, SensorHealth};
pub use capability::{capabilities_for_kind, Capability};
pub use clock::{finite_or_err, unix_now_s};
pub use correlation::CorrelationId;
pub use decision::{DecisionStatus, UnifiedDecision};
pub use error::{KernelError, KernelResult};
pub use evidence::{
    EvidenceTier, BOUNDED_TRUST_EVIDENCE, GATE_EVIDENCE, KERNEL_EVIDENCE, LEDGER_EVIDENCE,
    PFL_EVIDENCE,
};
pub use evidence_contract::PerceptionContract;
pub use fault::{CommandOutcome, FaultKind};
pub use honesty::HonestyStamp;
pub use ids::{
    CalibrationId, CommandId, DesignContentHash, FirmwareId, ReleaseHash, SerialOrAsBuilt,
    SessionId,
};
pub use layer::Layer;
pub use observation::ObservationEvidence;
pub use rail::{to_rail, to_rail_word, RailStatus};
pub use time::{deadline_from_ttl, MonoTime, SimTime, SyncTime};
pub use units::{
    require_exact_len, require_finite, require_finite_slice, require_positive, FiniteF64,
};
pub use violation::{Violation, ViolationCode};

/// Schema family. Bump when a public type changes meaning.
pub const SCHEMA_FAMILY: &str = "realityos.kernel/1";
