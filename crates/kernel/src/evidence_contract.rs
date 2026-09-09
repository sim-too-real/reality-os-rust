//! Contract another perception/estimation system must satisfy.
//!
//! This crate does not implement perception, SLAM, calibration, or estimation.
//! Observation evidence is trustworthy for a command only when each stage below
//! is bound by the *acquirer*, not by an untrusted proposer.

/// Stages an external perception/estimation stack must make explicit.
///
/// ```text
/// sensor
///   → acquisition (raw samples owned by the sensor process)
///   → calibration identity
///   → timestamp domain (capture vs receive vs command `now_s`)
///   → transform / frame epoch
///   → processing / model identity
///   → observation evidence (digest recomputed here, never trusted from a string)
///   → world / belief state
///   → physical predicate
///   → command
/// ```
///
/// An untrusted proposer may choose a verb and a requested action. It must not
/// supply the digest, calibration id, frame epoch, or timestamp that the
/// session binds. The session (or a trusted acquirer) recomputes the packet
/// hash from samples + time + frame + sensor id + sequence.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PerceptionContract;

impl PerceptionContract {
    /// Caller-provided hashes are not a security boundary.
    pub const fn proposer_may_supply_digest() -> bool {
        false
    }

    /// This kernel does not promote SIM evidence to MEASURED.
    pub const fn can_claim_measured() -> bool {
        false
    }
}
