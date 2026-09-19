//! Honesty stamp. Metal / MEASURED / invent_authority cannot be constructed true.

use crate::error::{KernelError, KernelResult};
use serde::{Deserialize, Serialize};

const SIM_PREFIX: &str = "SIM_";

/// Fail-closed honesty block copied onto every product output.
///
/// Fields that would overclaim are private and always false. There is no
/// constructor that can stamp metal, MEASURED, or invent authority.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct HonestyStamp {
    #[serde(skip_deserializing)]
    metal: bool,
    #[serde(skip_deserializing)]
    measured: bool,
    #[serde(skip_deserializing)]
    invent_authority: bool,
    #[serde(skip_deserializing)]
    measured_owned: bool,
    evidence_status: String,
}

impl HonestyStamp {
    /// SIM evidence token. Refuses tokens that do not declare SIM / not-metal.
    pub fn sim(evidence_status: impl Into<String>) -> KernelResult<Self> {
        let evidence_status = evidence_status.into();
        if evidence_status.trim().is_empty() {
            return Err(KernelError::HonestyEvidence("empty".into()));
        }
        if !evidence_status.starts_with(SIM_PREFIX) && !evidence_status.contains("NOT_METAL") {
            return Err(KernelError::HonestyEvidence(evidence_status));
        }
        Ok(Self {
            metal: false,
            measured: false,
            invent_authority: false,
            measured_owned: false,
            evidence_status,
        })
    }

    #[inline]
    pub const fn metal(&self) -> bool {
        false
    }

    #[inline]
    pub const fn measured(&self) -> bool {
        false
    }

    #[inline]
    pub const fn invent_authority(&self) -> bool {
        false
    }

    #[inline]
    pub const fn measured_owned(&self) -> bool {
        false
    }

    #[inline]
    pub fn evidence_status(&self) -> &str {
        &self.evidence_status
    }

    #[inline]
    pub const fn learned_actuator_authority() -> bool {
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sim_stamp_never_claims_metal() {
        let h = HonestyStamp::sim("SIM_GOVERNOR_GATE_NOT_METAL").unwrap();
        assert!(!h.metal());
        assert!(!h.measured());
        assert!(!h.invent_authority());
        assert!(!HonestyStamp::learned_actuator_authority());
    }

    #[test]
    fn measured_token_is_refused() {
        assert!(HonestyStamp::sim("MEASURED_DYNO").is_err());
    }

    #[test]
    fn virtual_metal_token_cannot_claim_measured() {
        assert!(HonestyStamp::sim("VIRTUAL_METAL").is_err());
        assert!(HonestyStamp::sim("VIRTUAL_METAL_PASS").is_err());
        assert!(HonestyStamp::sim("MEASURED_VIRTUAL_METAL").is_err());
        let h = HonestyStamp::sim("SIM_VIRTUAL_METAL_NOT_METAL").unwrap();
        assert!(!h.metal());
        assert!(!h.measured());
        assert!(!h.invent_authority());
        assert!(!h.measured_owned());
        assert_eq!(h.evidence_status(), "SIM_VIRTUAL_METAL_NOT_METAL");
    }
}
