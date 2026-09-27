//! Boundary for a later hardware experiment.
//!
//! The production reasoner cannot implement this trait. An unconfigured
//! authority refuses every issue. Nothing here writes a device, and every
//! claim stays `SIMULATION_ONLY` until a separately qualified issuer exists.

pub const EVIDENCE_STATUS: &str = "SIMULATION_ONLY";

#[derive(Debug, Clone, PartialEq)]
pub struct HardwareActionScope {
    pub action_key: String,
    pub witness_digest: String,
    pub observation_epoch: String,
}

/// Identity-bound hardware permission. No constructor is public: only a
/// qualified `HardwareAuthority` implementor can produce one, and this crate
/// does not contain a qualified implementor.
#[derive(Debug, Clone, PartialEq)]
pub struct HardwareGrant {
    id: String,
    scope_digest: String,
}

impl HardwareGrant {
    pub fn id(&self) -> &str {
        &self.id
    }

    pub fn scope_digest(&self) -> &str {
        &self.scope_digest
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum HardwareRefusal {
    NotQualified,
}

pub trait HardwareAuthority {
    fn issue(&mut self, scope: &HardwareActionScope) -> Result<HardwareGrant, HardwareRefusal>;
}

#[derive(Debug, Default)]
pub struct UnconfiguredHardware;

impl HardwareAuthority for UnconfiguredHardware {
    fn issue(&mut self, _scope: &HardwareActionScope) -> Result<HardwareGrant, HardwareRefusal> {
        Err(HardwareRefusal::NotQualified)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unconfigured_hardware_issues_nothing() {
        let mut authority = UnconfiguredHardware;
        let refused = authority.issue(&HardwareActionScope {
            action_key: "push:+x".into(),
            witness_digest: "abc".into(),
            observation_epoch: "epoch".into(),
        });
        assert_eq!(refused, Err(HardwareRefusal::NotQualified));
        assert_eq!(EVIDENCE_STATUS, "SIMULATION_ONLY");
    }
}
