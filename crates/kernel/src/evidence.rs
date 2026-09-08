//! Evidence tiers. Non-promotable. SIM never becomes MEASURED here.

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum EvidenceTier {
    SimVerified,
    SimScreen,
    NotEvidence,
}

impl EvidenceTier {
    #[inline]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::SimVerified => "SIM_VERIFIED",
            Self::SimScreen => "SIM_SCREEN",
            Self::NotEvidence => "NOT_EVIDENCE",
        }
    }

    /// Tiers cannot be promoted to MEASURED by this type.
    #[inline]
    pub const fn is_measured(self) -> bool {
        false
    }
}

pub const GATE_EVIDENCE: &str = "SIM_GOVERNOR_GATE_NOT_METAL";
pub const KERNEL_EVIDENCE: &str = "SIM_REALITY_OS_KERNEL_NOT_METAL";
pub const LEDGER_EVIDENCE: &str = "SIM_COMMAND_LEDGER_NOT_METAL";
pub const BOUNDED_TRUST_EVIDENCE: &str = "SIM_BOUNDED_TRUST_NOT_METAL";
pub const PFL_EVIDENCE: &str = "SIM_PFL_SCREEN_NOT_MEASURED";
