//! Debugger / UI rail. Display mapping is lossy by construction.
//! Emit `cert_status` verbatim beside the rail. Unmapped → Unknown, never Abort.

use crate::decision::DecisionStatus;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum RailStatus {
    Pass,
    Corrected,
    Probe,
    Refuse,
    Abort,
    Unknown,
}

impl RailStatus {
    #[inline]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Pass => "pass",
            Self::Corrected => "corrected",
            Self::Probe => "probe",
            Self::Refuse => "refuse",
            Self::Abort => "abort",
            Self::Unknown => "unknown",
        }
    }

    #[inline]
    pub const fn committed(self) -> bool {
        matches!(self, Self::Pass | Self::Corrected)
    }

    #[inline]
    pub const fn blocked(self) -> bool {
        matches!(self, Self::Refuse | Self::Abort)
    }
}

/// Map a certificate / decision onto the rail. Never defaults to abort.
pub fn to_rail(status: DecisionStatus) -> RailStatus {
    match status {
        DecisionStatus::Allow => RailStatus::Pass,
        DecisionStatus::Modify => RailStatus::Corrected,
        DecisionStatus::Probe => RailStatus::Probe,
        DecisionStatus::Refuse => RailStatus::Refuse,
        DecisionStatus::Abort => RailStatus::Abort,
    }
}

/// `veto` (VLA last-moment stop) maps to abort. Anything else is unknown.
pub fn to_rail_word(raw: &str) -> RailStatus {
    match raw.trim().to_ascii_lowercase().as_str() {
        "pass" | "allow" => RailStatus::Pass,
        "corrected" | "modify" => RailStatus::Corrected,
        "probe" => RailStatus::Probe,
        "refuse" => RailStatus::Refuse,
        "abort" | "veto" => RailStatus::Abort,
        _ => RailStatus::Unknown,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn probe_does_not_collapse_to_abort() {
        assert_eq!(to_rail(DecisionStatus::Probe), RailStatus::Probe);
        assert_eq!(to_rail(DecisionStatus::Refuse), RailStatus::Refuse);
        assert_eq!(to_rail_word("maybe"), RailStatus::Unknown);
    }

    #[test]
    fn every_decision_has_a_rail() {
        for d in DecisionStatus::ALL {
            let r = to_rail(d);
            assert_ne!(r, RailStatus::Unknown);
        }
    }
}
