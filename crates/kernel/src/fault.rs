//! Shared fault taxonomy. Unknown is never mapped to Allow.

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FaultKind {
    Unsupported,
    Validation,
    StaleEvidence,
    Signature,
    Replay,
    Sequence,
    WatchdogMiss,
    UnknownOutcome,
    BackendTimeout,
    Infeasible,
    Identity,
    Estop,
}

impl FaultKind {
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Unsupported => "unsupported",
            Self::Validation => "validation",
            Self::StaleEvidence => "stale_evidence",
            Self::Signature => "signature",
            Self::Replay => "replay",
            Self::Sequence => "sequence",
            Self::WatchdogMiss => "watchdog_miss",
            Self::UnknownOutcome => "unknown_outcome",
            Self::BackendTimeout => "backend_timeout",
            Self::Infeasible => "infeasible",
            Self::Identity => "identity",
            Self::Estop => "estop",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CommandOutcome {
    Executed,
    Refused,
    Unknown,
}

impl CommandOutcome {
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Executed => "executed",
            Self::Refused => "refused",
            Self::Unknown => "unknown_outcome",
        }
    }

    pub const fn may_retry(self) -> bool {
        matches!(self, Self::Refused)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unknown_outcome_is_not_retryable() {
        assert!(!CommandOutcome::Unknown.may_retry());
        assert!(CommandOutcome::Refused.may_retry());
    }
}
