//! Durable consume / write lifecycle.
//!
//! A command may cause at most one physical actuation attempt. After any
//! crash/restart the system must not retry a command whose outcome may already
//! have affected the plant.
//!
//! ```text
//! Unseen
//!   → prepare (journal synced)     // crash here: no physical write; id is spent
//!   → physical write attempted
//!       → outcome known (ack/consume)
//!       → outcome unknown          // any error or crash after the attempt
//! ```
//!
//! Restart rule: `prepare`, `consume`, and `unknown_outcome` all mark the
//! command id seen. `CommandOutcome::Unknown` is never retryable.

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ConsumePhase {
    Unseen,
    Prepared,
    Consumed,
    Unknown,
}

impl ConsumePhase {
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Unseen => "unseen",
            Self::Prepared => "prepared",
            Self::Consumed => "consumed",
            Self::Unknown => "unknown",
        }
    }

    /// Only a command that never reached prepare may be attempted.
    pub const fn may_attempt_write(self) -> bool {
        matches!(self, Self::Unseen)
    }
}

/// Crash location → required restart behavior. Used by tests and the TLA+ model.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CrashPoint {
    BeforePrepare,
    AfterPrepareBeforeWrite,
    AfterWriteBeforeAck,
    AfterAck,
}

impl CrashPoint {
    pub const fn restart_phase(self) -> ConsumePhase {
        match self {
            Self::BeforePrepare => ConsumePhase::Unseen,
            Self::AfterPrepareBeforeWrite | Self::AfterWriteBeforeAck => ConsumePhase::Prepared,
            Self::AfterAck => ConsumePhase::Consumed,
        }
    }

    pub const fn may_retry_on_restart(self) -> bool {
        self.restart_phase().may_attempt_write()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn only_unseen_may_write() {
        assert!(ConsumePhase::Unseen.may_attempt_write());
        assert!(!ConsumePhase::Prepared.may_attempt_write());
        assert!(!ConsumePhase::Consumed.may_attempt_write());
        assert!(!ConsumePhase::Unknown.may_attempt_write());
        assert!(!CrashPoint::AfterWriteBeforeAck.may_retry_on_restart());
        assert!(CrashPoint::BeforePrepare.may_retry_on_restart());
    }
}
