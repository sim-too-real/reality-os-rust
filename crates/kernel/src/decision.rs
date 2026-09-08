//! Five-word verdict vocabulary.
//!
//! Port of `theworld.core.decision`. Probe is not abort. Refuse is not abort.
//! Unknown words never become a safe-looking state.

use crate::error::{KernelError, KernelResult};
use serde::{Deserialize, Serialize};

/// Canonical kernel verdicts. Only Allow and Modify may emit motion.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum DecisionStatus {
    Allow,
    Modify,
    Probe,
    Refuse,
    Abort,
}

impl DecisionStatus {
    pub const ALL: [Self; 5] = [
        Self::Allow,
        Self::Modify,
        Self::Probe,
        Self::Refuse,
        Self::Abort,
    ];

    #[inline]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Allow => "allow",
            Self::Modify => "modify",
            Self::Probe => "probe",
            Self::Refuse => "refuse",
            Self::Abort => "abort",
        }
    }

    pub fn parse(raw: &str) -> KernelResult<Self> {
        match raw.trim().to_ascii_lowercase().as_str() {
            "allow" => Ok(Self::Allow),
            "modify" => Ok(Self::Modify),
            "probe" => Ok(Self::Probe),
            "refuse" => Ok(Self::Refuse),
            "abort" => Ok(Self::Abort),
            other => Err(KernelError::InvalidDecision(other.to_string())),
        }
    }

    /// Clear to proceed (possibly with a corrected action) without further evidence.
    #[inline]
    pub const fn allowed(self) -> bool {
        matches!(self, Self::Allow | Self::Modify)
    }

    /// Execution does not proceed as-is.
    #[inline]
    pub const fn blocked(self) -> bool {
        matches!(self, Self::Probe | Self::Refuse | Self::Abort)
    }

    /// An action was actually emitted. Only committed actions can be unsafe.
    #[inline]
    pub const fn committed(self) -> bool {
        self.allowed()
    }

    /// Governor may respond with allow / modify / abort only — never invent Probe.
    #[inline]
    pub const fn governor_may_emit(self) -> bool {
        matches!(self, Self::Allow | Self::Modify | Self::Abort)
    }
}

impl std::fmt::Display for DecisionStatus {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.as_str())
    }
}

/// Product-facing verdict. UI/API clients read this regardless of subsystem.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct UnifiedDecision {
    pub status: DecisionStatus,
    pub reason: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub next_probe: Option<String>,
    #[serde(default)]
    pub source: String,
}

impl UnifiedDecision {
    pub fn new(status: DecisionStatus, reason: impl Into<String>) -> Self {
        Self {
            status,
            reason: reason.into(),
            next_probe: None,
            source: String::new(),
        }
    }

    pub fn with_source(mut self, source: impl Into<String>) -> Self {
        self.source = source.into();
        self
    }

    pub fn with_probe(mut self, probe: impl Into<String>) -> Self {
        self.next_probe = Some(probe.into());
        self
    }

    #[inline]
    pub const fn allowed(&self) -> bool {
        self.status.allowed()
    }

    #[inline]
    pub const fn blocked(&self) -> bool {
        self.status.blocked()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_canonical_word_parses() {
        for d in DecisionStatus::ALL {
            assert_eq!(DecisionStatus::parse(d.as_str()).unwrap(), d);
        }
    }

    #[test]
    fn probe_is_not_abort() {
        assert_ne!(DecisionStatus::Probe, DecisionStatus::Abort);
        assert!(!DecisionStatus::Probe.committed());
        assert!(DecisionStatus::Probe.blocked());
    }

    #[test]
    fn unknown_word_is_error_not_abort() {
        let err = DecisionStatus::parse("maybe").unwrap_err();
        match err {
            KernelError::InvalidDecision(s) => assert_eq!(s, "maybe"),
            other => panic!("expected InvalidDecision, got {other:?}"),
        }
    }
}
