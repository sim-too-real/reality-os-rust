//! Fail-closed errors. Unknown safety state is never mapped to Allow.

use thiserror::Error;

#[derive(Debug, Error, Clone, PartialEq, Eq)]
pub enum KernelError {
    #[error("invalid decision status: {0}")]
    InvalidDecision(String),
    #[error("invalid identifier {field}: {reason}")]
    InvalidId { field: &'static str, reason: String },
    #[error("honesty forbidden claim: {0}")]
    HonestyForbidden(String),
    #[error("honesty evidence token is not SIM/not-metal: {0}")]
    HonestyEvidence(String),
    #[error("validation failed for {field}: {reason}")]
    Validation { field: &'static str, reason: String },
}

pub type KernelResult<T> = Result<T, KernelError>;

impl KernelError {
    pub fn validation(field: &'static str, reason: impl Into<String>) -> Self {
        Self::Validation {
            field,
            reason: reason.into(),
        }
    }
}
