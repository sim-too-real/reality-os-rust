use thiserror::Error;

#[derive(Debug, Error, Clone, PartialEq, Eq)]
pub enum PlantError {
    #[error("{0}")]
    Refused(String),
    #[error("online plant {method} requires execute_certified_command")]
    UncertifiedOnline { method: &'static str },
    #[error("journal unreadable: {0}")]
    JournalUnreadable(String),
    #[error("operator ack required to clear e-stop")]
    OperatorAckRequired,
    #[error("estop engaged")]
    EstopEngaged,
    #[error("plant has no live driver")]
    NoDriver,
}

impl PlantError {
    pub fn refused(reason: impl Into<String>) -> Self {
        Self::Refused(reason.into())
    }
}

pub type PlantResult<T> = Result<T, PlantError>;
