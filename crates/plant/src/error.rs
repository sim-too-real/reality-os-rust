use thiserror::Error;

#[derive(Debug, Error, Clone, PartialEq, Eq)]
pub enum PlantError {
    #[error("{0}")]
    Refused(String),
    #[error("online plant {method} requires execute_certified_command")]
    UncertifiedOnline { method: &'static str },
    #[error("journal unreadable: {0}")]
    JournalUnreadable(String),
    #[error("online journal missing (not first_online)")]
    JournalMissing,
    #[error("online journal deleted (seal present, journal absent)")]
    JournalDeleted,
    #[error("online journal rolled back (seal ahead of journal)")]
    JournalRollback,
    #[error("online journal replaced (chain does not extend the seal)")]
    JournalReplaced,
    #[error("online journal seal missing")]
    JournalSealMissing,
    #[error("operator ack required to clear e-stop")]
    OperatorAckRequired,
    #[error("estop engaged")]
    EstopEngaged,
    #[error("plant has no live driver")]
    NoDriver,
    #[error("driver not connected")]
    Disconnected,
    #[error("egress disabled; use RuntimeGovernor")]
    EgressDisabled,
    #[error("fieldbus not attached (named hole)")]
    FieldbusNotAttached,
    #[error("named hole: {0}")]
    NamedHole(&'static str),
    #[error("unknown command outcome: write issued without acknowledgement")]
    UnknownOutcome,
}

impl PlantError {
    pub fn refused(reason: impl Into<String>) -> Self {
        Self::Refused(reason.into())
    }
}

pub type PlantResult<T> = Result<T, PlantError>;
