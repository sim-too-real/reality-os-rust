//! E-stop, abort latch, HOLD/FREEZE/FAULT. Never auto-clear.

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum SafeState {
    Running,
    Hold,
    Freeze,
    Fault,
}

impl SafeState {
    pub fn parse(raw: &str) -> Result<Self, String> {
        match raw.trim().to_ascii_uppercase().as_str() {
            "RUNNING" => Ok(Self::Running),
            "HOLD" => Ok(Self::Hold),
            "FREEZE" => Ok(Self::Freeze),
            "FAULT" => Ok(Self::Fault),
            other => Err(format!("invalid_safe_state:{other}")),
        }
    }

    #[inline]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Running => "RUNNING",
            Self::Hold => "HOLD",
            Self::Freeze => "FREEZE",
            Self::Fault => "FAULT",
        }
    }

    #[inline]
    pub const fn blocks_actuation(self) -> bool {
        matches!(self, Self::Hold | Self::Freeze | Self::Fault)
    }

    #[inline]
    const fn restrictiveness(self) -> u8 {
        match self {
            Self::Running => 0,
            Self::Hold => 1,
            Self::Freeze => 2,
            Self::Fault => 3,
        }
    }

    /// ONLINE may only move toward a more restrictive state.
    #[inline]
    pub const fn tighten(self, requested: Self) -> Self {
        if requested.restrictiveness() >= self.restrictiveness() {
            requested
        } else {
            self
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct SafeStateLatch {
    pub session_id: String,
    pub state: SafeState,
    pub reason: String,
}

impl SafeStateLatch {
    pub fn running(session_id: impl Into<String>) -> Self {
        Self {
            session_id: session_id.into(),
            state: SafeState::Running,
            reason: String::new(),
        }
    }

    pub fn enter(&mut self, state: SafeState, reason: impl Into<String>) {
        self.state = state;
        self.reason = reason.into();
    }

    pub fn recover(&mut self, authorized: bool, reason: impl Into<String>) -> bool {
        if !authorized {
            return false;
        }
        self.enter(SafeState::Running, reason);
        true
    }
}

#[derive(Debug, Clone, Default)]
pub struct EstopLatch {
    pub engaged: bool,
    pub abort_latched: bool,
    pub reason: Option<String>,
}

impl EstopLatch {
    pub fn engage(&mut self, reason: impl Into<String>) {
        self.engaged = true;
        self.abort_latched = true;
        self.reason = Some(reason.into());
    }

    pub fn latch_abort(&mut self, reason: impl Into<String>) {
        self.abort_latched = true;
        self.reason = Some(reason.into());
    }

    pub fn clear(&mut self) {
        self.engaged = false;
        self.abort_latched = false;
        self.reason = None;
    }
}
