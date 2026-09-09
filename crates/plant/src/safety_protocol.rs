//! Software safety-island protocol. Not a certified MCU/PLC.

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SafeTransition {
    Disable,
    EnableRequest,
    ProtectiveStop,
    EmergencyStop,
    FaultLatch,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SafetyFrame {
    pub sequence: u64,
    pub expires_at_s: f64,
    pub transition: SafeTransition,
    pub ack: bool,
}

impl SafetyFrame {
    pub fn request(sequence: u64, expires_at_s: f64, transition: SafeTransition) -> Option<Self> {
        if !expires_at_s.is_finite() || expires_at_s <= 0.0 {
            return None;
        }
        Some(Self {
            sequence,
            expires_at_s,
            transition,
            ack: false,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn non_finite_expiry_is_unrepresentable() {
        assert!(SafetyFrame::request(1, f64::NAN, SafeTransition::ProtectiveStop).is_none());
    }
}
