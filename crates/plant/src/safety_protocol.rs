//! Software safety-island protocol. Not a certified MCU/PLC and not STO hardware.

use std::collections::HashSet;

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

impl SafeTransition {
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Disable => "disable",
            Self::EnableRequest => "enable_request",
            Self::ProtectiveStop => "protective_stop",
            Self::EmergencyStop => "emergency_stop",
            Self::FaultLatch => "fault_latch",
        }
    }
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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IslandVerdict {
    Ack,
    Nack,
}

/// Software supervisor for enable / protective-stop / e-stop.
/// `enabled` is a software interlock, not a hardware STO channel.
#[derive(Debug, Clone)]
pub struct SafetyIsland {
    last_seq: u64,
    seen: HashSet<u64>,
    enabled: bool,
    latched: bool,
    reason: Option<String>,
}

impl Default for SafetyIsland {
    fn default() -> Self {
        Self::new()
    }
}

impl SafetyIsland {
    pub fn new() -> Self {
        Self {
            last_seq: 0,
            seen: HashSet::new(),
            enabled: false,
            latched: false,
            reason: None,
        }
    }

    pub fn enabled(&self) -> bool {
        self.enabled && !self.latched
    }

    pub fn latched(&self) -> bool {
        self.latched
    }

    pub fn sto_software_asserted(&self) -> bool {
        !self.enabled()
    }

    pub fn metal(&self) -> bool {
        false
    }

    pub fn ingest(&mut self, frame: &SafetyFrame, now_s: f64) -> IslandVerdict {
        if !now_s.is_finite() || now_s > frame.expires_at_s {
            return IslandVerdict::Nack;
        }
        if frame.sequence == 0 || self.seen.contains(&frame.sequence) || frame.sequence <= self.last_seq
        {
            return IslandVerdict::Nack;
        }
        if matches!(frame.transition, SafeTransition::EnableRequest) && (self.latched || !frame.ack)
        {
            return IslandVerdict::Nack;
        }
        self.seen.insert(frame.sequence);
        self.last_seq = frame.sequence;
        match frame.transition {
            SafeTransition::EnableRequest => {
                self.enabled = true;
                IslandVerdict::Ack
            }
            SafeTransition::Disable => {
                self.enabled = false;
                IslandVerdict::Ack
            }
            SafeTransition::ProtectiveStop | SafeTransition::EmergencyStop | SafeTransition::FaultLatch => {
                self.enabled = false;
                self.latched = true;
                self.reason = Some(frame.transition.as_str().into());
                IslandVerdict::Ack
            }
        }
    }

    pub fn clear_latch(&mut self, operator_ack: bool) -> IslandVerdict {
        if !operator_ack {
            return IslandVerdict::Nack;
        }
        self.latched = false;
        self.enabled = false;
        self.reason = None;
        IslandVerdict::Ack
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn non_finite_expiry_is_unrepresentable() {
        assert!(SafetyFrame::request(1, f64::NAN, SafeTransition::ProtectiveStop).is_none());
    }

    #[test]
    fn island_is_software_sto_not_metal() {
        let mut isle = SafetyIsland::new();
        assert!(isle.sto_software_asserted());
        assert!(!isle.metal());
        let req = SafetyFrame::request(1, 10.0, SafeTransition::EnableRequest).unwrap();
        assert_eq!(isle.ingest(&req, 1.0), IslandVerdict::Nack);
        let mut acked = req;
        acked.ack = true;
        assert_eq!(isle.ingest(&acked, 1.0), IslandVerdict::Ack);
        assert!(isle.enabled());
        assert_eq!(isle.ingest(&acked, 1.0), IslandVerdict::Nack);
        let estop = SafetyFrame::request(2, 10.0, SafeTransition::EmergencyStop).unwrap();
        assert_eq!(isle.ingest(&estop, 1.0), IslandVerdict::Ack);
        assert!(isle.latched());
        assert_eq!(isle.clear_latch(false), IslandVerdict::Nack);
        assert_eq!(isle.clear_latch(true), IslandVerdict::Ack);
        assert!(isle.sto_software_asserted());
    }
}
