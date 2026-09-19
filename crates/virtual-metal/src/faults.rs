//! Explicit fault schedule. Not scattered `if fault` in the driver.

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FaultKind {
    DropStatusAfterApply,
    CorruptOutgoingCrc,
    WrongStatusId,
    VoltageOutOfRange,
    OverTemperature,
    WatchdogTrip,
    IdentitySwap { model: u16, firmware: u8 },
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct FaultEvent {
    pub after_packet: u32,
    pub kind: FaultKind,
}

#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct FaultSchedule {
    pub events: Vec<FaultEvent>,
}

impl FaultSchedule {
    pub fn empty() -> Self {
        Self { events: Vec::new() }
    }

    pub fn drop_ack_after(n: u32) -> Self {
        Self {
            events: vec![FaultEvent {
                after_packet: n,
                kind: FaultKind::DropStatusAfterApply,
            }],
        }
    }

    pub fn identity_swap_after(n: u32, model: u16, firmware: u8) -> Self {
        Self {
            events: vec![FaultEvent {
                after_packet: n,
                kind: FaultKind::IdentitySwap { model, firmware },
            }],
        }
    }

    pub fn take_at(&mut self, packet: u32) -> Vec<FaultKind> {
        let mut out = Vec::new();
        self.events.retain(|e| {
            if e.after_packet == packet {
                out.push(e.kind.clone());
                false
            } else {
                true
            }
        });
        out
    }
}
