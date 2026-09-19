//! Explicit fault schedule. Applied at the device or byte/transport layer.

use serde::{Deserialize, Serialize};

/// Write-lifecycle sites where crash/loss/reset/ACK-loss is injected.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum WriteLifecycleBoundary {
    BeforePacketConstruction,
    AfterPacketConstruction,
    BeforeSerialWrite,
    DuringSerialWrite,
    AfterSerialWrite,
    BeforeDeviceApply,
    AfterDeviceApply,
    BeforeStatusCreation,
    AfterStatusCreation,
    BeforeHostRead,
    DuringHostRead,
    AfterHostRead,
    BeforeLedgerAppend,
    AfterLedgerAppend,
    BeforeFsync,
    AfterFsync,
    BeforeCallerAck,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LifecycleLoss {
    ProcessCrash,
    SerialLoss,
    DeviceReset,
    AckLoss,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FaultKind {
    DropStatusAfterApply,
    DropStatus,
    DelayStatus {
        ms: u64,
    },
    TruncateStatus {
        keep: usize,
    },
    #[serde(alias = "corrupt_status_crc")]
    CorruptOutgoingCrc,
    WrongStatusId,
    DuplicateStatus,
    SplitStatusAcrossReads {
        first: usize,
    },
    GarbagePrefix,
    GarbageSuffix,
    DeviceSilent,
    Disconnect,
    Reconnect,
    RebootDuringRequest,
    StatusAfterTimeout {
        ms: u64,
    },
    VoltageOutOfRange,
    OverTemperature,
    WatchdogTrip,
    IdentitySwap {
        model: u16,
        firmware: u8,
    },
    /// e-Manual Shutdown table uses 0x01; voltage-limit prose uses 0x10.
    VoltageErrorBit {
        bit: u8,
    },
}

impl FaultKind {
    pub fn is_transport(&self) -> bool {
        matches!(
            self,
            Self::DropStatus
                | Self::DelayStatus { .. }
                | Self::TruncateStatus { .. }
                | Self::CorruptOutgoingCrc
                | Self::WrongStatusId
                | Self::DuplicateStatus
                | Self::SplitStatusAcrossReads { .. }
                | Self::GarbagePrefix
                | Self::GarbageSuffix
                | Self::DeviceSilent
                | Self::Disconnect
                | Self::Reconnect
                | Self::RebootDuringRequest
                | Self::StatusAfterTimeout { .. }
                | Self::DropStatusAfterApply
        )
    }
}

/// Mutate the on-wire CRC of an already-encoded Protocol 2.0 packet.
pub fn corrupt_crc_bytes(mut packet: Vec<u8>) -> Vec<u8> {
    if let Some(b) = packet.last_mut() {
        *b ^= 0xFF;
    }
    packet
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

    pub fn once(kind: FaultKind) -> Self {
        Self {
            events: vec![FaultEvent {
                after_packet: 1,
                kind,
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

    /// Drop one event at a time until `still_fails` is false; return the
    /// smallest prefix that still fails, or the original if every event is required.
    pub fn shrink(&self, mut still_fails: impl FnMut(&FaultSchedule) -> bool) -> FaultSchedule {
        if !still_fails(self) {
            return self.clone();
        }
        let mut events = self.events.clone();
        let mut i = 0;
        while i < events.len() {
            let mut candidate = events.clone();
            candidate.remove(i);
            let sched = FaultSchedule {
                events: candidate.clone(),
            };
            if still_fails(&sched) {
                events = candidate;
            } else {
                i += 1;
            }
        }
        FaultSchedule { events }
    }
}
