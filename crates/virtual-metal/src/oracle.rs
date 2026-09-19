//! Test-only privileged device truth. Never fed to decide/governor.

use serde::{Deserialize, Serialize};

use crate::device::VirtualXl330;
use crate::peer::VirtualSerialPeer;

/// What Reality OS / the driver concluded about a write.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EffectBelief {
    /// Write completed; host believes an actuator effect occurred.
    KnownSuccess,
    /// Write refused/failed before apply; host believes no effect.
    KnownRefusal,
    /// Host cannot know whether an effect occurred.
    Unknown,
}

/// Outcome class for A7. Different faults map to different classes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum OutcomeClass {
    Success,
    Refusal,
    Unknown,
    Disconnect,
    StartupFailure,
}

impl OutcomeClass {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Success => "success",
            Self::Refusal => "refusal",
            Self::Unknown => "unknown",
            Self::Disconnect => "disconnect",
            Self::StartupFailure => "startup_failure",
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DeviceTruth {
    pub physical_actions: u64,
    pub present: i32,
    pub goal: i32,
    pub torque: bool,
    pub model: u16,
    pub firmware: u8,
    pub voltage_v: f64,
    pub hardware_error: u8,
    pub voltage_error_bit: u8,
    pub reset_count: u64,
    pub tx_count: usize,
    pub rx_count: usize,
}

impl DeviceTruth {
    pub fn from_device(d: &VirtualXl330) -> Self {
        Self::from_device_peer(d, None)
    }

    pub fn from_device_peer(d: &VirtualXl330, peer: Option<&VirtualSerialPeer>) -> Self {
        let (tx, rx) = peer
            .map(|p| {
                let t = p.transcript();
                (t.tx.len(), t.rx.len())
            })
            .unwrap_or((0, 0));
        Self {
            physical_actions: d.physical_action_count(),
            present: d.present_position(),
            goal: d.goal_position(),
            torque: d.torque_enabled(),
            model: d.model(),
            firmware: d.firmware(),
            voltage_v: d.voltage_v(),
            hardware_error: d.hardware_error(),
            voltage_error_bit: d.voltage_error_bit(),
            reset_count: d.reset_count(),
            tx_count: tx,
            rx_count: rx,
        }
    }
}

/// A1: authority belief vs privileged device truth.
///
/// Forbidden: "definitely no effect" while the device applied, or
/// "definitely happened" while it did not — unless belief is Unknown.
pub fn check_belief_vs_truth(
    belief: EffectBelief,
    before: &DeviceTruth,
    after: &DeviceTruth,
) -> Result<(), String> {
    let applied = after.physical_actions > before.physical_actions;
    match belief {
        EffectBelief::Unknown => Ok(()),
        EffectBelief::KnownSuccess => {
            if !applied {
                Err("a1_false_happened: authority KnownSuccess but device applied no effect".into())
            } else {
                Ok(())
            }
        }
        EffectBelief::KnownRefusal => {
            if applied {
                Err(
                    "a1_false_no_effect: authority KnownRefusal but device applied an effect"
                        .into(),
                )
            } else {
                Ok(())
            }
        }
    }
}

pub fn belief_from_trace(ok: bool, event: &str) -> EffectBelief {
    if event.contains("unknown") {
        EffectBelief::Unknown
    } else if ok {
        EffectBelief::KnownSuccess
    } else {
        EffectBelief::KnownRefusal
    }
}

pub fn class_from_trace(ok: bool, event: &str, connected: bool) -> OutcomeClass {
    if event.contains("unknown") {
        OutcomeClass::Unknown
    } else if !connected || event.contains("disconnect") {
        OutcomeClass::Disconnect
    } else if event.contains("open") || event.contains("startup") {
        OutcomeClass::StartupFailure
    } else if ok {
        OutcomeClass::Success
    } else {
        OutcomeClass::Refusal
    }
}
