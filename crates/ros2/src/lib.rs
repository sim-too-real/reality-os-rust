//! ROS 2 adapter contracts. Not the last-gate. Hardware writes stay off.
//!
//! Product path is RuntimeGovernor. This crate publishes veto/status shapes
//! so a future rclrs node can wrap the gate without owning authority.

use realityos_governor::RuntimeTrace;
use realityos_kernel::DecisionStatus;
use serde::{Deserialize, Serialize};

/// Fail-closed default: veto is true until a live gate says otherwise.
pub const DEFAULT_VETO: bool = true;
pub const HARDWARE_WRITES_ENABLED: bool = false;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VetoMsg {
    pub veto: bool,
    pub reason: String,
    pub metal: bool,
}

impl VetoMsg {
    pub fn fail_closed() -> Self {
        Self {
            veto: true,
            reason: "hardware_writes_disabled_use_runtime_governor".into(),
            metal: false,
        }
    }

    pub fn from_trace(trace: &RuntimeTrace) -> Self {
        Self {
            veto: !trace.ok,
            reason: if trace.violations.is_empty() {
                trace.event.clone()
            } else {
                trace.violations.join(",")
            },
            metal: false,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DecisionMsg {
    pub status: String,
    pub allowed: bool,
    pub action: Vec<f64>,
    pub physical_reason: String,
    pub metal: bool,
}

impl DecisionMsg {
    pub fn from_status(
        status: DecisionStatus,
        action: Vec<f64>,
        reason: impl Into<String>,
    ) -> Self {
        Self {
            status: status.as_str().into(),
            allowed: status.allowed(),
            action,
            physical_reason: reason.into(),
            metal: false,
        }
    }
}

pub const TOPICS: &[(&str, &str)] = &[
    ("/governor/veto", "std_msgs/Bool"),
    ("/governor/status", "GovernorStatus"),
    ("/reality_os/intent_text", "std_msgs/String"),
    ("/reality_os/decision", "std_msgs/String"),
    ("/reality_os/allowed", "std_msgs/Bool"),
    ("/reality_os/safe_action", "std_msgs/Float64MultiArray"),
];

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn boot_veto_is_true() {
        let v = VetoMsg::fail_closed();
        assert!(v.veto);
        const {
            assert!(!HARDWARE_WRITES_ENABLED);
            assert!(DEFAULT_VETO);
        }
    }
}
