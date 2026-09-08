//! Governor-owned gate input/output. No Reality OS types (DIP).

use realityos_kernel::{DecisionStatus, HonestyStamp, KernelError, KernelResult, GATE_EVIDENCE};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GovernorGateRequest {
    pub command_id: String,
    pub sequence: i64,
    pub allowed_action: Vec<f64>,
    pub certificate_status: DecisionStatus,
    pub release_hash: String,
    #[serde(default)]
    pub expires_at_s: Option<f64>,
}

impl GovernorGateRequest {
    pub fn new(
        command_id: impl Into<String>,
        sequence: i64,
        allowed_action: Vec<f64>,
        certificate_status: DecisionStatus,
        release_hash: impl Into<String>,
    ) -> KernelResult<Self> {
        let command_id = command_id.into();
        let release_hash = release_hash.into();
        if command_id.trim().is_empty() {
            return Err(KernelError::validation("gate.command_id", "empty"));
        }
        if release_hash.trim().is_empty() {
            return Err(KernelError::validation("gate.release_hash", "empty"));
        }
        if allowed_action.iter().any(|x| !x.is_finite()) {
            return Err(KernelError::validation("gate.allowed_action", "non-finite"));
        }
        Ok(Self {
            command_id,
            sequence,
            allowed_action,
            certificate_status,
            release_hash,
            expires_at_s: None,
        })
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GovernorGateVerdict {
    pub ok: bool,
    pub event: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub command_id: Option<String>,
    pub release_hash: String,
    #[serde(default)]
    pub violations: Vec<String>,
    pub honesty: HonestyStamp,
}

impl GovernorGateVerdict {
    pub fn ok_event(event: impl Into<String>, release_hash: impl Into<String>) -> Self {
        Self {
            ok: true,
            event: event.into(),
            command_id: None,
            release_hash: release_hash.into(),
            violations: Vec::new(),
            honesty: HonestyStamp::sim(GATE_EVIDENCE).expect("GATE_EVIDENCE"),
        }
    }

    pub fn refused(
        event: impl Into<String>,
        release_hash: impl Into<String>,
        violations: Vec<String>,
    ) -> Self {
        Self {
            ok: false,
            event: event.into(),
            command_id: None,
            release_hash: release_hash.into(),
            violations,
            honesty: HonestyStamp::sim(GATE_EVIDENCE).expect("GATE_EVIDENCE"),
        }
    }
}

/// Map any ActuationCommand-shaped view into a gate request (one-way).
pub fn admit_from_parts(
    command_id: &str,
    sequence: i64,
    allowed_action: &[f64],
    certificate_status: DecisionStatus,
    release_hash: &str,
    expires_at_s: f64,
) -> KernelResult<GovernorGateRequest> {
    let mut req = GovernorGateRequest::new(
        command_id,
        sequence,
        allowed_action.to_vec(),
        certificate_status,
        release_hash,
    )?;
    req.expires_at_s = Some(expires_at_s);
    Ok(req)
}
