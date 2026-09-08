use realityos_kernel::{DecisionStatus, KernelError, KernelResult};
use realityos_plant::ActuationCommand;
use serde::{Deserialize, Serialize};

use crate::certificate::Certificate;

/// The ONE object that may cross from Reality OS into a Governor/driver.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CertifiedCommand {
    pub command_id: String,
    pub sequence: i64,
    pub issued_at_s: f64,
    pub expires_at_s: f64,
    pub certificate: Certificate,
    pub allowed_action: Vec<f64>,
    #[serde(default)]
    pub release_hash: String,
    #[serde(default)]
    pub as_built_hash: String,
    #[serde(default)]
    pub calibration_ids: Vec<String>,
    #[serde(default)]
    pub acknowledged: bool,
    #[serde(default)]
    pub issuer_allowed_action: Vec<f64>,
    #[serde(default)]
    pub issuer_certificate_status: String,
    #[serde(default)]
    pub issuer_physical_reason: String,
    #[serde(default)]
    pub sensor_snapshot_id: String,
    #[serde(default)]
    pub sensor_packet_hash: String,
    #[serde(default)]
    pub belief_snapshot_id: String,
    #[serde(default)]
    pub payload_hash: String,
    #[serde(default)]
    pub signature: String,
    #[serde(default)]
    pub signer: String,
    #[serde(default)]
    pub signing_scheme: String,
    #[serde(default)]
    pub actuator_ids: Vec<String>,
    #[serde(default)]
    pub waypoints: Vec<Vec<f64>>,
}

impl CertifiedCommand {
    pub fn issue(
        command_id: impl Into<String>,
        sequence: i64,
        now_s: f64,
        ttl_s: f64,
        certificate: Certificate,
        allowed_action: Vec<f64>,
    ) -> KernelResult<Self> {
        let command_id = command_id.into();
        if command_id.trim().is_empty() {
            return Err(KernelError::validation("command.command_id", "empty"));
        }
        if !now_s.is_finite() || !ttl_s.is_finite() {
            return Err(KernelError::validation("command.time", "non-finite"));
        }
        if allowed_action.iter().any(|x| !x.is_finite()) {
            return Err(KernelError::validation(
                "command.allowed_action",
                "non-finite",
            ));
        }
        let issuer_status = certificate.status.as_str().to_string();
        let issuer_reason = certificate.physical_reason.clone();
        Ok(Self {
            command_id,
            sequence,
            issued_at_s: now_s,
            expires_at_s: now_s + ttl_s,
            certificate,
            issuer_allowed_action: allowed_action.clone(),
            allowed_action,
            release_hash: String::new(),
            as_built_hash: String::new(),
            calibration_ids: Vec::new(),
            acknowledged: false,
            issuer_certificate_status: issuer_status,
            issuer_physical_reason: issuer_reason,
            sensor_snapshot_id: String::new(),
            sensor_packet_hash: String::new(),
            belief_snapshot_id: String::new(),
            payload_hash: String::new(),
            signature: String::new(),
            signer: String::new(),
            signing_scheme: String::new(),
            actuator_ids: Vec::new(),
            waypoints: Vec::new(),
        })
    }

    pub fn is_expired(&self, now_s: f64) -> bool {
        now_s > self.expires_at_s
    }

    pub fn sign(&mut self, key: &[u8]) {
        let hash = realityos_plant::command_payload_hash(self);
        self.payload_hash = hash.clone();
        self.signature = realityos_plant::sign_payload(key, &hash);
        self.signing_scheme = realityos_plant::SIGNING_SCHEME.into();
        self.signer = "realityos".into();
        if self.issuer_allowed_action.is_empty() {
            self.issuer_allowed_action = self.allowed_action.clone();
        }
    }

    pub fn acknowledge(&mut self) {
        self.acknowledged = true;
    }

    /// Bind empty identity fields only. Never overwrite a foreign hash.
    pub fn bind_identity(
        &mut self,
        release_hash: &str,
        as_built: &str,
        calibration_id: &str,
    ) -> Result<(), Vec<String>> {
        let mut errs = Vec::new();
        if !self.release_hash.is_empty() && self.release_hash != release_hash {
            errs.push("foreign_release_hash".into());
        }
        if !self.as_built_hash.is_empty() && !as_built.is_empty() && self.as_built_hash != as_built
        {
            errs.push("foreign_as_built_hash".into());
        }
        if !self.calibration_ids.is_empty()
            && !calibration_id.is_empty()
            && !self.calibration_ids.iter().any(|c| c == calibration_id)
        {
            errs.push("foreign_calibration_ids".into());
        }
        if !errs.is_empty() {
            return Err(errs);
        }
        if self.release_hash.is_empty() {
            self.release_hash = release_hash.into();
        }
        if self.as_built_hash.is_empty() {
            self.as_built_hash = as_built.into();
        }
        if self.calibration_ids.is_empty() && !calibration_id.is_empty() {
            self.calibration_ids = vec![calibration_id.into()];
        }
        Ok(())
    }
}

impl ActuationCommand for CertifiedCommand {
    fn command_id(&self) -> &str {
        &self.command_id
    }
    fn sequence(&self) -> i64 {
        self.sequence
    }
    fn issued_at_s(&self) -> f64 {
        self.issued_at_s
    }
    fn expires_at_s(&self) -> f64 {
        self.expires_at_s
    }
    fn allowed_action(&self) -> &[f64] {
        &self.allowed_action
    }
    fn issuer_allowed_action(&self) -> &[f64] {
        if self.issuer_allowed_action.is_empty() {
            &self.allowed_action
        } else {
            &self.issuer_allowed_action
        }
    }
    fn certificate_status(&self) -> DecisionStatus {
        self.certificate.status
    }
    fn issuer_certificate_status(&self) -> &str {
        if self.issuer_certificate_status.is_empty() {
            self.certificate.status.as_str()
        } else {
            &self.issuer_certificate_status
        }
    }
    fn issuer_physical_reason(&self) -> &str {
        if self.issuer_physical_reason.is_empty() {
            &self.certificate.physical_reason
        } else {
            &self.issuer_physical_reason
        }
    }
    fn release_hash(&self) -> &str {
        &self.release_hash
    }
    fn as_built_hash(&self) -> &str {
        &self.as_built_hash
    }
    fn calibration_ids(&self) -> &[String] {
        &self.calibration_ids
    }
    fn acknowledged(&self) -> bool {
        self.acknowledged
    }
    fn sensor_snapshot_id(&self) -> &str {
        &self.sensor_snapshot_id
    }
    fn sensor_packet_hash(&self) -> &str {
        &self.sensor_packet_hash
    }
    fn belief_snapshot_id(&self) -> &str {
        &self.belief_snapshot_id
    }
    fn payload_hash(&self) -> &str {
        &self.payload_hash
    }
    fn signature(&self) -> &str {
        &self.signature
    }
    fn signer(&self) -> &str {
        &self.signer
    }
    fn signing_scheme(&self) -> &str {
        &self.signing_scheme
    }
    fn actuator_ids(&self) -> &[String] {
        &self.actuator_ids
    }
    fn follow_waypoints(&self) -> Option<&[Vec<f64>]> {
        if self.waypoints.is_empty() {
            None
        } else {
            Some(&self.waypoints)
        }
    }
}

/// Governor may allow, narrow, or abort — never invent a new plan or upgrade a refuse.
pub fn narrow_certified_command(
    command: CertifiedCommand,
    status: DecisionStatus,
    allowed_action: Option<Vec<f64>>,
    reason: Option<String>,
) -> KernelResult<CertifiedCommand> {
    if !status.governor_may_emit() {
        return Err(KernelError::validation(
            "narrow.status",
            "governor may only allow|modify|abort",
        ));
    }
    let prior = command.certificate.status;
    if status == DecisionStatus::Allow && prior != DecisionStatus::Allow {
        return Err(KernelError::validation(
            "narrow.status",
            "Governor cannot upgrade a non-ALLOW RealityOS certificate to allow",
        ));
    }
    if status == DecisionStatus::Modify && !prior.allowed() {
        return Err(KernelError::validation(
            "narrow.status",
            "Governor cannot modify a command that RealityOS did not certify as executable",
        ));
    }
    if status == DecisionStatus::Modify && allowed_action.is_none() {
        return Err(KernelError::validation(
            "narrow.action",
            "status=modify requires an explicit allowed_action",
        ));
    }

    let new_action = match status {
        DecisionStatus::Allow => command.allowed_action.clone(),
        DecisionStatus::Abort => vec![0.0; command.allowed_action.len()],
        DecisionStatus::Modify => {
            let new_action = allowed_action.expect("checked");
            if new_action.len() != command.allowed_action.len() {
                return Err(KernelError::validation(
                    "narrow.action",
                    "must preserve action dimensionality",
                ));
            }
            let widened = new_action
                .iter()
                .zip(command.allowed_action.iter())
                .any(|(v, old)| !v.is_finite() || v.abs() > old.abs() + 1e-12);
            if widened {
                return Err(KernelError::validation(
                    "narrow.action",
                    "Governor modify may only narrow the certified action envelope",
                ));
            }
            new_action
        }
        _ => unreachable!(),
    };

    let mut next = command;
    if next.issuer_allowed_action.is_empty() {
        next.issuer_allowed_action = next.allowed_action.clone();
    }
    if next.issuer_certificate_status.is_empty() {
        next.issuer_certificate_status = next.certificate.status.as_str().into();
    }
    if next.issuer_physical_reason.is_empty() {
        next.issuer_physical_reason = next.certificate.physical_reason.clone();
    }
    next.allowed_action = new_action;
    next.certificate.status = status;
    if let Some(r) = reason {
        next.certificate.physical_reason = r;
    }
    Ok(next)
}
