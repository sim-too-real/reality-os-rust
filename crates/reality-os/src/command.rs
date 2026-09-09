use realityos_kernel::{deadline_from_ttl, DecisionStatus, KernelError, KernelResult, MonoTime};
use realityos_plant::ActuationCommand;
use serde::{Deserialize, Serialize};

use crate::certificate::Certificate;

/// The ONE object that may cross from Reality OS into a Governor/driver.
/// Immutable after construction. Narrowing yields a derived command.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CertifiedCommand {
    command_id: String,
    sequence: i64,
    issued_at_s: f64,
    expires_at_s: f64,
    certificate: Certificate,
    allowed_action: Vec<f64>,
    #[serde(default)]
    release_hash: String,
    #[serde(default)]
    as_built_hash: String,
    #[serde(default)]
    calibration_ids: Vec<String>,
    #[serde(default, skip_deserializing)]
    acknowledged: bool,
    #[serde(default)]
    issuer_allowed_action: Vec<f64>,
    #[serde(default)]
    issuer_certificate_status: String,
    #[serde(default)]
    issuer_physical_reason: String,
    #[serde(default)]
    sensor_snapshot_id: String,
    #[serde(default)]
    sensor_packet_hash: String,
    #[serde(default)]
    belief_snapshot_id: String,
    #[serde(default)]
    payload_hash: String,
    #[serde(default)]
    signature: String,
    #[serde(default)]
    signer: String,
    #[serde(default)]
    signing_scheme: String,
    #[serde(default)]
    actuator_ids: Vec<String>,
    #[serde(default)]
    waypoints: Vec<Vec<f64>>,
    #[serde(default)]
    parent_payload_hash: String,
    #[serde(default)]
    mode: String,
    #[serde(default)]
    frame_id: String,
    #[serde(default)]
    units: String,
    #[serde(default)]
    policy_hash: String,
    #[serde(default)]
    config_hash: String,
}

impl CertifiedCommand {
    pub(crate) fn issue(
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
        let now = MonoTime::from_secs(now_s)
            .map_err(|_| KernelError::validation("command.time", "non-finite"))?;
        let expires = deadline_from_ttl(now, ttl_s)?;
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
            issued_at_s: now.secs(),
            expires_at_s: expires.secs(),
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
            parent_payload_hash: String::new(),
            mode: String::new(),
            frame_id: String::new(),
            units: String::new(),
            policy_hash: String::new(),
            config_hash: String::new(),
        })
    }

    pub fn command_id(&self) -> &str {
        &self.command_id
    }
    pub fn sequence_value(&self) -> i64 {
        self.sequence
    }
    pub fn certificate(&self) -> &Certificate {
        &self.certificate
    }
    pub fn allowed_action_vec(&self) -> &[f64] {
        &self.allowed_action
    }
    pub fn is_acknowledged(&self) -> bool {
        self.acknowledged
    }
    pub fn actuator_ids(&self) -> &[String] {
        &self.actuator_ids
    }
    pub fn sensor_packet_hash(&self) -> &str {
        &self.sensor_packet_hash
    }
    pub fn release_hash(&self) -> &str {
        &self.release_hash
    }
    pub fn is_expired(&self, now_s: f64) -> bool {
        now_s.is_finite() && now_s > self.expires_at_s
    }

    fn invalidate_signature(&mut self) {
        self.signature.clear();
        self.payload_hash.clear();
        self.signing_scheme.clear();
        self.signer.clear();
    }

    pub fn with_identity(
        mut self,
        release_hash: impl Into<String>,
        as_built: impl Into<String>,
        calibration_id: impl Into<String>,
    ) -> Self {
        self.release_hash = release_hash.into();
        self.as_built_hash = as_built.into();
        let cal = calibration_id.into();
        if !cal.is_empty() {
            self.calibration_ids = vec![cal];
        }
        self.invalidate_signature();
        self
    }

    pub fn with_sensor_packet_hash(mut self, hash: impl Into<String>) -> Self {
        self.sensor_packet_hash = hash.into();
        self.invalidate_signature();
        self
    }

    pub fn with_actuator_ids(mut self, ids: Vec<String>) -> Self {
        self.actuator_ids = ids;
        self.invalidate_signature();
        self
    }

    pub fn with_mode(mut self, mode: impl Into<String>) -> Self {
        self.mode = mode.into();
        self.invalidate_signature();
        self
    }

    pub(crate) fn sign(mut self, key: &[u8]) -> Self {
        let hash = realityos_plant::command_payload_hash(&self);
        self.payload_hash = hash.clone();
        self.signature = realityos_plant::sign_payload(key, &hash);
        self.signing_scheme = realityos_plant::SIGNING_SCHEME.into();
        self.signer = "realityos".into();
        if self.issuer_allowed_action.is_empty() {
            self.issuer_allowed_action = self.allowed_action.clone();
        }
        self
    }

    pub(crate) fn acknowledge(mut self) -> Self {
        self.acknowledged = true;
        self
    }

    /// Bind empty identity fields only. Never overwrite a foreign hash.
    pub(crate) fn bind_identity(
        mut self,
        release_hash: &str,
        as_built: &str,
        calibration_id: &str,
    ) -> Result<Self, Vec<String>> {
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
        let mut mutated = false;
        if self.release_hash.is_empty() {
            self.release_hash = release_hash.into();
            mutated = true;
        }
        if self.as_built_hash.is_empty() {
            self.as_built_hash = as_built.into();
            mutated = true;
        }
        if self.calibration_ids.is_empty() && !calibration_id.is_empty() {
            self.calibration_ids = vec![calibration_id.into()];
            mutated = true;
        }
        if mutated {
            if !self.signature.is_empty() {
                return Err(vec!["cannot_bind_identity_after_sign".into()]);
            }
            self.invalidate_signature();
        }
        Ok(self)
    }

    pub(crate) fn bind_evidence(mut self, expected_hash: &str) -> Result<Self, Vec<String>> {
        if expected_hash.is_empty() {
            return Err(vec!["missing_expected_sensor_packet_hash".into()]);
        }
        if !self.sensor_packet_hash.is_empty() && self.sensor_packet_hash != expected_hash {
            return Err(vec!["foreign_sensor_packet_hash".into()]);
        }
        if self.sensor_packet_hash.is_empty() {
            if !self.signature.is_empty() {
                return Err(vec!["cannot_bind_evidence_after_sign".into()]);
            }
            self.sensor_packet_hash = expected_hash.into();
            self.invalidate_signature();
        }
        Ok(self)
    }

    /// HMAC-sign and acknowledge. Not an ONLINE execution capability.
    /// `RuntimeGovernor<OnlineLocked>` wraps the result in `OnlineWrite`.
    pub fn seal_online(self, key: &[u8]) -> Result<Self, Vec<String>> {
        if key.is_empty() {
            return Err(vec!["online_signing_key_missing".into()]);
        }
        if self.acknowledged {
            return Err(vec!["cannot_seal_acknowledged_command".into()]);
        }
        Ok(self.sign(key).acknowledge())
    }
}

/// Kernel-issued, unsigned command. Only [`crate::RealityOs::decide`] mints this
/// for ordinary consumers. Not an execution capability.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct IssuedCommand {
    command: CertifiedCommand,
}

impl IssuedCommand {
    pub(crate) fn from_certified(command: CertifiedCommand) -> Option<Self> {
        if command.is_acknowledged() || !command.signature().is_empty() {
            return None;
        }
        if !command.certificate().allowed() {
            return None;
        }
        Some(Self { command })
    }

    pub fn command_id(&self) -> &str {
        self.command.command_id()
    }

    pub fn sequence_value(&self) -> i64 {
        self.command.sequence_value()
    }

    pub fn is_acknowledged(&self) -> bool {
        self.command.is_acknowledged()
    }

    pub fn as_command(&self) -> &CertifiedCommand {
        &self.command
    }

    pub fn into_command(self) -> CertifiedCommand {
        self.command
    }

    /// Bind governor-owned identity, evidence, and actuator ids.
    /// Still unsigned. The ONLINE governor then seals with its private key.
    pub fn bind_online(
        self,
        release_hash: &str,
        as_built: &str,
        calibration_id: &str,
        evidence_hash: &str,
        actuator_ids: Vec<String>,
    ) -> Result<CertifiedCommand, Vec<String>> {
        if evidence_hash.is_empty() {
            return Err(vec!["online_requires_sensor_hash".into()]);
        }
        if actuator_ids.is_empty() {
            return Err(vec!["online_requires_actuator_ids".into()]);
        }
        if self.command.is_acknowledged() {
            return Err(vec!["issued_command_already_acknowledged".into()]);
        }
        if !self.command.signature().is_empty() {
            return Err(vec!["issued_command_already_signed".into()]);
        }
        let cmd = self
            .command
            .bind_identity(release_hash, as_built, calibration_id)?;
        let cmd = cmd.bind_evidence(evidence_hash)?;
        if cmd.actuator_ids().is_empty() {
            Ok(cmd.with_actuator_ids(actuator_ids))
        } else if !cmd
            .actuator_ids()
            .iter()
            .all(|id| actuator_ids.iter().any(|a| a == id))
        {
            Err(vec!["foreign_actuator_ids".into()])
        } else {
            Ok(cmd)
        }
    }
}

impl From<IssuedCommand> for CertifiedCommand {
    fn from(issued: IssuedCommand) -> Self {
        issued.into_command()
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
    fn parent_payload_hash(&self) -> &str {
        &self.parent_payload_hash
    }
    fn mode(&self) -> &str {
        &self.mode
    }
    fn frame_id(&self) -> &str {
        &self.frame_id
    }
    fn units(&self) -> &str {
        &self.units
    }
    fn policy_hash(&self) -> &str {
        &self.policy_hash
    }
    fn config_hash(&self) -> &str {
        &self.config_hash
    }
}

/// Governor may allow, narrow, or abort — never invent a new plan or upgrade a refuse.
/// Returns a derived command linked by parent hash. Does not mutate the signed parent.
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
            if !realityos_plant::action_within_issuer_envelope(&new_action, &command.allowed_action)
            {
                return Err(KernelError::validation(
                    "narrow.action",
                    "Governor modify may only narrow the certified action envelope",
                ));
            }
            new_action
        }
        _ => unreachable!(),
    };

    let parent = if command.payload_hash.is_empty() {
        realityos_plant::command_payload_hash(&command)
    } else {
        command.payload_hash.clone()
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
    next.parent_payload_hash = parent;
    next.allowed_action = new_action;
    next.certificate.status = status;
    if let Some(r) = reason {
        next.certificate.physical_reason = r;
    }
    next.payload_hash.clear();
    next.signature.clear();
    Ok(next)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn issue_rejects_non_positive_ttl() {
        let cert = Certificate::new(DecisionStatus::Allow, "ok");
        assert!(CertifiedCommand::issue("c", 1, 1.0, 0.0, cert.clone(), vec![0.1]).is_err());
        assert!(CertifiedCommand::issue("c", 1, 1.0, f64::NAN, cert, vec![0.1]).is_err());
    }

    #[test]
    fn narrow_rejects_sign_reversal() {
        let cert = Certificate::new(DecisionStatus::Allow, "ok");
        let cmd = CertifiedCommand::issue("c", 1, 0.0, 10.0, cert, vec![0.5]).unwrap();
        let err = narrow_certified_command(cmd, DecisionStatus::Modify, Some(vec![-0.1]), None)
            .unwrap_err();
        assert!(err.to_string().contains("only narrow"));
    }

    #[test]
    fn deserialize_cannot_ack() {
        let cert = Certificate::new(DecisionStatus::Allow, "ok");
        let cmd = CertifiedCommand::issue("c", 1, 0.0, 10.0, cert, vec![0.5])
            .unwrap()
            .acknowledge();
        let raw = serde_json::to_string(&cmd).unwrap();
        let back: CertifiedCommand = serde_json::from_str(&raw).unwrap();
        assert!(!back.is_acknowledged());
    }

    #[test]
    fn bind_after_sign_refuses() {
        let cert = Certificate::new(DecisionStatus::Allow, "ok");
        let cmd = CertifiedCommand::issue("c", 1, 0.0, 10.0, cert, vec![0.5])
            .unwrap()
            .sign(b"key");
        let err = cmd.bind_identity("rel", "", "cal").unwrap_err();
        assert!(err.iter().any(|e| e.contains("after_sign")));
    }
}

/// Test/demo constructors. Not part of the production certifier path.
/// Enable with feature `fixtures`. Production `decide()` uses crate-private issue.
#[cfg(any(test, feature = "fixtures"))]
pub mod fixture {
    use super::*;

    pub fn issue(
        command_id: impl Into<String>,
        sequence: i64,
        now_s: f64,
        ttl_s: f64,
        certificate: Certificate,
        allowed_action: Vec<f64>,
    ) -> KernelResult<CertifiedCommand> {
        CertifiedCommand::issue(
            command_id,
            sequence,
            now_s,
            ttl_s,
            certificate,
            allowed_action,
        )
    }

    pub fn acknowledge(command: CertifiedCommand) -> CertifiedCommand {
        command.acknowledge()
    }
}
