//! Typed fail-closed codes. String messages stay for journals; codes are the API.

use crate::layer::Layer;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ViolationCode {
    IncompleteRuntimeIdentity,
    IncompleteOnlineIdentity,
    EstopEngaged,
    AbortLatched,
    HeartbeatStale,
    SensorNeverMarked,
    SensorStale,
    SensorEmpty,
    SensorTimestampRequired,
    ReleaseHashMismatch,
    ReleaseHashMissing,
    AsBuiltMismatch,
    CalibrationMismatch,
    CommandNotAcknowledged,
    CertificateNotExecutable,
    EnvelopeExceeded,
    EnvelopeIncomplete,
    EnvelopeMissingAction,
    EnvelopeNonFinite,
    Replay,
    Expired,
    MissingCommandId,
    MissingAllowedAction,
    NonFiniteAction,
    SequenceNotMonotonic,
    SensorPacketHashMismatch,
    SensorPacketHashMissing,
    SignatureMismatch,
    CommandUnsigned,
    UncertifiedOnlineWrite,
    DriverDisconnected,
    DriverException,
    OperatorAckRequired,
    SafeStateLatched,
    ForeignIdentity,
    OnlineRailOptOut,
    OnlineSimPlaceholder,
    LearnedActuator,
    ForbiddenTool,
    PixelsNeverEntered,
    SceneNotCompiled,
    UnsupportedPlanKind,
    ActuatorUndeclared,
    WorkspaceUndeclared,
    PflUnknownRegion,
    PflExceeded,
    PhysicsBound,
    NonFinite,
    FieldbusNotAttached,
    EgressDisabled,
    NamedHole,
}

impl ViolationCode {
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::IncompleteRuntimeIdentity => "incomplete_runtime_identity",
            Self::IncompleteOnlineIdentity => "incomplete_online_runtime_identity",
            Self::EstopEngaged => "estop_engaged",
            Self::AbortLatched => "abort_latched",
            Self::HeartbeatStale => "heartbeat_missing_or_stale",
            Self::SensorNeverMarked => "sensor_never_marked",
            Self::SensorStale => "sensor_stale",
            Self::SensorEmpty => "sensor_reading_empty",
            Self::SensorTimestampRequired => "sensor_timestamp_required",
            Self::ReleaseHashMismatch => "release_hash_mismatch_command_vs_session",
            Self::ReleaseHashMissing => "command_missing_release_hash",
            Self::AsBuiltMismatch => "as_built_hash_mismatch_command_vs_session",
            Self::CalibrationMismatch => "calibration_id_not_on_command",
            Self::CommandNotAcknowledged => "command_not_acknowledged",
            Self::CertificateNotExecutable => "command_certificate_not_executable",
            Self::EnvelopeExceeded => "envelope_action_exceeds_max_abs",
            Self::EnvelopeIncomplete => "envelope_pack_incomplete",
            Self::EnvelopeMissingAction => "envelope_missing_allowed_action",
            Self::EnvelopeNonFinite => "envelope_non_finite_allowed_action",
            Self::Replay => "replayed command_id",
            Self::Expired => "command expired",
            Self::MissingCommandId => "missing command_id",
            Self::MissingAllowedAction => "missing_allowed_action",
            Self::NonFiniteAction => "non_finite_allowed_action",
            Self::SequenceNotMonotonic => "sequence not monotonic",
            Self::SensorPacketHashMismatch => "sensor_packet_hash mismatch",
            Self::SensorPacketHashMissing => "command missing sensor_packet_hash",
            Self::SignatureMismatch => "signature_mismatch",
            Self::CommandUnsigned => "command_unsigned",
            Self::UncertifiedOnlineWrite => "online_plant_act_requires_execute_certified_command",
            Self::DriverDisconnected => "driver_not_connected",
            Self::DriverException => "driver_exception",
            Self::OperatorAckRequired => "operator_ack_required",
            Self::SafeStateLatched => "dispatch_safe_state_latched",
            Self::ForeignIdentity => "foreign_release_hash",
            Self::OnlineRailOptOut => "online_refuses_safety_rail_opt_out",
            Self::OnlineSimPlaceholder => "online_requires_real_serial_or_as_built",
            Self::LearnedActuator => "learned_source_is_proposal_only",
            Self::ForbiddenTool => "forbidden_tool_name",
            Self::PixelsNeverEntered => "pixels_never_entered",
            Self::SceneNotCompiled => "compiled_this_decide_false",
            Self::UnsupportedPlanKind => "unsupported_plan_kind",
            Self::ActuatorUndeclared => "actuator_undeclared",
            Self::WorkspaceUndeclared => "workspace_undeclared",
            Self::PflUnknownRegion => "pfl_unknown_region",
            Self::PflExceeded => "pfl_exceeded",
            Self::PhysicsBound => "physics_bound",
            Self::NonFinite => "non_finite",
            Self::FieldbusNotAttached => "fieldbus_not_attached",
            Self::EgressDisabled => "egress_disabled_use_runtime_governor",
            Self::NamedHole => "named_hole",
        }
    }

    pub fn parse_known(raw: &str) -> Option<Self> {
        Self::ALL
            .iter()
            .copied()
            .find(|c| c.as_str() == raw || raw.starts_with(c.as_str()))
    }

    pub const ALL: [Self; 51] = [
        Self::IncompleteRuntimeIdentity,
        Self::IncompleteOnlineIdentity,
        Self::EstopEngaged,
        Self::AbortLatched,
        Self::HeartbeatStale,
        Self::SensorNeverMarked,
        Self::SensorStale,
        Self::SensorEmpty,
        Self::SensorTimestampRequired,
        Self::ReleaseHashMismatch,
        Self::ReleaseHashMissing,
        Self::AsBuiltMismatch,
        Self::CalibrationMismatch,
        Self::CommandNotAcknowledged,
        Self::CertificateNotExecutable,
        Self::EnvelopeExceeded,
        Self::EnvelopeIncomplete,
        Self::EnvelopeMissingAction,
        Self::EnvelopeNonFinite,
        Self::Replay,
        Self::Expired,
        Self::MissingCommandId,
        Self::MissingAllowedAction,
        Self::NonFiniteAction,
        Self::SequenceNotMonotonic,
        Self::SensorPacketHashMismatch,
        Self::SensorPacketHashMissing,
        Self::SignatureMismatch,
        Self::CommandUnsigned,
        Self::UncertifiedOnlineWrite,
        Self::DriverDisconnected,
        Self::DriverException,
        Self::OperatorAckRequired,
        Self::SafeStateLatched,
        Self::ForeignIdentity,
        Self::OnlineRailOptOut,
        Self::OnlineSimPlaceholder,
        Self::LearnedActuator,
        Self::ForbiddenTool,
        Self::PixelsNeverEntered,
        Self::SceneNotCompiled,
        Self::UnsupportedPlanKind,
        Self::ActuatorUndeclared,
        Self::WorkspaceUndeclared,
        Self::PflUnknownRegion,
        Self::PflExceeded,
        Self::PhysicsBound,
        Self::NonFinite,
        Self::FieldbusNotAttached,
        Self::EgressDisabled,
        Self::NamedHole,
    ];
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Violation {
    pub code: ViolationCode,
    pub layer: Layer,
    pub detail: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub correlation_id: Option<String>,
}

impl Violation {
    pub fn new(layer: Layer, code: ViolationCode) -> Self {
        Self {
            code,
            layer,
            detail: String::new(),
            correlation_id: None,
        }
    }

    pub fn detail(mut self, detail: impl Into<String>) -> Self {
        self.detail = detail.into();
        self
    }

    pub fn corr(mut self, id: impl Into<String>) -> Self {
        self.correlation_id = Some(id.into());
        self
    }

    pub fn as_journal_str(&self) -> String {
        if self.detail.is_empty() {
            self.code.as_str().to_string()
        } else if self.detail.starts_with(self.code.as_str()) {
            self.detail.clone()
        } else {
            format!("{}:{}", self.code.as_str(), self.detail)
        }
    }
}

impl std::fmt::Display for Violation {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}:{}", self.layer, self.as_journal_str())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_code_has_stable_string() {
        for c in ViolationCode::ALL {
            assert!(!c.as_str().is_empty());
        }
    }
}
