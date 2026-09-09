//! Driver-boundary view of a command. Governor and plant depend on this trait,
//! not on Reality OS types (DIP).

use realityos_kernel::DecisionStatus;

/// The only object shape that may cross into `execute_certified_command`.
pub trait ActuationCommand {
    fn command_id(&self) -> &str;
    fn sequence(&self) -> i64;
    fn issued_at_s(&self) -> f64;
    fn expires_at_s(&self) -> f64;
    fn allowed_action(&self) -> &[f64];
    fn issuer_allowed_action(&self) -> &[f64];
    fn certificate_status(&self) -> DecisionStatus;
    fn issuer_certificate_status(&self) -> &str;
    fn issuer_physical_reason(&self) -> &str;
    fn release_hash(&self) -> &str;
    fn as_built_hash(&self) -> &str;
    fn calibration_ids(&self) -> &[String];
    fn acknowledged(&self) -> bool;
    fn sensor_snapshot_id(&self) -> &str;
    fn sensor_packet_hash(&self) -> &str;
    fn belief_snapshot_id(&self) -> &str;
    fn payload_hash(&self) -> &str;
    fn signature(&self) -> &str;
    fn signer(&self) -> &str;
    fn signing_scheme(&self) -> &str;
    fn actuator_ids(&self) -> &[String];
    fn follow_waypoints(&self) -> Option<&[Vec<f64>]> {
        None
    }
    fn parent_payload_hash(&self) -> &str {
        ""
    }
    fn mode(&self) -> &str {
        ""
    }
    fn frame_id(&self) -> &str {
        ""
    }
    fn units(&self) -> &str {
        ""
    }
    fn policy_hash(&self) -> &str {
        ""
    }
    fn config_hash(&self) -> &str {
        ""
    }
}

#[derive(Debug, Clone)]
pub struct ExecuteBind<'a> {
    pub expected_release_hash: Option<&'a str>,
    pub expected_calibration_ids: &'a [String],
    pub expected_sensor_packet_hash: Option<&'a str>,
    pub require_sensor_packet_hash: bool,
    pub require_monotonic_sequence: bool,
    pub require_signature: bool,
    pub signing_key: Option<&'a [u8]>,
}
