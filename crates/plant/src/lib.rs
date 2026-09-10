//! Plant trait, command ledger, certified-write uniqueness.
//!
//! Governor depends on this crate. Reality OS issues commands that implement
//! [`ActuationCommand`]. No invent. No learned last-write.

pub mod backed;
pub mod caps;
pub mod command;
pub mod consume;
pub mod dynamics;
pub mod egress;
pub mod error;
pub mod execute;
pub mod fieldbus;
pub mod harness;
pub mod hil_faults;
pub mod ledger;
pub mod safety_protocol;
pub mod signing;
pub mod sim;
pub mod traits;
mod write_guard;

pub use backed::HardwareBackedPlant;
pub use caps::{check_hard_action_bounds, ActionParams, PlantCaps, PlantRealized};
pub use command::{ActuationCommand, ExecuteBind};
pub use consume::{ConsumePhase, CrashPoint};
pub use dynamics::{AnalyticIntegrator, DynamicsBackend, DynamicsState, MujocoBackend};
pub use egress::{CommandEgress, RecordingCommandEgress, RefuseCommandEgress};
pub use error::{PlantError, PlantResult};
pub use execute::{execute_certified_command, ExecuteResult};
pub use fieldbus::{FieldbusKind, FieldbusLink, LinkState};
pub use harness::SimulatedHardwarePort;
pub use ledger::{set_test_journal_write_delay_ms, CommandLedger, ContinuityState};
pub use safety_protocol::{SafeTransition, SafetyFrame};
pub use signing::{
    action_within_issuer_envelope, command_payload_hash, sign_payload, signature_violations,
    signing_key_hash, SIGNING_SCHEME,
};
pub use sim::SimPlant;
pub use traits::{
    hash_sensor_packet, hash_sensor_samples, HardwareDriverPort, HardwareIdentity, Plant,
    SensorPacket, HARNESS_EVIDENCE,
};
pub use write_guard::{plant_requires_certified_write, refuse_uncertified_online_write};

#[cfg(test)]
mod tests {
    use super::*;
    use realityos_kernel::{CommandOutcome, DecisionStatus};

    struct AllowCmd {
        id: String,
        action: Vec<f64>,
        ack: bool,
    }

    impl ActuationCommand for AllowCmd {
        fn command_id(&self) -> &str {
            &self.id
        }
        fn sequence(&self) -> i64 {
            1
        }
        fn issued_at_s(&self) -> f64 {
            0.0
        }
        fn expires_at_s(&self) -> f64 {
            1e9
        }
        fn allowed_action(&self) -> &[f64] {
            &self.action
        }
        fn issuer_allowed_action(&self) -> &[f64] {
            &self.action
        }
        fn certificate_status(&self) -> DecisionStatus {
            DecisionStatus::Allow
        }
        fn issuer_certificate_status(&self) -> &str {
            "allow"
        }
        fn issuer_physical_reason(&self) -> &str {
            "ok"
        }
        fn release_hash(&self) -> &str {
            "rel"
        }
        fn as_built_hash(&self) -> &str {
            ""
        }
        fn calibration_ids(&self) -> &[String] {
            &[]
        }
        fn acknowledged(&self) -> bool {
            self.ack
        }
        fn sensor_snapshot_id(&self) -> &str {
            ""
        }
        fn sensor_packet_hash(&self) -> &str {
            ""
        }
        fn belief_snapshot_id(&self) -> &str {
            ""
        }
        fn payload_hash(&self) -> &str {
            ""
        }
        fn signature(&self) -> &str {
            ""
        }
        fn signer(&self) -> &str {
            ""
        }
        fn signing_scheme(&self) -> &str {
            ""
        }
        fn actuator_ids(&self) -> &[String] {
            &[]
        }
    }

    #[test]
    fn online_act_without_scope_refuses() {
        let mut p = SimPlant::new("stub", 1, 1.0);
        p.go_online();
        let err = p.act(&[0.1], &ActionParams::empty()).unwrap_err();
        match err {
            PlantError::UncertifiedOnline { method } => assert_eq!(method, "act"),
            other => panic!("{other:?}"),
        }
        assert_eq!(p.write_count(), 0);
    }

    #[test]
    fn execute_certified_command_is_the_only_online_write() {
        let mut p = SimPlant::new("stub", 1, 1.0);
        p.go_online();
        let mut ledger = CommandLedger::new();
        let cmd = AllowCmd {
            id: "c1".into(),
            action: vec![0.2],
            ack: true,
        };
        let bind = ExecuteBind {
            expected_release_hash: None,
            expected_calibration_ids: &[],
            expected_sensor_packet_hash: None,
            require_sensor_packet_hash: false,
            require_monotonic_sequence: false,
            require_signature: false,
            signing_key: None,
            force_online_rails: false,
        };
        let out = execute_certified_command(
            &mut p,
            &cmd,
            &ActionParams::empty(),
            &mut ledger,
            1.0,
            &bind,
        );
        assert!(out.ok);
        assert_eq!(out.outcome, CommandOutcome::Executed);
        assert_eq!(p.write_count(), 1);
        assert!(!out.realized.unwrap().metal);
    }

    struct ExternalStylePort;

    impl HardwareDriverPort for ExternalStylePort {
        fn probe_identity(&self) -> HardwareIdentity {
            HardwareIdentity::harness("EXT-1")
        }
        fn read_sensor(&mut self, now_s: f64) -> PlantResult<SensorPacket> {
            Ok(SensorPacket::from_samples(vec![("x".into(), 0.0)], now_s))
        }
        fn write_action(
            &mut self,
            action: &[f64],
            _params: &ActionParams,
        ) -> PlantResult<PlantRealized> {
            Ok(PlantRealized::sim([("n".into(), action.len() as f64)]))
        }
        fn engage_hw_estop(&mut self, _reason: &str) {}
        fn clear_hw_estop(&mut self, _operator_ack: bool) -> PlantResult<()> {
            Ok(())
        }
        fn is_connected(&self) -> bool {
            true
        }
    }

    #[test]
    fn external_port_wraps_but_cannot_skip_certified_write() {
        let mut plant = HardwareBackedPlant::new(ExternalStylePort, "ext", 1, 1.0);
        let err = plant.act(&[0.1], &ActionParams::empty()).unwrap_err();
        match err {
            PlantError::UncertifiedOnline { method } => assert_eq!(method, "act"),
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn production_lock_refuses_unsigned_even_if_bind_opts_out() {
        let mut p = SimPlant::new("stub", 1, 1.0);
        p.lock_production(b"prod-key");
        let mut ledger = CommandLedger::new();
        let cmd = AllowCmd {
            id: "c-prod".into(),
            action: vec![0.2],
            ack: true,
        };
        let bind = ExecuteBind {
            expected_release_hash: None,
            expected_calibration_ids: &[],
            expected_sensor_packet_hash: None,
            require_sensor_packet_hash: false,
            require_monotonic_sequence: false,
            require_signature: false,
            signing_key: None,
            force_online_rails: false,
        };
        let out = execute_certified_command(
            &mut p,
            &cmd,
            &ActionParams::empty(),
            &mut ledger,
            1.0,
            &bind,
        );
        assert!(!out.ok);
        assert_eq!(p.write_count(), 0);
    }

    #[test]
    fn replay_after_unknown_is_refused() {
        let mut ledger = CommandLedger::new();
        let cmd = AllowCmd {
            id: "c-unknown".into(),
            action: vec![0.1],
            ack: true,
        };
        ledger.prepare(&cmd).unwrap();
        ledger.mark_unknown(&cmd).unwrap();
        let v = ledger.check(&cmd, 1.0, None, &[], None, false, false);
        assert!(v.iter().any(|s| s.contains("replayed")));
        assert!(!CommandOutcome::Unknown.may_retry());
    }
}
