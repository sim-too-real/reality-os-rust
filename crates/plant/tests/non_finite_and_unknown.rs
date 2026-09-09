use proptest::prelude::*;
use realityos_kernel::{CommandOutcome, DecisionStatus};
use realityos_plant::{
    execute_certified_command, ActionParams, ActuationCommand, CommandLedger, ExecuteBind, Plant,
    SimPlant,
};

struct Cmd {
    id: String,
    action: Vec<f64>,
    ack: bool,
}

impl ActuationCommand for Cmd {
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

fn empty_bind<'a>() -> ExecuteBind<'a> {
    ExecuteBind {
        expected_release_hash: None,
        expected_calibration_ids: &[],
        expected_sensor_packet_hash: None,
        require_sensor_packet_hash: false,
        require_monotonic_sequence: false,
        require_signature: false,
        signing_key: None,
        force_online_rails: false,
    }
}

proptest! {
    #[test]
    fn non_finite_never_writes(x in prop_oneof![Just(f64::NAN), Just(f64::INFINITY), Just(f64::NEG_INFINITY)]) {
        let mut p = SimPlant::new("p", 1, 10.0);
        let mut ledger = CommandLedger::new();
        let cmd = Cmd { id: "nf".into(), action: vec![x], ack: true };
        let out = execute_certified_command(&mut p, &cmd, &ActionParams::empty(), &mut ledger, 1.0, &empty_bind());
        prop_assert!(!out.ok);
        prop_assert_eq!(p.write_count(), 0);
        prop_assert_eq!(out.outcome, CommandOutcome::Refused);
    }
}

#[test]
fn unknown_is_not_retryable_after_prepare() {
    let mut ledger = CommandLedger::new();
    let cmd = Cmd {
        id: "u1".into(),
        action: vec![0.1],
        ack: true,
    };
    ledger.prepare(&cmd).unwrap();
    ledger.mark_unknown(&cmd).unwrap();
    assert!(!CommandOutcome::Unknown.may_retry());
    assert!(!ledger.command_phase("u1").may_attempt_write());
}
