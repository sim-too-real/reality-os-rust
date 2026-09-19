use realityos_virtual_metal::faults::{FaultKind, FaultSchedule};
use realityos_virtual_metal::tier2::{promote_to_tier2, Tier1Counterexample};

#[test]
fn promote_skips_non_serial_failures() {
    let cx = Tier1Counterexample {
        seed: 7,
        fault_sequence: FaultSchedule::empty(),
        realization: serde_json::json!({"kind": "policy"}),
        production_path_relevant: false,
        note: "governor-only".into(),
    };
    assert!(promote_to_tier2(&cx).is_none());
}

#[test]
fn promote_serial_fault_for_tier2_replay() {
    let cx = Tier1Counterexample {
        seed: 9,
        fault_sequence: FaultSchedule::once(FaultKind::CorruptOutgoingCrc),
        realization: serde_json::json!({"voltage_v": 5.0}),
        production_path_relevant: true,
        note: "crc on the wire".into(),
    };
    let spec = promote_to_tier2(&cx).expect("serial fault is production-path relevant");
    assert_eq!(spec.seed, 9);
    assert_eq!(spec.fault_sequence, cx.fault_sequence);
}
