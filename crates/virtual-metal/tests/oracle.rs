//! A1 belief-vs-truth is a pure comparison. Does not talk to PTY.

use realityos_virtual_metal::oracle::{
    belief_from_trace, check_belief_vs_truth, DeviceTruth, EffectBelief,
};

fn truth(actions: u64) -> DeviceTruth {
    DeviceTruth {
        physical_actions: actions,
        present: 2048,
        goal: 2048,
        torque: true,
        model: 1200,
        firmware: 46,
        voltage_v: 5.0,
        hardware_error: 0,
        voltage_error_bit: 1,
        reset_count: 0,
        tx_count: 0,
        rx_count: 0,
    }
}

#[test]
fn known_refusal_while_device_applied_is_a1_failure() {
    let err = check_belief_vs_truth(EffectBelief::KnownRefusal, &truth(1), &truth(2)).unwrap_err();
    assert!(err.contains("a1_false_no_effect"), "{err}");
}

#[test]
fn known_success_while_device_idle_is_a1_failure() {
    let err = check_belief_vs_truth(EffectBelief::KnownSuccess, &truth(3), &truth(3)).unwrap_err();
    assert!(err.contains("a1_false_happened"), "{err}");
}

#[test]
fn unknown_allows_either_effect() {
    assert!(check_belief_vs_truth(EffectBelief::Unknown, &truth(0), &truth(1)).is_ok());
    assert!(check_belief_vs_truth(EffectBelief::Unknown, &truth(4), &truth(4)).is_ok());
}

#[test]
fn known_success_requires_action_delta() {
    assert!(check_belief_vs_truth(EffectBelief::KnownSuccess, &truth(0), &truth(1)).is_ok());
}

#[test]
fn known_refusal_requires_no_delta() {
    assert!(check_belief_vs_truth(EffectBelief::KnownRefusal, &truth(2), &truth(2)).is_ok());
}

#[test]
fn unknown_event_maps_to_unknown_belief() {
    assert_eq!(
        belief_from_trace(false, "driver_write_unknown"),
        EffectBelief::Unknown
    );
    assert_eq!(
        belief_from_trace(true, "driver_write"),
        EffectBelief::KnownSuccess
    );
    assert_eq!(
        belief_from_trace(false, "driver_write_refused"),
        EffectBelief::KnownRefusal
    );
}
