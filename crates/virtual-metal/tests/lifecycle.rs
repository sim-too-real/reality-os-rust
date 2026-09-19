//! Write-lifecycle loss → UNKNOWN; same ONLINE instance cannot actuate further.

use std::cell::RefCell;
use std::rc::Rc;

use realityos_plant::ActionParams;
use realityos_virtual_metal::faults::{FaultSchedule, LifecycleLoss, WriteLifecycleBoundary};
use realityos_virtual_metal::{
    decide_hold, start_gov, LifecycleInject, VirtualMetalPort, VirtualXl330,
};

fn assert_unknown_stops(
    tag: &str,
    boundary: WriteLifecycleBoundary,
    loss: LifecycleLoss,
    expect_applied: bool,
) {
    let d = Rc::new(RefCell::new(VirtualXl330::xl330_m288()));
    let mut port = VirtualMetalPort::new(d.clone());
    port.inject_lifecycle(LifecycleInject { boundary, loss });
    let mut g = start_gov(port, tag, 11);
    let w = g.authorize_issued(decide_hold(1, 10.0)).unwrap();
    let t = g.write_online_now(&w, &ActionParams::empty());
    assert!(!t.ok, "{tag} must not succeed: {t:?}");
    assert_eq!(t.event, "driver_write_unknown");
    assert!(g.integrity_aborted(), "{tag} must integrity-abort");
    let n = d.borrow().physical_action_count();
    if expect_applied {
        assert!(n >= 1, "{tag} should have applied before loss, n={n}");
    }
    if let Ok(w2) = g.authorize_issued(decide_hold(2, 10.0)) {
        assert!(
            !g.write_online_now(&w2, &ActionParams::empty()).ok,
            "{tag} same ONLINE instance must not actuate further"
        );
    }
    assert_eq!(d.borrow().physical_action_count(), n);
}

#[test]
fn ack_loss_after_apply_is_unknown() {
    assert_unknown_stops(
        "life-ack",
        WriteLifecycleBoundary::AfterDeviceApply,
        LifecycleLoss::AckLoss,
        true,
    );
}

#[test]
fn crash_after_serial_write_is_unknown() {
    assert_unknown_stops(
        "life-crash-serial",
        WriteLifecycleBoundary::AfterSerialWrite,
        LifecycleLoss::ProcessCrash,
        true,
    );
}

#[test]
fn loss_before_apply_is_unknown_and_does_not_actuate() {
    assert_unknown_stops(
        "life-before-apply",
        WriteLifecycleBoundary::BeforeDeviceApply,
        LifecycleLoss::SerialLoss,
        false,
    );
}

#[test]
fn crash_before_packet_construction_is_unknown() {
    assert_unknown_stops(
        "life-before-pkt",
        WriteLifecycleBoundary::BeforePacketConstruction,
        LifecycleLoss::ProcessCrash,
        false,
    );
}

#[test]
fn ledger_boundary_after_apply_is_unknown() {
    assert_unknown_stops(
        "life-ledger",
        WriteLifecycleBoundary::AfterLedgerAppend,
        LifecycleLoss::ProcessCrash,
        true,
    );
}

#[test]
fn shrink_drops_irrelevant_faults() {
    let sched = FaultSchedule {
        events: vec![
            realityos_virtual_metal::faults::FaultEvent {
                after_packet: 1,
                kind: realityos_virtual_metal::FaultKind::GarbageSuffix,
            },
            realityos_virtual_metal::faults::FaultEvent {
                after_packet: 2,
                kind: realityos_virtual_metal::FaultKind::DropStatusAfterApply,
            },
            realityos_virtual_metal::faults::FaultEvent {
                after_packet: 3,
                kind: realityos_virtual_metal::FaultKind::GarbagePrefix,
            },
        ],
    };
    let mini = sched.shrink(|s| {
        s.events.iter().any(|e| {
            matches!(
                e.kind,
                realityos_virtual_metal::FaultKind::DropStatusAfterApply
            )
        })
    });
    assert_eq!(mini.events.len(), 1);
}
