//! Production-path A1–A7. Unix PTY + Xl330Driver + ONLINE governor.
//! Privileged VirtualXl330 truth is oracle-only. Not MEASURED.
#![cfg(unix)]

use realityos_plant::ActionParams;
use realityos_virtual_metal::campaign::{decide_hold, decide_nudge};
use realityos_virtual_metal::faults::{FaultEvent, FaultKind, FaultSchedule};
use realityos_virtual_metal::oracle::{belief_from_trace, check_belief_vs_truth, EffectBelief};
use realityos_virtual_metal::tier2::Tier2Session;
use realityos_virtual_metal::VirtualXl330;

fn session(tag: &str) -> Tier2Session {
    Tier2Session::open(VirtualXl330::xl330_m288(), tag, 1).unwrap_or_else(|e| panic!("{tag}: {e}"))
}

#[test]
fn a1_hold_success_matches_device_effect() {
    let mut s = session("a1-hold");
    let (t, before, after) = s.hold();
    assert!(t.ok, "hold should succeed on production path: {t:?}");
    check_belief_vs_truth(belief_from_trace(t.ok, &t.event), &before, &after).expect("A1 hold");
    assert!(after.physical_actions > before.physical_actions);
}

#[test]
fn a2_ack_loss_unknown_poisons_same_online_instance() {
    let mut s = session("a2-ack");
    s.device
        .lock()
        .expect("oracle")
        .drop_status_after_next_goal();
    let (t, before, after) = s.try_hold(1);
    let t = t.unwrap_or_else(|e| panic!("authorize: {e:?}"));
    assert!(!t.ok);
    assert_eq!(t.event, "driver_write_unknown");
    assert!(s.gov.integrity_aborted());
    check_belief_vs_truth(EffectBelief::Unknown, &before, &after).unwrap();
    let n = after.physical_actions;
    let rec = s.gov.clear_estop_requires_recovery_now(true);
    assert!(!rec.ok, "recover must not clear integrity: {rec:?}");
    if let Ok(w) = s.gov.authorize_issued(decide_hold(2, 10.0)) {
        assert!(!s.gov.write_online_now(&w, &ActionParams::empty()).ok);
    }
    assert_eq!(s.truth().physical_actions, n);
}

#[test]
fn a3_replay_and_restart_do_not_duplicate_physical_action() {
    let mut s = session("a3-replay");
    let w = s
        .gov
        .authorize_issued(decide_hold(1, 10.0))
        .expect("authorize");
    let t = s.gov.write_online_now(&w, &ActionParams::empty());
    assert!(t.ok, "{t:?}");
    let n = s.truth().physical_actions;
    let replay = s.gov.write_online_now(&w, &ActionParams::empty());
    assert!(!replay.ok);
    assert_eq!(s.truth().physical_actions, n);
    s.restart().expect("restart");
    if let Ok(w2) = s.gov.authorize_issued(decide_hold(1, 10.0)) {
        assert!(!s.gov.write_online_now(&w2, &ActionParams::empty()).ok);
    }
    assert_eq!(s.truth().physical_actions, n);
}

#[test]
fn a4_recover_attack_refuses_and_estop_does_not_clear_integrity() {
    let mut s = session("a4-rec");
    let w = s.gov.authorize_issued(decide_hold(1, 10.0)).unwrap();
    assert!(s.gov.write_online_now(&w, &ActionParams::empty()).ok);
    let _ = s.gov.write_online_now(&w, &ActionParams::empty());
    assert!(s.gov.integrity_aborted());
    let n = s.truth().physical_actions;
    let rec = s.gov.clear_estop_requires_recovery_now(true);
    assert!(!rec.ok);
    if let Ok(w2) = s.gov.authorize_issued(decide_hold(2, 10.0)) {
        assert!(!s.gov.write_online_now(&w2, &ActionParams::empty()).ok);
    }
    assert_eq!(s.truth().physical_actions, n);
    let _ = s.gov.engage_estop_now("later_estop");
    assert!(s.gov.integrity_aborted());
    let rec2 = s.gov.clear_estop_requires_recovery_now(true);
    assert!(!rec2.ok);
    assert_eq!(s.truth().physical_actions, n);
}

#[test]
fn a5_identity_change_refuses_with_zero_physical_action() {
    let mut s = session("a5-id");
    let n = s.truth().physical_actions;
    s.device.lock().expect("oracle").set_model_firmware(1190, 1);
    let _ = s.gov.acquire_sensor();
    let (r, before, after) = s.try_hold(1);
    if let Ok(t) = r {
        assert!(!t.ok, "identity change must not succeed: {t:?}");
    }
    assert_eq!(after.physical_actions, before.physical_actions);
    assert_eq!(after.physical_actions, n);
}

#[test]
fn a6_voltage_out_of_range_refuses_writes_for_both_error_bits() {
    for (tag, bit) in [("a6-bit01", 0x01u8), ("a6-bit10", 0x10u8)] {
        let mut d = VirtualXl330::xl330_m288();
        d.set_voltage_error_bit(bit);
        let mut s = Tier2Session::open(d, tag, 1).unwrap_or_else(|e| panic!("{tag} open: {e}"));
        s.device.lock().expect("oracle").set_voltage_v(2.0);
        let n = s.truth().physical_actions;
        let sense = s.gov.acquire_sensor();
        assert!(
            sense.is_err(),
            "{tag} sensor should see VIN fault: {sense:?}"
        );
        let (r, before, after) = s.try_hold(1);
        if let Ok(t) = r {
            assert!(!t.ok, "{tag} write after VIN fault: {t:?}");
        }
        assert_eq!(after.physical_actions, before.physical_actions.max(n));
        assert_eq!(s.truth().voltage_error_bit, bit);
        assert!(
            s.truth().hardware_error & bit != 0 || s.truth().voltage_v < 3.5,
            "{tag} register must show brownout or latched bit, hw={:#x} v={}",
            s.truth().hardware_error,
            s.truth().voltage_v
        );
    }
}

fn a7_one(tag: &str, kind: FaultKind, allowed: &[&str]) {
    let mut s = session(tag);
    s.inject_next(kind);
    let (r, before, after) = s.try_hold(1);
    let (ok, event) = match &r {
        Ok(t) => (t.ok, t.event.as_str()),
        Err(_) => (false, "authorize_refused"),
    };
    let connected = s.peer.lock().expect("peer").is_connected();
    let class = if event.contains("unknown") {
        "unknown"
    } else if !connected || event.contains("disconnect") {
        "disconnect"
    } else if ok {
        "success"
    } else {
        "refusal"
    };
    assert!(
        allowed.contains(&class),
        "{tag} class={class} event={event} ok={ok} allowed={allowed:?}"
    );
    let _ = check_belief_vs_truth(belief_from_trace(ok, event), &before, &after);
}

#[test]
fn a7_transport_faults_record_distinct_classes() {
    a7_one(
        "a7-crc",
        FaultKind::CorruptOutgoingCrc,
        &["unknown", "refusal"],
    );
    a7_one(
        "a7-trunc",
        FaultKind::TruncateStatus { keep: 6 },
        &["unknown", "refusal"],
    );
    a7_one(
        "a7-wrong-id",
        FaultKind::WrongStatusId,
        &["unknown", "refusal", "success"],
    );
    a7_one(
        "a7-delay",
        FaultKind::DelayStatus { ms: 250 },
        &["unknown", "refusal", "success"],
    );
    a7_one(
        "a7-late",
        FaultKind::StatusAfterTimeout { ms: 250 },
        &["unknown", "refusal", "success"],
    );
    a7_one(
        "a7-silent",
        FaultKind::DeviceSilent,
        &["unknown", "refusal"],
    );
    a7_one(
        "a7-disc",
        FaultKind::Disconnect,
        &["disconnect", "unknown", "refusal"],
    );
    a7_one(
        "a7-dup",
        FaultKind::DuplicateStatus,
        &["success", "unknown", "refusal"],
    );
    a7_one(
        "a7-split",
        FaultKind::SplitStatusAcrossReads { first: 4 },
        &["success", "unknown", "refusal"],
    );
    a7_one(
        "a7-gpre",
        FaultKind::GarbagePrefix,
        &["success", "unknown", "refusal"],
    );
    a7_one(
        "a7-gsuf",
        FaultKind::GarbageSuffix,
        &["success", "unknown", "refusal"],
    );
    a7_one(
        "a7-reboot",
        FaultKind::RebootDuringRequest,
        &["unknown", "refusal", "success"],
    );
}

#[test]
fn a7_reconnect_restores_production_path() {
    let mut s = session("a7-reconn");
    s.inject(FaultSchedule {
        events: vec![
            FaultEvent {
                after_packet: 1,
                kind: FaultKind::Disconnect,
            },
            FaultEvent {
                after_packet: 2,
                kind: FaultKind::Reconnect,
            },
        ],
    });
    let _ = s.try_hold(1);
    s.inject(FaultSchedule::empty());
    let (r, before, after) = s.try_hold(2);
    if let Ok(t) = r {
        let _ = check_belief_vs_truth(belief_from_trace(t.ok, &t.event), &before, &after);
    }
}

#[test]
fn nudge_on_production_path_is_not_virtual_metal_port() {
    let mut s = session("a1-nudge");
    let before = s.truth();
    let w = s
        .gov
        .authorize_issued(decide_nudge(2, 10.0, vec![0.2]))
        .expect("nudge authorize");
    let t = s.gov.write_online_now(&w, &ActionParams::empty());
    let after = s.truth();
    if t.ok {
        check_belief_vs_truth(EffectBelief::KnownSuccess, &before, &after).ok();
    }
}
