//! Authority + virtual-device invariants on shipped governor/plant paths.

use std::cell::RefCell;
use std::rc::Rc;

use realityos_plant::ActionParams;
use realityos_virtual_metal::{
    decide_hold, decide_nudge, start_gov, start_gov_opts, VirtualMetalPort, VirtualXl330,
};

fn pair(
    tag: &str,
) -> (
    Rc<RefCell<VirtualXl330>>,
    realityos_governor::RuntimeGovernor<
        realityos_plant::HardwareBackedPlant<VirtualMetalPort>,
        realityos_governor::OnlineLocked,
    >,
) {
    let d = Rc::new(RefCell::new(VirtualXl330::xl330_m288()));
    let port = VirtualMetalPort::new(d.clone());
    let g = start_gov(port, tag, 1);
    (d, g)
}

#[test]
fn replay_does_not_cause_second_physical_action() {
    let (d, mut g) = pair("inv-replay");
    let w = g.authorize_issued(decide_hold(1, 10.0)).unwrap();
    assert!(g.write_online_now(&w, &ActionParams::empty()).ok);
    let n = d.borrow().physical_action_count();
    assert!(n >= 1);
    let replay = g.write_online_now(&w, &ActionParams::empty());
    assert!(!replay.ok);
    assert_eq!(d.borrow().physical_action_count(), n);
    assert!(g.integrity_aborted());
}

#[test]
fn unknown_outcome_same_instance_refuses_further_actuation() {
    let (d, mut g) = pair("inv-unk");
    d.borrow_mut().drop_status_after_next_goal();
    let w = g.authorize_issued(decide_hold(1, 10.0)).unwrap();
    let t = g.write_online_now(&w, &ActionParams::empty());
    assert!(!t.ok);
    assert_eq!(t.event, "driver_write_unknown");
    assert!(g.integrity_aborted());
    let n = d.borrow().physical_action_count();
    assert!(n >= 1);
    if let Ok(w2) = g.authorize_issued(decide_hold(2, 10.0)) {
        assert!(!g.write_online_now(&w2, &ActionParams::empty()).ok);
    }
    assert_eq!(d.borrow().physical_action_count(), n);
}

#[test]
fn recover_after_integrity_refuses_and_physical_count_unchanged() {
    let (d, mut g) = pair("inv-rec");
    let w = g.authorize_issued(decide_hold(1, 10.0)).unwrap();
    assert!(g.write_online_now(&w, &ActionParams::empty()).ok);
    let _ = g.write_online_now(&w, &ActionParams::empty());
    assert!(g.integrity_aborted());
    let n = d.borrow().physical_action_count();
    let rec = g.clear_estop_requires_recovery_now(true);
    assert!(!rec.ok);
    assert!(rec
        .violations
        .iter()
        .any(|v| v == "integrity_abort_requires_online_restart"));
    if let Ok(w2) = g.authorize_issued(decide_hold(2, 10.0)) {
        assert!(!g.write_online_now(&w2, &ActionParams::empty()).ok);
    }
    assert_eq!(d.borrow().physical_action_count(), n);
    let _ = g.engage_estop_now("later_estop");
    assert!(g.integrity_aborted());
    assert!(!g.clear_estop_requires_recovery_now(true).ok);
    assert_eq!(d.borrow().physical_action_count(), n);
}

#[test]
fn identity_mismatch_refuses_actuation() {
    let d = Rc::new(RefCell::new(VirtualXl330::xl330_m288()));
    let port = VirtualMetalPort::new(d.clone());
    let mut g = start_gov(port, "inv-id", 1);
    d.borrow_mut().set_model_firmware(1190, 1);
    let n = d.borrow().physical_action_count();
    if let Ok(w) = g.authorize_issued(decide_hold(1, 10.0)) {
        let t = g.write_online_now(&w, &ActionParams::empty());
        assert!(!t.ok);
    }
    assert_eq!(d.borrow().physical_action_count(), n);
}

#[test]
fn nudge_moves_present_by_nonzero_ticks() {
    use realityos_plant::ActuationCommand;
    let (d, mut g) = pair("inv-nudge");
    let before = d.borrow().present_position();
    let w = g
        .authorize_issued(decide_nudge(1, 10.0, vec![5.0]))
        .unwrap();
    assert!(
        w.as_command()
            .allowed_action()
            .first()
            .copied()
            .unwrap_or(0.0)
            .abs()
            > 0.5,
        "nudge command must carry a non-zero action, got {:?}",
        w.as_command().allowed_action()
    );
    assert!(g.write_online_now(&w, &ActionParams::empty()).ok);
    assert_eq!(
        d.borrow().present_position(),
        before,
        "nudge must not teleport present without time advance"
    );
    assert_eq!(d.borrow().goal_position(), before + 32);
    d.borrow_mut().advance(0.05);
    let after = d.borrow().present_position();
    assert_ne!(after, before, "nudge must change present after advance");
    assert_eq!(after, before + 32);
}

#[test]
fn restart_does_not_duplicate_spent_command() {
    let d = Rc::new(RefCell::new(VirtualXl330::xl330_m288()));
    let port = VirtualMetalPort::new(d.clone());
    let mut g = start_gov(port, "inv-restart", 7);
    let w = g.authorize_issued(decide_hold(1, 10.0)).unwrap();
    assert!(g.write_online_now(&w, &ActionParams::empty()).ok);
    let n = d.borrow().physical_action_count();
    assert!(n >= 1);
    drop(g);
    let port = VirtualMetalPort::new(d.clone());
    let mut g2 = start_gov_opts(port, "inv-restart", 7, false);
    if let Ok(w2) = g2.authorize_issued(decide_hold(1, 10.0)) {
        let t = g2.write_online_now(&w2, &ActionParams::empty());
        assert!(!t.ok, "restart must not re-execute spent command_id: {t:?}");
    }
    assert_eq!(
        d.borrow().physical_action_count(),
        n,
        "restart duplicated a previously applied command"
    );
}
