//! Seeded Virtual Metal campaign. Deterministic. Not MEASURED.

use std::path::PathBuf;
use std::sync::{Arc, Mutex};

use realityos_core::{DecideRequest, Intent, PolicyProposal, RealityOs, WorldView};
use realityos_governor::{OnlineLocked, RuntimeGovernor, RuntimeIdentity};
use realityos_kernel::{
    AuthorityClock, CalibrationId, DesignContentHash, FakeClock, FirmwareId, ReleaseHash,
    SerialOrAsBuilt,
};
use realityos_metal::protocol::{
    decode_status, encode_ping, encode_write, ADDR_GOAL_POSITION, ADDR_TORQUE_ENABLE,
};
use realityos_plant::{ActionParams, HardwareDriverPort};

use crate::device::VirtualXl330;
use crate::evidence::{CampaignRecord, CampaignRow, CAMPAIGN_SCHEMA, VERDICT_FAIL, VERDICT_PASS};
use crate::faults::{
    corrupt_crc_bytes, FaultKind, FaultSchedule, LifecycleLoss, WriteLifecycleBoundary,
};
use crate::peer::VirtualSerialPeer;
use crate::port::{LifecycleInject, VirtualMetalPort, DEFAULT_ACTUATOR};
use crate::truth_pack::{Xl330TruthPack, TRUTH_PACK_SCHEMA};

fn splitmix(mut x: u64) -> u64 {
    x = x.wrapping_add(0x9E37_79B9_7F4A_7C15);
    let mut z = x;
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
}

pub fn decide_hold(seq: i64, now_s: f64) -> realityos_core::IssuedCommand {
    let mut ros = RealityOs::new();
    let mut req = DecideRequest::new(
        Intent::language("hold", "hold"),
        WorldView {
            tau_max: vec![5.0],
            ..WorldView::default()
        },
        now_s,
    );
    req.sequence = seq;
    req.command_id = format!("vm-cmd-{seq}-{now_s}");
    ros.decide(req).command.expect("allow")
}

pub fn decide_nudge(seq: i64, now_s: f64, action: Vec<f64>) -> realityos_core::IssuedCommand {
    let mut ros = RealityOs::new();
    let mut req = DecideRequest::new(
        Intent::language("nudge", "hold"),
        WorldView {
            tau_max: vec![5.0],
            ..WorldView::default()
        },
        now_s,
    );
    req.sequence = seq;
    req.command_id = format!("vm-nudge-{seq}-{now_s}");
    req.proposal = Some(PolicyProposal::operator(action, "virtual-metal-nudge"));
    ros.decide(req).command.expect("allow")
}

fn vm_identity(port: &VirtualMetalPort) -> RuntimeIdentity {
    let (serial, fw, cal, design) = port.runtime_identity_fields();
    RuntimeIdentity {
        release_hash: ReleaseHash::new("vm-rel-1").unwrap(),
        design_content_hash: Some(DesignContentHash::new(design).unwrap()),
        serial_or_as_built: Some(SerialOrAsBuilt::new(serial).unwrap()),
        firmware_id: Some(FirmwareId::new(fw).unwrap()),
        calibration_id: Some(CalibrationId::new(cal).unwrap()),
    }
}

fn journal_path(tag: &str, seed: u64) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("realityos-vm-{tag}-{seed}"));
    let _ = std::fs::create_dir_all(&dir);
    dir.join("driver.jsonl")
}

pub fn start_gov(
    port: VirtualMetalPort,
    tag: &str,
    seed: u64,
) -> RuntimeGovernor<HardwarePlant, OnlineLocked> {
    start_gov_opts(port, tag, seed, true)
}

type HardwarePlant = realityos_plant::HardwareBackedPlant<VirtualMetalPort>;

pub fn start_gov_opts(
    port: VirtualMetalPort,
    tag: &str,
    seed: u64,
    first_online: bool,
) -> RuntimeGovernor<HardwarePlant, OnlineLocked> {
    let path = journal_path(tag, seed);
    if first_online {
        if let Some(dir) = path.parent() {
            let _ = std::fs::remove_dir_all(dir);
            let _ = std::fs::create_dir_all(dir);
        }
    }
    let id = vm_identity(&port);
    let plant = HardwarePlant::new(port, "xl330-vm", 1, 5.0);
    let clock: Arc<dyn AuthorityClock> = FakeClock::arc(10.0);
    let mut g = RuntimeGovernor::<HardwarePlant, OnlineLocked>::new_online(
        id,
        plant,
        path,
        b"virtual-metal-key".to_vec(),
        first_online,
        vec![DEFAULT_ACTUATOR.into()],
        clock,
    )
    .expect("virtual metal online");
    g.acquire_sensor().expect("sensor");
    g
}

fn realization_of(d: &VirtualXl330) -> serde_json::Value {
    serde_json::json!({
        "voltage_v": d.voltage_v(),
        "gearbox_efficiency": d.gearbox_efficiency(),
        "boot_latency_s": d.boot_latency_s(),
        "transport_latency_s": d.transport_latency_s(),
        "voltage_error_bit": d.voltage_error_bit(),
        "present_ticks": d.present_position(),
        "goal_ticks": d.goal_position(),
    })
}

fn row(
    instance: u64,
    seed: u64,
    scenario: &str,
    category: &str,
    violations: Vec<String>,
    actions: u64,
    faults: serde_json::Value,
    d: &VirtualXl330,
) -> CampaignRow {
    CampaignRow {
        instance,
        seed,
        scenario: scenario.into(),
        category: category.into(),
        verdict: if violations.is_empty() {
            VERDICT_PASS.into()
        } else {
            VERDICT_FAIL.into()
        },
        physical_actions: actions,
        invariant_violations: violations,
        fault_sequence: faults,
        truth_pack_schema: TRUTH_PACK_SCHEMA.into(),
        truth_pack_content_hash: d.truth_pack().content_hash(),
        realization: realization_of(d),
        tier2: None,
    }
}

fn seeded_device(seed: u64) -> VirtualXl330 {
    VirtualXl330::from_seed(seed, Xl330TruthPack::xl330_m288())
}

fn scenario_boot(seed: u64, i: u64) -> CampaignRow {
    let mut d = seeded_device(seed);
    let ping = encode_ping(d.id());
    let st = decode_status(&d.process(&ping)).unwrap();
    let mut v = Vec::new();
    if st.params.len() < 3 {
        v.push("ping_identity_short".into());
    }
    if d.model() != 1200 {
        v.push(format!("model:{}", d.model()));
    }
    let t5 = d.stall_torque_nm_at(5.0).unwrap_or(0.0);
    if (t5 - 0.52).abs() > 1e-9 {
        v.push(format!("stall_5v:{t5}"));
    }
    let n5 = d.no_load_speed_rpm_at(5.0).unwrap_or(0.0);
    if (n5 - 103.0).abs() > 1e-12 {
        v.push(format!("noload_5v:{n5}"));
    }
    let actions = d.physical_action_count();
    row(
        i,
        seed,
        "boot_identity_sensor",
        "boot/setup",
        v,
        actions,
        serde_json::json!([]),
        &d,
    )
}

fn scenario_hold_nudge(seed: u64, i: u64) -> CampaignRow {
    let d = Arc::new(Mutex::new(seeded_device(seed)));
    let port = VirtualMetalPort::new(d.clone());
    let mut g = start_gov(port, "hold", seed);
    let w = g.authorize_issued(decide_hold(1, 10.0)).unwrap();
    let t = g.write_online_now(&w, &ActionParams::empty());
    let mut v = Vec::new();
    if !t.ok {
        v.push(format!("hold_refused:{:?}", t.violations));
    }
    d.lock().expect("virtual xl330").advance(0.02);
    let before_nudge = d.lock().expect("virtual xl330").present_position();
    let nw = g
        .authorize_issued(decide_nudge(2, 10.0, vec![5.0]))
        .unwrap();
    let nt = g.write_online_now(&nw, &ActionParams::empty());
    if !nt.ok {
        v.push(format!("nudge_refused:{:?}", nt.violations));
    }
    let present_at_write = d.lock().expect("virtual xl330").present_position();
    if present_at_write != before_nudge {
        v.push("nudge_teleported_without_advance".into());
    }
    d.lock().expect("virtual xl330").advance(0.05);
    if d.lock().expect("virtual xl330").present_position() == before_nudge {
        v.push("nudge_did_not_move_present".into());
    }
    let before = d.lock().expect("virtual xl330").physical_action_count();
    let replay = g.write_online_now(&w, &ActionParams::empty());
    if replay.ok {
        v.push("replay_executed".into());
    }
    if d.lock().expect("virtual xl330").physical_action_count() != before {
        v.push("replay_second_physical_action".into());
    }
    let snapshot = d.lock().expect("virtual xl330").clone();
    row(
        i,
        seed,
        "hold_nudge_replay",
        "hold",
        v,
        snapshot.physical_action_count(),
        serde_json::json!([]),
        &snapshot,
    )
}

fn scenario_unknown(seed: u64, i: u64) -> CampaignRow {
    let d = Arc::new(Mutex::new(seeded_device(seed)));
    let port = VirtualMetalPort::new(d.clone());
    let mut g = start_gov(port, "unk", seed);
    d.lock()
        .expect("virtual xl330")
        .drop_status_after_next_goal();
    let w = g.authorize_issued(decide_hold(1, 10.0)).unwrap();
    let t = g.write_online_now(&w, &ActionParams::empty());
    let mut v = Vec::new();
    if t.ok {
        v.push("unknown_path_succeeded".into());
    }
    if !g.integrity_aborted() {
        v.push("unknown_did_not_integrity_abort".into());
    }
    let actions = d.lock().expect("virtual xl330").physical_action_count();
    let rec = g.clear_estop_requires_recovery_now(true);
    if rec.ok {
        v.push("recover_cleared_integrity".into());
    }
    if let Ok(w2) = g.authorize_issued(decide_hold(2, 10.0)) {
        let t2 = g.write_online_now(&w2, &ActionParams::empty());
        if t2.ok {
            v.push("fresh_command_after_unknown".into());
        }
    }
    if d.lock().expect("virtual xl330").physical_action_count() != actions {
        v.push("unknown_then_extra_physical".into());
    }
    let snapshot = d.lock().expect("virtual xl330").clone();
    row(
        i,
        seed,
        "ack_lost_unknown",
        "unknown outcome",
        v,
        snapshot.physical_action_count(),
        serde_json::json!([{"after_packet":2,"kind":"drop_status_after_apply"}]),
        &snapshot,
    )
}

fn scenario_recover(seed: u64, i: u64) -> CampaignRow {
    let d = Arc::new(Mutex::new(seeded_device(seed)));
    let port = VirtualMetalPort::new(d.clone());
    let mut g = start_gov(port, "rec", seed);
    let w = g.authorize_issued(decide_hold(1, 10.0)).unwrap();
    let _ = g.write_online_now(&w, &ActionParams::empty());
    let _ = g.write_online_now(&w, &ActionParams::empty());
    let mut v = Vec::new();
    if !g.integrity_aborted() {
        v.push("replay_no_integrity".into());
    }
    let actions = d.lock().expect("virtual xl330").physical_action_count();
    let rec = g.clear_estop_requires_recovery_now(true);
    if rec.ok {
        v.push("recover_after_integrity".into());
    }
    let _ = g.engage_estop_now("later_estop");
    if !g.integrity_aborted() {
        v.push("estop_downgraded_integrity".into());
    }
    let rec2 = g.clear_estop_requires_recovery_now(true);
    if rec2.ok {
        v.push("estop_cleared_integrity".into());
    }
    if d.lock().expect("virtual xl330").physical_action_count() != actions {
        v.push("recover_extra_physical".into());
    }
    let snapshot = d.lock().expect("virtual xl330").clone();
    row(
        i,
        seed,
        "recover_integrity_estop",
        "recover attack",
        v,
        snapshot.physical_action_count(),
        serde_json::json!([]),
        &snapshot,
    )
}

fn scenario_identity(seed: u64, i: u64) -> CampaignRow {
    let d = Arc::new(Mutex::new(seeded_device(seed)));
    let port = VirtualMetalPort::new(d.clone());
    let mut g = start_gov(port, "id", seed);
    g.acquire_sensor().ok();
    d.lock().expect("virtual xl330").set_model_firmware(1190, 1);
    let mut v = Vec::new();
    if let Ok(w) = g.authorize_issued(decide_hold(1, 10.0)) {
        let t = g.write_online_now(&w, &ActionParams::empty());
        if t.ok {
            v.push("identity_swap_actuated".into());
        }
    }
    if d.lock().expect("virtual xl330").physical_action_count() != 0 {
        v.push("identity_swap_physical".into());
    }
    let snapshot = d.lock().expect("virtual xl330").clone();
    row(
        i,
        seed,
        "identity_swap",
        "identity swap",
        v,
        snapshot.physical_action_count(),
        serde_json::json!([]),
        &snapshot,
    )
}

fn scenario_torque_off_goal(seed: u64, i: u64) -> CampaignRow {
    let mut d = seeded_device(seed);
    let start = d.present_position();
    let goal = (start + 200).clamp(0, 4095);
    let wst = d.process(&encode_write(
        d.id(),
        ADDR_GOAL_POSITION,
        &goal.to_le_bytes(),
    ));
    let mut v = Vec::new();
    if decode_status(&wst).is_err() {
        v.push("goal_write_no_status".into());
    }
    if d.goal_position() != goal {
        v.push("goal_not_stored_with_torque_off".into());
    }
    if d.present_position() != start {
        v.push("torque_off_goal_moved_present".into());
    }
    if d.physical_action_count() != 0 {
        v.push("torque_off_counted_physical".into());
    }
    d.advance(0.05);
    if d.present_position() != start {
        v.push("torque_off_advance_moved".into());
    }
    row(
        i,
        seed,
        "goal_with_torque_off",
        "goal with torque off",
        v,
        d.physical_action_count(),
        serde_json::json!([]),
        &d,
    )
}

fn scenario_torque_on_pending(seed: u64, i: u64) -> CampaignRow {
    let mut d = seeded_device(seed);
    let start = d.present_position();
    let goal = (start + 200).clamp(0, 4095);
    let _ = d.process(&encode_write(
        d.id(),
        ADDR_GOAL_POSITION,
        &goal.to_le_bytes(),
    ));
    let _ = d.process(&encode_write(d.id(), ADDR_TORQUE_ENABLE, &[1]));
    let mut v = Vec::new();
    if d.present_position() != start {
        v.push("torque_on_teleported".into());
    }
    if d.goal_position() != goal {
        v.push("pending_goal_lost".into());
    }
    d.advance(0.2);
    if d.present_position() == start {
        v.push("torque_on_pending_did_not_move".into());
    }
    row(
        i,
        seed,
        "torque_on_pending_goal_apply",
        "torque-on pending-goal apply",
        v,
        d.physical_action_count(),
        serde_json::json!([]),
        &d,
    )
}

fn scenario_transport_crc(seed: u64, i: u64) -> CampaignRow {
    let d = Arc::new(Mutex::new(seeded_device(seed)));
    let mut peer = VirtualSerialPeer::new(d.clone());
    peer.set_transport_schedule(FaultSchedule::once(FaultKind::CorruptOutgoingCrc));
    let status = peer.exchange(&encode_ping(1));
    let mut v = Vec::new();
    match decode_status(&status) {
        Ok(_) => v.push("corrupt_crc_still_decoded".into()),
        Err(e) => {
            if e.to_string() != "dxl_bad_crc" {
                v.push(format!("corrupt_crc_wrong_error:{e}"));
            }
        }
    }
    if status.len() >= 2 {
        let clean = encode_ping(1);
        let _ = clean;
        let last = *status.last().unwrap();
        let uncorrupted = corrupt_crc_bytes(status.clone());
        if uncorrupted.last() == Some(&last) {
            v.push("crc_byte_not_mutated".into());
        }
    }
    let snapshot = d.lock().expect("virtual xl330").clone();
    row(
        i,
        seed,
        "transport_corrupt_crc",
        "transport corruption",
        v,
        snapshot.physical_action_count(),
        serde_json::json!([{"kind":"corrupt_outgoing_crc"}]),
        &snapshot,
    )
}

fn scenario_watchdog(seed: u64, i: u64) -> CampaignRow {
    let mut d = seeded_device(seed);
    let _ = d.process(&encode_write(d.id(), ADDR_TORQUE_ENABLE, &[1]));
    d.trip_watchdog();
    let goal = d.present_position().saturating_add(10).clamp(0, 4095);
    let st = decode_status(&d.process(&encode_write(
        d.id(),
        ADDR_GOAL_POSITION,
        &goal.to_le_bytes(),
    )));
    let mut v = Vec::new();
    match st {
        Ok(s) if s.error & !0x80 == 0 => v.push("watchdog_allowed_goal".into()),
        Ok(_) | Err(_) => {}
    }
    row(
        i,
        seed,
        "watchdog_trip",
        "watchdog",
        v,
        d.physical_action_count(),
        serde_json::json!([{"kind":"watchdog_trip"}]),
        &d,
    )
}

fn scenario_voltage(seed: u64, i: u64) -> CampaignRow {
    let mut d = seeded_device(seed);
    d.set_voltage_v(2.0);
    let st = decode_status(&d.process(&encode_ping(d.id()))).unwrap();
    let mut v = Vec::new();
    if st.error & 0x80 == 0 {
        v.push("voltage_fault_no_alert".into());
    }
    if d.hardware_error() == 0 {
        v.push("voltage_fault_no_hwerr".into());
    }
    let bit = d.voltage_error_bit();
    if d.hardware_error() & bit == 0 {
        v.push(format!(
            "voltage_bit_mismatch:hw={:#x}:bit={:#x}",
            d.hardware_error(),
            bit
        ));
    }
    row(
        i,
        seed,
        "voltage_fault",
        "voltage fault",
        v,
        d.physical_action_count(),
        serde_json::json!([{"kind":"voltage_out_of_range"}]),
        &d,
    )
}

fn scenario_temp(seed: u64, i: u64) -> CampaignRow {
    let mut d = seeded_device(seed);
    d.set_temperature_c(90.0);
    let st = decode_status(&d.process(&encode_ping(d.id()))).unwrap();
    let mut v = Vec::new();
    if st.error & 0x80 == 0 {
        v.push("temp_fault_no_alert".into());
    }
    row(
        i,
        seed,
        "temperature_fault",
        "temperature fault",
        v,
        d.physical_action_count(),
        serde_json::json!([{"kind":"over_temperature"}]),
        &d,
    )
}

fn scenario_reboot(seed: u64, i: u64) -> CampaignRow {
    let mut d = seeded_device(seed);
    let id = d.id();
    let _ = d.process(&encode_write(id, ADDR_TORQUE_ENABLE, &[1]));
    let _ = d.process(&realityos_metal::protocol::encode_reboot(id));
    let mut v = Vec::new();
    if d.torque_enabled() {
        v.push("reboot_left_torque_on".into());
    }
    if d.id() != id {
        v.push("reboot_cleared_eeprom_id".into());
    }
    row(
        i,
        seed,
        "reboot",
        "reboot",
        v,
        d.physical_action_count(),
        serde_json::json!([]),
        &d,
    )
}

fn scenario_disconnect(seed: u64, i: u64) -> CampaignRow {
    let d = Arc::new(Mutex::new(seeded_device(seed)));
    let mut port = VirtualMetalPort::new(d.clone());
    port.disconnect();
    let mut v = Vec::new();
    if port.read_sensor(0.0).is_ok() {
        v.push("disconnect_sensor_ok".into());
    }
    let snapshot = d.lock().expect("virtual xl330").clone();
    row(
        i,
        seed,
        "disconnect",
        "disconnect",
        v,
        snapshot.physical_action_count(),
        serde_json::json!([{"kind":"disconnect"}]),
        &snapshot,
    )
}

fn scenario_lifecycle_ack(seed: u64, i: u64) -> CampaignRow {
    let d = Arc::new(Mutex::new(seeded_device(seed)));
    let mut port = VirtualMetalPort::new(d.clone());
    port.inject_lifecycle(LifecycleInject {
        boundary: WriteLifecycleBoundary::AfterDeviceApply,
        loss: LifecycleLoss::AckLoss,
    });
    let mut g = start_gov(port, "life", seed);
    let w = g.authorize_issued(decide_hold(1, 10.0)).unwrap();
    let t = g.write_online_now(&w, &ActionParams::empty());
    let mut v = Vec::new();
    if t.ok {
        v.push("lifecycle_ack_loss_succeeded".into());
    }
    if !g.integrity_aborted() {
        v.push("lifecycle_no_integrity".into());
    }
    let n = d.lock().expect("virtual xl330").physical_action_count();
    if let Ok(w2) = g.authorize_issued(decide_hold(2, 10.0)) {
        if g.write_online_now(&w2, &ActionParams::empty()).ok {
            v.push("lifecycle_continued_after_unknown".into());
        }
    }
    if d.lock().expect("virtual xl330").physical_action_count() != n {
        v.push("lifecycle_extra_physical".into());
    }
    let snapshot = d.lock().expect("virtual xl330").clone();
    row(
        i,
        seed,
        "lost_ack_after_apply",
        "lost ACK",
        v,
        snapshot.physical_action_count(),
        serde_json::json!([{"boundary":"after_device_apply","loss":"ack_loss"}]),
        &snapshot,
    )
}

fn scenario_restart(seed: u64, i: u64) -> CampaignRow {
    let d = Arc::new(Mutex::new(seeded_device(seed)));
    let port = VirtualMetalPort::new(d.clone());
    let mut g = start_gov(port, "rst", seed);
    let w = g.authorize_issued(decide_hold(1, 10.0)).unwrap();
    let mut v = Vec::new();
    if !g.write_online_now(&w, &ActionParams::empty()).ok {
        v.push("restart_setup_write_failed".into());
    }
    let n = d.lock().expect("virtual xl330").physical_action_count();
    drop(g);
    let port = VirtualMetalPort::new(d.clone());
    let mut g2 = start_gov_opts(port, "rst", seed, false);
    if let Ok(w2) = g2.authorize_issued(decide_hold(1, 10.0)) {
        if g2.write_online_now(&w2, &ActionParams::empty()).ok {
            v.push("restart_reexecuted".into());
        }
    }
    if d.lock().expect("virtual xl330").physical_action_count() != n {
        v.push("restart_duplicated_physical".into());
    }
    let snapshot = d.lock().expect("virtual xl330").clone();
    row(
        i,
        seed,
        "restart",
        "restart",
        v,
        snapshot.physical_action_count(),
        serde_json::json!([]),
        &snapshot,
    )
}

fn scenario_late_ack(seed: u64, i: u64) -> CampaignRow {
    let d = Arc::new(Mutex::new(seeded_device(seed)));
    let mut port = VirtualMetalPort::new(d.clone());
    port.peer()
        .borrow_mut()
        .set_transport_schedule(FaultSchedule {
            events: vec![crate::faults::FaultEvent {
                after_packet: 2,
                kind: FaultKind::DelayStatus { ms: 250 },
            }],
        });
    let mut v = Vec::new();
    match port.write_action(&[0.0], &ActionParams::empty()) {
        Err(realityos_plant::PlantError::UnknownOutcome) => {}
        other => v.push(format!("late_ack_expected_unknown:{other:?}")),
    }
    let snapshot = d.lock().expect("virtual xl330").clone();
    row(
        i,
        seed,
        "late_ack",
        "late ACK",
        v,
        snapshot.physical_action_count(),
        serde_json::json!([{"kind":"delay_status","ms":250}]),
        &snapshot,
    )
}

fn scenario_reconnect(seed: u64, i: u64) -> CampaignRow {
    let d = Arc::new(Mutex::new(seeded_device(seed)));
    let mut peer = VirtualSerialPeer::new(d.clone());
    peer.set_transport_schedule(FaultSchedule {
        events: vec![
            crate::faults::FaultEvent {
                after_packet: 1,
                kind: FaultKind::Disconnect,
            },
            crate::faults::FaultEvent {
                after_packet: 2,
                kind: FaultKind::Reconnect,
            },
        ],
    });
    let mut v = Vec::new();
    if !peer.exchange(&encode_ping(1)).is_empty() {
        v.push("disconnect_still_replied".into());
    }
    if peer.is_connected() {
        v.push("disconnect_left_connected".into());
    }
    let restored = peer.exchange(&encode_ping(1));
    if !peer.is_connected() {
        v.push("reconnect_left_disconnected".into());
    }
    match decode_status(&restored) {
        Ok(st) if st.params.len() >= 3 => {}
        other => v.push(format!("reconnect_no_status:{other:?}")),
    }
    let snapshot = d.lock().expect("virtual xl330").clone();
    row(
        i,
        seed,
        "reconnect",
        "reconnect",
        v,
        snapshot.physical_action_count(),
        serde_json::json!([{"kind":"reconnect"}]),
        &snapshot,
    )
}

fn scenario_boundary_pwm(seed: u64, i: u64) -> CampaignRow {
    let mut d = seeded_device(seed);
    let start = d.present_position();
    let _ = d.process(&encode_write(d.id(), ADDR_TORQUE_ENABLE, &[1]));
    let _ = d.process(&encode_write(
        d.id(),
        realityos_metal::protocol::ADDR_GOAL_PWM,
        &0i16.to_le_bytes(),
    ));
    let goal = (start + 32).clamp(0, 4095);
    let _ = d.process(&encode_write(
        d.id(),
        ADDR_GOAL_POSITION,
        &goal.to_le_bytes(),
    ));
    d.advance(0.05);
    let mut v = Vec::new();
    if d.present_position() != start {
        v.push("pwm0_still_moved".into());
    }
    row(
        i,
        seed,
        "boundary_pwm_zero",
        "sensor",
        v,
        d.physical_action_count(),
        serde_json::json!([]),
        &d,
    )
}

pub fn run_instance(root_seed: u64, i: u64) -> CampaignRow {
    let seed = splitmix(root_seed ^ i.wrapping_mul(0xD1B5_4A32_D192_ED03));
    match i % 24 {
        0 => scenario_hold_nudge(seed, i),
        1 => scenario_unknown(seed, i),
        2 => scenario_recover(seed, i),
        3 => scenario_identity(seed, i),
        4 => scenario_torque_off_goal(seed, i),
        5 => scenario_torque_on_pending(seed, i),
        6 => scenario_transport_crc(seed, i),
        7 => scenario_watchdog(seed, i),
        8 => scenario_voltage(seed, i),
        9 => scenario_temp(seed, i),
        10 => scenario_reboot(seed, i),
        11 => scenario_disconnect(seed, i),
        12 => scenario_lifecycle_ack(seed, i),
        13 => scenario_restart(seed, i),
        14 => scenario_boundary_pwm(seed, i),
        15 => scenario_late_ack(seed, i),
        16 => scenario_reconnect(seed, i),
        _ => scenario_boot(seed, i),
    }
}

pub fn run_campaign(seed: u64, n: usize) -> CampaignRecord {
    let rows: Vec<CampaignRow> = (0..n as u64).map(|i| run_instance(seed, i)).collect();
    let mut rec = CampaignRecord::assemble(seed, rows);
    rec.schema = CAMPAIGN_SCHEMA.into();
    rec.hardware_present = false;
    rec
}

/// Production-driver ↔ virtual XL330 PTY smoke. Unix only.
pub fn run_tier2_smoke(seed: u64, n: usize) -> Result<CampaignRecord, String> {
    run_tier2_smoke_impl(seed, n)
}

#[cfg(not(unix))]
fn run_tier2_smoke_impl(_seed: u64, _n: usize) -> Result<CampaignRecord, String> {
    Err("tier2_requires_unix_pty".into())
}

#[cfg(unix)]
fn run_tier2_smoke_impl(seed: u64, n: usize) -> Result<CampaignRecord, String> {
    use crate::oracle::belief_from_trace;
    use crate::tier2::Tier2Session;

    let n = n.clamp(1, 500);
    let kinds: &[(&str, &str, Option<FaultKind>)] = &[
        ("hold", "hold", None),
        (
            "ack_lost",
            "unknown outcome",
            Some(FaultKind::DropStatusAfterApply),
        ),
        (
            "crc",
            "transport corruption",
            Some(FaultKind::CorruptOutgoingCrc),
        ),
        ("silent", "lost ACK", Some(FaultKind::DeviceSilent)),
        ("disconnect", "disconnect", Some(FaultKind::Disconnect)),
        (
            "wrong_id",
            "transport corruption",
            Some(FaultKind::WrongStatusId),
        ),
        (
            "garbage_prefix",
            "transport corruption",
            Some(FaultKind::GarbagePrefix),
        ),
        ("reboot", "reboot", Some(FaultKind::RebootDuringRequest)),
    ];
    let mut rows = Vec::new();
    for i in 0..n as u64 {
        let inst_seed = splitmix(seed ^ i.wrapping_mul(0xA5A5_A5A5_A5A5_A5A5));
        let (scenario, category, fault) = kinds[(i as usize) % kinds.len()];
        let device = seeded_device(inst_seed);
        let pack_hash = device.truth_pack().content_hash();
        let realization = realization_of(&device);
        let faults = fault
            .clone()
            .map(FaultSchedule::once)
            .unwrap_or_else(FaultSchedule::empty);
        let mut violations = Vec::new();
        let mut tier2 = None;
        let mut actions = 0u64;
        match Tier2Session::open_with_faults(device, &format!("t2-{i}"), inst_seed, faults.clone())
        {
            Ok(mut s) => {
                if fault.is_some() {
                    s.inject(faults.clone());
                }
                if scenario == "ack_lost" {
                    s.device
                        .lock()
                        .expect("oracle")
                        .drop_status_after_next_goal();
                }
                let (r, before, after) = s.try_hold(1);
                match r {
                    Ok(t) => {
                        if let Err(e) = s.a1(&t, &before, &after) {
                            violations.push(e);
                        }
                        let _ = belief_from_trace(t.ok, &t.event);
                        actions = after.physical_actions;
                        tier2 = Some(s.trace_row(&t, &before, &after));
                    }
                    Err(e) => {
                        if scenario == "hold" {
                            violations.push(format!("authorize:{e:?}"));
                        }
                        actions = after.physical_actions;
                    }
                }
            }
            Err(e) => {
                if scenario == "hold" {
                    violations.push(e);
                }
            }
        }
        rows.push(CampaignRow {
            instance: i,
            seed: inst_seed,
            scenario: format!("tier2_{scenario}"),
            category: category.into(),
            verdict: if violations.is_empty() {
                VERDICT_PASS.into()
            } else {
                VERDICT_FAIL.into()
            },
            physical_actions: actions,
            invariant_violations: violations,
            fault_sequence: serde_json::to_value(&faults).unwrap_or_else(|_| serde_json::json!([])),
            truth_pack_schema: TRUTH_PACK_SCHEMA.into(),
            truth_pack_content_hash: pack_hash,
            realization,
            tier2,
        });
    }
    let mut rec = CampaignRecord::assemble(seed, rows);
    rec.schema = CAMPAIGN_SCHEMA.into();
    rec.hardware_present = false;
    rec.note = "Tier 2 production RuntimeGovernor+Xl330Driver+PTY. SIM_VIRTUAL_METAL_NOT_METAL. Not MEASURED.".into();
    Ok(rec)
}
