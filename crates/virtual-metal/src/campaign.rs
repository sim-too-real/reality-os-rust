//! Seeded Virtual Metal campaign. Deterministic. Not MEASURED.

use std::cell::RefCell;
use std::path::PathBuf;
use std::rc::Rc;
use std::sync::Arc;

use realityos_core::{DecideRequest, Intent, PolicyProposal, RealityOs, WorldView};
use realityos_governor::{OnlineLocked, RuntimeGovernor, RuntimeIdentity};
use realityos_kernel::{
    AuthorityClock, CalibrationId, DesignContentHash, FakeClock, FirmwareId, ReleaseHash,
    SerialOrAsBuilt,
};
use realityos_plant::{ActionParams, HardwareBackedPlant};

use crate::device::VirtualXl330;
use crate::evidence::{CampaignRecord, CampaignRow, CAMPAIGN_SCHEMA, VERDICT_FAIL, VERDICT_PASS};
use crate::port::{VirtualMetalPort, DEFAULT_ACTUATOR};
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
) -> RuntimeGovernor<HardwareBackedPlant<VirtualMetalPort>, OnlineLocked> {
    start_gov_opts(port, tag, seed, true)
}

pub fn start_gov_opts(
    port: VirtualMetalPort,
    tag: &str,
    seed: u64,
    first_online: bool,
) -> RuntimeGovernor<HardwareBackedPlant<VirtualMetalPort>, OnlineLocked> {
    let path = journal_path(tag, seed);
    if first_online {
        if let Some(dir) = path.parent() {
            let _ = std::fs::remove_dir_all(dir);
            let _ = std::fs::create_dir_all(dir);
        }
    }
    let id = vm_identity(&port);
    let plant = HardwareBackedPlant::new(port, "xl330-vm", 1, 5.0);
    let clock: Arc<dyn AuthorityClock> = FakeClock::arc(10.0);
    let mut g = RuntimeGovernor::<HardwareBackedPlant<VirtualMetalPort>, OnlineLocked>::new_online(
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

fn row(
    instance: u64,
    seed: u64,
    scenario: &str,
    violations: Vec<String>,
    actions: u64,
    faults: serde_json::Value,
) -> CampaignRow {
    CampaignRow {
        instance,
        seed,
        scenario: scenario.into(),
        verdict: if violations.is_empty() {
            VERDICT_PASS.into()
        } else {
            VERDICT_FAIL.into()
        },
        physical_actions: actions,
        invariant_violations: violations,
        fault_sequence: faults,
        truth_pack_schema: TRUTH_PACK_SCHEMA.into(),
    }
}

fn scenario_boot(seed: u64, i: u64) -> CampaignRow {
    let mut d = VirtualXl330::from_seed(seed, Xl330TruthPack::xl330_m288());
    let ping = realityos_metal::protocol::encode_ping(d.id());
    let st = realityos_metal::protocol::decode_status(&d.process(&ping)).unwrap();
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
    if (n5 - 103.0).abs() > 1e-9 {
        v.push(format!("noload_5v:{n5}"));
    }
    row(
        i,
        seed,
        "boot_identity_sensor",
        v,
        d.physical_action_count(),
        serde_json::json!([]),
    )
}

fn scenario_hold_nudge(seed: u64, i: u64) -> CampaignRow {
    let d = Rc::new(RefCell::new(VirtualXl330::from_seed(
        seed,
        Xl330TruthPack::xl330_m288(),
    )));
    let port = VirtualMetalPort::new(d.clone());
    let mut g = start_gov(port, "hold", seed);
    let w = g.authorize_issued(decide_hold(1, 10.0)).unwrap();
    let t = g.write_online_now(&w, &ActionParams::empty());
    let mut v = Vec::new();
    if !t.ok {
        v.push(format!("hold_refused:{:?}", t.violations));
    }
    let before_nudge = d.borrow().present_position();
    let nw = g
        .authorize_issued(decide_nudge(2, 10.0, vec![5.0]))
        .unwrap();
    let nt = g.write_online_now(&nw, &ActionParams::empty());
    if !nt.ok {
        v.push(format!("nudge_refused:{:?}", nt.violations));
    }
    if d.borrow().present_position() == before_nudge {
        v.push("nudge_did_not_move_present".into());
    }
    let before = d.borrow().physical_action_count();
    let replay = g.write_online_now(&w, &ActionParams::empty());
    if replay.ok {
        v.push("replay_executed".into());
    }
    if d.borrow().physical_action_count() != before {
        v.push("replay_second_physical_action".into());
    }
    let actions = d.borrow().physical_action_count();
    row(
        i,
        seed,
        "hold_nudge_replay",
        v,
        actions,
        serde_json::json!([]),
    )
}

fn scenario_unknown(seed: u64, i: u64) -> CampaignRow {
    let d = Rc::new(RefCell::new(VirtualXl330::from_seed(
        seed,
        Xl330TruthPack::xl330_m288(),
    )));
    let port = VirtualMetalPort::new(d.clone());
    let mut g = start_gov(port, "unk", seed);
    d.borrow_mut().drop_status_after_next_goal();
    let w = g.authorize_issued(decide_hold(1, 10.0)).unwrap();
    let t = g.write_online_now(&w, &ActionParams::empty());
    let mut v = Vec::new();
    if t.ok {
        v.push("unknown_path_succeeded".into());
    }
    if !g.integrity_aborted() {
        v.push("unknown_did_not_integrity_abort".into());
    }
    let actions = d.borrow().physical_action_count();
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
    if d.borrow().physical_action_count() != actions {
        v.push("unknown_then_extra_physical".into());
    }
    let actions = d.borrow().physical_action_count();
    row(
        i,
        seed,
        "ack_lost_unknown",
        v,
        actions,
        serde_json::json!([{"after_packet":2,"kind":"drop_status_after_apply"}]),
    )
}

fn scenario_recover(seed: u64, i: u64) -> CampaignRow {
    let d = Rc::new(RefCell::new(VirtualXl330::from_seed(
        seed,
        Xl330TruthPack::xl330_m288(),
    )));
    let port = VirtualMetalPort::new(d.clone());
    let mut g = start_gov(port, "rec", seed);
    let w = g.authorize_issued(decide_hold(1, 10.0)).unwrap();
    let _ = g.write_online_now(&w, &ActionParams::empty());
    let _ = g.write_online_now(&w, &ActionParams::empty());
    let mut v = Vec::new();
    if !g.integrity_aborted() {
        v.push("replay_no_integrity".into());
    }
    let actions = d.borrow().physical_action_count();
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
    if d.borrow().physical_action_count() != actions {
        v.push("recover_extra_physical".into());
    }
    let actions = d.borrow().physical_action_count();
    row(
        i,
        seed,
        "recover_integrity_estop",
        v,
        actions,
        serde_json::json!([]),
    )
}

fn scenario_identity(seed: u64, i: u64) -> CampaignRow {
    let d = Rc::new(RefCell::new(VirtualXl330::from_seed(
        seed,
        Xl330TruthPack::xl330_m288(),
    )));
    let port = VirtualMetalPort::new(d.clone());
    let mut g = start_gov(port, "id", seed);
    g.acquire_sensor().ok();
    d.borrow_mut().set_model_firmware(1190, 1);
    let mut v = Vec::new();
    match g.authorize_issued(decide_hold(1, 10.0)) {
        Ok(w) => {
            let t = g.write_online_now(&w, &ActionParams::empty());
            if t.ok {
                v.push("identity_swap_actuated".into());
            }
        }
        Err(_) => {}
    }
    if d.borrow().physical_action_count() != 0 {
        v.push("identity_swap_physical".into());
    }
    let actions = d.borrow().physical_action_count();
    row(i, seed, "identity_swap", v, actions, serde_json::json!([]))
}

pub fn run_instance(root_seed: u64, i: u64) -> CampaignRow {
    let seed = splitmix(root_seed ^ i.wrapping_mul(0xD1B5_4A32_D192_ED03));
    match i % 20 {
        0 => scenario_hold_nudge(seed, i),
        1 => scenario_unknown(seed, i),
        2 => scenario_recover(seed, i),
        3 => scenario_identity(seed, i),
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
