//! Last driver gate. Identity + safety + envelope. Does not invent. Does not plan tasks.
//!
//! Distinct from any slide-task PhysicsGovernor. Foundry never spends through this crate.
//!
//! Typestates: [`Simulation`], [`Hil`], [`OnlineLocked`]. Only unlocked rails expose
//! `config_mut` / `envelope_mut` / `plant_mut` / `ledger_mut`.

pub mod envelope;
pub mod gate;
pub mod governor;
pub mod identity;
pub mod latch;
pub mod rail;
pub mod trace;

pub use envelope::{per_joint_clip, DriverEnvelopePack};
pub use gate::{admit_from_parts, GovernorGateRequest, GovernorGateVerdict};
pub use governor::{GovernorConfig, OnlineInitError, OnlineWrite, RuntimeGovernor};
pub use identity::{
    canonical_instance_bytes, match_expected_to_measured, ExpectedRuntimeIdentity,
    MeasuredHardwareIdentity, RuntimeIdentity, ValidatedRuntimeIdentity, INSTANCE_SCHEMA,
};
pub use latch::{EstopLatch, SafeState, SafeStateLatch};
pub use rail::{Hil, OnlineLocked, Rail, Simulation, UnlockedRail};
pub use trace::RuntimeTrace;

pub const SCHEMA: &str = "realityos.governor/1";

/// Veto polarity for stack-composed governors: true = halt.
#[inline]
pub const fn veto_means_halt() -> bool {
    true
}

pub fn assert_veto_polarity(safe_veto: bool, unsafe_veto: bool) -> Result<(), String> {
    if safe_veto {
        return Err("polarity inverted — safe state returned veto=true".into());
    }
    if !unsafe_veto {
        return Err("polarity inverted — unsafe state returned veto=false".into());
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Arc;

    use realityos_kernel::{
        AuthorityClock, CalibrationId, DesignContentHash, FakeClock, FirmwareId, ReleaseHash,
        SerialOrAsBuilt,
    };
    use realityos_plant::{HardwareIdentity, Plant, SimPlant};

    fn sim_gov() -> RuntimeGovernor<SimPlant> {
        let id = RuntimeIdentity::sim("rel-sim-1").unwrap();
        let plant = SimPlant::new("sim", 1, 10.0);
        let mut g = RuntimeGovernor::new(id, plant);
        g.heartbeat(10.0);
        g.mark_sensor(10.0, None);
        g
    }

    #[test]
    fn estop_never_auto_clears() {
        let mut g = sim_gov();
        g.engage_estop("manual", 11.0);
        assert!(g.estop());
        let t = g.clear_estop_requires_recovery(false, 11.0);
        assert!(!t.ok);
        assert!(g.estop());
        g.heartbeat(11.0);
        let t = g.clear_estop_requires_recovery(true, 11.0);
        assert!(t.ok);
        assert!(!g.estop());
    }

    #[test]
    fn online_identity_precheck() {
        let id = RuntimeIdentity {
            release_hash: ReleaseHash::new("rel1").unwrap(),
            design_content_hash: Some(DesignContentHash::new("d").unwrap()),
            serial_or_as_built: Some(SerialOrAsBuilt::new("SIM_SERIAL").unwrap()),
            firmware_id: Some(FirmwareId::new("FW").unwrap()),
            calibration_id: Some(CalibrationId::new("SIM_CAL").unwrap()),
        };
        let mut g = RuntimeGovernor::new(id, SimPlant::new("p", 1, 1.0));
        g.config_mut().require_online_identity = true;
        g.heartbeat(1.0);
        g.mark_sensor(1.0, None);
        let errs = g.pre_actuation_check(1.0);
        assert!(errs
            .iter()
            .any(|e| e == "incomplete_online_runtime_identity"));
    }

    #[test]
    fn veto_polarity() {
        assert!(assert_veto_polarity(false, true).is_ok());
        assert!(assert_veto_polarity(true, true).is_err());
    }

    #[test]
    fn online_locked_config_cannot_disable_rails() {
        let id = RuntimeIdentity {
            release_hash: ReleaseHash::new("rel1").unwrap(),
            design_content_hash: Some(DesignContentHash::new("des1").unwrap()),
            serial_or_as_built: Some(SerialOrAsBuilt::new("SN-1").unwrap()),
            firmware_id: Some(FirmwareId::new("FW-1").unwrap()),
            calibration_id: Some(CalibrationId::new("cal-1").unwrap()),
        };
        let dir = std::env::temp_dir().join(format!(
            "realityos-gov-online-{}",
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        let _ = std::fs::create_dir_all(&dir);
        let journal = dir.join("driver.jsonl");
        let plant = online_plant(&id);
        let g = RuntimeGovernor::<SimPlant, OnlineLocked>::new_online(
            id,
            plant,
            journal,
            b"test-signing-key-32bytes-minimum".to_vec(),
            true,
            vec!["a0".into()],
            test_clock(1.0),
        )
        .expect("online governor");
        let c = g.config();
        assert!(c.require_command_signature);
        assert!(c.require_monotonic_sequence);
        assert!(c.require_sensor_packet_hash);
        assert!(c.require_online_identity);
        assert!(c.require_sensor_before_write);
        assert!(g.plant().production_locked());
    }

    fn online_identity() -> RuntimeIdentity {
        RuntimeIdentity {
            release_hash: ReleaseHash::new("rel1").unwrap(),
            design_content_hash: Some(DesignContentHash::new("des1").unwrap()),
            serial_or_as_built: Some(SerialOrAsBuilt::new("SN-1").unwrap()),
            firmware_id: Some(FirmwareId::new("FW-1").unwrap()),
            calibration_id: Some(CalibrationId::new("cal-1").unwrap()),
        }
    }

    fn measured_matching(id: &RuntimeIdentity) -> HardwareIdentity {
        HardwareIdentity {
            serial: id.serial_str().to_string(),
            firmware_id: id.firmware_str().to_string(),
            calibration_id: id.calibration_id_str().to_string(),
            design_content_hash: id.design_str().to_string(),
            connected: true,
            metal: false,
            evidence_status: "TEST_ATTACHED".into(),
            actuator_ids: Vec::new(),
        }
    }

    fn online_plant(id: &RuntimeIdentity) -> SimPlant {
        let mut plant = SimPlant::new("p", 1, 1.0);
        plant.go_online();
        plant.bind_measured_identity(measured_matching(id));
        plant
    }

    fn test_clock(now_s: f64) -> Arc<dyn AuthorityClock> {
        FakeClock::arc(now_s)
    }

    fn temp_journal(tag: &str) -> std::path::PathBuf {
        let dir = std::env::temp_dir().join(format!(
            "realityos-gov-cap-{tag}-{}",
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        let _ = std::fs::create_dir_all(&dir);
        dir.join("driver.jsonl")
    }

    #[test]
    fn online_valid_path_is_exactly_one_actuation() {
        use realityos_core::{DecideRequest, Intent, RealityOs, WorldView};
        use realityos_plant::ActionParams;

        let mut g = RuntimeGovernor::<SimPlant, OnlineLocked>::new_online(
            online_identity(),
            online_plant(&online_identity()),
            temp_journal("one"),
            b"online-cap-key".to_vec(),
            true,
            vec!["joint-0".into()],
            test_clock(10.0),
        )
        .unwrap();
        g.record_sensor(&[("q0".into(), 0.0)], 10.0, 1, "frame", "s")
            .unwrap();
        let mut ros = RealityOs::new();
        let d = ros.decide(DecideRequest::new(
            Intent::language("hold", "hold"),
            WorldView {
                tau_max: vec![5.0],
                ..WorldView::default()
            },
            10.0,
        ));
        let issued = d.command.expect("allow");
        let write = g.authorize_issued(issued).expect("authorize");
        let t = g.write_online(&write, &ActionParams::empty(), 10.0);
        assert!(t.ok, "{:?}", t.violations);
        assert_eq!(g.plant().write_count(), 1);
        let t2 = g.write_online(&write, &ActionParams::empty(), 10.1);
        assert!(!t2.ok);
        assert_eq!(g.plant().write_count(), 1);
    }

    #[test]
    fn online_safe_state_cannot_return_to_running() {
        let mut g = RuntimeGovernor::<SimPlant, OnlineLocked>::new_online(
            online_identity(),
            online_plant(&online_identity()),
            temp_journal("hold"),
            b"online-cap-key".to_vec(),
            true,
            vec!["joint-0".into()],
            test_clock(10.0),
        )
        .unwrap();
        g.latch_safe_state(SafeState::Hold);
        assert_eq!(g.safe_state(), SafeState::Hold);
        g.latch_safe_state(SafeState::Running);
        assert_eq!(g.safe_state(), SafeState::Hold);
        g.latch_safe_state(SafeState::Fault);
        assert_eq!(g.safe_state(), SafeState::Fault);
    }

    fn online_gov(
        tag: &str,
        id: RuntimeIdentity,
        key: &[u8],
        actuators: Vec<String>,
    ) -> RuntimeGovernor<SimPlant, OnlineLocked> {
        let mut g = RuntimeGovernor::<SimPlant, OnlineLocked>::new_online(
            id.clone(),
            online_plant(&id),
            temp_journal(tag),
            key.to_vec(),
            true,
            actuators,
            test_clock(10.0),
        )
        .unwrap();
        g.record_sensor(&[("q0".into(), 0.0)], 10.0, 1, "frame", "s")
            .unwrap();
        g
    }

    fn decide_hold(seq: i64, now_s: f64) -> realityos_core::IssuedCommand {
        use realityos_core::{DecideRequest, Intent, RealityOs, WorldView};
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
        req.command_id = format!("cmd-inst-{seq}-{now_s}");
        ros.decide(req).command.expect("allow")
    }

    fn mutate_serial(mut id: RuntimeIdentity, serial: &str) -> RuntimeIdentity {
        id.serial_or_as_built = Some(SerialOrAsBuilt::new(serial).unwrap());
        id
    }

    fn mutate_fw(mut id: RuntimeIdentity, fw: &str) -> RuntimeIdentity {
        id.firmware_id = Some(FirmwareId::new(fw).unwrap());
        id
    }

    fn mutate_cal(mut id: RuntimeIdentity, cal: &str) -> RuntimeIdentity {
        id.calibration_id = Some(CalibrationId::new(cal).unwrap());
        id
    }

    #[test]
    fn online_write_refuses_same_design_different_serial() {
        use realityos_plant::ActionParams;
        let key = b"shared-key-across-instances";
        let mut a = online_gov("ser-a", online_identity(), key, vec!["joint-0".into()]);
        let mut b = online_gov(
            "ser-b",
            mutate_serial(online_identity(), "SN-2"),
            key,
            vec!["joint-0".into()],
        );
        let write = a.authorize_issued(decide_hold(1, 10.0)).unwrap();
        assert!(a.write_online(&write, &ActionParams::empty(), 10.0).ok);
        let t = b.write_online(&write, &ActionParams::empty(), 10.0);
        assert!(!t.ok, "{:?}", t.violations);
        assert!(
            t.violations
                .iter()
                .any(|v| v.contains("runtime_instance_mismatch")),
            "{:?}",
            t.violations
        );
        assert_eq!(b.plant().write_count(), 0);
    }

    #[test]
    fn online_write_refuses_same_serial_different_firmware() {
        use realityos_plant::ActionParams;
        let key = b"shared-key-across-instances";
        let a = online_gov("fw-a", online_identity(), key, vec!["joint-0".into()]);
        let mut b = online_gov(
            "fw-b",
            mutate_fw(online_identity(), "FW-2"),
            key,
            vec!["joint-0".into()],
        );
        let write = a.authorize_issued(decide_hold(2, 11.0)).unwrap();
        let t = b.write_online(&write, &ActionParams::empty(), 11.0);
        assert!(!t.ok);
        assert!(t
            .violations
            .iter()
            .any(|v| v.contains("runtime_instance_mismatch")));
        assert_eq!(b.plant().write_count(), 0);
    }

    #[test]
    fn online_write_refuses_changed_calibration() {
        use realityos_plant::ActionParams;
        let key = b"shared-key-across-instances";
        let a = online_gov("cal-a", online_identity(), key, vec!["joint-0".into()]);
        let mut b = online_gov(
            "cal-b",
            mutate_cal(online_identity(), "cal-2"),
            key,
            vec!["joint-0".into()],
        );
        let write = a.authorize_issued(decide_hold(3, 12.0)).unwrap();
        let t = b.write_online(&write, &ActionParams::empty(), 12.0);
        assert!(!t.ok);
        assert!(t
            .violations
            .iter()
            .any(|v| v.contains("runtime_instance_mismatch")));
        assert_eq!(b.plant().write_count(), 0);
    }

    #[test]
    fn online_write_refuses_changed_actuator_set() {
        use realityos_plant::ActionParams;
        let key = b"shared-key-across-instances";
        let a = online_gov("act-a", online_identity(), key, vec!["joint-0".into()]);
        let mut b = online_gov(
            "act-b",
            online_identity(),
            key,
            vec!["joint-0".into(), "joint-1".into()],
        );
        let write = a.authorize_issued(decide_hold(4, 13.0)).unwrap();
        let t = b.write_online(&write, &ActionParams::empty(), 13.0);
        assert!(!t.ok);
        assert!(t
            .violations
            .iter()
            .any(|v| v.contains("runtime_instance_mismatch")
                || v.contains("actuator_scope_not_authorized")));
        assert_eq!(b.plant().write_count(), 0);
    }

    #[test]
    fn online_write_refuses_reused_key_on_foreign_instance() {
        use realityos_plant::ActionParams;
        let key = b"accidentally-reused-signing-key";
        let a = online_gov("key-a", online_identity(), key, vec!["joint-0".into()]);
        let mut b = online_gov(
            "key-b",
            mutate_serial(online_identity(), "SN-OTHER"),
            key,
            vec!["joint-0".into()],
        );
        let write = a.authorize_issued(decide_hold(5, 14.0)).unwrap();
        let t = b.write_online(&write, &ActionParams::empty(), 14.0);
        assert!(!t.ok);
        assert_eq!(b.plant().write_count(), 0);
    }

    #[test]
    fn online_write_from_governor_a_refused_by_governor_b() {
        use realityos_plant::ActionParams;
        let a = online_gov(
            "ab-a",
            online_identity(),
            b"key-a-unique",
            vec!["joint-0".into()],
        );
        let mut b = online_gov(
            "ab-b",
            mutate_serial(online_identity(), "SN-B"),
            b"key-b-unique",
            vec!["joint-0".into()],
        );
        let write = a.authorize_issued(decide_hold(6, 15.0)).unwrap();
        let t = b.write_online(&write, &ActionParams::empty(), 15.0);
        assert!(!t.ok);
        assert_eq!(b.plant().write_count(), 0);
        assert_eq!(a.plant().write_count(), 0);
    }

    #[test]
    fn new_online_refuses_configured_vs_probed_mismatches() {
        let id = online_identity();
        let mut plant = online_plant(&id);
        let mut wrong = measured_matching(&id);
        wrong.serial = "SN-B".into();
        plant.replace_measured_identity(wrong);
        let err = RuntimeGovernor::<SimPlant, OnlineLocked>::new_online(
            id.clone(),
            plant,
            temp_journal("mm-sn"),
            b"k".to_vec(),
            true,
            vec!["joint-0".into()],
            test_clock(1.0),
        )
        .err()
        .unwrap();
        assert!(err.0.contains("hardware_serial_mismatch"), "{err}");

        let mut plant = online_plant(&id);
        let mut wrong = measured_matching(&id);
        wrong.firmware_id = "FW-B".into();
        plant.replace_measured_identity(wrong);
        let err = RuntimeGovernor::<SimPlant, OnlineLocked>::new_online(
            id.clone(),
            plant,
            temp_journal("mm-fw"),
            b"k".to_vec(),
            true,
            vec!["joint-0".into()],
            test_clock(1.0),
        )
        .err()
        .unwrap();
        assert!(err.0.contains("hardware_firmware_mismatch"), "{err}");

        let mut plant = online_plant(&id);
        let mut wrong = measured_matching(&id);
        wrong.calibration_id = "cal-B".into();
        plant.replace_measured_identity(wrong);
        let err = RuntimeGovernor::<SimPlant, OnlineLocked>::new_online(
            id.clone(),
            plant,
            temp_journal("mm-cal"),
            b"k".to_vec(),
            true,
            vec!["joint-0".into()],
            test_clock(1.0),
        )
        .err()
        .unwrap();
        assert!(err.0.contains("hardware_calibration_mismatch"), "{err}");

        let mut plant = online_plant(&id);
        let mut wrong = measured_matching(&id);
        wrong.design_content_hash = "des-B".into();
        plant.replace_measured_identity(wrong);
        let err = RuntimeGovernor::<SimPlant, OnlineLocked>::new_online(
            id,
            plant,
            temp_journal("mm-des"),
            b"k".to_vec(),
            true,
            vec!["joint-0".into()],
            test_clock(1.0),
        )
        .err()
        .unwrap();
        assert!(err.0.contains("hardware_design_mismatch"), "{err}");
    }

    #[test]
    fn new_online_refuses_disconnected_placeholder_and_missing() {
        let id = online_identity();
        let mut plant = online_plant(&id);
        plant.set_measured_connected(false);
        let err = RuntimeGovernor::<SimPlant, OnlineLocked>::new_online(
            id.clone(),
            plant,
            temp_journal("disc"),
            b"k".to_vec(),
            true,
            vec!["joint-0".into()],
            test_clock(1.0),
        )
        .err()
        .unwrap();
        assert!(err.0.contains("disconnected"), "{err}");

        let mut plant = SimPlant::new("p", 1, 1.0);
        plant.go_online();
        plant.bind_measured_identity(HardwareIdentity {
            serial: "SIM_SERIAL".into(),
            firmware_id: "SIM_FW".into(),
            calibration_id: "SIM_CAL".into(),
            design_content_hash: "des1".into(),
            connected: true,
            metal: false,
            evidence_status: "TEST".into(),
            actuator_ids: vec![],
        });
        let err = RuntimeGovernor::<SimPlant, OnlineLocked>::new_online(
            id.clone(),
            plant,
            temp_journal("simid"),
            b"k".to_vec(),
            true,
            vec!["joint-0".into()],
            test_clock(1.0),
        )
        .err()
        .unwrap();
        assert!(err.0.contains("placeholder"), "{err}");

        let mut plant = SimPlant::new("p", 1, 1.0);
        plant.go_online();
        let err = RuntimeGovernor::<SimPlant, OnlineLocked>::new_online(
            id,
            plant,
            temp_journal("noid"),
            b"k".to_vec(),
            true,
            vec!["joint-0".into()],
            test_clock(1.0),
        )
        .err()
        .unwrap();
        assert!(err.0.contains("online_requires_hardware_identity"), "{err}");
    }

    #[test]
    fn identity_change_after_online_faults_and_writes_zero() {
        use realityos_plant::ActionParams;
        let id = online_identity();
        let mut plant = SimPlant::new("p", 1, 1.0);
        plant.go_online();
        let handle = plant.bind_measured_identity(measured_matching(&id));
        let mut g = RuntimeGovernor::<SimPlant, OnlineLocked>::new_online(
            id,
            plant,
            temp_journal("hot"),
            b"online-cap-key".to_vec(),
            true,
            vec!["joint-0".into()],
            test_clock(10.0),
        )
        .unwrap();
        g.record_sensor(&[("q0".into(), 0.0)], 10.0, 1, "frame", "s")
            .unwrap();
        let write = g.authorize_issued(decide_hold(1, 10.0)).unwrap();
        assert!(g.write_online(&write, &ActionParams::empty(), 10.0).ok);
        assert_eq!(g.plant().write_count(), 1);

        handle.lock().unwrap().serial = "SN-REPLACED".into();
        let t = g.write_online(&write, &ActionParams::empty(), 10.1);
        assert!(!t.ok, "{:?}", t.violations);
        assert!(t
            .violations
            .iter()
            .any(|v| v.contains("hardware_serial_mismatch")
                || v.contains("hardware_session_requires_online_restart")));
        assert!(g.hardware_session_dead());
        assert_eq!(g.plant().write_count(), 1);
        assert_eq!(g.safe_state(), SafeState::Fault);
        let rec = g.clear_estop_requires_recovery(true, 10.2);
        assert!(!rec.ok);
        assert!(g.hardware_session_dead());
        assert!(g.authorize_issued(decide_hold(3, 10.3)).is_err());
    }

    #[test]
    fn disconnect_then_reconnect_different_device_stays_dead() {
        use realityos_plant::ActionParams;
        let id = online_identity();
        let mut plant = SimPlant::new("p", 1, 1.0);
        plant.go_online();
        let handle = plant.bind_measured_identity(measured_matching(&id));
        let mut g = RuntimeGovernor::<SimPlant, OnlineLocked>::new_online(
            id,
            plant,
            temp_journal("reconn"),
            b"online-cap-key".to_vec(),
            true,
            vec!["joint-0".into()],
            test_clock(10.0),
        )
        .unwrap();
        g.record_sensor(&[("q0".into(), 0.0)], 10.0, 1, "frame", "s")
            .unwrap();
        handle.lock().unwrap().connected = false;
        let write = g.authorize_issued(decide_hold(1, 10.0)).unwrap();
        let t = g.write_online(&write, &ActionParams::empty(), 10.0);
        assert!(!t.ok);
        assert!(g.hardware_session_dead());
        assert_eq!(g.plant().write_count(), 0);

        handle.lock().unwrap().connected = true;
        handle.lock().unwrap().serial = "SN-OTHER".into();
        let t2 = g.write_online(&write, &ActionParams::empty(), 10.1);
        assert!(!t2.ok);
        assert_eq!(g.plant().write_count(), 0);
        assert!(g.hardware_session_dead());
    }

    #[test]
    fn proposer_timestamp_cannot_refresh_online_freshness() {
        let clock = FakeClock::arc(10.0);
        let mut g = RuntimeGovernor::<SimPlant, OnlineLocked>::new_online(
            online_identity(),
            online_plant(&online_identity()),
            temp_journal("fresh"),
            b"online-cap-key".to_vec(),
            true,
            vec!["joint-0".into()],
            clock.clone(),
        )
        .unwrap();
        g.record_sensor(&[("q0".into(), 0.0)], 999.0, 1, "frame", "s")
            .unwrap();
        assert!((g.last_sensor_s() - 10.0).abs() < 1e-9);
        assert!((g.last_device_capture_s() - 999.0).abs() < 1e-9);
        clock.set(80.0);
        let errs = g.pre_actuation_check(80.0);
        assert!(
            errs.iter().any(|e| e == "sensor_stale"),
            "{errs:?} last_sensor={}",
            g.last_sensor_s()
        );
    }
}
