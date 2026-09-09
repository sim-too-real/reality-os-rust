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
pub use identity::RuntimeIdentity;
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
    use realityos_kernel::{
        CalibrationId, DesignContentHash, FirmwareId, ReleaseHash, SerialOrAsBuilt,
    };
    use realityos_plant::{Plant, SimPlant};

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
        let mut plant = SimPlant::new("p", 1, 1.0);
        plant.go_online();
        let g = RuntimeGovernor::<SimPlant, OnlineLocked>::new_online(
            id,
            plant,
            journal,
            b"test-signing-key-32bytes-minimum".to_vec(),
            true,
            vec!["a0".into()],
            1.0,
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

        let mut plant = SimPlant::new("p", 1, 1.0);
        plant.go_online();
        let mut g = RuntimeGovernor::<SimPlant, OnlineLocked>::new_online(
            online_identity(),
            plant,
            temp_journal("one"),
            b"online-cap-key".to_vec(),
            true,
            vec!["joint-0".into()],
            10.0,
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
        let mut plant = SimPlant::new("p", 1, 1.0);
        plant.go_online();
        let mut g = RuntimeGovernor::<SimPlant, OnlineLocked>::new_online(
            online_identity(),
            plant,
            temp_journal("hold"),
            b"online-cap-key".to_vec(),
            true,
            vec!["joint-0".into()],
            10.0,
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
        let mut plant = SimPlant::new("p", 1, 1.0);
        plant.go_online();
        let mut g = RuntimeGovernor::<SimPlant, OnlineLocked>::new_online(
            id,
            plant,
            temp_journal(tag),
            key.to_vec(),
            true,
            actuators,
            10.0,
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
}
