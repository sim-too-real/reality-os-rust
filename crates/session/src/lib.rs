//! RuntimeSession composition root.
//!
//! Foundry compiles embodiments. Reality OS compiles CertifiedCommand.
//! Governor proves permission. This crate alone binds identity and acknowledges.

pub mod bridge;
pub mod estimate;
pub mod mode;
pub mod packages;
pub mod session;

pub use bridge::HardwareControlBridge;
pub use estimate::HoldEstimator;
pub use mode::{RuntimeMode, SessionStartError};
pub use packages::{GovernorPackage, PackageReport, RealityOsPackage, SafetyEdge};
pub use session::{DispatchResult, RuntimeSession, StartArgs};

pub const SCHEMA: &str = "realityos.session/1";

#[cfg(test)]
mod tests {
    use super::*;
    use realityos_core::{fixture, Certificate, Intent, RealityOs, WorldView};
    use realityos_governor::OnlineLocked;
    use realityos_kernel::DecisionStatus;
    use realityos_plant::{ActionParams, HardwareIdentity, Plant, SimPlant};

    fn bind_online_plant(plant: &mut SimPlant, args: &StartArgs) {
        plant.go_online();
        let _ = plant.bind_measured_identity(HardwareIdentity {
            serial: args.serial_or_as_built.clone(),
            firmware_id: args.firmware_id.clone(),
            calibration_id: args.calibration_id.clone(),
            design_content_hash: args.design_content_hash.clone(),
            connected: true,
            metal: false,
            evidence_status: "TEST_ATTACHED".into(),
            actuator_ids: Vec::new(),
        });
    }

    #[test]
    fn online_refuses_rail_opt_out() {
        let mut plant = SimPlant::new("p", 1, 1.0);
        plant.go_online();
        let mut args = StartArgs::simulation("rel1");
        args.mode = RuntimeMode::Online;
        args.release_class = "MFG_CANDIDATE".into();
        args.serial_or_as_built = "SN-1".into();
        args.firmware_id = "FW-1".into();
        args.calibration_id = "cal-1".into();
        args.design_content_hash = "des1".into();
        args.require_verified_release = Some(false);
        match RuntimeSession::<SimPlant, OnlineLocked>::start_online(args, plant) {
            Err(err) => assert!(err.0.contains("online_refuses_safety_rail_opt_out")),
            Ok(_) => panic!("expected online_refuses_safety_rail_opt_out"),
        }
    }

    #[test]
    fn start_online_requires_journal() {
        let mut plant = SimPlant::new("p", 1, 1.0);
        plant.go_online();
        let mut args = StartArgs::simulation("rel1");
        args.release_class = "MFG_CANDIDATE".into();
        args.serial_or_as_built = "SN-1".into();
        args.firmware_id = "FW-1".into();
        args.calibration_id = "cal-1".into();
        args.design_content_hash = "des1".into();
        args.signing_key = Some(b"k".to_vec());
        args.actuator_ids = vec!["a0".into()];
        args.first_online = true;
        let err = RuntimeSession::<SimPlant, OnlineLocked>::start_online(args, plant)
            .err()
            .unwrap();
        assert!(err.0.contains("online_requires_durable_journal"));
    }

    #[test]
    fn sim_dispatch_allow() {
        let plant = SimPlant::new("p", 1, 10.0);
        let sess = RuntimeSession::start(StartArgs::simulation("rel-sim-aa"), plant, 10.0);
        let mut sess = sess.unwrap();
        sess.governor.mark_sensor(10.0, None);
        let mut ros = RealityOs::new();
        let world = WorldView {
            tau_max: vec![5.0],
            ..WorldView::default()
        };
        let d = ros.decide(realityos_core::DecideRequest::new(
            Intent::language("hold", "hold"),
            world,
            10.0,
        ));
        assert_eq!(d.status, DecisionStatus::Allow);
        let cmd = d.command.unwrap().into_command();
        let out = sess.bind_and_dispatch(cmd, &ActionParams::empty(), 10.0);
        assert!(out.ok, "{:?}", out.violations);
        assert_eq!(sess.governor.plant().write_count(), 1);
    }

    #[test]
    fn refuse_command_never_writes() {
        let plant = SimPlant::new("p", 1, 10.0);
        let mut sess =
            RuntimeSession::start(StartArgs::simulation("rel-sim-bb"), plant, 1.0).unwrap();
        sess.governor.mark_sensor(1.0, None);
        let cert = Certificate::new(DecisionStatus::Refuse, "no");
        let cmd = fixture::issue("x", 1, 1.0, 30.0, cert, vec![0.1]).unwrap();
        let out = sess.bind_and_dispatch(cmd, &ActionParams::empty(), 1.0);
        assert!(!out.ok);
        assert_eq!(sess.governor.plant().write_count(), 0);
    }

    #[test]
    fn hold_blocks_all_modes() {
        let plant = SimPlant::new("p", 1, 10.0);
        let mut sess =
            RuntimeSession::start(StartArgs::simulation("rel-sim-cc"), plant, 1.0).unwrap();
        sess.latch_safe_state(realityos_governor::SafeState::Hold, "test");
        let cert = Certificate::new(DecisionStatus::Allow, "ok");
        let cmd = fixture::issue("x", 1, 1.0, 30.0, cert, vec![0.1]).unwrap();
        let out = sess.bind_and_dispatch(cmd, &ActionParams::empty(), 1.0);
        assert!(out.violations.iter().any(|v| v.contains("safe_state")));
    }

    #[test]
    fn bare_action_refused() {
        let plant = SimPlant::new("p", 1, 10.0);
        let sess = RuntimeSession::start(StartArgs::simulation("rel-sim-dd"), plant, 1.0).unwrap();
        let out = sess.refuse_bare_action(&[1.0]);
        assert!(!out.ok);
    }

    #[test]
    fn harness_bridge_e2e_records_data_and_refuses_bare_online_act() {
        let mut br = HardwareControlBridge::sim_harness("rel-harness-1", 10.0).unwrap();
        let js = realityos_ros2::JointState {
            name: vec!["j1".into()],
            position: vec![0.1],
            velocity: vec![0.0],
            effort: vec![0.0],
            timestamp_s: 10.0,
        };
        br.ingest_joint_state(&js, 10.0).unwrap();
        let mut ros = RealityOs::new();
        let d = ros.decide(realityos_core::DecideRequest::new(
            Intent::language("hold", "hold"),
            WorldView {
                tau_max: vec![5.0],
                ..WorldView::default()
            },
            10.0,
        ));
        let cmd = d.command.expect("allow");
        let out = br.dispatch(cmd, 10.0);
        assert!(out.ok, "{:?}", out.violations);
        assert!(!br.events.is_empty());
        let report = br.connection_report();
        assert!(report
            .iter()
            .any(|l| l.name == "fieldbus" && l.state == realityos_ros2::LinkState::NamedHole));
        let err = br
            .session
            .governor
            .plant_mut()
            .act(&[0.3], &ActionParams::empty())
            .unwrap_err();
        match err {
            realityos_plant::PlantError::UncertifiedOnline { method } => {
                assert_eq!(method, "act");
            }
            other => panic!("{other:?}"),
        }
        assert!(!br.snapshot().metal);
        assert!(!br.fieldbus.attached);
    }

    #[test]
    fn online_dispatch_signs_after_bind_and_survives_restart() {
        let dir = std::env::temp_dir().join(format!(
            "realityos-online-sess-{}",
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        let _ = std::fs::create_dir_all(&dir);
        let journal = dir.join("driver.jsonl");
        let mut plant = SimPlant::new("p", 1, 10.0);
        let mut args = StartArgs::simulation("rel-online-1");
        args.release_class = "MFG_CANDIDATE".into();
        args.serial_or_as_built = "SN-1".into();
        args.firmware_id = "FW-1".into();
        args.calibration_id = "cal-1".into();
        args.design_content_hash = "des1".into();
        args.journal_path = Some(journal.clone());
        args.signing_key = Some(b"online-session-key".to_vec());
        args.first_online = true;
        args.actuator_ids = vec!["joint-0".into()];
        bind_online_plant(&mut plant, &args);
        let mut sess = RuntimeSession::<SimPlant, OnlineLocked>::start_online_with_clock(
            args.clone(),
            plant,
            realityos_kernel::FakeClock::arc(10.0),
        )
        .unwrap();
        sess.ingest_sensor_packet(realityos_plant::SensorPacket::from_samples(
            vec![("q0".into(), 0.0)],
            10.0,
        ))
        .unwrap();
        let mut ros = RealityOs::new();
        let d = ros.decide(realityos_core::DecideRequest::new(
            Intent::language("hold", "hold"),
            WorldView {
                tau_max: vec![5.0],
                ..WorldView::default()
            },
            10.0,
        ));
        let cmd = d.command.unwrap();
        let out = sess.dispatch_issued(cmd.clone(), &ActionParams::empty());
        assert!(out.ok, "{:?}", out.violations);
        assert_eq!(sess.governor.plant().write_count(), 1);
        drop(sess);

        let mut plant2 = SimPlant::new("p", 1, 10.0);
        args.first_online = false;
        bind_online_plant(&mut plant2, &args);
        let mut sess2 = RuntimeSession::<SimPlant, OnlineLocked>::start_online_with_clock(
            args,
            plant2,
            realityos_kernel::FakeClock::arc(11.0),
        )
        .unwrap();
        sess2
            .ingest_sensor_packet(realityos_plant::SensorPacket::from_samples(
                vec![("q0".into(), 0.0)],
                11.0,
            ))
            .unwrap();
        let out2 = sess2.dispatch_issued(cmd, &ActionParams::empty());
        assert!(!out2.ok);
        assert!(
            out2.violations
                .iter()
                .any(|v| v.contains("replayed") || v.contains("sequence")),
            "{:?}",
            out2.violations
        );
        assert_eq!(sess2.governor.plant().write_count(), 0);
    }

    #[test]
    fn online_hold_cannot_return_to_running() {
        let dir = std::env::temp_dir().join(format!(
            "realityos-online-hold-{}",
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        let _ = std::fs::create_dir_all(&dir);
        let journal = dir.join("driver.jsonl");
        let mut plant = SimPlant::new("p", 1, 10.0);
        let mut args = StartArgs::simulation("rel-online-hold");
        args.release_class = "MFG_CANDIDATE".into();
        args.serial_or_as_built = "SN-1".into();
        args.firmware_id = "FW-1".into();
        args.calibration_id = "cal-1".into();
        args.design_content_hash = "des1".into();
        args.journal_path = Some(journal);
        args.signing_key = Some(b"online-session-key".to_vec());
        args.first_online = true;
        args.actuator_ids = vec!["joint-0".into()];
        bind_online_plant(&mut plant, &args);
        let mut sess = RuntimeSession::<SimPlant, OnlineLocked>::start_online_with_clock(
            args,
            plant,
            realityos_kernel::FakeClock::arc(10.0),
        )
        .unwrap();
        sess.latch_safe_state(realityos_governor::SafeState::Hold, "test");
        sess.latch_safe_state(realityos_governor::SafeState::Running, "forged");
        sess.ingest_sensor_packet(realityos_plant::SensorPacket::from_samples(
            vec![("q0".into(), 0.0)],
            10.0,
        ))
        .unwrap();
        let mut ros = RealityOs::new();
        let d = ros.decide(realityos_core::DecideRequest::new(
            Intent::language("hold", "hold"),
            WorldView {
                tau_max: vec![5.0],
                ..WorldView::default()
            },
            10.0,
        ));
        let out = sess.dispatch_issued(d.command.unwrap(), &ActionParams::empty());
        assert!(!out.ok);
        assert!(
            out.violations.iter().any(|v| v.contains("safe_state")),
            "{:?}",
            out.violations
        );
        assert_eq!(sess.governor.plant().write_count(), 0);
    }

    #[test]
    fn stop_distance_domain_refuses_too_fast() {
        let mut ros = RealityOs::new();
        let mut req = realityos_core::DecideRequest::new(
            Intent::language("stop", "stop"),
            WorldView {
                speed_m_s: Some(10.0),
                decel_m_s2: Some(1.0),
                max_stop_m: Some(1.0),
                ..WorldView::default()
            },
            1.0,
        );
        req.intent.require_scene = false;
        let d = ros.decide(req);
        assert_eq!(d.status, DecisionStatus::Refuse);
        assert!(d.command.is_none());
    }

    #[test]
    fn start_online_refuses_probed_serial_mismatch() {
        let dir = std::env::temp_dir().join(format!(
            "realityos-online-mm-{}",
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        let _ = std::fs::create_dir_all(&dir);
        let mut plant = SimPlant::new("p", 1, 10.0);
        let mut args = StartArgs::simulation("rel-online-mm");
        args.release_class = "MFG_CANDIDATE".into();
        args.serial_or_as_built = "SN-A".into();
        args.firmware_id = "FW-1".into();
        args.calibration_id = "cal-1".into();
        args.design_content_hash = "des1".into();
        args.journal_path = Some(dir.join("driver.jsonl"));
        args.signing_key = Some(b"online-session-key".to_vec());
        args.first_online = true;
        args.actuator_ids = vec!["joint-0".into()];
        bind_online_plant(&mut plant, &args);
        plant.replace_measured_identity(HardwareIdentity {
            serial: "SN-B".into(),
            firmware_id: "FW-1".into(),
            calibration_id: "cal-1".into(),
            design_content_hash: "des1".into(),
            connected: true,
            metal: false,
            evidence_status: "TEST_ATTACHED".into(),
            actuator_ids: vec![],
        });
        let err = RuntimeSession::<SimPlant, OnlineLocked>::start_online(args, plant)
            .err()
            .unwrap();
        assert!(err.0.contains("hardware_serial_mismatch"), "{}", err.0);
    }

    #[test]
    fn start_online_uses_os_monotonic_clock() {
        let dir = std::env::temp_dir().join(format!(
            "realityos-online-osclk-{}",
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        let _ = std::fs::create_dir_all(&dir);
        let mut plant = SimPlant::new("p", 1, 10.0);
        let mut args = StartArgs::simulation("rel-online-osclk");
        args.release_class = "MFG_CANDIDATE".into();
        args.serial_or_as_built = "SN-1".into();
        args.firmware_id = "FW-1".into();
        args.calibration_id = "cal-1".into();
        args.design_content_hash = "des1".into();
        args.journal_path = Some(dir.join("driver.jsonl"));
        args.signing_key = Some(b"online-session-key".to_vec());
        args.first_online = true;
        args.actuator_ids = vec!["joint-0".into()];
        bind_online_plant(&mut plant, &args);
        let sess = RuntimeSession::<SimPlant, OnlineLocked>::start_online(args, plant).unwrap();
        let a = sess.governor.authority_now_s();
        std::thread::sleep(std::time::Duration::from_millis(8));
        let b = sess.governor.authority_now_s();
        assert!(a.is_finite() && b.is_finite());
        assert!(
            b + 1e-12 >= a,
            "OsMonotonicClock must be non-decreasing: {a} -> {b}"
        );
        assert!(
            b > a,
            "start_online must not freeze on a FakeClock: {a} -> {b}"
        );
    }
}
