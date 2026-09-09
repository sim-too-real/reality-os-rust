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
pub use estimate::{FuseEstimator, HoldEstimator};
pub use mode::{RuntimeMode, SessionStartError};
pub use packages::{GovernorPackage, PackageReport, RealityOsPackage, SafetyEdge};
pub use session::{DispatchResult, RuntimeSession, StartArgs};

pub const SCHEMA: &str = "realityos.session/1";

#[cfg(test)]
mod tests {
    use super::*;
    use realityos_core::{Certificate, CertifiedCommand, Intent, RealityOs, WorldView};
    use realityos_kernel::DecisionStatus;
    use realityos_plant::{ActionParams, Plant, SimPlant};

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
        match RuntimeSession::start(args, plant, 1.0) {
            Err(err) => assert!(err.0.contains("online_refuses_safety_rail_opt_out")),
            Ok(_) => panic!("expected online_refuses_safety_rail_opt_out"),
        }
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
        let cmd = d.command.unwrap();
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
        let cmd = CertifiedCommand::issue("x", 1, 1.0, 30.0, cert, vec![0.1]).unwrap();
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
        let cmd = CertifiedCommand::issue("x", 1, 1.0, 30.0, cert, vec![0.1]).unwrap();
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
    fn journal_kill_restart_refuses_replay_and_restores_estop() {
        let dir = std::env::temp_dir().join(format!(
            "realityos-journal-{}-kill",
            std::process::id()
        ));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("session.jsonl");
        let _ = std::fs::remove_file(&path);

        let plant = SimPlant::new("p", 1, 10.0);
        let mut args = StartArgs::simulation("rel-journal-aa");
        args.journal_path = Some(path.clone());
        let mut sess = RuntimeSession::start(args.clone(), plant, 10.0).unwrap();
        sess.governor.mark_sensor(10.0, None);
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
        let replay = cmd.clone();
        let out = sess.bind_and_dispatch(cmd, &ActionParams::empty(), 10.0);
        assert!(out.ok, "{:?}", out.violations);
        sess.governor.engage_estop("kill_test", 10.0);
        drop(sess);

        let plant2 = SimPlant::new("p", 1, 10.0);
        let mut sess2 = RuntimeSession::start(args, plant2, 10.0).unwrap();
        assert!(sess2.governor.estop());
        sess2.governor.mark_sensor(10.0, None);
        let out2 = sess2.bind_and_dispatch(replay, &ActionParams::empty(), 10.0);
        assert!(!out2.ok);
        assert_eq!(sess2.governor.plant().write_count(), 0);
        let blob = out2.violations.join(" ");
        assert!(
            blob.contains("estop") || blob.contains("replayed") || blob.contains("safe_state"),
            "{blob}"
        );
    }

    #[test]
    fn journal_missing_file_is_empty_genesis_and_tamper_fails_closed() {
        let dir = std::env::temp_dir().join(format!(
            "realityos-journal-{}-tamper",
            std::process::id()
        ));
        std::fs::create_dir_all(&dir).unwrap();
        let missing = dir.join("does-not-exist-yet.jsonl");
        let _ = std::fs::remove_file(&missing);
        let plant = SimPlant::new("p", 1, 10.0);
        let mut args = StartArgs::simulation("rel-journal-bb");
        args.journal_path = Some(missing.clone());
        let sess = RuntimeSession::start(args, plant, 1.0);
        assert!(sess.is_ok(), "{:?}", sess.err());

        let path = dir.join("tamper.jsonl");
        let _ = std::fs::remove_file(&path);
        let mut ledger = realityos_plant::CommandLedger::with_journal(&path, true).unwrap();
        let mut body = serde_json::Map::new();
        body.insert("event".into(), serde_json::json!("heartbeat"));
        ledger
            .append_event("governor_event", body)
            .unwrap();
        let raw = std::fs::read_to_string(&path).unwrap();
        let tampered = raw.replace("prev_hash", "prev_XXXX");
        std::fs::write(&path, tampered).unwrap();
        let mut args = StartArgs::simulation("rel-journal-cc");
        args.journal_path = Some(path);
        args.journal_fail_closed = true;
        let plant = SimPlant::new("p", 1, 10.0);
        match RuntimeSession::start(args, plant, 1.0) {
            Err(err) => assert!(
                err.0.contains("journal") || err.0.contains("unreadable"),
                "{}",
                err.0
            ),
            Ok(_) => panic!("expected journal tamper to fail closed"),
        }
    }

    #[test]
    fn mailbox_overload_latches_hold_without_writing() {
        let plant = SimPlant::new("p", 1, 10.0);
        let mut sess =
            RuntimeSession::start(StartArgs::simulation("rel-bus-1"), plant, 1.0).unwrap();
        for i in 0..32 {
            sess.push_outbound(realityos_rate::TelemetryFrame::new("t", 1.0, i.to_string()))
                .unwrap();
        }
        assert_eq!(
            sess.push_outbound(realityos_rate::TelemetryFrame::new("t", 1.0, "x")),
            Err(realityos_rate::OverloadDisposition::Hold)
        );
        let cert = Certificate::new(DecisionStatus::Allow, "ok");
        let cmd = CertifiedCommand::issue("bus1", 1, 1.0, 30.0, cert, vec![0.1]).unwrap();
        let out = sess.bind_and_dispatch(cmd, &ActionParams::empty(), 1.0);
        assert!(!out.ok);
        assert_eq!(sess.governor.plant().write_count(), 0);
    }

    #[test]
    fn sensor_timestamp_rollback_is_refused() {
        let plant = SimPlant::new("p", 1, 10.0);
        let mut sess =
            RuntimeSession::start(StartArgs::simulation("rel-roll-1"), plant, 5.0).unwrap();
        sess.ingest_sensor(&[("j".into(), 0.1)], Some(4.0), 5.0)
            .unwrap();
        let err = sess
            .ingest_sensor(&[("j".into(), 0.2)], Some(3.0), 5.0)
            .unwrap_err();
        assert!(err.contains("rollback"));
    }

    #[test]
    fn session_fuse_estimator_degrades_without_observation() {
        let plant = SimPlant::new("p", 1, 10.0);
        let mut sess =
            RuntimeSession::start(StartArgs::simulation("rel-fuse-1"), plant, 1.0).unwrap();
        sess.ingest_sensor(&[("q0".into(), 0.1)], Some(1.0), 1.0)
            .unwrap();
        assert_eq!(
            sess.belief().unwrap().health(),
            realityos_kernel::SensorHealth::Degraded
        );
        sess.ingest_observation(
            realityos_kernel::ObservationEvidence::new(
                "cam0",
                "cal0",
                1.0,
                1.0,
                "digest-ok",
                "vision/optical",
                0.9,
                0.1,
                2.0,
            )
            .unwrap(),
            1.0,
        )
        .unwrap();
        assert_eq!(
            sess.belief().unwrap().health(),
            realityos_kernel::SensorHealth::Ok
        );
    }

    #[test]
    fn hil_software_sto_blocks_dispatch_until_enable() {
        let plant = SimPlant::new("p", 1, 10.0);
        let mut args = StartArgs::simulation("rel-sto-1");
        args.mode = RuntimeMode::Hil;
        let mut sess = RuntimeSession::start(args, plant, 1.0).unwrap();
        sess.governor.mark_sensor(1.0, None);
        let cert = Certificate::new(DecisionStatus::Allow, "ok");
        let cmd = CertifiedCommand::issue("sto1", 1, 1.0, 30.0, cert, vec![0.1]).unwrap();
        let out = sess.bind_and_dispatch(cmd.clone(), &ActionParams::empty(), 1.0);
        assert!(!out.ok);
        assert!(out.violations.iter().any(|v| v.contains("software_sto")));
        let mut frame = realityos_plant::SafetyFrame::request(
            1,
            10.0,
            realityos_plant::SafeTransition::EnableRequest,
        )
        .unwrap();
        frame.ack = true;
        assert_eq!(
            sess.ingest_safety(&frame, 1.0),
            realityos_plant::IslandVerdict::Ack
        );
        assert!(sess.island().enabled());
        assert!(!sess.island().metal());
        let out2 = sess.bind_and_dispatch(cmd, &ActionParams::empty(), 1.0);
        assert!(out2.ok, "{:?}", out2.violations);
    }

    #[test]
    fn fieldbus_enable_requires_software_island() {
        let mut br = HardwareControlBridge::sim_harness("rel-bus-island", 1.0).unwrap();
        assert!(br.request_fieldbus_enable().is_err());
        assert!(!br.fieldbus.attached);
        let mut frame = realityos_plant::SafetyFrame::request(
            1,
            10.0,
            realityos_plant::SafeTransition::EnableRequest,
        )
        .unwrap();
        frame.ack = true;
        assert_eq!(
            br.session.ingest_safety(&frame, 1.0),
            realityos_plant::IslandVerdict::Ack
        );
        assert!(br.request_fieldbus_enable().is_err());
        assert!(!br.fieldbus.attached);
        assert!(!br.fieldbus.metal);
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
}
