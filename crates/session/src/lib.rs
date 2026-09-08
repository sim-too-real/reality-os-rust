//! RuntimeSession composition root.
//!
//! Foundry compiles embodiments. Reality OS compiles CertifiedCommand.
//! Governor proves permission. This crate alone binds identity and acknowledges.

pub mod mode;
pub mod packages;
pub mod session;

pub use mode::{RuntimeMode, SessionStartError};
pub use packages::{GovernorPackage, PackageReport, RealityOsPackage, SafetyEdge};
pub use session::{DispatchResult, RuntimeSession, StartArgs};

pub const SCHEMA: &str = "realityos.session/1";

#[cfg(test)]
mod tests {
    use super::*;
    use realityos_core::{Certificate, CertifiedCommand, Intent, RealityOs, WorldView};
    use realityos_kernel::DecisionStatus;
    use realityos_plant::{ActionParams, SimPlant};

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
        let mut cmd = d.command.unwrap();
        cmd.release_hash.clear();
        let out = sess.bind_and_dispatch(cmd, &ActionParams::empty(), 10.0);
        assert!(out.ok, "{:?}", out.violations);
        assert_eq!(sess.governor.plant.write_count(), 1);
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
        assert_eq!(sess.governor.plant.write_count(), 0);
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
}
