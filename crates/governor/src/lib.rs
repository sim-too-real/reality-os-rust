//! Last driver gate. Identity + safety + envelope. Does not invent. Does not plan tasks.
//!
//! Distinct from any slide-task PhysicsGovernor. Foundry never spends through this crate.

pub mod envelope;
pub mod gate;
pub mod governor;
pub mod identity;
pub mod latch;
pub mod trace;

pub use envelope::{per_joint_clip, DriverEnvelopePack};
pub use gate::{admit_from_parts, GovernorGateRequest, GovernorGateVerdict};
pub use governor::{GovernorConfig, RuntimeGovernor};
pub use identity::RuntimeIdentity;
pub use latch::{EstopLatch, SafeState, SafeStateLatch};
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
    use realityos_plant::SimPlant;

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
        g.config.require_online_identity = true;
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
}
