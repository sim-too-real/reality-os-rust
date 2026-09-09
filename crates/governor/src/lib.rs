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
pub use governor::{GovernorConfig, OnlineInitError, RuntimeGovernor};
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
}
