//! Plant trait, command ledger, certified-write uniqueness.
//!
//! Governor depends on this crate. Reality OS issues commands that implement
//! [`ActuationCommand`]. No invent. No learned last-write.

pub mod caps;
pub mod command;
pub mod error;
pub mod execute;
pub mod ledger;
pub mod signing;
pub mod sim;
pub mod traits;
pub mod write_guard;

pub use caps::{ActionParams, PlantCaps, PlantRealized};
pub use command::{ActuationCommand, ExecuteBind};
pub use error::{PlantError, PlantResult};
pub use execute::{execute_certified_command, ExecuteResult};
pub use ledger::{CommandLedger, ContinuityState};
pub use signing::{
    action_within_issuer_envelope, command_payload_hash, sign_payload, signature_violations,
    SIGNING_SCHEME,
};
pub use sim::SimPlant;
pub use traits::{HardwareDriverPort, HardwareIdentity, Plant, SensorPacket};
pub use write_guard::{
    in_certified_write, plant_requires_certified_write, refuse_uncertified_online_write,
    with_certified_write,
};

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn online_act_without_scope_refuses() {
        let mut p = SimPlant::new("stub", 1, 1.0);
        p.go_online();
        let err = p.act(&[0.1], &ActionParams::empty()).unwrap_err();
        match err {
            PlantError::UncertifiedOnline { method } => assert_eq!(method, "act"),
            other => panic!("{other:?}"),
        }
        assert_eq!(p.write_count(), 0);
    }

    #[test]
    fn certified_scope_allows_online_act() {
        let mut p = SimPlant::new("stub", 1, 1.0);
        p.go_online();
        let out = with_certified_write(|| p.act(&[0.2], &ActionParams::empty())).unwrap();
        assert!(out.ok);
        assert_eq!(p.write_count(), 1);
        assert!(!out.metal);
    }
}
