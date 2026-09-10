//! Simulation honesty. SIM ≠ METAL. This crate cannot mint physical proof.

use realityos_kernel::{EvidenceTier, HonestyStamp};
use serde::{Deserialize, Serialize};

/// Port identity token required by the specification.
pub const SIMULATION_ONLY: &str = "SIMULATION_ONLY";

/// HonestyStamp-legal token (must start with SIM_ or contain NOT_METAL).
pub const SIM_VERIFY_NOT_METAL: &str = "SIM_MUJOCO_VERIFY_NOT_METAL";

pub const EVIDENCE_SCHEMA: &str = "realityos.simulation_evidence/1";
pub const REPORT_SCHEMA: &str = "realityos.simulation_verification_report/1";
pub const METAL_PROOF_SCHEMA: &str = "realityos.metal_proof/1";
pub const PTY_STAND_IN: &str = "PTY_STAND_IN_NOT_METAL";

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum VerificationClass {
    SimulationVerified,
    MetalVerified,
    FunctionalSafetyCertified,
}

impl VerificationClass {
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::SimulationVerified => "SIMULATION_VERIFIED",
            Self::MetalVerified => "METAL_VERIFIED",
            Self::FunctionalSafetyCertified => "FUNCTIONAL-SAFETY CERTIFIED",
        }
    }

    pub const fn this_crate_may_emit(self) -> bool {
        matches!(self, Self::SimulationVerified)
    }
}

pub fn simulation_honesty() -> HonestyStamp {
    HonestyStamp::sim(SIM_VERIFY_NOT_METAL).expect("sim honesty token is well-formed")
}

pub fn evidence_tier() -> EvidenceTier {
    EvidenceTier::SimVerified
}

/// Refuse any attempt to treat this harness as metal / PTY / certified proof.
pub fn refuse_physical_proof_origin(value: &serde_json::Value) -> Result<(), String> {
    if let Some(schema) = value.get("schema").and_then(|v| v.as_str()) {
        if schema == METAL_PROOF_SCHEMA || schema.contains("metal_proof") {
            return Err("simulation_harness_cannot_emit_realityos.metal_proof".into());
        }
    }
    if value.get("metal").and_then(|v| v.as_bool()) == Some(true) {
        return Err("simulation_harness_cannot_set_metal_true".into());
    }
    if value.get("metal_verified").and_then(|v| v.as_bool()) == Some(true) {
        return Err("simulation_harness_cannot_claim_METAL_VERIFIED".into());
    }
    if value
        .get("functional_safety_certified")
        .and_then(|v| v.as_bool())
        == Some(true)
    {
        return Err("simulation_harness_cannot_claim_FUNCTIONAL_SAFETY_CERTIFIED".into());
    }
    let status = value
        .get("evidence_status")
        .and_then(|v| v.as_str())
        .unwrap_or("");
    if status == PTY_STAND_IN || status.contains("PTY_STAND_IN") {
        return Err("simulation_harness_cannot_originate_pty_metal_evidence".into());
    }
    if value
        .get("verification_class")
        .and_then(|v| v.as_str())
        .is_some_and(|c| {
            c == VerificationClass::MetalVerified.as_str() || c.contains("METAL_VERIFIED")
        })
    {
        return Err("simulation_harness_cannot_claim_METAL_VERIFIED".into());
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn honesty_stamp_is_never_metal() {
        let h = simulation_honesty();
        assert!(!h.metal());
        assert!(!h.measured());
        assert_eq!(h.evidence_status(), SIM_VERIFY_NOT_METAL);
        assert!(VerificationClass::SimulationVerified.this_crate_may_emit());
        assert!(!VerificationClass::MetalVerified.this_crate_may_emit());
        assert!(!VerificationClass::FunctionalSafetyCertified.this_crate_may_emit());
        assert!(!evidence_tier().is_measured());
    }

    #[test]
    fn metal_proof_schema_is_refused() {
        let v = serde_json::json!({"schema": METAL_PROOF_SCHEMA, "metal": false});
        assert!(refuse_physical_proof_origin(&v).is_err());
        let pty = serde_json::json!({"schema": EVIDENCE_SCHEMA, "evidence_status": PTY_STAND_IN});
        assert!(refuse_physical_proof_origin(&pty).is_err());
        let ok = serde_json::json!({
            "schema": EVIDENCE_SCHEMA,
            "metal": false,
            "evidence_status": SIMULATION_ONLY,
            "verification_class": "SIMULATION_VERIFIED"
        });
        assert!(refuse_physical_proof_origin(&ok).is_ok());
    }
}
