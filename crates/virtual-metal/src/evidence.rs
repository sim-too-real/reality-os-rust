//! Virtual Metal evidence. Cannot mint MEASURED or metal_proof.

use realityos_kernel::HonestyStamp;
use realityos_metal::{MetalProof, ProofMeta};
use serde::{Deserialize, Serialize};

pub const CAMPAIGN_SCHEMA: &str = "realityos.virtual_metal/1";
pub const EVIDENCE_STATUS: &str = "SIM_VIRTUAL_METAL_NOT_METAL";
pub const VERDICT_PASS: &str = "VIRTUAL_METAL_PASS";
pub const VERDICT_FAIL: &str = "VIRTUAL_METAL_FAIL";

pub fn honesty() -> HonestyStamp {
    HonestyStamp::sim(EVIDENCE_STATUS).expect("SIM_VIRTUAL_METAL_NOT_METAL is a legal sim token")
}

pub fn refuse_measured_token(token: &str) -> Result<(), String> {
    HonestyStamp::sim(token)
        .map(|_| ())
        .map_err(|e| e.to_string())
}

pub fn refuse_hardware_present_true() -> Result<(), String> {
    Err("virtual_metal_refuses_hardware_present=true".into())
}

pub fn refuse_metal_proof_install(meta: ProofMeta) -> Result<MetalProof, String> {
    if meta.hardware_present {
        return Err("virtual_metal_refuses_metal_proof_with_hardware_present".into());
    }
    MetalProof::from_measured(meta, vec![], vec![])
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CampaignRow {
    pub instance: u64,
    pub seed: u64,
    pub scenario: String,
    pub verdict: String,
    pub physical_actions: u64,
    pub invariant_violations: Vec<String>,
    pub fault_sequence: serde_json::Value,
    pub truth_pack_schema: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CampaignRecord {
    pub schema: String,
    pub verdict: String,
    pub hardware_present: bool,
    pub evidence_status: String,
    pub honesty_evidence_status: String,
    pub seed: u64,
    pub n_instances: usize,
    pub n_ok: usize,
    pub n_fail: usize,
    pub note: String,
    pub rows: Vec<CampaignRow>,
}

impl CampaignRecord {
    pub fn assemble(seed: u64, rows: Vec<CampaignRow>) -> Self {
        let n_fail = rows
            .iter()
            .filter(|r| !r.invariant_violations.is_empty() || r.verdict == VERDICT_FAIL)
            .count();
        let n_ok = rows.len().saturating_sub(n_fail);
        let verdict = if n_fail == 0 {
            VERDICT_PASS
        } else {
            VERDICT_FAIL
        };
        let honesty = honesty();
        Self {
            schema: CAMPAIGN_SCHEMA.into(),
            verdict: verdict.into(),
            hardware_present: false,
            evidence_status: EVIDENCE_STATUS.into(),
            honesty_evidence_status: honesty.evidence_status().into(),
            seed,
            n_instances: rows.len(),
            n_ok,
            n_fail,
            note: "VIRTUAL_METAL_PASS is not physical hardware verification. It cannot mint MEASURED, METAL_MEASURED, hardware_present=true, or docs/metal_proof.json.".into(),
            rows,
        }
    }

    pub fn try_set_hardware_present(&mut self, present: bool) -> Result<(), String> {
        if present {
            return refuse_hardware_present_true();
        }
        self.hardware_present = false;
        Ok(())
    }
}
