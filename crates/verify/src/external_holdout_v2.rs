//! Untouched second-external-arm first score. Fixture registry / recording only.
//! Do not change `realityos-semantics` after `GENERALITY_V2_FREEZE_SHA`.

use crate::bundle::RobotBundle;
use crate::external_generality::sample_reachable_world_targets_with_bounds;
use crate::fk_oracle::{compare_semantic_fk, FkOracleReport};
use crate::menagerie::{
    ensure_v2_holdout_model, v2_holdout_bundle_dir, MENAGERIE_REPO, MENAGERIE_SHA,
};
use crate::reach_foundation::{run_foundation_reach, FoundationReachReport};
use crate::runner::load_and_normalize;
use crate::semantics_map::embodiment_from_manifest;
use realityos_semantics::capability::derive_capabilities;
use serde::{Deserialize, Serialize};
use serde_json::Value;
use std::path::Path;

pub const GENERALITY_V2_FREEZE_SHA: &str = "b580c6275acc797c7433faa490cc712107b0adb5";

pub const V2_TARGET_PROCEDURE: &str =
    "sample_10_privileged_ee_world_points_zgt0.05_xygt0.08_rlt1.4";

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HoldoutV2FirstScore {
    pub source_repository: String,
    pub source_commit: String,
    pub generality_v2_freeze_sha: String,
    pub robot_id: String,
    pub model_hash: String,
    pub robot_model_checksum: String,
    pub provenance: Value,
    pub diagnostics: Vec<Value>,
    pub capabilities: Value,
    pub fk: Option<FkOracleReport>,
    pub fk_error: Option<String>,
    pub target_procedure: String,
    pub targets: Vec<[f64; 3]>,
    pub results: Vec<FoundationReachReport>,
    pub n_compile: usize,
    pub n_execute: usize,
    pub n_succeed: usize,
    pub failure_reasons: Vec<String>,
}

pub fn run_holdout_v2_first_score(freeze_sha: &str) -> Result<HoldoutV2FirstScore, String> {
    let provenance = ensure_v2_holdout_model()?;
    let bundle = RobotBundle::load(v2_holdout_bundle_dir()).map_err(|e| e.to_string())?;
    let model_checksum = provenance
        .get("files")
        .and_then(|f| f.get("iiwa14.xml"))
        .and_then(|v| v.as_str())
        .unwrap_or("")
        .to_string();
    let (inst, manifest) = load_and_normalize(&bundle, &[], 0)?;
    crate::mujoco_exec::checkin_worker(inst);
    let model = embodiment_from_manifest(&bundle, &manifest);
    let caps = derive_capabilities(&model, None);
    let diagnostics = model
        .diagnostics
        .iter()
        .map(|d| serde_json::json!({"code": d.code, "detail": d.detail}))
        .collect();
    let capabilities = serde_json::to_value(caps.nodes()).map_err(|e| e.to_string())?;
    let (fk, fk_error) = match compare_semantic_fk(&bundle, 100, 19) {
        Ok(r) => (Some(r), None),
        Err(e) => (None, Some(e)),
    };
    let targets = sample_reachable_world_targets_with_bounds(&bundle, 10, 23, 0.05, 0.08, 1.4)?;
    let mut results = Vec::new();
    let mut failure_reasons = Vec::new();
    for t in &targets {
        match run_foundation_reach(&bundle, *t, 0.08, 10.0, 0.25, None, false, false) {
            Ok(r) => {
                if let Some(refu) = &r.skill_refuse {
                    failure_reasons.push(format!("{t:?}: {refu}"));
                } else if !r.task_success {
                    failure_reasons.push(format!(
                        "{t:?}: execute_miss cart={:?} ik={:?}",
                        r.cartesian_residual, r.ik_residual
                    ));
                }
                results.push(r);
            }
            Err(e) => failure_reasons.push(format!("{t:?}: runner_error:{e}")),
        }
    }
    let n_compile = results.iter().filter(|r| r.skill_refuse.is_none()).count();
    let n_execute = results.iter().filter(|r| r.ctrl_writes > 0).count();
    let n_succeed = results
        .iter()
        .filter(|r| r.task_success && r.skill_refuse.is_none())
        .count();
    let model_hash = results
        .first()
        .map(|r| r.model_hash.clone())
        .unwrap_or_else(|| model.model_hash.clone());
    Ok(HoldoutV2FirstScore {
        source_repository: provenance
            .get("source_repository")
            .and_then(|v| v.as_str())
            .unwrap_or(MENAGERIE_REPO)
            .into(),
        source_commit: provenance
            .get("source_commit")
            .and_then(|v| v.as_str())
            .unwrap_or(MENAGERIE_SHA)
            .into(),
        generality_v2_freeze_sha: freeze_sha.into(),
        robot_id: bundle.manifest.robot_id,
        model_hash,
        robot_model_checksum: model_checksum,
        provenance,
        diagnostics,
        capabilities,
        fk,
        fk_error,
        target_procedure: V2_TARGET_PROCEDURE.into(),
        targets,
        results,
        n_compile,
        n_execute,
        n_succeed,
        failure_reasons,
    })
}

pub fn write_v2_first_score(score: &HoldoutV2FirstScore, out_dir: &Path) -> Result<(), String> {
    std::fs::create_dir_all(out_dir).map_err(|e| e.to_string())?;
    let json = serde_json::to_string_pretty(score).map_err(|e| e.to_string())?;
    std::fs::write(out_dir.join("external_holdout_v2_first_score.json"), json)
        .map_err(|e| e.to_string())?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mujoco_exec::ensure_mujoco_or_skip;

    #[test]
    fn v2_holdout_first_score_is_recorded_after_freeze() {
        if !ensure_mujoco_or_skip() {
            return;
        }
        if !v2_holdout_bundle_dir().join("robot.yaml").exists() {
            return;
        }
        if ensure_v2_holdout_model().is_err() {
            return;
        }
        let sha = std::env::var("GENERALITY_V2_FREEZE_SHA").unwrap_or_default();
        if sha.is_empty() {
            return;
        }
        let score = run_holdout_v2_first_score(&sha).expect("v2 first score");
        let evidence =
            Path::new(env!("CARGO_MANIFEST_DIR")).join("../../docs/superpowers/evidence");
        let out = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../verify-out");
        write_v2_first_score(&score, &evidence).expect("write v2 evidence");
        write_v2_first_score(&score, &out).expect("write v2 verify-out");
        assert_eq!(score.targets.len(), 10);
        assert_eq!(score.generality_v2_freeze_sha, sha);
    }
}
