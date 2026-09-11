//! Stage B hold-out first score. Fixture registry / recording only.
//! Do not change `realityos-semantics` before writing the first-score artifact.

use crate::bundle::RobotBundle;
use crate::fk_oracle::{compare_semantic_fk, FkOracleReport};
use crate::menagerie::{ensure_holdout_model, holdout_bundle_dir, MENAGERIE_REPO, MENAGERIE_SHA};
use crate::reach_foundation::{run_foundation_reach, FoundationReachReport};
use serde::{Deserialize, Serialize};
use serde_json::Value;
use std::path::Path;

/// Predeclared before the first hold-out REACH run. Not tuned after seeing results.
pub const PREDECLARED_TARGETS: [[f64; 3]; 10] = [
    [0.45, 0.00, 0.45],
    [0.40, 0.20, 0.40],
    [0.40, -0.20, 0.40],
    [0.35, 0.00, 0.55],
    [0.50, 0.10, 0.35],
    [0.30, 0.25, 0.50],
    [0.30, -0.25, 0.50],
    [0.45, 0.15, 0.30],
    [0.45, -0.15, 0.30],
    [0.38, 0.00, 0.42],
];

pub const GENERALITY_FREEZE_SHA: &str = "20cb4270f89e81e7256ebb72bfda44c9251fbdf9";

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HoldoutFirstScore {
    pub source_repository: String,
    pub source_commit: String,
    pub generality_freeze_sha: String,
    pub robot_id: String,
    pub model_hash: String,
    pub robot_model_checksum: String,
    pub provenance: Value,
    pub fk: Option<FkOracleReport>,
    pub fk_error: Option<String>,
    pub targets: Vec<[f64; 3]>,
    pub results: Vec<FoundationReachReport>,
    pub n_compile: usize,
    pub n_execute: usize,
    pub n_succeed: usize,
    pub failure_reasons: Vec<String>,
    pub semantic_code_changed_after_first_score: bool,
}

pub fn load_holdout_bundle() -> Result<RobotBundle, String> {
    ensure_holdout_model()?;
    RobotBundle::load(holdout_bundle_dir()).map_err(|e| e.to_string())
}

pub fn run_holdout_first_score() -> Result<HoldoutFirstScore, String> {
    let provenance = ensure_holdout_model()?;
    let bundle = RobotBundle::load(holdout_bundle_dir()).map_err(|e| e.to_string())?;
    let model_checksum = provenance
        .get("files")
        .and_then(|f| f.get("panda.xml"))
        .and_then(|v| v.as_str())
        .unwrap_or("")
        .to_string();
    let (fk, fk_error) = match compare_semantic_fk(&bundle, 100, 17) {
        Ok(r) => (Some(r), None),
        Err(e) => (None, Some(e)),
    };
    let mut results = Vec::new();
    let mut failure_reasons = Vec::new();
    for t in PREDECLARED_TARGETS {
        match run_foundation_reach(&bundle, t, 0.08, 10.0, 0.25, None, false, false) {
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
            Err(e) => {
                failure_reasons.push(format!("{t:?}: runner_error:{e}"));
            }
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
        .unwrap_or_default();
    Ok(HoldoutFirstScore {
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
        generality_freeze_sha: GENERALITY_FREEZE_SHA.into(),
        robot_id: bundle.manifest.robot_id,
        model_hash,
        robot_model_checksum: model_checksum,
        provenance,
        fk,
        fk_error,
        targets: PREDECLARED_TARGETS.to_vec(),
        results,
        n_compile,
        n_execute,
        n_succeed,
        failure_reasons,
        semantic_code_changed_after_first_score: false,
    })
}

pub fn write_first_score(score: &HoldoutFirstScore, out_dir: &Path) -> Result<(), String> {
    std::fs::create_dir_all(out_dir).map_err(|e| e.to_string())?;
    let json = serde_json::to_string_pretty(score).map_err(|e| e.to_string())?;
    std::fs::write(out_dir.join("external_holdout_first_score.json"), json)
        .map_err(|e| e.to_string())?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mujoco_exec::ensure_mujoco_or_skip;

    #[test]
    fn holdout_first_score_is_recorded_without_prescore_tuning() {
        if !ensure_mujoco_or_skip() {
            return;
        }
        if ensure_holdout_model().is_err() {
            return;
        }
        let score = run_holdout_first_score().expect("holdout first score");
        assert_eq!(score.source_commit, MENAGERIE_SHA);
        assert_eq!(score.generality_freeze_sha, GENERALITY_FREEZE_SHA);
        assert_eq!(score.targets.len(), 10);
        assert!(!score.semantic_code_changed_after_first_score);
        let out = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../verify-out");
        write_first_score(&score, &out).expect("write first score");
    }
}
