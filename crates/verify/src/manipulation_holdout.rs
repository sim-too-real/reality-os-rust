//! First hold-out manipulation score. Do not edit semantics after MANIPULATION_V1_FREEZE_SHA.

use crate::bundle::RobotBundle;
use crate::manipulation::{
    run_grasp_matrix, run_push_matrix, run_release_matrix, write_phase_b_evidence, ManipulationMetrics,
};
use crate::menagerie::{
    ensure_manipulation_holdout_model, manipulation_holdout_bundle_dir, manipulation_holdout_model_dir,
    MENAGERIE_REPO, MENAGERIE_SHA,
};
use crate::resource_discover::discover_resources;
use crate::runner::load_and_normalize;
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};
use std::path::Path;

pub const MANIPULATION_HOLDOUT_ROBOT: &str = "menagerie_wx250s";

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ManipulationHoldoutFirstScore {
    pub source_repository: String,
    pub source_commit: String,
    pub manipulation_v1_freeze_sha: String,
    pub robot_id: String,
    pub model_hash: String,
    pub provenance: Value,
    pub resource_discovery: Value,
    pub resource_qualification: Value,
    pub release: Value,
    pub grasp: Value,
    pub push: Value,
    pub conclusion: String,
    pub metal: bool,
    pub evidence_status: String,
    pub simulation_only: bool,
}

pub fn write_holdout_bundle_yaml() -> Result<(), String> {
    let dir = manipulation_holdout_bundle_dir();
    std::fs::create_dir_all(&dir).map_err(|e| e.to_string())?;
    let yaml = dir.join("robot.yaml");
    if yaml.exists() {
        return Ok(());
    }
    let model = if manipulation_holdout_model_dir().join("wx250s.xml").exists() {
        "model/wx250s.xml"
    } else if manipulation_holdout_model_dir().join("wx250s_gripper.xml").exists() {
        "model/wx250s_gripper.xml"
    } else {
        "model/wx250s.xml"
    };
    std::fs::write(
        yaml,
        format!(
            "robot_id: {MANIPULATION_HOLDOUT_ROBOT}\n\
             model_format: mjcf\n\
             model_file: {model}\n\
             asset_roots:\n\
               - model\n\
             expected_base_type: fixed\n\
             joint_aliases: {{}}\n\
             actuator_aliases: {{}}\n\
             end_effectors:\n\
               - name: tool0\n\
                 body: gripper_link\n\
             grippers:\n\
               - name: parallel_gripper\n\
                 body: gripper_link\n\
             feet: []\n\
             cameras: []\n\
             task_frames:\n\
               - name: tool0\n\
                 body: gripper_link\n\
             collision_groups: {{}}\n\
             default_controller_profile: pd_position\n\
             source:\n\
               license: BSD-3-Clause\n\
               attribution: Official Google DeepMind MuJoCo Menagerie (trossen_wx250s).\n\
               redistributable: true\n\
               notes: Untouched official Menagerie import for Phase B hold-out. Not used during development.\n"
        ),
    )
    .map_err(|e| e.to_string())
}

pub fn run_manipulation_holdout_first_score(
    freeze_sha: &str,
) -> Result<ManipulationHoldoutFirstScore, String> {
    let provenance = ensure_manipulation_holdout_model()?;
    write_holdout_bundle_yaml()?;
    let bundle = RobotBundle::load(manipulation_holdout_bundle_dir()).map_err(|e| e.to_string())?;
    let (inst, manifest) = load_and_normalize(&bundle, &[], 0)?;
    let inspect = inst.inspect.clone();
    crate::mujoco_exec::checkin_worker(inst);
    let discovered = discover_resources(&bundle, &manifest, &inspect);
    let sha = freeze_sha.to_string();
    let (rel, rel_m) = run_release_matrix(&bundle, 20, &sha)?;
    let (gr, gr_m) = if discovered.is_empty() {
        (vec![], ManipulationMetrics::default())
    } else {
        run_grasp_matrix(&bundle, 20, &sha)?
    };
    let (pu, pu_m) = run_push_matrix(&bundle, 20, &sha)?;
    let conclusion = if !discovered.is_empty()
        && rel_m.release_success > 0
        && (gr_m.grasp_acquisition_success > 0 || pu_m.push_task_success > 0)
    {
        "EXTERNAL MANIPULATION GENERALIZATION DEMONSTRATED"
    } else {
        "MANIPULATION GENERALIZATION LIMIT FOUND"
    };
    Ok(ManipulationHoldoutFirstScore {
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
        manipulation_v1_freeze_sha: freeze_sha.into(),
        robot_id: bundle.manifest.robot_id,
        model_hash: manifest.model_hash,
        provenance,
        resource_discovery: json!(discovered.iter().map(|r| json!({
            "id": r.id, "topology": r.topology, "actuators": r.actuator_inputs,
            "joints": r.affected_joints, "unsupported": r.unsupported_detail
        })).collect::<Vec<_>>()),
        resource_qualification: json!({}),
        release: json!({"n": rel.len(), "metrics": rel_m}),
        grasp: json!({"n": gr.len(), "metrics": gr_m}),
        push: json!({"n": pu.len(), "metrics": pu_m}),
        conclusion: conclusion.into(),
        metal: false,
        evidence_status: crate::honesty::SIMULATION_ONLY.into(),
        simulation_only: true,
    })
}

pub fn write_holdout_score(score: &ManipulationHoldoutFirstScore, out_dir: &Path) -> Result<(), String> {
    std::fs::create_dir_all(out_dir).map_err(|e| e.to_string())?;
    let json = serde_json::to_string_pretty(score).map_err(|e| e.to_string())?;
    std::fs::write(out_dir.join("manipulation_holdout_first_score.json"), json)
        .map_err(|e| e.to_string())?;
    let _ = write_phase_b_evidence;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mujoco_exec::ensure_mujoco_or_skip;

    #[test]
    fn holdout_is_gated_on_freeze_sha() {
        if std::env::var("MANIPULATION_V1_FREEZE_SHA").unwrap_or_default().is_empty() {
            return;
        }
        if !ensure_mujoco_or_skip() {
            return;
        }
        let sha = std::env::var("MANIPULATION_V1_FREEZE_SHA").unwrap();
        if ensure_manipulation_holdout_model().is_err() {
            return;
        }
        let score = run_manipulation_holdout_first_score(&sha).expect("holdout first score");
        let evidence =
            Path::new(env!("CARGO_MANIFEST_DIR")).join("../../docs/superpowers/evidence");
        write_holdout_score(&score, &evidence).expect("write");
        assert_eq!(score.manipulation_v1_freeze_sha, sha);
        assert!(!score.metal);
        assert!(score.simulation_only);
    }
}
