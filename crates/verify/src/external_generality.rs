//! External-robot generality campaign. Stage A development model only.

use crate::bundle::RobotBundle;
use crate::fk_oracle::{compare_semantic_fk, FkOracleReport};
use crate::menagerie::{
    development_bundle_dir, ensure_development_model, MENAGERIE_REPO, MENAGERIE_SHA,
};
use crate::mujoco_exec::checkin_worker;
use crate::observation::VerifierTruth;
use crate::reach_foundation::{run_foundation_reach_on, FoundationReachReport};
use crate::runner::load_and_normalize;
use crate::semantics_map::embodiment_from_manifest;
use realityos_semantics::capability::derive_capabilities;
use serde::{Deserialize, Serialize};
use serde_json::Value;
use std::path::Path;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExternalMatrixReport {
    pub source_repository: String,
    pub source_commit: String,
    pub robot_id: String,
    pub model_hash: String,
    pub fk: FkOracleReport,
    pub reachable: Vec<FoundationReachReport>,
    pub unreachable: Vec<FoundationReachReport>,
    pub stale: Vec<FoundationReachReport>,
    pub wrong_epoch: Vec<FoundationReachReport>,
    pub success_rate: f64,
    pub zero_write_invalid: bool,
    pub embodiment_summary: Value,
    pub capabilities: Value,
    pub provenance: Value,
}

pub fn load_development_bundle() -> Result<RobotBundle, String> {
    ensure_development_model()?;
    RobotBundle::load(development_bundle_dir()).map_err(|e| e.to_string())
}

pub fn sample_reachable_world_targets(
    bundle: &RobotBundle,
    n: usize,
    seed: u64,
) -> Result<Vec<[f64; 3]>, String> {
    sample_reachable_world_targets_with_bounds(bundle, n, seed, 0.05, 0.12, 1.2)
}

pub fn sample_reachable_world_targets_with_bounds(
    bundle: &RobotBundle,
    n: usize,
    seed: u64,
    z_min: f64,
    xy_min: f64,
    r_max: f64,
) -> Result<Vec<[f64; 3]>, String> {
    let (mut inst, manifest) = load_and_normalize(bundle, &[], seed)?;
    let model = embodiment_from_manifest(bundle, &manifest);
    let ee = bundle
        .manifest
        .end_effectors
        .first()
        .and_then(|e| e.site.clone().or_else(|| e.body.clone()))
        .unwrap_or_else(|| "ee".into());
    let mut rng = seed;
    let mut out = Vec::new();
    let mut qpos = vec![0.0; manifest.nq.max(0) as usize];
    for _ in 0..n * 8 {
        if out.len() >= n {
            break;
        }
        for j in &model.joints {
            rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1);
            let u = (rng >> 33) as f64 / (1u64 << 31) as f64;
            let lo = j.q_min.value.unwrap_or(-1.0);
            let hi = j.q_max.value.unwrap_or(1.0);
            if let Some(adr) = j.qpos_adr {
                if let Some(slot) = qpos.get_mut(adr as usize) {
                    *slot = lo + (hi - lo) * u.clamp(0.05, 0.95);
                }
            }
        }
        let st = inst
            .reset(Some(&qpos), Some(&vec![0.0; manifest.nv.max(0) as usize]))
            .map_err(|e| e.to_string())?;
        let truth = VerifierTruth::from_mujoco_state(st.get("state").unwrap_or(&st));
        if let Some(p) = truth.named_pos.get(&ee).or_else(|| truth.xpos.get(&ee)) {
            if p.len() >= 3 && p.iter().all(|v| v.is_finite()) {
                let r = (p[0] * p[0] + p[1] * p[1] + p[2] * p[2]).sqrt();
                let xy = (p[0] * p[0] + p[1] * p[1]).sqrt();
                if p[2] > z_min && xy > xy_min && r < r_max {
                    out.push([p[0], p[1], p[2]]);
                }
            }
        }
    }
    checkin_worker(inst);
    if out.len() < n {
        return Err(format!("only sampled {} reachable targets", out.len()));
    }
    Ok(out)
}

pub fn run_development_matrix() -> Result<ExternalMatrixReport, String> {
    let provenance = ensure_development_model()?;
    let bundle = RobotBundle::load(development_bundle_dir()).map_err(|e| e.to_string())?;
    let fk = compare_semantic_fk(&bundle, 100, 11)?;
    if fk.max_position_error > 1e-5 {
        return Err(format!(
            "FK oracle failed max_position_error={}",
            fk.max_position_error
        ));
    }
    let targets = sample_reachable_world_targets(&bundle, 20, 13)?;
    let (mut inst, manifest) = load_and_normalize(&bundle, &[], 0)?;
    let mut reachable = Vec::new();
    for t in targets {
        let (report, next) = run_foundation_reach_on(
            &bundle, inst, &manifest, t, 0.08, 10.0, 0.25, None, false, false,
        )?;
        inst = next;
        reachable.push(report);
    }
    let far = [
        [8.0, 0.0, 0.5],
        [-8.0, 0.0, 0.5],
        [0.0, 8.0, 0.5],
        [0.0, -8.0, 0.5],
        [0.0, 0.0, 8.0],
    ];
    let mut unreachable = Vec::new();
    for t in far {
        let (report, next) = run_foundation_reach_on(
            &bundle, inst, &manifest, t, 0.05, 10.0, 0.25, None, false, false,
        )?;
        inst = next;
        unreachable.push(report);
    }
    let mut stale = Vec::new();
    for t in reachable.iter().take(5).map(|r| {
        let _ = r;
        [0.3, 0.1, 0.4]
    }) {
        let (report, next) = run_foundation_reach_on(
            &bundle, inst, &manifest, t, 0.08, 10.0, 0.25, None, true, false,
        )?;
        inst = next;
        stale.push(report);
    }
    let mut wrong_epoch = Vec::new();
    for t in [[0.3, 0.1, 0.4]; 5] {
        let (report, next) = run_foundation_reach_on(
            &bundle,
            inst,
            &manifest,
            t,
            0.08,
            10.0,
            0.25,
            Some("deadbeef"),
            false,
            false,
        )?;
        inst = next;
        wrong_epoch.push(report);
    }
    let ok = reachable
        .iter()
        .filter(|r| r.task_success && r.skill_refuse.is_none())
        .count();
    let zero_write_invalid = unreachable
        .iter()
        .chain(stale.iter())
        .chain(wrong_epoch.iter())
        .all(|r| r.ctrl_writes == 0);
    let model_hash = reachable
        .first()
        .map(|r| r.model_hash.clone())
        .or_else(|| unreachable.first().map(|r| r.model_hash.clone()))
        .unwrap_or_default();
    let model = embodiment_from_manifest(&bundle, &manifest);
    checkin_worker(inst);
    let caps = derive_capabilities(
        &model,
        Some(fk.max_position_error <= 1e-5 && ok == reachable.len()),
    );
    Ok(ExternalMatrixReport {
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
        robot_id: bundle.manifest.robot_id,
        model_hash,
        fk,
        success_rate: ok as f64 / reachable.len().max(1) as f64,
        reachable,
        unreachable,
        stale,
        wrong_epoch,
        zero_write_invalid,
        embodiment_summary: embodiment_summary(&model),
        capabilities: serde_json::to_value(caps.nodes()).unwrap_or(Value::Null),
        provenance,
    })
}

fn embodiment_summary(model: &realityos_semantics::embodiment::EmbodimentModel) -> Value {
    serde_json::json!({
        "robot_id": model.robot_id,
        "model_hash": model.model_hash,
        "base": format!("{:?}", model.base),
        "n_bodies": model.bodies.len(),
        "n_joints": model.joints.len(),
        "n_actuators": model.actuators.len(),
        "n_frames": model.frames.len(),
        "end_effectors": model.end_effectors.iter().map(|e| {
            serde_json::json!({
                "name": e.name,
                "frame": e.frame,
                "chain": e.joint_chain,
            })
        }).collect::<Vec<_>>(),
        "joints": model.joints.iter().map(|j| {
            serde_json::json!({
                "name": j.name,
                "kind": format!("{:?}", j.kind),
                "axis": j.axis.value,
                "origin_in_child": j.origin_in_child.value,
                "parent": j.parent_body,
                "child": j.child_body,
                "q_min": j.q_min.value,
                "q_max": j.q_max.value,
                "qpos_adr": j.qpos_adr,
            })
        }).collect::<Vec<_>>(),
        "bodies": model.bodies.iter().map(|b| {
            serde_json::json!({
                "name": b.name,
                "parent": b.parent,
                "local_pose": b.local_pose.value.map(|p| {
                    serde_json::json!({ "xyz": p.xyz, "quat_wxyz": p.quat_wxyz })
                }),
            })
        }).collect::<Vec<_>>(),
        "frames": model.frames.iter().map(|f| {
            serde_json::json!({
                "name": f.name,
                "parent_body": f.parent_body,
                "translation": f.translation.value,
                "rotation": f.rotation.value,
            })
        }).collect::<Vec<_>>(),
        "actuators": model.actuators.iter().map(|a| {
            serde_json::json!({
                "name": a.name,
                "target_joint": a.target_joint,
                "control_mode": a.control_mode,
            })
        }).collect::<Vec<_>>(),
        "diagnostics": model.diagnostics.iter().map(|d| {
            serde_json::json!({ "code": d.code, "detail": d.detail })
        }).collect::<Vec<_>>(),
    })
}

pub fn write_matrix(report: &ExternalMatrixReport, out_dir: &Path) -> Result<(), String> {
    std::fs::create_dir_all(out_dir).map_err(|e| e.to_string())?;
    let json = serde_json::to_string_pretty(report).map_err(|e| e.to_string())?;
    std::fs::write(out_dir.join("external_ur5e_matrix.json"), json).map_err(|e| e.to_string())?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mujoco_exec::ensure_mujoco_or_skip;

    #[test]
    fn development_external_fk_matches_mujoco() {
        if !ensure_mujoco_or_skip() {
            return;
        }
        if ensure_development_model().is_err() {
            return;
        }
        let bundle = load_development_bundle().expect("development bundle");
        let fk = compare_semantic_fk(&bundle, 100, 11).expect("fk");
        assert!(
            fk.max_position_error <= 1e-5,
            "max_position_error={} mean={} worst_q={:?}",
            fk.max_position_error,
            fk.mean_position_error,
            fk.worst_q
        );
    }

    #[test]
    fn development_external_robot_fk_and_reach_or_honest_refuse() {
        if !ensure_mujoco_or_skip() {
            return;
        }
        if ensure_development_model().is_err() {
            return;
        }
        let report = run_development_matrix().expect("development matrix");
        assert_eq!(report.source_commit, MENAGERIE_SHA);
        assert!(report.fk.max_position_error <= 1e-5);
        assert_eq!(report.reachable.len(), 20);
        assert_eq!(report.unreachable.len(), 5);
        assert_eq!(report.stale.len(), 5);
        assert_eq!(report.wrong_epoch.len(), 5);
        assert!(report.zero_write_invalid);
        assert!(
            report.success_rate >= 0.5 || report.reachable.iter().all(|r| r.skill_refuse.is_some()),
            "expected REACH success or a uniform honest refuse, rate={}",
            report.success_rate
        );
        let out = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../verify-out");
        write_matrix(&report, &out).expect("write ur5e matrix");
    }
}
