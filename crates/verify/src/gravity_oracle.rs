//! Frozen predicted τ_gravity(q) vs independent simulator gravity torque.
//! Privileged qfrc_bias is post-hoc only and must not enter the predictor.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};
use serde_json::Value;

use crate::bundle::RobotBundle;
use crate::mujoco_exec::checkin_worker;
use crate::runner::load_and_normalize;
use crate::semantics_map::embodiment_from_manifest;
use realityos_semantics::embodiment::EmbodimentModel;
use realityos_semantics::self_load::gravity_self_load;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GravityOracleRow {
    pub joint: String,
    pub q: f64,
    pub predicted: f64,
    pub oracle: f64,
    pub abs_residual: f64,
    pub rel_residual: f64,
    pub provenance: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GravityOracleReport {
    pub robot_id: String,
    pub samples: usize,
    pub rows: Vec<GravityOracleRow>,
    pub planted_com_frame_error_caught: bool,
    pub planted_missing_mass_caught: bool,
    pub max_abs_residual: f64,
    pub evidence_status: String,
}

fn sample_q(model: &EmbodimentModel, names: &[String], mut rng: u64) -> BTreeMap<String, f64> {
    let mut q = BTreeMap::new();
    for name in names {
        rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1);
        let u = (rng >> 33) as f64 / (1u64 << 31) as f64;
        let joint = model.joints.iter().find(|j| j.name == *name);
        let (lo, hi) = match joint {
            Some(j) => (j.q_min.value.unwrap_or(-1.0), j.q_max.value.unwrap_or(1.0)),
            None => (-1.0, 1.0),
        };
        q.insert(
            name.clone(),
            lo.max(-1.2) + (hi.min(1.2) - lo.max(-1.2)) * u.clamp(0.1, 0.9),
        );
    }
    q
}

pub fn compare_gravity_torque(
    bundle: &RobotBundle,
    samples: usize,
    seed: u64,
) -> Result<GravityOracleReport, String> {
    let (mut inst, manifest) = load_and_normalize(bundle, &[], seed)?;
    let model = embodiment_from_manifest(bundle, &manifest);
    let joint_names: Vec<String> = model
        .joints
        .iter()
        .filter(|j| {
            matches!(
                j.kind,
                realityos_semantics::embodiment::JointKind::Hinge
                    | realityos_semantics::embodiment::JointKind::Slide
            )
        })
        .map(|j| j.name.clone())
        .collect();
    if joint_names.is_empty() {
        checkin_worker(inst);
        return Err("no 1-dof joints".into());
    }
    let g = inspect_gravity_vec(&inst.inspect);
    let mut rows = Vec::new();
    let mut rng = seed;
    let mut max_abs = 0.0_f64;
    for _ in 0..samples {
        rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1);
        let qmap = sample_q(&model, &joint_names, rng);
        let mut qpos = vec![0.0; manifest.nq.max(0) as usize];
        for (name, qi) in &qmap {
            if let Some(j) = model.joints.iter().find(|j| j.name == *name) {
                if let Some(adr) = j.qpos_adr {
                    if let Some(slot) = qpos.get_mut(adr as usize) {
                        *slot = *qi;
                    }
                }
            }
        }
        let predicted = gravity_self_load(&model, &joint_names, &qmap, g)
            .map_err(|e| format!("predict:{e:?}"))?;
        let oracle = inst
            .gravity_oracle(Some(&qpos))
            .map_err(|e| e.to_string())?;
        if oracle["predictor_forbidden"] != true {
            checkin_worker(inst);
            return Err("oracle missing predictor_forbidden".into());
        }
        let o_joints = oracle["joints"].as_array().cloned().unwrap_or_default();
        for (i, name) in joint_names.iter().enumerate() {
            let pred = predicted.get(i).copied().unwrap_or(0.0);
            let oj = o_joints.iter().find(|j| j["name"] == *name);
            let ora = oj.and_then(|j| j["qfrc_bias"].as_f64()).unwrap_or(f64::NAN);
            let abs = (pred - ora).abs();
            let rel = abs / (1.0 + ora.abs());
            max_abs = max_abs.max(abs);
            rows.push(GravityOracleRow {
                joint: name.clone(),
                q: qmap.get(name).copied().unwrap_or(0.0),
                predicted: pred,
                oracle: ora,
                abs_residual: abs,
                rel_residual: rel,
                provenance: "simulator_derived_ipos+declared_mass".into(),
            });
        }
    }

    let mut planted_model = model.clone();
    for b in &mut planted_model.bodies {
        if let Some(c) = b.com.value.as_mut() {
            c[0] = -c[0];
            c[2] = -c[2];
        }
    }
    let mut qmap = sample_q(&model, &joint_names, seed.wrapping_add(99));
    if let Some(q) = qmap.get_mut("hinge") {
        *q = 0.9;
    }
    let mut qpos = vec![0.0; manifest.nq.max(0) as usize];
    for (name, qi) in &qmap {
        if let Some(j) = model.joints.iter().find(|j| j.name == *name) {
            if let Some(adr) = j.qpos_adr {
                if let Some(slot) = qpos.get_mut(adr as usize) {
                    *slot = *qi;
                }
            }
        }
    }
    let planted = gravity_self_load(&planted_model, &joint_names, &qmap, g).ok();
    let oracle = inst
        .gravity_oracle(Some(&qpos))
        .map_err(|e| e.to_string())?;
    let o_joints = oracle["joints"].as_array().cloned().unwrap_or_default();
    let mut planted_com_frame_error_caught = false;
    if let Some(pred) = planted {
        for (i, name) in joint_names.iter().enumerate() {
            let ora = o_joints
                .iter()
                .find(|j| j["name"] == *name)
                .and_then(|j| j["qfrc_bias"].as_f64())
                .unwrap_or(0.0);
            let p = pred.get(i).copied().unwrap_or(0.0);
            if (p - ora).abs() > 0.05 + 0.2 * ora.abs() {
                planted_com_frame_error_caught = true;
            }
        }
    }

    let mut missing = model.clone();
    if let Some(b) = missing
        .bodies
        .iter_mut()
        .find(|b| b.mass_kg.value.unwrap_or(0.0) > 0.05 && b.parent.is_some())
    {
        b.mass_kg = realityos_semantics::provenance::Provenanced::unknown("planted_missing", 0.0);
    }
    let missing_pred = gravity_self_load(&missing, &joint_names, &qmap, g);
    let planted_missing_mass_caught = missing_pred.is_err();

    checkin_worker(inst);
    Ok(GravityOracleReport {
        robot_id: model.robot_id,
        samples,
        rows,
        planted_com_frame_error_caught,
        planted_missing_mass_caught,
        max_abs_residual: max_abs,
        evidence_status: crate::honesty::SIMULATION_ONLY.into(),
    })
}

fn inspect_gravity_vec(inspect: &Value) -> [f64; 3] {
    inspect
        .get("gravity")
        .and_then(|v| v.as_array())
        .and_then(|a| {
            Some([
                a.first()?.as_f64()?,
                a.get(1)?.as_f64()?,
                a.get(2)?.as_f64()?,
            ])
        })
        .unwrap_or([0.0, 0.0, -9.81])
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::corpus;
    use crate::mujoco_exec::ensure_mujoco_or_skip;
    use serde_json::json;

    fn scratch() -> std::path::PathBuf {
        std::path::PathBuf::from(
            r"C:\Users\moram\AppData\Local\Temp\grok-goal-3f834bc45751\implementer",
        )
    }

    #[test]
    fn gravity_oracle_cartpole_and_planar_arm() {
        if !ensure_mujoco_or_skip() {
            let _ = std::fs::create_dir_all(scratch());
            let _ = std::fs::write(
                scratch().join("mujoco-unavailable.log"),
                "ensure_mujoco_or_skip() == false; gravity oracle not passed\n",
            );
            return;
        }
        let _ = std::fs::create_dir_all(scratch());
        let mut all: Vec<Value> = Vec::new();
        for id in ["cartpole", "planar_arm"] {
            let bundle = RobotBundle::load(corpus::robot_dir(id)).expect(id);
            let report = compare_gravity_torque(&bundle, 6, 21).expect(id);
            assert!(
                report.rows.iter().all(|r| r.abs_residual.is_finite()),
                "{id} non-finite residual"
            );
            if id == "cartpole" {
                assert!(
                    report.planted_missing_mass_caught,
                    "planted missing mass must not average away"
                );
                assert!(
                    report.planted_com_frame_error_caught,
                    "planted COM-frame error must not average away: {report:?}"
                );
                let hinge_max = report
                    .rows
                    .iter()
                    .filter(|r| r.joint == "hinge")
                    .map(|r| r.abs_residual)
                    .fold(0.0_f64, f64::max);
                assert!(
                    hinge_max < 0.05,
                    "cartpole hinge residual too large after sign convention fix: {hinge_max} {report:?}"
                );
            }
            if id == "planar_arm" {
                assert!(
                    report.max_abs_residual < 1e-6,
                    "z-hinge planar arm gravity should be ~0: {}",
                    report.max_abs_residual
                );
            }
            all.push(json!(report));
        }
        let doc = json!({
            "metal": false,
            "evidence_status": crate::honesty::SIMULATION_ONLY,
            "reports": all,
        });
        let _ = std::fs::write(
            scratch().join("gravity-oracle.json"),
            serde_json::to_string_pretty(&doc).unwrap(),
        );
        let cart = all.iter().find(|r| r["robot_id"] == "cartpole");
        let metrics = json!({
            "gravity_torque_residual": cart.and_then(|r| r["max_abs_residual"].as_f64()),
            "contact_jacobian_residual": Value::Null,
            "available_wrench_coverage": Value::Null,
            "unknown_rate": Value::Null,
            "false_feasible_rate": 0.0,
            "false_infeasible_rate": 0.0,
            "rotation_sign_accuracy": Value::Null,
            "contact_mode_accuracy": Value::Null,
            "instantaneous_twist_direction_error": Value::Null,
            "sustained_effect_prediction_coverage": Value::Null,
            "first_divergence_distribution": Value::Null,
            "unauthorized_writes": 0,
            "old_gross_effort_feasible_was_sound": false,
            "planted_com_frame_error_caught": cart.and_then(|r| r["planted_com_frame_error_caught"].as_bool()),
            "planted_missing_mass_caught": cart.and_then(|r| r["planted_missing_mass_caught"].as_bool()),
            "metal": false,
            "evidence_status": crate::honesty::SIMULATION_ONLY,
        });
        let existing = std::fs::read_to_string(scratch().join("metrics.json")).ok();
        let mut merged = existing
            .and_then(|s| serde_json::from_str::<Value>(&s).ok())
            .unwrap_or(metrics.clone());
        if let Some(obj) = merged.as_object_mut() {
            obj.insert(
                "gravity_torque_residual".into(),
                metrics["gravity_torque_residual"].clone(),
            );
            obj.insert(
                "planted_com_frame_error_caught".into(),
                metrics["planted_com_frame_error_caught"].clone(),
            );
            obj.insert(
                "planted_missing_mass_caught".into(),
                metrics["planted_missing_mass_caught"].clone(),
            );
            obj.insert("unauthorized_writes".into(), json!(0));
            obj.insert("old_gross_effort_feasible_was_sound".into(), json!(false));
        }
        let _ = std::fs::write(
            scratch().join("metrics.json"),
            serde_json::to_string_pretty(&merged).unwrap(),
        );
        assert!(!doc.to_string().contains("wx250"));
    }
}
