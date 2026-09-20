//! Residual ablation for Mode B phase IK on development embodiments.
//! Lives in verify so semantics quarantine does not name those robots.

use crate::bundle::RobotBundle;
use crate::mujoco_exec::checkin_worker;
use crate::runner::load_and_normalize;
use crate::semantics_map::embodiment_from_manifest;
use realityos_semantics::contact_maneuver::{
    select_fixed_world_push_seeded_with_funnel, with_phase_ik_records, BoxObject,
    ContactManeuverSpec, SampledEePose, SupportPlane,
};
use realityos_semantics::embodiment::EmbodimentModel;
use realityos_semantics::kinematics::{forward_kinematics, ik_residual_is_precise, IK_ACCEPT_M};
use serde_json::{json, Value};

const DEVELOPMENT_LABELS: [&str; 4] = ["arm_gripper", "panda", "ur5e", "iiwa14"];
const N_SEEDS: usize = 16;

fn try_load(label: &str) -> Result<RobotBundle, String> {
    match label {
        "arm_gripper" => {
            RobotBundle::load(crate::corpus::robot_dir("arm_gripper")).map_err(|e| e.to_string())
        }
        "panda" => {
            crate::menagerie::ensure_holdout_model()?;
            RobotBundle::load(crate::menagerie::holdout_bundle_dir()).map_err(|e| e.to_string())
        }
        "ur5e" => {
            crate::menagerie::ensure_development_model()?;
            RobotBundle::load(crate::menagerie::development_bundle_dir()).map_err(|e| e.to_string())
        }
        "iiwa14" => {
            crate::menagerie::ensure_v2_holdout_model()?;
            RobotBundle::load(crate::menagerie::v2_holdout_bundle_dir()).map_err(|e| e.to_string())
        }
        other => Err(format!("unknown development label {other}")),
    }
}

fn sample_q(model: &EmbodimentModel, chain: &[String], mut rng: u64) -> Vec<f64> {
    let mut q = Vec::with_capacity(chain.len());
    for name in chain {
        rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1);
        let u = (rng >> 33) as f64 / (1u64 << 31) as f64;
        let joint = model.joints.iter().find(|j| j.name == *name);
        let (lo, hi) = match joint {
            Some(j) => (j.q_min.value.unwrap_or(-1.2), j.q_max.value.unwrap_or(1.2)),
            None => (-1.2, 1.2),
        };
        let lo = lo.max(-2.5);
        let hi = hi.min(2.5);
        q.push(lo + (hi - lo) * u.clamp(0.05, 0.95));
    }
    q
}

fn percentile(sorted: &[f64], p: f64) -> Option<f64> {
    if sorted.is_empty() {
        return None;
    }
    let idx = ((p / 100.0) * (sorted.len() as f64 - 1.0)).round() as usize;
    sorted.get(idx.min(sorted.len() - 1)).copied()
}

fn phase_stats(residuals: &[f64], n_fail: u64, n_precise: u64, n_loose_accepted: u64) -> Value {
    let mut s = residuals.to_vec();
    s.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    json!({
        "n": residuals.len() + n_fail as usize,
        "n_solved": residuals.len(),
        "n_fail": n_fail,
        "n_precise": n_precise,
        "n_loose_best_effort_accepted": n_loose_accepted,
        "p50": percentile(&s, 50.0),
        "p90": percentile(&s, 90.0),
        "p99": percentile(&s, 99.0),
        "max": s.last().copied(),
        "precise_accept_m": IK_ACCEPT_M,
    })
}

fn ablate_model(label: &str, bundle: &RobotBundle) -> Result<Value, String> {
    let (inst, manifest) = load_and_normalize(bundle, &[], 0)?;
    let model = embodiment_from_manifest(bundle, &manifest);
    checkin_worker(inst);
    let ee = bundle
        .manifest
        .end_effectors
        .first()
        .map(|e| e.name.as_str())
        .ok_or_else(|| "no end effector".to_string())?;
    let chain = model
        .ee_joint_chain(ee)
        .ok_or_else(|| "empty chain".to_string())?;
    let mut by_phase: std::collections::BTreeMap<String, Vec<f64>> =
        std::collections::BTreeMap::new();
    let mut fail_by_phase: std::collections::BTreeMap<String, u64> =
        std::collections::BTreeMap::new();
    let mut precise_by_phase: std::collections::BTreeMap<String, u64> =
        std::collections::BTreeMap::new();
    let mut loose_accepted: u64 = 0;
    let mut n_selected: u64 = 0;
    let mut n_selected_loose: u64 = 0;
    let mut n_complete_chains: u64 = 0;
    for i in 0..N_SEEDS {
        let q = sample_q(&model, &chain, 17 + i as u64 * 997);
        let Ok(fk) = forward_kinematics(&model, &chain, ee, &q) else {
            continue;
        };
        if !fk.ee.xyz.iter().all(|v| v.is_finite()) {
            continue;
        }
        let seed = SampledEePose {
            xyz: fk.ee.xyz,
            quat_wxyz: fk.ee.quat_wxyz,
            q: q.clone(),
            joint_names: chain.clone(),
        };
        let mut push = [fk.ee.xyz[0], fk.ee.xyz[1], 0.0];
        let pn = (push[0] * push[0] + push[1] * push[1]).sqrt();
        if pn < 1e-3 {
            push = [1.0, 0.0, 0.0];
        } else {
            push = [push[0] / pn, push[1] / pn, 0.0];
        }
        let half = [0.03, 0.03, 0.03];
        let face =
            half[0] * push[0].abs() + half[1] * push[1].abs() + half[2] * push[2].abs() + 0.015;
        let object = BoxObject {
            center: [
                fk.ee.xyz[0] + push[0] * face,
                fk.ee.xyz[1] + push[1] * face,
                fk.ee.xyz[2],
            ],
            half_extents: half,
        };
        let support = SupportPlane {
            origin: [
                object.center[0],
                object.center[1],
                object.center[2] - half[2],
            ],
            normal: [0.0, 0.0, 1.0],
        };
        let mut spec = ContactManeuverSpec::table_push(push, 0.02, seed.xyz, [0.0, 0.0, 0.0]);
        spec.min_stroke = 0.005;
        spec.requested_stroke = 0.01;
        let q2 = sample_q(&model, &chain, 91 + i as u64 * 449);
        let mut cloud = vec![seed.clone()];
        if let Ok(fk2) = forward_kinematics(&model, &chain, ee, &q2) {
            cloud.push(SampledEePose {
                xyz: fk2.ee.xyz,
                quat_wxyz: fk2.ee.quat_wxyz,
                q: q2,
                joint_names: chain.clone(),
            });
        }
        let ((res, funnel), recs) = with_phase_ik_records(|| {
            select_fixed_world_push_seeded_with_funnel(&model, ee, &cloud, object, support, &spec)
        });
        n_complete_chains += funnel.n_complete_ik_chains;
        let mut this_loose = false;
        for r in recs {
            if r.residual_m.is_finite() {
                by_phase
                    .entry(r.phase.clone())
                    .or_default()
                    .push(r.residual_m);
                if r.precise {
                    *precise_by_phase.entry(r.phase.clone()).or_default() += 1;
                }
            } else {
                *fail_by_phase.entry(r.phase.clone()).or_default() += 1;
            }
            if r.accepted && !r.precise {
                loose_accepted += 1;
                this_loose = true;
            }
        }
        if let Ok((m, _)) = res {
            n_selected += 1;
            let mut depend = this_loose;
            if let Some(w) = m.executable {
                if !ik_residual_is_precise(w.contact.residual) {
                    depend = true;
                }
            }
            if depend {
                n_selected_loose += 1;
            }
        }
    }
    let mut phases = serde_json::Map::new();
    for phase in ["approach", "contact", "mid", "end"] {
        let residuals = by_phase.remove(phase).unwrap_or_default();
        let n_fail = *fail_by_phase.get(phase).unwrap_or(&0);
        let n_precise = *precise_by_phase.get(phase).unwrap_or(&0);
        phases.insert(
            phase.to_string(),
            phase_stats(&residuals, n_fail, n_precise, 0),
        );
    }
    Ok(json!({
        "label": label,
        "robot_id": model.robot_id,
        "n_seeds": N_SEEDS,
        "n_complete_ik_chains": n_complete_chains,
        "n_selected_executable": n_selected,
        "n_selected_depending_on_loose_ik": n_selected_loose,
        "n_loose_best_effort_accepted": loose_accepted,
        "precise_accept_m": IK_ACCEPT_M,
        "by_phase": phases,
    }))
}

pub fn run_ik_residual_ablation() -> Value {
    let mut robots = serde_json::Map::new();
    let mut errors = serde_json::Map::new();
    for label in DEVELOPMENT_LABELS {
        match try_load(label) {
            Ok(bundle) => match ablate_model(label, &bundle) {
                Ok(v) => {
                    robots.insert(label.to_string(), v);
                }
                Err(e) => {
                    errors.insert(label.to_string(), json!(e));
                }
            },
            Err(e) => {
                errors.insert(label.to_string(), json!(e));
            }
        }
    }
    json!({
        "precise_accept_m": IK_ACCEPT_M,
        "note": "Collapse under precise IK is evidence. This table does not retune held-out robots.",
        "robots": robots,
        "load_errors": errors,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mujoco_exec::ensure_mujoco_or_skip;

    #[test]
    fn four_embodiment_ik_residual_ablation() {
        if !ensure_mujoco_or_skip() {
            panic!("MuJoCo required for residual ablation; skip is not this step");
        }
        let report = run_ik_residual_ablation();
        let robots = report["robots"].as_object().expect("robots object");
        for label in DEVELOPMENT_LABELS {
            assert!(
                robots.contains_key(label),
                "missing embodiment {label} errors={:?}",
                report["load_errors"]
            );
            let row = &robots[label];
            assert!(row["by_phase"]["approach"].is_object());
            assert!(row["by_phase"]["contact"].is_object());
            assert!(row["by_phase"]["mid"].is_object());
            assert!(row["by_phase"]["end"].is_object());
            assert_eq!(row["precise_accept_m"], json!(IK_ACCEPT_M));
        }
        eprintln!("{}", serde_json::to_string_pretty(&report).unwrap());
    }

    #[test]
    fn mode_b_select_on_arm_gripper_is_precise_or_distinct_refusal() {
        if !ensure_mujoco_or_skip() {
            panic!("MuJoCo required for consumer select; skip is not this step");
        }
        let bundle = try_load("arm_gripper").expect("arm_gripper bundle");
        let (inst, manifest) = load_and_normalize(&bundle, &[], 0).expect("load");
        let model = embodiment_from_manifest(&bundle, &manifest);
        checkin_worker(inst);
        let ee = bundle.manifest.end_effectors[0].name.as_str();
        let chain = model.ee_joint_chain(ee).expect("chain");
        let q = sample_q(&model, &chain, 21);
        let fk = forward_kinematics(&model, &chain, ee, &q).expect("fk");
        let seed = SampledEePose {
            xyz: fk.ee.xyz,
            quat_wxyz: fk.ee.quat_wxyz,
            q,
            joint_names: chain.clone(),
        };
        let push = [1.0, 0.0, 0.0];
        let half = [0.03, 0.03, 0.03];
        let object = BoxObject {
            center: [fk.ee.xyz[0] + 0.045, fk.ee.xyz[1], fk.ee.xyz[2]],
            half_extents: half,
        };
        let support = SupportPlane {
            origin: [object.center[0], object.center[1], object.center[2] - 0.03],
            normal: [0.0, 0.0, 1.0],
        };
        let mut spec = ContactManeuverSpec::table_push(push, 0.02, seed.xyz, [0.0, 0.0, 0.0]);
        spec.min_stroke = 0.005;
        let (res, funnel) =
            select_fixed_world_push_seeded_with_funnel(&model, ee, &[seed], object, support, &spec);
        assert_eq!(funnel.n_ik_solutions, funnel.n_complete_ik_chains);
        match res {
            Ok((m, _)) => {
                let w = m.executable.expect("selected maneuver stores a witness");
                assert!(
                    realityos_semantics::maneuver_witness::execution_block_reason(&w).is_none()
                );
                assert!(ik_residual_is_precise(w.contact.residual));
                assert!(w.contact.orientation_checked);
                assert!(w.contact.full_pose_feasible);
            }
            Err(e) => {
                let s = e.as_str();
                assert!(
                    s == "NO_IK_SOLUTION"
                        || s == "NO_EXECUTABLE_CONTACT_MANEUVER"
                        || s == "INSUFFICIENT_JOINT_MARGIN"
                        || s == "COLLISION_INADMISSIBLE"
                        || s == "WRONG_CONTACT_GEOMETRY"
                        || s == "ORIENTATION_INFEASIBLE"
                        || s == "NO_FEASIBLE_CONTACT_POSE"
                        || s == "APPROACH_COLLIDES_BEFORE_CONTACT"
                        || s == "SUPPORT_PLANE_BLOCKS_EE"
                        || s == "CONTACT_POSE_UNREACHABLE_FROM_APPROACH"
                        || s == "INSUFFICIENT_REMAINING_STROKE",
                    "distinct infeasibility label, got {s} funnel={funnel:?}"
                );
            }
        }
    }
}
