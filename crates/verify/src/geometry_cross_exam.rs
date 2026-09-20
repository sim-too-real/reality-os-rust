//! Post-hoc MuJoCo comparison of planner-visible declared geometry.
//! Simulator contacts never enter planning decisions.

use crate::mujoco_exec::ensure_mujoco_or_skip;
use realityos_semantics::allowed_contact::{
    adjacent_body_pairs, AllowedContactPolicy, ContactPhase, PairPermission,
};
use realityos_semantics::contact::classify_contact_pair;
use realityos_semantics::contact::{ContactClassContext, ContactEvidenceClass};
use realityos_semantics::geometry::CollisionRole;
use serde_json::{json, Value};

#[derive(Debug, Default)]
struct Counts {
    tp: u64,
    tn: u64,
    fp: u64,
    fn_: u64,
}

impl Counts {
    fn add(&mut self, planner: bool, mujoco: bool) {
        match (planner, mujoco) {
            (true, true) => self.tp += 1,
            (false, false) => self.tn += 1,
            (true, false) => self.fp += 1,
            (false, true) => self.fn_ += 1,
        }
    }
}

fn class_bucket(c: ContactEvidenceClass) -> &'static str {
    match c {
        ContactEvidenceClass::SelfCollision => "self",
        ContactEvidenceClass::SupportContact => "support",
        ContactEvidenceClass::IntendedToolContact => "tool_object",
        ContactEvidenceClass::UnintendedRobotContact => "unintended",
        ContactEvidenceClass::ObstacleContact => "obstacle",
    }
}

/// Compare planner-visible declared geometry against independent simulator geoms.
pub fn run_geometry_cross_exam() -> Result<Value, String> {
    if !ensure_mujoco_or_skip() {
        return Err("MUJOCO_UNAVAILABLE".into());
    }
    let bundle = crate::bundle::RobotBundle::load(crate::corpus::robot_dir("arm_gripper"))
        .map_err(|e| e.to_string())?;
    let (mut inst, manifest) =
        crate::runner::load_and_normalize(&bundle, &[], 0).map_err(|e| e.to_string())?;
    let model = crate::semantics_map::embodiment_from_manifest(&bundle, &manifest);
    let inspect = inst.inspect.clone();
    let nq = inspect["nq"].as_u64().unwrap_or(0) as usize;
    let mut buckets: std::collections::BTreeMap<&str, Counts> = std::collections::BTreeMap::new();
    for k in ["self", "support", "tool_object", "unintended", "obstacle"] {
        buckets.insert(k, Counts::default());
    }
    let robot_bodies: Vec<String> = model.bodies.iter().map(|b| b.name.clone()).collect();
    let intended = realityos_semantics::contact::declared_manipulation_contact_bodies(
        &model,
        model.resources.first(),
        model
            .end_effectors
            .first()
            .map(|e| e.name.as_str())
            .unwrap_or("ee"),
    );
    let support: Vec<String> = manifest.support_bodies.clone();
    let adjacent = adjacent_body_pairs(&model);
    let policy = AllowedContactPolicy::from_names(
        "object",
        intended.clone(),
        robot_bodies.clone(),
        support.clone(),
        Vec::new(),
        adjacent,
    );
    let mut false_negatives = Vec::new();
    for seed in 0u64..24 {
        let mut q = vec![0.0; nq.max(1)];
        for (i, slot) in q.iter_mut().enumerate() {
            let u = ((seed.wrapping_mul(17).wrapping_add(i as u64 * 13)) % 1000) as f64 / 1000.0;
            *slot = (u - 0.5) * 1.2;
        }
        let _ = inst.reset(Some(&q), Some(&vec![0.0; nq.max(1)]));
        let st = inst.state().map_err(|e| e.to_string())?;
        let contacts = st["contacts"].as_array().cloned().unwrap_or_default();
        let mut mj_pairs: std::collections::BTreeSet<(String, String, &'static str)> =
            std::collections::BTreeSet::new();
        for c in &contacts {
            let a = c["body1"].as_str().unwrap_or("").to_string();
            let b = c["body2"].as_str().unwrap_or("").to_string();
            let Some(class) = classify_contact_pair(
                &a,
                &b,
                &ContactClassContext {
                    object_id: "object",
                    intended: &intended,
                    robot_bodies: &robot_bodies,
                    support_bodies: &support,
                    obstacle_bodies: &[],
                },
            ) else {
                continue;
            };
            if policy.permit(&a, &b, ContactPhase::FreeSpace, 0, 1) == PairPermission::Allowed {
                continue;
            }
            let (x, y) = if a <= b { (a, b) } else { (b, a) };
            mj_pairs.insert((x, y, class_bucket(class)));
        }
        let mut planner_pairs: std::collections::BTreeSet<(String, String, &'static str)> =
            std::collections::BTreeSet::new();
        let named: Vec<(String, f64)> = model
            .joints
            .iter()
            .filter_map(|j| {
                let adr = j.qpos_adr? as usize;
                q.get(adr).copied().map(|v| (j.name.clone(), v))
            })
            .collect();
        if let Ok(bodies) = realityos_semantics::geometry_fk::body_world_transforms(&model, &named)
        {
            let geoms: Vec<_> = model
                .collision_geoms
                .iter()
                .filter(|g| g.collision_role == CollisionRole::Collision)
                .collect();
            for i in 0..geoms.len() {
                for j in (i + 1)..geoms.len() {
                    let ga = geoms[i];
                    let gb = geoms[j];
                    if ga.owner_body == gb.owner_body {
                        continue;
                    }
                    let Some(wa) = bodies.get(&ga.owner_body).copied() else {
                        continue;
                    };
                    let Some(wb) = bodies.get(&gb.owner_body).copied() else {
                        continue;
                    };
                    let qres = realityos_semantics::geometry_query::query_geoms(wa, ga, wb, gb);
                    if !qres.intersects() {
                        continue;
                    }
                    if policy.permit(
                        &ga.owner_body,
                        &gb.owner_body,
                        ContactPhase::FreeSpace,
                        0,
                        1,
                    ) == PairPermission::Allowed
                    {
                        continue;
                    }
                    let Some(class) = classify_contact_pair(
                        &ga.owner_body,
                        &gb.owner_body,
                        &ContactClassContext {
                            object_id: "object",
                            intended: &intended,
                            robot_bodies: &robot_bodies,
                            support_bodies: &support,
                            obstacle_bodies: &[],
                        },
                    ) else {
                        continue;
                    };
                    let (x, y) = if ga.owner_body <= gb.owner_body {
                        (ga.owner_body.clone(), gb.owner_body.clone())
                    } else {
                        (gb.owner_body.clone(), ga.owner_body.clone())
                    };
                    planner_pairs.insert((x, y, class_bucket(class)));
                }
            }
        }
        let keys: std::collections::BTreeSet<&'static str> =
            ["self", "support", "tool_object", "unintended", "obstacle"]
                .into_iter()
                .collect();
        for bucket in keys {
            let p: std::collections::BTreeSet<_> = planner_pairs
                .iter()
                .filter(|(_, _, b)| *b == bucket)
                .cloned()
                .collect();
            let m: std::collections::BTreeSet<_> = mj_pairs
                .iter()
                .filter(|(_, _, b)| *b == bucket)
                .cloned()
                .collect();
            for pair in p.union(&m) {
                let planner = p.contains(pair);
                let mujoco = m.contains(pair);
                buckets
                    .get_mut(bucket)
                    .expect("bucket")
                    .add(planner, mujoco);
                if mujoco && !planner {
                    false_negatives.push(json!({
                        "seed": seed,
                        "bucket": bucket,
                        "body_a": pair.0,
                        "body_b": pair.1,
                    }));
                }
            }
        }
    }
    crate::mujoco_exec::checkin_worker(inst);
    let mut out_buckets = serde_json::Map::new();
    for (k, c) in &buckets {
        out_buckets.insert(
            (*k).into(),
            json!({
                "true_positive": c.tp,
                "true_negative": c.tn,
                "false_positive": c.fp,
                "false_negative": c.fn_,
            }),
        );
    }
    Ok(json!({
        "schema": "realityos.geometry_cross_exam/1",
        "planner_visible_only": true,
        "privileged_mujoco": true,
        "buckets": out_buckets,
        "false_negatives": false_negatives,
        "n_false_negatives": false_negatives.len(),
        "embodiment": "arm_gripper",
        "note": "False negatives become deterministic regressions. This matrix is verifier-only.",
    }))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn geometry_cross_exam_against_mujoco_or_skip() {
        if !ensure_mujoco_or_skip() {
            return;
        }
        let report = run_geometry_cross_exam().expect("cross exam");
        assert_eq!(report["schema"], "realityos.geometry_cross_exam/1");
        assert_eq!(report["planner_visible_only"], true);
        let pretty = serde_json::to_string_pretty(&report).unwrap();
        eprintln!("{pretty}");
        if let Ok(p) = std::env::var("REALITYOS_GEOMETRY_CROSS_EXAM_OUT") {
            std::fs::write(&p, pretty).expect("write cross-exam json");
        }
    }
}
