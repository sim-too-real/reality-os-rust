//! Semantic FK vs MuJoCo privileged FK. MuJoCo is the oracle.

use crate::bundle::RobotBundle;
use crate::mujoco_exec::checkin_worker;
use crate::normalize::RobotManifest;
use crate::observation::VerifierTruth;
use crate::runner::load_and_normalize;
use crate::semantics_map::embodiment_from_manifest;
use realityos_semantics::embodiment::EmbodimentModel;
use realityos_semantics::kinematics::{forward_kinematics, orientation_error};
use realityos_semantics::transform::norm3;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FkOracleReport {
    pub robot_id: String,
    pub samples: usize,
    pub mean_position_error: f64,
    pub max_position_error: f64,
    pub mean_orientation_error: f64,
    pub max_orientation_error: f64,
    pub worst_q: Vec<f64>,
    pub worst_position_error: f64,
}

pub fn compare_semantic_fk(
    bundle: &RobotBundle,
    samples: usize,
    seed: u64,
) -> Result<FkOracleReport, String> {
    let (mut inst, manifest) = load_and_normalize(bundle, &[], seed)?;
    let model = embodiment_from_manifest(bundle, &manifest);
    let ee = bundle
        .manifest
        .end_effectors
        .first()
        .map(|e| e.name.clone())
        .ok_or_else(|| "no end effector".to_string())?;
    let chain = model
        .ee_joint_chain(&ee)
        .ok_or_else(|| "empty chain".to_string())?;
    let mut rng = seed;
    let mut sum_p = 0.0;
    let mut max_p = 0.0;
    let mut sum_o = 0.0;
    let mut max_o = 0.0;
    let mut worst_q = Vec::new();
    let mut worst_p = 0.0;
    let mut n = 0usize;
    for _ in 0..samples {
        rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1);
        let q = sample_q(&model, &chain, &manifest, rng);
        let mut qpos = vec![0.0; manifest.nq.max(0) as usize];
        for (name, qi) in chain.iter().zip(q.iter()) {
            if let Some(j) = model.joints.iter().find(|j| j.name == *name) {
                if let Some(adr) = j.qpos_adr {
                    if let Some(slot) = qpos.get_mut(adr as usize) {
                        *slot = *qi;
                    }
                }
            }
        }
        let st = inst
            .reset(Some(&qpos), Some(&vec![0.0; manifest.nv.max(0) as usize]))
            .map_err(|e| e.to_string())?;
        let truth = VerifierTruth::from_mujoco_state(st.get("state").unwrap_or(&st));
        let fk = forward_kinematics(&model, &chain, &ee, &q).map_err(|e| format!("{e:?}"))?;
        let site = bundle
            .manifest
            .end_effectors
            .first()
            .and_then(|e| e.site.clone())
            .unwrap_or_else(|| ee.clone());
        let oracle = truth
            .named_pos
            .get(&site)
            .cloned()
            .or_else(|| truth.xpos.get(&site).cloned())
            .ok_or_else(|| format!("missing privileged site {site}"))?;
        if oracle.len() < 3 {
            continue;
        }
        let oracle_xyz = [oracle[0], oracle[1], oracle[2]];
        let perr = norm3([
            fk.ee.xyz[0] - oracle_xyz[0],
            fk.ee.xyz[1] - oracle_xyz[1],
            fk.ee.xyz[2] - oracle_xyz[2],
        ]);
        let oerr = if let Some(q) = truth.site_xquat.get(&site) {
            if q.len() >= 4 {
                if let Ok(oracle_se3) = realityos_semantics::transform::Se3::try_new(
                    oracle_xyz,
                    [q[0], q[1], q[2], q[3]],
                ) {
                    orientation_error(fk.ee, oracle_se3)
                } else {
                    0.0
                }
            } else {
                0.0
            }
        } else {
            0.0
        };
        sum_p += perr;
        sum_o += oerr;
        if perr > max_p {
            max_p = perr;
        }
        if oerr > max_o {
            max_o = oerr;
        }
        if perr > worst_p {
            worst_p = perr;
            worst_q = q;
        }
        n += 1;
    }
    checkin_worker(inst);
    if n == 0 {
        return Err("no fk samples".into());
    }
    Ok(FkOracleReport {
        robot_id: model.robot_id,
        samples: n,
        mean_position_error: sum_p / n as f64,
        max_position_error: max_p,
        mean_orientation_error: sum_o / n as f64,
        max_orientation_error: max_o,
        worst_q,
        worst_position_error: worst_p,
    })
}

fn sample_q(
    model: &EmbodimentModel,
    chain: &[String],
    _manifest: &RobotManifest,
    mut rng: u64,
) -> Vec<f64> {
    let mut q = Vec::with_capacity(chain.len());
    for name in chain {
        rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1);
        let u = (rng >> 33) as f64 / (1u64 << 31) as f64;
        let joint = model.joints.iter().find(|j| j.name == *name);
        let (lo, hi) = match joint {
            Some(j) => (j.q_min.value.unwrap_or(-1.0), j.q_max.value.unwrap_or(1.0)),
            None => (-1.0, 1.0),
        };
        q.push(lo + (hi - lo) * u.clamp(0.0, 1.0));
    }
    q
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mujoco_exec::ensure_mujoco_or_skip;

    #[test]
    fn planar_and_spatial_fk_match_mujoco() {
        if !ensure_mujoco_or_skip() {
            return;
        }
        for id in ["planar_arm", "spatial_arm4"] {
            let b = RobotBundle::load(crate::corpus::bundled_robots_root().join(id)).unwrap();
            let r = compare_semantic_fk(&b, 100, 7).unwrap();
            assert!(
                r.max_position_error <= 1e-5,
                "{id} max_position_error={}",
                r.max_position_error
            );
        }
    }

    #[test]
    fn privileged_transform_graph_matches_ee() {
        if !ensure_mujoco_or_skip() {
            return;
        }
        let b = RobotBundle::load(crate::corpus::bundled_robots_root().join("planar_arm")).unwrap();
        let (mut inst, manifest) = load_and_normalize(&b, &[], 3).unwrap();
        let model = embodiment_from_manifest(&b, &manifest);
        let st = inst.step(0).unwrap();
        let truth = VerifierTruth::from_mujoco_state(st.get("state").unwrap_or(&st));
        crate::mujoco_exec::checkin_worker(inst);
        let mut g = realityos_semantics::transform::TransformGraph::new(&model.calibration_epoch);
        for (name, pos) in &truth.xpos {
            if pos.len() < 3 {
                continue;
            }
            let Some(q) = truth.xquat.get(name).filter(|q| q.len() >= 4) else {
                continue;
            };
            if let Ok(pose) = realityos_semantics::transform::Se3::try_new(
                [pos[0], pos[1], pos[2]],
                [q[0], q[1], q[2], q[3]],
            ) {
                let _ = g.insert(realityos_semantics::transform::TransformEdge::from_se3(
                    "world",
                    name,
                    pose,
                    &model.calibration_epoch,
                    1.0,
                    "privileged",
                ));
            }
        }
        let world_tool = g.resolve("world", "link3", 1.0, None).unwrap();
        let tool_world = g.resolve("link3", "world", 1.0, None).unwrap();
        let back = world_tool.compose(tool_world);
        assert!(realityos_semantics::transform::norm3(back.xyz) < 1e-9);
        let site = truth.named_pos.get("ee").expect("ee site");
        let body = truth.xpos.get("link3").expect("link3");
        let dx = site[0] - body[0];
        let dy = site[1] - body[1];
        let dz = site[2] - body[2];
        assert!((dx * dx + dy * dy + dz * dz).sqrt() < 0.2);
    }
}
