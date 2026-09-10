//! Practical controllability. Not global nonlinear controllability.

use crate::bundle::BaseType;
use crate::mujoco_exec::MujocoInstance;
use crate::normalize::RobotManifest;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum ControlClass {
    FullyActuated,
    Underactuated,
    PartiallyActuated,
    ControlMappingInvalid,
    ControllerUnstable,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ActuatorProbe {
    pub name: String,
    pub joint: String,
    pub pos_response: f64,
    pub neg_response: f64,
    pub sign_ok: Option<bool>,
    pub finite: bool,
    pub within_limits: bool,
    pub tracking_error: Option<f64>,
    pub settling_time_s: Option<f64>,
    pub metric_status: String,
    pub divergent: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ControlQualification {
    pub robot_id: String,
    pub model_hash: String,
    pub class: ControlClass,
    pub local_linear_controllability: Option<LocalLinearReport>,
    pub probes: Vec<ActuatorProbe>,
    pub rest_recovery_ok: bool,
    pub perturbation_recovery_ok: bool,
    pub workspace_samples: Vec<[f64; 3]>,
    pub controlled_dofs: Vec<String>,
    pub passive_dofs: Vec<String>,
    pub dynamically_coupled_dofs: Vec<String>,
    pub metal: bool,
    pub evidence_status: String,
    pub note: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LocalLinearReport {
    pub label: String,
    pub rank: usize,
    pub state_dim: usize,
    pub input_dim: usize,
    pub around: String,
    pub timestep: f64,
    pub a_shape: [usize; 2],
    pub b_shape: [usize; 2],
    pub tolerance: f64,
    pub mujoco_version: String,
    pub method: String,
    pub note: String,
}

pub fn qualify(
    inst: &mut MujocoInstance,
    manifest: &RobotManifest,
) -> Result<ControlQualification, String> {
    let mut probes = Vec::new();
    let mut unstable = false;
    let mut mapping_invalid = false;
    for (i, act) in manifest.actuators.iter().enumerate() {
        let mapped = manifest
            .joints
            .iter()
            .find(|j| j.name == act.transmission_target);
        let directional = mapped
            .map(|j| j.qpos_dim == 1 && !matches!(j.joint_type.as_str(), "free" | "ball"))
            .unwrap_or(false);
        if !directional {
            probes.push(ActuatorProbe {
                name: act.name.clone(),
                joint: act.transmission_target.clone(),
                pos_response: 0.0,
                neg_response: 0.0,
                sign_ok: None,
                finite: true,
                within_limits: true,
                tracking_error: None,
                settling_time_s: None,
                metric_status: "NOT_APPLICABLE".into(),
                divergent: false,
            });
            continue;
        }
        let span = (act.ctrlrange[1] - act.ctrlrange[0]).abs().max(0.2);
        let mid = 0.5 * (act.ctrlrange[0] + act.ctrlrange[1]);
        let delta = (0.08 * span).max(0.02);
        let pos_cmd = if act.actuator_type == "position" {
            mid + delta
        } else {
            delta
        };
        let neg_cmd = if act.actuator_type == "position" {
            mid - delta
        } else {
            -delta
        };
        let pos = probe_dir(inst, manifest, i, pos_cmd)?;
        let neg = probe_dir(inst, manifest, i, neg_cmd)?;
        let q0 = rest_q(inst)?;
        let pos_ok = pos.response > 1e-6;
        let neg_ok = neg.response < -1e-6;
        let sign_ok = pos_ok && neg_ok;
        if !sign_ok {
            mapping_invalid = true;
        }
        if pos.divergent || neg.divergent {
            unstable = true;
        }
        let tracking = if act.actuator_type == "position" {
            Some(pos.tracking.min(neg.tracking))
        } else {
            None
        };
        probes.push(ActuatorProbe {
            name: act.name.clone(),
            joint: act.transmission_target.clone(),
            pos_response: pos.response,
            neg_response: neg.response,
            sign_ok: Some(sign_ok),
            finite: pos.finite && neg.finite,
            within_limits: pos.within && neg.within,
            tracking_error: tracking,
            settling_time_s: pos.settle_s.or(neg.settle_s),
            metric_status: "OK".into(),
            divergent: pos.divergent || neg.divergent,
        });
        let _ = q0;
        let _ = inst.reset(None, None);
    }
    let rest_ok = rest_recovery(inst, manifest)?;
    let pert_ok = perturbation_recovery(inst, manifest)?;
    let workspace = if manifest.derived.base_type == BaseType::Fixed {
        workspace_samples(inst, manifest)?
    } else {
        Vec::new()
    };
    let class = if unstable {
        ControlClass::ControllerUnstable
    } else if mapping_invalid && probes.iter().all(|p| p.sign_ok != Some(true)) {
        ControlClass::ControlMappingInvalid
    } else if !manifest.derived.passive_dofs.is_empty() {
        if manifest.nu < manifest.nv {
            ControlClass::Underactuated
        } else {
            ControlClass::PartiallyActuated
        }
    } else if manifest.nu < manifest.nv {
        ControlClass::Underactuated
    } else if manifest.derived.passive_dofs.is_empty() {
        ControlClass::FullyActuated
    } else {
        ControlClass::PartiallyActuated
    };
    let llc = match inst.local_linear() {
        Ok(v) => Some(LocalLinearReport {
            label: "LOCAL_LINEAR_CONTROLLABILITY".into(),
            rank: v["rank"].as_u64().unwrap_or(0) as usize,
            state_dim: v["state_dim"].as_u64().unwrap_or(0) as usize,
            input_dim: v["input_dim"].as_u64().unwrap_or(0) as usize,
            around: v["state"].as_str().unwrap_or("compiled_qpos0_qvel0").into(),
            timestep: v["timestep"].as_f64().unwrap_or(manifest.timestep),
            a_shape: [
                v["A_shape"][0].as_u64().unwrap_or(0) as usize,
                v["A_shape"][1].as_u64().unwrap_or(0) as usize,
            ],
            b_shape: [
                v["B_shape"][0].as_u64().unwrap_or(0) as usize,
                v["B_shape"][1].as_u64().unwrap_or(0) as usize,
            ],
            tolerance: v["tolerance"].as_f64().unwrap_or(1e-6),
            mujoco_version: v["mujoco_version"]
                .as_str()
                .unwrap_or(&manifest.mujoco_version)
                .into(),
            method: v["method"].as_str().unwrap_or("mjd_transitionFD").into(),
            note: v["note"]
                .as_str()
                .unwrap_or(
                    "Local discrete linear controllability. Not global nonlinear controllability.",
                )
                .into(),
        }),
        Err(_) => None,
    };
    let note = if llc.is_some() {
        "Practical +/- probes plus LOCAL_LINEAR_CONTROLLABILITY from mjd_transitionFD. Not global nonlinear controllability.".into()
    } else {
        "LOCAL_LINEAR_CONTROLLABILITY=NOT_EVALUATED. Practical probes only. Rank was not fabricated from dimensions.".into()
    };
    Ok(ControlQualification {
        robot_id: manifest.robot_id.clone(),
        model_hash: manifest.model_hash.clone(),
        class,
        local_linear_controllability: llc,
        probes,
        rest_recovery_ok: rest_ok,
        perturbation_recovery_ok: pert_ok,
        workspace_samples: workspace,
        controlled_dofs: manifest.derived.actuated_dofs.clone(),
        passive_dofs: manifest.derived.passive_dofs.clone(),
        dynamically_coupled_dofs: manifest.derived.passive_dofs.clone(),
        metal: false,
        evidence_status: crate::honesty::SIMULATION_ONLY.into(),
        note,
    })
}

struct Dir {
    response: f64,
    finite: bool,
    within: bool,
    tracking: f64,
    settle_s: Option<f64>,
    divergent: bool,
}

fn rest_q(inst: &mut MujocoInstance) -> Result<Vec<f64>, String> {
    let r = inst.reset(None, None).map_err(|e| e.to_string())?;
    Ok(crate::mujoco_exec::json_f64_vec(&r["state"]["qpos"]))
}

fn probe_dir(
    inst: &mut MujocoInstance,
    manifest: &RobotManifest,
    idx: usize,
    cmd: f64,
) -> Result<Dir, String> {
    let _ = inst.reset(None, None).map_err(|e| e.to_string())?;
    let q0 = rest_q(inst)?;
    let act = &manifest.actuators[idx];
    let joint = manifest
        .joints
        .iter()
        .find(|j| j.name == act.transmission_target);
    let adr = joint
        .map(|j| j.qpos_address as usize)
        .unwrap_or(idx.min(q0.len().saturating_sub(1)));
    let mut ctrl = vec![0.0; manifest.nu as usize];
    ctrl[idx] = cmd;
    let dt = manifest.timestep.max(1e-4);
    let mut last_q = q0.get(adr).copied().unwrap_or(0.0);
    let mut settle_s = None;
    let mut tracking = f64::MAX;
    let mut finite = true;
    let mut within = true;
    let mut speed = 0.0;
    for k in 0..40 {
        inst.set_ctrl(&ctrl).map_err(|e| e.to_string())?;
        let stepped = inst.step(1).map_err(|e| e.to_string())?;
        let st = &stepped["state"];
        let q1 = crate::mujoco_exec::json_f64_vec(&st["qpos"]);
        let v1 = crate::mujoco_exec::json_f64_vec(&st["qvel"]);
        finite = finite && q1.iter().all(|x| x.is_finite()) && v1.iter().all(|x| x.is_finite());
        last_q = q1.get(adr).copied().unwrap_or(last_q);
        speed = v1.iter().fold(0.0_f64, |a, b| a.max(b.abs()));
        if act.actuator_type == "position" {
            tracking = tracking.min((last_q - cmd).abs());
            if (last_q - cmd).abs() < 0.05 && speed < 0.2 && settle_s.is_none() {
                settle_s = Some((k as f64 + 1.0) * dt);
            }
        } else if speed < 0.2 && (last_q - q0.get(adr).copied().unwrap_or(0.0)).abs() > 1e-4 {
            settle_s.get_or_insert((k as f64 + 1.0) * dt);
        }
        within = within
            && manifest.joints.iter().all(|j| {
                if !j.limited || j.qpos_dim != 1 {
                    return true;
                }
                q1.get(j.qpos_address as usize)
                    .map(|q| *q >= j.range[0] - 0.05 && *q <= j.range[1] + 0.05)
                    .unwrap_or(true)
            });
    }
    let response = last_q - q0.get(adr).copied().unwrap_or(0.0);
    if act.actuator_type != "position" {
        tracking = f64::NAN;
    }
    Ok(Dir {
        response,
        finite,
        within,
        tracking,
        settle_s,
        divergent: !finite || speed > 50.0,
    })
}

fn rest_recovery(inst: &mut MujocoInstance, manifest: &RobotManifest) -> Result<bool, String> {
    let q0 = rest_q(inst)?;
    inst.set_ctrl(&vec![0.0; manifest.nu as usize])
        .map_err(|e| e.to_string())?;
    let r = inst.step(30).map_err(|e| e.to_string())?;
    let q1 = crate::mujoco_exec::json_f64_vec(&r["state"]["qpos"]);
    let v = crate::mujoco_exec::json_f64_vec(&r["state"]["qvel"]);
    let near = q0.iter().zip(q1.iter()).all(|(a, b)| (a - b).abs() < 0.25);
    Ok(v.iter().all(|x| x.is_finite() && x.abs() < 15.0) && near)
}

fn perturbation_recovery(
    inst: &mut MujocoInstance,
    manifest: &RobotManifest,
) -> Result<bool, String> {
    let q0 = rest_q(inst)?;
    let body = manifest
        .bodies
        .iter()
        .find(|b| b.name != "world" && b.mass > 0.0)
        .map(|b| b.name.as_str())
        .unwrap_or("link1");
    inst.set_ctrl(&vec![0.0; manifest.nu as usize])
        .map_err(|e| e.to_string())?;
    let _ = inst.apply_force(body, [8.0, 0.0, 0.0]);
    let pert = inst.step(15).map_err(|e| e.to_string())?;
    let _ = inst.clear_forces();
    let mut q_pert = crate::mujoco_exec::json_f64_vec(&pert["state"]["qpos"]);
    if dist(&q0, &q_pert) < 1e-5 {
        let mut kicked = q0.clone();
        if let Some(j) = manifest.joints.iter().find(|j| j.qpos_dim == 1) {
            if (j.qpos_address as usize) < kicked.len() {
                kicked[j.qpos_address as usize] += 0.12;
            }
        } else if !kicked.is_empty() {
            kicked[0] += 0.12;
        }
        let kicked_state = inst
            .reset(Some(&kicked), Some(&vec![0.0; manifest.nv as usize]))
            .map_err(|e| e.to_string())?;
        q_pert = crate::mujoco_exec::json_f64_vec(&kicked_state["state"]["qpos"]);
        let rest_ctrl: Vec<f64> = manifest
            .actuators
            .iter()
            .map(|a| {
                if a.actuator_type == "position" {
                    0.5 * (a.ctrlrange[0] + a.ctrlrange[1]).clamp(-0.05, 0.05)
                } else {
                    0.0
                }
            })
            .collect();
        inst.set_ctrl(&rest_ctrl).map_err(|e| e.to_string())?;
    }
    let rec = inst.step(50).map_err(|e| e.to_string())?;
    let q_rec = crate::mujoco_exec::json_f64_vec(&rec["state"]["qpos"]);
    let nan = rec["state"]["nan"].as_bool().unwrap_or(false);
    let finite = q_rec.iter().all(|x| x.is_finite());
    let d_pert = dist(&q0, &q_pert);
    let d_rec = dist(&q0, &q_rec);
    Ok(!nan && finite && d_pert > 1e-6 && d_rec < d_pert)
}

fn dist(a: &[f64], b: &[f64]) -> f64 {
    a.iter()
        .zip(b.iter())
        .map(|(x, y)| (x - y).powi(2))
        .sum::<f64>()
        .sqrt()
}

fn workspace_samples(
    inst: &mut MujocoInstance,
    manifest: &RobotManifest,
) -> Result<Vec<[f64; 3]>, String> {
    let mut out = Vec::new();
    let n = 6usize;
    for k in 0..n {
        let mut q = vec![0.0; manifest.nq as usize];
        for j in manifest
            .joints
            .iter()
            .filter(|j| j.limited && j.qpos_dim == 1 && j.joint_type != "free")
        {
            let t = k as f64 / n.max(1) as f64;
            if (j.qpos_address as usize) < q.len() {
                q[j.qpos_address as usize] = j.range[0] * (1.0 - t) + j.range[1] * t * 0.25;
            }
        }
        let r = inst
            .reset(Some(&q), Some(&vec![0.0; manifest.nv as usize]))
            .map_err(|e| e.to_string())?;
        if let Some(ee) = r["state"]["sites"]
            .as_object()
            .and_then(|o| o.values().next())
        {
            let p = crate::mujoco_exec::json_f64_vec(ee);
            if p.len() >= 3 {
                out.push([p[0], p[1], p[2]]);
            }
        }
    }
    let _ = inst.reset(None, None);
    Ok(out)
}
