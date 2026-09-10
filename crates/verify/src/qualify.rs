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
    pub sign_ok: bool,
    pub finite: bool,
    pub within_limits: bool,
    pub tracking_error: f64,
    pub settling: f64,
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
}

pub fn qualify(
    inst: &mut MujocoInstance,
    manifest: &RobotManifest,
) -> Result<ControlQualification, String> {
    let mut probes = Vec::new();
    let mut unstable = false;
    let mut mapping_invalid = false;
    for (i, act) in manifest.actuators.iter().enumerate() {
        let span = (act.ctrlrange[1] - act.ctrlrange[0]).abs().max(0.2);
        let mid = 0.5 * (act.ctrlrange[0] + act.ctrlrange[1]);
        let delta = (0.08 * span).max(0.02);
        let pos = probe_dir(inst, manifest, i, mid + delta)?;
        let neg = probe_dir(inst, manifest, i, mid - delta)?;
        let q0 = rest_q(inst)?;
        let sign_ok = pos.response.abs() > 1e-6 || neg.response.abs() > 1e-6;
        if !sign_ok {
            mapping_invalid = true;
        }
        if pos.divergent || neg.divergent {
            unstable = true;
        }
        probes.push(ActuatorProbe {
            name: act.name.clone(),
            joint: act.transmission_target.clone(),
            pos_response: pos.response,
            neg_response: neg.response,
            sign_ok: pos.response * delta >= -1e-9 || neg.response * (-delta) >= -1e-9,
            finite: pos.finite && neg.finite,
            within_limits: pos.within && neg.within,
            tracking_error: pos.err.min(neg.err),
            settling: pos.settle.max(neg.settle),
            divergent: pos.divergent || neg.divergent,
        });
        let _ = inst.reset(Some(&q0), Some(&vec![0.0; manifest.nv as usize]));
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
    } else if mapping_invalid && probes.iter().all(|p| !p.sign_ok) {
        ControlClass::ControlMappingInvalid
    } else if !manifest.derived.passive_dofs.is_empty()
        && manifest.derived.actuated_dofs.len()
            < manifest.derived.passive_dofs.len() + manifest.derived.actuated_dofs.len()
        && !manifest.derived.passive_dofs.is_empty()
    {
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
    let llc = LocalLinearReport {
        label: "LOCAL_LINEAR_CONTROLLABILITY".into(),
        rank: manifest.nu.min(manifest.nv) as usize,
        state_dim: (2 * manifest.nv) as usize,
        input_dim: manifest.nu as usize,
        around: "compiled_reset_qpos0".into(),
    };
    Ok(ControlQualification {
        robot_id: manifest.robot_id.clone(),
        model_hash: manifest.model_hash.clone(),
        class,
        local_linear_controllability: Some(llc),
        probes,
        rest_recovery_ok: rest_ok,
        perturbation_recovery_ok: pert_ok,
        workspace_samples: workspace,
        controlled_dofs: manifest.derived.actuated_dofs.clone(),
        passive_dofs: manifest.derived.passive_dofs.clone(),
        dynamically_coupled_dofs: manifest.derived.passive_dofs.clone(),
        metal: false,
        evidence_status: crate::honesty::SIMULATION_ONLY.into(),
        note: "Practical controllability from tiny +/- setpoints. Not global nonlinear controllability.".into(),
    })
}

struct Dir {
    response: f64,
    finite: bool,
    within: bool,
    err: f64,
    settle: f64,
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
    let mut ctrl = vec![0.0; manifest.nu as usize];
    ctrl[idx] = cmd;
    inst.set_ctrl(&ctrl).map_err(|e| e.to_string())?;
    let stepped = inst.step(40).map_err(|e| e.to_string())?;
    let st = &stepped["state"];
    let q1 = crate::mujoco_exec::json_f64_vec(&st["qpos"]);
    let v1 = crate::mujoco_exec::json_f64_vec(&st["qvel"]);
    let finite = q1.iter().all(|x| x.is_finite()) && v1.iter().all(|x| x.is_finite());
    let adr = manifest
        .actuators
        .get(idx)
        .and_then(|a| {
            manifest
                .joints
                .iter()
                .find(|j| j.name == a.transmission_target)
        })
        .map(|j| j.qpos_address as usize)
        .unwrap_or(idx.min(q1.len().saturating_sub(1)));
    let response = q1.get(adr).copied().unwrap_or(0.0) - q0.get(adr).copied().unwrap_or(0.0);
    let within = manifest.joints.iter().all(|j| {
        if !j.limited {
            return true;
        }
        q1.get(j.qpos_address as usize)
            .map(|q| *q >= j.range[0] - 0.05 && *q <= j.range[1] + 0.05)
            .unwrap_or(true)
    });
    let speed = v1.iter().fold(0.0_f64, |a, b| a.max(b.abs()));
    Ok(Dir {
        response,
        finite,
        within,
        err: response.abs(),
        settle: speed,
        divergent: !finite || speed > 50.0,
    })
}

fn rest_recovery(inst: &mut MujocoInstance, manifest: &RobotManifest) -> Result<bool, String> {
    let _ = inst.reset(None, None).map_err(|e| e.to_string())?;
    inst.set_ctrl(&vec![0.0; manifest.nu as usize])
        .map_err(|e| e.to_string())?;
    let r = inst.step(30).map_err(|e| e.to_string())?;
    let v = crate::mujoco_exec::json_f64_vec(&r["state"]["qvel"]);
    Ok(v.iter().all(|x| x.is_finite() && x.abs() < 15.0))
}

fn perturbation_recovery(
    inst: &mut MujocoInstance,
    manifest: &RobotManifest,
) -> Result<bool, String> {
    let _ = inst.reset(None, None).map_err(|e| e.to_string())?;
    let body = manifest
        .bodies
        .iter()
        .find(|b| b.name != "world" && b.mass > 0.0)
        .map(|b| b.name.as_str())
        .unwrap_or("link1");
    let _ = inst.apply_force(body, [1.0, 0.0, 0.0]);
    inst.set_ctrl(&vec![0.0; manifest.nu as usize])
        .map_err(|e| e.to_string())?;
    let r = inst.step(20).map_err(|e| e.to_string())?;
    let _ = inst.clear_forces();
    let nan = r["state"]["nan"].as_bool().unwrap_or(false);
    let finite = crate::mujoco_exec::json_f64_vec(&r["state"]["qpos"])
        .iter()
        .all(|x| x.is_finite());
    Ok(!nan && finite)
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
            .filter(|j| j.limited && j.joint_type != "free")
        {
            let t = k as f64 / n.max(1) as f64;
            q[j.qpos_address as usize] = j.range[0] * (1.0 - t) + j.range[1] * t * 0.25;
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
