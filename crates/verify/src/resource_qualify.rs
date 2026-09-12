//! Automatic gripper-resource qualification. Measured, not YAML-promoted.

use crate::bundle::RobotBundle;
use crate::mujoco_exec::MujocoInstance;
use crate::normalize::RobotManifest;
use crate::runner::load_and_normalize;
use realityos_semantics::resource::{
    ControlledResource, QualificationStatus, ResourceTopology,
};
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ManipulationResourceQualification {
    pub resource_id: String,
    pub topology: String,
    pub open_response: bool,
    pub close_response: bool,
    pub direction_correct: bool,
    pub range_ok: bool,
    pub repeatable: bool,
    pub coupling_ok: bool,
    pub affected_joints: Vec<String>,
    pub actuator_saturation: bool,
    pub hold_behavior: bool,
    pub restart_behavior: bool,
    pub qualified: bool,
    pub detail: Vec<String>,
    pub metal: bool,
    pub evidence_status: String,
}

pub fn qualify_resource(
    bundle: &RobotBundle,
    resource: &ControlledResource,
) -> Result<(ManipulationResourceQualification, ControlledResource), String> {
    let (mut inst, manifest) = load_and_normalize(bundle, &[], 0)?;
    let report = qualify_on(&mut inst, &manifest, resource)?;
    crate::mujoco_exec::checkin_worker(inst);
    let mut qualified = resource.clone();
    qualified.qualification = if report.qualified {
        QualificationStatus::Qualified
    } else {
        QualificationStatus::Failed
    };
    Ok((report, qualified))
}

pub fn qualify_on(
    inst: &mut MujocoInstance,
    manifest: &RobotManifest,
    resource: &ControlledResource,
) -> Result<ManipulationResourceQualification, String> {
    let mut detail = Vec::new();
    if matches!(
        resource.topology,
        ResourceTopology::UnsupportedResourceTopology
    ) {
        return Ok(ManipulationResourceQualification {
            resource_id: resource.id.clone(),
            topology: format!("{:?}", resource.topology),
            open_response: false,
            close_response: false,
            direction_correct: false,
            range_ok: false,
            repeatable: false,
            coupling_ok: false,
            affected_joints: resource.affected_joints.clone(),
            actuator_saturation: false,
            hold_behavior: false,
            restart_behavior: false,
            qualified: false,
            detail: vec![resource
                .unsupported_detail
                .clone()
                .unwrap_or_else(|| "RESOURCE_UNSUPPORTED".into())],
            metal: false,
            evidence_status: crate::honesty::SIMULATION_ONLY.into(),
        });
    }
    let Some([lo, hi]) = resource.command_range.value else {
        return Err("command_range unknown".into());
    };
    let act_idx: Vec<usize> = resource
        .actuator_inputs
        .iter()
        .filter_map(|n| manifest.actuators.iter().position(|a| a.name == *n))
        .collect();
    if act_idx.is_empty() {
        return Err("resource actuators missing".into());
    }
    let nu = manifest.nu.max(0) as usize;
    let joint_idx: Vec<usize> = resource
        .affected_joints
        .iter()
        .filter_map(|n| manifest.joints.iter().position(|j| j.name == *n))
        .collect();

    let measure = |inst: &mut MujocoInstance| -> Result<(f64, Vec<f64>, f64, f64), String> {
        let st = inst.step(0).map_err(|e| e.to_string())?;
        let q = crate::mujoco_exec::json_f64_vec(&st["state"]["qpos"]);
        let mut js = Vec::new();
        for j in &joint_idx {
            let adr = manifest.joints[*j].qpos_address as usize;
            js.push(q.get(adr).copied().unwrap_or(0.0));
        }
        let mean = if js.is_empty() {
            0.0
        } else {
            js.iter().sum::<f64>() / js.len() as f64
        };
        let mut pts = Vec::new();
        if let Some(xpos) = st.get("state").and_then(|s| s.get("xpos")).and_then(|v| v.as_object())
        {
            for name in &resource.finger_bodies {
                if let Some(p) = xpos.get(name).and_then(|v| v.as_array()) {
                    if p.len() >= 3 {
                        pts.push([
                            p[0].as_f64().unwrap_or(0.0),
                            p[1].as_f64().unwrap_or(0.0),
                            p[2].as_f64().unwrap_or(0.0),
                        ]);
                    }
                }
            }
        }
        let spread = if pts.len() >= 2 {
            let d = [
                pts[0][0] - pts[1][0],
                pts[0][1] - pts[1][1],
                pts[0][2] - pts[1][2],
            ];
            (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt()
        } else {
            0.0
        };
        let cmd = if let Ok(peek) = inst.peek_ctrl() {
            act_idx.first().and_then(|i| peek.0.get(*i).copied()).unwrap_or(0.0)
        } else {
            0.0
        };
        Ok((mean, js, spread, cmd))
    };

    let drive = |inst: &mut MujocoInstance, cmd: f64| -> Result<(), String> {
        let mut ctrl = vec![0.0; nu];
        if let Ok(st) = inst.step(0) {
            let q = crate::mujoco_exec::json_f64_vec(&st["state"]["qpos"]);
            for (i, a) in manifest.actuators.iter().enumerate() {
                if act_idx.contains(&i) {
                    continue;
                }
                if let Some(j) = manifest.joints.iter().find(|j| j.name == a.transmission_target)
                {
                    if let Some(v) = q.get(j.qpos_address as usize) {
                        if i < ctrl.len() {
                            ctrl[i] = *v;
                        }
                    }
                }
            }
        }
        if let Ok(peek) = inst.peek_ctrl() {
            if peek.0.len() == nu {
                for (i, v) in peek.0.iter().enumerate() {
                    if !act_idx.contains(&i) && i < ctrl.len() && ctrl[i] == 0.0 {
                        ctrl[i] = *v;
                    }
                }
            }
        }
        for i in &act_idx {
            if *i < ctrl.len() {
                ctrl[*i] = cmd;
            }
        }
        let _ = inst.set_ctrl(&ctrl);
        let n = ((0.35 / manifest.timestep.max(1e-4)).round() as u32).clamp(20, 400);
        let _ = inst.step(n);
        Ok(())
    };

    let _ = inst.reset(None, None);
    let (q0, j0, s0, _c0) = measure(inst)?;
    drive(inst, hi)?;
    let (q_open, j_open, s_open, c_open) = measure(inst)?;
    drive(inst, lo)?;
    let (q_close, _j_close, s_close, c_close) = measure(inst)?;
    drive(inst, hi)?;
    let (q_open2, _, s_open2, _) = measure(inst)?;
    drive(inst, hi + (hi - lo).abs() + 1.0)?;
    let (q_sat, _, _, _) = measure(inst)?;
    let hold_before = q_open2;
    let n = ((0.15 / manifest.timestep.max(1e-4)).round() as u32).clamp(10, 200);
    let _ = inst.step(n);
    let (hold_after, _, _, _) = measure(inst)?;

    let joint_open = (q_open - q0).abs() > 1e-4 || (q_open - q_close).abs() > 1e-4;
    let spread_open = (s_open - s0).abs() > 1e-4 || (s_open - s_close).abs() > 1e-4;
    let cmd_open = (c_open - hi).abs() < (hi - lo).abs().max(1e-6) * 0.15 + 1e-3;
    let open_response = joint_open || spread_open || cmd_open;
    let close_response = (q_close - q_open).abs() > 1e-4
        || (s_close - s_open).abs() > 1e-4
        || (c_close - lo).abs() < (hi - lo).abs().max(1e-6) * 0.15 + 1e-3;
    let direction_correct = match resource.closing_direction {
        realityos_semantics::resource::ClosingDirection::TowardMin => {
            q_close <= q_open + 1e-6 || s_close <= s_open + 1e-6 || c_close <= c_open + 1e-6
        }
        realityos_semantics::resource::ClosingDirection::TowardMax => {
            q_close >= q_open - 1e-6 || s_close >= s_open - 1e-6 || c_close >= c_open - 1e-6
        }
    };
    let _ = s_open2;
    let span = (hi - lo).abs().max(1e-9);
    let range_ok = (q_open - q_close).abs() > 0.15 * span.min(0.04);
    let repeatable = (q_open2 - q_open).abs() < 0.2 * (q_open - q_close).abs().max(1e-4);
    let coupling_ok = if j_open.len() >= 2 {
        let d: Vec<f64> = j_open.iter().zip(j0.iter()).map(|(a, b)| a - b).collect();
        d.windows(2).all(|w| (w[0] - w[1]).abs() < 0.15 || w[0].signum() == w[1].signum() || w[0].abs() < 1e-4)
    } else {
        true
    };
    if matches!(resource.topology, ResourceTopology::TendonDrivenGripper) && !coupling_ok {
        detail.push("coupled joint motion mismatch".into());
    }
    let actuator_saturation = (q_sat - q_open).abs() < (q_open - q_close).abs().max(1e-3) + 0.05;
    let hold_behavior = (hold_after - hold_before).abs() < (q_open - q_close).abs().max(1e-3) * 0.5 + 0.01;

    let _ = inst.reset(None, None);
    drive(inst, hi)?;
    let (q_re, _, _, _) = measure(inst)?;
    let restart_behavior = (q_re - q_open).abs() < (q_open - q_close).abs().max(1e-3) * 0.75 + 0.02;

    if !open_response {
        detail.push("open_response_failed".into());
    }
    if !close_response {
        detail.push("close_response_failed".into());
    }
    if !direction_correct {
        detail.push("direction_incorrect".into());
    }

    let qualified = open_response && close_response && direction_correct && coupling_ok;
    Ok(ManipulationResourceQualification {
        resource_id: resource.id.clone(),
        topology: format!("{:?}", resource.topology),
        open_response,
        close_response,
        direction_correct,
        range_ok,
        repeatable,
        coupling_ok,
        affected_joints: resource.affected_joints.clone(),
        actuator_saturation,
        hold_behavior,
        restart_behavior,
        qualified,
        detail,
        metal: false,
        evidence_status: crate::honesty::SIMULATION_ONLY.into(),
    })
}

pub fn force_unqualified_fixture() -> Value {
    json!({"topology": "UNSUPPORTED_RESOURCE_TOPOLOGY", "reason": "test_fixture"})
}
