//! RobotManifest mechanically inspected from compiled mjModel (+ bundle semantics).

use crate::bundle::{BaseType, RobotBundle};
use crate::format::ModelFormat;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Default)]
pub struct JointRecord {
    pub name: String,
    pub joint_type: String,
    pub qpos_address: i32,
    pub velocity_address: i32,
    pub qpos_dim: i32,
    pub dof_dim: i32,
    pub range: [f64; 2],
    pub limited: bool,
    pub parent_body: String,
    pub child_body: String,
    #[serde(default)]
    pub unsupported_reason: Option<String>,
    #[serde(default)]
    pub axis: Option<[f64; 3]>,
    #[serde(default)]
    pub pos: Option<[f64; 3]>,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct ActuatorRecord {
    pub name: String,
    pub transmission_target: String,
    pub control_dimensions: usize,
    pub ctrlrange: [f64; 2],
    pub ctrllimited: bool,
    pub force_range: Option<[f64; 2]>,
    pub actuator_type: String,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SensorRecord {
    pub name: String,
    pub sensor_type: String,
    pub dimensions: i32,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct CameraRecord {
    pub name: String,
    pub parent_body: String,
    #[serde(default)]
    pub pos: Option<[f64; 3]>,
    #[serde(default)]
    pub quat: Option<[f64; 4]>,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SiteRecord {
    pub name: String,
    pub body: String,
    #[serde(default)]
    pub pos: Option<[f64; 3]>,
    #[serde(default)]
    pub quat: Option<[f64; 4]>,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct BodyRecord {
    pub name: String,
    pub mass: f64,
    pub inertia: [f64; 3],
    pub parent: String,
    #[serde(default)]
    pub pos: Option<[f64; 3]>,
    #[serde(default)]
    pub quat: Option<[f64; 4]>,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct DerivedInterface {
    pub base_type: BaseType,
    pub actuated_dofs: Vec<String>,
    pub passive_dofs: Vec<String>,
    pub end_effector_chains: Vec<Vec<String>>,
    #[serde(default)]
    pub end_effector_joint_chains: Vec<Vec<String>>,
    pub actuator_coverage: f64,
    pub potentially_uncontrollable_joints: Vec<String>,
}

impl Default for DerivedInterface {
    fn default() -> Self {
        Self {
            base_type: crate::bundle::BaseType::Fixed,
            actuated_dofs: Vec::new(),
            passive_dofs: Vec::new(),
            end_effector_chains: Vec::new(),
            end_effector_joint_chains: Vec::new(),
            actuator_coverage: 0.0,
            potentially_uncontrollable_joints: Vec::new(),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct RobotManifest {
    pub robot_id: String,
    pub nq: i32,
    pub nv: i32,
    pub nu: i32,
    pub nbody: i32,
    pub njoint: i32,
    pub nactuator: i32,
    pub nsensor: i32,
    pub ncamera: i32,
    pub timestep: f64,
    pub joints: Vec<JointRecord>,
    pub actuators: Vec<ActuatorRecord>,
    pub sensors: Vec<SensorRecord>,
    pub cameras: Vec<CameraRecord>,
    pub bodies: Vec<BodyRecord>,
    pub sites: Vec<(String, String)>,
    #[serde(default)]
    pub site_records: Vec<SiteRecord>,
    pub derived: DerivedInterface,
    pub model_hash: String,
    pub source_hash: String,
    pub mujoco_version: String,
    pub source_format: String,
    pub lost_features: Vec<String>,
    #[serde(default)]
    pub support_bodies: Vec<String>,
    #[serde(default)]
    pub collision_groups: std::collections::BTreeMap<String, Vec<String>>,
    pub metal: bool,
    pub evidence_status: String,
}

impl RobotManifest {
    pub fn from_inspect(bundle: &RobotBundle, inspect: &serde_json::Value) -> Self {
        let joints: Vec<JointRecord> = inspect
            .get("joints")
            .and_then(|v| v.as_array())
            .map(|arr| {
                arr.iter()
                    .map(|j| {
                        let jtype = joint_type_name(j["type"].as_i64().unwrap_or(-1));
                        let (qdim, ddim) = joint_dims(&jtype);
                        JointRecord {
                            name: j["name"].as_str().unwrap_or("").into(),
                            joint_type: jtype.clone(),
                            qpos_address: j["qposadr"].as_i64().unwrap_or(0) as i32,
                            velocity_address: j["dofadr"].as_i64().unwrap_or(0) as i32,
                            qpos_dim: j["qpos_dim"].as_i64().unwrap_or(qdim as i64) as i32,
                            dof_dim: j["dof_dim"].as_i64().unwrap_or(ddim as i64) as i32,
                            range: [
                                j["range"][0].as_f64().unwrap_or(0.0),
                                j["range"][1].as_f64().unwrap_or(0.0),
                            ],
                            limited: j["limited"].as_bool().unwrap_or(false),
                            parent_body: j["parent_body"].as_str().unwrap_or("").into(),
                            child_body: j["child_body"].as_str().unwrap_or("").into(),
                            unsupported_reason: j["unsupported_reason"]
                                .as_str()
                                .map(|s| s.to_string()),
                            axis: vec3_opt(&j["axis"]),
                            pos: vec3_opt(&j["pos"]),
                        }
                    })
                    .collect()
            })
            .unwrap_or_default();
        let actuators: Vec<ActuatorRecord> = inspect
            .get("actuators")
            .and_then(|v| v.as_array())
            .map(|arr| {
                arr.iter()
                    .map(|a| ActuatorRecord {
                        name: a["name"].as_str().unwrap_or("").into(),
                        transmission_target: a["target"].as_str().unwrap_or("").into(),
                        control_dimensions: 1,
                        ctrlrange: [
                            a["ctrlrange"][0].as_f64().unwrap_or(0.0),
                            a["ctrlrange"][1].as_f64().unwrap_or(0.0),
                        ],
                        ctrllimited: a["ctrllimited"].as_bool().unwrap_or(false),
                        force_range: if a["forcelimited"].as_bool().unwrap_or(false) {
                            Some([
                                a["forcerange"][0].as_f64().unwrap_or(0.0),
                                a["forcerange"][1].as_f64().unwrap_or(0.0),
                            ])
                        } else {
                            None
                        },
                        actuator_type: actuator_type_name(
                            a["gaintype"].as_i64().unwrap_or(0),
                            a["biastype"].as_i64().unwrap_or(0),
                        ),
                    })
                    .collect()
            })
            .unwrap_or_default();
        let sensors = inspect
            .get("sensors")
            .and_then(|v| v.as_array())
            .map(|arr| {
                arr.iter()
                    .map(|s| SensorRecord {
                        name: s["name"].as_str().unwrap_or("").into(),
                        sensor_type: s["type"].as_i64().unwrap_or(0).to_string(),
                        dimensions: s["dim"].as_i64().unwrap_or(0) as i32,
                    })
                    .collect()
            })
            .unwrap_or_default();
        let cameras = inspect
            .get("cameras")
            .and_then(|v| v.as_array())
            .map(|arr| {
                arr.iter()
                    .map(|c| CameraRecord {
                        name: c["name"].as_str().unwrap_or("").into(),
                        parent_body: c["parent_body"].as_str().unwrap_or("").into(),
                        pos: vec3_opt(&c["pos"]),
                        quat: vec4_opt(&c["quat"]),
                    })
                    .collect()
            })
            .unwrap_or_default();
        let bodies: Vec<BodyRecord> = inspect
            .get("bodies")
            .and_then(|v| v.as_array())
            .map(|arr| {
                arr.iter()
                    .map(|b| BodyRecord {
                        name: b["name"].as_str().unwrap_or("").into(),
                        mass: b["mass"].as_f64().unwrap_or(0.0),
                        inertia: [
                            b["inertia"][0].as_f64().unwrap_or(0.0),
                            b["inertia"][1].as_f64().unwrap_or(0.0),
                            b["inertia"][2].as_f64().unwrap_or(0.0),
                        ],
                        parent: b["parent"].as_str().unwrap_or("").into(),
                        pos: vec3_opt(&b["pos"]),
                        quat: vec4_opt(&b["quat"]),
                    })
                    .collect()
            })
            .unwrap_or_default();
        let site_records: Vec<SiteRecord> = inspect
            .get("sites")
            .and_then(|v| v.as_array())
            .map(|arr| {
                arr.iter()
                    .filter_map(|s| {
                        Some(SiteRecord {
                            name: s["name"].as_str()?.to_string(),
                            body: s["body"].as_str()?.to_string(),
                            pos: vec3_opt(&s["pos"]),
                            quat: vec4_opt(&s["quat"]),
                        })
                    })
                    .collect()
            })
            .unwrap_or_default();
        let sites = site_records
            .iter()
            .map(|s| (s.name.clone(), s.body.clone()))
            .collect();

        let actuated: Vec<String> = actuators
            .iter()
            .map(|a| a.transmission_target.clone())
            .filter(|t| !t.is_empty())
            .collect();
        let passive: Vec<String> = joints
            .iter()
            .filter(|j| j.joint_type != "free" && !actuated.iter().any(|t| t == &j.name))
            .map(|j| j.name.clone())
            .collect();
        let coverage = if joints.is_empty() {
            0.0
        } else {
            actuated.len() as f64
                / joints
                    .iter()
                    .filter(|j| j.joint_type != "free")
                    .count()
                    .max(1) as f64
        };
        let mut ee_chains = Vec::new();
        let mut ee_joint_chains = Vec::new();
        if let Some(arr) = inspect
            .get("end_effector_chains")
            .and_then(|v| v.as_array())
        {
            for ch in arr {
                let bodies = ch
                    .get("bodies")
                    .and_then(|v| v.as_array())
                    .map(|a| {
                        a.iter()
                            .filter_map(|x| x.as_str().map(|s| s.to_string()))
                            .collect::<Vec<_>>()
                    })
                    .unwrap_or_default();
                let jnts = ch
                    .get("joints")
                    .and_then(|v| v.as_array())
                    .map(|a| {
                        a.iter()
                            .filter_map(|x| x.as_str().map(|s| s.to_string()))
                            .collect::<Vec<_>>()
                    })
                    .unwrap_or_default();
                if !bodies.is_empty() {
                    ee_chains.push(bodies);
                    ee_joint_chains.push(jnts);
                }
            }
        }
        if ee_chains.is_empty() {
            for ee in &bundle.manifest.end_effectors {
                let tip = ee.body.clone().unwrap_or_else(|| ee.name.clone());
                let (bodies, jnts) = walk_body_chain(&bodies, &joints, &tip);
                ee_chains.push(bodies);
                ee_joint_chains.push(jnts);
            }
        }
        let inferred_base = infer_base(&joints, bundle.manifest.expected_base_type);
        let nq = inspect["nq"].as_i64().unwrap_or(0) as i32;
        let nv = inspect["nv"].as_i64().unwrap_or(0) as i32;
        let nu = inspect["nu"].as_i64().unwrap_or(0) as i32;
        let model_hash = stable_model_hash(bundle, nq, nv, nu, &joints, &actuators, &bodies);

        Self {
            robot_id: bundle.manifest.robot_id.clone(),
            nq,
            nv,
            nu,
            nbody: inspect["nbody"].as_i64().unwrap_or(0) as i32,
            njoint: inspect["njnt"].as_i64().unwrap_or(joints.len() as i64) as i32,
            nactuator: nu,
            nsensor: inspect["nsensor"].as_i64().unwrap_or(0) as i32,
            ncamera: inspect["ncam"].as_i64().unwrap_or(0) as i32,
            timestep: inspect["timestep"].as_f64().unwrap_or(0.002),
            joints,
            actuators,
            sensors,
            cameras,
            bodies,
            sites,
            site_records,
            derived: DerivedInterface {
                base_type: inferred_base,
                potentially_uncontrollable_joints: passive.clone(),
                actuated_dofs: actuated,
                passive_dofs: passive,
                end_effector_chains: ee_chains,
                end_effector_joint_chains: ee_joint_chains,
                actuator_coverage: coverage,
            },
            model_hash,
            source_hash: bundle.source_hash.clone(),
            mujoco_version: inspect["mujoco_version"].as_str().unwrap_or("").into(),
            source_format: match bundle.format.format {
                ModelFormat::Mjcf => "mjcf".into(),
                ModelFormat::Urdf => "urdf".into(),
                ModelFormat::Usd => "usd".into(),
                other => format!("{other:?}").to_ascii_lowercase(),
            },
            lost_features: bundle.format.lost_or_unreliable.clone(),
            support_bodies: bundle
                .manifest
                .feet
                .iter()
                .map(|f| f.body.clone().unwrap_or_else(|| f.name.clone()))
                .collect(),
            collision_groups: bundle.manifest.collision_groups.clone(),
            metal: false,
            evidence_status: crate::honesty::SIMULATION_ONLY.into(),
        }
    }

    pub fn tau_max(&self) -> Vec<f64> {
        self.actuators
            .iter()
            .map(|a| a.ctrlrange[0].abs().max(a.ctrlrange[1].abs()).max(1e-6))
            .collect()
    }

    pub fn q_min(&self) -> Vec<f64> {
        let mut out = vec![-1e6; self.nq.max(0) as usize];
        for j in &self.joints {
            if matches!(j.joint_type.as_str(), "free" | "ball") || j.qpos_dim != 1 {
                continue;
            }
            let adr = j.qpos_address as usize;
            if adr < out.len() {
                out[adr] = if j.limited { j.range[0] } else { -1e6 };
            }
        }
        out
    }

    pub fn q_max(&self) -> Vec<f64> {
        let mut out = vec![1e6; self.nq.max(0) as usize];
        for j in &self.joints {
            if matches!(j.joint_type.as_str(), "free" | "ball") || j.qpos_dim != 1 {
                continue;
            }
            let adr = j.qpos_address as usize;
            if adr < out.len() {
                out[adr] = if j.limited { j.range[1] } else { 1e6 };
            }
        }
        out
    }

    pub fn end_effector_name(&self) -> Option<String> {
        self.sites.first().map(|s| s.0.clone()).or_else(|| {
            self.derived
                .end_effector_chains
                .first()
                .and_then(|c| c.first().cloned())
        })
    }
}

fn infer_base(joints: &[JointRecord], expected: BaseType) -> BaseType {
    if joints.iter().any(|j| j.joint_type == "free") {
        BaseType::Floating
    } else {
        expected
    }
}

fn joint_dims(jtype: &str) -> (i32, i32) {
    match jtype {
        "free" => (7, 6),
        "ball" => (4, 3),
        _ => (1, 1),
    }
}

fn walk_body_chain(
    bodies: &[BodyRecord],
    joints: &[JointRecord],
    tip: &str,
) -> (Vec<String>, Vec<String>) {
    let mut body_chain = Vec::new();
    let mut cur = Some(tip.to_string());
    let mut guard = 0;
    while let Some(name) = cur {
        if name.is_empty() || name == "world" || guard > 64 {
            break;
        }
        body_chain.push(name.clone());
        cur = bodies
            .iter()
            .find(|b| b.name == name)
            .map(|b| b.parent.clone());
        guard += 1;
    }
    body_chain.reverse();
    let joint_chain = body_chain
        .iter()
        .filter_map(|b| {
            joints
                .iter()
                .find(|j| j.child_body == *b)
                .map(|j| j.name.clone())
        })
        .collect();
    (body_chain, joint_chain)
}

fn vec3_opt(v: &serde_json::Value) -> Option<[f64; 3]> {
    let arr = v.as_array()?;
    if arr.len() < 3 {
        return None;
    }
    Some([arr[0].as_f64()?, arr[1].as_f64()?, arr[2].as_f64()?])
}

fn vec4_opt(v: &serde_json::Value) -> Option<[f64; 4]> {
    let arr = v.as_array()?;
    if arr.len() < 4 {
        return None;
    }
    Some([
        arr[0].as_f64()?,
        arr[1].as_f64()?,
        arr[2].as_f64()?,
        arr[3].as_f64()?,
    ])
}

fn joint_type_name(code: i64) -> String {
    match code {
        0 => "free".into(),
        1 => "ball".into(),
        2 => "slide".into(),
        3 => "hinge".into(),
        _ => format!("type_{code}"),
    }
}

fn actuator_type_name(gain: i64, bias: i64) -> String {
    // MuJoCo position actuators use affine bias (ctrl is a position setpoint).
    if bias == 1 {
        "position".into()
    } else if gain == 0 {
        "motor".into()
    } else {
        format!("gain_{gain}")
    }
}

impl RobotManifest {
    pub fn actuator_qpos(&self, qpos: &[f64]) -> Vec<f64> {
        self.actuators
            .iter()
            .map(|a| {
                self.joints
                    .iter()
                    .find(|j| j.name == a.transmission_target)
                    .and_then(|j| qpos.get(j.qpos_address as usize).copied())
                    .unwrap_or(0.0)
            })
            .collect()
    }

    pub fn ctrl_ranges(&self) -> Vec<[f64; 2]> {
        self.actuators.iter().map(|a| a.ctrlrange).collect()
    }

    pub fn position_mask(&self) -> Vec<bool> {
        self.actuators
            .iter()
            .map(|a| a.actuator_type == "position")
            .collect()
    }
}

fn stable_model_hash(
    bundle: &RobotBundle,
    nq: i32,
    nv: i32,
    nu: i32,
    joints: &[JointRecord],
    actuators: &[ActuatorRecord],
    bodies: &[BodyRecord],
) -> String {
    let mut h = Sha256::new();
    h.update(b"realityos.normalized_robot/1\0");
    h.update(bundle.source_hash.as_bytes());
    h.update(nq.to_le_bytes());
    h.update(nv.to_le_bytes());
    h.update(nu.to_le_bytes());
    for j in joints {
        h.update(j.name.as_bytes());
        h.update(j.joint_type.as_bytes());
    }
    for a in actuators {
        h.update(a.name.as_bytes());
        h.update(a.transmission_target.as_bytes());
    }
    for b in bodies {
        h.update(b.name.as_bytes());
        h.update(b.mass.to_le_bytes());
    }
    hex::encode(h.finalize())
}
