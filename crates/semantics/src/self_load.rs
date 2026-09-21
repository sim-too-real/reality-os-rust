//! Embodiment mapping onto physics gravity torque. Missing mass/COM stays UNKNOWN.

use std::collections::BTreeMap;

use realityos_physics::{
    gravity_torque_nm, JointForGravity, JointMotionKind, RigidBodyInertial,
};

use crate::embodiment::{EmbodimentModel, JointKind};
use crate::kinematics::{motion_transform, world_to_body_ref};
use crate::provenance::{Provenance, Provenanced};
use crate::skill::SkillRefuse;
use crate::transform::Se3;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SelfLoadError {
    Unknown(String),
    Unsupported,
}

impl From<SkillRefuse> for SelfLoadError {
    fn from(r: SkillRefuse) -> Self {
        match r {
            SkillRefuse::Unsupported | SkillRefuse::KinematicsUnsupported => Self::Unsupported,
            _ => Self::Unknown(format!("{r:?}")),
        }
    }
}

fn body_local(model: &EmbodimentModel, name: &str) -> Result<Se3, SelfLoadError> {
    let body = model
        .bodies
        .iter()
        .find(|b| b.name == name)
        .ok_or(SelfLoadError::Unsupported)?;
    body.local_pose
        .value
        .ok_or(SelfLoadError::Unknown(format!("PARAMETER_MISSING:body_pose:{name}")))
}

fn joint_moving_child<'a>(model: &'a EmbodimentModel, child: &str) -> Option<&'a crate::embodiment::Joint> {
    model.joints.iter().find(|j| j.child_body == child && j.kind != JointKind::Fixed)
}

fn descendants_of(model: &EmbodimentModel, root: &str) -> Vec<String> {
    let mut out = vec![root.to_string()];
    let mut guard = 0;
    let mut changed = true;
    while changed && guard < model.bodies.len() + 2 {
        changed = false;
        guard += 1;
        for b in &model.bodies {
            if out.iter().any(|n| n == &b.name) {
                continue;
            }
            if b.parent.as_deref().is_some_and(|p| out.iter().any(|n| n == p)) {
                out.push(b.name.clone());
                changed = true;
            }
        }
    }
    out
}

/// World pose of every body. Joints missing from `q_by_joint` are evaluated at q = 0
/// (compiled model zero), which is identity motion, not an invented angle.
pub fn body_world_poses(
    model: &EmbodimentModel,
    q_by_joint: &BTreeMap<String, f64>,
) -> Result<BTreeMap<String, Se3>, SelfLoadError> {
    let mut poses: BTreeMap<String, Se3> = BTreeMap::new();
    let mut remaining: Vec<String> = model.bodies.iter().map(|b| b.name.clone()).collect();
    let mut guard = 0;
    while !remaining.is_empty() && guard < model.bodies.len() + 4 {
        guard += 1;
        let mut progressed = false;
        remaining.retain(|name| {
            let body = match model.bodies.iter().find(|b| b.name == *name) {
                Some(b) => b,
                None => return true,
            };
            let parent_pose = match &body.parent {
                None => Some(Se3::identity()),
                Some(p) if p == "world" || p.is_empty() => Some(Se3::identity()),
                Some(p) => poses.get(p).copied(),
            };
            let Some(parent_pose) = parent_pose else {
                return true;
            };
            let local = match body_local(model, name) {
                Ok(p) => p,
                Err(_) => {
                    return true;
                }
            };
            let pose = if let Some(joint) = joint_moving_child(model, name) {
                let q = q_by_joint.get(&joint.name).copied().unwrap_or(0.0);
                match motion_transform(joint, q) {
                    Ok(motion) => parent_pose.compose(local).compose(motion),
                    Err(_) => return true,
                }
            } else {
                parent_pose.compose(local)
            };
            poses.insert(name.clone(), pose);
            progressed = true;
            false
        });
        if !progressed {
            break;
        }
    }
    if poses.len() != model.bodies.len() {
        // Root-only models still need world_to_body_ref for the first parent.
        for b in &model.bodies {
            if poses.contains_key(&b.name) {
                continue;
            }
            if b.parent.is_none() {
                if let Ok(p) = world_to_body_ref(model, &b.name) {
                    poses.insert(b.name.clone(), p);
                }
            }
        }
    }
    Ok(poses)
}

fn motion_kind(kind: JointKind) -> Option<JointMotionKind> {
    match kind {
        JointKind::Hinge => Some(JointMotionKind::Revolute),
        JointKind::Slide => Some(JointMotionKind::Prismatic),
        JointKind::Fixed => Some(JointMotionKind::Fixed),
        JointKind::Ball | JointKind::Free | JointKind::Other => None,
    }
}

/// Gravity torque at `joint_names` from provenanced body mass and COM.
/// Any required missing inertial is Unknown, not a default.
pub fn gravity_self_load(
    model: &EmbodimentModel,
    joint_names: &[String],
    q_by_joint: &BTreeMap<String, f64>,
    gravity_m_s2: [f64; 3],
) -> Result<Vec<f64>, SelfLoadError> {
    let poses = body_world_poses(model, q_by_joint)?;
    let mut bodies_phys = Vec::new();
    let mut body_index: BTreeMap<String, usize> = BTreeMap::new();
    for b in &model.bodies {
        let Some(&mass) = b.mass_kg.known_value() else {
            if b.mass_kg.provenance == Provenance::Unknown {
                // Unknown mass: only fatal if this body is downstream of a requested joint.
                continue;
            }
            continue;
        };
        if mass == 0.0 {
            continue;
        }
        let Some(&com_local) = b.com.known_value() else {
            continue;
        };
        let Some(pose) = poses.get(&b.name) else {
            return Err(SelfLoadError::Unknown(format!(
                "PARAMETER_MISSING:body_pose:{}",
                b.name
            )));
        };
        body_index.insert(b.name.clone(), bodies_phys.len());
        bodies_phys.push(RigidBodyInertial {
            mass_kg: mass,
            com_world: pose.transform_point(com_local),
        });
    }

    let mut joints_phys = Vec::new();
    for name in joint_names {
        let joint = model
            .joints
            .iter()
            .find(|j| j.name == *name)
            .ok_or(SelfLoadError::Unknown(format!("JOINT_NOT_IN_MODEL:{name}")))?;
        let kind = motion_kind(joint.kind).ok_or(SelfLoadError::Unsupported)?;
        if kind == JointMotionKind::Fixed {
            joints_phys.push(JointForGravity {
                kind,
                origin_world: [0.0, 0.0, 0.0],
                axis_world: [0.0, 0.0, 1.0],
                downstream: Vec::new(),
            });
            continue;
        }
        let parent_pose = match poses.get(&joint.parent_body) {
            Some(p) => *p,
            None => world_to_body_ref(model, &joint.parent_body).map_err(SelfLoadError::from)?,
        };
        let child_local = body_local(model, &joint.child_body)?;
        let before = parent_pose.compose(child_local);
        let axis = joint
            .axis
            .known_value()
            .copied()
            .ok_or_else(|| SelfLoadError::Unknown(format!("PARAMETER_MISSING:axis:{name}")))?;
        let origin_local = joint.origin_in_child.value.unwrap_or([0.0, 0.0, 0.0]);
        let origin_world = before.transform_point(origin_local);
        let axis_world = before.rotate(axis);
        let tree = descendants_of(model, &joint.child_body);
        let mut downstream = Vec::new();
        for desc in &tree {
            let body = model.bodies.iter().find(|b| b.name == *desc);
            let Some(body) = body else {
                continue;
            };
            match (body.mass_kg.known_value().copied(), body.com.known_value()) {
                (None, _) => {
                    if body.mass_kg.provenance == Provenance::Unknown {
                        return Err(SelfLoadError::Unknown(format!(
                            "PARAMETER_MISSING:mass:{desc}"
                        )));
                    }
                    return Err(SelfLoadError::Unknown(format!(
                        "PARAMETER_MISSING:mass:{desc}"
                    )));
                }
                (Some(m), None) if m > 0.0 => {
                    return Err(SelfLoadError::Unknown(format!(
                        "PARAMETER_MISSING:com:{desc}"
                    )));
                }
                (Some(m), Some(_)) if m > 0.0 => {
                    if let Some(&idx) = body_index.get(desc) {
                        downstream.push(idx);
                    } else {
                        return Err(SelfLoadError::Unknown(format!(
                            "PARAMETER_MISSING:com:{desc}"
                        )));
                    }
                }
                _ => {}
            }
        }
        joints_phys.push(JointForGravity {
            kind,
            origin_world,
            axis_world,
            downstream,
        });
    }

    gravity_torque_nm(&joints_phys, &bodies_phys, gravity_m_s2).map_err(|e| {
        SelfLoadError::Unknown(format!("GRAVITY_TORQUE:{e}"))
    })
}

pub fn self_load_provenanced(
    tau: &[f64],
    source: &str,
) -> Vec<Provenanced<f64>> {
    tau.iter()
        .map(|v| Provenanced::declared(*v, source, 0.0))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapter::synth_planar_two_link;
    use crate::embodiment::JointKind;
    use crate::provenance::Provenanced;

    fn vertical_two_link() -> crate::embodiment::EmbodimentModel {
        let mut m = synth_planar_two_link();
        for j in &mut m.joints {
            if j.kind == JointKind::Hinge {
                j.axis = Provenanced::declared([0.0, 1.0, 0.0], "test", 0.0);
            }
        }
        m.bodies[1].mass_kg = Provenanced::declared(2.0, "test", 0.0);
        m.bodies[1].com = Provenanced::declared([0.075, 0.0, 0.0], "test", 0.0);
        m.bodies[2].mass_kg = Provenanced::declared(1.0, "test", 0.0);
        m.bodies[2].com = Provenanced::declared([0.075, 0.0, 0.0], "test", 0.0);
        m
    }

    #[test]
    fn missing_com_is_unknown_not_a_default() {
        let mut m = synth_planar_two_link();
        m.bodies[1].mass_kg = Provenanced::declared(1.0, "test", 0.0);
        // COM stays unknown.
        let mut q = BTreeMap::new();
        q.insert("j0".into(), 0.0);
        q.insert("j1".into(), 0.0);
        let err = gravity_self_load(&m, &["j0".into(), "j1".into()], &q, [0.0, 0.0, -9.80665])
            .unwrap_err();
        match err {
            SelfLoadError::Unknown(r) => assert!(r.contains("com"), "{r}"),
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn z_hinges_with_vertical_g_are_zero_when_inertials_known() {
        let mut m = synth_planar_two_link();
        m.bodies[1].mass_kg = Provenanced::declared(0.35, "test", 0.0);
        m.bodies[1].com = Provenanced::declared([0.075, 0.0, 0.0], "test", 0.0);
        m.bodies[2].mass_kg = Provenanced::declared(0.25, "test", 0.0);
        m.bodies[2].com = Provenanced::declared([0.075, 0.0, 0.0], "test", 0.0);
        let mut q = BTreeMap::new();
        q.insert("j0".into(), 0.4);
        q.insert("j1".into(), -0.3);
        let tau = gravity_self_load(&m, &["j0".into(), "j1".into()], &q, [0.0, 0.0, -9.80665]).unwrap();
        assert!(tau[0].abs() < 1e-12, "{tau:?}");
        assert!(tau[1].abs() < 1e-12, "{tau:?}");
    }

    #[test]
    fn vertical_two_link_matches_analytic_at_q0() {
        let m = vertical_two_link();
        let mut q = BTreeMap::new();
        q.insert("j0".into(), 0.0);
        q.insert("j1".into(), 0.0);
        let tau = gravity_self_load(&m, &["j0".into(), "j1".into()], &q, [0.0, 0.0, -9.80665]).unwrap();
        let g = 9.80665;
        let t0 = -g * (2.0 * 0.075 + 1.0 * (0.15 + 0.075));
        let t1 = -g * 1.0 * 0.075;
        assert!((tau[0] - t0).abs() < 1e-9, "{} vs {}", tau[0], t0);
        assert!((tau[1] - t1).abs() < 1e-9, "{} vs {}", tau[1], t1);
    }

    #[test]
    fn body_pose_uses_declared_local_not_identity_com() {
        let m = vertical_two_link();
        let mut q = BTreeMap::new();
        q.insert("j0".into(), 0.0);
        q.insert("j1".into(), 0.0);
        let poses = body_world_poses(&m, &q).unwrap();
        let p2 = poses.get("link2").copied().unwrap();
        // link2 origin at L1 along x when q=0.
        assert!((p2.xyz[0] - 0.15).abs() < 1e-12, "{:?}", p2.xyz);
        let com2 = p2.transform_point([0.075, 0.0, 0.0]);
        assert!((com2[0] - 0.225).abs() < 1e-12, "{com2:?}");
    }
}
