use crate::capability::{CapName, CapStatus, CapabilityGraph};
use crate::embodiment::{EmbodimentModel, Joint};
use crate::observation::ObservationFrame;
use crate::skill::{SkillContract, SkillName, SkillRefuse};
use crate::world::WorldState;

const IK_MAX_ITERS: usize = 40;
const IK_DAMP: f64 = 1e-3;

#[derive(Debug, Clone, PartialEq)]
pub struct CompiledCtrl {
    pub action: Vec<f64>,
    pub control_mode: String,
    pub adapter_id: String,
    pub adapter_version: String,
}

pub struct AdapterContract {
    pub input: &'static str,
    pub output_mode: &'static str,
    pub required_caps: &'static [CapName],
    pub stop: &'static str,
}

pub trait ControlAdapter {
    fn advertise(&self) -> AdapterContract;
    fn compile(
        &self,
        skill: &SkillContract,
        model: &EmbodimentModel,
        caps: &CapabilityGraph,
        world: &WorldState,
        obs: &ObservationFrame,
    ) -> Result<CompiledCtrl, SkillRefuse>;
}

pub struct ChainIkPositionPdAdapter;

impl ControlAdapter for ChainIkPositionPdAdapter {
    fn advertise(&self) -> AdapterContract {
        AdapterContract {
            input: "ee_xyz",
            output_mode: "position",
            required_caps: &[CapName::JointPositionControl],
            stop: "hold_last_position",
        }
    }

    fn compile(
        &self,
        skill: &SkillContract,
        model: &EmbodimentModel,
        caps: &CapabilityGraph,
        world: &WorldState,
        _obs: &ObservationFrame,
    ) -> Result<CompiledCtrl, SkillRefuse> {
        if skill.name != SkillName::Reach {
            return Err(SkillRefuse::Unsupported);
        }

        if !joint_position_usable(caps) {
            return Err(SkillRefuse::Unsupported);
        }

        let chain = model
            .ee_joint_chain(&world.target_frame)
            .filter(|c| !c.is_empty())
            .ok_or(SkillRefuse::Unsupported)?;

        let target = world
            .target_xyz
            .value
            .filter(|t| t.iter().all(|v| v.is_finite()))
            .ok_or(SkillRefuse::MissingTarget)?;

        let joints = resolve_chain_joints(model, &chain)?;
        let link_offsets = resolve_link_offsets(model, &joints)?;
        let ee_offset = resolve_ee_offset(model, &world.target_frame)?;

        let q = solve_ik(&joints, &link_offsets, &ee_offset, target)?;

        let mut joint_q: std::collections::HashMap<String, f64> =
            std::collections::HashMap::new();
        for (joint, qi) in joints.iter().zip(q.iter()) {
            joint_q.insert(joint.name.clone(), *qi);
        }

        let action = model
            .actuators
            .iter()
            .map(|act| {
                joint_q
                    .get(&act.target_joint)
                    .copied()
                    .filter(|v| v.is_finite())
                    .ok_or(SkillRefuse::MissingActuator)
            })
            .collect::<Result<Vec<f64>, SkillRefuse>>()?;

        Ok(CompiledCtrl {
            action,
            control_mode: "position".into(),
            adapter_id: "chain_ik_position_pd".into(),
            adapter_version: "1".into(),
        })
    }
}

fn joint_position_usable(caps: &CapabilityGraph) -> bool {
    matches!(
        caps.get(CapName::JointPositionControl).status,
        CapStatus::Proven | CapStatus::Supported | CapStatus::PartiallySupported
    )
}

fn resolve_chain_joints(
    model: &EmbodimentModel,
    chain: &[String],
) -> Result<Vec<Joint>, SkillRefuse> {
    let mut joints = Vec::with_capacity(chain.len());
    for name in chain {
        let joint = model
            .joints
            .iter()
            .find(|j| j.name == *name)
            .ok_or(SkillRefuse::Unsupported)?;
        if joint.axis.value.is_none() {
            return Err(SkillRefuse::Unreachable);
        }
        joints.push(joint.clone());
    }
    Ok(joints)
}

fn resolve_link_offsets(
    model: &EmbodimentModel,
    joints: &[Joint],
) -> Result<Vec<[f64; 3]>, SkillRefuse> {
    joints
        .iter()
        .map(|joint| {
            let frame_name = format!("link_{}", joint.name);
            model
                .frames
                .iter()
                .find(|f| f.name == frame_name && f.parent_body == joint.parent_body)
                .and_then(|f| f.translation.value)
                .ok_or(SkillRefuse::Unsupported)
        })
        .collect()
}

fn resolve_ee_offset(model: &EmbodimentModel, ee: &str) -> Result<[f64; 3], SkillRefuse> {
    let ee_def = model
        .end_effectors
        .iter()
        .find(|e| e.name == ee)
        .ok_or(SkillRefuse::Unsupported)?;
    let frame = model
        .frames
        .iter()
        .find(|f| f.name == ee_def.frame)
        .ok_or(SkillRefuse::Unsupported)?;
    frame.translation.value.ok_or(SkillRefuse::Unsupported)
}

fn ik_seed_candidates(joints: &[Joint]) -> Vec<Vec<f64>> {
    let n = joints.len();
    let mut primary = vec![0.0; n];
    for (i, joint) in joints.iter().enumerate().take(n) {
        primary[i] = match (joint.q_min.value, joint.q_max.value) {
            (Some(min), Some(max)) if min.is_finite() && max.is_finite() => (min + max) * 0.5,
            _ => {
                if i == 0 {
                    0.0
                } else {
                    0.4
                }
            }
        };
    }

    let mut seeds = vec![primary];
    for k in 1..=2u32 {
        let mut s = vec![0.0; n];
        for s_i in s.iter_mut().skip(1) {
            *s_i = 0.4 * f64::from(k);
        }
        seeds.push(s);
    }
    seeds
}

fn solve_ik_from_seed(
    joints: &[Joint],
    link_offsets: &[[f64; 3]],
    ee_offset: &[f64; 3],
    target: [f64; 3],
    mut q: Vec<f64>,
) -> Result<(Vec<f64>, f64), SkillRefuse> {
    let n = joints.len();

    for _ in 0..IK_MAX_ITERS {
        let fk = forward_kinematics(joints, link_offsets, ee_offset, &q)?;
        if !fk.ee.iter().all(|v| v.is_finite()) {
            return Err(SkillRefuse::Unreachable);
        }

        let err = sub3(target, fk.ee);
        if norm3(err) < 1e-6 {
            break;
        }

        let j = jacobian(&fk, n);
        let dq = damped_least_squares(&j, &err, IK_DAMP);
        for i in 0..n {
            q[i] += dq[i];
            q[i] = clamp_joint(&joints[i], q[i]);
        }
    }

    let fk = forward_kinematics(joints, link_offsets, ee_offset, &q)?;
    if !fk.ee.iter().all(|v| v.is_finite()) || !q.iter().all(|v| v.is_finite()) {
        return Err(SkillRefuse::Unreachable);
    }

    Ok((q, norm3(sub3(target, fk.ee))))
}

fn solve_ik(
    joints: &[Joint],
    link_offsets: &[[f64; 3]],
    ee_offset: &[f64; 3],
    target: [f64; 3],
) -> Result<Vec<f64>, SkillRefuse> {
    let mut best: Option<(Vec<f64>, f64)> = None;

    for seed in ik_seed_candidates(joints) {
        match solve_ik_from_seed(joints, link_offsets, ee_offset, target, seed) {
            Ok((q, err)) if q.iter().all(|v| v.is_finite()) => {
                if best.as_ref().map(|(_, e)| err < *e).unwrap_or(true) {
                    best = Some((q, err));
                }
            }
            _ => {}
        }
    }

    best.map(|(q, _)| q).ok_or(SkillRefuse::Unreachable)
}

struct FkState {
    ee: [f64; 3],
    joint_origins: Vec<[f64; 3]>,
    axes_world: Vec<[f64; 3]>,
}

fn forward_kinematics(
    joints: &[Joint],
    link_offsets: &[[f64; 3]],
    ee_offset: &[f64; 3],
    q: &[f64],
) -> Result<FkState, SkillRefuse> {
    let mut pos = [0.0, 0.0, 0.0];
    let mut rot = [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ];
    let mut joint_origins = Vec::with_capacity(joints.len());
    let mut axes_world = Vec::with_capacity(joints.len());

    for (joint, offset) in joints.iter().zip(link_offsets.iter()) {
        pos = add3(pos, mat_vec_mul(&rot, *offset));
        joint_origins.push(pos);
        let axis_local = joint.axis.value.ok_or(SkillRefuse::Unreachable)?;
        let axis_w = mat_vec_mul(&rot, axis_local);
        axes_world.push(normalize3(axis_w));
        let qi = q[joint_origins.len() - 1];
        rot = mat_mul(&rot, rotation_matrix(axis_local, qi));
    }

    let ee = add3(pos, mat_vec_mul(&rot, *ee_offset));
    Ok(FkState {
        ee,
        joint_origins,
        axes_world,
    })
}

#[allow(clippy::needless_range_loop)]
fn jacobian(fk: &FkState, n: usize) -> Vec<Vec<f64>> {
    let mut j = vec![vec![0.0; n], vec![0.0; n], vec![0.0; n]];
    for i in 0..n {
        let r = sub3(fk.ee, fk.joint_origins[i]);
        let col = cross3(fk.axes_world[i], r);
        j[0][i] = col[0];
        j[1][i] = col[1];
        j[2][i] = col[2];
    }
    j
}

fn damped_least_squares(j: &[Vec<f64>], err: &[f64; 3], damp: f64) -> Vec<f64> {
    let n = j[0].len();
    let mut jjt = [[0.0; 3]; 3];
    for (r, row) in jjt.iter_mut().enumerate() {
        for c in 0..3 {
            row[c] = (0..n).map(|k| j[r][k] * j[c][k]).sum();
        }
    }
    for (i, row) in jjt.iter_mut().enumerate() {
        row[i] += damp * damp;
    }
    let rhs = [
        dot_row_j(j, err, 0),
        dot_row_j(j, err, 1),
        dot_row_j(j, err, 2),
    ];
    let y = solve3x3(jjt, rhs);
    let mut dq = vec![0.0; n];
    for (k, dq_k) in dq.iter_mut().enumerate().take(n) {
        *dq_k = j[0][k] * y[0] + j[1][k] * y[1] + j[2][k] * y[2];
    }
    dq
}

fn dot_row_j(j: &[Vec<f64>], err: &[f64; 3], row: usize) -> f64 {
    j[row].iter().zip(err.iter()).map(|(a, b)| a * b).sum()
}

fn clamp_joint(joint: &Joint, q: f64) -> f64 {
    let q = match joint.q_min.value {
        Some(min) => q.max(min),
        None => q,
    };
    match joint.q_max.value {
        Some(max) => q.min(max),
        None => q,
    }
}

fn add3(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn sub3(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn norm3(v: [f64; 3]) -> f64 {
    (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
}

fn normalize3(v: [f64; 3]) -> [f64; 3] {
    let n = norm3(v);
    if n > 0.0 {
        [v[0] / n, v[1] / n, v[2] / n]
    } else {
        v
    }
}

fn cross3(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn mat_vec_mul(m: &[[f64; 3]; 3], v: [f64; 3]) -> [f64; 3] {
    [
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    ]
}

fn mat_mul(a: &[[f64; 3]; 3], b: [[f64; 3]; 3]) -> [[f64; 3]; 3] {
    let mut out = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            out[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j];
        }
    }
    out
}

fn rotation_matrix(axis: [f64; 3], angle: f64) -> [[f64; 3]; 3] {
    let a = normalize3(axis);
    let x = a[0];
    let y = a[1];
    let z = a[2];
    let c = angle.cos();
    let s = angle.sin();
    let t = 1.0 - c;
    [
        [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
        [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
        [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
    ]
}

fn solve3x3(a: [[f64; 3]; 3], b: [f64; 3]) -> [f64; 3] {
    let det = a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
        - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
        + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
    if det.abs() < 1e-12 {
        return [0.0, 0.0, 0.0];
    }
    let inv_det = 1.0 / det;
    let inv = [
        [
            (a[1][1] * a[2][2] - a[1][2] * a[2][1]) * inv_det,
            (a[0][2] * a[2][1] - a[0][1] * a[2][2]) * inv_det,
            (a[0][1] * a[1][2] - a[0][2] * a[1][1]) * inv_det,
        ],
        [
            (a[1][2] * a[2][0] - a[1][0] * a[2][2]) * inv_det,
            (a[0][0] * a[2][2] - a[0][2] * a[2][0]) * inv_det,
            (a[0][2] * a[1][0] - a[0][0] * a[1][2]) * inv_det,
        ],
        [
            (a[1][0] * a[2][1] - a[1][1] * a[2][0]) * inv_det,
            (a[0][1] * a[2][0] - a[0][0] * a[2][1]) * inv_det,
            (a[0][0] * a[1][1] - a[0][1] * a[1][0]) * inv_det,
        ],
    ];
    mat_vec_mul(&inv, b)
}

#[cfg(test)]
pub(crate) fn synth_planar_two_link() -> EmbodimentModel {
    use crate::embodiment::{Actuator, Body, EndEffector, FrameKind, Joint, JointKind, ModelFrame};
    use crate::provenance::Provenanced;

    const L1: f64 = 0.15;
    const L2: f64 = 0.15;

    let mut m = EmbodimentModel::new("synth_planar", "test", "hash", "epoch0", "1");
    m.bodies.push(Body {
        name: "base".into(),
        parent: None,
        mass_kg: Provenanced::unknown("test", 0.0),
        com: Provenanced::unknown("test", 0.0),
        inertia: Provenanced::unknown("test", 0.0),
    });
    m.bodies.push(Body {
        name: "link1".into(),
        parent: Some("base".into()),
        mass_kg: Provenanced::unknown("test", 0.0),
        com: Provenanced::unknown("test", 0.0),
        inertia: Provenanced::unknown("test", 0.0),
    });
    m.bodies.push(Body {
        name: "link2".into(),
        parent: Some("link1".into()),
        mass_kg: Provenanced::unknown("test", 0.0),
        com: Provenanced::unknown("test", 0.0),
        inertia: Provenanced::unknown("test", 0.0),
    });

    let hinge_z = Provenanced::declared([0.0, 0.0, 1.0], "test", 0.0);
    m.joints.push(Joint {
        name: "j0".into(),
        kind: JointKind::Hinge,
        axis: hinge_z.clone(),
        qpos_dim: 1,
        dof_dim: 1,
        parent_body: "base".into(),
        child_body: "link1".into(),
        q_min: Provenanced::unknown("test", 0.0),
        q_max: Provenanced::unknown("test", 0.0),
        dq_max: Provenanced::unknown("test", 0.0),
        effort_max: Provenanced::unknown("test", 0.0),
    });
    m.joints.push(Joint {
        name: "j1".into(),
        kind: JointKind::Hinge,
        axis: hinge_z,
        qpos_dim: 1,
        dof_dim: 1,
        parent_body: "link1".into(),
        child_body: "link2".into(),
        q_min: Provenanced::unknown("test", 0.0),
        q_max: Provenanced::unknown("test", 0.0),
        dq_max: Provenanced::unknown("test", 0.0),
        effort_max: Provenanced::unknown("test", 0.0),
    });

    m.actuators.push(Actuator {
        name: "a0".into(),
        target_joint: "j0".into(),
        control_mode: "position".into(),
        ctrlrange: Provenanced::unknown("test", 0.0),
        forcerange: Provenanced::unknown("test", 0.0),
        gear: Provenanced::unknown("test", 0.0),
    });
    m.actuators.push(Actuator {
        name: "a1".into(),
        target_joint: "j1".into(),
        control_mode: "position".into(),
        ctrlrange: Provenanced::unknown("test", 0.0),
        forcerange: Provenanced::unknown("test", 0.0),
        gear: Provenanced::unknown("test", 0.0),
    });

    m.frames.push(ModelFrame {
        name: "link_j0".into(),
        kind: FrameKind::Task,
        parent_body: "base".into(),
        translation: Provenanced::declared([L1, 0.0, 0.0], "test", 0.0),
    });
    m.frames.push(ModelFrame {
        name: "link_j1".into(),
        kind: FrameKind::Task,
        parent_body: "link1".into(),
        translation: Provenanced::declared([L2, 0.0, 0.0], "test", 0.0),
    });
    m.frames.push(ModelFrame {
        name: "ee".into(),
        kind: FrameKind::Ee,
        parent_body: "link2".into(),
        translation: Provenanced::declared([0.0, 0.0, 0.0], "test", 0.0),
    });

    m.end_effectors.push(EndEffector {
        name: "ee".into(),
        frame: "ee".into(),
        joint_chain: vec!["j0".into(), "j1".into()],
    });

    m
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::capability::derive_capabilities;
    use crate::provenance::{Provenance, Provenanced};

    #[test]
    fn adapter_selected_by_caps_not_name() {
        let m = synth_planar_two_link();
        let caps = derive_capabilities(&m, None);
        let world = WorldState::empty("e0", 1.0).with_target(
            "ee",
            [0.2, 0.0, 0.0],
            5.0,
            "e0",
            1.0,
            Provenance::UserDeclared,
        );
        let obs = ObservationFrame {
            frame_id: "f".into(),
            transform_epoch: "e0".into(),
            observations: vec![],
            as_of_s: 1.0,
        };
        let a = ChainIkPositionPdAdapter;
        let out = a
            .compile(&SkillContract::reach(), &m, &caps, &world, &obs)
            .unwrap();
        assert_eq!(out.control_mode, "position");
        assert_eq!(out.action.len(), m.actuators.len());
        assert!(out.action.iter().all(|x| x.is_finite()));
    }

    #[test]
    fn stretched_planar_chain_ik_nonzero_from_bent_seed() {
        let m = synth_planar_two_link();
        let caps = derive_capabilities(&m, None);
        let world = WorldState::empty("e0", 1.0).with_target(
            "ee",
            [0.15, 0.12, 0.0],
            5.0,
            "e0",
            1.0,
            Provenance::UserDeclared,
        );
        let obs = ObservationFrame {
            frame_id: "f".into(),
            transform_epoch: "e0".into(),
            observations: vec![],
            as_of_s: 1.0,
        };
        let out = ChainIkPositionPdAdapter
            .compile(&SkillContract::reach(), &m, &caps, &world, &obs)
            .unwrap();
        assert!(out.action.iter().all(|x| x.is_finite()));
        assert!(
            out.action.iter().any(|x| x.abs() > 1e-6),
            "expected non-zero joint action, got {:?}",
            out.action
        );
    }

    #[test]
    fn unknown_axis_is_unreachable_not_invented() {
        let mut m = synth_planar_two_link();
        m.joints[0].axis = Provenanced::unknown("bundle", 0.0);
        let caps = derive_capabilities(&m, None);
        let world = WorldState::empty("e0", 1.0).with_target(
            "ee",
            [0.2, 0.0, 0.0],
            5.0,
            "e0",
            1.0,
            Provenance::UserDeclared,
        );
        let obs = ObservationFrame {
            frame_id: "f".into(),
            transform_epoch: "e0".into(),
            observations: vec![],
            as_of_s: 1.0,
        };
        let err = ChainIkPositionPdAdapter
            .compile(&SkillContract::reach(), &m, &caps, &world, &obs)
            .unwrap_err();
        assert_eq!(err, SkillRefuse::Unreachable);
    }

    #[test]
    fn unknown_ee_translation_is_refused_not_invented() {
        let mut m = synth_planar_two_link();
        let ee_frame = m
            .frames
            .iter()
            .position(|f| f.name == "ee")
            .expect("ee frame");
        m.frames[ee_frame].translation = Provenanced::unknown("bundle", 0.0);
        let caps = derive_capabilities(&m, None);
        let world = WorldState::empty("e0", 1.0).with_target(
            "ee",
            [0.2, 0.0, 0.0],
            5.0,
            "e0",
            1.0,
            Provenance::UserDeclared,
        );
        let obs = ObservationFrame {
            frame_id: "f".into(),
            transform_epoch: "e0".into(),
            observations: vec![],
            as_of_s: 1.0,
        };
        let err = ChainIkPositionPdAdapter
            .compile(&SkillContract::reach(), &m, &caps, &world, &obs)
            .unwrap_err();
        assert!(matches!(
            err,
            SkillRefuse::Unsupported | SkillRefuse::Unreachable
        ));
    }
}
