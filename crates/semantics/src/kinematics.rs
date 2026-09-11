use crate::command::IkTrace;
use crate::embodiment::{EmbodimentModel, Joint, JointKind};
use crate::skill::SkillRefuse;
use crate::transform::{cross3, norm3, normalize3, quat_to_mat, sub3, Se3, TransformError};
use std::collections::HashMap;

const IK_MAX_ITERS: usize = 80;
const IK_DAMP: f64 = 2e-2;
const IK_ACCEPT: f64 = 1e-3;
const IK_MAX_STEP: f64 = 0.45;

#[derive(Debug, Clone)]
pub struct FkState {
    pub ee: Se3,
    pub joint_origins: Vec<[f64; 3]>,
    pub axes_world: Vec<[f64; 3]>,
    pub kinds: Vec<JointKind>,
}

pub fn chain_root_body(model: &EmbodimentModel, chain: &[String]) -> Result<String, SkillRefuse> {
    let first = chain.first().ok_or(SkillRefuse::Unsupported)?;
    let joint = model
        .joints
        .iter()
        .find(|j| j.name == *first)
        .ok_or(SkillRefuse::Unsupported)?;
    Ok(joint.parent_body.clone())
}

/// Reference pose of a body in the world, using only declared body local transforms
/// (no joint motion). Unknown local poses stay unknown.
pub fn world_to_body_ref(model: &EmbodimentModel, body: &str) -> Result<Se3, SkillRefuse> {
    if body.is_empty() || body == "world" {
        return Ok(Se3::identity());
    }
    let mut names = Vec::new();
    let mut cur = Some(body.to_string());
    let mut guard = 0;
    while let Some(name) = cur {
        if name == "world" || guard > model.bodies.len() + 2 {
            break;
        }
        names.push(name.clone());
        cur = model
            .bodies
            .iter()
            .find(|b| b.name == name)
            .and_then(|b| b.parent.clone());
        guard += 1;
    }
    names.reverse();
    let mut t = Se3::identity();
    for name in names {
        t = t.compose(body_local(model, &name)?);
    }
    Ok(t)
}

pub fn require_supported_joint(joint: &Joint) -> Result<(), SkillRefuse> {
    match joint.kind {
        JointKind::Hinge | JointKind::Slide | JointKind::Fixed => Ok(()),
        JointKind::Ball | JointKind::Free | JointKind::Other => {
            Err(SkillRefuse::KinematicsUnsupported)
        }
    }
}

pub fn motion_transform(joint: &Joint, q: f64) -> Result<Se3, SkillRefuse> {
    require_supported_joint(joint)?;
    let axis = joint.axis.value.ok_or(SkillRefuse::Unreachable)?;
    let axis = normalize3(axis).ok_or(SkillRefuse::Unreachable)?;
    let origin = joint.origin_in_child.value.unwrap_or([0.0, 0.0, 0.0]);
    if !origin.iter().all(|v| v.is_finite()) {
        return Err(SkillRefuse::Unreachable);
    }
    match joint.kind {
        JointKind::Fixed => Ok(Se3::identity()),
        JointKind::Hinge => {
            let rot = Se3::from_axis_angle(axis, q).map_err(|_| SkillRefuse::Unreachable)?;
            let t = Se3::translation(origin).map_err(|_| SkillRefuse::Unreachable)?;
            let tinv = Se3::translation([-origin[0], -origin[1], -origin[2]])
                .map_err(|_| SkillRefuse::Unreachable)?;
            Ok(t.compose(rot).compose(tinv))
        }
        JointKind::Slide => {
            let delta = [axis[0] * q, axis[1] * q, axis[2] * q];
            Se3::translation(delta).map_err(|_| SkillRefuse::Unreachable)
        }
        JointKind::Ball | JointKind::Free | JointKind::Other => {
            Err(SkillRefuse::KinematicsUnsupported)
        }
    }
}

fn body_local(model: &EmbodimentModel, name: &str) -> Result<Se3, SkillRefuse> {
    let body = model
        .bodies
        .iter()
        .find(|b| b.name == name)
        .ok_or(SkillRefuse::Unsupported)?;
    body.local_pose.value.ok_or(SkillRefuse::Unsupported)
}

fn last_joint_child(model: &EmbodimentModel, chain: &[String]) -> Result<String, SkillRefuse> {
    let last = chain.last().ok_or(SkillRefuse::Unsupported)?;
    let joint = model
        .joints
        .iter()
        .find(|j| j.name == *last)
        .ok_or(SkillRefuse::Unsupported)?;
    Ok(joint.child_body.clone())
}

fn ee_parent_body(model: &EmbodimentModel, ee: &str) -> Result<String, SkillRefuse> {
    let ee_def = model
        .end_effectors
        .iter()
        .find(|e| e.name == ee)
        .ok_or(SkillRefuse::ModelFeatureUnsupported)?;
    let frame = model
        .frames
        .iter()
        .find(|f| f.name == ee_def.frame)
        .ok_or(SkillRefuse::ModelFeatureUnsupported)?;
    if frame.parent_body.is_empty() {
        return Err(SkillRefuse::ModelFeatureUnsupported);
    }
    Ok(frame.parent_body.clone())
}

/// Compose declared body local poses from `from` (exclusive) down to `to` (inclusive).
fn compose_body_path(model: &EmbodimentModel, from: &str, to: &str) -> Result<Se3, SkillRefuse> {
    if from == to {
        return Ok(Se3::identity());
    }
    let mut names = Vec::new();
    let mut cur = Some(to.to_string());
    let mut reached = false;
    let mut guard = 0;
    while let Some(name) = cur {
        if name == from {
            reached = true;
            break;
        }
        if name == "world" || guard > model.bodies.len() + 2 {
            return Err(SkillRefuse::ModelFeatureUnsupported);
        }
        names.push(name.clone());
        cur = model
            .bodies
            .iter()
            .find(|b| b.name == name)
            .and_then(|b| b.parent.clone());
        guard += 1;
    }
    if !reached && from != "world" {
        return Err(SkillRefuse::ModelFeatureUnsupported);
    }
    names.reverse();
    let mut t = Se3::identity();
    for name in names {
        t = t.compose(body_local(model, &name)?);
    }
    Ok(t)
}

fn ee_pose(model: &EmbodimentModel, ee: &str) -> Result<Se3, SkillRefuse> {
    let ee_def = model
        .end_effectors
        .iter()
        .find(|e| e.name == ee)
        .ok_or(SkillRefuse::ModelFeatureUnsupported)?;
    let frame = model
        .frames
        .iter()
        .find(|f| f.name == ee_def.frame)
        .ok_or(SkillRefuse::ModelFeatureUnsupported)?;
    frame.pose().ok_or(SkillRefuse::ModelFeatureUnsupported)
}

pub fn forward_kinematics(
    model: &EmbodimentModel,
    chain: &[String],
    ee: &str,
    q: &[f64],
) -> Result<FkState, SkillRefuse> {
    if chain.len() != q.len() {
        return Err(SkillRefuse::Unsupported);
    }
    let root = chain_root_body(model, chain)?;
    let mut t = world_to_body_ref(model, &root)?;
    let mut joint_origins = Vec::with_capacity(chain.len());
    let mut axes_world = Vec::with_capacity(chain.len());
    let mut kinds = Vec::with_capacity(chain.len());

    for (name, qi) in chain.iter().zip(q.iter()) {
        let joint = model
            .joints
            .iter()
            .find(|j| j.name == *name)
            .ok_or(SkillRefuse::Unsupported)?;
        require_supported_joint(joint)?;
        if joint.kind == JointKind::Fixed {
            let local = body_local(model, &joint.child_body)?;
            t = t.compose(local);
            continue;
        }
        let local = body_local(model, &joint.child_body)?;
        let before = t.compose(local);
        let axis = joint.axis.value.ok_or(SkillRefuse::Unreachable)?;
        let axis = normalize3(axis).ok_or(SkillRefuse::Unreachable)?;
        let origin_local = joint.origin_in_child.value.unwrap_or([0.0, 0.0, 0.0]);
        joint_origins.push(before.transform_point(origin_local));
        axes_world.push(before.rotate(axis));
        kinds.push(joint.kind);
        let motion = motion_transform(joint, *qi)?;
        t = before.compose(motion);
    }

    let tip = last_joint_child(model, chain)?;
    let parent = ee_parent_body(model, ee)?;
    let to_parent = compose_body_path(model, &tip, &parent)?;
    let ee_local = ee_pose(model, ee)?;
    let ee_world = t.compose(to_parent).compose(ee_local);
    if !ee_world.xyz.iter().all(|v| v.is_finite()) {
        return Err(SkillRefuse::Unreachable);
    }
    Ok(FkState {
        ee: ee_world,
        joint_origins,
        axes_world,
        kinds,
    })
}

pub fn jacobian_translational(fk: &FkState) -> Vec<Vec<f64>> {
    let n = fk.axes_world.len();
    let mut j = vec![vec![0.0; n], vec![0.0; n], vec![0.0; n]];
    for (i, (kind, axis)) in fk.kinds.iter().zip(fk.axes_world.iter()).enumerate() {
        let col = match kind {
            JointKind::Slide => *axis,
            JointKind::Hinge => {
                let r = sub3(fk.ee.xyz, fk.joint_origins[i]);
                cross3(*axis, r)
            }
            JointKind::Fixed => [0.0, 0.0, 0.0],
            JointKind::Ball | JointKind::Free | JointKind::Other => [0.0, 0.0, 0.0],
        };
        j[0][i] = col[0];
        j[1][i] = col[1];
        j[2][i] = col[2];
    }
    j
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

fn in_limits(joint: &Joint, q: f64) -> bool {
    if let Some(min) = joint.q_min.value {
        if q < min - 1e-9 {
            return false;
        }
    }
    if let Some(max) = joint.q_max.value {
        if q > max + 1e-9 {
            return false;
        }
    }
    q.is_finite()
}

fn damped_least_squares(j: &[Vec<f64>], err: &[f64; 3], damp: f64) -> Vec<f64> {
    let n = j[0].len();
    let mut a = vec![vec![0.0; n]; n];
    let mut b = vec![0.0; n];
    for i in 0..n {
        b[i] = j[0][i] * err[0] + j[1][i] * err[1] + j[2][i] * err[2];
        for k in 0..n {
            a[i][k] = j[0][i] * j[0][k] + j[1][i] * j[1][k] + j[2][i] * j[2][k];
        }
        a[i][i] += damp * damp;
    }
    match solve_dense(a, b) {
        Some(dq) => dq,
        None => (0..n)
            .map(|i| j[0][i] * err[0] + j[1][i] * err[1] + j[2][i] * err[2])
            .collect(),
    }
}

#[allow(clippy::needless_range_loop)]
fn solve_dense(mut a: Vec<Vec<f64>>, mut b: Vec<f64>) -> Option<Vec<f64>> {
    let n = b.len();
    if n == 0 {
        return Some(Vec::new());
    }
    for i in 0..n {
        let mut piv = i;
        let mut best = a[i][i].abs();
        for r in (i + 1)..n {
            let v = a[r][i].abs();
            if v > best {
                best = v;
                piv = r;
            }
        }
        if best < 1e-18 {
            return None;
        }
        if piv != i {
            a.swap(i, piv);
            b.swap(i, piv);
        }
        let diag = a[i][i];
        for r in (i + 1)..n {
            let f = a[r][i] / diag;
            for c in i..n {
                a[r][c] -= f * a[i][c];
            }
            b[r] -= f * b[i];
        }
    }
    let mut x = vec![0.0; n];
    for i in (0..n).rev() {
        let mut acc = b[i];
        for c in (i + 1)..n {
            acc -= a[i][c] * x[c];
        }
        if a[i][i].abs() < 1e-18 {
            return None;
        }
        x[i] = acc / a[i][i];
    }
    Some(x)
}

fn solve_from_seed(
    model: &EmbodimentModel,
    chain: &[String],
    joints: &[Joint],
    ee: &str,
    target: [f64; 3],
    mut q: Vec<f64>,
) -> Result<(Vec<f64>, f64), SkillRefuse> {
    for (i, qi) in q.iter_mut().enumerate() {
        *qi = clamp_joint(&joints[i], *qi);
    }
    let mut last_err = f64::INFINITY;
    let mut stall = 0;
    for _ in 0..IK_MAX_ITERS {
        let fk = forward_kinematics(model, chain, ee, &q)?;
        let err = sub3(target, fk.ee.xyz);
        let nerr = norm3(err);
        if nerr < 1e-6 {
            break;
        }
        if nerr > last_err - 1e-9 {
            stall += 1;
            if stall >= 8 {
                break;
            }
        } else {
            stall = 0;
        }
        last_err = nerr;
        let j = jacobian_translational(&fk);
        let mut dq = damped_least_squares(&j, &err, IK_DAMP);
        let step = dq.iter().map(|v| v * v).sum::<f64>().sqrt();
        if step > IK_MAX_STEP {
            let s = IK_MAX_STEP / step;
            for v in dq.iter_mut() {
                *v *= s;
            }
        }
        for (i, qi) in q.iter_mut().enumerate() {
            *qi += dq[i];
            *qi = clamp_joint(&joints[i], *qi);
        }
    }
    let fk = forward_kinematics(model, chain, ee, &q)?;
    if !q.iter().all(|v| v.is_finite()) {
        return Err(SkillRefuse::Unreachable);
    }
    Ok((q, norm3(sub3(target, fk.ee.xyz))))
}

fn extra_seeds(n: usize) -> Vec<Vec<f64>> {
    const PATTERNS: &[&[f64]] = &[
        &[0.45, 0.70, -0.55, 0.35, 0.25, 0.15],
        &[-0.40, 0.85, 0.50, -0.30, 0.20, -0.10],
        &[0.25, -0.75, 0.90, 0.40, -0.35, 0.20],
        &[0.80, 0.80, 0.80, 0.40, 0.40, 0.40],
        &[1.10, -1.00, 1.05, -0.50, 0.45, 0.00],
        &[0.10, 1.40, 1.90, 0.20, 0.10, 0.00],
        &[0.10, -1.40, -1.90, 0.20, 0.10, 0.00],
        &[-0.20, 1.10, 2.10, -0.40, 0.30, 0.10],
    ];
    let mut seeds: Vec<Vec<f64>> = PATTERNS
        .iter()
        .map(|p| (0..n).map(|i| p[i % p.len()]).collect())
        .collect();
    for a in [-1.2, 1.2] {
        for b in [-1.5, 1.5] {
            for c in [-1.8, 1.8] {
                let mut s = vec![0.15; n];
                if n > 0 {
                    s[0] = a;
                }
                if n > 1 {
                    s[1] = b;
                }
                if n > 2 {
                    s[2] = c;
                }
                seeds.push(s);
            }
        }
    }
    seeds
}

fn radical_inverse(mut n: u32, base: u32) -> f64 {
    let mut f = 1.0;
    let mut x = 0.0;
    while n > 0 {
        f /= f64::from(base);
        x += f * f64::from(n % base);
        n /= base;
    }
    x
}

fn limit_space_seeds(joints: &[Joint], count: usize) -> Vec<Vec<f64>> {
    const BASES: [u32; 8] = [2, 3, 5, 7, 11, 13, 17, 19];
    let mut seeds = Vec::with_capacity(count);
    for k in 1..=count {
        let mut s = Vec::with_capacity(joints.len());
        for (i, joint) in joints.iter().enumerate() {
            let lo = joint.q_min.value.unwrap_or(-2.0);
            let hi = joint.q_max.value.unwrap_or(2.0);
            let u = radical_inverse(k as u32, BASES[i % BASES.len()]);
            s.push(lo + (hi - lo) * u);
        }
        seeds.push(s);
    }
    seeds
}

fn distance_from_current(q: &[f64], current: &[f64]) -> f64 {
    q.iter()
        .zip(current.iter())
        .map(|(a, b)| {
            let d = a - b;
            d * d
        })
        .sum::<f64>()
        .sqrt()
}

pub fn solve_ik(
    model: &EmbodimentModel,
    chain: &[String],
    ee: &str,
    target: [f64; 3],
    current_q: &[f64],
) -> Result<(Vec<f64>, IkTrace), SkillRefuse> {
    let joints = resolve_chain_joints(model, chain)?;
    if current_q.len() != joints.len() {
        return Err(SkillRefuse::MissingJointState);
    }
    let mut seeds = vec![current_q.to_vec()];
    seeds.extend(extra_seeds(joints.len()));
    seeds.extend(limit_space_seeds(&joints, 16));

    let mut precise: Option<(Vec<f64>, f64, f64)> = None;
    let mut best_effort: Option<(Vec<f64>, f64, f64)> = None;
    let mut structural = None;
    for seed in seeds {
        let solved = match solve_from_seed(model, chain, &joints, ee, target, seed) {
            Ok(v) => v,
            Err(
                e @ (SkillRefuse::ModelFeatureUnsupported | SkillRefuse::KinematicsUnsupported),
            ) => {
                structural = Some(e);
                continue;
            }
            Err(_) => continue,
        };
        let (q, err) = solved;
        if !q
            .iter()
            .enumerate()
            .all(|(i, qi)| in_limits(&joints[i], *qi))
        {
            continue;
        }
        let dist = distance_from_current(&q, current_q);
        if err <= IK_ACCEPT {
            let better = match &precise {
                None => true,
                Some((_, best_dist, best_err)) => {
                    dist + 1e-6 < *best_dist
                        || ((dist - *best_dist).abs() <= 1e-6 && err < *best_err)
                }
            };
            if better {
                precise = Some((q.clone(), dist, err));
            }
        }
        let better_effort = match &best_effort {
            None => true,
            Some((_, best_dist, best_err)) => {
                err + 1e-9 < *best_err || ((err - *best_err).abs() <= 1e-9 && dist < *best_dist)
            }
        };
        if better_effort {
            best_effort = Some((q, dist, err));
        }
        if precise
            .as_ref()
            .is_some_and(|(_, best_dist, _)| *best_dist < 1e-9)
        {
            break;
        }
    }
    let best = precise.or(best_effort);

    let (q, _dist, residual) = best.ok_or(structural.unwrap_or(SkillRefuse::Unreachable))?;
    let mut max_move: f64 = 0.0;
    let mut sumsq = 0.0;
    for (a, b) in q.iter().zip(current_q.iter()) {
        let d = (a - b).abs();
        max_move = max_move.max(d);
        sumsq += d * d;
    }
    let trace = IkTrace {
        initial_q: current_q.to_vec(),
        target_q: q.clone(),
        joint_delta_norm: sumsq.sqrt(),
        max_joint_move: max_move,
        residual,
    };
    Ok((q, trace))
}

pub fn resolve_chain_joints(
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
        require_supported_joint(joint)?;
        if joint.kind != JointKind::Fixed && joint.axis.value.is_none() {
            return Err(SkillRefuse::Unreachable);
        }
        joints.push(joint.clone());
    }
    Ok(joints)
}

pub fn q_map(names: &[String], q: &[f64]) -> HashMap<String, f64> {
    names
        .iter()
        .zip(q.iter())
        .map(|(n, v)| (n.clone(), *v))
        .collect()
}

pub fn orientation_error(a: Se3, b: Se3) -> f64 {
    let ra = quat_to_mat(a.quat_wxyz);
    let rb = quat_to_mat(b.quat_wxyz);
    let mut rel = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            rel[i][j] = ra[0][i] * rb[0][j] + ra[1][i] * rb[1][j] + ra[2][i] * rb[2][j];
        }
    }
    let trace = rel[0][0] + rel[1][1] + rel[2][2];
    let c = ((trace - 1.0) * 0.5).clamp(-1.0, 1.0);
    c.acos()
}

pub fn se3_from_parts(xyz: [f64; 3], quat: [f64; 4]) -> Result<Se3, TransformError> {
    Se3::try_new(xyz, quat)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapter::synth_planar_two_link;

    #[test]
    fn hinge_jacobian_is_axis_cross_r() {
        let m = synth_planar_two_link();
        let chain = m.ee_joint_chain("ee").unwrap();
        let fk = forward_kinematics(&m, &chain, "ee", &[0.0, 0.0]).unwrap();
        let j = jacobian_translational(&fk);
        let r = sub3(fk.ee.xyz, fk.joint_origins[0]);
        let col = cross3(fk.axes_world[0], r);
        assert!((j[0][0] - col[0]).abs() < 1e-12);
        assert!((j[1][0] - col[1]).abs() < 1e-12);
    }

    #[test]
    fn slide_jacobian_is_axis() {
        let mut m = synth_planar_two_link();
        m.joints[0].kind = JointKind::Slide;
        let chain = m.ee_joint_chain("ee").unwrap();
        let fk = forward_kinematics(&m, &chain, "ee", &[0.05, 0.0]).unwrap();
        let j = jacobian_translational(&fk);
        assert!((j[0][0] - fk.axes_world[0][0]).abs() < 1e-12);
        assert!((j[1][0] - fk.axes_world[0][1]).abs() < 1e-12);
        assert!((j[2][0] - fk.axes_world[0][2]).abs() < 1e-12);
    }

    #[test]
    fn ball_joint_is_unsupported_not_hinge() {
        let mut m = synth_planar_two_link();
        m.joints[0].kind = JointKind::Ball;
        m.joints[0].qpos_dim = 4;
        m.joints[0].dof_dim = 3;
        let chain = m.ee_joint_chain("ee").unwrap();
        let err = forward_kinematics(&m, &chain, "ee", &[0.0, 0.0]).unwrap_err();
        assert_eq!(err, SkillRefuse::KinematicsUnsupported);
    }

    #[test]
    fn inboard_target_from_stretched_seed_converges() {
        let m = synth_planar_two_link();
        let chain = m.ee_joint_chain("ee").unwrap();
        let (q, trace) = solve_ik(&m, &chain, "ee", [0.2, 0.0, 0.0], &[0.0, 0.0]).unwrap();
        assert!(trace.residual < 1e-3, "residual={}", trace.residual);
        let fk = forward_kinematics(&m, &chain, "ee", &q).unwrap();
        assert!((fk.ee.xyz[0] - 0.2).abs() < 1e-3);
        assert!(fk.ee.xyz[1].abs() < 1e-3);
    }

    #[test]
    fn prefers_near_current_seed() {
        let m = synth_planar_two_link();
        let chain = m.ee_joint_chain("ee").unwrap();
        let current = vec![0.2, 0.3];
        let fk = forward_kinematics(&m, &chain, "ee", &current).unwrap();
        let (q, trace) = solve_ik(&m, &chain, "ee", fk.ee.xyz, &current).unwrap();
        assert!(distance_from_current(&q, &current) < 0.15);
        assert!(trace.residual < 1e-4);
        assert_eq!(trace.initial_q, current);
    }

    #[test]
    fn fk_walks_from_last_joint_child_to_declared_ee_body() {
        use crate::embodiment::{Body, EndEffector, FrameKind, ModelFrame};
        use crate::provenance::Provenanced;
        let mut m = synth_planar_two_link();
        m.bodies.push(Body {
            name: "palm".into(),
            parent: Some("link2".into()),
            mass_kg: Provenanced::unknown("test", 0.0),
            com: Provenanced::unknown("test", 0.0),
            inertia: Provenanced::unknown("test", 0.0),
            local_pose: Provenanced::declared(
                Se3::try_new([0.0, 0.0, 0.11], [0.9238795325, 0.0, 0.0, 0.3826834324]).unwrap(),
                "test",
                0.0,
            ),
        });
        m.frames.clear();
        m.frames.push(ModelFrame {
            name: "ee:tool0".into(),
            kind: FrameKind::Ee,
            parent_body: "palm".into(),
            translation: Provenanced::declared([0.0, 0.0, 0.0], "BUNDLE_DECLARED_BODY_FRAME", 0.0),
            rotation: Provenanced::declared(
                [1.0, 0.0, 0.0, 0.0],
                "BUNDLE_DECLARED_BODY_FRAME",
                0.0,
            ),
        });
        m.end_effectors.clear();
        m.end_effectors.push(EndEffector {
            name: "tool0".into(),
            frame: "ee:tool0".into(),
            joint_chain: vec!["j0".into(), "j1".into()],
        });
        let fk = forward_kinematics(&m, &["j0".into(), "j1".into()], "tool0", &[0.0, 0.0]).unwrap();
        assert!(
            (fk.ee.xyz[2] - 0.11).abs() < 1e-9,
            "must compose distal body offset, got z={}",
            fk.ee.xyz[2]
        );
    }
}
