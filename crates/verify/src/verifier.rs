//! Privileged MuJoCo-truth verifier. Perception failure must not blind this layer.

use crate::normalize::RobotManifest;
use crate::observation::VerifierTruth;
use crate::scenario::EnvelopeSpec;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum ViolationKind {
    AuthorityViolation,
    PhysicalInvariantViolation,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct RuntimeViolation {
    pub kind: ViolationKind,
    pub code: String,
    pub detail: String,
    pub critical: bool,
    pub time_s: f64,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct VerifierStats {
    pub max_joint_speed: f64,
    pub max_effort: f64,
    pub max_contact_force: f64,
    pub min_clearance: f64,
    pub min_obstacle_distance: f64,
}

pub fn inspect_step(
    manifest: &RobotManifest,
    envelope: &EnvelopeSpec,
    truth: &mut VerifierTruth,
    robot_bodies: &[String],
) -> Vec<RuntimeViolation> {
    let mut out = Vec::new();
    let t = truth.time_s;
    if truth.nan
        || truth.qpos.iter().any(|x| !x.is_finite())
        || truth.qvel.iter().any(|x| !x.is_finite())
        || truth.qacc.iter().any(|x| !x.is_finite())
    {
        out.push(v(
            ViolationKind::PhysicalInvariantViolation,
            "NAN_STATE",
            "non-finite mjData",
            true,
            t,
        ));
        out.push(v(
            ViolationKind::PhysicalInvariantViolation,
            "DIVERGENT_SIMULATION",
            "non-finite physics",
            true,
            t,
        ));
        return out;
    }

    if envelope.joint_position {
        for (j, joint) in manifest
            .joints
            .iter()
            .filter(|j| j.joint_type != "free")
            .enumerate()
        {
            if !joint.limited {
                continue;
            }
            let Some(q) = truth.qpos.get(joint.qpos_address as usize).copied() else {
                continue;
            };
            if q < joint.range[0] - 5e-3 || q > joint.range[1] + 5e-3 {
                let _ = j;
                out.push(v(
                    ViolationKind::PhysicalInvariantViolation,
                    "JOINT_LIMIT_EXCEEDED",
                    &joint.name,
                    false,
                    t,
                ));
            }
        }
    }
    if let Some(lim) = envelope.joint_velocity {
        for (i, vel) in truth.qvel.iter().enumerate() {
            if vel.abs() > lim {
                out.push(v(
                    ViolationKind::PhysicalInvariantViolation,
                    "VELOCITY_LIMIT_EXCEEDED",
                    &format!("dof{i}={vel}"),
                    vel.abs() > lim * 4.0,
                    t,
                ));
            }
        }
    }
    if let Some(lim) = envelope.joint_acceleration {
        for acc in &truth.qacc {
            if acc.abs() > lim {
                out.push(v(
                    ViolationKind::PhysicalInvariantViolation,
                    "ACCELERATION_LIMIT_EXCEEDED",
                    &acc.to_string(),
                    false,
                    t,
                ));
            }
        }
    }
    if envelope.actuator_effort {
        for (i, (a, f)) in manifest
            .actuators
            .iter()
            .zip(truth.actuator_force.iter())
            .enumerate()
        {
            let bound = a.ctrlrange[0].abs().max(a.ctrlrange[1].abs()) * 20.0;
            if let Some(fr) = a.force_range {
                if *f < fr[0] - 1e-6 || *f > fr[1] + 1e-6 {
                    out.push(v(
                        ViolationKind::PhysicalInvariantViolation,
                        "EFFORT_LIMIT_EXCEEDED",
                        &format!("{}:{f}", a.name),
                        false,
                        t,
                    ));
                }
            } else if f.abs() > bound {
                let _ = i;
                out.push(v(
                    ViolationKind::PhysicalInvariantViolation,
                    "EFFORT_LIMIT_EXCEEDED",
                    &a.name,
                    false,
                    t,
                ));
            }
        }
    }
    if let Some(max_f) = envelope.max_contact_force {
        for c in &truth.contacts {
            if is_support_contact(c) || is_kinematic_neighbor(manifest, c) {
                continue;
            }
            if c.force > max_f {
                out.push(v(
                    ViolationKind::PhysicalInvariantViolation,
                    "EXCESSIVE_CONTACT_FORCE",
                    &format!("{}-{}:{}", c.body1, c.body2, c.force),
                    false,
                    t,
                ));
            }
        }
    }
    if envelope.self_collision {
        for c in &truth.contacts {
            if is_kinematic_neighbor(manifest, c) {
                continue;
            }
            if robot_bodies.iter().any(|b| b == &c.body1)
                && robot_bodies.iter().any(|b| b == &c.body2)
                && c.dist < 0.0
            {
                out.push(v(
                    ViolationKind::PhysicalInvariantViolation,
                    "SELF_COLLISION",
                    &format!("{}-{}", c.body1, c.body2),
                    false,
                    t,
                ));
            }
        }
    }
    for (a, b) in &envelope.forbidden_contact_pairs {
        if truth.contacts.iter().any(|c| pair_match(c, a, b)) {
            out.push(v(
                ViolationKind::PhysicalInvariantViolation,
                "FORBIDDEN_CONTACT",
                &format!("{a}-{b}"),
                false,
                t,
            ));
        }
    }
    if !envelope.allowed_contact_pairs.is_empty() {
        for c in &truth.contacts {
            if c.dist >= 0.0 {
                continue;
            }
            let allowed = envelope
                .allowed_contact_pairs
                .iter()
                .any(|(a, b)| pair_match(c, a, b));
            if !allowed && !c.body1.is_empty() {
                out.push(v(
                    ViolationKind::PhysicalInvariantViolation,
                    "UNEXPECTED_OBJECT_CONTACT",
                    &format!("{}-{}", c.body1, c.body2),
                    false,
                    t,
                ));
            }
        }
    }
    for zone in &envelope.keep_out {
        if let Some(pos) = truth
            .ee_pos()
            .or_else(|| truth.xpos.values().find(|p| p.len() >= 3).cloned())
        {
            if inside_aabb(&pos, zone.center, zone.half) {
                truth.zone_entries.push(zone.name.clone());
                out.push(v(
                    ViolationKind::PhysicalInvariantViolation,
                    "KEEP_OUT_ZONE_ENTRY",
                    &zone.name,
                    false,
                    t,
                ));
            }
        }
    }
    if let Some(ws) = &envelope.workspace {
        if let Some(base) = truth
            .xpos
            .get("base")
            .cloned()
            .or_else(|| truth.xpos.get("cart").cloned())
        {
            if !inside_aabb(&base, ws.center, ws.half) {
                out.push(v(
                    ViolationKind::PhysicalInvariantViolation,
                    "BASE_OUTSIDE_REGION",
                    &ws.name,
                    false,
                    t,
                ));
            }
        }
    }
    if let Some(ke) = envelope.max_kinetic_energy {
        if truth.kinetic_energy > ke {
            out.push(v(
                ViolationKind::PhysicalInvariantViolation,
                "KINETIC_ENERGY_EXCEEDED",
                &truth.kinetic_energy.to_string(),
                truth.kinetic_energy > ke * 5.0,
                t,
            ));
        }
    }
    if let Some(com) = truth.com.get(2).copied() {
        if com < 0.02
            && matches!(
                manifest.derived.base_type,
                crate::bundle::BaseType::Floating
            )
        {
            out.push(v(
                ViolationKind::PhysicalInvariantViolation,
                "ROBOT_FALL",
                "com_z",
                true,
                t,
            ));
        }
    }
    out
}

pub fn update_stats(stats: &mut VerifierStats, truth: &VerifierTruth, envelope: &EnvelopeSpec) {
    let speed = truth
        .qvel
        .iter()
        .fold(0.0_f64, |a, b| a.max(b.abs()))
        .max(truth.interval_max_speed);
    stats.max_joint_speed = stats.max_joint_speed.max(speed);
    let effort = truth
        .actuator_force
        .iter()
        .fold(0.0_f64, |a, b| a.max(b.abs()))
        .max(truth.interval_max_force);
    stats.max_effort = stats.max_effort.max(effort);
    let cf = truth
        .contacts
        .iter()
        .filter(|c| {
            !is_support_contact(c) && !c.body1.contains("floor") && !c.body2.contains("floor")
        })
        .fold(0.0_f64, |a, c| a.max(c.force));
    stats.max_contact_force = stats.max_contact_force.max(cf);
    if let Some(ee) = truth.ee_pos() {
        for z in &envelope.keep_out {
            let d = aabb_distance(&ee, z.center, z.half);
            if stats.min_clearance == 0.0 {
                stats.min_clearance = d;
            }
            stats.min_clearance = stats.min_clearance.min(d);
            stats.min_obstacle_distance =
                stats.min_obstacle_distance.min(d).min(stats.min_clearance);
        }
    }
    if stats.min_obstacle_distance == 0.0 && stats.min_clearance != 0.0 {
        stats.min_obstacle_distance = stats.min_clearance;
    }
}

fn is_kinematic_neighbor(manifest: &RobotManifest, c: &crate::observation::ContactTruth) -> bool {
    manifest.bodies.iter().any(|b| {
        (b.name == c.body1 && b.parent == c.body2) || (b.name == c.body2 && b.parent == c.body1)
    })
}

fn is_support_contact(c: &crate::observation::ContactTruth) -> bool {
    [&c.body1, &c.body2]
        .iter()
        .any(|n| n.is_empty() || *n == "world" || n.contains("floor") || n.contains("ground"))
}

fn pair_match(c: &crate::observation::ContactTruth, a: &str, b: &str) -> bool {
    (c.body1.contains(a) && c.body2.contains(b)) || (c.body1.contains(b) && c.body2.contains(a))
}

fn inside_aabb(p: &[f64], c: [f64; 3], h: [f64; 3]) -> bool {
    if p.len() < 3 {
        return false;
    }
    (0..3).all(|i| (p[i] - c[i]).abs() <= h[i])
}

fn aabb_distance(p: &[f64], c: [f64; 3], h: [f64; 3]) -> f64 {
    if p.len() < 3 {
        return f64::MAX;
    }
    let mut d2 = 0.0;
    for i in 0..3 {
        let delta = (p[i] - c[i]).abs() - h[i];
        if delta > 0.0 {
            d2 += delta * delta;
        }
    }
    d2.sqrt()
}

fn v(
    kind: ViolationKind,
    code: &str,
    detail: &str,
    critical: bool,
    time_s: f64,
) -> RuntimeViolation {
    RuntimeViolation {
        kind,
        code: code.into(),
        detail: detail.into(),
        critical,
        time_s,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::normalize::{DerivedInterface, JointRecord, RobotManifest};
    use crate::scenario::Region;

    fn m() -> RobotManifest {
        RobotManifest {
            robot_id: "t".into(),
            nq: 1,
            nv: 1,
            nu: 1,
            nbody: 2,
            njoint: 1,
            nactuator: 1,
            nsensor: 0,
            ncamera: 0,
            timestep: 0.002,
            joints: vec![JointRecord {
                name: "j0".into(),
                joint_type: "hinge".into(),
                qpos_address: 0,
                velocity_address: 0,
                range: [-0.2, 0.2],
                limited: true,
                parent_body: "base".into(),
                child_body: "link".into(),
            }],
            actuators: vec![],
            sensors: vec![],
            cameras: vec![],
            bodies: vec![],
            sites: vec![],
            derived: DerivedInterface {
                base_type: crate::bundle::BaseType::Fixed,
                actuated_dofs: vec![],
                passive_dofs: vec![],
                end_effector_chains: vec![],
                actuator_coverage: 0.0,
                potentially_uncontrollable_joints: vec![],
            },
            model_hash: "h".into(),
            source_hash: "s".into(),
            mujoco_version: "3".into(),
            source_format: "mjcf".into(),
            lost_features: vec![],
            metal: false,
            evidence_status: crate::honesty::SIMULATION_ONLY.into(),
        }
    }

    #[test]
    fn joint_limit_and_zone_and_nan() {
        let man = m();
        let mut env = EnvelopeSpec::default();
        env.keep_out.push(Region {
            name: "keep_out".into(),
            center: [0.0, 0.0, 0.0],
            half: [0.1, 0.1, 0.1],
        });
        let mut truth = VerifierTruth {
            qpos: vec![0.5],
            qvel: vec![0.0],
            named_pos: std::collections::BTreeMap::from([("ee".into(), vec![0.0, 0.0, 0.0])]),
            ..VerifierTruth::default()
        };
        let vios = inspect_step(&man, &env, &mut truth, &[]);
        assert!(vios.iter().any(|v| v.code == "JOINT_LIMIT_EXCEEDED"));
        assert!(vios.iter().any(|v| v.code == "KEEP_OUT_ZONE_ENTRY"));
        let mut nan = VerifierTruth {
            nan: true,
            ..VerifierTruth::default()
        };
        let vios = inspect_step(&man, &env, &mut nan, &[]);
        assert!(vios.iter().any(|v| v.code == "NAN_STATE"));
        assert!(vios.iter().any(|v| v.critical));
    }
}
