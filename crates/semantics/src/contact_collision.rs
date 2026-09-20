//! Planner-visible collision admissibility for Mode B interpolations.
//!
//! Uses declared world/embodiment geometry only. Privileged simulator
//! future labels are not an input.

use crate::contact::{classify_contact_pair, ContactClassContext, ContactEvidenceClass};
use crate::embodiment::EmbodimentModel;
use crate::kinematics::forward_kinematics;
use crate::maneuver_witness::{
    interpolate_named_q, interpolation_count, ExecutableContactManeuver, PhaseTransition,
    TransitionKind, TransitionVerdict,
};
use crate::transform::{add3, norm3, rotate_by_quat, sub3};

pub const BLOCK_SELF_COLLISION: &str = "SELF_COLLISION";
pub const BLOCK_SUPPORT_COLLISION: &str = "SUPPORT_COLLISION";
pub const BLOCK_OBSTACLE_COLLISION: &str = "OBSTACLE_COLLISION";
pub const BLOCK_UNINTENDED_CONTACT: &str = "UNINTENDED_ROBOT_OBJECT_CONTACT";
pub const BLOCK_WRONG_PHASE_CONTACT: &str = "WRONG_PHASE_OBJECT_CONTACT";

#[derive(Debug, Clone, PartialEq)]
pub struct AttachedSphere {
    pub body: String,
    pub radius: f64,
    pub offset: [f64; 3],
}

#[derive(Debug, Clone, PartialEq)]
pub struct NamedBox {
    pub name: String,
    pub center: [f64; 3],
    pub half_extents: [f64; 3],
}

#[derive(Debug, Clone, PartialEq)]
pub struct CollisionWorld {
    pub object_id: String,
    pub object_center: [f64; 3],
    pub object_half: [f64; 3],
    pub support_id: String,
    pub support_origin: [f64; 3],
    pub support_normal: [f64; 3],
    pub intended_tool_bodies: Vec<String>,
    pub robot_volumes: Vec<AttachedSphere>,
    pub obstacles: Vec<NamedBox>,
    pub ee_radius: f64,
    pub object_probe_radius: f64,
    pub tool_offset_ee: [f64; 3],
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct Sphere {
    center: [f64; 3],
    radius: f64,
}

fn clamp(v: f64, lo: f64, hi: f64) -> f64 {
    v.max(lo).min(hi)
}

fn sphere_aabb(s: Sphere, center: [f64; 3], half: [f64; 3]) -> bool {
    let closest = [
        center[0] + clamp(s.center[0] - center[0], -half[0], half[0]),
        center[1] + clamp(s.center[1] - center[1], -half[1], half[1]),
        center[2] + clamp(s.center[2] - center[2], -half[2], half[2]),
    ];
    norm3(sub3(s.center, closest)) <= s.radius + 1e-9
}

fn sphere_plane(s: Sphere, origin: [f64; 3], normal: [f64; 3]) -> bool {
    let n = crate::transform::normalize3(normal).unwrap_or([0.0, 0.0, 1.0]);
    let h = n[0] * (s.center[0] - origin[0])
        + n[1] * (s.center[1] - origin[1])
        + n[2] * (s.center[2] - origin[2]);
    h < s.radius - 1e-9
}

fn sphere_sphere(a: Sphere, b: Sphere) -> bool {
    norm3(sub3(a.center, b.center)) <= a.radius + b.radius + 1e-9
}

fn block_reason(class: ContactEvidenceClass, wrong_phase: bool) -> &'static str {
    if wrong_phase && class == ContactEvidenceClass::IntendedToolContact {
        return BLOCK_WRONG_PHASE_CONTACT;
    }
    match class {
        ContactEvidenceClass::SelfCollision => BLOCK_SELF_COLLISION,
        ContactEvidenceClass::SupportContact => BLOCK_SUPPORT_COLLISION,
        ContactEvidenceClass::ObstacleContact => BLOCK_OBSTACLE_COLLISION,
        ContactEvidenceClass::UnintendedRobotContact => BLOCK_UNINTENDED_CONTACT,
        ContactEvidenceClass::IntendedToolContact => "",
    }
}

fn intended_ok_for(kind: TransitionKind, sample_i: usize, n: usize) -> bool {
    match kind {
        TransitionKind::ContactToMidStroke | TransitionKind::MidToEndStroke => true,
        // Last sample is the contact configuration. Earlier samples are early strikes.
        TransitionKind::ApproachToContact => n > 0 && sample_i + 1 == n,
        // Start may already sit near the face; retracting to approach is not a strike.
        TransitionKind::CurrentToApproach => true,
    }
}

fn volume_sphere(
    model: &EmbodimentModel,
    ee: &str,
    names: &[String],
    q: &[f64],
    vol: &AttachedSphere,
    tool_fallback: Sphere,
) -> Option<Sphere> {
    let fk = forward_kinematics(model, names, ee, q).ok()?;
    if vol.body.is_empty() || vol.body == ee || vol.body == "ee" || vol.body == "tool" {
        let c = add3(fk.ee.xyz, rotate_by_quat(fk.ee.quat_wxyz, vol.offset));
        return Some(Sphere {
            center: c,
            radius: vol.radius,
        });
    }
    for (i, jn) in names.iter().enumerate() {
        let Some(j) = model.joints.iter().find(|j| j.name == *jn) else {
            continue;
        };
        if j.child_body == vol.body || j.name == vol.body {
            let origin = *fk.joint_origins.get(i)?;
            return Some(Sphere {
                center: add3(origin, vol.offset),
                radius: vol.radius,
            });
        }
    }
    let _ = tool_fallback;
    None
}

fn tool_sphere(
    model: &EmbodimentModel,
    ee: &str,
    names: &[String],
    q: &[f64],
    world: &CollisionWorld,
) -> Option<Sphere> {
    let fk = forward_kinematics(model, names, ee, q).ok()?;
    let tool = add3(
        fk.ee.xyz,
        rotate_by_quat(fk.ee.quat_wxyz, world.tool_offset_ee),
    );
    Some(Sphere {
        center: tool,
        radius: world.ee_radius.max(1e-4),
    })
}

fn classify_named(
    a: &str,
    b: &str,
    world: &CollisionWorld,
    robot_names: &[String],
) -> Option<ContactEvidenceClass> {
    let obstacle_names: Vec<String> = world.obstacles.iter().map(|o| o.name.clone()).collect();
    let intended = if world.intended_tool_bodies.is_empty() {
        vec!["tool".to_string()]
    } else {
        world.intended_tool_bodies.clone()
    };
    let mut robots = robot_names.to_vec();
    if !robots.iter().any(|n| n == "tool") {
        robots.push("tool".into());
    }
    classify_contact_pair(
        a,
        b,
        &ContactClassContext {
            object_id: &world.object_id,
            intended: &intended,
            robot_bodies: &robots,
            support_bodies: std::slice::from_ref(&world.support_id),
            obstacle_bodies: &obstacle_names,
        },
    )
}

/// First forbidden contact class on this interpolated segment, if any.
pub fn forbidden_class_on_interpolation(
    model: &EmbodimentModel,
    ee: &str,
    names: &[String],
    qa: &[f64],
    qb: &[f64],
    world: &CollisionWorld,
    kind: TransitionKind,
) -> Option<(&'static str, ContactEvidenceClass)> {
    let n = interpolation_count(qa, qb);
    let samples = interpolate_named_q(qa, qb, n).ok()?;
    let mut robot_names: Vec<String> = world.robot_volumes.iter().map(|v| v.body.clone()).collect();
    robot_names.push("tool".into());
    for (i, q) in samples.iter().enumerate() {
        let intended_ok = intended_ok_for(kind, i, samples.len());
        let Some(tool) = tool_sphere(model, ee, names, q, world) else {
            continue;
        };
        let object_probe = Sphere {
            center: tool.center,
            radius: world.object_probe_radius.max(1e-4),
        };
        if sphere_aabb(object_probe, world.object_center, world.object_half) {
            if let Some(class) = classify_named("tool", &world.object_id, world, &robot_names) {
                let reason = block_reason(class, !intended_ok);
                if !reason.is_empty() {
                    return Some((reason, class));
                }
            }
        }
        if sphere_plane(tool, world.support_origin, world.support_normal) {
            return Some((
                BLOCK_SUPPORT_COLLISION,
                ContactEvidenceClass::SupportContact,
            ));
        }
        for obs in &world.obstacles {
            if sphere_aabb(tool, obs.center, obs.half_extents) {
                return Some((
                    BLOCK_OBSTACLE_COLLISION,
                    ContactEvidenceClass::ObstacleContact,
                ));
            }
        }
        let mut vol_spheres: Vec<(String, Sphere)> = Vec::new();
        for vol in &world.robot_volumes {
            if let Some(s) = volume_sphere(model, ee, names, q, vol, tool) {
                if s.radius <= 0.0 {
                    continue;
                }
                let name = if vol.body.is_empty() {
                    "tool".into()
                } else {
                    vol.body.clone()
                };
                if sphere_aabb(s, world.object_center, world.object_half) {
                    if let Some(class) =
                        classify_named(&name, &world.object_id, world, &robot_names)
                    {
                        let wrong_phase =
                            class == ContactEvidenceClass::IntendedToolContact && !intended_ok;
                        let reason = block_reason(class, wrong_phase);
                        if !reason.is_empty() {
                            return Some((reason, class));
                        }
                    }
                }
                if sphere_plane(s, world.support_origin, world.support_normal) {
                    return Some((
                        BLOCK_SUPPORT_COLLISION,
                        ContactEvidenceClass::SupportContact,
                    ));
                }
                for obs in &world.obstacles {
                    if sphere_aabb(s, obs.center, obs.half_extents) {
                        return Some((
                            BLOCK_OBSTACLE_COLLISION,
                            ContactEvidenceClass::ObstacleContact,
                        ));
                    }
                }
                vol_spheres.push((name, s));
            }
        }
        for i in 0..vol_spheres.len() {
            for j in (i + 1)..vol_spheres.len() {
                if sphere_sphere(vol_spheres[i].1, vol_spheres[j].1) {
                    if let Some(class) =
                        classify_named(&vol_spheres[i].0, &vol_spheres[j].0, world, &robot_names)
                    {
                        if class == ContactEvidenceClass::SelfCollision {
                            return Some((BLOCK_SELF_COLLISION, class));
                        }
                    }
                }
            }
        }
    }
    None
}

pub fn is_collision_block_reason(reason: &str) -> bool {
    matches!(
        reason,
        BLOCK_SELF_COLLISION
            | BLOCK_SUPPORT_COLLISION
            | BLOCK_OBSTACLE_COLLISION
            | BLOCK_UNINTENDED_CONTACT
            | BLOCK_WRONG_PHASE_CONTACT
    )
}

fn refuse_if_needed(t: &mut PhaseTransition, hit: Option<(&'static str, ContactEvidenceClass)>) {
    if t.verdict != TransitionVerdict::Feasible {
        return;
    }
    if let Some((reason, _)) = hit {
        *t = PhaseTransition::refused(t.kind, reason, t.n_samples);
    }
}

/// Joint-limit assessment stays unchanged. Collision is a later, distinct layer.
pub fn apply_collision_admissibility(
    w: &mut ExecutableContactManeuver,
    model: &EmbodimentModel,
    ee: &str,
    world: &CollisionWorld,
) {
    let names = &w.joint_names;
    let hits = [
        (
            TransitionKind::CurrentToApproach,
            forbidden_class_on_interpolation(
                model,
                ee,
                names,
                &w.start_q,
                &w.approach.q,
                world,
                TransitionKind::CurrentToApproach,
            ),
        ),
        (
            TransitionKind::ApproachToContact,
            forbidden_class_on_interpolation(
                model,
                ee,
                names,
                &w.approach.q,
                &w.contact.q,
                world,
                TransitionKind::ApproachToContact,
            ),
        ),
        (
            TransitionKind::ContactToMidStroke,
            forbidden_class_on_interpolation(
                model,
                ee,
                names,
                &w.contact.q,
                &w.mid_stroke.q,
                world,
                TransitionKind::ContactToMidStroke,
            ),
        ),
        (
            TransitionKind::MidToEndStroke,
            forbidden_class_on_interpolation(
                model,
                ee,
                names,
                &w.mid_stroke.q,
                &w.end_stroke.q,
                world,
                TransitionKind::MidToEndStroke,
            ),
        ),
    ];
    refuse_if_needed(&mut w.current_to_approach, hits[0].1);
    refuse_if_needed(&mut w.approach_to_contact, hits[1].1);
    refuse_if_needed(&mut w.contact_to_mid, hits[2].1);
    refuse_if_needed(&mut w.mid_to_end, hits[3].1);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapter::synth_planar_two_link;
    use crate::maneuver_witness::{witness_from_continuing_phases, ManeuverPhase};
    use crate::transform::Se3;

    fn named_phase(q: Vec<f64>, seed: Vec<f64>) -> ManeuverPhase {
        ManeuverPhase::positional(Se3::identity(), q, seed, 1e-4, 0.0)
    }

    fn world_at(object_center: [f64; 3]) -> CollisionWorld {
        CollisionWorld {
            object_id: "object".into(),
            object_center,
            object_half: [0.03, 0.03, 0.03],
            support_id: "table".into(),
            support_origin: [object_center[0], object_center[1], -0.05],
            support_normal: [0.0, 0.0, 1.0],
            intended_tool_bodies: vec!["tool".into()],
            robot_volumes: vec![],
            obstacles: vec![],
            ee_radius: 0.015,
            object_probe_radius: 1e-3,
            tool_offset_ee: [0.0, 0.0, 0.0],
        }
    }

    #[test]
    fn joint_limit_safe_support_collision_is_not_executable() {
        let model = synth_planar_two_link();
        let names = vec!["j0".into(), "j1".into()];
        let mut w = witness_from_continuing_phases(
            names,
            vec![0.0, 0.0],
            named_phase(vec![0.2, 0.2], vec![0.0, 0.0]),
            named_phase(vec![0.2, 0.2], vec![0.2, 0.2]),
            named_phase(vec![0.2, 0.2], vec![0.2, 0.2]),
            named_phase(vec![0.2, 0.2], vec![0.2, 0.2]),
            &model.joints,
        );
        assert!(w.is_executable(), "precondition: joint-limit safe");
        let mut world = world_at([0.40, 0.0, 0.0]);
        world.support_origin = [0.0, 0.0, 0.20];
        world.support_normal = [0.0, 0.0, 1.0];
        apply_collision_admissibility(&mut w, &model, "ee", &world);
        assert!(!w.is_executable());
        assert_eq!(
            crate::maneuver_witness::execution_block_reason(&w).as_deref(),
            Some(BLOCK_SUPPORT_COLLISION)
        );
    }

    #[test]
    fn obstacle_on_interpolation_is_not_executable() {
        let model = synth_planar_two_link();
        let names = vec!["j0".into(), "j1".into()];
        let mut w = witness_from_continuing_phases(
            names,
            vec![0.0, 0.0],
            named_phase(vec![0.9, 0.2], vec![0.0, 0.0]),
            named_phase(vec![0.9, 0.2], vec![0.9, 0.2]),
            named_phase(vec![0.9, 0.2], vec![0.9, 0.2]),
            named_phase(vec![0.9, 0.2], vec![0.9, 0.2]),
            &model.joints,
        );
        assert!(w.is_executable());
        let mut world = world_at([2.0, 0.0, 0.0]);
        world.obstacles.push(NamedBox {
            name: "obstacle".into(),
            center: [0.30, 0.0, 0.0],
            half_extents: [0.05, 0.05, 0.05],
        });
        apply_collision_admissibility(&mut w, &model, "ee", &world);
        assert!(!w.is_executable());
        assert_eq!(
            crate::maneuver_witness::execution_block_reason(&w).as_deref(),
            Some(BLOCK_OBSTACLE_COLLISION)
        );
    }

    #[test]
    fn overlapping_robot_volumes_are_self_collision() {
        let model = synth_planar_two_link();
        let names = vec!["j0".into(), "j1".into()];
        let q = vec![0.0, 0.0];
        let mut w = witness_from_continuing_phases(
            names,
            q.clone(),
            named_phase(q.clone(), q.clone()),
            named_phase(q.clone(), q.clone()),
            named_phase(q.clone(), q.clone()),
            named_phase(q.clone(), q.clone()),
            &model.joints,
        );
        let mut world = world_at([2.0, 0.0, 0.0]);
        world.support_origin = [0.0, 0.0, -1.0];
        world.robot_volumes.push(AttachedSphere {
            body: "link1".into(),
            radius: 0.12,
            offset: [0.0, 0.0, 0.0],
        });
        world.robot_volumes.push(AttachedSphere {
            body: "link2".into(),
            radius: 0.12,
            offset: [0.0, 0.0, 0.0],
        });
        apply_collision_admissibility(&mut w, &model, "ee", &world);
        assert!(!w.is_executable());
        assert_eq!(
            crate::maneuver_witness::execution_block_reason(&w).as_deref(),
            Some(BLOCK_SELF_COLLISION)
        );
    }

    #[test]
    fn forearm_volume_hitting_object_is_unintended() {
        let model = synth_planar_two_link();
        let names = vec!["j0".into(), "j1".into()];
        let q = vec![0.4, 0.4];
        let mut w = witness_from_continuing_phases(
            names,
            q.clone(),
            named_phase(q.clone(), q.clone()),
            named_phase(q.clone(), q.clone()),
            named_phase(q.clone(), q.clone()),
            named_phase(q.clone(), q.clone()),
            &model.joints,
        );
        assert!(w.is_executable());
        let mut world = world_at([0.0, 0.0, 0.0]);
        world.support_origin = [0.0, 0.0, -1.0];
        world.object_half = [0.02, 0.02, 0.02];
        world.ee_radius = 0.001;
        world.object_probe_radius = 1e-3;
        world.robot_volumes.push(AttachedSphere {
            body: "link1".into(),
            radius: 0.20,
            offset: [0.0, 0.0, 0.0],
        });
        apply_collision_admissibility(&mut w, &model, "ee", &world);
        assert!(!w.is_executable());
        let reason = crate::maneuver_witness::execution_block_reason(&w).unwrap();
        assert!(
            reason == BLOCK_UNINTENDED_CONTACT || reason == BLOCK_WRONG_PHASE_CONTACT,
            "got {reason}"
        );
    }

    #[test]
    fn approach_samples_inside_object_are_wrong_phase() {
        let model = synth_planar_two_link();
        let names = vec!["j0".into(), "j1".into()];
        let q_approach = vec![0.0, 0.0];
        let q_contact = vec![0.0, 0.0];
        let mut w = witness_from_continuing_phases(
            names,
            q_approach.clone(),
            named_phase(q_approach.clone(), q_approach.clone()),
            named_phase(q_contact.clone(), q_approach.clone()),
            named_phase(q_contact.clone(), q_contact.clone()),
            named_phase(q_contact.clone(), q_contact.clone()),
            &model.joints,
        );
        let fk =
            forward_kinematics(&model, &["j0".into(), "j1".into()], "ee", &[0.0, 0.0]).unwrap();
        let mut world = world_at(fk.ee.xyz);
        world.object_probe_radius = 0.05;
        world.object_half = [0.04, 0.04, 0.04];
        apply_collision_admissibility(&mut w, &model, "ee", &world);
        let reason = crate::maneuver_witness::execution_block_reason(&w);
        assert_eq!(reason.as_deref(), Some(BLOCK_WRONG_PHASE_CONTACT));
    }

    #[test]
    fn intended_tool_object_at_contact_is_not_a_block() {
        let model = synth_planar_two_link();
        let names = vec!["j0".into(), "j1".into()];
        let q = vec![0.3, 0.4];
        let mut w = witness_from_continuing_phases(
            names,
            q.clone(),
            named_phase(q.clone(), q.clone()),
            named_phase(q.clone(), q.clone()),
            named_phase(q.clone(), q.clone()),
            named_phase(q.clone(), q.clone()),
            &model.joints,
        );
        let fk = forward_kinematics(&model, &["j0".into(), "j1".into()], "ee", &q).unwrap();
        let mut world = world_at(fk.ee.xyz);
        world.ee_radius = 0.02;
        world.object_half = [0.01, 0.01, 0.01];
        apply_collision_admissibility(&mut w, &model, "ee", &world);
        assert!(
            w.is_executable()
                || crate::maneuver_witness::execution_block_reason(&w).as_deref()
                    != Some(BLOCK_UNINTENDED_CONTACT)
        );
    }
}
