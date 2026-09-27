//! Planner-visible collision admissibility for Mode B interpolations.
//!
//! Uses declared world/embodiment geometry only. Privileged simulator
//! future labels are not an input.

use crate::allowed_contact::{adjacent_body_pairs, AllowedContactPolicy, ContactPhase};
use crate::contact::{classify_contact_pair, ContactClassContext, ContactEvidenceClass};
use crate::embodiment::EmbodimentModel;
use crate::geometry::{
    ApproximationClass, CollisionRole, CollisionScene, PrimitiveShape, RigidGeometry, SemanticRole,
};
use crate::kinematics::{forward_kinematics, uniform_translational_lipschitz};
use crate::maneuver_witness::{
    interpolate_named_q, interpolation_count, ExecutableContactManeuver, PhaseTransition,
    TransitionKind, TransitionVerdict,
};
use crate::provenance::Provenance;
use crate::transform::{add3, norm3, quat_conj, rotate_by_quat, scale3, sub3, Se3};
use crate::transition_validity::validate_transition;

pub const BLOCK_SELF_COLLISION: &str = "SELF_COLLISION";
pub const BLOCK_SUPPORT_COLLISION: &str = "SUPPORT_COLLISION";
pub const BLOCK_OBSTACLE_COLLISION: &str = "OBSTACLE_COLLISION";
pub const BLOCK_UNINTENDED_CONTACT: &str = "UNINTENDED_ROBOT_OBJECT_CONTACT";
pub const BLOCK_WRONG_PHASE_CONTACT: &str = "WRONG_PHASE_OBJECT_CONTACT";
pub const BLOCK_INVALID_COLLISION_WORLD: &str = "INVALID_COLLISION_WORLD";
pub const BLOCK_COLLISION_PROOF_INCOMPLETE: &str = "COLLISION_PROOF_INCOMPLETE";

#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum CollisionWorldError {
    #[error("invalid object pose in collision world")]
    InvalidObjectPose,
    #[error("invalid pose for obstacle {name}")]
    InvalidObstaclePose { name: String },
    #[error("invalid support plane in collision world")]
    InvalidSupportPlane,
    #[error("joint interpolation is unavailable")]
    InterpolationUnavailable,
    #[error("forward kinematics is unavailable")]
    ForwardKinematicsUnavailable,
}

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
    pub quat_wxyz: [f64; 4],
}

impl NamedBox {
    pub fn aabb(name: impl Into<String>, center: [f64; 3], half_extents: [f64; 3]) -> Self {
        Self {
            name: name.into(),
            center,
            half_extents,
            quat_wxyz: [1.0, 0.0, 0.0, 0.0],
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct CollisionWorld {
    pub object_id: String,
    pub object_center: [f64; 3],
    pub object_half: [f64; 3],
    pub object_quat: [f64; 4],
    pub support_id: String,
    pub support_origin: [f64; 3],
    pub support_normal: [f64; 3],
    pub intended_tool_bodies: Vec<String>,
    pub robot_volumes: Vec<AttachedSphere>,
    pub obstacles: Vec<NamedBox>,
    pub ee_radius: f64,
    pub object_probe_radius: f64,
    pub tool_offset_ee: [f64; 3],
    pub declared_geoms: Vec<RigidGeometry>,
}

fn validate_world_geometry(
    world: &CollisionWorld,
) -> Result<(Se3, [f64; 3], Vec<Se3>), CollisionWorldError> {
    let object_pose = Se3::try_new(world.object_center, world.object_quat)
        .map_err(|_| CollisionWorldError::InvalidObjectPose)?;
    if !world.support_origin.iter().all(|v| v.is_finite()) {
        return Err(CollisionWorldError::InvalidSupportPlane);
    }
    let support_normal = crate::transform::normalize3(world.support_normal)
        .ok_or(CollisionWorldError::InvalidSupportPlane)?;
    let obstacle_poses = world
        .obstacles
        .iter()
        .map(|obstacle| {
            Se3::try_new(obstacle.center, obstacle.quat_wxyz).map_err(|_| {
                CollisionWorldError::InvalidObstaclePose {
                    name: obstacle.name.clone(),
                }
            })
        })
        .collect::<Result<Vec<_>, _>>()?;
    Ok((object_pose, support_normal, obstacle_poses))
}

fn proxy_geom(
    id: impl Into<String>,
    body: impl Into<String>,
    offset: [f64; 3],
    shape: PrimitiveShape,
    role: SemanticRole,
) -> RigidGeometry {
    RigidGeometry {
        id: id.into(),
        owner_body: body.into(),
        local_pose: Se3::translation(offset).unwrap_or_else(|_| Se3::identity()),
        shape,
        collision_role: CollisionRole::Collision,
        semantic_role: role,
        provenance: Provenance::Unknown,
        approximation: ApproximationClass::NonconservativeApproximation,
        source: "proxy".into(),
    }
}

fn ee_parent_and_local(model: &EmbodimentModel, ee: &str) -> Option<(String, Se3)> {
    let ee_def = model.end_effectors.iter().find(|e| e.name == ee)?;
    let frame = model.frames.iter().find(|f| f.name == ee_def.frame)?;
    let local = frame.pose().unwrap_or_else(Se3::identity);
    if frame.parent_body.is_empty() {
        return None;
    }
    Some((frame.parent_body.clone(), local))
}

fn is_kinematic_robot_body(model: &EmbodimentModel, name: &str) -> bool {
    if name.is_empty() || name == "world" {
        return false;
    }
    model
        .joints
        .iter()
        .any(|j| j.parent_body == name || j.child_body == name)
        || model.frames.iter().any(|f| {
            f.parent_body == name
                && matches!(
                    f.kind,
                    crate::embodiment::FrameKind::Ee | crate::embodiment::FrameKind::Tool
                )
        })
}

fn kinematic_root_bodies(model: &EmbodimentModel) -> Vec<String> {
    let children: std::collections::BTreeSet<&str> =
        model.joints.iter().map(|j| j.child_body.as_str()).collect();
    let mut roots = std::collections::BTreeSet::new();
    for j in &model.joints {
        if j.parent_body.is_empty() || j.parent_body == "world" {
            continue;
        }
        if !children.contains(j.parent_body.as_str()) {
            roots.insert(j.parent_body.clone());
        }
    }
    roots.into_iter().collect()
}

/// Planner-visible scene. Declared geoms win; proxy spheres are classified.
pub fn scene_from_collision_world(
    model: &EmbodimentModel,
    ee: &str,
    world: &CollisionWorld,
) -> Result<CollisionScene, CollisionWorldError> {
    let (object_pose, n, obstacle_poses) = validate_world_geometry(world)?;
    let keep_robot = |g: &RigidGeometry| {
        g.participates_in_collision()
            && is_kinematic_robot_body(model, &g.owner_body)
            && g.owner_body != world.object_id
            && g.owner_body != world.support_id
    };
    let mut robot = world
        .declared_geoms
        .iter()
        .filter(|g| keep_robot(g))
        .cloned()
        .collect::<Vec<_>>();
    if robot.is_empty() {
        robot.extend(
            model
                .collision_geoms
                .iter()
                .filter(|g| keep_robot(g))
                .cloned(),
        );
    }
    if robot.is_empty() {
        for vol in &world.robot_volumes {
            if vol.radius <= 0.0 {
                continue;
            }
            let body = if vol.body.is_empty() {
                "tool".to_string()
            } else {
                vol.body.clone()
            };
            robot.push(proxy_geom(
                format!("proxy:{body}"),
                body,
                vol.offset,
                PrimitiveShape::Sphere { radius: vol.radius },
                SemanticRole::RobotLink,
            ));
        }
        if let Some((parent, ee_local)) = ee_parent_and_local(model, ee) {
            let tool_local = ee_local.compose(
                Se3::translation(world.tool_offset_ee).unwrap_or_else(|_| Se3::identity()),
            );
            robot.push(RigidGeometry {
                id: "tool".into(),
                owner_body: parent,
                local_pose: tool_local,
                shape: PrimitiveShape::Sphere {
                    radius: world.ee_radius.max(1e-4),
                },
                collision_role: CollisionRole::Collision,
                semantic_role: SemanticRole::Tool,
                provenance: Provenance::Unknown,
                approximation: ApproximationClass::NonconservativeApproximation,
                source: "proxy_tool".into(),
            });
        }
    }
    let object = vec![RigidGeometry::declared(
        world.object_id.clone(),
        world.object_id.clone(),
        object_pose,
        PrimitiveShape::Box {
            half_extents: world.object_half,
        },
        CollisionRole::Collision,
        SemanticRole::Object,
        "scene.object",
    )];
    let thick = 0.02;
    let support_center = [
        world.support_origin[0] - n[0] * thick,
        world.support_origin[1] - n[1] * thick,
        world.support_origin[2] - n[2] * thick,
    ];
    let extent = world.object_half[0]
        .abs()
        .max(world.object_half[1].abs())
        .max(0.15)
        * 3.0;
    let support_pose =
        Se3::translation(support_center).map_err(|_| CollisionWorldError::InvalidSupportPlane)?;
    let support = vec![RigidGeometry {
        id: world.support_id.clone(),
        owner_body: world.support_id.clone(),
        local_pose: support_pose,
        shape: PrimitiveShape::Box {
            half_extents: [extent, extent, thick],
        },
        collision_role: CollisionRole::Collision,
        semantic_role: SemanticRole::Support,
        provenance: Provenance::UserDeclared,
        approximation: ApproximationClass::ConservativeApproximation,
        source: "scene.support_slab".into(),
    }];
    let obstacles = world
        .obstacles
        .iter()
        .zip(obstacle_poses)
        .map(|(o, pose)| {
            RigidGeometry::declared(
                o.name.clone(),
                o.name.clone(),
                pose,
                PrimitiveShape::Box {
                    half_extents: o.half_extents,
                },
                CollisionRole::Collision,
                SemanticRole::Obstacle,
                "scene.obstacle",
            )
        })
        .collect();
    let mut intended = world.intended_tool_bodies.clone();
    if let Some((parent, _)) = ee_parent_and_local(model, ee) {
        if !intended.iter().any(|n| n == &parent) {
            intended.push(parent);
        }
    }
    Ok(CollisionScene {
        robot,
        object,
        support,
        obstacles,
        intended_tool_bodies: intended,
        object_id: world.object_id.clone(),
        support_id: world.support_id.clone(),
        adjacent_body_pairs: adjacent_body_pairs(model),
    })
}

pub fn policy_from_scene(scene: &CollisionScene) -> AllowedContactPolicy {
    policy_from_scene_model(None, scene)
}

pub fn policy_from_scene_model(
    model: Option<&EmbodimentModel>,
    scene: &CollisionScene,
) -> AllowedContactPolicy {
    let mut robot_bodies: Vec<String> = scene.robot.iter().map(|g| g.owner_body.clone()).collect();
    robot_bodies.extend(scene.intended_tool_bodies.iter().cloned());
    robot_bodies.sort();
    robot_bodies.dedup();
    let mut policy = AllowedContactPolicy::from_names(
        scene.object_id.clone(),
        scene.intended_tool_bodies.clone(),
        robot_bodies,
        vec![scene.support_id.clone()],
        scene
            .obstacles
            .iter()
            .map(|g| g.owner_body.clone())
            .collect(),
        scene.adjacent_body_pairs.clone(),
    );
    if let Some(model) = model {
        policy.fixed_mount_bodies = kinematic_root_bodies(model);
    }
    policy
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

fn sphere_plane(
    s: Sphere,
    origin: [f64; 3],
    normal: [f64; 3],
) -> Result<bool, CollisionWorldError> {
    if !origin.iter().all(|v| v.is_finite()) {
        return Err(CollisionWorldError::InvalidSupportPlane);
    }
    let n = crate::transform::normalize3(normal).ok_or(CollisionWorldError::InvalidSupportPlane)?;
    let h = n[0] * (s.center[0] - origin[0])
        + n[1] * (s.center[1] - origin[1])
        + n[2] * (s.center[2] - origin[2]);
    Ok(h < s.radius - 1e-9)
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
) -> Result<Option<(&'static str, ContactEvidenceClass)>, CollisionWorldError> {
    let (_, support_normal, _) = validate_world_geometry(world)?;
    let n = interpolation_count(qa, qb);
    let samples = interpolate_named_q(qa, qb, n)
        .map_err(|_| CollisionWorldError::InterpolationUnavailable)?;
    let mut robot_names: Vec<String> = world.robot_volumes.iter().map(|v| v.body.clone()).collect();
    robot_names.push("tool".into());
    for (i, q) in samples.iter().enumerate() {
        let intended_ok = intended_ok_for(kind, i, samples.len());
        let tool = tool_sphere(model, ee, names, q, world)
            .ok_or(CollisionWorldError::ForwardKinematicsUnavailable)?;
        let object_probe = Sphere {
            center: tool.center,
            radius: world.object_probe_radius.max(1e-4),
        };
        if sphere_aabb(object_probe, world.object_center, world.object_half) {
            if let Some(class) = classify_named("tool", &world.object_id, world, &robot_names) {
                let reason = block_reason(class, !intended_ok);
                if !reason.is_empty() {
                    return Ok(Some((reason, class)));
                }
            }
        }
        if sphere_plane(tool, world.support_origin, support_normal)? {
            return Ok(Some((
                BLOCK_SUPPORT_COLLISION,
                ContactEvidenceClass::SupportContact,
            )));
        }
        for obs in &world.obstacles {
            if sphere_aabb(tool, obs.center, obs.half_extents) {
                return Ok(Some((
                    BLOCK_OBSTACLE_COLLISION,
                    ContactEvidenceClass::ObstacleContact,
                )));
            }
        }
        let mut vol_spheres: Vec<(String, Sphere)> = Vec::new();
        for vol in &world.robot_volumes {
            let Some(s) = volume_sphere(model, ee, names, q, vol, tool) else {
                return Err(CollisionWorldError::ForwardKinematicsUnavailable);
            };
            if s.radius <= 0.0 {
                continue;
            }
            {
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
                            return Ok(Some((reason, class)));
                        }
                    }
                }
                if sphere_plane(s, world.support_origin, support_normal)? {
                    return Ok(Some((
                        BLOCK_SUPPORT_COLLISION,
                        ContactEvidenceClass::SupportContact,
                    )));
                }
                for obs in &world.obstacles {
                    if sphere_aabb(s, obs.center, obs.half_extents) {
                        return Ok(Some((
                            BLOCK_OBSTACLE_COLLISION,
                            ContactEvidenceClass::ObstacleContact,
                        )));
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
                            return Ok(Some((BLOCK_SELF_COLLISION, class)));
                        }
                    }
                }
            }
        }
    }
    Ok(None)
}

pub fn is_collision_block_reason(reason: &str) -> bool {
    matches!(
        reason,
        BLOCK_SELF_COLLISION
            | BLOCK_SUPPORT_COLLISION
            | BLOCK_OBSTACLE_COLLISION
            | BLOCK_UNINTENDED_CONTACT
            | BLOCK_WRONG_PHASE_CONTACT
            | BLOCK_INVALID_COLLISION_WORLD
            | BLOCK_COLLISION_PROOF_INCOMPLETE
    )
}

/// Uniform-Lipschitz proof that a coupled arm/object stroke stays clear of
/// forbidden contact. Sample counts are not a proof.
#[derive(Debug, Clone, PartialEq)]
pub struct CoupledStrokeProof {
    pub subdivisions: u32,
    pub lipschitz_m_per_unit: f64,
    pub min_forbidden_clearance_m: f64,
    pub stroke_m: f64,
    pub assumptions: Vec<String>,
}

#[derive(Debug, Clone, PartialEq)]
pub enum StickingStrokeAssessment {
    /// Forbidden contact was absent at the named samples only.
    ClearAtSamples {
        sample_count: u32,
    },
    Continuous(CoupledStrokeProof),
    Forbidden {
        reason: &'static str,
    },
    Unknown {
        reason: &'static str,
    },
}

const MAX_COUPLED_DEPTH: u32 = 10;

/// Assess the witness contact, mid-stroke, and end configurations against the
/// object translated along `push_direction`, then attempt a Lipschitz cover of
/// the segments between them.
pub fn prove_coupled_sticking_stroke(
    model: &EmbodimentModel,
    ee: &str,
    names: &[String],
    contact_q: &[f64],
    mid_q: &[f64],
    end_q: &[f64],
    world: &CollisionWorld,
    push_direction: [f64; 3],
    stroke_m: f64,
) -> StickingStrokeAssessment {
    if !stroke_m.is_finite() || stroke_m < 0.0 {
        return StickingStrokeAssessment::Unknown {
            reason: "STROKE_INVALID",
        };
    }
    let Some(push) = crate::transform::normalize3(push_direction) else {
        return StickingStrokeAssessment::Unknown {
            reason: "PUSH_DIRECTION_INVALID",
        };
    };
    if !orientation_is_usable(world.object_quat)
        || world
            .obstacles
            .iter()
            .any(|obstacle| !orientation_is_usable(obstacle.quat_wxyz))
    {
        return StickingStrokeAssessment::Unknown {
            reason: "BOX_ORIENTATION_INVALID",
        };
    }
    let axis_aligned = axis_aligned_quat(world.object_quat)
        && world
            .obstacles
            .iter()
            .all(|obstacle| axis_aligned_quat(obstacle.quat_wxyz));
    let chain = model.ee_joint_chain(ee).unwrap_or_default();
    if chain.is_empty() || names != chain.as_slice() {
        return StickingStrokeAssessment::Unknown {
            reason: "JOINT_CHAIN_MISMATCH",
        };
    }
    let samples = [(0.0, contact_q), (0.5 * stroke_m, mid_q), (stroke_m, end_q)];
    for (distance, q) in samples {
        if q.len() != names.len() {
            return StickingStrokeAssessment::Unknown {
                reason: "Q_LENGTH_MISMATCH",
            };
        }
        let mut shifted = world.clone();
        shifted.object_center = add3(world.object_center, scale3(push, distance));
        match forbidden_class_on_interpolation(
            model,
            ee,
            names,
            q,
            q,
            &shifted,
            TransitionKind::ContactToMidStroke,
        ) {
            Ok(Some((reason, _))) => {
                return StickingStrokeAssessment::Forbidden { reason };
            }
            Ok(None) => {}
            Err(CollisionWorldError::InterpolationUnavailable) => {
                return StickingStrokeAssessment::Unknown {
                    reason: "INTERPOLATION_UNAVAILABLE",
                };
            }
            Err(CollisionWorldError::ForwardKinematicsUnavailable) => {
                return StickingStrokeAssessment::Unknown {
                    reason: "FORWARD_KINEMATICS_UNAVAILABLE",
                };
            }
            Err(_) => {
                return StickingStrokeAssessment::Unknown {
                    reason: "COLLISION_WORLD_INVALID",
                };
            }
        }
    }
    let lipschitz = match uniform_translational_lipschitz(model, &chain, ee) {
        Ok(value) => value,
        Err(_) => {
            return StickingStrokeAssessment::ClearAtSamples { sample_count: 3 };
        }
    };
    let mut assumptions = vec![
        "REVOLUTE_OR_PRISMATIC_LIPSCHITZ_FROM_DECLARED_LINK_LENGTHS".into(),
        if axis_aligned {
            "OBJECT_AND_OBSTACLES_ARE_AXIS_ALIGNED".into()
        } else {
            "ORIENTED_BOX_CLEARANCE_IS_NOT_AN_AABB_PROOF".into()
        },
        "INTENDED_TOOL_OBJECT_CONTACT_IS_ALLOWED_DURING_THE_STROKE".into(),
        "DECLARED_TOOL_AND_ROBOT_SPHERES_ONLY".into(),
    ];
    if world.robot_volumes.is_empty() {
        assumptions.push("NO_ADDITIONAL_ROBOT_VOLUMES_DECLARED".into());
    }
    let mut min_clearance = f64::INFINITY;
    let mut subdivisions = 0u32;
    let segments = [
        (contact_q, mid_q, 0.0, 0.5 * stroke_m),
        (mid_q, end_q, 0.5 * stroke_m, stroke_m),
    ];
    for (qa, qb, s0, s1) in segments {
        match cover_coupled_segment(
            model,
            ee,
            names,
            qa,
            qb,
            world,
            push,
            s0,
            s1,
            lipschitz,
            0,
            &mut subdivisions,
            &mut min_clearance,
        ) {
            Ok(()) => {}
            Err(SegmentGap::Forbidden(reason)) => {
                return StickingStrokeAssessment::Forbidden { reason };
            }
            Err(SegmentGap::Unknown(reason)) => {
                return StickingStrokeAssessment::Unknown { reason };
            }
        }
    }
    if !min_clearance.is_finite() {
        min_clearance = stroke_m;
    }
    if min_clearance <= 1e-9 {
        return StickingStrokeAssessment::Unknown {
            reason: "BOUNDARY_OR_ZERO_CLEARANCE_IS_NOT_A_CLEARANCE_PROOF",
        };
    }
    StickingStrokeAssessment::Continuous(CoupledStrokeProof {
        subdivisions,
        lipschitz_m_per_unit: lipschitz,
        min_forbidden_clearance_m: min_clearance,
        stroke_m,
        assumptions,
    })
}

enum SegmentGap {
    Forbidden(&'static str),
    Unknown(&'static str),
}

fn cover_coupled_segment(
    model: &EmbodimentModel,
    ee: &str,
    names: &[String],
    qa: &[f64],
    qb: &[f64],
    world: &CollisionWorld,
    push: [f64; 3],
    s0: f64,
    s1: f64,
    lipschitz: f64,
    depth: u32,
    subdivisions: &mut u32,
    min_clearance: &mut f64,
) -> Result<(), SegmentGap> {
    let clear0 = forbidden_clearance(model, ee, names, qa, world, push, s0)?;
    let clear1 = forbidden_clearance(model, ee, names, qb, world, push, s1)?;
    *min_clearance = min_clearance.min(clear0).min(clear1);
    let q_step = qa
        .iter()
        .zip(qb.iter())
        .map(|(a, b)| {
            let delta = a - b;
            delta * delta
        })
        .sum::<f64>()
        .sqrt();
    let motion = lipschitz * q_step + (s1 - s0).abs();
    if clear0 > motion + 1e-6 && clear1 > motion + 1e-6 {
        return Ok(());
    }
    if depth >= MAX_COUPLED_DEPTH {
        return Err(SegmentGap::Unknown("SUBDIVISION_BUDGET_EXHAUSTED"));
    }
    *subdivisions = subdivisions.saturating_add(1);
    let qm: Vec<f64> = qa
        .iter()
        .zip(qb.iter())
        .map(|(a, b)| 0.5 * (a + b))
        .collect();
    let sm = 0.5 * (s0 + s1);
    cover_coupled_segment(
        model,
        ee,
        names,
        qa,
        &qm,
        world,
        push,
        s0,
        sm,
        lipschitz,
        depth + 1,
        subdivisions,
        min_clearance,
    )?;
    cover_coupled_segment(
        model,
        ee,
        names,
        &qm,
        qb,
        world,
        push,
        sm,
        s1,
        lipschitz,
        depth + 1,
        subdivisions,
        min_clearance,
    )
}

fn forbidden_clearance(
    model: &EmbodimentModel,
    ee: &str,
    names: &[String],
    q: &[f64],
    world: &CollisionWorld,
    push: [f64; 3],
    distance: f64,
) -> Result<f64, SegmentGap> {
    crate::work_counters::note_collision_query();
    let object_center = add3(world.object_center, scale3(push, distance));
    let spheres = posed_spheres(model, ee, names, q, world)?;
    let support_normal = crate::transform::normalize3(world.support_normal)
        .ok_or(SegmentGap::Unknown("SUPPORT_NORMAL_INVALID"))?;
    let mut min_clearance = f64::INFINITY;
    for (name, sphere, tool) in &spheres {
        if *tool {
            // Intended tool/object overlap is allowed. Other bodies are not.
        } else {
            let clearance =
                sphere_box_clearance(*sphere, object_center, world.object_half, world.object_quat)?;
            if clearance < -1e-9 {
                return Err(SegmentGap::Forbidden(BLOCK_UNINTENDED_CONTACT));
            }
            min_clearance = min_clearance.min(clearance);
        }
        let plane = sphere_plane_clearance(*sphere, world.support_origin, support_normal);
        if plane < -1e-9 {
            return Err(SegmentGap::Forbidden(BLOCK_SUPPORT_COLLISION));
        }
        min_clearance = min_clearance.min(plane);
        for obstacle in &world.obstacles {
            let clearance = sphere_box_clearance(
                *sphere,
                obstacle.center,
                obstacle.half_extents,
                obstacle.quat_wxyz,
            )?;
            if clearance < -1e-9 {
                return Err(SegmentGap::Forbidden(BLOCK_OBSTACLE_COLLISION));
            }
            min_clearance = min_clearance.min(clearance);
        }
        let _ = name;
    }
    for i in 0..spheres.len() {
        for j in (i + 1)..spheres.len() {
            if spheres[i].2 && spheres[j].2 {
                continue;
            }
            let clearance = sphere_sphere_clearance(spheres[i].1, spheres[j].1);
            if clearance < -1e-9 {
                return Err(SegmentGap::Forbidden(BLOCK_SELF_COLLISION));
            }
            min_clearance = min_clearance.min(clearance);
        }
    }
    if !min_clearance.is_finite() {
        // No forbidden body was close enough to produce a finite clearance.
        // The motion bound is then automatically satisfied.
        min_clearance = 1.0e3;
    }
    Ok(min_clearance)
}

fn posed_spheres(
    model: &EmbodimentModel,
    ee: &str,
    names: &[String],
    q: &[f64],
    world: &CollisionWorld,
) -> Result<Vec<(String, Sphere, bool)>, SegmentGap> {
    let tool = tool_sphere(model, ee, names, q, world)
        .ok_or(SegmentGap::Unknown("FORWARD_KINEMATICS_UNAVAILABLE"))?;
    let mut spheres = vec![("tool".into(), tool, true)];
    for volume in &world.robot_volumes {
        let posed = volume_sphere(model, ee, names, q, volume, tool)
            .ok_or(SegmentGap::Unknown("FORWARD_KINEMATICS_UNAVAILABLE"))?;
        if posed.radius <= 0.0 {
            continue;
        }
        let name = if volume.body.is_empty() {
            "tool".into()
        } else {
            volume.body.clone()
        };
        let tool_body = name == "tool" || name == ee;
        spheres.push((name, posed, tool_body));
    }
    Ok(spheres)
}

pub(crate) fn axis_aligned_quat(quat: [f64; 4]) -> bool {
    let Some(unit) = unit_quat(quat) else {
        return false;
    };
    unit[0].abs() > 1.0 - 1e-6
        && unit[1].abs() < 1e-6
        && unit[2].abs() < 1e-6
        && unit[3].abs() < 1e-6
}

fn orientation_is_usable(quat: [f64; 4]) -> bool {
    unit_quat(quat).is_some()
}

fn unit_quat(quat: [f64; 4]) -> Option<[f64; 4]> {
    if !quat.iter().all(|value| value.is_finite()) {
        return None;
    }
    let norm = quat.iter().map(|value| value * value).sum::<f64>().sqrt();
    if !(norm.is_finite() && norm > 1e-9) {
        return None;
    }
    Some([
        quat[0] / norm,
        quat[1] / norm,
        quat[2] / norm,
        quat[3] / norm,
    ])
}

fn sphere_box_clearance(
    sphere: Sphere,
    center: [f64; 3],
    half: [f64; 3],
    quat: [f64; 4],
) -> Result<f64, SegmentGap> {
    if !half.iter().all(|value| value.is_finite() && *value >= 0.0) {
        return Err(SegmentGap::Unknown("BOX_EXTENT_INVALID"));
    }
    let unit = unit_quat(quat).ok_or(SegmentGap::Unknown("BOX_ORIENTATION_INVALID"))?;
    let local = rotate_by_quat(quat_conj(unit), sub3(sphere.center, center));
    Ok(sphere_aabb_clearance(
        Sphere {
            center: local,
            radius: sphere.radius,
        },
        [0.0, 0.0, 0.0],
        half,
    ))
}

/// Occupied-set class of one axis-aligned box sweep. A rotated box is not
/// admitted by renaming the world axis-aligned.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AxisSweepClass {
    Clear { gap_m: f64 },
    BoundaryContact,
    Penetration,
    NotAnAxisSweep,
    RotatedBox,
    Invalid,
}

pub fn classify_axis_aligned_sweep(
    object_center: [f64; 3],
    object_half: [f64; 3],
    object_quat: [f64; 4],
    direction: [f64; 3],
    distance: f64,
    obstacles: &[NamedBox],
) -> AxisSweepClass {
    if !object_center.iter().all(|value| value.is_finite())
        || !object_half
            .iter()
            .all(|value| value.is_finite() && *value >= 0.0)
        || !distance.is_finite()
        || distance < 0.0
        || !direction.iter().all(|value| value.is_finite())
    {
        return AxisSweepClass::Invalid;
    }
    if !axis_aligned_quat(object_quat)
        || obstacles
            .iter()
            .any(|obstacle| !axis_aligned_quat(obstacle.quat_wxyz))
    {
        return AxisSweepClass::RotatedBox;
    }
    if direction.iter().filter(|value| value.abs() > 1e-9).count() != 1 {
        return AxisSweepClass::NotAnAxisSweep;
    }
    let mut swept_center = object_center;
    let mut swept_half = object_half;
    for axis in 0..3 {
        let delta = direction[axis] * distance;
        swept_center[axis] += 0.5 * delta;
        swept_half[axis] += 0.5 * delta.abs();
    }
    let mut gap = f64::INFINITY;
    for obstacle in obstacles {
        if !obstacle
            .half_extents
            .iter()
            .all(|value| value.is_finite() && *value >= 0.0)
            || !obstacle.center.iter().all(|value| value.is_finite())
        {
            return AxisSweepClass::Invalid;
        }
        let separated = (0..3).any(|axis| {
            (swept_center[axis] - obstacle.center[axis]).abs()
                > swept_half[axis] + obstacle.half_extents[axis] + 1e-9
        });
        if !separated {
            let penetration = (0..3)
                .map(|axis| {
                    swept_half[axis] + obstacle.half_extents[axis]
                        - (swept_center[axis] - obstacle.center[axis]).abs()
                })
                .fold(f64::INFINITY, f64::min);
            if penetration > 1e-9 {
                return AxisSweepClass::Penetration;
            }
            return AxisSweepClass::BoundaryContact;
        }
        for axis in 0..3 {
            let separation = (swept_center[axis] - obstacle.center[axis]).abs()
                - swept_half[axis]
                - obstacle.half_extents[axis];
            if separation >= 0.0 {
                gap = gap.min(separation);
            }
        }
    }
    if !gap.is_finite() {
        return AxisSweepClass::Clear {
            gap_m: distance.max(1.0),
        };
    }
    if gap <= 1e-9 {
        AxisSweepClass::BoundaryContact
    } else {
        AxisSweepClass::Clear { gap_m: gap }
    }
}

fn sphere_aabb_clearance(sphere: Sphere, center: [f64; 3], half: [f64; 3]) -> f64 {
    let closest = [
        center[0] + clamp(sphere.center[0] - center[0], -half[0], half[0]),
        center[1] + clamp(sphere.center[1] - center[1], -half[1], half[1]),
        center[2] + clamp(sphere.center[2] - center[2], -half[2], half[2]),
    ];
    norm3(sub3(sphere.center, closest)) - sphere.radius
}

fn sphere_plane_clearance(sphere: Sphere, origin: [f64; 3], normal: [f64; 3]) -> f64 {
    let height = normal[0] * (sphere.center[0] - origin[0])
        + normal[1] * (sphere.center[1] - origin[1])
        + normal[2] * (sphere.center[2] - origin[2]);
    height - sphere.radius
}

fn sphere_sphere_clearance(a: Sphere, b: Sphere) -> f64 {
    norm3(sub3(a.center, b.center)) - a.radius - b.radius
}

fn refuse_if_needed(t: &mut PhaseTransition, hit: Option<(&'static str, ContactEvidenceClass)>) {
    if let Some((reason, _)) = hit {
        refuse_with_reason_if_needed(t, reason);
    }
}

fn refuse_with_reason_if_needed(t: &mut PhaseTransition, reason: &'static str) {
    if t.verdict == TransitionVerdict::Feasible {
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
    let scene = match scene_from_collision_world(model, ee, world) {
        Ok(scene) => scene,
        Err(_) => {
            refuse_with_reason_if_needed(&mut w.current_to_approach, BLOCK_INVALID_COLLISION_WORLD);
            refuse_with_reason_if_needed(&mut w.approach_to_contact, BLOCK_INVALID_COLLISION_WORLD);
            refuse_with_reason_if_needed(&mut w.contact_to_mid, BLOCK_INVALID_COLLISION_WORLD);
            refuse_with_reason_if_needed(&mut w.mid_to_end, BLOCK_INVALID_COLLISION_WORLD);
            return;
        }
    };
    let policy = policy_from_scene_model(Some(model), &scene);
    let segs = [
        (
            TransitionKind::CurrentToApproach,
            w.start_q.as_slice(),
            w.approach.q.as_slice(),
        ),
        (
            TransitionKind::ApproachToContact,
            w.approach.q.as_slice(),
            w.contact.q.as_slice(),
        ),
        (
            TransitionKind::ContactToMidStroke,
            w.contact.q.as_slice(),
            w.mid_stroke.q.as_slice(),
        ),
        (
            TransitionKind::MidToEndStroke,
            w.mid_stroke.q.as_slice(),
            w.end_stroke.q.as_slice(),
        ),
    ];
    let reports = segs.map(|(kind, qa, qb)| {
        let report = validate_transition(
            model,
            names,
            qa,
            qb,
            &scene,
            &policy,
            ContactPhase::from_transition(kind),
        );
        let hit = report.execution_refusal().map(|reason| {
            (
                reason,
                report
                    .class
                    .unwrap_or(ContactEvidenceClass::ObstacleContact),
            )
        });
        (kind, hit)
    });
    refuse_if_needed(&mut w.current_to_approach, reports[0].1);
    refuse_if_needed(&mut w.approach_to_contact, reports[1].1);
    refuse_if_needed(&mut w.contact_to_mid, reports[2].1);
    refuse_if_needed(&mut w.mid_to_end, reports[3].1);
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
            object_quat: [1.0, 0.0, 0.0, 0.0],
            support_id: "table".into(),
            support_origin: [object_center[0], object_center[1], -0.05],
            support_normal: [0.0, 0.0, 1.0],
            intended_tool_bodies: vec!["tool".into()],
            robot_volumes: vec![],
            obstacles: vec![],
            ee_radius: 0.015,
            object_probe_radius: 1e-3,
            tool_offset_ee: [0.0, 0.0, 0.0],
            declared_geoms: vec![],
        }
    }

    fn executable_witness(model: &EmbodimentModel) -> ExecutableContactManeuver {
        let names = vec!["j0".into(), "j1".into()];
        witness_from_continuing_phases(
            names,
            vec![0.0, 0.0],
            named_phase(vec![0.2, 0.2], vec![0.0, 0.0]),
            named_phase(vec![0.2, 0.2], vec![0.2, 0.2]),
            named_phase(vec![0.2, 0.2], vec![0.2, 0.2]),
            named_phase(vec![0.2, 0.2], vec![0.2, 0.2]),
            &model.joints,
        )
    }

    #[test]
    fn invalid_collision_world_refuses_feasible_phases() {
        let model = synth_planar_two_link();
        let mut witness = executable_witness(&model);
        assert!(witness.is_executable(), "precondition: all phases feasible");

        let mut world = world_at([2.0, 0.0, 0.0]);
        world.object_quat = [0.0; 4];
        assert!(matches!(
            scene_from_collision_world(&model, "ee", &world),
            Err(CollisionWorldError::InvalidObjectPose)
        ));
        apply_collision_admissibility(&mut witness, &model, "ee", &world);

        assert!(
            !witness.is_executable(),
            "invalid object pose must refuse a previously feasible witness"
        );
        for transition in [
            &witness.current_to_approach,
            &witness.approach_to_contact,
            &witness.contact_to_mid,
            &witness.mid_to_end,
        ] {
            assert_eq!(transition.verdict, TransitionVerdict::Refused);
            assert_eq!(transition.reason, "INVALID_COLLISION_WORLD");
        }
        assert!(is_collision_block_reason(BLOCK_INVALID_COLLISION_WORLD));

        let mut invalid_support = world_at([2.0, 0.0, 0.0]);
        invalid_support.support_normal = [0.0; 3];
        assert!(matches!(
            scene_from_collision_world(&model, "ee", &invalid_support),
            Err(CollisionWorldError::InvalidSupportPlane)
        ));
        assert!(matches!(
            forbidden_class_on_interpolation(
                &model,
                "ee",
                &["j0".into(), "j1".into()],
                &[0.0, 0.0],
                &[0.2, 0.2],
                &invalid_support,
                TransitionKind::CurrentToApproach,
            ),
            Err(CollisionWorldError::InvalidSupportPlane)
        ));

        let mut invalid_support_origin = world_at([2.0, 0.0, 0.0]);
        invalid_support_origin.support_origin[0] = f64::INFINITY;
        assert!(matches!(
            scene_from_collision_world(&model, "ee", &invalid_support_origin),
            Err(CollisionWorldError::InvalidSupportPlane)
        ));

        let mut preserved = executable_witness(&model);
        preserved.approach_to_contact =
            PhaseTransition::refused(TransitionKind::ApproachToContact, "PREEXISTING_REFUSAL", 7);
        apply_collision_admissibility(&mut preserved, &model, "ee", &world);
        assert_eq!(preserved.approach_to_contact.reason, "PREEXISTING_REFUSAL");
        assert_eq!(
            preserved.current_to_approach.reason,
            BLOCK_INVALID_COLLISION_WORLD
        );
    }

    #[test]
    fn malformed_obstacle_invalidates_scene() {
        let model = synth_planar_two_link();
        let mut world = world_at([2.0, 0.0, 0.0]);
        world.obstacles.push(NamedBox {
            name: "malformed-obstacle".into(),
            center: [f64::NAN, 0.0, 0.0],
            half_extents: [0.05; 3],
            quat_wxyz: [0.0; 4],
        });

        assert!(matches!(
            scene_from_collision_world(&model, "ee", &world),
            Err(CollisionWorldError::InvalidObstaclePose { name })
                if name == "malformed-obstacle"
        ));
        assert!(matches!(
            forbidden_class_on_interpolation(
                &model,
                "ee",
                &["j0".into(), "j1".into()],
                &[0.0, 0.0],
                &[0.2, 0.2],
                &world,
                TransitionKind::CurrentToApproach,
            ),
            Err(CollisionWorldError::InvalidObstaclePose { name })
                if name == "malformed-obstacle"
        ));
        let mut witness = executable_witness(&model);
        apply_collision_admissibility(&mut witness, &model, "ee", &world);
        assert_eq!(
            crate::maneuver_witness::execution_block_reason(&witness).as_deref(),
            Some(BLOCK_INVALID_COLLISION_WORLD)
        );
    }

    #[test]
    fn valid_non_unit_pose_and_support_direction_are_preserved() {
        let model = synth_planar_two_link();
        let mut world = world_at([0.0, 0.0, 0.03]);
        world.object_quat = [2.0, 0.0, 0.0, 0.0];
        world.support_normal = [0.0, 0.0, 2.0];

        let scene = scene_from_collision_world(&model, "ee", &world).unwrap();
        assert_eq!(scene.object[0].local_pose.quat_wxyz, [1.0, 0.0, 0.0, 0.0]);
        assert_eq!(
            scene.support[0].local_pose.xyz,
            [0.0, 0.0, world.support_origin[2] - 0.02]
        );
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
        // Finite support slab whose top face occupies the EE height so the
        // joint-limit-safe interpolation still collides with the table.
        world.support_origin = [0.30, 0.0, 0.0];
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
        world.obstacles.push(NamedBox::aabb(
            "obstacle",
            [0.30, 0.0, 0.0],
            [0.05, 0.05, 0.05],
        ));
        apply_collision_admissibility(&mut w, &model, "ee", &world);
        assert!(!w.is_executable());
        assert_eq!(
            crate::maneuver_witness::execution_block_reason(&w).as_deref(),
            Some(BLOCK_OBSTACLE_COLLISION)
        );
    }

    #[test]
    fn overlapping_robot_volumes_are_self_collision() {
        let mut model = synth_planar_two_link();
        model.bodies.push(crate::embodiment::Body {
            name: "stub".into(),
            parent: Some("base".into()),
            mass_kg: crate::provenance::Provenanced::unknown("test", 0.0),
            com: crate::provenance::Provenanced::unknown("test", 0.0),
            inertia: crate::provenance::Provenanced::unknown("test", 0.0),
            local_pose: crate::provenance::Provenanced::declared(Se3::identity(), "test", 0.0),
        });
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
        // stub and link2 are not parent-child; overlap is forbidden self-collision.
        world.robot_volumes.push(AttachedSphere {
            body: "stub".into(),
            radius: 0.20,
            offset: [0.0, 0.0, 0.0],
        });
        world.robot_volumes.push(AttachedSphere {
            body: "link2".into(),
            radius: 0.20,
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
    fn tool_above_table_is_not_support_collision() {
        let model = synth_planar_two_link();
        let fk =
            forward_kinematics(&model, &["j0".into(), "j1".into()], "ee", &[0.0, 0.0]).unwrap();
        let world = world_at(fk.ee.xyz);
        let scene = scene_from_collision_world(&model, "ee", &world).unwrap();
        let policy = policy_from_scene(&scene);
        let names = vec!["j0".into(), "j1".into()];
        let q = vec![0.0, 0.0];
        let r = crate::transition_validity::validate_transition(
            &model,
            &names,
            &q,
            &q,
            &scene,
            &policy,
            crate::allowed_contact::ContactPhase::CurrentToApproach,
        );
        assert_ne!(
            r.block_reason(),
            Some(BLOCK_SUPPORT_COLLISION),
            "witness={:?} coverage={:?}",
            r.witness,
            r.coverage.qualification
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

    #[test]
    fn an_obstacle_between_stroke_samples_is_not_a_continuous_clear() {
        let model = synth_planar_two_link();
        let names = model.ee_joint_chain("ee").unwrap();
        let contact = vec![0.15, -0.25];
        let end = vec![1.05, -1.2];
        let mid: Vec<f64> = contact
            .iter()
            .zip(&end)
            .map(|(start, finish)| 0.5 * (start + finish))
            .collect();
        let quarter: Vec<f64> = contact
            .iter()
            .zip(&end)
            .map(|(start, finish)| 0.75 * start + 0.25 * finish)
            .collect();
        let tool = |q: &[f64]| forward_kinematics(&model, &names, "ee", q).unwrap().ee.xyz;
        let hit_at = tool(&quarter);
        let mut world = world_at([2.0, 0.0, 0.0]);
        world.support_origin = [0.0, 0.0, -1.0];
        world.ee_radius = 0.008;
        world.obstacles.push(NamedBox::aabb(
            "between-samples",
            hit_at,
            [0.003, 0.003, 0.003],
        ));
        for q in [&contact, &mid, &end] {
            let sample = tool(q);
            let gap = norm3(crate::transform::sub3(sample, hit_at));
            assert!(
                gap > world.ee_radius + 0.006,
                "stroke sample is already inside the interior obstacle, gap={gap}"
            );
        }
        let assessment = prove_coupled_sticking_stroke(
            &model,
            "ee",
            &names,
            &contact,
            &mid,
            &end,
            &world,
            [1.0, 0.0, 0.0],
            0.0,
        );
        assert!(
            matches!(assessment, StickingStrokeAssessment::Forbidden { .. }),
            "interior obstacle must refuse the stroke, got {assessment:?}"
        );
    }
}
