//! First-class qa → qb geometry query. Discrete 5..=20 samples are not a
//! clearance proof. Linear shape-cast plus rotation-bounded subdivision.

use crate::allowed_contact::{AllowedContactPolicy, ContactPhase, PairPermission};
use crate::contact::ContactEvidenceClass;
use crate::contact_collision::{
    BLOCK_OBSTACLE_COLLISION, BLOCK_SELF_COLLISION, BLOCK_SUPPORT_COLLISION,
    BLOCK_UNINTENDED_CONTACT, BLOCK_WRONG_PHASE_CONTACT,
};
use crate::embodiment::EmbodimentModel;
use crate::geometry::{
    CollisionScene, CoverageQualification, CoverageReport, PrimitiveShape, RigidGeometry,
};
use crate::geometry_fk::body_world_transforms;
use crate::geometry_query::{cast_linear, query_geoms, rotation_delta, ContactQuery, QueryStatus};
use crate::transform::{norm3, sub3, Se3};

const SUBDIVIDE_MOTION_M: f64 = 0.008;
const MAX_SUBDIVIDE: u32 = 14;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GeometryVerdict {
    ClearUnderCompleteDeclared,
    NoCollisionInProxy,
    AllowedPhaseContact,
    ForbiddenContact,
    UnsupportedGeometry,
    UnknownCoverage,
}

impl GeometryVerdict {
    pub fn name(self) -> &'static str {
        match self {
            Self::ClearUnderCompleteDeclared => "CLEAR_UNDER_COMPLETE_DECLARED_GEOMETRY",
            Self::NoCollisionInProxy => "NO_COLLISION_FOUND_IN_PROXY_MODEL",
            Self::AllowedPhaseContact => "CONTACT_AT_ALLOWED_PHASE",
            Self::ForbiddenContact => "FORBIDDEN_CONTACT",
            Self::UnsupportedGeometry => "UNSUPPORTED_GEOMETRY",
            Self::UnknownCoverage => "UNKNOWN_COVERAGE",
        }
    }

    pub fn is_unqualified_collision_free(self) -> bool {
        self == Self::ClearUnderCompleteDeclared
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct TransitionWitness {
    pub body_a: String,
    pub body_b: String,
    pub shape_a: String,
    pub shape_b: String,
    pub interval: [f64; 2],
    pub contact_point_a: Option<[f64; 3]>,
    pub contact_point_b: Option<[f64; 3]>,
    pub normal: Option<[f64; 3]>,
    pub clearance: Option<f64>,
    pub failed_constraint: String,
}

#[derive(Debug, Clone, PartialEq)]
pub struct TransitionReport {
    pub verdict: GeometryVerdict,
    pub witness: Option<TransitionWitness>,
    pub coverage: CoverageReport,
    pub class: Option<ContactEvidenceClass>,
}

impl TransitionReport {
    pub fn block_reason(&self) -> Option<&'static str> {
        if self.verdict != GeometryVerdict::ForbiddenContact {
            return None;
        }
        match self.class {
            Some(ContactEvidenceClass::SelfCollision) => Some(BLOCK_SELF_COLLISION),
            Some(ContactEvidenceClass::SupportContact) => Some(BLOCK_SUPPORT_COLLISION),
            Some(ContactEvidenceClass::ObstacleContact) => Some(BLOCK_OBSTACLE_COLLISION),
            Some(ContactEvidenceClass::UnintendedRobotContact) => Some(BLOCK_UNINTENDED_CONTACT),
            Some(ContactEvidenceClass::IntendedToolContact) => Some(BLOCK_WRONG_PHASE_CONTACT),
            None => Some(BLOCK_UNINTENDED_CONTACT),
        }
    }
}

fn lerp_q(qa: &[f64], qb: &[f64], t: f64) -> Vec<f64> {
    qa.iter()
        .zip(qb.iter())
        .map(|(a, b)| a * (1.0 - t) + b * t)
        .collect()
}

fn named_q(names: &[String], q: &[f64]) -> Vec<(String, f64)> {
    names.iter().cloned().zip(q.iter().copied()).collect()
}

struct Posed<'a> {
    geom: &'a RigidGeometry,
    world: Se3,
}

fn posed_at<'a>(
    model: &EmbodimentModel,
    names: &[String],
    q: &[f64],
    scene: &'a CollisionScene,
) -> Result<Vec<Posed<'a>>, &'static str> {
    let bodies = body_world_transforms(model, &named_q(names, q)).map_err(|_| "fk")?;
    let mut out = Vec::new();
    for g in scene.robot.iter().filter(|g| g.participates_in_collision()) {
        let Some(body) = bodies.get(&g.owner_body).copied() else {
            continue;
        };
        out.push(Posed {
            geom: g,
            world: body,
        });
    }
    Ok(out)
}

fn env_world(g: &RigidGeometry) -> Se3 {
    // Environment geoms store their world pose in `local_pose`.
    g.local_pose
}

fn pairs_to_check<'a>(
    posed: &'a [Posed<'a>],
    scene: &'a CollisionScene,
) -> Vec<(&'a RigidGeometry, Se3, &'a RigidGeometry, Se3, bool)> {
    let mut out = Vec::new();
    // robot × robot
    for i in 0..posed.len() {
        for j in (i + 1)..posed.len() {
            if posed[i].geom.owner_body == posed[j].geom.owner_body {
                continue;
            }
            out.push((
                posed[i].geom,
                posed[i].world,
                posed[j].geom,
                posed[j].world,
                true,
            ));
        }
    }
    let env: Vec<&RigidGeometry> = scene
        .object
        .iter()
        .chain(scene.support.iter())
        .chain(scene.obstacles.iter())
        .filter(|g| g.participates_in_collision())
        .collect();
    for p in posed {
        for e in &env {
            // Identity so query_geoms composes env.local_pose as world.
            out.push((p.geom, p.world, *e, Se3::identity(), false));
        }
    }
    out
}

fn classify_hit(
    policy: &AllowedContactPolicy,
    a: &RigidGeometry,
    b: &RigidGeometry,
    phase: ContactPhase,
    sample_i: usize,
    n_samples: usize,
    q: &ContactQuery,
    t0: f64,
    t1: f64,
    coverage: CoverageReport,
) -> Option<TransitionReport> {
    match q.status {
        QueryStatus::Unsupported => {
            return Some(TransitionReport {
                verdict: GeometryVerdict::UnsupportedGeometry,
                witness: Some(TransitionWitness {
                    body_a: a.owner_body.clone(),
                    body_b: b.owner_body.clone(),
                    shape_a: a.id.clone(),
                    shape_b: b.id.clone(),
                    interval: [t0, t1],
                    contact_point_a: None,
                    contact_point_b: None,
                    normal: None,
                    clearance: None,
                    failed_constraint: "GEOMETRY_QUERY_UNSUPPORTED".into(),
                }),
                coverage,
                class: None,
            });
        }
        QueryStatus::Unknown => {
            return Some(TransitionReport {
                verdict: GeometryVerdict::UnknownCoverage,
                witness: Some(TransitionWitness {
                    body_a: a.owner_body.clone(),
                    body_b: b.owner_body.clone(),
                    shape_a: a.id.clone(),
                    shape_b: b.id.clone(),
                    interval: [t0, t1],
                    contact_point_a: None,
                    contact_point_b: None,
                    normal: None,
                    clearance: None,
                    failed_constraint: "GEOMETRY_QUERY_UNKNOWN".into(),
                }),
                coverage,
                class: None,
            });
        }
        QueryStatus::Separated => return None,
        QueryStatus::Intersection => {}
    }
    let permit = policy.permit(&a.owner_body, &b.owner_body, phase, sample_i, n_samples);
    let class = policy.class_of(&a.owner_body, &b.owner_body);
    match permit {
        PairPermission::Allowed => Some(TransitionReport {
            verdict: GeometryVerdict::AllowedPhaseContact,
            witness: Some(TransitionWitness {
                body_a: a.owner_body.clone(),
                body_b: b.owner_body.clone(),
                shape_a: a.id.clone(),
                shape_b: b.id.clone(),
                interval: [t0, t1],
                contact_point_a: q.point_a,
                contact_point_b: q.point_b,
                normal: q.normal_a_to_b,
                clearance: q.distance,
                failed_constraint: String::new(),
            }),
            coverage,
            class,
        }),
        PairPermission::Forbidden => Some(TransitionReport {
            verdict: GeometryVerdict::ForbiddenContact,
            witness: Some(TransitionWitness {
                body_a: a.owner_body.clone(),
                body_b: b.owner_body.clone(),
                shape_a: a.id.clone(),
                shape_b: b.id.clone(),
                interval: [t0, t1],
                contact_point_a: q.point_a,
                contact_point_b: q.point_b,
                normal: q.normal_a_to_b,
                clearance: q.distance,
                failed_constraint: class
                    .map(|c| format!("{c:?}"))
                    .unwrap_or_else(|| "FORBIDDEN_CONTACT".into()),
            }),
            coverage,
            class,
        }),
        PairPermission::Unknown => None,
    }
}

fn is_blocking_verdict(v: GeometryVerdict) -> bool {
    matches!(
        v,
        GeometryVerdict::ForbiddenContact
            | GeometryVerdict::UnsupportedGeometry
            | GeometryVerdict::UnknownCoverage
    )
}

fn absorb_hit(
    acc: &mut Option<TransitionReport>,
    hit: Option<TransitionReport>,
) -> Option<TransitionReport> {
    let h = hit?;
    if is_blocking_verdict(h.verdict) {
        return Some(h);
    }
    if h.verdict == GeometryVerdict::AllowedPhaseContact {
        *acc = Some(h);
    }
    None
}

fn need_subdivide(a0: Se3, a1: Se3, shape: &PrimitiveShape) -> bool {
    let linear = norm3(sub3(a0.xyz, a1.xyz));
    let ang = rotation_delta(a0, a1);
    let r = shape.characteristic_radius().max(1e-4);
    linear > SUBDIVIDE_MOTION_M || ang * r > SUBDIVIDE_MOTION_M
}

fn check_interval(
    model: &EmbodimentModel,
    names: &[String],
    qa: &[f64],
    qb: &[f64],
    t0: f64,
    t1: f64,
    scene: &CollisionScene,
    policy: &AllowedContactPolicy,
    phase: ContactPhase,
    depth: u32,
    coverage: &CoverageReport,
) -> Option<TransitionReport> {
    let q0 = lerp_q(qa, qb, t0);
    let q1 = lerp_q(qa, qb, t1);
    let posed0 = posed_at(model, names, &q0, scene).ok()?;
    let posed1 = posed_at(model, names, &q1, scene).ok()?;
    let phase_n = 8usize;
    let sample_of = |t: f64| {
        ((t * (phase_n.saturating_sub(1) as f64)).round() as usize).min(phase_n.saturating_sub(1))
    };
    let mut allowed: Option<TransitionReport> = None;
    // Endpoint discrete queries.
    for (ga, wa, gb, wb, moving_b) in pairs_to_check(&posed0, scene) {
        let _ = moving_b;
        if let Some(block) = absorb_hit(
            &mut allowed,
            classify_hit(
                policy,
                ga,
                gb,
                phase,
                sample_of(t0),
                phase_n,
                &query_geoms(wa, ga, wb, gb),
                t0,
                t1,
                coverage.clone(),
            ),
        ) {
            return Some(block);
        }
    }
    for (ga, wa, gb, wb, _) in pairs_to_check(&posed1, scene) {
        if let Some(block) = absorb_hit(
            &mut allowed,
            classify_hit(
                policy,
                ga,
                gb,
                phase,
                sample_of(t1),
                phase_n,
                &query_geoms(wa, ga, wb, gb),
                t0,
                t1,
                coverage.clone(),
            ),
        ) {
            return Some(block);
        }
    }
    let mut must_split = false;
    for p0 in &posed0 {
        if let Some(p1) = posed1.iter().find(|p| p.geom.id == p0.geom.id) {
            let g0 = p0.world.compose(p0.geom.local_pose);
            let g1 = p1.world.compose(p1.geom.local_pose);
            if need_subdivide(g0, g1, &p0.geom.shape) {
                must_split = true;
                break;
            }
        }
    }
    if must_split && depth < MAX_SUBDIVIDE {
        let tm = 0.5 * (t0 + t1);
        if let Some(block) = absorb_hit(
            &mut allowed,
            check_interval(
                model,
                names,
                qa,
                qb,
                t0,
                tm,
                scene,
                policy,
                phase,
                depth + 1,
                coverage,
            ),
        ) {
            return Some(block);
        }
        if let Some(block) = absorb_hit(
            &mut allowed,
            check_interval(
                model,
                names,
                qa,
                qb,
                tm,
                t1,
                scene,
                policy,
                phase,
                depth + 1,
                coverage,
            ),
        ) {
            return Some(block);
        }
        return allowed;
    }
    // Linear shape-cast of each robot geom against environment.
    let env: Vec<&RigidGeometry> = scene
        .object
        .iter()
        .chain(scene.support.iter())
        .chain(scene.obstacles.iter())
        .filter(|g| g.participates_in_collision())
        .collect();
    for p0 in &posed0 {
        let Some(p1) = posed1.iter().find(|p| p.geom.id == p0.geom.id) else {
            continue;
        };
        let g0 = p0.world.compose(p0.geom.local_pose);
        let g1 = p1.world.compose(p1.geom.local_pose);
        if norm3(sub3(g0.xyz, g1.xyz)) < 1e-12 && rotation_delta(g0, g1) < 1e-9 {
            continue;
        }
        for e in &env {
            match cast_linear(g0, g1, &p0.geom.shape, env_world(e), &e.shape) {
                Err(QueryStatus::Unsupported) => {
                    return Some(TransitionReport {
                        verdict: GeometryVerdict::UnsupportedGeometry,
                        witness: Some(TransitionWitness {
                            body_a: p0.geom.owner_body.clone(),
                            body_b: e.owner_body.clone(),
                            shape_a: p0.geom.id.clone(),
                            shape_b: e.id.clone(),
                            interval: [t0, t1],
                            contact_point_a: None,
                            contact_point_b: None,
                            normal: None,
                            clearance: None,
                            failed_constraint: "GEOMETRY_QUERY_UNSUPPORTED".into(),
                        }),
                        coverage: coverage.clone(),
                        class: None,
                    });
                }
                Ok(Some(hit)) => {
                    let q = ContactQuery {
                        status: QueryStatus::Intersection,
                        distance: Some(0.0),
                        point_a: Some(hit.point_a),
                        point_b: Some(hit.point_b),
                        normal_a_to_b: Some(hit.normal_a_to_b),
                    };
                    let t_hit = t0 + (t1 - t0) * hit.time_of_impact;
                    let n = 8usize;
                    let sample_i = ((hit.time_of_impact * (n.saturating_sub(1) as f64)).round()
                        as usize)
                        .min(n.saturating_sub(1));
                    if let Some(block) = absorb_hit(
                        &mut allowed,
                        classify_hit(
                            policy,
                            p0.geom,
                            e,
                            phase,
                            sample_i,
                            n,
                            &q,
                            t0,
                            t_hit.max(t0),
                            coverage.clone(),
                        ),
                    ) {
                        return Some(block);
                    }
                }
                _ => {}
            }
        }
        // Robot-robot: discrete at both ends already; subdivide covers mid.
    }
    allowed
}

fn finish_clear(coverage: CoverageReport) -> TransitionReport {
    let verdict = if coverage.qualification == CoverageQualification::CompleteDeclared
        && coverage.is_unqualified_collision_free_allowed()
    {
        GeometryVerdict::ClearUnderCompleteDeclared
    } else if coverage.qualification == CoverageQualification::ProxyModel {
        GeometryVerdict::NoCollisionInProxy
    } else if coverage.qualification == CoverageQualification::UnsupportedPresent {
        GeometryVerdict::UnsupportedGeometry
    } else {
        GeometryVerdict::UnknownCoverage
    };
    TransitionReport {
        verdict,
        witness: None,
        coverage,
        class: None,
    }
}

/// Reconstructable qa → qb geometry verdict.
pub fn validate_transition(
    model: &EmbodimentModel,
    names: &[String],
    qa: &[f64],
    qb: &[f64],
    scene: &CollisionScene,
    policy: &AllowedContactPolicy,
    phase: ContactPhase,
) -> TransitionReport {
    let mut coverage = scene.coverage();
    if names.len() != qa.len() || names.len() != qb.len() || qa.is_empty() {
        coverage.missing.push("q_len".into());
        return TransitionReport {
            verdict: GeometryVerdict::UnknownCoverage,
            witness: Some(TransitionWitness {
                body_a: String::new(),
                body_b: String::new(),
                shape_a: String::new(),
                shape_b: String::new(),
                interval: [0.0, 1.0],
                contact_point_a: None,
                contact_point_b: None,
                normal: None,
                clearance: None,
                failed_constraint: "q_len_mismatch".into(),
            }),
            coverage,
            class: None,
        };
    }
    if let Some(hit) = check_interval(
        model, names, qa, qb, 0.0, 1.0, scene, policy, phase, 0, &coverage,
    ) {
        return hit;
    }
    finish_clear(coverage)
}

/// Discrete-only interpolation used to document the thin-obstacle false negative.
/// Not a clearance proof.
pub fn discrete_interpolation_first_forbidden(
    model: &EmbodimentModel,
    names: &[String],
    qa: &[f64],
    qb: &[f64],
    scene: &CollisionScene,
    policy: &AllowedContactPolicy,
    phase: ContactPhase,
    n: usize,
) -> Option<TransitionReport> {
    let n = n.max(2);
    let coverage = scene.coverage();
    for i in 0..n {
        let t = i as f64 / (n - 1) as f64;
        let q = lerp_q(qa, qb, t);
        let posed = posed_at(model, names, &q, scene).ok()?;
        for (ga, wa, gb, wb, _) in pairs_to_check(&posed, scene) {
            if let Some(hit) = classify_hit(
                policy,
                ga,
                gb,
                phase,
                i,
                n,
                &query_geoms(wa, ga, wb, gb),
                t,
                t,
                coverage.clone(),
            ) {
                if hit.verdict == GeometryVerdict::ForbiddenContact {
                    return Some(hit);
                }
            }
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapter::synth_planar_two_link;
    use crate::allowed_contact::{adjacent_body_pairs, AllowedContactPolicy};
    use crate::embodiment::{unknown_se3, Body, EmbodimentModel, Joint, JointKind};
    use crate::geometry::{CollisionRole, SemanticRole};
    use crate::maneuver_witness::interpolation_count;
    use crate::provenance::Provenanced;
    use crate::transform::Se3;

    fn slider() -> EmbodimentModel {
        let mut m = EmbodimentModel::new("slider", "test", "hash", "epoch0", "1");
        m.bodies.push(Body {
            name: "base".into(),
            parent: None,
            mass_kg: Provenanced::unknown("test", 0.0),
            com: Provenanced::unknown("test", 0.0),
            inertia: Provenanced::unknown("test", 0.0),
            local_pose: Provenanced::declared(Se3::identity(), "test", 0.0),
        });
        m.bodies.push(Body {
            name: "link".into(),
            parent: Some("base".into()),
            mass_kg: Provenanced::unknown("test", 0.0),
            com: Provenanced::unknown("test", 0.0),
            inertia: Provenanced::unknown("test", 0.0),
            local_pose: Provenanced::declared(Se3::identity(), "test", 0.0),
        });
        m.joints.push(Joint {
            name: "slide".into(),
            kind: JointKind::Slide,
            axis: Provenanced::declared([1.0, 0.0, 0.0], "test", 0.0),
            qpos_dim: 1,
            dof_dim: 1,
            parent_body: "base".into(),
            child_body: "link".into(),
            q_min: Provenanced::unknown("test", 0.0),
            q_max: Provenanced::unknown("test", 0.0),
            dq_max: Provenanced::unknown("test", 0.0),
            effort_max: Provenanced::unknown("test", 0.0),
            origin_in_child: Provenanced::declared([0.0, 0.0, 0.0], "test", 0.0),
            parent_to_joint: unknown_se3("test"),
            joint_to_child: unknown_se3("test"),
            qpos_adr: Some(0),
            dof_adr: Some(0),
        });
        m
    }

    fn thin_wall_scene() -> (EmbodimentModel, CollisionScene, AllowedContactPolicy) {
        let model = slider();
        let link_box = RigidGeometry::declared(
            "link_box",
            "link",
            Se3::identity(),
            PrimitiveShape::Box {
                half_extents: [0.02, 0.02, 0.02],
            },
            CollisionRole::Collision,
            SemanticRole::RobotLink,
            "test",
        );
        // Place the 1 mm wall between discrete interpolation samples, not on one.
        let wall = RigidGeometry::declared(
            "wall",
            "obstacle",
            Se3::translation([-0.21875, 0.0, 0.0]).unwrap(),
            PrimitiveShape::Box {
                half_extents: [0.0005, 0.2, 0.2],
            },
            CollisionRole::Collision,
            SemanticRole::Obstacle,
            "test",
        );
        let object = RigidGeometry::declared(
            "obj",
            "object",
            Se3::translation([5.0, 0.0, 0.0]).unwrap(),
            PrimitiveShape::Box {
                half_extents: [0.01, 0.01, 0.01],
            },
            CollisionRole::Collision,
            SemanticRole::Object,
            "test",
        );
        let scene = CollisionScene {
            robot: vec![link_box],
            object: vec![object],
            support: vec![],
            obstacles: vec![wall],
            intended_tool_bodies: vec!["tool".into()],
            object_id: "object".into(),
            support_id: "table".into(),
            adjacent_body_pairs: adjacent_body_pairs(&model),
        };
        let policy = AllowedContactPolicy::from_names(
            "object",
            vec!["tool".into()],
            vec!["link".into(), "base".into()],
            vec!["table".into()],
            vec!["obstacle".into()],
            scene.adjacent_body_pairs.clone(),
        );
        (model, scene, policy)
    }

    #[test]
    fn discrete_samples_miss_thin_obstacle() {
        let (model, scene, policy) = thin_wall_scene();
        let names = vec!["slide".into()];
        let qa = vec![-0.25];
        let qb = vec![0.25];
        let n = interpolation_count(&qa, &qb);
        let miss = discrete_interpolation_first_forbidden(
            &model,
            &names,
            &qa,
            &qb,
            &scene,
            &policy,
            ContactPhase::FreeSpace,
            n,
        );
        assert!(
            miss.is_none(),
            "precondition: discrete samples must miss the 1 mm wall, n={n}"
        );
    }

    #[test]
    fn transition_query_catches_thin_obstacle() {
        let (model, scene, policy) = thin_wall_scene();
        let names = vec!["slide".into()];
        let report = validate_transition(
            &model,
            &names,
            &[-0.25],
            &[0.25],
            &scene,
            &policy,
            ContactPhase::FreeSpace,
        );
        assert_eq!(report.verdict, GeometryVerdict::ForbiddenContact);
        assert_eq!(report.block_reason(), Some(BLOCK_OBSTACLE_COLLISION));
        let w = report.witness.expect("reconstructable witness");
        assert_eq!(w.body_a, "link");
        assert_eq!(w.body_b, "obstacle");
        assert!(w.interval[0] >= 0.0 && w.interval[1] <= 1.0);
        assert_eq!(
            w.failed_constraint.contains("Obstacle").then_some(()),
            Some(())
        );
    }

    #[test]
    fn missing_geom_is_not_unqualified_clear() {
        let model = synth_planar_two_link();
        let scene = CollisionScene {
            robot: vec![],
            object: vec![],
            support: vec![],
            obstacles: vec![],
            intended_tool_bodies: vec![],
            object_id: "object".into(),
            support_id: "table".into(),
            adjacent_body_pairs: vec![],
        };
        let policy =
            AllowedContactPolicy::from_names("object", vec![], vec![], vec![], vec![], vec![]);
        let names = vec!["j0".into(), "j1".into()];
        let r = validate_transition(
            &model,
            &names,
            &[0.0, 0.0],
            &[0.1, 0.0],
            &scene,
            &policy,
            ContactPhase::FreeSpace,
        );
        assert!(!r.verdict.is_unqualified_collision_free());
        assert_eq!(r.verdict, GeometryVerdict::UnknownCoverage);
    }

    #[test]
    fn unsupported_mesh_is_not_collision_free() {
        let model = slider();
        let mesh = RigidGeometry::declared(
            "mesh",
            "link",
            Se3::identity(),
            PrimitiveShape::Unsupported {
                kind: "mesh".into(),
            },
            CollisionRole::Collision,
            SemanticRole::RobotLink,
            "test",
        );
        let wall = RigidGeometry::declared(
            "wall",
            "obstacle",
            Se3::identity(),
            PrimitiveShape::Box {
                half_extents: [0.1, 0.1, 0.1],
            },
            CollisionRole::Collision,
            SemanticRole::Obstacle,
            "test",
        );
        let scene = CollisionScene {
            robot: vec![mesh],
            object: vec![],
            support: vec![],
            obstacles: vec![wall],
            intended_tool_bodies: vec![],
            object_id: "object".into(),
            support_id: "table".into(),
            adjacent_body_pairs: vec![],
        };
        let policy = AllowedContactPolicy::from_names(
            "object",
            vec![],
            vec!["link".into()],
            vec![],
            vec!["obstacle".into()],
            vec![],
        );
        let r = validate_transition(
            &model,
            &["slide".into()],
            &[0.0],
            &[0.01],
            &scene,
            &policy,
            ContactPhase::FreeSpace,
        );
        assert!(!r.verdict.is_unqualified_collision_free());
        assert_eq!(r.verdict, GeometryVerdict::UnsupportedGeometry);
    }

    #[test]
    fn contact_stroke_intended_intersection_is_allowed_phase_contact() {
        let model = slider();
        let link_box = RigidGeometry::declared(
            "link_box",
            "link",
            Se3::identity(),
            PrimitiveShape::Box {
                half_extents: [0.02, 0.02, 0.02],
            },
            CollisionRole::Collision,
            SemanticRole::Tool,
            "test",
        );
        let object = RigidGeometry::declared(
            "obj",
            "object",
            Se3::identity(),
            PrimitiveShape::Box {
                half_extents: [0.02, 0.02, 0.02],
            },
            CollisionRole::Collision,
            SemanticRole::Object,
            "test",
        );
        let scene = CollisionScene {
            robot: vec![link_box],
            object: vec![object],
            support: vec![],
            obstacles: vec![],
            intended_tool_bodies: vec!["link".into()],
            object_id: "object".into(),
            support_id: "table".into(),
            adjacent_body_pairs: adjacent_body_pairs(&model),
        };
        let policy = AllowedContactPolicy::from_names(
            "object",
            vec!["link".into()],
            vec!["link".into(), "base".into()],
            vec!["table".into()],
            vec![],
            scene.adjacent_body_pairs.clone(),
        );
        let report = validate_transition(
            &model,
            &["slide".into()],
            &[0.0],
            &[0.0],
            &scene,
            &policy,
            ContactPhase::ContactStroke,
        );
        assert_eq!(
            report.verdict,
            GeometryVerdict::AllowedPhaseContact,
            "intended tool-object overlap in ContactStroke must be CONTACT_AT_ALLOWED_PHASE, got {:?}",
            report.verdict
        );
        assert!(!report.verdict.is_unqualified_collision_free());
    }

    #[test]
    fn reconstructable_refusal_names_the_pair() {
        let (model, scene, policy) = thin_wall_scene();
        let r = validate_transition(
            &model,
            &["slide".into()],
            &[-0.25],
            &[0.25],
            &scene,
            &policy,
            ContactPhase::FreeSpace,
        );
        let w = r.witness.unwrap();
        assert!(!w.body_a.is_empty());
        assert!(!w.body_b.is_empty());
        assert!(!w.failed_constraint.is_empty());
    }
}
