//! Body-geometry FK: joint state → body transform → geom-local → geom world.

use crate::embodiment::{EmbodimentModel, JointKind};
use crate::geometry::RigidGeometry;
use crate::kinematics::motion_transform;
use crate::skill::SkillRefuse;
use crate::transform::Se3;
use std::collections::BTreeMap;

/// World transform of every body at named q.
/// Joints missing from `named_q` are treated as q = 0.
pub fn body_world_transforms(
    model: &EmbodimentModel,
    named_q: &[(String, f64)],
) -> Result<BTreeMap<String, Se3>, SkillRefuse> {
    let q_of = |name: &str| -> f64 {
        named_q
            .iter()
            .find(|(n, _)| n == name)
            .map(|(_, v)| *v)
            .unwrap_or(0.0)
    };
    let mut remaining: Vec<String> = model.bodies.iter().map(|b| b.name.clone()).collect();
    let mut out: BTreeMap<String, Se3> = BTreeMap::new();
    let mut guard = 0;
    while !remaining.is_empty() {
        guard += 1;
        if guard > model.bodies.len() + 4 {
            return Err(SkillRefuse::ModelFeatureUnsupported);
        }
        let mut progressed = false;
        remaining.retain(|name| {
            let Some(body) = model.bodies.iter().find(|b| b.name == *name) else {
                progressed = true;
                return false;
            };
            let parent_world = match body.parent.as_deref() {
                None | Some("") | Some("world") => Se3::identity(),
                Some(p) => match out.get(p) {
                    Some(t) => *t,
                    None => return true,
                },
            };
            let Some(local) = body.local_pose.value else {
                progressed = true;
                return false;
            };
            let world = if let Some(j) = model.joints.iter().find(|j| j.child_body == *name) {
                match j.kind {
                    JointKind::Ball | JointKind::Free | JointKind::Other => {
                        progressed = true;
                        return false;
                    }
                    _ => {}
                }
                match motion_transform(j, q_of(&j.name)) {
                    Ok(motion) => parent_world.compose(local).compose(motion),
                    Err(_) => {
                        progressed = true;
                        return false;
                    }
                }
            } else {
                parent_world.compose(local)
            };
            out.insert(name.clone(), world);
            progressed = true;
            false
        });
        if !progressed {
            break;
        }
    }
    Ok(out)
}

/// Geometry world transform: body world ∘ geometry-local.
pub fn geom_world(body_world: Se3, geom: &RigidGeometry) -> Se3 {
    body_world.compose(geom.local_pose)
}

pub fn geom_world_at(bodies: &BTreeMap<String, Se3>, geom: &RigidGeometry) -> Option<Se3> {
    bodies
        .get(&geom.owner_body)
        .copied()
        .map(|b| geom_world(b, geom))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapter::synth_planar_two_link;
    use crate::geometry::{CollisionRole, PrimitiveShape, SemanticRole};
    use crate::kinematics::forward_kinematics;
    use crate::provenance::Provenanced;
    use crate::transform::{norm3, sub3, Se3};

    fn named(model: &EmbodimentModel, q: &[f64]) -> Vec<(String, f64)> {
        model
            .ee_joint_chain("ee")
            .unwrap()
            .into_iter()
            .zip(q.iter().copied())
            .collect()
    }

    #[test]
    fn body_fk_matches_chain_fk_tip() {
        let model = synth_planar_two_link();
        let q = [0.3, -0.4];
        let chain = model.ee_joint_chain("ee").unwrap();
        let fk = forward_kinematics(&model, &chain, "ee", &q).unwrap();
        let bodies = body_world_transforms(&model, &named(&model, &q)).unwrap();
        let link2 = bodies.get("link2").expect("link2");
        let ee = model.frames.iter().find(|f| f.name == "ee").unwrap();
        let ee_local = ee.pose().unwrap();
        let ee_world = link2.compose(ee_local);
        let d = norm3(sub3(ee_world.xyz, fk.ee.xyz));
        assert!(d < 1e-9, "ee mismatch {d}");
    }

    #[test]
    fn offset_geom_is_not_parked_at_joint_origin() {
        let model = synth_planar_two_link();
        let q = [0.0, 0.0];
        let bodies = body_world_transforms(&model, &named(&model, &q)).unwrap();
        let geom = RigidGeometry::declared(
            "link1_box",
            "link1",
            Se3::translation([0.07, 0.0, 0.0]).unwrap(),
            PrimitiveShape::Box {
                half_extents: [0.02, 0.02, 0.02],
            },
            CollisionRole::Collision,
            SemanticRole::RobotLink,
            "test",
        );
        let gw = geom_world_at(&bodies, &geom).unwrap();
        let joint_origin = bodies.get("link1").unwrap().xyz;
        let d = norm3(sub3(gw.xyz, joint_origin));
        assert!(
            (d - 0.07).abs() < 1e-9,
            "offset geom world {} vs joint {}",
            gw.xyz[0],
            joint_origin[0]
        );
    }

    #[test]
    fn multiple_geoms_on_one_body_keep_distinct_local_poses() {
        let model = synth_planar_two_link();
        let bodies = body_world_transforms(&model, &named(&model, &[0.0, 0.0])).unwrap();
        let a = RigidGeometry::declared(
            "a",
            "link2",
            Se3::translation([0.01, 0.0, 0.0]).unwrap(),
            PrimitiveShape::Sphere { radius: 0.01 },
            CollisionRole::Collision,
            SemanticRole::RobotLink,
            "test",
        );
        let b = RigidGeometry::declared(
            "b",
            "link2",
            Se3::translation([-0.01, 0.0, 0.0]).unwrap(),
            PrimitiveShape::Sphere { radius: 0.01 },
            CollisionRole::Collision,
            SemanticRole::RobotLink,
            "test",
        );
        let wa = geom_world_at(&bodies, &a).unwrap();
        let wb = geom_world_at(&bodies, &b).unwrap();
        assert!(norm3(sub3(wa.xyz, wb.xyz)) > 0.015);
    }

    #[test]
    fn rotated_collision_shape_keeps_orientation() {
        let model = synth_planar_two_link();
        let bodies = body_world_transforms(&model, &named(&model, &[0.0, 0.0])).unwrap();
        let local = Se3::from_axis_angle([0.0, 0.0, 1.0], 0.4).unwrap();
        let geom = RigidGeometry::declared(
            "rot",
            "link1",
            local,
            PrimitiveShape::Box {
                half_extents: [0.04, 0.01, 0.01],
            },
            CollisionRole::Collision,
            SemanticRole::RobotLink,
            "test",
        );
        let gw = geom_world_at(&bodies, &geom).unwrap();
        let body = *bodies.get("link1").unwrap();
        let expected = body.compose(local);
        let dq = (0..4)
            .map(|i| (gw.quat_wxyz[i] - expected.quat_wxyz[i]).abs())
            .fold(0.0, f64::max);
        assert!(dq < 1e-9, "quat delta {dq}");
    }

    #[test]
    fn fixed_distal_body_follows_parent() {
        let mut model = synth_planar_two_link();
        model.bodies.push(crate::embodiment::Body {
            name: "finger".into(),
            parent: Some("link2".into()),
            mass_kg: Provenanced::unknown("test", 0.0),
            com: Provenanced::unknown("test", 0.0),
            inertia: Provenanced::unknown("test", 0.0),
            local_pose: Provenanced::declared(
                Se3::translation([0.05, 0.0, 0.0]).unwrap(),
                "test",
                0.0,
            ),
        });
        let q = [0.5, -0.2];
        let bodies = body_world_transforms(&model, &named(&model, &q)).unwrap();
        let link2 = *bodies.get("link2").unwrap();
        let finger = *bodies.get("finger").unwrap();
        let expected = link2.compose(Se3::translation([0.05, 0.0, 0.0]).unwrap());
        let d = norm3(sub3(finger.xyz, expected.xyz));
        assert!(d < 1e-9, "fixed distal d={d}");
    }
}
