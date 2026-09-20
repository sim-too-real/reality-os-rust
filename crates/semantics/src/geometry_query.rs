//! Reality OS geometry queries. Computational geometry is behind this
//! interface so a library can be swapped without leaking into skill policy.

use crate::geometry::{PrimitiveShape, RigidGeometry};
use crate::transform::{norm3, Se3};
use parry3d_f64::math::{Pose, Vector};
use parry3d_f64::query::{self, ShapeCastOptions};
use parry3d_f64::shape::{Ball, Capsule, Cuboid, Cylinder, HalfSpace, Shape};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QueryStatus {
    Intersection,
    Separated,
    Unsupported,
    Unknown,
}

#[derive(Debug, Clone, PartialEq)]
pub struct ContactQuery {
    pub status: QueryStatus,
    pub distance: Option<f64>,
    pub point_a: Option<[f64; 3]>,
    pub point_b: Option<[f64; 3]>,
    pub normal_a_to_b: Option<[f64; 3]>,
}

impl ContactQuery {
    fn unsupported() -> Self {
        Self {
            status: QueryStatus::Unsupported,
            distance: None,
            point_a: None,
            point_b: None,
            normal_a_to_b: None,
        }
    }

    fn unknown() -> Self {
        Self {
            status: QueryStatus::Unknown,
            distance: None,
            point_a: None,
            point_b: None,
            normal_a_to_b: None,
        }
    }

    pub fn intersects(&self) -> bool {
        self.status == QueryStatus::Intersection
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ShapeCastHit {
    pub time_of_impact: f64,
    pub point_a: [f64; 3],
    pub point_b: [f64; 3],
    pub normal_a_to_b: [f64; 3],
}

fn se3_to_pose(p: Se3) -> Pose {
    let [w, x, y, z] = p.quat_wxyz;
    Pose::from_parts(
        Vector::new(p.xyz[0], p.xyz[1], p.xyz[2]),
        parry3d_f64::math::Rotation::from_xyzw(x, y, z, w),
    )
}

fn vec_to_arr(v: Vector) -> [f64; 3] {
    [v.x, v.y, v.z]
}

/// MuJoCo cylinder is Z-aligned. Parry Cylinder is Y-aligned.
fn cylinder_pose(world: Se3) -> Pose {
    let corr = Se3::from_axis_angle([1.0, 0.0, 0.0], -std::f64::consts::FRAC_PI_2)
        .unwrap_or_else(|_| Se3::identity());
    se3_to_pose(world.compose(corr))
}

fn query_pose(world: Se3, shape: &PrimitiveShape) -> Pose {
    match shape {
        PrimitiveShape::Cylinder { .. } => cylinder_pose(world),
        _ => se3_to_pose(world),
    }
}

fn with_shape<R>(
    shape: &PrimitiveShape,
    f: impl FnOnce(&dyn Shape) -> R,
) -> Result<R, QueryStatus> {
    match *shape {
        PrimitiveShape::Sphere { radius } => {
            if !(radius.is_finite() && radius > 0.0) {
                return Err(QueryStatus::Unknown);
            }
            Ok(f(&Ball::new(radius)))
        }
        PrimitiveShape::Box { half_extents } => {
            if !half_extents.iter().all(|v| v.is_finite() && *v > 0.0) {
                return Err(QueryStatus::Unknown);
            }
            Ok(f(&Cuboid::new(Vector::new(
                half_extents[0],
                half_extents[1],
                half_extents[2],
            ))))
        }
        PrimitiveShape::Capsule {
            radius,
            half_length,
        } => {
            if !(radius.is_finite()
                && radius > 0.0
                && half_length.is_finite()
                && half_length >= 0.0)
            {
                return Err(QueryStatus::Unknown);
            }
            let a = Vector::new(0.0, 0.0, -half_length);
            let b = Vector::new(0.0, 0.0, half_length);
            Ok(f(&Capsule::new(a, b, radius)))
        }
        PrimitiveShape::Cylinder {
            radius,
            half_length,
        } => {
            if !(radius.is_finite() && radius > 0.0 && half_length.is_finite() && half_length > 0.0)
            {
                return Err(QueryStatus::Unknown);
            }
            Ok(f(&Cylinder::new(half_length, radius)))
        }
        PrimitiveShape::Plane { normal } => {
            // Reality OS plane normal is outward from the solid (table +Z).
            // Parry 0.30 HalfSpace occupies the half-space along its normal,
            // so pass the inward (-outward) direction.
            let n = Vector::new(-normal[0], -normal[1], -normal[2]);
            let Some(unit) = n.try_normalize() else {
                return Err(QueryStatus::Unknown);
            };
            Ok(f(&HalfSpace::new(unit)))
        }
        PrimitiveShape::Unsupported { .. } => Err(QueryStatus::Unsupported),
    }
}

/// Intersection / contact between two posed primitives.
pub fn query_contact(
    pose_a: Se3,
    a: &PrimitiveShape,
    pose_b: Se3,
    b: &PrimitiveShape,
) -> ContactQuery {
    let pa = query_pose(pose_a, a);
    let pb = query_pose(pose_b, b);
    let built = with_shape(a, |sa| {
        with_shape(b, |sb| query::contact(&pa, sa, &pb, sb, 0.0))
    });
    match built {
        Err(QueryStatus::Unsupported) | Ok(Err(QueryStatus::Unsupported)) | Ok(Ok(Err(_))) => {
            ContactQuery::unsupported()
        }
        Err(_) | Ok(Err(_)) => ContactQuery::unknown(),
        Ok(Ok(Ok(None))) => ContactQuery {
            status: QueryStatus::Separated,
            distance: None,
            point_a: None,
            point_b: None,
            normal_a_to_b: None,
        },
        Ok(Ok(Ok(Some(c)))) => {
            let intersecting = c.dist <= 1e-9;
            ContactQuery {
                status: if intersecting {
                    QueryStatus::Intersection
                } else {
                    QueryStatus::Separated
                },
                distance: Some(c.dist),
                point_a: Some(vec_to_arr(c.point1)),
                point_b: Some(vec_to_arr(c.point2)),
                normal_a_to_b: Some(vec_to_arr(c.normal1)),
            }
        }
    }
}

pub fn query_geoms(
    world_a: Se3,
    ga: &RigidGeometry,
    world_b: Se3,
    gb: &RigidGeometry,
) -> ContactQuery {
    let pose_a = world_a.compose(ga.local_pose);
    let pose_b = world_b.compose(gb.local_pose);
    query_contact(pose_a, &ga.shape, pose_b, &gb.shape)
}

/// Linear shape-cast of A from `pose_a0` toward `pose_a1` against stationary B.
/// Rotation of A is not part of the cast; callers must subdivide when rotation is large.
pub fn cast_linear(
    pose_a0: Se3,
    pose_a1: Se3,
    a: &PrimitiveShape,
    pose_b: Se3,
    b: &PrimitiveShape,
) -> Result<Option<ShapeCastHit>, QueryStatus> {
    let p0 = query_pose(pose_a0, a);
    let p1 = query_pose(pose_a1, a);
    let pb = query_pose(pose_b, b);
    let vel = Vector::new(
        p1.translation.x - p0.translation.x,
        p1.translation.y - p0.translation.y,
        p1.translation.z - p0.translation.z,
    );
    let options = ShapeCastOptions {
        max_time_of_impact: 1.0,
        target_distance: 0.0,
        stop_at_penetration: true,
        compute_impact_geometry_on_penetration: true,
    };
    let built = with_shape(a, |sa| {
        with_shape(b, |sb| {
            query::cast_shapes(&p0, vel, sa, &pb, Vector::ZERO, sb, options)
        })
    });
    match built {
        Err(st) | Ok(Err(st)) => Err(st),
        Ok(Ok(Err(_))) => Err(QueryStatus::Unsupported),
        Ok(Ok(Ok(None))) => Ok(None),
        Ok(Ok(Ok(Some(hit)))) => {
            let n = vec_to_arr(hit.normal1);
            let n = if norm3(n) < 1e-12 { [0.0, 0.0, 1.0] } else { n };
            Ok(Some(ShapeCastHit {
                time_of_impact: hit.time_of_impact,
                point_a: vec_to_arr(hit.witness1),
                point_b: vec_to_arr(hit.witness2),
                normal_a_to_b: n,
            }))
        }
    }
}

/// Rotation magnitude between two poses, radians in [0, pi].
pub fn rotation_delta(a: Se3, b: Se3) -> f64 {
    let d = a.inverse().compose(b);
    let [w, _x, _y, _z] = d.quat_wxyz;
    let w = w.abs().min(1.0);
    2.0 * w.acos()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::transform::Se3;

    #[test]
    fn separated_spheres_are_not_intersection() {
        let a = PrimitiveShape::Sphere { radius: 0.05 };
        let b = PrimitiveShape::Sphere { radius: 0.05 };
        let pa = Se3::translation([0.0, 0.0, 0.0]).unwrap();
        let pb = Se3::translation([1.0, 0.0, 0.0]).unwrap();
        let q = query_contact(pa, &a, pb, &b);
        assert_eq!(q.status, QueryStatus::Separated);
        assert!(!q.intersects());
    }

    #[test]
    fn overlapping_boxes_intersect() {
        let a = PrimitiveShape::Box {
            half_extents: [0.05, 0.05, 0.05],
        };
        let q = query_contact(Se3::identity(), &a, Se3::identity(), &a);
        assert_eq!(q.status, QueryStatus::Intersection);
    }

    #[test]
    fn rotated_box_is_not_a_world_aabb() {
        let box_s = PrimitiveShape::Box {
            half_extents: [0.10, 0.01, 0.01],
        };
        let rot = Se3::from_axis_angle([0.0, 0.0, 1.0], std::f64::consts::FRAC_PI_4).unwrap();
        let posed = Se3 {
            xyz: [0.0, 0.0, 0.0],
            quat_wxyz: rot.quat_wxyz,
        };
        let probe = PrimitiveShape::Sphere { radius: 0.008 };
        // World-AABB of the unrotated box would occupy x=±0.10, y=±0.01.
        // After 45° rotation the world-y extent is ~0.078. A probe at
        // (0.0, 0.06, 0.0) is outside the AABB-of-unrotated-extents in y
        // wait: unrotated y-half is 0.01, so (0, 0.06) is outside AABB too.
        // Probe at the rotated long-axis tip: length 0.10 along (√2/2, √2/2).
        let tip = [
            0.10 * std::f64::consts::FRAC_1_SQRT_2,
            0.10 * std::f64::consts::FRAC_1_SQRT_2,
            0.0,
        ];
        let probe_pose = Se3::translation(tip).unwrap();
        let at_tip = query_contact(posed, &box_s, probe_pose, &probe);
        assert_eq!(at_tip.status, QueryStatus::Intersection);
        // World-AABB of the *unrotated* box would claim occupancy at (0.10, 0, 0).
        // The rotated thin box does not occupy that world-AABB face center.
        let aabb_face = Se3::translation([0.10, 0.0, 0.0]).unwrap();
        let at_aabb = query_contact(posed, &box_s, aabb_face, &probe);
        assert_eq!(at_aabb.status, QueryStatus::Separated);
    }

    #[test]
    fn sphere_and_plane_both_translated_are_separated() {
        let plane = PrimitiveShape::Plane {
            normal: [0.0, 0.0, 1.0],
        };
        let ball = PrimitiveShape::Sphere { radius: 0.015 };
        let q = query_contact(
            Se3::translation([0.30, 0.0, 0.0]).unwrap(),
            &ball,
            Se3::translation([0.30, 0.0, -0.05]).unwrap(),
            &plane,
        );
        assert_eq!(
            q.status,
            QueryStatus::Separated,
            "dist={:?} status={:?} pa={:?} pb={:?}",
            q.distance,
            q.status,
            q.point_a,
            q.point_b
        );
    }

    #[test]
    fn sphere_above_plane_is_separated() {
        let plane = PrimitiveShape::Plane {
            normal: [0.0, 0.0, 1.0],
        };
        let ball = PrimitiveShape::Sphere { radius: 0.015 };
        let plane_pose = Se3::translation([0.0, 0.0, -0.05]).unwrap();
        let ball_pose = Se3::identity();
        let q = query_contact(ball_pose, &ball, plane_pose, &plane);
        assert_eq!(
            q.status,
            QueryStatus::Separated,
            "dist={:?} status={:?}",
            q.distance,
            q.status
        );
    }

    #[test]
    fn sphere_below_plane_intersects() {
        let plane = PrimitiveShape::Plane {
            normal: [0.0, 0.0, 1.0],
        };
        let ball = PrimitiveShape::Sphere { radius: 0.015 };
        let plane_pose = Se3::translation([0.0, 0.0, 0.0]).unwrap();
        let ball_pose = Se3::translation([0.0, 0.0, -0.02]).unwrap();
        let q = query_contact(ball_pose, &ball, plane_pose, &plane);
        assert_eq!(q.status, QueryStatus::Intersection);
    }

    #[test]
    fn mesh_is_unsupported_not_clear() {
        let mesh = PrimitiveShape::Unsupported {
            kind: "mesh".into(),
        };
        let ball = PrimitiveShape::Sphere { radius: 0.1 };
        let q = query_contact(Se3::identity(), &mesh, Se3::identity(), &ball);
        assert_eq!(q.status, QueryStatus::Unsupported);
    }

    #[test]
    fn linear_cast_hits_thin_wall() {
        let moving = PrimitiveShape::Box {
            half_extents: [0.02, 0.02, 0.02],
        };
        let wall = PrimitiveShape::Box {
            half_extents: [0.0005, 0.2, 0.2],
        };
        let a0 = Se3::translation([-0.25, 0.0, 0.0]).unwrap();
        let a1 = Se3::translation([0.25, 0.0, 0.0]).unwrap();
        let wall_pose = Se3::identity();
        let hit = cast_linear(a0, a1, &moving, wall_pose, &wall)
            .expect("supported")
            .expect("must hit");
        assert!(hit.time_of_impact > 0.0);
        assert!(hit.time_of_impact < 1.0);
    }
}
