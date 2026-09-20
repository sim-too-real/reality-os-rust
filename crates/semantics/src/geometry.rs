//! Planner-visible declared rigid geometry. Not a CAD engine.
//!
//! MuJoCo / privileged simulator query results must not enter these types.

use crate::provenance::Provenance;
use crate::transform::Se3;
use serde::{Deserialize, Serialize};

/// How a geometric representation relates to declared physical geometry.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum ApproximationClass {
    ExactDeclared,
    ConservativeApproximation,
    NonconservativeApproximation,
    Unknown,
    PrivilegedVerifierOnly,
}

/// Collision vs visual role as declared by the source model.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum CollisionRole {
    Collision,
    Visual,
    Unknown,
}

/// Semantic role where known. Unknown is honest.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum SemanticRole {
    RobotLink,
    Tool,
    Finger,
    Object,
    Support,
    Obstacle,
    Unknown,
}

/// Primitive actually encountered in the development corpus.
/// Unsupported shapes stay unsupported; they are not invented.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PrimitiveShape {
    Sphere { radius: f64 },
    Box { half_extents: [f64; 3] },
    Capsule { radius: f64, half_length: f64 },
    Cylinder { radius: f64, half_length: f64 },
    Plane { normal: [f64; 3] },
    Unsupported { kind: String },
}

impl PrimitiveShape {
    pub fn kind_name(&self) -> &str {
        match self {
            Self::Sphere { .. } => "sphere",
            Self::Box { .. } => "box",
            Self::Capsule { .. } => "capsule",
            Self::Cylinder { .. } => "cylinder",
            Self::Plane { .. } => "plane",
            Self::Unsupported { kind } => kind.as_str(),
        }
    }

    pub fn query_supported(&self) -> bool {
        !matches!(self, Self::Unsupported { .. })
    }

    /// Characteristic half-size used for motion-bounded subdivision.
    pub fn characteristic_radius(&self) -> f64 {
        match *self {
            Self::Sphere { radius } => radius.abs(),
            Self::Box { half_extents } => half_extents[0]
                .abs()
                .max(half_extents[1].abs())
                .max(half_extents[2].abs()),
            Self::Capsule {
                radius,
                half_length,
            }
            | Self::Cylinder {
                radius,
                half_length,
            } => radius.abs() + half_length.abs(),
            Self::Plane { .. } => 0.0,
            Self::Unsupported { .. } => 0.0,
        }
    }
}

/// One declared rigid geom attached to an owner body.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RigidGeometry {
    pub id: String,
    pub owner_body: String,
    pub local_pose: Se3,
    pub shape: PrimitiveShape,
    pub collision_role: CollisionRole,
    pub semantic_role: SemanticRole,
    pub provenance: Provenance,
    pub approximation: ApproximationClass,
    pub source: String,
}

impl RigidGeometry {
    pub fn declared(
        id: impl Into<String>,
        owner_body: impl Into<String>,
        local_pose: Se3,
        shape: PrimitiveShape,
        collision_role: CollisionRole,
        semantic_role: SemanticRole,
        source: impl Into<String>,
    ) -> Self {
        Self {
            id: id.into(),
            owner_body: owner_body.into(),
            local_pose,
            shape,
            collision_role,
            semantic_role,
            provenance: Provenance::ModelDeclared,
            approximation: ApproximationClass::ExactDeclared,
            source: source.into(),
        }
    }

    pub fn participates_in_collision(&self) -> bool {
        self.collision_role != CollisionRole::Visual
    }
}

/// How complete the checked geometry was.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum CoverageQualification {
    CompleteDeclared,
    ProxyModel,
    MissingRequired,
    UnsupportedPresent,
}

impl CoverageQualification {
    /// Unqualified COLLISION_FREE is only honest under complete declared geometry.
    pub fn permits_unqualified_clear(self) -> bool {
        self == Self::CompleteDeclared
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CoverageReport {
    pub robot_bodies: Vec<String>,
    pub robot_shapes: Vec<String>,
    pub environment_entities: Vec<String>,
    pub target_object: Vec<String>,
    pub support: Vec<String>,
    pub missing: Vec<String>,
    pub pairs_checked: usize,
    pub pairs_allowed: usize,
    pub pairs_unsupported: usize,
    pub approximations: Vec<(String, ApproximationClass)>,
    pub qualification: CoverageQualification,
}

impl CoverageReport {
    pub fn empty(qualification: CoverageQualification) -> Self {
        Self {
            robot_bodies: Vec::new(),
            robot_shapes: Vec::new(),
            environment_entities: Vec::new(),
            target_object: Vec::new(),
            support: Vec::new(),
            missing: Vec::new(),
            pairs_checked: 0,
            pairs_allowed: 0,
            pairs_unsupported: 0,
            approximations: Vec::new(),
            qualification,
        }
    }

    pub fn is_unqualified_collision_free_allowed(&self) -> bool {
        self.qualification.permits_unqualified_clear()
            && self.missing.is_empty()
            && self.pairs_unsupported == 0
    }
}

/// Planner-visible collision scene. Privileged simulator contacts are not stored here.
#[derive(Debug, Clone, PartialEq)]
pub struct CollisionScene {
    pub robot: Vec<RigidGeometry>,
    pub object: Vec<RigidGeometry>,
    pub support: Vec<RigidGeometry>,
    pub obstacles: Vec<RigidGeometry>,
    pub intended_tool_bodies: Vec<String>,
    pub object_id: String,
    pub support_id: String,
    pub adjacent_body_pairs: Vec<(String, String)>,
}

impl CollisionScene {
    pub fn all_geoms(&self) -> impl Iterator<Item = &RigidGeometry> {
        self.robot
            .iter()
            .chain(self.object.iter())
            .chain(self.support.iter())
            .chain(self.obstacles.iter())
    }

    pub fn collision_geoms(&self) -> impl Iterator<Item = &RigidGeometry> {
        self.all_geoms().filter(|g| g.participates_in_collision())
    }

    pub fn coverage(&self) -> CoverageReport {
        let mut report = CoverageReport::empty(CoverageQualification::CompleteDeclared);
        let mut missing = Vec::new();
        let mut unsupported = 0usize;
        let mut proxy = false;
        for g in self.robot.iter() {
            if !report.robot_bodies.iter().any(|b| b == &g.owner_body) {
                report.robot_bodies.push(g.owner_body.clone());
            }
            report
                .robot_shapes
                .push(format!("{}:{}", g.id, g.shape.kind_name()));
            report.approximations.push((g.id.clone(), g.approximation));
            if g.approximation != ApproximationClass::ExactDeclared {
                proxy = true;
            }
            if !g.shape.query_supported() {
                unsupported += 1;
                missing.push(format!("unsupported:{}", g.id));
            }
        }
        for g in &self.object {
            report
                .target_object
                .push(format!("{}:{}", g.id, g.shape.kind_name()));
            report.approximations.push((g.id.clone(), g.approximation));
            if g.approximation != ApproximationClass::ExactDeclared {
                proxy = true;
            }
            if !g.shape.query_supported() {
                unsupported += 1;
                missing.push(format!("unsupported:{}", g.id));
            }
        }
        for g in &self.support {
            report
                .support
                .push(format!("{}:{}", g.id, g.shape.kind_name()));
            report.approximations.push((g.id.clone(), g.approximation));
        }
        for g in &self.obstacles {
            report
                .environment_entities
                .push(format!("{}:{}", g.id, g.shape.kind_name()));
            report.approximations.push((g.id.clone(), g.approximation));
            if g.approximation != ApproximationClass::ExactDeclared {
                proxy = true;
            }
            if !g.shape.query_supported() {
                unsupported += 1;
                missing.push(format!("unsupported:{}", g.id));
            }
        }
        if self.robot.is_empty() {
            missing.push("robot_collision_geometry".into());
        }
        if self.object.is_empty() {
            missing.push("target_object_geometry".into());
        }
        report.missing = missing;
        report.pairs_unsupported = unsupported;
        report.qualification = if unsupported > 0 {
            CoverageQualification::UnsupportedPresent
        } else if report
            .missing
            .iter()
            .any(|m| !m.starts_with("unsupported:"))
        {
            CoverageQualification::MissingRequired
        } else if proxy {
            CoverageQualification::ProxyModel
        } else {
            CoverageQualification::CompleteDeclared
        };
        report
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AssumptionRecord {
    pub name: String,
    pub classification: ApproximationClass,
    pub status: &'static str,
    pub detail: String,
}

/// Explicit ledger of physical substitutions found on the import→contact path.
pub fn physical_assumption_ledger() -> Vec<AssumptionRecord> {
    vec![
        AssumptionRecord {
            name: "invented_ee_local_plus_x_tool_axis".into(),
            classification: ApproximationClass::Unknown,
            status: "removed",
            detail: "tool_axis_of no longer rotates EE-local [1,0,0] when the tool offset is too short. Missing axis stays UNKNOWN and cannot become ALIGNED.".into(),
        },
        AssumptionRecord {
            name: "world_aabb_box_object".into(),
            classification: ApproximationClass::NonconservativeApproximation,
            status: "replaced",
            detail: "BoxObject now carries an object-frame orientation. A face is a face of the object, not a world-AABB support.".into(),
        },
        AssumptionRecord {
            name: "sphere_at_joint_origin".into(),
            classification: ApproximationClass::NonconservativeApproximation,
            status: "classified",
            detail: "AttachedSphere at a joint origin is a proxy. Absence of sphere overlap is NO_COLLISION_FOUND_IN_PROXY_MODEL, not unqualified COLLISION_FREE.".into(),
        },
        AssumptionRecord {
            name: "axis_aligned_obstacle_box".into(),
            classification: ApproximationClass::NonconservativeApproximation,
            status: "replaced",
            detail: "Obstacles preserve declared orientation. World-AABB obstacles remain a proxy when orientation is missing.".into(),
        },
        AssumptionRecord {
            name: "discrete_interpolation_5_to_20".into(),
            classification: ApproximationClass::NonconservativeApproximation,
            status: "replaced",
            detail: "Joint-limit interpolation stays 5..=20 samples. Transition geometry uses linear shape-cast plus rotation-bounded subdivision. Discrete-only sampling is not a clearance proof.".into(),
        },
        AssumptionRecord {
            name: "privileged_simulator_contacts".into(),
            classification: ApproximationClass::PrivilegedVerifierOnly,
            status: "enforced",
            detail: "MuJoCo contact is post-hoc verification. It does not enter decide, select, or runtime evidence unless modeled as a policy-visible sensor.".into(),
        },
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unsupported_mesh_is_not_a_queryable_shape() {
        let s = PrimitiveShape::Unsupported {
            kind: "mesh".into(),
        };
        assert!(!s.query_supported());
        assert_eq!(s.kind_name(), "mesh");
    }

    #[test]
    fn missing_required_geom_is_not_complete_declared() {
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
        let c = scene.coverage();
        assert_eq!(c.qualification, CoverageQualification::MissingRequired);
        assert!(!c.is_unqualified_collision_free_allowed());
    }

    #[test]
    fn invented_plus_x_axis_is_removed_on_the_ledger() {
        let ledger = physical_assumption_ledger();
        let row = ledger
            .iter()
            .find(|a| a.name == "invented_ee_local_plus_x_tool_axis")
            .expect("ledger row");
        assert_eq!(row.status, "removed");
        assert_eq!(row.classification, ApproximationClass::Unknown);
    }

    #[test]
    fn proxy_sphere_is_not_exact_declared() {
        let g = RigidGeometry {
            id: "proxy".into(),
            owner_body: "link".into(),
            local_pose: Se3::identity(),
            shape: PrimitiveShape::Sphere { radius: 0.05 },
            collision_role: CollisionRole::Collision,
            semantic_role: SemanticRole::RobotLink,
            provenance: Provenance::Unknown,
            approximation: ApproximationClass::NonconservativeApproximation,
            source: "proxy".into(),
        };
        let scene = CollisionScene {
            robot: vec![g],
            object: vec![RigidGeometry::declared(
                "obj",
                "object",
                Se3::identity(),
                PrimitiveShape::Box {
                    half_extents: [0.03, 0.03, 0.03],
                },
                CollisionRole::Collision,
                SemanticRole::Object,
                "test",
            )],
            support: vec![],
            obstacles: vec![],
            intended_tool_bodies: vec!["tool".into()],
            object_id: "object".into(),
            support_id: "table".into(),
            adjacent_body_pairs: vec![],
        };
        assert_eq!(
            scene.coverage().qualification,
            CoverageQualification::ProxyModel
        );
        assert!(!scene.coverage().is_unqualified_collision_free_allowed());
    }
}
