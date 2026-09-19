use crate::embodiment::{EmbodimentModel, FrameKind};
use crate::provenance::{Provenance, Provenanced};
use crate::resource::ControlledResource;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ContactState {
    pub entity_a: String,
    pub entity_b: String,
    pub contact_points: Vec<[f64; 3]>,
    pub normal: Provenanced<[f64; 3]>,
    pub normal_force: Provenanced<f64>,
    pub tangential_force: Provenanced<f64>,
    pub penetration_depth: Provenanced<f64>,
    pub relative_velocity: Provenanced<[f64; 3]>,
    pub slipping: Option<bool>,
    pub allowed: bool,
    pub expected: bool,
    pub timestamp_s: f64,
    pub provenance: Provenance,
}

impl ContactState {
    pub fn pair_matches(&self, a: &str, b: &str) -> bool {
        (self.entity_a == a && self.entity_b == b) || (self.entity_a == b && self.entity_b == a)
    }

    pub fn force_bound_unavailable(&self) -> bool {
        self.normal_force.value.is_none() && self.tangential_force.value.is_none()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum SupportKind {
    SupportedBy,
    Unsupported,
    Shared,
    Airborne,
    Unknown,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SupportRelation {
    pub object_id: String,
    pub surface_id: String,
    pub kind: SupportKind,
    pub evidence: Vec<String>,
    pub provenance: Provenance,
    pub timestamp_s: f64,
}

impl SupportRelation {
    pub fn unknown(object_id: impl Into<String>, now_s: f64) -> Self {
        Self {
            object_id: object_id.into(),
            surface_id: String::new(),
            kind: SupportKind::Unknown,
            evidence: Vec::new(),
            provenance: Provenance::Unknown,
            timestamp_s: now_s,
        }
    }
}

pub const FORCE_BOUND_UNAVAILABLE: &str = "FORCE_BOUND_UNAVAILABLE";

/// Privileged evaluation classes. Not ranking inputs.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ContactEvidenceClass {
    IntendedToolContact,
    UnintendedRobotContact,
    SupportContact,
    SelfCollision,
    ObstacleContact,
}

pub struct ContactClassContext<'a> {
    pub object_id: &'a str,
    pub intended: &'a [String],
    pub robot_bodies: &'a [String],
    pub support_bodies: &'a [String],
    pub obstacle_bodies: &'a [String],
}

/// Generic support names (table/floor/world/ground), not robot identity.
pub fn is_support_surface_name(name: &str) -> bool {
    let n = name.to_ascii_lowercase();
    n == "table" || n == "floor" || n == "world" || n.contains("ground") || n.ends_with("/table")
}

fn refers(name: &str, key: &str) -> bool {
    if key.is_empty() {
        return false;
    }
    name == key || name.contains(key)
}

fn hits_any(name: &str, keys: &[String]) -> bool {
    keys.iter().any(|k| refers(name, k))
}

fn is_object(name: &str, object_id: &str) -> bool {
    refers(name, object_id)
}

/// Declared manipulation contact geometry: semantic EE, tool frames, fingers.
/// Ancestors of the EE (forearm/wrist links) are not included unless they
/// *are* the EE/tool body.
pub fn declared_manipulation_contact_bodies(
    model: &EmbodimentModel,
    resource: Option<&ControlledResource>,
    ee_name: &str,
) -> Vec<String> {
    let mut out = Vec::new();
    let mut push_unique = |s: String| {
        if !s.is_empty() && !out.contains(&s) {
            out.push(s);
        }
    };
    if let Some(r) = resource {
        for f in &r.finger_bodies {
            push_unique(f.clone());
        }
    }
    let ee = model.end_effectors.iter().find(|e| e.name == ee_name);
    let ee_frame = ee.and_then(|e| model.frames.iter().find(|f| f.name == e.frame));
    for fr in &model.frames {
        if fr.kind == FrameKind::Tool {
            push_unique(fr.parent_body.clone());
        }
        if fr.kind == FrameKind::Ee && ee.is_some_and(|e| e.frame == fr.name) {
            push_unique(fr.parent_body.clone());
        }
    }
    if let Some(fr) = ee_frame {
        let fingers_declared = resource.is_some_and(|r| !r.finger_bodies.is_empty());
        if !fingers_declared {
            for b in &model.bodies {
                if b.parent.as_deref() == Some(fr.parent_body.as_str()) {
                    push_unique(b.name.clone());
                }
            }
        }
    }
    out
}

pub fn classify_contact_pair(
    a: &str,
    b: &str,
    ctx: &ContactClassContext,
) -> Option<ContactEvidenceClass> {
    let a_obj = is_object(a, ctx.object_id);
    let b_obj = is_object(b, ctx.object_id);
    let a_sup = is_support_surface_name(a) || hits_any(a, ctx.support_bodies);
    let b_sup = is_support_surface_name(b) || hits_any(b, ctx.support_bodies);
    let a_obs = hits_any(a, ctx.obstacle_bodies);
    let b_obs = hits_any(b, ctx.obstacle_bodies);
    let a_int = hits_any(a, ctx.intended);
    let b_int = hits_any(b, ctx.intended);
    let a_rob = hits_any(a, ctx.robot_bodies) && !a_obj && !a_sup && !a_obs;
    let b_rob = hits_any(b, ctx.robot_bodies) && !b_obj && !b_sup && !b_obs;

    if a_obj && b_obj {
        return None;
    }
    if (a_obj && b_int) || (b_obj && a_int) {
        return Some(ContactEvidenceClass::IntendedToolContact);
    }
    if (a_obj && b_rob) || (b_obj && a_rob) {
        return Some(ContactEvidenceClass::UnintendedRobotContact);
    }
    if (a_obj && b_sup)
        || (b_obj && a_sup)
        || ((a_rob || a_int) && b_sup)
        || ((b_rob || b_int) && a_sup)
    {
        return Some(ContactEvidenceClass::SupportContact);
    }
    if a_rob && b_rob {
        return Some(ContactEvidenceClass::SelfCollision);
    }
    if a_obs || b_obs {
        return Some(ContactEvidenceClass::ObstacleContact);
    }
    None
}

/// True only when the non-object body is declared manipulation geometry.
pub fn names_are_intended_tool_object_contact(
    a: &str,
    b: &str,
    object_id: &str,
    intended: &[String],
) -> bool {
    if object_id.is_empty() {
        return false;
    }
    classify_contact_pair(
        a,
        b,
        &ContactClassContext {
            object_id,
            intended,
            robot_bodies: &[],
            support_bodies: &[],
            obstacle_bodies: &[],
        },
    ) == Some(ContactEvidenceClass::IntendedToolContact)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn missing_force_is_unavailable_not_zero() {
        let c = ContactState {
            entity_a: "finger".into(),
            entity_b: "obj".into(),
            contact_points: vec![],
            normal: Provenanced::unknown("contact.n", 0.0),
            normal_force: Provenanced::unknown("contact.fn", 0.0),
            tangential_force: Provenanced::unknown("contact.ft", 0.0),
            penetration_depth: Provenanced::unknown("contact.d", 0.0),
            relative_velocity: Provenanced::unknown("contact.v", 0.0),
            slipping: None,
            allowed: true,
            expected: true,
            timestamp_s: 0.0,
            provenance: Provenance::Unknown,
        };
        assert!(c.force_bound_unavailable());
        assert_eq!(FORCE_BOUND_UNAVAILABLE, "FORCE_BOUND_UNAVAILABLE");
    }

    #[test]
    fn tool_face_is_intended_forearm_is_not() {
        let intended = vec!["finger".into(), "hand".into()];
        let robot = vec![
            "forearm".into(),
            "wrist_link".into(),
            "hand".into(),
            "finger".into(),
        ];
        let ctx = ContactClassContext {
            object_id: "obj0",
            intended: &intended,
            robot_bodies: &robot,
            support_bodies: &["table".into()],
            obstacle_bodies: &["obstacle".into()],
        };
        assert_eq!(
            classify_contact_pair("finger", "obj0", &ctx),
            Some(ContactEvidenceClass::IntendedToolContact)
        );
        assert_eq!(
            classify_contact_pair("hand", "obj0", &ctx),
            Some(ContactEvidenceClass::IntendedToolContact)
        );
        assert_eq!(
            classify_contact_pair("forearm", "obj0", &ctx),
            Some(ContactEvidenceClass::UnintendedRobotContact)
        );
        assert_eq!(
            classify_contact_pair("wrist_link", "obj0", &ctx),
            Some(ContactEvidenceClass::UnintendedRobotContact)
        );
        assert_eq!(
            classify_contact_pair("table", "obj0", &ctx),
            Some(ContactEvidenceClass::SupportContact)
        );
        assert_eq!(
            classify_contact_pair("forearm", "wrist_link", &ctx),
            Some(ContactEvidenceClass::SelfCollision)
        );
        assert_eq!(
            classify_contact_pair("finger", "obstacle", &ctx),
            Some(ContactEvidenceClass::ObstacleContact)
        );
        assert!(!names_are_intended_tool_object_contact(
            "forearm", "obj0", "obj0", &intended
        ));
        assert!(names_are_intended_tool_object_contact(
            "finger", "obj0", "obj0", &intended
        ));
    }

    #[test]
    fn declared_contact_bodies_exclude_forearm_ancestors() {
        use crate::embodiment::{unknown_se3, Body, EndEffector, Joint, JointKind, ModelFrame};
        use crate::provenance::Provenanced;
        use crate::resource::{
            ClosingDirection, CouplingModel, QualificationStatus, ResourceKind, ResourceTopology,
        };
        use crate::transform::Se3;

        let mut m = EmbodimentModel::new("uuid-emb", "src", "hash", "e0", "1");
        for (name, parent) in [
            ("base", None),
            ("forearm", Some("base")),
            ("hand", Some("forearm")),
            ("finger", Some("hand")),
        ] {
            m.bodies.push(Body {
                name: name.into(),
                parent: parent.map(str::to_string),
                mass_kg: Provenanced::unknown("t", 0.0),
                com: Provenanced::unknown("t", 0.0),
                inertia: Provenanced::unknown("t", 0.0),
                local_pose: Provenanced::declared(Se3::identity(), "t", 0.0),
            });
        }
        m.joints.push(Joint {
            name: "j_arm".into(),
            kind: JointKind::Hinge,
            axis: Provenanced::declared([0.0, 0.0, 1.0], "t", 0.0),
            qpos_dim: 1,
            dof_dim: 1,
            parent_body: "base".into(),
            child_body: "forearm".into(),
            q_min: Provenanced::unknown("t", 0.0),
            q_max: Provenanced::unknown("t", 0.0),
            dq_max: Provenanced::unknown("t", 0.0),
            effort_max: Provenanced::unknown("t", 0.0),
            origin_in_child: Provenanced::declared([0.0, 0.0, 0.0], "t", 0.0),
            parent_to_joint: unknown_se3("t"),
            joint_to_child: unknown_se3("t"),
            qpos_adr: Some(0),
            dof_adr: Some(0),
        });
        m.frames.push(ModelFrame {
            name: "ee".into(),
            kind: FrameKind::Ee,
            parent_body: "hand".into(),
            translation: Provenanced::declared([0.0, 0.0, 0.0], "t", 0.0),
            rotation: Provenanced::declared([1.0, 0.0, 0.0, 0.0], "t", 0.0),
        });
        m.end_effectors.push(EndEffector {
            name: "ee".into(),
            frame: "ee".into(),
            joint_chain: vec!["j_arm".into()],
        });
        let resource = ControlledResource {
            id: "g0".into(),
            kind: ResourceKind::Gripper,
            topology: ResourceTopology::DirectJointGripper,
            actuator_inputs: vec!["grip".into()],
            affected_joints: vec!["finger_j".into()],
            finger_bodies: vec!["finger".into()],
            coupling: CouplingModel::none(),
            command_coordinate: "opening".into(),
            opening_range: Provenanced::declared([0.0, 0.03], "t", 0.0),
            command_range: Provenanced::declared([0.0, 0.03], "t", 0.0),
            closing_direction: ClosingDirection::TowardMin,
            force_bound: Provenanced::unknown("t", 0.0),
            qualification: QualificationStatus::Qualified,
            unsupported_detail: None,
        };
        let bodies = declared_manipulation_contact_bodies(&m, Some(&resource), "ee");
        assert!(bodies.contains(&"finger".to_string()));
        assert!(bodies.contains(&"hand".to_string()), "semantic EE body");
        assert!(
            !bodies.contains(&"forearm".to_string()),
            "forearm ancestor is not declared contact geometry: {bodies:?}"
        );
    }
}
