use crate::provenance::{Provenance, Provenanced};
use crate::transform::Se3;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum PerceptionKind {
    PerfectPerception,
    RobotSensor,
    Unknown,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum GeometryClass {
    Box,
    Cylinder,
    Asymmetric,
    Unknown,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ObjectGeometry {
    pub class: GeometryClass,
    pub bounds: Provenanced<[f64; 3]>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum GraspOccupancy {
    Free,
    Held,
    Unknown,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ObjectState {
    pub object_id: String,
    pub pose: Provenanced<Se3>,
    pub reference_frame: String,
    pub linear_velocity: Provenanced<[f64; 3]>,
    pub angular_velocity: Provenanced<[f64; 3]>,
    pub geometry: ObjectGeometry,
    pub mass: Provenanced<f64>,
    pub support_relation: Option<String>,
    pub contact_relations: Vec<String>,
    pub grasp_state: GraspOccupancy,
    pub provenance: Provenance,
    pub timestamp_s: f64,
    pub expires_at_s: f64,
}

impl ObjectState {
    pub fn unknown(object_id: impl Into<String>, now_s: f64) -> Self {
        Self {
            object_id: object_id.into(),
            pose: Provenanced::unknown("object.pose", now_s),
            reference_frame: String::new(),
            linear_velocity: Provenanced::unknown("object.v", now_s),
            angular_velocity: Provenanced::unknown("object.w", now_s),
            geometry: ObjectGeometry {
                class: GeometryClass::Unknown,
                bounds: Provenanced::unknown("object.bounds", now_s),
            },
            mass: Provenanced::unknown("object.mass", now_s),
            support_relation: None,
            contact_relations: Vec::new(),
            grasp_state: GraspOccupancy::Unknown,
            provenance: Provenance::Unknown,
            timestamp_s: now_s,
            expires_at_s: now_s,
        }
    }

    pub fn fresh(&self, now_s: f64, freshness_s: f64) -> bool {
        if now_s > self.expires_at_s {
            return false;
        }
        if now_s - self.timestamp_s > freshness_s {
            return false;
        }
        let Some(pose) = self.pose.value else {
            return false;
        };
        pose.xyz.iter().all(|v| v.is_finite())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unknown_object_has_no_invented_mass_or_class() {
        let o = ObjectState::unknown("obj", 1.0);
        assert!(o.mass.value.is_none());
        assert_eq!(o.geometry.class, GeometryClass::Unknown);
        assert!(!o.fresh(1.0, 0.25));
    }
}
