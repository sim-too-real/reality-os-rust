use crate::provenance::{Provenance, Provenanced};
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
}
