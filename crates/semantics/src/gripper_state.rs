use crate::resource::ControlledResource;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum GripperSemanticState {
    Open,
    ClosedEmpty,
    ClosedOnObject,
    PartiallyClosed,
    Moving,
    Blocked,
    Unknown,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GripperStateEvidence {
    pub state: GripperSemanticState,
    pub opening_01: Option<f64>,
    pub used: Vec<String>,
    pub timestamp_s: f64,
    pub stale: bool,
}

impl GripperStateEvidence {
    pub fn unknown(now_s: f64) -> Self {
        Self {
            state: GripperSemanticState::Unknown,
            opening_01: None,
            used: vec![],
            timestamp_s: now_s,
            stale: true,
        }
    }

    pub fn fresh(&self, now_s: f64, freshness_s: f64) -> bool {
        !self.stale && now_s - self.timestamp_s <= freshness_s && self.opening_01.is_some()
    }
}

pub fn derive_gripper_state(
    resource: &ControlledResource,
    opening_01: Option<f64>,
    opening_rate: Option<f64>,
    expected_finger_object_contact: Option<bool>,
    closure_blocked_before_empty: Option<bool>,
    now_s: f64,
) -> GripperStateEvidence {
    let mut used = Vec::new();
    let Some(open) = opening_01.filter(|v| v.is_finite()) else {
        return GripperStateEvidence::unknown(now_s);
    };
    used.push("joint_or_command_opening".into());
    if let Some(rate) = opening_rate {
        used.push("opening_rate".into());
        if rate.abs() > 0.05 {
            return GripperStateEvidence {
                state: GripperSemanticState::Moving,
                opening_01: Some(open),
                used,
                timestamp_s: now_s,
                stale: false,
            };
        }
    }
    let contact = expected_finger_object_contact;
    if contact.is_some() {
        used.push("finger_object_contact".into());
    }
    if closure_blocked_before_empty.is_some() {
        used.push("closure_blocked".into());
    }
    let open_thresh = 0.85;
    let closed_thresh = 0.12;
    let state = if open >= open_thresh {
        GripperSemanticState::Open
    } else if contact == Some(true) && closure_blocked_before_empty != Some(false) {
        GripperSemanticState::ClosedOnObject
    } else if open <= closed_thresh && contact != Some(true) {
        GripperSemanticState::ClosedEmpty
    } else if closure_blocked_before_empty == Some(true) && contact != Some(true) {
        GripperSemanticState::Blocked
    } else {
        GripperSemanticState::PartiallyClosed
    };
    let _ = resource;
    GripperStateEvidence {
        state,
        opening_01: Some(open),
        used,
        timestamp_s: now_s,
        stale: false,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::provenance::Provenanced;
    use crate::resource::{
        ClosingDirection, CouplingModel, QualificationStatus, ResourceKind, ResourceTopology,
    };

    fn res() -> ControlledResource {
        ControlledResource {
            id: "g".into(),
            kind: ResourceKind::Gripper,
            topology: ResourceTopology::DirectJointGripper,
            actuator_inputs: vec!["a".into()],
            affected_joints: vec!["j".into()],
            finger_bodies: vec!["f".into()],
            coupling: CouplingModel::none(),
            command_coordinate: "opening".into(),
            opening_range: Provenanced::declared([0.0, 1.0], "t", 0.0),
            command_range: Provenanced::declared([0.0, 1.0], "t", 0.0),
            closing_direction: ClosingDirection::TowardMin,
            force_bound: Provenanced::unknown("t", 0.0),
            qualification: QualificationStatus::Qualified,
            unsupported_detail: None,
        }
    }

    #[test]
    fn close_command_is_not_closed_on_object() {
        let e = derive_gripper_state(&res(), Some(0.05), Some(0.0), Some(false), Some(false), 1.0);
        assert_eq!(e.state, GripperSemanticState::ClosedEmpty);
        assert_ne!(e.state, GripperSemanticState::ClosedOnObject);
    }

    #[test]
    fn contact_plus_blocked_closure_is_held() {
        let e = derive_gripper_state(&res(), Some(0.4), Some(0.0), Some(true), Some(true), 1.0);
        assert_eq!(e.state, GripperSemanticState::ClosedOnObject);
        assert!(e.used.iter().any(|u| u.contains("contact")));
    }
}
