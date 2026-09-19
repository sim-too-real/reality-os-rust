use crate::capability::{CapName, CapStatus, CapabilityGraph};
use crate::embodiment::EmbodimentModel;
use crate::interaction::{world_pose_of, InteractionFrameKind};
use crate::object::ObjectState;
use crate::observation::ObservationFrame;
use crate::plan::{SkillPlan, SkillStep};
use crate::provenance::Provenanced;
use crate::skill::{SkillContract, SkillName, SkillRefuse};
use crate::transform::{Se3, TransformGraph};
use crate::world::PoseEvidence;

#[derive(Debug, Clone, PartialEq)]
pub struct PushCandidate {
    pub object_id: String,
    pub contact: Se3,
    pub approach: Se3,
    pub direction: [f64; 3],
    pub distance_m: f64,
    pub target_region: Option<[f64; 3]>,
    pub reference_frame: String,
}

fn cap_usable(caps: &CapabilityGraph, name: CapName) -> bool {
    caps.get_opt(name)
        .map(|n| {
            matches!(
                n.status,
                CapStatus::Proven | CapStatus::Supported | CapStatus::PartiallySupported
            )
        })
        .unwrap_or(false)
}

impl SkillContract {
    pub fn push() -> Self {
        Self {
            id: "skill.push".into(),
            name: SkillName::Push,
            required: vec![
                CapName::CartesianPositionControl,
                CapName::ContactManipulation,
            ],
            required_world: vec![
                "target_object".into(),
                "push_contact".into(),
                "push_direction".into(),
            ],
            success_evidence: vec![
                "object_displaced_along_direction".into(),
                "controlled_contact_established".into(),
            ],
            failure_evidence: vec![
                "MISS".into(),
                "SLIP_AROUND_OBJECT".into(),
                "OBJECT_NOT_MOVABLE".into(),
                "EXCESS_FORCE".into(),
                "UNEXPECTED_CONTACT".into(),
                "ROBOT_BLOCKED".into(),
                "TARGET_STALE".into(),
            ],
            authority_ceiling: "allow".into(),
            exploration_allowance: false,
        }
    }
}

fn pose_evidence(pose: Se3, now_s: f64, expires: f64) -> PoseEvidence {
    PoseEvidence {
        frame_id: "world".into(),
        pose: Provenanced {
            value: Some(pose),
            provenance: crate::provenance::Provenance::UserDeclared,
            source: "perfect_perception.scenario".into(),
            as_of_s: now_s,
            uncertainty: None,
        },
        expires_at_s: expires,
    }
}

#[allow(clippy::too_many_arguments)]
pub fn compile_push(
    model: &EmbodimentModel,
    caps: &CapabilityGraph,
    object: &ObjectState,
    candidate: &PushCandidate,
    obs: &ObservationFrame,
    transforms: &TransformGraph,
    ee: &str,
    expected_model_hash: &str,
    now_s: f64,
    freshness_s: f64,
) -> Result<SkillPlan, SkillRefuse> {
    if expected_model_hash != model.model_hash {
        return Err(SkillRefuse::WrongModelHash);
    }
    if !object.fresh(now_s, freshness_s) {
        return Err(SkillRefuse::StaleObject);
    }
    if object.object_id != candidate.object_id {
        return Err(SkillRefuse::MissingTarget);
    }
    let cartesian_ok = cap_usable(caps, CapName::CartesianPositionControl);
    let joint_ok = cap_usable(caps, CapName::JointPositionControl);
    let chain_ok = model.ee_joint_chain(ee).is_some_and(|c| !c.is_empty())
        || model
            .end_effectors
            .iter()
            .any(|e| !e.joint_chain.is_empty());
    if !(cartesian_ok || (joint_ok && chain_ok)) {
        if model.position_actuators().next().is_none() {
            return Err(SkillRefuse::MissingActuator);
        }
        return Err(SkillRefuse::Unsupported);
    }
    if !cap_usable(caps, CapName::ContactManipulation)
        && !cap_usable(caps, CapName::Pushing)
        && !cartesian_ok
    {
        return Err(SkillRefuse::Unsupported);
    }
    if obs.required_stale(now_s, freshness_s) {
        return Err(SkillRefuse::StaleEvidence);
    }
    if model.end_effectors.is_empty() {
        return Err(SkillRefuse::Unsupported);
    }

    let contact_id = InteractionFrameKind::PushContact.frame_id(&candidate.object_id);
    let approach_id = InteractionFrameKind::Approach.frame_id(&candidate.object_id);
    let contact =
        world_pose_of(transforms, &contact_id, now_s, freshness_s).unwrap_or(candidate.contact);
    let approach =
        world_pose_of(transforms, &approach_id, now_s, freshness_s).unwrap_or(candidate.approach);
    let n = (candidate.direction[0].powi(2)
        + candidate.direction[1].powi(2)
        + candidate.direction[2].powi(2))
    .sqrt();
    if n < 1e-8 {
        return Err(SkillRefuse::Unsupported);
    }
    let dir = [
        candidate.direction[0] / n,
        candidate.direction[1] / n,
        candidate.direction[2] / n,
    ];
    let stroke = effective_push_distance(candidate.distance_m);
    let end = Se3::try_new(
        [
            contact.xyz[0] + dir[0] * stroke,
            contact.xyz[1] + dir[1] * stroke,
            contact.xyz[2] + dir[2] * stroke,
        ],
        contact.quat_wxyz,
    )
    .map_err(|_| SkillRefuse::Unsupported)?;

    let expires = now_s + freshness_s;
    let mut plan = SkillPlan::empty(SkillName::Push, "skill.push");
    plan.success_evidence = SkillContract::push().success_evidence;
    plan.failure_evidence = SkillContract::push().failure_evidence;
    plan.steps.push(SkillStep::Reach {
        end_effector: ee.into(),
        target: pose_evidence(approach, now_s, expires),
        success_radius: 0.08,
    });
    plan.steps.push(SkillStep::Reach {
        end_effector: ee.into(),
        target: pose_evidence(contact, now_s, expires),
        success_radius: 0.06,
    });
    plan.steps.push(SkillStep::Reach {
        end_effector: ee.into(),
        target: pose_evidence(end, now_s, expires),
        success_radius: push_stroke_success_radius(stroke),
    });
    plan.steps.push(SkillStep::Stop);
    Ok(plan)
}

/// Floor a commanded push so the stroke is not smaller than contact slack.
/// Embodiment-agnostic: no robot-identity branch.
pub fn effective_push_distance(distance_m: f64) -> f64 {
    distance_m.max(0.02)
}

/// Final Reach must actually travel. A radius ≥ stroke lets the controller
/// succeed at the contact pose without displacing the object.
pub fn push_stroke_success_radius(distance_m: f64) -> f64 {
    (distance_m * 0.35).clamp(0.008, 0.03)
}

pub fn push_unexpected_recovery() -> Vec<SkillStep> {
    vec![SkillStep::Stop]
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapter::{identity_base_graph, zero_joint_obs};
    use crate::capability::derive_capabilities;
    use crate::object::{GeometryClass, GraspOccupancy, ObjectGeometry};
    use crate::provenance::Provenance;

    #[test]
    fn stale_target_refuses_push() {
        let m = crate::adapter::synth_planar_two_link();
        let caps = derive_capabilities(&m, None);
        let mut object = ObjectState::unknown("obj", 1.0);
        object.pose = Provenanced {
            value: Se3::try_new([0.2, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]).ok(),
            provenance: Provenance::UserDeclared,
            source: "perfect_perception".into(),
            as_of_s: 1.0,
            uncertainty: None,
        };
        object.reference_frame = "world".into();
        object.expires_at_s = 0.1;
        object.timestamp_s = 0.0;
        object.geometry = ObjectGeometry {
            class: GeometryClass::Box,
            bounds: Provenanced::declared([0.03, 0.03, 0.03], "t", 1.0),
        };
        object.grasp_state = GraspOccupancy::Free;
        let cand = PushCandidate {
            object_id: "obj".into(),
            contact: Se3::try_new([0.18, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]).unwrap(),
            approach: Se3::try_new([0.14, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]).unwrap(),
            direction: [1.0, 0.0, 0.0],
            distance_m: 0.05,
            target_region: None,
            reference_frame: "world".into(),
        };
        let obs = zero_joint_obs(&m, "e0", 1.0);
        let g = identity_base_graph("e0");
        let err = compile_push(
            &m,
            &caps,
            &object,
            &cand,
            &obs,
            &g,
            "ee",
            &m.model_hash,
            1.0,
            0.25,
        )
        .unwrap_err();
        assert_eq!(err, SkillRefuse::StaleObject);
    }

    #[test]
    fn stroke_success_radius_is_stricter_than_distance() {
        for d in [0.012, 0.02, 0.03, 0.05, 0.08] {
            let r = push_stroke_success_radius(effective_push_distance(d));
            assert!(
                r < effective_push_distance(d),
                "radius {r} must be < stroke {}",
                effective_push_distance(d)
            );
        }
        assert_eq!(effective_push_distance(0.005), 0.02);
        assert_eq!(effective_push_distance(0.05), 0.05);
    }
}
