use crate::capability::{CapName, CapStatus, CapabilityGraph};
use crate::command::HoldSemantics;
use crate::embodiment::EmbodimentModel;
use crate::gripper_state::GripperStateEvidence;
use crate::interaction::{world_pose_of, InteractionFrameKind};
use crate::object::ObjectState;
use crate::observation::ObservationFrame;
use crate::plan::{SkillPlan, SkillStep, VerifyMotion};
use crate::provenance::Provenanced;
use crate::resource::{lower_resource_opening, ControlledResource};
use crate::skill::{SkillContract, SkillName, SkillRefuse};
use crate::transform::{Se3, TransformGraph};
use crate::world::PoseEvidence;

#[derive(Debug, Clone, PartialEq)]
pub struct GraspCandidate {
    pub object_id: String,
    pub approach: Se3,
    pub grasp: Se3,
    pub required_opening: f64,
    pub closing_axis: [f64; 3],
    pub verify_displacement: [f64; 3],
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
    pub fn grasp() -> Self {
        Self {
            id: "skill.grasp".into(),
            name: SkillName::Grasp,
            required: vec![
                CapName::CartesianPositionControl,
                CapName::GripperOpenClose,
            ],
            required_world: vec![
                "target_object".into(),
                "target_pose".into(),
                "candidate_grasp".into(),
            ],
            success_evidence: vec![
                "expected_finger_object_contact".into(),
                "closure_blocked_before_empty".into(),
                "object_follows_ee".into(),
                "support_relation_change".into(),
                "no_forbidden_contact".into(),
            ],
            failure_evidence: vec![
                "MISS".into(),
                "UNREACHABLE".into(),
                "OBJECT_MOVED".into(),
                "STALE_OBJECT".into(),
                "BLOCKED_APPROACH".into(),
                "GRIPPER_EMPTY_CLOSE".into(),
                "SLIP".into(),
                "EXCESS_FORCE".into(),
                "UNEXPECTED_CONTACT".into(),
                "RESOURCE_UNSUPPORTED".into(),
                "COUPLING_UNSUPPORTED".into(),
            ],
            authority_ceiling: "allow".into(),
            exploration_allowance: false,
        }
    }
}

fn pose_evidence(frame: &str, pose: Se3, now_s: f64, expires: f64) -> PoseEvidence {
    PoseEvidence {
        frame_id: frame.into(),
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
pub fn compile_grasp(
    model: &EmbodimentModel,
    caps: &CapabilityGraph,
    resource: &ControlledResource,
    object: &ObjectState,
    candidate: &GraspCandidate,
    gripper: &GripperStateEvidence,
    obs: &ObservationFrame,
    transforms: &TransformGraph,
    ee: &str,
    expected_model_hash: &str,
    now_s: f64,
    freshness_s: f64,
    command_lifetime_s: f64,
) -> Result<SkillPlan, SkillRefuse> {
    if expected_model_hash != model.model_hash {
        return Err(SkillRefuse::WrongModelHash);
    }
    if !resource.is_supported() {
        return Err(SkillRefuse::ResourceUnsupported);
    }
    if matches!(
        resource.topology,
        crate::resource::ResourceTopology::UnsupportedResourceTopology
    ) {
        return Err(SkillRefuse::ModelFeatureUnsupported);
    }
    if !object.fresh(now_s, freshness_s) {
        return Err(SkillRefuse::StaleObject);
    }
    if object.object_id != candidate.object_id {
        return Err(SkillRefuse::MissingTarget);
    }
    if !gripper.fresh(now_s, freshness_s) {
        return Err(SkillRefuse::StaleGripperState);
    }
    let cartesian_ok = cap_usable(caps, CapName::CartesianPositionControl);
    let joint_ok = cap_usable(caps, CapName::JointPositionControl);
    let chain_ok = model
        .ee_joint_chain(ee)
        .is_some_and(|c| !c.is_empty());
    if !(cartesian_ok || (joint_ok && chain_ok)) {
        return Err(SkillRefuse::Unsupported);
    }
    if resource.qualification != crate::resource::QualificationStatus::Qualified
        && !cap_usable(caps, CapName::GripperOpenClose)
        && !cap_usable(caps, CapName::ParallelGripper)
    {
        return Err(SkillRefuse::ResourceUnsupported);
    }
    if obs.required_stale(now_s, freshness_s) {
        return Err(SkillRefuse::StaleEvidence);
    }

    let approach_id = InteractionFrameKind::Approach.frame_id(&candidate.object_id);
    let grasp_id = InteractionFrameKind::Grasp.frame_id(&candidate.object_id);
    let approach_world = world_pose_of(transforms, &approach_id, now_s, freshness_s)
        .or_else(|_| Ok::<Se3, SkillRefuse>(candidate.approach))
        .map_err(|_| SkillRefuse::Unsupported)?;
    let grasp_world = world_pose_of(transforms, &grasp_id, now_s, freshness_s)
        .or_else(|_| Ok::<Se3, SkillRefuse>(candidate.grasp))
        .map_err(|_| SkillRefuse::Unsupported)?;

    let expires = now_s + freshness_s;
    let open_cmd = lower_resource_opening(
        model,
        resource,
        candidate.required_opening.clamp(0.0, 1.0),
        "skill.grasp",
        HoldSemantics::KeepCurrent,
    )?;
    let close_cmd = lower_resource_opening(
        model,
        resource,
        0.0,
        "skill.grasp",
        HoldSemantics::KeepCurrent,
    )?;

    let mut plan = SkillPlan::empty(SkillName::Grasp, "skill.grasp");
    plan.resource_id = Some(resource.id.clone());
    plan.topology = Some(format!("{:?}", resource.topology));
    plan.success_evidence = SkillContract::grasp().success_evidence;
    plan.failure_evidence = SkillContract::grasp().failure_evidence;
    plan.steps.push(SkillStep::ResourceCommand {
        resource_id: resource.id.clone(),
        opening_01: candidate.required_opening.clamp(0.0, 1.0),
        commands: open_cmd,
        expires_at_s: now_s + command_lifetime_s,
        safe_on_expiry: "hold_last_opening".into(),
    });
    plan.steps.push(SkillStep::Reach {
        end_effector: ee.into(),
        target: pose_evidence("world", approach_world, now_s, expires),
        success_radius: 0.08,
    });
    plan.steps.push(SkillStep::Reach {
        end_effector: ee.into(),
        target: pose_evidence("world", grasp_world, now_s, expires),
        success_radius: 0.06,
    });
    plan.steps.push(SkillStep::ResourceCommand {
        resource_id: resource.id.clone(),
        opening_01: 0.0,
        commands: close_cmd,
        expires_at_s: now_s + command_lifetime_s,
        safe_on_expiry: "hold_last_opening".into(),
    });
    let disp = candidate.verify_displacement;
    let mag = (disp[0] * disp[0] + disp[1] * disp[1] + disp[2] * disp[2]).sqrt();
    let displacement_xyz = if mag < 1e-4 {
        [0.0, 0.0, 0.03]
    } else {
        disp
    };
    plan.steps.push(SkillStep::VerifyMotion(VerifyMotion {
        displacement_xyz,
        bound_m: 0.05,
        frame: "world".into(),
    }));
    Ok(plan)
}

pub fn grasp_miss_recovery() -> Vec<SkillStep> {
    vec![SkillStep::OpenThenRetract, SkillStep::Stop]
}

pub fn grasp_slip_recovery() -> Vec<SkillStep> {
    vec![SkillStep::Stop]
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapter::{identity_base_graph, zero_joint_obs};
    use crate::capability::derive_capabilities;
    use crate::gripper_state::GripperSemanticState;
    use crate::object::{GeometryClass, GraspOccupancy, ObjectGeometry};
    use crate::provenance::Provenance;
    use crate::resource::{
        ClosingDirection, CouplingModel, QualificationStatus, ResourceKind, ResourceTopology,
    };

    fn setup() -> (
        EmbodimentModel,
        CapabilityGraph,
        ControlledResource,
        ObjectState,
        GraspCandidate,
        GripperStateEvidence,
        ObservationFrame,
        TransformGraph,
    ) {
        let mut m = crate::adapter::synth_planar_two_link();
        m.actuators.push(crate::embodiment::Actuator {
            name: "grip".into(),
            target_joint: "finger".into(),
            control_mode: "position".into(),
            transmission_kind: "joint".into(),
            ctrlrange: Provenanced::declared([0.0, 0.03], "t", 0.0),
            forcerange: Provenanced::unknown("t", 0.0),
            gear: Provenanced::unknown("t", 0.0),
        });
        let caps = derive_capabilities(&m, None);
        let r = ControlledResource {
            id: "g0".into(),
            kind: ResourceKind::Gripper,
            topology: ResourceTopology::DirectJointGripper,
            actuator_inputs: vec!["grip".into()],
            affected_joints: vec!["finger".into()],
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
        let pose = Se3::try_new([0.2, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]).unwrap();
        let object = ObjectState {
            object_id: "obj".into(),
            pose: Provenanced {
                value: Some(pose),
                provenance: Provenance::UserDeclared,
                source: "perfect_perception".into(),
                as_of_s: 1.0,
                uncertainty: None,
            },
            reference_frame: "world".into(),
            linear_velocity: Provenanced::unknown("v", 1.0),
            angular_velocity: Provenanced::unknown("w", 1.0),
            geometry: ObjectGeometry {
                class: GeometryClass::Box,
                bounds: Provenanced::declared([0.03, 0.03, 0.03], "t", 1.0),
            },
            mass: Provenanced::declared(0.05, "t", 1.0),
            support_relation: Some("table".into()),
            contact_relations: vec![],
            grasp_state: GraspOccupancy::Free,
            provenance: Provenance::UserDeclared,
            timestamp_s: 1.0,
            expires_at_s: 5.0,
        };
        let cand = GraspCandidate {
            object_id: "obj".into(),
            approach: Se3::try_new([0.18, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]).unwrap(),
            grasp: pose,
            required_opening: 0.8,
            closing_axis: [0.0, 1.0, 0.0],
            verify_displacement: [0.03, 0.0, 0.0],
            reference_frame: "world".into(),
        };
        let gripper = GripperStateEvidence {
            state: GripperSemanticState::Open,
            opening_01: Some(1.0),
            used: vec!["joint_or_command_opening".into()],
            timestamp_s: 1.0,
            stale: false,
        };
        let obs = zero_joint_obs(&m, "e0", 1.0);
        let g = identity_base_graph("e0");
        (m, caps, r, object, cand, gripper, obs, g)
    }

    #[test]
    fn stale_object_refuses() {
        let (m, caps, r, mut object, cand, gripper, obs, g) = setup();
        object.expires_at_s = 0.5;
        let err = compile_grasp(
            &m,
            &caps,
            &r,
            &object,
            &cand,
            &gripper,
            &obs,
            &g,
            "ee",
            &m.model_hash,
            1.0,
            0.25,
            0.4,
        )
        .unwrap_err();
        assert_eq!(err, SkillRefuse::StaleObject);
        assert!(!err.writes_allowed());
    }

    #[test]
    fn grasp_plan_uses_reach_steps() {
        let (m, caps, r, object, cand, gripper, obs, g) = setup();
        let plan = compile_grasp(
            &m,
            &caps,
            &r,
            &object,
            &cand,
            &gripper,
            &obs,
            &g,
            "ee",
            &m.model_hash,
            1.0,
            0.25,
            0.4,
        )
        .unwrap();
        assert!(plan
            .steps
            .iter()
            .any(|s| matches!(s, SkillStep::Reach { .. })));
        assert!(plan
            .steps
            .iter()
            .any(|s| matches!(s, SkillStep::ResourceCommand { opening_01: 0.0, .. })));
        assert!(plan
            .steps
            .iter()
            .any(|s| matches!(s, SkillStep::VerifyMotion(_))));
    }
}
