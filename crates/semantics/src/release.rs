use crate::capability::{CapName, CapStatus, CapabilityGraph};
use crate::command::HoldSemantics;
use crate::embodiment::EmbodimentModel;
use crate::gripper_state::{GripperSemanticState, GripperStateEvidence};
use crate::observation::ObservationFrame;
use crate::plan::{SkillPlan, SkillStep};
use crate::resource::{lower_resource_opening, ControlledResource};
use crate::skill::{SkillContract, SkillName, SkillRefuse};

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
    pub fn release() -> Self {
        Self {
            id: "skill.release".into(),
            name: SkillName::Release,
            required: vec![CapName::GripperOpenClose],
            required_world: vec!["gripper_state".into()],
            success_evidence: vec![
                "gripper_opening_threshold".into(),
                "held_object_stops_following".into(),
                "no_unexpected_trapped_contact".into(),
            ],
            failure_evidence: vec![
                "RESOURCE_UNSUPPORTED".into(),
                "STALE_GRIPPER_STATE".into(),
                "BLOCKED".into(),
                "ACTUATOR_SATURATION".into(),
                "UNEXPECTED_CONTACT".into(),
            ],
            authority_ceiling: "allow".into(),
            exploration_allowance: false,
        }
    }
}

pub fn compile_release(
    model: &EmbodimentModel,
    caps: &CapabilityGraph,
    resource: &ControlledResource,
    gripper: &GripperStateEvidence,
    obs: &ObservationFrame,
    expected_model_hash: &str,
    now_s: f64,
    freshness_s: f64,
    command_lifetime_s: f64,
    required_opening: f64,
) -> Result<SkillPlan, SkillRefuse> {
    if expected_model_hash != model.model_hash {
        return Err(SkillRefuse::WrongModelHash);
    }
    if !resource.is_supported() {
        return Err(SkillRefuse::ResourceUnsupported);
    }
    if resource.command_range.value.is_none() {
        return Err(SkillRefuse::ModelFeatureUnsupported);
    }
    let resource_ok = resource.qualification == crate::resource::QualificationStatus::Qualified;
    if !resource_ok
        && !cap_usable(caps, CapName::GripperOpenClose)
        && !cap_usable(caps, CapName::ParallelGripper)
        && !cap_usable(caps, CapName::Grasping)
    {
        return Err(SkillRefuse::Unsupported);
    }
    if !gripper.fresh(now_s, freshness_s) {
        return Err(SkillRefuse::StaleGripperState);
    }
    if gripper.state == GripperSemanticState::Blocked {
        return Err(SkillRefuse::Blocked);
    }
    if obs.required_stale(now_s, freshness_s) {
        return Err(SkillRefuse::StaleEvidence);
    }
    let commands = lower_resource_opening(
        model,
        resource,
        required_opening.clamp(0.0, 1.0),
        "skill.release",
        HoldSemantics::KeepCurrent,
    )?;
    let mut plan = SkillPlan::empty(SkillName::Release, "skill.release");
    plan.resource_id = Some(resource.id.clone());
    plan.topology = Some(format!("{:?}", resource.topology));
    plan.success_evidence = SkillContract::release().success_evidence;
    plan.failure_evidence = SkillContract::release().failure_evidence;
    plan.steps.push(SkillStep::ResourceCommand {
        resource_id: resource.id.clone(),
        opening_01: required_opening.clamp(0.0, 1.0),
        commands,
        expires_at_s: now_s + command_lifetime_s,
        safe_on_expiry: "hold_last_opening".into(),
    });
    Ok(plan)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapter::zero_joint_obs;
    use crate::capability::derive_capabilities;
    use crate::embodiment::{EndEffector, Gripper};
    use crate::provenance::Provenanced;
    use crate::resource::{
        ClosingDirection, CouplingModel, QualificationStatus, ResourceKind, ResourceTopology,
    };

    fn model_and_resource() -> (EmbodimentModel, CapabilityGraph, ControlledResource) {
        let mut m = crate::adapter::synth_planar_two_link();
        m.grippers.push(Gripper {
            name: "g0".into(),
            actuator: "grip".into(),
            opening_range: Provenanced::declared([0.0, 0.03], "t", 0.0),
        });
        m.actuators.push(crate::embodiment::Actuator {
            name: "grip".into(),
            target_joint: "finger".into(),
            control_mode: "position".into(),
            transmission_kind: "joint".into(),
            ctrlrange: Provenanced::declared([0.0, 0.03], "t", 0.0),
            forcerange: Provenanced::unknown("t", 0.0),
            gear: Provenanced::unknown("t", 0.0),
        });
        m.end_effectors.push(EndEffector {
            name: "ee2".into(),
            frame: "ee".into(),
            joint_chain: vec!["j0".into()],
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
        (m, caps, r)
    }

    #[test]
    fn stale_gripper_refuses_with_zero_intent() {
        let (m, caps, r) = model_and_resource();
        let obs = zero_joint_obs(&m, "e0", 1.0);
        let g = GripperStateEvidence::unknown(0.0);
        let err = compile_release(&m, &caps, &r, &g, &obs, &m.model_hash, 1.0, 0.25, 0.4, 1.0)
            .unwrap_err();
        assert_eq!(err, SkillRefuse::StaleGripperState);
        assert!(!err.writes_allowed());
    }

    #[test]
    fn unsupported_resource_refuses() {
        let (m, caps, mut r) = model_and_resource();
        r.topology = ResourceTopology::UnsupportedResourceTopology;
        r.unsupported_detail = Some("network".into());
        let obs = zero_joint_obs(&m, "e0", 1.0);
        let mut g = GripperStateEvidence::unknown(1.0);
        g.stale = false;
        g.opening_01 = Some(0.2);
        g.timestamp_s = 1.0;
        let err = compile_release(&m, &caps, &r, &g, &obs, &m.model_hash, 1.0, 0.25, 0.4, 1.0)
            .unwrap_err();
        assert_eq!(err, SkillRefuse::ResourceUnsupported);
    }
}
