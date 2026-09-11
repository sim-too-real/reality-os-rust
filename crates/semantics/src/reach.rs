use crate::adapter::{CompiledCtrl, ControlAdapter};
use crate::capability::{CapName, CapStatus, CapabilityGraph};
use crate::embodiment::EmbodimentModel;
use crate::observation::ObservationFrame;
use crate::skill::{SkillContract, SkillRefuse};
use crate::transform::TransformGraph;
use crate::world::WorldState;

fn cap_usable(caps: &CapabilityGraph, name: CapName) -> bool {
    matches!(
        caps.get(name).status,
        CapStatus::Proven | CapStatus::Supported | CapStatus::PartiallySupported
    )
}

#[allow(clippy::too_many_arguments)]
pub fn compile_reach(
    model: &EmbodimentModel,
    caps: &CapabilityGraph,
    world: &WorldState,
    obs: &ObservationFrame,
    transforms: &TransformGraph,
    expected_model_hash: &str,
    now_s: f64,
    freshness_s: f64,
    adapter: &dyn ControlAdapter,
) -> Result<CompiledCtrl, SkillRefuse> {
    if expected_model_hash != model.model_hash {
        return Err(SkillRefuse::WrongModelHash);
    }

    if world.transform_epoch != obs.transform_epoch {
        return Err(SkillRefuse::EpochMismatch);
    }

    if world.target_xyz().value.is_none() {
        return Err(SkillRefuse::MissingTarget);
    }

    if !world.target_fresh(now_s, freshness_s) || obs.required_stale(now_s, freshness_s) {
        return Err(SkillRefuse::StaleEvidence);
    }

    if obs.wrong_epoch(&world.transform_epoch) {
        return Err(SkillRefuse::EpochMismatch);
    }

    let cartesian_ok = cap_usable(caps, CapName::CartesianPositionControl);
    let joint_ok = cap_usable(caps, CapName::JointPositionControl);
    let chain = model.ee_joint_chain(world.end_effector());
    let chain_ok = chain.as_ref().is_some_and(|c| !c.is_empty());

    if let Some(chain) = &chain {
        if obs.missing_required(chain) {
            return Err(SkillRefuse::MissingJointState);
        }
    }

    if !(cartesian_ok || (joint_ok && chain_ok)) {
        if model.position_actuators().next().is_none() {
            return Err(SkillRefuse::MissingActuator);
        }
        return Err(SkillRefuse::Unsupported);
    }

    adapter.compile(
        &SkillContract::reach(),
        model,
        caps,
        world,
        obs,
        transforms,
        now_s,
        freshness_s,
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapter::{identity_base_graph, zero_joint_obs, ChainIkPositionPdAdapter};
    use crate::capability::derive_capabilities;
    use crate::provenance::Provenance;

    fn base() -> (
        EmbodimentModel,
        CapabilityGraph,
        WorldState,
        ObservationFrame,
        TransformGraph,
    ) {
        let m = crate::adapter::synth_planar_two_link();
        let caps = derive_capabilities(&m, None);
        let world = WorldState::empty("e0", 1.0).with_target(
            "ee",
            [0.2, 0.0, 0.0],
            5.0,
            "e0",
            1.0,
            Provenance::UserDeclared,
        );
        let obs = zero_joint_obs(&m, "e0", 1.0);
        let g = identity_base_graph("e0");
        (m, caps, world, obs, g)
    }

    #[test]
    fn missing_target_is_probe_path() {
        let (m, caps, _, obs, g) = base();
        let world = WorldState::empty("e0", 1.0);
        let err = compile_reach(
            &m,
            &caps,
            &world,
            &obs,
            &g,
            &m.model_hash,
            1.0,
            0.25,
            &ChainIkPositionPdAdapter,
        )
        .unwrap_err();
        assert_eq!(err, SkillRefuse::MissingTarget);
    }

    #[test]
    fn stale_target_refuses() {
        let (m, caps, mut world, obs, g) = base();
        world.goal.target.expires_at_s = 0.5;
        let err = compile_reach(
            &m,
            &caps,
            &world,
            &obs,
            &g,
            &m.model_hash,
            1.0,
            0.25,
            &ChainIkPositionPdAdapter,
        )
        .unwrap_err();
        assert_eq!(err, SkillRefuse::StaleEvidence);
    }

    #[test]
    fn wrong_hash_refuses() {
        let (m, caps, world, obs, g) = base();
        let err = compile_reach(
            &m,
            &caps,
            &world,
            &obs,
            &g,
            "other",
            1.0,
            0.25,
            &ChainIkPositionPdAdapter,
        )
        .unwrap_err();
        assert_eq!(err, SkillRefuse::WrongModelHash);
    }

    #[test]
    fn epoch_mismatch_refuses() {
        let (m, caps, mut world, obs, g) = base();
        world.transform_epoch = "e1".into();
        let err = compile_reach(
            &m,
            &caps,
            &world,
            &obs,
            &g,
            &m.model_hash,
            1.0,
            0.25,
            &ChainIkPositionPdAdapter,
        )
        .unwrap_err();
        assert_eq!(err, SkillRefuse::EpochMismatch);
    }

    #[test]
    fn missing_actuator_refuses() {
        let (mut m, _, world, obs, g) = base();
        m.actuators.clear();
        let caps = derive_capabilities(&m, None);
        let err = compile_reach(
            &m,
            &caps,
            &world,
            &obs,
            &g,
            &m.model_hash,
            1.0,
            0.25,
            &ChainIkPositionPdAdapter,
        )
        .unwrap_err();
        assert_eq!(err, SkillRefuse::MissingActuator);
    }

    #[test]
    fn missing_joint_state_is_probe() {
        let (m, caps, world, mut obs, g) = base();
        obs.joint_state.clear();
        let err = compile_reach(
            &m,
            &caps,
            &world,
            &obs,
            &g,
            &m.model_hash,
            1.0,
            0.25,
            &ChainIkPositionPdAdapter,
        )
        .unwrap_err();
        assert_eq!(err, SkillRefuse::MissingJointState);
    }
}
