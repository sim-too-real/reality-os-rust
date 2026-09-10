use crate::adapter::{CompiledCtrl, ControlAdapter};
use crate::capability::{CapName, CapStatus, CapabilityGraph};
use crate::embodiment::EmbodimentModel;
use crate::observation::ObservationFrame;
use crate::skill::{SkillContract, SkillRefuse};
use crate::world::WorldState;

fn cap_usable(caps: &CapabilityGraph, name: CapName) -> bool {
    matches!(
        caps.get(name).status,
        CapStatus::Proven | CapStatus::Supported | CapStatus::PartiallySupported
    )
}

pub fn compile_reach(
    model: &EmbodimentModel,
    caps: &CapabilityGraph,
    world: &WorldState,
    obs: &ObservationFrame,
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

    if world.target_xyz.value.is_none() {
        return Err(SkillRefuse::MissingTarget);
    }

    if !world.target_fresh(now_s, freshness_s) || obs.required_stale(now_s, freshness_s) {
        return Err(SkillRefuse::StaleEvidence);
    }

    let cartesian_ok = cap_usable(caps, CapName::CartesianPositionControl);
    let joint_ok = cap_usable(caps, CapName::JointPositionControl);
    let chain_ok = model.ee_joint_chain(&world.target_frame).is_some();

    if !(cartesian_ok || (joint_ok && chain_ok)) {
        if model.position_actuators().next().is_none() {
            return Err(SkillRefuse::MissingActuator);
        }
        return Err(SkillRefuse::Unsupported);
    }

    adapter.compile(&SkillContract::reach(), model, caps, world, obs)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapter::ChainIkPositionPdAdapter;
    use crate::capability::derive_capabilities;
    use crate::provenance::Provenance;

    fn base() -> (
        EmbodimentModel,
        CapabilityGraph,
        WorldState,
        ObservationFrame,
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
        let obs = ObservationFrame {
            frame_id: "f".into(),
            transform_epoch: "e0".into(),
            observations: vec![],
            as_of_s: 1.0,
        };
        (m, caps, world, obs)
    }

    #[test]
    fn missing_target_is_probe_path() {
        let (m, caps, _, obs) = base();
        let world = WorldState::empty("e0", 1.0);
        let err = compile_reach(
            &m,
            &caps,
            &world,
            &obs,
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
        let (m, caps, mut world, obs) = base();
        world.target_expires_at_s = 0.5;
        let err = compile_reach(
            &m,
            &caps,
            &world,
            &obs,
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
        let (m, caps, world, obs) = base();
        let err = compile_reach(
            &m,
            &caps,
            &world,
            &obs,
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
        let (m, caps, mut world, obs) = base();
        world.transform_epoch = "e1".into();
        let err = compile_reach(
            &m,
            &caps,
            &world,
            &obs,
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
        let (mut m, _, world, obs) = base();
        m.actuators.clear();
        let caps = derive_capabilities(&m, None);
        let err = compile_reach(
            &m,
            &caps,
            &world,
            &obs,
            &m.model_hash,
            1.0,
            0.25,
            &ChainIkPositionPdAdapter,
        )
        .unwrap_err();
        assert_eq!(err, SkillRefuse::MissingActuator);
    }
}
