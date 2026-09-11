use crate::capability::{CapName, CapStatus, CapabilityGraph};
use crate::command::{HoldSemantics, IkTrace, JointTarget, JointTargetSet};
use crate::embodiment::{EmbodimentModel, JointKind};
use crate::kinematics::{resolve_chain_joints, solve_ik};
use crate::observation::ObservationFrame;
use crate::skill::{SkillContract, SkillName, SkillRefuse};
use crate::transform::{Se3, TransformError, TransformGraph};
use crate::world::WorldState;

#[derive(Debug, Clone, PartialEq)]
pub struct CompiledCtrl {
    pub action: Vec<f64>,
    pub targets: JointTargetSet,
    pub ik: Option<IkTrace>,
    pub control_mode: String,
    pub adapter_id: String,
    pub adapter_version: String,
}

pub struct AdapterContract {
    pub input: &'static str,
    pub output_mode: &'static str,
    pub required_caps: &'static [CapName],
    pub stop: &'static str,
}

pub trait ControlAdapter {
    fn advertise(&self) -> AdapterContract;
    fn compile(
        &self,
        skill: &SkillContract,
        model: &EmbodimentModel,
        caps: &CapabilityGraph,
        world: &WorldState,
        obs: &ObservationFrame,
        transforms: &TransformGraph,
        now_s: f64,
        freshness_s: f64,
    ) -> Result<CompiledCtrl, SkillRefuse>;
}

pub struct ChainIkPositionPdAdapter;

impl ControlAdapter for ChainIkPositionPdAdapter {
    fn advertise(&self) -> AdapterContract {
        AdapterContract {
            input: "ee_xyz",
            output_mode: "position",
            required_caps: &[CapName::JointPositionControl],
            stop: "hold_last_position",
        }
    }

    fn compile(
        &self,
        skill: &SkillContract,
        model: &EmbodimentModel,
        caps: &CapabilityGraph,
        world: &WorldState,
        obs: &ObservationFrame,
        transforms: &TransformGraph,
        now_s: f64,
        freshness_s: f64,
    ) -> Result<CompiledCtrl, SkillRefuse> {
        if skill.name != SkillName::Reach {
            return Err(SkillRefuse::Unsupported);
        }

        if !joint_position_usable(caps) {
            return Err(SkillRefuse::Unsupported);
        }

        let ee = world.end_effector();
        let chain = model
            .ee_joint_chain(ee)
            .filter(|c| !c.is_empty())
            .ok_or(SkillRefuse::Unsupported)?;

        let joints = resolve_chain_joints(model, &chain)?;
        let current_q = current_chain_q(obs, &chain, &world.transform_epoch, now_s, freshness_s)?;

        let target_xyz = world
            .target_xyz()
            .value
            .filter(|t| t.iter().all(|v| v.is_finite()))
            .ok_or(SkillRefuse::MissingTarget)?;

        let ik_target = transforms
            .resolve(world.target_frame(), "world", now_s, Some(freshness_s))
            .or_else(|e| match e {
                TransformError::Disconnected if world.target_frame() == "world" => {
                    Ok(Se3::identity())
                }
                TransformError::Stale | TransformError::EpochMismatch => {
                    Err(SkillRefuse::StaleEvidence)
                }
                _ => Err(SkillRefuse::Unsupported),
            })?
            .transform_point(target_xyz);

        let (q, trace) = solve_ik(model, &chain, ee, ik_target, &current_q)?;
        if trace.residual > world.goal.success_radius.max(1e-4) {
            return Err(SkillRefuse::Unreachable);
        }

        let mut targets = Vec::new();
        for (joint, qi) in joints.iter().zip(q.iter()) {
            if joint.kind == JointKind::Fixed {
                continue;
            }
            targets.push(JointTarget {
                joint_name: joint.name.clone(),
                actuator_name: model
                    .actuator_for_joint(&joint.name)
                    .map(|a| a.name.clone()),
                value: *qi,
                control_mode: "position".into(),
                skill_id: skill.id.clone(),
            });
        }

        let action = targets.iter().map(|t| t.value).collect();
        Ok(CompiledCtrl {
            action,
            targets: JointTargetSet {
                targets,
                hold_outside: HoldSemantics::KeepCurrent,
            },
            ik: Some(trace),
            control_mode: "position".into(),
            adapter_id: "chain_ik_position_pd".into(),
            adapter_version: "2".into(),
        })
    }
}

fn joint_position_usable(caps: &CapabilityGraph) -> bool {
    matches!(
        caps.get(CapName::JointPositionControl).status,
        CapStatus::Proven | CapStatus::Supported | CapStatus::PartiallySupported
    )
}

fn current_chain_q(
    obs: &ObservationFrame,
    chain: &[String],
    epoch: &str,
    now_s: f64,
    freshness_s: f64,
) -> Result<Vec<f64>, SkillRefuse> {
    let mut q = Vec::with_capacity(chain.len());
    for name in chain {
        let sample = obs
            .joint_state
            .iter()
            .find(|s| s.joint_name == *name)
            .ok_or(SkillRefuse::MissingJointState)?;
        if sample.transform_epoch != epoch {
            return Err(SkillRefuse::EpochMismatch);
        }
        if sample.stale(now_s, freshness_s) {
            return Err(SkillRefuse::StaleEvidence);
        }
        if !sample.q.is_finite() {
            return Err(SkillRefuse::MissingJointState);
        }
        q.push(sample.q);
    }
    Ok(q)
}

pub fn lower_named_targets(
    model: &EmbodimentModel,
    targets: &JointTargetSet,
    current_by_joint: &std::collections::HashMap<String, f64>,
) -> Result<Vec<(String, f64)>, SkillRefuse> {
    let mut out = Vec::new();
    for act in &model.actuators {
        if let Some(t) = targets.targets.iter().find(|t| {
            t.joint_name == act.target_joint
                || t.actuator_name.as_deref() == Some(act.name.as_str())
        }) {
            out.push((act.name.clone(), t.value));
        } else {
            match targets.hold_outside {
                HoldSemantics::KeepCurrent | HoldSemantics::ExplicitSafe => {
                    let hold = current_by_joint
                        .get(&act.target_joint)
                        .copied()
                        .ok_or(SkillRefuse::MissingJointState)?;
                    out.push((act.name.clone(), hold));
                }
            }
        }
    }
    Ok(out)
}

#[cfg(test)]
use crate::embodiment::{unknown_se3, Actuator, Body, EndEffector, FrameKind, Joint, ModelFrame};
#[cfg(test)]
use crate::observation::JointStateSample;
#[cfg(test)]
use crate::provenance::Provenanced;

#[cfg(test)]
fn declared_pose(xyz: [f64; 3]) -> (Provenanced<[f64; 3]>, Provenanced<[f64; 4]>) {
    (
        Provenanced::declared(xyz, "test", 0.0),
        Provenanced::declared([1.0, 0.0, 0.0, 0.0], "test", 0.0),
    )
}

#[cfg(test)]
pub(crate) fn synth_planar_two_link() -> EmbodimentModel {
    const L1: f64 = 0.15;
    const L2: f64 = 0.15;

    let mut m = EmbodimentModel::new("synth_planar", "test", "hash", "epoch0", "1");
    m.bodies.push(Body {
        name: "base".into(),
        parent: None,
        mass_kg: Provenanced::unknown("test", 0.0),
        com: Provenanced::unknown("test", 0.0),
        inertia: Provenanced::unknown("test", 0.0),
        local_pose: Provenanced::declared(Se3::identity(), "test", 0.0),
    });
    m.bodies.push(Body {
        name: "link1".into(),
        parent: Some("base".into()),
        mass_kg: Provenanced::unknown("test", 0.0),
        com: Provenanced::unknown("test", 0.0),
        inertia: Provenanced::unknown("test", 0.0),
        local_pose: Provenanced::declared(Se3::identity(), "test", 0.0),
    });
    m.bodies.push(Body {
        name: "link2".into(),
        parent: Some("link1".into()),
        mass_kg: Provenanced::unknown("test", 0.0),
        com: Provenanced::unknown("test", 0.0),
        inertia: Provenanced::unknown("test", 0.0),
        local_pose: Provenanced::declared(Se3::translation([L1, 0.0, 0.0]).unwrap(), "test", 0.0),
    });

    let hinge_z = Provenanced::declared([0.0, 0.0, 1.0], "test", 0.0);
    m.joints.push(Joint {
        name: "j0".into(),
        kind: JointKind::Hinge,
        axis: hinge_z.clone(),
        qpos_dim: 1,
        dof_dim: 1,
        parent_body: "base".into(),
        child_body: "link1".into(),
        q_min: Provenanced::unknown("test", 0.0),
        q_max: Provenanced::unknown("test", 0.0),
        dq_max: Provenanced::unknown("test", 0.0),
        effort_max: Provenanced::unknown("test", 0.0),
        origin_in_child: Provenanced::declared([0.0, 0.0, 0.0], "test", 0.0),
        parent_to_joint: unknown_se3("test"),
        joint_to_child: unknown_se3("test"),
        qpos_adr: Some(0),
        dof_adr: Some(0),
    });
    m.joints.push(Joint {
        name: "j1".into(),
        kind: JointKind::Hinge,
        axis: hinge_z,
        qpos_dim: 1,
        dof_dim: 1,
        parent_body: "link1".into(),
        child_body: "link2".into(),
        q_min: Provenanced::unknown("test", 0.0),
        q_max: Provenanced::unknown("test", 0.0),
        dq_max: Provenanced::unknown("test", 0.0),
        effort_max: Provenanced::unknown("test", 0.0),
        origin_in_child: Provenanced::declared([0.0, 0.0, 0.0], "test", 0.0),
        parent_to_joint: unknown_se3("test"),
        joint_to_child: unknown_se3("test"),
        qpos_adr: Some(1),
        dof_adr: Some(1),
    });

    m.actuators.push(Actuator {
        name: "a0".into(),
        target_joint: "j0".into(),
        control_mode: "position".into(),
        ctrlrange: Provenanced::unknown("test", 0.0),
        forcerange: Provenanced::unknown("test", 0.0),
        gear: Provenanced::unknown("test", 0.0),
    });
    m.actuators.push(Actuator {
        name: "a1".into(),
        target_joint: "j1".into(),
        control_mode: "position".into(),
        ctrlrange: Provenanced::unknown("test", 0.0),
        forcerange: Provenanced::unknown("test", 0.0),
        gear: Provenanced::unknown("test", 0.0),
    });

    let (t0, r0) = declared_pose([L2, 0.0, 0.0]);
    m.frames.push(ModelFrame {
        name: "ee".into(),
        kind: FrameKind::Ee,
        parent_body: "link2".into(),
        translation: t0,
        rotation: r0,
    });

    m.end_effectors.push(EndEffector {
        name: "ee".into(),
        frame: "ee".into(),
        joint_chain: vec!["j0".into(), "j1".into()],
    });

    m
}

#[cfg(test)]
pub(crate) fn zero_joint_obs(model: &EmbodimentModel, epoch: &str, now_s: f64) -> ObservationFrame {
    ObservationFrame {
        frame_id: "f".into(),
        transform_epoch: epoch.into(),
        observations: vec![],
        joint_state: model
            .joints
            .iter()
            .filter(|j| j.kind != JointKind::Fixed)
            .map(|j| {
                JointStateSample::new(&j.name, 0.0, Some(0.0), now_s, now_s, "enc", "cal", epoch)
            })
            .collect(),
        as_of_s: now_s,
    }
}

#[cfg(test)]
pub(crate) fn identity_base_graph(epoch: &str) -> TransformGraph {
    let mut g = TransformGraph::new(epoch);
    g.insert(crate::transform::TransformEdge::identity(
        "world", "base", epoch, 1.0,
    ))
    .unwrap();
    g
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::capability::derive_capabilities;
    use crate::provenance::Provenance;

    #[test]
    fn adapter_selected_by_caps_not_name() {
        let m = synth_planar_two_link();
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
        let out = ChainIkPositionPdAdapter
            .compile(
                &SkillContract::reach(),
                &m,
                &caps,
                &world,
                &obs,
                &g,
                1.0,
                0.25,
            )
            .unwrap();
        assert_eq!(out.control_mode, "position");
        assert_eq!(out.targets.targets.len(), 2);
        assert!(out.action.iter().all(|x| x.is_finite()));
        assert!(!out.targets.contains_joint("gripper"));
    }

    #[test]
    fn stretched_planar_chain_ik_nonzero_from_bent_seed() {
        let m = synth_planar_two_link();
        let caps = derive_capabilities(&m, None);
        let world = WorldState::empty("e0", 1.0).with_target(
            "ee",
            [0.15, 0.12, 0.0],
            5.0,
            "e0",
            1.0,
            Provenance::UserDeclared,
        );
        let mut obs = zero_joint_obs(&m, "e0", 1.0);
        obs.joint_state[0].q = 0.2;
        obs.joint_state[1].q = 0.4;
        let g = identity_base_graph("e0");
        let out = ChainIkPositionPdAdapter
            .compile(
                &SkillContract::reach(),
                &m,
                &caps,
                &world,
                &obs,
                &g,
                1.0,
                0.25,
            )
            .unwrap();
        assert!(out.action.iter().all(|x| x.is_finite()));
        assert!(
            out.action.iter().any(|x| x.abs() > 1e-6),
            "expected non-zero joint action, got {:?}",
            out.action
        );
        let ik = out.ik.expect("ik trace");
        assert_eq!(ik.initial_q, vec![0.2, 0.4]);
    }

    #[test]
    fn unknown_axis_is_unreachable_not_invented() {
        let mut m = synth_planar_two_link();
        m.joints[0].axis = Provenanced::unknown("bundle", 0.0);
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
        let err = ChainIkPositionPdAdapter
            .compile(
                &SkillContract::reach(),
                &m,
                &caps,
                &world,
                &obs,
                &g,
                1.0,
                0.25,
            )
            .unwrap_err();
        assert_eq!(err, SkillRefuse::Unreachable);
    }

    #[test]
    fn unknown_ee_translation_is_refused_not_invented() {
        let mut m = synth_planar_two_link();
        let ee_frame = m
            .frames
            .iter()
            .position(|f| f.name == "ee")
            .expect("ee frame");
        m.frames[ee_frame].translation = Provenanced::unknown("bundle", 0.0);
        m.frames[ee_frame].rotation = Provenanced::unknown("bundle", 0.0);
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
        let err = ChainIkPositionPdAdapter
            .compile(
                &SkillContract::reach(),
                &m,
                &caps,
                &world,
                &obs,
                &g,
                1.0,
                0.25,
            )
            .unwrap_err();
        assert!(matches!(
            err,
            SkillRefuse::Unsupported | SkillRefuse::Unreachable
        ));
    }

    #[test]
    fn extra_gripper_actuator_is_not_in_reach_targets() {
        let mut m = synth_planar_two_link();
        m.actuators.push(Actuator {
            name: "grip".into(),
            target_joint: "finger".into(),
            control_mode: "position".into(),
            ctrlrange: Provenanced::unknown("test", 0.0),
            forcerange: Provenanced::unknown("test", 0.0),
            gear: Provenanced::unknown("test", 0.0),
        });
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
        let out = ChainIkPositionPdAdapter
            .compile(
                &SkillContract::reach(),
                &m,
                &caps,
                &world,
                &obs,
                &g,
                1.0,
                0.25,
            )
            .unwrap();
        assert!(!out.targets.contains_joint("finger"));
        assert_eq!(out.targets.targets.len(), 2);
    }

    #[test]
    fn actuator_reorder_preserves_named_targets() {
        let mut m = synth_planar_two_link();
        m.actuators.swap(0, 1);
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
        let out = ChainIkPositionPdAdapter
            .compile(
                &SkillContract::reach(),
                &m,
                &caps,
                &world,
                &obs,
                &g,
                1.0,
                0.25,
            )
            .unwrap();
        assert!(out.targets.contains_joint("j0"));
        assert!(out.targets.contains_joint("j1"));
        let mut current = std::collections::HashMap::new();
        current.insert("j0".into(), 0.0);
        current.insert("j1".into(), 0.0);
        let lowered = lower_named_targets(&m, &out.targets, &current).unwrap();
        assert_eq!(lowered[0].0, "a1");
        assert_eq!(lowered[1].0, "a0");
    }

    #[test]
    fn missing_joint_state_probes() {
        let m = synth_planar_two_link();
        let caps = derive_capabilities(&m, None);
        let world = WorldState::empty("e0", 1.0).with_target(
            "ee",
            [0.2, 0.0, 0.0],
            5.0,
            "e0",
            1.0,
            Provenance::UserDeclared,
        );
        let mut obs = zero_joint_obs(&m, "e0", 1.0);
        obs.joint_state.clear();
        let g = identity_base_graph("e0");
        let err = ChainIkPositionPdAdapter
            .compile(
                &SkillContract::reach(),
                &m,
                &caps,
                &world,
                &obs,
                &g,
                1.0,
                0.25,
            )
            .unwrap_err();
        assert_eq!(err, SkillRefuse::MissingJointState);
    }
}
