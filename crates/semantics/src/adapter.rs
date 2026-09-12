use crate::capability::{CapName, CapStatus, CapabilityGraph};
use crate::command::{ActuatorCommandSet, HoldSemantics, IkTrace, JointTarget, JointTargetSet};
use crate::embodiment::{Actuator, EmbodimentModel, JointKind};
use crate::kinematics::{resolve_chain_joints, solve_ik};
use crate::observation::ObservationFrame;
use crate::skill::{SkillContract, SkillName, SkillRefuse};
use crate::transform::{Se3, TransformError, TransformGraph};
use crate::world::WorldState;
use std::collections::{BTreeMap, HashMap, HashSet};

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
                explicit_safe: std::collections::BTreeMap::new(),
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

fn finite_or_invalid(v: f64) -> Result<f64, SkillRefuse> {
    if v.is_finite() {
        Ok(v)
    } else {
        Err(SkillRefuse::InvalidCommand)
    }
}

fn observed_for(act: &Actuator, current: &HashMap<String, f64>) -> Option<f64> {
    current
        .get(&act.name)
        .or_else(|| current.get(&act.target_joint))
        .copied()
        .filter(|v| v.is_finite())
}

fn hold_value(
    hold: HoldSemantics,
    explicit_safe: &BTreeMap<String, f64>,
    act: &Actuator,
    current: &HashMap<String, f64>,
) -> Result<f64, SkillRefuse> {
    match hold {
        HoldSemantics::KeepCurrent => {
            observed_for(act, current).ok_or(SkillRefuse::MissingJointState)
        }
        HoldSemantics::ExplicitSafe => explicit_safe
            .get(&act.name)
            .copied()
            .ok_or(SkillRefuse::InvalidCommand)
            .and_then(finite_or_invalid),
    }
}

fn bind_joint_target<'a>(
    model: &'a EmbodimentModel,
    t: &JointTarget,
) -> Result<&'a Actuator, SkillRefuse> {
    finite_or_invalid(t.value)?;
    let hits: Vec<&Actuator> = model
        .actuators
        .iter()
        .filter(|act| {
            t.joint_name == act.target_joint
                || t.actuator_name.as_deref() == Some(act.name.as_str())
        })
        .collect();
    match hits.as_slice() {
        [act] => {
            if act.control_mode != t.control_mode {
                return Err(SkillRefuse::InvalidCommand);
            }
            Ok(*act)
        }
        [] => Err(SkillRefuse::MissingActuator),
        _ => Err(SkillRefuse::InvalidCommand),
    }
}

fn fill_holds(
    model: &EmbodimentModel,
    requested: HashMap<&str, f64>,
    hold: HoldSemantics,
    explicit_safe: &BTreeMap<String, f64>,
    current: &HashMap<String, f64>,
) -> Result<Vec<(String, f64)>, SkillRefuse> {
    let mut out = Vec::with_capacity(model.actuators.len());
    for act in &model.actuators {
        let v = if let Some(v) = requested.get(act.name.as_str()) {
            *v
        } else {
            hold_value(hold, explicit_safe, act, current)?
        };
        out.push((act.name.clone(), v));
    }
    if out.is_empty() {
        return Err(SkillRefuse::MissingActuator);
    }
    Ok(out)
}

pub fn lower_actuator_commands(
    model: &EmbodimentModel,
    commands: &ActuatorCommandSet,
    current_by_name: &HashMap<String, f64>,
) -> Result<Vec<(String, f64)>, SkillRefuse> {
    let mut requested = HashMap::new();
    let mut seen = HashSet::new();
    for c in &commands.commands {
        let v = finite_or_invalid(c.value)?;
        let act = model
            .actuators
            .iter()
            .find(|a| a.name == c.actuator_name)
            .ok_or(SkillRefuse::MissingActuator)?;
        if !seen.insert(c.actuator_name.as_str()) {
            return Err(SkillRefuse::InvalidCommand);
        }
        if act.control_mode != c.control_mode {
            return Err(SkillRefuse::InvalidCommand);
        }
        requested.insert(act.name.as_str(), v);
    }
    fill_holds(
        model,
        requested,
        commands.hold_outside,
        &commands.explicit_safe,
        current_by_name,
    )
}

pub fn lower_named_targets(
    model: &EmbodimentModel,
    targets: &JointTargetSet,
    current_by_joint: &HashMap<String, f64>,
) -> Result<Vec<(String, f64)>, SkillRefuse> {
    let mut requested = HashMap::new();
    for t in &targets.targets {
        let act = bind_joint_target(model, t)?;
        if requested.insert(act.name.as_str(), t.value).is_some() {
            return Err(SkillRefuse::InvalidCommand);
        }
    }
    fill_holds(
        model,
        requested,
        targets.hold_outside,
        &targets.explicit_safe,
        current_by_joint,
    )
}

#[cfg(test)]
use crate::embodiment::{unknown_se3, Body, EndEffector, FrameKind, Joint, ModelFrame};
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
        transmission_kind: "joint".into(),
        ctrlrange: Provenanced::unknown("test", 0.0),
        forcerange: Provenanced::unknown("test", 0.0),
        gear: Provenanced::unknown("test", 0.0),
    });
    m.actuators.push(Actuator {
        name: "a1".into(),
        target_joint: "j1".into(),
        control_mode: "position".into(),
        transmission_kind: "joint".into(),
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
    use crate::command::{ActuatorCommand, ActuatorCommandSet};
    use crate::provenance::Provenance;

    fn cmd(name: &str, value: f64, mode: &str) -> ActuatorCommand {
        ActuatorCommand {
            actuator_name: name.into(),
            value,
            control_mode: mode.into(),
            skill_id: "t".into(),
        }
    }

    fn current_complete() -> std::collections::HashMap<String, f64> {
        let mut m = std::collections::HashMap::new();
        m.insert("j0".into(), 0.11);
        m.insert("j1".into(), 0.22);
        m.insert("a0".into(), 0.11);
        m.insert("a1".into(), 0.22);
        m
    }

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
        assert_eq!(err, SkillRefuse::ModelFeatureUnsupported);
    }

    #[test]
    fn extra_gripper_actuator_is_not_in_reach_targets() {
        let mut m = synth_planar_two_link();
        m.actuators.push(Actuator {
            name: "grip".into(),
            target_joint: "finger".into(),
            control_mode: "position".into(),
            transmission_kind: "joint".into(),
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
    fn reach_compiles_without_commanding_tendon_actuator() {
        let mut m = synth_planar_two_link();
        m.actuators.push(Actuator {
            name: "split".into(),
            target_joint: "split".into(),
            control_mode: "position".into(),
            transmission_kind: "tendon".into(),
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
            .expect("arm REACH must compile without the tendon actuator");
        assert!(!out.targets.contains_joint("split"));
        assert_eq!(out.targets.targets.len(), 2);
    }

    #[test]
    fn keep_current_holds_exact_measured_on_omitted_actuator() {
        let model = synth_planar_two_link();
        let set = ActuatorCommandSet {
            commands: vec![cmd("a0", 0.4, "position")],
            hold_outside: HoldSemantics::KeepCurrent,
            explicit_safe: Default::default(),
        };
        let out = lower_actuator_commands(&model, &set, &current_complete()).unwrap();
        assert_eq!(out, vec![("a0".into(), 0.4), ("a1".into(), 0.22)]);
    }

    #[test]
    fn keep_current_missing_state_refuses_and_emits_no_vector() {
        let model = synth_planar_two_link();
        let set = ActuatorCommandSet {
            commands: vec![cmd("a0", 0.4, "position")],
            hold_outside: HoldSemantics::KeepCurrent,
            explicit_safe: Default::default(),
        };
        let mut current = current_complete();
        current.remove("a1");
        current.remove("j1");
        let err = lower_actuator_commands(&model, &set, &current).unwrap_err();
        assert_eq!(err, SkillRefuse::MissingJointState);
        assert!(!err.writes_allowed());
    }

    #[test]
    fn explicit_safe_uses_declared_value_including_zero() {
        let model = synth_planar_two_link();
        let set = ActuatorCommandSet {
            commands: vec![cmd("a0", 0.4, "position")],
            hold_outside: HoldSemantics::ExplicitSafe,
            explicit_safe: std::collections::BTreeMap::from([("a1".into(), 0.0)]),
        };
        let out = lower_actuator_commands(&model, &set, &Default::default()).unwrap();
        assert_eq!(out, vec![("a0".into(), 0.4), ("a1".into(), 0.0)]);
    }

    #[test]
    fn explicit_safe_without_declaration_refuses() {
        let model = synth_planar_two_link();
        let set = ActuatorCommandSet {
            commands: vec![cmd("a0", 0.4, "position")],
            hold_outside: HoldSemantics::ExplicitSafe,
            explicit_safe: Default::default(),
        };
        let err = lower_actuator_commands(&model, &set, &current_complete()).unwrap_err();
        assert_eq!(err, SkillRefuse::InvalidCommand);
    }

    #[test]
    fn unknown_actuator_refuses() {
        let model = synth_planar_two_link();
        let set = ActuatorCommandSet {
            commands: vec![cmd("nope", 0.1, "position")],
            hold_outside: HoldSemantics::KeepCurrent,
            explicit_safe: Default::default(),
        };
        let err = lower_actuator_commands(&model, &set, &current_complete()).unwrap_err();
        assert_eq!(err, SkillRefuse::MissingActuator);
    }

    #[test]
    fn duplicate_actuator_refuses() {
        let model = synth_planar_two_link();
        let set = ActuatorCommandSet {
            commands: vec![cmd("a0", 0.1, "position"), cmd("a0", 0.2, "position")],
            hold_outside: HoldSemantics::KeepCurrent,
            explicit_safe: Default::default(),
        };
        let err = lower_actuator_commands(&model, &set, &current_complete()).unwrap_err();
        assert_eq!(err, SkillRefuse::InvalidCommand);
    }

    #[test]
    fn nan_refuses() {
        let model = synth_planar_two_link();
        let set = ActuatorCommandSet {
            commands: vec![cmd("a0", f64::NAN, "position")],
            hold_outside: HoldSemantics::KeepCurrent,
            explicit_safe: Default::default(),
        };
        assert_eq!(
            lower_actuator_commands(&model, &set, &current_complete()).unwrap_err(),
            SkillRefuse::InvalidCommand
        );
    }

    #[test]
    fn inf_refuses() {
        let model = synth_planar_two_link();
        for v in [f64::INFINITY, f64::NEG_INFINITY] {
            let set = ActuatorCommandSet {
                commands: vec![cmd("a0", v, "position")],
                hold_outside: HoldSemantics::KeepCurrent,
                explicit_safe: Default::default(),
            };
            assert_eq!(
                lower_actuator_commands(&model, &set, &current_complete()).unwrap_err(),
                SkillRefuse::InvalidCommand
            );
        }
    }

    #[test]
    fn incompatible_control_mode_refuses() {
        let model = synth_planar_two_link();
        let set = ActuatorCommandSet {
            commands: vec![cmd("a0", 0.1, "velocity")],
            hold_outside: HoldSemantics::KeepCurrent,
            explicit_safe: Default::default(),
        };
        assert_eq!(
            lower_actuator_commands(&model, &set, &current_complete()).unwrap_err(),
            SkillRefuse::InvalidCommand
        );
    }

    #[test]
    fn tendon_omitted_without_current_refuses() {
        let mut m = synth_planar_two_link();
        m.actuators.push(Actuator {
            name: "split".into(),
            target_joint: "split".into(),
            control_mode: "position".into(),
            transmission_kind: "tendon".into(),
            ctrlrange: Provenanced::unknown("test", 0.0),
            forcerange: Provenanced::unknown("test", 0.0),
            gear: Provenanced::unknown("test", 0.0),
        });
        let targets = JointTargetSet {
            targets: vec![JointTarget {
                joint_name: "j0".into(),
                actuator_name: Some("a0".into()),
                value: 0.3,
                control_mode: "position".into(),
                skill_id: "skill.reach".into(),
            }],
            hold_outside: HoldSemantics::KeepCurrent,
            explicit_safe: Default::default(),
        };
        let mut current = current_complete();
        current.remove("split");
        let err = lower_named_targets(&m, &targets, &current).unwrap_err();
        assert_eq!(err, SkillRefuse::MissingJointState);
    }

    #[test]
    fn tendon_omitted_with_named_current_holds_exact() {
        let mut m = synth_planar_two_link();
        m.actuators.push(Actuator {
            name: "split".into(),
            target_joint: "split".into(),
            control_mode: "position".into(),
            transmission_kind: "tendon".into(),
            ctrlrange: Provenanced::unknown("test", 0.0),
            forcerange: Provenanced::unknown("test", 0.0),
            gear: Provenanced::unknown("test", 0.0),
        });
        let targets = JointTargetSet {
            targets: vec![JointTarget {
                joint_name: "j0".into(),
                actuator_name: Some("a0".into()),
                value: 0.3,
                control_mode: "position".into(),
                skill_id: "skill.reach".into(),
            }],
            hold_outside: HoldSemantics::KeepCurrent,
            explicit_safe: Default::default(),
        };
        let mut current = current_complete();
        current.insert("split".into(), 0.77);
        let out = lower_named_targets(&m, &targets, &current).unwrap();
        let split = out.iter().find(|(n, _)| n == "split").unwrap();
        assert_eq!(split.1, 0.77);
        let a1 = out.iter().find(|(n, _)| n == "a1").unwrap();
        assert_eq!(a1.1, 0.22);
    }

    #[test]
    fn ambiguous_joint_target_refuses() {
        let mut m = synth_planar_two_link();
        m.actuators.push(Actuator {
            name: "a0_alias".into(),
            target_joint: "j0".into(),
            control_mode: "position".into(),
            transmission_kind: "joint".into(),
            ctrlrange: Provenanced::unknown("test", 0.0),
            forcerange: Provenanced::unknown("test", 0.0),
            gear: Provenanced::unknown("test", 0.0),
        });
        let targets = JointTargetSet {
            targets: vec![JointTarget {
                joint_name: "j0".into(),
                actuator_name: None,
                value: 0.3,
                control_mode: "position".into(),
                skill_id: "skill.reach".into(),
            }],
            hold_outside: HoldSemantics::KeepCurrent,
            explicit_safe: Default::default(),
        };
        assert_eq!(
            lower_named_targets(&m, &targets, &current_complete()).unwrap_err(),
            SkillRefuse::InvalidCommand
        );
    }

    #[test]
    fn named_and_actuator_lowerers_agree_on_keep_current() {
        let model = synth_planar_two_link();
        let current = current_complete();
        let cmds = ActuatorCommandSet {
            commands: vec![cmd("a0", 0.4, "position")],
            hold_outside: HoldSemantics::KeepCurrent,
            explicit_safe: Default::default(),
        };
        let targets = JointTargetSet {
            targets: vec![JointTarget {
                joint_name: "j0".into(),
                actuator_name: Some("a0".into()),
                value: 0.4,
                control_mode: "position".into(),
                skill_id: "t".into(),
            }],
            hold_outside: HoldSemantics::KeepCurrent,
            explicit_safe: Default::default(),
        };
        let a = lower_actuator_commands(&model, &cmds, &current).unwrap();
        let b = lower_named_targets(&model, &targets, &current).unwrap();
        assert_eq!(a, b);
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
