//! Foundation REACH loop: SkillContract → ActionProposal → SimAuthority → plant.

use crate::authority::SimAuthority;
use crate::bundle::RobotBundle;
use crate::driver::{SharedMujoco, SharedSimPort, SimActuationProbe};
use crate::honesty::SIMULATION_ONLY;
use crate::mujoco_exec::{checkin_worker, json_f64_vec};
use crate::normalize::RobotManifest;
use crate::observation::{policy_observation, PolicyObservation, VerifierTruth, VisionMode};
use crate::policy::ActionProposal;
use crate::runner::load_and_normalize;
use crate::semantics_map::embodiment_from_manifest;
use crate::task::TaskSpec;
use realityos_plant::HardwareBackedPlant;
use realityos_semantics::adapter::{lower_named_targets, ChainIkPositionPdAdapter, CompiledCtrl};
use realityos_semantics::capability::derive_capabilities;
use realityos_semantics::embodiment::EmbodimentModel;
use realityos_semantics::observation::{JointStateSample, ObservationFrame, SensorObservation};
use realityos_semantics::provenance::Provenance;
use realityos_semantics::reach::compile_reach;
use realityos_semantics::skill::SkillRefuse;
use realityos_semantics::transform::{Se3, TransformEdge, TransformGraph};
use realityos_semantics::world::WorldState;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::HashMap;
use std::sync::{Arc, Mutex};

const ADAPTER_ID: &str = "chain_ik_position_pd";
const HORIZON_S: f64 = 1.0;
const CONTROL_HZ: f64 = 50.0;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FoundationReachReport {
    pub robot_id: String,
    pub model_hash: String,
    pub skill: String,
    pub adapter_id: String,
    pub adaptation: String,
    pub metal: bool,
    pub evidence_status: String,
    pub task_success: bool,
    pub skill_refuse: Option<String>,
    pub ctrl_writes: u64,
    pub replay_write_delta: Option<i64>,
    pub authority_decisions: Vec<String>,
    pub seed: u64,
    #[serde(default)]
    pub cartesian_residual: Option<f64>,
    #[serde(default)]
    pub ik_residual: Option<f64>,
    #[serde(default)]
    pub max_joint_move: Option<f64>,
    #[serde(default)]
    pub joint_delta_norm: Option<f64>,
    #[serde(default)]
    pub initial_q: Vec<f64>,
    #[serde(default)]
    pub target_q: Vec<f64>,
}

pub fn run_foundation_reach(
    bundle: &RobotBundle,
    target: [f64; 3],
    radius: f64,
    now_s: f64,
    freshness_s: f64,
    expected_hash: Option<&str>,
    force_stale: bool,
    replay: bool,
) -> Result<FoundationReachReport, String> {
    let (inst, manifest) = load_and_normalize(bundle, &[], 0)?;
    let (report, inst) = run_foundation_reach_on(
        bundle,
        inst,
        &manifest,
        target,
        radius,
        now_s,
        freshness_s,
        expected_hash,
        force_stale,
        replay,
    )?;
    checkin_worker(inst);
    Ok(report)
}

#[allow(clippy::too_many_arguments)]
pub fn run_foundation_reach_on(
    bundle: &RobotBundle,
    mut inst: crate::mujoco_exec::MujocoInstance,
    manifest: &RobotManifest,
    target: [f64; 3],
    radius: f64,
    now_s: f64,
    freshness_s: f64,
    expected_hash: Option<&str>,
    force_stale: bool,
    replay: bool,
) -> Result<(FoundationReachReport, crate::mujoco_exec::MujocoInstance), String> {
    let seed = 0u64;
    let _ = inst.reset(None, None);
    let model = embodiment_from_manifest(bundle, manifest);
    let caps = derive_capabilities(&model, None);

    let initial = inst.step(0).map_err(|e| e.to_string()).and_then(|st| {
        st.get("state")
            .ok_or_else(|| "missing initial state".into())
            .map(VerifierTruth::from_mujoco_state)
    })?;

    let epoch = model.calibration_epoch.clone();
    let model_hash = model.model_hash.clone();
    let hash = expected_hash.unwrap_or(&model_hash);
    let expires = if force_stale {
        now_s - freshness_s - 0.05
    } else {
        now_s + freshness_s
    };
    let ee = semantic_ee(bundle);
    let world = WorldState::empty(&epoch, now_s)
        .with_target_in_frame(
            &ee,
            "world",
            target,
            expires,
            &epoch,
            now_s,
            Provenance::UserDeclared,
        )
        .with_success_radius(radius);
    let obs_frame = build_observation_frame(
        &model,
        &initial.qpos,
        &epoch,
        now_s,
        freshness_s,
        force_stale,
    );
    let transforms = graph_from_truth(&model, &initial, &epoch, now_s);

    let compile = compile_reach(
        &model,
        &caps,
        &world,
        &obs_frame,
        &transforms,
        hash,
        now_s,
        freshness_s,
        &ChainIkPositionPdAdapter,
    );

    let base = base_report(&model, seed);
    let ctrl = match compile {
        Err(refuse) => {
            return Ok((error_report(base, refuse_string(refuse)), inst));
        }
        Ok(ctrl) => ctrl,
    };

    let shared = Arc::new(SharedMujoco {
        inst: Mutex::new(inst),
        probe: SimActuationProbe::default(),
        robot_id: manifest.robot_id.clone(),
        model_hash: manifest.model_hash.clone(),
    });
    let port = SharedSimPort::new(shared.clone());
    let max_a = manifest.tau_max().into_iter().fold(1.0, f64::max);
    let plant =
        HardwareBackedPlant::new(port, &manifest.robot_id, manifest.nu.max(1) as usize, max_a);
    let journal = std::env::temp_dir().join(format!(
        "realityos-foundation-reach-{}-{}-{}.jsonl",
        manifest.robot_id,
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_nanos())
            .unwrap_or(0)
    ));
    let _ = std::fs::remove_file(&journal);
    let _ = std::fs::remove_file(journal.with_extension("lease.json"));

    let mut auth = SimAuthority::open_with_journal(plant, manifest, now_s, Some(journal))?;
    auth.freshness_s = freshness_s;
    auth.command_lifetime_s = HORIZON_S + 0.5;

    let episode_id = format!("foundation-reach-{}", manifest.robot_id);
    let observation_id = "obs-0".to_string();
    let policy_obs = build_policy_observation(
        manifest,
        &episode_id,
        &observation_id,
        &initial,
        target,
        now_s,
    );
    let current_by_joint = current_joint_map(&model, &initial.qpos);
    let proposal = action_proposal_from_ctrl(
        &policy_obs,
        &ctrl,
        &model,
        &current_by_joint,
        now_s,
        auth.command_lifetime_s,
    )?;
    let task = TaskSpec::Reach {
        end_effector: privileged_ee(bundle),
        target,
        radius,
    };

    let command_id = format!("{episode_id}:{observation_id}");
    let mut decisions = Vec::new();
    let rec = auth.decide_and_maybe_write(
        &proposal,
        manifest,
        &task,
        &policy_obs,
        now_s,
        None,
        Some("reach"),
    );
    decisions.push(decision_label(&rec));

    let mut replay_write_delta = None;
    if replay {
        let before = shared.probe.snapshot().policy_ctrl_writes;
        let replay_rec = auth.decide_and_maybe_write(
            &proposal,
            manifest,
            &task,
            &policy_obs,
            now_s + 0.001,
            Some(command_id),
            Some("reach"),
        );
        decisions.push(decision_label(&replay_rec));
        let after = shared.probe.snapshot().policy_ctrl_writes;
        replay_write_delta = Some(after as i64 - before as i64);
    }

    let dt = manifest.timestep.max(1e-4);
    let substeps = ((1.0 / CONTROL_HZ) / dt).round().max(1.0) as u32;
    let cycles = ((HORIZON_S * CONTROL_HZ).round() as u32).max(1);
    let mut truth = initial;
    for _ in 0..cycles {
        let stepped = {
            let mut g = shared.inst.lock().map_err(|e| e.to_string())?;
            g.step(substeps).map_err(|e| e.to_string())?
        };
        truth = VerifierTruth::from_mujoco_state(stepped.get("state").unwrap_or(&stepped));
    }

    let ctrl_writes = shared.probe.snapshot().policy_ctrl_writes;
    let task_success = task.evaluate(&truth, manifest);
    let ee_site = privileged_ee(bundle);
    let cartesian_residual = truth
        .named_pos
        .get(&ee_site)
        .or_else(|| truth.xpos.get(&ee_site))
        .and_then(|p| {
            if p.len() < 3 {
                None
            } else {
                let dx = p[0] - target[0];
                let dy = p[1] - target[1];
                let dz = p[2] - target[2];
                Some((dx * dx + dy * dy + dz * dz).sqrt())
            }
        });

    drop(auth);
    let inst = match Arc::try_unwrap(shared) {
        Ok(owned) => owned
            .inst
            .into_inner()
            .map_err(|_| "mujoco mutex poisoned".to_string())?,
        Err(_) => return Err("shared mujoco still referenced".into()),
    };

    Ok((
        FoundationReachReport {
            robot_id: model.robot_id,
            model_hash: model.model_hash,
            skill: "REACH".into(),
            adapter_id: ctrl.adapter_id,
            adaptation: "CONFIGURED".into(),
            metal: false,
            evidence_status: SIMULATION_ONLY.into(),
            task_success,
            skill_refuse: None,
            ctrl_writes,
            replay_write_delta,
            authority_decisions: decisions,
            seed,
            cartesian_residual,
            ik_residual: ctrl.ik.as_ref().map(|t| t.residual),
            max_joint_move: ctrl.ik.as_ref().map(|t| t.max_joint_move),
            joint_delta_norm: ctrl.ik.as_ref().map(|t| t.joint_delta_norm),
            initial_q: ctrl
                .ik
                .as_ref()
                .map(|t| t.initial_q.clone())
                .unwrap_or_default(),
            target_q: ctrl
                .ik
                .as_ref()
                .map(|t| t.target_q.clone())
                .unwrap_or_default(),
        },
        inst,
    ))
}

pub fn run_foundation_reach_missing_target(
    bundle: &RobotBundle,
) -> Result<FoundationReachReport, String> {
    let seed = 0u64;
    let (mut inst, manifest) = load_and_normalize(bundle, &[], seed)?;
    let model = embodiment_from_manifest(bundle, &manifest);
    let caps = derive_capabilities(&model, None);

    let initial = inst.step(0).map_err(|e| e.to_string()).and_then(|st| {
        st.get("state")
            .ok_or_else(|| "missing initial state".into())
            .map(|s| json_f64_vec(&s["qpos"]))
    })?;

    let epoch = model.calibration_epoch.clone();
    let now_s = 10.0;
    let freshness_s = 0.25;
    let world = WorldState::empty(&epoch, now_s);
    let obs_frame = build_observation_frame(&model, &initial, &epoch, now_s, freshness_s, false);
    let transforms = TransformGraph::new(&epoch);

    let err = compile_reach(
        &model,
        &caps,
        &world,
        &obs_frame,
        &transforms,
        &model.model_hash,
        now_s,
        freshness_s,
        &ChainIkPositionPdAdapter,
    )
    .unwrap_err();

    checkin_worker(inst);
    Ok(error_report(base_report(&model, seed), refuse_string(err)))
}

fn semantic_ee(bundle: &RobotBundle) -> String {
    bundle
        .manifest
        .end_effectors
        .first()
        .map(|e| e.name.clone())
        .unwrap_or_else(|| "ee".into())
}

fn privileged_ee(bundle: &RobotBundle) -> String {
    bundle
        .manifest
        .end_effectors
        .first()
        .and_then(|e| e.site.clone().or_else(|| e.body.clone()))
        .unwrap_or_else(|| "ee".into())
}

fn graph_from_truth(
    model: &EmbodimentModel,
    truth: &VerifierTruth,
    epoch: &str,
    now_s: f64,
) -> TransformGraph {
    let mut g = TransformGraph::new(epoch);
    for (name, pos) in &truth.xpos {
        if name == "world" || pos.len() < 3 {
            continue;
        }
        let quat = truth
            .xquat
            .get(name)
            .filter(|q| q.len() >= 4)
            .map(|q| [q[0], q[1], q[2], q[3]]);
        let Some(quat) = quat else {
            continue;
        };
        let Ok(pose) = Se3::try_new([pos[0], pos[1], pos[2]], quat) else {
            continue;
        };
        let _ = g.insert(TransformEdge::from_se3(
            "world",
            name,
            pose,
            epoch,
            now_s,
            "verify.privileged_fk",
        ));
    }
    if g.lookup("world", "base").is_none() {
        if let Some(base) = model.bodies.iter().find(|b| b.parent.is_none()) {
            if let Some(pose) = base.local_pose.value {
                let _ = g.insert(TransformEdge::from_se3(
                    "world",
                    &base.name,
                    pose,
                    epoch,
                    now_s,
                    "verify.model_base",
                ));
            }
        }
    }
    g
}

fn current_joint_map(model: &EmbodimentModel, qpos: &[f64]) -> HashMap<String, f64> {
    let mut out = HashMap::new();
    for joint in &model.joints {
        if let Some(adr) = joint.qpos_adr {
            if let Some(q) = qpos.get(adr as usize) {
                out.insert(joint.name.clone(), *q);
            }
        }
    }
    out
}

fn qpos_digest(qpos: &[f64]) -> String {
    let mut h = Sha256::new();
    h.update(b"realityos.foundation.qpos/1\0");
    for q in qpos {
        h.update(q.to_le_bytes());
    }
    hex::encode(h.finalize())
}

fn build_observation_frame(
    model: &EmbodimentModel,
    qpos: &[f64],
    epoch: &str,
    now_s: f64,
    freshness_s: f64,
    force_stale: bool,
) -> ObservationFrame {
    let (receive_s, expires_at_s) = if force_stale {
        (now_s - freshness_s - 0.05, now_s - 0.01)
    } else {
        (now_s, now_s + freshness_s)
    };
    let joint_state = model
        .joints
        .iter()
        .filter_map(|j| {
            let adr = j.qpos_adr? as usize;
            let q = *qpos.get(adr)?;
            Some(JointStateSample::new(
                &j.name,
                q,
                None,
                receive_s,
                receive_s,
                format!("enc.{}", j.name),
                "sim_cal",
                epoch,
            ))
        })
        .collect();
    ObservationFrame {
        frame_id: "foundation-obs".into(),
        transform_epoch: epoch.to_string(),
        observations: vec![SensorObservation::joint_encoder(
            "joint_encoders",
            "base",
            "sim_cal",
            receive_s,
            receive_s,
            1,
            "sim",
            qpos_digest(qpos),
            expires_at_s,
        )],
        joint_state,
        as_of_s: receive_s,
    }
}

fn build_policy_observation(
    manifest: &RobotManifest,
    episode_id: &str,
    observation_id: &str,
    truth: &VerifierTruth,
    target: [f64; 3],
    now_s: f64,
) -> PolicyObservation {
    let mut obs = policy_observation(
        manifest,
        episode_id,
        observation_id,
        &TaskSpec::Reach {
            end_effector: manifest.end_effector_name().unwrap_or_else(|| "ee".into()),
            target,
            radius: 0.0,
        },
        VisionMode::State,
        truth,
        0.0,
        false,
    );
    obs.timestamp_s = now_s;
    obs
}

fn action_proposal_from_ctrl(
    obs: &PolicyObservation,
    ctrl: &CompiledCtrl,
    model: &EmbodimentModel,
    current_by_joint: &HashMap<String, f64>,
    now_s: f64,
    horizon_s: f64,
) -> Result<ActionProposal, String> {
    let lowered = lower_named_targets(model, &ctrl.targets, current_by_joint)
        .map_err(|e| format!("lower targets: {e:?}"))?;
    let action = lowered.into_iter().map(|(_, v)| v).collect();
    Ok(ActionProposal {
        robot_id: obs.robot_id.clone(),
        model_hash: obs.model_hash.clone(),
        episode_id: obs.episode_id.clone(),
        observation_id: obs.observation_id.clone(),
        observation_timestamp: now_s,
        task_id: obs.task_id.clone(),
        action,
        control_mode: ctrl.control_mode.clone(),
        requested_horizon_s: horizon_s,
        confidence: Some(1.0),
        policy_id: ctrl.adapter_id.clone(),
        policy_version: ctrl.adapter_version.clone(),
    })
}

fn base_report(model: &EmbodimentModel, seed: u64) -> FoundationReachReport {
    FoundationReachReport {
        robot_id: model.robot_id.clone(),
        model_hash: model.model_hash.clone(),
        skill: "REACH".into(),
        adapter_id: ADAPTER_ID.into(),
        adaptation: "CONFIGURED".into(),
        metal: false,
        evidence_status: SIMULATION_ONLY.into(),
        task_success: false,
        skill_refuse: None,
        ctrl_writes: 0,
        replay_write_delta: None,
        authority_decisions: Vec::new(),
        seed,
        cartesian_residual: None,
        ik_residual: None,
        max_joint_move: None,
        joint_delta_norm: None,
        initial_q: Vec::new(),
        target_q: Vec::new(),
    }
}

fn error_report(mut base: FoundationReachReport, refuse: String) -> FoundationReachReport {
    base.skill_refuse = Some(refuse);
    base.task_success = false;
    base.ctrl_writes = 0;
    base
}

fn refuse_string(err: SkillRefuse) -> String {
    match err {
        SkillRefuse::MissingTarget | SkillRefuse::Probe => "PROBE".into(),
        SkillRefuse::Unreachable => "UNREACHABLE".into(),
        SkillRefuse::Unsupported | SkillRefuse::MissingActuator | SkillRefuse::NotInIr => {
            "UNSUPPORTED".into()
        }
        SkillRefuse::StaleEvidence
        | SkillRefuse::WrongModelHash
        | SkillRefuse::EpochMismatch
        | SkillRefuse::Refuse => "REFUSE".into(),
        SkillRefuse::MissingJointState => "PROBE".into(),
        SkillRefuse::KinematicsUnsupported => "KINEMATICS_UNSUPPORTED_FOR_ADAPTER".into(),
        SkillRefuse::ModelFeatureUnsupported => "MODEL_FEATURE_UNSUPPORTED".into(),
    }
}

fn decision_label(rec: &crate::authority::AuthorityRecord) -> String {
    format!("{:?}:{}", rec.outcome, rec.status)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::honesty::SIMULATION_ONLY;
    use crate::mujoco_exec::ensure_mujoco_or_skip;

    #[test]
    fn planar_and_spatial_use_the_same_reach_contract() {
        if !ensure_mujoco_or_skip() {
            return;
        }
        for id in ["planar_arm", "spatial_arm4"] {
            let b = RobotBundle::load(crate::corpus::bundled_robots_root().join(id)).unwrap();
            let r =
                run_foundation_reach(&b, [0.22, 0.0, 0.12], 0.20, 10.0, 0.25, None, false, false)
                    .unwrap();
            assert_eq!(r.skill, "REACH");
            assert_eq!(r.adapter_id, "chain_ik_position_pd");
            assert!(!r.metal);
            assert_eq!(r.evidence_status, SIMULATION_ONLY);
            assert_eq!(r.adaptation, "CONFIGURED");
            assert!(r.skill_refuse.is_none(), "{id} {:?}", r.skill_refuse);
            assert!(r.ctrl_writes > 0, "{id}");
            assert!(
                r.task_success,
                "{id} must reach under privileged verifier {r:?}"
            );
        }
    }

    #[test]
    fn missing_target_probes_without_writes() {
        if !ensure_mujoco_or_skip() {
            return;
        }
        let b = RobotBundle::load(crate::corpus::robot_dir("planar_arm")).unwrap();
        let r = run_foundation_reach_missing_target(&b).unwrap();
        assert_eq!(r.skill_refuse.as_deref(), Some("PROBE"));
        assert_eq!(r.ctrl_writes, 0);
    }

    #[test]
    fn replay_does_not_add_policy_writes() {
        if !ensure_mujoco_or_skip() {
            return;
        }
        let b = RobotBundle::load(crate::corpus::robot_dir("planar_arm")).unwrap();
        let r = run_foundation_reach(&b, [0.22, 0.0, 0.12], 0.20, 10.0, 0.25, None, false, true)
            .unwrap();
        assert_eq!(r.replay_write_delta, Some(0));
    }

    #[test]
    fn wrong_hash_zero_writes() {
        if !ensure_mujoco_or_skip() {
            return;
        }
        let b = RobotBundle::load(crate::corpus::robot_dir("planar_arm")).unwrap();
        let r = run_foundation_reach(
            &b,
            [0.22, 0.0, 0.12],
            0.20,
            10.0,
            0.25,
            Some("deadbeef"),
            false,
            false,
        )
        .unwrap();
        assert_eq!(r.ctrl_writes, 0);
        assert!(r.skill_refuse.is_some());
    }

    #[test]
    fn stale_zero_writes() {
        if !ensure_mujoco_or_skip() {
            return;
        }
        let b = RobotBundle::load(crate::corpus::robot_dir("planar_arm")).unwrap();
        let r = run_foundation_reach(&b, [0.22, 0.0, 0.12], 0.20, 10.0, 0.25, None, true, false)
            .unwrap();
        assert_eq!(r.ctrl_writes, 0);
    }

    #[test]
    fn held_out_first_evaluation_is_recorded() {
        if !ensure_mujoco_or_skip() {
            return;
        }
        let b = RobotBundle::load(crate::held_out::held_out_bundle()).unwrap();
        let r = run_foundation_reach(&b, [0.20, 0.0, 0.12], 0.25, 10.0, 0.25, None, false, false)
            .unwrap();
        assert_eq!(r.adaptation, "CONFIGURED");
        assert!(!r.metal);
        assert!(!r.model_hash.is_empty());
    }
}
