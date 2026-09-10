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
use realityos_semantics::adapter::{ChainIkPositionPdAdapter, CompiledCtrl};
use realityos_semantics::capability::derive_capabilities;
use realityos_semantics::embodiment::{EmbodimentModel, FrameKind, ModelFrame};
use realityos_semantics::observation::{ObservationFrame, SensorObservation};
use realityos_semantics::provenance::{Provenance, Provenanced};
use realityos_semantics::reach::compile_reach;
use realityos_semantics::skill::SkillRefuse;
use realityos_semantics::world::WorldState;
use serde_json::Value;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::HashMap;
use std::sync::{Arc, Mutex};

const ADAPTER_ID: &str = "chain_ik_position_pd";
const INSPECT_SOURCE: &str = "verify.inspect";
const HORIZON_S: f64 = 0.5;
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
    let seed = 0u64;
    let (mut inst, manifest) = load_and_normalize(bundle, &[], seed)?;
    let inspect = inst.inspect.clone();
    let mut model = embodiment_from_manifest(bundle, &manifest);
    let mut caps = derive_capabilities(&model, None);

    let initial = inst
        .step(0)
        .map_err(|e| e.to_string())
        .and_then(|st| {
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
    let ik_target = fk_frame_target(&initial, &manifest, target);
    let world = WorldState::empty(&epoch, now_s).with_target(
        "ee",
        ik_target,
        expires,
        &epoch,
        now_s,
        Provenance::UserDeclared,
    );
    let obs_frame = build_observation_frame(&initial.qpos, &epoch, now_s, freshness_s, force_stale);

    let compile = try_compile_reach(
        &mut model,
        &mut caps,
        &inspect,
        &world,
        &obs_frame,
        hash,
        now_s,
        freshness_s,
    );

    let base = base_report(&model, seed);
    let ctrl = match compile {
        Err(refuse) => {
            checkin_worker(inst);
            return Ok(error_report(base, refuse_string(refuse)));
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
    let plant = HardwareBackedPlant::new(
        port,
        &manifest.robot_id,
        manifest.nu.max(1) as usize,
        max_a,
    );
    let journal = std::env::temp_dir().join(format!(
        "realityos-foundation-reach-{}-{}.jsonl",
        manifest.robot_id,
        std::process::id()
    ));
    let _ = std::fs::remove_file(&journal);
    let _ = std::fs::remove_file(journal.with_extension("lease.json"));

    let mut auth = SimAuthority::open_with_journal(plant, &manifest, now_s, Some(journal))?;
    auth.freshness_s = freshness_s;
    auth.command_lifetime_s = 1.0;

    let episode_id = format!("foundation-reach-{}", manifest.robot_id);
    let observation_id = "obs-0".to_string();
    let policy_obs = build_policy_observation(
        &manifest,
        &episode_id,
        &observation_id,
        &initial,
        target,
        now_s,
    );
    let proposal = action_proposal_from_ctrl(
        &policy_obs,
        &ctrl,
        now_s,
        auth.command_lifetime_s,
    );
    let task = TaskSpec::Reach {
        end_effector: "ee".into(),
        target,
        radius,
    };

    let command_id = format!("{episode_id}:{observation_id}");
    let mut decisions = Vec::new();
    let rec = auth.decide_and_maybe_write(
        &proposal,
        &manifest,
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
            &manifest,
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
    let task_success = task.evaluate(&truth, &manifest);

    drop(auth);
    if let Ok(owned) = Arc::try_unwrap(shared) {
        if let Ok(inst) = owned.inst.into_inner() {
            checkin_worker(inst);
        }
    }

    Ok(FoundationReachReport {
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
    })
}

pub fn run_foundation_reach_missing_target(
    bundle: &RobotBundle,
) -> Result<FoundationReachReport, String> {
    let seed = 0u64;
    let (mut inst, manifest) = load_and_normalize(bundle, &[], seed)?;
    let model = embodiment_from_manifest(bundle, &manifest);
    let caps = derive_capabilities(&model, None);

    let initial = inst
        .step(0)
        .map_err(|e| e.to_string())
        .and_then(|st| {
            st.get("state")
                .ok_or_else(|| "missing initial state".into())
                .map(|s| json_f64_vec(&s["qpos"]))
        })?;

    let epoch = model.calibration_epoch.clone();
    let now_s = 10.0;
    let freshness_s = 0.25;
    let world = WorldState::empty(&epoch, now_s);
    let obs_frame = build_observation_frame(&initial, &epoch, now_s, freshness_s, false);

    let err = compile_reach(
        &model,
        &caps,
        &world,
        &obs_frame,
        &model.model_hash,
        now_s,
        freshness_s,
        &ChainIkPositionPdAdapter,
    )
    .unwrap_err();

    checkin_worker(inst);
    Ok(error_report(base_report(&model, seed), refuse_string(err)))
}

fn try_compile_reach(
    model: &mut EmbodimentModel,
    caps: &mut realityos_semantics::capability::CapabilityGraph,
    inspect: &Value,
    world: &WorldState,
    obs: &ObservationFrame,
    hash: &str,
    now_s: f64,
    freshness_s: f64,
) -> Result<CompiledCtrl, SkillRefuse> {
    match compile_reach(
        model,
        caps,
        world,
        obs,
        hash,
        now_s,
        freshness_s,
        &ChainIkPositionPdAdapter,
    ) {
        Ok(ctrl) => Ok(ctrl),
        Err(SkillRefuse::Unreachable) => {
            fill_from_inspect(model, inspect);
            *caps = derive_capabilities(model, None);
            compile_reach(
                model,
                caps,
                world,
                obs,
                hash,
                now_s,
                freshness_s,
                &ChainIkPositionPdAdapter,
            )
        }
        Err(e) => Err(e),
    }
}

fn fill_from_inspect(model: &mut EmbodimentModel, inspect: &Value) {
    let joint_axes = joint_axis_map(inspect);
    let body_pos = body_pos_map(inspect);
    let site_pos = site_pos_map(inspect);

    for joint in &mut model.joints {
        if joint.axis.value.is_none() {
            if let Some(axis) = joint_axes.get(&joint.name) {
                joint.axis = Provenanced::simulator_derived(*axis, INSPECT_SOURCE, 0.0);
            }
        }
    }

    for joint in model.joints.clone() {
        let frame_name = format!("link_{}", joint.name);
        let pos = body_pos.get(&joint.child_body).copied();
        if let Some(pos) = pos {
            if let Some(frame) = model.frames.iter_mut().find(|f| {
                f.name == frame_name && f.parent_body == joint.parent_body
            }) {
                if frame.translation.value.is_none() {
                    frame.translation =
                        Provenanced::simulator_derived(pos, INSPECT_SOURCE, 0.0);
                }
            } else {
                model.frames.push(ModelFrame {
                    name: frame_name,
                    kind: FrameKind::Task,
                    parent_body: joint.parent_body.clone(),
                    translation: Provenanced::simulator_derived(pos, INSPECT_SOURCE, 0.0),
                });
            }
        }
    }

    for frame in &mut model.frames {
        if frame.translation.value.is_none() {
            if let Some(pos) = site_pos.get(&frame.name) {
                frame.translation = Provenanced::simulator_derived(*pos, INSPECT_SOURCE, 0.0);
            }
        }
    }
}

fn joint_axis_map(inspect: &Value) -> HashMap<String, [f64; 3]> {
    let mut out = HashMap::new();
    if let Some(arr) = inspect.get("joints").and_then(|v| v.as_array()) {
        for j in arr {
            let name = j["name"].as_str().unwrap_or("").to_string();
            if name.is_empty() {
                continue;
            }
            if let Some(axis) = vec3_from_json(&j["axis"]) {
                out.insert(name, axis);
            }
        }
    }
    out
}

fn body_pos_map(inspect: &Value) -> HashMap<String, [f64; 3]> {
    let mut out = HashMap::new();
    if let Some(arr) = inspect.get("bodies").and_then(|v| v.as_array()) {
        for b in arr {
            let name = b["name"].as_str().unwrap_or("").to_string();
            if name.is_empty() {
                continue;
            }
            if let Some(pos) = vec3_from_json(&b["pos"]) {
                out.insert(name, pos);
            }
        }
    }
    out
}

fn site_pos_map(inspect: &Value) -> HashMap<String, [f64; 3]> {
    let mut out = HashMap::new();
    if let Some(arr) = inspect.get("sites").and_then(|v| v.as_array()) {
        for s in arr {
            let name = s["name"].as_str().unwrap_or("").to_string();
            if name.is_empty() {
                continue;
            }
            if let Some(pos) = vec3_from_json(&s["pos"]) {
                out.insert(name, pos);
            }
        }
    }
    out
}

fn vec3_from_json(v: &Value) -> Option<[f64; 3]> {
    let arr = v.as_array()?;
    if arr.len() < 3 {
        return None;
    }
    Some([
        arr[0].as_f64()?,
        arr[1].as_f64()?,
        arr[2].as_f64()?,
    ])
}

fn fk_frame_target(
    truth: &VerifierTruth,
    manifest: &RobotManifest,
    target: [f64; 3],
) -> [f64; 3] {
    let base_name = manifest
        .derived
        .end_effector_joint_chains
        .first()
        .and_then(|chain| chain.first())
        .and_then(|j| manifest.joints.iter().find(|x| x.name == *j))
        .map(|j| j.parent_body.as_str())
        .unwrap_or("base");
    let origin = truth
        .xpos
        .get(base_name)
        .map(|p| [p[0], p[1], p[2]])
        .unwrap_or([0.0, 0.0, 0.0]);
    [
        target[0] - origin[0],
        target[1] - origin[1],
        target[2] - origin[2],
    ]
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
            end_effector: "ee".into(),
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
    now_s: f64,
    horizon_s: f64,
) -> ActionProposal {
    ActionProposal {
        robot_id: obs.robot_id.clone(),
        model_hash: obs.model_hash.clone(),
        episode_id: obs.episode_id.clone(),
        observation_id: obs.observation_id.clone(),
        observation_timestamp: now_s,
        task_id: obs.task_id.clone(),
        action: ctrl.action.clone(),
        control_mode: ctrl.control_mode.clone(),
        requested_horizon_s: horizon_s,
        confidence: Some(1.0),
        policy_id: ctrl.adapter_id.clone(),
        policy_version: ctrl.adapter_version.clone(),
    }
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
            let b =
                RobotBundle::load(crate::corpus::bundled_robots_root().join(id)).unwrap();
            let r = run_foundation_reach(
                &b,
                [0.22, 0.0, 0.12],
                0.20,
                10.0,
                0.25,
                None,
                false,
                false,
            )
            .unwrap();
            assert_eq!(r.skill, "REACH");
            assert_eq!(r.adapter_id, "chain_ik_position_pd");
            assert!(!r.metal);
            assert_eq!(r.evidence_status, SIMULATION_ONLY);
            assert_eq!(r.adaptation, "CONFIGURED");
            assert!(r.skill_refuse.is_none(), "{id} {:?}", r.skill_refuse);
            assert!(r.ctrl_writes > 0, "{id}");
            assert!(r.task_success, "{id} must reach under privileged verifier");
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
        let r = run_foundation_reach(
            &b,
            [0.22, 0.0, 0.12],
            0.20,
            10.0,
            0.25,
            None,
            false,
            true,
        )
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
        let r = run_foundation_reach(
            &b,
            [0.22, 0.0, 0.12],
            0.20,
            10.0,
            0.25,
            None,
            true,
            false,
        )
        .unwrap();
        assert_eq!(r.ctrl_writes, 0);
    }

    #[test]
    fn held_out_first_evaluation_is_recorded() {
        if !ensure_mujoco_or_skip() {
            return;
        }
        let b = RobotBundle::load(crate::held_out::held_out_bundle()).unwrap();
        let r = run_foundation_reach(
            &b,
            [0.20, 0.0, 0.12],
            0.25,
            10.0,
            0.25,
            None,
            false,
            false,
        )
        .unwrap();
        assert_eq!(r.adaptation, "CONFIGURED");
        assert!(!r.metal);
        assert!(!r.model_hash.is_empty());
    }
}
