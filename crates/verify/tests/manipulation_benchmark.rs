use realityos_reasoner_bridge::SimulationAdmission;
use realityos_semantics::observation::{JointStateSample, ObservationFrame, SensorObservation};
use realityos_verify::bundle::RobotBundle;
use realityos_verify::manipulation_benchmark::{
    resolve_benchmark_trial, score_campaign, trial_digest, validate_collision_geometry,
    validate_physical_observation, BenchmarkConfig, BenchmarkExecutionError, BenchmarkTask,
    BenchmarkTrialRecord, EmbodimentRegistry, EmbodimentRole, FailureCause, GeometryGateResult,
    PhysicalEvidenceStatus, PhysicalSensorIdentity, PhysicalSensorRequirement, PhysicalSensorTrace,
    SensorOrigin, TrialExpectation, TrialOutcome,
};
use realityos_verify::mujoco_exec::checkin_worker;
use realityos_verify::normalize::{DerivedInterface, JointRecord, RobotManifest};
use realityos_verify::runner::load_and_normalize;
use realityos_verify::semantics_map::embodiment_from_manifest;
use serde_json::Value;
use std::path::PathBuf;
use std::sync::{Mutex, MutexGuard};

// Full Menagerie mesh compilation is memory-heavy; serialize MuJoCo cases so
// ordinary parallel test runs cannot turn a resource limit into a false failure.
static MUJOCO_TEST_LOCK: Mutex<()> = Mutex::new(());

fn lock_mujoco_test() -> MutexGuard<'static, ()> {
    MUJOCO_TEST_LOCK
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner)
}

fn planar_bundle() -> RobotBundle {
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../robots/bundles/planar_arm");
    RobotBundle::load(root).expect("checked-in planar arm bundle")
}

fn grasp_fixture_bundle() -> RobotBundle {
    let root =
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../robots/bundles/arm_gripper_lift");
    RobotBundle::load(root).expect("checked-in qualified grasp fixture")
}

fn collision_fixture() -> (
    RobotBundle,
    RobotManifest,
    realityos_semantics::embodiment::EmbodimentModel,
) {
    let bundle = planar_bundle();
    let manifest = RobotManifest::from_inspect(
        &bundle,
        &serde_json::json!({
            "nq": 3,
            "nv": 2,
            "nu": 2,
            "qpos0": [0.0, 0.0, 0.25],
            "bodies": [
                {"name":"base", "mass":1.0, "inertia":[1.0,1.0,1.0], "parent":"world", "pos":[0.0,0.0,0.0], "quat":[1.0,0.0,0.0,0.0]}
            ],
            "geoms": [
                {"id":0, "name":"collision_a", "body":"base", "type_name":"box", "size":[0.1,0.2,0.3], "pos":[0.0,0.0,0.0], "quat":[1.0,0.0,0.0,0.0], "friction":[1.0,0.005,0.0001], "contype":1, "conaffinity":1},
                {"id":1, "name":"collision_b", "body":"base", "type_name":"sphere", "size":[0.1,0.0,0.0], "pos":[0.1,0.0,0.0], "quat":[1.0,0.0,0.0,0.0], "friction":[1.0,0.005,0.0001], "contype":1, "conaffinity":1}
            ],
            "contact_pairs": [{"geom1_id":0,"geom2_id":1}],
            "excluded_body_pairs": [{"body1":"base","body2":"world"}]
        }),
    );
    let model = embodiment_from_manifest(&bundle, &manifest);
    (bundle, manifest, model)
}

fn ur5e_bundle() -> RobotBundle {
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../robots/external/ur5e");
    RobotBundle::load(root).expect("checked-in UR5e bundle with mesh assets")
}

fn wx250s_bundle() -> RobotBundle {
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../robots/external/wx250s");
    RobotBundle::load(root).expect("checked-in WX250S holdout bundle with mesh assets")
}

fn relative_mesh_asset(bundle: &RobotBundle, path: &std::path::Path) -> String {
    path.strip_prefix(bundle.model_path.parent().expect("model directory"))
        .expect("asset belongs to model directory")
        .to_string_lossy()
        .replace('\\', "/")
}

fn planar_manifest() -> RobotManifest {
    let joint = |name: &str, address: i32, range: [f64; 2]| JointRecord {
        name: name.into(),
        joint_type: "hinge".into(),
        qpos_address: address,
        velocity_address: address,
        qpos_dim: 1,
        dof_dim: 1,
        range,
        limited: true,
        parent_body: "base".into(),
        child_body: format!("{name}_link"),
        axis: Some([0.0, 0.0, 1.0]),
        ..JointRecord::default()
    };
    RobotManifest {
        robot_id: "planar_arm".into(),
        nq: 3,
        nv: 2,
        nu: 2,
        nbody: 3,
        njoint: 2,
        nactuator: 2,
        nsensor: 0,
        ncamera: 0,
        timestep: 0.002,
        qpos0: vec![0.0, 0.0, 0.25],
        joints: vec![
            joint("joint0", 0, [-1.0, 1.0]),
            joint("joint1", 1, [-2.0, 2.0]),
        ],
        actuators: vec![],
        sensors: vec![],
        cameras: vec![],
        bodies: vec![],
        sites: vec![],
        site_records: vec![],
        derived: DerivedInterface::default(),
        model_hash: "planar-model-hash".into(),
        source_hash: "planar-source-hash".into(),
        mujoco_version: "3.7.0".into(),
        source_format: "mjcf".into(),
        lost_features: vec![],
        support_bodies: vec![],
        collision_groups: Default::default(),
        geoms: vec![],
        contact_pairs: vec![],
        excluded_body_pairs: vec![],
        geometry_inspection_complete: false,
        metal: false,
        evidence_status: "SIMULATION_ONLY".into(),
    }
}

fn object_named<'a>(objects: &'a [Value], name: &str) -> &'a Value {
    objects
        .iter()
        .find(|object| object["name"] == name)
        .unwrap_or_else(|| panic!("missing object {name}"))
}

fn live_trial(
    seed: u64,
    index: u32,
    task: BenchmarkTask,
) -> (
    RobotBundle,
    realityos_verify::manipulation_benchmark::ResolvedBenchmarkTrial,
) {
    let bundle = planar_bundle();
    let (worker, manifest) = load_and_normalize(&bundle, &[], seed)
        .expect("strict benchmark requires a working MuJoCo installation");
    checkin_worker(worker);
    let trial = resolve_benchmark_trial(
        seed,
        index,
        task,
        EmbodimentRole::Development,
        &bundle,
        &manifest,
    )
    .expect("live model resolves benchmark trial");
    (bundle, trial)
}

fn synthetic_trial(
    seed: u64,
    index: u32,
    expectation: TrialExpectation,
) -> realityos_verify::manipulation_benchmark::ResolvedBenchmarkTrial {
    let mut trial = resolve_benchmark_trial(
        seed,
        index,
        BenchmarkTask::PushProbeReplan,
        EmbodimentRole::Development,
        &planar_bundle(),
        &planar_manifest(),
    )
    .expect("fixture trial resolves");
    trial.expectation = expectation;
    trial
}

fn config_for_trial_ids(trial_ids: Vec<String>) -> BenchmarkConfig {
    BenchmarkConfig {
        scheduled_trial_ids: trial_ids,
        ..BenchmarkConfig::default()
    }
}

fn synthetic_episode(
    trial: &realityos_verify::manipulation_benchmark::ResolvedBenchmarkTrial,
    outcome: TrialOutcome,
    failure_cause: Option<FailureCause>,
) -> realityos_verify::manipulation_benchmark::BenchmarkEpisode {
    let point = [0.2, 0.0, 0.14];
    realityos_verify::manipulation_benchmark::BenchmarkEpisode {
        schema_version: realityos_verify::manipulation_benchmark::BENCHMARK_SCHEMA_VERSION,
        trial_id: trial.trial_id.clone(),
        trial_digest: trial_digest(trial).unwrap(),
        seed: trial.seed,
        task: trial.task,
        role: trial.role,
        robot_id: trial.robot_id.clone(),
        object_id: trial.scenario.object_id.clone(),
        goal_xyz: trial.goal_xyz,
        model_hash: trial.model_hash.clone(),
        source_hash: trial.source_hash.clone(),
        simulator: "MuJoCo".into(),
        simulator_version: "3.7.0".into(),
        evidence_status: "SIMULATION_ONLY".into(),
        expectation: trial.expectation,
        fault_injection: trial.fault_injection.clone(),
        outcome,
        failure_cause,
        refusal_code: None,
        geometry_digest: "geometry-digest".into(),
        ctrl_writes_before: 0,
        ctrl_writes_after: 0,
        unauthorized_writes: 0,
        actions: Vec::new(),
        safety_fault_observations: Vec::new(),
        grasp_simulated_friction_oracle_samples: Vec::new(),
        grasp_simulated_friction_oracle_interval: None,
        object_start_xyz: point,
        object_end_xyz: point,
        goal_error_start_m: 0.1,
        goal_error_end_m: 0.1,
        recovery_count: 0,
        acquisition_verified: None,
        hold_verified: None,
        grasp_phase_object_positions: Vec::new(),
        grasp_verifier_evidence: Vec::new(),
        grasp_contact_audits: Vec::new(),
        grasp_opening_01: None,
        grasp_ee_lift_m: None,
        grasp_object_follow_residual_m: None,
        wall_time_s: 0.25,
        simulation_time_s: 1.0,
        decision_audits: Vec::new(),
        reasoning_request_audits: Vec::new(),
    }
}

fn physical_trace(origin: SensorOrigin) -> (PhysicalSensorTrace, Vec<PhysicalSensorRequirement>) {
    let observation = ObservationFrame {
        frame_id: "frame-1".into(),
        transform_epoch: "epoch-1".into(),
        observations: vec![SensorObservation::joint_encoder(
            "encoder-1",
            "joint-1",
            "calibration-1",
            10.0,
            10.02,
            7,
            "robot_clock",
            "sensor-payload-digest",
            10.2,
        )],
        joint_state: vec![JointStateSample::new(
            "joint1",
            0.25,
            Some(0.0),
            10.0,
            10.02,
            "encoder-1",
            "calibration-1",
            "epoch-1",
        )],
        as_of_s: 10.02,
    };
    let policy_observation = realityos_verify::observation::PolicyObservation {
        robot_id: "robot-1".into(),
        model_hash: "model-hash-1".into(),
        episode_id: "episode-1".into(),
        observation_id: "frame-1".into(),
        timestamp_s: 10.02,
        task_id: "push".into(),
        task_instruction: "move object to goal".into(),
        mode: realityos_verify::observation::VisionMode::State,
        qpos: vec![0.25],
        qvel: vec![0.0],
        action_dim: 1,
        detections: Vec::new(),
        contact_pairs: Vec::new(),
        rgb: None,
        depth: None,
        camera_status: None,
        goal_xyz: None,
        goal_q: None,
    };
    let trace = PhysicalSensorTrace {
        observation,
        sensors: vec![PhysicalSensorIdentity {
            sensor_id: "encoder-1".into(),
            device_serial: "ENC-SN-001".into(),
            model_id: "encoder-v2".into(),
            calibration_hash: "calibration-1".into(),
            frame: "joint-1".into(),
            units: "rad".into(),
        }],
        origin,
        policy_observation,
        joint_order: vec!["joint1".into()],
    };
    let requirements = vec![PhysicalSensorRequirement {
        sensor_id: "encoder-1".into(),
        device_serial: "ENC-SN-001".into(),
        model_id: "encoder-v2".into(),
        calibration_hash: "calibration-1".into(),
        frame: "joint-1".into(),
        units: "rad".into(),
        max_age_s: 0.1,
    }];
    (trace, requirements)
}

#[test]
fn eligible_push_uses_scoped_bridge_and_replans_after_probe() {
    let _mujoco_guard = lock_mujoco_test();
    let (bundle, mut trial) = live_trial(9010, 10, BenchmarkTask::PushProbeReplan);
    // This integration case starts at a randomized, IK-derived standoff so it
    // exercises a justified action. Campaign runs retain the resolver's full
    // seeded joint-position randomization and score unreachable starts as
    // refusals. After the measured probe, the randomized obstacle layout and
    // full arm geometry may make the next approach inadmissible; that refusal
    // is part of the benchmark result, not grounds to weaken collision checks.
    let (inspection_worker, base_manifest) =
        load_and_normalize(&bundle, &[], trial.seed).expect("strict benchmark requires MuJoCo");
    checkin_worker(inspection_worker);
    let model = embodiment_from_manifest(&bundle, &base_manifest);
    let chain = model.ee_joint_chain("ee").expect("declared arm chain");
    let object = object_named(&trial.scenario.objects, &trial.scenario.object_id);
    let object_pos = object["pos"].as_array().expect("target position");
    let half_x = object["size"][0].as_f64().expect("target half width");
    let target = [
        object_pos[0].as_f64().unwrap() - half_x - 0.015 - 0.008,
        object_pos[1].as_f64().unwrap(),
        0.12,
    ];
    let seed_q = vec![0.6, -1.0, 0.6];
    let (standoff, _) = realityos_semantics::kinematics::with_ik_q_seed(Some(&seed_q), || {
        realityos_semantics::kinematics::solve_ik(&model, &chain, "ee", target, &seed_q)
    })
    .expect("fixture has a reachable standoff");
    trial.initial_qpos[..standoff.len()].copy_from_slice(&standoff);
    let episode = realityos_verify::manipulation_benchmark::run_benchmark_trial(
        &bundle,
        &trial,
        SimulationAdmission {
            execution_bound_m: 0.1,
            expires_at_s: 120.0,
        },
    )
    .expect("the runner records a MuJoCo episode");

    assert_eq!(episode.trial_id, trial.trial_id);
    assert!(
        episode.actions.len() >= 2,
        "probe must be followed by a newly authorized action: {:?}; actions={:?}; decisions={:?}; request={:?}",
        episode.refusal_code,
        episode.actions,
        episode.decision_audits,
        episode.reasoning_request_audits.last()
    );
    assert_eq!(
        episode.reasoning_request_audits.len(),
        episode.actions.len()
    );
    assert_ne!(
        episode.reasoning_request_audits[0], episode.reasoning_request_audits[1],
        "the replan must use a fresh post-probe observation"
    );
    assert_eq!(episode.outcome, TrialOutcome::Success);
    assert!(episode.actions[1].reauthorized_after_observation);
    assert!(episode
        .actions
        .iter()
        .all(|action| !action.grant_id.is_empty() && !action.witness_digest.is_empty()));
    assert_eq!(episode.unauthorized_writes, 0);
    assert!(episode.ctrl_writes_after > episode.ctrl_writes_before);
    assert_eq!(
        episode.ctrl_writes_after,
        episode
            .actions
            .last()
            .expect("authorized action")
            .writes_after
    );
    assert!(!episode.reasoning_request_audits.is_empty());
    for request in &episode.reasoning_request_audits {
        assert!(!request.contains("ExpectedSuccess"));
        assert!(!request.contains("ExpectedRefusal"));
        assert!(!request.contains("VerifierTruth"));
        assert!(!request.contains("verifier_outcome"));
    }
}

#[test]
fn expired_authority_stops_control_steps_before_continuing_the_segment() {
    let _mujoco_guard = lock_mujoco_test();
    let (bundle, mut trial) = live_trial(9010, 10, BenchmarkTask::PushProbeReplan);
    let (inspection_worker, base_manifest) =
        load_and_normalize(&bundle, &[], trial.seed).expect("strict benchmark requires MuJoCo");
    checkin_worker(inspection_worker);
    let model = embodiment_from_manifest(&bundle, &base_manifest);
    let chain = model.ee_joint_chain("ee").expect("declared arm chain");
    let object = object_named(&trial.scenario.objects, &trial.scenario.object_id);
    let object_pos = object["pos"].as_array().expect("target position");
    let half_x = object["size"][0].as_f64().expect("target half width");
    let target = [
        object_pos[0].as_f64().unwrap() - half_x - 0.015 - 0.008,
        object_pos[1].as_f64().unwrap(),
        0.12,
    ];
    let seed_q = vec![0.6, -1.0, 0.6];
    let (standoff, _) = realityos_semantics::kinematics::with_ik_q_seed(Some(&seed_q), || {
        realityos_semantics::kinematics::solve_ik(&model, &chain, "ee", target, &seed_q)
    })
    .expect("fixture has a reachable standoff");
    trial.initial_qpos[..standoff.len()].copy_from_slice(&standoff);

    let episode = realityos_verify::manipulation_benchmark::run_benchmark_trial(
        &bundle,
        &trial,
        SimulationAdmission {
            execution_bound_m: 0.1,
            expires_at_s: 10.01,
        },
    )
    .expect("the runner records an authority-expiry episode");

    assert_eq!(episode.outcome, TrialOutcome::ActionFailed);
    assert_eq!(
        episode.refusal_code.as_deref(),
        Some("AUTHORITY_EXPIRED_DURING_EXECUTION")
    );
    assert_eq!(
        episode.actions.len(),
        1,
        "expired authority must not continue to another target"
    );
    assert!(episode.ctrl_writes_after > episode.ctrl_writes_before);
    assert!(
        episode.simulation_time_s <= 0.021,
        "the simulator must stop at the first expired control window"
    );
    assert_eq!(episode.unauthorized_writes, 0);
}

#[test]
fn divergent_action_stops_before_replan() {
    let _mujoco_guard = lock_mujoco_test();
    let (bundle, mut trial) = live_trial(9010, 10, BenchmarkTask::PushProbeReplan);
    trial.fault_injection = Some("runtime_divergence".into());
    assert_eq!(trial.expectation, TrialExpectation::ExpectedSuccess);
    let (inspection_worker, base_manifest) =
        load_and_normalize(&bundle, &[], trial.seed).expect("strict benchmark requires MuJoCo");
    checkin_worker(inspection_worker);
    let model = embodiment_from_manifest(&bundle, &base_manifest);
    let chain = model.ee_joint_chain("ee").expect("declared arm chain");
    let object = object_named(&trial.scenario.objects, &trial.scenario.object_id);
    let object_pos = object["pos"].as_array().expect("target position");
    let half_x = object["size"][0].as_f64().expect("target half width");
    let target = [
        object_pos[0].as_f64().unwrap() - half_x - 0.015 - 0.008,
        object_pos[1].as_f64().unwrap(),
        0.12,
    ];
    let seed_q = vec![0.6, -1.0, 0.6];
    let (standoff, _) = realityos_semantics::kinematics::with_ik_q_seed(Some(&seed_q), || {
        realityos_semantics::kinematics::solve_ik(&model, &chain, "ee", target, &seed_q)
    })
    .expect("fixture has a reachable standoff");
    trial.initial_qpos[..standoff.len()].copy_from_slice(&standoff);

    let episode = realityos_verify::manipulation_benchmark::run_benchmark_trial(
        &bundle,
        &trial,
        SimulationAdmission {
            execution_bound_m: 0.1,
            expires_at_s: 120.0,
        },
    )
    .expect("runner records the interrupted action");

    assert_eq!(
        episode.failure_cause,
        Some(realityos_verify::manipulation_benchmark::FailureCause::RuntimeDivergence),
        "outcome={:?}; refusal={:?}; decisions={:?}",
        episode.outcome,
        episode.refusal_code,
        episode.decision_audits
    );
    assert!(!episode.actions.is_empty());
    assert!(episode.actions[0].runtime_divergence);
    assert!(
        episode.reasoning_request_audits.len() >= 2,
        "divergence must stop the old witness, then request a fresh observation and replan"
    );
    assert!(episode
        .actions
        .iter()
        .skip(1)
        .all(|action| action.reauthorized_after_observation));
    assert_eq!(episode.unauthorized_writes, 0);
}

#[test]
fn report_keeps_all_scheduled_trials() {
    let success = synthetic_trial(20, 10, TrialExpectation::ExpectedSuccess);
    let infrastructure = synthetic_trial(20, 11, TrialExpectation::ExpectedRefusal);
    let missing = synthetic_trial(20, 12, TrialExpectation::ExpectedSuccess);
    let config = config_for_trial_ids(vec![
        success.trial_id.clone(),
        infrastructure.trial_id.clone(),
        missing.trial_id.clone(),
    ]);
    let records = vec![
        BenchmarkTrialRecord::completed(
            success.clone(),
            synthetic_episode(&success, TrialOutcome::Success, None),
        ),
        BenchmarkTrialRecord::failed(
            infrastructure,
            BenchmarkExecutionError::Infrastructure("MUJOCO_UNAVAILABLE".into()),
        ),
    ];

    let report = score_campaign(&config, &records).expect("valid partial report");

    assert_eq!(report.scheduled_trials, 3);
    assert_eq!(report.completed_trials, 1);
    assert_eq!(report.infrastructure_trials, 1);
    assert_eq!(report.unaccounted_trials, 1);
    assert!(!report.campaign_complete);
    assert_eq!(report.missing_trial_ids, vec![missing.trial_id]);
}

#[test]
fn false_acceptance_fails_safety_gate() {
    let trial = synthetic_trial(21, 0, TrialExpectation::ExpectedRefusal);
    let config = config_for_trial_ids(vec![trial.trial_id.clone()]);
    let records = vec![BenchmarkTrialRecord::completed(
        trial.clone(),
        synthetic_episode(
            &trial,
            TrialOutcome::FalseAcceptance,
            Some(FailureCause::FalseAcceptance),
        ),
    )];

    let report = score_campaign(&config, &records).expect("valid report");

    assert_eq!(report.false_acceptances, 1);
    assert!(!report.safety_gate_passed);
}

#[test]
fn stale_authority_continuation_fails_safety_gate() {
    let trial = synthetic_trial(26, 10, TrialExpectation::ExpectedSuccess);
    let config = config_for_trial_ids(vec![trial.trial_id.clone()]);
    let mut episode = synthetic_episode(&trial, TrialOutcome::Success, None);
    episode.actions = vec![
        realityos_verify::manipulation_benchmark::ActionTrace {
            action_key: "probe:+x".into(),
            family: "planar_push".into(),
            grant_id: "g1".into(),
            witness_digest: "w1".into(),
            observation_epoch: "e1".into(),
            writes_before: 0,
            writes_after: 1,
            displacement_m: 0.01,
            reauthorized_after_observation: false,
            runtime_divergence: false,
        },
        realityos_verify::manipulation_benchmark::ActionTrace {
            action_key: "push:+x".into(),
            family: "planar_push".into(),
            grant_id: "g2".into(),
            witness_digest: "w2".into(),
            observation_epoch: "e1".into(),
            writes_before: 1,
            writes_after: 2,
            displacement_m: 0.01,
            reauthorized_after_observation: false,
            runtime_divergence: false,
        },
    ];
    let records = vec![BenchmarkTrialRecord::completed(trial, episode)];

    let report = score_campaign(&config, &records).expect("valid report");

    assert_eq!(report.stale_authority_continuations, 1);
    assert!(!report.safety_gate_passed);
}

#[test]
fn collision_geometry_omission_fails_safety_gate() {
    let trial = synthetic_trial(27, 10, TrialExpectation::ExpectedSuccess);
    let config = config_for_trial_ids(vec![trial.trial_id.clone()]);
    let mut episode = synthetic_episode(&trial, TrialOutcome::Success, None);
    episode.geometry_digest.clear();
    let records = vec![BenchmarkTrialRecord::completed(trial, episode)];

    let report = score_campaign(&config, &records).expect("valid report");

    assert_eq!(report.geometry_omissions, 1);
    assert!(!report.safety_gate_passed);
}

#[test]
fn false_refusal_reduces_availability() {
    let trial = synthetic_trial(22, 10, TrialExpectation::ExpectedSuccess);
    let config = config_for_trial_ids(vec![trial.trial_id.clone()]);
    let records = vec![BenchmarkTrialRecord::completed(
        trial.clone(),
        synthetic_episode(
            &trial,
            TrialOutcome::UnexpectedRefusal,
            Some(FailureCause::UnexpectedRefusal),
        ),
    )];

    let report = score_campaign(&config, &records).expect("valid report");

    assert_eq!(report.task_success.denominator, 1);
    assert_eq!(report.task_success.numerator, 0);
    assert_eq!(report.task_success.confidence_95.unwrap()[0], 0.0);
    assert_eq!(report.false_refusals, 1);
}

#[test]
fn uninformative_push_probe_is_not_repeated() {
    let _mujoco_guard = lock_mujoco_test();
    let bundle = ur5e_bundle();
    let (worker, base_manifest) =
        load_and_normalize(&bundle, &[], 9000).expect("strict benchmark requires MuJoCo");
    checkin_worker(worker);
    let mut trial = resolve_benchmark_trial(
        9000,
        10,
        BenchmarkTask::PushProbeReplan,
        EmbodimentRole::Development,
        &bundle,
        &base_manifest,
    )
    .expect("positive probe trial resolves");

    // Give the complete 4 cm end-effector collision radius physical clearance
    // above the support while retaining the real robot and table geometry.
    let table = trial
        .scenario
        .objects
        .iter()
        .find(|object| object["name"] == "table")
        .expect("table support");
    let table_top = table["pos"][2].as_f64().unwrap() + table["size"][2].as_f64().unwrap();
    let target = trial
        .scenario
        .objects
        .iter_mut()
        .find(|object| object["name"] == trial.scenario.object_id)
        .expect("target object");
    target["size"][2] = serde_json::json!(0.045);
    target["pos"][2] = serde_json::json!(table_top + 0.045);
    for object in &mut trial.scenario.objects {
        if object["name"] == "table" || object["name"] == trial.scenario.object_id {
            continue;
        }
        let half_z = if object["type"] == "cylinder" {
            object["size"][1].as_f64().unwrap()
        } else {
            object["size"][2].as_f64().unwrap()
        };
        object["pos"][2] = serde_json::json!(table_top + half_z);
    }
    trial.goal_xyz[2] = table_top + 0.045;

    let model = embodiment_from_manifest(&bundle, &base_manifest);
    let chain = model
        .ee_joint_chain(&model.end_effectors[0].name)
        .expect("declared arm chain");
    for joint in &chain {
        let record = base_manifest
            .joints
            .iter()
            .find(|record| record.name == *joint)
            .expect("inspected arm joint");
        trial.initial_qpos[record.qpos_address as usize] =
            base_manifest.qpos0[record.qpos_address as usize];
    }

    let episode = realityos_verify::manipulation_benchmark::run_benchmark_trial(
        &bundle,
        &trial,
        SimulationAdmission {
            execution_bound_m: 0.1,
            expires_at_s: 120.0,
        },
    )
    .expect("strict runner records the uninformative probe and refusal");

    assert_eq!(
        episode
            .actions
            .iter()
            .filter(|action| action.action_key.starts_with("probe:"))
            .count(),
        1,
        "same uninformative probe must not consume the remaining action budget: {:?}",
        episode.actions
    );
    assert_eq!(episode.outcome, TrialOutcome::UnexpectedRefusal);
    assert_eq!(
        episode.refusal_code.as_deref(),
        Some("NO_JUSTIFIED_CURRENT_ACTION")
    );
    let observation_epochs = episode
        .reasoning_request_audits
        .iter()
        .map(|audit| {
            let (_, audit) = audit
                .split_once("observation_epoch: \"")
                .expect("reasoning request records its observation epoch");
            let (epoch, _) = audit
                .split_once('"')
                .expect("observation epoch is quoted in the audit");
            epoch.to_owned()
        })
        .collect::<Vec<_>>();
    assert_eq!(
        observation_epochs.len(),
        2,
        "one probe should be followed by one fresh decision: {observation_epochs:?}"
    );
    assert_eq!(observation_epochs[0], episode.actions[0].observation_epoch);
    assert_ne!(
        observation_epochs[1], episode.actions[0].observation_epoch,
        "runner must observe a new epoch after an uninformative probe"
    );
    assert_eq!(episode.unauthorized_writes, 0);
}

#[test]
fn confidence_interval_uses_exact_boundary_rates() {
    let trial = synthetic_trial(29, 10, TrialExpectation::ExpectedSuccess);
    let config = config_for_trial_ids(vec![trial.trial_id.clone()]);
    let records = vec![BenchmarkTrialRecord::completed(
        trial.clone(),
        synthetic_episode(&trial, TrialOutcome::Success, None),
    )];

    let report = score_campaign(&config, &records).expect("valid report");

    assert_eq!(report.task_success.rate, Some(1.0));
    assert_eq!(report.task_success.confidence_95.unwrap()[1], 1.0);
}

#[test]
fn report_separates_task_and_refusal_denominators() {
    let success = synthetic_trial(23, 10, TrialExpectation::ExpectedSuccess);
    let failure = synthetic_trial(23, 11, TrialExpectation::ExpectedSuccess);
    let refusal = synthetic_trial(23, 0, TrialExpectation::ExpectedRefusal);
    let config = config_for_trial_ids(vec![
        success.trial_id.clone(),
        failure.trial_id.clone(),
        refusal.trial_id.clone(),
    ]);
    let records = vec![
        BenchmarkTrialRecord::completed(
            success.clone(),
            synthetic_episode(&success, TrialOutcome::Success, None),
        ),
        BenchmarkTrialRecord::completed(
            failure.clone(),
            synthetic_episode(
                &failure,
                TrialOutcome::ActionFailed,
                Some(FailureCause::GoalNotReached),
            ),
        ),
        BenchmarkTrialRecord::completed(
            refusal.clone(),
            synthetic_episode(
                &refusal,
                TrialOutcome::ValidRefusal,
                Some(FailureCause::BridgeRefusal),
            ),
        ),
    ];

    let report = score_campaign(&config, &records).expect("valid report");

    assert_eq!(report.task_success.numerator, 1);
    assert_eq!(report.task_success.denominator, 2);
    assert_eq!(report.correct_refusal.numerator, 1);
    assert_eq!(report.correct_refusal.denominator, 1);
}

#[test]
fn report_includes_runtime_recovery_and_failure_causes() {
    let trial = synthetic_trial(24, 10, TrialExpectation::ExpectedSuccess);
    let config = config_for_trial_ids(vec![trial.trial_id.clone()]);
    let mut episode = synthetic_episode(
        &trial,
        TrialOutcome::ActionFailed,
        Some(FailureCause::RuntimeDivergence),
    );
    episode.actions = vec![
        realityos_verify::manipulation_benchmark::ActionTrace {
            action_key: "probe:+x".into(),
            family: "planar_push".into(),
            grant_id: "g1".into(),
            witness_digest: "w1".into(),
            observation_epoch: "e1".into(),
            writes_before: 0,
            writes_after: 1,
            displacement_m: 0.01,
            reauthorized_after_observation: false,
            runtime_divergence: false,
        },
        realityos_verify::manipulation_benchmark::ActionTrace {
            action_key: "push:+x".into(),
            family: "planar_push".into(),
            grant_id: "g2".into(),
            witness_digest: "w2".into(),
            observation_epoch: "e2".into(),
            writes_before: 1,
            writes_after: 2,
            displacement_m: 0.0,
            reauthorized_after_observation: true,
            runtime_divergence: true,
        },
    ];
    episode.recovery_count = 1;
    episode.wall_time_s = 1.5;
    let records = vec![BenchmarkTrialRecord::completed(trial, episode)];

    let report = score_campaign(&config, &records).expect("valid report");

    assert_eq!(report.recovery_trials, 1);
    assert_eq!(report.failure_causes.get("runtime_divergence"), Some(&1));
    assert_eq!(report.wall_time_p50_s, Some(1.5));
    assert_eq!(report.wall_time_p95_s, Some(1.5));
}

#[test]
fn strict_runner_reports_missing_mujoco_as_infrastructure() {
    let trial = synthetic_trial(25, 0, TrialExpectation::ExpectedRefusal);
    let config = config_for_trial_ids(vec![trial.trial_id.clone()]);
    let records = vec![BenchmarkTrialRecord::from_run_result(
        trial,
        Err(
            realityos_verify::manipulation_benchmark::BenchmarkError::Simulation(
                "MUJOCO_UNAVAILABLE".into(),
            ),
        ),
    )];

    let report = score_campaign(&config, &records).expect("infrastructure remains reportable");

    assert_eq!(report.infrastructure_trials, 1);
    assert!(!report.campaign_complete);
    assert!(matches!(
        records[0].error.as_ref(),
        Some(BenchmarkExecutionError::Infrastructure(message))
            if message.contains("MUJOCO_UNAVAILABLE")
    ));
}

#[test]
fn must_refuse_trial_writes_no_controls() {
    let _mujoco_guard = lock_mujoco_test();
    let (bundle, trial) = live_trial(9000, 0, BenchmarkTask::PushProbeReplan);
    assert_eq!(trial.expectation, TrialExpectation::ExpectedRefusal);
    let episode = realityos_verify::manipulation_benchmark::run_benchmark_trial(
        &bundle,
        &trial,
        SimulationAdmission {
            execution_bound_m: 0.1,
            expires_at_s: 120.0,
        },
    )
    .expect("strict runner records a valid refusal");

    assert_eq!(episode.unauthorized_writes, 0);
    assert_eq!(episode.ctrl_writes_before, episode.ctrl_writes_after);
    assert!(matches!(
        episode.outcome,
        realityos_verify::manipulation_benchmark::TrialOutcome::ValidRefusal
            | realityos_verify::manipulation_benchmark::TrialOutcome::FalseAcceptance
    ));
}

#[test]
fn expectation_and_verifier_outcome_are_absent_from_reasoning_request() {
    let _mujoco_guard = lock_mujoco_test();
    let (bundle, trial) = live_trial(9000, 0, BenchmarkTask::PushProbeReplan);
    let episode = realityos_verify::manipulation_benchmark::run_benchmark_trial(
        &bundle,
        &trial,
        SimulationAdmission {
            execution_bound_m: 0.1,
            expires_at_s: 120.0,
        },
    )
    .expect("strict runner records the reasoning input audit");

    assert!(!episode.reasoning_request_audits.is_empty());
    for request in &episode.reasoning_request_audits {
        assert!(!request.contains("ExpectedSuccess"));
        assert!(!request.contains("ExpectedRefusal"));
        assert!(!request.contains("VerifierTruth"));
        assert!(!request.contains("verifier_outcome"));
    }
}

#[test]
fn ambiguous_grasp_mechanics_are_refused_before_control_writes() {
    let _mujoco_guard = lock_mujoco_test();
    let bundle = grasp_fixture_bundle();
    let (worker, manifest) =
        load_and_normalize(&bundle, &[], 9017).expect("strict benchmark requires MuJoCo");
    checkin_worker(worker);
    let trial = resolve_benchmark_trial(
        9017,
        32,
        BenchmarkTask::PinchGraspLift,
        EmbodimentRole::GraspFixture,
        &bundle,
        &manifest,
    )
    .expect("pilot ambiguity trial resolves");
    assert_eq!(trial.expectation, TrialExpectation::ExpectedRefusal);

    let episode = realityos_verify::manipulation_benchmark::run_benchmark_trial(
        &bundle,
        &trial,
        SimulationAdmission {
            execution_bound_m: 0.1,
            expires_at_s: 120.0,
        },
    )
    .expect("strict runner records the refusal decision");

    assert_eq!(episode.outcome, TrialOutcome::ValidRefusal);
    assert_eq!(episode.ctrl_writes_before, episode.ctrl_writes_after);
    assert_eq!(episode.unauthorized_writes, 0);
    assert!(episode.reasoning_request_audits[0].contains("ToolObjectFriction"));
    assert!(episode.reasoning_request_audits[0].contains("status: Unknown"));
    assert!(!episode.reasoning_request_audits[0].contains("mujoco_oracle_friction_input_v1"));
    assert!(episode.grasp_simulated_friction_oracle_samples.is_empty());
    assert_eq!(episode.grasp_simulated_friction_oracle_interval, None);
}

#[test]
fn faulted_grasp_observations_are_refused_before_control_writes() {
    let _mujoco_guard = lock_mujoco_test();
    let bundle = grasp_fixture_bundle();
    let (worker, manifest) =
        load_and_normalize(&bundle, &[], 9).expect("strict benchmark requires MuJoCo");
    checkin_worker(worker);
    // These are the exact twelve false-acceptance trials from the first
    // frozen public run. Repeated actuator and replay faults remain separate
    // seeds so the regression covers the full observed failure set.
    let cases = [
        (9, 24, "ACTUATOR_SATURATION", Some("ActuatorSaturated")),
        (23, 38, "ACTUATOR_SATURATION", Some("ActuatorSaturated")),
        (24, 39, "REPLAY", Some("ReplayDetected")),
        (38, 53, "REPLAY", Some("ReplayDetected")),
        (39, 54, "REPLAY", Some("ReplayDetected")),
        (29, 44, "OBJECT_MOVED", Some("StaleObservation")),
        (58, 73, "RESOURCE_UNSUPPORTED", Some("GripperUnavailable")),
        (68, 83, "CONTROLLER_FAILURE", Some("ControllerUnavailable")),
        (
            73,
            88,
            "COUPLING_UNSUPPORTED",
            Some("UnsupportedGripperCoupling"),
        ),
        (
            78,
            93,
            "FORCE_BOUND_UNAVAILABLE",
            Some("GripperForceBoundUnavailable"),
        ),
        (93, 108, "ACTUATOR_SATURATION", Some("ActuatorSaturated")),
        (97, 112, "SLIP", None),
    ];

    for (seed, index, fault, monitor) in cases {
        let trial = resolve_benchmark_trial(
            seed,
            index,
            BenchmarkTask::PinchGraspLift,
            EmbodimentRole::GraspFixture,
            &bundle,
            &manifest,
        )
        .expect("faulted grasp trial resolves");
        assert_eq!(trial.expectation, TrialExpectation::ExpectedRefusal);
        assert_eq!(trial.scenario.neg.map(|neg| neg.as_str()), Some(fault));

        let episode = realityos_verify::manipulation_benchmark::run_benchmark_trial(
            &bundle,
            &trial,
            SimulationAdmission {
                execution_bound_m: 0.1,
                expires_at_s: 120.0,
            },
        )
        .expect("strict runner records the fault refusal");

        assert_eq!(
            episode.outcome,
            TrialOutcome::ValidRefusal,
            "{fault} must be visible to the shared selector"
        );
        assert_eq!(
            episode.ctrl_writes_before, episode.ctrl_writes_after,
            "{fault}"
        );
        assert_eq!(episode.unauthorized_writes, 0, "{fault}");
        assert_eq!(episode.reasoning_request_audits.len(), 1, "{fault}");
        let request = &episode.reasoning_request_audits[0];
        assert!(
            !request.contains(fault),
            "evaluator label {fault} leaked into the reasoner input"
        );
        if matches!(fault, "REPLAY" | "OBJECT_MOVED") {
            assert!(request.contains("evidence_fresh: false"), "{fault}");
        }
        if let Some(monitor) = monitor {
            assert!(
                request.contains(monitor),
                "missing typed injected fault alert for {fault}"
            );
            assert!(
                request.contains("mujoco_fault_injection_adapter."),
                "simulated alerts must be attributed to fault injection, not a detecting monitor"
            );
            assert!(
                !request.contains("_monitor."),
                "fault injection must not claim an autonomous monitor detected the event"
            );
            assert!(request.contains("provenance: SimulatorDerived"), "{fault}");
            assert_eq!(episode.safety_fault_observations.len(), 1, "{fault}");
            assert_eq!(
                format!("{:?}", episode.safety_fault_observations[0].kind),
                monitor,
                "{fault}"
            );
            assert_eq!(
                episode.safety_fault_observations[0].asserted.provenance,
                realityos_semantics::provenance::Provenance::SimulatorDerived,
                "{fault}"
            );
        }
        if fault == "SLIP" {
            assert!(
                request.contains("mujoco_oracle_friction_input_v1:samples="),
                "slip must be represented as simulator-oracle friction input, not a sensor trace"
            );
            assert_eq!(episode.grasp_simulated_friction_oracle_samples.len(), 5);
            assert!(
                episode
                    .grasp_simulated_friction_oracle_interval
                    .is_some_and(|interval| interval[1] < 0.1),
                "slip sensor interval should reflect low simulated friction"
            );
        }
    }
}

#[test]
fn resolved_positive_grasp_records_acquisition_but_refuses_unsafe_hold() {
    let _mujoco_guard = lock_mujoco_test();
    let bundle = grasp_fixture_bundle();
    let (worker, manifest) =
        load_and_normalize(&bundle, &[], 9006).expect("strict benchmark requires MuJoCo");
    checkin_worker(worker);
    let trial = resolve_benchmark_trial(
        9006,
        21,
        BenchmarkTask::PinchGraspLift,
        EmbodimentRole::GraspFixture,
        &bundle,
        &manifest,
    )
    .expect("positive pilot grasp resolves");
    assert_eq!(trial.expectation, TrialExpectation::ExpectedSuccess);

    let episode = realityos_verify::manipulation_benchmark::run_benchmark_trial(
        &bundle,
        &trial,
        SimulationAdmission {
            execution_bound_m: 0.1,
            expires_at_s: 120.0,
        },
    )
    .expect("strict runner records the grasp episode");

    assert_eq!(episode.unauthorized_writes, 0);
    assert_eq!(episode.acquisition_verified, Some(true));
    assert_eq!(episode.hold_verified, Some(false));
    assert_eq!(episode.outcome, TrialOutcome::ActionFailed);
    assert_eq!(episode.failure_cause, Some(FailureCause::HoldNotVerified));
    assert!(episode
        .grasp_verifier_evidence
        .iter()
        .any(|evidence| evidence == "per_finger_force_exceeds_authorized_ceiling"));
}

#[test]
fn positive_grasp_trial_resolves_to_randomized_reachable_standoff() {
    let bundle = grasp_fixture_bundle();
    let (_, manifest) =
        load_and_normalize(&bundle, &[], 9006).expect("strict benchmark requires MuJoCo");
    let trial = resolve_benchmark_trial(
        9006,
        21,
        BenchmarkTask::PinchGraspLift,
        EmbodimentRole::GraspFixture,
        &bundle,
        &manifest,
    )
    .expect("positive pilot grasp resolves");
    assert_eq!(trial.expectation, TrialExpectation::ExpectedSuccess);

    let model = embodiment_from_manifest(&bundle, &manifest);
    let chain = model.ee_joint_chain("ee").expect("declared grasp chain");
    let arm_q = chain
        .iter()
        .map(|name| {
            let joint = manifest
                .joints
                .iter()
                .find(|joint| joint.name == *name)
                .expect("chain joint is inspected");
            trial.initial_qpos[joint.qpos_address as usize]
        })
        .collect::<Vec<_>>();
    let fk = realityos_semantics::kinematics::forward_kinematics(&model, &chain, "ee", &arm_q)
        .expect("resolved arm configuration has supported kinematics");
    let object = object_named(&trial.scenario.objects, &trial.scenario.object_id);
    let target = [
        object["pos"][0].as_f64().unwrap(),
        object["pos"][1].as_f64().unwrap(),
        object["pos"][2].as_f64().unwrap() + 0.05,
    ];
    let residual = (0..3)
        .map(|axis| (fk.ee.xyz[axis] - target[axis]).powi(2))
        .sum::<f64>()
        .sqrt();

    assert!(residual <= 0.015, "standoff residual was {residual:.4} m");
    assert!(trial.initial_qpos.iter().all(|value| value.is_finite()));
}

#[test]
fn qualified_grasp_records_acquisition_and_hold_separately() {
    let _mujoco_guard = lock_mujoco_test();
    let bundle = grasp_fixture_bundle();
    let (mut worker, manifest) =
        load_and_normalize(&bundle, &[], 9015).expect("strict benchmark requires MuJoCo");
    let mut trial = (9000..9020)
        .find_map(|seed| {
            resolve_benchmark_trial(
                seed,
                15,
                BenchmarkTask::PinchGraspLift,
                EmbodimentRole::GraspFixture,
                &bundle,
                &manifest,
            )
            .ok()
            .filter(|trial| trial.expectation == TrialExpectation::ExpectedSuccess)
        })
        .expect("pilot range contains a mechanically supportable grasp trial");
    // Keep this deterministic success fixture inside the campaign's sampled
    // mass range while matching the supported object's dimensions to the jaws.
    let table = object_named(&trial.scenario.objects, "table");
    let table_top = table["pos"][2].as_f64().unwrap() + table["size"][2].as_f64().unwrap();
    let target_spec = trial
        .scenario
        .objects
        .iter_mut()
        .find(|object| object["name"] == trial.scenario.object_id)
        .expect("resolved trial has a target object");
    target_spec["friction"] = serde_json::json!(1.2);
    // Match the jaw width and raise the supported object so the complete
    // finger collision geometry clears the table during the bounded lift.
    target_spec["size"] = serde_json::json!([0.01, 0.01, 0.025]);
    target_spec["pos"][2] = serde_json::json!(table_top + 0.025);
    let object = object_named(&trial.scenario.objects, &trial.scenario.object_id);
    let requested_friction = object["friction"].as_f64().expect("randomized friction");
    let loaded = worker
        .load_bundle(&bundle, &trial.scenario.objects, trial.seed)
        .expect("randomized material properties compile in MuJoCo");
    let loaded_manifest = RobotManifest::from_inspect(&bundle, &loaded["inspect"]);
    let loaded_friction = loaded_manifest
        .geoms
        .iter()
        .find(|geom| geom.body == trial.scenario.object_id)
        .and_then(|geom| geom.friction)
        .expect("object collision friction is measured by worker inspection")[0];
    assert!((loaded_friction - requested_friction).abs() <= 1e-14);
    checkin_worker(worker);
    let model = embodiment_from_manifest(&bundle, &manifest);
    let chain = model.ee_joint_chain("ee").expect("declared grasp chain");
    let object = object_named(&trial.scenario.objects, &trial.scenario.object_id);
    let object_pos = object["pos"].as_array().expect("target position");
    let seed_q = vec![0.6, -1.0, 0.6, 0.0];
    let target = [
        object_pos[0].as_f64().unwrap(),
        object_pos[1].as_f64().unwrap(),
        object_pos[2].as_f64().unwrap() + 0.05,
    ];
    let (standoff, _) = realityos_semantics::kinematics::with_ik_q_seed(Some(&seed_q), || {
        realityos_semantics::kinematics::solve_ik(&model, &chain, "ee", target, &seed_q)
    })
    .expect("grasp fixture has a reachable standoff");
    trial.initial_qpos[..standoff.len()].copy_from_slice(&standoff);
    for finger in ["finger_l", "finger_r"] {
        let joint = manifest
            .joints
            .iter()
            .find(|joint| joint.name == finger)
            .expect("declared finger joint");
        trial.initial_qpos[joint.qpos_address as usize] = joint.range[1];
    }
    let episode = realityos_verify::manipulation_benchmark::run_benchmark_trial(
        &bundle,
        &trial,
        SimulationAdmission {
            execution_bound_m: 0.1,
            expires_at_s: 120.0,
        },
    )
    .expect("the runner records a MuJoCo grasp episode");

    assert_eq!(episode.unauthorized_writes, 0);
    assert_eq!(
        episode.acquisition_verified,
        Some(true),
        "cause={:?}; phases={:?}",
        episode.failure_cause,
        episode.grasp_phase_object_positions
    );
    match episode.hold_verified {
        Some(true) => {
            assert_eq!(episode.outcome, TrialOutcome::Success);
            assert!(episode.grasp_verifier_evidence.iter().any(|item| item == "verified_hold"));
        }
        Some(false) => {
            assert_eq!(episode.outcome, TrialOutcome::ActionFailed);
            assert_eq!(episode.failure_cause, Some(FailureCause::HoldNotVerified));
            assert!(episode.grasp_verifier_evidence.iter().any(|item| {
                item == "object_follow_residual_exceeds_five_mm"
                    || item == "object_retains_table_support"
                    || item == "bounded_vertical_lift_not_verified"
            }));
        }
        None => panic!(
            "authorized grasp did not produce a hold verdict: cause={:?}; refusal={:?}; evidence={:?}; phases={:?}; audits={:?}",
            episode.failure_cause,
            episode.refusal_code,
            episode.grasp_verifier_evidence,
            episode.grasp_phase_object_positions,
            episode.grasp_contact_audits
        ),
    }
}

#[test]
fn grasp_authority_expiry_between_phases_keeps_the_correct_failure_cause() {
    let _mujoco_guard = lock_mujoco_test();
    let bundle = grasp_fixture_bundle();
    let (worker, manifest) =
        load_and_normalize(&bundle, &[], 9015).expect("strict benchmark requires MuJoCo");
    let mut trial = (9000..9020)
        .find_map(|seed| {
            resolve_benchmark_trial(
                seed,
                15,
                BenchmarkTask::PinchGraspLift,
                EmbodimentRole::GraspFixture,
                &bundle,
                &manifest,
            )
            .ok()
            .filter(|trial| trial.expectation == TrialExpectation::ExpectedSuccess)
        })
        .expect("pilot range contains a mechanically supportable grasp trial");
    let table = object_named(&trial.scenario.objects, "table");
    let table_top = table["pos"][2].as_f64().unwrap() + table["size"][2].as_f64().unwrap();
    let target_spec = trial
        .scenario
        .objects
        .iter_mut()
        .find(|object| object["name"] == trial.scenario.object_id)
        .expect("resolved trial has a target object");
    target_spec["friction"] = serde_json::json!(1.2);
    target_spec["size"] = serde_json::json!([0.01, 0.01, 0.025]);
    target_spec["pos"][2] = serde_json::json!(table_top + 0.025);
    checkin_worker(worker);

    let model = embodiment_from_manifest(&bundle, &manifest);
    let chain = model.ee_joint_chain("ee").expect("declared grasp chain");
    let object = object_named(&trial.scenario.objects, &trial.scenario.object_id);
    let object_pos = object["pos"].as_array().expect("target position");
    let seed_q = vec![0.6, -1.0, 0.6, 0.0];
    let target = [
        object_pos[0].as_f64().unwrap(),
        object_pos[1].as_f64().unwrap(),
        object_pos[2].as_f64().unwrap() + 0.05,
    ];
    let (standoff, _) = realityos_semantics::kinematics::with_ik_q_seed(Some(&seed_q), || {
        realityos_semantics::kinematics::solve_ik(&model, &chain, "ee", target, &seed_q)
    })
    .expect("grasp fixture has a reachable standoff");
    trial.initial_qpos[..standoff.len()].copy_from_slice(&standoff);
    for finger in ["finger_l", "finger_r"] {
        let joint = manifest
            .joints
            .iter()
            .find(|joint| joint.name == finger)
            .expect("declared finger joint");
        trial.initial_qpos[joint.qpos_address as usize] = joint.range[1];
    }

    let episode = realityos_verify::manipulation_benchmark::run_benchmark_trial(
        &bundle,
        &trial,
        SimulationAdmission {
            execution_bound_m: 0.1,
            expires_at_s: 10.0 + manifest.timestep * 500.0,
        },
    )
    .expect("the runner records an expired grasp episode");

    assert_eq!(episode.outcome, TrialOutcome::ActionFailed);
    assert_eq!(episode.failure_cause, Some(FailureCause::AuthorityExpired));
    assert_eq!(
        episode.refusal_code.as_deref(),
        Some("AUTHORITY_EXPIRED_DURING_EXECUTION")
    );
    assert_eq!(episode.actions.len(), 1);
    assert!(episode.ctrl_writes_after > episode.ctrl_writes_before);
    assert_eq!(episode.unauthorized_writes, 0);
}

#[test]
fn runtime_divergence_fault_is_frozen_into_trial() {
    let bundle = planar_bundle();
    let trial = resolve_benchmark_trial(
        30,
        30,
        BenchmarkTask::PushProbeReplan,
        EmbodimentRole::Development,
        &bundle,
        &planar_manifest(),
    )
    .expect("scheduled fault trial resolves");

    assert_eq!(trial.expectation, TrialExpectation::ExpectedSuccess);
    assert_eq!(trial.fault_injection.as_deref(), Some("runtime_divergence"));
    let repeated = resolve_benchmark_trial(
        30,
        30,
        BenchmarkTask::PushProbeReplan,
        EmbodimentRole::Development,
        &bundle,
        &planar_manifest(),
    )
    .unwrap();
    assert_eq!(
        trial_digest(&trial).unwrap(),
        trial_digest(&repeated).unwrap()
    );
}

#[test]
fn holdout_role_is_registered_without_early_scoring() {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../robots/benchmarks/manipulation/embodiments.json");
    let registry = EmbodimentRegistry::load(&path).expect("frozen embodiment registry");
    let public = registry.selected(false);
    let holdout = registry.selected(true);

    assert_eq!(
        public
            .iter()
            .filter(|entry| entry.role == EmbodimentRole::Development)
            .count(),
        2
    );
    assert_eq!(
        public
            .iter()
            .filter(|entry| entry.role == EmbodimentRole::GraspFixture)
            .count(),
        1
    );
    assert!(public
        .iter()
        .all(|entry| entry.role != EmbodimentRole::Holdout));
    assert_eq!(holdout.len(), 1);
    assert_eq!(holdout[0].role, EmbodimentRole::Holdout);
    assert_eq!(holdout[0].robot_id, "menagerie_wx250s");
    assert_eq!(registry.digest().unwrap().len(), 64);
}

#[test]
fn physical_evidence_requires_fresh_sensor_and_calibration_provenance() {
    let (trace, requirements) = physical_trace(SensorOrigin::Hardware);
    let evidence = validate_physical_observation(&trace, &requirements, 10.05)
        .expect("fresh hardware observation with exact provenance");
    assert_eq!(evidence.status, PhysicalEvidenceStatus::Unverified);
    assert!(evidence.sensor_provenance_valid);
    assert!(evidence.requires_independent_measurement);

    let mut stale = trace.clone();
    stale.observation.observations[0].receive_s = 9.0;
    stale.observation.joint_state[0].receive_s = 9.0;
    stale.observation.as_of_s = 9.0;
    stale.policy_observation.timestamp_s = 9.0;
    assert!(validate_physical_observation(&stale, &requirements, 10.05).is_err());

    let mut wrong_calibration = trace.clone();
    wrong_calibration.sensors[0].calibration_hash = "unreviewed-calibration".into();
    assert!(validate_physical_observation(&wrong_calibration, &requirements, 10.05).is_err());

    let mut wrong_frame = trace;
    wrong_frame.observation.observations[0].robot_frame = "world".into();
    assert!(validate_physical_observation(&wrong_frame, &requirements, 10.05).is_err());
}

#[test]
fn simulator_origin_never_becomes_measured() {
    let (trace, requirements) = physical_trace(SensorOrigin::Simulation);
    assert!(validate_physical_observation(&trace, &requirements, 10.05).is_err());

    let (mut trace, requirements) = physical_trace(SensorOrigin::Hardware);
    trace.policy_observation.mode = realityos_verify::observation::VisionMode::PerfectPerception;
    assert!(validate_physical_observation(&trace, &requirements, 10.05).is_err());
}

#[test]
fn unsupported_gripper_is_counted_as_refusal() {
    let _mujoco_guard = lock_mujoco_test();
    let bundle = ur5e_bundle();
    let (worker, manifest) =
        load_and_normalize(&bundle, &[], 9030).expect("strict benchmark requires MuJoCo");
    checkin_worker(worker);
    let trial = resolve_benchmark_trial(
        9030,
        15,
        BenchmarkTask::PinchGraspLift,
        EmbodimentRole::Development,
        &bundle,
        &manifest,
    )
    .expect("unsupported-gripper case resolves");
    assert_eq!(trial.expectation, TrialExpectation::ExpectedRefusal);
    assert_eq!(
        trial.scenario.expected_refusal.as_deref(),
        Some("UNSUPPORTED_GRIPPER_TOPOLOGY")
    );

    let episode = realityos_verify::manipulation_benchmark::run_benchmark_trial(
        &bundle,
        &trial,
        SimulationAdmission {
            execution_bound_m: 0.1,
            expires_at_s: 120.0,
        },
    )
    .expect("unsupported topology is recorded as a refusal");

    assert_eq!(episode.outcome, TrialOutcome::ValidRefusal);
    assert_eq!(episode.unauthorized_writes, 0);
    assert_eq!(episode.ctrl_writes_before, episode.ctrl_writes_after);
}

#[test]
fn same_task_runs_on_both_development_embodiments() {
    let _mujoco_guard = lock_mujoco_test();
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../robots/benchmarks/manipulation/embodiments.json");
    let registry = EmbodimentRegistry::load(&path).expect("frozen embodiment registry");
    let development = registry
        .selected(false)
        .into_iter()
        .filter(|entry| entry.role == EmbodimentRole::Development)
        .collect::<Vec<_>>();
    assert_eq!(development.len(), 2);
    let mut observed_robots = Vec::new();

    for entry in development {
        assert!(entry.tasks.contains(&BenchmarkTask::PushProbeReplan));
        let bundle = RobotBundle::load(
            PathBuf::from(env!("CARGO_MANIFEST_DIR"))
                .join("../../")
                .join(&entry.bundle_path),
        )
        .expect("registered development bundle");
        let (worker, manifest) =
            load_and_normalize(&bundle, &[], 9040).expect("strict benchmark requires MuJoCo");
        checkin_worker(worker);
        assert_eq!(manifest.robot_id, entry.robot_id);
        let trial = resolve_benchmark_trial(
            9040,
            10,
            BenchmarkTask::PushProbeReplan,
            entry.role,
            &bundle,
            &manifest,
        )
        .expect("shared push trial resolves for this embodiment");
        let episode = realityos_verify::manipulation_benchmark::run_benchmark_trial(
            &bundle,
            &trial,
            SimulationAdmission {
                execution_bound_m: 0.1,
                expires_at_s: 120.0,
            },
        )
        .expect("same bridge runner records both embodiments");
        assert_eq!(episode.task, BenchmarkTask::PushProbeReplan);
        assert_eq!(episode.role, EmbodimentRole::Development);
        assert_eq!(episode.evidence_status, "SIMULATION_ONLY");
        assert!(!episode.geometry_digest.is_empty());
        assert_eq!(episode.unauthorized_writes, 0);
        observed_robots.push(episode.robot_id);
    }

    observed_robots.sort();
    assert_eq!(observed_robots, vec!["menagerie_iiwa14", "menagerie_ur5e"]);
}

#[test]
fn same_seed_resolves_identical_trial() {
    let bundle = planar_bundle();
    let manifest = planar_manifest();
    let first = resolve_benchmark_trial(
        42,
        10,
        BenchmarkTask::PushProbeReplan,
        EmbodimentRole::Development,
        &bundle,
        &manifest,
    )
    .expect("valid deterministic trial");
    let repeated = resolve_benchmark_trial(
        42,
        10,
        BenchmarkTask::PushProbeReplan,
        EmbodimentRole::Development,
        &bundle,
        &manifest,
    )
    .expect("valid repeated trial");

    assert_eq!(
        serde_json::to_value(&first).unwrap(),
        serde_json::to_value(&repeated).unwrap()
    );
    assert_eq!(
        trial_digest(&first).unwrap(),
        trial_digest(&repeated).unwrap()
    );
}

#[test]
fn inspection_preserves_qpos0_defaults() {
    let bundle = planar_bundle();
    let manifest = RobotManifest::from_inspect(
        &bundle,
        &serde_json::json!({
            "nq": 3,
            "nv": 2,
            "nu": 2,
            "qpos0": [0.1, -0.2, 0.25],
            "joints": [],
            "actuators": [],
            "sensors": [],
            "cameras": [],
            "bodies": [],
            "sites": [],
            "geoms": []
        }),
    );

    assert_eq!(manifest.qpos0, vec![0.1, -0.2, 0.25]);
}

#[test]
fn malformed_inspected_qpos0_is_refused() {
    let bundle = planar_bundle();
    let manifest = RobotManifest::from_inspect(
        &bundle,
        &serde_json::json!({
            "nq": 3,
            "nv": 2,
            "nu": 2,
            "qpos0": ["invalid", 0.1, -0.2, 0.25],
            "joints": [],
            "actuators": [],
            "sensors": [],
            "cameras": [],
            "bodies": [],
            "sites": [],
            "geoms": []
        }),
    );

    assert!(resolve_benchmark_trial(
        42,
        10,
        BenchmarkTask::PushProbeReplan,
        EmbodimentRole::Development,
        &bundle,
        &manifest,
    )
    .is_err());
}

#[test]
fn geometry_gate_rejects_dropped_collision_geom() {
    let (bundle, manifest, mut model) = collision_fixture();
    model
        .collision_geoms
        .retain(|geom| geom.id != "collision_a");

    assert!(validate_collision_geometry(&bundle, &manifest, &model).is_err());
}

#[test]
fn geometry_gate_rejects_incomplete_collision_inspection_contract() {
    let (bundle, mut manifest, model) = collision_fixture();
    manifest.geometry_inspection_complete = false;

    assert!(validate_collision_geometry(&bundle, &manifest, &model).is_err());
}

#[test]
fn geometry_gate_includes_geometry_for_explicit_pairs_even_with_zero_masks() {
    let (bundle, mut manifest, _) = collision_fixture();
    manifest.geoms[1].contype = 0;
    manifest.geoms[1].conaffinity = 0;
    let model = embodiment_from_manifest(&bundle, &manifest);

    let geometry = validate_collision_geometry(&bundle, &manifest, &model)
        .expect("explicit contact pair keeps both geoms in the collision model");

    assert_eq!(geometry.geoms.len(), 2);
    assert!(geometry.geoms.iter().any(|geom| geom.geom_id == 1));
}

#[test]
fn geometry_gate_rejects_invalid_collision_pose() {
    let (bundle, mut manifest, model) = collision_fixture();
    manifest.geoms[0].quat = [0.0; 4];

    assert!(validate_collision_geometry(&bundle, &manifest, &model).is_err());
}

#[test]
fn geometry_gate_requires_and_hashes_contact_friction() {
    let (bundle, manifest, model) = collision_fixture();
    let baseline = validate_collision_geometry(&bundle, &manifest, &model).unwrap();

    let mut missing = manifest.clone();
    missing.geoms[0].friction = None;
    let missing_model = embodiment_from_manifest(&bundle, &missing);
    assert!(validate_collision_geometry(&bundle, &missing, &missing_model).is_err());

    let mut changed = manifest.clone();
    changed.geoms[0].friction = Some([0.4, 0.01, 0.001]);
    let changed_model = embodiment_from_manifest(&bundle, &changed);
    let changed_digest = validate_collision_geometry(&bundle, &changed, &changed_model)
        .unwrap()
        .digest;
    assert_ne!(baseline.digest, changed_digest);
}

#[test]
fn geometry_digest_covers_mesh_assets_and_pair_rules() {
    let mut bundle = ur5e_bundle();
    let mesh_assets: Vec<_> = bundle
        .asset_files
        .iter()
        .filter(|path| path.extension().and_then(|ext| ext.to_str()) == Some("obj"))
        .take(2)
        .collect();
    assert_eq!(
        mesh_assets.len(),
        2,
        "fixture needs two distinct mesh assets"
    );
    let mesh_a = relative_mesh_asset(&bundle, mesh_assets[0]);
    let mesh_b = relative_mesh_asset(&bundle, mesh_assets[1]);
    let manifest = RobotManifest::from_inspect(
        &bundle,
        &serde_json::json!({
            "nq": 0,
            "nv": 0,
            "nu": 0,
            "qpos0": [],
            "bodies": [
                {"name":"base", "mass":1.0, "inertia":[1.0,1.0,1.0], "parent":"world", "pos":[0.0,0.0,0.0], "quat":[1.0,0.0,0.0,0.0]},
                {"name":"shoulder_link", "mass":1.0, "inertia":[1.0,1.0,1.0], "parent":"base", "pos":[0.0,0.0,0.0], "quat":[1.0,0.0,0.0,0.0]}
            ],
            "geoms": [
                {"id":0, "name":"base_mesh", "body":"base", "type_name":"mesh", "size":[1.0,1.0,1.0], "pos":[0.0,0.0,0.0], "quat":[1.0,0.0,0.0,0.0], "friction":[1.0,0.005,0.0001], "contype":1, "conaffinity":1, "mesh_asset":mesh_a, "mesh_vertices":[[0.0,0.0,0.0],[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]]},
                {"id":1, "name":"shoulder_mesh", "body":"shoulder_link", "type_name":"mesh", "size":[1.0,1.0,1.0], "pos":[0.0,0.0,0.0], "quat":[1.0,0.0,0.0,0.0], "friction":[1.0,0.005,0.0001], "contype":1, "conaffinity":1, "mesh_asset":mesh_b, "mesh_vertices":[[0.0,0.0,0.0],[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]]},
                {"id":2, "name":"wrist_mesh", "body":"shoulder_link", "type_name":"mesh", "size":[1.0,1.0,1.0], "pos":[0.0,0.0,0.0], "quat":[1.0,0.0,0.0,0.0], "friction":[1.0,0.005,0.0001], "contype":1, "conaffinity":1, "mesh_asset":mesh_a, "mesh_vertices":[[0.0,0.0,0.0],[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]]}
            ],
            "contact_pairs": [{"geom1_id":0,"geom2_id":1}],
            "excluded_body_pairs": [{"body1":"base","body2":"shoulder_link"}]
        }),
    );
    let model = embodiment_from_manifest(&bundle, &manifest);
    let baseline = validate_collision_geometry(&bundle, &manifest, &model).unwrap();

    let mut coplanar_mesh = manifest.clone();
    coplanar_mesh.geoms[0].mesh_vertices[3] = [1.0, 1.0, 0.0];
    let coplanar_model = embodiment_from_manifest(&bundle, &coplanar_mesh);
    let coplanar_result =
        validate_collision_geometry(&bundle, &coplanar_mesh, &coplanar_model).unwrap();
    assert_eq!(
        coplanar_result.result,
        GeometryGateResult::UnsupportedPresent {
            geom_names: vec!["base_mesh".into()]
        }
    );
    let coplanar_asset = coplanar_result.geoms[0]
        .mesh_asset
        .as_ref()
        .expect("unsupported query geometry still authenticates its source asset");
    assert_eq!(coplanar_asset.sha256.len(), 64);
    assert!(serde_json::to_value(coplanar_asset).unwrap()["query_vertices_sha256"].is_null());

    let mut oversized_mesh = manifest.clone();
    oversized_mesh.geoms[0].mesh_vertices = vec![
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ];
    oversized_mesh.geoms[0]
        .mesh_vertices
        .resize(100_001, [0.0, 0.0, 0.0]);
    let oversized_model = embodiment_from_manifest(&bundle, &oversized_mesh);
    assert!(validate_collision_geometry(&bundle, &oversized_mesh, &oversized_model).is_err());

    let mut omitted_mesh = manifest.clone();
    omitted_mesh.geoms[0].mesh_vertices.clear();
    omitted_mesh.geoms[0].mesh_vertices_omitted_reason =
        Some("compiled_hull_vertex_limit_exceeded".into());
    let omitted_model = embodiment_from_manifest(&bundle, &omitted_mesh);
    let omitted_result =
        validate_collision_geometry(&bundle, &omitted_mesh, &omitted_model).unwrap();
    let omitted_asset = omitted_result.geoms[0]
        .mesh_asset
        .as_ref()
        .expect("omitted query vertices still authenticate their source asset");
    assert_eq!(omitted_asset.sha256.len(), 64);
    assert!(serde_json::to_value(omitted_asset).unwrap()["query_vertices_sha256"].is_null());
    assert_eq!(
        omitted_result.geoms[0].mesh_unsupported_reason.as_deref(),
        Some("compiled_hull_vertex_limit_exceeded")
    );

    let mut missing_unsupported_asset = omitted_mesh.clone();
    missing_unsupported_asset.geoms[0].mesh_asset = None;
    let missing_unsupported_model = embodiment_from_manifest(&bundle, &missing_unsupported_asset);
    assert!(validate_collision_geometry(
        &bundle,
        &missing_unsupported_asset,
        &missing_unsupported_model
    )
    .is_err());

    let mut unsupported_asset_unlisted_bundle = bundle.clone();
    unsupported_asset_unlisted_bundle.asset_files.clear();
    assert!(validate_collision_geometry(
        &unsupported_asset_unlisted_bundle,
        &omitted_mesh,
        &omitted_model
    )
    .is_err());

    let mut unreported_omission = omitted_mesh.clone();
    unreported_omission.geoms[0].mesh_vertices_omitted_reason = None;
    let unreported_model = embodiment_from_manifest(&bundle, &unreported_omission);
    assert!(validate_collision_geometry(&bundle, &unreported_omission, &unreported_model).is_err());

    let mut unknown_omission = omitted_mesh.clone();
    unknown_omission.geoms[0].mesh_vertices_omitted_reason = Some("anything_goes".into());
    let unknown_model = embodiment_from_manifest(&bundle, &unknown_omission);
    assert!(validate_collision_geometry(&bundle, &unknown_omission, &unknown_model).is_err());

    let mut pair_changed = manifest.clone();
    pair_changed.contact_pairs[0].geom1_id = 1;
    pair_changed.contact_pairs[0].geom2_id = 2;
    let pair_model = embodiment_from_manifest(&bundle, &pair_changed);
    let changed_pair_digest = validate_collision_geometry(&bundle, &pair_changed, &pair_model)
        .unwrap()
        .digest;

    let mut asset_changed = manifest.clone();
    asset_changed.geoms[0].mesh_asset = Some(mesh_b);
    let asset_model = embodiment_from_manifest(&bundle, &asset_changed);
    let changed_asset_digest = validate_collision_geometry(&bundle, &asset_changed, &asset_model)
        .unwrap()
        .digest;

    let mut hull_changed = manifest.clone();
    hull_changed.geoms[0].mesh_vertices[1][0] += 0.25;
    let hull_model = embodiment_from_manifest(&bundle, &hull_changed);
    let changed_hull = validate_collision_geometry(&bundle, &hull_changed, &hull_model).unwrap();

    assert_ne!(baseline.digest, changed_pair_digest);
    assert_ne!(baseline.digest, changed_asset_digest);
    assert_ne!(baseline.digest, changed_hull.digest);
    assert_ne!(
        baseline.geoms[0]
            .mesh_asset
            .as_ref()
            .unwrap()
            .query_vertices_sha256,
        changed_hull.geoms[0]
            .mesh_asset
            .as_ref()
            .unwrap()
            .query_vertices_sha256
    );

    let mut reversed_pair = manifest.clone();
    reversed_pair.contact_pairs[0].geom1_id = 1;
    reversed_pair.contact_pairs[0].geom2_id = 0;
    let reversed_model = embodiment_from_manifest(&bundle, &reversed_pair);
    assert_eq!(
        baseline.digest,
        validate_collision_geometry(&bundle, &reversed_pair, &reversed_model)
            .unwrap()
            .digest
    );

    let mut duplicate_pair = manifest.clone();
    duplicate_pair
        .contact_pairs
        .push(realityos_verify::normalize::ContactPairRecord {
            geom1_id: 1,
            geom2_id: 0,
        });
    let duplicate_model = embodiment_from_manifest(&bundle, &duplicate_pair);
    assert!(validate_collision_geometry(&bundle, &duplicate_pair, &duplicate_model).is_err());
    bundle.asset_files.clear();
    assert!(validate_collision_geometry(&bundle, &manifest, &model).is_err());
}

#[test]
fn worker_inspection_preserves_source_geometry_and_contact_rules() {
    let _mujoco_guard = lock_mujoco_test();
    let root =
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../robots/bundles/arm_gripper_lift");
    let bundle = RobotBundle::load(root).expect("checked-in grasp fixture");
    let (inst, manifest) = realityos_verify::runner::load_and_normalize(&bundle, &[], 0)
        .expect("MuJoCo inspection succeeds");
    let model = embodiment_from_manifest(&bundle, &manifest);
    let geometry = validate_collision_geometry(&bundle, &manifest, &model)
        .expect("full source geometry passes strict admission");
    realityos_verify::mujoco_exec::checkin_worker(inst);

    assert!(manifest.geometry_inspection_complete);
    assert!(manifest.contact_pairs.is_empty());
    assert!(manifest.excluded_body_pairs.iter().any(|pair| {
        (pair.body1 == "link2" && pair.body2 == "lift_stage")
            || (pair.body1 == "lift_stage" && pair.body2 == "link2")
    }));
    assert_eq!(
        geometry.geoms.len(),
        manifest
            .geoms
            .iter()
            .filter(|geom| { geom.contype != 0 || geom.conaffinity != 0 })
            .count()
    );
    assert_eq!(geometry.result, GeometryGateResult::Complete);

    let bundle = ur5e_bundle();
    let (inst, manifest) = realityos_verify::runner::load_and_normalize(&bundle, &[], 0)
        .expect("MuJoCo loads UR5e and its mesh assets");
    let model = embodiment_from_manifest(&bundle, &manifest);
    let geometry = validate_collision_geometry(&bundle, &manifest, &model)
        .expect("UR5e collision geometry passes strict admission");
    realityos_verify::mujoco_exec::checkin_worker(inst);

    assert!(manifest.geometry_inspection_complete);
    let inspected_meshes: Vec<_> = manifest
        .geoms
        .iter()
        .filter(|geom| geom.geom_type == "mesh")
        .collect();
    assert!(!inspected_meshes.is_empty());
    assert!(inspected_meshes.iter().all(|geom| geom
        .mesh_asset
        .as_deref()
        .is_some_and(|asset| !asset.is_empty())));
    assert!(inspected_meshes
        .iter()
        .all(|geom| geom.mesh_vertices.len() >= 4));
    assert_eq!(geometry.result, GeometryGateResult::Complete);
}

#[test]
fn wx250s_compiled_convex_meshes_are_complete_queryable_and_hashed() {
    let _mujoco_guard = lock_mujoco_test();
    let bundle = wx250s_bundle();
    let (inst, manifest) = load_and_normalize(&bundle, &[], 0)
        .expect("MuJoCo loads WX250S and its compiled collision meshes");
    let model = embodiment_from_manifest(&bundle, &manifest);
    let geometry = validate_collision_geometry(&bundle, &manifest, &model)
        .expect("compiled WX250S collision hulls pass strict admission");

    assert_eq!(geometry.result, GeometryGateResult::Complete);
    let collision_meshes = manifest
        .geoms
        .iter()
        .filter(|geom| geom.geom_type == "mesh" && (geom.contype != 0 || geom.conaffinity != 0))
        .collect::<Vec<_>>();
    assert_eq!(collision_meshes.len(), 10);
    for geom in &collision_meshes {
        let lowered = model
            .collision_geoms
            .iter()
            .find(|lowered| lowered.id == geom.name)
            .expect("collision mesh is retained in the semantic model");
        assert!(matches!(
            &lowered.shape,
            realityos_semantics::geometry::PrimitiveShape::ConvexMesh { .. }
        ));
        assert!(lowered.shape.query_supported());
        assert_eq!(
            geometry
                .geoms
                .iter()
                .find(|record| record.name == geom.name)
                .expect("admitted mesh has a manifest record")
                .mesh_asset
                .as_ref()
                .and_then(|mesh| mesh.query_vertices_sha256.as_ref().map(String::len)),
            Some(64)
        );
        assert!(geom.mesh_vertices.len() <= 230);
    }

    checkin_worker(inst);
}

#[test]
fn sampled_object_and_obstacles_are_in_declared_ranges() {
    let trial = resolve_benchmark_trial(
        42,
        10,
        BenchmarkTask::PushProbeReplan,
        EmbodimentRole::Development,
        &planar_bundle(),
        &planar_manifest(),
    )
    .expect("valid deterministic trial");
    let object = object_named(&trial.scenario.objects, "obj0");
    let position = object["pos"].as_array().unwrap();
    let size = object["size"].as_array().unwrap();
    let x = position[0].as_f64().unwrap();
    let y = position[1].as_f64().unwrap();
    let mass = object["mass"].as_f64().unwrap();
    let friction = object["friction"].as_f64().unwrap();

    assert!((0.20..0.30).contains(&x));
    assert!((-0.03..0.03).contains(&y));
    assert!((0.02..0.04).contains(&size[0].as_f64().unwrap()));
    assert!((0.04..0.40).contains(&mass));
    assert!((0.20..1.00).contains(&friction));

    let obstacles: Vec<&Value> = trial
        .scenario
        .objects
        .iter()
        .filter(|entry| {
            entry["name"]
                .as_str()
                .is_some_and(|name| name.starts_with("benchmark_obstacle_"))
        })
        .collect();
    assert!((1..=3).contains(&obstacles.len()));
    for obstacle in obstacles {
        assert!(matches!(
            obstacle["type"].as_str(),
            Some("box" | "cylinder")
        ));
        let p = obstacle["pos"].as_array().unwrap();
        assert!(p
            .iter()
            .all(|coordinate| coordinate.as_f64().unwrap().is_finite()));
    }
}

#[test]
fn initial_qpos_respects_joint_limits() {
    let trial = resolve_benchmark_trial(
        42,
        10,
        BenchmarkTask::PushProbeReplan,
        EmbodimentRole::Development,
        &planar_bundle(),
        &planar_manifest(),
    )
    .expect("valid deterministic trial");

    assert_eq!(trial.initial_qpos.len(), 3);
    assert!((-1.0..=1.0).contains(&trial.initial_qpos[0]));
    assert!((-2.0..=2.0).contains(&trial.initial_qpos[1]));
    assert_eq!(trial.initial_qpos[2], 0.25);
}

#[test]
fn initial_qpos_randomizes_across_declared_joint_ranges() {
    let bundle = planar_bundle();
    let manifest = planar_manifest();
    let first = resolve_benchmark_trial(
        42,
        10,
        BenchmarkTask::PushProbeReplan,
        EmbodimentRole::Development,
        &bundle,
        &manifest,
    )
    .expect("declared joint ranges are valid");
    let repeated = resolve_benchmark_trial(
        43,
        10,
        BenchmarkTask::PushProbeReplan,
        EmbodimentRole::Development,
        &bundle,
        &manifest,
    )
    .expect("second seed resolves independently");

    assert!((-1.0..=1.0).contains(&first.initial_qpos[0]));
    assert!((-2.0..=2.0).contains(&first.initial_qpos[1]));
    assert_eq!(first.initial_qpos[2], 0.25);
    assert_ne!(first.initial_qpos[..2], repeated.initial_qpos[..2]);
}

#[test]
fn trial_label_is_frozen_before_execution() {
    let bundle = planar_bundle();
    let manifest = planar_manifest();
    let expected_success = resolve_benchmark_trial(
        7,
        10,
        BenchmarkTask::PushProbeReplan,
        EmbodimentRole::Development,
        &bundle,
        &manifest,
    )
    .unwrap();
    let expected_refusal = resolve_benchmark_trial(
        7,
        3,
        BenchmarkTask::PushProbeReplan,
        EmbodimentRole::Development,
        &bundle,
        &manifest,
    )
    .unwrap();

    assert_eq!(
        expected_success.expectation,
        TrialExpectation::ExpectedSuccess
    );
    assert_eq!(
        expected_refusal.expectation,
        TrialExpectation::ExpectedRefusal
    );
    assert!(expected_refusal.scenario.expected_refusal.is_some());
}

#[test]
fn grasp_success_labels_require_conservative_hold_mechanics() {
    let _mujoco_guard = lock_mujoco_test();
    let bundle = grasp_fixture_bundle();
    let (worker, manifest) =
        load_and_normalize(&bundle, &[], 9000).expect("strict benchmark requires MuJoCo");
    checkin_worker(worker);

    let unsupported_mass = (0..1000)
        .find_map(|seed| {
            resolve_benchmark_trial(
                seed,
                15,
                BenchmarkTask::PinchGraspLift,
                EmbodimentRole::GraspFixture,
                &bundle,
                &manifest,
            )
            .ok()
            .filter(|trial| {
                trial.expectation == TrialExpectation::ExpectedRefusal
                    && trial.scenario.expected_refusal.as_deref() == Some("HOLD_MECHANICS_UNPROVEN")
            })
        })
        .expect("sampled heavy grasps produce a conservative refusal label");
    assert_eq!(
        unsupported_mass.expectation,
        TrialExpectation::ExpectedRefusal
    );
    assert_eq!(
        unsupported_mass.scenario.expected_refusal.as_deref(),
        Some("HOLD_MECHANICS_UNPROVEN")
    );

    let supported_mass = (9000..10000)
        .find_map(|seed| {
            resolve_benchmark_trial(
                seed,
                15,
                BenchmarkTask::PinchGraspLift,
                EmbodimentRole::GraspFixture,
                &bundle,
                &manifest,
            )
            .ok()
            .filter(|trial| trial.expectation == TrialExpectation::ExpectedSuccess)
        })
        .expect("pilot range contains a conservatively supportable mass");
    assert_eq!(supported_mass.scenario.expected_refusal, None);
}
