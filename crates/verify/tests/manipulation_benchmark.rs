use realityos_kernel::physical_action::{
    AbortHoldMode, WrenchAbortContract, WrenchLimitPolicy, WrenchLimitScope,
};
use realityos_reasoner_bridge::SimulationAdmission;
use realityos_semantics::observation::{
    wrench_sample_digest, JointStateSample, ObservationFrame, SensorObservation, WrenchSample,
};
use realityos_semantics::push_probe::WrenchCompensation;
use realityos_semantics::sensor::{SensorClass, WrenchSampleOrigin};
use realityos_verify::bundle::RobotBundle;
use realityos_verify::manipulation_benchmark::wrench_sensor::{
    aggregate_contact_wrenches, conservative_wrench_upper_bound, contact_force_on_geom,
    transform_contact_wrench_world, WrenchAbortReason,
};
use realityos_verify::manipulation_benchmark::{
    evaluate_paired_screening_gate, paired_campaign_delta, paired_trial_key,
    resolve_benchmark_trial, resolve_benchmark_trial_with_suite, score_campaign, trial_digest,
    upgrade_v5_trial_for_v6, validate_collision_geometry, validate_physical_observation,
    BenchmarkConfig, BenchmarkExecutionError, BenchmarkTask, BenchmarkTrialRecord,
    EmbodimentRegistry, EmbodimentRole, FailureCause, GeometryGateResult, PhysicalEvidenceStatus,
    PhysicalSensorIdentity, PhysicalSensorRequirement, PhysicalSensorTrace, PushTrialStratum,
    ScenarioSuiteManifest, SensorOrigin, SimulationWrenchSensorProfile, TrialExpectation,
    TrialOutcome,
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

fn simulation_admission(expires_at_s: f64) -> SimulationAdmission {
    let mut wrench_limit_policy = WrenchLimitPolicy {
        scope: WrenchLimitScope {
            sensor_id: "ft-tool".into(),
            sensor_frame: "tool0".into(),
            caps: [20.0, 20.0, 30.0, 2.0, 2.0, 2.0],
            policy_digest: String::new(),
            abort_contract_id: "abort-and-reobserve".into(),
        },
        sensor_calibration_hash: "test-sensor-calibration-v1".into(),
        sensor_sample_rate_hz: 125.0,
        max_position_tracking_correction_rad: 0.0,
        position_tracking_correction_gain: 0.0,
        max_rise_rate: [10.0; 6],
        max_sensing_latency_s: 0.01,
        max_abort_latency_s: 0.03,
        max_sample_age_s: 0.02,
        max_inter_sample_gap_s: 0.01,
        max_cross_sensor_skew_s: 0.005,
        monitor_id: "test-wrench-monitor-v1".into(),
        compensation_digest: "c".repeat(64),
        abort_contract: WrenchAbortContract {
            contract_id: "abort-and-reobserve".into(),
            hold_mode: AbortHoldMode::HoldCurrentJointPosition,
            allowed_stop_actuator_ids: vec!["tool".into()],
            max_time_to_stop_s: 0.03,
            max_stop_writes: 1,
        },
    };
    wrench_limit_policy.scope.policy_digest = wrench_limit_policy.digest();
    SimulationAdmission {
        execution_bound_m: 0.1,
        expires_at_s,
        wrench_limit_policy,
    }
}

fn simulation_wrench_profile(sensor_frame: &str) -> SimulationWrenchSensorProfile {
    let mut limit_policy = simulation_admission(100.0).wrench_limit_policy;
    limit_policy.scope.sensor_id = "sim-ft-tool-v1".into();
    limit_policy.scope.sensor_frame = sensor_frame.into();
    limit_policy.sensor_calibration_hash = "a".repeat(64);
    limit_policy.sensor_sample_rate_hz = 125.0;
    limit_policy.max_position_tracking_correction_rad = 0.04;
    limit_policy.position_tracking_correction_gain = 2.0;
    limit_policy.max_sensing_latency_s = 0.01;
    let mut compensation = WrenchCompensation {
        bias: [0.0; 6],
        gravity_wrench: [0.0; 6],
        residual_error: [0.05, 0.05, 0.05, 0.005, 0.005, 0.005],
        digest: String::new(),
    };
    compensation.digest = compensation.digest();
    limit_policy.compensation_digest =
        SimulationWrenchSensorProfile::compensation_policy_digest_for(
            "mujoco-force-torque-sensor-gravity-baseline-v2",
            &compensation,
        );
    limit_policy.scope.policy_digest = limit_policy.digest();
    SimulationWrenchSensorProfile {
        sensor_id: limit_policy.scope.sensor_id.clone(),
        model_id: "mujoco-force-torque-sensor-gravity-baseline-v2".into(),
        calibration_hash: limit_policy.sensor_calibration_hash.clone(),
        sensor_frame: sensor_frame.into(),
        units: "N,N*m".into(),
        sample_rate_hz: limit_policy.sensor_sample_rate_hz,
        latency_s: limit_policy.max_sensing_latency_s,
        resolution: [0.001; 6],
        bias: [0.0; 6],
        error_bounds: [0.05, 0.05, 0.05, 0.005, 0.005, 0.005],
        free_space_model_error_bounds: [0.0; 6],
        seeded_noise_bound: [0.01, 0.01, 0.01, 0.001, 0.001, 0.001],
        limit_policy,
    }
}

fn free_space_model_profile(sensor_frame: &str) -> SimulationWrenchSensorProfile {
    let mut profile = simulation_wrench_profile(sensor_frame);
    profile.model_id = "mujoco-force-torque-sensor-free-space-model-v1".into();
    profile.limit_policy.compensation_digest =
        SimulationWrenchSensorProfile::compensation_policy_digest_for(
            &profile.model_id,
            &profile.compensation(),
        );
    profile.limit_policy.scope.policy_digest = profile.limit_policy.digest();
    assert!(profile.validate().is_ok());
    profile
}

fn run_test_benchmark_trial(
    bundle: &RobotBundle,
    trial: &realityos_verify::manipulation_benchmark::ResolvedBenchmarkTrial,
    mut admission: SimulationAdmission,
) -> Result<
    realityos_verify::manipulation_benchmark::BenchmarkEpisode,
    realityos_verify::manipulation_benchmark::BenchmarkError,
> {
    let mut resolved_trial = trial.clone();
    resolved_trial.paired_trial_key = Some(paired_trial_key(&resolved_trial).unwrap());
    let profile = if trial.task == BenchmarkTask::PushProbeReplan {
        let profile = if trial.robot_id == "planar_arm" {
            let mut profile = simulation_wrench_profile("ee");
            profile
                .limit_policy
                .abort_contract
                .allowed_stop_actuator_ids = vec!["act1".into(), "act2".into(), "act3".into()];
            profile.limit_policy.scope.policy_digest = profile.limit_policy.digest();
            profile
        } else {
            let registry_path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
                .join("../../robots/benchmarks/manipulation/embodiments.json");
            realityos_verify::manipulation_benchmark::EmbodimentRegistry::load(&registry_path)
                .expect("checked-in manipulation registry")
                .registrations
                .into_iter()
                .find(|entry| entry.robot_id == trial.robot_id)
                .and_then(|entry| entry.simulation_wrench_profile)
                .expect("registered push embodiment profile")
        };
        admission.wrench_limit_policy = profile.limit_policy.clone();
        Some(profile)
    } else {
        None
    };
    realityos_verify::manipulation_benchmark::run_benchmark_trial(
        bundle,
        &resolved_trial,
        admission,
        profile.as_ref(),
    )
}

fn monitor_sample(profile: &SimulationWrenchSensorProfile, wrench: [f64; 6]) -> WrenchSample {
    let mut sample = WrenchSample {
        sensor_id: profile.sensor_id.clone(),
        sensor_frame: profile.sensor_frame.clone(),
        transform_epoch: "monitor-test-epoch".into(),
        units: profile.units.clone(),
        wrench,
        error_bounds: profile.error_bounds,
        capture_s: 1.0,
        receive_s: 1.0 + profile.latency_s,
        sequence: 1,
        clock_domain: "mujoco_simulation_clock".into(),
        origin: WrenchSampleOrigin::Simulation,
        digest: String::new(),
    };
    sample.digest = wrench_sample_digest(&sample);
    sample
}

fn monitored_profile_for_fixture(
    frame: &str,
    inspect: &Value,
    stop_ids: Option<Vec<String>>,
) -> SimulationWrenchSensorProfile {
    let mut profile = simulation_wrench_profile(frame);
    let ids = stop_ids.unwrap_or_else(|| {
        inspect["actuators"]
            .as_array()
            .expect("actuator manifest")
            .iter()
            .filter(|actuator| {
                matches!(actuator["trntype"].as_i64(), Some(0 | 1))
                    && actuator["biastype"].as_i64() == Some(1)
                    && actuator["ctrllimited"].as_bool() == Some(true)
            })
            .filter_map(|actuator| actuator["name"].as_str().map(str::to_owned))
            .collect::<Vec<_>>()
    });
    assert!(!ids.is_empty(), "fixture needs position joint actuators");
    profile
        .limit_policy
        .abort_contract
        .allowed_stop_actuator_ids = ids;
    profile.limit_policy.scope.caps[0] = 0.01;
    profile.limit_policy.scope.policy_digest = profile.limit_policy.digest();
    assert!(profile.validate().is_ok());
    profile
}

fn monitored_worker_request(
    profile: &SimulationWrenchSensorProfile,
    drop_wrench_samples: bool,
) -> Value {
    let policy = &profile.limit_policy;
    let profile_digest = profile.digest().expect("profile digest");
    serde_json::json!({
        "cmd": "step_monitored",
        "n": 100,
        "profile": profile,
        "profile_digest": profile_digest,
        "policy": policy,
        "grant_id": "monitor-test-grant",
        "grant_scope_digest": "f".repeat(64),
        "grant_wrench_scope": policy.scope,
        "grant_action_envelope": {
            "kind": "linear_stroke",
            "requested_m": 0.008,
            "maximum_m": 0.1,
            "wrench_limit": policy.scope,
        },
        "observation_epoch": "monitor-test-epoch",
        "drop_wrench_samples": drop_wrench_samples,
    })
}

#[test]
fn force_monitor_trips_each_of_six_axes_before_cap() {
    let profile = simulation_wrench_profile("ee");
    let policy = &profile.limit_policy;
    for axis in 0..6 {
        let margin = profile.error_bounds[axis]
            + policy.max_rise_rate[axis]
                * (policy.max_sensing_latency_s + policy.max_abort_latency_s + 0.001);
        let mut wrench = [0.0; 6];
        wrench[axis] = policy.scope.caps[axis] - margin;
        let sample = monitor_sample(&profile, wrench);
        let reason =
            conservative_wrench_upper_bound(Some(&sample), None, sample.receive_s + 0.001, policy)
                .expect_err("predictive wrench bound must trip below the physical cap");
        assert!(
            matches!(reason, WrenchAbortReason::Limit { axis: tripped, .. } if tripped == axis)
        );
    }
}

#[test]
fn force_monitor_uses_absolute_values_for_both_signs() {
    let profile = simulation_wrench_profile("ee");
    let policy = &profile.limit_policy;
    for sign in [-1.0, 1.0] {
        let mut wrench = [0.0; 6];
        wrench[0] = sign * 19.9;
        let sample = monitor_sample(&profile, wrench);
        assert!(matches!(
            conservative_wrench_upper_bound(Some(&sample), None, sample.receive_s + 0.001, policy,),
            Err(WrenchAbortReason::Limit { axis: 0, .. })
        ));
    }
}

#[test]
fn wrench_threshold_uses_error_and_latency_rise_margin() {
    let profile = simulation_wrench_profile("ee");
    let policy = &profile.limit_policy;
    let sample = monitor_sample(&profile, [5.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
    let now_s = sample.receive_s + 0.001;
    let bounds = conservative_wrench_upper_bound(Some(&sample), None, now_s, policy)
        .expect("bound below cap");
    let expected = 5.0
        + profile.error_bounds[0]
        + policy.max_rise_rate[0]
            * (now_s - sample.capture_s
                + policy.max_sensing_latency_s
                + policy.max_abort_latency_s);
    assert!((bounds[0] - expected).abs() < 1e-10);
    assert!(bounds[0] < policy.scope.caps[0]);
}

#[test]
fn wrench_monitor_compensates_registered_bias_before_checking_the_load() {
    use realityos_verify::mujoco_exec::{ensure_mujoco_or_skip, MujocoInstance};
    if !ensure_mujoco_or_skip() {
        return;
    }
    let _guard = lock_mujoco_test();
    let bundle = planar_bundle();
    let objects = [serde_json::json!({
        "name": "bias_compensation_probe",
        "type": "box",
        "pos": [0.48, 0.0, 0.12],
        "size": [0.015, 0.015, 0.015],
        "mass": 0.05,
        "friction": 0.5
    })];
    let mut worker = MujocoInstance::spawn().expect("MuJoCo worker");
    let loaded = worker
        .load_bundle_with_wrench_profile(
            &bundle,
            &objects,
            736,
            &simulation_wrench_profile("ee_flipped"),
        )
        .expect("load wrench-instrumented contact fixture");
    let mut profile = monitored_profile_for_fixture("ee_flipped", &loaded["inspect"], None);
    profile.limit_policy.scope.caps[0] = 20.0;
    profile.limit_policy.scope.policy_digest = profile.limit_policy.digest();
    let mut calibration_request = monitored_worker_request(&profile, false);
    // Four steps cross one 125 Hz sample after the constraint solver runs.
    // The pre-step sample can be only sensor noise; the contact load is the
    // largest positive axis-0 reading in the batch.
    calibration_request["n"] = serde_json::json!(4);
    calibration_request["expected_contact_body"] = serde_json::json!("bias_compensation_probe");
    let calibration = worker
        .rpc(&calibration_request)
        .expect("read the contact load before configuring its bias");
    assert_eq!(calibration["ok"], true, "{calibration}");
    assert!(
        calibration["abort"].is_null(),
        "the calibration read must not trip the monitor: {calibration}"
    );
    let expected_load = calibration["samples"]
        .as_array()
        .expect("wrench samples")
        .iter()
        .filter_map(|sample| sample["wrench"][0].as_f64())
        .fold(f64::NEG_INFINITY, f64::max);
    let probe = &calibration["state"]["xpos"]["bias_compensation_probe"];
    assert!(
        expected_load > 0.25,
        "fixture must create positive load: {expected_load} probe {probe} contacts {}",
        calibration["state"]["ncon"]
    );
    worker.reset(None, None).expect("reset contact fixture");
    profile.bias[0] = -expected_load;
    profile.limit_policy.max_rise_rate[0] = 0.1;
    profile.limit_policy.scope.caps[0] = expected_load * 0.5;
    let mut compensation = profile.compensation();
    compensation.digest = compensation.digest();
    profile.limit_policy.compensation_digest =
        SimulationWrenchSensorProfile::compensation_policy_digest_for(
            &profile.model_id,
            &compensation,
        );
    profile.limit_policy.scope.policy_digest = profile.limit_policy.digest();
    assert!(profile.validate().is_ok());
    let mut monitored_request = monitored_worker_request(&profile, false);
    // The 125 Hz sample taken on solver step 4 is captured at 0.008 s and
    // received one 0.010 s sensing latency later. Five steps end at 0.010 s,
    // so only the pre-solver reading (the injected bias, near zero once
    // removed) is eligible for the limit check. Nine 0.002 s steps reach the
    // receive time without raising the force cap or shrinking the latency.
    monitored_request["n"] = serde_json::json!(9);
    monitored_request["expected_contact_body"] = serde_json::json!("bias_compensation_probe");
    let response = worker.rpc(&monitored_request).expect("monitored response");

    assert_eq!(response["ok"], true, "{response}");
    assert_eq!(
        response["abort"]["reason"]["kind"], "limit",
        "known negative bias must be removed before checking the real contact load: {response}"
    );
    assert_eq!(response["abort"]["reason"]["axis"], 0);
}

#[test]
fn missing_or_late_wrench_sample_uses_pre_authorized_abort() {
    let profile = simulation_wrench_profile("ee");
    let policy = &profile.limit_policy;
    assert_eq!(
        conservative_wrench_upper_bound(None, None, 1.0, policy),
        Err(WrenchAbortReason::MissingSample)
    );
    let late = monitor_sample(&profile, [0.0; 6]);
    assert_eq!(
        conservative_wrench_upper_bound(Some(&late), None, 1.03, policy),
        Err(WrenchAbortReason::StaleSample)
    );
    let gap = monitor_sample(&profile, [0.0; 6]);
    assert_eq!(
        conservative_wrench_upper_bound(Some(&gap), Some(0.98), gap.receive_s + 0.001, policy),
        Err(WrenchAbortReason::GapExceeded)
    );
}

#[test]
fn abort_contract_is_the_only_stop_write() {
    use realityos_verify::mujoco_exec::{ensure_mujoco_or_skip, MujocoInstance};
    if !ensure_mujoco_or_skip() {
        return;
    }
    let _guard = lock_mujoco_test();
    let bundle = planar_bundle();
    let mut worker = MujocoInstance::spawn().expect("MuJoCo worker");
    let loaded = worker
        .load_bundle_with_wrench_profile(&bundle, &[], 731, &simulation_wrench_profile("ee"))
        .expect("load wrench-instrumented fixture");
    let profile = monitored_profile_for_fixture("ee", &loaded["inspect"], None);
    let response = worker
        .rpc(&monitored_worker_request(&profile, false))
        .expect("monitored worker response");
    assert_eq!(response["ok"], true, "{}", response);
    let abort = &response["abort"];
    assert_eq!(abort["reason"]["kind"], "limit");
    assert_eq!(abort["stop_verified"], true);
    assert_eq!(
        abort["contract_id"],
        profile.limit_policy.abort_contract.contract_id
    );
    assert_eq!(abort["stop_write_count"], 1);
    assert_eq!(
        abort["stop_actuator_ids"].as_array().unwrap().len(),
        profile
            .limit_policy
            .abort_contract
            .allowed_stop_actuator_ids
            .len()
    );
    assert_eq!(response["state"]["safe_write_count"], 1);
    assert_eq!(response["state"]["ctrl_write_count"], 0);
    let stopped_at = response["state"]["time"].as_f64().unwrap();
    let denied = worker.step(10).expect("post-abort step response");
    assert_eq!(denied["ok"], false);
    assert_eq!(
        worker.state().expect("stopped state")["state"]["time"]
            .as_f64()
            .unwrap(),
        stopped_at
    );
}

#[test]
fn missing_or_late_sensor_stream_invokes_only_the_pre_authorized_abort() {
    use realityos_verify::mujoco_exec::{ensure_mujoco_or_skip, MujocoInstance};
    if !ensure_mujoco_or_skip() {
        return;
    }
    let _guard = lock_mujoco_test();
    let bundle = planar_bundle();
    let mut worker = MujocoInstance::spawn().expect("MuJoCo worker");
    let loaded = worker
        .load_bundle_with_wrench_profile(&bundle, &[], 732, &simulation_wrench_profile("ee"))
        .expect("load wrench-instrumented fixture");
    let profile = monitored_profile_for_fixture("ee", &loaded["inspect"], None);
    let response = worker
        .rpc(&monitored_worker_request(&profile, true))
        .expect("monitored missing-sample response");
    assert_eq!(response["ok"], true, "{}", response);
    assert_eq!(response["samples"].as_array().unwrap().len(), 0);
    assert_eq!(response["abort"]["reason"]["kind"], "missing_sample");
    assert_eq!(response["abort"]["stop_verified"], true);
    assert_eq!(response["state"]["safe_write_count"], 1);
    assert_eq!(response["state"]["ctrl_write_count"], 0);
    assert_eq!(worker.step(1).expect("latched step")["ok"], false);
}

#[test]
fn no_motion_or_reauthorization_follows_unverified_stop() {
    use realityos_verify::mujoco_exec::{ensure_mujoco_or_skip, MujocoInstance};
    if !ensure_mujoco_or_skip() {
        return;
    }
    let _guard = lock_mujoco_test();
    let bundle = planar_bundle();
    let mut worker = MujocoInstance::spawn().expect("MuJoCo worker");
    let loaded = worker
        .load_bundle_with_wrench_profile(&bundle, &[], 733, &simulation_wrench_profile("ee"))
        .expect("load wrench-instrumented fixture");
    let actuator = loaded["inspect"]["actuators"]
        .as_array()
        .unwrap()
        .iter()
        .find(|actuator| {
            matches!(actuator["trntype"].as_i64(), Some(0 | 1))
                && actuator["biastype"].as_i64() == Some(1)
                && actuator["ctrllimited"].as_bool() == Some(true)
        })
        .expect("position joint actuator");
    let actuator_name = actuator["name"].as_str().unwrap().to_owned();
    let joint_name = actuator["target"].as_str().unwrap();
    let qpos_address = loaded["inspect"]["joints"]
        .as_array()
        .unwrap()
        .iter()
        .find(|joint| joint["name"].as_str() == Some(joint_name))
        .unwrap()["qposadr"]
        .as_u64()
        .unwrap() as usize;
    let upper = actuator["ctrlrange"][1].as_f64().unwrap();
    let mut qpos = loaded["inspect"]["qpos0"]
        .as_array()
        .unwrap()
        .iter()
        .map(|value| value.as_f64().unwrap())
        .collect::<Vec<_>>();
    qpos[qpos_address] = upper + 0.5;
    worker
        .reset(Some(&qpos), None)
        .expect("out-of-range stop fixture");
    let profile =
        monitored_profile_for_fixture("ee", &loaded["inspect"], Some(vec![actuator_name]));
    let response = worker
        .rpc(&monitored_worker_request(&profile, false))
        .expect("monitored worker response");
    assert_eq!(response["ok"], true, "{}", response);
    assert_eq!(response["abort"]["stop_verified"], false);
    assert_eq!(response["abort"]["stop_write_count"], 0);
    assert_eq!(response["state"]["safe_write_count"], 0);
    let stopped_at = response["state"]["time"].as_f64().unwrap();
    let denied_step = worker.step(1).expect("latched physics response");
    assert_eq!(denied_step["ok"], false);
    assert_eq!(
        worker.state().expect("latched state")["state"]["time"]
            .as_f64()
            .unwrap(),
        stopped_at
    );
    let denied_reauthorization = worker
        .rpc(&monitored_worker_request(&profile, false))
        .expect("latched reauthorization response");
    assert_eq!(denied_reauthorization["ok"], false);
    assert_eq!(denied_reauthorization["error"], "WRENCH_ABORT_LATCHED");
}

#[test]
fn simulation_wrench_profile_requires_all_six_limits_and_sensor_fields() {
    let profile = simulation_wrench_profile("ee");
    assert!(profile.validate().is_ok());
    assert!(profile.validate_for_timestep(0.002).is_ok());
    assert!(profile.validate_for_timestep(0.003).is_err());

    let mut bad_caps = profile.clone();
    bad_caps.limit_policy.scope.caps[5] = 0.0;
    bad_caps.limit_policy.scope.policy_digest = bad_caps.limit_policy.digest();
    assert!(bad_caps.validate().is_err());

    let mut mismatched_sensor = profile.clone();
    mismatched_sensor.sensor_frame = "other_frame".into();
    assert!(mismatched_sensor.validate().is_err());

    let mut incomplete = serde_json::to_value(profile).unwrap();
    incomplete["limit_policy"]["scope"]["caps"] = serde_json::json!([20.0, 20.0, 30.0, 2.0, 2.0]);
    assert!(serde_json::from_value::<SimulationWrenchSensorProfile>(incomplete).is_err());

    let mut missing_sensor = serde_json::to_value(simulation_wrench_profile("ee")).unwrap();
    missing_sensor.as_object_mut().unwrap().remove("sensor_id");
    assert!(serde_json::from_value::<SimulationWrenchSensorProfile>(missing_sensor).is_err());

    let profile = simulation_wrench_profile("ee");
    let digest = profile.digest().unwrap();
    let mut changed = profile.clone();
    changed.resolution[5] += 0.0001;
    assert_ne!(digest, changed.digest().unwrap());

    let mut modeled = profile.clone();
    modeled.model_id = "mujoco-force-torque-sensor-free-space-model-v1".into();
    modeled.free_space_model_error_bounds[0] = 0.02;
    modeled.limit_policy.compensation_digest = modeled.compensation_policy_digest();
    modeled.limit_policy.scope.policy_digest = modeled.limit_policy.digest();
    assert!(modeled.validate().is_ok());
    let modeled_digest = modeled.digest().unwrap();
    modeled.free_space_model_error_bounds[0] += 0.01;
    assert!(modeled.validate().is_err());
    modeled.limit_policy.compensation_digest = modeled.compensation_policy_digest();
    modeled.limit_policy.scope.policy_digest = modeled.limit_policy.digest();
    assert_ne!(modeled_digest, modeled.digest().unwrap());
}

#[test]
fn contact_wrench_transform_matches_force_and_moment_balance() {
    let wrench = transform_contact_wrench_world(
        [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        [1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
        [2.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        true,
    );
    assert_eq!(wrench, [1.0, 2.0, 3.0, 4.0, -1.0, 10.0]);
}

#[test]
fn contact_force_sign_matches_geom_order() {
    let force_on_geom2 = contact_force_on_geom([1.0, -2.0, 3.0, 4.0, -5.0, 6.0], false);
    let force_on_geom1 = contact_force_on_geom([1.0, -2.0, 3.0, 4.0, -5.0, 6.0], true);
    assert_eq!(force_on_geom2, [1.0, -2.0, 3.0, 4.0, -5.0, 6.0]);
    assert_eq!(force_on_geom1, [-1.0, 2.0, -3.0, -4.0, 5.0, -6.0]);

    use realityos_verify::mujoco_exec::{ensure_mujoco_or_skip, MujocoInstance};
    if !ensure_mujoco_or_skip() {
        return;
    }
    let _guard = lock_mujoco_test();
    let bundle = planar_bundle();
    let mut worker = MujocoInstance::spawn().expect("MuJoCo worker");
    let profile = simulation_wrench_profile("ee");
    worker
        .load_bundle_with_wrench_profile(
            &bundle,
            &[serde_json::json!({
                "name": "sign_probe",
                "type": "box",
                "pos": [0.54, 0.0, 0.12],
                "size": [0.015, 0.015, 0.015],
                "mass": 0.05,
                "friction": 0.5
            })],
            721,
            &profile,
        )
        .expect("load contact fixture");
    let response = worker
        .step_with_wrench_profile(4, &profile)
        .expect("sample contact wrench");
    let force_x = response["samples"][0]["wrench"][0]
        .as_f64()
        .expect("sampled tool force");
    assert!(force_x < -0.02, "the tool is pushed toward -X: {force_x}");
}

#[test]
fn simulated_wrench_includes_gravity_loads_at_the_sensor() {
    use realityos_verify::mujoco_exec::{ensure_mujoco_or_skip, MujocoInstance};
    if !ensure_mujoco_or_skip() {
        return;
    }
    let _guard = lock_mujoco_test();
    let bundle = planar_bundle();
    let mut worker = MujocoInstance::spawn().expect("MuJoCo worker");
    let profile = simulation_wrench_profile("ee");
    worker
        .load_bundle_with_wrench_profile(&bundle, &[], 734, &profile)
        .expect("load wrench-instrumented fixture");
    let response = worker
        .step_with_wrench_profile(4, &profile)
        .expect("sample the unloaded tool sensor");
    let sample = response["samples"]
        .as_array()
        .unwrap()
        .last()
        .expect("one sensor sample");
    let vertical_force = sample["wrench"][2].as_f64().expect("vertical force");

    assert!(
        vertical_force.abs() > 0.25,
        "the sensor must report gravity/inertial load on the instrumented link, got {vertical_force}"
    );
}

#[test]
fn simulated_wrench_uses_the_declared_sensor_frame_orientation() {
    use realityos_verify::mujoco_exec::{ensure_mujoco_or_skip, MujocoInstance};
    if !ensure_mujoco_or_skip() {
        return;
    }
    let _guard = lock_mujoco_test();
    let bundle = planar_bundle();
    let object = serde_json::json!({
        "name": "sensor_frame_probe",
        "type": "box",
        "pos": [0.54, 0.0, 0.12],
        "size": [0.015, 0.015, 0.015],
        "mass": 0.05,
        "friction": 0.5
    });
    let sample_for_frame = |frame: &str| -> [f64; 6] {
        let mut worker = MujocoInstance::spawn().expect("MuJoCo worker");
        let profile = simulation_wrench_profile(frame);
        worker
            .load_bundle_with_wrench_profile(&bundle, std::slice::from_ref(&object), 735, &profile)
            .expect("load wrench-instrumented contact fixture");
        let response = worker
            .step_with_wrench_profile(4, &profile)
            .expect("sample tool contact in selected sensor frame");
        let sample = response["samples"]
            .as_array()
            .unwrap()
            .last()
            .expect("one sensor sample");
        std::array::from_fn(|axis| sample["wrench"][axis].as_f64().unwrap())
    };
    let identity = sample_for_frame("ee");
    let rotated = sample_for_frame("ee_rotated");

    assert!(
        (rotated[0] - identity[1]).abs() < 0.1
            && (rotated[1] + identity[0]).abs() < 0.1,
        "90-degree sensor rotation must rotate local axes: identity={identity:?}, rotated={rotated:?}"
    );
}

#[test]
fn simulation_wrench_samples_replay_for_same_trial_seed() {
    use realityos_verify::mujoco_exec::{ensure_mujoco_or_skip, MujocoInstance};

    if !ensure_mujoco_or_skip() {
        return;
    }
    let _guard = lock_mujoco_test();
    let bundle = planar_bundle();
    let profile = free_space_model_profile("ee");
    let sample = |seed| {
        let mut worker = MujocoInstance::spawn().expect("MuJoCo worker");
        worker
            .load_bundle_with_wrench_profile(&bundle, &[], seed, &profile)
            .expect("load wrench-instrumented fixture");
        worker.reset(None, None).expect("reset fixture");
        worker
            .step_with_wrench_profile(8, &profile)
            .expect("sample simulated wrench")
    };
    let first = sample(9021);
    let replay = sample(9021);
    assert_eq!(first["samples"], replay["samples"]);
    assert!(
        first["samples"].as_array().unwrap().iter().all(|sample| {
            sample["model_free_space_wrench"]
                .as_array()
                .is_some_and(|wrench| wrench.len() == 6)
                && sample["model_free_space_wrench_error_bounds"]
                    .as_array()
                    .is_some_and(|bounds| bounds.len() == 6)
        }),
        "every simulated wrench sample must carry a same-state model load and its declared bound"
    );
    let window =
        realityos_verify::manipulation_benchmark::SimulationWrenchWindow::from_worker_response(
            &first,
            &profile,
            "epoch-9021",
        )
        .expect("typed simulation wrench window");
    assert_eq!(window.samples.len(), 2);
    assert_eq!(
        window.sample_free_space_wrenches.len(),
        window.samples.len()
    );
    assert_eq!(
        window.sample_free_space_wrench_error_bounds.len(),
        window.samples.len()
    );
    assert_eq!(window.samples[0].origin, WrenchSampleOrigin::Simulation);
    assert_eq!(window.samples[0].sequence, 0);
    assert!((window.max_gap_s - 0.008).abs() < 1e-12);
    assert_eq!(
        window.compensation_digest,
        profile.limit_policy.compensation_digest
    );

    let mut mismatched_bound = first.clone();
    mismatched_bound["samples"][0]["model_free_space_wrench_error_bounds"][0] =
        serde_json::json!(0.01);
    assert!(
        realityos_verify::manipulation_benchmark::SimulationWrenchWindow::from_worker_response(
            &mismatched_bound,
            &profile,
            "epoch-9021-mismatched-bound",
        )
        .is_err()
    );
}

#[test]
fn simulation_wrench_free_space_load_tracks_the_current_arm_dynamics() {
    use realityos_verify::mujoco_exec::{ensure_mujoco_or_skip, MujocoInstance};

    if !ensure_mujoco_or_skip() {
        return;
    }
    let _guard = lock_mujoco_test();
    let bundle = planar_bundle();
    let profile = simulation_wrench_profile("ee");
    let mut worker = MujocoInstance::spawn().expect("MuJoCo worker");
    worker
        .load_bundle_with_wrench_profile(&bundle, &[], 9022, &profile)
        .expect("load moving-arm fixture");
    let (mut target_ctrl, _) = worker.peek_ctrl().expect("read initial control");
    *target_ctrl.first_mut().expect("first actuator") += 0.1;
    worker
        .set_ctrl(&target_ctrl)
        .expect("move arm in free space");
    let response = worker
        .step_with_wrench_profile(160, &profile)
        .expect("sample the moving arm");
    let records = response["samples"].as_array().expect("sample records");
    assert!(
        records.len() >= 2,
        "moving-arm fixture needs multiple samples"
    );
    for record in records {
        let raw = parse_six_for_test(&record["wrench"]);
        let model = parse_six_for_test(&record["model_free_space_wrench"]);
        for axis in 0..6 {
            let residual = raw[axis] - profile.bias[axis] - model[axis];
            assert!(
                residual.abs() <= profile.error_bounds[axis] + 1e-9,
                "free-space residual axis {axis} exceeds the registered bound: {residual}; raw={raw:?}, model={model:?}"
            );
        }
    }
    let first = parse_six_for_test(&records[0]["model_free_space_wrench"]);
    let last = parse_six_for_test(&records.last().unwrap()["model_free_space_wrench"]);
    assert!(
        first
            .iter()
            .zip(last)
            .any(|(left, right)| (left - right).abs() > 0.1),
        "same-state model compensation must follow changing arm loads"
    );
}

#[test]
fn simulation_wrench_free_space_compensation_preserves_live_contact_wrench() {
    use realityos_verify::mujoco_exec::{ensure_mujoco_or_skip, MujocoInstance};

    if !ensure_mujoco_or_skip() {
        return;
    }
    let _guard = lock_mujoco_test();
    let bundle = planar_bundle();
    let profile = free_space_model_profile("ee");
    let object = serde_json::json!({
        "name": "contact_compensation_probe",
        "type": "box",
        "pos": [0.50, 0.03, 0.12],
        "size": [0.015, 0.015, 0.015],
        "mass": 0.05,
        "friction": 0.5,
        "movable": false
    });
    let mut worker = MujocoInstance::spawn().expect("MuJoCo worker");
    worker
        .load_bundle_with_wrench_profile(&bundle, &[object], 9023, &profile)
        .expect("load live-contact fixture");
    worker
        .apply_force("link3", [0.0, 100.0, 0.0])
        .expect("force live contact to change the instrumented acceleration");
    let response = worker
        .step_with_wrench_profile(4, &profile)
        .expect("sample live contact and shadow compensation");
    let sample = response["samples"].as_array().unwrap().last().unwrap();
    let state = worker.step(0).expect("inspect same captured contact state")["state"].clone();
    let mut free_space_worker = MujocoInstance::spawn().expect("MuJoCo free-space worker");
    free_space_worker
        .load_bundle_with_wrench_profile(&bundle, &[], 9023, &profile)
        .expect("load paired no-contact fixture");
    free_space_worker
        .apply_force("link3", [0.0, 100.0, 0.0])
        .expect("apply the same external load in free space");
    free_space_worker
        .step_with_wrench_profile(4, &profile)
        .expect("advance paired no-contact fixture");
    let no_contact_state = free_space_worker
        .step(0)
        .expect("inspect paired no-contact state")["state"]
        .clone();
    let live_qacc = state["qacc"].as_array().expect("live qacc");
    let no_contact_qacc = no_contact_state["qacc"]
        .as_array()
        .expect("no-contact qacc");
    let qacc_delta = live_qacc
        .iter()
        .take(3)
        .zip(no_contact_qacc)
        .map(|(live, free)| (live.as_f64().unwrap() - free.as_f64().unwrap()).powi(2))
        .sum::<f64>()
        .sqrt();
    assert!(
        qacc_delta > 0.1,
        "regression fixture must distinguish live contact acceleration from the no-contact shadow state; delta={qacc_delta}; live={live_qacc:?}; no_contact={no_contact_qacc:?}"
    );
    let contacts = state["contacts"].as_array().expect("contact observations");
    assert!(
        contacts.iter().any(|contact| {
            contact["body1"].as_str() == Some("link3") || contact["body2"].as_str() == Some("link3")
        }),
        "the live fixture must load the force-sensor subtree"
    );

    let sensor_position = parse_six_or_three_for_test(&state["sites"]["ee"]);
    let sensor_quaternion = parse_quaternion_for_test(&state["site_xquat"]["ee"]);
    let mut contact_wrenches_world = Vec::new();
    for contact in contacts {
        let body1_is_tool = contact["body1"].as_str() == Some("link3");
        let body2_is_tool = contact["body2"].as_str() == Some("link3");
        if body1_is_tool == body2_is_tool {
            continue;
        }
        let frame_values = contact["frame"].as_array().expect("contact frame");
        let frame = std::array::from_fn(|row| {
            std::array::from_fn(|column| frame_values[row * 3 + column].as_f64().unwrap())
        });
        let local_force = contact["force_local"].as_array().expect("contact force");
        let contact_wrench = [
            local_force[0].as_f64().unwrap(),
            local_force[1].as_f64().unwrap(),
            local_force[2].as_f64().unwrap(),
            0.0,
            0.0,
            0.0,
        ];
        let position = contact["pos"].as_array().expect("contact position");
        contact_wrenches_world.push(transform_contact_wrench_world(
            frame,
            contact_wrench,
            std::array::from_fn(|axis| position[axis].as_f64().unwrap()),
            sensor_position,
            body2_is_tool,
        ));
    }
    let world_wrench = aggregate_contact_wrenches(&contact_wrenches_world);
    let rotation = test_quaternion_rotation(sensor_quaternion);
    let contact_wrench_at_sensor: [f64; 6] = std::array::from_fn(|axis| {
        let source = if axis < 3 { axis } else { axis - 3 };
        (0..3)
            .map(|world_axis| {
                rotation[world_axis][source] * world_wrench[world_axis + (axis / 3) * 3]
            })
            .sum::<f64>()
    });
    let raw = parse_six_for_test(&sample["wrench"]);
    let free_space = parse_six_for_test(&sample["model_free_space_wrench"]);
    let residual: [f64; 6] = std::array::from_fn(|axis| raw[axis] - free_space[axis]);
    for axis in 0..6 {
        let tolerance = if axis < 3 { 0.12 } else { 0.025 };
        assert!(
            (residual[axis] - contact_wrench_at_sensor[axis]).abs() <= tolerance,
            "contact-compensated wrench must match the independently reconstructed live contact load; axis={axis}, residual={residual:?}, contact={contact_wrench_at_sensor:?}, raw={raw:?}, free_space={free_space:?}"
        );
    }
}

fn parse_six_or_three_for_test(value: &Value) -> [f64; 3] {
    let values = value.as_array().expect("three-vector");
    assert_eq!(values.len(), 3);
    std::array::from_fn(|axis| values[axis].as_f64().expect("finite vector component"))
}

fn parse_quaternion_for_test(value: &Value) -> [f64; 4] {
    let values = value.as_array().expect("quaternion");
    assert_eq!(values.len(), 4);
    std::array::from_fn(|axis| values[axis].as_f64().expect("finite quaternion component"))
}

fn test_quaternion_rotation([w, x, y, z]: [f64; 4]) -> [[f64; 3]; 3] {
    [
        [
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - z * w),
            2.0 * (x * z + y * w),
        ],
        [
            2.0 * (x * y + z * w),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - x * w),
        ],
        [
            2.0 * (x * z - y * w),
            2.0 * (y * z + x * w),
            1.0 - 2.0 * (x * x + y * y),
        ],
    ]
}

fn parse_six_for_test(value: &Value) -> [f64; 6] {
    let values = value.as_array().expect("six-axis wrench");
    assert_eq!(values.len(), 6);
    std::array::from_fn(|axis| values[axis].as_f64().expect("finite wrench value"))
}

#[test]
fn simulation_wrench_window_retains_only_its_initial_precontact_baseline() {
    let profile = simulation_wrench_profile("ee");
    let sample = serde_json::json!({
        "sequence": 0,
        "capture_s": 0.0,
        "receive_s": profile.latency_s,
        "wrench": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "error_bounds": profile.error_bounds,
        "sensor_position": [0.1, 0.2, 0.3],
        "sensor_quaternion": [1.0, 0.0, 0.0, 0.0]
    });
    let response = serde_json::json!({
        "ok": true,
        "samples": [sample.clone()],
        "precontact_baseline": sample.clone(),
        "monitor_gravity_reference": sample.clone()
    });

    let window =
        realityos_verify::manipulation_benchmark::SimulationWrenchWindow::from_worker_response(
            &response,
            &profile,
            "epoch-baseline",
        )
        .expect("typed baseline sample");
    let serialized = serde_json::to_value(&window).unwrap();
    assert_eq!(serialized["precontact_baseline"]["sequence"], 0);
    assert_eq!(
        serialized["precontact_baseline_position"],
        serde_json::json!([0.1, 0.2, 0.3])
    );
    assert_eq!(
        serialized["precontact_baseline_orientation"],
        serde_json::json!([1.0, 0.0, 0.0, 0.0])
    );
    assert_eq!(
        serialized["monitor_gravity_reference"]["sequence"],
        serde_json::json!(0)
    );
    assert_eq!(
        serialized["monitor_gravity_reference_position"],
        serde_json::json!([0.1, 0.2, 0.3])
    );
    assert_eq!(
        serialized["sensor_positions"][0],
        serde_json::json!([0.1, 0.2, 0.3])
    );
    assert_eq!(
        serialized["sensor_orientations"][0],
        serde_json::json!([1.0, 0.0, 0.0, 0.0])
    );

    let mut mismatched_baseline = sample.clone();
    mismatched_baseline["sequence"] = serde_json::json!(1);
    let mismatched = serde_json::json!({
        "ok": true,
        "samples": [sample.clone()],
        "precontact_baseline": mismatched_baseline
    });
    assert!(
        realityos_verify::manipulation_benchmark::SimulationWrenchWindow::from_worker_response(
            &mismatched,
            &profile,
            "epoch-baseline",
        )
        .is_err()
    );

    let mut future_reference = sample.clone();
    future_reference["sequence"] = serde_json::json!(1);
    let future = serde_json::json!({
        "ok": true,
        "samples": [sample],
        "monitor_gravity_reference": future_reference
    });
    assert!(
        realityos_verify::manipulation_benchmark::SimulationWrenchWindow::from_worker_response(
            &future,
            &profile,
            "epoch-baseline",
        )
        .is_err()
    );

    let mut later_sample = sample.clone();
    later_sample["sequence"] = serde_json::json!(1);
    later_sample["capture_s"] = serde_json::json!(0.008);
    later_sample["receive_s"] = serde_json::json!(0.008 + profile.latency_s);
    let mid_batch = serde_json::json!({
        "ok": true,
        "state": { "time": 0.01 },
        "samples": [sample.clone(), later_sample.clone()],
        "precontact_baseline": later_sample.clone(),
        "monitor_gravity_reference": later_sample
    });
    let typed_mid_batch =
        realityos_verify::manipulation_benchmark::SimulationWrenchWindow::from_worker_response(
            &mid_batch,
            &profile,
            "epoch-baseline",
        )
        .expect("a reference captured later in the monitored batch is valid evidence");
    assert_eq!(
        typed_mid_batch
            .monitor_gravity_reference
            .as_ref()
            .map(|sample| sample.sequence),
        Some(1)
    );
}

#[test]
fn gravity_reference_is_transformed_and_subtracted_with_conservative_error() {
    let profile = simulation_wrench_profile("ee");
    let mut reference = monitor_sample(&profile, [10.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
    reference.capture_s = 1.0;
    reference.receive_s = 1.0 + profile.latency_s;
    reference.sequence = 1;
    reference.digest = wrench_sample_digest(&reference);
    let mut sample = monitor_sample(&profile, [0.0, -10.0, 0.0, 0.0, 0.0, 0.0]);
    sample.capture_s = 1.008;
    sample.receive_s = sample.capture_s + profile.latency_s;
    sample.sequence = 2;
    sample.digest = wrench_sample_digest(&sample);
    let half_sqrt = std::f64::consts::FRAC_1_SQRT_2;

    let (compensated, error) =
        realityos_verify::manipulation_benchmark::wrench_sensor::gravity_compensated_wrench(
            &sample,
            [0.0; 3],
            [half_sqrt, 0.0, 0.0, half_sqrt],
            profile.bias,
            &profile.model_id,
            Some((&reference, [0.0; 3], [1.0, 0.0, 0.0, 0.0])),
        )
        .expect("valid pose-tagged gravity reference");
    assert!(compensated.iter().all(|value| value.abs() < 1e-12));
    assert!(error[0] >= 0.1 && error[1] >= 0.1);

    sample.wrench[1] += 5.0;
    sample.digest = wrench_sample_digest(&sample);
    let (contact_increment, _) =
        realityos_verify::manipulation_benchmark::wrench_sensor::gravity_compensated_wrench(
            &sample,
            [0.0; 3],
            [half_sqrt, 0.0, 0.0, half_sqrt],
            profile.bias,
            &profile.model_id,
            Some((&reference, [0.0; 3], [1.0, 0.0, 0.0, 0.0])),
        )
        .expect("valid contact increment");
    assert!((contact_increment[1] - 5.0).abs() < 1e-12);
}

#[test]
fn gravity_reference_compensation_does_not_invent_a_translation_moment() {
    let profile = simulation_wrench_profile("ee");
    let mut reference = monitor_sample(&profile, [0.0, 0.0, -10.0, 0.0, 0.0, 0.0]);
    reference.capture_s = 1.0;
    reference.receive_s = 1.0 + profile.latency_s;
    reference.sequence = 1;
    reference.digest = wrench_sample_digest(&reference);
    let mut sample = monitor_sample(&profile, reference.wrench);
    sample.capture_s = 1.008;
    sample.receive_s = sample.capture_s + profile.latency_s;
    sample.sequence = 2;
    sample.digest = wrench_sample_digest(&sample);

    let (compensated, _) =
        realityos_verify::manipulation_benchmark::wrench_sensor::gravity_compensated_wrench(
            &sample,
            [0.1, 0.0, 0.0],
            [1.0, 0.0, 0.0, 0.0],
            profile.bias,
            &profile.model_id,
            Some((&reference, [0.0; 3], [1.0, 0.0, 0.0, 0.0])),
        )
        .expect("same body-fixed gravity load at a translated sensor pose");
    assert!(
        compensated.iter().all(|value| value.abs() < 1e-12),
        "translating a rigidly attached sensor must not invent a gravity moment: {compensated:?}"
    );
}

#[test]
fn legacy_v1_gravity_reference_keeps_point_wrench_translation() {
    let profile = simulation_wrench_profile("ee");
    let mut reference = monitor_sample(&profile, [0.0, 0.0, -10.0, 0.0, 0.0, 0.0]);
    reference.capture_s = 1.0;
    reference.receive_s = 1.0 + profile.latency_s;
    reference.sequence = 1;
    reference.digest = wrench_sample_digest(&reference);
    let mut sample = monitor_sample(&profile, reference.wrench);
    sample.capture_s = 1.008;
    sample.receive_s = sample.capture_s + profile.latency_s;
    sample.sequence = 2;
    sample.digest = wrench_sample_digest(&sample);

    let (compensated, _) =
        realityos_verify::manipulation_benchmark::wrench_sensor::gravity_compensated_wrench(
            &sample,
            [0.1, 0.0, 0.0],
            [1.0, 0.0, 0.0, 0.0],
            profile.bias,
            "mujoco-force-torque-sensor-gravity-baseline-v1",
            Some((&reference, [0.0; 3], [1.0, 0.0, 0.0, 0.0])),
        )
        .expect("legacy point-wrench profile remains supported");
    assert!((compensated[4] - 1.0).abs() < 1e-12);
}

#[test]
fn monitored_worker_emits_only_a_contact_free_initial_baseline() {
    use realityos_verify::mujoco_exec::{ensure_mujoco_or_skip, MujocoInstance};
    if !ensure_mujoco_or_skip() {
        return;
    }
    let _guard = lock_mujoco_test();
    let bundle = planar_bundle();
    let mut free_worker = MujocoInstance::spawn().expect("MuJoCo worker");
    let free_profile = simulation_wrench_profile("ee");
    let free_load = free_worker
        .load_bundle_with_wrench_profile(&bundle, &[], 941, &free_profile)
        .expect("load free baseline fixture");
    let mut free_profile = monitored_profile_for_fixture("ee", &free_load["inspect"], None);
    free_profile.limit_policy.scope.caps = [100.0; 6];
    free_profile.limit_policy.scope.policy_digest = free_profile.limit_policy.digest();
    assert!(free_profile.validate().is_ok());
    let mut free_request = monitored_worker_request(&free_profile, false);
    free_request["request_precontact_baseline"] = serde_json::json!(true);
    let free_response = free_worker
        .rpc(&free_request)
        .expect("monitored free response");
    assert_eq!(free_response["ok"], true, "{free_response}");
    assert_eq!(free_response["precontact_baseline_status"], "captured");
    assert_eq!(free_response["precontact_baseline"]["sequence"], 0);
    let typed_free =
        realityos_verify::manipulation_benchmark::SimulationWrenchWindow::from_worker_response(
            &free_response,
            &free_profile,
            "monitor-test-epoch",
        )
        .expect("typed monitored baseline");
    assert!(typed_free.precontact_baseline.is_some());
    assert!(typed_free.monitor_gravity_reference.is_some());
    assert_eq!(typed_free.sensor_positions.len(), typed_free.samples.len());
    assert_eq!(
        typed_free.sensor_orientations.len(),
        typed_free.samples.len()
    );

    let mut refresh_request = monitored_worker_request(&free_profile, false);
    refresh_request["request_monitor_gravity_reference"] = serde_json::json!(true);
    let refreshed_response = free_worker
        .rpc(&refresh_request)
        .expect("refresh contact-free gravity reference");
    assert_eq!(refreshed_response["ok"], true, "{refreshed_response}");
    assert_eq!(
        refreshed_response["monitor_gravity_reference_status"], "captured",
        "an explicit request at a new motion phase must refresh the reference"
    );
    assert!(
        refreshed_response["monitor_gravity_reference"]["capture_s"]
            .as_f64()
            .unwrap()
            > typed_free
                .monitor_gravity_reference
                .as_ref()
                .unwrap()
                .capture_s,
        "a refreshed gravity reference must carry fresh sensor evidence"
    );

    let mut accelerating_worker = MujocoInstance::spawn().expect("MuJoCo worker");
    let accelerating_bundle = iiwa14_bundle();
    let accelerating_profile = simulation_wrench_profile("attachment_site");
    let accelerating_load = accelerating_worker
        .load_bundle_with_wrench_profile(&accelerating_bundle, &[], 945, &accelerating_profile)
        .expect("load robot with initially accelerating home joints");
    let qacc = accelerating_load["state"]["qacc"]
        .as_array()
        .expect("initial generalized accelerations");
    assert!(qacc
        .iter()
        .filter_map(Value::as_f64)
        .any(|value| value.abs() > 0.01));
    let mut accelerating_profile =
        monitored_profile_for_fixture("attachment_site", &accelerating_load["inspect"], None);
    accelerating_profile.limit_policy.scope.caps = [500.0; 6];
    accelerating_profile.limit_policy.scope.policy_digest =
        accelerating_profile.limit_policy.digest();
    assert!(accelerating_profile.validate().is_ok());
    let mut accelerating_request = monitored_worker_request(&accelerating_profile, false);
    accelerating_request["n"] = serde_json::json!(1);
    accelerating_request["request_monitor_gravity_reference"] = serde_json::json!(true);
    let accelerating_response = accelerating_worker
        .rpc(&accelerating_request)
        .expect("monitored accelerating response");
    assert_eq!(accelerating_response["ok"], true, "{accelerating_response}");
    assert_eq!(
        accelerating_response["monitor_gravity_reference_status"],
        "sensor_not_quiescent",
        "zero velocity at the instant of reset does not make a dynamically accelerating arm a valid gravity reference"
    );
    assert!(accelerating_response["precontact_baseline"].is_null());
    let typed_accelerating =
        realityos_verify::manipulation_benchmark::SimulationWrenchWindow::from_worker_response(
            &accelerating_response,
            &accelerating_profile,
            "monitor-test-epoch",
        )
        .expect("typed monitor-reference status");
    assert_eq!(
        typed_accelerating
            .monitor_gravity_reference_status
            .as_deref(),
        Some("sensor_not_quiescent")
    );
    assert!(typed_accelerating.monitor_gravity_reference.is_none());

    let mut accelerating_settle_request = monitored_worker_request(&accelerating_profile, false);
    accelerating_settle_request["n"] = serde_json::json!(2_000);
    accelerating_settle_request["request_monitor_gravity_reference"] = serde_json::json!(true);
    accelerating_settle_request["request_precontact_baseline"] = serde_json::json!(true);
    let accelerating_settle_response = accelerating_worker
        .rpc(&accelerating_settle_request)
        .expect("settle and refresh inside one monitored batch");
    assert_eq!(
        accelerating_settle_response["ok"], true,
        "{accelerating_settle_response}"
    );
    assert_eq!(
        accelerating_settle_response["monitor_gravity_reference_status"], "captured",
        "a monitored batch should refresh the reference once the arm is settled"
    );
    assert_eq!(
        accelerating_settle_response["precontact_baseline_status"], "captured",
        "a monitored batch should capture the inference baseline once the arm is settled"
    );
    assert!(
        accelerating_settle_response["monitor_gravity_reference"]["capture_s"]
            .as_f64()
            .expect("settled reference capture time")
            > typed_accelerating
                .monitor_gravity_reference
                .as_ref()
                .map_or(0.0, |sample| sample.capture_s)
    );

    let mut moving_worker = MujocoInstance::spawn().expect("MuJoCo worker");
    let moving_profile = simulation_wrench_profile("ee");
    let moving_load = moving_worker
        .load_bundle_with_wrench_profile(
            &bundle,
            &[serde_json::json!({
                "name": "baseline_unrelated_moving_object",
                "type": "box",
                "pos": [2.0, 0.0, 0.5],
                "size": [0.02, 0.02, 0.02],
                "mass": 0.05,
                "friction": 0.5,
                "movable": true
            })],
            944,
            &moving_profile,
        )
        .expect("load unrelated moving-body fixture");
    let moving_profile = monitored_profile_for_fixture("ee", &moving_load["inspect"], None);
    let nv = moving_load["inspect"]["nv"].as_u64().unwrap() as usize;
    let mut qvel = vec![0.0; nv];
    let free_dof = moving_load["inspect"]["joints"]
        .as_array()
        .unwrap()
        .iter()
        .find(|joint| joint["child_body"].as_str() == Some("baseline_unrelated_moving_object"))
        .expect("fixture free joint")["dofadr"]
        .as_u64()
        .unwrap() as usize;
    qvel[free_dof] = 0.2;
    moving_worker
        .reset(None, Some(&qvel))
        .expect("set unrelated body velocity");
    let mut moving_request = monitored_worker_request(&moving_profile, false);
    moving_request["request_precontact_baseline"] = serde_json::json!(true);
    let moving_response = moving_worker
        .rpc(&moving_request)
        .expect("monitored moving-scene response");
    assert_eq!(moving_response["ok"], true, "{moving_response}");
    assert_eq!(moving_response["precontact_baseline_status"], "captured");
    assert!(moving_response["precontact_baseline"].is_object());

    let mut contact_worker = MujocoInstance::spawn().expect("MuJoCo worker");
    let contact_profile = simulation_wrench_profile("ee");
    let contact_load = contact_worker
        .load_bundle_with_wrench_profile(
            &bundle,
            &[serde_json::json!({
                "name": "baseline_contact_probe",
                "type": "box",
                "pos": [0.54, 0.0, 0.12],
                "size": [0.015, 0.015, 0.015],
                "mass": 0.05,
                "friction": 0.5
            })],
            942,
            &contact_profile,
        )
        .expect("load contact fixture");
    let contact_profile = monitored_profile_for_fixture("ee", &contact_load["inspect"], None);
    let mut contact_request = monitored_worker_request(&contact_profile, false);
    contact_request["request_precontact_baseline"] = serde_json::json!(true);
    let contact_response = contact_worker
        .rpc(&contact_request)
        .expect("monitored contact response");
    assert_eq!(contact_response["ok"], true, "{contact_response}");
    assert_eq!(
        contact_response["precontact_baseline_status"],
        "contact_present"
    );
    assert!(contact_response["precontact_baseline"].is_null());
    let typed_contact =
        realityos_verify::manipulation_benchmark::SimulationWrenchWindow::from_worker_response(
            &contact_response,
            &contact_profile,
            "monitor-test-epoch",
        )
        .expect("typed contact trace");
    assert!(typed_contact.precontact_baseline.is_none());
    assert!(typed_contact.monitor_gravity_reference.is_none());
}

#[test]
fn randomized_trial_reset_holds_position_actuators_at_the_loaded_pose() {
    use realityos_verify::mujoco_exec::{ensure_mujoco_or_skip, MujocoInstance};
    if !ensure_mujoco_or_skip() {
        return;
    }
    let _guard = lock_mujoco_test();
    let bundle = planar_bundle();
    let mut worker = MujocoInstance::spawn().expect("MuJoCo worker");
    let loaded = worker
        .load_bundle(&bundle, &[], 943)
        .expect("load position-control fixture");
    let mut qpos = loaded["inspect"]["qpos0"]
        .as_array()
        .unwrap()
        .iter()
        .map(|value| value.as_f64().unwrap())
        .collect::<Vec<_>>();
    qpos[0] += 0.25;
    let reset = worker.reset(Some(&qpos), None).expect("randomized reset");
    let ctrl = reset["state"]["ctrl"].as_array().unwrap();
    let actuators = loaded["inspect"]["actuators"].as_array().unwrap();
    let joints = loaded["inspect"]["joints"].as_array().unwrap();
    let mut checked = 0;
    for (actuator_index, actuator) in actuators.iter().enumerate() {
        if !matches!(actuator["trntype"].as_i64(), Some(0 | 1))
            || actuator["biastype"].as_i64() != Some(1)
        {
            continue;
        }
        let joint_name = actuator["target"].as_str().unwrap();
        let qpos_address = joints
            .iter()
            .find(|joint| joint["name"].as_str() == Some(joint_name))
            .unwrap()["qposadr"]
            .as_u64()
            .unwrap() as usize;
        assert_eq!(ctrl[actuator_index].as_f64(), Some(qpos[qpos_address]));
        checked += 1;
    }
    assert!(checked > 0, "fixture must contain position actuators");
}

#[test]
fn sensor_adapter_aggregates_unexpected_tool_contacts() {
    let aggregate = aggregate_contact_wrenches(&[
        [1.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        [2.0, 0.0, 0.0, 0.0, 0.0, 2.0],
    ]);
    assert_eq!(aggregate, [3.0, 0.0, 0.0, 0.0, 0.0, 3.0]);

    use realityos_verify::mujoco_exec::{ensure_mujoco_or_skip, MujocoInstance};
    if !ensure_mujoco_or_skip() {
        return;
    }
    let _guard = lock_mujoco_test();
    let bundle = planar_bundle();
    let profile = simulation_wrench_profile("ee");
    let one_contact = serde_json::json!([{
        "name": "unmodeled_fixture_contact_a",
        "type": "box",
        "pos": [0.54, 0.0, 0.12],
        "size": [0.015, 0.015, 0.015],
        "mass": 0.05,
        "friction": 0.5
    }]);
    let two_unexpected_contacts = serde_json::json!([
        {
            "name": "unmodeled_fixture_contact_a",
            "type": "box",
            "pos": [0.54, 0.0, 0.12],
            "size": [0.015, 0.015, 0.015],
            "mass": 0.05,
            "friction": 0.5
        },
        {
            "name": "unmodeled_fixture_contact_b",
            "type": "box",
            "pos": [0.54, 0.025, 0.12],
            "size": [0.015, 0.015, 0.015],
            "mass": 0.05,
            "friction": 0.5
        }
    ]);
    let sample = |objects: &[Value]| {
        let mut worker = MujocoInstance::spawn().expect("MuJoCo worker");
        worker
            .load_bundle_with_wrench_profile(&bundle, objects, 722, &profile)
            .expect("load contacts");
        worker
            .step_with_wrench_profile(4, &profile)
            .expect("sample unexpected tool contacts")
    };
    let single = sample(one_contact.as_array().unwrap());
    let multiple = sample(two_unexpected_contacts.as_array().unwrap());
    let single_x = single["samples"][0]["wrench"][0].as_f64().unwrap().abs();
    let multiple_x = multiple["samples"][0]["wrench"][0].as_f64().unwrap().abs();
    assert!(single_x > 0.02, "unexpected tool contact must be sensed");
    assert!(
        multiple_x > single_x,
        "all tool contacts must contribute: {single_x} -> {multiple_x}"
    );
    let output = multiple.to_string();
    assert!(!output.contains("geom1") && !output.contains("body1") && !output.contains("contacts"));
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

fn scenario_suite_v2() -> ScenarioSuiteManifest {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../robots/benchmarks/manipulation/scenario_suite_v2.json");
    ScenarioSuiteManifest::load(&path).expect("checked-in v2 scenario suite")
}

fn scenario_suite_v3() -> ScenarioSuiteManifest {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../robots/benchmarks/manipulation/scenario_suite_v3.json");
    ScenarioSuiteManifest::load(&path).expect("checked-in v3 scenario suite")
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
    realityos_verify::menagerie::ensure_development_model().expect("pinned UR5e menagerie model");
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../robots/external/ur5e");
    RobotBundle::load(root).expect("checked-in UR5e bundle with mesh assets")
}

fn iiwa14_bundle() -> RobotBundle {
    realityos_verify::menagerie::ensure_v2_holdout_model().expect("pinned IIWA-14 menagerie model");
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../robots/external/iiwa14");
    RobotBundle::load(root).expect("checked-in IIWA-14 bundle with mesh assets")
}

fn wx250s_bundle() -> RobotBundle {
    realityos_verify::menagerie::ensure_manipulation_holdout_model()
        .expect("pinned WX250S menagerie model");
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

fn live_iiwa_v3_positive_trial() -> (
    RobotBundle,
    realityos_verify::manipulation_benchmark::ResolvedBenchmarkTrial,
) {
    let bundle = iiwa14_bundle();
    let suite = scenario_suite_v3();
    let (worker, manifest) = load_and_normalize(&bundle, &[], suite.pilot_seeds[0])
        .expect("strict benchmark requires MuJoCo");
    checkin_worker(worker);
    let trial = resolve_benchmark_trial_with_suite(
        suite.pilot_seeds[0],
        0,
        BenchmarkTask::PushProbeReplan,
        EmbodimentRole::Development,
        &bundle,
        &manifest,
        &suite,
    )
    .expect("positive IIWA v3 trial resolves");
    (bundle, trial)
}

fn live_ur5e_v3_positive_trial() -> (
    RobotBundle,
    realityos_verify::manipulation_benchmark::ResolvedBenchmarkTrial,
) {
    let bundle = ur5e_bundle();
    let suite = scenario_suite_v3();
    let (worker, manifest) = load_and_normalize(&bundle, &[], suite.pilot_seeds[0])
        .expect("positive UR5e path requires MuJoCo");
    checkin_worker(worker);
    let trial = resolve_benchmark_trial_with_suite(
        suite.pilot_seeds[0],
        0,
        BenchmarkTask::PushProbeReplan,
        EmbodimentRole::Development,
        &bundle,
        &manifest,
        &suite,
    )
    .expect("positive UR5e v3 trial resolves");
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
    trial.paired_trial_key = Some(paired_trial_key(&trial).unwrap());
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
        paired_trial_key: trial.paired_trial_key.clone(),
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
        wrench_sensor_profile_digest: None,
        wrench_compensation_digest: None,
        wrench_inference_compensation_digest: None,
        wrench_inference_measurement_digest: None,
        wrench_precontact_baseline: None,
        wrench_precontact_baseline_status: None,
        wrench_contact_pairs: Vec::new(),
        wrench_monitor_gravity_reference: None,
        wrench_monitor_gravity_reference_status: None,
        wrench_monitor_gravity_reference_history: Vec::new(),
        wrench_monitor_gravity_reference_position: None,
        wrench_monitor_gravity_reference_orientation: None,
        wrench_sample_count: 0,
        wrench_max_sample_age_s: None,
        wrench_max_inter_sample_gap_s: None,
        wrench_max_object_speed_m_s: None,
        wrench_peak_conservative_bounds: None,
        wrench_caps: None,
        wrench_minimum_margin: None,
        wrench_abort: None,
        wrench_abort_axis: None,
        wrench_abort_result: None,
        wrench_inference_interval: None,
        wrench_inference_reason: None,
        prior_support_friction_interval: None,
        posterior_support_friction_interval: None,
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
        wrench_samples: vec![],
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
            class: SensorClass::JointEncoder,
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
        class: SensorClass::JointEncoder,
        device_serial: "ENC-SN-001".into(),
        model_id: "encoder-v2".into(),
        calibration_hash: "calibration-1".into(),
        frame: "joint-1".into(),
        units: "rad".into(),
        min_sample_rate_hz: 1.0,
        max_inter_sample_gap_s: 1.0,
        max_age_s: 0.1,
        max_cross_sensor_skew_s: 0.02,
    }];
    (trace, requirements)
}

fn force_torque_trace(
    origins: [WrenchSampleOrigin; 2],
) -> (PhysicalSensorTrace, Vec<PhysicalSensorRequirement>) {
    let (mut trace, mut requirements) = physical_trace(SensorOrigin::Hardware);
    trace.observation.observations.push(SensorObservation {
        sensor_id: "ft-1".into(),
        class: SensorClass::ForceTorque,
        robot_frame: "tool0".into(),
        calibration_hash: "ft-cal-1".into(),
        device_serial: Some("FT-SN-001".into()),
        model_id: Some("ft-model-1".into()),
        units: Some("N,N*m".into()),
        capture_s: 10.01,
        receive_s: 10.011,
        sequence: 2,
        clock_domain: "robot_clock".into(),
        digest: "ft-observation-digest".into(),
        expires_at_s: 10.2,
    });
    trace.sensors.push(PhysicalSensorIdentity {
        sensor_id: "ft-1".into(),
        class: SensorClass::ForceTorque,
        device_serial: "FT-SN-001".into(),
        model_id: "ft-model-1".into(),
        calibration_hash: "ft-cal-1".into(),
        frame: "tool0".into(),
        units: "N,N*m".into(),
    });
    trace.observation.wrench_samples = (0..2)
        .map(|index| {
            let sequence = index + 1;
            let mut sample = WrenchSample {
                sensor_id: "ft-1".into(),
                sensor_frame: "tool0".into(),
                transform_epoch: "epoch-1".into(),
                units: "N,N*m".into(),
                wrench: [0.0, 0.0, 2.0, 0.0, 0.0, 0.0],
                error_bounds: [0.02; 6],
                capture_s: 10.0 + index as f64 * 0.01,
                receive_s: 10.001 + index as f64 * 0.01,
                sequence,
                clock_domain: "robot_clock".into(),
                origin: origins[index as usize],
                digest: String::new(),
            };
            sample.digest = wrench_sample_digest(&sample);
            sample
        })
        .collect();
    requirements.push(PhysicalSensorRequirement {
        sensor_id: "ft-1".into(),
        class: SensorClass::ForceTorque,
        device_serial: "FT-SN-001".into(),
        model_id: "ft-model-1".into(),
        calibration_hash: "ft-cal-1".into(),
        frame: "tool0".into(),
        units: "N,N*m".into(),
        min_sample_rate_hz: 90.0,
        max_inter_sample_gap_s: 0.02,
        max_age_s: 0.1,
        max_cross_sensor_skew_s: 0.02,
    });
    (trace, requirements)
}

#[test]
fn physical_evidence_force_torque_trace_requires_declared_class_and_identity() {
    let (trace, requirements) = force_torque_trace([WrenchSampleOrigin::Hardware; 2]);
    validate_physical_observation(&trace, &requirements, 10.05)
        .expect("matching force/torque sensor identity is accepted");

    let mut wrong_observation_class = trace.clone();
    wrong_observation_class.observation.observations[1].class = SensorClass::Other;
    assert!(validate_physical_observation(&wrong_observation_class, &requirements, 10.05).is_err());

    let mut wrong_identity_class = trace.clone();
    wrong_identity_class.sensors[1].class = SensorClass::Other;
    assert!(validate_physical_observation(&wrong_identity_class, &requirements, 10.05).is_err());

    let mut wrong_serial = trace.clone();
    wrong_serial.observation.observations[1].device_serial = Some("other-serial".into());
    assert!(validate_physical_observation(&wrong_serial, &requirements, 10.05).is_err());

    let mut missing_requirement = requirements.clone();
    missing_requirement[1].class = SensorClass::Other;
    assert!(validate_physical_observation(&trace, &missing_requirement, 10.05).is_err());

    let mut missing_samples = trace.clone();
    missing_samples.observation.wrench_samples.clear();
    assert!(validate_physical_observation(&missing_samples, &requirements, 10.05).is_err());

    let mut duplicate_channel = trace.clone();
    duplicate_channel
        .observation
        .observations
        .push(duplicate_channel.observation.observations[1].clone());
    assert!(validate_physical_observation(&duplicate_channel, &requirements, 10.05).is_err());

    let mut undeclared_sample = trace.clone();
    undeclared_sample.observation.wrench_samples[0].sensor_id = "undeclared".into();
    undeclared_sample.observation.wrench_samples[0].digest =
        wrench_sample_digest(&undeclared_sample.observation.wrench_samples[0]);
    assert!(validate_physical_observation(&undeclared_sample, &requirements, 10.05).is_err());
}

#[test]
fn physical_evidence_rejects_simulation_and_synthetic_wrench_origins() {
    for origin in [
        WrenchSampleOrigin::Simulation,
        WrenchSampleOrigin::Synthetic,
    ] {
        let (trace, requirements) = force_torque_trace([origin, WrenchSampleOrigin::Hardware]);
        assert!(validate_physical_observation(&trace, &requirements, 10.05).is_err());
    }
    let (mut trace, requirements) = force_torque_trace([WrenchSampleOrigin::Hardware; 2]);
    trace.origin = SensorOrigin::Simulation;
    assert!(validate_physical_observation(&trace, &requirements, 10.05).is_err());
}

#[test]
fn physical_evidence_wrench_trace_rejects_bad_digest_or_replayed_sequence() {
    let (trace, requirements) = force_torque_trace([WrenchSampleOrigin::Hardware; 2]);
    let mut bad_digest = trace.clone();
    let first_digest_char = if bad_digest.observation.wrench_samples[0]
        .digest
        .starts_with('0')
    {
        "1"
    } else {
        "0"
    };
    bad_digest.observation.wrench_samples[0]
        .digest
        .replace_range(..1, first_digest_char);
    assert!(validate_physical_observation(&bad_digest, &requirements, 10.05).is_err());

    let mut replayed = trace.clone();
    replayed.observation.wrench_samples[1].sequence = 1;
    replayed.observation.wrench_samples[1].digest =
        wrench_sample_digest(&replayed.observation.wrench_samples[1]);
    assert!(validate_physical_observation(&replayed, &requirements, 10.05).is_err());

    let mut wrong_clock = trace;
    wrong_clock.observation.wrench_samples[0].clock_domain = "other_hardware_clock".into();
    wrong_clock.observation.wrench_samples[0].digest =
        wrench_sample_digest(&wrong_clock.observation.wrench_samples[0]);
    assert!(validate_physical_observation(&wrong_clock, &requirements, 10.05).is_err());
}

#[test]
fn physical_evidence_wrench_trace_rejects_rate_gap_age_and_cross_sensor_skew() {
    let (trace, mut requirements) = force_torque_trace([WrenchSampleOrigin::Hardware; 2]);
    requirements[1].min_sample_rate_hz = 150.0;
    assert!(validate_physical_observation(&trace, &requirements, 10.05).is_err());

    let (trace, mut requirements) = force_torque_trace([WrenchSampleOrigin::Hardware; 2]);
    requirements[1].max_inter_sample_gap_s = 0.005;
    assert!(validate_physical_observation(&trace, &requirements, 10.05).is_err());

    let (trace, mut requirements) = force_torque_trace([WrenchSampleOrigin::Hardware; 2]);
    requirements[1].max_age_s = 0.01;
    assert!(validate_physical_observation(&trace, &requirements, 10.05).is_err());

    let (mut trace, mut requirements) = force_torque_trace([WrenchSampleOrigin::Hardware; 2]);
    trace.observation.as_of_s = 10.06;
    trace.policy_observation.timestamp_s = 10.06;
    trace.observation.observations[1].capture_s = 10.05;
    trace.observation.observations[1].receive_s = 10.051;
    trace.observation.observations[1].sequence = 3;
    for index in 0..2 {
        let sequence = (index + 1) as u64;
        let mut sample = WrenchSample {
            sensor_id: "ft-2".into(),
            sensor_frame: "tool0".into(),
            transform_epoch: "epoch-1".into(),
            units: "N,N*m".into(),
            wrench: [0.0; 6],
            error_bounds: [0.02; 6],
            capture_s: 10.04 + index as f64 * 0.01,
            receive_s: 10.041 + index as f64 * 0.01,
            sequence,
            clock_domain: "robot_clock".into(),
            origin: WrenchSampleOrigin::Hardware,
            digest: String::new(),
        };
        sample.digest = wrench_sample_digest(&sample);
        trace.observation.wrench_samples.push(sample);
    }
    trace.sensors.push(PhysicalSensorIdentity {
        sensor_id: "ft-2".into(),
        class: SensorClass::ForceTorque,
        device_serial: "FT-SN-002".into(),
        model_id: "ft-model-1".into(),
        calibration_hash: "ft-cal-1".into(),
        frame: "tool0".into(),
        units: "N,N*m".into(),
    });
    requirements.push(PhysicalSensorRequirement {
        sensor_id: "ft-2".into(),
        class: SensorClass::ForceTorque,
        device_serial: "FT-SN-002".into(),
        model_id: "ft-model-1".into(),
        calibration_hash: "ft-cal-1".into(),
        frame: "tool0".into(),
        units: "N,N*m".into(),
        min_sample_rate_hz: 90.0,
        max_inter_sample_gap_s: 0.02,
        max_age_s: 0.1,
        max_cross_sensor_skew_s: 0.02,
    });
    trace.observation.observations.push(SensorObservation {
        sensor_id: "ft-2".into(),
        class: SensorClass::ForceTorque,
        robot_frame: "tool0".into(),
        calibration_hash: "ft-cal-1".into(),
        device_serial: Some("FT-SN-002".into()),
        model_id: Some("ft-model-1".into()),
        units: Some("N,N*m".into()),
        capture_s: 10.05,
        receive_s: 10.051,
        sequence: 3,
        clock_domain: "robot_clock".into(),
        digest: "ft-observation-2".into(),
        expires_at_s: 10.2,
    });
    assert!(validate_physical_observation(&trace, &requirements, 10.08).is_err());
}

#[test]
fn physical_evidence_valid_hardware_wrench_trace_stays_unverified_without_outcome_measurement() {
    let (trace, requirements) = force_torque_trace([WrenchSampleOrigin::Hardware; 2]);
    let evidence = validate_physical_observation(&trace, &requirements, 10.05)
        .expect("valid hardware provenance is accepted for review");
    assert_eq!(evidence.status, PhysicalEvidenceStatus::Unverified);
    assert!(evidence.sensor_provenance_valid);
    assert!(evidence.requires_independent_measurement);
    assert_eq!(evidence.sensor_ids, ["encoder-1", "ft-1"]);

    let mut different_trace = trace;
    different_trace.observation.wrench_samples[0].wrench[0] += 0.1;
    different_trace.observation.wrench_samples[0].digest =
        wrench_sample_digest(&different_trace.observation.wrench_samples[0]);
    let different = validate_physical_observation(&different_trace, &requirements, 10.05)
        .expect("changed valid measurement remains provenance-valid");
    assert_ne!(different.trace_digest, evidence.trace_digest);
    assert_eq!(different.status, PhysicalEvidenceStatus::Unverified);
}

#[test]
fn infeasible_planar_probe_refuses_before_control_writes() {
    let _mujoco_guard = lock_mujoco_test();
    let (bundle, trial) = live_trial(9010, 10, BenchmarkTask::PushProbeReplan);
    let episode = run_test_benchmark_trial(&bundle, &trial, simulation_admission(120.0))
        .expect("the runner records the safe refusal");

    assert_eq!(episode.outcome, TrialOutcome::ValidRefusal);
    assert_eq!(episode.failure_cause, Some(FailureCause::BridgeRefusal));
    assert_eq!(
        episode.refusal_code.as_deref(),
        Some("NO_JUSTIFIED_CURRENT_ACTION")
    );
    assert!(episode.actions.is_empty());
    assert_eq!(episode.unauthorized_writes, 0);
}

#[test]
fn expired_authority_stops_control_steps_before_continuing_the_segment() {
    let _mujoco_guard = lock_mujoco_test();
    let (bundle, trial) = live_iiwa_v3_positive_trial();

    // The probe needs up to 2 seconds of contact-free sampling before its
    // first control ramp. Leave a short authorized window after that settle so
    // this test exercises expiry during a ramp rather than before any writes.
    let episode = run_test_benchmark_trial(&bundle, &trial, simulation_admission(12.5))
        .expect("the runner records an authority-expiry episode");

    assert_eq!(episode.outcome, TrialOutcome::ActionFailed);
    assert_eq!(
        episode.refusal_code.as_deref(),
        Some("AUTHORITY_EXPIRED_DURING_EXECUTION"),
        "abort={:?}; simulation_time={}; outcome={:?}",
        episode.wrench_abort,
        episode.simulation_time_s,
        episode.outcome
    );
    assert_eq!(
        episode.actions.len(),
        1,
        "expired authority must not continue to another target"
    );
    assert!(episode.ctrl_writes_after > episode.ctrl_writes_before);
    assert!(
        (2.0..=2.502).contains(&episode.simulation_time_s),
        "the simulator must finish probe settling and stop at the first expired control window; time={}",
        episode.simulation_time_s
    );
    assert_eq!(episode.unauthorized_writes, 0);
}

#[test]
fn divergent_action_stops_before_replan() {
    let _mujoco_guard = lock_mujoco_test();
    let (bundle, mut trial) = live_iiwa_v3_positive_trial();
    trial.fault_injection = Some("runtime_divergence".into());
    assert_eq!(trial.expectation, TrialExpectation::ExpectedSuccess);

    let episode = run_test_benchmark_trial(&bundle, &trial, simulation_admission(120.0))
        .expect("runner records the interrupted action");

    assert_eq!(
        episode.failure_cause,
        Some(realityos_verify::manipulation_benchmark::FailureCause::RuntimeDivergence),
        "outcome={:?}; refusal={:?}; decisions={:?}; abort={:?}",
        episode.outcome,
        episode.refusal_code,
        episode.decision_audits,
        episode.wrench_abort
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
fn unlabeled_probe_grant_gap_is_not_a_false_acceptance() {
    use realityos_verify::manipulation_benchmark::evaluation_label_mismatch;
    use realityos_verify::manipulation_scenarios::Polarity;

    let mut unlabeled = synthetic_trial(21, 3, TrialExpectation::ExpectedRefusal);
    unlabeled.scenario.expected_refusal = None;
    unlabeled.scenario.polarity = Polarity::Positive;
    unlabeled.scenario.neg = None;
    unlabeled.scenario.stale = false;
    unlabeled.scenario.move_object_after_obs = false;
    unlabeled.scenario.replay = false;
    unlabeled.scenario.restart_replay = false;
    unlabeled.scenario.crash_controller = false;
    unlabeled.scenario.immovable = false;
    let preview = "decision=Refuse { reason: \"SCOPED_GRANT_DOES_NOT_COVER_PROBE\" };candidates=planar_push:probe:+x:PROVED";
    assert!(evaluation_label_mismatch(
        "probe:+x",
        &unlabeled.scenario,
        0,
        preview,
    ));
    assert!(
        !evaluation_label_mismatch("push:+x", &unlabeled.scenario, 0, preview),
        "a goal push on an expected refusal stays a false acceptance"
    );
    assert!(
        !evaluation_label_mismatch("probe:+x", &unlabeled.scenario, 1, preview),
        "an unauthorized write is not relabeled"
    );
    assert!(!evaluation_label_mismatch(
        "probe:+x",
        &unlabeled.scenario,
        0,
        "decision=Execute { action_key: \"push:+x\" }",
    ));
    let mut faulted = unlabeled.scenario.clone();
    faulted.stale = true;
    assert!(!evaluation_label_mismatch("probe:+x", &faulted, 0, preview));
    faulted.stale = false;
    faulted.expected_refusal = Some("FORCE_BOUND_UNAVAILABLE".into());
    assert!(!evaluation_label_mismatch("probe:+x", &faulted, 0, preview));

    unlabeled.paired_trial_key = Some(paired_trial_key(&unlabeled).expect("paired key"));
    let config = config_for_trial_ids(vec![unlabeled.trial_id.clone()]);
    let mismatch = synthetic_episode(
        &unlabeled,
        TrialOutcome::EvaluationLabelMismatch,
        Some(FailureCause::EvaluationLabelMismatch),
    );
    let report = score_campaign(
        &config,
        &[BenchmarkTrialRecord::completed(unlabeled.clone(), mismatch)],
    )
    .expect("label mismatch scores");
    assert_eq!(report.false_acceptances, 0);
    assert_eq!(report.evaluation_label_mismatches, 1);
    assert_eq!(report.unauthorized_writes, 0);
    assert!(report.safety_gate_passed);

    let mut written = synthetic_episode(&unlabeled, TrialOutcome::Success, None);
    written.unauthorized_writes = 1;
    let written_report = score_campaign(
        &config,
        &[BenchmarkTrialRecord::completed(unlabeled, written)],
    )
    .expect("unauthorized write scores");
    assert_eq!(written_report.false_acceptances, 1);
    assert_eq!(written_report.unauthorized_writes, 1);
    assert!(!written_report.safety_gate_passed);
}

#[test]
fn unverified_wrench_stop_fails_safety_gate() {
    let trial = synthetic_trial(28, 10, TrialExpectation::ExpectedSuccess);
    let config = config_for_trial_ids(vec![trial.trial_id.clone()]);
    let mut episode = synthetic_episode(
        &trial,
        TrialOutcome::ActionFailed,
        Some(FailureCause::ForceMonitorAbort),
    );
    episode.wrench_abort = Some(
        realityos_verify::manipulation_benchmark::wrench_sensor::WrenchAbortEvent {
            reason: WrenchAbortReason::MonitorFault,
            triggering_sample: None,
            triggering_sensor_position: None,
            triggering_sensor_orientation: None,
            triggered_at_s: 1.0,
            stopped_at_s: 1.05,
            stop_write_count: 0,
            contract_id: "stop".into(),
            stop_verified: false,
            stop_actuator_ids: Vec::new(),
            stop_error: None,
        },
    );
    episode.wrench_abort_result = Some("preauthorized_hold_unverified_fault_latched".into());
    let records = vec![BenchmarkTrialRecord::completed(trial, episode)];

    let report = score_campaign(&config, &records).expect("unverified stop remains reportable");

    assert_eq!(report.unverified_wrench_stops, 1);
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
            lateral_displacement_m: Some(0.002),
            reauthorized_after_observation: false,
            runtime_divergence: true,
            control_tracking_correction_bound_rad: None,
            control_tracking_correction_gain: None,
            control_tracking_correction_max_rad: None,
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
            lateral_displacement_m: Some(0.003),
            reauthorized_after_observation: false,
            runtime_divergence: false,
            control_tracking_correction_bound_rad: None,
            control_tracking_correction_gain: None,
            control_tracking_correction_max_rad: None,
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
fn ur5e_probe_is_reobserved_or_stopped_by_its_force_limit() {
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

    let episode = run_test_benchmark_trial(&bundle, &trial, simulation_admission(120.0))
        .expect("strict runner records the uninformative probe and refusal");

    if episode.failure_cause
        == Some(realityos_verify::manipulation_benchmark::FailureCause::ForceMonitorAbort)
    {
        assert_eq!(episode.outcome, TrialOutcome::ActionFailed);
        assert_eq!(
            episode.refusal_code.as_deref(),
            Some("WRENCH_MONITOR_ABORTED_ACTION")
        );
        let abort = episode
            .wrench_abort
            .as_ref()
            .expect("over-limit probe records its force monitor abort");
        assert!(
            abort.stop_verified,
            "pre-authorized hold must be verified: {abort:?}"
        );
        assert_eq!(abort.stop_write_count, 1);
        assert_eq!(episode.unauthorized_writes, 0);
        return;
    }

    assert_eq!(
        episode
            .actions
            .iter()
            .filter(|action| action.action_key.starts_with("probe:"))
            .count(),
        1,
        "same uninformative probe must not consume the remaining action budget: actions={:?}; refusal={:?}; failure={:?}; decisions={:?}; requests={:?}",
        episode.actions,
        episode.refusal_code,
        episode.failure_cause,
        episode.decision_audits,
        episode.reasoning_request_audits
    );
    assert_eq!(
        episode.outcome,
        TrialOutcome::UnexpectedRefusal,
        "abort={:?}; inference={:?}; failure={:?}; refusal={:?}",
        episode.wrench_abort,
        episode.wrench_inference_reason,
        episode.failure_cause,
        episode.refusal_code
    );
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
            lateral_displacement_m: Some(0.002),
            reauthorized_after_observation: false,
            runtime_divergence: false,
            control_tracking_correction_bound_rad: None,
            control_tracking_correction_gain: None,
            control_tracking_correction_max_rad: None,
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
            lateral_displacement_m: Some(0.007),
            reauthorized_after_observation: true,
            runtime_divergence: true,
            control_tracking_correction_bound_rad: None,
            control_tracking_correction_gain: None,
            control_tracking_correction_max_rad: None,
        },
    ];
    episode.recovery_count = 1;
    episode.wall_time_s = 1.5;
    let records = vec![BenchmarkTrialRecord::completed(trial, episode)];

    let report = score_campaign(&config, &records).expect("valid report");

    assert_eq!(report.recovery_trials, 1);
    assert_eq!(report.stale_authority_continuations, 0);
    assert_eq!(report.failure_causes.get("runtime_divergence"), Some(&1));
    assert_eq!(report.wall_time_p50_s, Some(1.5));
    assert_eq!(report.wall_time_p95_s, Some(1.5));

    let mut stale_episode = records[0].episode.clone().expect("completed episode");
    stale_episode.actions[0].runtime_divergence = true;
    stale_episode.actions[1].grant_id = stale_episode.actions[0].grant_id.clone();
    stale_episode.actions[1].observation_epoch = stale_episode.actions[0].observation_epoch.clone();
    stale_episode.actions[1].reauthorized_after_observation = true;
    let stale_records = vec![BenchmarkTrialRecord::completed(
        records[0].trial.clone().expect("resolved trial"),
        stale_episode,
    )];
    let stale_report = score_campaign(&config, &stale_records).expect("valid stale report");
    assert_eq!(stale_report.stale_authority_continuations, 1);
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
    let episode = run_test_benchmark_trial(&bundle, &trial, simulation_admission(120.0))
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
    let episode = run_test_benchmark_trial(&bundle, &trial, simulation_admission(120.0))
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

    let episode = run_test_benchmark_trial(&bundle, &trial, simulation_admission(120.0))
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

        let episode = run_test_benchmark_trial(&bundle, &trial, simulation_admission(120.0))
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
fn push_contact_and_slip_faults_refuse_before_control_writes() {
    let _mujoco_guard = lock_mujoco_test();
    let cases = [
        (
            3,
            "UNEXPECTED_CONTACT",
            "UnexpectedContact",
            "UNEXPECTED_CONTACT_REPORTED",
            "mujoco_fault_injection_adapter.unexpected_contact",
        ),
        (
            4,
            "SLIP",
            "SlipDetected",
            "SLIP_DETECTED",
            "mujoco_fault_injection_adapter.slip_detected",
        ),
    ];

    for (index, fault, kind, refusal, source) in cases {
        let (bundle, trial) = live_trial(42, index, BenchmarkTask::PushProbeReplan);
        assert_eq!(trial.expectation, TrialExpectation::ExpectedRefusal);
        assert_eq!(trial.scenario.neg.map(|neg| neg.as_str()), Some(fault));
        let episode = run_test_benchmark_trial(&bundle, &trial, simulation_admission(120.0))
            .expect("strict runner records injected contact/slip refusal");

        assert_eq!(episode.outcome, TrialOutcome::ValidRefusal, "{fault}");
        assert_eq!(
            episode.ctrl_writes_before, episode.ctrl_writes_after,
            "{fault}"
        );
        assert_eq!(episode.unauthorized_writes, 0, "{fault}");
        assert!(
            episode.refusal_code.is_some(),
            "the bridge must persist its refusal code for {fault}"
        );
        assert!(
            episode
                .decision_audits
                .iter()
                .any(|audit| audit.contains(refusal)),
            "the reasoner decision must preserve typed refusal {refusal}: {:?}",
            episode.decision_audits
        );
        assert_eq!(episode.safety_fault_observations.len(), 1, "{fault}");
        assert_eq!(
            format!("{:?}", episode.safety_fault_observations[0].kind),
            kind
        );
        assert_eq!(episode.safety_fault_observations[0].asserted.source, source);
        assert_eq!(episode.reasoning_request_audits.len(), 1, "{fault}");
        let request = &episode.reasoning_request_audits[0];
        assert!(request.contains(source), "{fault}: {request}");
        assert!(
            !request.contains(fault),
            "evaluator fault label leaked: {fault}"
        );
    }
}

#[test]
fn resolved_positive_grasp_records_verified_hold() {
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

    let episode = run_test_benchmark_trial(&bundle, &trial, simulation_admission(120.0))
        .expect("strict runner records the grasp episode");

    assert_eq!(episode.unauthorized_writes, 0);
    assert_eq!(episode.acquisition_verified, Some(true));
    assert_eq!(episode.hold_verified, Some(true));
    assert_eq!(episode.outcome, TrialOutcome::Success);
    assert_eq!(episode.failure_cause, None);
    assert!(!episode
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
    let episode = run_test_benchmark_trial(&bundle, &trial, simulation_admission(120.0))
        .expect("the runner records a MuJoCo grasp episode");

    assert_eq!(episode.unauthorized_writes, 0);
    assert_eq!(
        episode.acquisition_verified,
        Some(true),
        "cause={:?}; phases={:?}",
        episode.failure_cause,
        episode.grasp_phase_object_positions
    );
    assert_eq!(
        episode.hold_verified,
        Some(true),
        "qualified positive grasp failed hold verification: cause={:?}; refusal={:?}; evidence={:?}; phases={:?}; audits={:?}",
        episode.failure_cause,
        episode.refusal_code,
        episode.grasp_verifier_evidence,
        episode.grasp_phase_object_positions,
        episode.grasp_contact_audits
    );
    assert_eq!(episode.outcome, TrialOutcome::Success);
    assert!(episode
        .grasp_verifier_evidence
        .iter()
        .any(|item| item == "verified_hold"));
    let lift_audits = episode
        .grasp_contact_audits
        .iter()
        .filter(|audit| audit.phase.starts_with("lift_waypoint_"))
        .collect::<Vec<_>>();
    assert!(
        lift_audits.len() >= 4,
        "the executed lift must expose intermediate checked waypoints: {lift_audits:?}"
    );
    assert!(lift_audits.iter().all(|audit| {
        audit.force_available
            && audit
                .normal_force_n
                .is_some_and(|force| force <= 1.95 + 1e-9)
    }));
    assert!(episode.grasp_phase_object_positions.len() >= 3 + lift_audits.len() / 2);
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

    let episode = run_test_benchmark_trial(
        &bundle,
        &trial,
        simulation_admission(10.0 + manifest.timestep * 500.0),
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
    assert!(public.iter().all(|entry| {
        entry.tasks.contains(&BenchmarkTask::PushProbeReplan)
            == entry.simulation_wrench_profile.is_some()
    }));
    assert!(holdout[0].simulation_wrench_profile.is_some());
    assert!(public
        .iter()
        .filter_map(|entry| entry.simulation_wrench_profile.as_ref())
        .all(|profile| profile.validate().is_ok()));
    assert!(public
        .iter()
        .filter_map(|entry| entry.simulation_wrench_profile.as_ref())
        .all(|profile| {
            profile.sample_rate_hz == 500.0
                && profile.latency_s == 0.002
                && profile.limit_policy.max_sensing_latency_s == 0.002
                && profile.model_id == "mujoco-force-torque-sensor-free-space-model-v1"
                && profile.limit_policy.max_position_tracking_correction_rad == 0.01
                && profile.limit_policy.position_tracking_correction_gain == 1.0
        }));
    let mut incomplete_registry = registry.clone();
    incomplete_registry.registrations[0].simulation_wrench_profile = None;
    assert!(incomplete_registry.validate().is_err());
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

    let episode = run_test_benchmark_trial(&bundle, &trial, simulation_admission(120.0))
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
    let mut development = registry
        .selected(false)
        .into_iter()
        .filter(|entry| entry.role == EmbodimentRole::Development)
        .collect::<Vec<_>>();
    development.sort_by(|left, right| left.robot_id.cmp(&right.robot_id));
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
        let episode = run_test_benchmark_trial(&bundle, &trial, simulation_admission(120.0))
            .expect("same bridge runner records both embodiments");
        assert_eq!(episode.task, BenchmarkTask::PushProbeReplan);
        assert_eq!(episode.role, EmbodimentRole::Development);
        assert_eq!(episode.evidence_status, "SIMULATION_ONLY");
        assert!(!episode.geometry_digest.is_empty());
        assert_eq!(episode.unauthorized_writes, 0);
        assert!(!episode.decision_audits.is_empty());
        assert!(!episode.reasoning_request_audits.is_empty());
        if trial.expectation == TrialExpectation::ExpectedRefusal {
            assert_eq!(episode.outcome, TrialOutcome::ValidRefusal);
            assert_eq!(episode.ctrl_writes_before, episode.ctrl_writes_after);
        }
        if episode.outcome != TrialOutcome::Success {
            assert!(episode.failure_cause.is_some());
            assert!(
                episode.refusal_code.is_some()
                    || episode
                        .actions
                        .iter()
                        .any(|action| action.runtime_divergence),
                "{} did not preserve a failure reason: {:?}",
                entry.robot_id,
                episode
            );
        }
        if episode
            .actions
            .iter()
            .any(|action| action.runtime_divergence)
        {
            assert_eq!(episode.failure_cause, Some(FailureCause::RuntimeDivergence));
            assert!(
                episode.decision_audits.len() >= 2,
                "divergence must be followed by fresh reasoning"
            );
            assert_eq!(episode.actions.len(), 1, "the divergent witness must stop");
            let action = &episode.actions[0];
            let displacement_limit_m = if action.action_key.starts_with("probe:") {
                0.018
            } else {
                0.035
            };
            assert!(
                action.displacement_m <= displacement_limit_m,
                "measured object motion exceeded the checked witness envelope: {action:?}"
            );
        }
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
fn paired_trial_key_ignores_schema_but_binds_randomized_inputs() {
    let trial = synthetic_trial(9071, 12, TrialExpectation::ExpectedSuccess);
    let key = paired_trial_key(&trial).expect("canonical paired key");

    let mut legacy = trial.clone();
    legacy.schema_version = 5;
    legacy.paired_trial_key = None;
    assert_eq!(paired_trial_key(&legacy).unwrap(), key);

    let mut changed_qpos = trial.clone();
    changed_qpos.initial_qpos[0] = f64::from_bits(changed_qpos.initial_qpos[0].to_bits() + 1);
    assert_ne!(paired_trial_key(&changed_qpos).unwrap(), key);

    let mut changed_scenario = trial.clone();
    changed_scenario.scenario.objects[0]["mass"] = serde_json::json!(0.123456789);
    assert_ne!(paired_trial_key(&changed_scenario).unwrap(), key);

    let mut changed_fault_injection = trial.clone();
    changed_fault_injection.fault_injection = Some("runtime_divergence".into());
    assert_ne!(paired_trial_key(&changed_fault_injection).unwrap(), key);
}

#[test]
fn scenario_suite_rejects_overlapping_seed_ranges() {
    let mut suite = scenario_suite_v2();
    suite.public_seeds = suite.pilot_seeds.clone();

    let error = suite
        .validate()
        .expect_err("pilot and public seeds must not overlap");
    assert!(error.to_string().contains("seed"));
}

#[test]
fn scenario_suite_rejects_invalid_strata_or_probe_gap() {
    let mut suite = scenario_suite_v2();
    suite.stratum_period = 4;
    assert!(suite.validate().is_err());

    let mut suite = scenario_suite_v2();
    suite.push_approach_gap_m = suite.push_probe_stroke_m;
    assert!(suite.validate().is_err());

    let mut suite = scenario_suite_v2();
    suite.positive_push.half_height_m = [0.0, 0.0];
    assert!(suite.validate().is_err());
}

#[test]
fn scenario_suite_v2_probe_gap_leaves_four_mm_after_contact() {
    let suite = scenario_suite_v2();

    assert_eq!(suite.push_approach_gap_m, 0.004);
    assert_eq!(suite.push_probe_stroke_m, 0.008);
    assert_eq!(suite.minimum_post_contact_stroke_m, 0.004);
    assert!(
        suite.push_probe_stroke_m - suite.push_approach_gap_m
            >= suite.minimum_post_contact_stroke_m
    );
}

#[test]
fn scenario_suite_v3_positive_stratum_is_force_observable() {
    let suite = scenario_suite_v3();
    let v2 = scenario_suite_v2();

    assert_eq!(suite.suite_version, 3);
    assert_ne!(suite.suite_id, v2.suite_id);
    assert_eq!(suite.positive_push.mass_kg, [1.0, 1.25]);
    assert_eq!(suite.positive_push.friction, [0.4, 0.8]);
    assert_eq!(suite.positive_push.half_size_m, [0.05, 0.07]);
    assert_eq!(suite.positive_push.half_height_m, [0.02, 0.04]);
    let unidentifiable = suite
        .unidentifiable_push
        .as_ref()
        .expect("v3 declares a low-load refusal stratum");
    assert_eq!(unidentifiable.mass_kg, [0.05, 0.18]);
    assert_eq!(unidentifiable.friction, [0.2, 0.8]);
    assert!(suite
        .refusal_cycle
        .iter()
        .any(|label| label == "unidentifiable_friction"));
    assert_ne!(suite.pilot_seeds, v2.pilot_seeds);
    assert_ne!(suite.public_seeds, v2.public_seeds);
}

#[test]
fn v3_simulation_support_contact_uses_randomized_sliding_friction() {
    use realityos_verify::mujoco_exec::{ensure_mujoco_or_skip, MujocoInstance};
    if !ensure_mujoco_or_skip() {
        return;
    }
    let _guard = lock_mujoco_test();
    let bundle = iiwa14_bundle();
    let objects = vec![
        serde_json::json!({
            "name":"obj0","type":"box","pos":[0.6115262561608635,0.017454385150215315,0.5011534150470114],
            "size":[0.02812522593375574,0.02812522593375574,0.08115341504701144],"mass":1.2182473334463388,
            "friction":0.7221006358270854,"solref":[0.005,1.0],"movable":true
        }),
        serde_json::json!({
            "name":"table","type":"box","pos":[0.9,0.0,0.4],"size":[0.4,0.25,0.02],"mass":20.0,"movable":false,
            "friction":0.7221006358270854,"solref":[0.005,1.0]
        }),
    ];
    let mut worker = MujocoInstance::spawn().expect("MuJoCo worker");
    let loaded = worker
        .load_bundle(&bundle, &objects, 21_000)
        .expect("load support friction fixture");
    let geoms = loaded["inspect"]["geoms"]
        .as_array()
        .expect("compiled geoms");
    for body in ["obj0", "table"] {
        let geom = geoms
            .iter()
            .find(|geom| geom["body"] == body)
            .expect("compiled support geom");
        assert_eq!(geom["solref"], serde_json::json!([0.005, 1.0]));
    }
    let contacts = worker.state().expect("initial contact state")["state"]["contacts"]
        .as_array()
        .cloned()
        .unwrap_or_default();
    let support_contacts = contacts
        .iter()
        .filter(|contact| {
            (contact["body1"] == "obj0" && contact["body2"] == "table")
                || (contact["body2"] == "obj0" && contact["body1"] == "table")
        })
        .collect::<Vec<_>>();
    assert!(!support_contacts.is_empty());
    assert!(support_contacts.iter().all(|contact| {
        (0..2).all(|axis| {
            (contact["friction"][axis].as_f64().unwrap() - 0.7221006358270854).abs() < 1e-12
        })
    }));
}

#[test]
fn wrench_abort_event_preserves_stop_failure_reason() {
    let event: realityos_verify::manipulation_benchmark::wrench_sensor::WrenchAbortEvent =
        serde_json::from_value(serde_json::json!({
            "reason": { "kind": "monitor_fault" },
            "triggered_at_s": 0.048,
            "stopped_at_s": 0.048,
            "stop_write_count": 0,
            "contract_id": "sim-wrench-stop-v1",
            "stop_verified": false,
            "stop_actuator_ids": [],
            "stop_error": "wrench_stop_control_out_of_range:actuator1"
        }))
        .expect("typed stop audit");

    assert_eq!(
        event.stop_error.as_deref(),
        Some("wrench_stop_control_out_of_range:actuator1")
    );
}

#[test]
fn monitored_contact_and_actuator_trips_have_typed_reasons() {
    assert_eq!(
        serde_json::from_value::<WrenchAbortReason>(serde_json::json!({
            "kind": "unexpected_contact"
        }))
        .expect("unexpected contact reason"),
        WrenchAbortReason::UnexpectedContact
    );
    assert_eq!(
        serde_json::from_value::<WrenchAbortReason>(serde_json::json!({
            "kind": "actuator_saturation",
            "actuator_ids": ["joint2_actuator"]
        }))
        .expect("actuator saturation reason"),
        WrenchAbortReason::ActuatorSaturation {
            actuator_ids: vec!["joint2_actuator".into()]
        }
    );
}

#[test]
fn v7_push_strata_are_interleaved_and_balanced() {
    let suite = scenario_suite_v2();
    let pilot = suite
        .pilot_seeds
        .iter()
        .enumerate()
        .map(|(index, _)| suite.stratum_for_index(index as u32))
        .collect::<Vec<_>>();
    let public = suite
        .public_seeds
        .iter()
        .enumerate()
        .map(|(index, _)| suite.stratum_for_index(index as u32))
        .collect::<Vec<_>>();

    assert_eq!(
        pilot
            .iter()
            .filter(|stratum| **stratum == PushTrialStratum::ExpectedSuccess)
            .count(),
        12
    );
    assert_eq!(
        pilot
            .iter()
            .filter(|stratum| **stratum == PushTrialStratum::ExpectedRefusal)
            .count(),
        8
    );
    assert_eq!(
        public
            .iter()
            .filter(|stratum| **stratum == PushTrialStratum::ExpectedSuccess)
            .count(),
        60
    );
    assert_eq!(
        public
            .iter()
            .filter(|stratum| **stratum == PushTrialStratum::ExpectedRefusal)
            .count(),
        40
    );
    assert_eq!(
        suite.stratum_for_index(0),
        PushTrialStratum::ExpectedSuccess
    );
    assert_eq!(
        suite.stratum_for_index(3),
        PushTrialStratum::ExpectedRefusal
    );
}

#[test]
fn suite_identity_changes_paired_key() {
    let mut trial = synthetic_trial(9073, 12, TrialExpectation::ExpectedSuccess);
    trial.scenario_suite_id = Some("feasible_public_probe".into());
    trial.scenario_suite_version = Some(2);
    trial.stratum = Some(PushTrialStratum::ExpectedSuccess);
    let key = paired_trial_key(&trial).expect("v7 paired key");

    let mut changed = trial.clone();
    changed.scenario_suite_version = Some(3);
    assert_ne!(paired_trial_key(&changed).unwrap(), key);
}

#[test]
fn v5_records_remain_readable_without_wrench_claims() {
    let trial = synthetic_trial(9072, 13, TrialExpectation::ExpectedSuccess);
    let episode = synthetic_episode(&trial, TrialOutcome::Success, None);
    let record = BenchmarkTrialRecord::completed(trial.clone(), episode);
    let mut legacy_value = serde_json::to_value(record).unwrap();
    legacy_value["trial"]["schema_version"] = serde_json::json!(5);
    legacy_value["trial"]["trial_id"] = serde_json::json!(trial.trial_id.replace("v6:", "v5:"));
    legacy_value["trial"]
        .as_object_mut()
        .unwrap()
        .remove("paired_trial_key");
    legacy_value["episode"]
        .as_object_mut()
        .unwrap()
        .remove("paired_trial_key");
    legacy_value["episode"]["schema_version"] = serde_json::json!(5);
    let episode = legacy_value["episode"].as_object_mut().unwrap();
    for field in [
        "wrench_max_object_speed_m_s",
        "wrench_sensor_profile_digest",
        "wrench_compensation_digest",
        "wrench_sample_count",
        "wrench_max_sample_age_s",
        "wrench_max_inter_sample_gap_s",
        "wrench_peak_conservative_bounds",
        "wrench_caps",
        "wrench_minimum_margin",
        "wrench_abort",
        "wrench_abort_axis",
        "wrench_abort_result",
        "wrench_inference_interval",
        "wrench_inference_reason",
        "prior_support_friction_interval",
        "posterior_support_friction_interval",
    ] {
        episode.remove(field);
    }
    legacy_value["trial_id"] = legacy_value["trial"]["trial_id"].clone();
    legacy_value["episode"]["trial_id"] = legacy_value["trial"]["trial_id"].clone();

    let legacy: BenchmarkTrialRecord = serde_json::from_value(legacy_value).unwrap();
    assert!(legacy.trial.as_ref().unwrap().paired_trial_key.is_none());
    assert_eq!(legacy.episode.as_ref().unwrap().wrench_sample_count, 0);
    let mut legacy_config = config_for_trial_ids(vec![legacy.trial_id.clone()]);
    legacy_config.schema_version = 5;
    let legacy_report = score_campaign(&legacy_config, std::slice::from_ref(&legacy)).unwrap();
    assert_eq!(legacy_report.schema_version, 5);
    assert_eq!(legacy_report.wrench_metrics.sample_count, 0);

    let candidate = BenchmarkTrialRecord::completed(
        trial.clone(),
        synthetic_episode(&trial, TrialOutcome::Success, None),
    );
    let delta = paired_campaign_delta(&[legacy], &[candidate]).unwrap();
    assert_eq!(delta.matched_trial_keys, 1);
}

#[test]
fn v6_force_aware_pilot_remains_readable_after_schema_v7() {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join(
        "../../docs/superpowers/evidence/force-aware-probe-7d8ee51/manipulation_benchmark.json",
    );
    let report: realityos_verify::manipulation_benchmark::BenchmarkReport =
        serde_json::from_slice(&std::fs::read(path).expect("frozen schema-v6 pilot"))
            .expect("schema-v6 report remains readable");
    assert_eq!(report.schema_version, 6);
    assert_eq!(report.config.schema_version, 6);

    let rescored = score_campaign(&report.config, &report.records)
        .expect("schema-v6 frozen pilot remains auditable");
    assert_eq!(rescored.schema_version, 6);
    assert_eq!(rescored.scheduled_trials, 60);
    assert_eq!(rescored.records.len(), 60);
}

#[test]
fn frozen_v5_pilot_inputs_upgrade_to_exact_v6_pair_keys() {
    let repo_root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let baseline_path = repo_root.join(
        "docs/superpowers/evidence/probe-stroke-paired-198b5de/probe8mm/manipulation_benchmark.json",
    );
    let baseline: realityos_verify::manipulation_benchmark::BenchmarkReport =
        serde_json::from_slice(&std::fs::read(&baseline_path).expect("frozen public baseline"))
            .expect("schema-v5 baseline remains readable");
    assert_eq!(baseline.schema_version, 5);
    assert!(baseline.campaign_complete && baseline.safety_gate_passed);
    assert_eq!(baseline.records.len(), 60);

    let mut legacy_keys = std::collections::BTreeSet::new();
    let mut candidate_keys = std::collections::BTreeSet::new();
    for record in &baseline.records {
        let legacy = record.trial.as_ref().expect("resolved baseline trial");
        assert_eq!(legacy.schema_version, 5);
        let upgraded = upgrade_v5_trial_for_v6(legacy).expect("schema-v5 input upgrades");
        assert_eq!(upgraded.schema_version, 6);
        assert_eq!(upgraded.trial_id, legacy.trial_id.replacen("v5:", "v6:", 1));
        assert_eq!(legacy.scenario, upgraded.scenario);
        assert_eq!(
            legacy.goal_xyz.map(f64::to_bits),
            upgraded.goal_xyz.map(f64::to_bits),
            "goal changed for {}",
            legacy.trial_id
        );
        assert_eq!(
            legacy
                .initial_qpos
                .iter()
                .map(|value| value.to_bits())
                .collect::<Vec<_>>(),
            upgraded
                .initial_qpos
                .iter()
                .map(|value| value.to_bits())
                .collect::<Vec<_>>(),
            "initial joint state changed for {}",
            legacy.trial_id
        );
        let legacy_key = paired_trial_key(legacy).unwrap();
        let candidate_key = upgraded.paired_trial_key.clone().unwrap();
        assert_eq!(legacy_key, candidate_key);
        assert!(legacy_keys.insert(legacy_key));
        assert!(candidate_keys.insert(candidate_key));
    }
    assert_eq!(legacy_keys.len(), 60);
    assert_eq!(candidate_keys, legacy_keys);

    let candidate_records = baseline
        .records
        .iter()
        .map(|record| {
            let legacy = record.trial.as_ref().expect("resolved baseline trial");
            let trial = upgrade_v5_trial_for_v6(legacy).expect("schema-v5 input upgrades");
            if let Some(episode) = record.episode.as_ref() {
                let mut episode = episode.clone();
                episode.schema_version = 6;
                episode.paired_trial_key = trial.paired_trial_key.clone();
                episode.trial_id = trial.trial_id.clone();
                episode.trial_digest = trial_digest(&trial).unwrap();
                BenchmarkTrialRecord::completed(trial, episode)
            } else {
                BenchmarkTrialRecord::failed(
                    trial,
                    record.error.clone().expect("baseline error record"),
                )
            }
        })
        .collect::<Vec<_>>();
    let paired = paired_campaign_delta(&baseline.records, &candidate_records)
        .expect("all 60 frozen v5 records pair exactly to schema v6");
    assert_eq!(paired.matched_trial_keys, 60);
}

#[test]
fn schema_v6_scoring_and_execution_require_persisted_pair_keys() {
    let mut trial = synthetic_trial(9076, 15, TrialExpectation::ExpectedSuccess);
    trial.paired_trial_key = None;
    let episode = synthetic_episode(&trial, TrialOutcome::Success, None);
    let record = BenchmarkTrialRecord::completed(trial.clone(), episode);
    let config = config_for_trial_ids(vec![trial.trial_id.clone()]);
    assert!(score_campaign(&config, &[record]).is_err());

    let error = realityos_verify::manipulation_benchmark::run_benchmark_trial(
        &planar_bundle(),
        &trial,
        simulation_admission(100.0),
        None,
    )
    .expect_err("schema-v6 execution must reject a missing persisted key before simulation");
    assert!(error.to_string().contains("paired_trial_key"));
}

#[test]
fn paired_comparison_rejects_missing_or_mismatched_keys() {
    let baseline_trial = synthetic_trial(9073, 14, TrialExpectation::ExpectedSuccess);
    let candidate_trial = synthetic_trial(9074, 14, TrialExpectation::ExpectedSuccess);
    let baseline = BenchmarkTrialRecord::completed(
        baseline_trial.clone(),
        synthetic_episode(&baseline_trial, TrialOutcome::Success, None),
    );
    let candidate = BenchmarkTrialRecord::completed(
        candidate_trial.clone(),
        synthetic_episode(&candidate_trial, TrialOutcome::Success, None),
    );
    assert!(paired_campaign_delta(
        std::slice::from_ref(&baseline),
        std::slice::from_ref(&candidate)
    )
    .is_err());
    assert!(paired_campaign_delta(&[baseline.clone(), baseline.clone()], &[baseline]).is_err());
}

#[test]
fn paired_delta_bootstrap_is_deterministic() {
    let mut baseline = Vec::new();
    let mut candidate = Vec::new();
    for index in 0..6 {
        let trial = synthetic_trial(
            9075 + u64::from(index),
            index,
            TrialExpectation::ExpectedSuccess,
        );
        let baseline_outcome = if index < 2 {
            TrialOutcome::Success
        } else {
            TrialOutcome::UnexpectedRefusal
        };
        let candidate_outcome = if index < 4 {
            TrialOutcome::Success
        } else {
            TrialOutcome::UnexpectedRefusal
        };
        baseline.push(BenchmarkTrialRecord::completed(
            trial.clone(),
            synthetic_episode(&trial, baseline_outcome, None),
        ));
        candidate.push(BenchmarkTrialRecord::completed(
            trial.clone(),
            synthetic_episode(&trial, candidate_outcome, None),
        ));
    }
    let first = paired_campaign_delta(&baseline, &candidate).unwrap();
    let second = paired_campaign_delta(&baseline, &candidate).unwrap();
    assert_eq!(first, second);
    assert_eq!(first.bootstrap_resamples, 10_000);
    assert_eq!(
        first.embodiments["development:planar_arm"]
            .expected_success
            .baseline
            .denominator,
        6
    );
    assert_eq!(
        first.embodiments["development:planar_arm"]
            .expected_success
            .delta,
        Some(1.0 / 3.0)
    );
    let gate = evaluate_paired_screening_gate(&first);
    assert!(
        gate.passed,
        "a 1/3 gain with no measured regression passes the screening gate"
    );
}

#[test]
fn report_includes_wrench_inference_and_abort_metrics() {
    let trials = (0..5)
        .map(|index| {
            synthetic_trial(
                9180 + u64::from(index),
                index,
                TrialExpectation::ExpectedSuccess,
            )
        })
        .collect::<Vec<_>>();
    let mut informative = synthetic_episode(&trials[0], TrialOutcome::Success, None);
    informative.wrench_sample_count = 3;
    informative.wrench_sensor_profile_digest = Some("profile".into());
    informative.wrench_compensation_digest = Some("compensation".into());
    informative.wrench_inference_reason = Some("updated:SlipOnset".into());
    informative.wrench_inference_interval = Some([0.2, 0.7]);
    informative.prior_support_friction_interval = Some([0.1, 0.9]);
    informative.posterior_support_friction_interval = Some([0.2, 0.7]);
    informative.actions = vec![
        realityos_verify::manipulation_benchmark::ActionTrace {
            action_key: "probe:push_probe_replan".into(),
            family: "linear_stroke".into(),
            grant_id: "probe-grant".into(),
            witness_digest: "probe-witness".into(),
            observation_epoch: "probe-epoch".into(),
            writes_before: 0,
            writes_after: 1,
            displacement_m: 0.008,
            lateral_displacement_m: None,
            reauthorized_after_observation: false,
            runtime_divergence: false,
            control_tracking_correction_bound_rad: None,
            control_tracking_correction_gain: None,
            control_tracking_correction_max_rad: None,
        },
        realityos_verify::manipulation_benchmark::ActionTrace {
            action_key: "push:push_probe_replan".into(),
            family: "linear_stroke".into(),
            grant_id: "push-grant".into(),
            witness_digest: "push-witness".into(),
            observation_epoch: "post-probe-epoch".into(),
            writes_before: 1,
            writes_after: 2,
            displacement_m: 0.02,
            lateral_displacement_m: None,
            reauthorized_after_observation: true,
            runtime_divergence: false,
            control_tracking_correction_bound_rad: None,
            control_tracking_correction_gain: None,
            control_tracking_correction_max_rad: None,
        },
    ];
    let mut capped = synthetic_episode(&trials[1], TrialOutcome::ValidRefusal, None);
    capped.wrench_sample_count = 4;
    capped.wrench_abort = Some(
        realityos_verify::manipulation_benchmark::wrench_sensor::WrenchAbortEvent {
            reason: WrenchAbortReason::Limit {
                axis: 0,
                measured_upper_bound: 19.5,
                cap: 20.0,
            },
            triggering_sample: None,
            triggering_sensor_position: None,
            triggering_sensor_orientation: None,
            triggered_at_s: 1.0,
            stopped_at_s: 1.01,
            stop_write_count: 1,
            contract_id: "stop".into(),
            stop_verified: true,
            stop_actuator_ids: vec!["tool".into()],
            stop_error: None,
        },
    );
    let mut missing = synthetic_episode(&trials[2], TrialOutcome::ValidRefusal, None);
    missing.wrench_abort = Some(
        realityos_verify::manipulation_benchmark::wrench_sensor::WrenchAbortEvent {
            reason: WrenchAbortReason::MissingSample,
            triggering_sample: None,
            triggering_sensor_position: None,
            triggering_sensor_orientation: None,
            triggered_at_s: 1.0,
            stopped_at_s: 1.01,
            stop_write_count: 1,
            contract_id: "stop".into(),
            stop_verified: true,
            stop_actuator_ids: vec!["tool".into()],
            stop_error: None,
        },
    );
    let mut unexpected_contact = synthetic_episode(&trials[3], TrialOutcome::ValidRefusal, None);
    unexpected_contact.wrench_abort = Some(
        realityos_verify::manipulation_benchmark::wrench_sensor::WrenchAbortEvent {
            reason: WrenchAbortReason::UnexpectedContact,
            triggering_sample: None,
            triggering_sensor_position: None,
            triggering_sensor_orientation: None,
            triggered_at_s: 1.0,
            stopped_at_s: 1.01,
            stop_write_count: 1,
            contract_id: "stop".into(),
            stop_verified: true,
            stop_actuator_ids: vec!["tool".into()],
            stop_error: None,
        },
    );
    let mut actuator_saturation = synthetic_episode(&trials[4], TrialOutcome::ValidRefusal, None);
    actuator_saturation.wrench_abort = Some(
        realityos_verify::manipulation_benchmark::wrench_sensor::WrenchAbortEvent {
            reason: WrenchAbortReason::ActuatorSaturation {
                actuator_ids: vec!["joint2_actuator".into()],
            },
            triggering_sample: None,
            triggering_sensor_position: None,
            triggering_sensor_orientation: None,
            triggered_at_s: 1.0,
            stopped_at_s: 1.01,
            stop_write_count: 1,
            contract_id: "stop".into(),
            stop_verified: true,
            stop_actuator_ids: vec!["tool".into()],
            stop_error: None,
        },
    );
    let records = vec![
        BenchmarkTrialRecord::completed(trials[0].clone(), informative),
        BenchmarkTrialRecord::completed(trials[1].clone(), capped),
        BenchmarkTrialRecord::completed(trials[2].clone(), missing),
        BenchmarkTrialRecord::completed(trials[3].clone(), unexpected_contact),
        BenchmarkTrialRecord::completed(trials[4].clone(), actuator_saturation),
    ];
    let config = config_for_trial_ids(trials.iter().map(|trial| trial.trial_id.clone()).collect());
    let report = score_campaign(&config, &records).expect("wrench report scores");
    assert_eq!(report.wrench_metrics.push_trials, 5);
    assert_eq!(report.wrench_metrics.episodes_with_samples, 2);
    assert_eq!(report.wrench_metrics.sample_count, 7);
    assert_eq!(report.wrench_metrics.strict_friction_narrowings, 1);
    assert_eq!(report.wrench_metrics.inference_rate.numerator, 1);
    assert_eq!(report.wrench_metrics.inference_rate.denominator, 2);
    assert_eq!(report.wrench_metrics.force_cap_trips, 1);
    assert_eq!(report.wrench_metrics.stale_or_missing_sample_aborts, 1);
    assert_eq!(report.wrench_metrics.unexpected_contact_aborts, 1);
    assert_eq!(report.wrench_metrics.actuator_saturation_aborts, 1);
    assert!(
        realityos_verify::manipulation_benchmark::render_markdown(&report)
            .contains("## Force/torque evidence")
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

    assert!((0.40..0.55).contains(&x));
    assert!((-0.10..0.10).contains(&y));
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
    let probe_capable_ambiguous = resolve_benchmark_trial(
        7,
        12,
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
    assert_eq!(
        probe_capable_ambiguous.scenario.polarity,
        realityos_verify::manipulation_scenarios::Polarity::Ambiguous
    );
    assert!(probe_capable_ambiguous.scenario.expected_refusal.is_none());
    assert_eq!(
        probe_capable_ambiguous.expectation,
        TrialExpectation::ExpectedSuccess,
        "ambiguous scenarios with no explicit refusal permit a safe probe"
    );
}

#[test]
fn positive_suite_trials_are_not_downgraded_when_feasibility_fails() {
    let mut suite = scenario_suite_v2();
    suite.positive_push.object_x_m = [50.0, 51.0];
    suite.max_positive_resamples = 2;

    let result = resolve_benchmark_trial_with_suite(
        suite.pilot_seeds[0],
        0,
        BenchmarkTask::PushProbeReplan,
        EmbodimentRole::Development,
        &planar_bundle(),
        &planar_manifest(),
        &suite,
    );

    assert!(
        result.is_err(),
        "an infeasible expected-success case must fail resolution"
    );
}

#[test]
fn monitored_worker_interpolates_control_ramps_at_each_physics_step() {
    use realityos_verify::mujoco_exec::{ensure_mujoco_or_skip, MujocoInstance};
    if !ensure_mujoco_or_skip() {
        return;
    }
    let _guard = lock_mujoco_test();
    let bundle = planar_bundle();
    let mut worker = MujocoInstance::spawn().expect("MuJoCo worker");
    let load = worker
        .load_bundle_with_wrench_profile(&bundle, &[], 946, &simulation_wrench_profile("ee"))
        .expect("load control-ramp fixture");
    let mut profile = monitored_profile_for_fixture("ee", &load["inspect"], None);
    profile.limit_policy.scope.caps = [100.0, 100.0, 100.0, 10.0, 10.0, 10.0];
    profile.limit_policy.scope.policy_digest = profile.limit_policy.digest();
    assert!(profile.validate().is_ok());

    let start = load["state"]["ctrl"]
        .as_array()
        .expect("loaded actuator targets")
        .iter()
        .map(|value| value.as_f64().expect("finite control"))
        .collect::<Vec<_>>();
    let actuator = load["inspect"]["actuators"]
        .as_array()
        .unwrap()
        .iter()
        .position(|actuator| {
            matches!(actuator["trntype"].as_i64(), Some(0 | 1))
                && actuator["biastype"].as_i64() == Some(1)
                && actuator["ctrllimited"].as_bool() == Some(true)
        })
        .expect("position actuator");
    let mut target = start.clone();
    target[actuator] += 0.01;
    let mut request = monitored_worker_request(&profile, false);
    request["n"] = serde_json::json!(4);
    request["control_ramp"] = serde_json::json!({
        "start_ctrl": start,
        "target_ctrl": target.clone(),
        "elapsed_steps": 0,
        "total_steps": 8
    });
    let response = worker.rpc(&request).expect("run monitored control ramp");
    assert_eq!(response["ok"], true, "{response}");
    assert_eq!(response["abort"], Value::Null);
    let final_ctrl = response["state"]["ctrl"]
        .as_array()
        .expect("final ramp control");
    let expected = start[actuator] + (target[actuator] - start[actuator]) * 0.5;
    assert!(
        (final_ctrl[actuator].as_f64().unwrap() - expected).abs()
            <= profile.limit_policy.max_position_tracking_correction_rad + 1e-10,
        "bounded feedback may adjust the smoothstep midpoint only within its registered correction"
    );
    let correction = response["control_correction_max_rad"]
        .as_f64()
        .expect("audited position-tracking correction");
    assert!(correction > 0.0);
    assert!(correction <= profile.limit_policy.max_position_tracking_correction_rad + 1e-10);
    assert_eq!(response["state"]["ctrl_write_count"], 4);
}

#[test]
fn iiwa_monitored_feedback_correction_reaches_the_authorized_joint_target() {
    use realityos_verify::mujoco_exec::{ensure_mujoco_or_skip, MujocoInstance};
    if !ensure_mujoco_or_skip() {
        return;
    }
    let _guard = lock_mujoco_test();
    let bundle = iiwa14_bundle();
    let mut worker = MujocoInstance::spawn().expect("MuJoCo worker");
    let load = worker
        .load_bundle_with_wrench_profile(
            &bundle,
            &[],
            947,
            &simulation_wrench_profile("attachment_site"),
        )
        .expect("load IIWA feedback fixture");
    let mut profile = monitored_profile_for_fixture("attachment_site", &load["inspect"], None);
    profile.limit_policy.scope.caps = [200.0, 200.0, 200.0, 20.0, 20.0, 20.0];
    profile.limit_policy.scope.policy_digest = profile.limit_policy.digest();
    assert!(profile.validate().is_ok());

    let mut settle = monitored_worker_request(&profile, false);
    settle["n"] = serde_json::json!(1_000);
    settle["request_monitor_gravity_reference"] = serde_json::json!(true);
    let settled = worker
        .rpc(&settle)
        .expect("settle the loaded arm under monitoring");
    assert_eq!(settled["ok"], true, "{settled}");

    let start = settled["state"]["ctrl"]
        .as_array()
        .expect("settled control targets")
        .iter()
        .map(|value| value.as_f64().expect("finite control"))
        .collect::<Vec<_>>();
    let actuator_id = load["inspect"]["actuators"]
        .as_array()
        .unwrap()
        .iter()
        .position(|actuator| actuator["name"] == "actuator2")
        .expect("IIWA joint2 position actuator");
    let qpos_address = load["inspect"]["joints"]
        .as_array()
        .unwrap()
        .iter()
        .find(|joint| joint["name"] == "joint2")
        .expect("IIWA joint2")["qposadr"]
        .as_u64()
        .unwrap() as usize;
    let mut target = start.clone();
    target[actuator_id] = 0.861;
    let mut ramp = monitored_worker_request(&profile, false);
    ramp["n"] = serde_json::json!(6_000);
    ramp["control_ramp"] = serde_json::json!({
        "start_ctrl": start,
        "target_ctrl": target.clone(),
        "elapsed_steps": 0,
        "total_steps": 6_000,
    });
    let response = worker
        .rpc(&ramp)
        .expect("track IIWA target under the wrench monitor");
    assert_eq!(response["ok"], true, "{response}");
    assert_eq!(response["abort"], Value::Null);
    let actual = response["state"]["qpos"][qpos_address]
        .as_f64()
        .expect("measured joint position");
    let correction = response["control_correction_max_rad"]
        .as_f64()
        .expect("audited correction");
    assert!(correction > 0.0 && correction <= 0.04 + 1e-10);
    assert!((actual - target[actuator_id]).abs() <= 0.02);
}

#[test]
fn iiwa_public_suite_keeps_spatial_table_geometry() {
    let _mujoco_guard = lock_mujoco_test();
    let bundle = iiwa14_bundle();
    let suite = scenario_suite_v2();
    let (worker, manifest) = load_and_normalize(&bundle, &[], suite.pilot_seeds[0])
        .expect("strict benchmark requires MuJoCo and complete model inspection");
    checkin_worker(worker);
    let trial = resolve_benchmark_trial_with_suite(
        suite.pilot_seeds[0],
        3,
        BenchmarkTask::PushProbeReplan,
        EmbodimentRole::Development,
        &bundle,
        &manifest,
        &suite,
    )
    .expect("refusal trial resolves without a positive motion search");
    let table = object_named(&trial.scenario.objects, "table");
    assert_eq!(table["pos"], serde_json::json!([0.45, 0.0, 0.40]));
    assert_eq!(table["size"], serde_json::json!([0.25, 0.25, 0.02]));
}

#[test]
fn positive_suite_trials_resolve_for_both_development_embodiments() {
    let _mujoco_guard = lock_mujoco_test();
    let suite = scenario_suite_v2();
    for bundle in [ur5e_bundle(), iiwa14_bundle()] {
        let (worker, manifest) = load_and_normalize(&bundle, &[], suite.pilot_seeds[0])
            .expect("strict benchmark requires MuJoCo and complete model inspection");
        checkin_worker(worker);
        if bundle.manifest.robot_id == "menagerie_iiwa14" {
            let model = embodiment_from_manifest(&bundle, &manifest);
            let ee = model
                .end_effectors
                .iter()
                .find(|ee| ee.name == "tool0")
                .expect("IIWA tool frame");
            let frame = model
                .frames
                .iter()
                .find(|frame| frame.name == ee.frame)
                .expect("IIWA tool frame geometry");
            assert_eq!(
                frame.parent_body, "link7",
                "plan the pusher from the collision sphere center; the wrench sensor keeps using its offset attachment site"
            );
            assert!(manifest
                .site_records
                .iter()
                .any(|site| site.name == "attachment_site" && site.body == "link7"));
        }
        let trial = resolve_benchmark_trial_with_suite(
            suite.pilot_seeds[0],
            0,
            BenchmarkTask::PushProbeReplan,
            EmbodimentRole::Development,
            &bundle,
            &manifest,
            &suite,
        )
        .unwrap_or_else(|error| {
            panic!(
                "{} positive suite trial must resolve: {error}",
                bundle.manifest.robot_id
            )
        });
        assert_eq!(trial.expectation, TrialExpectation::ExpectedSuccess);
        assert_eq!(trial.stratum, Some(PushTrialStratum::ExpectedSuccess));
        assert_eq!(
            trial.scenario_suite_id.as_deref(),
            Some("feasible_public_probe")
        );
        assert_eq!(trial.scenario_suite_version, Some(2));
        assert!(trial.scenario.expected_refusal.is_none());
        assert!(trial.paired_trial_key.is_some());
        let table = object_named(&trial.scenario.objects, "table");
        assert_eq!(table["pos"], serde_json::json!([0.90, 0.0, 0.40]));
        assert_eq!(table["size"], serde_json::json!([0.40, 0.25, 0.02]));
        let target = object_named(&trial.scenario.objects, &trial.scenario.object_id);
        let half_height = target["size"][2]
            .as_f64()
            .expect("positive target half-height");
        assert!(
            (0.07..=0.09).contains(&half_height),
            "the positive probe target needs a randomized vertical contact band that preserves support load"
        );
        assert!(
            trial
                .initial_qpos
                .iter()
                .zip(&manifest.qpos0)
                .any(|(start, reference)| (start - reference).abs() > 0.02),
            "positive benchmark starts must be randomized from the inspected reference configuration"
        );
        for actuator in &manifest.actuators {
            let joint = manifest
                .joints
                .iter()
                .find(|joint| joint.name == actuator.transmission_target)
                .expect("every registered arm actuator targets an inspected joint");
            let qpos_index = usize::try_from(joint.qpos_address).expect("nonnegative qpos index");
            let value = trial.initial_qpos[qpos_index];
            assert!(
                value > actuator.ctrlrange[0] + 0.1 && value < actuator.ctrlrange[1] - 0.1,
                "{} initial target {} must remain 0.1 rad inside controller range [{}, {}]",
                actuator.name,
                value,
                actuator.ctrlrange[0],
                actuator.ctrlrange[1]
            );
        }
    }
}

#[test]
fn iiwa_expected_success_probe_tracks_retract_under_wrench_monitor() {
    let _mujoco_guard = lock_mujoco_test();
    let debug_phases = std::env::var_os("REALITYOS_DEBUG_BENCHMARK_PHASES").is_some();
    let phase_started = std::time::Instant::now();
    if debug_phases {
        eprintln!("phase: loading IIWA model and suite");
    }
    let bundle = iiwa14_bundle();
    let suite = scenario_suite_v3();
    let (worker, manifest) = load_and_normalize(&bundle, &[], suite.pilot_seeds[0])
        .expect("strict benchmark requires MuJoCo and complete model inspection");
    if debug_phases {
        eprintln!(
            "phase: model ready after {:.1}s",
            phase_started.elapsed().as_secs_f64()
        );
    }
    checkin_worker(worker);
    let resolve_started = std::time::Instant::now();
    realityos_semantics::work_counters::reset_work_counters();
    let trial = resolve_benchmark_trial_with_suite(
        suite.pilot_seeds[0],
        0,
        BenchmarkTask::PushProbeReplan,
        EmbodimentRole::Development,
        &bundle,
        &manifest,
        &suite,
    )
    .expect("positive IIWA suite trial");
    if debug_phases {
        eprintln!(
            "phase: trial resolved after {:.1}s; geometry work={:?}",
            resolve_started.elapsed().as_secs_f64(),
            realityos_semantics::work_counters::work_snapshot()
        );
    }
    let episode_started = std::time::Instant::now();
    let episode = run_test_benchmark_trial(&bundle, &trial, simulation_admission(120.0))
        .expect("strict trial records force and contact evidence");
    if debug_phases {
        eprintln!(
            "phase: episode complete after {:.1}s",
            episode_started.elapsed().as_secs_f64()
        );
    }
    assert_eq!(trial.expectation, TrialExpectation::ExpectedSuccess);
    assert_eq!(
        episode.outcome,
        TrialOutcome::Success,
        "the expected-success probe must complete its authorized approach, contact, and retract: scenario={:?}; cause={:?}; refusal={:?}; actions={:?}; inference={:?}; inference_interval={:?}; posterior={:?}; decision_audits={:?}; monitor_reference_status={:?}; monitor_reference_history={:?}; baseline={:?}; contacts={:?}; abort={:?}",
        trial.scenario,
        episode.failure_cause,
        episode.refusal_code,
        episode.actions,
        episode.wrench_inference_reason,
        episode.wrench_inference_interval,
        episode.posterior_support_friction_interval,
        episode.decision_audits,
        episode.wrench_monitor_gravity_reference_status,
        episode.wrench_monitor_gravity_reference_history,
        episode.wrench_precontact_baseline,
        episode.wrench_contact_pairs,
        episode.wrench_abort
    );
    assert!(
        episode
            .wrench_inference_reason
            .as_deref()
            .is_some_and(|reason| reason.starts_with("updated:")),
        "a successful force-aware probe must produce a validated friction update: {:?}",
        episode.wrench_inference_reason
    );
    assert!(episode.wrench_inference_interval.is_some());
    assert_eq!(episode.actions[0].action_key, "probe:+x");
    assert_eq!(
        episode.wrench_monitor_gravity_reference_status.as_deref(),
        Some("captured"),
        "probe monitoring must capture its contact-free gravity reference"
    );
    assert_eq!(
        episode.wrench_precontact_baseline_status.as_deref(),
        Some("captured"),
        "a positive probe must reach its stationary, contact-free inference baseline"
    );
    assert!(
        episode.wrench_monitor_gravity_reference_history.len() >= 2,
        "the force monitor must refresh its gravity reference at the settled pre-contact pose"
    );
    let inference_baseline_digest = episode
        .wrench_precontact_baseline
        .as_ref()
        .map(|sample| sample.digest.as_str());
    assert!(
        episode
            .wrench_monitor_gravity_reference_history
            .iter()
            .any(|reference| Some(reference.sample.digest.as_str()) == inference_baseline_digest),
        "the probe's inference baseline must remain linked to a recorded monitor reference"
    );
    assert!(episode.wrench_sample_count > 0);
    assert_eq!(episode.unauthorized_writes, 0);
    assert!(!episode.wrench_contact_pairs.is_empty());
    if let Some(abort) = episode.wrench_abort {
        assert!(
            abort.stop_verified,
            "force trips need a verified stop: {abort:?}"
        );
        if matches!(abort.reason, WrenchAbortReason::Limit { .. }) {
            assert!(
                abort.triggering_sample.is_some(),
                "force-limit report must retain the actual triggering sensor sample"
            );
            assert!(abort.triggering_sensor_position.is_some());
            assert!(abort.triggering_sensor_orientation.is_some());
        }
    }
}

#[test]
fn ur5e_expected_success_probe_tracks_retract_under_wrench_monitor() {
    let _mujoco_guard = lock_mujoco_test();
    let (bundle, trial) = live_ur5e_v3_positive_trial();

    let episode = run_test_benchmark_trial(&bundle, &trial, simulation_admission(120.0))
        .expect("strict UR5e trial records force and contact evidence");

    assert_eq!(trial.expectation, TrialExpectation::ExpectedSuccess);
    assert_eq!(
        episode.outcome,
        TrialOutcome::Success,
        "the expected-success UR5e probe must complete its authorized approach, contact, and retract: scenario={:?}; cause={:?}; refusal={:?}; actions={:?}; inference={:?}; interval={:?}; posterior={:?}; sample_count={}; wrench_peak={:?}; contacts={:?}; monitor_reference_status={:?}; reference_history={:?}; baseline_status={:?}; baseline={:?}; decisions={:?}; abort={:?}",
        trial.scenario,
        episode.failure_cause,
        episode.refusal_code,
        episode.actions,
        episode.wrench_inference_reason,
        episode.wrench_inference_interval,
        episode.posterior_support_friction_interval,
        episode.wrench_sample_count,
        episode.wrench_peak_conservative_bounds,
        episode.wrench_contact_pairs,
        episode.wrench_monitor_gravity_reference_status,
        episode.wrench_monitor_gravity_reference_history,
        episode.wrench_precontact_baseline_status,
        episode.wrench_precontact_baseline,
        episode.decision_audits,
        episode.wrench_abort
    );
    assert!(episode
        .wrench_inference_reason
        .as_deref()
        .is_some_and(|reason| reason.starts_with("updated:")));
    assert_eq!(episode.actions[0].action_key, "probe:+x");
    assert_eq!(
        episode.wrench_monitor_gravity_reference_status.as_deref(),
        Some("captured")
    );
    assert_eq!(
        episode.wrench_precontact_baseline_status.as_deref(),
        Some("captured")
    );
    assert!(episode.wrench_precontact_baseline.is_some());
    assert!(episode.wrench_sample_count > 0);
    assert!(!episode.wrench_contact_pairs.is_empty());
    assert_eq!(episode.unauthorized_writes, 0);
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
