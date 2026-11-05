use realityos_semantics::observation::{wrench_sample_digest, WrenchSample};
use realityos_semantics::physical_belief::{
    BeliefEpistemicStatus, BeliefEvidenceReference, PhysicalParameter, PhysicalParameterBelief,
};
use realityos_semantics::push_probe::{
    apply_first_onset_outside_prior, apply_successive_probe_cadence,
    friction_interval_proves_mu_required, infer_support_friction,
    intersect_measurement_bounds_with_prior, prior_envelope_miss_margin,
    transform_sensor_wrench_to_support, FirstOnsetOutsidePriorOutcome,
    FirstOnsetOutsidePriorPolicy, FrictionInferenceMode, FrictionInferenceResult,
    FrictionInferredQuantity, FrictionNoInferenceReason, IntendedContactEvidence,
    PriorEnvelopeMissMargin, ProbeContactPair, ProbeMotionWindow, ProbeObjectPoseSample,
    ProbeObjectPoseTrace, PushProbeMeasurement, SuccessiveProbeCadenceOutcome,
    SuccessiveProbeCadencePolicy, WrenchCompensation, WrenchEvidenceOrigin, WrenchTransform,
};
use realityos_semantics::sensor::WrenchSampleOrigin;

fn sample(sequence: u64, capture_s: f64, wrench: [f64; 6]) -> WrenchSample {
    let mut sample = WrenchSample {
        sensor_id: "ft:tool".into(),
        sensor_frame: "tool0".into(),
        transform_epoch: "epoch:probe-1".into(),
        units: "N,N*m".into(),
        wrench,
        error_bounds: [0.05; 6],
        capture_s,
        receive_s: capture_s + 0.0001,
        sequence,
        clock_domain: "sim:1".into(),
        origin: WrenchSampleOrigin::Simulation,
        digest: String::new(),
    };
    sample.digest = wrench_sample_digest(&sample);
    sample
}

fn compensation() -> WrenchCompensation {
    let mut compensation = WrenchCompensation {
        bias: [0.0; 6],
        gravity_wrench: [0.0; 6],
        residual_error: [0.01; 6],
        digest: String::new(),
    };
    compensation.digest = compensation.digest();
    compensation
}

fn measurement() -> PushProbeMeasurement {
    let epoch = "epoch:probe-1".to_string();
    let collision_digest = "b".repeat(64);
    PushProbeMeasurement {
        samples: vec![
            sample(1, 1.002, [-3.1, 0.0, 0.0, 0.0, 0.0, 0.0]),
            sample(2, 1.008, [-3.1, 0.0, 0.0, 0.0, 0.0, 0.0]),
        ],
        precontact_baseline: Some(sample(0, 1.001, [0.0; 6])),
        sample_free_space_wrenches: Vec::new(),
        sample_free_space_wrench_error_bounds: Vec::new(),
        sample_sensor_transforms: Vec::new(),
        sensor_to_support: WrenchTransform {
            rotation_to_support: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            translation_to_support_m: [0.0; 3],
        },
        compensation: compensation(),
        mass_kg: [1.0, 1.0],
        support_normal_world: [0.0, 0.0, 1.0],
        motion: ProbeMotionWindow {
            start_s: 1.0,
            end_s: 1.01,
            displacement_m: [0.0; 3],
            max_speed_m_s: 0.0005,
            max_quasi_static_speed_m_s: 0.01,
            pose_resolution_m: 0.001,
            first_motion_s: None,
            unresolved_motion_before_force: false,
        },
        motion_trace: None,
        prior_support_friction: [0.1, 0.8],
        current_observation_epoch: epoch.clone(),
        current_collision_model_digest: collision_digest.clone(),
        now_s: 1.02,
        max_sample_age_s: 0.05,
        max_inter_sample_gap_s: 0.01,
        max_sample_motion_skew_s: 0.01,
        max_wrench_rise_rate: [1.0; 6],
        intended_contact: IntendedContactEvidence {
            observation_epoch: epoch,
            collision_model_digest: collision_digest,
            tool_body_id: "tool".into(),
            target_body_id: "object".into(),
            plausible_contact_sets: vec![vec!["tool".into(), "object".into()]],
        },
        runtime_contact_ambiguous: false,
        saturated: false,
        origin: WrenchEvidenceOrigin::Simulation,
    }
}

fn sealed_pose_sample(
    step: u64,
    wrench_sequence: Option<u64>,
    capture_s: f64,
    position_world_m: [f64; 3],
) -> ProbeObjectPoseSample {
    let mut sample = ProbeObjectPoseSample {
        object_id: "object".into(),
        simulator_step: step,
        wrench_sequence,
        contact_pairs: None,
        contact_force_samples: None,
        capture_s,
        clock_domain: "sim:1".into(),
        position_world_m,
        orientation_world_wxyz: [1.0, 0.0, 0.0, 0.0],
        digest: String::new(),
    };
    sample.digest = sample.digest();
    sample
}

fn sealed_motion_trace() -> ProbeObjectPoseTrace {
    let mut trace = ProbeObjectPoseTrace {
        object_id: "object".into(),
        clock_domain: "sim:1".into(),
        start_s: 1.0,
        start_position_world_m: [0.0; 3],
        start_orientation_world_wxyz: [1.0, 0.0, 0.0, 0.0],
        start_wrench_sequence: None,
        start_contact_pairs: None,
        samples: vec![
            sealed_pose_sample(10, Some(1), 1.002, [0.0; 3]),
            sealed_pose_sample(11, Some(2), 1.008, [0.0; 3]),
            sealed_pose_sample(12, None, 1.01, [0.0; 3]),
        ],
        digest: String::new(),
    };
    trace.digest = trace.digest();
    trace
}

#[test]
fn pose_trace_brackets_cumulative_motion_from_the_original_probe_origin() {
    let mut first = ProbeObjectPoseSample {
        object_id: "object".into(),
        simulator_step: 41,
        wrench_sequence: Some(8),
        capture_s: 2.001,
        clock_domain: "mujoco:simulation".into(),
        position_world_m: [0.0006, 0.0, 0.0],
        orientation_world_wxyz: [1.0, 0.0, 0.0, 0.0],
        contact_pairs: None,
        contact_force_samples: None,
        digest: String::new(),
    };
    first.digest = first.digest();
    let mut crossing = ProbeObjectPoseSample {
        object_id: "object".into(),
        simulator_step: 42,
        wrench_sequence: Some(9),
        capture_s: 2.002,
        clock_domain: "mujoco:simulation".into(),
        position_world_m: [0.0011, 0.0, 0.0],
        orientation_world_wxyz: [1.0, 0.0, 0.0, 0.0],
        contact_pairs: None,
        contact_force_samples: None,
        digest: String::new(),
    };
    crossing.digest = crossing.digest();
    let trace = ProbeObjectPoseTrace {
        object_id: "object".into(),
        clock_domain: "mujoco:simulation".into(),
        start_s: 2.0,
        start_position_world_m: [0.0, 0.0, 0.0],
        start_orientation_world_wxyz: [1.0, 0.0, 0.0, 0.0],
        start_wrench_sequence: None,
        start_contact_pairs: None,
        samples: vec![first, crossing],
        digest: String::new(),
    };

    assert_eq!(
        trace.first_xy_motion_bracket(0.001),
        Some([2.001, 2.002]),
        "the 0.6 mm sub-threshold step must accumulate against the original origin"
    );
}

#[test]
fn probe_inference_rejects_pose_trace_wrench_pairs_without_matching_sequence_or_time() {
    let mut measured = measurement();
    let mut trace = sealed_motion_trace();
    trace.samples[2].wrench_sequence = Some(99);
    trace.samples[2].digest = trace.samples[2].digest();
    trace.digest = trace.digest();
    measured.motion_trace = Some(trace);

    assert_eq!(
        infer_support_friction(&measured),
        FrictionInferenceResult::NoInference(FrictionNoInferenceReason::InvalidMotionTrace)
    );

    let mut time_mismatch = measurement();
    time_mismatch.precontact_baseline = Some(sample(0, 0.995, [0.0; 6]));
    time_mismatch.samples = vec![
        sample(1, 1.0, [-3.1, 0.0, 0.0, 0.0, 0.0, 0.0]),
        sample(2, 1.008, [-3.1, 0.0, 0.0, 0.0, 0.0, 0.0]),
    ];
    time_mismatch.motion.start_s = 1.0;
    time_mismatch.motion.end_s = 1.01;
    time_mismatch.now_s = 1.02;
    let mut trace = ProbeObjectPoseTrace {
        object_id: "object".into(),
        clock_domain: "sim:1".into(),
        start_s: 1.0,
        start_position_world_m: [0.0; 3],
        start_orientation_world_wxyz: [1.0, 0.0, 0.0, 0.0],
        start_wrench_sequence: Some(1),
        start_contact_pairs: None,
        samples: vec![
            sealed_pose_sample(11, Some(2), 1.008, [0.0; 3]),
            sealed_pose_sample(12, None, 1.01, [0.0; 3]),
        ],
        digest: String::new(),
    };
    trace.digest = trace.digest();
    time_mismatch.samples[0].capture_s = 1.001;
    time_mismatch.samples[0].receive_s = 1.0011;
    time_mismatch.samples[0].digest = wrench_sample_digest(&time_mismatch.samples[0]);
    time_mismatch.motion_trace = Some(trace);

    assert_eq!(
        infer_support_friction(&time_mismatch),
        FrictionInferenceResult::NoInference(FrictionNoInferenceReason::InvalidMotionTrace)
    );
}

#[test]
fn pose_trace_accepts_a_wrench_paired_at_the_start_boundary() {
    let mut measured = measurement();
    measured.precontact_baseline = Some(sample(0, 0.995, [0.0; 6]));
    measured.samples = vec![
        sample(1, 1.0, [-3.1, 0.0, 0.0, 0.0, 0.0, 0.0]),
        sample(2, 1.008, [-3.1, 0.0, 0.0, 0.0, 0.0, 0.0]),
    ];
    measured.motion.start_s = 1.0;
    measured.motion.end_s = 1.01;
    measured.now_s = 1.02;
    let mut trace = ProbeObjectPoseTrace {
        object_id: "object".into(),
        clock_domain: "sim:1".into(),
        start_s: 1.0,
        start_position_world_m: [0.0; 3],
        start_orientation_world_wxyz: [1.0, 0.0, 0.0, 0.0],
        start_wrench_sequence: None,
        start_contact_pairs: None,
        samples: vec![
            sealed_pose_sample(10, Some(1), 1.0, [0.0; 3]),
            sealed_pose_sample(11, Some(2), 1.008, [0.0; 3]),
            sealed_pose_sample(12, None, 1.01, [0.0; 3]),
        ],
        digest: String::new(),
    };
    trace.digest = trace.digest();
    measured.motion_trace = Some(trace);

    let inference = infer_support_friction(&measured);
    assert!(
        matches!(inference, FrictionInferenceResult::Updated { .. }),
        "a time-aligned boundary pose should keep inference valid: {inference:?}"
    );
}

#[test]
fn pose_trace_rejects_a_boundary_pose_that_disagrees_with_its_origin() {
    let mut trace = sealed_motion_trace();
    trace.samples[0].capture_s = trace.start_s;
    trace.samples[0].position_world_m = [0.0001, 0.0, 0.0];
    trace.samples[0].digest = trace.samples[0].digest();
    trace.digest = trace.digest();

    assert_eq!(trace.validate(), Err("start_pose_mismatch"));
}

#[test]
fn pose_trace_pairs_a_bootstrap_wrench_with_its_sealed_start_pose() {
    let mut measured = measurement();
    measured.precontact_baseline = Some(sample(0, 0.995, [0.0; 6]));
    measured.samples = vec![
        sample(1, 1.0, [-3.1, 0.0, 0.0, 0.0, 0.0, 0.0]),
        sample(2, 1.008, [-3.1, 0.0, 0.0, 0.0, 0.0, 0.0]),
    ];
    measured.motion.start_s = 1.0;
    measured.motion.end_s = 1.01;
    measured.now_s = 1.02;
    let mut trace = ProbeObjectPoseTrace {
        object_id: "object".into(),
        clock_domain: "sim:1".into(),
        start_s: 1.0,
        start_position_world_m: [0.0; 3],
        start_orientation_world_wxyz: [1.0, 0.0, 0.0, 0.0],
        start_wrench_sequence: Some(1),
        start_contact_pairs: None,
        samples: vec![
            sealed_pose_sample(11, Some(2), 1.008, [0.0; 3]),
            sealed_pose_sample(12, None, 1.01, [0.0; 3]),
        ],
        digest: String::new(),
    };
    trace.digest = trace.digest();
    measured.motion_trace = Some(trace);

    let inference = infer_support_friction(&measured);
    assert!(
        matches!(inference, FrictionInferenceResult::Updated { .. }),
        "the start pose must be explicitly pairable with a bootstrap wrench: {inference:?}"
    );
}

#[test]
fn bootstrap_wrench_sequence_is_sealed_into_trace_digest() {
    let mut measured = measurement();
    measured.precontact_baseline = Some(sample(0, 0.995, [0.0; 6]));
    measured.samples = vec![
        sample(1, 1.0, [-3.1, 0.0, 0.0, 0.0, 0.0, 0.0]),
        sample(2, 1.008, [-3.1, 0.0, 0.0, 0.0, 0.0, 0.0]),
    ];
    measured.motion.start_s = 1.0;
    measured.motion.end_s = 1.01;
    measured.now_s = 1.02;
    let mut trace = ProbeObjectPoseTrace {
        object_id: "object".into(),
        clock_domain: "sim:1".into(),
        start_s: 1.0,
        start_position_world_m: [0.0; 3],
        start_orientation_world_wxyz: [1.0, 0.0, 0.0, 0.0],
        start_wrench_sequence: Some(1),
        start_contact_pairs: None,
        samples: vec![
            sealed_pose_sample(11, Some(2), 1.008, [0.0; 3]),
            sealed_pose_sample(12, None, 1.01, [0.0; 3]),
        ],
        digest: String::new(),
    };
    trace.digest = trace.digest();
    let v2_digest = trace.digest.clone();
    trace.start_wrench_sequence = Some(99);
    trace.digest = trace.digest();
    assert_ne!(
        trace.digest, v2_digest,
        "the bootstrap sequence is digest-sealed"
    );
    measured.motion_trace = Some(trace);

    assert_eq!(
        infer_support_friction(&measured),
        FrictionInferenceResult::NoInference(FrictionNoInferenceReason::InvalidMotionTrace)
    );
}

#[test]
fn valid_pose_trace_is_bound_into_the_measurement_digest() {
    let mut measured = measurement();
    let legacy_digest = measured.digest();
    measured.motion_trace = Some(sealed_motion_trace());
    let traced_digest = measured.digest();

    assert_ne!(traced_digest, legacy_digest);
    assert!(matches!(
        infer_support_friction(&measured),
        FrictionInferenceResult::Updated { .. }
    ));

    let trace = measured.motion_trace.as_mut().expect("pose trace");
    trace.samples[0].position_world_m[0] = 0.00001;
    trace.samples[0].digest = trace.samples[0].digest();
    trace.digest = trace.digest();
    assert_ne!(measured.digest(), traced_digest);
}

#[test]
fn sticking_probe_adds_only_a_conservative_lower_bound() {
    let result = infer_support_friction(&measurement());
    let FrictionInferenceResult::Updated {
        interval,
        mode,
        inferred_quantity,
    } = result
    else {
        panic!("sticking wrench and motion evidence should provide a lower bound: {result:?}");
    };
    assert_eq!(mode, FrictionInferenceMode::Sticking);
    assert_eq!(
        inferred_quantity,
        FrictionInferredQuantity::OnsetOrStickingForceRatio
    );
    assert!(interval[0] > 0.30 && interval[0] < 0.32, "{interval:?}");
    assert_eq!(
        interval[1], 0.8,
        "sticking evidence provides no upper bound"
    );
}

#[test]
fn precontact_gravity_baseline_is_removed_from_each_inference_sample() {
    let mut measured = measurement();
    measured.samples[0].wrench[0] = -2.1;
    measured.samples[1].wrench[0] = -2.1;
    for sample in &mut measured.samples {
        sample.digest = wrench_sample_digest(sample);
    }
    measured.sample_free_space_wrenches = vec![[1.0, 0.0, 0.0, 0.0, 0.0, 0.0]; 2];
    measured.sample_free_space_wrench_error_bounds = vec![[0.0; 6]; 2];

    let FrictionInferenceResult::Updated { interval, .. } = infer_support_friction(&measured)
    else {
        panic!(
            "gravity-compensated sticking probe should infer a lower bound: {:?}",
            infer_support_friction(&measured)
        );
    };
    assert!(
        interval[0] > 0.30,
        "gravity baseline must be removed: {interval:?}"
    );
}

#[test]
fn probe_refuses_friction_updates_without_a_precontact_baseline() {
    let mut measured = measurement();
    measured.precontact_baseline = None;
    assert!(matches!(
        infer_support_friction(&measured),
        FrictionInferenceResult::NoInference(FrictionNoInferenceReason::PrecontactBaselineMissing)
    ));
}

#[test]
fn per_sample_sensor_pose_transforms_the_wrench_into_support_axes() {
    let mut measured = measurement();
    measured.samples[1].wrench = [0.0, 3.1, 0.0, 0.0, 0.0, 0.0];
    measured.samples[1].digest = wrench_sample_digest(&measured.samples[1]);
    measured.sample_free_space_wrenches = vec![[0.0; 6]; 2];
    measured.sample_free_space_wrench_error_bounds = vec![[0.0; 6]; 2];
    measured.sample_sensor_transforms = vec![
        measured.sensor_to_support.clone(),
        WrenchTransform {
            rotation_to_support: [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
            translation_to_support_m: [0.0; 3],
        },
    ];

    assert!(matches!(
        infer_support_friction(&measured),
        FrictionInferenceResult::Updated {
            mode: FrictionInferenceMode::Sticking,
            interval,
            ..
        } if interval[0] > 0.30
    ));
}

#[test]
fn free_space_model_error_is_added_to_the_conservative_force_bound() {
    let mut nominal = measurement();
    nominal.sample_free_space_wrenches = vec![[0.0; 6]; nominal.samples.len()];
    nominal.sample_free_space_wrench_error_bounds = vec![[0.0; 6]; nominal.samples.len()];
    let FrictionInferenceResult::Updated {
        interval: nominal_interval,
        ..
    } = infer_support_friction(&nominal)
    else {
        panic!("nominal evidence should produce an interval");
    };

    let mut bounded = nominal.clone();
    bounded.sample_free_space_wrench_error_bounds =
        vec![[0.25, 0.0, 0.0, 0.0, 0.0, 0.0]; bounded.samples.len()];
    let FrictionInferenceResult::Updated {
        interval: bounded_interval,
        ..
    } = infer_support_friction(&bounded)
    else {
        panic!("bounded free-space uncertainty should preserve a conservative interval");
    };
    assert!(
        bounded_interval[0] < nominal_interval[0],
        "a larger free-space model bound must lower the friction lower bound: nominal={nominal_interval:?}, bounded={bounded_interval:?}"
    );

    let mut misaligned = bounded;
    misaligned.sample_free_space_wrench_error_bounds.pop();
    assert!(matches!(
        infer_support_friction(&misaligned),
        FrictionInferenceResult::NoInference(FrictionNoInferenceReason::InvalidMeasurement)
    ));
}

#[test]
fn slip_onset_interval_intersects_prior() {
    let mut measurement = measurement();
    measurement.samples = vec![
        sample(10, 1.004, [-3.1, 0.0, 0.0, 0.0, 0.0, 0.0]),
        sample(11, 1.006, [-3.2, 0.0, 0.0, 0.0, 0.0, 0.0]),
    ];
    measurement.motion.first_motion_s = Some(1.005);
    measurement.motion.displacement_m = [0.002, 0.0, 0.0];
    let FrictionInferenceResult::Updated {
        interval,
        mode,
        inferred_quantity,
    } = infer_support_friction(&measurement)
    else {
        panic!("a synchronized slip-onset bracket should bound friction");
    };
    assert_eq!(mode, FrictionInferenceMode::SlipOnset);
    assert_eq!(
        inferred_quantity,
        FrictionInferredQuantity::OnsetOrStickingForceRatio
    );
    assert!(interval[0] > 0.29 && interval[0] < 0.32, "{interval:?}");
    assert!(
        interval[1] > interval[0] && interval[1] < 0.36,
        "{interval:?}"
    );
    assert!(interval[0] >= measurement.prior_support_friction[0]);
    assert!(interval[1] <= measurement.prior_support_friction[1]);
}

#[test]
fn zero_or_uncertain_normal_load_returns_no_inference() {
    let mut zero_load = measurement();
    zero_load.mass_kg = [0.1, 0.1];
    for sample in &mut zero_load.samples {
        sample.wrench[2] = -1.0;
        sample.digest = wrench_sample_digest(sample);
    }
    assert!(matches!(
        infer_support_friction(&zero_load),
        FrictionInferenceResult::NoInference(FrictionNoInferenceReason::NonPositiveNormalLoad)
    ));

    let mut uncertain_load = measurement();
    uncertain_load.mass_kg = [0.1, 0.2];
    for sample in &mut uncertain_load.samples {
        sample.wrench[2] = -0.8;
        sample.error_bounds[2] = 1.0;
        sample.digest = wrench_sample_digest(sample);
    }
    assert!(matches!(
        infer_support_friction(&uncertain_load),
        FrictionInferenceResult::NoInference(FrictionNoInferenceReason::NonPositiveNormalLoad)
    ));
}

#[test]
fn unknown_mass_or_dynamic_sliding_returns_no_inference() {
    let mut unknown_mass = measurement();
    unknown_mass.mass_kg = [0.0, 0.0];
    assert!(matches!(
        infer_support_friction(&unknown_mass),
        FrictionInferenceResult::NoInference(FrictionNoInferenceReason::UnknownMass)
    ));

    let mut dynamic = measurement();
    dynamic.motion.max_speed_m_s = 0.02;
    dynamic.motion.first_motion_s = Some(1.005);
    dynamic.motion.displacement_m = [0.01, 0.0, 0.0];
    assert!(matches!(
        infer_support_friction(&dynamic),
        FrictionInferenceResult::NoInference(FrictionNoInferenceReason::DynamicMotion)
    ));

    let mut unbracketed_slide = measurement();
    unbracketed_slide.motion.displacement_m = [0.01, 0.0, 0.0];
    assert!(matches!(
        infer_support_friction(&unbracketed_slide),
        FrictionInferenceResult::NoInference(FrictionNoInferenceReason::UnbracketedMotion)
    ));
}

#[test]
fn cumulative_motion_before_resolved_force_returns_no_inference() {
    let mut measurement = measurement();
    // The runner may reset its local motion origin while the force lower bound
    // is unresolved, leaving a zero endpoint displacement despite path motion.
    measurement.motion.displacement_m = [0.0; 3];
    let legacy_digest = measurement.digest();
    measurement.motion.unresolved_motion_before_force = true;
    let unresolved_digest = measurement.digest();
    assert_ne!(unresolved_digest, legacy_digest);
    assert_eq!(
        infer_support_friction(&measurement),
        FrictionInferenceResult::NoInference(FrictionNoInferenceReason::UnbracketedMotion)
    );
    measurement.motion.unresolved_motion_before_force = false;
    assert_eq!(measurement.digest(), legacy_digest);
}

#[test]
fn alternate_contact_or_saturation_returns_no_inference() {
    let mut alternate = measurement();
    alternate
        .intended_contact
        .plausible_contact_sets
        .push(vec!["tool".into(), "obstacle".into()]);
    assert!(matches!(
        infer_support_friction(&alternate),
        FrictionInferenceResult::NoInference(FrictionNoInferenceReason::AmbiguousContact)
    ));

    let mut saturated = measurement();
    saturated.saturated = true;
    assert!(matches!(
        infer_support_friction(&saturated),
        FrictionInferenceResult::NoInference(FrictionNoInferenceReason::ActuatorSaturated)
    ));
}

#[test]
fn measurement_origin_and_digest_are_appended_to_belief() {
    let measurement = measurement();
    let FrictionInferenceResult::Updated { interval, .. } = infer_support_friction(&measurement)
    else {
        panic!("valid sticking measurement should update the interval");
    };
    let declared = 0.5;
    let mut belief = PhysicalParameterBelief::declared_point(
        PhysicalParameter::SupportFriction,
        declared,
        "scene.support_friction",
    );
    belief.narrow_interval(PhysicalParameter::SupportFriction, [0.1, 0.8], "prior");
    let reference = BeliefEvidenceReference {
        measurement_digest: measurement.digest(),
        origin: measurement.origin,
    };
    let updated = belief
        .intersect_interval_with_evidence(PhysicalParameter::SupportFriction, interval, reference)
        .expect("evidence interval intersects prior");
    let entry = belief.entry(PhysicalParameter::SupportFriction).unwrap();
    assert_eq!(updated, entry.empirical_interval.unwrap());
    assert_eq!(entry.declared.value, Some(declared));
    assert_eq!(entry.status, BeliefEpistemicStatus::DerivedConstraint);
    assert_eq!(entry.evidence.len(), 1);
    assert_eq!(entry.evidence[0].origin, WrenchEvidenceOrigin::Simulation);
    assert_eq!(entry.evidence[0].measurement_digest, measurement.digest());
}

#[test]
fn direct_belief_update_rejects_caller_labeled_hardware_evidence() {
    let mut belief = PhysicalParameterBelief::declared_point(
        PhysicalParameter::SupportFriction,
        0.5,
        "scene.support_friction",
    );
    belief.narrow_interval(PhysicalParameter::SupportFriction, [0.1, 0.9], "prior");
    let digest = "a".repeat(64);
    let result = belief.intersect_interval_with_evidence(
        PhysicalParameter::SupportFriction,
        [0.7, 0.8],
        BeliefEvidenceReference {
            measurement_digest: digest,
            origin: WrenchEvidenceOrigin::Hardware,
        },
    );
    assert_eq!(
        result,
        Err(realityos_semantics::physical_belief::BeliefUpdateError::HardwareOriginUntrusted)
    );
    let entry = belief.entry(PhysicalParameter::SupportFriction).unwrap();
    assert_eq!(entry.empirical_interval, Some([0.1, 0.9]));
    assert!(entry.evidence.is_empty());
}

#[test]
fn sensor_transform_applies_action_reaction_and_moment_arm() {
    let mut sample = sample(1, 1.0, [-2.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
    sample.error_bounds = [0.0; 6];
    sample.digest = wrench_sample_digest(&sample);
    let mut compensation = WrenchCompensation {
        bias: [0.0; 6],
        gravity_wrench: [0.0; 6],
        residual_error: [0.0; 6],
        digest: String::new(),
    };
    compensation.digest = compensation.digest();
    let interval = transform_sensor_wrench_to_support(
        &sample,
        &WrenchTransform {
            rotation_to_support: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            translation_to_support_m: [0.0, 0.0, 1.0],
        },
        &compensation,
        0.0,
        [0.0; 6],
    )
    .expect("well-formed wrench transform");
    assert_eq!(
        interval.lower[0], 2.0,
        "target force reverses sensor reaction"
    );
    assert_eq!(interval.upper[0], 2.0);
    assert_eq!(interval.lower[4], 2.0, "moment includes r cross F");
    assert_eq!(interval.upper[4], 2.0);
}

#[test]
fn stale_or_mismatched_lineage_never_updates_friction() {
    let mut stale = measurement();
    stale.current_observation_epoch = "epoch:newer".into();
    assert!(matches!(
        infer_support_friction(&stale),
        FrictionInferenceResult::NoInference(FrictionNoInferenceReason::StaleEvidence)
    ));

    let mut wrong_collision_model = measurement();
    wrong_collision_model.current_collision_model_digest = "c".repeat(64);
    assert!(matches!(
        infer_support_friction(&wrong_collision_model),
        FrictionInferenceResult::NoInference(FrictionNoInferenceReason::StaleEvidence)
    ));

    let mut approached = measurement();
    approached.motion.start_s = 0.4;
    let approached_result = infer_support_friction(&approached);
    assert!(
        !matches!(
            approached_result,
            FrictionInferenceResult::NoInference(FrictionNoInferenceReason::StaleEvidence)
        ),
        "the action window may begin before transit reaches the precontact baseline: {approached_result:?}"
    );

    let mut stale_baseline = measurement();
    if let Some(baseline) = stale_baseline.precontact_baseline.as_mut() {
        baseline.capture_s = 0.4;
        baseline.receive_s = 0.4001;
        baseline.digest = wrench_sample_digest(baseline);
    }
    assert!(matches!(
        infer_support_friction(&stale_baseline),
        FrictionInferenceResult::NoInference(
            FrictionNoInferenceReason::PrecontactBaselineGapExceeded
        )
    ));

    let mut wrong_origin = measurement();
    wrong_origin.samples[0].origin = WrenchSampleOrigin::Synthetic;
    wrong_origin.samples[0].digest = wrench_sample_digest(&wrong_origin.samples[0]);
    assert!(matches!(
        infer_support_friction(&wrong_origin),
        FrictionInferenceResult::NoInference(FrictionNoInferenceReason::SyntheticEvidence)
    ));

    let mut stale_sample = measurement();
    stale_sample.now_s = 2.0;
    assert!(matches!(
        infer_support_friction(&stale_sample),
        FrictionInferenceResult::NoInference(FrictionNoInferenceReason::StaleSample)
    ));
}

#[test]
fn completed_probe_trace_can_be_inferred_after_its_early_samples_age() {
    let mut trace = measurement();
    for sequence in 3..=30 {
        let capture_s = 1.014 + (sequence - 3) as f64 * 0.007;
        trace
            .samples
            .push(sample(sequence, capture_s, [-3.1, 0.0, 0.0, 0.0, 0.0, 0.0]));
    }
    trace.motion.end_s = trace.samples.last().unwrap().capture_s;
    trace.now_s = trace.samples.last().unwrap().receive_s;

    assert!(
        matches!(
            infer_support_friction(&trace),
            FrictionInferenceResult::Updated {
                mode: FrictionInferenceMode::Sticking,
                ..
            }
        ),
        "each sample arrived within the declared latency and the final sample is fresh"
    );
}

#[test]
fn non_planar_support_motion_refuses_friction_inference() {
    let mut tilted = measurement();
    tilted.support_normal_world = [0.0, 0.1, 0.99498743710662];
    assert!(matches!(
        infer_support_friction(&tilted),
        FrictionInferenceResult::NoInference(FrictionNoInferenceReason::UnsupportedSupport)
    ));

    let mut vertical_motion = measurement();
    vertical_motion.motion.displacement_m = [0.0, 0.0, 0.002];
    assert!(matches!(
        infer_support_friction(&vertical_motion),
        FrictionInferenceResult::NoInference(FrictionNoInferenceReason::NonPlanarMotion)
    ));

    let mut seated_slide = measurement();
    seated_slide.motion.pose_resolution_m = 0.0001;
    seated_slide.motion.displacement_m = [0.0031, 0.0, -0.000108];
    seated_slide.motion.first_motion_s = Some(1.005);
    let seated = infer_support_friction(&seated_slide);
    assert!(
        !matches!(
            seated,
            FrictionInferenceResult::NoInference(FrictionNoInferenceReason::NonPlanarMotion)
        ),
        "a 0.108 mm support seat on a 3.1 mm slide is still planar: {seated:?}"
    );

    let mut tilted_slide = measurement();
    tilted_slide.motion.pose_resolution_m = 0.0001;
    tilted_slide.motion.displacement_m = [0.0031, 0.0, 0.0005];
    tilted_slide.motion.first_motion_s = Some(1.005);
    assert_eq!(
        infer_support_friction(&tilted_slide),
        FrictionInferenceResult::NoInference(FrictionNoInferenceReason::NonPlanarMotion),
        "vertical travel over five percent of the planar probe is not a planar friction witness"
    );
}

#[test]
fn hardware_and_simulation_origins_are_preserved_and_must_match_samples() {
    let mut hardware = measurement();
    hardware.origin = WrenchEvidenceOrigin::Hardware;
    for sample in &mut hardware.samples {
        sample.origin = WrenchSampleOrigin::Hardware;
        sample.digest = wrench_sample_digest(sample);
    }
    let baseline = hardware.precontact_baseline.as_mut().unwrap();
    baseline.origin = WrenchSampleOrigin::Hardware;
    baseline.digest = wrench_sample_digest(baseline);
    assert!(matches!(
        infer_support_friction(&hardware),
        FrictionInferenceResult::NoInference(FrictionNoInferenceReason::HardwareOriginUntrusted)
    ));

    let mut mismatched = measurement();
    mismatched.samples[0].origin = WrenchSampleOrigin::Hardware;
    mismatched.samples[0].digest = wrench_sample_digest(&mismatched.samples[0]);
    assert!(matches!(
        infer_support_friction(&mismatched),
        FrictionInferenceResult::NoInference(FrictionNoInferenceReason::OriginMismatch)
    ));
}

#[test]
fn runtime_unexpected_tool_contact_refuses_friction_inference() {
    let mut measurement = measurement();
    measurement.runtime_contact_ambiguous = true;
    assert_eq!(
        infer_support_friction(&measurement),
        FrictionInferenceResult::NoInference(FrictionNoInferenceReason::UnexpectedContact)
    );
}

#[test]
fn absent_runtime_target_contact_refuses_friction_inference() {
    let mut measurement = measurement();
    let mut trace = sealed_motion_trace();
    let table_contact = ProbeContactPair {
        body1: "object".into(),
        geom1_name: "geom_1".into(),
        body2: "table".into(),
        geom2_name: "geom_2".into(),
    };
    trace.start_contact_pairs = Some(vec![table_contact.clone()]);
    for sample in &mut trace.samples {
        sample.contact_pairs = Some(vec![table_contact.clone()]);
        sample.digest = sample.digest();
    }
    trace.digest = trace.digest();
    measurement.motion_trace = Some(trace);
    assert_eq!(
        infer_support_friction(&measurement),
        FrictionInferenceResult::NoInference(FrictionNoInferenceReason::NoIntendedContact)
    );
}

#[test]
fn runtime_probe_trace_with_intended_contact_keeps_friction_inference_available() {
    let mut measurement = measurement();
    let mut trace = sealed_motion_trace();
    let table_contact = ProbeContactPair {
        body1: "object".into(),
        geom1_name: "geom_1".into(),
        body2: "table".into(),
        geom2_name: "geom_2".into(),
    };
    trace.start_contact_pairs = Some(vec![table_contact.clone()]);
    for sample in &mut trace.samples {
        sample.contact_pairs = Some(vec![table_contact.clone()]);
        sample.digest = sample.digest();
    }
    trace.samples[0].contact_pairs = Some(vec![ProbeContactPair {
        body1: "object".into(),
        geom1_name: "geom_3".into(),
        body2: "tool".into(),
        geom2_name: "geom_4".into(),
    }]);
    trace.samples[0].digest = trace.samples[0].digest();
    trace.digest = trace.digest();
    measurement.motion_trace = Some(trace);

    assert!(matches!(
        infer_support_friction(&measurement),
        FrictionInferenceResult::Updated { .. }
    ));
}

#[test]
fn partial_runtime_contact_trace_is_rejected() {
    let mut measurement = measurement();
    let mut trace = sealed_motion_trace();
    trace.start_contact_pairs = Some(Vec::new());
    trace.samples[0].contact_pairs = Some(Vec::new());
    trace.samples[0].digest = trace.samples[0].digest();
    trace.samples[2].contact_pairs = Some(Vec::new());
    trace.samples[2].digest = trace.samples[2].digest();
    trace.digest = trace.digest();
    measurement.motion_trace = Some(trace);

    assert_eq!(
        infer_support_friction(&measurement),
        FrictionInferenceResult::NoInference(FrictionNoInferenceReason::InvalidMotionTrace)
    );
}

#[test]
fn sample_gap_and_motion_skew_are_bounded() {
    let mut gap = measurement();
    gap.samples[1] = sample(2, 1.02, [-3.1, 0.0, 0.0, 0.0, 0.0, 0.0]);
    gap.motion.end_s = 1.03;
    gap.now_s = 1.04;
    assert!(matches!(
        infer_support_friction(&gap),
        FrictionInferenceResult::NoInference(FrictionNoInferenceReason::SampleGapExceeded)
    ));

    let mut skew = measurement();
    skew.motion.end_s = 1.03;
    skew.now_s = 1.04;
    assert!(matches!(
        infer_support_friction(&skew),
        FrictionInferenceResult::NoInference(FrictionNoInferenceReason::TimeSkewExceeded)
    ));
}

#[test]
fn empty_friction_intersection_produces_no_inference() {
    let mut measurement = measurement();
    measurement.prior_support_friction = [0.1, 0.2];
    let result = infer_support_friction(&measurement);
    match result {
        FrictionInferenceResult::NoInference(
            FrictionNoInferenceReason::PriorMeasurementIncompatible
            | FrictionNoInferenceReason::EmptyIntersection,
        ) => {}
        other => panic!("expected empty/prior-incompatible no-inference, got {other:?}"),
    }
}

#[test]
fn prior_incompatible_distinct_from_empty_measurement_bounds() {
    // 21003 second onset offline numbers: non-empty bounds entirely above prior.
    assert_eq!(
        intersect_measurement_bounds_with_prior(
            [0.4173207930536931, 0.491342106128889],
            [0.2590453156056787, 0.33705505290410587],
        ),
        Err(FrictionNoInferenceReason::PriorMeasurementIncompatible)
    );
    assert_eq!(
        intersect_measurement_bounds_with_prior([0.5, 0.4], [0.05, 0.8]),
        Err(FrictionNoInferenceReason::EmptyIntersection)
    );
    assert_eq!(
        intersect_measurement_bounds_with_prior([0.2, 0.3], [0.05, 0.8]),
        Ok([0.2, 0.3])
    );
}

#[test]
fn successive_probe_cadence_policies_on_21003_numbers() {
    // Sealed 21003 offline diagnosis: first SlipOnset posterior, second bounds
    // entirely above it, original prior [0.05, 0.8], mu_required = 0.3.
    let first = [0.2590453156056787, 0.33705505290410587];
    let second_bounds = [0.4173207930536931, 0.491342106128889];
    let orig_prior = [0.05, 0.8];
    let mu_required = 0.3;

    assert_eq!(
        apply_successive_probe_cadence(
            SuccessiveProbeCadencePolicy::StrictIntersect,
            first,
            second_bounds,
            Some(first),
            Some(orig_prior),
        ),
        SuccessiveProbeCadenceOutcome::Rejected {
            reason: FrictionNoInferenceReason::PriorMeasurementIncompatible,
        }
    );

    let retained = apply_successive_probe_cadence(
        SuccessiveProbeCadencePolicy::RetainFirstOnPriorIncompatible,
        first,
        second_bounds,
        Some(first),
        Some(orig_prior),
    );
    assert_eq!(
        retained,
        SuccessiveProbeCadenceOutcome::RetainedFirst {
            retained_interval: first,
            quarantined_bounds: second_bounds,
        }
    );
    let SuccessiveProbeCadenceOutcome::RetainedFirst {
        retained_interval, ..
    } = retained
    else {
        panic!("retain-first");
    };
    assert!(
        !friction_interval_proves_mu_required(retained_interval, mu_required),
        "retain-first leaves push MECHANICS_UNKNOWN when first posterior straddles mu_required"
    );

    let escalated = apply_successive_probe_cadence(
        SuccessiveProbeCadencePolicy::PreferHigherLoadFamilyOnIncompatible,
        first,
        second_bounds,
        Some(first),
        Some(orig_prior),
    );
    assert_eq!(
        escalated,
        SuccessiveProbeCadenceOutcome::EscalatedHigherLoadFamily {
            interval: second_bounds,
            quarantined_first_interval: first,
            measurement_bounds: second_bounds,
        }
    );
    let SuccessiveProbeCadenceOutcome::EscalatedHigherLoadFamily { interval, .. } = escalated
    else {
        panic!("escalate");
    };
    assert!(
        friction_interval_proves_mu_required(interval, mu_required),
        "higher-load-family escalation yields a push-proving lower bound on 21003 numbers"
    );
}

#[test]
fn successive_probe_cadence_transfers_to_heldout_numeric_family() {
    // Held-out numeric family: different first/second intervals than 21003,
    // same structural pattern (second entirely above first; straddling then proving).
    let first = [0.18, 0.28];
    let second_bounds = [0.35, 0.45];
    let orig_prior = [0.05, 0.9];
    let mu_required = 0.3;

    assert!(
        !friction_interval_proves_mu_required(first, mu_required),
        "held-out first posterior must straddle/fail mu_required"
    );
    let escalated = apply_successive_probe_cadence(
        SuccessiveProbeCadencePolicy::PreferHigherLoadFamilyOnIncompatible,
        first,
        second_bounds,
        Some(first),
        Some(orig_prior),
    );
    let SuccessiveProbeCadenceOutcome::EscalatedHigherLoadFamily { interval, .. } = escalated
    else {
        panic!("expected escalation on held-out numeric family, got {escalated:?}");
    };
    assert!(friction_interval_proves_mu_required(interval, mu_required));

    let retained = apply_successive_probe_cadence(
        SuccessiveProbeCadencePolicy::RetainFirstOnPriorIncompatible,
        first,
        second_bounds,
        Some(first),
        Some(orig_prior),
    );
    assert!(matches!(
        retained,
        SuccessiveProbeCadenceOutcome::RetainedFirst { .. }
    ));
}

#[test]
fn first_onset_outside_prior_policies_on_21008_numbers() {
    // Sealed 21008 FRICTION_BOUND_DIAGNOSIS: entirely_above_prior; no first SlipOnset.
    let prior = [0.05, 0.8];
    let bounds = [0.8410610796690302, 0.9046710475934353];
    let mu_required = 0.3;

    assert!(
        !friction_interval_proves_mu_required(prior, mu_required),
        "declared prior lower bound sits below mu_required"
    );

    let retained = apply_first_onset_outside_prior(
        FirstOnsetOutsidePriorPolicy::RetainDeclaredPrior,
        prior,
        bounds,
    );
    assert_eq!(
        retained,
        FirstOnsetOutsidePriorOutcome::Rejected {
            reason: FrictionNoInferenceReason::PriorMeasurementIncompatible,
        }
    );

    // PreferHigherLoadFamily does not apply without a first-onset posterior.
    let successive = apply_successive_probe_cadence(
        SuccessiveProbeCadencePolicy::PreferHigherLoadFamilyOnIncompatible,
        prior,
        bounds,
        None,
        None,
    );
    assert_eq!(
        successive,
        SuccessiveProbeCadenceOutcome::Rejected {
            reason: FrictionNoInferenceReason::PriorMeasurementIncompatible,
        }
    );

    let refused = apply_first_onset_outside_prior(
        FirstOnsetOutsidePriorPolicy::RefusePriorEnvelopeAsNotCredible,
        prior,
        bounds,
    );
    assert_eq!(
        refused,
        FirstOnsetOutsidePriorOutcome::RefusedPriorEnvelope {
            measurement_bounds: bounds,
            declared_prior: prior,
        }
    );

    let narrowed = apply_first_onset_outside_prior(
        FirstOnsetOutsidePriorPolicy::NarrowToMeasurementOnEntirelyAbovePrior,
        prior,
        bounds,
    );
    assert_eq!(
        narrowed,
        FirstOnsetOutsidePriorOutcome::AcceptedByReplacingPrior {
            interval: bounds,
            quarantined_prior: prior,
        }
    );
    let FirstOnsetOutsidePriorOutcome::AcceptedByReplacingPrior { interval, .. } = narrowed
    else {
        panic!("narrow");
    };
    assert!(
        friction_interval_proves_mu_required(interval, mu_required),
        "narrow-to-measurement yields a push-proving lower bound on 21008 numbers"
    );
}

#[test]
fn first_onset_outside_prior_transfers_to_heldout_numeric_family() {
    let prior = [0.05, 0.75];
    let bounds = [0.82, 0.91];
    let mu_required = 0.3;
    assert!(bounds[0] > prior[1]);
    let narrowed = apply_first_onset_outside_prior(
        FirstOnsetOutsidePriorPolicy::NarrowToMeasurementOnEntirelyAbovePrior,
        prior,
        bounds,
    );
    let FirstOnsetOutsidePriorOutcome::AcceptedByReplacingPrior { interval, .. } = narrowed
    else {
        panic!("expected narrow on held-out family, got {narrowed:?}");
    };
    assert!(friction_interval_proves_mu_required(interval, mu_required));
    let retained = apply_first_onset_outside_prior(
        FirstOnsetOutsidePriorPolicy::RetainDeclaredPrior,
        prior,
        bounds,
    );
    assert!(matches!(
        retained,
        FirstOnsetOutsidePriorOutcome::Rejected { .. }
    ));
}

#[test]
fn probe_task_continuation_diverges_on_21008_retain_vs_narrow() {
    use realityos_semantics::push_probe::probe_task_continuation_from_first_onset;
    use realityos_semantics::push_probe::ProbeTaskContinuationDecision;

    // Sealed 21008 numbers — complete-task continuation under competing policies.
    let prior = [0.05, 0.75];
    let bounds = [0.8410610796690302, 0.9046710475934353];
    let mu_required = 0.3;

    let retain = apply_first_onset_outside_prior(
        FirstOnsetOutsidePriorPolicy::RetainDeclaredPrior,
        prior,
        bounds,
    );
    let retain_task = probe_task_continuation_from_first_onset(&retain, mu_required);
    assert_eq!(
        retain_task,
        ProbeTaskContinuationDecision::CorrectRefuseIncompatiblePrior
    );

    let narrow = apply_first_onset_outside_prior(
        FirstOnsetOutsidePriorPolicy::NarrowToMeasurementOnEntirelyAbovePrior,
        prior,
        bounds,
    );
    let narrow_task = probe_task_continuation_from_first_onset(&narrow, mu_required);
    match narrow_task {
        ProbeTaskContinuationDecision::ContinueProbeWithBelief {
            interval,
            proves_mu_required,
        } => {
            assert_eq!(interval, bounds);
            assert!(proves_mu_required);
        }
        other => panic!("narrow must continue probe with proving belief, got {other:?}"),
    }
}

#[test]
fn matched_trials_miss_margin_gate_vs_narrow_on_21008_and_heldout() {
    use realityos_semantics::push_probe::probe_task_continuation_from_first_onset;
    use realityos_semantics::push_probe::ProbeTaskContinuationDecision;

    // Competing belief interventions; Governor thresholds untouched; offline only.
    let mu_required = 0.3;
    let max_relative_miss = 0.05;

    // 21008 sealed: relative miss ≈ 0.121 > 0.05 → gated refuse; bare Narrow continues.
    let prior_21008 = [0.05, 0.75];
    let bounds_21008 = [0.8410610796690302, 0.9046710475934353];
    let narrow_21008 = apply_first_onset_outside_prior(
        FirstOnsetOutsidePriorPolicy::NarrowToMeasurementOnEntirelyAbovePrior,
        prior_21008,
        bounds_21008,
    );
    let gated_21008 = apply_first_onset_outside_prior(
        FirstOnsetOutsidePriorPolicy::NarrowWhenRelativeMissAtMost {
            max_relative_miss,
        },
        prior_21008,
        bounds_21008,
    );
    assert!(matches!(
        probe_task_continuation_from_first_onset(&narrow_21008, mu_required),
        ProbeTaskContinuationDecision::ContinueProbeWithBelief {
            proves_mu_required: true,
            ..
        }
    ));
    assert_eq!(
        probe_task_continuation_from_first_onset(&gated_21008, mu_required),
        ProbeTaskContinuationDecision::CorrectRefusePriorEnvelopeNotCredible
    );

    // Held-out small miss: relative ≈ 0.0125 ≤ 0.05 → both continue with belief.
    let prior_small = [0.05, 0.8];
    let bounds_small = [0.81, 0.85];
    let narrow_small = apply_first_onset_outside_prior(
        FirstOnsetOutsidePriorPolicy::NarrowToMeasurementOnEntirelyAbovePrior,
        prior_small,
        bounds_small,
    );
    let gated_small = apply_first_onset_outside_prior(
        FirstOnsetOutsidePriorPolicy::NarrowWhenRelativeMissAtMost {
            max_relative_miss,
        },
        prior_small,
        bounds_small,
    );
    for outcome in [&narrow_small, &gated_small] {
        match probe_task_continuation_from_first_onset(outcome, mu_required) {
            ProbeTaskContinuationDecision::ContinueProbeWithBelief {
                interval,
                proves_mu_required,
            } => {
                assert_eq!(interval, bounds_small);
                assert!(proves_mu_required);
            }
            other => panic!("small-miss must continue with belief, got {other:?}"),
        }
    }
}

#[test]
fn matched_trials_reprobe_replan_vs_stop_refuse_on_21008_and_heldout() {
    use realityos_semantics::push_probe::{
        probe_task_continuation_from_first_onset_with_replan, PriorEnvelopeRefuseContinuation,
        ProbeTaskContinuationDecision,
    };

    // Competing complete-task interventions after a gated prior-envelope refuse.
    // Does not change Governor thresholds or invent μ / prove μ_required.
    let mu_required = 0.3;
    let prior_21008 = [0.05, 0.75];
    let bounds_21008 = [0.8410610796690302, 0.9046710475934353];
    let gated_21008 = apply_first_onset_outside_prior(
        FirstOnsetOutsidePriorPolicy::NarrowWhenRelativeMissAtMost {
            max_relative_miss: 0.05,
        },
        prior_21008,
        bounds_21008,
    );
    let stop = probe_task_continuation_from_first_onset_with_replan(
        &gated_21008,
        mu_required,
        PriorEnvelopeRefuseContinuation::StopAsCorrectRefuse,
    );
    let replan = probe_task_continuation_from_first_onset_with_replan(
        &gated_21008,
        mu_required,
        PriorEnvelopeRefuseContinuation::RequestReprobe,
    );
    assert_eq!(
        stop,
        ProbeTaskContinuationDecision::CorrectRefusePriorEnvelopeNotCredible
    );
    match replan {
        ProbeTaskContinuationDecision::RequestReprobeAfterPriorEnvelopeRefuse {
            measurement_bounds,
            declared_prior,
        } => {
            assert_eq!(measurement_bounds, bounds_21008);
            assert_eq!(declared_prior, prior_21008);
        }
        other => panic!("21008 gated refuse+replan must request reprobe, got {other:?}"),
    }
    // Replan must not silently become a continue-with-belief (unsafe acceptance).
    assert!(!matches!(
        replan,
        ProbeTaskContinuationDecision::ContinueProbeWithBelief { .. }
    ));

    // Held-out small miss: gated Narrow continues; replan policy unused on continue path.
    let prior_small = [0.05, 0.8];
    let bounds_small = [0.81, 0.85];
    let gated_small = apply_first_onset_outside_prior(
        FirstOnsetOutsidePriorPolicy::NarrowWhenRelativeMissAtMost {
            max_relative_miss: 0.05,
        },
        prior_small,
        bounds_small,
    );
    match probe_task_continuation_from_first_onset_with_replan(
        &gated_small,
        mu_required,
        PriorEnvelopeRefuseContinuation::RequestReprobe,
    ) {
        ProbeTaskContinuationDecision::ContinueProbeWithBelief {
            interval,
            proves_mu_required,
        } => {
            assert_eq!(interval, bounds_small);
            assert!(proves_mu_required);
        }
        other => panic!("small-miss must continue with belief even under replan policy, got {other:?}"),
    }
}

#[test]
fn prior_envelope_miss_margin_quantifies_21008_and_heldout() {
    let prior = [0.05, 0.8];
    let bounds = [0.8410610796690302, 0.9046710475934353];
    let miss = prior_envelope_miss_margin(prior, bounds);
    match miss {
        PriorEnvelopeMissMargin::EntirelyAbovePrior {
            absolute_miss,
            relative_miss,
            ..
        } => {
            assert!((absolute_miss - (bounds[0] - prior[1])).abs() < 1e-12);
            assert!((relative_miss - absolute_miss / prior[1]).abs() < 1e-12);
            assert!(relative_miss > 0.05);
        }
        other => panic!("expected entirely_above miss on 21008, got {other:?}"),
    }

    let heldout_prior = [0.05, 0.75];
    let heldout_bounds = [0.82, 0.91];
    assert!(matches!(
        prior_envelope_miss_margin(heldout_prior, heldout_bounds),
        PriorEnvelopeMissMargin::EntirelyAbovePrior { .. }
    ));
    assert!(matches!(
        prior_envelope_miss_margin([0.05, 0.8], [0.2, 0.4]),
        PriorEnvelopeMissMargin::NotEntirelyAbovePrior { .. }
    ));
}


#[test]
fn measurement_digest_binds_sample_values_origin_and_transform() {
    let baseline = measurement();
    let baseline_digest = baseline.digest();

    let mut changed_contacts = baseline.clone();
    let mut trace = sealed_motion_trace();
    trace.start_contact_pairs = Some(vec![ProbeContactPair {
        body1: "object".into(),
        geom1_name: "geom_1".into(),
        body2: "table".into(),
        geom2_name: "geom_2".into(),
    }]);
    for sample in &mut trace.samples {
        sample.contact_pairs = Some(vec![ProbeContactPair {
            body1: "tool".into(),
            geom1_name: "geom_1".into(),
            body2: "object".into(),
            geom2_name: "geom_3".into(),
        }]);
        sample.digest = sample.digest();
    }
    trace.digest = trace.digest();
    changed_contacts.motion_trace = Some(trace);
    assert_ne!(baseline_digest, changed_contacts.digest());

    let mut changed_force = baseline.clone();
    changed_force.samples[0].wrench[0] -= 0.1;
    changed_force.samples[0].digest = wrench_sample_digest(&changed_force.samples[0]);
    assert_ne!(baseline_digest, changed_force.digest());

    let mut changed_origin = baseline.clone();
    changed_origin.origin = WrenchEvidenceOrigin::Hardware;
    for sample in &mut changed_origin.samples {
        sample.origin = WrenchSampleOrigin::Hardware;
        sample.digest = wrench_sample_digest(sample);
    }
    assert_ne!(baseline_digest, changed_origin.digest());

    let mut changed_transform = baseline;
    changed_transform.sensor_to_support.translation_to_support_m[2] = 0.01;
    assert_ne!(baseline_digest, changed_transform.digest());

    let baseline = measurement();
    let mut changed_gravity = baseline.clone();
    changed_gravity.sample_free_space_wrenches = vec![[0.1, 0.0, 0.0, 0.0, 0.0, 0.0]; 2];
    changed_gravity.sample_free_space_wrench_error_bounds = vec![[0.0; 6]; 2];
    assert_ne!(baseline.digest(), changed_gravity.digest());

    let mut changed_model_bound = baseline.clone();
    changed_model_bound.sample_free_space_wrenches = vec![[0.0; 6]; 2];
    changed_model_bound.sample_free_space_wrench_error_bounds =
        vec![[0.01, 0.0, 0.0, 0.0, 0.0, 0.0]; 2];
    assert_ne!(baseline.digest(), changed_model_bound.digest());

    let mut changed_sample_pose = baseline.clone();
    changed_sample_pose.sample_sensor_transforms = vec![
        baseline.sensor_to_support.clone(),
        WrenchTransform {
            rotation_to_support: [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
            translation_to_support_m: [0.0; 3],
        },
    ];
    assert_ne!(baseline.digest(), changed_sample_pose.digest());
}

#[test]
fn legacy_parameter_belief_defaults_evidence_to_empty() {
    let belief = PhysicalParameterBelief::declared_point(
        PhysicalParameter::SupportFriction,
        0.5,
        "legacy.prior",
    );
    let mut stored =
        serde_json::to_value(belief.entry(PhysicalParameter::SupportFriction).unwrap())
            .expect("serialize a current parameter belief");
    stored
        .as_object_mut()
        .expect("parameter belief object")
        .remove("evidence");
    let restored: realityos_semantics::physical_belief::ParameterBelief =
        serde_json::from_value(stored).expect("old beliefs remain readable");
    assert!(restored.evidence.is_empty());
}

#[test]
fn matched_trials_after_reprobe_feedback_on_21008_and_heldout() {
    use realityos_semantics::push_probe::{
        probe_task_continuation_after_reprobe_measurement,
        probe_task_continuation_from_first_onset_with_replan, AfterReprobeFeedbackDecision,
        FirstOnsetOutsidePriorPolicy, PriorEnvelopeRefuseContinuation,
        ProbeTaskContinuationDecision,
    };

    let mu_required = 0.3;
    let policy = FirstOnsetOutsidePriorPolicy::NarrowWhenRelativeMissAtMost {
        max_relative_miss: 0.05,
    };
    let prior_21008 = [0.05, 0.75];
    let bounds_21008 = [0.8410610796690302, 0.9046710475934353];
    let gated_21008 = apply_first_onset_outside_prior(policy, prior_21008, bounds_21008);
    let pending = probe_task_continuation_from_first_onset_with_replan(
        &gated_21008,
        mu_required,
        PriorEnvelopeRefuseContinuation::RequestReprobe,
    );
    assert!(matches!(
        pending,
        ProbeTaskContinuationDecision::RequestReprobeAfterPriorEnvelopeRefuse { .. }
    ));

    // Without pending RequestReprobe: must not continue.
    let stop_only = ProbeTaskContinuationDecision::CorrectRefusePriorEnvelopeNotCredible;
    assert_eq!(
        probe_task_continuation_after_reprobe_measurement(
            &stop_only,
            [0.2, 0.4],
            policy,
            mu_required,
        ),
        AfterReprobeFeedbackDecision::RejectedMissingPendingReprobe
    );

    // Second probe still entirely above prior with large miss → refuse again (not Continue).
    let still_miss = probe_task_continuation_after_reprobe_measurement(
        &pending,
        [0.86, 0.92],
        policy,
        mu_required,
    );
    assert!(matches!(
        still_miss,
        AfterReprobeFeedbackDecision::CorrectRefusePriorEnvelopeStillNotCredible { .. }
    ));
    assert!(!matches!(
        still_miss,
        AfterReprobeFeedbackDecision::ContinueProbeWithBelief { .. }
    ));

    // Second probe intersects prior → Continue with belief (offline feedback success).
    let recovered = probe_task_continuation_after_reprobe_measurement(
        &pending,
        [0.4, 0.7],
        policy,
        mu_required,
    );
    match recovered {
        AfterReprobeFeedbackDecision::ContinueProbeWithBelief {
            interval,
            proves_mu_required,
            declared_prior,
            ..
        } => {
            assert_eq!(declared_prior, prior_21008);
            assert!(proves_mu_required);
            assert!(interval[0] <= interval[1]);
        }
        other => panic!("overlapping reprobe should continue with belief, got {other:?}"),
    }

    // Held-out: invalid bounds rejected; no unsafe accept from garbage measurement.
    assert!(matches!(
        probe_task_continuation_after_reprobe_measurement(
            &pending,
            [f64::NAN, 0.5],
            policy,
            mu_required,
        ),
        AfterReprobeFeedbackDecision::RejectedInvalidReprobeMeasurement { .. }
    ));
}

#[test]
fn matched_trials_next_action_from_belief_after_reprobe_on_21008_and_heldout() {
    use realityos_semantics::push_probe::{
        blind_proceed_ignoring_belief_proof, next_probe_action_after_reprobe_feedback,
        probe_task_continuation_after_reprobe_measurement,
        probe_task_continuation_from_first_onset_with_replan, AfterReprobeFeedbackDecision,
        FirstOnsetOutsidePriorPolicy, NextProbeActionDecision,
        PriorEnvelopeRefuseContinuation, ProbeTaskContinuationDecision,
    };

    let mu_required = 0.3;
    let policy = FirstOnsetOutsidePriorPolicy::NarrowWhenRelativeMissAtMost {
        max_relative_miss: 0.05,
    };
    let prior_21008 = [0.05, 0.75];
    let bounds_21008 = [0.8410610796690302, 0.9046710475934353];
    let gated_21008 = apply_first_onset_outside_prior(policy, prior_21008, bounds_21008);
    let pending = probe_task_continuation_from_first_onset_with_replan(
        &gated_21008,
        mu_required,
        PriorEnvelopeRefuseContinuation::RequestReprobe,
    );
    assert!(matches!(
        pending,
        ProbeTaskContinuationDecision::RequestReprobeAfterPriorEnvelopeRefuse { .. }
    ));

    // Recovered overlapping reprobe → Continue with belief that proves μ → proceed.
    let recovered = probe_task_continuation_after_reprobe_measurement(
        &pending,
        [0.4, 0.7],
        policy,
        mu_required,
    );
    let proceed = next_probe_action_after_reprobe_feedback(&recovered, mu_required);
    assert!(matches!(
        proceed,
        NextProbeActionDecision::ProceedCompleteTaskWithBelief { .. }
    ));

    // Still-miss after reprobe → hold after correct refuse (not proceed).
    let still_miss = probe_task_continuation_after_reprobe_measurement(
        &pending,
        [0.86, 0.92],
        policy,
        mu_required,
    );
    let hold = next_probe_action_after_reprobe_feedback(&still_miss, mu_required);
    assert!(matches!(
        hold,
        NextProbeActionDecision::HoldAfterCorrectRefuse { .. }
    ));
    assert!(!matches!(
        hold,
        NextProbeActionDecision::ProceedCompleteTaskWithBelief { .. }
    ));

    // Negative control: Continue without proof must not blind-proceed.
    let continue_unproven = AfterReprobeFeedbackDecision::ContinueProbeWithBelief {
        interval: [0.5, 0.6],
        proves_mu_required: false,
        new_measurement_bounds: [0.5, 0.6],
        declared_prior: prior_21008,
    };
    assert_eq!(
        next_probe_action_after_reprobe_feedback(&continue_unproven, mu_required),
        NextProbeActionDecision::HoldRefuseInsufficientBeliefCoverage {
            interval: [0.5, 0.6],
            mu_required,
            declared_prior: prior_21008,
        }
    );
    assert_eq!(
        blind_proceed_ignoring_belief_proof(&continue_unproven),
        NextProbeActionDecision::RejectedBlindProceedWithoutBeliefProof
    );

    // Held-out: invalid measurement → rejected not continuing (no unsafe accept).
    let invalid = probe_task_continuation_after_reprobe_measurement(
        &pending,
        [f64::NAN, 0.5],
        policy,
        mu_required,
    );
    assert!(matches!(
        next_probe_action_after_reprobe_feedback(&invalid, mu_required),
        NextProbeActionDecision::RejectedNotContinuing { .. }
    ));
}
