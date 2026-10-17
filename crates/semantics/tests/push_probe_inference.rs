use realityos_semantics::observation::{wrench_sample_digest, WrenchSample};
use realityos_semantics::physical_belief::{
    BeliefEpistemicStatus, BeliefEvidenceReference, PhysicalParameter, PhysicalParameterBelief,
};
use realityos_semantics::push_probe::{
    infer_support_friction, transform_sensor_wrench_to_support, FrictionInferenceMode,
    FrictionInferenceResult, FrictionNoInferenceReason, IntendedContactEvidence, ProbeMotionWindow,
    PushProbeMeasurement, WrenchCompensation, WrenchEvidenceOrigin, WrenchTransform,
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
        sample_gravity_wrenches: Vec::new(),
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
        },
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
        saturated: false,
        origin: WrenchEvidenceOrigin::Simulation,
    }
}

#[test]
fn sticking_probe_adds_only_a_conservative_lower_bound() {
    let result = infer_support_friction(&measurement());
    let FrictionInferenceResult::Updated { interval, mode } = result else {
        panic!("sticking wrench and motion evidence should provide a lower bound: {result:?}");
    };
    assert_eq!(mode, FrictionInferenceMode::Sticking);
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
    measured.sample_gravity_wrenches = vec![[1.0, 0.0, 0.0, 0.0, 0.0, 0.0]; 2];

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
    measured.sample_gravity_wrenches = vec![[0.0; 6]; 2];
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
        } if interval[0] > 0.30
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
    let FrictionInferenceResult::Updated { interval, mode } = infer_support_friction(&measurement)
    else {
        panic!("a synchronized slip-onset bracket should bound friction");
    };
    assert_eq!(mode, FrictionInferenceMode::SlipOnset);
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
        FrictionInferenceResult::Updated {
            mode: FrictionInferenceMode::Sticking,
            ..
        }
    ));

    let mut mismatched = measurement();
    mismatched.origin = WrenchEvidenceOrigin::Hardware;
    assert!(matches!(
        infer_support_friction(&mismatched),
        FrictionInferenceResult::NoInference(FrictionNoInferenceReason::OriginMismatch)
    ));
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
    assert!(matches!(
        infer_support_friction(&measurement),
        FrictionInferenceResult::NoInference(FrictionNoInferenceReason::EmptyIntersection)
    ));
}

#[test]
fn measurement_digest_binds_sample_values_origin_and_transform() {
    let baseline = measurement();
    let baseline_digest = baseline.digest();

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
    changed_gravity.sample_gravity_wrenches = vec![[0.1, 0.0, 0.0, 0.0, 0.0, 0.0]; 2];
    assert_ne!(baseline.digest(), changed_gravity.digest());

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
