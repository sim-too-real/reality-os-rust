use realityos_semantics::observation::{
    wrench_sample_digest, JointStateSample, ObservationFrame, WrenchSample,
};
use realityos_semantics::sensor::WrenchSampleOrigin;

fn sample(sequence: u64, epoch: &str) -> WrenchSample {
    let mut sample = WrenchSample {
        sensor_id: "wrist_ft".into(),
        sensor_frame: "tool0".into(),
        transform_epoch: epoch.into(),
        units: "N,N*m".into(),
        wrench: [1.0, -2.0, 3.0, 0.1, -0.2, 0.3],
        error_bounds: [0.01; 6],
        capture_s: 1.0 + sequence as f64 * 0.01,
        receive_s: 1.001 + sequence as f64 * 0.01,
        sequence,
        clock_domain: "sim-clock".into(),
        origin: WrenchSampleOrigin::Simulation,
        digest: String::new(),
    };
    sample.digest = wrench_sample_digest(&sample);
    sample
}

fn frame(samples: Vec<WrenchSample>, epoch: &str) -> ObservationFrame {
    ObservationFrame {
        frame_id: "frame-1".into(),
        transform_epoch: epoch.into(),
        observations: Vec::new(),
        joint_state: Vec::<JointStateSample>::new(),
        wrench_samples: samples,
        as_of_s: 2.0,
    }
}

#[test]
fn wrench_digest_binds_every_sample_field() {
    let original = sample(1, "epoch-1");
    let digest = wrench_sample_digest(&original);

    let mut changed = original.clone();
    changed.sensor_id.push_str("-other");
    assert_ne!(digest, wrench_sample_digest(&changed));
    let mut changed = original.clone();
    changed.sensor_frame.push_str("-other");
    assert_ne!(digest, wrench_sample_digest(&changed));
    let mut changed = original.clone();
    changed.transform_epoch.push_str("-other");
    assert_ne!(digest, wrench_sample_digest(&changed));
    let mut changed = original.clone();
    changed.units = "N, N*m".into();
    assert_ne!(digest, wrench_sample_digest(&changed));
    let mut changed = original.clone();
    changed.wrench[0] += 0.01;
    assert_ne!(digest, wrench_sample_digest(&changed));
    let mut changed = original.clone();
    changed.error_bounds[0] += 0.01;
    assert_ne!(digest, wrench_sample_digest(&changed));
    let mut changed = original.clone();
    changed.capture_s += 0.01;
    assert_ne!(digest, wrench_sample_digest(&changed));
    let mut changed = original.clone();
    changed.receive_s += 0.01;
    assert_ne!(digest, wrench_sample_digest(&changed));
    let mut changed = original.clone();
    changed.sequence += 1;
    assert_ne!(digest, wrench_sample_digest(&changed));
    let mut changed = original.clone();
    changed.clock_domain.push_str("-other");
    assert_ne!(digest, wrench_sample_digest(&changed));
    let mut changed = original;
    changed.origin = WrenchSampleOrigin::Synthetic;
    assert_ne!(digest, wrench_sample_digest(&changed));
}

#[test]
fn invalid_wrench_or_error_bound_is_rejected() {
    let mut invalid = sample(1, "epoch-1");
    invalid.wrench[2] = f64::NAN;
    invalid.digest = wrench_sample_digest(&invalid);
    assert!(invalid.validate().is_err());

    let mut invalid = sample(1, "epoch-1");
    invalid.error_bounds[4] = -0.001;
    invalid.digest = wrench_sample_digest(&invalid);
    assert!(invalid.validate().is_err());

    let mut invalid = sample(1, "epoch-1");
    invalid.sensor_id.clear();
    invalid.digest = wrench_sample_digest(&invalid);
    assert!(invalid.validate().is_err());

    let mut invalid = sample(1, "epoch-1");
    invalid.units = "kg".into();
    invalid.digest = wrench_sample_digest(&invalid);
    assert!(invalid.validate().is_err());
}

#[test]
fn wrench_samples_require_monotonic_sequence_and_matching_epoch() {
    let out_of_order = frame(vec![sample(2, "epoch-1"), sample(1, "epoch-1")], "epoch-1");
    assert!(out_of_order.validate_wrench_samples(2.0, 1.0).is_err());

    let wrong_epoch = frame(vec![sample(1, "epoch-old")], "epoch-1");
    assert!(wrong_epoch.validate_wrench_samples(2.0, 1.0).is_err());

    let stale = frame(vec![sample(1, "epoch-1")], "epoch-1");
    assert!(stale.validate_wrench_samples(2.0, 0.5).is_err());

    let fresh = frame(vec![sample(1, "epoch-1"), sample(2, "epoch-1")], "epoch-1");
    assert!(fresh.validate_wrench_samples(2.0, 1.0).is_ok());

    let mut delayed_first = sample(1, "epoch-1");
    delayed_first.receive_s = 1.5;
    delayed_first.digest = wrench_sample_digest(&delayed_first);
    let mut delayed_second = sample(2, "epoch-1");
    delayed_second.receive_s = 1.03;
    delayed_second.digest = wrench_sample_digest(&delayed_second);
    let delayed = frame(vec![delayed_first, delayed_second], "epoch-1");
    assert!(delayed.validate_wrench_samples(2.0, 1.0).is_err());
}

#[test]
fn observation_frame_round_trips_wrench_samples() {
    let original = frame(vec![sample(1, "epoch-1")], "epoch-1");
    let encoded = serde_json::to_vec(&original).unwrap();
    let decoded: ObservationFrame = serde_json::from_slice(&encoded).unwrap();
    assert_eq!(decoded, original);

    let legacy = serde_json::json!({
        "frame_id": "legacy-frame",
        "transform_epoch": "epoch-1",
        "observations": [],
        "joint_state": [],
        "as_of_s": 2.0
    });
    let decoded: ObservationFrame = serde_json::from_value(legacy).unwrap();
    assert!(decoded.wrench_samples.is_empty());
}
