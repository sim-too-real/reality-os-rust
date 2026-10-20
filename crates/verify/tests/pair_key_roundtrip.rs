//! serde_json can move an f64 by one ulp when a trial record is reloaded.
//! Pairing must keep the key stamped before that write and still reject a
//! different serialized pose.
use realityos_verify::manipulation_benchmark::{
    evaluate_paired_screening_gate, load_benchmark_trial_record, paired_campaign_delta,
    paired_trial_key, BenchmarkTrialRecord,
};
use std::fs;
use std::path::Path;

fn load_records(dir: &Path) -> Vec<BenchmarkTrialRecord> {
    let mut paths = fs::read_dir(dir.join("trials"))
        .unwrap()
        .map(|entry| entry.unwrap().path())
        .filter(|path| path.extension().and_then(|ext| ext.to_str()) == Some("json"))
        .collect::<Vec<_>>();
    paths.sort();
    paths
        .iter()
        .map(|path| load_benchmark_trial_record(&fs::read(path).unwrap()).unwrap())
        .collect()
}

#[test]
fn reloaded_suite_trials_pair_on_the_persisted_key() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../docs/superpowers/evidence");
    let baseline = load_records(&root.join("force-aware-probe-4e5e6fc"));
    let candidate = load_records(&root.join("suite-v3-paired-after-label-fix"));
    assert_eq!(baseline.len(), 60);
    assert_eq!(candidate.len(), 60);

    let sample = baseline
        .iter()
        .find(|record| record.trial_id.ends_with("21000:0"))
        .unwrap();
    let trial = sample.trial.as_ref().unwrap();
    println!(
        "sample stored={} computed={} qpos5_bits={:x}",
        trial.paired_trial_key.as_deref().unwrap_or(""),
        paired_trial_key(trial).unwrap(),
        trial.initial_qpos[5].to_bits()
    );
    let delta = paired_campaign_delta(&baseline, &candidate).expect("persisted keys pair");
    assert_eq!(delta.matched_trial_keys, 60);
    let gate = evaluate_paired_screening_gate(&delta);
    println!(
        "matched={} screening_gate={} best_gain={:?} worst_regression={:?}",
        delta.matched_trial_keys, gate.passed, gate.best_gain, gate.worst_regression
    );

    let mut broken = candidate[0].clone();
    let trial = broken.trial.as_mut().expect("resolved trial");
    trial.initial_qpos[0] += 0.25;
    let error = paired_campaign_delta(std::slice::from_ref(&broken), std::slice::from_ref(&broken))
        .expect_err("a different serialized pose is not the persisted key");
    let message = error.to_string();
    assert!(
        message.contains("mismatched paired_trial_key"),
        "{message}"
    );

    let mut stamped = candidate[0].clone();
    let trial = stamped.trial.as_mut().expect("resolved trial");
    trial.paired_trial_key = Some(paired_trial_key(trial).unwrap());
    paired_campaign_delta(std::slice::from_ref(&stamped), std::slice::from_ref(&stamped))
        .expect("the reloaded bits are a valid key when stamped as such");
}
