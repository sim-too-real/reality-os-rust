use realityos_virtual_metal::{
    run_campaign, CAMPAIGN_SCHEMA, EVIDENCE_STATUS, FAULT_MODEL_VERSION,
    SCENARIO_GENERATOR_VERSION, TRUTH_PACK_SCHEMA, VERDICT_PASS,
};

#[test]
fn campaign_1000_twice_deterministic_and_not_metal() {
    let a = run_campaign(42, 1000);
    let b = run_campaign(42, 1000);
    assert!(a.n_instances >= 1000);
    assert_eq!(a.n_instances, b.n_instances);
    assert_eq!(a.seed, b.seed);
    assert_eq!(a.verdict, b.verdict);
    assert_eq!(a.n_fail, b.n_fail);
    assert_eq!(a.n_ok, b.n_ok);
    assert!(!a.hardware_present);
    assert!(!b.hardware_present);
    assert_eq!(a.schema, CAMPAIGN_SCHEMA);
    assert_eq!(a.evidence_status, EVIDENCE_STATUS);
    assert_eq!(a.scenario_generator_version, SCENARIO_GENERATOR_VERSION);
    assert_eq!(a.fault_model_version, FAULT_MODEL_VERSION);
    assert_eq!(a.truth_pack_schema, TRUTH_PACK_SCHEMA);
    assert!(!a.truth_pack_content_hash.is_empty());
    assert_eq!(a.truth_pack_content_hash, b.truth_pack_content_hash);
    assert_eq!(a.reality_os_sha, b.reality_os_sha);
    assert_eq!(a.virtual_metal_sha, b.virtual_metal_sha);
    assert_eq!(a.coverage, b.coverage);
    assert!(
        a.coverage.len() >= 8,
        "coverage must be reported by scenario category, got {:?}",
        a.coverage
    );
    for (x, y) in a.rows.iter().zip(b.rows.iter()) {
        assert_eq!(x.seed, y.seed);
        assert_eq!(x.scenario, y.scenario);
        assert_eq!(x.category, y.category);
        assert_eq!(x.verdict, y.verdict);
        assert_eq!(x.invariant_violations, y.invariant_violations);
        assert_eq!(x.physical_actions, y.physical_actions);
        assert_eq!(x.realization, y.realization);
    }
    assert_eq!(a.n_fail, 0, "failing seeds: {:?}", failing(&a));
    assert_eq!(a.verdict, VERDICT_PASS);
    let raw = serde_json::to_value(&a).unwrap();
    assert_ne!(raw["hardware_present"], true);
    assert_ne!(raw["schema"], "realityos.metal_proof/1");
    assert!(raw["coverage"].is_object());
}

#[test]
fn two_seeds_sample_different_realizations() {
    let a = run_campaign(1, 1);
    let b = run_campaign(2, 1);
    assert_ne!(
        a.rows[0].realization, b.rows[0].realization,
        "different seeds must sample different uncertainty"
    );
}

fn failing(a: &realityos_virtual_metal::CampaignRecord) -> Vec<(u64, String, Vec<String>)> {
    a.rows
        .iter()
        .filter(|r| !r.invariant_violations.is_empty())
        .map(|r| {
            (
                r.instance,
                r.scenario.clone(),
                r.invariant_violations.clone(),
            )
        })
        .collect()
}
