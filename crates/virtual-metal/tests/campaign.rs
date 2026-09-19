use realityos_virtual_metal::{run_campaign, CAMPAIGN_SCHEMA, EVIDENCE_STATUS, VERDICT_PASS};

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
    for (x, y) in a.rows.iter().zip(b.rows.iter()) {
        assert_eq!(x.seed, y.seed);
        assert_eq!(x.scenario, y.scenario);
        assert_eq!(x.verdict, y.verdict);
        assert_eq!(x.invariant_violations, y.invariant_violations);
        assert_eq!(x.physical_actions, y.physical_actions);
    }
    assert_eq!(a.n_fail, 0, "failing seeds: {:?}", failing(&a));
    assert_eq!(a.verdict, VERDICT_PASS);
    let raw = serde_json::to_value(&a).unwrap();
    assert_ne!(raw["hardware_present"], true);
    assert_ne!(raw["schema"], "realityos.metal_proof/1");
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
