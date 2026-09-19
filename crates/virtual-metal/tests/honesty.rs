use realityos_kernel::HonestyStamp;
use realityos_metal::{MetalProof, ProofMeta};
use realityos_virtual_metal::{
    honesty, refuse_hardware_present_true, refuse_measured_token, refuse_metal_proof_install,
    run_campaign, CAMPAIGN_SCHEMA, EVIDENCE_STATUS, VERDICT_PASS,
};

fn meta(present: bool) -> ProofMeta {
    ProofMeta {
        hardware_model: "XL330-M288-T".into(),
        controller_model: "none".into(),
        real_device_identity: serde_json::json!({
            "hardware_identity": {
                "metal": true,
                "evidence_status": EVIDENCE_STATUS,
                "serial": "VM-XL330-M288-1"
            }
        }),
        software_commit_sha: "x".into(),
        authority_uid: "a".into(),
        autonomy_uid: "b".into(),
        test_date: "t".into(),
        hardware_present: present,
        used_os_monotonic_clock: true,
        used_hardware_driver_port: true,
        cutoff_mechanism: "none".into(),
        cutoff_tested: false,
        cutoff_operator_attested: false,
        cutoff_live_observed: false,
        unplug_live_observed: false,
        pwm_limit_requested: None,
        pwm_limit_measured: None,
        experiment_min: None,
        experiment_max: None,
        startup_present: None,
        direct_device_open_attempts: 0,
        direct_device_open_successes: 0,
        direct_device_write_successes: 0,
        duplicate_writes_after_restart: 0,
        sensor_source: String::new(),
        device_capture_s: None,
        authority_receive_s: None,
        freshness_threshold_s: None,
    }
}

#[test]
fn constructors_refuse_measured_and_hardware_present() {
    assert!(refuse_measured_token("MEASURED").is_err());
    assert!(refuse_measured_token("METAL_MEASURED").is_err());
    assert!(refuse_measured_token("VIRTUAL_METAL").is_err());
    assert!(refuse_measured_token("VIRTUAL_METAL_PASS").is_err());
    assert!(HonestyStamp::sim("MEASURED").is_err());
    let h = honesty();
    assert!(!h.metal());
    assert!(!h.measured());
    assert_eq!(h.evidence_status(), EVIDENCE_STATUS);
    assert!(refuse_hardware_present_true().is_err());
    assert!(MetalProof::from_measured(meta(false), vec![], vec![]).is_err());
    assert!(refuse_metal_proof_install(meta(true)).is_err());
}

#[test]
fn campaign_schema_is_virtual_metal_not_metal_proof() {
    let rec = run_campaign(7, 5);
    assert_eq!(rec.schema, CAMPAIGN_SCHEMA);
    assert!(!rec.hardware_present);
    assert_eq!(rec.evidence_status, EVIDENCE_STATUS);
    assert_ne!(rec.schema, realityos_metal::PROOF_SCHEMA);
    let raw = serde_json::to_value(&rec).unwrap();
    assert_eq!(raw["hardware_present"], false);
    assert_ne!(raw["verdict"], "MEASURED");
    if rec.n_fail == 0 {
        assert_eq!(rec.verdict, VERDICT_PASS);
    }
    let mut rec = rec;
    assert!(rec.try_set_hardware_present(true).is_err());
    assert!(!rec.hardware_present);
}

#[test]
fn is_sim_harness_false_still_cannot_mint_measured() {
    use realityos_plant::HardwareDriverPort;
    use realityos_virtual_metal::{VirtualMetalPort, VirtualXl330};
    use std::sync::{Arc, Mutex};

    let d = Arc::new(Mutex::new(VirtualXl330::xl330_m288()));
    let port = VirtualMetalPort::new(d);
    assert!(
        !port.is_sim_harness(),
        "Virtual Metal is a device surrogate, not the plant HardwareDriver harness"
    );
    let id = port.probe_identity();
    assert!(!id.metal);
    assert_eq!(id.evidence_status, EVIDENCE_STATUS);
    assert!(id.evidence_status.starts_with("SIM_"));
    assert!(HonestyStamp::sim("MEASURED").is_err());
}
