//! Constants audit artifact must classify every production-relevant XL330 field.

use serde_json::Value;

fn audit() -> Value {
    let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../docs/virtual_metal/constants_audit.json");
    let raw = std::fs::read_to_string(&p).unwrap_or_else(|e| panic!("{}: {e}", p.display()));
    serde_json::from_str(&raw).expect("constants_audit json")
}

#[test]
fn every_listed_constant_has_a_classification() {
    let v = audit();
    let items = v["items"].as_array().expect("items");
    assert!(items.len() >= 16);
    let mut saw_vel_p = false;
    let mut saw_volt_bit = false;
    for it in items {
        let class = it["classification"].as_str().expect("classification");
        assert!(
            matches!(
                class,
                "CONFIRMED"
                    | "CONFIRMED_STALE_CODE"
                    | "DOCUMENTATION_CONFLICT"
                    | "FIRMWARE_DEPENDENT"
                    | "MODEL_DEPENDENT"
                    | "UNKNOWN"
            ),
            "bad class {class}"
        );
        let name = it["constant"].as_str().unwrap_or("");
        if name.starts_with("Velocity P Gain") {
            saw_vel_p = true;
            assert_eq!(class, "CONFIRMED_STALE_CODE");
            assert_eq!(it["e_manual"], 180);
            assert_eq!(it["production_code"], 180);
            assert_eq!(it["virtual_metal"], 180);
        }
        if name.contains("Input Voltage bit") {
            saw_volt_bit = true;
            assert_eq!(class, "DOCUMENTATION_CONFLICT");
        }
    }
    assert!(saw_vel_p);
    assert!(saw_volt_bit);
}

#[test]
fn production_factory_velocity_p_gain_is_180() {
    assert_eq!(realityos_metal::protocol::FACTORY_VELOCITY_P_GAIN, 180);
}

#[test]
fn unknowns_ledger_has_required_fields() {
    let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../docs/virtual_metal/unknowns.json");
    let v: Value = serde_json::from_str(&std::fs::read_to_string(p).unwrap()).unwrap();
    for it in v["items"].as_array().unwrap() {
        for k in [
            "property",
            "why_it_matters",
            "public_data_status",
            "current_virtual_assumption",
            "confidence",
            "dependent_invariant",
            "smallest_real_experiment",
        ] {
            assert!(
                it.get(k).and_then(|x| x.as_str()).map(|s| !s.is_empty()) == Some(true),
                "missing {k} in {it}"
            );
        }
    }
}
