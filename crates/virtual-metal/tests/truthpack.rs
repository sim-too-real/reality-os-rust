use realityos_virtual_metal::{Provenance, VirtualXl330, Xl330TruthPack, TRUTH_PACK_SCHEMA};

#[test]
fn manufacturer_specified_keeps_provenance_and_units() {
    let p = Xl330TruthPack::xl330_m288();
    assert_eq!(p.schema, TRUTH_PACK_SCHEMA);
    assert_eq!(
        p.stall_torque_5v_nm.provenance,
        Provenance::ManufacturerSpecified
    );
    assert_eq!(p.stall_torque_5v_nm.units, "N.m");
    assert_eq!(p.stall_torque_5v_nm.value, Some(0.52));
    assert!(!p.stall_torque_5v_nm.source.is_empty());
}

#[test]
fn unknown_field_remains_unknown() {
    let p = Xl330TruthPack::xl330_m288();
    assert!(p.backlash_rad.is_unknown());
    assert!(p.motor_resistance_ohm.is_unknown());
    assert!(p.stall_torque_measured_nm.is_unknown());
    assert_eq!(p.backlash_rad.provenance, Provenance::Unknown);
    assert!(p.backlash_rad.value.is_none());
}

#[test]
fn published_5v_envelopes_hold() {
    let d = VirtualXl330::xl330_m288();
    let stall = d.stall_torque_nm_at(5.0).unwrap();
    let noload = d.no_load_speed_rpm_at(5.0).unwrap();
    assert!((stall - 0.52).abs() < 1e-12);
    assert!((noload - 103.0).abs() < 1e-12);
    let p = d.truth_pack();
    assert_eq!(p.model_number.value, Some(1200));
    assert_eq!(p.input_voltage_min_v.value, Some(3.7));
    assert_eq!(p.input_voltage_max_v.value, Some(6.0));
    assert_eq!(p.recommended_voltage_v.value, Some(5.0));
    assert_eq!(p.encoder_pulses_per_rev.value, Some(4096));
    assert_eq!(p.position_mode_min.value, Some(0));
    assert_eq!(p.position_mode_max.value, Some(4095));
    assert_eq!(p.gear_ratio.value, Some(288.4));
    let s37 = d.stall_torque_nm_at(3.7).unwrap();
    let s60 = d.stall_torque_nm_at(6.0).unwrap();
    assert!((s37 - 0.42).abs() < 1e-12);
    assert!((s60 - 0.60).abs() < 1e-12);
}

#[test]
fn estimated_and_manufacturer_coexist_without_silent_constants() {
    let p = Xl330TruthPack::xl330_m288();
    assert_eq!(p.gearbox_efficiency.provenance, Provenance::Estimated);
    assert!(p.gearbox_efficiency.range.is_some());
    assert!(p.gearbox_efficiency.value.is_none());
    assert_eq!(
        p.stall_torque_5v_nm.provenance,
        Provenance::ManufacturerSpecified
    );
    assert!(p.motor_kt_nm_per_a.is_unknown());
}
