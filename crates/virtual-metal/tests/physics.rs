//! Time-stepped envelope surrogate. Goal write must not teleport present.

use realityos_metal::protocol::{
    encode_write, ADDR_GOAL_POSITION, ADDR_GOAL_PWM, ADDR_TORQUE_ENABLE,
};
use realityos_virtual_metal::physics::{no_load_speed_rpm_at, rpm_to_rad_s, ticks_to_rad};
use realityos_virtual_metal::{VirtualXl330, Xl330TruthPack};

#[test]
fn from_seed_hits_both_ends_of_truth_pack_ranges() {
    let pack = Xl330TruthPack::xl330_m288();
    let v_lo = pack.input_voltage_min_v.value.expect("min V");
    let v_hi = pack.input_voltage_max_v.value.expect("max V");
    let (eff_lo, eff_hi) = pack.gearbox_efficiency.range.expect("gearbox range");
    let (boot_lo, boot_hi) = pack.boot_delay_s.range.expect("boot range");
    let mut v_min = f64::INFINITY;
    let mut v_max = f64::NEG_INFINITY;
    let mut e_min = f64::INFINITY;
    let mut e_max = f64::NEG_INFINITY;
    let mut b_min = f64::INFINITY;
    let mut b_max = f64::NEG_INFINITY;
    for seed in 0..4096u64 {
        let d = VirtualXl330::from_seed(seed, pack.clone());
        v_min = v_min.min(d.voltage_v());
        v_max = v_max.max(d.voltage_v());
        e_min = e_min.min(d.gearbox_efficiency());
        e_max = e_max.max(d.gearbox_efficiency());
        b_min = b_min.min(d.boot_latency_s());
        b_max = b_max.max(d.boot_latency_s());
    }
    let span_v = v_hi - v_lo;
    let span_e = eff_hi - eff_lo;
    let span_b = boot_hi - boot_lo;
    assert!(
        v_min <= v_lo + 0.1 * span_v,
        "from_seed never approached min voltage {v_lo}: min={v_min} max={v_max}"
    );
    assert!(
        v_max >= v_hi - 0.1 * span_v,
        "from_seed never approached max voltage {v_hi}: min={v_min} max={v_max}"
    );
    assert!(
        e_min <= eff_lo + 0.1 * span_e,
        "from_seed never approached min gearbox_efficiency {eff_lo}: min={e_min} max={e_max}"
    );
    assert!(
        e_max >= eff_hi - 0.1 * span_e,
        "from_seed never approached max gearbox_efficiency {eff_hi}: min={e_min} max={e_max}"
    );
    assert!(
        b_min <= boot_lo + 0.1 * span_b,
        "from_seed never approached min boot_delay {boot_lo}: min={b_min} max={b_max}"
    );
    assert!(
        b_max >= boot_hi - 0.1 * span_b,
        "from_seed never approached max boot_delay {boot_hi}: min={b_min} max={b_max}"
    );
}

#[test]
fn goal_write_stores_target_without_teleport() {
    let mut d = VirtualXl330::xl330_m288();
    let start = d.present_position();
    assert_ne!(start, 3000);
    let _ = d.process(&encode_write(1, ADDR_TORQUE_ENABLE, &[1]));
    let _ = d.process(&encode_write(1, ADDR_GOAL_POSITION, &3000i32.to_le_bytes()));
    assert_eq!(d.goal_position(), 3000);
    assert_eq!(
        d.present_position(),
        start,
        "present must not teleport to goal without advance"
    );
}

#[test]
fn advance_evolves_under_no_load_envelope_and_settles() {
    let mut d = VirtualXl330::xl330_m288();
    let start = d.present_position();
    let _ = d.process(&encode_write(1, ADDR_TORQUE_ENABLE, &[1]));
    let goal = start + 32;
    let _ = d.process(&encode_write(1, ADDR_GOAL_POSITION, &goal.to_le_bytes()));
    d.advance(0.001);
    let mid = d.present_position();
    assert_ne!(mid, start, "1 ms at no-load must move some ticks");
    assert_ne!(mid, goal, "1 ms must not finish a 32-tick step at teleport");
    d.advance(0.05);
    assert_eq!(d.present_position(), goal);
}

#[test]
fn never_exceeds_no_load_speed() {
    let pack = Xl330TruthPack::xl330_m288();
    let noload_rpm = no_load_speed_rpm_at(&pack, 5.0).unwrap();
    let vmax = rpm_to_rad_s(noload_rpm);
    let mut d = VirtualXl330::xl330_m288();
    let _ = d.process(&encode_write(1, ADDR_TORQUE_ENABLE, &[1]));
    let _ = d.process(&encode_write(1, ADDR_GOAL_POSITION, &4095i32.to_le_bytes()));
    let dt = 0.002;
    let before = ticks_to_rad(d.present_position(), 4096);
    d.advance(dt);
    let after = ticks_to_rad(d.present_position(), 4096);
    let speed = (after - before).abs() / dt;
    assert!(
        speed <= vmax * 1.02 + 1e-9,
        "speed {speed} rad/s exceeds no-load {vmax}"
    );
}

#[test]
fn pwm_zero_does_not_move() {
    let mut d = VirtualXl330::xl330_m288();
    let start = d.present_position();
    let _ = d.process(&encode_write(1, ADDR_TORQUE_ENABLE, &[1]));
    let _ = d.process(&encode_write(1, ADDR_GOAL_PWM, &0i16.to_le_bytes()));
    let _ = d.process(&encode_write(
        1,
        ADDR_GOAL_POSITION,
        &(start + 32).to_le_bytes(),
    ));
    d.advance(0.05);
    assert_eq!(d.present_position(), start);
}

#[test]
fn stall_load_blocks_motion() {
    let mut d = VirtualXl330::xl330_m288();
    let start = d.present_position();
    d.set_load_torque_nm(10.0);
    let _ = d.process(&encode_write(1, ADDR_TORQUE_ENABLE, &[1]));
    let _ = d.process(&encode_write(
        1,
        ADDR_GOAL_POSITION,
        &(start + 32).clamp(0, 4095).to_le_bytes(),
    ));
    d.advance(0.05);
    assert_eq!(d.present_position(), start);
}

#[test]
fn sampled_voltage_changes_distance_in_fixed_dt() {
    let mut slow = VirtualXl330::from_pack(Xl330TruthPack::xl330_m288(), 3.7, 2000);
    let mut fast = VirtualXl330::from_pack(Xl330TruthPack::xl330_m288(), 6.0, 2000);
    for d in [&mut slow, &mut fast] {
        let _ = d.process(&encode_write(1, ADDR_TORQUE_ENABLE, &[1]));
        let _ = d.process(&encode_write(1, ADDR_GOAL_POSITION, &2400i32.to_le_bytes()));
        d.advance(0.01);
    }
    assert_ne!(
        slow.present_position(),
        fast.present_position(),
        "voltage must change execution"
    );
}

#[test]
fn gearbox_efficiency_range_changes_execution() {
    let mut a = VirtualXl330::from_pack(Xl330TruthPack::xl330_m288(), 5.0, 2000);
    let mut b = VirtualXl330::from_pack(Xl330TruthPack::xl330_m288(), 5.0, 2000);
    a.set_gearbox_efficiency(0.4);
    b.set_gearbox_efficiency(0.85);
    for d in [&mut a, &mut b] {
        let _ = d.process(&encode_write(1, ADDR_TORQUE_ENABLE, &[1]));
        let _ = d.process(&encode_write(1, ADDR_GOAL_POSITION, &2400i32.to_le_bytes()));
        d.advance(0.01);
    }
    assert_ne!(a.present_position(), b.present_position());
}

#[test]
fn position_limits_clamp() {
    let mut d = VirtualXl330::from_pack(Xl330TruthPack::xl330_m288(), 5.0, 2048);
    let _ = d.process(&encode_write(
        1,
        realityos_metal::protocol::ADDR_MAX_POSITION_LIMIT,
        &2100i32.to_le_bytes(),
    ));
    let _ = d.process(&encode_write(
        1,
        realityos_metal::protocol::ADDR_MIN_POSITION_LIMIT,
        &2000i32.to_le_bytes(),
    ));
    let _ = d.process(&encode_write(1, ADDR_TORQUE_ENABLE, &[1]));
    let _ = d.process(&encode_write(1, ADDR_GOAL_POSITION, &2100i32.to_le_bytes()));
    d.advance(1.0);
    assert!(d.present_position() <= 2100);
    assert!(d.present_position() >= 2000);
}
