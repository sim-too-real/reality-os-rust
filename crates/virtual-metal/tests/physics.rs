//! Time-stepped envelope surrogate. Goal write must not teleport present.

use realityos_metal::protocol::{
    encode_write, ADDR_GOAL_POSITION, ADDR_GOAL_PWM, ADDR_TORQUE_ENABLE,
};
use realityos_virtual_metal::physics::{no_load_speed_rpm_at, rpm_to_rad_s, ticks_to_rad};
use realityos_virtual_metal::{VirtualXl330, Xl330TruthPack};

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
