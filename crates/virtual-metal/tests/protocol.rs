//! Drive shipped Protocol 2.0 encode/decode through the virtual device.

use realityos_metal::protocol::{
    decode_instruction, decode_status, encode_ping, encode_read, encode_reboot, encode_status,
    encode_write, instruction_ok, ADDR_GOAL_POSITION, ADDR_GOAL_PWM, ADDR_ID,
    ADDR_MAX_POSITION_LIMIT, ADDR_MIN_POSITION_LIMIT, ADDR_MODEL_NUMBER, ADDR_TORQUE_ENABLE,
    BROADCAST_ID, ERR_ACCESS, ERR_DATA_LIMIT, HEADER, INST_PING, INST_REBOOT, INST_STATUS,
    INST_WRITE, STATUS_ALERT, XL330_M288_MODEL,
};
use std::cell::RefCell;
use std::rc::Rc;

use realityos_plant::HardwareDriverPort;
use realityos_virtual_metal::{VirtualMetalPort, VirtualXl330};

#[test]
fn header_length_crc_stuffing_and_instructions() {
    let pkt = encode_ping(1);
    assert_eq!(&pkt[0..4], &HEADER);
    assert_eq!(pkt[7], INST_PING);
    let inst = decode_instruction(&pkt).unwrap();
    assert_eq!(inst.id, 1);
    assert_eq!(inst.instruction, INST_PING);
    let length = u16::from_le_bytes([pkt[5], pkt[6]]);
    assert_eq!(length, 3);

    let stuffed = encode_write(1, ADDR_GOAL_POSITION, &[0xFF, 0xFF, 0xFD, 0x00]);
    assert!(stuffed.windows(4).any(|w| w == [0xFF, 0xFF, 0xFD, 0xFD]));
    let w = decode_instruction(&stuffed).unwrap();
    assert_eq!(w.instruction, INST_WRITE);

    let mut d = VirtualXl330::xl330_m288();
    let st = decode_status(&d.process(&encode_ping(1))).unwrap();
    assert_eq!(st.id, 1);
    assert_eq!(st.error & !STATUS_ALERT, 0);
    assert_eq!(
        u16::from_le_bytes([st.params[0], st.params[1]]),
        XL330_M288_MODEL
    );

    let read = d.process(&encode_read(1, ADDR_MODEL_NUMBER, 2));
    let rst = decode_status(&read).unwrap();
    assert!(instruction_ok(rst.error));
    assert_eq!(u16::from_le_bytes([rst.params[0], rst.params[1]]), 1200);

    assert!(decode_status(&d.process(&encode_write(1, ADDR_TORQUE_ENABLE, &[1]))).is_ok());
    let g = 2048i32.to_le_bytes();
    let wst = decode_status(&d.process(&encode_write(1, ADDR_GOAL_POSITION, &g))).unwrap();
    assert!(instruction_ok(wst.error));
    assert_eq!(d.goal_position(), 2048);
    assert_eq!(d.physical_action_count(), 1);

    let reboot = encode_reboot(1);
    assert_eq!(reboot[7], INST_REBOOT);
    let _ = d.process(&reboot);
    assert!(!d.torque_enabled());
}

#[test]
fn torque_on_applies_pending_goal_and_counts_physical_only_then() {
    let mut d = VirtualXl330::xl330_m288();
    let start = d.present_position();
    assert_ne!(start, 3000);
    assert!(!d.torque_enabled());
    let wst =
        decode_status(&d.process(&encode_write(1, ADDR_GOAL_POSITION, &3000i32.to_le_bytes())))
            .unwrap();
    assert!(instruction_ok(wst.error));
    assert_eq!(d.goal_position(), 3000);
    assert_eq!(d.present_position(), start);
    assert_eq!(d.physical_action_count(), 0);
    let on = decode_status(&d.process(&encode_write(1, ADDR_TORQUE_ENABLE, &[1]))).unwrap();
    assert!(instruction_ok(on.error));
    assert!(d.torque_enabled());
    assert_eq!(d.present_position(), 3000);
    assert_eq!(d.physical_action_count(), 1);
}

#[test]
fn read_sensor_reads_present_voltage_not_datasheet_constant() {
    let d = Rc::new(RefCell::new(VirtualXl330::xl330_m288()));
    let mut port = VirtualMetalPort::new(d.clone());
    let pkt = port.read_sensor(0.0).expect("5.0 V is in range");
    let vin = pkt
        .samples
        .iter()
        .find(|(k, _)| k == "vin_v")
        .map(|(_, v)| *v)
        .expect("vin_v sample");
    assert!((vin - 5.0).abs() < 1e-9, "got vin_v={vin}");
    d.borrow_mut().set_voltage_v(4.0);
    let pkt = port.read_sensor(1.0).expect("4.0 V is in range");
    let vin = pkt
        .samples
        .iter()
        .find(|(k, _)| k == "vin_v")
        .map(|(_, v)| *v)
        .expect("vin_v sample");
    assert!(
        (vin - 4.0).abs() < 1e-9,
        "read_sensor must READ present voltage, not datasheet 5.0; got {vin}"
    );
    d.borrow_mut().set_voltage_v(2.0);
    let err = port.read_sensor(2.0);
    assert!(
        err.is_err(),
        "2.0 V must not succeed as healthy 5.0 V: {err:?}"
    );
    let msg = err.unwrap_err().to_string();
    assert!(
        msg.contains("dxl_vin_outside_wizard_limits") || msg.contains("dxl_vin_unreadable"),
        "unexpected refuse: {msg}"
    );
    assert!(!msg.contains("5.0"));
}

#[test]
fn id_broadcast_and_status_alert_bits() {
    let mut d = VirtualXl330::xl330_m288();
    assert!(d.process(&encode_ping(2)).is_empty());
    let b = decode_status(&d.process(&encode_ping(BROADCAST_ID))).unwrap();
    assert_eq!(b.id, 1);
    assert_eq!(b.params.len(), 3);
    let none = d.process(&encode_write(BROADCAST_ID, ADDR_TORQUE_ENABLE, &[0]));
    assert!(none.is_empty());

    d.set_temperature_c(90.0);
    let st = decode_status(&d.process(&encode_ping(1))).unwrap();
    assert_eq!(st.error & STATUS_ALERT, STATUS_ALERT);
    assert!(instruction_ok(STATUS_ALERT));
    assert!(!instruction_ok(ERR_ACCESS));
}

#[test]
fn control_table_relationships() {
    let mut d = VirtualXl330::xl330_m288();
    assert!(instruction_ok(
        decode_status(&d.process(&encode_write(1, ADDR_TORQUE_ENABLE, &[1])))
            .unwrap()
            .error
    ));
    let id_write = decode_status(&d.process(&encode_write(1, ADDR_ID, &[2]))).unwrap();
    assert_eq!(id_write.error & !STATUS_ALERT, ERR_ACCESS);
    assert_eq!(d.id(), 1);

    assert!(instruction_ok(
        decode_status(&d.process(&encode_write(1, ADDR_TORQUE_ENABLE, &[0])))
            .unwrap()
            .error
    ));
    assert!(instruction_ok(
        decode_status(&d.process(&encode_write(1, ADDR_ID, &[2])))
            .unwrap()
            .error
    ));
    assert_eq!(d.id(), 2);
    assert!(instruction_ok(
        decode_status(&d.process(&encode_write(2, ADDR_ID, &[1])))
            .unwrap()
            .error
    ));

    let eeprom_id = d.id();
    let _ = d.process(&encode_write(eeprom_id, ADDR_TORQUE_ENABLE, &[1]));
    let _ = d.process(&encode_write(
        eeprom_id,
        ADDR_GOAL_POSITION,
        &100i32.to_le_bytes(),
    ));
    let _ = d.process(&encode_reboot(eeprom_id));
    assert!(!d.torque_enabled());
    assert_eq!(d.id(), eeprom_id);

    d.trip_watchdog();
    let wd = decode_status(&d.process(&encode_write(
        d.id(),
        ADDR_GOAL_POSITION,
        &200i32.to_le_bytes(),
    )))
    .unwrap();
    assert_eq!(wd.error & !STATUS_ALERT, ERR_ACCESS);

    let mut d = VirtualXl330::xl330_m288();
    let _ = d.process(&encode_write(
        1,
        ADDR_MIN_POSITION_LIMIT,
        &100i32.to_le_bytes(),
    ));
    let _ = d.process(&encode_write(
        1,
        ADDR_MAX_POSITION_LIMIT,
        &200i32.to_le_bytes(),
    ));
    let lim =
        decode_status(&d.process(&encode_write(1, ADDR_GOAL_POSITION, &300i32.to_le_bytes())))
            .unwrap();
    assert_eq!(lim.error & !STATUS_ALERT, ERR_DATA_LIMIT);

    let pwm =
        decode_status(&d.process(&encode_write(1, ADDR_GOAL_PWM, &900i16.to_le_bytes()))).unwrap();
    assert_eq!(pwm.error & !STATUS_ALERT, ERR_DATA_LIMIT);

    d.set_voltage_v(2.0);
    let ping = decode_status(&d.process(&encode_ping(1))).unwrap();
    assert_eq!(ping.error & STATUS_ALERT, STATUS_ALERT);
    assert!(!d.torque_enabled() || d.hardware_error() != 0);
}

#[test]
fn python_pty_echo_is_not_virtual_metal_v1() {
    let py =
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../metal/tests/xl330_responder.py");
    let src = std::fs::read_to_string(py).expect("python stand-in");
    assert!(src.contains("Not metal"));
    assert!(!src.contains("Virtual Metal V1"));
}

#[test]
fn emanual_status_roundtrip_on_virtual_device() {
    let status = encode_status(1, 0, &[0x06, 0x04, 0x26]);
    assert_eq!(status[7], INST_STATUS);
    let st = decode_status(&status).unwrap();
    assert_eq!(st.params, vec![0x06, 0x04, 0x26]);
    let mut d = VirtualXl330::xl330_m288();
    let live = d.process(&encode_ping(1));
    assert_eq!(&live[0..4], &HEADER);
    let got = decode_status(&live).unwrap();
    assert_eq!(got.id, 1);
}
