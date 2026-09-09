//! Dynamixel Protocol 2.0 encode/decode. Transport only. No authority policy.

use std::fmt;

pub const HEADER: [u8; 4] = [0xFF, 0xFF, 0xFD, 0x00];
pub const INST_PING: u8 = 0x01;
pub const INST_READ: u8 = 0x02;
pub const INST_WRITE: u8 = 0x03;
pub const INST_REBOOT: u8 = 0x08;
pub const INST_STATUS: u8 = 0x55;
/// Protocol 2.0 broadcast. Status replies carry the servo's own ID.
pub const BROADCAST_ID: u8 = 254;
/// Protocol 2.0 bit 7: Hardware Error Status is latched. The instruction still completed.
pub const STATUS_ALERT: u8 = 0x80;

pub const XL330_M288_MODEL: u16 = 1190;
pub const XL330_M077_MODEL: u16 = 1200;

pub const ADDR_MODEL_NUMBER: u16 = 0;
pub const ADDR_FIRMWARE_VERSION: u16 = 6;
pub const ADDR_ID: u16 = 7;
/// EEPROM. Bit2=1 is time-based profile (Wizard); 0 = velocity-based.
pub const ADDR_DRIVE_MODE: u16 = 10;
pub const DRIVE_MODE_VELOCITY_BASED: u8 = 0;
/// EEPROM. 3 = position control (factory XL330 default).
pub const ADDR_OPERATING_MODE: u16 = 11;
pub const OPERATING_MODE_POSITION: u8 = 3;
/// EEPROM. 255 = disabled. Wizard can make one servo answer a second ID.
pub const ADDR_SECONDARY_ID: u16 = 12;
pub const SECONDARY_ID_DISABLED: u8 = 255;
/// EEPROM. Signed. Wizard "zero the horn" shifts Present outside 0–4095.
pub const ADDR_HOMING_OFFSET: u16 = 20;
/// EEPROM. Unit ≈ 0.229 rpm. Moving=1 only while |Present Velocity| > this.
/// Factory 10. Wizard ≥ profile velocity keeps Moving=0 for the whole nudge.
pub const ADDR_MOVING_THRESHOLD: u16 = 24;
pub const FACTORY_MOVING_THRESHOLD: u32 = 10;
/// EEPROM. Unit 0.1 V. Factory XL330 max 70 / min 35.
pub const ADDR_MAX_VOLTAGE_LIMIT: u16 = 32;
pub const ADDR_MIN_VOLTAGE_LIMIT: u16 = 34;
/// EEPROM. Unit 0.113%. Factory 885. Wizard 0 produces no PWM output.
pub const ADDR_PWM_LIMIT: u16 = 36;
pub const FACTORY_PWM_LIMIT: u16 = 885;
/// Below this, a 32-tick no-load step will not move present.
pub const MIN_PWM_LIMIT: u16 = 80;
pub const ADDR_CURRENT_LIMIT: u16 = 38;
/// EEPROM. Unit ≈ 0.229 rpm. 0 or 1 makes a 32-tick nudge still Moving=0 at the old present.
pub const ADDR_VELOCITY_LIMIT: u16 = 44;
/// EEPROM. Factory max 4095 / min 0. Wizard can shrink this window.
pub const ADDR_MAX_POSITION_LIMIT: u16 = 48;
pub const ADDR_MIN_POSITION_LIMIT: u16 = 52;
pub const ADDR_TORQUE_ENABLE: u16 = 64;
/// RAM. 0 = no status except PING (Wizard); 2 = all instructions (factory).
pub const ADDR_STATUS_RETURN_LEVEL: u16 = 68;
pub const STATUS_RETURN_ALL: u8 = 2;
pub const ADDR_HARDWARE_ERROR: u16 = 70;
/// RAM. Factory 1600. Wizard 0 leaves profile following with a dead I-term.
pub const ADDR_VELOCITY_I_GAIN: u16 = 76;
pub const FACTORY_VELOCITY_I_GAIN: u16 = 1600;
pub const MIN_VELOCITY_I_GAIN: u16 = 200;
/// RAM. Factory 100. Wizard 0 means the profile velocity loop does not track.
pub const ADDR_VELOCITY_P_GAIN: u16 = 78;
pub const FACTORY_VELOCITY_P_GAIN: u16 = 100;
pub const MIN_VELOCITY_P_GAIN: u16 = 20;
/// RAM. Factory 400. Wizard 0 means the servo never tracks a goal.
pub const ADDR_POSITION_P_GAIN: u16 = 84;
pub const FACTORY_POSITION_P_GAIN: u16 = 400;
/// Below this, a 32-tick nudge will not finish before the campaign Moving wait.
pub const MIN_POSITION_P_GAIN: u16 = 80;
/// RAM. Unit 20 ms. 0 = off; 0xFF (-1) = tripped (goal registers read-only).
pub const ADDR_BUS_WATCHDOG: u16 = 98;
pub const ADDR_PROFILE_ACCEL: u16 = 108;
pub const ADDR_PROFILE_VELOCITY: u16 = 112;
pub const ADDR_GOAL_POSITION: u16 = 116;
pub const ADDR_PRESENT_CURRENT: u16 = 126;
pub const ADDR_PRESENT_VELOCITY: u16 = 128;
pub const ADDR_PRESENT_POSITION: u16 = 132;
pub const ADDR_PRESENT_VOLTAGE: u16 = 144;
pub const ADDR_REALTIME_TICK: u16 = 120;
/// RAM. 1 while |Present Velocity| > Moving Threshold (addr 24). That is
/// not "arrived": accel below the threshold leaves Moving=0 at the old present.
pub const ADDR_MOVING: u16 = 122;

const CRC_TABLE: [u16; 256] = [
    0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011, 0x8033, 0x0036, 0x003C, 0x8039,
    0x0028, 0x802D, 0x8027, 0x0022, 0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
    0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041, 0x80C3, 0x00C6, 0x00CC, 0x80C9,
    0x00D8, 0x80DD, 0x80D7, 0x00D2, 0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
    0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1, 0x8093, 0x0096, 0x009C, 0x8099,
    0x0088, 0x808D, 0x8087, 0x0082, 0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
    0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1, 0x01E0, 0x81E5, 0x81EF, 0x01EA,
    0x81FB, 0x01FE, 0x01F4, 0x81F1, 0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
    0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151, 0x8173, 0x0176, 0x017C, 0x8179,
    0x0168, 0x816D, 0x8167, 0x0162, 0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
    0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101, 0x8303, 0x0306, 0x030C, 0x8309,
    0x0318, 0x831D, 0x8317, 0x0312, 0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
    0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371, 0x8353, 0x0356, 0x035C, 0x8359,
    0x0348, 0x834D, 0x8347, 0x0342, 0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
    0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2, 0x83A3, 0x03A6, 0x03AC, 0x83A9,
    0x03B8, 0x83BD, 0x83B7, 0x03B2, 0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
    0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291, 0x82B3, 0x02B6, 0x02BC, 0x82B9,
    0x02A8, 0x82AD, 0x82A7, 0x02A2, 0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
    0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1, 0x8243, 0x0246, 0x024C, 0x8249,
    0x0258, 0x825D, 0x8257, 0x0252, 0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
    0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231, 0x8213, 0x0216, 0x021C, 0x8219,
    0x0208, 0x820D, 0x8207, 0x0202,
];

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StatusPacket {
    pub id: u8,
    pub error: u8,
    pub params: Vec<u8>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ProtocolError {
    TooShort,
    BadHeader,
    BadCrc,
    BadInstruction,
    Truncated,
}

impl fmt::Display for ProtocolError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::TooShort => f.write_str("dxl_too_short"),
            Self::BadHeader => f.write_str("dxl_bad_header"),
            Self::BadCrc => f.write_str("dxl_bad_crc"),
            Self::BadInstruction => f.write_str("dxl_bad_instruction"),
            Self::Truncated => f.write_str("dxl_truncated"),
        }
    }
}

impl std::error::Error for ProtocolError {}

pub fn crc16(data: &[u8]) -> u16 {
    let mut crc: u16 = 0;
    for &b in data {
        let i = ((crc >> 8) ^ u16::from(b)) & 0xFF;
        crc = (crc << 8) ^ CRC_TABLE[i as usize];
    }
    crc
}

pub fn is_xl330_model(model: u16) -> bool {
    model == XL330_M288_MODEL || model == XL330_M077_MODEL
}

pub fn encode_packet(id: u8, inst: u8, params: &[u8]) -> Vec<u8> {
    let length = u16::try_from(params.len() + 3).unwrap_or(u16::MAX);
    let mut unstuffed = Vec::with_capacity(10 + params.len());
    unstuffed.extend_from_slice(&HEADER);
    unstuffed.push(id);
    unstuffed.extend_from_slice(&length.to_le_bytes());
    unstuffed.push(inst);
    unstuffed.extend_from_slice(params);
    let crc = crc16(&unstuffed);
    unstuffed.extend_from_slice(&crc.to_le_bytes());
    stuff(&unstuffed)
}

pub fn encode_ping(id: u8) -> Vec<u8> {
    encode_packet(id, INST_PING, &[])
}

pub fn encode_reboot(id: u8) -> Vec<u8> {
    encode_packet(id, INST_REBOOT, &[])
}

/// Instruction completed. `STATUS_ALERT` is leftover hardware-error state, not a NAK.
pub fn instruction_ok(error: u8) -> bool {
    error & !STATUS_ALERT == 0
}

pub fn encode_read(id: u8, addr: u16, len: u16) -> Vec<u8> {
    let mut p = Vec::with_capacity(4);
    p.extend_from_slice(&addr.to_le_bytes());
    p.extend_from_slice(&len.to_le_bytes());
    encode_packet(id, INST_READ, &p)
}

pub fn encode_write(id: u8, addr: u16, data: &[u8]) -> Vec<u8> {
    let mut p = Vec::with_capacity(2 + data.len());
    p.extend_from_slice(&addr.to_le_bytes());
    p.extend_from_slice(data);
    encode_packet(id, INST_WRITE, &p)
}

pub fn decode_status(buf: &[u8]) -> Result<StatusPacket, ProtocolError> {
    let destuffed = destuff(buf)?;
    if destuffed.len() < 11 {
        return Err(ProtocolError::TooShort);
    }
    if destuffed[0..4] != HEADER {
        return Err(ProtocolError::BadHeader);
    }
    let length = u16::from_le_bytes([destuffed[5], destuffed[6]]) as usize;
    let need = 7 + length;
    if destuffed.len() < need {
        return Err(ProtocolError::Truncated);
    }
    let body = &destuffed[..need - 2];
    let got = u16::from_le_bytes([destuffed[need - 2], destuffed[need - 1]]);
    if crc16(body) != got {
        return Err(ProtocolError::BadCrc);
    }
    let inst = destuffed[7];
    if inst != INST_STATUS {
        return Err(ProtocolError::BadInstruction);
    }
    let error = destuffed[8];
    let params = destuffed[9..need - 2].to_vec();
    Ok(StatusPacket {
        id: destuffed[4],
        error,
        params,
    })
}

/// Half-duplex USB-UART adapters often echo the request before the status.
/// Scan past non-status frames instead of treating the echo as the reply.
pub fn decode_status_scan(buf: &[u8]) -> Result<StatusPacket, ProtocolError> {
    let mut search = 0;
    let mut last_err = ProtocolError::BadHeader;
    while search < buf.len() {
        let Some(rel) = find_header(&buf[search..]) else {
            return Err(last_err);
        };
        let start = search + rel;
        match decode_status(&buf[start..]) {
            Ok(st) => return Ok(st),
            Err(ProtocolError::Truncated) | Err(ProtocolError::TooShort) => {
                return Err(ProtocolError::Truncated);
            }
            Err(e) => {
                last_err = e;
                search = start + 1;
            }
        }
    }
    Err(last_err)
}

/// Unique servo IDs from every status frame in `buf`. Broadcast sniff uses
/// this so a second XL330 on the drop is not commanded by accident.
pub fn unique_status_ids(buf: &[u8]) -> Vec<u8> {
    let mut out = Vec::new();
    let mut search = 0;
    while search < buf.len() {
        let Some(rel) = find_header(&buf[search..]) else {
            break;
        };
        let start = search + rel;
        match decode_status(&buf[start..]) {
            Ok(st) => {
                // Protocol 2.0 ID 0 is valid (Wizard). Only 254 is broadcast.
                if st.id != BROADCAST_ID && !out.contains(&st.id) {
                    out.push(st.id);
                }
                search = start + 4;
            }
            Err(ProtocolError::Truncated) | Err(ProtocolError::TooShort) => break,
            Err(_) => search = start + 1,
        }
    }
    out
}

/// Stuff after CRC (Robotis): insert 0xFD after 0xFF 0xFF 0xFD except in the header.
fn stuff(unstuffed: &[u8]) -> Vec<u8> {
    if unstuffed.len() < 4 {
        return unstuffed.to_vec();
    }
    let mut out = unstuffed[..4].to_vec();
    let mut i = 4;
    while i < unstuffed.len() {
        if i + 2 < unstuffed.len()
            && unstuffed[i] == 0xFF
            && unstuffed[i + 1] == 0xFF
            && unstuffed[i + 2] == 0xFD
        {
            out.extend_from_slice(&unstuffed[i..i + 3]);
            out.push(0xFD);
            i += 3;
        } else {
            out.push(unstuffed[i]);
            i += 1;
        }
    }
    out
}

fn destuff(stuffed: &[u8]) -> Result<Vec<u8>, ProtocolError> {
    let start = find_header(stuffed).ok_or(ProtocolError::BadHeader)?;
    let s = &stuffed[start..];
    if s.len() < 4 {
        return Err(ProtocolError::TooShort);
    }
    let mut out = s[..4].to_vec();
    let mut i = 4;
    while i < s.len() {
        if i + 3 < s.len()
            && s[i] == 0xFF
            && s[i + 1] == 0xFF
            && s[i + 2] == 0xFD
            && s[i + 3] == 0xFD
        {
            out.extend_from_slice(&s[i..i + 3]);
            i += 4;
        } else {
            out.push(s[i]);
            i += 1;
        }
    }
    Ok(out)
}

pub fn find_header(buf: &[u8]) -> Option<usize> {
    buf.windows(4).position(|w| w == HEADER)
}

pub fn le_u16(b: &[u8]) -> Option<u16> {
    Some(u16::from_le_bytes([*b.first()?, *b.get(1)?]))
}

pub fn le_u32(b: &[u8]) -> Option<u32> {
    Some(u32::from_le_bytes([
        *b.first()?,
        *b.get(1)?,
        *b.get(2)?,
        *b.get(3)?,
    ]))
}

pub fn le_i32(b: &[u8]) -> Option<i32> {
    Some(i32::from_le_bytes([
        *b.first()?,
        *b.get(1)?,
        *b.get(2)?,
        *b.get(3)?,
    ]))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ping_roundtrip_crc() {
        let pkt = encode_ping(1);
        assert_eq!(pkt.len(), 10);
        assert_eq!(encode_ping(BROADCAST_ID).len(), 10);
        assert_eq!(&pkt[0..4], &HEADER);
        assert_eq!(pkt[4], 1);
        let destuffed = destuff(&pkt).unwrap();
        let crc_got = u16::from_le_bytes([
            destuffed[destuffed.len() - 2],
            destuffed[destuffed.len() - 1],
        ]);
        assert_eq!(crc16(&destuffed[..destuffed.len() - 2]), crc_got);
    }

    #[test]
    fn status_decode_rejects_bad_crc() {
        let mut pkt = encode_packet(1, INST_STATUS, &[0]);
        let n = pkt.len();
        pkt[n - 1] ^= 0xFF;
        assert_eq!(decode_status(&pkt), Err(ProtocolError::BadCrc));
    }

    #[test]
    fn write_read_encode_stable() {
        let w = encode_write(1, ADDR_GOAL_POSITION, &10i32.to_le_bytes());
        let r = encode_read(1, ADDR_PRESENT_POSITION, 4);
        assert!(w.len() > 11);
        assert!(r.len() > 11);
        assert_ne!(w, r);
    }

    #[test]
    fn stuffing_roundtrip_when_body_looks_like_header() {
        let params = [0xFF, 0xFF, 0xFD, 0x01];
        let stuffed = encode_packet(1, INST_WRITE, &params);
        assert!(stuffed.windows(4).any(|w| w == [0xFF, 0xFF, 0xFD, 0xFD]));
        let destuffed = destuff(&stuffed).unwrap();
        assert_eq!(&destuffed[8..12], &params);
        let crc_got = u16::from_le_bytes([
            destuffed[destuffed.len() - 2],
            destuffed[destuffed.len() - 1],
        ]);
        assert_eq!(crc16(&destuffed[..destuffed.len() - 2]), crc_got);
    }

    #[test]
    fn xl330_models() {
        assert!(is_xl330_model(XL330_M288_MODEL));
        assert!(is_xl330_model(XL330_M077_MODEL));
        assert!(!is_xl330_model(1030));
    }

    #[test]
    fn unique_status_ids_collects_two_servos() {
        let a = encode_packet(1, INST_STATUS, &[0]);
        let b = encode_packet(7, INST_STATUS, &[0]);
        let mut both = a;
        both.extend_from_slice(&b);
        assert_eq!(unique_status_ids(&both), vec![1, 7]);
        let zero = encode_packet(0, INST_STATUS, &[0]);
        assert_eq!(unique_status_ids(&zero), vec![0]);
    }

    #[test]
    fn status_scan_skips_request_echo() {
        let request = encode_ping(1);
        let status = encode_packet(1, INST_STATUS, &[0]);
        let mut both = request;
        both.extend_from_slice(&status);
        let got = decode_status_scan(&both).expect("status after echo");
        assert_eq!(got.id, 1);
        assert_eq!(got.error, 0);
        assert!(decode_status(&both).is_err());
    }

    #[test]
    fn alert_bit_is_not_an_instruction_fault() {
        assert!(instruction_ok(0));
        assert!(instruction_ok(STATUS_ALERT));
        assert!(!instruction_ok(0x01));
        assert!(!instruction_ok(STATUS_ALERT | 0x01));
        let reboot = encode_reboot(1);
        assert_eq!(reboot[7], INST_REBOOT);
    }
}
