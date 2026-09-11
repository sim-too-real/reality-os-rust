#!/usr/bin/env python3
"""Protocol 2.0 XL330 stand-in on a PTY. Echoes each request (half-duplex). Not metal."""

from __future__ import annotations

import os
import pty
import signal
import struct
import sys
import termios
import time
import tty

# Host crash_if after write_all+flush closes the slave while we emit status.
# Default SIGPIPE would kill this stand-in and destroy the pts node.
signal.signal(signal.SIGPIPE, signal.SIG_IGN)

HEADER = bytes([0xFF, 0xFF, 0xFD, 0x00])
INST_PING, INST_READ, INST_WRITE, INST_REBOOT, INST_STATUS = 0x01, 0x02, 0x03, 0x08, 0x55
STATUS_ALERT = 0x80

CRC_TABLE = [
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
]


def crc16(data: bytes) -> int:
    crc = 0
    for b in data:
        i = ((crc >> 8) ^ b) & 0xFF
        crc = ((crc << 8) ^ CRC_TABLE[i]) & 0xFFFF
    return crc


def stuff(unstuffed: bytes) -> bytes:
    out = bytearray(unstuffed[:4])
    i = 4
    while i < len(unstuffed):
        if (
            i + 2 < len(unstuffed)
            and unstuffed[i] == 0xFF
            and unstuffed[i + 1] == 0xFF
            and unstuffed[i + 2] == 0xFD
        ):
            out.extend(unstuffed[i : i + 3])
            out.append(0xFD)
            i += 3
        else:
            out.append(unstuffed[i])
            i += 1
    return bytes(out)


def destuff(stuffed: bytes) -> bytes | None:
    start = stuffed.find(HEADER)
    if start < 0:
        return None
    s = stuffed[start:]
    out = bytearray(s[:4])
    i = 4
    while i < len(s):
        if (
            i + 3 < len(s)
            and s[i] == 0xFF
            and s[i + 1] == 0xFF
            and s[i + 2] == 0xFD
            and s[i + 3] == 0xFD
        ):
            out.extend(s[i : i + 3])
            i += 4
        else:
            out.append(s[i])
            i += 1
    return bytes(out)


def encode_status(servo_id: int, params: bytes, error: int = 0) -> bytes:
    payload = bytes([error]) + params
    length = len(payload) + 3
    unstuffed = HEADER + bytes([servo_id]) + struct.pack("<H", length) + bytes([INST_STATUS]) + payload
    unstuffed += struct.pack("<H", crc16(unstuffed))
    return stuff(unstuffed)


def parse_request(buf: bytes) -> tuple[int, int, bytes, int] | None:
    destuffed = destuff(buf)
    # Ping is 10 bytes (header+id+len+inst+crc). A 11-byte floor left PING
    # unparsed until a later READ/WRITE arrived, so a single broadcast PING
    # got no status and Wizard ID discovery missed.
    if destuffed is None or len(destuffed) < 10:
        return None
    length = struct.unpack_from("<H", destuffed, 5)[0]
    need = 7 + length
    if len(destuffed) < need:
        return None
    body = destuffed[: need - 2]
    got = struct.unpack_from("<H", destuffed, need - 2)[0]
    if crc16(body) != got:
        return None
    servo_id = destuffed[4]
    inst = destuffed[7]
    params = destuffed[8 : need - 2]
    start = buf.find(HEADER)
    return servo_id, inst, params, start + need


def init_regs() -> bytearray:
    regs = bytearray(256)
    regs[0:2] = struct.pack("<H", 1200)
    regs[6] = 46
    try:
        own = int(os.environ.get("REALITYOS_METAL_PTY_ID", "1"))
    except ValueError:
        own = 1
    regs[7] = own if own != 254 else 1
    regs[12] = 255
    if os.environ.get("REALITYOS_METAL_PTY_SECONDARY") == "1":
        regs[12] = 7
    regs[13] = 20 if os.environ.get("REALITYOS_METAL_PTY_PROTOCOL_RC") == "1" else 2
    regs[10] = 4 if os.environ.get("REALITYOS_METAL_PTY_TIME_BASED") == "1" else 0
    regs[11] = 3
    regs[31] = 80 if os.environ.get("REALITYOS_METAL_PTY_HOT") == "1" else (
        0 if os.environ.get("REALITYOS_METAL_PTY_ZERO_TEMP_LIMIT") == "1" else 70
    )
    regs[32:34] = struct.pack("<H", 70)
    regs[34:36] = struct.pack("<H", 60 if os.environ.get("REALITYOS_METAL_PTY_HIGH_MINVIN") == "1" else 35)
    pwm = 0 if os.environ.get("REALITYOS_METAL_PTY_ZERO_PWM") == "1" else 885
    regs[36:38] = struct.pack("<H", pwm)
    # Position Mode live limiter. PWM Limit only caps this register.
    # Reboot / mode-switch copies PWM Limit here; Wizard can leave 0.
    if os.environ.get("REALITYOS_METAL_PTY_ZERO_GOAL_PWM") == "1":
        regs[100:102] = struct.pack("<h", 0)
    elif os.environ.get("REALITYOS_METAL_PTY_LOW_GOAL_PWM") == "1":
        regs[100:102] = struct.pack("<h", 1)
    else:
        regs[100:102] = struct.pack("<h", pwm if pwm <= 32767 else 32767)
    regs[38:40] = struct.pack("<H", 200)
    vel = 1 if os.environ.get("REALITYOS_METAL_PTY_SLOW_VEL") == "1" else 445
    regs[44:48] = struct.pack("<I", vel)
    regs[48:52] = struct.pack("<i", 4095)
    regs[52:56] = struct.pack("<i", 0)
    if os.environ.get("REALITYOS_METAL_PTY_AT_MAX") == "1":
        regs[48:52] = struct.pack("<i", 2048)
    if os.environ.get("REALITYOS_METAL_PTY_PRESENT_OUTSIDE") == "1":
        regs[48:52] = struct.pack("<i", 2100)
        regs[52:56] = struct.pack("<i", 2000)
    if os.environ.get("REALITYOS_METAL_PTY_INV_LIMITS") == "1":
        regs[48:52] = struct.pack("<i", 1000)
        regs[52:56] = struct.pack("<i", 3000)
    if os.environ.get("REALITYOS_METAL_PTY_PWM") == "1":
        regs[11] = 16
    if os.environ.get("REALITYOS_METAL_PTY_HW_ERROR") == "1":
        # Latched Hardware Error Status. Reboot clears it and (on XL330)
        # Startup Configuration can re-enable torque; EEPROM then needs torque off.
        regs[70] = 4
        regs[11] = 16
    if os.environ.get("REALITYOS_METAL_PTY_HIGH_P") == "1":
        p_gain = 8000
    else:
        p_gain = 0 if os.environ.get("REALITYOS_METAL_PTY_ZERO_P") == "1" else 400
    regs[84:86] = struct.pack("<H", p_gain)
    if os.environ.get("REALITYOS_METAL_PTY_WIZARD_PID") == "1":
        # Wizard position I/D. Factory is 0. High I/D overshoots a 32-tick step.
        regs[80:82] = struct.pack("<H", 4000)
        regs[82:84] = struct.pack("<H", 4000)
    if os.environ.get("REALITYOS_METAL_PTY_FEEDFORWARD") == "1":
        regs[88:90] = struct.pack("<H", 8000)
        regs[90:92] = struct.pack("<H", 8000)
    vel_p = 0 if os.environ.get("REALITYOS_METAL_PTY_ZERO_VEL_P") == "1" else 100
    regs[78:80] = struct.pack("<H", vel_p)
    vel_i = 0 if os.environ.get("REALITYOS_METAL_PTY_ZERO_VEL_I") == "1" else 1600
    regs[76:78] = struct.pack("<H", vel_i)
    # Default Goal Position is 0 (unset). Present is 2048. Torque-on without
    # syncing goal jumps present — that is the stale-Wizard-goal landmine.
    if os.environ.get("REALITYOS_METAL_PTY_BUS_WATCHDOG") == "1":
        regs[98] = 0xFF  # tripped; Goal Position is read-only until written 0
    if os.environ.get("REALITYOS_METAL_PTY_STARTUP_TORQUE") == "1":
        regs[64] = 1
        regs[60] = 1
    if os.environ.get("REALITYOS_METAL_PTY_STARTUP_YANK") == "1":
        regs[64] = 1
        regs[60] = 1
    if os.environ.get("REALITYOS_METAL_PTY_ZERO_PWM_SLOPE") == "1":
        regs[62] = 0
    elif os.environ.get("REALITYOS_METAL_PTY_LOW_PWM_SLOPE") == "1":
        regs[62] = 1
    else:
        regs[62] = 140
    regs[68] = 0 if os.environ.get("REALITYOS_METAL_PTY_SRL0") == "1" else 2
    regs[120:122] = struct.pack("<H", 1234)
    regs[126:128] = struct.pack("<h", 0)
    regs[128:132] = struct.pack("<i", 0)
    regs[132:136] = struct.pack("<i", 2048)
    if os.environ.get("REALITYOS_METAL_PTY_PRESENT_OUTSIDE") == "1":
        regs[132:136] = struct.pack("<i", 100)
    if os.environ.get("REALITYOS_METAL_PTY_HIGH_MOVING_THRESHOLD") == "1":
        regs[24:28] = struct.pack("<I", 1023)
    else:
        regs[24:28] = struct.pack("<I", 10)
    if os.environ.get("REALITYOS_METAL_PTY_HOMING") == "1":
        regs[20:24] = struct.pack("<i", 10000)
        regs[132:136] = struct.pack("<i", 12048)
    if os.environ.get("REALITYOS_METAL_PTY_HOMING_IN_WINDOW") == "1":
        regs[20:24] = struct.pack("<i", 1024)
        regs[132:136] = struct.pack("<i", 2048)
    regs[144:146] = struct.pack("<H", 0 if os.environ.get("REALITYOS_METAL_PTY_NO_VIN") == "1" else 50)
    regs[146] = 80 if os.environ.get("REALITYOS_METAL_PTY_HOT") == "1" else 25
    return regs


def status_wanted(srl: int, inst: int) -> bool:
    # Protocol 2.0 Status Return Level: 0=PING only, 1=PING+READ, 2=all.
    if inst == INST_PING:
        return True
    if srl >= 2:
        return True
    if srl == 1 and inst == INST_READ:
        return True
    return False


_motion_block_reads = 0
_corrupt_next_crc = False
_silent_next_status = False
_travel_reads = 0
_travel_from: int | None = None
_travel_to: int | None = None
_boot = time.monotonic()
# One-shot READ refuses. A failed setup read used to look like factory 0
# and skip the safe write (Startup Configuration, I/D, feedforward, watchdog).
_fail_reads: dict[int, int] = {}
if os.environ.get("REALITYOS_METAL_PTY_UNREAD_STARTUP") == "1":
    _fail_reads[60] = 1
if os.environ.get("REALITYOS_METAL_PTY_UNREAD_PID") == "1":
    _fail_reads[80] = 1
    _fail_reads[82] = 1
if os.environ.get("REALITYOS_METAL_PTY_UNREAD_FF") == "1":
    _fail_reads[88] = 1
    _fail_reads[90] = 1
if os.environ.get("REALITYOS_METAL_PTY_UNREAD_WATCHDOG") == "1":
    _fail_reads[98] = 1
if os.environ.get("REALITYOS_METAL_PTY_UNREAD_MOVING_THRESHOLD") == "1":
    _fail_reads[24] = 1


def maybe_startup_yank(regs: bytearray) -> None:
    """Startup Configuration tracks Goal after DTR-RESET without a host write.
    After 80 ms (inside the 100 ms PTY open settle) copy goal→present while
    torque is still on. Open must broadcast torque-off during settle or the
    stale Wizard goal (0) slams present away from 2048."""
    if os.environ.get("REALITYOS_METAL_PTY_STARTUP_YANK") != "1":
        return
    if regs[64] != 1:
        return
    if time.monotonic() - _boot < 0.08:
        return
    if regs[116:120] != regs[132:136]:
        regs[132:136] = regs[116:120]


def advance_delayed_travel(regs: bytearray) -> None:
    """Real XL330 does not teleport present. Moving stays 0 until velocity
    exceeds Moving Threshold. Used by the campaign PTY sequence."""
    global _travel_reads, _travel_from, _travel_to
    if os.environ.get("REALITYOS_METAL_PTY_DELAY_MOTION") != "1" or _travel_to is None:
        return
    assert _travel_from is not None
    _travel_reads += 1
    if _travel_reads < 2:
        regs[132:136] = struct.pack("<i", _travel_from)
        regs[122] = 0
    elif _travel_reads < 4:
        mid = (_travel_from + _travel_to) // 2
        regs[132:136] = struct.pack("<i", mid)
        regs[122] = 1
    else:
        regs[132:136] = struct.pack("<i", _travel_to)
        regs[122] = 0
        _travel_to = None
        _travel_from = None


def handle(regs: bytearray, inst: int, params: bytes) -> tuple[bytes, int]:
    global _motion_block_reads
    maybe_startup_yank(regs)
    if inst == INST_PING:
        return b"", 0
    if inst == INST_READ and len(params) >= 4:
        addr, ln = struct.unpack_from("<HH", params)
        global _fail_reads
        if addr in _fail_reads and _fail_reads[addr] > 0:
            _fail_reads[addr] -= 1
            return b"", 0x80
        if os.environ.get("REALITYOS_METAL_PTY_NO_PRESENT") == "1" and addr == 132:
            return b"", 0x80  # refuse present so setup cannot invent goal=0
        if os.environ.get("REALITYOS_METAL_PTY_NO_VLIMIT") == "1" and addr in (32, 34):
            return b"", 0x80  # refuse voltage EEPROM so setup cannot invent 35/70
        if (
            os.environ.get("REALITYOS_METAL_PTY_NO_HWERR") == "1"
            and addr == 70
            and regs[64] == 1
        ):
            return b"", 0x80  # torque is on; do not skip the post-enable check
        if (
            os.environ.get("REALITYOS_METAL_PTY_HWERR_REFRESH_FAIL") == "1"
            and addr == 70
            and regs[70] != 0
        ):
            # Setup already read register 70 as 0. After STATUS_ALERT latches
            # a non-zero error, drop the diagnostic refresh so the driver
            # must not treat that timeout as bus_lost.
            global _silent_next_status
            _silent_next_status = True
            return bytes([regs[70]]), 0
        if addr == 120:
            _motion_block_reads += 1
            advance_delayed_travel(regs)
            # Setup reads Present Voltage at addr 144. Live I/O uses the
            # motion block. Drop VIN only after the first healthy live
            # sample so start_online can bind, then the next acquire
            # must refuse instead of publishing a writable session.
            if (
                os.environ.get("REALITYOS_METAL_PTY_LIVE_LOW_VIN") == "1"
                and _motion_block_reads >= 2
            ):
                regs[144:146] = struct.pack("<H", 0)
            if (
                os.environ.get("REALITYOS_METAL_PTY_LIVE_BROWN_VIN") == "1"
                and _motion_block_reads >= 2
            ):
                regs[144:146] = struct.pack("<H", 20)
        chunk = bytearray(regs[addr : addr + ln])
        # After two motion-block reads, flip model/fw so live confirm_eeprom
        # sees a physical servo swap on the same UART (not just hot_swap.json).
        if (
            os.environ.get("REALITYOS_METAL_PTY_FLIP_IDENTITY") == "1"
            and addr == 0
            and _motion_block_reads >= 2
        ):
            if len(chunk) >= 2:
                chunk[0:2] = struct.pack("<H", 1190)
            if len(chunk) >= 7:
                chunk[6] = 99
        # After the first live motion sample, corrupt identity CRC so a
        # glitch cannot latch bus_lost. Identify (addr 0 before any motion
        # read) still answers with a good CRC.
        if (
            os.environ.get("REALITYOS_METAL_PTY_NO_IDENTITY") == "1"
            and addr == 0
            and ln >= 8
            and _motion_block_reads >= 1
        ):
            global _corrupt_next_crc
            _corrupt_next_crc = True
        # First motion-block / Moving read after a goal step reports Moving=1,
        # then clears so the campaign wait-for-Moving=0 path is exercised.
        if addr <= 122 < addr + ln and regs[122] == 1:
            regs[122] = 0
        return bytes(chunk), 0
    if inst == INST_WRITE and len(params) >= 2:
        addr = struct.unpack_from("<H", params)[0]
        data = params[2:]
        # Goal PWM/Current/Velocity/Position are read-only while Watchdog=0xFF.
        if regs[98] == 0xFF and addr in (100, 102, 104, 116):
            return b"", 0x08
        if addr == 116 and len(data) >= 4:
            goal = struct.unpack_from("<i", data)[0]
            max_p = struct.unpack_from("<i", regs, 48)[0]
            min_p = struct.unpack_from("<i", regs, 52)[0]
            if goal < min_p or goal > max_p:
                return b"", 0x08  # Protocol 2.0 data range
        if addr == 64 and data:
            was = regs[64]
            regs[64] = data[0]
            if data[0] == 1 and os.environ.get("REALITYOS_METAL_PTY_TORQUE_DROP") == "1":
                # Overload Shutdown: torque enable does not stick.
                regs[64] = 0
                regs[70] = 4
                return b"", 0
            if was == 0 and data[0] == 1 and os.environ.get("REALITYOS_METAL_PTY_HW_AFTER_TORQUE") == "1":
                # Torque sticks; Hardware Error Status latches after enable.
                regs[70] = 4
            if was == 1 and data[0] == 0 and os.environ.get("REALITYOS_METAL_PTY_DRIFT_ON_TORQUE_OFF") == "1":
                present = struct.unpack_from("<i", regs, 132)[0]
                regs[132:136] = struct.pack("<i", present + 20)
            if was == 0 and data[0] == 1:
                goal = struct.unpack_from("<i", regs, 116)[0]
                present = struct.unpack_from("<i", regs, 132)[0]
                if goal != present:
                    regs[132:136] = regs[116:120]
                if os.environ.get("REALITYOS_METAL_PTY_TORQUE_JUMP_PRESENT") == "1":
                    # Robotis resets Present to absolute-within-one-rotation
                    # on torque-on in Position Control.
                    jumped = struct.unpack_from("<i", regs, 132)[0] + 16
                    regs[132:136] = struct.pack("<i", jumped)
                offset = struct.unpack_from("<i", regs, 20)[0]
                if offset != 0:
                    # Leftover Homing Offset vs that reset throws Present
                    # past the 48-tick cage even when pre-torque Present
                    # was still inside 0–4095.
                    present = struct.unpack_from("<i", regs, 132)[0]
                    regs[132:136] = struct.pack("<i", present + 64)
            return b"", 0
        # Protocol 2.0 access error: EEPROM (0–63) is read-only while torque is on.
        if addr < 64 and regs[64] == 1:
            return b"", 0x40
        # ACK but do not store: setup used to trust write_reg Ok and lie
        # about applied PWM Slope / Position P / profile.
        if addr == 62 and os.environ.get("REALITYOS_METAL_PTY_DROP_PWM_SLOPE") == "1":
            return b"", 0
        if addr == 84 and os.environ.get("REALITYOS_METAL_PTY_DROP_POSITION_P") == "1":
            return b"", 0
        if addr in (108, 112) and os.environ.get("REALITYOS_METAL_PTY_DROP_PROFILE") == "1":
            return b"", 0
        if addr == 11 and os.environ.get("REALITYOS_METAL_PTY_DROP_OPERATING_MODE") == "1":
            return b"", 0
        if addr == 10 and os.environ.get("REALITYOS_METAL_PTY_DROP_DRIVE_MODE") == "1":
            return b"", 0
        if addr == 44 and os.environ.get("REALITYOS_METAL_PTY_DROP_VELOCITY_LIMIT") == "1":
            return b"", 0
        if addr == 78 and os.environ.get("REALITYOS_METAL_PTY_DROP_VELOCITY_P") == "1":
            return b"", 0
        if addr == 76 and os.environ.get("REALITYOS_METAL_PTY_DROP_VELOCITY_I") == "1":
            return b"", 0
        if addr == 116 and os.environ.get("REALITYOS_METAL_PTY_DROP_GOAL_POSITION") == "1":
            return b"", 0
        if addr == 13 and os.environ.get("REALITYOS_METAL_PTY_DROP_PROTOCOL_TYPE") == "1":
            return b"", 0
        if addr == 12 and os.environ.get("REALITYOS_METAL_PTY_DROP_SECONDARY_ID") == "1":
            return b"", 0
        if addr == 20 and os.environ.get("REALITYOS_METAL_PTY_DROP_HOMING_OFFSET") == "1":
            return b"", 0
        if addr == 24 and os.environ.get("REALITYOS_METAL_PTY_DROP_MOVING_THRESHOLD") == "1":
            return b"", 0
        if addr == 20 and len(data) >= 4:
            old = struct.unpack_from("<i", regs, 20)[0]
            new = struct.unpack_from("<i", data)[0]
            present = struct.unpack_from("<i", regs, 132)[0]
            regs[addr : addr + len(data)] = data
            regs[132:136] = struct.pack("<i", present - old + new)
            return b"", 0
        regs[addr : addr + len(data)] = data
        if (
            addr == 116
            and os.environ.get("REALITYOS_METAL_PTY_ALERT") == "1"
            and regs[64] == 1
        ):
            # Latch only after torque-on. Setup writes goal=present with
            # torque off; a boot-time latch would reboot and fail open.
            regs[70] = 4
        if addr == 116 and len(data) >= 4:
            p_gain = struct.unpack_from("<H", regs, 84)[0]
            pwm_limit = struct.unpack_from("<H", regs, 36)[0]
            goal_pwm = struct.unpack_from("<h", regs, 100)[0]
            vel_p = struct.unpack_from("<H", regs, 78)[0]
            slope = regs[62]
            if (
                p_gain > 0
                and pwm_limit > 0
                and abs(goal_pwm) >= 80
                and vel_p > 0
                and slope >= 20
            ):
                old_present = struct.unpack_from("<i", regs, 132)[0]
                new_goal = struct.unpack_from("<i", data)[0]
                # Wizard P above factory 400 overshoots a 32-tick step
                # past the 48-tick cage. Setup must cap P first.
                landed = new_goal
                if p_gain > 400 and new_goal != old_present:
                    step = new_goal - old_present
                    landed = new_goal + (32 if step >= 0 else -32)
                if (
                    new_goal != old_present
                    and os.environ.get("REALITYOS_METAL_PTY_DELAY_MOTION") == "1"
                ):
                    global _travel_from, _travel_to, _travel_reads
                    _travel_from = old_present
                    _travel_to = landed
                    _travel_reads = 0
                    regs[122] = 0
                else:
                    regs[132:136] = struct.pack("<i", landed)
                    if new_goal != old_present:
                        regs[122] = 1
        return b"", 0
    if inst == INST_REBOOT:
        regs[70] = 0
        regs[68] = 2  # RAM reset; factory Status Return Level
        regs[98] = 0
        if os.environ.get("REALITYOS_METAL_PTY_HW_ERROR") == "1":
            regs[64] = 1  # Startup Configuration torque-on after reboot
        return b"", 0
    return b"", 0


def main() -> None:
    master, slave = pty.openpty()
    tty.setraw(master, when=termios.TCSANOW)
    print(os.ttyname(slave), flush=True)
    regs = init_regs()
    buf = bytearray()
    while True:
        try:
            chunk = os.read(master, 256)
        except OSError:
            return
        if not chunk:
            return
        buf.extend(chunk)
        parsed = parse_request(bytes(buf))
        if parsed is None:
            if len(buf) > 512:
                del buf[:256]
            continue
        req_id, inst, params, _consumed = parsed
        del buf[:]
        own = regs[7]
        secondary = regs[12]
        if req_id not in (254, own) and not (
            secondary != 255 and req_id == secondary
        ):
            continue
        if (
            (
                os.environ.get("REALITYOS_METAL_PTY_MULTI") == "1"
                or os.environ.get("REALITYOS_METAL_PTY_SECONDARY") == "1"
            )
            and req_id == 254
            and inst == INST_PING
        ):
            echo = HEADER + bytes([own, 0x07, 0x00, INST_PING, 0x00, 0x00])
            os.write(
                master,
                echo + encode_status(1, b"") + encode_status(7, b""),
            )
            continue
        alert = STATUS_ALERT if os.environ.get("REALITYOS_METAL_PTY_ALERT") == "1" else 0
        srl = regs[68]
        payload, inst_err = handle(regs, inst, params)
        # Half-duplex adapters often echo a request-shaped frame before status.
        echo = HEADER + bytes([own, 0x07, 0x00, INST_PING, 0x00, 0x00])
        global _silent_next_status
        if _silent_next_status:
            _silent_next_status = False
            continue
        if status_wanted(srl, inst):
            pkt = echo + encode_status(own, payload, error=alert | inst_err)
            global _corrupt_next_crc
            if _corrupt_next_crc:
                _corrupt_next_crc = False
                pkt = bytearray(pkt)
                pkt[-1] ^= 0xFF
                pkt = bytes(pkt)
            try:
                os.write(master, pkt)
            except OSError:
                continue
        else:
            try:
                os.write(master, echo)
            except OSError:
                continue


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"responder_error:{e}", file=sys.stderr)
        sys.exit(1)
