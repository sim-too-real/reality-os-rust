//! Protocol 2.0 XL330 virtual device. Uses shipped metal encode/decode/CRC.

use realityos_metal::protocol::{
    decode_instruction, encode_status, xl330_model_slug, ADDR_BAUD_RATE, ADDR_BUS_WATCHDOG,
    ADDR_CURRENT_LIMIT, ADDR_DRIVE_MODE, ADDR_FEEDFORWARD_1ST, ADDR_FEEDFORWARD_2ND,
    ADDR_FIRMWARE_VERSION, ADDR_GOAL_CURRENT, ADDR_GOAL_POSITION, ADDR_GOAL_PWM,
    ADDR_GOAL_VELOCITY, ADDR_HARDWARE_ERROR, ADDR_ID, ADDR_LED, ADDR_MAX_POSITION_LIMIT,
    ADDR_MAX_VOLTAGE_LIMIT, ADDR_MIN_POSITION_LIMIT, ADDR_MIN_VOLTAGE_LIMIT, ADDR_MODEL_NUMBER,
    ADDR_MOVING, ADDR_MOVING_THRESHOLD, ADDR_OPERATING_MODE, ADDR_POSITION_D_GAIN,
    ADDR_POSITION_I_GAIN, ADDR_POSITION_P_GAIN, ADDR_PRESENT_POSITION, ADDR_PRESENT_TEMPERATURE,
    ADDR_PRESENT_VELOCITY, ADDR_PRESENT_VOLTAGE, ADDR_PROFILE_ACCEL, ADDR_PROFILE_VELOCITY,
    ADDR_PROTOCOL_TYPE, ADDR_PWM_LIMIT, ADDR_PWM_SLOPE, ADDR_REALTIME_TICK,
    ADDR_REGISTERED_INSTRUCTION, ADDR_RETURN_DELAY_TIME, ADDR_SECONDARY_ID, ADDR_SHUTDOWN,
    ADDR_STARTUP_CONFIGURATION, ADDR_STATUS_RETURN_LEVEL, ADDR_TEMPERATURE_LIMIT,
    ADDR_TORQUE_ENABLE, ADDR_VELOCITY_I_GAIN, ADDR_VELOCITY_LIMIT, ADDR_VELOCITY_P_GAIN,
    BROADCAST_ID, ERR_ACCESS, ERR_DATA_LENGTH, ERR_DATA_LIMIT, ERR_DATA_RANGE, ERR_INSTRUCTION,
    FACTORY_MOVING_THRESHOLD, FACTORY_PWM_LIMIT, FACTORY_PWM_SLOPE, FACTORY_SHUTDOWN,
    FACTORY_STARTUP_CONFIGURATION, FACTORY_VELOCITY_I_GAIN, HWERR_INPUT_VOLTAGE, HWERR_OVERHEATING,
    INST_PING, INST_READ, INST_REBOOT, INST_WRITE, OPERATING_MODE_POSITION, PROTOCOL_TYPE_2,
    SECONDARY_ID_DISABLED, STATUS_ALERT, STATUS_RETURN_ALL, XL330_M288_MODEL,
    XL330_POSITION_MODE_MAX, XL330_POSITION_MODE_MIN,
};

use crate::faults::{corrupt_crc_bytes, FaultKind, FaultSchedule};
use crate::physics::{
    no_load_speed_rpm_at, rpm_to_rad_s, stall_torque_nm_at, ticks_to_rad, MotionLimits,
    PhysicsState,
};
use crate::truth_pack::Xl330TruthPack;

const TABLE: usize = 252;
const FACTORY_FW: u8 = 46;
const FACTORY_VEL_P: u16 = 180;
const FACTORY_POS_P: u16 = 400;
const FACTORY_CURRENT_LIMIT: u16 = 1750;
const FACTORY_VEL_LIMIT: u32 = 445;

#[derive(Clone, Copy)]
struct Field {
    addr: u16,
    size: u8,
    eeprom: bool,
    readonly: bool,
}

const FIELDS: &[Field] = &[
    Field {
        addr: 0,
        size: 2,
        eeprom: true,
        readonly: true,
    },
    Field {
        addr: 6,
        size: 1,
        eeprom: true,
        readonly: true,
    },
    Field {
        addr: 7,
        size: 1,
        eeprom: true,
        readonly: false,
    },
    Field {
        addr: 8,
        size: 1,
        eeprom: true,
        readonly: false,
    },
    Field {
        addr: 9,
        size: 1,
        eeprom: true,
        readonly: false,
    },
    Field {
        addr: 10,
        size: 1,
        eeprom: true,
        readonly: false,
    },
    Field {
        addr: 11,
        size: 1,
        eeprom: true,
        readonly: false,
    },
    Field {
        addr: 12,
        size: 1,
        eeprom: true,
        readonly: false,
    },
    Field {
        addr: 13,
        size: 1,
        eeprom: true,
        readonly: false,
    },
    Field {
        addr: 20,
        size: 4,
        eeprom: true,
        readonly: false,
    },
    Field {
        addr: 24,
        size: 4,
        eeprom: true,
        readonly: false,
    },
    Field {
        addr: 31,
        size: 1,
        eeprom: true,
        readonly: false,
    },
    Field {
        addr: 32,
        size: 2,
        eeprom: true,
        readonly: false,
    },
    Field {
        addr: 34,
        size: 2,
        eeprom: true,
        readonly: false,
    },
    Field {
        addr: 36,
        size: 2,
        eeprom: true,
        readonly: false,
    },
    Field {
        addr: 38,
        size: 2,
        eeprom: true,
        readonly: false,
    },
    Field {
        addr: 44,
        size: 4,
        eeprom: true,
        readonly: false,
    },
    Field {
        addr: 48,
        size: 4,
        eeprom: true,
        readonly: false,
    },
    Field {
        addr: 52,
        size: 4,
        eeprom: true,
        readonly: false,
    },
    Field {
        addr: 60,
        size: 1,
        eeprom: true,
        readonly: false,
    },
    Field {
        addr: 62,
        size: 1,
        eeprom: true,
        readonly: false,
    },
    Field {
        addr: 63,
        size: 1,
        eeprom: true,
        readonly: false,
    },
    Field {
        addr: 64,
        size: 1,
        eeprom: false,
        readonly: false,
    },
    Field {
        addr: 65,
        size: 1,
        eeprom: false,
        readonly: false,
    },
    Field {
        addr: 68,
        size: 1,
        eeprom: false,
        readonly: false,
    },
    Field {
        addr: 69,
        size: 1,
        eeprom: false,
        readonly: true,
    },
    Field {
        addr: 70,
        size: 1,
        eeprom: false,
        readonly: true,
    },
    Field {
        addr: 76,
        size: 2,
        eeprom: false,
        readonly: false,
    },
    Field {
        addr: 78,
        size: 2,
        eeprom: false,
        readonly: false,
    },
    Field {
        addr: 80,
        size: 2,
        eeprom: false,
        readonly: false,
    },
    Field {
        addr: 82,
        size: 2,
        eeprom: false,
        readonly: false,
    },
    Field {
        addr: 84,
        size: 2,
        eeprom: false,
        readonly: false,
    },
    Field {
        addr: 88,
        size: 2,
        eeprom: false,
        readonly: false,
    },
    Field {
        addr: 90,
        size: 2,
        eeprom: false,
        readonly: false,
    },
    Field {
        addr: 98,
        size: 1,
        eeprom: false,
        readonly: false,
    },
    Field {
        addr: 100,
        size: 2,
        eeprom: false,
        readonly: false,
    },
    Field {
        addr: 102,
        size: 2,
        eeprom: false,
        readonly: false,
    },
    Field {
        addr: 104,
        size: 4,
        eeprom: false,
        readonly: false,
    },
    Field {
        addr: 108,
        size: 4,
        eeprom: false,
        readonly: false,
    },
    Field {
        addr: 112,
        size: 4,
        eeprom: false,
        readonly: false,
    },
    Field {
        addr: 116,
        size: 4,
        eeprom: false,
        readonly: false,
    },
    Field {
        addr: 120,
        size: 2,
        eeprom: false,
        readonly: true,
    },
    Field {
        addr: 122,
        size: 1,
        eeprom: false,
        readonly: true,
    },
    Field {
        addr: 126,
        size: 2,
        eeprom: false,
        readonly: true,
    },
    Field {
        addr: 128,
        size: 4,
        eeprom: false,
        readonly: true,
    },
    Field {
        addr: 132,
        size: 4,
        eeprom: false,
        readonly: true,
    },
    Field {
        addr: 144,
        size: 2,
        eeprom: false,
        readonly: true,
    },
    Field {
        addr: 146,
        size: 1,
        eeprom: false,
        readonly: true,
    },
];

fn field_at(addr: u16) -> Option<Field> {
    FIELDS
        .iter()
        .copied()
        .find(|f| addr >= f.addr && addr < f.addr + u16::from(f.size))
}

#[derive(Clone)]
pub struct VirtualXl330 {
    table: [u8; TABLE],
    physics: PhysicsState,
    pack: Xl330TruthPack,
    packets: u32,
    physical_actions: u64,
    faults: FaultSchedule,
    drop_next_status: bool,
    drop_status_after_goal: bool,
    wrong_status_id: bool,
    corrupt_next_crc: bool,
    moving_latched: bool,
    tick_ms: u16,
    /// Shutdown-table bit 0 (0x01) or voltage-prose bit 4 (0x10).
    voltage_error_bit: u8,
    boot_latency_s: f64,
    transport_latency_s: f64,
    sim_time_s: f64,
}

/// Uniform in [0, 1] from the high 32 bits. (`>> 33` / `u32::MAX` only reached ~0.5.)
fn unit01(x: u64) -> f64 {
    (x >> 32) as f64 / f64::from(u32::MAX)
}

impl VirtualXl330 {
    pub fn xl330_m288() -> Self {
        Self::from_pack(Xl330TruthPack::xl330_m288(), 5.0, 2048)
    }

    pub fn from_seed(seed: u64, pack: Xl330TruthPack) -> Self {
        let mut rng = seed ^ 0x9E37_79B9_7F4A_7C15;
        rng = rng.wrapping_mul(0xBF58_476D_1CE4_E5B9);
        let u = unit01(rng);
        let v_lo = pack.input_voltage_min_v.value.unwrap_or(3.7);
        let v_hi = pack.input_voltage_max_v.value.unwrap_or(6.0);
        let v = v_lo + u * (v_hi - v_lo);
        rng = rng.wrapping_mul(0x94D0_49BB_1331_11EB);
        let pos = 100 + ((rng >> 32) % 3900) as i32;
        rng = rng.wrapping_mul(0xBF58_476D_1CE4_E5B9);
        let u_eff = unit01(rng);
        let (eff_lo, eff_hi) = pack.gearbox_efficiency.range.unwrap_or((1.0, 1.0));
        let eff = eff_lo + u_eff * (eff_hi - eff_lo);
        rng = rng.wrapping_mul(0x94D0_49BB_1331_11EB);
        let u_boot = unit01(rng);
        let (b_lo, b_hi) = pack.boot_delay_s.range.unwrap_or((0.0, 0.0));
        let boot = b_lo + u_boot * (b_hi - b_lo);
        rng = rng.wrapping_mul(0xBF58_476D_1CE4_E5B9);
        let u_lat = unit01(rng);
        let transport = u_lat * 0.020;
        rng = rng.wrapping_mul(0x94D0_49BB_1331_11EB);
        let bit = if (rng >> 63) == 0 {
            HWERR_INPUT_VOLTAGE
        } else {
            1 << 4
        };
        let mut d = Self::from_pack(pack, v, pos);
        d.physics.gearbox_efficiency = eff;
        d.boot_latency_s = boot;
        d.transport_latency_s = transport;
        d.voltage_error_bit = bit;
        d
    }

    pub fn from_pack(pack: Xl330TruthPack, voltage_v: f64, present: i32) -> Self {
        let mut d = Self {
            table: [0; TABLE],
            physics: PhysicsState::from_ticks(present, 4096, voltage_v),
            pack,
            packets: 0,
            physical_actions: 0,
            faults: FaultSchedule::empty(),
            drop_next_status: false,
            drop_status_after_goal: false,
            wrong_status_id: false,
            corrupt_next_crc: false,
            moving_latched: false,
            tick_ms: 0,
            voltage_error_bit: HWERR_INPUT_VOLTAGE,
            boot_latency_s: 0.0,
            transport_latency_s: 0.0,
            sim_time_s: 0.0,
        };
        d.load_factory();
        d.sync_present();
        d
    }

    fn load_factory(&mut self) {
        self.put_u16(ADDR_MODEL_NUMBER, XL330_M288_MODEL);
        self.table[ADDR_FIRMWARE_VERSION as usize] = FACTORY_FW;
        self.table[ADDR_ID as usize] = 1;
        self.table[ADDR_BAUD_RATE as usize] = 1;
        self.table[ADDR_RETURN_DELAY_TIME as usize] = 250;
        self.table[ADDR_DRIVE_MODE as usize] = 0;
        self.table[ADDR_OPERATING_MODE as usize] = OPERATING_MODE_POSITION;
        self.table[ADDR_SECONDARY_ID as usize] = SECONDARY_ID_DISABLED;
        self.table[ADDR_PROTOCOL_TYPE as usize] = PROTOCOL_TYPE_2;
        self.put_u32(ADDR_MOVING_THRESHOLD, FACTORY_MOVING_THRESHOLD);
        self.table[ADDR_TEMPERATURE_LIMIT as usize] = 70;
        self.put_u16(ADDR_MAX_VOLTAGE_LIMIT, 70);
        self.put_u16(ADDR_MIN_VOLTAGE_LIMIT, 35);
        self.put_u16(ADDR_PWM_LIMIT, FACTORY_PWM_LIMIT);
        self.put_u16(ADDR_CURRENT_LIMIT, FACTORY_CURRENT_LIMIT);
        self.put_u32(ADDR_VELOCITY_LIMIT, FACTORY_VEL_LIMIT);
        self.put_i32(ADDR_MAX_POSITION_LIMIT, XL330_POSITION_MODE_MAX);
        self.put_i32(ADDR_MIN_POSITION_LIMIT, XL330_POSITION_MODE_MIN);
        self.table[ADDR_STARTUP_CONFIGURATION as usize] = FACTORY_STARTUP_CONFIGURATION;
        self.table[ADDR_PWM_SLOPE as usize] = FACTORY_PWM_SLOPE;
        self.table[ADDR_SHUTDOWN as usize] = FACTORY_SHUTDOWN;
        self.reset_ram();
    }

    fn reset_ram(&mut self) {
        self.table[ADDR_TORQUE_ENABLE as usize] = 0;
        self.table[ADDR_LED as usize] = 0;
        self.table[ADDR_STATUS_RETURN_LEVEL as usize] = STATUS_RETURN_ALL;
        self.table[ADDR_REGISTERED_INSTRUCTION as usize] = 0;
        self.table[ADDR_HARDWARE_ERROR as usize] = 0;
        self.put_u16(ADDR_VELOCITY_I_GAIN, FACTORY_VELOCITY_I_GAIN);
        self.put_u16(ADDR_VELOCITY_P_GAIN, FACTORY_VEL_P);
        self.put_u16(ADDR_POSITION_D_GAIN, 0);
        self.put_u16(ADDR_POSITION_I_GAIN, 0);
        self.put_u16(ADDR_POSITION_P_GAIN, FACTORY_POS_P);
        self.put_u16(ADDR_FEEDFORWARD_2ND, 0);
        self.put_u16(ADDR_FEEDFORWARD_1ST, 0);
        self.table[ADDR_BUS_WATCHDOG as usize] = 0;
        let pwm = self.u16_at(ADDR_PWM_LIMIT);
        self.put_i16(ADDR_GOAL_PWM, pwm as i16);
        self.put_i16(ADDR_GOAL_CURRENT, self.u16_at(ADDR_CURRENT_LIMIT) as i16);
        self.put_i32(ADDR_GOAL_VELOCITY, 0);
        self.put_u32(ADDR_PROFILE_ACCEL, 0);
        self.put_u32(ADDR_PROFILE_VELOCITY, 0);
        let present = self.physics.present_ticks(4096).clamp(0, 4095);
        self.put_i32(ADDR_GOAL_POSITION, present);
        self.put_i32(ADDR_PRESENT_POSITION, present);
        self.table[ADDR_MOVING as usize] = 0;
        self.sync_present();
    }

    fn sync_present(&mut self) {
        let ticks = self.physics.present_ticks(4096).clamp(0, 4095);
        self.put_i32(ADDR_PRESENT_POSITION, ticks);
        let dv = (self.physics.voltage_v * 10.0).round().clamp(0.0, 255.0) as u16;
        self.put_u16(ADDR_PRESENT_VOLTAGE, dv);
        self.table[ADDR_PRESENT_TEMPERATURE as usize] =
            self.physics.temperature_c.round().clamp(0.0, 100.0) as u8;
        self.put_u16(ADDR_REALTIME_TICK, self.tick_ms);
        let rpm = self.physics.omega_rad_s * 60.0 / std::f64::consts::TAU;
        let vel_raw = (rpm / 0.229).round() as i32;
        self.put_i32(ADDR_PRESENT_VELOCITY, vel_raw);
        let moving_th = self.u32_at(ADDR_MOVING_THRESHOLD);
        self.table[ADDR_MOVING as usize] =
            u8::from(vel_raw.unsigned_abs() > moving_th || self.moving_latched);
        self.refresh_faults();
    }

    fn refresh_faults(&mut self) {
        let mut hw = 0u8;
        let vin = self.u16_at(ADDR_PRESENT_VOLTAGE);
        if vin < self.u16_at(ADDR_MIN_VOLTAGE_LIMIT) || vin > self.u16_at(ADDR_MAX_VOLTAGE_LIMIT) {
            hw |= self.voltage_error_bit;
        }
        if self.table[ADDR_PRESENT_TEMPERATURE as usize]
            > self.table[ADDR_TEMPERATURE_LIMIT as usize]
        {
            hw |= HWERR_OVERHEATING;
        }
        self.table[ADDR_HARDWARE_ERROR as usize] = hw;
        let shutdown = self.table[ADDR_SHUTDOWN as usize];
        if hw != 0 && (shutdown & hw) != 0 {
            self.table[ADDR_TORQUE_ENABLE as usize] = 0;
        }
    }

    pub fn set_fault_schedule(&mut self, s: FaultSchedule) {
        self.faults = s;
    }

    pub fn id(&self) -> u8 {
        self.table[ADDR_ID as usize]
    }

    pub fn model(&self) -> u16 {
        self.u16_at(ADDR_MODEL_NUMBER)
    }

    pub fn firmware(&self) -> u8 {
        self.table[ADDR_FIRMWARE_VERSION as usize]
    }

    pub fn firmware_id_string(&self) -> String {
        let slug = xl330_model_slug(self.model()).unwrap_or("unknown");
        format!("{slug}:{}:{}", self.model(), self.firmware())
    }

    pub fn physical_action_count(&self) -> u64 {
        self.physical_actions
    }

    pub fn present_position(&self) -> i32 {
        self.i32_at(ADDR_PRESENT_POSITION)
    }

    pub fn goal_position(&self) -> i32 {
        self.i32_at(ADDR_GOAL_POSITION)
    }

    pub fn torque_enabled(&self) -> bool {
        self.table[ADDR_TORQUE_ENABLE as usize] == 1
    }

    pub fn hardware_error(&self) -> u8 {
        self.table[ADDR_HARDWARE_ERROR as usize]
    }

    pub fn set_model_firmware(&mut self, model: u16, fw: u8) {
        self.put_u16(ADDR_MODEL_NUMBER, model);
        self.table[ADDR_FIRMWARE_VERSION as usize] = fw;
    }

    pub fn set_voltage_v(&mut self, v: f64) {
        self.physics.voltage_v = v;
        self.sync_present();
    }

    pub fn set_temperature_c(&mut self, t: f64) {
        self.physics.temperature_c = t;
        self.sync_present();
    }

    pub fn stall_torque_nm_at(&self, voltage_v: f64) -> Option<f64> {
        stall_torque_nm_at(&self.pack, voltage_v)
    }

    pub fn no_load_speed_rpm_at(&self, voltage_v: f64) -> Option<f64> {
        no_load_speed_rpm_at(&self.pack, voltage_v)
    }

    pub fn truth_pack(&self) -> &Xl330TruthPack {
        &self.pack
    }

    pub fn trip_watchdog(&mut self) {
        self.table[ADDR_BUS_WATCHDOG as usize] = 0xFF;
    }

    pub fn drop_status_after_next_goal(&mut self) {
        self.drop_status_after_goal = true;
    }

    pub fn boot_latency_s(&self) -> f64 {
        self.boot_latency_s
    }

    pub fn transport_latency_s(&self) -> f64 {
        self.transport_latency_s
    }

    pub fn voltage_error_bit(&self) -> u8 {
        self.voltage_error_bit
    }

    pub fn gearbox_efficiency(&self) -> f64 {
        self.physics.gearbox_efficiency
    }

    pub fn voltage_v(&self) -> f64 {
        self.physics.voltage_v
    }

    pub fn sim_time_s(&self) -> f64 {
        self.sim_time_s
    }

    pub fn set_voltage_error_bit(&mut self, bit: u8) {
        self.voltage_error_bit = bit;
        self.refresh_faults();
    }

    pub fn set_gearbox_efficiency(&mut self, eff: f64) {
        self.physics.gearbox_efficiency = eff.clamp(0.0, 1.0);
    }

    pub fn set_load_torque_nm(&mut self, tau: f64) {
        self.physics.load_torque_nm = tau;
    }

    pub fn apply_fault_kind(&mut self, kind: FaultKind) {
        match kind {
            FaultKind::DropStatusAfterApply | FaultKind::DropStatus => {
                self.drop_next_status = true;
            }
            FaultKind::WrongStatusId => self.wrong_status_id = true,
            FaultKind::CorruptOutgoingCrc => self.corrupt_next_crc = true,
            FaultKind::VoltageOutOfRange => self.set_voltage_v(9.0),
            FaultKind::OverTemperature => self.set_temperature_c(90.0),
            FaultKind::WatchdogTrip => self.trip_watchdog(),
            FaultKind::IdentitySwap { model, firmware } => {
                self.set_model_firmware(model, firmware);
            }
            FaultKind::VoltageErrorBit { bit } => self.set_voltage_error_bit(bit),
            FaultKind::RebootDuringRequest => {
                self.reset_ram();
            }
            _ => {}
        }
    }

    /// Advance the envelope surrogate. Goal writes only store a target.
    pub fn advance(&mut self, dt: f64) {
        let limits = self.motion_limits();
        self.physics.advance(dt, &self.pack, &limits);
        self.sim_time_s += dt.max(0.0);
        self.tick_ms = self
            .tick_ms
            .wrapping_add((dt.max(0.0) * 1000.0).round() as u16);
        if (self.physics.present_ticks(4096) - self.i32_at(ADDR_GOAL_POSITION)).abs() <= 1 {
            self.moving_latched = false;
        }
        self.sync_present();
    }

    fn motion_limits(&self) -> MotionLimits {
        let pwm_lim = self.u16_at(ADDR_PWM_LIMIT).max(1);
        let goal_pwm = i16::from_le_bytes([
            self.table[ADDR_GOAL_PWM as usize],
            self.table[ADDR_GOAL_PWM as usize + 1],
        ]);
        let cur_lim = self.u16_at(ADDR_CURRENT_LIMIT).max(1);
        let goal_cur = i16::from_le_bytes([
            self.table[ADDR_GOAL_CURRENT as usize],
            self.table[ADDR_GOAL_CURRENT as usize + 1],
        ]);
        let pulses = 4096u16;
        let profile_vel = self.u32_at(ADDR_PROFILE_VELOCITY);
        let profile_accel = self.u32_at(ADDR_PROFILE_ACCEL);
        let noload = no_load_speed_rpm_at(&self.pack, self.physics.voltage_v).unwrap_or(0.0);
        MotionLimits {
            torque_on: self.torque_enabled(),
            pwm_frac: (goal_pwm.unsigned_abs() as f64) / f64::from(pwm_lim),
            current_frac: (goal_cur.unsigned_abs() as f64) / f64::from(cur_lim),
            pos_min_rad: ticks_to_rad(self.i32_at(ADDR_MIN_POSITION_LIMIT), pulses),
            pos_max_rad: ticks_to_rad(self.i32_at(ADDR_MAX_POSITION_LIMIT), pulses),
            profile_velocity_rad_s: if profile_vel == 0 {
                0.0
            } else {
                rpm_to_rad_s(f64::from(profile_vel) * 0.229).min(rpm_to_rad_s(noload))
            },
            profile_accel_rad_s2: if profile_accel == 0 {
                0.0
            } else {
                f64::from(profile_accel)
            },
        }
    }

    pub fn process(&mut self, bytes: &[u8]) -> Vec<u8> {
        self.packets = self.packets.saturating_add(1);
        self.tick_ms = self.tick_ms.wrapping_add(1);
        for kind in self.faults.take_at(self.packets) {
            self.apply_fault_kind(kind);
        }
        let pkt = match decode_instruction(bytes) {
            Ok(p) => p,
            Err(_) => return Vec::new(),
        };
        let mine = pkt.id == self.id()
            || pkt.id == BROADCAST_ID
            || (self.table[ADDR_SECONDARY_ID as usize] < 253
                && pkt.id == self.table[ADDR_SECONDARY_ID as usize]);
        if !mine {
            return Vec::new();
        }
        self.sync_present();
        let secondary = pkt.id != self.id() && pkt.id != BROADCAST_ID;
        let (err, params, applied_goal) = match pkt.instruction {
            INST_PING => (0, self.ping_params(), false),
            INST_READ => self.do_read(&pkt.params),
            INST_WRITE => self.do_write(&pkt.params),
            INST_REBOOT => {
                let status = self.status_bytes(pkt.id, 0, &[], secondary, INST_REBOOT);
                self.reset_ram();
                return status;
            }
            _ => (ERR_INSTRUCTION, Vec::new(), false),
        };
        if applied_goal {
            self.physical_actions = self.physical_actions.saturating_add(1);
            if self.drop_status_after_goal {
                self.drop_status_after_goal = false;
                self.drop_next_status = true;
            }
        }
        self.status_bytes(pkt.id, err, &params, secondary, pkt.instruction)
    }

    fn ping_params(&self) -> Vec<u8> {
        let m = self.model().to_le_bytes();
        vec![m[0], m[1], self.firmware()]
    }

    fn do_read(&mut self, params: &[u8]) -> (u8, Vec<u8>, bool) {
        if params.len() < 4 {
            return (ERR_DATA_LENGTH, Vec::new(), false);
        }
        let addr = u16::from_le_bytes([params[0], params[1]]);
        let len = u16::from_le_bytes([params[2], params[3]]) as usize;
        let start = addr as usize;
        if start >= TABLE || start + len > TABLE {
            return (ERR_ACCESS, Vec::new(), false);
        }
        if addr == ADDR_MOVING {
            let v = u8::from(self.moving_latched);
            self.moving_latched = false;
            self.table[ADDR_MOVING as usize] = v;
        }
        self.sync_present();
        (0, self.table[start..start + len].to_vec(), false)
    }

    fn apply_goal_to_physics(&mut self) {
        let goal = self.i32_at(ADDR_GOAL_POSITION);
        self.physics.set_target_ticks(goal, 4096);
        self.moving_latched = goal != self.physics.present_ticks(4096);
        self.table[ADDR_MOVING as usize] = u8::from(self.moving_latched);
    }

    fn do_write(&mut self, params: &[u8]) -> (u8, Vec<u8>, bool) {
        if params.len() < 3 {
            return (ERR_DATA_LENGTH, Vec::new(), false);
        }
        let addr = u16::from_le_bytes([params[0], params[1]]);
        let data = &params[2..];
        if let Some(err) = self.write_check(addr, data) {
            return (err, Vec::new(), false);
        }
        let start = addr as usize;
        self.table[start..start + data.len()].copy_from_slice(data);
        let mut applied_goal = false;
        if addr == ADDR_GOAL_POSITION && data.len() == 4 && self.torque_enabled() {
            self.apply_goal_to_physics();
            applied_goal = true;
        }
        if addr == ADDR_TORQUE_ENABLE && data.first() == Some(&1) {
            let before = self.physics.present_ticks(4096);
            self.apply_goal_to_physics();
            if self.i32_at(ADDR_GOAL_POSITION) != before {
                applied_goal = true;
            }
        }
        if addr == ADDR_OPERATING_MODE {
            self.put_u32(ADDR_PROFILE_ACCEL, 0);
            self.put_u32(ADDR_PROFILE_VELOCITY, 0);
            let pwm = self.u16_at(ADDR_PWM_LIMIT);
            self.put_i16(ADDR_GOAL_PWM, pwm as i16);
        }
        if addr == ADDR_BUS_WATCHDOG && data[0] == 0 {
            // clear trip
        }
        if addr == ADDR_ID {
            // identity follows EEPROM
        }
        self.sync_present();
        (0, Vec::new(), applied_goal)
    }

    fn write_check(&self, addr: u16, data: &[u8]) -> Option<u8> {
        let Some(f) = field_at(addr) else {
            return Some(ERR_ACCESS);
        };
        if addr != f.addr || data.len() != f.size as usize {
            return Some(ERR_DATA_LENGTH);
        }
        if f.readonly {
            return Some(ERR_ACCESS);
        }
        if f.eeprom && self.torque_enabled() {
            return Some(ERR_ACCESS);
        }
        if self.table[ADDR_BUS_WATCHDOG as usize] == 0xFF
            && matches!(
                addr,
                ADDR_GOAL_POSITION | ADDR_GOAL_PWM | ADDR_GOAL_VELOCITY | ADDR_GOAL_CURRENT
            )
        {
            return Some(ERR_ACCESS);
        }
        if addr == ADDR_ID {
            let id = data[0];
            if id > 252 {
                return Some(ERR_DATA_RANGE);
            }
        }
        if addr == ADDR_GOAL_POSITION {
            let goal = i32::from_le_bytes(data.try_into().ok()?);
            let min = self.i32_at(ADDR_MIN_POSITION_LIMIT);
            let max = self.i32_at(ADDR_MAX_POSITION_LIMIT);
            if self.table[ADDR_OPERATING_MODE as usize] == OPERATING_MODE_POSITION
                && (goal < min || goal > max)
            {
                return Some(ERR_DATA_LIMIT);
            }
            if (goal < XL330_POSITION_MODE_MIN || goal > XL330_POSITION_MODE_MAX)
                && self.table[ADDR_OPERATING_MODE as usize] == OPERATING_MODE_POSITION
            {
                return Some(ERR_DATA_RANGE);
            }
        }
        if addr == ADDR_GOAL_PWM {
            let g = i16::from_le_bytes(data.try_into().ok()?);
            let lim = self.u16_at(ADDR_PWM_LIMIT) as i16;
            if g < -lim || g > lim {
                return Some(ERR_DATA_LIMIT);
            }
        }
        if addr == ADDR_PWM_LIMIT {
            let v = u16::from_le_bytes(data.try_into().ok()?);
            if v > FACTORY_PWM_LIMIT {
                return Some(ERR_DATA_RANGE);
            }
        }
        None
    }

    fn status_bytes(
        &mut self,
        request_id: u8,
        err: u8,
        params: &[u8],
        secondary: bool,
        inst: u8,
    ) -> Vec<u8> {
        if self.drop_next_status {
            self.drop_next_status = false;
            return Vec::new();
        }
        if request_id == BROADCAST_ID && inst != INST_PING {
            return Vec::new();
        }
        if secondary {
            return Vec::new();
        }
        let srl = self.table[ADDR_STATUS_RETURN_LEVEL as usize];
        let want = match inst {
            INST_PING => true,
            INST_READ => srl >= 1,
            _ => srl >= 2,
        };
        if !want {
            return Vec::new();
        }
        let mut error = err;
        if self.table[ADDR_HARDWARE_ERROR as usize] != 0 {
            error |= STATUS_ALERT;
        }
        let mut id = self.id();
        if self.wrong_status_id {
            id = id.wrapping_add(1);
            self.wrong_status_id = false;
        }
        let mut bytes = encode_status(id, error, params);
        if self.corrupt_next_crc {
            self.corrupt_next_crc = false;
            bytes = corrupt_crc_bytes(bytes);
        }
        bytes
    }

    fn put_u16(&mut self, addr: u16, v: u16) {
        let b = v.to_le_bytes();
        self.table[addr as usize] = b[0];
        self.table[addr as usize + 1] = b[1];
    }
    fn put_i16(&mut self, addr: u16, v: i16) {
        self.put_u16(addr, v as u16);
    }
    fn put_u32(&mut self, addr: u16, v: u32) {
        let b = v.to_le_bytes();
        self.table[addr as usize..addr as usize + 4].copy_from_slice(&b);
    }
    fn put_i32(&mut self, addr: u16, v: i32) {
        self.put_u32(addr, v as u32);
    }
    fn u16_at(&self, addr: u16) -> u16 {
        u16::from_le_bytes([self.table[addr as usize], self.table[addr as usize + 1]])
    }
    fn i32_at(&self, addr: u16) -> i32 {
        i32::from_le_bytes([
            self.table[addr as usize],
            self.table[addr as usize + 1],
            self.table[addr as usize + 2],
            self.table[addr as usize + 3],
        ])
    }
    fn u32_at(&self, addr: u16) -> u32 {
        u32::from_le_bytes([
            self.table[addr as usize],
            self.table[addr as usize + 1],
            self.table[addr as usize + 2],
            self.table[addr as usize + 3],
        ])
    }
}
