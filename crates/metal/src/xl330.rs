//! One XL330 HardwareDriverPort. Translates authorized actions to Protocol 2.0.
//! Contains no Reality OS policy.

use std::fs::{File, OpenOptions};
use std::io::{self, Read, Write};
use std::os::unix::fs::{OpenOptionsExt, PermissionsExt};
use std::os::unix::io::AsRawFd;
use std::path::{Path, PathBuf};
use std::time::Duration;

use nix::sys::termios::{tcgetattr, tcsetattr, ControlFlags, SetArg};

use fs2::FileExt;
use realityos_plant::{
    ActionParams, HardwareDriverPort, HardwareIdentity, PlantError, PlantRealized, PlantResult,
    SensorPacket,
};
use serialport::SerialPort;

use crate::config::{
    candidate_bauds, candidate_servo_ids, discover_baud_attempts, MetalConfig, BUS_DIR, GOAL_FILE,
    LOCK_FILE, MOVING_FILE, PRESENT_FILE, VIN_FILE,
};
use crate::egress::EgressLog;
use crate::identity::{
    adapter_identity_aliases, adapter_still_same_as_latched, device_node_identity, is_pty_path,
    rematch_discover_device, usb_identity_for_tty, MeasuredIdentity,
};
use crate::protocol::{
    decode_status_scan, encode_ping, encode_read, encode_reboot, encode_write, find_header,
    instruction_ok, is_xl330_model, le_i32, le_u16, le_u32, unique_status_ids, ProtocolError,
    ADDR_BUS_WATCHDOG, ADDR_CURRENT_LIMIT, ADDR_DRIVE_MODE, ADDR_FEEDFORWARD_1ST,
    ADDR_FEEDFORWARD_2ND, ADDR_FIRMWARE_VERSION, ADDR_GOAL_POSITION, ADDR_HARDWARE_ERROR,
    ADDR_HOMING_OFFSET, ADDR_ID, ADDR_MAX_POSITION_LIMIT, ADDR_MAX_VOLTAGE_LIMIT,
    ADDR_MIN_POSITION_LIMIT, ADDR_MIN_VOLTAGE_LIMIT, ADDR_MODEL_NUMBER, ADDR_MOVING,
    ADDR_MOVING_THRESHOLD, ADDR_OPERATING_MODE, ADDR_POSITION_P_GAIN, ADDR_PRESENT_POSITION,
    ADDR_PRESENT_VOLTAGE, ADDR_PROFILE_ACCEL, ADDR_PROFILE_VELOCITY, ADDR_PROTOCOL_TYPE,
    ADDR_PWM_LIMIT, ADDR_REALTIME_TICK, ADDR_SECONDARY_ID, ADDR_STATUS_RETURN_LEVEL,
    ADDR_TORQUE_ENABLE, ADDR_VELOCITY_I_GAIN, ADDR_VELOCITY_LIMIT, ADDR_VELOCITY_P_GAIN,
    BROADCAST_ID, DRIVE_MODE_VELOCITY_BASED, FACTORY_MOVING_THRESHOLD, FACTORY_POSITION_P_GAIN,
    FACTORY_PWM_LIMIT, FACTORY_VELOCITY_I_GAIN, FACTORY_VELOCITY_P_GAIN, MIN_POSITION_P_GAIN,
    MIN_PWM_LIMIT, MIN_VELOCITY_I_GAIN, MIN_VELOCITY_P_GAIN, OPERATING_MODE_POSITION,
    PROTOCOL_TYPE_2, SECONDARY_ID_DISABLED, STATUS_RETURN_ALL,
};

pub struct Xl330Driver {
    port: Option<Box<dyn SerialPort>>,
    #[allow(dead_code)]
    lock: File,
    cfg: MetalConfig,
    bus: PathBuf,
    egress: EgressLog,
    connected: bool,
    estop: bool,
    last_identity: HardwareIdentity,
    last_tick_s: f64,
    last_present: i32,
    last_goal: Option<i32>,
    model: u16,
    firmware: u8,
    seq: u64,
    torque_enabled: bool,
    /// After identify+limits, one serial attempt / 40 ms timeout so a USB-UART
    /// propose stays inside the 100 ms ONLINE software-watchdog miss.
    live_io: bool,
    /// Adapter identity captured while `cfg.device` still resolved.
    /// chmod / the first exclusive open emit udev change; by-id then
    /// dangles and ttyUSB0 can rename. Re-reading that vanished name
    /// blanks serial: serve dies as metal_serial_mismatch after ping,
    /// or the first hold dies as identity mismatch while this fd is live.
    latched_usb_serial: Option<String>,
    latched_usb_fallback: Option<String>,
    latched_node: Option<String>,
    latched_pty: bool,
    last_hw_error: u8,
    min_position: i32,
    max_position: i32,
    velocity_limit: u32,
    drive_mode: u8,
    position_p_gain: u16,
    velocity_p_gain: u16,
    velocity_i_gain: u16,
    pwm_limit: u16,
    homing_offset: i32,
    bus_watchdog: u8,
    moving_threshold: u32,
    protocol_type: u8,
    feedforward_1st: u16,
    feedforward_2nd: u16,
}

impl Xl330Driver {
    pub fn open(cfg: MetalConfig, root: impl AsRef<Path>) -> io::Result<Self> {
        Self::open_inner(cfg, root, true)
    }

    fn open_inner(
        cfg: MetalConfig,
        root: impl AsRef<Path>,
        apply_limits: bool,
    ) -> io::Result<Self> {
        let bus = root.as_ref().join(BUS_DIR);
        std::fs::create_dir_all(&bus)?;
        let _ = std::fs::set_permissions(&bus, std::fs::Permissions::from_mode(0o700));
        let lock_path = bus.join(LOCK_FILE);
        let lock = OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .truncate(false)
            .mode(0o600)
            .open(&lock_path)?;
        lock.try_lock_exclusive()?;
        let egress = EgressLog::open(&bus)?;
        let mut driver = Self {
            port: None,
            lock,
            cfg,
            bus,
            egress,
            connected: false,
            estop: false,
            last_identity: HardwareIdentity {
                serial: String::new(),
                firmware_id: String::new(),
                calibration_id: String::new(),
                design_content_hash: String::new(),
                connected: false,
                metal: true,
                evidence_status: "MEASURED_XL330_PROTOCOL2".into(),
                actuator_ids: Vec::new(),
            },
            last_tick_s: 0.0,
            last_present: 0,
            last_goal: None,
            model: 0,
            firmware: 0,
            seq: 0,
            torque_enabled: false,
            live_io: false,
            last_hw_error: 0,
            min_position: 0,
            max_position: 4095,
            velocity_limit: 0,
            drive_mode: 0,
            position_p_gain: 0,
            velocity_p_gain: 0,
            velocity_i_gain: 0,
            pwm_limit: 0,
            homing_offset: 0,
            bus_watchdog: 0,
            moving_threshold: 0,
            protocol_type: 0,
            feedforward_1st: 0,
            feedforward_2nd: 0,
            latched_usb_serial: None,
            latched_usb_fallback: None,
            latched_node: None,
            latched_pty: false,
        };
        driver.connect_serial()?;
        driver.refresh_identity();
        if driver.connected && apply_limits {
            driver
                .apply_bench_limits()
                .map_err(|e| io::Error::other(e.to_string()))?;
            driver.enter_live_io();
        } else if driver.connected {
            // Startup Configuration can enable torque after a DTR reboot.
            // Identify-only must not leave the horn tracking a stale goal.
            driver.quiesce_found_torque();
        }
        Ok(driver)
    }

    /// Probe-only: try configured baud/id first, then common XL330 bus settings.
    /// Production `serve` keeps using [`Self::open`] with the bound config.
    pub fn open_discovering(
        mut cfg: MetalConfig,
        root: impl AsRef<Path>,
    ) -> io::Result<(Self, MetalConfig)> {
        if !cfg.device.exists() {
            return Err(io::Error::new(
                io::ErrorKind::NotFound,
                format!("metal_device_missing:{}", cfg.device.display()),
            ));
        }
        // Latch adapter aliases before the first open. Each sniff/open can
        // emit udev change; FTDI/U2D2 by-id then dangles for 1–3 s. Sniff
        // used to treat !exists as "no servo at this baud" and skip the
        // rest of the scan instead of rematching ttyUSB1.
        let latched_aliases = adapter_identity_aliases(&cfg.device, cfg.servo_id);
        let extra_baud = std::env::var("REALITYOS_METAL_BAUD")
            .ok()
            .and_then(|s| s.parse().ok());
        let extra_id = std::env::var("REALITYOS_METAL_SERVO_ID")
            .ok()
            .and_then(|s| s.parse().ok());
        let bauds = discover_baud_attempts(&candidate_bauds(cfg.baud, extra_baud));
        let ids = candidate_servo_ids(cfg.servo_id, extra_id);
        let mut last_err: Option<io::Error> = None;
        for baud in bauds {
            let live = rematch_discover_device(cfg.device.clone(), &latched_aliases, cfg.servo_id);
            if live != cfg.device {
                eprintln!(
                    "metal-discover: rematched {} -> {}",
                    cfg.device.display(),
                    live.display()
                );
                cfg.device = live;
            }
            if !cfg.device.exists() {
                last_err = Some(io::Error::new(
                    io::ErrorKind::NotFound,
                    format!("metal_device_missing:{}", cfg.device.display()),
                ));
                std::thread::sleep(Duration::from_millis(100));
                continue;
            }
            // Wizard may leave a non-1/2 ID. Broadcast PING still answers at
            // SRL=0 and the status carries the servo's own ID.
            let sniffed = sniff_servo_ids(&cfg.device, baud)?;
            let try_ids = prefer_servo_id(&ids, sniffed.first().copied());
            for id in try_ids {
                let mut attempt = cfg.clone();
                attempt.baud = baud;
                attempt.servo_id = id;
                match Self::open_inner(attempt.clone(), root.as_ref(), false) {
                    Ok(driver) => return Ok((driver, attempt)),
                    Err(e) if e.kind() == io::ErrorKind::WouldBlock => return Err(e),
                    Err(e) => {
                        last_err = Some(e);
                        // U2D2/FTDI often NAKs the next open if we reopen at a
                        // new baud immediately after a failed ping.
                        std::thread::sleep(Duration::from_millis(100));
                    }
                }
            }
        }
        // Configured pair is first, when the adapter is coldest. One more
        // attempt after the scan has opened the tty — discover does not
        // retry a pair, so a single cold first ping would skip the real bus.
        let configured = cfg.clone();
        match Self::open_inner(configured.clone(), root.as_ref(), false) {
            Ok(driver) => Ok((driver, configured)),
            Err(e) if e.kind() == io::ErrorKind::WouldBlock => Err(e),
            Err(e) => Err(last_err.unwrap_or(e)),
        }
    }

    pub fn bus_dir(&self) -> &Path {
        &self.bus
    }

    pub fn last_present_position(&self) -> i32 {
        self.last_present
    }

    pub fn last_goal_position(&self) -> Option<i32> {
        self.last_goal
    }

    pub fn applied_velocity_limit(&self) -> u32 {
        self.velocity_limit
    }

    pub fn applied_drive_mode(&self) -> u8 {
        self.drive_mode
    }

    pub fn applied_position_p_gain(&self) -> u16 {
        self.position_p_gain
    }

    pub fn applied_velocity_p_gain(&self) -> u16 {
        self.velocity_p_gain
    }

    pub fn applied_velocity_i_gain(&self) -> u16 {
        self.velocity_i_gain
    }

    pub fn applied_pwm_limit(&self) -> u16 {
        self.pwm_limit
    }

    pub fn applied_homing_offset(&self) -> i32 {
        self.homing_offset
    }

    pub fn applied_bus_watchdog(&self) -> u8 {
        self.bus_watchdog
    }

    pub fn applied_moving_threshold(&self) -> u32 {
        self.moving_threshold
    }

    pub fn applied_protocol_type(&self) -> u8 {
        self.protocol_type
    }

    pub fn applied_feedforward_1st(&self) -> u16 {
        self.feedforward_1st
    }

    pub fn applied_feedforward_2nd(&self) -> u16 {
        self.feedforward_2nd
    }

    pub fn torque_is_enabled(&self) -> bool {
        self.torque_enabled
    }

    pub fn measured(&self) -> MeasuredIdentity {
        let (usb_serial, usb_fallback, node) = if self.port.is_some() {
            (
                self.latched_usb_serial.clone(),
                self.latched_usb_fallback.clone(),
                self.latched_node.clone(),
            )
        } else {
            let (usb, fb) = usb_identity_for_tty(&self.cfg.device);
            (usb, fb, device_node_identity(&self.cfg.device))
        };
        MeasuredIdentity::from_adapter(
            &self.cfg,
            usb_serial,
            usb_fallback,
            node,
            self.model,
            self.firmware,
            self.connected,
        )
    }

    fn latch_open_adapter_identity(&mut self) {
        // Caller must invoke this before chmod/open, while cfg.device still
        // resolves. usb_identity_for_tty canonicalizes a live by-id.
        let (usb, fb) = usb_identity_for_tty(&self.cfg.device);
        self.latched_usb_serial = usb;
        self.latched_usb_fallback = fb;
        self.latched_node = device_node_identity(&self.cfg.device);
        self.latched_pty = is_pty_path(&self.cfg.device);
    }

    fn latched_adapter_aliases(&self) -> Vec<String> {
        MeasuredIdentity::from_adapter(
            &self.cfg,
            self.latched_usb_serial.clone(),
            self.latched_usb_fallback.clone(),
            self.latched_node.clone(),
            0,
            0,
            false,
        )
        .adapter_aliases()
    }

    fn connect_serial(&mut self) -> io::Result<()> {
        if !self.cfg.device.exists() {
            self.connected = false;
            return Err(io::Error::new(
                io::ErrorKind::NotFound,
                format!("metal_device_missing:{}", self.cfg.device.display()),
            ));
        }
        // Latch while the path still resolves. Campaign stabilize prefers
        // `/dev/serial/by-id` on FTDI/U2D2. chmod + the first exclusive
        // open emit udev change; that symlink then dangles for 1–3 s.
        // Latching after open used to re-walk the vanished name, blank
        // serial, and fail serve as metal_serial_mismatch after a good ping.
        self.latch_open_adapter_identity();
        // A no-op chmod still emits udev change on typical Ubuntu — the
        // same class as campaign chown resetting FTDI latency_timer to
        // 16 ms before the first live hold. Skip when already 0600.
        {
            let mode = std::fs::metadata(&self.cfg.device)
                .map(|m| m.permissions().mode() & 0o777)
                .unwrap_or(0);
            if mode != 0o600 {
                let _ = std::fs::set_permissions(
                    &self.cfg.device,
                    std::fs::Permissions::from_mode(0o600),
                );
            }
        }
        // PTY stand-in: TIOCEXCL survives process::exit (crash_if) and the
        // next serve gets EBUSY. Sidecar flock still serializes. Real tty
        // keeps exclusive (TIOCEXCL+flock).
        let port = open_xl330_serial(&self.cfg.device, self.cfg.baud)?;
        self.port = Some(port);
        // Path still present after open: it must still be the bound adapter.
        // chmod/open can recycle ttyUSB0 onto a different UART. The pre-open
        // latch would then name adapter A while this fd is adapter B.
        // A vanished by-id / renamed tty keeps the latch (same fd).
        // Exact expected_serial alone false-refuses FTDI when iSerial
        // flaps empty after open and only dest remains.
        if !self.cfg.expected_serial.trim().is_empty()
            && !adapter_still_same_as_latched(
                &self.cfg.device,
                &self.cfg.expected_serial,
                &self.latched_adapter_aliases(),
                self.cfg.servo_id,
            )
        {
            self.port = None;
            self.connected = false;
            return Err(io::Error::other(format!(
                "metal_adapter_recycled_after_open:expected={} actual={} device={}",
                self.cfg.expected_serial,
                crate::identity::adapter_serial_for_tty(&self.cfg.device, self.cfg.servo_id),
                self.cfg.device.display()
            )));
        }
        match self.ping_and_identify() {
            Ok(()) => {
                self.connected = true;
                Ok(())
            }
            Err(e) => {
                self.connected = false;
                Err(e)
            }
        }
    }

    fn ping_and_identify(&mut self) -> io::Result<()> {
        let _ = self.xfer(&encode_ping(self.cfg.servo_id), true)?;
        // Wizard can set Status Return Level to 0 (PING only). Then READ and
        // WRITE have no status and identify/setup fail. Poke 2 without
        // requiring an ack — there may be no status packet to read.
        self.force_status_return_all()?;
        let model_pkt = self.xfer(&encode_read(self.cfg.servo_id, ADDR_MODEL_NUMBER, 2), true)?;
        let model = le_u16(&model_pkt.params).unwrap_or(0);
        let fw_pkt = self.xfer(
            &encode_read(self.cfg.servo_id, ADDR_FIRMWARE_VERSION, 1),
            true,
        )?;
        let fw = fw_pkt.params.first().copied().unwrap_or(0);
        self.model = model;
        self.firmware = fw;
        if !is_xl330_model(model) {
            return Err(io::Error::other(format!(
                "metal_refuses_non_xl330_model:{model}"
            )));
        }
        Ok(())
    }

    fn force_status_return_all(&mut self) -> io::Result<()> {
        let frame = encode_write(
            self.cfg.servo_id,
            ADDR_STATUS_RETURN_LEVEL,
            &[STATUS_RETURN_ALL],
        );
        let port = self
            .port
            .as_mut()
            .ok_or_else(|| io::Error::other("metal_serial_closed"))?;
        let saved = port.timeout();
        port.clear(serialport::ClearBuffer::Input)
            .map_err(io::Error::other)?;
        port.write_all(&frame).map_err(io::Error::other)?;
        port.flush().map_err(io::Error::other)?;
        half_duplex_turnaround(&self.cfg.device);
        // Factory SRL=2 replies; Wizard SRL=0 does not. Consume an optional
        // status so a late USB packet is not decoded as the model READ.
        // 25 ms > default FTDI latency_timer (16 ms).
        port.set_timeout(Duration::from_millis(25))
            .map_err(io::Error::other)?;
        let mut acc = Vec::new();
        let mut tmp = [0u8; 64];
        let end = std::time::Instant::now() + Duration::from_millis(25);
        while std::time::Instant::now() < end {
            match port.read(&mut tmp) {
                Ok(0) => {}
                Ok(n) => {
                    acc.extend_from_slice(&tmp[..n]);
                    if decode_status_scan(&acc).is_ok() {
                        break;
                    }
                }
                Err(e) if e.kind() == io::ErrorKind::TimedOut => break,
                Err(e) => {
                    let _ = port.set_timeout(saved);
                    return Err(e);
                }
            }
        }
        port.clear(serialport::ClearBuffer::Input)
            .map_err(io::Error::other)?;
        port.set_timeout(saved).map_err(io::Error::other)?;
        Ok(())
    }

    fn apply_bench_limits(&mut self) -> PlantResult<()> {
        // Current limit / operating mode are EEPROM; only write with torque off, and only if needed.
        self.write_reg(ADDR_TORQUE_ENABLE, &[0], "setup_torque_off", None, false)?;
        self.torque_enabled = false;
        // Protocol 2.0 sets STATUS_ALERT on every packet while Hardware Error
        // Status is latched (Wizard overload, VIN blip). That is not a NAK.
        // Reboot once *before* RAM profile writes — reboot clears RAM.
        if let Ok(b) = self.read_reg(ADDR_HARDWARE_ERROR, 1) {
            self.last_hw_error = b.first().copied().unwrap_or(0);
        }
        if self.last_hw_error != 0 {
            let _ = self.xfer(&encode_reboot(self.cfg.servo_id), true);
            std::thread::sleep(Duration::from_millis(400));
            self.ping_and_identify()
                .map_err(|e| PlantError::refused(format!("dxl_reboot_identify:{e}")))?;
            // Startup Configuration can re-enable torque after reboot.
            // EEPROM writes then access-NAK, and a stale Wizard goal moves.
            self.write_reg(
                ADDR_TORQUE_ENABLE,
                &[0],
                "setup_torque_off_after_reboot",
                None,
                false,
            )?;
            self.torque_enabled = false;
            self.last_hw_error = self
                .read_reg(ADDR_HARDWARE_ERROR, 1)
                .ok()
                .and_then(|b| b.first().copied())
                .unwrap_or(self.last_hw_error);
            if self.last_hw_error != 0 {
                return Err(PlantError::refused(format!(
                    "dxl_hardware_error_latched:{}",
                    self.last_hw_error
                )));
            }
        }
        let mut eeprom_changed = false;
        let secondary = self
            .read_reg(ADDR_SECONDARY_ID, 1)
            .ok()
            .and_then(|b| b.first().copied());
        if secondary != Some(SECONDARY_ID_DISABLED) {
            self.write_reg(
                ADDR_SECONDARY_ID,
                &[SECONDARY_ID_DISABLED],
                "setup_secondary_id_off",
                None,
                false,
            )?;
            eeprom_changed = true;
        }
        let proto = self
            .read_reg(ADDR_PROTOCOL_TYPE, 1)
            .ok()
            .and_then(|b| b.first().copied());
        self.protocol_type = proto.unwrap_or(0);
        // Wizard 20/21/22 is S.BUS / iBUS / RC-PWM. RC-detected boot auto
        // torque-ons and stops speaking Protocol 2.0. A no-RC fallback still
        // talks 2.0; write 2 so a line glitch cannot switch mid-campaign.
        if proto != Some(PROTOCOL_TYPE_2) {
            self.write_reg(
                ADDR_PROTOCOL_TYPE,
                &[PROTOCOL_TYPE_2],
                "setup_protocol_type_2",
                None,
                false,
            )?;
            self.protocol_type = PROTOCOL_TYPE_2;
            eeprom_changed = true;
        }
        let drive = self
            .read_reg(ADDR_DRIVE_MODE, 1)
            .ok()
            .and_then(|b| b.first().copied());
        self.drive_mode = drive.unwrap_or(0);
        if drive != Some(DRIVE_MODE_VELOCITY_BASED) {
            self.write_reg(
                ADDR_DRIVE_MODE,
                &[DRIVE_MODE_VELOCITY_BASED],
                "setup_drive_mode_velocity",
                None,
                false,
            )?;
            self.drive_mode = DRIVE_MODE_VELOCITY_BASED;
            eeprom_changed = true;
        }
        let mode = self
            .read_reg(ADDR_OPERATING_MODE, 1)
            .ok()
            .and_then(|b| b.first().copied());
        if mode != Some(OPERATING_MODE_POSITION) {
            self.write_reg(
                ADDR_OPERATING_MODE,
                &[OPERATING_MODE_POSITION],
                "setup_operating_mode_position",
                None,
                false,
            )?;
            eeprom_changed = true;
        }
        if eeprom_changed {
            // EEPROM mode/drive writes can drop the next RAM instruction.
            std::thread::sleep(Duration::from_millis(400));
            self.ping_and_identify()
                .map_err(|e| PlantError::refused(format!("dxl_mode_identify:{e}")))?;
        }
        self.refresh_position_limits()?;
        let want = self.cfg.current_limit_milli;
        let got = self
            .read_reg(ADDR_CURRENT_LIMIT, 2)
            .ok()
            .and_then(|b| le_u16(&b));
        if got != Some(want) {
            self.write_reg(
                ADDR_CURRENT_LIMIT,
                &want.to_le_bytes(),
                "setup_current_limit",
                None,
                false,
            )?;
        }
        let want_vel = self.cfg.max_profile_velocity;
        let got_vel = self
            .read_reg(ADDR_VELOCITY_LIMIT, 4)
            .ok()
            .and_then(|b| le_u32(&b))
            .unwrap_or(0);
        self.velocity_limit = got_vel;
        if got_vel < want_vel {
            self.write_reg(
                ADDR_VELOCITY_LIMIT,
                &want_vel.to_le_bytes(),
                "setup_velocity_limit",
                None,
                false,
            )?;
            self.velocity_limit = want_vel;
        }
        self.write_reg(
            ADDR_PROFILE_VELOCITY,
            &self.cfg.max_profile_velocity.to_le_bytes(),
            "setup_profile_velocity",
            None,
            false,
        )?;
        self.write_reg(
            ADDR_PROFILE_ACCEL,
            &self.cfg.max_profile_acceleration.to_le_bytes(),
            "setup_profile_accel",
            None,
            false,
        )?;
        let got_mt = self
            .read_reg(ADDR_MOVING_THRESHOLD, 4)
            .ok()
            .and_then(|b| le_u32(&b))
            .unwrap_or(0);
        self.moving_threshold = got_mt;
        // Moving=1 only while |velocity| > this. A Wizard value ≥ profile
        // velocity keeps Moving=0 for the whole 32-tick nudge. Campaign
        // settle waits for present≈goal, but lower the threshold so Moving
        // is still a usable in-motion flag.
        if got_mt > self.cfg.max_profile_velocity {
            self.write_reg(
                ADDR_MOVING_THRESHOLD,
                &FACTORY_MOVING_THRESHOLD.to_le_bytes(),
                "setup_moving_threshold",
                None,
                false,
            )?;
            self.moving_threshold = FACTORY_MOVING_THRESHOLD;
        }
        let got_p = self
            .read_reg(ADDR_POSITION_P_GAIN, 2)
            .ok()
            .and_then(|b| le_u16(&b))
            .unwrap_or(0);
        self.position_p_gain = got_p;
        if got_p < MIN_POSITION_P_GAIN {
            self.write_reg(
                ADDR_POSITION_P_GAIN,
                &FACTORY_POSITION_P_GAIN.to_le_bytes(),
                "setup_position_p_gain",
                None,
                false,
            )?;
            self.position_p_gain = FACTORY_POSITION_P_GAIN;
        }
        let got_vp = self
            .read_reg(ADDR_VELOCITY_P_GAIN, 2)
            .ok()
            .and_then(|b| le_u16(&b))
            .unwrap_or(0);
        self.velocity_p_gain = got_vp;
        if got_vp < MIN_VELOCITY_P_GAIN {
            self.write_reg(
                ADDR_VELOCITY_P_GAIN,
                &FACTORY_VELOCITY_P_GAIN.to_le_bytes(),
                "setup_velocity_p_gain",
                None,
                false,
            )?;
            self.velocity_p_gain = FACTORY_VELOCITY_P_GAIN;
        }
        let got_vi = self
            .read_reg(ADDR_VELOCITY_I_GAIN, 2)
            .ok()
            .and_then(|b| le_u16(&b))
            .unwrap_or(0);
        self.velocity_i_gain = got_vi;
        if got_vi < MIN_VELOCITY_I_GAIN {
            self.write_reg(
                ADDR_VELOCITY_I_GAIN,
                &FACTORY_VELOCITY_I_GAIN.to_le_bytes(),
                "setup_velocity_i_gain",
                None,
                false,
            )?;
            self.velocity_i_gain = FACTORY_VELOCITY_I_GAIN;
        }
        let got_pwm = self
            .read_reg(ADDR_PWM_LIMIT, 2)
            .ok()
            .and_then(|b| le_u16(&b))
            .unwrap_or(0);
        self.pwm_limit = got_pwm;
        if got_pwm < MIN_PWM_LIMIT {
            self.write_reg(
                ADDR_PWM_LIMIT,
                &FACTORY_PWM_LIMIT.to_le_bytes(),
                "setup_pwm_limit",
                None,
                false,
            )?;
            self.pwm_limit = FACTORY_PWM_LIMIT;
        }
        let ff2 = self
            .read_reg(ADDR_FEEDFORWARD_2ND, 2)
            .ok()
            .and_then(|b| le_u16(&b))
            .unwrap_or(0);
        let ff1 = self
            .read_reg(ADDR_FEEDFORWARD_1ST, 2)
            .ok()
            .and_then(|b| le_u16(&b))
            .unwrap_or(0);
        self.feedforward_2nd = ff2;
        self.feedforward_1st = ff1;
        // Factory 0. Wizard feedforward makes the certified 32-tick step
        // overshoot; that is not a tiny bounded nudge.
        if ff1 != 0 || ff2 != 0 {
            self.write_reg(
                ADDR_FEEDFORWARD_2ND,
                &0u16.to_le_bytes(),
                "setup_feedforward_2nd_zero",
                None,
                false,
            )?;
            self.write_reg(
                ADDR_FEEDFORWARD_1ST,
                &0u16.to_le_bytes(),
                "setup_feedforward_1st_zero",
                None,
                false,
            )?;
            self.feedforward_2nd = 0;
            self.feedforward_1st = 0;
        }
        // EEPROM writes can NAK the next instruction if we immediately continue.
        std::thread::sleep(Duration::from_millis(50));
        let max_v = self
            .read_reg(ADDR_MAX_VOLTAGE_LIMIT, 2)
            .ok()
            .and_then(|b| le_u16(&b))
            .filter(|v| *v != 0)
            .ok_or_else(|| PlantError::refused("dxl_voltage_limits_unreadable_before_torque"))?;
        let min_v = self
            .read_reg(ADDR_MIN_VOLTAGE_LIMIT, 2)
            .ok()
            .and_then(|b| le_u16(&b))
            .filter(|v| *v != 0)
            .ok_or_else(|| PlantError::refused("dxl_voltage_limits_unreadable_before_torque"))?;
        let vin = self
            .read_reg(ADDR_PRESENT_VOLTAGE, 2)
            .ok()
            .and_then(|b| le_u16(&b))
            .filter(|v| *v != 0)
            .ok_or_else(|| PlantError::refused("dxl_vin_unreadable_before_torque"))?;
        self.persist_vin(vin);
        if vin < min_v || vin > max_v {
            return Err(PlantError::refused(format!(
                "dxl_vin_outside_wizard_limits:vin_0.1v={vin}:min={min_v}:max={max_v}"
            )));
        }
        // Wizard Bus Watchdog (20 ms units). Non-zero trips after a quiet
        // gap and latches 0xFF; Goal Position then NAKs with data-range.
        let wd = self
            .read_reg(ADDR_BUS_WATCHDOG, 1)
            .ok()
            .and_then(|b| b.first().copied())
            .unwrap_or(0);
        self.bus_watchdog = wd;
        if wd != 0 {
            self.write_reg(
                ADDR_BUS_WATCHDOG,
                &[0],
                "setup_bus_watchdog_off",
                None,
                false,
            )?;
            self.bus_watchdog = 0;
        }
        // Torque-on tracks Goal Position. A stale Wizard goal (often 0)
        // would move before any certified command. Match present first.
        // Not counted as command egress.
        let mut present = self
            .read_reg(ADDR_PRESENT_POSITION, 4)
            .ok()
            .and_then(|b| le_i32(&b))
            .ok_or_else(|| PlantError::refused("dxl_present_unreadable_before_torque"))?;
        // Wizard Homing Offset shifts Present without moving the horn.
        // Limits stay 0–4095, so a "zeroed" horn is outside the window and
        // goal=present would NAK. Clearing offset with torque off is not motion.
        let offset = self
            .read_reg(ADDR_HOMING_OFFSET, 4)
            .ok()
            .and_then(|b| le_i32(&b))
            .unwrap_or(0);
        self.homing_offset = offset;
        if (present < self.min_position || present > self.max_position) && offset != 0 {
            self.write_reg(
                ADDR_HOMING_OFFSET,
                &0i32.to_le_bytes(),
                "setup_homing_offset_zero",
                None,
                false,
            )?;
            self.homing_offset = 0;
            std::thread::sleep(Duration::from_millis(50));
            present = self
                .read_reg(ADDR_PRESENT_POSITION, 4)
                .ok()
                .and_then(|b| le_i32(&b))
                .ok_or_else(|| PlantError::refused("dxl_present_unreadable_before_torque"))?;
        }
        // Do not yank present onto the Wizard window. Clamping then
        // torque-on would move before any certified command and break
        // the zero-motion baseline.
        if present < self.min_position || present > self.max_position {
            return Err(PlantError::refused(format!(
                "dxl_present_outside_wizard_limits:present={present}:min={}:max={}:homing_offset={}",
                self.min_position, self.max_position, self.homing_offset
            )));
        }
        self.last_present = present;
        self.write_reg(
            ADDR_GOAL_POSITION,
            &present.to_le_bytes(),
            "setup_goal_match_present",
            Some(present),
            false,
        )?;
        self.last_goal = Some(present);
        self.persist_positions();
        // Torque-on here (software watchdog not running yet) so the first
        // certified write is a single goal_position xfer, not torque_on + goal.
        self.write_reg(ADDR_TORQUE_ENABLE, &[1], "setup_torque_on", None, false)?;
        // A tight fixture / overload Shutdown can drop torque immediately.
        // Then a certified nudge writes a goal and present never moves.
        let still_on = self
            .read_reg(ADDR_TORQUE_ENABLE, 1)
            .ok()
            .and_then(|b| b.first().copied());
        if still_on != Some(1) {
            self.torque_enabled = false;
            return Err(PlantError::refused(format!(
                "dxl_torque_dropped_after_enable:{}",
                still_on.unwrap_or(0)
            )));
        }
        let hw = self
            .read_reg(ADDR_HARDWARE_ERROR, 1)
            .ok()
            .and_then(|b| b.first().copied());
        let Some(hw) = hw else {
            let _ = self.write_reg(
                ADDR_TORQUE_ENABLE,
                &[0],
                "setup_torque_off_hw_unread",
                None,
                false,
            );
            self.torque_enabled = false;
            return Err(PlantError::refused("dxl_hw_error_unreadable_after_torque"));
        };
        self.last_hw_error = hw;
        if hw != 0 {
            let _ = self.write_reg(
                ADDR_TORQUE_ENABLE,
                &[0],
                "setup_torque_off_hw_error",
                None,
                false,
            );
            self.torque_enabled = false;
            return Err(PlantError::refused(format!(
                "dxl_hardware_error_after_torque_on:{hw}"
            )));
        }
        self.torque_enabled = true;
        // Robotis: Present resets to absolute-within-one-rotation when
        // torque turns on in Position Control. Goal still holds the
        // pre-reset value and the horn yanks before any certified write.
        let after = match self
            .read_reg(ADDR_PRESENT_POSITION, 4)
            .ok()
            .and_then(|b| le_i32(&b))
        {
            Some(p) => p,
            None => {
                let _ = self.write_reg(
                    ADDR_TORQUE_ENABLE,
                    &[0],
                    "setup_torque_off_present_unread",
                    None,
                    false,
                );
                self.torque_enabled = false;
                return Err(PlantError::refused("dxl_present_unreadable_after_torque"));
            }
        };
        if after != self.last_present {
            if after < self.min_position || after > self.max_position {
                let _ = self.write_reg(
                    ADDR_TORQUE_ENABLE,
                    &[0],
                    "setup_torque_off_present_jump",
                    None,
                    false,
                );
                self.torque_enabled = false;
                return Err(PlantError::refused(format!(
                    "dxl_present_outside_wizard_limits_after_torque:present={after}:min={}:max={}",
                    self.min_position, self.max_position
                )));
            }
            self.write_reg(
                ADDR_GOAL_POSITION,
                &after.to_le_bytes(),
                "setup_goal_match_present_after_torque",
                Some(after),
                false,
            )?;
            self.last_present = after;
            self.last_goal = Some(after);
            self.persist_positions();
        }
        Ok(())
    }

    /// Startup Configuration torque-on (after DTR reboot) tracks the last
    /// Wizard goal. Probe does not enable torque; it only writes 0 if the
    /// register is already 1. Not command egress.
    fn quiesce_found_torque(&mut self) {
        let on = self
            .read_reg(ADDR_TORQUE_ENABLE, 1)
            .ok()
            .and_then(|b| b.first().copied())
            == Some(1);
        if on {
            let _ = self.write_reg(
                ADDR_TORQUE_ENABLE,
                &[0],
                "probe_quiesce_torque",
                None,
                false,
            );
        }
        self.torque_enabled = false;
    }

    fn refresh_position_limits(&mut self) -> PlantResult<()> {
        let max_b = self.read_reg(ADDR_MAX_POSITION_LIMIT, 4)?;
        let min_b = self.read_reg(ADDR_MIN_POSITION_LIMIT, 4)?;
        let max =
            le_i32(&max_b).ok_or_else(|| PlantError::refused("dxl_position_limits_unreadable"))?;
        let min =
            le_i32(&min_b).ok_or_else(|| PlantError::refused("dxl_position_limits_unreadable"))?;
        if !(0..=4095).contains(&min) || !(0..=4095).contains(&max) || min > max {
            return Err(PlantError::refused(format!(
                "dxl_position_limits_invalid:min={min}:max={max}"
            )));
        }
        self.min_position = min;
        self.max_position = max;
        Ok(())
    }

    fn enter_live_io(&mut self) {
        self.live_io = true;
        if let Some(port) = self.port.as_mut() {
            let _ = port.set_timeout(Duration::from_millis(40));
        }
    }

    fn refresh_identity(&mut self) {
        let pty = if self.port.is_some() {
            self.latched_pty
        } else {
            is_pty_path(&self.cfg.device)
        };
        let mut id = self.measured().hardware_identity_class(&self.cfg, pty);
        if self.campaign_disconnected() {
            // Identity overlay only. Keep the serial path up so propose
            // reaches verify_live_hardware instead of failing acquire first.
            id.connected = false;
        }
        if let Some(overlay) = self.campaign_hot_swap() {
            if let Some(s) = overlay.get("serial").and_then(|v| v.as_str()) {
                id.serial = s.to_string();
            }
            if let Some(s) = overlay.get("firmware_id").and_then(|v| v.as_str()) {
                id.firmware_id = s.to_string();
            }
            if let Some(s) = overlay.get("calibration_id").and_then(|v| v.as_str()) {
                id.calibration_id = s.to_string();
            }
            if let Some(s) = overlay.get("design_content_hash").and_then(|v| v.as_str()) {
                id.design_content_hash = s.to_string();
            }
        }
        if !self.bus_up() {
            id.connected = false;
        }
        self.last_identity = id;
    }

    fn persist_positions(&self) {
        let _ = std::fs::write(self.bus.join(PRESENT_FILE), self.last_present.to_string());
        if let Some(g) = self.last_goal {
            let _ = std::fs::write(self.bus.join(GOAL_FILE), g.to_string());
        }
    }

    fn persist_vin(&self, vin_tenth: u16) {
        let _ = std::fs::write(self.bus.join(VIN_FILE), vin_tenth.to_string());
    }

    /// Model (0) + firmware (6) + ID (7). Not a motion-block field.
    /// I/O failure must not turn a CRC glitch into `bus_lost` after a good
    /// motion sample. A real swap updates latched model/fw so verify fails.
    fn confirm_eeprom_identity(&mut self) {
        let was_connected = self.connected;
        let Ok(b) = self.read_reg(ADDR_MODEL_NUMBER, 8) else {
            self.connected = was_connected;
            return;
        };
        if b.len() < 8 {
            return;
        }
        let model = le_u16(&b[0..2]).unwrap_or(0);
        let fw = b[(ADDR_FIRMWARE_VERSION - ADDR_MODEL_NUMBER) as usize];
        let id = b[(ADDR_ID - ADDR_MODEL_NUMBER) as usize];
        if id != self.cfg.servo_id {
            // EEPROM ID no longer matches the bound bus ID. Blank firmware so
            // write-time verify fail-closes without failing this sensor.
            self.model = 0;
            self.firmware = 0;
            return;
        }
        if model != self.model || fw != self.firmware {
            self.model = model;
            self.firmware = fw;
        }
    }

    fn campaign_disconnected(&self) -> bool {
        self.cfg.campaign_hooks && self.bus.join("force_disconnect").exists()
    }

    fn campaign_fail_sensor(&self) -> bool {
        self.cfg.campaign_hooks && self.bus.join("fail_sensor").exists()
    }

    fn campaign_hot_swap(&self) -> Option<serde_json::Value> {
        if !self.cfg.campaign_hooks {
            return None;
        }
        let raw = std::fs::read_to_string(self.bus.join("hot_swap.json")).ok()?;
        serde_json::from_str(&raw).ok()
    }

    fn xfer(
        &mut self,
        request: &[u8],
        expect_status: bool,
    ) -> io::Result<crate::protocol::StatusPacket> {
        let mut last = io::Error::other("metal_xfer_empty");
        let attempts = if self.live_io { 1 } else { 2 };
        for _ in 0..attempts {
            match self.xfer_once(request, expect_status) {
                Ok(st) => return Ok(st),
                Err(e) => last = e,
            }
        }
        Err(last)
    }

    fn xfer_once(
        &mut self,
        request: &[u8],
        expect_status: bool,
    ) -> io::Result<crate::protocol::StatusPacket> {
        let port = self
            .port
            .as_mut()
            .ok_or_else(|| io::Error::other("metal_serial_closed"))?;
        port.clear(serialport::ClearBuffer::Input)
            .map_err(io::Error::other)?;
        port.write_all(request).map_err(io::Error::other)?;
        port.flush().map_err(io::Error::other)?;
        half_duplex_turnaround(&self.cfg.device);
        if !expect_status {
            return Err(io::Error::other("metal_no_status_expected"));
        }
        let mut acc = Vec::new();
        let mut tmp = [0u8; 64];
        // Setup may wait through several 150 ms timeouts. Live I/O has a 40 ms
        // *overall* deadline: 4×40 ms dribbled reads are 160 ms and latch the
        // 100 ms ONLINE software watchdog (PTY hold at authority_receive_s≈0.77).
        let deadline = self
            .live_io
            .then(|| std::time::Instant::now() + Duration::from_millis(40));
        let rounds = if self.live_io { 8 } else { 16 };
        for _ in 0..rounds {
            if let Some(end) = deadline {
                let left = end.saturating_duration_since(std::time::Instant::now());
                if left.is_zero() {
                    return Err(io::Error::new(
                        io::ErrorKind::TimedOut,
                        "metal_live_io_deadline",
                    ));
                }
                port.set_timeout(left).map_err(io::Error::other)?;
            }
            match port.read(&mut tmp) {
                Ok(0) => continue,
                Ok(n) => acc.extend_from_slice(&tmp[..n]),
                Err(e) if e.kind() == io::ErrorKind::TimedOut => {
                    if acc.len() >= 11 {
                        break;
                    }
                    if self.live_io {
                        return Err(e);
                    }
                }
                Err(e) => return Err(e),
            }
            if find_header(&acc).is_some() && acc.len() >= 11 {
                match decode_status_scan(&acc) {
                    Ok(st) => return Ok(st),
                    Err(ProtocolError::Truncated) | Err(ProtocolError::TooShort) => {}
                    Err(ProtocolError::BadCrc) => {
                        // Complete status with a bad CRC. Waiting out the
                        // 40 ms live deadline would miss the watchdog.
                        return Err(io::Error::other("dxl_bad_crc"));
                    }
                    Err(_) => {}
                }
            }
        }
        decode_status_scan(&acc).map_err(|e| io::Error::other(e.to_string()))
    }

    fn write_reg(
        &mut self,
        addr: u16,
        data: &[u8],
        instruction: &str,
        goal: Option<i32>,
        count_command_egress: bool,
    ) -> PlantResult<crate::protocol::StatusPacket> {
        if self.estop && count_command_egress {
            return Err(PlantError::EstopEngaged);
        }
        if !self.bus_up() {
            return Err(PlantError::Disconnected);
        }
        let frame = encode_write(self.cfg.servo_id, addr, data);
        if count_command_egress {
            self.egress
                .record_attempt(instruction, addr, data.len(), goal)
                .map_err(|e| PlantError::refused(format!("egress_log:{e}")))?;
            realityos_plant::hil_faults::crash_if("during_write");
        }
        match self.xfer(&frame, true) {
            Ok(st) => {
                let ok = instruction_ok(st.error);
                if count_command_egress {
                    let _ = self
                        .egress
                        .record_ack(ok, st.error, Some(self.last_present));
                }
                if !ok {
                    return Err(PlantError::refused(format!(
                        "dxl_status_error:{}",
                        st.error
                    )));
                }
                Ok(st)
            }
            Err(e) => {
                self.connected = false;
                if count_command_egress {
                    let _ = self.egress.record_ack(false, 0xFF, None);
                }
                Err(PlantError::refused(format!("dxl_io:{e}")))
            }
        }
    }

    fn read_reg(&mut self, addr: u16, len: u16) -> PlantResult<Vec<u8>> {
        if !self.bus_up() {
            return Err(PlantError::Disconnected);
        }
        let frame = encode_read(self.cfg.servo_id, addr, len);
        match self.xfer(&frame, true) {
            Ok(st) if instruction_ok(st.error) => Ok(st.params),
            Ok(st) => Err(PlantError::refused(format!(
                "dxl_status_error:{}",
                st.error
            ))),
            Err(e) => {
                self.connected = false;
                Err(PlantError::refused(format!("dxl_io:{e}")))
            }
        }
    }

    fn bus_up(&self) -> bool {
        // Campaign `force_disconnect` is an identity overlay (USB may stay
        // enumerated). Keep the serial path up so propose reaches
        // `verify_live_hardware` and FAULT/ABORTs the instance. Folding the
        // hook into bus_up failed the sensor first, left the session alive,
        // and made recover-after-disconnect a vacuous refuse.
        //
        // Do not use `cfg.device.exists()`. After serve open, udev change
        // can dangle `/dev/serial/by-id` or rename ttyUSB0 → ttyUSB1 while
        // this exclusive fd is still the live UART. That exists() miss
        // used to look like unplug and abort the first hold as disconnect.
        // A real unplug fails the next xfer and clears `connected`.
        self.connected && self.port.is_some()
    }

    /// Realtime Tick (120) through Present Input Voltage (144) is 26 bytes.
    /// One register read: a 5-read fallback would miss the 100 ms software watchdog.
    fn read_motion_block(&mut self) -> PlantResult<(i32, i32, i16, u16, u16)> {
        const LEN: u16 = 26;
        let b = self.read_reg(ADDR_REALTIME_TICK, LEN)?;
        if b.len() < LEN as usize {
            return Err(PlantError::refused("dxl_short_motion_block"));
        }
        let tick = le_u16(&b[0..2]).unwrap_or(0);
        let moving = b[(ADDR_MOVING - ADDR_REALTIME_TICK) as usize];
        let cur = i16::from_le_bytes([b[6], b[7]]);
        let vel = le_i32(&b[8..12]).unwrap_or(0);
        let pos =
            le_i32(&b[12..16]).ok_or_else(|| PlantError::refused("dxl_short_present_position"))?;
        let volt = le_u16(&b[24..26]).unwrap_or(0);
        self.last_present = pos;
        self.persist_positions();
        let _ = std::fs::write(self.bus.join(MOVING_FILE), moving.to_string());
        Ok((pos, vel, cur, volt, tick))
    }

    /// Authorized action[0]==0 → hold present. Non-zero → one bounded tick step.
    fn goal_from_action(&self, action: &[f64]) -> i32 {
        let a0 = action.first().copied().unwrap_or(0.0);
        if !a0.is_finite() || a0.abs() < 1e-12 {
            return self.last_present;
        }
        let scale = if self.cfg.tau_max.abs() < 1e-12 {
            0.0
        } else {
            f64::from(self.cfg.max_position_delta_ticks) / self.cfg.tau_max.abs()
        };
        let ticks = (a0 * scale).round().clamp(
            f64::from(-self.cfg.max_position_delta_ticks),
            f64::from(self.cfg.max_position_delta_ticks),
        ) as i32;
        // Hold also clamps: Wizard can leave present outside the EEPROM window
        // only after a limit change; writing that present would NAK.
        if ticks == 0 {
            return self
                .last_present
                .clamp(self.min_position, self.max_position);
        }
        let goal = self
            .last_present
            .saturating_add(ticks)
            .clamp(self.min_position, self.max_position);
        if goal == self.last_present {
            let inward = self
                .last_present
                .saturating_sub(ticks)
                .clamp(self.min_position, self.max_position);
            if inward != self.last_present {
                return inward;
            }
        }
        goal
    }

    /// Test-only: udev can dangle by-id / rename ttyUSB0 while the exclusive
    /// fd remains the live UART. The first hold must not treat that as unplug
    /// or blank the open-time adapter serial.
    pub fn simulate_udev_path_vanished(&mut self) {
        self.cfg.device = PathBuf::from("/dev/realityos-metal-vanished-udev-name");
    }
}

impl HardwareDriverPort for Xl330Driver {
    fn probe_identity(&self) -> HardwareIdentity {
        let mut id = self.last_identity.clone();
        if self.campaign_disconnected() || !self.bus_up() {
            id.connected = false;
        }
        if let Some(overlay) = self.campaign_hot_swap() {
            if let Some(s) = overlay.get("serial").and_then(|v| v.as_str()) {
                id.serial = s.to_string();
            }
            if let Some(s) = overlay.get("firmware_id").and_then(|v| v.as_str()) {
                id.firmware_id = s.to_string();
            }
        }
        id
    }

    fn read_sensor(&mut self, _authority_now_s: f64) -> PlantResult<SensorPacket> {
        // `_authority_now_s` is a receive hint for the governor. Capture time is
        // the device realtime tick, not the authority clock.
        if !self.bus_up() {
            return Err(PlantError::Disconnected);
        }
        if self.campaign_fail_sensor() {
            return Err(PlantError::refused("metal_sensor_missing"));
        }
        let t0 = std::time::Instant::now();
        let (pos, vel, cur, volt, tick) = self.read_motion_block()?;
        // Motion block has no model/fw. Re-read EEPROM so a physical swap on
        // this UART updates last_identity before write-time verify. Overlay
        // hot_swap still wins in refresh_identity. Skip when the motion read
        // already used most of the 40 ms live budget — a CRC miss on this
        // extra READ must not fail a good present sample (`bus_lost`).
        if self.live_io && t0.elapsed() < Duration::from_millis(15) {
            self.confirm_eeprom_identity();
        }
        let err = self.last_hw_error;
        self.persist_vin(volt);
        self.last_tick_s = f64::from(tick) / 1000.0;
        self.seq = self.seq.saturating_add(1);
        let samples = vec![
            ("q0".into(), f64::from(pos)),
            ("dq0".into(), f64::from(vel)),
            ("i0".into(), f64::from(cur)),
            ("vin_0.1v".into(), f64::from(volt)),
            ("hw_error".into(), f64::from(err)),
            ("realtime_tick_s".into(), self.last_tick_s),
        ];
        let mut pkt = SensorPacket::from_samples(samples, self.last_tick_s);
        pkt.frame_id = "xl330/joint".into();
        pkt.sensor_id = self.cfg.actuator_id();
        pkt.sequence = self.seq;
        pkt.calibration_hash = self.cfg.calibration_id.clone();
        pkt.rehash();
        self.refresh_identity();
        Ok(pkt)
    }

    fn write_action(
        &mut self,
        action: &[f64],
        _params: &ActionParams,
    ) -> PlantResult<PlantRealized> {
        if self.estop {
            return Err(PlantError::EstopEngaged);
        }
        if !self.bus_up() {
            return Err(PlantError::Disconnected);
        }
        if !is_xl330_model(self.model) {
            return Err(PlantError::refused("metal_refuses_non_xl330_model"));
        }
        // propose() already acquired sensors. Extra register pokes here would
        // exceed the 100 ms software-watchdog miss on a USB-UART bench.
        let goal = self.goal_from_action(action);
        if !self.torque_enabled {
            self.write_reg(ADDR_TORQUE_ENABLE, &[1], "torque_on", None, false)?;
            self.torque_enabled = true;
        }
        self.write_reg(
            ADDR_GOAL_POSITION,
            &goal.to_le_bytes(),
            "goal_position",
            Some(goal),
            true,
        )?;
        self.last_goal = Some(goal);
        let present = self.last_present;
        self.persist_positions();
        Ok(PlantRealized {
            ok: true,
            values: vec![
                ("q0".into(), f64::from(present)),
                ("goal".into(), f64::from(goal)),
            ],
            metal: !crate::identity::is_pty_path(&self.cfg.device),
        })
    }

    fn engage_hw_estop(&mut self, _reason: &str) {
        if self.bus_up() {
            let _ = self.write_reg(ADDR_TORQUE_ENABLE, &[0], "hw_estop_torque_off", None, false);
            self.torque_enabled = false;
        }
        self.estop = true;
    }

    fn clear_hw_estop(&mut self, operator_ack: bool) -> PlantResult<()> {
        if !operator_ack {
            return Err(PlantError::OperatorAckRequired);
        }
        self.estop = false;
        Ok(())
    }

    fn is_connected(&self) -> bool {
        self.bus_up() && !self.campaign_disconnected()
    }

    fn close(&mut self) {
        // Identify-only probe never enables torque. Writing 0 here is still a
        // physical bus write and used to increment egress before serve.
        if self.bus_up() && self.torque_enabled {
            let _ = self.write_reg(ADDR_TORQUE_ENABLE, &[0], "close_torque_off", None, false);
            self.torque_enabled = false;
        }
        self.port = None;
        self.connected = false;
        self.latched_usb_serial = None;
        self.latched_usb_fallback = None;
        self.latched_node = None;
        self.latched_pty = false;
    }

    fn is_sim_harness(&self) -> bool {
        false
    }
}

impl Drop for Xl330Driver {
    fn drop(&mut self) {
        self.close();
    }
}

/// Cheap FTDI/CP2102 boards wire DTR to servo RESET. Linux asserts DTR on
/// the first open. A fresh USB-serial session restores kernel-default
/// HUPCL (`cfmakeraw` does not clear it). After `TIOCEXCL`, a child
/// `stty -F` on the node or `/proc/<pid>/fd/N` is EBUSY, and
/// `/proc/self/fd/N` misses an `O_CLOEXEC` tty. Clear HUPCL on the live
/// exclusive fd via termios so probe close / `crash_if` / Drop does not
/// lower DTR. A shared-open + `stty` window races ModemManager. U2D2
/// does not wire DTR to RESET. Do not toggle DTR/RTS from userspace.
fn clear_hupcl_on_fd(port: &impl AsRawFd) -> io::Result<()> {
    let fd = port.as_raw_fd();
    let mut termios =
        tcgetattr(fd).map_err(|e| io::Error::other(format!("dxl_hupcl_tcgetattr:{e}")))?;
    termios.control_flags.remove(ControlFlags::HUPCL);
    tcsetattr(fd, SetArg::TCSANOW, &termios)
        .map_err(|e| io::Error::other(format!("dxl_hupcl_tcsetattr:{e}")))?;
    let after =
        tcgetattr(fd).map_err(|e| io::Error::other(format!("dxl_hupcl_tcgetattr_verify:{e}")))?;
    if after.control_flags.contains(ControlFlags::HUPCL) {
        return Err(io::Error::other("dxl_hupcl_still_set"));
    }
    Ok(())
}

/// Cheap TTL/RS485 adapters need DE/RE after host TX. U2D2 does this
/// internally. PTY has no half-duplex. 1.5 ms stays inside the 40 ms live
/// deadline and covers cheap MAX485 DE/RE that need more than 500 µs.
fn half_duplex_turnaround(device: &Path) {
    if is_pty_path(device) {
        return;
    }
    std::thread::sleep(Duration::from_micros(1500));
}

fn open_settle(device: &Path) {
    // Cheap FTDI/CP2102 DTR-RESET plus low VIN can exceed 300 ms. Robotis
    // documents ~100–300 ms; 500 ms covers the first-open reboot window.
    // PTY has no DTR; keep tests fast.
    let ms = if is_pty_path(device) { 100 } else { 500 };
    std::thread::sleep(Duration::from_millis(ms));
}

fn open_xl330_serial(device: &Path, baud: u32) -> io::Result<Box<dyn SerialPort>> {
    open_xl330_serial_with(device, baud, !is_pty_path(device))
}

fn open_xl330_serial_with(
    device: &Path,
    baud: u32,
    exclusive: bool,
) -> io::Result<Box<dyn SerialPort>> {
    // Real UART takes exclusive on the first open, then clears HUPCL on
    // that fd. PTY skips TIOCEXCL (`process::exit` leaves it on pts).
    let take_exclusive = exclusive && !is_pty_path(device);
    let port = serialport::new(device.to_string_lossy(), baud)
        .timeout(Duration::from_millis(150))
        .exclusive(take_exclusive)
        .open_native()
        .map_err(io::Error::other)?;
    if !is_pty_path(device) {
        clear_hupcl_on_fd(&port)?;
    }
    // U2D2/FTDI often drops the first packet if we ping immediately after
    // open. Discover tries each baud/id pair once; a cold miss on the real
    // pair never comes back.
    open_settle(device);
    Ok(Box::new(port))
}

fn prefer_servo_id(ids: &[u8], found: Option<u8>) -> Vec<u8> {
    let mut out = Vec::new();
    if let Some(id) = found {
        if id != BROADCAST_ID {
            out.push(id);
        }
    }
    for id in ids {
        if !out.contains(id) {
            out.push(*id);
        }
    }
    out
}

/// Broadcast PING. Status ID is the servo's own ID even when Status Return Level is 0.
/// Waits the full window so a second servo on the drop is visible.
fn sniff_servo_ids(device: &Path, baud: u32) -> io::Result<Vec<u8>> {
    if !device.exists() {
        return Ok(Vec::new());
    }
    // Real UART takes exclusive so ModemManager cannot AT-probe during
    // the 500 ms open-settle. Drop before a second open (Secondary ID
    // check). PTY stays shared (`process::exit` can leave TIOCEXCL).
    let mut port = match open_xl330_serial_with(device, baud, !is_pty_path(device)) {
        Ok(p) => p,
        Err(_) => return Ok(Vec::new()),
    };
    let frame = encode_ping(BROADCAST_ID);
    if port.clear(serialport::ClearBuffer::Input).is_err()
        || port.write_all(&frame).is_err()
        || port.flush().is_err()
    {
        return Ok(Vec::new());
    }
    half_duplex_turnaround(device);
    let mut acc = Vec::new();
    let mut tmp = [0u8; 64];
    let end = std::time::Instant::now() + Duration::from_millis(150);
    while std::time::Instant::now() < end {
        match port.read(&mut tmp) {
            Ok(0) => {}
            Ok(n) => acc.extend_from_slice(&tmp[..n]),
            Err(e) if e.kind() == io::ErrorKind::TimedOut => break,
            Err(_) => return Ok(Vec::new()),
        }
    }
    let ids = unique_status_ids(&acc);
    drop(port);
    if ids.len() > 1 {
        if let Some(primary) = primary_if_secondary_pair(device, baud, &ids) {
            return Ok(vec![primary]);
        }
        return Err(io::Error::other(format!(
            "dxl_multiple_servos_on_bus:{}",
            ids.iter()
                .map(ToString::to_string)
                .collect::<Vec<_>>()
                .join(",")
        )));
    }
    Ok(ids)
}

/// Wizard Secondary ID makes one servo answer two IDs. Both addresses then
/// read the same ID register. Two distinct servos read two distinct IDs.
fn primary_if_secondary_pair(device: &Path, baud: u32, ids: &[u8]) -> Option<u8> {
    if ids.len() != 2 {
        return None;
    }
    let mut port = open_xl330_serial_with(device, baud, !is_pty_path(device)).ok()?;
    for &id in ids {
        poke_srl_all(&mut *port, device, id);
    }
    let via_a = read_id_register(&mut *port, device, ids[0])?;
    let via_b = read_id_register(&mut *port, device, ids[1])?;
    if via_a == via_b && via_a != BROADCAST_ID {
        Some(via_a)
    } else {
        None
    }
}

fn poke_srl_all(port: &mut dyn SerialPort, device: &Path, id: u8) {
    let frame = encode_write(id, ADDR_STATUS_RETURN_LEVEL, &[STATUS_RETURN_ALL]);
    let _ = port.clear(serialport::ClearBuffer::Input);
    let _ = port.write_all(&frame);
    let _ = port.flush();
    half_duplex_turnaround(device);
    let saved = port.timeout();
    let _ = port.set_timeout(Duration::from_millis(25));
    let mut tmp = [0u8; 64];
    let end = std::time::Instant::now() + Duration::from_millis(25);
    while std::time::Instant::now() < end {
        if port.read(&mut tmp).is_err() {
            break;
        }
    }
    let _ = port.set_timeout(saved);
}

fn read_id_register(port: &mut dyn SerialPort, device: &Path, id: u8) -> Option<u8> {
    let frame = encode_read(id, ADDR_ID, 1);
    let _ = port.clear(serialport::ClearBuffer::Input);
    port.write_all(&frame).ok()?;
    port.flush().ok()?;
    half_duplex_turnaround(device);
    let saved = port.timeout();
    let _ = port.set_timeout(Duration::from_millis(40));
    let mut acc = Vec::new();
    let mut tmp = [0u8; 64];
    let end = std::time::Instant::now() + Duration::from_millis(40);
    while std::time::Instant::now() < end {
        match port.read(&mut tmp) {
            Ok(0) => {}
            Ok(n) => {
                acc.extend_from_slice(&tmp[..n]);
                if let Ok(st) = decode_status_scan(&acc) {
                    let _ = port.set_timeout(saved);
                    return st.params.first().copied();
                }
            }
            Err(_) => break,
        }
    }
    let _ = port.set_timeout(saved);
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use serialport::TTYPort;

    #[test]
    fn hupcl_clear_on_exclusive_fd_sticks() {
        let (_master, slave) = TTYPort::pair().expect("pty pair");
        let mut slave = slave;
        slave.set_exclusive(true).expect("take exclusive");
        let fd = slave.as_raw_fd();
        let mut termios = tcgetattr(fd).expect("tcgetattr before");
        termios.control_flags.insert(ControlFlags::HUPCL);
        tcsetattr(fd, SetArg::TCSANOW, &termios).expect("force HUPCL");
        assert!(
            tcgetattr(fd)
                .expect("read forced HUPCL")
                .control_flags
                .contains(ControlFlags::HUPCL),
            "test must start with HUPCL set"
        );
        clear_hupcl_on_fd(&slave).expect("clear HUPCL on exclusive fd");
        assert!(
            !tcgetattr(fd)
                .expect("tcgetattr after")
                .control_flags
                .contains(ControlFlags::HUPCL),
            "HUPCL must stay clear on the exclusive fd"
        );
    }
}
