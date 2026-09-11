//! One XL330 HardwareDriverPort. Translates authorized actions to Protocol 2.0.
//! Contains no Reality OS policy.

use std::fs::{File, OpenOptions};
use std::io::{self, Read, Write};
use std::os::unix::fs::{OpenOptionsExt, PermissionsExt};
use std::os::unix::io::AsRawFd;
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use nix::sys::termios::{tcgetattr, tcsetattr, ControlFlags, SetArg};

use fs2::FileExt;
use realityos_plant::{
    ActionParams, HardwareDriverPort, HardwareIdentity, PlantError, PlantRealized, PlantResult,
    SensorPacket,
};
use serialport::SerialPort;

use crate::config::{
    cage_allows_inbound_nudge_after_hold_still, candidate_bauds, candidate_servo_ids,
    chosen_nudge_survives_slack, discover_baud_attempts, pick_inbound_nudge_action, MetalConfig,
    BUS_DIR, CAGE_EVIDENCE_FILE, CANDIDATE_BAUDS, GOAL_FILE, HOLD_STILL_HEADROOM_TICKS, LOCK_FILE,
    MOVING_FILE, NUDGE_PRESENT_SLACK_TICKS, PRESENT_FILE, PWM_EVIDENCE_FILE, VIN_FILE,
};
use crate::egress::EgressLog;
use crate::identity::{
    adapter_identity_aliases, adapter_still_same_as_latched, device_node_identity, is_pty_path,
    rematch_discover_device, usb_identity_for_tty, MeasuredIdentity,
};
use crate::protocol::{
    decode_status_scan, encode_ping, encode_read, encode_reboot, encode_write, find_header,
    instruction_ok, is_xl330_model, le_i16, le_i32, le_u16, le_u32, pwm_limit_percent,
    unique_status_ids, ProtocolError, ADDR_BUS_WATCHDOG, ADDR_CURRENT_LIMIT, ADDR_DRIVE_MODE,
    ADDR_FEEDFORWARD_1ST, ADDR_FEEDFORWARD_2ND, ADDR_FIRMWARE_VERSION, ADDR_GOAL_POSITION,
    ADDR_GOAL_PWM, ADDR_HARDWARE_ERROR, ADDR_HOMING_OFFSET, ADDR_ID, ADDR_MAX_POSITION_LIMIT,
    ADDR_MAX_VOLTAGE_LIMIT, ADDR_MIN_POSITION_LIMIT, ADDR_MIN_VOLTAGE_LIMIT, ADDR_MODEL_NUMBER,
    ADDR_MOVING, ADDR_MOVING_THRESHOLD, ADDR_OPERATING_MODE, ADDR_POSITION_D_GAIN,
    ADDR_POSITION_I_GAIN, ADDR_POSITION_P_GAIN, ADDR_PRESENT_POSITION, ADDR_PRESENT_TEMPERATURE,
    ADDR_PRESENT_VOLTAGE, ADDR_PROFILE_ACCEL, ADDR_PROFILE_VELOCITY, ADDR_PROTOCOL_TYPE,
    ADDR_PWM_LIMIT, ADDR_PWM_SLOPE, ADDR_REALTIME_TICK, ADDR_SECONDARY_ID,
    ADDR_STARTUP_CONFIGURATION, ADDR_STATUS_RETURN_LEVEL, ADDR_TEMPERATURE_LIMIT,
    ADDR_TORQUE_ENABLE, ADDR_VELOCITY_I_GAIN, ADDR_VELOCITY_LIMIT, ADDR_VELOCITY_P_GAIN,
    BROADCAST_ID, DRIVE_MODE_VELOCITY_BASED, FACTORY_MOVING_THRESHOLD, FACTORY_POSITION_P_GAIN,
    FACTORY_PWM_SLOPE, FACTORY_STARTUP_CONFIGURATION, FACTORY_VELOCITY_I_GAIN,
    FACTORY_VELOCITY_P_GAIN, MIN_POSITION_P_GAIN, MIN_PWM_SLOPE, MIN_VELOCITY_I_GAIN,
    MIN_VELOCITY_P_GAIN, OPERATING_MODE_POSITION, PROTOCOL_TYPE_2, SECONDARY_ID_DISABLED,
    STATUS_ALERT, STATUS_RETURN_ALL, XL330_POSITION_MODE_MAX, XL330_POSITION_MODE_MIN,
    XL330_PWM_LIMIT_MAX,
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
    /// STATUS_ALERT on a later packet means Hardware Error Status may
    /// have changed since setup. Re-read register 70 before publishing
    /// the `hw_error` sample; do not keep the setup-time latch forever.
    hw_error_needs_refresh: bool,
    min_position: i32,
    max_position: i32,
    startup_present: i32,
    experiment_min: i32,
    experiment_max: i32,
    eeprom_min_saved: Option<i32>,
    eeprom_max_saved: Option<i32>,
    pwm_limit_requested: u16,
    velocity_limit: u32,
    drive_mode: u8,
    position_p_gain: u16,
    position_i_gain: u16,
    position_d_gain: u16,
    velocity_p_gain: u16,
    velocity_i_gain: u16,
    pwm_limit: u16,
    goal_pwm: i16,
    homing_offset: i32,
    bus_watchdog: u8,
    moving_threshold: u32,
    protocol_type: u8,
    feedforward_1st: u16,
    feedforward_2nd: u16,
    pwm_slope: u8,
    startup_configuration: u8,
    min_voltage: u16,
    max_voltage: u16,
    /// Live VIN outside Wizard min/max (or unreadable 0) is a supply
    /// fault / cutoff, not a healthy sample. Latch so write_action
    /// cannot emit a certified goal after the campaign already saw
    /// bus/vin drop.
    vin_fault: bool,
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
        let mut driver = Self::prepare_locked(cfg, root)?;
        driver.connect_serial()?;
        driver.finish_open(apply_limits)?;
        Ok(driver)
    }

    fn prepare_locked(cfg: MetalConfig, root: impl AsRef<Path>) -> io::Result<Self> {
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
        Ok(Self {
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
            hw_error_needs_refresh: false,
            min_position: XL330_POSITION_MODE_MIN,
            max_position: XL330_POSITION_MODE_MAX,
            startup_present: 0,
            experiment_min: XL330_POSITION_MODE_MIN,
            experiment_max: XL330_POSITION_MODE_MAX,
            eeprom_min_saved: None,
            eeprom_max_saved: None,
            pwm_limit_requested: 0,
            velocity_limit: 0,
            drive_mode: 0,
            position_p_gain: 0,
            position_i_gain: 0,
            position_d_gain: 0,
            velocity_p_gain: 0,
            velocity_i_gain: 0,
            pwm_limit: 0,
            goal_pwm: 0,
            homing_offset: 0,
            bus_watchdog: 0,
            moving_threshold: 0,
            protocol_type: 0,
            feedforward_1st: 0,
            feedforward_2nd: 0,
            pwm_slope: 0,
            startup_configuration: 0,
            min_voltage: 0,
            max_voltage: 0,
            vin_fault: false,
            latched_usb_serial: None,
            latched_usb_fallback: None,
            latched_node: None,
            latched_pty: false,
        })
    }

    fn finish_open(&mut self, apply_limits: bool) -> io::Result<()> {
        self.refresh_identity();
        if self.connected && apply_limits {
            self.apply_bench_limits()
                .map_err(|e| io::Error::other(e.to_string()))?;
            self.enter_live_io();
        } else if self.connected {
            // Startup Configuration can enable torque after a DTR reboot.
            // Identify-only must not leave the horn tracking a stale goal.
            self.quiesce_found_torque();
        }
        Ok(())
    }

    /// Probe-only: try configured baud/id first, then common XL330 bus settings.
    /// Production `serve` keeps using [`Self::open`] with the bound config.
    ///
    /// Cheap FTDI/CP2102 boards wire DTR to servo RESET. Each new USB-serial
    /// open asserts DTR. Discover used to open+close for every sniff and
    /// again for every identify, so a leftover 1 Mbps / 2/3/4 Mbps scan
    /// DTR-RESET the horn on every rate. Hold one exclusive fd and retune
    /// baud in place; reopen only when udev rematches a new node.
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
        let mut held: Option<HeldDiscover> = None;
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
                held = None;
                std::thread::sleep(Duration::from_millis(100));
                continue;
            }
            match held.as_mut() {
                Some(session) => {
                    if let Err(e) = session.ensure(&cfg.device, baud) {
                        last_err = Some(e);
                        held = None;
                        continue;
                    }
                }
                None => match HeldDiscover::open(cfg.device.clone(), baud) {
                    Ok(session) => held = Some(session),
                    Err(e) => {
                        last_err = Some(e);
                        continue;
                    }
                },
            }
            let Some(session) = held.as_mut() else {
                continue;
            };
            // Wizard may leave a non-1/2 ID. Broadcast PING still answers at
            // SRL=0 and the status carries the servo's own ID.
            let sniffed = sniff_on_port(&mut *session.port, &cfg.device)?;
            let try_ids = prefer_servo_id(&ids, sniffed.first().copied());
            let mut session = held.take().expect("discover session");
            for id in try_ids {
                let mut attempt = cfg.clone();
                attempt.baud = baud;
                attempt.servo_id = id;
                let mut driver = match Self::prepare_locked(attempt.clone(), root.as_ref()) {
                    Ok(driver) => driver,
                    Err(e) if e.kind() == io::ErrorKind::WouldBlock => return Err(e),
                    Err(e) => {
                        last_err = Some(e);
                        break;
                    }
                };
                match driver.adopt_held_serial(session.port) {
                    Ok(()) => {
                        driver.finish_open(false)?;
                        return Ok((driver, attempt));
                    }
                    Err((e, _port)) if e.kind() == io::ErrorKind::WouldBlock => return Err(e),
                    Err((e, port)) => {
                        last_err = Some(e);
                        session.port = port;
                        std::thread::sleep(Duration::from_millis(100));
                    }
                }
            }
            held = Some(session);
        }
        // Factory 57 600 again after the scan has opened the tty. Do not
        // retry a leftover 2/3/4 Mbps hint here: that open is after the
        // factory rates and can wedge CH340 so a late identify never
        // happens. Retune the held fd; do not close+open (DTR-RESET).
        let mut configured = cfg.clone();
        configured.baud = CANDIDATE_BAUDS[0];
        if let Some(mut session) = held.take() {
            if let Err(e) = session.ensure(&cfg.device, CANDIDATE_BAUDS[0]) {
                last_err = Some(e);
            } else {
                match Self::prepare_locked(configured.clone(), root.as_ref()) {
                    Ok(mut driver) => match driver.adopt_held_serial(session.port) {
                        Ok(()) => {
                            driver.finish_open(false)?;
                            return Ok((driver, configured));
                        }
                        Err((e, _)) if e.kind() == io::ErrorKind::WouldBlock => return Err(e),
                        Err((e, _)) => return Err(last_err.unwrap_or(e)),
                    },
                    Err(e) if e.kind() == io::ErrorKind::WouldBlock => return Err(e),
                    Err(e) => last_err = Some(e),
                }
            }
        }
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

    pub fn applied_goal_pwm(&self) -> i16 {
        self.goal_pwm
    }

    pub fn pwm_limit_requested(&self) -> u16 {
        self.pwm_limit_requested
    }

    pub fn experiment_cage(&self) -> (i32, i32) {
        (self.experiment_min, self.experiment_max)
    }

    pub fn startup_present(&self) -> i32 {
        self.startup_present
    }

    pub fn applied_homing_offset(&self) -> i32 {
        self.homing_offset
    }

    pub fn applied_startup_configuration(&self) -> u8 {
        self.startup_configuration
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

    pub fn applied_position_i_gain(&self) -> u16 {
        self.position_i_gain
    }

    pub fn applied_position_d_gain(&self) -> u16 {
        self.position_d_gain
    }

    pub fn applied_feedforward_1st(&self) -> u16 {
        self.feedforward_1st
    }

    pub fn applied_feedforward_2nd(&self) -> u16 {
        self.feedforward_2nd
    }

    pub fn applied_pwm_slope(&self) -> u8 {
        self.pwm_slope
    }

    pub fn applied_voltage_limits(&self) -> (u16, u16) {
        (self.min_voltage, self.max_voltage)
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

    fn ensure_tty_mode_0600(&self) {
        // A no-op chmod still emits udev change on typical Ubuntu — the
        // same class as campaign chown resetting FTDI latency_timer to
        // 16 ms before the first live hold. Skip when already 0600.
        let mode = std::fs::metadata(&self.cfg.device)
            .map(|m| m.permissions().mode() & 0o777)
            .unwrap_or(0);
        if mode != 0o600 {
            let _ =
                std::fs::set_permissions(&self.cfg.device, std::fs::Permissions::from_mode(0o600));
        }
    }

    fn adapter_recycled_after_open(&self) -> Option<io::Error> {
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
            Some(io::Error::other(format!(
                "metal_adapter_recycled_after_open:expected={} actual={} device={}",
                self.cfg.expected_serial,
                crate::identity::adapter_serial_for_tty(&self.cfg.device, self.cfg.servo_id),
                self.cfg.device.display()
            )))
        } else {
            None
        }
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
        self.ensure_tty_mode_0600();
        // PTY stand-in: TIOCEXCL survives process::exit (crash_if) and the
        // next serve gets EBUSY. Sidecar flock still serializes. Real tty
        // keeps exclusive (TIOCEXCL+flock).
        let port = open_xl330_serial(&self.cfg.device, self.cfg.baud)?;
        self.port = Some(port);
        if let Some(e) = self.adapter_recycled_after_open() {
            self.port = None;
            self.connected = false;
            return Err(e);
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

    /// Identify on a discover fd that is already exclusive. On failure the
    /// caller keeps that fd so the next baud is a termios retune, not a
    /// close+open DTR-RESET.
    fn adopt_held_serial(
        &mut self,
        port: Box<dyn SerialPort>,
    ) -> Result<(), (io::Error, Box<dyn SerialPort>)> {
        if !self.cfg.device.exists() {
            self.connected = false;
            return Err((
                io::Error::new(
                    io::ErrorKind::NotFound,
                    format!("metal_device_missing:{}", self.cfg.device.display()),
                ),
                port,
            ));
        }
        self.latch_open_adapter_identity();
        self.ensure_tty_mode_0600();
        self.port = Some(port);
        if let Some(e) = self.adapter_recycled_after_open() {
            self.connected = false;
            let port = self.port.take().expect("adopted serial");
            return Err((e, port));
        }
        match self.ping_and_identify() {
            Ok(()) => {
                self.connected = true;
                Ok(())
            }
            Err(e) => {
                self.connected = false;
                let port = self.port.take().expect("adopted serial");
                Err((e, port))
            }
        }
    }

    fn ping_and_identify(&mut self) -> io::Result<()> {
        let _ = self.xfer(&encode_ping(self.cfg.servo_id), true)?;
        // Wizard can set Status Return Level to 0 (PING only) or 1
        // (PING+READ). WRITE then has no status. Identify READs still
        // succeed at SRL=1, and the first setup write_reg times out or
        // decodes the half-duplex echo as dxl_truncated. Poke 2 and
        // read it back — the poke itself may have no status.
        self.force_status_return_all()?;
        // Startup Configuration can already be tracking a stale goal.
        // Identify READs used to run while torque was on.
        self.force_torque_off_no_egress()?;
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
        let mut last: Option<u8> = None;
        for _ in 0..3 {
            self.poke_status_return_all()?;
            last = self.peek_status_return_level()?;
            if last == Some(STATUS_RETURN_ALL) {
                return Ok(());
            }
        }
        Err(io::Error::other(format!(
            "dxl_status_return_level_unverified:{}",
            last.map(|v| v.to_string())
                .unwrap_or_else(|| "unread".into())
        )))
    }

    fn poke_status_return_all(&mut self) -> io::Result<()> {
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
        // Factory SRL=2 replies; Wizard SRL=0/1 does not. Consume an
        // optional status so a late USB packet is not decoded as the
        // readback or the model READ. 25 ms > default FTDI latency (16 ms).
        port.set_timeout(Duration::from_millis(25))
            .map_err(io::Error::other)?;
        let mut acc = Vec::new();
        let mut tmp = [0u8; 64];
        let end = Instant::now() + Duration::from_millis(25);
        while Instant::now() < end {
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

    /// Identify has not set `connected` yet, so this must not use `read_reg`.
    /// SRL=0 still has no READ status; treat that as unread and retry the poke.
    fn peek_status_return_level(&mut self) -> io::Result<Option<u8>> {
        let frame = encode_read(self.cfg.servo_id, ADDR_STATUS_RETURN_LEVEL, 1);
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
        port.set_timeout(Duration::from_millis(25))
            .map_err(io::Error::other)?;
        let mut acc = Vec::new();
        let mut tmp = [0u8; 64];
        let end = Instant::now() + Duration::from_millis(25);
        let mut got = None;
        while Instant::now() < end {
            match port.read(&mut tmp) {
                Ok(0) => {}
                Ok(n) => {
                    acc.extend_from_slice(&tmp[..n]);
                    if let Ok(st) = decode_status_scan(&acc) {
                        got = st.params.first().copied();
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
        Ok(got)
    }

    /// Directed torque-off before identify READs. Not command egress.
    /// `write_reg` needs `connected`, which is only set after identify.
    fn force_torque_off_no_egress(&mut self) -> io::Result<()> {
        let frame = encode_write(self.cfg.servo_id, ADDR_TORQUE_ENABLE, &[0]);
        let port = self
            .port
            .as_mut()
            .ok_or_else(|| io::Error::other("metal_serial_closed"))?;
        let saved = port.timeout();
        let _ = port.clear(serialport::ClearBuffer::Input);
        port.write_all(&frame).map_err(io::Error::other)?;
        port.flush().map_err(io::Error::other)?;
        half_duplex_turnaround(&self.cfg.device);
        port.set_timeout(Duration::from_millis(25))
            .map_err(io::Error::other)?;
        let mut tmp = [0u8; 64];
        let end = Instant::now() + Duration::from_millis(25);
        while Instant::now() < end {
            if port.read(&mut tmp).is_err() {
                break;
            }
        }
        let _ = port.clear(serialport::ClearBuffer::Input);
        port.set_timeout(saved).map_err(io::Error::other)?;
        self.torque_enabled = false;
        Ok(())
    }

    fn apply_bench_limits(&mut self) -> PlantResult<()> {
        self.cfg
            .validate_xl330_limits()
            .map_err(PlantError::refused)?;
        // Current limit / operating mode are EEPROM; only write with torque off, and only if needed.
        self.write_reg(ADDR_TORQUE_ENABLE, &[0], "setup_torque_off", None, false)?;
        self.torque_enabled = false;
        // Protocol 2.0 sets STATUS_ALERT on every packet while Hardware Error
        // Status is latched (Wizard overload, VIN blip). That is not a NAK.
        // Reboot once *before* RAM profile writes — reboot clears RAM.
        if let Ok(b) = self.read_reg(ADDR_HARDWARE_ERROR, 1) {
            self.last_hw_error = b.first().copied().unwrap_or(0);
            self.hw_error_needs_refresh = false;
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
            self.hw_error_needs_refresh = false;
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
        let startup = self
            .read_reg(ADDR_STARTUP_CONFIGURATION, 1)
            .ok()
            .and_then(|b| b.first().copied());
        self.startup_configuration = startup.unwrap_or(0xFF);
        // Bit 0 torque-on at boot. A DTR-RESET then tracks Goal (RAM 0)
        // during the next open settle before any certified command.
        // Factory 0. Probe must not write this EEPROM.
        // Do not treat a failed read as 0: that skipped the write and
        // the next crash_if DTR-RESET yanked.
        if startup != Some(FACTORY_STARTUP_CONFIGURATION) {
            self.write_reg(
                ADDR_STARTUP_CONFIGURATION,
                &[FACTORY_STARTUP_CONFIGURATION],
                "setup_startup_configuration_off",
                None,
                false,
            )?;
            eeprom_changed = true;
        }
        let startup_got = self
            .read_reg(ADDR_STARTUP_CONFIGURATION, 1)
            .ok()
            .and_then(|b| b.first().copied());
        if startup_got != Some(FACTORY_STARTUP_CONFIGURATION) {
            return Err(PlantError::refused(format!(
                "dxl_startup_configuration_unverified:{}",
                startup_got
                    .map(|v| v.to_string())
                    .unwrap_or_else(|| "unread".into())
            )));
        }
        self.startup_configuration = FACTORY_STARTUP_CONFIGURATION;
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
        // A write_reg ACK that does not store used to leave applied
        // drive/mode lying. Wizard PWM (16) then treats Goal PWM as the
        // command: setup matching that register to the PWM cap would
        // spin at torque-on. Time-based drive treats profile 20/10 as
        // milliseconds, so the 32-tick step is not the rpm cage.
        let mode_got = self
            .read_reg(ADDR_OPERATING_MODE, 1)
            .ok()
            .and_then(|b| b.first().copied());
        if mode_got != Some(OPERATING_MODE_POSITION) {
            return Err(PlantError::refused(format!(
                "dxl_operating_mode_unverified:{mode_got:?}"
            )));
        }
        let drive_got = self
            .read_reg(ADDR_DRIVE_MODE, 1)
            .ok()
            .and_then(|b| b.first().copied());
        if drive_got != Some(DRIVE_MODE_VELOCITY_BASED) {
            return Err(PlantError::refused(format!(
                "dxl_drive_mode_unverified:{drive_got:?}"
            )));
        }
        self.drive_mode = DRIVE_MODE_VELOCITY_BASED;
        // Wizard 20/21/22 at the next DTR-RESET leaves RC boot (auto
        // torque-on, no Protocol 2.0). A write_reg ACK that does not
        // store used to leave applied_protocol_type lying as 2.
        // Secondary ID 255 is this experiment's one-actuator invariant.
        let proto_got = self
            .read_reg(ADDR_PROTOCOL_TYPE, 1)
            .ok()
            .and_then(|b| b.first().copied());
        if proto_got != Some(PROTOCOL_TYPE_2) {
            return Err(PlantError::refused(format!(
                "dxl_protocol_type_unverified:{proto_got:?}"
            )));
        }
        self.protocol_type = PROTOCOL_TYPE_2;
        let secondary_got = self
            .read_reg(ADDR_SECONDARY_ID, 1)
            .ok()
            .and_then(|b| b.first().copied());
        if secondary_got != Some(SECONDARY_ID_DISABLED) {
            return Err(PlantError::refused(format!(
                "dxl_secondary_id_unverified:{secondary_got:?}"
            )));
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
        }
        let vel_got = self
            .read_reg(ADDR_VELOCITY_LIMIT, 4)
            .ok()
            .and_then(|b| le_u32(&b));
        let Some(vel_ok) = vel_got.filter(|v| *v >= want_vel) else {
            return Err(PlantError::refused(format!(
                "dxl_velocity_limit_unverified:{vel_got:?}"
            )));
        };
        self.velocity_limit = vel_ok;
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
        let prof_v = self
            .read_reg(ADDR_PROFILE_VELOCITY, 4)
            .ok()
            .and_then(|b| le_u32(&b));
        let prof_a = self
            .read_reg(ADDR_PROFILE_ACCEL, 4)
            .ok()
            .and_then(|b| le_u32(&b));
        if prof_v != Some(self.cfg.max_profile_velocity)
            || prof_a != Some(self.cfg.max_profile_acceleration)
        {
            return Err(PlantError::refused(format!(
                "dxl_profile_unverified:vel={prof_v:?}:accel={prof_a:?}"
            )));
        }
        let got_mt = self
            .read_reg(ADDR_MOVING_THRESHOLD, 4)
            .ok()
            .and_then(|b| le_u32(&b));
        self.moving_threshold = got_mt.unwrap_or(u32::MAX);
        // Moving=1 only while |velocity| > this. A Wizard value ≥ profile
        // velocity keeps Moving=0 for the whole 32-tick nudge. Campaign
        // settle waits for present≈goal, but lower the threshold so Moving
        // is still a usable in-motion flag. A failed read used to look like
        // safe 0 and skip the write; an ACK that does not store used to
        // leave applied_moving_threshold lying as factory 10.
        if got_mt
            .map(|v| v > self.cfg.max_profile_velocity)
            .unwrap_or(true)
        {
            self.write_reg(
                ADDR_MOVING_THRESHOLD,
                &FACTORY_MOVING_THRESHOLD.to_le_bytes(),
                "setup_moving_threshold",
                None,
                false,
            )?;
        }
        let mt_got = self
            .read_reg(ADDR_MOVING_THRESHOLD, 4)
            .ok()
            .and_then(|b| le_u32(&b));
        let Some(mt_ok) = mt_got.filter(|v| *v <= self.cfg.max_profile_velocity) else {
            return Err(PlantError::refused(format!(
                "dxl_moving_threshold_unverified:{mt_got:?}"
            )));
        };
        self.moving_threshold = mt_ok;
        let got_p = self
            .read_reg(ADDR_POSITION_P_GAIN, 2)
            .ok()
            .and_then(|b| le_u16(&b))
            .unwrap_or(0);
        self.position_p_gain = got_p;
        // Factory 400. Below 80 the 32-tick step never tracks. Above
        // factory a Wizard PID tune overshoots past the 48-tick cage
        // (same class as I/D and feedforward).
        if got_p < MIN_POSITION_P_GAIN || got_p > FACTORY_POSITION_P_GAIN {
            self.write_reg(
                ADDR_POSITION_P_GAIN,
                &FACTORY_POSITION_P_GAIN.to_le_bytes(),
                "setup_position_p_gain",
                None,
                false,
            )?;
        }
        let p_got = self
            .read_reg(ADDR_POSITION_P_GAIN, 2)
            .ok()
            .and_then(|b| le_u16(&b));
        let Some(p_ok) =
            p_got.filter(|p| (MIN_POSITION_P_GAIN..=FACTORY_POSITION_P_GAIN).contains(p))
        else {
            return Err(PlantError::refused(format!(
                "dxl_position_p_unverified:{p_got:?}"
            )));
        };
        self.position_p_gain = p_ok;
        let got_i = self
            .read_reg(ADDR_POSITION_I_GAIN, 2)
            .ok()
            .and_then(|b| le_u16(&b));
        let got_d = self
            .read_reg(ADDR_POSITION_D_GAIN, 2)
            .ok()
            .and_then(|b| le_u16(&b));
        self.position_i_gain = got_i.unwrap_or(u16::MAX);
        self.position_d_gain = got_d.unwrap_or(u16::MAX);
        // Factory 0. Wizard position I/D overshoots the certified 32-tick
        // step past the 48-tick session cage. A failed read used to look
        // like factory 0 and skip the write.
        if got_i != Some(0) || got_d != Some(0) {
            self.write_reg(
                ADDR_POSITION_I_GAIN,
                &0u16.to_le_bytes(),
                "setup_position_i_gain_zero",
                None,
                false,
            )?;
            self.write_reg(
                ADDR_POSITION_D_GAIN,
                &0u16.to_le_bytes(),
                "setup_position_d_gain_zero",
                None,
                false,
            )?;
        }
        let i_got = self
            .read_reg(ADDR_POSITION_I_GAIN, 2)
            .ok()
            .and_then(|b| le_u16(&b));
        let d_got = self
            .read_reg(ADDR_POSITION_D_GAIN, 2)
            .ok()
            .and_then(|b| le_u16(&b));
        if i_got != Some(0) || d_got != Some(0) {
            return Err(PlantError::refused(format!(
                "dxl_position_id_unverified:i={i_got:?}:d={d_got:?}"
            )));
        }
        self.position_i_gain = 0;
        self.position_d_gain = 0;
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
        }
        let vp_got = self
            .read_reg(ADDR_VELOCITY_P_GAIN, 2)
            .ok()
            .and_then(|b| le_u16(&b));
        let Some(vp_ok) = vp_got.filter(|v| *v >= MIN_VELOCITY_P_GAIN) else {
            return Err(PlantError::refused(format!(
                "dxl_velocity_p_unverified:{vp_got:?}"
            )));
        };
        self.velocity_p_gain = vp_ok;
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
        }
        let vi_got = self
            .read_reg(ADDR_VELOCITY_I_GAIN, 2)
            .ok()
            .and_then(|b| le_u16(&b));
        let Some(vi_ok) = vi_got.filter(|v| *v >= MIN_VELOCITY_I_GAIN) else {
            return Err(PlantError::refused(format!(
                "dxl_velocity_i_unverified:{vi_got:?}"
            )));
        };
        self.velocity_i_gain = vi_ok;
        self.apply_pwm_output_cap()?;
        let slope = self
            .read_reg(ADDR_PWM_SLOPE, 1)
            .ok()
            .and_then(|b| b.first().copied())
            .unwrap_or(0);
        self.pwm_slope = slope;
        // Factory 140. Wizard 0 is outside the e-Manual 1..=255 range.
        // Wizard 1..=19 is legal but ramps too slowly for the 32-tick
        // nudge to leave the hold-still band before settle timeout.
        if slope < MIN_PWM_SLOPE {
            self.write_reg(
                ADDR_PWM_SLOPE,
                &[FACTORY_PWM_SLOPE],
                "setup_pwm_slope_factory",
                None,
                false,
            )?;
        }
        let slope_got = self
            .read_reg(ADDR_PWM_SLOPE, 1)
            .ok()
            .and_then(|b| b.first().copied());
        let Some(slope_ok) = slope_got.filter(|s| *s >= MIN_PWM_SLOPE) else {
            return Err(PlantError::refused(format!(
                "dxl_pwm_slope_unverified:{}",
                slope_got
                    .map(|v| v.to_string())
                    .unwrap_or_else(|| "unread".into())
            )));
        };
        self.pwm_slope = slope_ok;
        let ff2 = self
            .read_reg(ADDR_FEEDFORWARD_2ND, 2)
            .ok()
            .and_then(|b| le_u16(&b));
        let ff1 = self
            .read_reg(ADDR_FEEDFORWARD_1ST, 2)
            .ok()
            .and_then(|b| le_u16(&b));
        self.feedforward_2nd = ff2.unwrap_or(u16::MAX);
        self.feedforward_1st = ff1.unwrap_or(u16::MAX);
        // Factory 0. Wizard feedforward makes the certified 32-tick step
        // overshoot. A failed read used to look like factory 0 and skip
        // the write.
        if ff1 != Some(0) || ff2 != Some(0) {
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
        }
        let ff2_got = self
            .read_reg(ADDR_FEEDFORWARD_2ND, 2)
            .ok()
            .and_then(|b| le_u16(&b));
        let ff1_got = self
            .read_reg(ADDR_FEEDFORWARD_1ST, 2)
            .ok()
            .and_then(|b| le_u16(&b));
        if ff1_got != Some(0) || ff2_got != Some(0) {
            return Err(PlantError::refused(format!(
                "dxl_feedforward_unverified:ff1={ff1_got:?}:ff2={ff2_got:?}"
            )));
        }
        self.feedforward_2nd = 0;
        self.feedforward_1st = 0;
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
        self.min_voltage = min_v;
        self.max_voltage = max_v;
        self.persist_vin(vin);
        if vin < min_v || vin > max_v {
            return Err(PlantError::refused(format!(
                "dxl_vin_outside_wizard_limits:vin_0.1v={vin}:min={min_v}:max={max_v}"
            )));
        }
        let temp_limit = self
            .read_reg(ADDR_TEMPERATURE_LIMIT, 1)
            .ok()
            .and_then(|b| b.first().copied())
            .ok_or_else(|| PlantError::refused("dxl_temperature_limit_unreadable_before_torque"))?;
        let present_temp = self
            .read_reg(ADDR_PRESENT_TEMPERATURE, 1)
            .ok()
            .and_then(|b| b.first().copied())
            .ok_or_else(|| {
                PlantError::refused("dxl_present_temperature_unreadable_before_torque")
            })?;
        if temp_limit == 0 {
            return Err(PlantError::refused("dxl_temperature_limit_zero"));
        }
        if present_temp >= temp_limit {
            return Err(PlantError::refused(format!(
                "dxl_present_temperature_at_or_above_limit:present={present_temp}:limit={temp_limit}"
            )));
        }
        // Wizard Bus Watchdog (20 ms units). Non-zero trips after a quiet
        // gap and latches 0xFF; Goal Position then NAKs with data-range.
        let wd = self
            .read_reg(ADDR_BUS_WATCHDOG, 1)
            .ok()
            .and_then(|b| b.first().copied());
        self.bus_watchdog = wd.unwrap_or(0xFF);
        if wd != Some(0) {
            self.write_reg(
                ADDR_BUS_WATCHDOG,
                &[0],
                "setup_bus_watchdog_off",
                None,
                false,
            )?;
        }
        let wd_got = self
            .read_reg(ADDR_BUS_WATCHDOG, 1)
            .ok()
            .and_then(|b| b.first().copied());
        if wd_got != Some(0) {
            return Err(PlantError::refused(format!(
                "dxl_bus_watchdog_unverified:{}",
                wd_got
                    .map(|v| v.to_string())
                    .unwrap_or_else(|| "unread".into())
            )));
        }
        self.bus_watchdog = 0;
        // Position Mode uses Goal PWM(100) as the live output limiter.
        // PWM Limit(36) only caps that register. A Wizard leftover 0
        // (or |Goal PWM| below MIN_PWM_LIMIT) leaves the 32-tick nudge
        // stuck after we write PWM Limit 200. Match the measured cap
        // after watchdog is cleared (Goal Values are read-only at 0xFF).
        self.sync_goal_pwm_to_measured_cap()?;
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
        // goal=present would NAK. An in-window leftover still lets the
        // torque-on Present reset throw past the 48-tick cage. Clearing
        // offset with torque off is not motion. Do not treat a failed
        // read as 0: that skipped the write.
        let offset = self
            .read_reg(ADDR_HOMING_OFFSET, 4)
            .ok()
            .and_then(|b| le_i32(&b));
        self.homing_offset = offset.unwrap_or(i32::MAX);
        if offset != Some(0) {
            self.write_reg(
                ADDR_HOMING_OFFSET,
                &0i32.to_le_bytes(),
                "setup_homing_offset_zero",
                None,
                false,
            )?;
        }
        let offset_got = self
            .read_reg(ADDR_HOMING_OFFSET, 4)
            .ok()
            .and_then(|b| le_i32(&b));
        if offset_got != Some(0) {
            return Err(PlantError::refused(format!(
                "dxl_homing_offset_unverified:{offset_got:?}"
            )));
        }
        self.homing_offset = 0;
        if offset != Some(0) {
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
        self.establish_startup_cage(present)?;
        self.last_present = present;
        self.write_and_verify_goal(present, "setup_goal_match_present")?;
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
        self.hw_error_needs_refresh = false;
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
            self.recenter_or_rematch_after_torque_present_reset(after)?;
        }
        Ok(())
    }

    fn wizard_legal_window(&self) -> (i32, i32) {
        let min = self.eeprom_min_saved.unwrap_or(self.min_position);
        let max = self.eeprom_max_saved.unwrap_or(self.max_position);
        (
            XL330_POSITION_MODE_MIN.max(min),
            XL330_POSITION_MODE_MAX.min(max),
        )
    }

    fn torque_off_setup(&mut self, why: &'static str) {
        let _ = self.write_reg(ADDR_TORQUE_ENABLE, &[0], why, None, false);
        self.torque_enabled = false;
    }

    /// Robotis Present reset is a register wrap, not certified excursion.
    /// A cage built around the pre-reset number leaves too little inbound
    /// room: the campaign picker then fails after hold, or `write_action`
    /// abort-latches ONLINE. EEPROM Min/Max are read-only while torque is
    /// on, so re-center with torque off. Do not widen the Wizard window.
    ///
    /// EEPROM writes are slow. Torque-off lets the horn settle; matching
    /// Goal to the pre-off number yanks before any certified command.
    /// Park the new cage on the live Present. A second wrap larger than
    /// the hold-still band still refuses (do not loop).
    fn recenter_or_rematch_after_torque_present_reset(&mut self, after: i32) -> PlantResult<()> {
        let (wizard_min, wizard_max) = self.wizard_legal_window();
        if after < wizard_min || after > wizard_max {
            self.torque_off_setup("setup_torque_off_present_jump");
            return Err(PlantError::refused(format!(
                "dxl_present_outside_wizard_limits_after_torque:present={after}:min={wizard_min}:max={wizard_max}"
            )));
        }
        if !self.in_experiment_cage(after) {
            self.torque_off_setup("setup_torque_off_present_jump");
            return Err(PlantError::refused(format!(
                "dxl_present_outside_experiment_cage_after_torque:present={after}:min={}:max={}",
                self.experiment_min, self.experiment_max
            )));
        }
        // Rematch-only when the preferred +delta still survives slack.
        // Either-sign cage_allows would keep 2000..2096 after a 16-tick
        // wrap (minus still fits) and skip the live-park recenter.
        let old_plus_ok = pick_inbound_nudge_action(
            after,
            self.experiment_min,
            self.experiment_max,
            self.cfg.max_position_delta_ticks,
            self.cfg.tau_max,
        )
        .ok()
        .is_some_and(|action| {
            action > 0.0
                && chosen_nudge_survives_slack(
                    after,
                    self.experiment_min,
                    self.experiment_max,
                    self.cfg.max_position_delta_ticks,
                    action,
                    NUDGE_PRESENT_SLACK_TICKS,
                )
                .is_ok()
        });
        if old_plus_ok {
            if let Err(e) =
                self.write_and_verify_goal(after, "setup_goal_match_present_after_torque")
            {
                self.torque_off_setup("setup_torque_off_goal_unverified");
                return Err(e);
            }
            self.last_present = after;
            return Ok(());
        }
        self.torque_off_setup("setup_torque_off_recenter_cage");
        let park = self.live_park_for_recenter()?;
        self.establish_startup_cage(park)?;
        let park = self.live_park_for_recenter()?;
        if !self.in_experiment_cage(park) {
            return Err(PlantError::refused(format!(
                "dxl_present_outside_experiment_cage_after_recenter:present={park}:min={}:max={}",
                self.experiment_min, self.experiment_max
            )));
        }
        cage_allows_inbound_nudge_after_hold_still(
            park,
            self.experiment_min,
            self.experiment_max,
            self.cfg.max_position_delta_ticks,
            self.cfg.tau_max,
        )
        .map_err(|e| PlantError::refused(format!("metal_experiment_cage_no_inbound_step:{e}")))?;
        if park != self.startup_present {
            self.startup_present = park;
            self.persist_cage_evidence();
        }
        self.last_present = park;
        self.write_and_verify_goal(park, "setup_goal_match_present_after_recenter")?;
        self.write_reg(
            ADDR_TORQUE_ENABLE,
            &[1],
            "setup_torque_on_after_recenter",
            None,
            false,
        )?;
        let still_on = self
            .read_reg(ADDR_TORQUE_ENABLE, 1)
            .ok()
            .and_then(|b| b.first().copied());
        if still_on != Some(1) {
            self.torque_enabled = false;
            return Err(PlantError::refused(format!(
                "dxl_torque_dropped_after_recenter:{}",
                still_on.unwrap_or(0)
            )));
        }
        let again = self
            .read_present_setup("dxl_present_unreadable_after_torque")
            .inspect_err(|_| {
                self.torque_off_setup("setup_torque_off_present_unread_recenter");
            })?;
        if again != park {
            let hunt = again.abs_diff(park);
            let (wizard_min, wizard_max) = self.wizard_legal_window();
            let still_legal = again >= wizard_min
                && again <= wizard_max
                && self.in_experiment_cage(again)
                && hunt <= HOLD_STILL_HEADROOM_TICKS as u32;
            if !still_legal {
                self.torque_off_setup("setup_torque_off_present_jumped_twice");
                return Err(PlantError::refused(format!(
                    "dxl_present_jumped_twice_after_torque_recenter:first={park}:second={again}"
                )));
            }
            if let Err(e) =
                self.write_and_verify_goal(again, "setup_goal_match_present_after_recenter_hunt")
            {
                self.torque_off_setup("setup_torque_off_goal_unverified");
                return Err(e);
            }
            self.last_present = again;
            if again != self.startup_present {
                self.startup_present = again;
                self.persist_cage_evidence();
            }
        }
        self.torque_enabled = true;
        Ok(())
    }

    fn read_present_setup(&mut self, unread: &'static str) -> PlantResult<i32> {
        self.read_reg(ADDR_PRESENT_POSITION, 4)
            .ok()
            .and_then(|b| le_i32(&b))
            .ok_or_else(|| PlantError::refused(unread))
    }

    /// Torque-off + EEPROM rewrite lets the horn settle. Use the live
    /// Present, not the pre-off wrap, so Goal match cannot yank.
    fn live_park_for_recenter(&mut self) -> PlantResult<i32> {
        let park = self.read_present_setup("dxl_present_unreadable_after_torque")?;
        let (wizard_min, wizard_max) = self.wizard_legal_window();
        if park < wizard_min || park > wizard_max {
            return Err(PlantError::refused(format!(
                "dxl_present_outside_wizard_limits_after_torque:present={park}:min={wizard_min}:max={wizard_max}"
            )));
        }
        Ok(park)
    }

    fn write_and_verify_goal(&mut self, present: i32, why: &'static str) -> PlantResult<()> {
        self.write_reg(
            ADDR_GOAL_POSITION,
            &present.to_le_bytes(),
            why,
            Some(present),
            false,
        )?;
        let goal_got = self
            .read_reg(ADDR_GOAL_POSITION, 4)
            .ok()
            .and_then(|b| le_i32(&b));
        if goal_got != Some(present) {
            return Err(PlantError::refused(format!(
                "dxl_goal_unverified:got={goal_got:?}:want={present}"
            )));
        }
        self.last_goal = Some(present);
        self.persist_positions();
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

    fn apply_pwm_output_cap(&mut self) -> PlantResult<()> {
        let want = self.cfg.max_pwm_limit_raw;
        if want > XL330_PWM_LIMIT_MAX {
            return Err(PlantError::refused(format!(
                "metal_pwm_limit_raw_out_of_range:{want} max={XL330_PWM_LIMIT_MAX}"
            )));
        }
        self.pwm_limit_requested = want;
        let got = self
            .read_reg(ADDR_PWM_LIMIT, 2)
            .ok()
            .and_then(|b| le_u16(&b));
        if got != Some(want) {
            self.write_reg(
                ADDR_PWM_LIMIT,
                &want.to_le_bytes(),
                "setup_pwm_limit_cap",
                None,
                false,
            )?;
        }
        let measured = self
            .read_reg(ADDR_PWM_LIMIT, 2)
            .ok()
            .and_then(|b| le_u16(&b))
            .ok_or_else(|| PlantError::refused("metal_pwm_limit_readback_unverified"))?;
        if measured > want {
            return Err(PlantError::refused(format!(
                "metal_pwm_limit_readback_exceeds_cap:requested={want} measured={measured}"
            )));
        }
        if measured == crate::protocol::FACTORY_PWM_LIMIT
            && want < crate::protocol::FACTORY_PWM_LIMIT
        {
            return Err(PlantError::refused(format!(
                "metal_pwm_limit_silently_restored_factory:requested={want} measured={measured}"
            )));
        }
        self.pwm_limit = measured;
        self.persist_pwm_evidence();
        Ok(())
    }

    /// Goal PWM is a Goal Value. Bus Watchdog 0xFF makes it read-only,
    /// so this runs after setup writes watchdog 0.
    fn sync_goal_pwm_to_measured_cap(&mut self) -> PlantResult<()> {
        let measured = self.pwm_limit;
        let want_goal = i16::try_from(measured).map_err(|_| {
            PlantError::refused(format!("metal_goal_pwm_cap_out_of_range:{measured}"))
        })?;
        let got_goal = self
            .read_reg(ADDR_GOAL_PWM, 2)
            .ok()
            .and_then(|b| le_i16(&b));
        self.goal_pwm = got_goal.unwrap_or(0);
        if got_goal != Some(want_goal) {
            self.write_reg(
                ADDR_GOAL_PWM,
                &want_goal.to_le_bytes(),
                "setup_goal_pwm_match_cap",
                None,
                false,
            )?;
            self.goal_pwm = want_goal;
        }
        let measured_goal = self
            .read_reg(ADDR_GOAL_PWM, 2)
            .ok()
            .and_then(|b| le_i16(&b))
            .ok_or_else(|| PlantError::refused("metal_goal_pwm_readback_unverified"))?;
        if measured_goal != want_goal {
            return Err(PlantError::refused(format!(
                "metal_goal_pwm_readback_mismatch:want={want_goal} measured={measured_goal}"
            )));
        }
        self.goal_pwm = measured_goal;
        Ok(())
    }

    fn establish_startup_cage(&mut self, present: i32) -> PlantResult<()> {
        // crash_if skips Drop, so EEPROM still holds the previous session
        // cage. Intersecting a new ±excursion window with that leftover
        // cage ratchets headroom away; a later nudge then refuses and the
        // frozen ONLINE mapping abort-latches the instance. Restore the
        // recorded Wizard window first (fail-safe while serve is down).
        self.restore_recorded_wizard_window_before_new_cage()?;
        let excursion = self.cfg.max_total_excursion_ticks;
        if excursion < 0 {
            return Err(PlantError::refused(format!(
                "metal_max_total_excursion_ticks_negative:{excursion}"
            )));
        }
        let legal_min = XL330_POSITION_MODE_MIN.max(self.min_position);
        let legal_max = XL330_POSITION_MODE_MAX.min(self.max_position);
        let experiment_min = present.saturating_sub(excursion).max(legal_min);
        let experiment_max = present.saturating_add(excursion).min(legal_max);
        if experiment_min > experiment_max {
            return Err(PlantError::refused(format!(
                "metal_experiment_cage_empty:present={present}:min={experiment_min}:max={experiment_max}"
            )));
        }
        // A leftover Wizard window tighter than the certified step (or only
        // barely 32/36 ticks at the edge) used to pass setup, run valid_hold,
        // then miss after propose re-acquires last_present and abort-latch
        // ONLINE. Do not widen EEPROM against a fixture; refuse before write.
        cage_allows_inbound_nudge_after_hold_still(
            present,
            experiment_min,
            experiment_max,
            self.cfg.max_position_delta_ticks,
            self.cfg.tau_max,
        )
        .map_err(|e| PlantError::refused(format!("metal_experiment_cage_no_inbound_step:{e}")))?;
        self.startup_present = present;
        self.experiment_min = experiment_min;
        self.experiment_max = experiment_max;
        self.eeprom_min_saved = Some(self.min_position);
        self.eeprom_max_saved = Some(self.max_position);
        if self.min_position != experiment_min {
            self.write_reg(
                ADDR_MIN_POSITION_LIMIT,
                &experiment_min.to_le_bytes(),
                "setup_experiment_min_position",
                None,
                false,
            )?;
        }
        if self.max_position != experiment_max {
            self.write_reg(
                ADDR_MAX_POSITION_LIMIT,
                &experiment_max.to_le_bytes(),
                "setup_experiment_max_position",
                None,
                false,
            )?;
        }
        let got_min = self
            .read_reg(ADDR_MIN_POSITION_LIMIT, 4)
            .ok()
            .and_then(|b| le_i32(&b))
            .ok_or_else(|| PlantError::refused("metal_experiment_min_readback_unverified"))?;
        let got_max = self
            .read_reg(ADDR_MAX_POSITION_LIMIT, 4)
            .ok()
            .and_then(|b| le_i32(&b))
            .ok_or_else(|| PlantError::refused("metal_experiment_max_readback_unverified"))?;
        if got_min != experiment_min || got_max != experiment_max {
            return Err(PlantError::refused(format!(
                "metal_experiment_cage_readback_mismatch:want={experiment_min}..{experiment_max} got={got_min}..{got_max}"
            )));
        }
        self.min_position = experiment_min;
        self.max_position = experiment_max;
        self.persist_cage_evidence();
        Ok(())
    }

    fn persist_pwm_evidence(&self) {
        let v = serde_json::json!({
            "requested": self.pwm_limit_requested,
            "measured": self.pwm_limit,
            "percent_requested": pwm_limit_percent(self.pwm_limit_requested),
            "percent_measured": pwm_limit_percent(self.pwm_limit),
            "unit": "raw * 0.113 = percent of full PWM output; 885 ≈ 100%",
            "role": "output_pwm_cap_not_certified_torque_limit",
            "current_limit_milli_configured": self.cfg.current_limit_milli,
            "current_limit_is_not_position_mode_torque_boundary": true,
        });
        let _ = std::fs::write(self.bus.join(PWM_EVIDENCE_FILE), v.to_string());
    }

    fn restore_recorded_wizard_window_before_new_cage(&mut self) -> PlantResult<()> {
        let path = self.bus.join(CAGE_EVIDENCE_FILE);
        let Ok(raw) = std::fs::read_to_string(&path) else {
            return Ok(());
        };
        let Ok(v) = serde_json::from_str::<serde_json::Value>(&raw) else {
            return Ok(());
        };
        let Some(min) = v.get("eeprom_previous_min").and_then(|x| x.as_i64()) else {
            return Ok(());
        };
        let Some(max) = v.get("eeprom_previous_max").and_then(|x| x.as_i64()) else {
            return Ok(());
        };
        let min = i32::try_from(min).map_err(|_| {
            PlantError::refused(format!("metal_wizard_window_restore_min_overflow:{min}"))
        })?;
        let max = i32::try_from(max).map_err(|_| {
            PlantError::refused(format!("metal_wizard_window_restore_max_overflow:{max}"))
        })?;
        if !(XL330_POSITION_MODE_MIN..=XL330_POSITION_MODE_MAX).contains(&min)
            || !(XL330_POSITION_MODE_MIN..=XL330_POSITION_MODE_MAX).contains(&max)
            || min > max
        {
            return Err(PlantError::refused(format!(
                "metal_wizard_window_restore_invalid:min={min}:max={max}"
            )));
        }
        if min == self.min_position && max == self.max_position {
            return Ok(());
        }
        self.write_reg(
            ADDR_MIN_POSITION_LIMIT,
            &min.to_le_bytes(),
            "setup_restore_wizard_min",
            None,
            false,
        )?;
        self.write_reg(
            ADDR_MAX_POSITION_LIMIT,
            &max.to_le_bytes(),
            "setup_restore_wizard_max",
            None,
            false,
        )?;
        let got_min = self
            .read_reg(ADDR_MIN_POSITION_LIMIT, 4)
            .ok()
            .and_then(|b| le_i32(&b))
            .ok_or_else(|| PlantError::refused("metal_wizard_window_restore_min_unverified"))?;
        let got_max = self
            .read_reg(ADDR_MAX_POSITION_LIMIT, 4)
            .ok()
            .and_then(|b| le_i32(&b))
            .ok_or_else(|| PlantError::refused("metal_wizard_window_restore_max_unverified"))?;
        if got_min != min || got_max != max {
            return Err(PlantError::refused(format!(
                "metal_wizard_window_restore_readback_mismatch:want={min}..{max} got={got_min}..{got_max}"
            )));
        }
        self.min_position = min;
        self.max_position = max;
        Ok(())
    }

    fn persist_cage_evidence(&self) {
        let v = serde_json::json!({
            "startup_present": self.startup_present,
            "experiment_min": self.experiment_min,
            "experiment_max": self.experiment_max,
            "max_total_excursion_ticks": self.cfg.max_total_excursion_ticks,
            "eeprom_previous_min": self.eeprom_min_saved,
            "eeprom_previous_max": self.eeprom_max_saved,
            "eeprom_applied": true,
            "legal_position_mode": [XL330_POSITION_MODE_MIN, XL330_POSITION_MODE_MAX],
        });
        let _ = std::fs::write(self.bus.join(CAGE_EVIDENCE_FILE), v.to_string());
    }

    fn restore_eeprom_position_limits(&mut self) {
        let (Some(min), Some(max)) = (self.eeprom_min_saved, self.eeprom_max_saved) else {
            return;
        };
        if min == self.min_position && max == self.max_position {
            return;
        }
        let _ = self.write_reg(ADDR_TORQUE_ENABLE, &[0], "teardown_torque_off", None, false);
        self.torque_enabled = false;
        let _ = self.write_reg(
            ADDR_MIN_POSITION_LIMIT,
            &min.to_le_bytes(),
            "teardown_restore_min_position",
            None,
            false,
        );
        let _ = self.write_reg(
            ADDR_MAX_POSITION_LIMIT,
            &max.to_le_bytes(),
            "teardown_restore_max_position",
            None,
            false,
        );
        self.min_position = min;
        self.max_position = max;
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

    fn send_frame(&mut self, request: &[u8]) -> io::Result<()> {
        let port = self
            .port
            .as_mut()
            .ok_or_else(|| io::Error::other("metal_serial_closed"))?;
        port.clear(serialport::ClearBuffer::Input)
            .map_err(io::Error::other)?;
        port.write_all(request).map_err(io::Error::other)?;
        port.flush().map_err(io::Error::other)?;
        Ok(())
    }

    fn xfer_once(
        &mut self,
        request: &[u8],
        expect_status: bool,
    ) -> io::Result<crate::protocol::StatusPacket> {
        self.send_frame(request)?;
        self.recv_status(expect_status)
    }

    fn recv_status(&mut self, expect_status: bool) -> io::Result<crate::protocol::StatusPacket> {
        half_duplex_turnaround(&self.cfg.device);
        if !expect_status {
            return Err(io::Error::other("metal_no_status_expected"));
        }
        let port = self
            .port
            .as_mut()
            .ok_or_else(|| io::Error::other("metal_serial_closed"))?;
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
            // Before transport. Distinguished from after_serial_tx_before_status.
            realityos_plant::hil_faults::crash_if("during_write");
        }
        if let Err(e) = self.send_frame(&frame) {
            self.connected = false;
            if count_command_egress {
                let _ = self.egress.record_ack(false, 0xFF, None);
            }
            return Err(PlantError::refused(format!("dxl_io:{e}")));
        }
        if count_command_egress {
            self.egress
                .record_serial_tx(instruction, addr)
                .map_err(|e| PlantError::refused(format!("egress_log:{e}")))?;
            realityos_plant::hil_faults::crash_if("after_serial_tx_before_status");
        }
        match self.recv_status(true) {
            Ok(st) => {
                self.note_status_error(st.error);
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
            Ok(st) if instruction_ok(st.error) => {
                self.note_status_error(st.error);
                Ok(st.params)
            }
            Ok(st) => {
                self.note_status_error(st.error);
                Err(PlantError::refused(format!(
                    "dxl_status_error:{}",
                    st.error
                )))
            }
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
        // VIN fault is a supply/cutoff latch, not a vanished fd — ESTOP
        // / close must still be able to torque-off on this exclusive port.
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

    fn ticks_from_action(&self, action: &[f64]) -> i32 {
        let a0 = action.first().copied().unwrap_or(0.0);
        if !a0.is_finite() || a0.abs() < 1e-12 {
            return 0;
        }
        let scale = if self.cfg.tau_max.abs() < 1e-12 {
            0.0
        } else {
            f64::from(self.cfg.max_position_delta_ticks) / self.cfg.tau_max.abs()
        };
        (a0 * scale).round().clamp(
            f64::from(-self.cfg.max_position_delta_ticks),
            f64::from(self.cfg.max_position_delta_ticks),
        ) as i32
    }

    fn enable_torque_matched_to_present(&mut self) -> PlantResult<()> {
        let present = self
            .read_reg(ADDR_PRESENT_POSITION, 4)
            .ok()
            .and_then(|b| le_i32(&b))
            .ok_or_else(|| PlantError::refused("dxl_present_unreadable_before_torque"))?;
        if !self.in_experiment_cage(present) {
            return Err(PlantError::refused(format!(
                "experiment_cage_violation:present={present}:min={}:max={}",
                self.experiment_min, self.experiment_max
            )));
        }
        self.last_present = present;
        self.write_and_verify_goal(present, "reenable_goal_match_present")?;
        self.write_reg(ADDR_TORQUE_ENABLE, &[1], "torque_on", None, false)?;
        self.torque_enabled = true;
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
                    "reenable_torque_off_present_unread",
                    None,
                    false,
                );
                self.torque_enabled = false;
                return Err(PlantError::refused("dxl_present_unreadable_after_torque"));
            }
        };
        if after != present {
            if !self.in_experiment_cage(after) {
                let _ = self.write_reg(
                    ADDR_TORQUE_ENABLE,
                    &[0],
                    "reenable_torque_off_present_jump",
                    None,
                    false,
                );
                self.torque_enabled = false;
                return Err(PlantError::refused(format!(
                    "dxl_present_outside_experiment_cage_after_torque:present={after}:min={}:max={}",
                    self.experiment_min, self.experiment_max
                )));
            }
            if let Err(e) =
                self.write_and_verify_goal(after, "reenable_goal_match_present_after_torque")
            {
                let _ = self.write_reg(
                    ADDR_TORQUE_ENABLE,
                    &[0],
                    "reenable_torque_off_goal_unverified",
                    None,
                    false,
                );
                self.torque_enabled = false;
                return Err(e);
            }
            self.last_present = after;
        }
        Ok(())
    }

    fn note_status_error(&mut self, error: u8) {
        if error & STATUS_ALERT != 0 {
            self.hw_error_needs_refresh = true;
        }
    }

    fn refresh_hw_error_status(&mut self) {
        // Same contract as confirm_eeprom_identity: this extra READ is
        // diagnostic. A CRC/timeout after a good motion sample must not
        // latch `connected=false` / bus_lost and kill the session.
        let was_connected = self.connected;
        match self.read_reg(ADDR_HARDWARE_ERROR, 1) {
            Ok(b) => {
                if let Some(v) = b.first().copied() {
                    self.last_hw_error = v;
                    self.hw_error_needs_refresh = false;
                }
            }
            Err(_) => {
                self.connected = was_connected;
            }
        }
    }

    /// Intended goal before cage refuse. Does not clamp outbound steps inward.
    fn intended_goal(&self, action: &[f64]) -> i32 {
        let ticks = self.ticks_from_action(action);
        if ticks == 0 {
            return self.last_present;
        }
        self.last_present.saturating_add(ticks)
    }

    fn in_experiment_cage(&self, pos: i32) -> bool {
        pos >= self.experiment_min && pos <= self.experiment_max
    }

    /// Test-only: udev can dangle by-id / rename ttyUSB0 while the exclusive
    /// fd remains the live UART. The first hold must not treat that as unplug
    /// or blank the open-time adapter serial.
    pub fn simulate_udev_path_vanished(&mut self) {
        self.cfg.device = PathBuf::from("/dev/realityos-metal-vanished-udev-name");
    }

    /// crash_if / SIGKILL skip Drop. Tests need the same leftover EEPROM
    /// cage plus a surviving `position_cage.json` without releasing the
    /// PTY by restoring Wizard limits.
    pub fn abandon_without_eeprom_restore_for_test(&mut self) {
        if self.bus_up() && self.torque_enabled {
            let _ = self.write_reg(
                ADDR_TORQUE_ENABLE,
                &[0],
                "test_abandon_torque_off",
                None,
                false,
            );
            self.torque_enabled = false;
        }
        self.eeprom_min_saved = None;
        self.eeprom_max_saved = None;
        self.port = None;
        self.connected = false;
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
        if self.vin_fault {
            return Err(PlantError::refused("dxl_vin_unreadable"));
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
        if self.hw_error_needs_refresh
            && (!self.live_io || t0.elapsed() < Duration::from_millis(15))
        {
            self.refresh_hw_error_status();
        }
        let err = self.last_hw_error;
        self.persist_vin(volt);
        if self.live_io {
            if volt == 0 {
                self.vin_fault = true;
                return Err(PlantError::refused("dxl_vin_unreadable"));
            }
            if self.min_voltage != 0
                && self.max_voltage != 0
                && (volt < self.min_voltage || volt > self.max_voltage)
            {
                self.vin_fault = true;
                return Err(PlantError::refused(format!(
                    "dxl_vin_outside_wizard_limits:vin_0.1v={volt}:min={}:max={}",
                    self.min_voltage, self.max_voltage
                )));
            }
        }
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
        if self.vin_fault {
            return Err(PlantError::refused("dxl_vin_unreadable"));
        }
        if !is_xl330_model(self.model) {
            return Err(PlantError::refused("metal_refuses_non_xl330_model"));
        }
        // Torque-on tracks Goal Position. After ESTOP the last certified
        // goal can differ from present. Re-enable without rematching yanks
        // the horn before the certified write. Match present first.
        if !self.torque_enabled {
            self.enable_torque_matched_to_present()?;
        }
        // propose() already acquired sensors. Extra register pokes here would
        // exceed the 100 ms software-watchdog miss on a USB-UART bench.
        let goal = self.intended_goal(action);
        if !self.in_experiment_cage(goal) {
            return Err(PlantError::refused(format!(
                "experiment_cage_violation:goal={goal}:min={}:max={}",
                self.experiment_min, self.experiment_max
            )));
        }
        if !self.in_experiment_cage(self.last_present) {
            return Err(PlantError::refused(format!(
                "experiment_cage_violation:present={}:min={}:max={}",
                self.last_present, self.experiment_min, self.experiment_max
            )));
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
        if self.bus_up() {
            self.restore_eeprom_position_limits();
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

fn early_broadcast_torque_off(port: &mut dyn SerialPort, device: &Path) {
    let frame = encode_write(BROADCAST_ID, ADDR_TORQUE_ENABLE, &[0]);
    let _ = port.clear(serialport::ClearBuffer::Input);
    let _ = port.write_all(&frame);
    let _ = port.flush();
    half_duplex_turnaround(device);
    let saved = port.timeout();
    let _ = port.set_timeout(Duration::from_millis(10));
    let mut tmp = [0u8; 64];
    let end = Instant::now() + Duration::from_millis(10);
    while Instant::now() < end {
        if port.read(&mut tmp).is_err() {
            break;
        }
    }
    let _ = port.set_timeout(saved);
}

/// Discover's first open is factory 57 600. Wizard 115 200 or 9 600 plus
/// Startup Configuration bit 0 tracks Goal 0 during that settle unless
/// we also speak those rates on the held fd. 9 600 is Wizard baud index
/// 0 and is safe to speak. Do not include 1 Mbps / 2 / 3 / 4 Mbps: a
/// CH340 can wedge.
fn settle_quiesce_bauds(open_baud: u32) -> Vec<u32> {
    let mut out = if open_baud == 115_200 {
        vec![115_200, 57_600]
    } else {
        vec![open_baud, 115_200]
    };
    if !out.contains(&9_600) {
        out.push(9_600);
    }
    out
}

fn broadcast_torque_off_at(
    port: &mut dyn SerialPort,
    device: &Path,
    current: &mut u32,
    target: u32,
) {
    if *current != target {
        if retune_held_baud(port, device, target).is_err() {
            return;
        }
        *current = target;
    }
    early_broadcast_torque_off(port, device);
}

fn open_settle_and_quiesce(port: &mut dyn SerialPort, device: &Path, baud: u32) -> io::Result<()> {
    // Cheap FTDI/CP2102 DTR-RESET plus low VIN can exceed 300 ms. Robotis
    // documents ~100–300 ms; 500 ms covers the first-open reboot window.
    // PTY has no DTR; keep tests fast.
    //
    // Do not sit silent for that whole window. Startup Configuration bit 0
    // torque-ons after reboot and tracks Goal (RAM initial 0). Broadcast
    // torque-off as soon as the servo might answer, then keep retrying.
    // Alternate the open baud with 115 200 and Wizard 9 600 so those
    // leftover rates are not left tracking until the later discover
    // retune. Do not speak 1 Mbps here (CH340 wedge).
    let total_ms = if is_pty_path(device) { 100 } else { 500 };
    let step_ms = if is_pty_path(device) { 20 } else { 50 };
    let end = Instant::now() + Duration::from_millis(total_ms);
    let bauds = settle_quiesce_bauds(baud);
    let mut current = baud;
    let mut step = 0usize;
    broadcast_torque_off_at(port, device, &mut current, bauds[0]);
    while Instant::now() < end {
        let remain = end.saturating_duration_since(Instant::now());
        if remain.is_zero() {
            break;
        }
        std::thread::sleep(remain.min(Duration::from_millis(step_ms)));
        if Instant::now() < end {
            step += 1;
            broadcast_torque_off_at(port, device, &mut current, bauds[step % bauds.len()]);
        }
    }
    // HeldDiscover records the open baud. An ignored restore left the fd at
    // 115 200 while sniff still thought 57 600, so a factory servo missed
    // and the next automatic open was 1 Mbps (CH340 wedge).
    if current != baud {
        retune_held_baud(port, device, baud)?;
    }
    Ok(())
}

fn open_xl330_serial(device: &Path, baud: u32) -> io::Result<Box<dyn SerialPort>> {
    open_xl330_serial_with(device, baud, !is_pty_path(device))
}

/// Discover holds one exclusive fd. `set_baud_rate` must not re-open the
/// tty: that would assert DTR and RESET a cheap FTDI/CP2102 servo again.
fn retune_held_baud(port: &mut dyn SerialPort, device: &Path, baud: u32) -> io::Result<()> {
    port.set_baud_rate(baud)
        .map_err(|e| io::Error::other(format!("dxl_set_baud:{e}")))?;
    let _ = port.clear(serialport::ClearBuffer::Input);
    if !is_pty_path(device) {
        std::thread::sleep(Duration::from_millis(50));
    }
    Ok(())
}

struct HeldDiscover {
    port: Box<dyn SerialPort>,
    baud: u32,
    device: PathBuf,
}

impl HeldDiscover {
    fn open(device: PathBuf, baud: u32) -> io::Result<Self> {
        let port = open_xl330_serial(&device, baud)?;
        Ok(Self { port, baud, device })
    }

    fn ensure(&mut self, device: &Path, baud: u32) -> io::Result<()> {
        if self.device != device {
            *self = Self::open(device.to_path_buf(), baud)?;
            return Ok(());
        }
        if self.baud == baud {
            return Ok(());
        }
        match retune_held_baud(&mut *self.port, device, baud) {
            Ok(()) => {
                self.baud = baud;
                // This rate may be the live one. Broadcast now; do not wait
                // for identify while Startup Configuration tracks a stale goal.
                early_broadcast_torque_off(&mut *self.port, device);
                Ok(())
            }
            Err(_) => {
                // Adapter rejected an in-place retune. Reopen at the new
                // rate (one DTR-RESET) rather than walking the rest of the
                // scan on a wedged termios.
                *self = Self::open(device.to_path_buf(), baud)?;
                Ok(())
            }
        }
    }
}

fn open_xl330_serial_with(
    device: &Path,
    baud: u32,
    exclusive: bool,
) -> io::Result<Box<dyn SerialPort>> {
    // Real UART takes exclusive on the first open, then clears HUPCL on
    // that fd. PTY skips TIOCEXCL (`process::exit` leaves it on pts).
    let take_exclusive = exclusive && !is_pty_path(device);
    let mut port = serialport::new(device.to_string_lossy(), baud)
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
    open_settle_and_quiesce(&mut port, device, baud)?;
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
fn sniff_on_port(port: &mut dyn SerialPort, device: &Path) -> io::Result<Vec<u8>> {
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
    if ids.len() > 1 {
        if let Some(primary) = primary_if_secondary_pair_on(port, device, &ids) {
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
fn primary_if_secondary_pair_on(
    port: &mut dyn SerialPort,
    device: &Path,
    ids: &[u8],
) -> Option<u8> {
    if ids.len() != 2 {
        return None;
    }
    for &id in ids {
        poke_srl_all(port, device, id);
    }
    let via_a = read_id_register(port, device, ids[0])?;
    let via_b = read_id_register(port, device, ids[1])?;
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
        slave
            .set_baud_rate(115_200)
            .expect("in-place baud retune must not require reopen");
        assert!(
            !tcgetattr(fd)
                .expect("tcgetattr after set_baud_rate")
                .control_flags
                .contains(ControlFlags::HUPCL),
            "serialport set_baud_rate must not restore HUPCL (discover retune would DTR-RESET on close)"
        );
    }

    #[test]
    fn settle_quiesce_speaks_wizard_115200_without_one_megabit() {
        let factory = settle_quiesce_bauds(57_600);
        assert_eq!(factory[0], 57_600);
        assert_eq!(factory[1], 115_200);
        assert!(
            factory.contains(&9_600),
            "Wizard baud-index 0 tracks Goal 0 during settle unless spoken"
        );
        assert!(!factory.contains(&1_000_000));
        assert!(!factory.contains(&2_000_000));
        let wizard = settle_quiesce_bauds(115_200);
        assert_eq!(wizard[0], 115_200);
        assert_eq!(wizard[1], 57_600);
        assert!(wizard.contains(&9_600));
        assert!(!wizard.contains(&1_000_000));
        let slow = settle_quiesce_bauds(9_600);
        assert_eq!(slow[0], 9_600);
        assert_eq!(slow[1], 115_200);
        assert_eq!(slow.iter().filter(|b| **b == 9_600).count(), 1);
        assert!(!slow.contains(&1_000_000));
    }

    #[test]
    fn settle_restores_factory_open_baud_after_115200_poke() {
        let (_master, mut slave) = TTYPort::pair().expect("pty pair");
        slave.set_baud_rate(57_600).expect("open baud");
        let path = Path::new("/dev/pts/settle-restore");
        assert!(is_pty_path(path));
        open_settle_and_quiesce(&mut slave, path, 57_600).expect("settle");
        assert_eq!(
            slave.baud_rate().expect("read baud"),
            57_600,
            "HeldDiscover still records 57600; a leftover 115200 fd misses the factory servo"
        );
    }

    #[test]
    fn settle_restores_wizard_open_baud_after_factory_poke() {
        let (_master, mut slave) = TTYPort::pair().expect("pty pair");
        slave.set_baud_rate(115_200).expect("open baud");
        let path = Path::new("/dev/pts/settle-restore-wizard");
        assert!(is_pty_path(path));
        open_settle_and_quiesce(&mut slave, path, 115_200).expect("settle");
        assert_eq!(
            slave.baud_rate().expect("read baud"),
            115_200,
            "open at Wizard 115200 must leave the fd at 115200 after the factory poke"
        );
    }
}
