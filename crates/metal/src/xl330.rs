//! One XL330 HardwareDriverPort. Translates authorized actions to Protocol 2.0.
//! Contains no Reality OS policy.

use std::fs::{File, OpenOptions};
use std::io::{self, Read, Write};
use std::os::unix::fs::{OpenOptionsExt, PermissionsExt};
use std::path::{Path, PathBuf};
use std::time::Duration;

use fs2::FileExt;
use realityos_plant::{
    ActionParams, HardwareDriverPort, HardwareIdentity, PlantError, PlantRealized, PlantResult,
    SensorPacket,
};
use serialport::SerialPort;

use crate::config::{
    candidate_bauds, candidate_servo_ids, MetalConfig, BUS_DIR, GOAL_FILE, LOCK_FILE, PRESENT_FILE,
    VIN_FILE,
};
use crate::egress::EgressLog;
use crate::identity::{usb_identity_for_tty, MeasuredIdentity};
use crate::protocol::{
    decode_status_scan, encode_ping, encode_read, encode_write, find_header, is_xl330_model,
    le_i32, le_u16, ADDR_CURRENT_LIMIT, ADDR_FIRMWARE_VERSION, ADDR_GOAL_POSITION,
    ADDR_HARDWARE_ERROR, ADDR_MODEL_NUMBER, ADDR_OPERATING_MODE, ADDR_PROFILE_ACCEL,
    ADDR_PROFILE_VELOCITY, ADDR_REALTIME_TICK, ADDR_TORQUE_ENABLE, OPERATING_MODE_POSITION,
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
    last_hw_error: u8,
}

impl Xl330Driver {
    pub fn open(cfg: MetalConfig, root: impl AsRef<Path>) -> io::Result<Self> {
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
        };
        driver.connect_serial()?;
        driver.refresh_identity();
        if driver.connected {
            driver
                .apply_bench_limits()
                .map_err(|e| io::Error::other(e.to_string()))?;
            driver.enter_live_io();
        }
        Ok(driver)
    }

    /// Probe-only: try configured baud/id first, then common XL330 bus settings.
    /// Production `serve` keeps using [`Self::open`] with the bound config.
    pub fn open_discovering(
        cfg: MetalConfig,
        root: impl AsRef<Path>,
    ) -> io::Result<(Self, MetalConfig)> {
        if !cfg.device.exists() {
            return Err(io::Error::new(
                io::ErrorKind::NotFound,
                format!("metal_device_missing:{}", cfg.device.display()),
            ));
        }
        let extra_baud = std::env::var("REALITYOS_METAL_BAUD")
            .ok()
            .and_then(|s| s.parse().ok());
        let extra_id = std::env::var("REALITYOS_METAL_SERVO_ID")
            .ok()
            .and_then(|s| s.parse().ok());
        let bauds = candidate_bauds(cfg.baud, extra_baud);
        let ids = candidate_servo_ids(cfg.servo_id, extra_id);
        let mut last_err: Option<io::Error> = None;
        for baud in bauds {
            for id in &ids {
                let mut attempt = cfg.clone();
                attempt.baud = baud;
                attempt.servo_id = *id;
                match Self::open(attempt.clone(), root.as_ref()) {
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
        match Self::open(configured.clone(), root.as_ref()) {
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

    pub fn measured(&self) -> MeasuredIdentity {
        let (usb_serial, usb_fallback) = usb_identity_for_tty(&self.cfg.device);
        MeasuredIdentity::from_hardware(
            &self.cfg,
            usb_serial,
            usb_fallback,
            self.model,
            self.firmware,
            self.connected,
        )
    }

    fn connect_serial(&mut self) -> io::Result<()> {
        if !self.cfg.device.exists() {
            self.connected = false;
            return Err(io::Error::new(
                io::ErrorKind::NotFound,
                format!("metal_device_missing:{}", self.cfg.device.display()),
            ));
        }
        let _ = std::fs::set_permissions(&self.cfg.device, std::fs::Permissions::from_mode(0o600));
        let mut port = serialport::new(self.cfg.device.to_string_lossy(), self.cfg.baud)
            .timeout(Duration::from_millis(150))
            .open()
            .map_err(io::Error::other)?;
        port.write_data_terminal_ready(true).ok();
        // U2D2/FTDI often drops the first packet if we ping immediately
        // after open. Discover tries each baud/id pair once; a cold miss
        // on the real pair never comes back.
        std::thread::sleep(Duration::from_millis(100));
        self.port = Some(port);
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

    fn apply_bench_limits(&mut self) -> PlantResult<()> {
        // Current limit / operating mode are EEPROM; only write with torque off, and only if needed.
        self.write_reg(ADDR_TORQUE_ENABLE, &[0], "setup_torque_off", None, false)?;
        self.torque_enabled = false;
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
        }
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
        if let Ok(b) = self.read_reg(ADDR_HARDWARE_ERROR, 1) {
            self.last_hw_error = b.first().copied().unwrap_or(0);
        }
        // EEPROM writes can NAK the next instruction if we immediately continue.
        std::thread::sleep(Duration::from_millis(50));
        // Torque-on here (watchdog not running yet) so the first certified
        // write is a single goal_position xfer, not torque_on + goal.
        self.write_reg(ADDR_TORQUE_ENABLE, &[1], "setup_torque_on", None, false)?;
        self.torque_enabled = true;
        Ok(())
    }

    fn enter_live_io(&mut self) {
        self.live_io = true;
        if let Some(port) = self.port.as_mut() {
            let _ = port.set_timeout(Duration::from_millis(40));
        }
    }

    fn refresh_identity(&mut self) {
        let mut id = self.measured().hardware_identity(&self.cfg);
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
        if !self.cfg.device.exists() || !self.bus_up() {
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
                if let Ok(st) = decode_status_scan(&acc) {
                    return Ok(st);
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
                let ok = st.error == 0;
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
            Ok(st) if st.error == 0 => Ok(st.params),
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
        self.connected && self.port.is_some() && self.cfg.device.exists()
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
        let cur = i16::from_le_bytes([b[6], b[7]]);
        let vel = le_i32(&b[8..12]).unwrap_or(0);
        let pos =
            le_i32(&b[12..16]).ok_or_else(|| PlantError::refused("dxl_short_present_position"))?;
        let volt = le_u16(&b[24..26]).unwrap_or(0);
        self.last_present = pos;
        self.persist_positions();
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
        self.last_present.saturating_add(ticks).clamp(0, 4095)
    }
}

impl HardwareDriverPort for Xl330Driver {
    fn probe_identity(&self) -> HardwareIdentity {
        let mut id = self.last_identity.clone();
        if self.campaign_disconnected() || !self.cfg.device.exists() || !self.bus_up() {
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
        let (pos, vel, cur, volt, tick) = self.read_motion_block()?;
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
        if self.bus_up() {
            let _ = self.write_reg(ADDR_TORQUE_ENABLE, &[0], "close_torque_off", None, false);
            self.torque_enabled = false;
        }
        self.port = None;
        self.connected = false;
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
