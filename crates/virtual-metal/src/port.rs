//! HardwareDriverPort that speaks Protocol 2.0 to [`crate::VirtualXl330`].
//!
//! In-process path: production encode/decode through [`crate::peer::VirtualSerialPeer`].
//! `is_sim_harness` stays false: this is not the plant test harness. Honesty is
//! `metal=false` + `SIM_VIRTUAL_METAL_NOT_METAL` on identity. `HardwareBackedPlant`
//! uses `is_sim_harness` only to force `effective_metal=false`; Virtual Metal already
//! cannot mint measured evidence through identity or constructors.

use std::cell::RefCell;
use std::rc::Rc;

use realityos_metal::protocol::{
    decode_status_scan, encode_ping, encode_read, encode_write, instruction_ok, le_u16,
    ADDR_GOAL_POSITION, ADDR_MAX_VOLTAGE_LIMIT, ADDR_MIN_VOLTAGE_LIMIT, ADDR_PRESENT_POSITION,
    ADDR_PRESENT_VOLTAGE, ADDR_TORQUE_ENABLE,
};
use realityos_plant::{
    ActionParams, HardwareDriverPort, HardwareIdentity, PlantError, PlantRealized, PlantResult,
    SensorPacket,
};

use crate::device::VirtualXl330;
use crate::evidence::EVIDENCE_STATUS;
use crate::faults::{LifecycleLoss, WriteLifecycleBoundary};
use crate::peer::VirtualSerialPeer;

pub const DEFAULT_SERIAL: &str = "VM-XL330-M288-1";
pub const DEFAULT_CAL: &str = "VM-CAL-1";
pub const DEFAULT_DESIGN: &str = "vm-design-xl330-m288";
pub const DEFAULT_ACTUATOR: &str = "xl330:1";
/// Matches production live-I/O recv budget. Delay beyond this is a late ACK.
pub const REQUEST_TIMEOUT_MS: u64 = 40;

#[derive(Clone, Copy, Debug)]
pub struct LifecycleInject {
    pub boundary: WriteLifecycleBoundary,
    pub loss: LifecycleLoss,
}

#[derive(Clone)]
pub struct VirtualMetalPort {
    peer: Rc<RefCell<VirtualSerialPeer>>,
    serial: String,
    calibration_id: String,
    design: String,
    connected: bool,
    estop: bool,
    vin_fault: bool,
    tau_max: f64,
    max_position_delta_ticks: i32,
    lifecycle: Option<LifecycleInject>,
    request_timeout_ms: u64,
}

impl VirtualMetalPort {
    pub fn new(device: Rc<RefCell<VirtualXl330>>) -> Self {
        Self::from_peer(Rc::new(RefCell::new(VirtualSerialPeer::new(device))))
    }

    pub fn from_peer(peer: Rc<RefCell<VirtualSerialPeer>>) -> Self {
        Self {
            peer,
            serial: DEFAULT_SERIAL.into(),
            calibration_id: DEFAULT_CAL.into(),
            design: DEFAULT_DESIGN.into(),
            connected: true,
            estop: false,
            vin_fault: false,
            tau_max: 5.0,
            max_position_delta_ticks: 32,
            lifecycle: None,
            request_timeout_ms: REQUEST_TIMEOUT_MS,
        }
    }

    pub fn inject_lifecycle(&mut self, inj: LifecycleInject) {
        self.lifecycle = Some(inj);
    }

    pub fn disconnect(&mut self) {
        self.connected = false;
    }

    pub fn reconnect(&mut self) {
        self.connected = true;
    }

    fn ticks_from_action(&self, action: &[f64]) -> i32 {
        let a0 = action.first().copied().unwrap_or(0.0);
        if !a0.is_finite() || a0.abs() < 1e-12 {
            return 0;
        }
        let scale = if self.tau_max.abs() < 1e-12 {
            0.0
        } else {
            f64::from(self.max_position_delta_ticks) / self.tau_max.abs()
        };
        (a0 * scale).round().clamp(
            f64::from(-self.max_position_delta_ticks),
            f64::from(self.max_position_delta_ticks),
        ) as i32
    }

    fn exchange(&self, bytes: &[u8]) -> Vec<u8> {
        self.peer.borrow_mut().exchange(bytes)
    }

    fn read_u16_reg(&self, id: u8, addr: u16) -> PlantResult<u16> {
        let reply = self.exchange(&encode_read(id, addr, 2));
        let st = decode_status_scan(&reply).map_err(|e| PlantError::refused(e.to_string()))?;
        if !instruction_ok(st.error) {
            return Err(PlantError::refused(format!(
                "dxl_status_error:{}",
                st.error
            )));
        }
        le_u16(&st.params).ok_or_else(|| PlantError::refused("dxl_short_register"))
    }

    pub fn device(&self) -> Rc<RefCell<VirtualXl330>> {
        self.peer.borrow().device()
    }

    pub fn peer(&self) -> Rc<RefCell<VirtualSerialPeer>> {
        self.peer.clone()
    }

    pub fn runtime_identity_fields(&self) -> (String, String, String, String) {
        let fw = self.device().borrow().firmware_id_string();
        (
            self.serial.clone(),
            fw,
            self.calibration_id.clone(),
            self.design.clone(),
        )
    }

    pub fn physical_actions(&self) -> u64 {
        self.device().borrow().physical_action_count()
    }

    fn late_or_missing(reply: &[u8], delay_ms: u64, timeout_ms: u64) -> bool {
        reply.is_empty() || delay_ms > timeout_ms
    }
}

impl HardwareDriverPort for VirtualMetalPort {
    fn probe_identity(&self) -> HardwareIdentity {
        let id = self.device().borrow().id();
        let reply = self.exchange(&encode_ping(id));
        let _ = decode_status_scan(&reply);
        HardwareIdentity {
            serial: self.serial.clone(),
            firmware_id: self.device().borrow().firmware_id_string(),
            calibration_id: self.calibration_id.clone(),
            design_content_hash: self.design.clone(),
            connected: self.connected && !self.estop,
            metal: false,
            evidence_status: EVIDENCE_STATUS.into(),
            actuator_ids: vec![DEFAULT_ACTUATOR.into()],
        }
    }

    fn read_sensor(&mut self, now_s: f64) -> PlantResult<SensorPacket> {
        if !self.connected {
            return Err(PlantError::Disconnected);
        }
        if self.vin_fault {
            return Err(PlantError::refused("dxl_vin_unreadable"));
        }
        let id = self.device().borrow().id();
        let reply = self.exchange(&encode_read(id, ADDR_PRESENT_POSITION, 4));
        let st = decode_status_scan(&reply).map_err(|e| PlantError::refused(e.to_string()))?;
        if !instruction_ok(st.error) {
            return Err(PlantError::refused(format!(
                "dxl_status_error:{}",
                st.error
            )));
        }
        let pos = if st.params.len() >= 4 {
            i32::from_le_bytes(st.params[..4].try_into().unwrap()) as f64
        } else {
            return Err(PlantError::refused("dxl_short_present_position"));
        };
        let volt = self.read_u16_reg(id, ADDR_PRESENT_VOLTAGE)?;
        let min_v = self.read_u16_reg(id, ADDR_MIN_VOLTAGE_LIMIT)?;
        let max_v = self.read_u16_reg(id, ADDR_MAX_VOLTAGE_LIMIT)?;
        if volt == 0 {
            self.vin_fault = true;
            return Err(PlantError::refused("dxl_vin_unreadable"));
        }
        if min_v != 0 && max_v != 0 && (volt < min_v || volt > max_v) {
            self.vin_fault = true;
            return Err(PlantError::refused(format!(
                "dxl_vin_outside_wizard_limits:vin_0.1v={volt}:min={min_v}:max={max_v}"
            )));
        }
        Ok(SensorPacket::from_samples(
            vec![
                ("present_position".into(), pos),
                ("vin_0.1v".into(), f64::from(volt)),
                ("vin_v".into(), f64::from(volt) / 10.0),
            ],
            now_s,
        ))
    }

    fn write_action(
        &mut self,
        action: &[f64],
        _params: &ActionParams,
    ) -> PlantResult<PlantRealized> {
        if self.estop {
            return Err(PlantError::EstopEngaged);
        }
        if !self.connected {
            return Err(PlantError::Disconnected);
        }
        if self.vin_fault {
            return Err(PlantError::refused("dxl_vin_unreadable"));
        }
        let inj = self.lifecycle.take();
        if let Some(inj) = inj {
            if skip_apply(inj) {
                return Err(PlantError::UnknownOutcome);
            }
        }
        let ticks = self.ticks_from_action(action);
        let id = self.device().borrow().id();
        if !self.device().borrow().torque_enabled() {
            let on = self.exchange(&encode_write(id, ADDR_TORQUE_ENABLE, &[1]));
            let _ = decode_status_scan(&on);
        }
        let present = self.device().borrow().present_position();
        let goal = if ticks == 0 {
            present
        } else {
            present.saturating_add(ticks)
        };
        if let Some(inj) = inj {
            if matches!(inj.loss, LifecycleLoss::DeviceReset)
                && matches!(
                    inj.boundary,
                    WriteLifecycleBoundary::AfterDeviceApply
                        | WriteLifecycleBoundary::AfterSerialWrite
                )
            {
                let _ = self.exchange(&encode_write(id, ADDR_GOAL_POSITION, &goal.to_le_bytes()));
                self.device()
                    .borrow_mut()
                    .apply_fault_kind(crate::faults::FaultKind::RebootDuringRequest);
                return Err(PlantError::UnknownOutcome);
            }
        }
        let reply = self.exchange(&encode_write(id, ADDR_GOAL_POSITION, &goal.to_le_bytes()));
        let delay = self.peer.borrow().last_delay_ms();
        if let Some(inj) = inj {
            if unknown_after_apply(inj) {
                return Err(PlantError::UnknownOutcome);
            }
        }
        if Self::late_or_missing(&reply, delay, self.request_timeout_ms) {
            return Err(PlantError::UnknownOutcome);
        }
        let st = decode_status_scan(&reply).map_err(|_| PlantError::UnknownOutcome)?;
        if !instruction_ok(st.error) {
            return Err(PlantError::refused(format!(
                "dxl_status_error:{}",
                st.error
            )));
        }
        Ok(PlantRealized::sim([
            (
                "present_position".into(),
                self.device().borrow().present_position() as f64,
            ),
            ("goal_position".into(), goal as f64),
        ]))
    }

    fn engage_hw_estop(&mut self, _reason: &str) {
        self.estop = true;
        let id = self.device().borrow().id();
        let _ = self.exchange(&encode_write(id, ADDR_TORQUE_ENABLE, &[0]));
    }

    fn clear_hw_estop(&mut self, operator_ack: bool) -> PlantResult<()> {
        if !operator_ack {
            return Err(PlantError::OperatorAckRequired);
        }
        self.estop = false;
        Ok(())
    }

    fn is_connected(&self) -> bool {
        self.connected && !self.estop
    }

    fn is_sim_harness(&self) -> bool {
        false
    }
}

fn skip_apply(inj: LifecycleInject) -> bool {
    matches!(
        inj.boundary,
        WriteLifecycleBoundary::BeforePacketConstruction
            | WriteLifecycleBoundary::AfterPacketConstruction
            | WriteLifecycleBoundary::BeforeSerialWrite
            | WriteLifecycleBoundary::DuringSerialWrite
            | WriteLifecycleBoundary::BeforeDeviceApply
    )
}

fn unknown_after_apply(inj: LifecycleInject) -> bool {
    matches!(
        inj.loss,
        LifecycleLoss::AckLoss | LifecycleLoss::ProcessCrash | LifecycleLoss::SerialLoss
    ) && matches!(
        inj.boundary,
        WriteLifecycleBoundary::AfterDeviceApply
            | WriteLifecycleBoundary::BeforeStatusCreation
            | WriteLifecycleBoundary::AfterStatusCreation
            | WriteLifecycleBoundary::BeforeHostRead
            | WriteLifecycleBoundary::DuringHostRead
            | WriteLifecycleBoundary::AfterHostRead
            | WriteLifecycleBoundary::AfterSerialWrite
            | WriteLifecycleBoundary::BeforeLedgerAppend
            | WriteLifecycleBoundary::AfterLedgerAppend
            | WriteLifecycleBoundary::BeforeFsync
            | WriteLifecycleBoundary::AfterFsync
            | WriteLifecycleBoundary::BeforeCallerAck
    )
}
