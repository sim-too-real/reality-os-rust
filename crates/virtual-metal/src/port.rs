//! HardwareDriverPort that speaks Protocol 2.0 to [`crate::VirtualXl330`].

use std::cell::RefCell;
use std::rc::Rc;

use realityos_metal::protocol::{
    decode_status, encode_ping, encode_read, encode_write, instruction_ok, le_u16,
    ADDR_GOAL_POSITION, ADDR_MAX_VOLTAGE_LIMIT, ADDR_MIN_VOLTAGE_LIMIT, ADDR_PRESENT_POSITION,
    ADDR_PRESENT_VOLTAGE, ADDR_TORQUE_ENABLE,
};
use realityos_plant::{
    ActionParams, HardwareDriverPort, HardwareIdentity, PlantError, PlantRealized, PlantResult,
    SensorPacket,
};

use crate::device::VirtualXl330;
use crate::evidence::EVIDENCE_STATUS;

pub const DEFAULT_SERIAL: &str = "VM-XL330-M288-1";
pub const DEFAULT_CAL: &str = "VM-CAL-1";
pub const DEFAULT_DESIGN: &str = "vm-design-xl330-m288";
pub const DEFAULT_ACTUATOR: &str = "xl330:1";

#[derive(Clone)]
pub struct VirtualMetalPort {
    device: Rc<RefCell<VirtualXl330>>,
    serial: String,
    calibration_id: String,
    design: String,
    connected: bool,
    estop: bool,
    vin_fault: bool,
    tau_max: f64,
    max_position_delta_ticks: i32,
}

impl VirtualMetalPort {
    pub fn new(device: Rc<RefCell<VirtualXl330>>) -> Self {
        Self {
            device,
            serial: DEFAULT_SERIAL.into(),
            calibration_id: DEFAULT_CAL.into(),
            design: DEFAULT_DESIGN.into(),
            connected: true,
            estop: false,
            vin_fault: false,
            tau_max: 5.0,
            max_position_delta_ticks: 32,
        }
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

    fn read_u16_reg(d: &mut VirtualXl330, id: u8, addr: u16) -> PlantResult<u16> {
        let reply = d.process(&encode_read(id, addr, 2));
        let st = decode_status(&reply).map_err(|e| PlantError::refused(e.to_string()))?;
        if !instruction_ok(st.error) {
            return Err(PlantError::refused(format!(
                "dxl_status_error:{}",
                st.error
            )));
        }
        le_u16(&st.params).ok_or_else(|| PlantError::refused("dxl_short_register"))
    }

    pub fn device(&self) -> Rc<RefCell<VirtualXl330>> {
        self.device.clone()
    }

    pub fn runtime_identity_fields(&self) -> (String, String, String, String) {
        let fw = self.device.borrow().firmware_id_string();
        (
            self.serial.clone(),
            fw,
            self.calibration_id.clone(),
            self.design.clone(),
        )
    }

    pub fn physical_actions(&self) -> u64 {
        self.device.borrow().physical_action_count()
    }
}

impl HardwareDriverPort for VirtualMetalPort {
    fn probe_identity(&self) -> HardwareIdentity {
        let mut d = self.device.borrow_mut();
        let id = d.id();
        let reply = d.process(&encode_ping(id));
        let _ = decode_status(&reply);
        HardwareIdentity {
            serial: self.serial.clone(),
            firmware_id: d.firmware_id_string(),
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
        let (pos, volt, min_v, max_v) = {
            let mut d = self.device.borrow_mut();
            let id = d.id();
            let reply = d.process(&encode_read(id, ADDR_PRESENT_POSITION, 4));
            let st = decode_status(&reply).map_err(|e| PlantError::refused(e.to_string()))?;
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
            let volt = Self::read_u16_reg(&mut d, id, ADDR_PRESENT_VOLTAGE)?;
            let min_v = Self::read_u16_reg(&mut d, id, ADDR_MIN_VOLTAGE_LIMIT)?;
            let max_v = Self::read_u16_reg(&mut d, id, ADDR_MAX_VOLTAGE_LIMIT)?;
            (pos, volt, min_v, max_v)
        };
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
        let ticks = self.ticks_from_action(action);
        let mut d = self.device.borrow_mut();
        let id = d.id();
        if !d.torque_enabled() {
            let on = d.process(&encode_write(id, ADDR_TORQUE_ENABLE, &[1]));
            let _ = decode_status(&on);
        }
        let present = d.present_position();
        let goal = if ticks == 0 {
            present
        } else {
            present.saturating_add(ticks)
        };
        let reply = d.process(&encode_write(id, ADDR_GOAL_POSITION, &goal.to_le_bytes()));
        if reply.is_empty() {
            return Err(PlantError::UnknownOutcome);
        }
        let st = decode_status(&reply).map_err(|_| PlantError::UnknownOutcome)?;
        if !instruction_ok(st.error) {
            return Err(PlantError::refused(format!(
                "dxl_status_error:{}",
                st.error
            )));
        }
        Ok(PlantRealized::sim([
            ("present_position".into(), d.present_position() as f64),
            ("goal_position".into(), goal as f64),
        ]))
    }

    fn engage_hw_estop(&mut self, _reason: &str) {
        self.estop = true;
        let id = self.device.borrow().id();
        self.device
            .borrow_mut()
            .process(&encode_write(id, ADDR_TORQUE_ENABLE, &[0]));
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
