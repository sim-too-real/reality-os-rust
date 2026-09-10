//! SimulationDriverPort: HardwareDriverPort with metal=false, SIMULATION_ONLY.
//! No simulator-only bypass around the authority gate.

use crate::honesty::SIMULATION_ONLY;
use crate::mujoco_exec::{json_f64_vec, MujocoInstance};
use realityos_plant::{
    ActionParams, HardwareDriverPort, HardwareIdentity, PlantRealized, PlantResult, SensorPacket,
};
use serde_json::Value;
use std::sync::{Arc, Mutex};

#[derive(Debug, Clone, Default)]
pub struct SimActuationState {
    pub ctrl: Vec<f64>,
    pub write_count: u64,
    pub policy_ctrl_writes: u64,
    pub authority_safe_state_writes: u64,
    pub last_authorized_ctrl: Vec<f64>,
}

#[derive(Clone, Default)]
pub struct SimActuationProbe {
    inner: Arc<Mutex<SimActuationState>>,
}

impl SimActuationProbe {
    pub fn snapshot(&self) -> SimActuationState {
        self.inner.lock().map(|g| g.clone()).unwrap_or_default()
    }
}

/// Dedicated simulation plant port. Identifies as metal=false / SIMULATION_ONLY.
pub struct SimulationDriverPort {
    pub inst: MujocoInstance,
    probe: SimActuationProbe,
    connected: bool,
    estop: bool,
    seq: u64,
    last_state: Value,
    robot_id: String,
    model_hash: String,
}

impl SimulationDriverPort {
    pub fn new(
        inst: MujocoInstance,
        robot_id: impl Into<String>,
        model_hash: impl Into<String>,
    ) -> Self {
        let nu = inst.inspect["nu"].as_u64().unwrap_or(0) as usize;
        let probe = SimActuationProbe::default();
        if let Ok(mut g) = probe.inner.lock() {
            g.ctrl = vec![0.0; nu];
        }
        Self {
            inst,
            probe,
            connected: true,
            estop: false,
            seq: 0,
            last_state: Value::Null,
            robot_id: robot_id.into(),
            model_hash: model_hash.into(),
        }
    }

    pub fn probe_handle(&self) -> SimActuationProbe {
        self.probe.clone()
    }

    pub fn last_state(&self) -> &Value {
        &self.last_state
    }

    pub fn peek_ctrl(&mut self) -> Result<(Vec<f64>, u64), String> {
        self.inst.peek_ctrl().map_err(|e| e.to_string())
    }

    pub fn step_physics(&mut self, n: u32) -> Result<Value, String> {
        let r = self.inst.step(n).map_err(|e| e.to_string())?;
        self.last_state = r.get("state").cloned().unwrap_or(r);
        Ok(self.last_state.clone())
    }

    pub fn apply_force(&mut self, body: &str, force: [f64; 3]) -> Result<(), String> {
        self.inst
            .apply_force(body, force)
            .map_err(|e| e.to_string())
    }

    pub fn clear_forces(&mut self) -> Result<(), String> {
        self.inst.clear_forces().map_err(|e| e.to_string())
    }

    pub fn reset(&mut self, qpos: Option<&[f64]>, qvel: Option<&[f64]>) -> Result<Value, String> {
        let r = self.inst.reset(qpos, qvel).map_err(|e| e.to_string())?;
        self.last_state = r.get("state").cloned().unwrap_or(r);
        Ok(self.last_state.clone())
    }

    pub fn identity_for(&self) -> HardwareIdentity {
        HardwareIdentity {
            serial: format!("SIM_{}", self.robot_id),
            firmware_id: "SIM_MUJOCO_WORKER".into(),
            calibration_id: "SIM_CAL".into(),
            design_content_hash: self.model_hash.clone(),
            connected: self.connected,
            metal: false,
            evidence_status: SIMULATION_ONLY.into(),
            actuator_ids: Vec::new(),
        }
    }
}

impl HardwareDriverPort for SimulationDriverPort {
    fn probe_identity(&self) -> HardwareIdentity {
        self.identity_for()
    }

    fn read_sensor(&mut self, now_s: f64) -> PlantResult<SensorPacket> {
        self.seq += 1;
        let qpos = json_f64_vec(&self.last_state["qpos"]);
        let mut samples = vec![("sim_t".into(), now_s)];
        for (i, q) in qpos.iter().enumerate() {
            samples.push((format!("q{i}"), *q));
        }
        let mut pkt = SensorPacket::from_samples(samples, now_s);
        pkt.sequence = self.seq;
        pkt.sensor_id = format!("sim/{}", self.robot_id);
        pkt.frame_id = "sim/privileged_not_policy".into();
        Ok(pkt)
    }

    fn write_action(
        &mut self,
        action: &[f64],
        _params: &ActionParams,
    ) -> PlantResult<PlantRealized> {
        if self.estop {
            return Err(realityos_plant::PlantError::EstopEngaged);
        }
        if !self.connected {
            return Err(realityos_plant::PlantError::Disconnected);
        }
        self.inst
            .set_ctrl(action)
            .map_err(|e| realityos_plant::PlantError::refused(e.to_string()))?;
        if let Ok(mut g) = self.probe.inner.lock() {
            g.ctrl = action.to_vec();
            g.last_authorized_ctrl = action.to_vec();
            g.write_count = g.write_count.saturating_add(1);
            g.policy_ctrl_writes = g.policy_ctrl_writes.saturating_add(1);
        }
        Ok(PlantRealized::sim([
            ("n".into(), action.len() as f64),
            ("metal".into(), 0.0),
        ]))
    }

    fn engage_hw_estop(&mut self, _reason: &str) {
        self.estop = true;
    }

    fn clear_hw_estop(&mut self, operator_ack: bool) -> PlantResult<()> {
        if !operator_ack {
            return Err(realityos_plant::PlantError::OperatorAckRequired);
        }
        self.estop = false;
        Ok(())
    }

    fn is_connected(&self) -> bool {
        self.connected
    }

    fn is_sim_harness(&self) -> bool {
        true
    }
}

/// In-process recording port for authority tests that do not need physics.
pub struct RecordingSimPort {
    pub probe: SimActuationProbe,
    nu: usize,
    connected: bool,
}

impl RecordingSimPort {
    pub fn new(nu: usize) -> Self {
        Self {
            probe: SimActuationProbe::default(),
            nu: nu.max(1),
            connected: true,
        }
    }
}

/// Shared isolated MuJoCo instance: authority writes ctrl; the runner may only step/observe.
pub struct SharedMujoco {
    pub inst: Mutex<MujocoInstance>,
    pub probe: SimActuationProbe,
    pub robot_id: String,
    pub model_hash: String,
}

pub struct SharedSimPort {
    shared: Arc<SharedMujoco>,
}

impl SharedSimPort {
    pub fn new(shared: Arc<SharedMujoco>) -> Self {
        Self { shared }
    }
}

impl SharedMujoco {
    pub fn write_safe_ctrl(&self, action: &[f64]) -> Result<(), String> {
        let mut inst = self.inst.lock().map_err(|e| e.to_string())?;
        inst.set_safe_ctrl(action).map_err(|e| e.to_string())?;
        if let Ok(mut g) = self.probe.inner.lock() {
            g.ctrl = action.to_vec();
            g.authority_safe_state_writes = g.authority_safe_state_writes.saturating_add(1);
            g.write_count = g.write_count.saturating_add(1);
        }
        Ok(())
    }
}

impl HardwareDriverPort for SharedSimPort {
    fn probe_identity(&self) -> HardwareIdentity {
        HardwareIdentity {
            serial: format!("SIM_{}", self.shared.robot_id),
            firmware_id: "SIM_MUJOCO_WORKER".into(),
            calibration_id: "SIM_CAL".into(),
            design_content_hash: self.shared.model_hash.clone(),
            connected: true,
            metal: false,
            evidence_status: SIMULATION_ONLY.into(),
            actuator_ids: Vec::new(),
        }
    }

    fn read_sensor(&mut self, now_s: f64) -> PlantResult<SensorPacket> {
        Ok(SensorPacket::from_samples(vec![("t".into(), now_s)], now_s))
    }

    fn write_action(
        &mut self,
        action: &[f64],
        _params: &ActionParams,
    ) -> PlantResult<PlantRealized> {
        let mut inst = self
            .shared
            .inst
            .lock()
            .map_err(|_| realityos_plant::PlantError::refused("lock"))?;
        inst.set_ctrl(action)
            .map_err(|e| realityos_plant::PlantError::refused(e.to_string()))?;
        if let Ok(mut g) = self.shared.probe.inner.lock() {
            g.ctrl = action.to_vec();
            g.last_authorized_ctrl = action.to_vec();
            g.write_count = g.write_count.saturating_add(1);
            g.policy_ctrl_writes = g.policy_ctrl_writes.saturating_add(1);
        }
        Ok(PlantRealized::sim([("n".into(), action.len() as f64)]))
    }

    fn engage_hw_estop(&mut self, _reason: &str) {}
    fn clear_hw_estop(&mut self, _operator_ack: bool) -> PlantResult<()> {
        Ok(())
    }
    fn is_connected(&self) -> bool {
        true
    }
    fn is_sim_harness(&self) -> bool {
        true
    }
}

impl HardwareDriverPort for RecordingSimPort {
    fn probe_identity(&self) -> HardwareIdentity {
        HardwareIdentity {
            serial: "SIM_RECORDING".into(),
            firmware_id: "SIM_FW".into(),
            calibration_id: "SIM_CAL".into(),
            design_content_hash: "recording".into(),
            connected: self.connected,
            metal: false,
            evidence_status: SIMULATION_ONLY.into(),
            actuator_ids: Vec::new(),
        }
    }

    fn read_sensor(&mut self, now_s: f64) -> PlantResult<SensorPacket> {
        Ok(SensorPacket::from_samples(vec![("t".into(), now_s)], now_s))
    }

    fn write_action(
        &mut self,
        action: &[f64],
        _params: &ActionParams,
    ) -> PlantResult<PlantRealized> {
        if let Ok(mut g) = self.probe.inner.lock() {
            g.ctrl = action.to_vec();
            g.last_authorized_ctrl = action.to_vec();
            g.write_count += 1;
            g.policy_ctrl_writes += 1;
        }
        let _ = self.nu;
        Ok(PlantRealized::sim([("n".into(), action.len() as f64)]))
    }

    fn engage_hw_estop(&mut self, _reason: &str) {}
    fn clear_hw_estop(&mut self, _operator_ack: bool) -> PlantResult<()> {
        Ok(())
    }
    fn is_connected(&self) -> bool {
        self.connected
    }
    fn is_sim_harness(&self) -> bool {
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn port_identity_is_simulation_only() {
        let p = RecordingSimPort::new(2);
        let id = p.probe_identity();
        assert!(!id.metal);
        assert_eq!(id.evidence_status, SIMULATION_ONLY);
        assert!(p.is_sim_harness());
    }
}
