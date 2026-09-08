//! HardwareBackedPlant: Plant adapter over HardwareDriverPort.
//! Governor never imports a robot SDK.

use crate::caps::{ActionParams, PlantCaps, PlantRealized};
use crate::error::{PlantError, PlantResult};
use crate::traits::{HardwareDriverPort, HardwareIdentity, Plant};
use crate::write_guard::refuse_uncertified_online_write;

pub struct HardwareBackedPlant<P: HardwareDriverPort> {
    pub port: P,
    plant_id: String,
    online: bool,
    action_dim: usize,
    max_action: Vec<f64>,
    estop: bool,
    last_sense: PlantRealized,
    last_identity: Option<HardwareIdentity>,
}

impl<P: HardwareDriverPort> HardwareBackedPlant<P> {
    pub fn new(port: P, plant_id: impl Into<String>, action_dim: usize, max_action: f64) -> Self {
        Self {
            port,
            plant_id: plant_id.into(),
            online: true,
            action_dim: action_dim.max(1),
            max_action: vec![max_action; action_dim.max(1)],
            estop: false,
            last_sense: PlantRealized::sim([]),
            last_identity: None,
        }
    }

    pub fn probe_identity(&mut self) -> HardwareIdentity {
        let id = self.port.probe_identity();
        self.last_identity = Some(id.clone());
        id
    }

    fn effective_metal(&self) -> bool {
        if self.port.is_sim_harness() {
            return false;
        }
        self.last_identity
            .as_ref()
            .map(|i| i.metal && !i.evidence_status.starts_with("SIM_"))
            .unwrap_or(false)
    }
}

impl<P: HardwareDriverPort> Plant for HardwareBackedPlant<P> {
    fn caps(&self) -> PlantCaps {
        let mut c = PlantCaps::sim(&self.plant_id, self.action_dim, self.max_action[0]);
        c.kind = "hardware".into();
        c.online = self.online && self.port.is_connected();
        c.has_driver = self.port.is_connected();
        c.sim_backend = "hardware_driver".into();
        c.max_action = self.max_action.clone();
        let _ = self.effective_metal();
        c
    }

    fn is_online(&self) -> bool {
        self.online
    }

    fn act(&mut self, action: &[f64], params: &ActionParams) -> PlantResult<PlantRealized> {
        refuse_uncertified_online_write(self, "act")?;
        if self.estop {
            return Err(PlantError::EstopEngaged);
        }
        if !self.port.is_connected() {
            return Err(PlantError::Disconnected);
        }
        let mut realized = self.port.write_action(action, params)?;
        realized.metal = self.effective_metal();
        self.last_sense = realized.clone();
        Ok(realized)
    }

    fn sense(&self) -> PlantRealized {
        self.last_sense.clone()
    }

    fn engage_estop(&mut self, reason: &str) {
        self.estop = true;
        self.port.engage_hw_estop(reason);
    }

    fn clear_estop(&mut self, operator_ack: bool) -> PlantResult<()> {
        if self.online && !operator_ack {
            return Err(PlantError::OperatorAckRequired);
        }
        self.port.clear_hw_estop(operator_ack)?;
        self.estop = false;
        Ok(())
    }
}
