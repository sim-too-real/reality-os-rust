use std::sync::{Arc, Mutex};

use crate::caps::{check_hard_action_bounds, ActionParams, PlantCaps, PlantRealized};
use crate::error::{PlantError, PlantResult};
use crate::signing::signing_key_hash;
use crate::traits::{HardwareIdentity, Plant};
use crate::write_guard::refuse_uncertified_online_write;

/// In-process plant. ONLINE flag enables certified-write uniqueness.
pub struct SimPlant {
    caps: PlantCaps,
    online: bool,
    estop: bool,
    last_action: Vec<f64>,
    writes: u32,
    backend: Option<crate::dynamics::BoxBackend>,
    production: bool,
    production_key_hash: Option<String>,
    measured: Option<Arc<Mutex<HardwareIdentity>>>,
    measured_connected: bool,
}

impl SimPlant {
    pub fn new(plant_id: impl Into<String>, action_dim: usize, max_action: f64) -> Self {
        Self {
            caps: PlantCaps::sim(plant_id, action_dim, max_action),
            online: false,
            estop: false,
            last_action: vec![0.0; action_dim.max(1)],
            writes: 0,
            backend: None,
            production: false,
            production_key_hash: None,
            measured: None,
            measured_connected: false,
        }
    }

    pub fn hardware_stub(plant_id: impl Into<String>, action_dim: usize) -> Self {
        let mut p = Self::new(plant_id, action_dim, 1.0);
        p.caps = PlantCaps::hardware_stub(p.caps.plant_id.clone(), action_dim);
        p
    }

    pub fn go_online(&mut self) {
        self.online = true;
        self.caps.online = true;
        self.caps.has_driver = true;
        if self.caps.kind == "hardware_stub" {
            self.caps.kind = "hardware_stub_online".into();
        }
    }

    pub fn last_action(&self) -> &[f64] {
        &self.last_action
    }

    pub fn attach_backend(&mut self, backend: crate::dynamics::BoxBackend) {
        self.backend = Some(backend);
    }

    /// Test/HIL attachment: what `probe_identity` will report. Not a config overwrite.
    /// The returned handle can mutate the probe after the plant is locked in ONLINE.
    pub fn bind_measured_identity(&mut self, id: HardwareIdentity) -> Arc<Mutex<HardwareIdentity>> {
        self.measured_connected = id.connected;
        let handle = Arc::new(Mutex::new(id));
        self.measured = Some(handle.clone());
        handle
    }

    pub fn set_measured_connected(&mut self, connected: bool) {
        self.measured_connected = connected;
        if let Some(id) = &self.measured {
            if let Ok(mut g) = id.lock() {
                g.connected = connected;
            }
        }
    }

    pub fn replace_measured_identity(&mut self, id: HardwareIdentity) {
        let _ = self.bind_measured_identity(id);
    }
}

impl Plant for SimPlant {
    fn caps(&self) -> PlantCaps {
        self.caps.clone()
    }

    fn is_online(&self) -> bool {
        self.online
    }

    fn act(&mut self, action: &[f64], _params: &ActionParams) -> PlantResult<PlantRealized> {
        refuse_uncertified_online_write(self, "act")?;
        if self.estop {
            return Err(PlantError::EstopEngaged);
        }
        check_hard_action_bounds(action, &self.caps())?;
        self.last_action = action.to_vec();
        self.writes += 1;
        if let Some(backend) = &mut self.backend {
            let stepped = backend.step(&self.last_action, &[], action, 0.001);
            self.last_action = stepped.q;
        }
        let peak = action.iter().fold(0.0_f64, |a, b| a.max(b.abs()));
        Ok(PlantRealized::sim([
            ("stop_x".into(), peak),
            ("n".into(), action.len() as f64),
            (
                "backend".into(),
                f64::from(u8::from(self.backend.is_some())),
            ),
        ]))
    }

    fn sense(&self) -> PlantRealized {
        PlantRealized::sim([("estop".into(), f64::from(u8::from(self.estop)))])
    }

    fn engage_estop(&mut self, _reason: &str) {
        self.estop = true;
    }

    fn clear_estop(&mut self, operator_ack: bool) -> PlantResult<()> {
        if self.online && !operator_ack {
            return Err(PlantError::OperatorAckRequired);
        }
        self.estop = false;
        Ok(())
    }

    fn probe_identity(&mut self) -> Option<HardwareIdentity> {
        let guard = self.measured.as_ref()?.lock().ok()?;
        let mut id = guard.clone();
        id.connected = self.measured_connected && id.connected && !self.estop;
        Some(id)
    }

    fn write_count(&self) -> u32 {
        self.writes
    }

    fn lock_production(&mut self, signing_key: &[u8]) {
        self.go_online();
        self.production = true;
        self.production_key_hash = Some(signing_key_hash(signing_key));
    }

    fn production_locked(&self) -> bool {
        self.production
    }

    fn production_key_hash(&self) -> Option<&str> {
        self.production_key_hash.as_deref()
    }

    fn follow_waypoints(&mut self, waypoints: &[Vec<f64>]) -> PlantResult<PlantRealized> {
        refuse_uncertified_online_write(self, "follow_waypoints")?;
        if self.estop {
            return Err(PlantError::EstopEngaged);
        }
        self.writes += 1;
        Ok(PlantRealized::sim([
            ("n_waypoints".into(), waypoints.len() as f64),
            ("ok".into(), 1.0),
        ]))
    }
}
