//! In-process driver harness. Behaves like hardware for rail tests. Never metal.

use crate::caps::{ActionParams, PlantRealized};
use crate::error::{PlantError, PlantResult};
use crate::traits::{HardwareDriverPort, HardwareIdentity, SensorPacket, HARNESS_EVIDENCE};

#[derive(Debug)]
pub struct SimulatedHardwarePort {
    pub identity: HardwareIdentity,
    connected: bool,
    estop: bool,
    seq: u64,
    last: Vec<(String, f64)>,
}

impl SimulatedHardwarePort {
    pub fn new(serial: impl Into<String>) -> Self {
        Self {
            identity: HardwareIdentity::harness(serial),
            connected: true,
            estop: false,
            seq: 0,
            last: vec![("joint_pos_rad".into(), 0.0), ("force_n".into(), 0.0)],
        }
    }

    pub fn disconnect(&mut self) {
        self.connected = false;
    }
}

impl HardwareDriverPort for SimulatedHardwarePort {
    fn probe_identity(&self) -> HardwareIdentity {
        let mut id = self.identity.clone();
        id.metal = false;
        id.evidence_status = HARNESS_EVIDENCE.into();
        id.connected = self.connected && !self.estop;
        id
    }

    fn read_sensor(&mut self, now_s: f64) -> PlantResult<SensorPacket> {
        if !self.connected {
            return Err(PlantError::Disconnected);
        }
        self.seq += 1;
        let mut pkt = SensorPacket::from_samples(self.last.clone(), now_s);
        pkt.sequence = self.seq;
        pkt.frame_id = "harness/sensor".into();
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
        if !self.connected {
            return Err(PlantError::Disconnected);
        }
        let peak = action.iter().copied().fold(0.0_f64, |a, b| a.max(b.abs()));
        self.last = vec![
            ("stop_x".into(), peak),
            ("joint_pos_rad".into(), peak),
            ("force_n".into(), peak),
        ];
        Ok(PlantRealized::sim(self.last.clone()))
    }

    fn engage_hw_estop(&mut self, _reason: &str) {
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
        self.connected && !self.estop
    }

    fn is_sim_harness(&self) -> bool {
        true
    }
}
