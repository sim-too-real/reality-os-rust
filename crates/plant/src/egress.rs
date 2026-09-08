//! Command egress. Default is refuse — motors stay off until a real port is wired
//! *behind* execute_certified_command.

use crate::caps::{ActionParams, PlantRealized};
use crate::error::{PlantError, PlantResult};

pub trait CommandEgress {
    fn enabled(&self) -> bool;
    fn write(&mut self, action: &[f64], params: &ActionParams) -> PlantResult<PlantRealized>;
}

/// Matches Python `RefuseCommandEgress` / ROS node kill-switch.
#[derive(Debug, Default, Clone)]
pub struct RefuseCommandEgress;

impl CommandEgress for RefuseCommandEgress {
    fn enabled(&self) -> bool {
        false
    }

    fn write(&mut self, _action: &[f64], _params: &ActionParams) -> PlantResult<PlantRealized> {
        Err(PlantError::EgressDisabled)
    }
}

/// Test/SIM recording sink. metal=false always.
#[derive(Debug, Default)]
pub struct RecordingCommandEgress {
    pub writes: Vec<Vec<f64>>,
}

impl CommandEgress for RecordingCommandEgress {
    fn enabled(&self) -> bool {
        true
    }

    fn write(&mut self, action: &[f64], _params: &ActionParams) -> PlantResult<PlantRealized> {
        if action.iter().any(|x| !x.is_finite()) {
            return Err(PlantError::refused("non_finite_allowed_action"));
        }
        self.writes.push(action.to_vec());
        let peak = action.iter().fold(0.0_f64, |a, b| a.max(b.abs()));
        Ok(PlantRealized::sim([
            ("stop_x".into(), peak),
            ("n".into(), action.len() as f64),
            ("egress".into(), 1.0),
        ]))
    }
}
