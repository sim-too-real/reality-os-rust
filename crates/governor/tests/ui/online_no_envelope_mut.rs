use realityos_governor::{OnlineLocked, RuntimeGovernor};
use realityos_plant::SimPlant;

fn assert_no_envelope_mut(g: &mut RuntimeGovernor<SimPlant, OnlineLocked>) {
    let _ = g.envelope_mut();
}

fn main() {}
