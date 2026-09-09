use realityos_governor::{OnlineLocked, RuntimeGovernor};
use realityos_plant::SimPlant;

fn assert_no_config_mut(g: &mut RuntimeGovernor<SimPlant, OnlineLocked>) {
    let _ = g.config_mut();
}

fn main() {}
