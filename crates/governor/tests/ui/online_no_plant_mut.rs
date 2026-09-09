use realityos_governor::{OnlineLocked, RuntimeGovernor};
use realityos_plant::SimPlant;

fn assert_no_plant_mut(g: &mut RuntimeGovernor<SimPlant, OnlineLocked>) {
    let _ = g.plant_mut();
}

fn main() {}
