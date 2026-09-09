use realityos_governor::{OnlineLocked, RuntimeGovernor};
use realityos_plant::SimPlant;

fn assert_no_key_replace(g: &mut RuntimeGovernor<SimPlant, OnlineLocked>) {
    g.set_signing_key(None);
}

fn main() {}
