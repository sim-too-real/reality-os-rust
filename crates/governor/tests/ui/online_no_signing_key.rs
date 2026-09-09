use realityos_governor::{OnlineLocked, RuntimeGovernor};
use realityos_plant::SimPlant;

fn assert_no_key_export(g: &RuntimeGovernor<SimPlant, OnlineLocked>) {
    let _ = g.signing_key();
}

fn main() {}
