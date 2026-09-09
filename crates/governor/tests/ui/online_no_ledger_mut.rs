use realityos_governor::{OnlineLocked, RuntimeGovernor};
use realityos_plant::SimPlant;

fn assert_no_ledger_mut(g: &mut RuntimeGovernor<SimPlant, OnlineLocked>) {
    let _ = g.ledger_mut();
}

fn main() {}
