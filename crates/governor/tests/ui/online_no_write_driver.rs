use realityos_governor::{OnlineLocked, RuntimeGovernor};
use realityos_plant::{ActionParams, ActuationCommand, SimPlant};

fn assert_no_write_driver(
    g: &mut RuntimeGovernor<SimPlant, OnlineLocked>,
    cmd: &dyn ActuationCommand,
) {
    let _ = g.write_driver(cmd, &ActionParams::empty(), 1.0);
}

fn main() {}
