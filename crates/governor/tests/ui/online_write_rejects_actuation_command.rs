use realityos_governor::{OnlineLocked, RuntimeGovernor};
use realityos_plant::{ActionParams, ActuationCommand, SimPlant};

fn assert_write_online_is_not_actuation_command(
    g: &mut RuntimeGovernor<SimPlant, OnlineLocked>,
    cmd: &dyn ActuationCommand,
) {
    let _ = g.write_online_now(cmd, &ActionParams::empty());
}

fn main() {}
