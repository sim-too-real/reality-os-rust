use realityos_governor::{OnlineLocked, OnlineWrite, RuntimeGovernor};
use realityos_plant::{ActionParams, SimPlant};

fn assert_write_online_caller_time_is_not_public(
    g: &mut RuntimeGovernor<SimPlant, OnlineLocked>,
    write: &OnlineWrite,
) {
    let _ = g.write_online(write, &ActionParams::empty(), 1.0);
}

fn main() {}
