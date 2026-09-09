use realityos_governor::{OnlineLocked, RuntimeGovernor};
use realityos_plant::SimPlant;

fn assert_online_rejects_caller_time(g: &mut RuntimeGovernor<SimPlant, OnlineLocked>) {
    let _ = g.heartbeat(1.0);
    let _ = g.watchdog_tick(1.0);
    let _ = g.engage_estop("x", 1.0);
    let _ = g.clear_estop_requires_recovery(true, 1.0);
}

fn main() {}
