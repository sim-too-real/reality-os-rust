use realityos_core::IssuedCommand;
use realityos_governor::OnlineLocked;
use realityos_plant::{ActionParams, SimPlant};
use realityos_session::RuntimeSession;

fn assert_no_caller_dispatch_time(
    sess: &mut RuntimeSession<SimPlant, OnlineLocked>,
    cmd: IssuedCommand,
) {
    let _ = sess.dispatch_issued(cmd, &ActionParams::empty(), 10.0);
}

fn main() {}
