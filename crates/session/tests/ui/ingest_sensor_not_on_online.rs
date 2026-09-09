use realityos_governor::OnlineLocked;
use realityos_plant::SimPlant;
use realityos_session::RuntimeSession;

fn assert_no_caller_sensor_time(sess: &mut RuntimeSession<SimPlant, OnlineLocked>) {
    let _ = sess.ingest_sensor(&[("q0".into(), 0.0)], Some(99.0), 1.0);
}

fn main() {}
