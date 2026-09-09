use realityos_plant::SimPlant;
use realityos_session::{RuntimeSession, StartArgs};

fn main() {
    let plant = SimPlant::new("p", 1, 1.0);
    let args = StartArgs::simulation("rel");
    let _ = RuntimeSession::<SimPlant, realityos_governor::OnlineLocked>::start_online(
        args, plant, 1.0,
    );
}
