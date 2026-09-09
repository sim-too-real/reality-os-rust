use realityos_plant::{ActionParams, Plant, PlantCaps, PlantRealized, PlantResult};

struct Rogue;

impl Plant for Rogue {
    fn caps(&self) -> PlantCaps {
        PlantCaps::sim("rogue", 1, 1.0)
    }
    fn is_online(&self) -> bool {
        true
    }
    fn act(&mut self, _action: &[f64], _params: &ActionParams) -> PlantResult<PlantRealized> {
        Ok(PlantRealized::sim([]))
    }
    fn sense(&self) -> PlantRealized {
        PlantRealized::sim([])
    }
    fn engage_estop(&mut self, _reason: &str) {}
    fn clear_estop(&mut self, _operator_ack: bool) -> PlantResult<()> {
        Ok(())
    }
}

fn main() {}
