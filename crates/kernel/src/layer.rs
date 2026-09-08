//! Stack layers. Every refuse names which layer produced it.

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Layer {
    Kernel,
    Physics,
    RealityOs,
    Governor,
    Plant,
    Session,
    Bridge,
    Fieldbus,
    Adapter,
}

impl Layer {
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Kernel => "kernel",
            Self::Physics => "physics",
            Self::RealityOs => "reality_os",
            Self::Governor => "governor",
            Self::Plant => "plant",
            Self::Session => "session",
            Self::Bridge => "bridge",
            Self::Fieldbus => "fieldbus",
            Self::Adapter => "adapter",
        }
    }
}

impl std::fmt::Display for Layer {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.as_str())
    }
}
