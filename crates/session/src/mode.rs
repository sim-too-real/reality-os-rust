use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum RuntimeMode {
    Simulation,
    Hil,
    Online,
}

impl RuntimeMode {
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Simulation => "SIMULATION",
            Self::Hil => "HIL",
            Self::Online => "ONLINE",
        }
    }

    pub fn parse(raw: &str) -> Result<Self, String> {
        match raw.trim().to_ascii_uppercase().as_str() {
            "SIMULATION" | "SIM" => Ok(Self::Simulation),
            "HIL" => Ok(Self::Hil),
            "ONLINE" => Ok(Self::Online),
            other => Err(format!("unknown_runtime_mode:{other}")),
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SessionStartError(pub String);

impl std::fmt::Display for SessionStartError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for SessionStartError {}
