use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RuntimeTrace {
    pub ok: bool,
    pub event: String,
    pub now_s: f64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub command_id: Option<String>,
    #[serde(default)]
    pub violations: Vec<String>,
    pub metal: bool,
}

impl RuntimeTrace {
    pub fn new(ok: bool, event: impl Into<String>, now_s: f64) -> Self {
        Self {
            ok,
            event: event.into(),
            now_s,
            command_id: None,
            violations: Vec::new(),
            metal: false,
        }
    }

    pub fn with_command(mut self, id: impl Into<String>) -> Self {
        self.command_id = Some(id.into());
        self
    }

    pub fn with_violations(mut self, v: Vec<String>) -> Self {
        self.violations = v;
        self
    }
}
