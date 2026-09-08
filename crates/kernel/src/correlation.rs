//! Correlation IDs for decide→gate→write traces.

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct CorrelationId(String);

impl CorrelationId {
    pub fn new(raw: impl Into<String>) -> Self {
        let s = raw.into();
        Self(if s.trim().is_empty() {
            "corr-unknown".into()
        } else {
            s
        })
    }

    pub fn from_command(command_id: &str, sequence: i64) -> Self {
        Self(format!("{command_id}#{sequence}"))
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl std::fmt::Display for CorrelationId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}
