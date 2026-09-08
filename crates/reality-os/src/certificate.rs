use realityos_kernel::{DecisionStatus, KernelError, KernelResult, UnifiedDecision};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Certificate {
    pub status: DecisionStatus,
    pub physical_reason: String,
    #[serde(default)]
    pub reasons: Vec<String>,
    #[serde(default)]
    pub predicted_slacks: BTreeMap<String, f64>,
    #[serde(default)]
    pub margin: Option<f64>,
    #[serde(default)]
    pub source: String,
}

impl Certificate {
    pub fn new(status: DecisionStatus, physical_reason: impl Into<String>) -> Self {
        Self {
            status,
            physical_reason: physical_reason.into(),
            reasons: Vec::new(),
            predicted_slacks: BTreeMap::new(),
            margin: None,
            source: "reality_layer".into(),
        }
    }

    pub fn with_reasons(mut self, reasons: impl IntoIterator<Item = impl Into<String>>) -> Self {
        self.reasons = reasons.into_iter().map(Into::into).collect();
        self
    }

    pub fn with_margin(mut self, margin: f64) -> Self {
        self.margin = Some(margin);
        self
    }

    #[inline]
    pub const fn allowed(&self) -> bool {
        self.status.allowed()
    }

    pub fn to_unified(&self) -> UnifiedDecision {
        UnifiedDecision::new(self.status, self.physical_reason.clone()).with_source(&self.source)
    }

    pub fn require_valid(&self) -> KernelResult<()> {
        let _ = self.status;
        if self.physical_reason.trim().is_empty() && self.status.blocked() {
            return Err(KernelError::validation(
                "certificate.physical_reason",
                "blocked verdict needs a reason",
            ));
        }
        Ok(())
    }
}
