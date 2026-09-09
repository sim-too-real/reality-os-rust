//! Evidence-linked, expiring memory. Prose is never motion.

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct EvidenceTrace {
    pub id: String,
    pub evidence_digest: String,
    pub skill_id: String,
    pub expires_at_s: f64,
    pub note: String,
}

#[derive(Debug, Clone, Default)]
pub struct EvidenceMemory {
    traces: Vec<EvidenceTrace>,
}

impl EvidenceMemory {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn remember(&mut self, trace: EvidenceTrace) {
        self.traces.retain(|t| t.id != trace.id);
        self.traces.push(trace);
    }

    pub fn expire(&mut self, now_s: f64) {
        self.traces
            .retain(|t| t.expires_at_s.is_finite() && t.expires_at_s > now_s);
    }

    pub fn recall(&self, digest: &str, now_s: f64) -> Option<&EvidenceTrace> {
        self.traces.iter().find(|t| {
            t.evidence_digest == digest && t.expires_at_s.is_finite() && t.expires_at_s > now_s
        })
    }

    /// Free text cannot retrieve or authorize a skill.
    pub fn recall_prose(&self, _text: &str, _now_s: f64) -> Option<&EvidenceTrace> {
        None
    }

    pub fn len(&self) -> usize {
        self.traces.len()
    }

    pub fn is_empty(&self) -> bool {
        self.traces.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn memory_is_digest_keyed_and_expires() {
        let mut mem = EvidenceMemory::new();
        mem.remember(EvidenceTrace {
            id: "m1".into(),
            evidence_digest: "abc".into(),
            skill_id: "skill.place".into(),
            expires_at_s: 10.0,
            note: "seen".into(),
        });
        assert!(mem.recall("abc", 9.0).is_some());
        mem.expire(10.0);
        assert!(mem.recall("abc", 10.0).is_none());
        assert!(mem.recall_prose("place the cube", 1.0).is_none());
    }
}
