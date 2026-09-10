use crate::provenance::Provenanced;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Se3 {
    pub xyz: [f64; 3],
    pub quat_wxyz: [f64; 4],
}

impl Se3 {
    pub fn identity() -> Self {
        Self {
            xyz: [0.0, 0.0, 0.0],
            quat_wxyz: [1.0, 0.0, 0.0, 0.0],
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TransformEdge {
    pub parent: String,
    pub child: String,
    pub pose: Provenanced<Se3>,
    pub timestamp_s: f64,
    pub calibration_epoch: String,
    pub source: String,
}

impl TransformEdge {
    pub fn identity(
        parent: impl Into<String>,
        child: impl Into<String>,
        epoch: impl Into<String>,
        timestamp_s: f64,
    ) -> Self {
        let parent = parent.into();
        let child = child.into();
        let source = format!("transform.{parent}.{child}");
        Self {
            parent,
            child,
            pose: Provenanced::declared(Se3::identity(), &source, timestamp_s),
            timestamp_s,
            calibration_epoch: epoch.into(),
            source,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum TransformError {
    #[error("calibration epoch mismatch")]
    EpochMismatch,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TransformGraph {
    pub calibration_epoch: String,
    edges: Vec<TransformEdge>,
}

impl TransformGraph {
    pub fn new(epoch: impl Into<String>) -> Self {
        Self {
            calibration_epoch: epoch.into(),
            edges: Vec::new(),
        }
    }

    pub fn insert(&mut self, edge: TransformEdge) -> Result<(), TransformError> {
        if edge.calibration_epoch != self.calibration_epoch {
            return Err(TransformError::EpochMismatch);
        }
        for existing in &self.edges {
            if existing.calibration_epoch != edge.calibration_epoch {
                return Err(TransformError::EpochMismatch);
            }
        }
        self.edges.push(edge);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn refuses_to_mix_calibration_epochs() {
        let mut g = TransformGraph::new("e0");
        g.insert(TransformEdge::identity("world", "base", "e0", 1.0)).unwrap();
        let err = g
            .insert(TransformEdge::identity("base", "ee", "e1", 1.0))
            .unwrap_err();
        assert_eq!(err, TransformError::EpochMismatch);
    }
}
