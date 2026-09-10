use crate::provenance::{Provenance, Provenanced};
use crate::transform::Se3;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ObjectHypothesis {
    pub id: String,
    pub pose: Provenanced<Se3>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WorldState {
    pub transform_epoch: String,
    pub as_of_s: f64,
    pub target_frame: String,
    pub target_xyz: Provenanced<[f64; 3]>,
    pub target_expires_at_s: f64,
    pub objects: Vec<ObjectHypothesis>,
}

impl WorldState {
    pub fn empty(transform_epoch: impl Into<String>, as_of_s: f64) -> Self {
        Self {
            transform_epoch: transform_epoch.into(),
            as_of_s,
            target_frame: String::new(),
            target_xyz: Provenanced::unknown("world.target", as_of_s),
            target_expires_at_s: as_of_s,
            objects: Vec::new(),
        }
    }

    pub fn with_target(
        mut self,
        frame: impl Into<String>,
        xyz: [f64; 3],
        expires: f64,
        epoch: impl Into<String>,
        now: f64,
        provenance: Provenance,
    ) -> Self {
        self.transform_epoch = epoch.into();
        self.target_frame = frame.into();
        self.as_of_s = now;
        self.target_xyz = Provenanced {
            value: Some(xyz),
            provenance,
            source: "world.target".into(),
            as_of_s: now,
            uncertainty: None,
        };
        self.target_expires_at_s = expires;
        self
    }

    pub fn target_fresh(&self, now: f64, freshness: f64) -> bool {
        let Some(xyz) = self.target_xyz.value else {
            return false;
        };
        if now > self.target_expires_at_s {
            return false;
        }
        if now - self.as_of_s > freshness {
            return false;
        }
        xyz.iter().all(|v| v.is_finite())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::provenance::Provenance;

    #[test]
    fn missing_target_is_unknown_not_origin() {
        let w = WorldState::empty("e0", 1.0);
        assert!(w.target_xyz.value.is_none());
        assert_eq!(w.target_xyz.provenance, Provenance::Unknown);
        assert!(!w.target_fresh(1.0, 0.25));
    }
}
