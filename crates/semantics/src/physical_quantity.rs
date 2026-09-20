//! Command-domain quantities are not physical effort.

use serde::{Deserialize, Serialize};

use crate::provenance::Provenance;

/// Planner-facing physical-fact provenance. Distinct from silent defaults.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum PhysicalFactProvenance {
    Declared,
    Measured,
    SimulatorDerived,
    Inferred,
    Estimated,
    Unknown,
}

impl From<Provenance> for PhysicalFactProvenance {
    fn from(p: Provenance) -> Self {
        match p {
            Provenance::ModelDeclared | Provenance::UserDeclared => Self::Declared,
            Provenance::CalibrationMeasured | Provenance::HardwareMeasured => Self::Measured,
            Provenance::SimulatorDerived => Self::SimulatorDerived,
            Provenance::Assumed => Self::Inferred,
            Provenance::LearnedEstimate => Self::Estimated,
            Provenance::Unknown => Self::Unknown,
        }
    }
}

/// Physical meaning of a numeric bound. These are not interchangeable.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum EffortDomain {
    CommandRange,
    ForceRange,
    JointTorqueLimit,
}

/// A justified joint-effort magnitude, or an explicit refusal to invent one.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum PhysicalEffort {
    JointTorque {
        tau_abs_nm: f64,
        domain: EffortDomain,
        provenance: PhysicalFactProvenance,
        source: String,
    },
    Unknown {
        reason: String,
    },
}

impl PhysicalEffort {
    pub fn tau_abs_nm(&self) -> Option<f64> {
        match self {
            Self::JointTorque { tau_abs_nm, .. } => Some(*tau_abs_nm),
            Self::Unknown { .. } => None,
        }
    }

    pub fn is_unknown(&self) -> bool {
        matches!(self, Self::Unknown { .. })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unknown_provenance_is_not_declared() {
        assert_eq!(
            PhysicalFactProvenance::from(Provenance::Unknown),
            PhysicalFactProvenance::Unknown
        );
        assert_eq!(
            PhysicalFactProvenance::from(Provenance::UserDeclared),
            PhysicalFactProvenance::Declared
        );
    }
}
