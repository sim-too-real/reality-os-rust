//! Typed, dimensioned, provenance-bearing physical parameters.
//! Declared numbers are never rewritten. New evidence is appended.

use serde::{Deserialize, Serialize};

use crate::provenance::Provenanced;
use crate::sensor::WrenchSampleOrigin;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum PhysicalParameter {
    SupportFriction,
    ToolObjectFriction,
    ObjectMassKg,
    SupportPressureModelApplicability,
    QuasiStaticApplicability,
    ContactModeConsistency,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum BeliefEpistemicStatus {
    DeclaredFact,
    MeasuredFact,
    DerivedConstraint,
    Unknown,
    Contradicted,
}

pub const DECLARED_MODEL_INCONSISTENT_WITH_OBSERVATION: &str =
    "DECLARED_MODEL_INCONSISTENT_WITH_OBSERVATION";

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BeliefLineage {
    pub belief_before: String,
    pub observation: String,
    pub inference: String,
    pub belief_after: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct BeliefEvidenceReference {
    pub measurement_digest: String,
    pub origin: WrenchSampleOrigin,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ParameterBelief {
    pub parameter: PhysicalParameter,
    pub status: BeliefEpistemicStatus,
    /// Original declared number. Stays put when observations disagree with it.
    pub declared: Provenanced<f64>,
    pub empirical_interval: Option<[f64; 2]>,
    pub lineage: Vec<BeliefLineage>,
    #[serde(default)]
    pub evidence: Vec<BeliefEvidenceReference>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum BeliefUpdateError {
    #[error("parameter is missing from the belief")]
    MissingParameter,
    #[error("parameter has no finite empirical prior interval")]
    MissingEmpiricalPrior,
    #[error("prior or evidence interval is invalid")]
    InvalidInterval,
    #[error("evidence digest is invalid")]
    InvalidEvidenceDigest,
    #[error("synthetic evidence cannot update physical friction")]
    SyntheticEvidence,
    #[error("hardware evidence requires provenance from a trusted device adapter")]
    HardwareOriginUntrusted,
    #[error("measurement evidence was already applied")]
    DuplicateEvidence,
    #[error("contradicted belief cannot be narrowed by this update")]
    ContradictedBelief,
    #[error("evidence interval does not intersect the empirical prior")]
    EmptyIntersection,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PhysicalParameterBelief {
    pub parameters: Vec<ParameterBelief>,
}

impl PhysicalParameterBelief {
    pub fn declared_point(
        parameter: PhysicalParameter,
        value: f64,
        source: impl Into<String>,
    ) -> Self {
        Self {
            parameters: vec![ParameterBelief {
                parameter,
                status: BeliefEpistemicStatus::DeclaredFact,
                declared: Provenanced::declared(value, source, 0.0),
                empirical_interval: None,
                lineage: Vec::new(),
                evidence: Vec::new(),
            }],
        }
    }

    pub fn with_unknown(mut self, parameter: PhysicalParameter, source: impl Into<String>) -> Self {
        self.parameters.push(ParameterBelief {
            parameter,
            status: BeliefEpistemicStatus::Unknown,
            declared: Provenanced::unknown(source, 0.0),
            empirical_interval: None,
            lineage: Vec::new(),
            evidence: Vec::new(),
        });
        self
    }

    pub fn entry(&self, parameter: PhysicalParameter) -> Option<&ParameterBelief> {
        self.parameters.iter().find(|p| p.parameter == parameter)
    }

    pub fn entry_mut(&mut self, parameter: PhysicalParameter) -> Option<&mut ParameterBelief> {
        self.parameters
            .iter_mut()
            .find(|p| p.parameter == parameter)
    }

    pub fn declared_value(&self, parameter: PhysicalParameter) -> Option<f64> {
        self.entry(parameter)?.declared.value
    }

    /// Record that observations contradict the model that used the declared value.
    /// The declared number and its provenance source stay as they were.
    pub fn contradict_declared(&mut self, parameter: PhysicalParameter, observation: &str) {
        let Some(entry) = self.entry_mut(parameter) else {
            return;
        };
        let before = format!(
            "{parameter:?} status={:?} declared={:?} source={}",
            entry.status, entry.declared.value, entry.declared.source
        );
        let declared_value = entry.declared.value;
        let source = entry.declared.source.clone();
        let provenance = entry.declared.provenance;
        entry.status = BeliefEpistemicStatus::Contradicted;
        entry.declared.value = declared_value;
        entry.declared.source = source;
        entry.declared.provenance = provenance;
        let after = format!(
            "{parameter:?} status={:?} declared={:?} source={}",
            entry.status, entry.declared.value, entry.declared.source
        );
        entry.lineage.push(BeliefLineage {
            belief_before: before,
            observation: observation.to_string(),
            inference: DECLARED_MODEL_INCONSISTENT_WITH_OBSERVATION.to_string(),
            belief_after: after,
        });
    }

    /// Narrow an empirical interval. The declared number is left untouched.
    pub fn narrow_interval(
        &mut self,
        parameter: PhysicalParameter,
        interval: [f64; 2],
        observation: &str,
    ) {
        let Some(entry) = self.entry_mut(parameter) else {
            return;
        };
        let before = format!(
            "{parameter:?} status={:?} declared={:?} interval={:?}",
            entry.status, entry.declared.value, entry.empirical_interval
        );
        let declared_value = entry.declared.value;
        entry.empirical_interval = Some(interval);
        entry.declared.value = declared_value;
        if entry.status == BeliefEpistemicStatus::DeclaredFact {
            entry.status = BeliefEpistemicStatus::DerivedConstraint;
        }
        let after = format!(
            "{parameter:?} status={:?} declared={:?} interval={:?}",
            entry.status, entry.declared.value, entry.empirical_interval
        );
        entry.lineage.push(BeliefLineage {
            belief_before: before,
            observation: observation.to_string(),
            inference: "EMPIRICAL_INTERVAL_NARROWED".into(),
            belief_after: after,
        });
    }

    /// Intersect one evidence-derived constraint with an existing finite prior.
    /// The original declared value remains immutable and every source is retained.
    pub fn intersect_interval_with_evidence(
        &mut self,
        parameter: PhysicalParameter,
        interval: [f64; 2],
        reference: BeliefEvidenceReference,
    ) -> Result<[f64; 2], BeliefUpdateError> {
        let Some(entry) = self.entry_mut(parameter) else {
            return Err(BeliefUpdateError::MissingParameter);
        };
        if reference.measurement_digest.len() != 64
            || !reference
                .measurement_digest
                .bytes()
                .all(|byte| byte.is_ascii_hexdigit())
        {
            return Err(BeliefUpdateError::InvalidEvidenceDigest);
        }
        if reference.origin == WrenchSampleOrigin::Synthetic {
            return Err(BeliefUpdateError::SyntheticEvidence);
        }
        if reference.origin == WrenchSampleOrigin::Hardware {
            return Err(BeliefUpdateError::HardwareOriginUntrusted);
        }
        if entry.evidence.contains(&reference) {
            return Err(BeliefUpdateError::DuplicateEvidence);
        }
        if entry.status == BeliefEpistemicStatus::Contradicted {
            return Err(BeliefUpdateError::ContradictedBelief);
        }
        let prior = entry
            .empirical_interval
            .ok_or(BeliefUpdateError::MissingEmpiricalPrior)?;
        let valid_interval = |bounds: [f64; 2]| {
            bounds[0].is_finite()
                && bounds[1].is_finite()
                && bounds[0] >= 0.0
                && bounds[0] <= bounds[1]
        };
        if !valid_interval(prior) || !valid_interval(interval) {
            return Err(BeliefUpdateError::InvalidInterval);
        }
        let intersection = [prior[0].max(interval[0]), prior[1].min(interval[1])];
        if intersection[0] > intersection[1] {
            return Err(BeliefUpdateError::EmptyIntersection);
        }

        let before = format!(
            "{parameter:?} status={:?} declared={:?} interval={:?}",
            entry.status, entry.declared.value, entry.empirical_interval
        );
        let declared_value = entry.declared.value;
        entry.empirical_interval = Some(intersection);
        if matches!(
            entry.status,
            BeliefEpistemicStatus::DeclaredFact | BeliefEpistemicStatus::Unknown
        ) {
            entry.status = BeliefEpistemicStatus::DerivedConstraint;
        }
        let after = format!(
            "{parameter:?} status={:?} declared={:?} interval={:?}",
            entry.status, entry.declared.value, entry.empirical_interval
        );
        let origin = reference.origin.as_str();
        let observation = reference.measurement_digest.clone();
        entry.evidence.push(reference);
        entry.lineage.push(BeliefLineage {
            belief_before: before,
            observation,
            inference: format!("WRENCH_EVIDENCE_INTERVAL_INTERSECTED:{origin}"),
            belief_after: after,
        });
        entry.declared.value = declared_value;
        Ok(intersection)
    }

    /// Two measurements that cannot both be true. No replacement value is invented.
    pub fn contradictory_measurements(&mut self, parameter: PhysicalParameter, observation: &str) {
        let Some(entry) = self.entry_mut(parameter) else {
            return;
        };
        let before = format!(
            "{parameter:?} status={:?} declared={:?}",
            entry.status, entry.declared.value
        );
        let declared_value = entry.declared.value;
        entry.status = BeliefEpistemicStatus::Contradicted;
        entry.declared.value = declared_value;
        entry.empirical_interval = None;
        let after = format!(
            "{parameter:?} status={:?} declared={:?} interval=None",
            entry.status, entry.declared.value
        );
        entry.lineage.push(BeliefLineage {
            belief_before: before,
            observation: observation.to_string(),
            inference: "CONTRADICTORY_MEASUREMENTS".into(),
            belief_after: after,
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn declared_mu_is_not_rewritten_when_the_model_is_inconsistent() {
        let mu = 0.42;
        let mut belief = PhysicalParameterBelief::declared_point(
            PhysicalParameter::SupportFriction,
            mu,
            "scene.mu",
        );
        belief.contradict_declared(PhysicalParameter::SupportFriction, "disp_ratio=3.1");
        let entry = belief.entry(PhysicalParameter::SupportFriction).unwrap();
        assert_eq!(entry.declared.value, Some(mu));
        assert_eq!(entry.declared.source, "scene.mu");
        assert_eq!(entry.status, BeliefEpistemicStatus::Contradicted);
        let line = entry.lineage.last().unwrap();
        assert_eq!(line.observation, "disp_ratio=3.1");
        assert_eq!(line.inference, DECLARED_MODEL_INCONSISTENT_WITH_OBSERVATION);
        assert!(line.belief_before.contains("0.42"));
        assert!(line.belief_after.contains("0.42"));
        assert!(line.belief_after.contains("Contradicted"));
    }

    #[test]
    fn unknown_mass_stays_unknown_and_contradictory_measurements_invent_no_value() {
        let mut belief = PhysicalParameterBelief::declared_point(
            PhysicalParameter::SupportFriction,
            0.3,
            "scene.mu",
        )
        .with_unknown(PhysicalParameter::ObjectMassKg, "mass");
        let mass = belief.entry(PhysicalParameter::ObjectMassKg).unwrap();
        assert_eq!(mass.status, BeliefEpistemicStatus::Unknown);
        assert!(mass.declared.value.is_none());
        belief.contradictory_measurements(PhysicalParameter::SupportFriction, "mu_a=0.1 mu_b=0.8");
        let friction = belief.entry(PhysicalParameter::SupportFriction).unwrap();
        assert_eq!(friction.status, BeliefEpistemicStatus::Contradicted);
        assert_eq!(friction.declared.value, Some(0.3));
        assert!(friction.empirical_interval.is_none());
        assert_eq!(
            belief
                .entry(PhysicalParameter::ObjectMassKg)
                .unwrap()
                .status,
            BeliefEpistemicStatus::Unknown
        );
    }
}
