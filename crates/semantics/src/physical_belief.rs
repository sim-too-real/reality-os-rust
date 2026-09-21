//! Typed, dimensioned, provenance-bearing physical parameters.
//! Declared numbers are never rewritten. New evidence is appended.

use serde::{Deserialize, Serialize};

use crate::provenance::Provenanced;

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

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ParameterBelief {
    pub parameter: PhysicalParameter,
    pub status: BeliefEpistemicStatus,
    /// Original declared number. Stays put when observations disagree with it.
    pub declared: Provenanced<f64>,
    pub empirical_interval: Option<[f64; 2]>,
    pub lineage: Vec<BeliefLineage>,
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
