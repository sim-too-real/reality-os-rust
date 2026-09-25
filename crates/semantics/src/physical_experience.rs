//! Append-only physical experience. It can inform a later decision.
//! It cannot authorize a motor write.

use serde::{Deserialize, Serialize};

use crate::discrepancy::DiscrepancyKind;
use crate::physical_belief::PhysicalParameterBelief;
use crate::probe_selection::{rank_goal_or_probe, DecisionCandidate, Ranking};

pub const APPLICABILITY_UNKNOWN: &str = "APPLICABILITY UNKNOWN";

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ApplicabilityQuery {
    pub world_model_id: String,
    pub embodiment_applicability: String,
    pub object_support_geometry: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum ExperienceApplicability {
    Applies,
    ApplicabilityUnknown,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PhysicalExperienceRecord {
    pub world_model_id: String,
    pub embodiment_applicability: String,
    pub object_support_geometry: String,
    pub belief_before: PhysicalParameterBelief,
    pub selected_action: String,
    pub frozen_prediction: String,
    pub authority_result: String,
    pub execution_envelope: String,
    pub observed_consequence: String,
    pub first_divergence: Option<String>,
    pub hypotheses_before: Vec<DiscrepancyKind>,
    pub hypotheses_after: Vec<DiscrepancyKind>,
    pub belief_after: PhysicalParameterBelief,
    pub goal_effect: String,
    pub recoverability_result: String,
    pub provenance: String,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct ExperienceLog {
    records: Vec<PhysicalExperienceRecord>,
}

impl ExperienceLog {
    pub fn append(&mut self, record: PhysicalExperienceRecord) {
        self.records.push(record);
    }

    pub fn records(&self) -> &[PhysicalExperienceRecord] {
        &self.records
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ExperienceAuthorityAttempt {
    pub allow: bool,
    pub unauthorized_writes: u64,
    pub reason: String,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct InitialDecision {
    pub ranking: Ranking,
    pub used_experience: bool,
    pub applicability: String,
    pub live_hypotheses: Vec<DiscrepancyKind>,
}

pub fn experience_applicability(
    record: &PhysicalExperienceRecord,
    query: &ApplicabilityQuery,
) -> ExperienceApplicability {
    if record.world_model_id == query.world_model_id
        && record.embodiment_applicability == query.embodiment_applicability
        && record.object_support_geometry == query.object_support_geometry
    {
        ExperienceApplicability::Applies
    } else {
        ExperienceApplicability::ApplicabilityUnknown
    }
}

/// Experience never mints an authority allow. Writes stay zero.
pub fn authorize_from_experience(_record: &PhysicalExperienceRecord) -> ExperienceAuthorityAttempt {
    ExperienceAuthorityAttempt {
        allow: false,
        unauthorized_writes: 0,
        reason: "EXPERIENCE_CANNOT_AUTHORIZE".into(),
    }
}

pub fn initial_decision(
    log: &ExperienceLog,
    query: &ApplicabilityQuery,
    candidates: &[DecisionCandidate],
    declared_belief: &PhysicalParameterBelief,
    quasi_static_limit_m: f64,
) -> InitialDecision {
    let applicable: Vec<&PhysicalExperienceRecord> = log
        .records()
        .iter()
        .filter(|record| {
            experience_applicability(record, query) == ExperienceApplicability::Applies
        })
        .collect();
    let (live, used, applicability) = if let Some(record) = applicable.last() {
        (record.hypotheses_after.clone(), true, "APPLIES".to_string())
    } else if log.records().is_empty() {
        (Vec::new(), false, "EMPTY".to_string())
    } else {
        (Vec::new(), false, APPLICABILITY_UNKNOWN.to_string())
    };
    let _ = declared_belief;
    InitialDecision {
        ranking: rank_goal_or_probe(candidates, &live, quasi_static_limit_m),
        used_experience: used,
        applicability,
        live_hypotheses: live,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::discrepancy::{DiscrepancyKind, Identifiability};
    use crate::physical_belief::{PhysicalParameter, PhysicalParameterBelief};
    use crate::probe_selection::{candidates_for_uncertainty, DecisionClass};

    const LIMIT: f64 = 0.015;

    fn query() -> ApplicabilityQuery {
        ApplicabilityQuery {
            world_model_id: "world-a".into(),
            embodiment_applicability: "serial-planar".into(),
            object_support_geometry: "box-on-plane".into(),
        }
    }

    fn record(hypotheses_after: Vec<DiscrepancyKind>) -> PhysicalExperienceRecord {
        let belief = PhysicalParameterBelief::declared_point(
            PhysicalParameter::SupportFriction,
            0.5,
            "scene.mu",
        );
        PhysicalExperienceRecord {
            world_model_id: "world-a".into(),
            embodiment_applicability: "serial-planar".into(),
            object_support_geometry: "box-on-plane".into(),
            belief_before: belief.clone(),
            selected_action: "probe_separating".into(),
            frozen_prediction: "sticking_ratio_1".into(),
            authority_result: "AUTHORIZE".into(),
            execution_envelope: "quasi_static_stroke".into(),
            observed_consequence: "nominal_ratio_on_short_stroke".into(),
            first_divergence: Some("MODEL_DISAGREEMENT".into()),
            hypotheses_before: vec![
                DiscrepancyKind::SupportFrictionInconsistent,
                DiscrepancyKind::QuasiStaticAssumptionBroken,
            ],
            hypotheses_after,
            belief_after: belief,
            goal_effect: "probe_no_task_reduction".into(),
            recoverability_result: "PROGRESS_AND_RECOVERABLE".into(),
            provenance: "episode-1".into(),
        }
    }

    #[test]
    fn experience_changes_the_initial_decision_and_cannot_authorize() {
        let candidates = candidates_for_uncertainty(LIMIT);
        let declared = PhysicalParameterBelief::declared_point(
            PhysicalParameter::SupportFriction,
            0.5,
            "scene.mu",
        );
        let empty = initial_decision(
            &ExperienceLog::default(),
            &query(),
            &candidates,
            &declared,
            LIMIT,
        );
        assert!(!empty.used_experience);
        assert_eq!(
            empty.ranking.selected_class,
            Some(DecisionClass::GoalAction)
        );
        assert_eq!(empty.ranking.selected_id.as_deref(), Some("goal_large"));

        let mut log = ExperienceLog::default();
        let stored = record(vec![DiscrepancyKind::QuasiStaticAssumptionBroken]);
        log.append(stored.clone());
        let with = initial_decision(&log, &query(), &candidates, &declared, LIMIT);
        assert!(with.used_experience);
        assert_ne!(with.ranking.selected_id, empty.ranking.selected_id);
        assert_eq!(with.ranking.selected_id.as_deref(), Some("goal_small"));

        let mut other = query();
        other.embodiment_applicability = "different-embodiment".into();
        let blocked = initial_decision(&log, &other, &candidates, &declared, LIMIT);
        assert_eq!(blocked.applicability, APPLICABILITY_UNKNOWN);
        assert!(!blocked.used_experience);
        assert_eq!(blocked.ranking.selected_id, empty.ranking.selected_id);
        assert_eq!(
            experience_applicability(log.records().first().unwrap(), &other),
            ExperienceApplicability::ApplicabilityUnknown
        );

        let attempt = authorize_from_experience(&stored);
        assert!(!attempt.allow);
        assert_eq!(attempt.unauthorized_writes, 0);
        let _ = Identifiability::Identified;
    }
}
