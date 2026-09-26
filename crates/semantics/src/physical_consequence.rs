//! Shared prediction-versus-observation assessment for completed and aborted actions.
//! Missing observations remain `None`; this module never fills them with numeric defaults.

use serde::{Deserialize, Serialize};

use crate::discrepancy::{DiscrepancyKind, Identifiability};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FrozenPrediction {
    pub action_id: String,
    pub witness_id: String,
    pub stroke_m: f64,
    pub predicted_displacement_m: Option<f64>,
    pub predicted_yaw_change_rad: Option<f64>,
    pub predicted_contact_persists: bool,
    pub quasi_static_stroke_limit_m: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ObservedConsequence {
    pub action_id: String,
    pub witness_id: String,
    pub observation_id: String,
    pub freshness_ok: bool,
    pub execution_aborted: bool,
    pub stroke_consumed_m: Option<f64>,
    pub displacement_m: Option<f64>,
    pub yaw_change_rad: Option<f64>,
    pub contact_persisted: Option<bool>,
    pub tracking_error_m: Option<f64>,
    pub geometry_residual_m: Option<f64>,
    pub quasi_static_applicable: Option<bool>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum ConsequenceStatus {
    Consistent,
    Contradicted,
    Underdetermined,
    InsufficientEvidence,
    OutsideModelRegime,
    ExecutionDiverged,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ConsequenceAssessment {
    pub status: ConsequenceStatus,
    pub identifiability: Identifiability,
    pub prediction: FrozenPrediction,
    pub observation: ObservedConsequence,
    /// Existing discrepancy taxonomy is reused to avoid inventing parallel causes.
    pub hypotheses: Vec<DiscrepancyKind>,
}

fn has_missing_required_observation(observation: &ObservedConsequence) -> bool {
    observation.stroke_consumed_m.is_none()
        || observation.displacement_m.is_none()
        || observation.yaw_change_rad.is_none()
        || observation.contact_persisted.is_none()
        || observation.tracking_error_m.is_none()
        || observation.geometry_residual_m.is_none()
        || observation.quasi_static_applicable.is_none()
}

fn assessment(
    status: ConsequenceStatus,
    identifiability: Identifiability,
    prediction: &FrozenPrediction,
    observation: &ObservedConsequence,
    hypotheses: Vec<DiscrepancyKind>,
) -> ConsequenceAssessment {
    ConsequenceAssessment {
        status,
        identifiability,
        prediction: prediction.clone(),
        observation: observation.clone(),
        hypotheses,
    }
}

/// Assess one exact frozen action using only its policy-visible consequence.
pub fn assess_consequence(
    prediction: &FrozenPrediction,
    observation: &ObservedConsequence,
) -> ConsequenceAssessment {
    if observation.action_id != prediction.action_id
        || observation.witness_id != prediction.witness_id
    {
        return assessment(
            ConsequenceStatus::ExecutionDiverged,
            Identifiability::Unknown,
            prediction,
            observation,
            vec![DiscrepancyKind::ExecutionTrackingDivergence],
        );
    }
    let aborted = observation.execution_aborted;
    if !aborted
        && (observation.quasi_static_applicable == Some(false)
            || prediction.stroke_m > prediction.quasi_static_stroke_limit_m)
    {
        return assessment(
            ConsequenceStatus::OutsideModelRegime,
            Identifiability::Unknown,
            prediction,
            observation,
            vec![DiscrepancyKind::QuasiStaticAssumptionBroken],
        );
    }
    let evidence_incomplete =
        !observation.freshness_ok || has_missing_required_observation(observation);
    if !aborted && evidence_incomplete {
        return assessment(
            ConsequenceStatus::InsufficientEvidence,
            Identifiability::Unknown,
            prediction,
            observation,
            vec![DiscrepancyKind::StaleOrInsufficientObservation],
        );
    }

    let mut live = Vec::new();
    if evidence_incomplete {
        // Preserve unknown required sensors alongside any independent evidence
        // that was actually observed. In particular, a missing geometry
        // residual must not erase a fresh contact-loss observation.
        live.push(DiscrepancyKind::StaleOrInsufficientObservation);
    }
    if let (Some(expected), Some(actual)) = (
        prediction.predicted_displacement_m,
        observation.displacement_m,
    ) {
        if !expected.is_finite() || !actual.is_finite() || (expected - actual).abs() > 0.01 {
            live.push(DiscrepancyKind::SupportFrictionInconsistent);
        }
    } else {
        live.push(DiscrepancyKind::SupportFrictionInconsistent);
        live.push(DiscrepancyKind::QuasiStaticAssumptionBroken);
    }
    if let (Some(expected), Some(actual)) = (
        prediction.predicted_yaw_change_rad,
        observation.yaw_change_rad,
    ) {
        if !expected.is_finite() || !actual.is_finite() || (expected - actual).abs() > 0.05 {
            live.push(DiscrepancyKind::ContactModeModelInconsistent);
        }
    } else {
        live.push(DiscrepancyKind::ContactModeModelInconsistent);
        live.push(DiscrepancyKind::ContactGeometryDisagreement);
    }
    if prediction.predicted_contact_persists && observation.contact_persisted == Some(false) {
        live.push(DiscrepancyKind::ToolContactFrictionInconsistent);
    }
    if observation
        .tracking_error_m
        .is_some_and(|error| !error.is_finite() || error > 0.03)
    {
        live.push(DiscrepancyKind::ExecutionTrackingDivergence);
    }
    if observation
        .geometry_residual_m
        .is_some_and(|residual| !residual.is_finite() || residual > 0.02)
    {
        live.push(DiscrepancyKind::ContactGeometryDisagreement);
    }
    if aborted {
        // The abort is an execution event, not a substitute for assessing the
        // physical observation that caused it. Keep both in the evidence.
        live.push(DiscrepancyKind::ExecutionTrackingDivergence);
    }
    live.sort_by_key(|kind| *kind as u8);
    live.dedup();

    if aborted {
        let physical_hypothesis_count = live
            .iter()
            .filter(|kind| {
                **kind != DiscrepancyKind::ExecutionTrackingDivergence
                    && **kind != DiscrepancyKind::StaleOrInsufficientObservation
            })
            .count();
        let identifiability = if physical_hypothesis_count > 1 {
            Identifiability::Underdetermined
        } else {
            Identifiability::Unknown
        };
        return assessment(
            ConsequenceStatus::ExecutionDiverged,
            identifiability,
            prediction,
            observation,
            live,
        );
    }

    if live.is_empty() {
        assessment(
            ConsequenceStatus::Consistent,
            Identifiability::Identified,
            prediction,
            observation,
            live,
        )
    } else if live.len() > 1 {
        assessment(
            ConsequenceStatus::Underdetermined,
            Identifiability::Underdetermined,
            prediction,
            observation,
            live,
        )
    } else {
        assessment(
            ConsequenceStatus::Contradicted,
            Identifiability::Contradicted,
            prediction,
            observation,
            live,
        )
    }
}
