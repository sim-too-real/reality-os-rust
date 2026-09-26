//! Competing physical explanations of one disagreement.
//! A shared motion does not identify a single cause.

use serde::{Deserialize, Serialize};

use crate::physical_belief::{
    BeliefEpistemicStatus, BeliefLineage, PhysicalParameter, PhysicalParameterBelief,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum DiscrepancyKind {
    SupportFrictionInconsistent,
    ToolContactFrictionInconsistent,
    ContactModeModelInconsistent,
    QuasiStaticAssumptionBroken,
    ExecutionTrackingDivergence,
    ContactGeometryDisagreement,
    StaleOrInsufficientObservation,
    UnidentifiableFromCurrentEvidence,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum Identifiability {
    Identified,
    Underdetermined,
    Unknown,
    Contradicted,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum ObservationTag {
    HighDisplacementRatio,
    NominalDisplacementRatio,
    ContactLost,
    TrackingErrorHigh,
    YawSignFlip,
    GeometryMismatch,
    Insufficient,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum PhysicalReasoningOutcome {
    ActionFailed,
    ModelContradicted,
    ParameterUncertain,
    PhysicalRegimeChanged,
    ExecutionDiverged,
    RecoverabilityAtRisk,
    StateAlreadyUnrecoverable,
    SafeProbeAvailable,
    NoSafeProbe,
    GoalCurrentlyUnachievable,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PhysicalDiscrepancyHypothesis {
    pub kind: DiscrepancyKind,
    pub supporting_evidence: Vec<String>,
    pub contradicting_evidence: Vec<String>,
    pub predicted_observations: Vec<String>,
    pub distinguishing_observation: String,
    pub applicability_regime: String,
    pub provenance: String,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct DiscrepancyObservation {
    pub displacement_ratio: Option<f64>,
    pub yaw_change_rad: Option<f64>,
    pub predicted_yaw_sign: Option<i8>,
    pub observed_yaw_sign: Option<i8>,
    pub contact_persisted: Option<bool>,
    pub tracking_error_m: Option<f64>,
    pub geometry_residual_m: Option<f64>,
    pub freshness_ok: bool,
    pub stroke_m: f64,
    pub quasi_static_stroke_limit_m: f64,
    pub contradictory: bool,
    pub reachable: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Stimulus {
    pub stroke_m: f64,
    pub quasi_static_stroke_limit_m: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct HypothesisReport {
    pub status: Identifiability,
    pub hypotheses: Vec<PhysicalDiscrepancyHypothesis>,
    pub kinds: Vec<DiscrepancyKind>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ProbeUpdate {
    pub belief: PhysicalParameterBelief,
    pub remaining: Vec<DiscrepancyKind>,
    pub eliminated: Vec<DiscrepancyKind>,
    pub status: Identifiability,
    pub observation_id: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct TaxonomyFacts {
    pub action_failed: bool,
    pub model_contradicted: bool,
    pub parameter_uncertain: bool,
    pub regime_changed: bool,
    pub execution_diverged: bool,
    pub recoverability_at_risk: bool,
    pub already_unrecoverable: bool,
    pub safe_probe: Option<bool>,
    pub goal_unachievable: bool,
}

impl TaxonomyFacts {
    pub fn none() -> Self {
        Self {
            action_failed: false,
            model_contradicted: false,
            parameter_uncertain: false,
            regime_changed: false,
            execution_diverged: false,
            recoverability_at_risk: false,
            already_unrecoverable: false,
            safe_probe: None,
            goal_unachievable: false,
        }
    }
}

/// Classify a probe's measured motion. Lost contact or almost no motion is not
/// a nominal quasi-static result.
pub fn tag_probe_motion(
    displacement_m: f64,
    stroke_m: f64,
    contact_persisted: bool,
) -> ObservationTag {
    let stroke = stroke_m.max(1e-9);
    if !contact_persisted || !displacement_m.is_finite() || displacement_m < 0.25 * stroke {
        return ObservationTag::Insufficient;
    }
    let ratio = displacement_m / stroke;
    if ratio > crate::execution_envelope::QUASI_STATIC_DISPLACEMENT_RATIO {
        ObservationTag::HighDisplacementRatio
    } else {
        ObservationTag::NominalDisplacementRatio
    }
}

/// Friction value and quasi-static flag used to freeze the next prediction.
/// The declared friction number is not rewritten.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct PredictionRegime {
    /// Declared coefficient. Not replaced when the declaration is contradicted.
    pub support_friction: f64,
    pub friction_contradicted: bool,
    pub quasi_static: bool,
}

pub fn prediction_regime(
    belief: &PhysicalParameterBelief,
    live: &[DiscrepancyKind],
    stroke_m: f64,
    quasi_static_limit_m: f64,
) -> PredictionRegime {
    let support_friction = belief
        .declared_value(PhysicalParameter::SupportFriction)
        .unwrap_or(f64::NAN);
    let friction_contradicted = live.contains(&DiscrepancyKind::SupportFrictionInconsistent)
        || belief
            .entry(PhysicalParameter::SupportFriction)
            .is_some_and(|entry| entry.status == BeliefEpistemicStatus::Contradicted);
    let quasi_broken = live.contains(&DiscrepancyKind::QuasiStaticAssumptionBroken)
        || belief
            .entry(PhysicalParameter::QuasiStaticApplicability)
            .is_some_and(|entry| {
                matches!(
                    entry.status,
                    BeliefEpistemicStatus::DerivedConstraint | BeliefEpistemicStatus::Contradicted
                )
            });
    PredictionRegime {
        support_friction,
        friction_contradicted,
        quasi_static: !quasi_broken || stroke_m <= quasi_static_limit_m,
    }
}

/// What `kind` predicts for a stimulus of this stroke. Not a probability.
pub fn predicted_tag(kind: DiscrepancyKind, stimulus: Stimulus) -> ObservationTag {
    match kind {
        DiscrepancyKind::SupportFrictionInconsistent => ObservationTag::HighDisplacementRatio,
        DiscrepancyKind::QuasiStaticAssumptionBroken => {
            if stimulus.stroke_m > stimulus.quasi_static_stroke_limit_m {
                ObservationTag::HighDisplacementRatio
            } else {
                ObservationTag::NominalDisplacementRatio
            }
        }
        DiscrepancyKind::ToolContactFrictionInconsistent => ObservationTag::ContactLost,
        DiscrepancyKind::ContactModeModelInconsistent => ObservationTag::YawSignFlip,
        DiscrepancyKind::ExecutionTrackingDivergence => ObservationTag::TrackingErrorHigh,
        DiscrepancyKind::ContactGeometryDisagreement => ObservationTag::GeometryMismatch,
        DiscrepancyKind::StaleOrInsufficientObservation
        | DiscrepancyKind::UnidentifiableFromCurrentEvidence => ObservationTag::Insufficient,
    }
}

fn hypothesis(
    kind: DiscrepancyKind,
    supporting: &[&str],
    contradicting: &[&str],
    stimulus: Stimulus,
) -> PhysicalDiscrepancyHypothesis {
    let tag = predicted_tag(kind, stimulus);
    PhysicalDiscrepancyHypothesis {
        kind,
        supporting_evidence: supporting.iter().map(|s| (*s).to_string()).collect(),
        contradicting_evidence: contradicting.iter().map(|s| (*s).to_string()).collect(),
        predicted_observations: vec![format!("{tag:?}")],
        distinguishing_observation: format!(
            "compare {tag:?} at stroke_m={:.4} against limit_m={:.4}",
            stimulus.stroke_m, stimulus.quasi_static_stroke_limit_m
        ),
        applicability_regime: format!(
            "planar_push stroke_m={:.4} limit_m={:.4}",
            stimulus.stroke_m, stimulus.quasi_static_stroke_limit_m
        ),
        provenance: "runtime_observation".into(),
    }
}

fn high_ratio(ratio: f64) -> bool {
    ratio > crate::execution_envelope::QUASI_STATIC_DISPLACEMENT_RATIO
}

/// Explanations consistent with this observation. Several may remain.
pub fn hypothesize(observation: &DiscrepancyObservation) -> HypothesisReport {
    let stimulus = Stimulus {
        stroke_m: observation.stroke_m,
        quasi_static_stroke_limit_m: observation.quasi_static_stroke_limit_m,
    };
    if !observation.reachable {
        let h = hypothesis(
            DiscrepancyKind::UnidentifiableFromCurrentEvidence,
            &[],
            &["object_outside_interaction_region"],
            stimulus,
        );
        return HypothesisReport {
            status: Identifiability::Unknown,
            kinds: vec![h.kind],
            hypotheses: vec![h],
        };
    }
    if observation.contradictory {
        let h = hypothesis(
            DiscrepancyKind::UnidentifiableFromCurrentEvidence,
            &[],
            &["contradictory_measurements"],
            stimulus,
        );
        return HypothesisReport {
            status: Identifiability::Contradicted,
            kinds: vec![h.kind],
            hypotheses: vec![h],
        };
    }
    if !observation.freshness_ok {
        let h = hypothesis(
            DiscrepancyKind::StaleOrInsufficientObservation,
            &["missing_or_stale_motion"],
            &[],
            stimulus,
        );
        return HypothesisReport {
            status: Identifiability::Unknown,
            kinds: vec![h.kind],
            hypotheses: vec![h],
        };
    }
    let mut kinds = Vec::new();
    let motion_ratio_available = observation.displacement_ratio.is_some();
    let ratio = observation.displacement_ratio.unwrap_or(0.0);
    let tracking_low = observation
        .tracking_error_m
        .is_some_and(|v| v.is_finite() && v >= 0.0 && v <= 0.03);
    let tracking_high = observation
        .tracking_error_m
        .is_some_and(|v| v.is_finite() && v > 0.03);
    let geometry_low = observation
        .geometry_residual_m
        .is_some_and(|v| v.is_finite() && v >= 0.0 && v <= 0.02);
    let geometry_bad = observation
        .geometry_residual_m
        .is_some_and(|v| v.is_finite() && v > 0.02);
    if !motion_ratio_available
        || !(tracking_low || tracking_high)
        || !(geometry_low || geometry_bad)
    {
        kinds.push(DiscrepancyKind::StaleOrInsufficientObservation);
    }
    let yaw_flip = matches!(
        (observation.predicted_yaw_sign, observation.observed_yaw_sign),
        (Some(a), Some(b)) if a != 0 && b != 0 && a != b
    );
    let contact_lost = observation.contact_persisted == Some(false);
    if tracking_high {
        kinds.push(DiscrepancyKind::ExecutionTrackingDivergence);
    }
    if geometry_bad {
        kinds.push(DiscrepancyKind::ContactGeometryDisagreement);
    }
    if yaw_flip {
        kinds.push(DiscrepancyKind::ContactModeModelInconsistent);
    }
    if contact_lost {
        kinds.push(DiscrepancyKind::ToolContactFrictionInconsistent);
    }
    if motion_ratio_available && high_ratio(ratio) && tracking_low && geometry_low {
        kinds.push(DiscrepancyKind::SupportFrictionInconsistent);
        if observation.stroke_m > observation.quasi_static_stroke_limit_m {
            kinds.push(DiscrepancyKind::QuasiStaticAssumptionBroken);
        }
    }
    if kinds.is_empty() {
        kinds.push(DiscrepancyKind::UnidentifiableFromCurrentEvidence);
    }
    let hypotheses = kinds
        .iter()
        .map(|kind| {
            let supporting = match kind {
                DiscrepancyKind::UnidentifiableFromCurrentEvidence => &[][..],
                DiscrepancyKind::StaleOrInsufficientObservation => {
                    &["missing_or_invalid_tracking_or_geometry_measurement"][..]
                }
                _ => &["motion_consistent_with_hypothesis"][..],
            };
            hypothesis(*kind, supporting, &[], stimulus)
        })
        .collect();
    let status = match kinds.as_slice() {
        [DiscrepancyKind::UnidentifiableFromCurrentEvidence]
        | [DiscrepancyKind::StaleOrInsufficientObservation] => Identifiability::Unknown,
        [_] => Identifiability::Identified,
        _ => Identifiability::Underdetermined,
    };
    HypothesisReport {
        status,
        hypotheses,
        kinds,
    }
}

pub fn classify_reasoning_outcome(facts: &TaxonomyFacts) -> PhysicalReasoningOutcome {
    if facts.goal_unachievable {
        PhysicalReasoningOutcome::GoalCurrentlyUnachievable
    } else if facts.already_unrecoverable {
        PhysicalReasoningOutcome::StateAlreadyUnrecoverable
    } else if facts.execution_diverged {
        PhysicalReasoningOutcome::ExecutionDiverged
    } else if facts.regime_changed {
        PhysicalReasoningOutcome::PhysicalRegimeChanged
    } else if facts.model_contradicted {
        PhysicalReasoningOutcome::ModelContradicted
    } else if facts.recoverability_at_risk {
        PhysicalReasoningOutcome::RecoverabilityAtRisk
    } else if facts.safe_probe == Some(true) {
        PhysicalReasoningOutcome::SafeProbeAvailable
    } else if facts.safe_probe == Some(false) {
        PhysicalReasoningOutcome::NoSafeProbe
    } else if facts.parameter_uncertain {
        PhysicalReasoningOutcome::ParameterUncertain
    } else if facts.action_failed {
        PhysicalReasoningOutcome::ActionFailed
    } else {
        PhysicalReasoningOutcome::ParameterUncertain
    }
}

/// Drop hypotheses whose predicted tag disagrees with `observed`.
/// Declared numeric values are copied unchanged. A remaining friction
/// hypothesis records `DECLARED_MODEL_INCONSISTENT_WITH_OBSERVATION`.
pub fn apply_probe_observation(
    belief: &PhysicalParameterBelief,
    live: &[DiscrepancyKind],
    stimulus: Stimulus,
    observed: ObservationTag,
    observation_id: &str,
) -> ProbeUpdate {
    if observed == ObservationTag::Insufficient {
        let mut belief = belief.clone();
        if let Some(entry) = belief.entry_mut(PhysicalParameter::SupportFriction) {
            let declared = entry.declared.value;
            entry.lineage.push(BeliefLineage {
                belief_before: format!(
                    "support_friction declared={declared:?} status={:?}",
                    entry.status
                ),
                observation: observation_id.to_string(),
                inference: "INSUFFICIENT_PROBE_MOTION".into(),
                belief_after: format!(
                    "support_friction declared={declared:?} status={:?} unchanged",
                    entry.status
                ),
            });
        }
        return ProbeUpdate {
            belief,
            remaining: live.to_vec(),
            eliminated: Vec::new(),
            status: if live.len() > 1 {
                Identifiability::Underdetermined
            } else {
                Identifiability::Unknown
            },
            observation_id: observation_id.to_string(),
        };
    }
    let mut remaining = Vec::new();
    let mut eliminated = Vec::new();
    for kind in live {
        if predicted_tag(*kind, stimulus) == observed {
            remaining.push(*kind);
        } else {
            eliminated.push(*kind);
        }
    }
    let mut belief = belief.clone();
    let status = match remaining.len() {
        0 => Identifiability::Unknown,
        1 => Identifiability::Identified,
        _ => Identifiability::Underdetermined,
    };
    if remaining.contains(&DiscrepancyKind::SupportFrictionInconsistent) {
        if belief.entry(PhysicalParameter::SupportFriction).is_none() {
            // Nothing to contradict if the parameter was never declared.
        } else {
            belief.contradict_declared(PhysicalParameter::SupportFriction, observation_id);
        }
    } else if eliminated.contains(&DiscrepancyKind::SupportFrictionInconsistent) {
        if let Some(entry) = belief.entry_mut(PhysicalParameter::SupportFriction) {
            let before = format!(
                "support_friction declared={:?} status={:?}",
                entry.declared.value, entry.status
            );
            let after = format!(
                "support_friction declared={:?} status={:?} eliminated=SUPPORT_FRICTION_INCONSISTENT",
                entry.declared.value, entry.status
            );
            entry.lineage.push(BeliefLineage {
                belief_before: before,
                observation: observation_id.to_string(),
                inference: "HYPOTHESIS_ELIMINATED_BY_PROBE".into(),
                belief_after: after,
            });
        }
    }
    if remaining.contains(&DiscrepancyKind::QuasiStaticAssumptionBroken) {
        if let Some(entry) = belief.entry_mut(PhysicalParameter::QuasiStaticApplicability) {
            let before = format!(
                "quasi_static declared={:?} status={:?}",
                entry.declared.value, entry.status
            );
            let declared = entry.declared.value;
            entry.status = BeliefEpistemicStatus::DerivedConstraint;
            entry.declared.value = declared;
            entry.lineage.push(BeliefLineage {
                belief_before: before,
                observation: observation_id.to_string(),
                inference: "QUASI_STATIC_ONLY_BELOW_STROKE_LIMIT".into(),
                belief_after: format!(
                    "quasi_static declared={declared:?} status=DerivedConstraint above_limit_inapplicable"
                ),
            });
        }
    } else if observed == ObservationTag::HighDisplacementRatio
        && belief
            .entry(PhysicalParameter::QuasiStaticApplicability)
            .is_some()
    {
        // A high ratio eliminates the "broken only on long strokes" story and
        // contradicts the declaration that quasi-static mechanics still applies.
        // The declared number is left unchanged.
        belief.contradict_declared(PhysicalParameter::QuasiStaticApplicability, observation_id);
    }
    ProbeUpdate {
        belief,
        remaining,
        eliminated,
        status,
        observation_id: observation_id.to_string(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physical_belief::{
        BeliefEpistemicStatus, PhysicalParameter, PhysicalParameterBelief,
        DECLARED_MODEL_INCONSISTENT_WITH_OBSERVATION,
    };

    fn large_slip() -> DiscrepancyObservation {
        DiscrepancyObservation {
            displacement_ratio: Some(3.0),
            yaw_change_rad: Some(0.0),
            predicted_yaw_sign: Some(0),
            observed_yaw_sign: Some(0),
            contact_persisted: Some(true),
            tracking_error_m: Some(0.0),
            geometry_residual_m: Some(0.0),
            freshness_ok: true,
            stroke_m: 0.04,
            quasi_static_stroke_limit_m: 0.015,
            contradictory: false,
            reachable: true,
        }
    }

    #[test]
    fn same_large_motion_keeps_friction_and_quasi_static_underdetermined() {
        let report = hypothesize(&large_slip());
        assert_eq!(report.status, Identifiability::Underdetermined);
        assert!(report
            .kinds
            .contains(&DiscrepancyKind::SupportFrictionInconsistent));
        assert!(report
            .kinds
            .contains(&DiscrepancyKind::QuasiStaticAssumptionBroken));
        let stimulus = Stimulus {
            stroke_m: 0.04,
            quasi_static_stroke_limit_m: 0.015,
        };
        assert_eq!(
            predicted_tag(DiscrepancyKind::SupportFrictionInconsistent, stimulus),
            predicted_tag(DiscrepancyKind::QuasiStaticAssumptionBroken, stimulus)
        );
    }

    #[test]
    fn removing_required_discriminators_never_identifies_friction() {
        let mut fully_observed = large_slip();
        fully_observed.tracking_error_m = Some(0.0);
        fully_observed.geometry_residual_m = Some(0.0);
        let full_report = hypothesize(&fully_observed);
        assert!(full_report
            .kinds
            .contains(&DiscrepancyKind::SupportFrictionInconsistent));

        let discriminator_combinations = [
            (None, None),
            (None, Some(0.0)),
            (Some(0.0), None),
            (Some(0.0), Some(0.0)),
        ];
        for (tracking, geometry) in discriminator_combinations {
            let mut observation = large_slip();
            observation.tracking_error_m = tracking;
            observation.geometry_residual_m = geometry;
            let report = hypothesize(&observation);
            if tracking.is_some() && geometry.is_some() {
                assert!(report
                    .kinds
                    .contains(&DiscrepancyKind::SupportFrictionInconsistent));
            } else {
                assert!(!report
                    .kinds
                    .contains(&DiscrepancyKind::SupportFrictionInconsistent));
                assert!(report
                    .kinds
                    .contains(&DiscrepancyKind::StaleOrInsufficientObservation));
                assert_ne!(report.status, Identifiability::Identified);
            }
        }

        for (tracking, geometry) in [
            (Some(f64::NAN), Some(0.0)),
            (Some(0.0), Some(f64::INFINITY)),
        ] {
            let mut observation = large_slip();
            observation.tracking_error_m = tracking;
            observation.geometry_residual_m = geometry;
            let report = hypothesize(&observation);
            assert!(!report
                .kinds
                .contains(&DiscrepancyKind::SupportFrictionInconsistent));
            assert!(report
                .kinds
                .contains(&DiscrepancyKind::StaleOrInsufficientObservation));
            assert_ne!(report.status, Identifiability::Identified);
        }
    }

    #[test]
    fn present_positive_discrepancy_evidence_is_preserved() {
        let low = hypothesize(&large_slip());
        assert!(low
            .kinds
            .contains(&DiscrepancyKind::SupportFrictionInconsistent));

        let mut tracking_high = large_slip();
        tracking_high.tracking_error_m = Some(0.031);
        let tracking_report = hypothesize(&tracking_high);
        assert!(tracking_report
            .kinds
            .contains(&DiscrepancyKind::ExecutionTrackingDivergence));
        assert!(!tracking_report
            .kinds
            .contains(&DiscrepancyKind::SupportFrictionInconsistent));

        let mut geometry_high = large_slip();
        geometry_high.geometry_residual_m = Some(0.021);
        let geometry_report = hypothesize(&geometry_high);
        assert!(geometry_report
            .kinds
            .contains(&DiscrepancyKind::ContactGeometryDisagreement));
        assert!(!geometry_report
            .kinds
            .contains(&DiscrepancyKind::SupportFrictionInconsistent));
    }

    #[test]
    fn insufficient_contradictory_and_unreachable_stay_distinct() {
        let insufficient = hypothesize(&DiscrepancyObservation {
            displacement_ratio: None,
            yaw_change_rad: None,
            predicted_yaw_sign: None,
            observed_yaw_sign: None,
            contact_persisted: None,
            tracking_error_m: None,
            geometry_residual_m: None,
            freshness_ok: false,
            stroke_m: 0.0,
            quasi_static_stroke_limit_m: 0.015,
            contradictory: false,
            reachable: true,
        });
        assert_eq!(insufficient.status, Identifiability::Unknown);
        assert_eq!(
            insufficient.kinds,
            vec![DiscrepancyKind::StaleOrInsufficientObservation]
        );

        let mut contradictory_obs = large_slip();
        contradictory_obs.contradictory = true;
        let contradictory = hypothesize(&contradictory_obs);
        assert_eq!(contradictory.status, Identifiability::Contradicted);

        let mut unreachable = large_slip();
        unreachable.reachable = false;
        let unreachable = hypothesize(&unreachable);
        assert_eq!(unreachable.status, Identifiability::Unknown);
        assert_ne!(insufficient.status, contradictory.status);
        assert_ne!(insufficient.kinds[0], contradictory.kinds[0]);
    }

    #[test]
    fn missing_motion_ratio_does_not_erase_fresh_contact_loss_evidence() {
        let observation = DiscrepancyObservation {
            displacement_ratio: None,
            yaw_change_rad: Some(0.0),
            predicted_yaw_sign: Some(0),
            observed_yaw_sign: Some(0),
            contact_persisted: Some(false),
            tracking_error_m: Some(0.0),
            geometry_residual_m: None,
            freshness_ok: true,
            stroke_m: 0.0,
            quasi_static_stroke_limit_m: 0.015,
            contradictory: false,
            reachable: true,
        };

        let report = hypothesize(&observation);
        assert_eq!(report.status, Identifiability::Underdetermined);
        assert!(report
            .kinds
            .contains(&DiscrepancyKind::StaleOrInsufficientObservation));
        assert!(report
            .kinds
            .contains(&DiscrepancyKind::ToolContactFrictionInconsistent));
        assert!(!report
            .kinds
            .contains(&DiscrepancyKind::SupportFrictionInconsistent));
    }

    #[test]
    fn probe_observation_does_not_rewrite_declared_mu() {
        let mu = 0.55;
        let belief = PhysicalParameterBelief::declared_point(
            PhysicalParameter::SupportFriction,
            mu,
            "scene.mu",
        );
        let live = vec![
            DiscrepancyKind::SupportFrictionInconsistent,
            DiscrepancyKind::QuasiStaticAssumptionBroken,
        ];
        let stimulus = Stimulus {
            stroke_m: 0.008,
            quasi_static_stroke_limit_m: 0.015,
        };
        let update = apply_probe_observation(
            &belief,
            &live,
            stimulus,
            ObservationTag::HighDisplacementRatio,
            "probe-1",
        );
        assert!(update
            .remaining
            .contains(&DiscrepancyKind::SupportFrictionInconsistent));
        assert!(update
            .eliminated
            .contains(&DiscrepancyKind::QuasiStaticAssumptionBroken));
        let entry = update
            .belief
            .entry(PhysicalParameter::SupportFriction)
            .unwrap();
        assert_eq!(entry.declared.value, Some(mu));
        assert_eq!(entry.declared.source, "scene.mu");
        assert_eq!(entry.status, BeliefEpistemicStatus::Contradicted);
        let line = entry.lineage.last().unwrap();
        assert_eq!(line.observation, "probe-1");
        assert_eq!(line.inference, DECLARED_MODEL_INCONSISTENT_WITH_OBSERVATION);
        assert!(!line.belief_before.is_empty());
        assert!(!line.belief_after.is_empty());
    }

    #[test]
    fn missed_contact_or_no_motion_does_not_identify_quasi_static() {
        assert_eq!(
            tag_probe_motion(0.0, 0.008, false),
            ObservationTag::Insufficient
        );
        assert_eq!(
            tag_probe_motion(0.0001, 0.008, true),
            ObservationTag::Insufficient
        );
        assert_eq!(
            tag_probe_motion(0.007, 0.008, true),
            ObservationTag::NominalDisplacementRatio
        );
        let belief = PhysicalParameterBelief::declared_point(
            PhysicalParameter::SupportFriction,
            0.3,
            "scene.mu",
        );
        let live = vec![
            DiscrepancyKind::SupportFrictionInconsistent,
            DiscrepancyKind::QuasiStaticAssumptionBroken,
        ];
        let update = apply_probe_observation(
            &belief,
            &live,
            Stimulus {
                stroke_m: 0.008,
                quasi_static_stroke_limit_m: 0.015,
            },
            ObservationTag::Insufficient,
            "probe-miss",
        );
        assert_eq!(update.eliminated, Vec::<DiscrepancyKind>::new());
        assert_eq!(update.remaining, live);
        assert_eq!(update.status, Identifiability::Underdetermined);
        assert_eq!(
            update
                .belief
                .declared_value(PhysicalParameter::SupportFriction),
            Some(0.3)
        );
        let regime = prediction_regime(
            &belief,
            &[DiscrepancyKind::QuasiStaticAssumptionBroken],
            0.03,
            0.015,
        );
        assert_eq!(regime.support_friction, 0.3);
        assert!(!regime.quasi_static);
        assert!(!regime.friction_contradicted);
        let short = prediction_regime(
            &belief,
            &[DiscrepancyKind::QuasiStaticAssumptionBroken],
            0.008,
            0.015,
        );
        assert!(short.quasi_static);
        let mut with_regime = PhysicalParameterBelief::declared_point(
            PhysicalParameter::SupportFriction,
            0.3,
            "scene.mu",
        );
        with_regime
            .parameters
            .push(crate::physical_belief::ParameterBelief {
                parameter: PhysicalParameter::QuasiStaticApplicability,
                status: BeliefEpistemicStatus::DeclaredFact,
                declared: crate::provenance::Provenanced::declared(
                    1.0,
                    "declared.quasi_static",
                    0.0,
                ),
                empirical_interval: None,
                lineage: Vec::new(),
            });
        let revised = apply_probe_observation(
            &with_regime,
            &live,
            Stimulus {
                stroke_m: 0.008,
                quasi_static_stroke_limit_m: 0.015,
            },
            ObservationTag::HighDisplacementRatio,
            "probe-high",
        );
        let quasi = revised
            .belief
            .entry(PhysicalParameter::QuasiStaticApplicability)
            .unwrap();
        assert_eq!(quasi.declared.value, Some(1.0));
        assert_eq!(quasi.status, BeliefEpistemicStatus::Contradicted);
        assert_eq!(
            quasi.lineage.last().unwrap().inference,
            DECLARED_MODEL_INCONSISTENT_WITH_OBSERVATION
        );
        let long = prediction_regime(&revised.belief, &revised.remaining, 0.03, 0.015);
        assert!(!long.quasi_static);
        assert!(long.friction_contradicted);
        let kept = prediction_regime(&revised.belief, &revised.remaining, 0.004, 0.015);
        assert!(kept.quasi_static);
        let above = apply_probe_observation(
            &with_regime,
            &live,
            Stimulus {
                stroke_m: 0.02,
                quasi_static_stroke_limit_m: 0.015,
            },
            ObservationTag::NominalDisplacementRatio,
            "probe-executed-above-limit",
        );
        assert_eq!(above.status, Identifiability::Unknown);
        assert!(above.remaining.is_empty());
        assert!(above
            .eliminated
            .contains(&DiscrepancyKind::SupportFrictionInconsistent));
        assert!(above
            .eliminated
            .contains(&DiscrepancyKind::QuasiStaticAssumptionBroken));
        assert_eq!(
            above
                .belief
                .entry(PhysicalParameter::SupportFriction)
                .unwrap()
                .status,
            BeliefEpistemicStatus::DeclaredFact
        );
        assert_eq!(
            above
                .belief
                .entry(PhysicalParameter::QuasiStaticApplicability)
                .unwrap()
                .status,
            BeliefEpistemicStatus::DeclaredFact
        );
        assert_eq!(
            above
                .belief
                .declared_value(PhysicalParameter::QuasiStaticApplicability),
            Some(1.0)
        );
    }

    #[test]
    fn taxonomy_labels_are_distinct_and_are_not_recover() {
        let cases = [
            (
                TaxonomyFacts {
                    goal_unachievable: true,
                    ..TaxonomyFacts::none()
                },
                PhysicalReasoningOutcome::GoalCurrentlyUnachievable,
            ),
            (
                TaxonomyFacts {
                    already_unrecoverable: true,
                    ..TaxonomyFacts::none()
                },
                PhysicalReasoningOutcome::StateAlreadyUnrecoverable,
            ),
            (
                TaxonomyFacts {
                    execution_diverged: true,
                    ..TaxonomyFacts::none()
                },
                PhysicalReasoningOutcome::ExecutionDiverged,
            ),
            (
                TaxonomyFacts {
                    regime_changed: true,
                    ..TaxonomyFacts::none()
                },
                PhysicalReasoningOutcome::PhysicalRegimeChanged,
            ),
            (
                TaxonomyFacts {
                    model_contradicted: true,
                    ..TaxonomyFacts::none()
                },
                PhysicalReasoningOutcome::ModelContradicted,
            ),
            (
                TaxonomyFacts {
                    recoverability_at_risk: true,
                    ..TaxonomyFacts::none()
                },
                PhysicalReasoningOutcome::RecoverabilityAtRisk,
            ),
            (
                TaxonomyFacts {
                    safe_probe: Some(true),
                    ..TaxonomyFacts::none()
                },
                PhysicalReasoningOutcome::SafeProbeAvailable,
            ),
            (
                TaxonomyFacts {
                    safe_probe: Some(false),
                    ..TaxonomyFacts::none()
                },
                PhysicalReasoningOutcome::NoSafeProbe,
            ),
            (
                TaxonomyFacts {
                    parameter_uncertain: true,
                    ..TaxonomyFacts::none()
                },
                PhysicalReasoningOutcome::ParameterUncertain,
            ),
            (
                TaxonomyFacts {
                    action_failed: true,
                    ..TaxonomyFacts::none()
                },
                PhysicalReasoningOutcome::ActionFailed,
            ),
        ];
        let mut labels = Vec::new();
        for (facts, expected) in cases {
            let got = classify_reasoning_outcome(&facts);
            assert_eq!(got, expected);
            let label = serde_json::to_value(got).unwrap();
            assert_ne!(label, serde_json::json!("RECOVER"));
            labels.push(label);
        }
        for (i, a) in labels.iter().enumerate() {
            for b in labels.iter().skip(i + 1) {
                assert_ne!(a, b);
            }
        }
    }
}
