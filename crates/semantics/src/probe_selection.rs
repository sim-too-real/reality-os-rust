//! Goal action versus a safe physical probe.
//! Information gain is a count of distinguishable hypotheses, not a probability.

use serde::{Deserialize, Serialize};

use crate::discrepancy::{predicted_tag, DiscrepancyKind, ObservationTag, Stimulus};
use crate::planar_goal::GoalProgressClass;
use crate::recoverability::RecoverabilityClass;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum DecisionClass {
    GoalAction,
    PhysicalProbe,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum BeliefRobustness {
    RobustStrictProgress,
    ConditionalProgress,
    Ambiguous,
    RobustRegression,
    UnsafeForPartOfBeliefSet,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct DecisionCandidate {
    pub id: String,
    pub class: DecisionClass,
    pub stroke_m: f64,
    /// Higher is better immediate goal-error reduction. Probes may be zero.
    pub immediate_progress: f64,
    pub safe: bool,
    pub recoverability: RecoverabilityClass,
    pub recoverability_if_low_friction: RecoverabilityClass,
    pub progress_declared: GoalProgressClass,
    pub progress_if_low_friction: GoalProgressClass,
    pub shrinks_interval: bool,
    pub determines_contact_regime: bool,
    pub determines_quasi_static: bool,
    pub resolves_unknown_predicate: bool,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Ranking {
    pub selected_id: Option<String>,
    pub selected_class: Option<DecisionClass>,
    pub information_gain: u32,
    pub refused: Vec<(String, String)>,
    pub robustness: Vec<(String, BeliefRobustness)>,
}

pub fn structural_information_gain(tags: &[ObservationTag], extra: u32) -> u32 {
    let mut pairs = 0u32;
    for i in 0..tags.len() {
        for j in (i + 1)..tags.len() {
            if tags[i] != tags[j] {
                pairs += 1;
            }
        }
    }
    pairs.saturating_add(extra)
}

pub fn classify_robustness(
    endpoint_progress: &[GoalProgressClass],
    any_unsafe: bool,
) -> BeliefRobustness {
    if any_unsafe {
        return BeliefRobustness::UnsafeForPartOfBeliefSet;
    }
    if endpoint_progress.is_empty() {
        return BeliefRobustness::Ambiguous;
    }
    let all_strict = endpoint_progress
        .iter()
        .all(|p| *p == GoalProgressClass::StrictProgress);
    let all_regression = endpoint_progress
        .iter()
        .all(|p| *p == GoalProgressClass::Regression);
    let any_strict = endpoint_progress
        .iter()
        .any(|p| *p == GoalProgressClass::StrictProgress);
    let any_regression = endpoint_progress
        .iter()
        .any(|p| *p == GoalProgressClass::Regression);
    if all_strict {
        BeliefRobustness::RobustStrictProgress
    } else if all_regression {
        BeliefRobustness::RobustRegression
    } else if any_strict && any_regression {
        BeliefRobustness::ConditionalProgress
    } else {
        BeliefRobustness::Ambiguous
    }
}

fn robustness_of(
    candidate: &DecisionCandidate,
    live: &[DiscrepancyKind],
    limit_m: f64,
) -> BeliefRobustness {
    if live.len() >= 2 {
        return BeliefRobustness::Ambiguous;
    }
    if live.contains(&DiscrepancyKind::QuasiStaticAssumptionBroken) && candidate.stroke_m > limit_m
    {
        return BeliefRobustness::UnsafeForPartOfBeliefSet;
    }
    if live.contains(&DiscrepancyKind::SupportFrictionInconsistent) {
        let unsafe_low = matches!(
            candidate.recoverability_if_low_friction,
            RecoverabilityClass::ProgressButCanEnterUnrecoverableState
                | RecoverabilityClass::PhysicallyInfeasible
        );
        return classify_robustness(
            &[
                candidate.progress_declared,
                candidate.progress_if_low_friction,
            ],
            unsafe_low,
        );
    }
    classify_robustness(&[candidate.progress_declared], false)
}

fn effective_recoverability(
    candidate: &DecisionCandidate,
    live: &[DiscrepancyKind],
) -> RecoverabilityClass {
    if live.contains(&DiscrepancyKind::SupportFrictionInconsistent) {
        candidate.recoverability_if_low_friction
    } else {
        candidate.recoverability
    }
}

pub fn rank_goal_or_probe(
    candidates: &[DecisionCandidate],
    live: &[DiscrepancyKind],
    quasi_static_limit_m: f64,
) -> Ranking {
    let mut refused = Vec::new();
    let mut robustness = Vec::new();
    let mut best_goal: Option<(usize, f64)> = None;
    let mut best_probe: Option<(usize, u32)> = None;
    for (index, candidate) in candidates.iter().enumerate() {
        let stimulus = Stimulus {
            stroke_m: candidate.stroke_m,
            quasi_static_stroke_limit_m: quasi_static_limit_m,
        };
        let tags: Vec<ObservationTag> = live
            .iter()
            .map(|kind| predicted_tag(*kind, stimulus))
            .collect();
        let extra = u32::from(candidate.shrinks_interval)
            + u32::from(candidate.determines_contact_regime)
            + u32::from(candidate.determines_quasi_static)
            + u32::from(candidate.resolves_unknown_predicate);
        let gain = structural_information_gain(&tags, extra);
        if !candidate.safe {
            refused.push((candidate.id.clone(), "UNSAFE".into()));
            continue;
        }
        if candidate.recoverability == RecoverabilityClass::PhysicallyInfeasible
            || effective_recoverability(candidate, live)
                == RecoverabilityClass::PhysicallyInfeasible
        {
            refused.push((candidate.id.clone(), "PHYSICALLY_INFEASIBLE".into()));
            continue;
        }
        match candidate.class {
            DecisionClass::GoalAction => {
                let robust = robustness_of(candidate, live, quasi_static_limit_m);
                robustness.push((candidate.id.clone(), robust));
                if robust != BeliefRobustness::RobustStrictProgress {
                    refused.push((candidate.id.clone(), format!("{robust:?}")));
                    continue;
                }
                let recovered = effective_recoverability(candidate, live);
                if recovered != RecoverabilityClass::ProgressAndRecoverable {
                    refused.push((candidate.id.clone(), format!("{recovered:?}")));
                    continue;
                }
                let better = best_goal
                    .map(|(_, progress)| candidate.immediate_progress > progress)
                    .unwrap_or(true);
                if better {
                    best_goal = Some((index, candidate.immediate_progress));
                }
            }
            DecisionClass::PhysicalProbe => {
                if matches!(
                    effective_recoverability(candidate, live),
                    RecoverabilityClass::ProgressButCanEnterUnrecoverableState
                        | RecoverabilityClass::PhysicallyInfeasible
                ) {
                    refused.push((candidate.id.clone(), "UNSAFE".into()));
                    continue;
                }
                if gain == 0 {
                    refused.push((candidate.id.clone(), "NO_INFORMATION".into()));
                    continue;
                }
                let better = match best_probe {
                    None => true,
                    Some((_, best_gain)) => gain > best_gain,
                };
                if better {
                    best_probe = Some((index, gain));
                }
            }
        }
    }
    if let Some((index, _)) = best_goal {
        return Ranking {
            selected_id: Some(candidates[index].id.clone()),
            selected_class: Some(DecisionClass::GoalAction),
            information_gain: 0,
            refused,
            robustness,
        };
    }
    if let Some((index, gain)) = best_probe {
        return Ranking {
            selected_id: Some(candidates[index].id.clone()),
            selected_class: Some(DecisionClass::PhysicalProbe),
            information_gain: gain,
            refused,
            robustness,
        };
    }
    Ranking {
        selected_id: None,
        selected_class: None,
        information_gain: 0,
        refused,
        robustness,
    }
}

/// A goal contact whose declared-friction prediction is in doubt.
///
/// The low-friction recoverability endpoint is the geometric class already
/// proved for that contact. It is not replaced with a constant
/// unrecoverable label.
pub fn goal_contact_at_contradicted_declared_friction(
    id: impl Into<String>,
    stroke_m: f64,
    immediate_progress: f64,
    progress_at_declared: GoalProgressClass,
    recoverability_at_declared: RecoverabilityClass,
    executable: bool,
) -> DecisionCandidate {
    DecisionCandidate {
        id: id.into(),
        class: DecisionClass::GoalAction,
        stroke_m,
        immediate_progress,
        safe: executable,
        recoverability: recoverability_at_declared,
        recoverability_if_low_friction: recoverability_at_declared,
        progress_declared: progress_at_declared,
        progress_if_low_friction: progress_at_declared,
        shrinks_interval: false,
        determines_contact_regime: false,
        determines_quasi_static: false,
        resolves_unknown_predicate: false,
    }
}

/// Same geometry, same goal, candidates distinguished only by stroke scale
/// relative to the quasi-static limit.
pub fn candidates_for_uncertainty(quasi_static_limit_m: f64) -> Vec<DecisionCandidate> {
    let limit = quasi_static_limit_m;
    let recoverable = RecoverabilityClass::ProgressAndRecoverable;
    let can_leave = RecoverabilityClass::ProgressButCanEnterUnrecoverableState;
    vec![
        goal(
            "goal_large",
            limit * 2.5,
            0.9,
            recoverable,
            can_leave,
            GoalProgressClass::StrictProgress,
            GoalProgressClass::StrictProgress,
        ),
        goal(
            "goal_small",
            limit * 0.5,
            0.4,
            recoverable,
            can_leave,
            GoalProgressClass::StrictProgress,
            GoalProgressClass::StrictProgress,
        ),
        goal(
            "goal_guarded_short",
            limit * 0.25,
            0.2,
            recoverable,
            recoverable,
            GoalProgressClass::StrictProgress,
            GoalProgressClass::StrictProgress,
        ),
        probe("probe_separating", limit * 0.4, 0.0, true),
        probe("probe_repeat", limit * 3.0, 0.05, true),
        probe("probe_unsafe", limit * 0.4, 0.0, false),
    ]
}

fn goal(
    id: &str,
    stroke_m: f64,
    progress: f64,
    recoverability: RecoverabilityClass,
    low_friction: RecoverabilityClass,
    declared: GoalProgressClass,
    low: GoalProgressClass,
) -> DecisionCandidate {
    DecisionCandidate {
        id: id.into(),
        class: DecisionClass::GoalAction,
        stroke_m,
        immediate_progress: progress,
        safe: true,
        recoverability,
        recoverability_if_low_friction: low_friction,
        progress_declared: declared,
        progress_if_low_friction: low,
        shrinks_interval: false,
        determines_contact_regime: false,
        determines_quasi_static: false,
        resolves_unknown_predicate: false,
    }
}

fn probe(id: &str, stroke_m: f64, progress: f64, safe: bool) -> DecisionCandidate {
    DecisionCandidate {
        id: id.into(),
        class: DecisionClass::PhysicalProbe,
        stroke_m,
        immediate_progress: progress,
        safe,
        recoverability: RecoverabilityClass::NoProgress,
        recoverability_if_low_friction: RecoverabilityClass::NoProgress,
        progress_declared: GoalProgressClass::Neutral,
        progress_if_low_friction: GoalProgressClass::Neutral,
        shrinks_interval: false,
        determines_contact_regime: false,
        determines_quasi_static: false,
        resolves_unknown_predicate: false,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::discrepancy::{
        apply_probe_observation, hypothesize, DiscrepancyObservation, Identifiability,
        ObservationTag, Stimulus,
    };
    use crate::physical_belief::{PhysicalParameter, PhysicalParameterBelief};

    const LIMIT: f64 = 0.015;

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
            stroke_m: LIMIT * 2.5,
            quasi_static_stroke_limit_m: LIMIT,
            contradictory: false,
            reachable: true,
        }
    }

    #[test]
    fn separating_probe_beats_higher_progress_and_unsafe_or_empty_probes() {
        let candidates = candidates_for_uncertainty(LIMIT);
        let report = hypothesize(&large_slip());
        assert_eq!(report.status, Identifiability::Underdetermined);
        let ranking = rank_goal_or_probe(&candidates, &report.kinds, LIMIT);
        assert_eq!(ranking.selected_class, Some(DecisionClass::PhysicalProbe));
        assert_eq!(ranking.selected_id.as_deref(), Some("probe_separating"));
        assert_eq!(ranking.information_gain, 1);
        let goal_large = candidates.iter().find(|c| c.id == "goal_large").unwrap();
        let probe = candidates
            .iter()
            .find(|c| c.id == "probe_separating")
            .unwrap();
        assert!(goal_large.immediate_progress > probe.immediate_progress);
        assert!(ranking
            .refused
            .iter()
            .any(|(id, reason)| id == "probe_unsafe" && reason == "UNSAFE"));
        assert!(ranking
            .refused
            .iter()
            .any(|(id, reason)| id == "probe_repeat" && reason == "NO_INFORMATION"));
        let optimistic = DecisionCandidate {
            id: "goal_optimistic".into(),
            class: DecisionClass::GoalAction,
            stroke_m: LIMIT * 0.5,
            immediate_progress: 5.0,
            safe: true,
            recoverability: RecoverabilityClass::ProgressAndRecoverable,
            recoverability_if_low_friction: RecoverabilityClass::ProgressAndRecoverable,
            progress_declared: GoalProgressClass::StrictProgress,
            progress_if_low_friction: GoalProgressClass::Regression,
            shrinks_interval: false,
            determines_contact_regime: false,
            determines_quasi_static: false,
            resolves_unknown_predicate: false,
        };
        let mut with_optimistic = candidates.clone();
        with_optimistic.push(optimistic);
        let again = rank_goal_or_probe(
            &with_optimistic,
            &[DiscrepancyKind::SupportFrictionInconsistent],
            LIMIT,
        );
        assert_ne!(again.selected_id.as_deref(), Some("goal_optimistic"));
        assert!(again.refused.iter().any(|(id, _)| id == "goal_optimistic"));
    }

    #[test]
    fn two_geometries_share_the_first_observation_then_diverge_after_evidence() {
        let candidates = candidates_for_uncertainty(LIMIT);
        let belief = PhysicalParameterBelief::declared_point(
            PhysicalParameter::SupportFriction,
            0.5,
            "scene.mu",
        )
        .with_unknown(PhysicalParameter::ObjectMassKg, "mass")
        .with_unknown(PhysicalParameter::QuasiStaticApplicability, "quasi_static");
        // Declared applicability is a point after the unknown push overwrote nothing;
        // add an explicit declared quasi-static entry by contradict path later.
        let mut belief = belief;
        belief
            .parameters
            .push(crate::physical_belief::ParameterBelief {
                parameter: PhysicalParameter::QuasiStaticApplicability,
                status: crate::physical_belief::BeliefEpistemicStatus::DeclaredFact,
                declared: crate::provenance::Provenanced::declared(1.0, "regime", 0.0),
                empirical_interval: None,
                lineage: Vec::new(),
            });
        let report = hypothesize(&large_slip());
        let before = rank_goal_or_probe(&candidates, &report.kinds, LIMIT);
        let stimulus = Stimulus {
            stroke_m: LIMIT * 0.4,
            quasi_static_stroke_limit_m: LIMIT,
        };
        let low_friction = apply_probe_observation(
            &belief,
            &report.kinds,
            stimulus,
            ObservationTag::HighDisplacementRatio,
            "probe-low-friction",
        );
        let dynamic = apply_probe_observation(
            &belief,
            &report.kinds,
            stimulus,
            ObservationTag::NominalDisplacementRatio,
            "probe-dynamic",
        );
        let after_friction = rank_goal_or_probe(&candidates, &low_friction.remaining, LIMIT);
        let after_dynamic = rank_goal_or_probe(&candidates, &dynamic.remaining, LIMIT);
        let first_again = rank_goal_or_probe(&candidates, &report.kinds, LIMIT);
        assert_eq!(first_again.selected_id, before.selected_id);
        assert_eq!(
            first_again.selected_class,
            Some(DecisionClass::PhysicalProbe)
        );
        assert_ne!(low_friction.remaining, dynamic.remaining);
        assert_ne!(after_friction.selected_id, after_dynamic.selected_id);
        assert_ne!(after_friction.selected_id, before.selected_id);
        assert_ne!(after_dynamic.selected_id, before.selected_id);
        assert_eq!(
            low_friction
                .belief
                .declared_value(PhysicalParameter::SupportFriction),
            Some(0.5)
        );
        assert_eq!(
            dynamic
                .belief
                .declared_value(PhysicalParameter::SupportFriction),
            Some(0.5)
        );
        assert_eq!(
            low_friction
                .belief
                .entry(PhysicalParameter::ObjectMassKg)
                .unwrap()
                .status,
            crate::physical_belief::BeliefEpistemicStatus::Unknown
        );
        assert!(low_friction
            .belief
            .entry(PhysicalParameter::SupportFriction)
            .unwrap()
            .lineage
            .iter()
            .any(|line| line.observation == "probe-low-friction"));
        assert!(dynamic
            .belief
            .entry(PhysicalParameter::SupportFriction)
            .unwrap()
            .lineage
            .iter()
            .any(|line| line.observation == "probe-dynamic"));
        assert_eq!(
            after_dynamic.selected_class,
            Some(DecisionClass::GoalAction)
        );
        assert_eq!(after_dynamic.selected_id.as_deref(), Some("goal_small"));
        assert_eq!(
            after_friction.selected_id.as_deref(),
            Some("goal_guarded_short")
        );
    }

    #[test]
    fn recoverable_short_contact_is_robust_under_live_friction() {
        let short = goal_contact_at_contradicted_declared_friction(
            "qs:0.0040:-x:-0.55",
            0.004,
            1.0,
            GoalProgressClass::StrictProgress,
            RecoverabilityClass::ProgressAndRecoverable,
            true,
        );
        assert_eq!(
            short.recoverability_if_low_friction,
            RecoverabilityClass::ProgressAndRecoverable
        );
        let ranking = rank_goal_or_probe(
            &[short],
            &[DiscrepancyKind::SupportFrictionInconsistent],
            LIMIT,
        );
        assert_eq!(ranking.selected_id.as_deref(), Some("qs:0.0040:-x:-0.55"));
        assert_eq!(ranking.selected_class, Some(DecisionClass::GoalAction));
        assert!(ranking.robustness.iter().any(|(id, robust)| {
            id == "qs:0.0040:-x:-0.55" && *robust == BeliefRobustness::RobustStrictProgress
        }));
        let long = goal_contact_at_contradicted_declared_friction(
            "long",
            0.03,
            2.0,
            GoalProgressClass::StrictProgress,
            RecoverabilityClass::ProgressButCanEnterUnrecoverableState,
            true,
        );
        assert_eq!(
            long.recoverability_if_low_friction,
            RecoverabilityClass::ProgressButCanEnterUnrecoverableState
        );
        let refused = rank_goal_or_probe(
            &[long],
            &[DiscrepancyKind::SupportFrictionInconsistent],
            LIMIT,
        );
        assert_eq!(refused.selected_id, None);
        assert!(refused
            .refused
            .iter()
            .any(|(id, reason)| { id == "long" && reason == "UnsafeForPartOfBeliefSet" }));
    }
}
