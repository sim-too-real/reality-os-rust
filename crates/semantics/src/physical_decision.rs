//! One semantic authority for choosing a goal interaction or physical probe.
//!
//! This module ranks only identity-bound actions for which an executable witness,
//! authority result, robustness assessment, and recoverability result already
//! exist. It cannot write to a plant or manufacture any of those proofs.

use serde::{Deserialize, Serialize};

use crate::planar_goal::GoalProgressClass;
use crate::probe_selection::BeliefRobustness;
use crate::recoverability::RecoverabilityClass;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum CandidateRole {
    GoalAction,
    PhysicalProbe,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum CandidateRejection {
    GeometryInvalid,
    CollisionInadmissible,
    Unreachable,
    WitnessUnavailable,
    MechanicsUnknown,
    AuthorityRefused,
    Forbidden,
    Unsafe,
    ObservationUnavailable,
    Other(String),
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct LexicographicPreference {
    /// Lower is better. Required for a goal action.
    pub error_derivative: Option<f64>,
    /// Lower is preferred after error derivative.
    pub angular_rate_abs: Option<f64>,
    /// Lower is preferred after angular rate.
    pub contact_offset_abs_m: Option<f64>,
    /// Lower is preferred when comparing equally informative probes.
    pub stroke_m: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct ProbeEvidence {
    /// Count of live physical explanations whose predicted observations differ.
    pub decision_relevant_distinctions: u32,
    /// Count that the declared observation channel can distinguish.
    pub observable_distinctions: u32,
    /// A bounded-state proof says the experiment preserves future interaction
    /// capability; no goal progress is required from an information action.
    pub future_interaction: ProbeRecoverabilityAssessment,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum ProbeRecoverabilityAssessment {
    Preserved,
    AtRisk,
    Unknown,
}

/// Physical and goal effect predicted for one proved witness. Unsupported
/// magnitudes stay unknown until a calibrated mechanics model supplies them.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct PredictedPhysicalEffect {
    pub object_translation_world_m: Option<[f64; 2]>,
    pub yaw_change_rad: Option<f64>,
    pub contact_persists: Option<bool>,
    pub goal_error_derivative: Option<f64>,
    pub goal_progress: Option<GoalProgressClass>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CandidateEvidence {
    pub candidate_id: String,
    /// Serialized identity-bearing candidate snapshot used by later freeze/execution checks.
    pub candidate_contents: String,
    pub action_key: String,
    pub contact_id: String,
    pub role: CandidateRole,
    pub strict_goal_progress: bool,
    pub authority_ok: bool,
    /// Stable identity of the independently-produced executable witness.
    pub executable_witness_id: Option<String>,
    /// Stable identity of the exact witness contents frozen for execution.
    pub witness_digest: Option<String>,
    /// Exact serialized witness contents associated with `witness_digest`.
    pub witness_contents: Option<String>,
    pub robustness: BeliefRobustness,
    pub recoverability: RecoverabilityClass,
    pub predicted_effect: PredictedPhysicalEffect,
    pub probe: ProbeEvidence,
    pub preference: LexicographicPreference,
    /// Hard failures accumulated by geometry, mechanics, and execution proof.
    pub hard_rejections: Vec<CandidateRejection>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct DecisionContext {
    pub goal_id: String,
    pub goal_reached: bool,
    pub evidence_fresh: bool,
    pub remaining_attempts: u32,
    pub current_contact_id: Option<String>,
    pub forbidden_action_keys: Vec<String>,
    pub candidates: Vec<CandidateEvidence>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum ContactTransitionPhase {
    LeaveCurrentContact,
    Reobserve,
    RecheckCollisionAndWitness,
    ApproachNewContact,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SelectedPhysicalAction {
    pub goal_id: String,
    pub candidate_id: String,
    pub candidate_contents: String,
    pub action_key: String,
    pub contact_id: String,
    pub witness_id: String,
    pub witness_digest: String,
    pub witness_contents: String,
    pub role: CandidateRole,
    pub predicted_effect: PredictedPhysicalEffect,
    pub robustness: BeliefRobustness,
    pub recoverability: RecoverabilityClass,
    pub probe: ProbeEvidence,
    pub rationale: String,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum DecisionKind {
    GoalInteraction {
        action: SelectedPhysicalAction,
    },
    PhysicalProbe {
        action: SelectedPhysicalAction,
        decision_relevant_distinctions: u32,
    },
    ContactTransition {
        action: SelectedPhysicalAction,
        phases: Vec<ContactTransitionPhase>,
    },
    GoalReached {
        goal_id: String,
    },
    Refuse {
        reason: String,
    },
    InsufficientEvidence {
        reason: String,
    },
    PhysicallyInfeasible {
        reason: String,
    },
    CurrentlyUnachievable {
        reason: String,
    },
}

#[derive(Debug, Clone, PartialEq, Eq)]
enum Eligibility {
    Eligible,
    Rejected,
    AuthorityRefused,
    EvidenceUnavailable,
    Infeasible,
    ProbeRecoverabilityUnknown,
}

fn eligibility(candidate: &CandidateEvidence, context: &DecisionContext) -> Eligibility {
    // Hard rejections always precede class membership and preferences, while
    // preserving whether the missing proof is evidence, authority, or safety.
    if context
        .forbidden_action_keys
        .iter()
        .any(|key| key == &candidate.action_key)
    {
        return Eligibility::Rejected;
    }
    let mut rejection = Eligibility::Rejected;
    for hard_rejection in &candidate.hard_rejections {
        let classified = match hard_rejection {
            CandidateRejection::AuthorityRefused => Eligibility::AuthorityRefused,
            CandidateRejection::WitnessUnavailable
            | CandidateRejection::MechanicsUnknown
            | CandidateRejection::ObservationUnavailable => Eligibility::EvidenceUnavailable,
            CandidateRejection::GeometryInvalid
            | CandidateRejection::CollisionInadmissible
            | CandidateRejection::Unreachable
            | CandidateRejection::Unsafe => Eligibility::Infeasible,
            CandidateRejection::Forbidden | CandidateRejection::Other(_) => Eligibility::Rejected,
        };
        rejection = match (rejection, classified) {
            (Eligibility::AuthorityRefused, _) | (_, Eligibility::AuthorityRefused) => {
                Eligibility::AuthorityRefused
            }
            (Eligibility::EvidenceUnavailable, _) | (_, Eligibility::EvidenceUnavailable) => {
                Eligibility::EvidenceUnavailable
            }
            (Eligibility::Infeasible, _) | (_, Eligibility::Infeasible) => Eligibility::Infeasible,
            _ => Eligibility::Rejected,
        };
    }
    if rejection != Eligibility::Rejected {
        return rejection;
    }
    if !candidate.authority_ok {
        return Eligibility::AuthorityRefused;
    }
    if !context.evidence_fresh {
        return Eligibility::EvidenceUnavailable;
    }
    if candidate
        .executable_witness_id
        .as_deref()
        .map_or(true, str::is_empty)
        || candidate
            .witness_digest
            .as_deref()
            .map_or(true, str::is_empty)
        || candidate
            .witness_contents
            .as_deref()
            .map_or(true, str::is_empty)
        || candidate.candidate_contents.is_empty()
    {
        return Eligibility::EvidenceUnavailable;
    }
    if matches!(
        candidate.recoverability,
        RecoverabilityClass::PhysicallyInfeasible
            | RecoverabilityClass::ProgressButCanEnterUnrecoverableState
    ) || candidate.robustness == BeliefRobustness::UnsafeForPartOfBeliefSet
    {
        return Eligibility::Infeasible;
    }
    match candidate.role {
        CandidateRole::GoalAction => {
            if candidate.robustness == BeliefRobustness::Ambiguous
                || candidate.recoverability == RecoverabilityClass::ProgressButRecoverabilityUnknown
            {
                return Eligibility::EvidenceUnavailable;
            }
            if !candidate.strict_goal_progress
                || candidate.robustness != BeliefRobustness::RobustStrictProgress
                || candidate.recoverability != RecoverabilityClass::ProgressAndRecoverable
                || !candidate
                    .preference
                    .error_derivative
                    .is_some_and(f64::is_finite)
            {
                return Eligibility::Rejected;
            }
        }
        CandidateRole::PhysicalProbe => {
            if !matches!(
                candidate.recoverability,
                RecoverabilityClass::NoProgress | RecoverabilityClass::ProgressAndRecoverable
            ) || candidate.probe.decision_relevant_distinctions == 0
                || candidate.probe.observable_distinctions == 0
            {
                return Eligibility::Rejected;
            }
            match candidate.probe.future_interaction {
                ProbeRecoverabilityAssessment::Preserved => {}
                ProbeRecoverabilityAssessment::AtRisk => return Eligibility::Infeasible,
                ProbeRecoverabilityAssessment::Unknown => {
                    return Eligibility::ProbeRecoverabilityUnknown;
                }
            }
        }
    }
    Eligibility::Eligible
}

fn finite_or_inf(value: Option<f64>) -> f64 {
    value.filter(|v| v.is_finite()).unwrap_or(f64::INFINITY)
}

fn selected_action(
    context: &DecisionContext,
    candidate: &CandidateEvidence,
    rationale: String,
) -> SelectedPhysicalAction {
    SelectedPhysicalAction {
        goal_id: context.goal_id.clone(),
        candidate_id: candidate.candidate_id.clone(),
        candidate_contents: candidate.candidate_contents.clone(),
        action_key: candidate.action_key.clone(),
        contact_id: candidate.contact_id.clone(),
        witness_id: candidate
            .executable_witness_id
            .clone()
            .expect("eligible action has an executable witness identity"),
        witness_digest: candidate
            .witness_digest
            .clone()
            .expect("eligible action has a witness content identity"),
        witness_contents: candidate
            .witness_contents
            .clone()
            .expect("eligible action has frozen witness contents"),
        role: candidate.role,
        predicted_effect: candidate.predicted_effect,
        robustness: candidate.robustness,
        recoverability: candidate.recoverability,
        probe: candidate.probe,
        rationale,
    }
}

fn contact_transition(context: &DecisionContext, action: SelectedPhysicalAction) -> DecisionKind {
    if context.current_contact_id.as_deref() != Some(action.contact_id.as_str()) {
        let mut phases = Vec::new();
        if context.current_contact_id.is_some() {
            phases.push(ContactTransitionPhase::LeaveCurrentContact);
        }
        phases.extend([
            ContactTransitionPhase::Reobserve,
            ContactTransitionPhase::RecheckCollisionAndWitness,
            ContactTransitionPhase::ApproachNewContact,
        ]);
        DecisionKind::ContactTransition { action, phases }
    } else if action.role == CandidateRole::PhysicalProbe {
        DecisionKind::PhysicalProbe {
            action,
            decision_relevant_distinctions: 1,
        }
    } else {
        DecisionKind::GoalInteraction { action }
    }
}

/// Select exactly one already-proved physical action using hard gates followed
/// by class tiers and deterministic lexicographic preferences. No plant I/O.
pub fn decide_physical_action(context: &DecisionContext) -> DecisionKind {
    if context.goal_reached {
        return DecisionKind::GoalReached {
            goal_id: context.goal_id.clone(),
        };
    }
    if context.remaining_attempts == 0 {
        return DecisionKind::CurrentlyUnachievable {
            reason: "PHYSICAL_INTERACTION_BUDGET_EXHAUSTED".into(),
        };
    }
    if !context.evidence_fresh {
        return DecisionKind::InsufficientEvidence {
            reason: "POLICY_OBSERVATION_STALE_OR_UNAVAILABLE".into(),
        };
    }

    let mut eligible_goals = Vec::new();
    let mut eligible_probes = Vec::new();
    let mut authority_refused = false;
    let mut evidence_unavailable = false;
    let mut infeasible = false;
    let mut probe_recoverability_unknown = false;
    for (index, candidate) in context.candidates.iter().enumerate() {
        match eligibility(candidate, context) {
            Eligibility::Eligible => match candidate.role {
                CandidateRole::GoalAction => eligible_goals.push(index),
                CandidateRole::PhysicalProbe => eligible_probes.push(index),
            },
            Eligibility::AuthorityRefused => authority_refused = true,
            Eligibility::EvidenceUnavailable => evidence_unavailable = true,
            Eligibility::Infeasible => infeasible = true,
            Eligibility::ProbeRecoverabilityUnknown => probe_recoverability_unknown = true,
            Eligibility::Rejected => {}
        }
    }

    if let Some(index) = eligible_goals.into_iter().min_by(|a, b| {
        let a = &context.candidates[*a];
        let b = &context.candidates[*b];
        finite_or_inf(a.preference.error_derivative)
            .total_cmp(&finite_or_inf(b.preference.error_derivative))
            .then_with(|| {
                finite_or_inf(a.preference.angular_rate_abs)
                    .total_cmp(&finite_or_inf(b.preference.angular_rate_abs))
            })
            .then_with(|| {
                finite_or_inf(a.preference.contact_offset_abs_m)
                    .total_cmp(&finite_or_inf(b.preference.contact_offset_abs_m))
            })
            .then_with(|| a.candidate_id.cmp(&b.candidate_id))
            .then_with(|| a.action_key.cmp(&b.action_key))
    }) {
        let candidate = &context.candidates[index];
        let action = selected_action(
            context,
            candidate,
            "ROBUST_STRICT_PROGRESS_AND_RECOVERABLE".into(),
        );
        return contact_transition(context, action);
    }

    if let Some(index) = eligible_probes.into_iter().max_by(|a, b| {
        let a = &context.candidates[*a];
        let b = &context.candidates[*b];
        a.probe
            .decision_relevant_distinctions
            .min(a.probe.observable_distinctions)
            .cmp(
                &b.probe
                    .decision_relevant_distinctions
                    .min(b.probe.observable_distinctions),
            )
            .then_with(|| {
                finite_or_inf(b.preference.stroke_m.into())
                    .total_cmp(&finite_or_inf(a.preference.stroke_m.into()))
            })
            .then_with(|| b.candidate_id.cmp(&a.candidate_id))
    }) {
        let candidate = &context.candidates[index];
        let gain = candidate
            .probe
            .decision_relevant_distinctions
            .min(candidate.probe.observable_distinctions);
        let action = selected_action(
            context,
            candidate,
            "SAFE_OBSERVABLE_INFORMATION_GAIN".into(),
        );
        let decision = contact_transition(context, action);
        return match decision {
            DecisionKind::ContactTransition { action, phases } => {
                DecisionKind::ContactTransition { action, phases }
            }
            _ => DecisionKind::PhysicalProbe {
                action: selected_action(
                    context,
                    candidate,
                    "SAFE_OBSERVABLE_INFORMATION_GAIN".into(),
                ),
                decision_relevant_distinctions: gain,
            },
        };
    }

    if probe_recoverability_unknown {
        return DecisionKind::InsufficientEvidence {
            reason: "PROBE_FUTURE_INTERACTION_UNPROVEN".into(),
        };
    }

    if authority_refused {
        DecisionKind::Refuse {
            reason: "AUTHORITY_REFUSED_ALL_REMAINING_PHYSICAL_ACTIONS".into(),
        }
    } else if evidence_unavailable {
        DecisionKind::InsufficientEvidence {
            reason: "EXECUTABLE_WITNESS_OR_REQUIRED_OBSERVATION_UNAVAILABLE".into(),
        }
    } else if infeasible {
        DecisionKind::PhysicallyInfeasible {
            reason: "ALL_REMAINING_PHYSICAL_ACTIONS_ARE_UNSAFE_OR_UNRECOVERABLE".into(),
        }
    } else if context.candidates.is_empty() {
        DecisionKind::CurrentlyUnachievable {
            reason: "NO_PHYSICAL_CANDIDATES".into(),
        }
    } else {
        DecisionKind::CurrentlyUnachievable {
            reason: "NO_JUSTIFIED_GOAL_ACTION_OR_SAFE_OBSERVABLE_PROBE".into(),
        }
    }
}
