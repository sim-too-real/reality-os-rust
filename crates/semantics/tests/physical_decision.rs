use realityos_semantics::physical_decision::{
    decide_physical_action, CandidateEvidence, CandidateRole, DecisionContext, DecisionKind,
    LexicographicPreference, PredictedPhysicalEffect, ProbeEvidence, ProbeRecoverabilityAssessment,
};
use realityos_semantics::planar_goal::GoalProgressClass;
use realityos_semantics::probe_selection::BeliefRobustness;
use realityos_semantics::recoverability::RecoverabilityClass;

fn candidate(id: &str, role: CandidateRole) -> CandidateEvidence {
    CandidateEvidence {
        candidate_id: id.into(),
        candidate_contents: format!("candidate-snapshot:{id}"),
        action_key: format!("face:{id}"),
        contact_id: format!("contact:{id}"),
        role,
        strict_goal_progress: role == CandidateRole::GoalAction,
        authority_ok: true,
        executable_witness_id: Some(format!("witness:{id}")),
        witness_digest: Some(format!("digest:{id}")),
        witness_contents: Some(format!("witness-content:{id}")),
        robustness: BeliefRobustness::RobustStrictProgress,
        recoverability: if role == CandidateRole::PhysicalProbe {
            RecoverabilityClass::NoProgress
        } else {
            RecoverabilityClass::ProgressAndRecoverable
        },
        predicted_effect: PredictedPhysicalEffect {
            object_translation_world_m: None,
            yaw_change_rad: None,
            contact_persists: None,
            goal_error_derivative: Some(-0.1),
            goal_progress: (role == CandidateRole::GoalAction)
                .then_some(GoalProgressClass::StrictProgress),
        },
        probe: ProbeEvidence {
            decision_relevant_distinctions: 0,
            observable_distinctions: 0,
            future_interaction: if role == CandidateRole::PhysicalProbe {
                ProbeRecoverabilityAssessment::Preserved
            } else {
                ProbeRecoverabilityAssessment::Unknown
            },
        },
        preference: LexicographicPreference {
            error_derivative: Some(-0.1),
            angular_rate_abs: Some(0.0),
            contact_offset_abs_m: Some(0.0),
            stroke_m: 0.02,
        },
        hard_rejections: vec![],
    }
}

fn context(candidates: Vec<CandidateEvidence>) -> DecisionContext {
    DecisionContext {
        goal_id: "goal:box-to-region".into(),
        goal_reached: false,
        evidence_fresh: true,
        remaining_attempts: 2,
        current_contact_id: Some("contact:old".into()),
        forbidden_action_keys: vec![],
        candidates,
    }
}

#[test]
fn forbidden_keys_are_rejected_for_goal_and_probe_classes() {
    for role in [CandidateRole::GoalAction, CandidateRole::PhysicalProbe] {
        let mut forbidden = candidate("same", role);
        forbidden.strict_goal_progress = true;
        forbidden.probe = ProbeEvidence {
            decision_relevant_distinctions: 1,
            observable_distinctions: 1,
            future_interaction: ProbeRecoverabilityAssessment::Preserved,
        };
        let mut cx = context(vec![forbidden.clone()]);
        cx.forbidden_action_keys.push(forbidden.action_key);
        assert!(!matches!(
            decide_physical_action(&cx),
            DecisionKind::GoalInteraction { .. }
                | DecisionKind::PhysicalProbe { .. }
                | DecisionKind::ContactTransition { .. }
        ));
    }
}

#[test]
fn a_probe_cannot_resurrect_an_authority_ineligible_or_unwitnessed_action() {
    let mut no_authority = candidate("probe", CandidateRole::PhysicalProbe);
    no_authority.authority_ok = false;
    no_authority.probe = ProbeEvidence {
        decision_relevant_distinctions: 2,
        observable_distinctions: 2,
        future_interaction: ProbeRecoverabilityAssessment::Preserved,
    };
    let mut no_witness = candidate("no-witness", CandidateRole::PhysicalProbe);
    no_witness.executable_witness_id = None;
    no_witness.witness_contents = None;
    no_witness.probe = no_authority.probe;
    let result = decide_physical_action(&context(vec![no_authority, no_witness]));
    assert!(matches!(result, DecisionKind::Refuse { .. }));
}

#[test]
fn robust_recoverable_goal_progress_precedes_every_probe() {
    let goal = candidate("goal", CandidateRole::GoalAction);
    let mut probe = candidate("probe", CandidateRole::PhysicalProbe);
    probe.strict_goal_progress = false;
    probe.probe = ProbeEvidence {
        decision_relevant_distinctions: 3,
        observable_distinctions: 2,
        future_interaction: ProbeRecoverabilityAssessment::Preserved,
    };
    assert!(matches!(
        decide_physical_action(&context(vec![probe, goal])),
        DecisionKind::ContactTransition { action, .. }
            | DecisionKind::GoalInteraction { action, .. } if action.candidate_id == "goal"
    ));
}

#[test]
fn probe_requires_positive_decision_relevant_observable_distinctions() {
    let mut probe = candidate("probe", CandidateRole::PhysicalProbe);
    probe.strict_goal_progress = false;
    probe.probe = ProbeEvidence {
        decision_relevant_distinctions: 1,
        observable_distinctions: 0,
        future_interaction: ProbeRecoverabilityAssessment::Preserved,
    };
    assert!(!matches!(
        decide_physical_action(&context(vec![probe])),
        DecisionKind::PhysicalProbe { .. } | DecisionKind::ContactTransition { .. }
    ));
}

#[test]
fn probe_requires_a_separate_future_interaction_safety_proof() {
    let mut probe = candidate("probe", CandidateRole::PhysicalProbe);
    probe.probe = ProbeEvidence {
        decision_relevant_distinctions: 2,
        observable_distinctions: 2,
        future_interaction: ProbeRecoverabilityAssessment::Unknown,
    };
    assert!(matches!(
        decide_physical_action(&context(vec![probe.clone()])),
        DecisionKind::InsufficientEvidence { reason }
            if reason == "PROBE_FUTURE_INTERACTION_UNPROVEN"
    ));
    probe.probe.future_interaction = ProbeRecoverabilityAssessment::AtRisk;
    assert!(!matches!(
        decide_physical_action(&context(vec![probe])),
        DecisionKind::PhysicalProbe { .. } | DecisionKind::ContactTransition { .. }
    ));
}

#[test]
fn missing_scoped_authority_refuses_a_probe_before_safety_ranking() {
    let mut probe = candidate("unscoped-probe", CandidateRole::PhysicalProbe);
    probe.authority_ok = false;
    probe.probe = ProbeEvidence {
        decision_relevant_distinctions: 2,
        observable_distinctions: 2,
        future_interaction: ProbeRecoverabilityAssessment::Unknown,
    };

    assert!(matches!(
        decide_physical_action(&context(vec![probe])),
        DecisionKind::Refuse { reason }
            if reason == "AUTHORITY_REFUSED_ALL_REMAINING_PHYSICAL_ACTIONS"
    ));
}

#[test]
fn canonical_decision_selects_a_safe_probe_when_no_robust_goal_action_exists() {
    let mut probe = candidate("probe:separating", CandidateRole::PhysicalProbe);
    probe.strict_goal_progress = false;
    probe.robustness = BeliefRobustness::Ambiguous;
    probe.probe = ProbeEvidence {
        decision_relevant_distinctions: 2,
        observable_distinctions: 1,
        future_interaction: ProbeRecoverabilityAssessment::Preserved,
    };
    let mut cx = context(vec![probe]);
    cx.current_contact_id = Some("contact:probe:separating".into());
    match decide_physical_action(&cx) {
        DecisionKind::PhysicalProbe {
            action,
            decision_relevant_distinctions,
        } => {
            assert_eq!(action.candidate_id, "probe:separating");
            assert_eq!(action.role, CandidateRole::PhysicalProbe);
            assert_eq!(decision_relevant_distinctions, 1);
            assert_eq!(action.recoverability, RecoverabilityClass::NoProgress);
        }
        other => panic!("expected canonical physical probe, got {other:?}"),
    }
}

#[test]
fn contact_change_carries_the_frozen_candidate_and_required_transition() {
    let action = candidate("next", CandidateRole::GoalAction);
    match decide_physical_action(&context(vec![action])) {
        DecisionKind::ContactTransition { action, phases } => {
            assert_eq!(action.candidate_id, "next");
            assert_eq!(action.witness_digest, "digest:next");
            assert_eq!(action.witness_contents, "witness-content:next");
            assert_eq!(action.robustness, BeliefRobustness::RobustStrictProgress);
            assert_eq!(
                action.recoverability,
                RecoverabilityClass::ProgressAndRecoverable
            );
            assert_eq!(action.predicted_effect.goal_error_derivative, Some(-0.1));
            assert_eq!(action.predicted_effect.object_translation_world_m, None);
            assert_eq!(
                action.probe.future_interaction,
                ProbeRecoverabilityAssessment::Unknown
            );
            assert_eq!(phases.len(), 4);
        }
        other => panic!("expected contact transition, got {other:?}"),
    }
}

#[test]
fn missing_current_contact_does_not_request_leaving_a_nonexistent_contact() {
    let mut cx = context(vec![candidate("next", CandidateRole::GoalAction)]);
    cx.current_contact_id = None;
    match decide_physical_action(&cx) {
        DecisionKind::ContactTransition { phases, .. } => assert_eq!(
            phases,
            vec![
                realityos_semantics::physical_decision::ContactTransitionPhase::Reobserve,
                realityos_semantics::physical_decision::ContactTransitionPhase::RecheckCollisionAndWitness,
                realityos_semantics::physical_decision::ContactTransitionPhase::ApproachNewContact,
            ]
        ),
        other => panic!("expected contact transition, got {other:?}"),
    }
}
