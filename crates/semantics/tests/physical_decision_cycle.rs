use std::collections::BTreeMap;

use realityos_semantics::discrepancy::{
    apply_probe_observation, tag_probe_motion, DiscrepancyKind, Identifiability, ObservationTag,
    Stimulus,
};
use realityos_semantics::execution_envelope::{
    supervise_execution, ExecutionEnvelope, ExecutionProgress, FrozenAction, ObservationContract,
    ObservationField, RuntimePolicyObservation, SupervisorDecision,
};
use realityos_semantics::physical_belief::{PhysicalParameter, PhysicalParameterBelief};
use realityos_semantics::physical_consequence::{
    assess_consequence, ConsequenceStatus, FrozenPrediction, ObservedConsequence,
};
use realityos_semantics::physical_decision::{
    decide_physical_action, CandidateEvidence, CandidateRole, ContactTransitionPhase,
    DecisionContext, DecisionKind, LexicographicPreference, PredictedPhysicalEffect, ProbeEvidence,
    ProbeRecoverabilityAssessment, SelectedPhysicalAction,
};
use realityos_semantics::planar_goal::GoalProgressClass;
use realityos_semantics::probe_selection::BeliefRobustness;
use realityos_semantics::recoverability::RecoverabilityClass;

fn candidate(id: &str, role: CandidateRole) -> CandidateEvidence {
    CandidateEvidence {
        candidate_id: id.into(),
        candidate_contents: format!("candidate:{id}"),
        action_key: format!("action-key:{id}"),
        contact_id: format!("contact:{id}"),
        role,
        strict_goal_progress: role == CandidateRole::GoalAction,
        authority_ok: true,
        executable_witness_id: Some(format!("witness:{id}")),
        witness_digest: Some(format!("digest:{id}")),
        witness_contents: Some(format!("frozen-witness:{id}")),
        robustness: if role == CandidateRole::PhysicalProbe {
            BeliefRobustness::Ambiguous
        } else {
            BeliefRobustness::RobustStrictProgress
        },
        recoverability: if role == CandidateRole::PhysicalProbe {
            RecoverabilityClass::NoProgress
        } else {
            RecoverabilityClass::ProgressAndRecoverable
        },
        predicted_effect: PredictedPhysicalEffect {
            object_translation_world_m: None,
            yaw_change_rad: Some(0.0),
            contact_persists: Some(true),
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
            stroke_m: 0.012,
        },
        hard_rejections: vec![],
    }
}

fn selected_probe(decision: DecisionKind) -> SelectedPhysicalAction {
    match decision {
        DecisionKind::PhysicalProbe { action, .. }
        | DecisionKind::ContactTransition { action, .. } => {
            assert_eq!(action.role, CandidateRole::PhysicalProbe);
            action
        }
        other => panic!("expected canonical probe selection, got {other:?}"),
    }
}

#[test]
fn abort_probe_observe_update_and_replan_stays_on_the_canonical_path() {
    let old_goal = candidate("failed-goal", CandidateRole::GoalAction);
    let mut ambiguous_goal = candidate("goal-short", CandidateRole::GoalAction);
    ambiguous_goal.robustness = BeliefRobustness::Ambiguous;
    let mut probe = candidate("probe-separating", CandidateRole::PhysicalProbe);
    probe.probe = ProbeEvidence {
        decision_relevant_distinctions: 1,
        observable_distinctions: 1,
        future_interaction: ProbeRecoverabilityAssessment::Preserved,
    };

    let before_context = DecisionContext {
        goal_id: "goal:box-to-region".into(),
        goal_reached: false,
        evidence_fresh: true,
        remaining_attempts: 2,
        current_contact_id: None,
        forbidden_action_keys: vec![old_goal.action_key.clone()],
        candidates: vec![old_goal, ambiguous_goal, probe],
    };
    let probe_action = selected_probe(decide_physical_action(&before_context));
    assert_eq!(probe_action.candidate_id, "probe-separating");
    assert!(before_context
        .forbidden_action_keys
        .contains(&"action-key:failed-goal".to_string()));

    let probe_prediction = FrozenPrediction {
        action_id: "action:probe-separating".into(),
        witness_id: probe_action.witness_id.clone(),
        stroke_m: 0.012,
        predicted_displacement_m: Some(0.006),
        predicted_yaw_change_rad: Some(0.0),
        predicted_contact_persists: true,
        quasi_static_stroke_limit_m: 0.015,
    };
    let frozen_probe = FrozenAction {
        action_id: probe_prediction.action_id.clone(),
        action_key: probe_action.action_key.clone(),
        candidate_id: probe_action.candidate_id.clone(),
        contact_id: probe_action.contact_id.clone(),
        witness_id: probe_action.witness_id.clone(),
        witness_contents: probe_action.witness_contents.clone(),
        requested_stroke_m: probe_prediction.stroke_m,
        prediction: probe_prediction.clone(),
        belief_snapshot: PhysicalParameterBelief::declared_point(
            PhysicalParameter::SupportFriction,
            0.3,
            "scene.support_friction",
        )
        .with_unknown(PhysicalParameter::QuasiStaticApplicability, "scene.regime"),
        recoverability: RecoverabilityClass::NoProgress,
        envelope: ExecutionEnvelope::for_quasi_static_stroke(0.012, 0.006),
        observation_contract: ObservationContract {
            source: "policy-sensors".into(),
            model_epoch: "model:1".into(),
            calibration_epoch: "calibration:1".into(),
            max_age_s: 0.2,
            required_units: BTreeMap::new(),
            required_fields: vec![
                ObservationField::StrokeConsumed,
                ObservationField::Displacement,
                ObservationField::Yaw,
                ObservationField::Contact,
                ObservationField::GoalError,
                ObservationField::Tracking,
                ObservationField::Reachability,
                ObservationField::QuasiStaticApplicability,
            ],
        },
        authority_granted: true,
    };
    let progress = ExecutionProgress {
        action_id: frozen_probe.action_id.clone(),
        witness_id: frozen_probe.witness_id.clone(),
        completed_quanta: 1,
        total_quanta: 1,
        stroke_consumed_m: Some(0.012),
        contact_guard_active: true,
        now_s: 10.1,
        remainder_invalidated: false,
    };
    let observation = RuntimePolicyObservation {
        action_id: frozen_probe.action_id.clone(),
        witness_id: frozen_probe.witness_id.clone(),
        observation_id: "observation:probe".into(),
        source: "policy-sensors".into(),
        timestamp_s: 10.1,
        model_epoch: "model:1".into(),
        calibration_epoch: "calibration:1".into(),
        units: BTreeMap::new(),
        stroke_consumed_m: Some(0.012),
        object_displacement_m: Some(0.006),
        yaw_change_rad: Some(0.0),
        intended_contact_persists: Some(true),
        goal_error_before: Some(1.0),
        goal_error_now: Some(0.99),
        robot_tracking_error_m: Some(0.001),
        reachability_margin_m: Some(0.02),
        quasi_static_applicable: Some(true),
        authority_ok: Some(true),
    };
    assert!(matches!(
        supervise_execution(&frozen_probe, &progress, &observation),
        SupervisorDecision::Completed { action_id, witness_id }
            if action_id == frozen_probe.action_id && witness_id == frozen_probe.witness_id
    ));
    let consequence = assess_consequence(
        &probe_prediction,
        &ObservedConsequence {
            action_id: frozen_probe.action_id.clone(),
            witness_id: frozen_probe.witness_id.clone(),
            observation_id: observation.observation_id.clone(),
            freshness_ok: true,
            execution_aborted: false,
            stroke_consumed_m: observation.stroke_consumed_m,
            displacement_m: observation.object_displacement_m,
            yaw_change_rad: observation.yaw_change_rad,
            contact_persisted: observation.intended_contact_persists,
            tracking_error_m: observation.robot_tracking_error_m,
            geometry_residual_m: Some(0.0),
            quasi_static_applicable: observation.quasi_static_applicable,
        },
    );
    assert_eq!(consequence.status, ConsequenceStatus::Consistent);

    let belief_before = frozen_probe.belief_snapshot.clone();
    let live = [
        DiscrepancyKind::SupportFrictionInconsistent,
        DiscrepancyKind::QuasiStaticAssumptionBroken,
    ];
    let observed_tag = tag_probe_motion(
        observation.object_displacement_m.unwrap(),
        observation.stroke_consumed_m.unwrap(),
        observation.intended_contact_persists.unwrap(),
    );
    assert_eq!(observed_tag, ObservationTag::NominalDisplacementRatio);
    let update = apply_probe_observation(
        &belief_before,
        &live,
        Stimulus {
            stroke_m: 0.012,
            quasi_static_stroke_limit_m: 0.015,
        },
        observed_tag,
        &observation.observation_id,
    );
    assert_eq!(update.status, Identifiability::Identified);
    assert_eq!(
        update.remaining,
        vec![DiscrepancyKind::QuasiStaticAssumptionBroken]
    );
    assert_eq!(
        update.eliminated,
        vec![DiscrepancyKind::SupportFrictionInconsistent]
    );
    assert_ne!(update.belief, belief_before);
    assert!(update
        .belief
        .entry(PhysicalParameter::SupportFriction)
        .unwrap()
        .lineage
        .iter()
        .any(|line| line.observation == observation.observation_id));

    let mut recovered_goal = candidate("goal-after-probe", CandidateRole::GoalAction);
    recovered_goal.robustness = BeliefRobustness::RobustStrictProgress;
    recovered_goal.preference.stroke_m = 0.012;
    let after_context = DecisionContext {
        goal_id: "goal:box-to-region".into(),
        goal_reached: false,
        evidence_fresh: true,
        remaining_attempts: 1,
        current_contact_id: None,
        forbidden_action_keys: vec!["action-key:failed-goal".into()],
        candidates: vec![recovered_goal],
    };
    let next_action = match decide_physical_action(&after_context) {
        DecisionKind::ContactTransition { action, phases } => {
            assert_eq!(
                phases,
                vec![
                    ContactTransitionPhase::Reobserve,
                    ContactTransitionPhase::RecheckCollisionAndWitness,
                    ContactTransitionPhase::ApproachNewContact,
                ]
            );
            action
        }
        DecisionKind::GoalInteraction { action } => action,
        other => panic!("expected canonical goal action after probe, got {other:?}"),
    };
    assert_eq!(next_action.candidate_id, "goal-after-probe");
    assert_ne!(next_action.action_key, "action-key:failed-goal");
    assert_eq!(next_action.role, CandidateRole::GoalAction);
}
