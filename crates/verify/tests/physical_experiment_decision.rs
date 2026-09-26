//! Decision matrix for the shipped physical-decision and authority paths.

use realityos_core::{PolicyProposal, RealityOs};
use realityos_semantics::discrepancy::{
    apply_probe_observation, DiscrepancyKind, Identifiability, ObservationTag, Stimulus,
};
use realityos_semantics::execution_envelope::{
    supervise_execution, ExecutionEnvelope, ExecutionProgress, FrozenAction, ObservationContract,
    ObservationField, RuntimePolicyObservation, SupervisorDecision,
};
use realityos_semantics::future_interaction::{
    assess_probe_future, ProbeFutureInputs, SupportedProbeOutcome,
};
use realityos_semantics::physical_belief::{PhysicalParameter, PhysicalParameterBelief};
use realityos_semantics::physical_decision::{
    decide_physical_action, CandidateEvidence, CandidateRole, DecisionContext, DecisionKind,
    LexicographicPreference, PredictedPhysicalEffect, ProbeEvidence, ProbeRecoverabilityAssessment,
    ScopedGrantBinding,
};
use realityos_semantics::probe_selection::BeliefRobustness;
use realityos_semantics::recoverability::RecoverabilityClass;
use realityos_session::{
    evaluate_action_eligibility, grant_covers, issue_scoped_simulation_grant, PhysicalActionScope,
    ScopeRefusal,
};
use std::collections::BTreeMap;

fn scope(action: &str, witness: &str, epoch: &str) -> PhysicalActionScope {
    PhysicalActionScope {
        model_id: "model-hash".into(),
        embodiment_id: "embodiment".into(),
        observation_epoch: epoch.into(),
        candidate_id: format!("candidate:{action}"),
        action_key: action.into(),
        witness_digest: witness.into(),
        requested_stroke_m: 0.012,
        execution_bound_m: 0.02,
        issued_at_s: 10.0,
        expires_at_s: 40.0,
        actuator_id: "tool".into(),
        observation_contract_id: format!("sensors:{epoch}"),
        abort_contract_id: "abort-and-reobserve".into(),
    }
}

fn binding(scope: &PhysicalActionScope, grant_id: &str) -> ScopedGrantBinding {
    ScopedGrantBinding {
        grant_id: grant_id.into(),
        scope_digest: realityos_session::scope_digest(scope),
        action_key: scope.action_key.clone(),
        candidate_id: scope.candidate_id.clone(),
        witness_digest: scope.witness_digest.clone(),
        observation_epoch: scope.observation_epoch.clone(),
        requested_stroke_m: scope.requested_stroke_m,
        expires_at_s: scope.expires_at_s,
    }
}

fn probe(
    scope: &PhysicalActionScope,
    grant: Option<ScopedGrantBinding>,
    future: ProbeRecoverabilityAssessment,
) -> CandidateEvidence {
    CandidateEvidence {
        candidate_id: scope.candidate_id.clone(),
        candidate_contents: format!("snapshot:{}", scope.candidate_id),
        action_key: scope.action_key.clone(),
        contact_id: "contact:probe".into(),
        role: CandidateRole::PhysicalProbe,
        strict_goal_progress: false,
        authority_ok: true,
        scoped_grant: grant,
        executable_witness_id: Some(format!("witness:{}", scope.candidate_id)),
        witness_digest: Some(scope.witness_digest.clone()),
        witness_contents: Some(format!("contents:{}", scope.witness_digest)),
        robustness: BeliefRobustness::Ambiguous,
        recoverability: RecoverabilityClass::NoProgress,
        predicted_effect: PredictedPhysicalEffect {
            object_translation_world_m: None,
            yaw_change_rad: None,
            contact_persists: None,
            goal_error_derivative: None,
            goal_progress: None,
        },
        probe: ProbeEvidence {
            decision_relevant_distinctions: 2,
            observable_distinctions: 1,
            future_interaction: future,
        },
        preference: LexicographicPreference {
            error_derivative: None,
            angular_rate_abs: None,
            contact_offset_abs_m: None,
            stroke_m: scope.requested_stroke_m,
        },
        hard_rejections: vec![],
    }
}

fn context(
    candidate: CandidateEvidence,
    epoch: &str,
    forbidden: Vec<String>,
    fresh: bool,
) -> DecisionContext {
    DecisionContext {
        goal_id: "goal:box".into(),
        goal_reached: false,
        evidence_fresh: fresh,
        now_s: 10.0,
        observation_epoch: epoch.into(),
        remaining_attempts: 2,
        current_contact_id: Some("contact:probe".into()),
        forbidden_action_keys: forbidden,
        candidates: vec![candidate],
    }
}

fn preserved_outcome(label: &str) -> SupportedProbeOutcome {
    SupportedProbeOutcome {
        label: label.into(),
        object_supported: Some(true),
        inside_reachable_workspace: Some(true),
        joint_margin_rad: Some(0.15),
        collision_admissible: Some(true),
        motion_within_declared_bound: Some(true),
        belief_outcome_bounded: Some(true),
        contact_persists: Some(false),
        return_contact_witness_digest: Some(format!("return:{label}")),
    }
}

fn proposal() -> PolicyProposal {
    PolicyProposal::external_deterministic(vec![1.0], "canonical-physical-decision")
}

#[test]
fn shipped_decision_and_authority_matrix() {
    let mut os = RealityOs::new();
    let probe_scope = scope("probe-action", "digest-probe", "epoch:1");
    let goal_scope = scope("goal-action", "digest-goal", "epoch:1");
    let learned = PolicyProposal::learned(vec![1.0], "experience_log");
    assert_eq!(
        issue_scoped_simulation_grant(&mut os, &probe_scope, &learned, 10.0, &[], true),
        Err(ScopeRefusal::LearnedSourceCannotAuthorize)
    );
    assert!(!evaluate_action_eligibility(&probe_scope, &learned, &[], true).eligible);

    let goal_grant =
        issue_scoped_simulation_grant(&mut os, &goal_scope, &proposal(), 10.0, &[], true)
            .expect("goal grant");
    let probe_grant =
        issue_scoped_simulation_grant(&mut os, &probe_scope, &proposal(), 10.0, &[], true)
            .expect("probe grant");
    assert!(!probe_grant.metal);
    assert_eq!(probe_grant.evidence_status, "SIMULATION_ONLY");
    assert_ne!(probe_grant.command_id, goal_grant.command_id);
    assert!(grant_covers(&probe_grant, &probe_scope, 10.0).is_ok());
    assert_eq!(
        grant_covers(&goal_grant, &probe_scope, 10.0),
        Err(ScopeRefusal::BindingMismatch)
    );

    let future_unknown = assess_probe_future(&ProbeFutureInputs {
        consumed_stroke_m: 0.0,
        observed_displacement_m: Some(0.013),
        geometry_residual_m: Some(0.001),
        outcomes: vec![preserved_outcome("current")],
    });
    assert_eq!(
        future_unknown.assessment,
        ProbeRecoverabilityAssessment::Unknown
    );
    let granted = binding(&probe_scope, &probe_grant.command_id);
    let unknown_with_grant = decide_physical_action(&context(
        probe(
            &probe_scope,
            Some(granted.clone()),
            ProbeRecoverabilityAssessment::Unknown,
        ),
        "epoch:1",
        vec![],
        true,
    ));
    assert!(matches!(
        unknown_with_grant,
        DecisionKind::InsufficientEvidence { ref reason }
            if reason == "PROBE_FUTURE_INTERACTION_UNPROVEN"
    ));

    let proved = assess_probe_future(&ProbeFutureInputs {
        consumed_stroke_m: 0.01,
        observed_displacement_m: Some(0.008),
        geometry_residual_m: Some(0.002),
        outcomes: vec![preserved_outcome("stay"), preserved_outcome("stroke")],
    });
    assert_eq!(proved.assessment, ProbeRecoverabilityAssessment::Preserved);
    let missing_grant = decide_physical_action(&context(
        probe(&probe_scope, None, ProbeRecoverabilityAssessment::Preserved),
        "epoch:1",
        vec![],
        true,
    ));
    assert!(matches!(
        missing_grant,
        DecisionKind::Refuse { ref reason } if reason == "SCOPED_GRANT_DOES_NOT_COVER_PROBE"
    ));

    let reused = binding(&goal_scope, &goal_grant.command_id);
    let mut reused_probe = probe(
        &probe_scope,
        Some(reused),
        ProbeRecoverabilityAssessment::Preserved,
    );
    reused_probe.candidate_id = goal_scope.candidate_id.clone();
    reused_probe.action_key = goal_scope.action_key.clone();
    reused_probe.witness_digest = Some(goal_scope.witness_digest.clone());
    reused_probe.action_key = probe_scope.action_key.clone();
    let reuse = decide_physical_action(&context(reused_probe, "epoch:1", vec![], true));
    assert!(matches!(
        reuse,
        DecisionKind::Refuse { ref reason } if reason == "SCOPED_GRANT_DOES_NOT_COVER_PROBE"
    ));

    let mut changed = granted.clone();
    changed.witness_digest = "digest-changed".into();
    let mut changed_probe = probe(
        &probe_scope,
        Some(changed),
        ProbeRecoverabilityAssessment::Preserved,
    );
    changed_probe.witness_digest = Some("digest-changed".into());
    changed_probe.witness_contents = Some("contents:digest-changed".into());
    let digest_mismatch = decide_physical_action(&context(changed_probe, "epoch:2", vec![], true));
    assert!(matches!(
        digest_mismatch,
        DecisionKind::Refuse { ref reason } if reason == "SCOPED_GRANT_DOES_NOT_COVER_PROBE"
    ));

    let blacklisted = decide_physical_action(&context(
        probe(
            &probe_scope,
            Some(granted.clone()),
            ProbeRecoverabilityAssessment::Preserved,
        ),
        "epoch:1",
        vec![probe_scope.action_key.clone()],
        true,
    ));
    assert!(!matches!(
        blacklisted,
        DecisionKind::PhysicalProbe { .. } | DecisionKind::ContactTransition { .. }
    ));
    assert_eq!(
        issue_scoped_simulation_grant(
            &mut os,
            &probe_scope,
            &proposal(),
            12.0,
            &[probe_scope.action_key.clone()],
            true
        ),
        Err(ScopeRefusal::ActionBlacklisted)
    );

    let missing_sensor = decide_physical_action(&context(
        probe(
            &probe_scope,
            Some(granted.clone()),
            ProbeRecoverabilityAssessment::Preserved,
        ),
        "epoch:1",
        vec![],
        false,
    ));
    assert!(matches!(
        missing_sensor,
        DecisionKind::InsufficientEvidence { .. }
    ));
    assert_eq!(
        issue_scoped_simulation_grant(&mut os, &probe_scope, &proposal(), 12.0, &[], false),
        Err(ScopeRefusal::SensorEvidenceMissing)
    );

    let selected = decide_physical_action(&context(
        probe(
            &probe_scope,
            Some(granted),
            ProbeRecoverabilityAssessment::Preserved,
        ),
        "epoch:1",
        vec![],
        true,
    ));
    let DecisionKind::PhysicalProbe { action, .. } = selected else {
        panic!("both proofs should select the probe, got {selected:?}");
    };
    assert_eq!(action.candidate_id, probe_scope.candidate_id);
    assert_eq!(action.witness_digest, probe_scope.witness_digest);

    let frozen = FrozenAction {
        action_id: "probe-action".into(),
        action_key: action.action_key.clone(),
        candidate_id: action.candidate_id.clone(),
        contact_id: action.contact_id.clone(),
        witness_id: action.witness_id.clone(),
        witness_contents: action.witness_contents.clone(),
        requested_stroke_m: 0.012,
        prediction: realityos_semantics::physical_consequence::FrozenPrediction {
            action_id: "probe-action".into(),
            witness_id: action.witness_id.clone(),
            stroke_m: 0.012,
            predicted_displacement_m: Some(0.006),
            predicted_yaw_change_rad: Some(0.0),
            predicted_contact_persists: true,
            quasi_static_stroke_limit_m: 0.015,
        },
        belief_snapshot: PhysicalParameterBelief::declared_point(
            PhysicalParameter::SupportFriction,
            0.3,
            "reasoner.disclosure.support_friction",
        )
        .with_unknown(
            PhysicalParameter::QuasiStaticApplicability,
            "declared.quasi_static",
        ),
        recoverability: RecoverabilityClass::NoProgress,
        envelope: ExecutionEnvelope::for_quasi_static_stroke(0.012, 0.006),
        observation_contract: ObservationContract {
            source: "policy-sensors".into(),
            model_epoch: "epoch:1".into(),
            calibration_epoch: "epoch:1".into(),
            max_age_s: 1.0,
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
    let observation = RuntimePolicyObservation {
        action_id: frozen.action_id.clone(),
        witness_id: frozen.witness_id.clone(),
        observation_id: "observation:probe".into(),
        source: "policy-sensors".into(),
        timestamp_s: 10.1,
        model_epoch: "epoch:1".into(),
        calibration_epoch: "epoch:1".into(),
        units: BTreeMap::new(),
        stroke_consumed_m: Some(0.006),
        object_displacement_m: Some(0.004),
        yaw_change_rad: Some(0.0),
        intended_contact_persists: Some(true),
        goal_error_before: Some(0.05),
        goal_error_now: Some(0.046),
        robot_tracking_error_m: Some(0.001),
        reachability_margin_m: Some(0.01),
        quasi_static_applicable: Some(true),
        authority_ok: Some(true),
    };
    let abort_obs = RuntimePolicyObservation {
        object_displacement_m: Some(0.02),
        yaw_change_rad: Some(0.5),
        intended_contact_persists: Some(false),
        ..observation.clone()
    };
    let progress = ExecutionProgress {
        action_id: frozen.action_id.clone(),
        witness_id: frozen.witness_id.clone(),
        completed_quanta: 2,
        total_quanta: 4,
        stroke_consumed_m: Some(0.006),
        contact_guard_active: true,
        now_s: 10.1,
        remainder_invalidated: false,
    };
    let aborted = supervise_execution(&frozen, &progress, &abort_obs);
    assert!(matches!(
        aborted,
        SupervisorDecision::AbortAndReobserve {
            remainder_invalidated: true,
            ..
        }
    ));
    let after_abort = ExecutionProgress {
        remainder_invalidated: true,
        ..progress.clone()
    };
    assert!(matches!(
        supervise_execution(&frozen, &after_abort, &observation),
        SupervisorDecision::AbortAndReobserve {
            remainder_invalidated: true,
            ..
        }
    ));

    let belief = frozen.belief_snapshot.clone();
    let live = [
        DiscrepancyKind::SupportFrictionInconsistent,
        DiscrepancyKind::QuasiStaticAssumptionBroken,
    ];
    let stimulus = Stimulus {
        stroke_m: 0.012,
        quasi_static_stroke_limit_m: 0.015,
    };
    let insufficient = apply_probe_observation(
        &belief,
        &live,
        stimulus,
        ObservationTag::Insufficient,
        "observation:insufficient",
    );
    assert_eq!(insufficient.remaining, live);
    assert!(insufficient.eliminated.is_empty());
    assert_eq!(
        insufficient
            .belief
            .declared_value(PhysicalParameter::SupportFriction),
        Some(0.3)
    );
    let discriminating = apply_probe_observation(
        &belief,
        &live,
        stimulus,
        ObservationTag::NominalDisplacementRatio,
        "observation:probe",
    );
    assert_eq!(discriminating.status, Identifiability::Identified);
    assert_eq!(
        discriminating.eliminated,
        vec![DiscrepancyKind::SupportFrictionInconsistent]
    );
    assert_eq!(
        discriminating
            .belief
            .declared_value(PhysicalParameter::SupportFriction),
        Some(0.3)
    );
    assert_ne!(discriminating.belief, belief);

    let without = probe(
        &probe_scope,
        Some(binding(&probe_scope, &probe_grant.command_id)),
        ProbeRecoverabilityAssessment::Preserved,
    );
    let mut with_evidence = without.clone();
    with_evidence.candidate_id = "goal-after".into();
    with_evidence.action_key = "goal-after-key".into();
    with_evidence.role = CandidateRole::GoalAction;
    with_evidence.strict_goal_progress = true;
    with_evidence.robustness = BeliefRobustness::RobustStrictProgress;
    with_evidence.recoverability = RecoverabilityClass::ProgressAndRecoverable;
    with_evidence.preference.error_derivative = Some(-0.02);
    with_evidence.scoped_grant = None;
    let before = decide_physical_action(&context(without.clone(), "epoch:1", vec![], true));
    assert!(matches!(before, DecisionKind::PhysicalProbe { .. }));
    let mut after_context = context(with_evidence, "epoch:1", vec![], true);
    after_context.candidates.insert(0, without);
    let after = decide_physical_action(&after_context);
    let (DecisionKind::GoalInteraction { action: next }
    | DecisionKind::ContactTransition { action: next, .. }) = after
    else {
        panic!("learned belief should change the canonical action, got {after:?}");
    };
    assert_eq!(next.candidate_id, "goal-after");
    assert_ne!(next.role, CandidateRole::PhysicalProbe);

    println!(
        "matrix=pass grant={} future={:?}",
        probe_grant.command_id, proved.assessment
    );
}
