//! Decision matrix for the shipped physical-decision and authority paths.

use realityos_core::{PolicyProposal, RealityOs};
use realityos_semantics::discrepancy::{
    apply_probe_observation, DiscrepancyKind, Identifiability, ObservationTag, Stimulus,
};
use realityos_semantics::execution_envelope::{
    contents_digest, require_phase_measurement, supervise_execution, ExecutionAuthorization,
    ExecutionEnvelope, ExecutionProgress, FrozenAction, ObservationContract, ObservationField,
    PhasePolicyFields, RuntimePolicyObservation, SupervisorDecision,
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
use realityos_semantics::probe_selection::{
    candidates_for_uncertainty, rank_goal_or_probe, BeliefRobustness, DecisionClass,
};
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
        domain: None,
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
        domain: None,
    });
    assert_eq!(proved.assessment, ProbeRecoverabilityAssessment::Unknown);
    assert_ne!(proved.assessment, ProbeRecoverabilityAssessment::Preserved);
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
            std::slice::from_ref(&probe_scope.action_key),
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
        witness_digest: String::new(),
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
        model_id: String::new(),
        embodiment_id: String::new(),
        observation_epoch: String::new(),
        actuator_id: String::new(),
        observation_contract_id: String::new(),
        abort_contract_id: String::new(),
        authority_granted: true,
        execution_authorization: None,
    }
    .with_matching_authorization(10.0, 40.0);
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

fn issued_authorization(
    grant: &realityos_session::ScopedSimulationGrant,
) -> ExecutionAuthorization {
    ExecutionAuthorization {
        grant_id: grant.command_id.clone(),
        scope_digest: grant.scope_digest.clone(),
        model_id: grant.scope.model_id.clone(),
        embodiment_id: grant.scope.embodiment_id.clone(),
        observation_epoch: grant.scope.observation_epoch.clone(),
        candidate_id: grant.scope.candidate_id.clone(),
        action_key: grant.scope.action_key.clone(),
        witness_digest: grant.scope.witness_digest.clone(),
        actuator_id: grant.scope.actuator_id.clone(),
        requested_stroke_m: grant.scope.requested_stroke_m,
        execution_bound_m: grant.scope.execution_bound_m,
        issued_at_s: grant.scope.issued_at_s,
        expires_at_s: grant.scope.expires_at_s,
        observation_contract_id: grant.scope.observation_contract_id.clone(),
        abort_contract_id: grant.scope.abort_contract_id.clone(),
    }
}

fn authorized_frozen(
    action_key: &str,
    candidate_id: &str,
    witness_contents: &str,
    grant: &realityos_session::ScopedSimulationGrant,
) -> FrozenAction {
    let authorization = issued_authorization(grant);
    FrozenAction {
        action_id: format!("action:{candidate_id}"),
        action_key: action_key.into(),
        candidate_id: candidate_id.into(),
        contact_id: "contact".into(),
        witness_id: format!("witness:{candidate_id}"),
        witness_contents: witness_contents.into(),
        witness_digest: authorization.witness_digest.clone(),
        requested_stroke_m: authorization.requested_stroke_m,
        prediction: realityos_semantics::physical_consequence::FrozenPrediction {
            action_id: format!("action:{candidate_id}"),
            witness_id: format!("witness:{candidate_id}"),
            stroke_m: authorization.requested_stroke_m,
            predicted_displacement_m: Some(0.004),
            predicted_yaw_change_rad: Some(0.0),
            predicted_contact_persists: true,
            quasi_static_stroke_limit_m: 0.015,
        },
        belief_snapshot: PhysicalParameterBelief::declared_point(
            PhysicalParameter::SupportFriction,
            0.3,
            "reasoner.disclosure.support_friction",
        ),
        recoverability: RecoverabilityClass::ProgressAndRecoverable,
        envelope: ExecutionEnvelope::for_quasi_static_stroke(
            authorization.requested_stroke_m,
            0.004,
        ),
        observation_contract: ObservationContract {
            source: "policy-sensors".into(),
            model_epoch: "epoch:1".into(),
            calibration_epoch: "epoch:1".into(),
            max_age_s: 1.0,
            required_units: BTreeMap::new(),
            required_fields: vec![ObservationField::Displacement, ObservationField::Contact],
        },
        model_id: authorization.model_id.clone(),
        embodiment_id: authorization.embodiment_id.clone(),
        observation_epoch: authorization.observation_epoch.clone(),
        actuator_id: authorization.actuator_id.clone(),
        observation_contract_id: authorization.observation_contract_id.clone(),
        abort_contract_id: authorization.abort_contract_id.clone(),
        authority_granted: true,
        execution_authorization: Some(authorization),
    }
}

fn phase_observation(error_before: f64, error_now: f64) -> RuntimePolicyObservation {
    RuntimePolicyObservation {
        action_id: String::new(),
        witness_id: String::new(),
        observation_id: "observation:measured".into(),
        source: "policy-sensors".into(),
        timestamp_s: 10.1,
        model_epoch: "epoch:1".into(),
        calibration_epoch: "epoch:1".into(),
        units: BTreeMap::new(),
        stroke_consumed_m: Some(0.006),
        object_displacement_m: Some(0.004),
        yaw_change_rad: Some(0.0),
        intended_contact_persists: Some(true),
        goal_error_before: Some(error_before),
        goal_error_now: Some(error_now),
        robot_tracking_error_m: Some(0.001),
        reachability_margin_m: Some(0.02),
        quasi_static_applicable: Some(true),
        authority_ok: Some(true),
    }
}

#[test]
fn negative_experiment_conditions_refuse_without_execution() {
    let mut os = RealityOs::new();
    let probe_scope = scope("probe-action", "digest-probe", "epoch:1");
    let grant = issue_scoped_simulation_grant(&mut os, &probe_scope, &proposal(), 10.0, &[], true)
        .expect("probe grant");
    let bound = binding(&probe_scope, &grant.command_id);

    let at_risk = decide_physical_action(&context(
        probe(
            &probe_scope,
            Some(bound.clone()),
            ProbeRecoverabilityAssessment::AtRisk,
        ),
        "epoch:1",
        vec![],
        true,
    ));
    assert!(
        matches!(at_risk, DecisionKind::PhysicallyInfeasible { .. }),
        "informative but unrecoverable probe must be refused, got {at_risk:?}"
    );

    let unknown = decide_physical_action(&context(
        probe(
            &probe_scope,
            Some(bound.clone()),
            ProbeRecoverabilityAssessment::Unknown,
        ),
        "epoch:1",
        vec![],
        true,
    ));
    assert!(
        matches!(unknown, DecisionKind::InsufficientEvidence { .. }),
        "unknown future interaction must not execute, got {unknown:?}"
    );

    let missing_grant = decide_physical_action(&context(
        probe(&probe_scope, None, ProbeRecoverabilityAssessment::Preserved),
        "epoch:1",
        vec![],
        true,
    ));
    assert!(
        matches!(missing_grant, DecisionKind::Refuse { .. }),
        "missing scoped authority must not execute, got {missing_grant:?}"
    );
    let mut flagged = authorized_frozen(
        &probe_scope.action_key,
        &probe_scope.candidate_id,
        "contents:digest-probe",
        &grant,
    );
    flagged.authority_granted = true;
    flagged.execution_authorization = None;
    flagged.witness_contents = "contents:digest-probe".into();
    let progress = ExecutionProgress {
        action_id: flagged.action_id.clone(),
        witness_id: flagged.witness_id.clone(),
        completed_quanta: 0,
        total_quanta: 4,
        stroke_consumed_m: Some(0.0),
        contact_guard_active: false,
        now_s: 10.0,
        remainder_invalidated: false,
    };
    let mut observation = phase_observation(0.2, 0.2);
    observation.action_id = flagged.action_id.clone();
    observation.witness_id = flagged.witness_id.clone();
    observation.timestamp_s = 10.0;
    assert!(
        matches!(
            supervise_execution(&flagged, &progress, &observation),
            SupervisorDecision::AuthorityLost { .. }
        ),
        "a boolean authority flag must not start execution"
    );

    assert_eq!(
        require_phase_measurement(None).unwrap_err(),
        "EVIDENCE_UNAVAILABLE"
    );
    let missing_contact = require_phase_measurement(Some(PhasePolicyFields {
        displacement_m: None,
        yaw_change_rad: None,
        contact_persists: None,
        pose_observed: false,
    }));
    assert!(missing_contact.is_err());
    assert!(missing_contact
        .ok()
        .and_then(|fields| fields.displacement_m)
        .is_none());

    let indistinguishable = candidates_for_uncertainty(0.015)
        .into_iter()
        .filter(|candidate| {
            candidate.id == "probe_repeat" || candidate.class == DecisionClass::GoalAction
        })
        .collect::<Vec<_>>();
    let live = [
        DiscrepancyKind::SupportFrictionInconsistent,
        DiscrepancyKind::QuasiStaticAssumptionBroken,
    ];
    let same_prediction = rank_goal_or_probe(&indistinguishable, &live, 0.015);
    assert_eq!(
        same_prediction.selected_id, None,
        "indistinguishable hypotheses must not select an action: {same_prediction:?}"
    );

    let mut leaving = probe(
        &probe_scope,
        Some(bound),
        ProbeRecoverabilityAssessment::Preserved,
    );
    leaving.recoverability = RecoverabilityClass::ProgressButCanEnterUnrecoverableState;
    let regime = decide_physical_action(&context(leaving, "epoch:1", vec![], true));
    assert!(
        !matches!(
            regime,
            DecisionKind::PhysicalProbe { .. } | DecisionKind::GoalInteraction { .. }
        ),
        "leaving the model regime must not execute, got {regime:?}"
    );
}

#[test]
fn belief_update_changes_the_next_goal_and_records_progress() {
    let limit = 0.015;
    let candidates = candidates_for_uncertainty(limit);
    let before_live = [
        DiscrepancyKind::SupportFrictionInconsistent,
        DiscrepancyKind::QuasiStaticAssumptionBroken,
    ];
    let before = rank_goal_or_probe(&candidates, &before_live, limit);
    assert_eq!(before.selected_class, Some(DecisionClass::PhysicalProbe));
    let before_id = before.selected_id.clone().expect("probe");

    let belief = PhysicalParameterBelief::declared_point(
        PhysicalParameter::SupportFriction,
        0.3,
        "reasoner.disclosure.support_friction",
    );
    let update = apply_probe_observation(
        &belief,
        &before_live,
        Stimulus {
            stroke_m: limit * 0.8,
            quasi_static_stroke_limit_m: limit,
        },
        realityos_semantics::discrepancy::ObservationTag::NominalDisplacementRatio,
        "observation:probe",
    );
    assert_ne!(update.belief, belief);
    assert!(update
        .eliminated
        .contains(&DiscrepancyKind::SupportFrictionInconsistent));
    let after = rank_goal_or_probe(&candidates, &update.remaining, limit);
    assert_eq!(after.selected_class, Some(DecisionClass::GoalAction));
    let after_id = after.selected_id.clone().expect("goal");
    assert_ne!(after_id, before_id);
    let counterfactual = rank_goal_or_probe(&candidates, &before_live, limit);
    assert_eq!(
        counterfactual.selected_id.as_deref(),
        Some(before_id.as_str())
    );
    let selected = candidates
        .iter()
        .find(|candidate| candidate.id == after_id)
        .expect("selected goal candidate");
    let stroke = selected.stroke_m;
    assert!(stroke.is_finite() && stroke > 0.0 && stroke <= limit);

    let goal = realityos_semantics::planar_goal::PlanarObjectGoal {
        object_id: "obj0".into(),
        world_id: "sim".into(),
        model_id: "sim".into(),
        target_xy: Some([stroke * 4.0, 0.0]),
        target_xy_region: None,
        target_yaw: None,
        target_yaw_interval: None,
        translation_tolerance_m: stroke * 0.25,
        orientation_tolerance_rad: 0.2,
        freshness_s: 1.0,
        allowed_interaction_family: realityos_semantics::planar_goal::InteractionFamily::PlanarPush,
        safety: realityos_semantics::planar_goal::SafetyConstraints::default(),
        max_bounded_attempts: 4,
    };
    let before_pose = realityos_semantics::goal_loop::WorldObservation {
        object_id: "obj0".into(),
        xy: [0.0, 0.0],
        yaw: 0.0,
        robot_q: Vec::new(),
        freshness_ok: true,
        intended_contact_face: None,
        authority_ok: true,
        observed_at_s: 10.0,
    };
    let planned = realityos_semantics::goal_loop::receding_horizon_step(
        &before_pose,
        &goal,
        &[],
        realityos_semantics::goal_loop::LoopState::default(),
        None,
    );
    let target = goal.target_xy.expect("point goal");
    let target_norm = target[0].hypot(target[1]).max(1e-9);
    let measured_dxy = [
        target[0] / target_norm * stroke,
        target[1] / target_norm * stroke,
    ];
    let recorded = realityos_semantics::goal_loop::record_after_measured_displacement(
        planned,
        measured_dxy,
        before_pose.yaw,
        before_pose.observed_at_s + 0.1,
        &goal,
    );
    let before_err = recorded
        .record
        .goal_error_before
        .as_ref()
        .expect("shipped path writes goal_error_before");
    let after_err = recorded
        .record
        .goal_error_after
        .as_ref()
        .expect("shipped path writes goal_error_after");
    assert_eq!(
        recorded.record.outcome,
        realityos_semantics::goal_loop::GoalLoopOutcome::GoalProgress
    );
    assert!(
        after_err.translation_residual_m < before_err.translation_residual_m,
        "recorded residual {} -> {}",
        before_err.translation_residual_m,
        after_err.translation_residual_m
    );
    let measured = before_err.translation_residual_m - after_err.translation_residual_m;
    assert!(
        (measured - stroke).abs() < 1e-9,
        "recorded progress {measured} is the selected stroke {stroke}"
    );

    let mut os = RealityOs::new();
    let witness_contents = "goal-witness-bytes";
    let goal_scope = scope(&after_id, &contents_digest(witness_contents), "epoch:1");
    let grant = issue_scoped_simulation_grant(&mut os, &goal_scope, &proposal(), 10.0, &[], true)
        .expect("goal grant");
    assert_eq!(grant.evidence_status, "SIMULATION_ONLY");
    assert!(!grant.metal);
    let action = authorized_frozen(
        &after_id,
        &goal_scope.candidate_id,
        witness_contents,
        &grant,
    );
    assert!(action
        .execution_authorization
        .as_ref()
        .unwrap()
        .covers(&action, 10.0));
    let progress = ExecutionProgress {
        action_id: action.action_id.clone(),
        witness_id: action.witness_id.clone(),
        completed_quanta: 1,
        total_quanta: 4,
        stroke_consumed_m: Some(stroke),
        contact_guard_active: true,
        now_s: 10.1,
        remainder_invalidated: false,
    };
    let mut observation = phase_observation(
        before_err.translation_residual_m,
        after_err.translation_residual_m,
    );
    observation.action_id = action.action_id.clone();
    observation.witness_id = action.witness_id.clone();
    observation.stroke_consumed_m = Some(stroke);
    observation.object_displacement_m = Some(stroke);
    let decision = supervise_execution(&action, &progress, &observation);
    assert!(
        matches!(decision, SupervisorDecision::Continue { .. }),
        "authorized goal must execute, got {decision:?}"
    );
    println!(
        "counterfactual={before_id} next={after_id} outcome={:?} residual_before_m={} residual_after_m={} progress_m={measured} grant={}",
        recorded.record.outcome,
        before_err.translation_residual_m,
        after_err.translation_residual_m,
        grant.command_id
    );
}
