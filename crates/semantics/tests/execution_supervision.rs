use std::collections::BTreeMap;

use realityos_semantics::execution_envelope::{
    contents_digest, supervise_execution, ExecutionAuthorization, ExecutionEnvelope,
    ExecutionProgress, FrozenAction, ObservationContract, ObservationField,
    RuntimePolicyObservation, SupervisorDecision,
};
use realityos_semantics::physical_belief::PhysicalParameterBelief;
use realityos_semantics::physical_consequence::FrozenPrediction;
use realityos_semantics::recoverability::RecoverabilityClass;

fn frozen() -> FrozenAction {
    let action = FrozenAction {
        action_id: "action:1".into(),
        action_key: "face:1".into(),
        candidate_id: "candidate:1".into(),
        contact_id: "contact:1".into(),
        witness_id: "witness:1".into(),
        witness_contents: "frozen-witness-content".into(),
        witness_digest: String::new(),
        requested_stroke_m: 0.02,
        prediction: FrozenPrediction {
            action_id: "action:1".into(),
            witness_id: "witness:1".into(),
            stroke_m: 0.02,
            predicted_displacement_m: Some(0.018),
            predicted_yaw_change_rad: Some(0.0),
            predicted_contact_persists: true,
            quasi_static_stroke_limit_m: 0.05,
        },
        belief_snapshot: PhysicalParameterBelief { parameters: vec![] },
        recoverability: RecoverabilityClass::ProgressAndRecoverable,
        envelope: ExecutionEnvelope::for_quasi_static_stroke(0.02, 0.018),
        observation_contract: ObservationContract {
            source: "sim-perfect-perception".into(),
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
    };
    // Test fixture only. Production execution has to bind an issued grant.
    fixture_grant(action, 10.0, 40.0)
}

fn fixture_grant(mut action: FrozenAction, issued_at_s: f64, expires_at_s: f64) -> FrozenAction {
    if action.witness_digest.is_empty() {
        action.witness_digest = contents_digest(&action.witness_contents);
    }
    if action.model_id.is_empty() {
        action.model_id = "model".into();
    }
    if action.embodiment_id.is_empty() {
        action.embodiment_id = "embodiment".into();
    }
    if action.observation_epoch.is_empty() {
        action.observation_epoch = "epoch".into();
    }
    if action.actuator_id.is_empty() {
        action.actuator_id = "actuator".into();
    }
    if action.observation_contract_id.is_empty() {
        action.observation_contract_id = format!("sensors:{}", action.observation_epoch);
    }
    if action.abort_contract_id.is_empty() {
        action.abort_contract_id = "abort-and-reobserve".into();
    }
    let scope_digest = action.witness_digest.clone();
    let authorization = ExecutionAuthorization {
        grant_id: format!("sim-scope:{scope_digest}"),
        scope_digest,
        model_id: action.model_id.clone(),
        embodiment_id: action.embodiment_id.clone(),
        observation_epoch: action.observation_epoch.clone(),
        candidate_id: action.candidate_id.clone(),
        action_key: action.action_key.clone(),
        witness_digest: action.witness_digest.clone(),
        actuator_id: action.actuator_id.clone(),
        requested_stroke_m: action.requested_stroke_m,
        execution_bound_m: action.requested_stroke_m,
        issued_at_s,
        expires_at_s,
        observation_contract_id: action.observation_contract_id.clone(),
        abort_contract_id: action.abort_contract_id.clone(),
    };
    action
        .bind_issued_authorization(authorization, issued_at_s)
        .expect("fixture grant covers the action")
}

fn progress() -> ExecutionProgress {
    ExecutionProgress {
        action_id: "action:1".into(),
        witness_id: "witness:1".into(),
        completed_quanta: 1,
        total_quanta: 4,
        stroke_consumed_m: Some(0.01),
        contact_guard_active: true,
        now_s: 10.1,
        remainder_invalidated: false,
    }
}

fn observation() -> RuntimePolicyObservation {
    RuntimePolicyObservation {
        action_id: "action:1".into(),
        witness_id: "witness:1".into(),
        observation_id: "obs:1".into(),
        source: "sim-perfect-perception".into(),
        timestamp_s: 10.1,
        model_epoch: "model:1".into(),
        calibration_epoch: "calibration:1".into(),
        units: BTreeMap::new(),
        stroke_consumed_m: Some(0.01),
        object_displacement_m: Some(0.009),
        yaw_change_rad: Some(0.0),
        intended_contact_persists: Some(true),
        goal_error_before: Some(1.0),
        goal_error_now: Some(0.99),
        robot_tracking_error_m: Some(0.0),
        reachability_margin_m: Some(0.05),
        quasi_static_applicable: Some(true),
        authority_ok: Some(true),
    }
}

#[test]
fn supervisor_only_continues_the_same_frozen_action_quantum() {
    assert!(matches!(
        supervise_execution(&frozen(), &progress(), &observation()),
        SupervisorDecision::Continue { action_id, next_quantum: 2, .. }
            if action_id == "action:1"
    ));
}

#[test]
fn missing_tracking_and_wrong_epoch_fail_closed() {
    let mut obs = observation();
    obs.robot_tracking_error_m = None;
    assert!(matches!(
        supervise_execution(&frozen(), &progress(), &obs),
        SupervisorDecision::EvidenceUnavailable { .. }
    ));
    let mut obs = observation();
    obs.model_epoch = "model:stale".into();
    assert!(matches!(
        supervise_execution(&frozen(), &progress(), &obs),
        SupervisorDecision::EvidenceUnavailable { .. }
    ));
}

#[test]
fn a_boolean_grant_flag_does_not_execute() {
    let mut action = frozen();
    action.authority_granted = true;
    action.execution_authorization = None;
    assert!(matches!(
        supervise_execution(&action, &progress(), &observation()),
        SupervisorDecision::AuthorityLost { .. }
    ));
    let mut expired = frozen();
    if let Some(authorization) = expired.execution_authorization.as_mut() {
        authorization.expires_at_s = 10.0;
    }
    assert!(matches!(
        supervise_execution(&expired, &progress(), &observation()),
        SupervisorDecision::AuthorityLost { .. }
    ));
    let mut other = frozen();
    other.action_key = "face:other".into();
    assert!(matches!(
        supervise_execution(&other, &progress(), &observation()),
        SupervisorDecision::AuthorityLost { .. }
    ));
    let mut changed_witness = frozen();
    changed_witness.witness_contents = "rewritten-witness".into();
    assert!(matches!(
        supervise_execution(&changed_witness, &progress(), &observation()),
        SupervisorDecision::AuthorityLost { .. }
    ));
    let mut changed_epoch = frozen();
    changed_epoch.observation_epoch = "epoch:later".into();
    assert!(matches!(
        supervise_execution(&changed_epoch, &progress(), &observation()),
        SupervisorDecision::AuthorityLost { .. }
    ));
    let mut aborted = progress();
    aborted.remainder_invalidated = true;
    assert!(matches!(
        supervise_execution(&frozen(), &aborted, &observation()),
        SupervisorDecision::AbortAndReobserve {
            remainder_invalidated: true,
            ..
        }
    ));
}

#[test]
fn divergence_and_authority_loss_stop_the_frozen_remainder() {
    let mut obs = observation();
    obs.object_displacement_m = Some(0.08);
    assert!(matches!(
        supervise_execution(&frozen(), &progress(), &obs),
        SupervisorDecision::AbortAndReobserve {
            remainder_invalidated: true,
            ..
        }
    ));
    let mut obs = observation();
    obs.authority_ok = Some(false);
    assert!(matches!(
        supervise_execution(&frozen(), &progress(), &obs),
        SupervisorDecision::AuthorityLost { .. }
    ));
}
