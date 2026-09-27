use std::collections::BTreeMap;

use realityos_semantics::execution_envelope::{
    supervise_execution, ExecutionEnvelope, ExecutionProgress, FrozenAction, ObservationContract,
    ObservationField, RuntimePolicyObservation, SupervisorDecision,
};
use realityos_semantics::physical_belief::PhysicalParameterBelief;
use realityos_semantics::physical_consequence::FrozenPrediction;
use realityos_semantics::recoverability::RecoverabilityClass;

fn frozen() -> FrozenAction {
    FrozenAction {
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
    }
    .with_matching_authorization(10.0, 40.0)
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
