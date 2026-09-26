//! Continue or abort one already-authorized action. Not a planner.
//!
//! Inputs are quantities a runtime sensor could report: stroke consumed,
//! object motion, yaw, contact persistence, goal error, tracking, reachability
//! margin, quasi-static applicability, and authority. No future state.

use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

use crate::physical_belief::PhysicalParameterBelief;
use crate::physical_consequence::FrozenPrediction;
use crate::recoverability::RecoverabilityClass;

/// Displacement / consumed stroke above this is outside the quasi-static push.
pub const QUASI_STATIC_DISPLACEMENT_RATIO: f64 = 1.75;

/// Geometry slack added to a sticking-contact displacement prediction.
pub const DISPLACEMENT_SLACK_M: f64 = 0.01;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ExecutionEnvelope {
    pub commanded_stroke_m: f64,
    pub predicted_displacement_m: f64,
    pub displacement_slack_m: f64,
    pub max_displacement_ratio: f64,
    pub max_yaw_abs_rad: f64,
    pub require_intended_contact: bool,
    pub max_goal_error_increase: f64,
    pub max_tracking_error_m: f64,
    pub min_reachability_margin_m: f64,
    /// Consumed stroke at or below this, with the object already outside the
    /// interaction region, means no control response could have prevented it.
    pub control_quantum_m: f64,
}

impl ExecutionEnvelope {
    /// Frozen bounds for one quasi-static sticking push of `commanded_stroke_m`.
    pub fn for_quasi_static_stroke(commanded_stroke_m: f64, predicted_displacement_m: f64) -> Self {
        Self {
            commanded_stroke_m,
            predicted_displacement_m,
            displacement_slack_m: DISPLACEMENT_SLACK_M,
            max_displacement_ratio: QUASI_STATIC_DISPLACEMENT_RATIO,
            max_yaw_abs_rad: 0.35,
            require_intended_contact: true,
            max_goal_error_increase: 0.02,
            max_tracking_error_m: 0.03,
            min_reachability_margin_m: 0.0,
            control_quantum_m: 1e-4,
        }
    }
}

/// One runtime sample during an authorized stroke.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RuntimeExecutionObservation {
    pub stroke_consumed_m: f64,
    pub commanded_stroke_m: f64,
    pub object_displacement_m: f64,
    pub yaw_change_rad: f64,
    pub intended_contact_persists: bool,
    pub goal_error_before: f64,
    pub goal_error_now: f64,
    /// Missing tracking evidence is not evidence of zero error.
    pub robot_tracking_error_m: Option<f64>,
    pub reachability_margin_m: f64,
    pub quasi_static_applicable: Option<bool>,
    pub authority_ok: bool,
}

/// Fields the selected action requires from the policy-visible sensor contract.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum ObservationField {
    StrokeConsumed,
    Displacement,
    Yaw,
    Contact,
    GoalError,
    Tracking,
    Reachability,
    QuasiStaticApplicability,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ObservationContract {
    pub source: String,
    pub model_epoch: String,
    pub calibration_epoch: String,
    pub max_age_s: f64,
    pub required_units: BTreeMap<String, String>,
    pub required_fields: Vec<ObservationField>,
}

/// Immutable execution contract for one selected and independently authorized action.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FrozenAction {
    pub action_id: String,
    pub action_key: String,
    pub candidate_id: String,
    pub contact_id: String,
    pub witness_id: String,
    /// Exact serialized witness contents used by the executor.
    pub witness_contents: String,
    pub requested_stroke_m: f64,
    pub prediction: FrozenPrediction,
    pub belief_snapshot: PhysicalParameterBelief,
    pub recoverability: RecoverabilityClass,
    pub envelope: ExecutionEnvelope,
    pub observation_contract: ObservationContract,
    pub authority_granted: bool,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RuntimePolicyObservation {
    pub action_id: String,
    pub witness_id: String,
    pub observation_id: String,
    pub source: String,
    pub timestamp_s: f64,
    pub model_epoch: String,
    pub calibration_epoch: String,
    pub units: BTreeMap<String, String>,
    pub stroke_consumed_m: Option<f64>,
    pub object_displacement_m: Option<f64>,
    pub yaw_change_rad: Option<f64>,
    pub intended_contact_persists: Option<bool>,
    pub goal_error_before: Option<f64>,
    pub goal_error_now: Option<f64>,
    pub robot_tracking_error_m: Option<f64>,
    pub reachability_margin_m: Option<f64>,
    pub quasi_static_applicable: Option<bool>,
    pub authority_ok: Option<bool>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ExecutionProgress {
    pub action_id: String,
    pub witness_id: String,
    pub completed_quanta: u32,
    pub total_quanta: u32,
    pub stroke_consumed_m: Option<f64>,
    /// Intended-contact guard activates after the witness reaches contact.
    pub contact_guard_active: bool,
    pub now_s: f64,
    /// Once true, the frozen action's remainder can never be dispatched.
    pub remainder_invalidated: bool,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum SupervisorDecision {
    Continue {
        action_id: String,
        witness_id: String,
        next_quantum: u32,
    },
    AbortAndReobserve {
        action_id: String,
        witness_id: String,
        failed_guard: String,
        remainder_invalidated: bool,
    },
    Completed {
        action_id: String,
        witness_id: String,
    },
    EvidenceUnavailable {
        action_id: String,
        witness_id: String,
        reason: String,
    },
    AuthorityLost {
        action_id: String,
        witness_id: String,
    },
}

fn policy_field_present(field: ObservationField, observation: &RuntimePolicyObservation) -> bool {
    match field {
        ObservationField::StrokeConsumed => observation.stroke_consumed_m.is_some(),
        ObservationField::Displacement => observation.object_displacement_m.is_some(),
        ObservationField::Yaw => observation.yaw_change_rad.is_some(),
        ObservationField::Contact => observation.intended_contact_persists.is_some(),
        ObservationField::GoalError => {
            observation.goal_error_before.is_some() && observation.goal_error_now.is_some()
        }
        ObservationField::Tracking => observation.robot_tracking_error_m.is_some(),
        ObservationField::Reachability => observation.reachability_margin_m.is_some(),
        ObservationField::QuasiStaticApplicability => observation.quasi_static_applicable.is_some(),
    }
}

fn unavailable(action: &FrozenAction, reason: impl Into<String>) -> SupervisorDecision {
    SupervisorDecision::EvidenceUnavailable {
        action_id: action.action_id.clone(),
        witness_id: action.witness_id.clone(),
        reason: reason.into(),
    }
}

/// Supervise continuation of one immutable action. This function reports only
/// continue, abort, completion, unavailable evidence, or lost authority.
pub fn supervise_execution(
    action: &FrozenAction,
    progress: &ExecutionProgress,
    observation: &RuntimePolicyObservation,
) -> SupervisorDecision {
    if !action.authority_granted || observation.authority_ok == Some(false) {
        return SupervisorDecision::AuthorityLost {
            action_id: action.action_id.clone(),
            witness_id: action.witness_id.clone(),
        };
    }
    if observation.authority_ok.is_none() {
        return unavailable(action, "AUTHORITY_STATE_UNOBSERVED");
    }
    if progress.action_id != action.action_id
        || progress.witness_id != action.witness_id
        || observation.action_id != action.action_id
        || observation.witness_id != action.witness_id
        || observation.observation_id.is_empty()
    {
        return unavailable(action, "ACTION_WITNESS_OR_OBSERVATION_ID_MISMATCH");
    }
    if progress.total_quanta == 0 || progress.completed_quanta > progress.total_quanta {
        return unavailable(action, "INVALID_EXECUTION_QUANTUM_PROGRESS");
    }
    if progress.remainder_invalidated {
        return SupervisorDecision::AbortAndReobserve {
            action_id: action.action_id.clone(),
            witness_id: action.witness_id.clone(),
            failed_guard: "REMAINDER_ALREADY_INVALIDATED".into(),
            remainder_invalidated: true,
        };
    }
    let contract = &action.observation_contract;
    if observation.source != contract.source
        || observation.model_epoch != contract.model_epoch
        || observation.calibration_epoch != contract.calibration_epoch
        || !observation.timestamp_s.is_finite()
        || !progress.now_s.is_finite()
        || observation.timestamp_s > progress.now_s + 1e-9
        || progress.now_s - observation.timestamp_s > contract.max_age_s
    {
        return unavailable(
            action,
            "POLICY_OBSERVATION_SOURCE_EPOCH_OR_FRESHNESS_MISMATCH",
        );
    }
    if contract
        .required_units
        .iter()
        .any(|(field, unit)| observation.units.get(field) != Some(unit))
    {
        return unavailable(action, "POLICY_OBSERVATION_UNIT_MISMATCH");
    }
    let envelope_fields = [
        ObservationField::StrokeConsumed,
        ObservationField::Displacement,
        ObservationField::Yaw,
        ObservationField::Contact,
        ObservationField::GoalError,
        ObservationField::Tracking,
        ObservationField::Reachability,
        ObservationField::QuasiStaticApplicability,
    ];
    if envelope_fields
        .iter()
        .chain(contract.required_fields.iter())
        .any(|field| !policy_field_present(*field, observation))
    {
        return unavailable(action, "REQUIRED_POLICY_OBSERVATION_FIELD_MISSING");
    }
    let (Some(consumed), Some(progress_consumed)) =
        (observation.stroke_consumed_m, progress.stroke_consumed_m)
    else {
        return unavailable(action, "STROKE_PROGRESS_UNKNOWN");
    };
    if !consumed.is_finite()
        || consumed < 0.0
        || !progress_consumed.is_finite()
        || progress_consumed < 0.0
        || (consumed - progress_consumed).abs() > 1e-6
    {
        return unavailable(action, "STROKE_PROGRESS_MISMATCH_OR_INVALID");
    }

    let runtime_observation = RuntimeExecutionObservation {
        stroke_consumed_m: consumed,
        commanded_stroke_m: action.requested_stroke_m,
        object_displacement_m: observation
            .object_displacement_m
            .expect("required envelope field checked above"),
        yaw_change_rad: observation
            .yaw_change_rad
            .expect("required envelope field checked above"),
        intended_contact_persists: observation
            .intended_contact_persists
            .expect("required envelope field checked above"),
        goal_error_before: observation
            .goal_error_before
            .expect("required envelope field checked above"),
        goal_error_now: observation
            .goal_error_now
            .expect("required envelope field checked above"),
        robot_tracking_error_m: Some(
            observation
                .robot_tracking_error_m
                .expect("required envelope field checked above"),
        ),
        reachability_margin_m: observation
            .reachability_margin_m
            .expect("required envelope field checked above"),
        quasi_static_applicable: observation.quasi_static_applicable,
        authority_ok: true,
    };
    let mut active_envelope = action.envelope.clone();
    if !progress.contact_guard_active {
        active_envelope.require_intended_contact = false;
    }
    let check = check_execution_envelope(&active_envelope, &runtime_observation);
    if check.verdict == EnvelopeVerdict::AbortAndReobserve {
        return SupervisorDecision::AbortAndReobserve {
            action_id: action.action_id.clone(),
            witness_id: action.witness_id.clone(),
            failed_guard: check
                .failed_guard
                .unwrap_or_else(|| "EXECUTION_ENVELOPE".into()),
            remainder_invalidated: true,
        };
    }
    if progress.completed_quanta == progress.total_quanta {
        SupervisorDecision::Completed {
            action_id: action.action_id.clone(),
            witness_id: action.witness_id.clone(),
        }
    } else {
        SupervisorDecision::Continue {
            action_id: action.action_id.clone(),
            witness_id: action.witness_id.clone(),
            next_quantum: progress.completed_quanta + 1,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum EnvelopeVerdict {
    Continue,
    AbortAndReobserve,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct EnvelopeCheck {
    pub verdict: EnvelopeVerdict,
    pub failed_guard: Option<String>,
    /// True when the failure is observed before the commanded stroke is consumed.
    pub early: bool,
    /// True when the object was already outside every admissible interaction
    /// before a control response on this action could have prevented it.
    pub prevention_impossible: bool,
    pub displacement_m: f64,
}

/// First justified guard failure. Order is fixed so the same sample always
/// names the same guard.
pub fn check_execution_envelope(
    envelope: &ExecutionEnvelope,
    observation: &RuntimeExecutionObservation,
) -> EnvelopeCheck {
    let consumed = observation.stroke_consumed_m;
    let commanded = envelope
        .commanded_stroke_m
        .max(observation.commanded_stroke_m);
    let early = consumed + 1e-9 < commanded;
    let ratio = if consumed > 1e-9 {
        observation.object_displacement_m / consumed
    } else {
        0.0
    };
    let predicted = if commanded > 1e-9 {
        envelope.predicted_displacement_m * (consumed / commanded)
    } else {
        0.0
    };
    let disagreement = consumed > 1e-9
        && (observation.object_displacement_m - predicted).abs()
            > envelope.displacement_slack_m + 0.5 * consumed;
    let outside = observation.reachability_margin_m < envelope.min_reachability_margin_m;
    let prevention_impossible = outside && consumed <= envelope.control_quantum_m;
    let failed = if !observation.authority_ok {
        Some("AUTHORITY_STATE")
    } else if prevention_impossible || outside {
        Some("REACHABILITY_MARGIN")
    } else if (consumed > 1e-9 && ratio > envelope.max_displacement_ratio) || disagreement {
        Some("MODEL_DISAGREEMENT")
    } else if observation.yaw_change_rad.abs() > envelope.max_yaw_abs_rad {
        Some("YAW_CHANGE")
    } else if observation.goal_error_now
        > observation.goal_error_before + envelope.max_goal_error_increase
    {
        Some("GOAL_ERROR_REGRESSION")
    } else if let Some(reason) = tracking_error_failure(
        observation.robot_tracking_error_m,
        envelope.max_tracking_error_m,
    ) {
        Some(reason)
    } else if envelope.require_intended_contact && !observation.intended_contact_persists {
        Some("INTENDED_CONTACT")
    } else if observation.quasi_static_applicable == Some(false) {
        Some("QUASI_STATIC")
    } else {
        None
    };
    let verdict = if failed.is_some() {
        EnvelopeVerdict::AbortAndReobserve
    } else {
        EnvelopeVerdict::Continue
    };
    EnvelopeCheck {
        verdict,
        failed_guard: failed.map(str::to_string),
        early: failed.is_some() && early,
        prevention_impossible,
        displacement_m: observation.object_displacement_m,
    }
}

fn tracking_error_failure(error_m: Option<f64>, max_error_m: f64) -> Option<&'static str> {
    match error_m {
        None => Some("TRACKING_EVIDENCE_MISSING"),
        Some(error) if !error.is_finite() || error < 0.0 => Some("TRACKING_EVIDENCE_INVALID"),
        Some(error) if error > max_error_m => Some("ROBOT_TRACKING"),
        Some(_) => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn nominal(consumed: f64, displacement: f64, commanded: f64) -> RuntimeExecutionObservation {
        RuntimeExecutionObservation {
            stroke_consumed_m: consumed,
            commanded_stroke_m: commanded,
            object_displacement_m: displacement,
            yaw_change_rad: 0.0,
            intended_contact_persists: true,
            goal_error_before: 1.0,
            goal_error_now: 0.9,
            robot_tracking_error_m: Some(0.0),
            reachability_margin_m: 0.05,
            quasi_static_applicable: Some(true),
            authority_ok: true,
        }
    }

    #[test]
    fn nominal_increment_continues_and_excess_motion_aborts_before_stroke_end() {
        let commanded = 0.04;
        let envelope = ExecutionEnvelope::for_quasi_static_stroke(commanded, commanded);
        let first = check_execution_envelope(&envelope, &nominal(0.01, 0.009, commanded));
        assert_eq!(first.verdict, EnvelopeVerdict::Continue);
        assert!(first.failed_guard.is_none());
        assert!(!first.prevention_impossible);

        let mid = nominal(0.02, 0.08, commanded);
        let abort = check_execution_envelope(&envelope, &mid);
        assert_eq!(abort.verdict, EnvelopeVerdict::AbortAndReobserve);
        assert_eq!(abort.failed_guard.as_deref(), Some("MODEL_DISAGREEMENT"));
        assert!(abort.early);
        assert!(mid.stroke_consumed_m < commanded);
        let unguarded_end = 0.16;
        assert!(abort.displacement_m < unguarded_end);
    }

    #[test]
    fn malformed_tracking_evidence_aborts_closed() {
        let commanded = 0.04;
        let envelope = ExecutionEnvelope::for_quasi_static_stroke(commanded, commanded);
        let mut observation = nominal(commanded * 0.25, commanded * 0.2, commanded);
        observation.robot_tracking_error_m = Some(f64::NAN);

        let check = check_execution_envelope(&envelope, &observation);

        assert_eq!(check.verdict, EnvelopeVerdict::AbortAndReobserve);
        assert_eq!(
            check.failed_guard.as_deref(),
            Some("TRACKING_EVIDENCE_INVALID")
        );
    }

    #[test]
    fn missing_tracking_evidence_aborts_closed() {
        let commanded = 0.04;
        let envelope = ExecutionEnvelope::for_quasi_static_stroke(commanded, commanded);
        let mut observation = nominal(commanded * 0.25, commanded * 0.2, commanded);
        observation.robot_tracking_error_m = None;

        let check = check_execution_envelope(&envelope, &observation);

        assert_eq!(check.verdict, EnvelopeVerdict::AbortAndReobserve);
        assert_eq!(
            check.failed_guard.as_deref(),
            Some("TRACKING_EVIDENCE_MISSING")
        );
    }

    #[test]
    fn already_outside_before_a_response_is_not_called_a_recovery() {
        let envelope = ExecutionEnvelope::for_quasi_static_stroke(0.04, 0.04);
        let mut sample = nominal(0.0, 0.2, 0.04);
        sample.reachability_margin_m = -0.01;
        let check = check_execution_envelope(&envelope, &sample);
        assert!(check.prevention_impossible);
        assert_eq!(check.verdict, EnvelopeVerdict::AbortAndReobserve);
        assert_eq!(check.failed_guard.as_deref(), Some("REACHABILITY_MARGIN"));
    }

    #[test]
    fn observation_schema_has_no_privileged_future_or_hidden_parameter() {
        let sample = nominal(0.01, 0.01, 0.04);
        let value = serde_json::to_value(&sample).unwrap();
        let object = value.as_object().unwrap();
        let allowed = [
            "stroke_consumed_m",
            "commanded_stroke_m",
            "object_displacement_m",
            "yaw_change_rad",
            "intended_contact_persists",
            "goal_error_before",
            "goal_error_now",
            "robot_tracking_error_m",
            "reachability_margin_m",
            "quasi_static_applicable",
            "authority_ok",
        ];
        assert_eq!(object.len(), allowed.len());
        for key in object.keys() {
            assert!(allowed.contains(&key.as_str()), "{key}");
        }
        let text = serde_json::to_string(&sample).unwrap();
        for needle in [
            "cfrc_ext",
            "qacc",
            "actuator_force",
            "hidden_friction",
            "qvel",
        ] {
            assert!(!text.contains(needle), "{needle}");
        }
    }
}
