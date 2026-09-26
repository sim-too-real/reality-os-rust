//! Continue or abort one already-authorized action. Not a planner.
//!
//! Inputs are quantities a runtime sensor could report: stroke consumed,
//! object motion, yaw, contact persistence, goal error, tracking, reachability
//! margin, quasi-static applicability, and authority. No future state.

use serde::{Deserialize, Serialize};

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
