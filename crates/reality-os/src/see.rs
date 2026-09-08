//! See-before-act. Gifted xyz without pixels cannot ALLOW when compile is required.

use crate::certificate::Certificate;
use crate::plan::Intent;
use realityos_kernel::DecisionStatus;

#[derive(Debug, Clone, PartialEq)]
pub struct ObservationGate {
    pub ok: bool,
    pub status: DecisionStatus,
    pub physical_reason: String,
    pub reasons: Vec<String>,
}

pub fn evaluate_manip_observation_gate(
    intent: &Intent,
    pixels_present: bool,
    compiled_this_decide: bool,
    pose_std_m: Option<f64>,
    max_pose_std_m: f64,
) -> ObservationGate {
    if !intent.require_scene {
        return ObservationGate {
            ok: true,
            status: DecisionStatus::Allow,
            physical_reason: "observation_not_required".into(),
            reasons: Vec::new(),
        };
    }
    if !pixels_present {
        return ObservationGate {
            ok: false,
            status: DecisionStatus::Refuse,
            physical_reason: "pixels_never_entered; gifted pose is not see".into(),
            reasons: vec!["pixels_never_entered".into(), "gifted_scene_stamp".into()],
        };
    }
    if !compiled_this_decide {
        return ObservationGate {
            ok: false,
            status: DecisionStatus::Probe,
            physical_reason: "scene not compiled this decide cycle".into(),
            reasons: vec!["compiled_this_decide_false".into()],
        };
    }
    if let Some(std) = pose_std_m {
        if std > max_pose_std_m {
            return ObservationGate {
                ok: false,
                status: DecisionStatus::Probe,
                physical_reason: format!("pose_std {std} > {max_pose_std_m}"),
                reasons: vec!["pose_std_too_wide".into()],
            };
        }
    }
    ObservationGate {
        ok: true,
        status: DecisionStatus::Allow,
        physical_reason: "scene compiled from pixels".into(),
        reasons: Vec::new(),
    }
}

pub fn certificate_from_gate(g: &ObservationGate) -> Option<Certificate> {
    if g.ok {
        None
    } else {
        Some(Certificate::new(g.status, g.physical_reason.clone()).with_reasons(g.reasons.clone()))
    }
}
