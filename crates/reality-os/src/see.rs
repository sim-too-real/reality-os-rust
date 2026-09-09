//! See-before-act. Gifted xyz without observation evidence cannot ALLOW.

use crate::certificate::Certificate;
use crate::plan::Intent;
use realityos_kernel::{DecisionStatus, ObservationEvidence};

#[derive(Debug, Clone, PartialEq)]
pub struct ObservationGate {
    pub ok: bool,
    pub status: DecisionStatus,
    pub physical_reason: String,
    pub reasons: Vec<String>,
}

pub fn evaluate_manip_observation_gate(
    intent: &Intent,
    observation: Option<&ObservationEvidence>,
    pose_std_m: Option<f64>,
    max_pose_std_m: f64,
    now_s: f64,
) -> ObservationGate {
    if !intent.require_scene {
        return ObservationGate {
            ok: true,
            status: DecisionStatus::Allow,
            physical_reason: "observation_not_required".into(),
            reasons: Vec::new(),
        };
    }
    let Some(ev) = observation else {
        return ObservationGate {
            ok: false,
            status: DecisionStatus::Refuse,
            physical_reason: "observation_evidence_missing; gifted pose is not see".into(),
            reasons: vec![
                "observation_evidence_missing".into(),
                "gifted_scene_stamp".into(),
            ],
        };
    };
    if ev.is_expired(now_s) {
        return ObservationGate {
            ok: false,
            status: DecisionStatus::Refuse,
            physical_reason: "observation_expired".into(),
            reasons: vec!["observation_expired".into()],
        };
    }
    if ev.is_ood(0.8) {
        return ObservationGate {
            ok: false,
            status: DecisionStatus::Probe,
            physical_reason: "observation_ood".into(),
            reasons: vec!["observation_ood".into()],
        };
    }
    if ev.quality() < 0.2 {
        return ObservationGate {
            ok: false,
            status: DecisionStatus::Probe,
            physical_reason: "observation_low_quality".into(),
            reasons: vec!["observation_low_quality".into()],
        };
    }
    if let Some(std) = pose_std_m {
        if !std.is_finite() || std > max_pose_std_m {
            return ObservationGate {
                ok: false,
                status: DecisionStatus::Probe,
                physical_reason: format!("pose_std {std:?} > {max_pose_std_m}"),
                reasons: vec!["pose_std_too_wide".into()],
            };
        }
    }
    ObservationGate {
        ok: true,
        status: DecisionStatus::Allow,
        physical_reason: "scene compiled from observation evidence".into(),
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::plan::Intent;

    #[test]
    fn boolean_spoof_cannot_satisfy_place() {
        let intent = Intent::language("place", "place");
        let g = evaluate_manip_observation_gate(&intent, None, None, 0.05, 1.0);
        assert!(!g.ok);
        assert_eq!(g.status, DecisionStatus::Refuse);
    }
}
