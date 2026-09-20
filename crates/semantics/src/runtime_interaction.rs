//! Phase-aware runtime revalidation and bounded contact-seek.
//! Planning truth is not execution truth. Authority remains the write gate.

use crate::geometry::CoverageReport;
use crate::transform::{norm3, sub3, Se3};
use crate::transition_validity::{GeometryVerdict, TransitionReport};

/// Claim strength. Do not collapse these levels.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ClaimLayer {
    GeometricHypothesis,
    KinematicallyReachable,
    ContactConstraintSatisfied,
    PathGeometryAdmissible,
    ExecutionAdmissible,
    ExecutionStarted,
    RuntimeContactConfirmed,
    PhysicalEffectObserved,
    TaskVerified,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RuntimeStop {
    IntendedContactConfirmed,
    ForbiddenContact,
    StaleEvidence,
    UnexpectedObjectMotion,
    AuthorityRefusal,
    TrackingDivergence,
    ModelIdentityMismatch,
    LiveQDivergedFromPlan,
    RemainingTransitionInvalid,
    ContactSensorUnavailable,
}

impl RuntimeStop {
    pub fn name(self) -> &'static str {
        match self {
            Self::IntendedContactConfirmed => "INTENDED_CONTACT_CONFIRMED",
            Self::ForbiddenContact => "FORBIDDEN_CONTACT",
            Self::StaleEvidence => "STALE_EVIDENCE",
            Self::UnexpectedObjectMotion => "UNEXPECTED_OBJECT_MOTION",
            Self::AuthorityRefusal => "AUTHORITY_REFUSAL",
            Self::TrackingDivergence => "TRACKING_DIVERGENCE",
            Self::ModelIdentityMismatch => "MODEL_IDENTITY_MISMATCH",
            Self::LiveQDivergedFromPlan => "LIVE_Q_DIVERGED_FROM_PLAN",
            Self::RemainingTransitionInvalid => "REMAINING_TRANSITION_INVALID",
            Self::ContactSensorUnavailable => "CONTACT_SENSOR_UNAVAILABLE",
        }
    }

    pub fn continue_execution(self) -> bool {
        false
    }
}

/// Policy-visible contact observation. Privileged simulator contact is not this.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ContactObservation {
    PolicyVisible {
        intended: bool,
        forbidden: bool,
    },
    Unavailable,
    /// Must not be used as runtime evidence.
    PrivilegedVerifierOnly,
}

impl ContactObservation {
    pub fn runtime_usable(self) -> bool {
        matches!(self, Self::PolicyVisible { .. })
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct ExecutionSnapshot {
    pub live_q: Vec<f64>,
    pub planned_start_q: Vec<f64>,
    pub live_model_hash: String,
    pub planned_model_hash: String,
    pub evidence_fresh: bool,
    pub object_pose: Option<Se3>,
    pub last_object_pose: Option<Se3>,
    pub object_motion_limit_m: f64,
    pub q_divergence_limit: f64,
    pub tracking_err_m: f64,
    pub tracking_limit_m: f64,
    pub remaining: Option<TransitionReport>,
    pub contact: ContactObservation,
    pub authority_refused: bool,
}

pub fn q_max_abs_error(a: &[f64], b: &[f64]) -> f64 {
    a.iter()
        .zip(b.iter())
        .map(|(x, y)| (x - y).abs())
        .fold(0.0, f64::max)
}

/// Revalidate remaining transitions from live observations before a meaningful step.
pub fn revalidate_remaining(snap: &ExecutionSnapshot) -> Result<ClaimLayer, RuntimeStop> {
    if !snap.evidence_fresh {
        return Err(RuntimeStop::StaleEvidence);
    }
    if snap.live_model_hash != snap.planned_model_hash {
        return Err(RuntimeStop::ModelIdentityMismatch);
    }
    if snap.authority_refused {
        return Err(RuntimeStop::AuthorityRefusal);
    }
    if q_max_abs_error(&snap.live_q, &snap.planned_start_q) > snap.q_divergence_limit {
        return Err(RuntimeStop::LiveQDivergedFromPlan);
    }
    if snap.tracking_err_m > snap.tracking_limit_m {
        return Err(RuntimeStop::TrackingDivergence);
    }
    if let (Some(now), Some(prev)) = (snap.object_pose, snap.last_object_pose) {
        if norm3(sub3(now.xyz, prev.xyz)) > snap.object_motion_limit_m {
            return Err(RuntimeStop::UnexpectedObjectMotion);
        }
    }
    if let Some(rep) = &snap.remaining {
        match rep.verdict {
            GeometryVerdict::ForbiddenContact => return Err(RuntimeStop::ForbiddenContact),
            GeometryVerdict::UnsupportedGeometry | GeometryVerdict::UnknownCoverage => {
                return Err(RuntimeStop::RemainingTransitionInvalid);
            }
            GeometryVerdict::ClearUnderCompleteDeclared
            | GeometryVerdict::NoCollisionInProxy
            | GeometryVerdict::AllowedPhaseContact => {}
        }
    }
    Ok(ClaimLayer::ExecutionAdmissible)
}

#[derive(Debug, Clone, PartialEq)]
pub struct ContactSeekInput {
    pub toward_contact_q: Vec<f64>,
    pub live_q: Vec<f64>,
    pub snapshot: ExecutionSnapshot,
}

#[derive(Debug, Clone, PartialEq)]
pub struct ContactSeekOutcome {
    pub stop: Option<RuntimeStop>,
    pub requested_q: Option<Vec<f64>>,
    pub contact_evidence: ContactObservation,
    pub claim: ClaimLayer,
}

/// Bounded contact-seek: advance toward the intended manifold, observe, stop
/// on confirmed intended contact / forbidden / stale / motion / authority /
/// tracking divergence. Does not mint actuator writes.
pub fn contact_seek_step(input: &ContactSeekInput) -> ContactSeekOutcome {
    if let Err(stop) = revalidate_remaining(&input.snapshot) {
        return ContactSeekOutcome {
            stop: Some(stop),
            requested_q: None,
            contact_evidence: input.snapshot.contact,
            claim: ClaimLayer::ExecutionAdmissible,
        };
    }
    match input.snapshot.contact {
        ContactObservation::PolicyVisible {
            intended: true,
            forbidden: false,
        } => {
            return ContactSeekOutcome {
                stop: Some(RuntimeStop::IntendedContactConfirmed),
                requested_q: None,
                contact_evidence: input.snapshot.contact,
                claim: ClaimLayer::RuntimeContactConfirmed,
            };
        }
        ContactObservation::PolicyVisible {
            forbidden: true, ..
        } => {
            return ContactSeekOutcome {
                stop: Some(RuntimeStop::ForbiddenContact),
                requested_q: None,
                contact_evidence: input.snapshot.contact,
                claim: ClaimLayer::PathGeometryAdmissible,
            };
        }
        ContactObservation::PrivilegedVerifierOnly => {
            return ContactSeekOutcome {
                stop: Some(RuntimeStop::ContactSensorUnavailable),
                requested_q: None,
                contact_evidence: ContactObservation::Unavailable,
                claim: ClaimLayer::ExecutionAdmissible,
            };
        }
        ContactObservation::Unavailable | ContactObservation::PolicyVisible { .. } => {}
    }
    ContactSeekOutcome {
        stop: None,
        requested_q: Some(input.toward_contact_q.clone()),
        contact_evidence: input.snapshot.contact,
        claim: ClaimLayer::ExecutionStarted,
    }
}

/// Geometry / planner / verifier / contact observer cannot mint authority.
pub fn unauthorized_writes_from_geometry(_coverage: &CoverageReport) -> u64 {
    0
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::{CoverageQualification, CoverageReport};
    use crate::transition_validity::GeometryVerdict;

    fn base_snap() -> ExecutionSnapshot {
        ExecutionSnapshot {
            live_q: vec![0.1, 0.2],
            planned_start_q: vec![0.1, 0.2],
            live_model_hash: "h1".into(),
            planned_model_hash: "h1".into(),
            evidence_fresh: true,
            object_pose: Some(Se3::identity()),
            last_object_pose: Some(Se3::identity()),
            object_motion_limit_m: 0.01,
            q_divergence_limit: 0.05,
            tracking_err_m: 0.0,
            tracking_limit_m: 0.02,
            remaining: None,
            contact: ContactObservation::Unavailable,
            authority_refused: false,
        }
    }

    #[test]
    fn stale_evidence_does_not_continue() {
        let mut s = base_snap();
        s.evidence_fresh = false;
        assert_eq!(
            revalidate_remaining(&s).unwrap_err(),
            RuntimeStop::StaleEvidence
        );
    }

    #[test]
    fn live_q_divergence_does_not_finish_stored_witness() {
        let mut s = base_snap();
        s.live_q = vec![0.9, 0.2];
        assert_eq!(
            revalidate_remaining(&s).unwrap_err(),
            RuntimeStop::LiveQDivergedFromPlan
        );
    }

    #[test]
    fn model_identity_mismatch_stops() {
        let mut s = base_snap();
        s.live_model_hash = "other".into();
        assert_eq!(
            revalidate_remaining(&s).unwrap_err(),
            RuntimeStop::ModelIdentityMismatch
        );
    }

    #[test]
    fn forbidden_remaining_transition_stops() {
        let mut s = base_snap();
        s.remaining = Some(TransitionReport {
            verdict: GeometryVerdict::ForbiddenContact,
            witness: None,
            coverage: CoverageReport::empty(CoverageQualification::CompleteDeclared),
            class: None,
        });
        assert_eq!(
            revalidate_remaining(&s).unwrap_err(),
            RuntimeStop::ForbiddenContact
        );
    }

    #[test]
    fn contact_seek_stops_on_intended_policy_visible_contact() {
        let mut s = base_snap();
        s.contact = ContactObservation::PolicyVisible {
            intended: true,
            forbidden: false,
        };
        let out = contact_seek_step(&ContactSeekInput {
            toward_contact_q: vec![0.3, 0.2],
            live_q: s.live_q.clone(),
            snapshot: s,
        });
        assert_eq!(out.stop, Some(RuntimeStop::IntendedContactConfirmed));
        assert!(out.requested_q.is_none());
        assert_eq!(out.claim, ClaimLayer::RuntimeContactConfirmed);
    }

    #[test]
    fn privileged_mujoco_contact_is_not_runtime_evidence() {
        let mut s = base_snap();
        s.contact = ContactObservation::PrivilegedVerifierOnly;
        let out = contact_seek_step(&ContactSeekInput {
            toward_contact_q: vec![0.3, 0.2],
            live_q: s.live_q.clone(),
            snapshot: s,
        });
        assert_eq!(out.stop, Some(RuntimeStop::ContactSensorUnavailable));
        assert_eq!(out.contact_evidence, ContactObservation::Unavailable);
    }

    #[test]
    fn contact_seek_stops_on_authority_refusal() {
        let mut s = base_snap();
        s.authority_refused = true;
        let out = contact_seek_step(&ContactSeekInput {
            toward_contact_q: vec![0.3],
            live_q: s.live_q.clone(),
            snapshot: s,
        });
        assert_eq!(out.stop, Some(RuntimeStop::AuthorityRefusal));
        assert!(out.requested_q.is_none());
    }

    #[test]
    fn geometry_cannot_mint_authority() {
        let c = CoverageReport::empty(CoverageQualification::CompleteDeclared);
        assert_eq!(unauthorized_writes_from_geometry(&c), 0);
    }

    #[test]
    fn unexpected_object_motion_stops() {
        let mut s = base_snap();
        s.object_pose = Some(Se3::translation([0.05, 0.0, 0.0]).unwrap());
        s.last_object_pose = Some(Se3::identity());
        assert_eq!(
            revalidate_remaining(&s).unwrap_err(),
            RuntimeStop::UnexpectedObjectMotion
        );
    }
}
