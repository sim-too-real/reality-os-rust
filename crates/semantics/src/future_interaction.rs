//! Bounded future-interaction argument for one physical probe.
//!
//! A contact that is reachable now is not an input. Nominal goal progress is
//! not an input. Missing bounds stay `Unknown`.

use serde::{Deserialize, Serialize};

use crate::contact_manifold::{box_push_face_manifold_posed, manifold_coords};
use crate::physical_decision::ProbeRecoverabilityAssessment;
use crate::transform::norm3;
use crate::transform::Se3;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SupportedProbeOutcome {
    pub label: String,
    pub object_supported: Option<bool>,
    pub inside_reachable_workspace: Option<bool>,
    pub joint_margin_rad: Option<f64>,
    pub collision_admissible: Option<bool>,
    pub motion_within_declared_bound: Option<bool>,
    pub belief_outcome_bounded: Option<bool>,
    /// `Some(false)` requires `return_contact_witness_digest` for preservation.
    pub contact_persists: Option<bool>,
    pub return_contact_witness_digest: Option<String>,
}

/// Bounds of the post-probe object states the domain claims to cover.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct UncertaintyBounds {
    pub translation_radius_m: f64,
    pub yaw_abs_rad: f64,
    pub residual_high_m: f64,
}

/// How the supported set was covered. Sampled states are not a cover.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum CoverageMethod {
    Uncovered,
    SampledStates {
        count: u32,
        sufficiency_property: Option<String>,
    },
    /// A fixed return-contact witness whose face margin contains the ball,
    /// with collision proved on the body grown by that ball.
    UncertaintyBallInsideWitnessClearance,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FutureInteractionWitnessRecord {
    pub label: String,
    pub digest: String,
    /// Meters of face margin remaining after the uncertainty ball is removed.
    pub clearance_m: Option<f64>,
    pub obligations_met: bool,
}

/// The record a `Preserved` assessment has to carry. Outcomes alone are not this.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SupportedOutcomeDomain {
    pub derivation: String,
    pub bounds: UncertaintyBounds,
    pub coverage: CoverageMethod,
    pub obligations: Vec<String>,
    pub witnesses: Vec<FutureInteractionWitnessRecord>,
    pub model_applicable: bool,
    pub model_applicability: String,
    pub unresolved: Vec<String>,
    pub failure_reasons: Vec<String>,
}

/// Geometry and collision facts for one fixed witness over a translation ball.
#[derive(Debug, Clone, PartialEq)]
pub struct CommonWitnessCoverage {
    pub label: String,
    pub witness_digest: String,
    pub contact_point: [f64; 3],
    pub object_center: [f64; 3],
    pub object_quat: [f64; 4],
    pub object_half: [f64; 3],
    pub push_direction: [f64; 3],
    pub support_normal: [f64; 3],
    pub face_gap_m: f64,
    pub translation_radius_m: f64,
    pub yaw_abs_rad: f64,
    pub geometry_residual_m: f64,
    pub support_clearance_m: f64,
    pub min_support_clearance_m: f64,
    pub joint_margin_rad: f64,
    /// True only when collision admissibility was proved on the object grown by the ball.
    pub collision_admissible_for_grown_object: bool,
    pub model_applicable: bool,
    pub model_applicability: String,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ProbeFutureInputs {
    /// Stroke the aborted or proposed action had already consumed.
    pub consumed_stroke_m: f64,
    /// Policy-visible object travel. `None` means the sensor did not report it.
    pub observed_displacement_m: Option<f64>,
    /// Policy-visible contact residual. `None` means geometry was not measured.
    pub geometry_residual_m: Option<f64>,
    /// Justified post-probe extremes the caller actually proved or failed to prove.
    pub outcomes: Vec<SupportedProbeOutcome>,
    /// Present only when a covering argument was actually proved.
    #[serde(default)]
    pub domain: Option<SupportedOutcomeDomain>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FutureInteractionReport {
    pub assessment: ProbeRecoverabilityAssessment,
    pub evidence: Vec<String>,
    #[serde(default)]
    pub domain: Option<SupportedOutcomeDomain>,
}

fn unknown(evidence: &[&str]) -> FutureInteractionReport {
    FutureInteractionReport {
        assessment: ProbeRecoverabilityAssessment::Unknown,
        evidence: evidence.iter().map(|line| (*line).to_string()).collect(),
        domain: None,
    }
}

fn finite_nonnegative(value: f64) -> bool {
    value.is_finite() && value >= 0.0
}

/// Face-margin cover for one fixed witness. Sampled corners are not accepted here.
pub fn domain_from_common_witness(
    input: &CommonWitnessCoverage,
) -> Result<SupportedOutcomeDomain, String> {
    if input.label.is_empty() || input.witness_digest.is_empty() {
        return Err("WITNESS_IDENTITY_MISSING".into());
    }
    if !input.model_applicable || input.model_applicability.trim().is_empty() {
        return Err("MODEL_NOT_APPLICABLE".into());
    }
    if !finite_nonnegative(input.translation_radius_m)
        || !finite_nonnegative(input.yaw_abs_rad)
        || !finite_nonnegative(input.geometry_residual_m)
        || !finite_nonnegative(input.face_gap_m)
        || !input.support_clearance_m.is_finite()
        || !input.joint_margin_rad.is_finite()
    {
        return Err("COVERAGE_BOUNDS_INVALID".into());
    }
    if input.yaw_abs_rad > 0.2 {
        return Err("YAW_UNCERTAINTY_LEAVES_THE_SMALL_ANGLE_MODEL".into());
    }
    if input.support_clearance_m <= input.min_support_clearance_m {
        return Err("SUPPORT_CLEARANCE_DOES_NOT_COVER".into());
    }
    if input.joint_margin_rad <= 0.0 {
        return Err("JOINT_MARGIN_DOES_NOT_COVER".into());
    }
    if !input.collision_admissible_for_grown_object {
        return Err("COLLISION_NOT_PROVED_ON_GROWN_OBJECT".into());
    }
    let pose = Se3::try_new(input.object_center, input.object_quat)
        .map_err(|_| "OBJECT_POSE_INVALID".to_string())?;
    let Some(manifold) = box_push_face_manifold_posed(
        pose,
        input.object_half,
        input.push_direction,
        input.support_normal,
        input.face_gap_m,
    )
    .map_err(|_| "FACE_MANIFOLD_REJECTED".to_string())?
    else {
        return Err("FACE_MANIFOLD_MISSING".into());
    };
    let coords = manifold_coords(&manifold, input.contact_point);
    if (coords.normal - input.face_gap_m).abs() > 1e-3 {
        return Err("CONTACT_POINT_IS_NOT_ON_THE_DECLARED_FACE".into());
    }
    // A vertical rotation about the object center moves the contact by at most
    // the face-offset arc. The bound is first-order and is refused above.
    let face_offset = norm3([
        input.contact_point[0] - input.object_center[0],
        input.contact_point[1] - input.object_center[1],
        input.contact_point[2] - input.object_center[2],
    ]);
    let yaw_arc = face_offset * input.yaw_abs_rad;
    let tangent_clearance =
        (manifold.u_half - coords.u.abs()).min(manifold.v_half - coords.v.abs()) - yaw_arc;
    if !tangent_clearance.is_finite() || tangent_clearance + 1e-12 < input.translation_radius_m {
        return Err("TRANSLATION_BALL_EXCEEDS_FACE_MARGIN".into());
    }
    if input.geometry_residual_m > input.translation_radius_m + 1e-12 {
        return Err("RESIDUAL_EXCEEDS_PROVED_BALL".into());
    }
    let obligations = vec![
        "OBJECT_SUPPORTED".into(),
        "FIXED_WITNESS_JOINT_MARGIN".into(),
        "COLLISION_ADMISSIBLE_ON_GROWN_BODY".into(),
        "FACE_MARGIN_CONTAINS_TRANSLATION_BALL".into(),
        "MOTION_WITHIN_DECLARED_BOUND".into(),
        "MODEL_APPLICABLE".into(),
    ];
    Ok(SupportedOutcomeDomain {
        derivation: format!(
            "fixed witness {} covers an L2 translation ball of radius {:.6} m because the contact point stays inside the convex face by {:.6} m and collision was proved on the object grown by that radius",
            input.witness_digest, input.translation_radius_m, tangent_clearance
        ),
        bounds: UncertaintyBounds {
            translation_radius_m: input.translation_radius_m,
            yaw_abs_rad: input.yaw_abs_rad,
            residual_high_m: input.geometry_residual_m,
        },
        coverage: CoverageMethod::UncertaintyBallInsideWitnessClearance,
        obligations,
        witnesses: vec![FutureInteractionWitnessRecord {
            label: input.label.clone(),
            digest: input.witness_digest.clone(),
            clearance_m: Some(tangent_clearance),
            obligations_met: true,
        }],
        model_applicable: true,
        model_applicability: input.model_applicability.clone(),
        unresolved: Vec::new(),
        failure_reasons: Vec::new(),
    })
}

/// Assess whether every supported post-probe outcome still has a safe,
/// executable future interaction. Incomplete inputs stay `Unknown`.
/// `Preserved` is returned only when `inputs.domain` covers the outcomes.
pub fn assess_probe_future(inputs: &ProbeFutureInputs) -> FutureInteractionReport {
    let displacement = inputs
        .observed_displacement_m
        .filter(|value| value.is_finite());
    if displacement.is_some_and(|value| value > 1e-4) && inputs.consumed_stroke_m <= 1e-9 {
        return unknown(&[
            "DISPLACEMENT_WITHOUT_CONSUMED_STROKE",
            "CURRENT_REACHABLE_CONTACT_IS_NOT_FUTURE_INTERACTION_PROOF",
        ]);
    }
    if !inputs.consumed_stroke_m.is_finite() || inputs.consumed_stroke_m < 0.0 {
        return unknown(&["CONSUMED_STROKE_INVALID"]);
    }
    match inputs.geometry_residual_m {
        None => {
            return unknown(&[
                "GEOMETRY_RESIDUAL_UNMEASURED",
                "CURRENT_REACHABLE_CONTACT_IS_NOT_FUTURE_INTERACTION_PROOF",
            ]);
        }
        Some(residual) if !residual.is_finite() || residual < 0.0 => {
            return unknown(&["GEOMETRY_RESIDUAL_INVALID"]);
        }
        Some(_) => {}
    }
    let mut report = assess_future_interaction(&inputs.outcomes);
    if report.assessment == ProbeRecoverabilityAssessment::AtRisk {
        report.evidence.push("AT_RISK_OUTCOME_NOT_REPLACED".into());
        return report;
    }
    if report.assessment != ProbeRecoverabilityAssessment::Unknown {
        report.assessment = ProbeRecoverabilityAssessment::Unknown;
    }
    if report.evidence.iter().any(|line| {
        line.contains("UNMEASURED") || line.contains("INVALID") || line.contains("NO_SUPPORTED")
    }) {
        report
            .evidence
            .push("CURRENT_REACHABLE_CONTACT_IS_NOT_FUTURE_INTERACTION_PROOF".into());
        return report;
    }
    match coverage_failure(inputs) {
        None => {
            report.assessment = ProbeRecoverabilityAssessment::Preserved;
            report.domain = inputs.domain.clone();
            report
        }
        Some(reason) => {
            report.assessment = ProbeRecoverabilityAssessment::Unknown;
            report.evidence.push(reason);
            report
                .evidence
                .push("SAMPLED_OR_LOCAL_WITNESS_DOES_NOT_COVER_DOMAIN".into());
            report
        }
    }
}

fn duplicated_single_witness(outcomes: &[SupportedProbeOutcome]) -> bool {
    let Some(first) = outcomes.first() else {
        return false;
    };
    outcomes.len() >= 2
        && outcomes.iter().all(|outcome| {
            outcome.label == first.label
                && outcome.return_contact_witness_digest == first.return_contact_witness_digest
                && outcome.inside_reachable_workspace == Some(true)
                && outcome.collision_admissible == Some(true)
                && outcome.belief_outcome_bounded == Some(true)
        })
}

fn coverage_failure(inputs: &ProbeFutureInputs) -> Option<String> {
    if duplicated_single_witness(&inputs.outcomes) {
        return Some("DUPLICATED_SINGLE_WITNESS_IS_NOT_A_DOMAIN".into());
    }
    let Some(domain) = inputs.domain.as_ref() else {
        return Some("DOMAIN_ABSENT".into());
    };
    if domain.derivation.trim().is_empty() {
        return Some("DOMAIN_DERIVATION_ABSENT".into());
    }
    if !domain.unresolved.is_empty() {
        return Some(format!("UNRESOLVED:{}", domain.unresolved.join(",")));
    }
    if !domain.failure_reasons.is_empty() {
        return Some(format!(
            "DOMAIN_FAILURE:{}",
            domain.failure_reasons.join(",")
        ));
    }
    if !domain.model_applicable || domain.model_applicability.trim().is_empty() {
        return Some("MODEL_NOT_APPLICABLE".into());
    }
    if domain.obligations.is_empty() || domain.witnesses.is_empty() {
        return Some("OBLIGATIONS_OR_WITNESSES_ABSENT".into());
    }
    let bounds = &domain.bounds;
    if !finite_nonnegative(bounds.translation_radius_m)
        || !finite_nonnegative(bounds.yaw_abs_rad)
        || !finite_nonnegative(bounds.residual_high_m)
    {
        return Some("UNCERTAINTY_BOUNDS_INVALID".into());
    }
    let residual = inputs.geometry_residual_m.unwrap_or(f64::INFINITY);
    if residual > bounds.residual_high_m + 1e-12 {
        return Some("RESIDUAL_OUTSIDE_DOMAIN".into());
    }
    match &domain.coverage {
        CoverageMethod::Uncovered => Some("COVERAGE_UNCOVERED".into()),
        CoverageMethod::SampledStates { .. } => {
            Some("SAMPLED_STATES_DO_NOT_COVER_THE_DOMAIN".into())
        }
        CoverageMethod::UncertaintyBallInsideWitnessClearance => {
            if domain.witnesses.len() != 1 || inputs.outcomes.len() != 1 {
                return Some("BALL_COVER_IS_ONE_WITNESS_NOT_A_COPIED_SET".into());
            }
            let witness = &domain.witnesses[0];
            let outcome = &inputs.outcomes[0];
            if !witness.obligations_met
                || witness.digest.is_empty()
                || outcome.return_contact_witness_digest.as_deref() != Some(witness.digest.as_str())
                || outcome.label != witness.label
            {
                return Some("WITNESS_DOES_NOT_MATCH_OUTCOME".into());
            }
            let Some(clearance) = witness.clearance_m.filter(|value| value.is_finite()) else {
                return Some("WITNESS_CLEARANCE_UNMEASURED".into());
            };
            if clearance + 1e-12 < bounds.translation_radius_m {
                return Some("CLEARANCE_DOES_NOT_CONTAIN_BALL".into());
            }
            let required = [
                "OBJECT_SUPPORTED",
                "FIXED_WITNESS_JOINT_MARGIN",
                "COLLISION_ADMISSIBLE_ON_GROWN_BODY",
                "FACE_MARGIN_CONTAINS_TRANSLATION_BALL",
                "MOTION_WITHIN_DECLARED_BOUND",
                "MODEL_APPLICABLE",
            ];
            if required
                .iter()
                .any(|obligation| !domain.obligations.iter().any(|have| have == obligation))
            {
                return Some("PROOF_OBLIGATION_MISSING".into());
            }
            None
        }
    }
}

pub fn assess_future_interaction(outcomes: &[SupportedProbeOutcome]) -> FutureInteractionReport {
    if outcomes.is_empty() {
        return unknown(&[
            "NO_SUPPORTED_POST_PROBE_OUTCOMES",
            "CURRENT_REACHABLE_CONTACT_IS_NOT_FUTURE_INTERACTION_PROOF",
        ]);
    }
    let mut evidence = Vec::new();
    let mut unmeasured = false;
    let mut at_risk = false;
    for outcome in outcomes {
        let label = outcome.label.as_str();
        if label.is_empty() {
            unmeasured = true;
            evidence.push("OUTCOME_LABEL_MISSING".into());
        }
        for (name, value) in [
            ("object_supported", outcome.object_supported),
            (
                "inside_reachable_workspace",
                outcome.inside_reachable_workspace,
            ),
            ("collision_admissible", outcome.collision_admissible),
            (
                "motion_within_declared_bound",
                outcome.motion_within_declared_bound,
            ),
            ("belief_outcome_bounded", outcome.belief_outcome_bounded),
        ] {
            match value {
                None => {
                    unmeasured = true;
                    evidence.push(format!("{label}:{name}:UNMEASURED"));
                }
                Some(true) => evidence.push(format!("{label}:{name}:OK")),
                Some(false) => {
                    at_risk = true;
                    evidence.push(format!("{label}:{name}:FAILED"));
                }
            }
        }
        match outcome.joint_margin_rad {
            None => {
                unmeasured = true;
                evidence.push(format!("{label}:joint_margin_rad:UNMEASURED"));
            }
            Some(margin) if !margin.is_finite() => {
                unmeasured = true;
                evidence.push(format!("{label}:joint_margin_rad:INVALID"));
            }
            Some(margin) if margin <= 0.0 => {
                at_risk = true;
                evidence.push(format!("{label}:joint_margin_rad:FAILED:{margin}"));
            }
            Some(margin) => evidence.push(format!("{label}:joint_margin_rad:OK:{margin}")),
        }
        match (
            outcome.contact_persists,
            outcome.return_contact_witness_digest.as_deref(),
        ) {
            (None, _) => {
                unmeasured = true;
                evidence.push(format!("{label}:future_contact:UNMEASURED"));
            }
            (Some(true), _) => evidence.push(format!("{label}:contact_persists:OK")),
            (Some(false), Some(digest)) if !digest.is_empty() => {
                evidence.push(format!("{label}:return_contact_witness:{digest}"));
            }
            (Some(false), _) => {
                at_risk = true;
                evidence.push(format!("{label}:future_contact:FAILED"));
            }
        }
    }
    // A proved failure stays AtRisk even when another outcome was not measured.
    // Consistent flags are not Preserved until a domain covers them.
    let assessment = if at_risk {
        ProbeRecoverabilityAssessment::AtRisk
    } else {
        ProbeRecoverabilityAssessment::Unknown
    };
    if at_risk || unmeasured {
        evidence.push("CURRENT_REACHABLE_CONTACT_IS_NOT_FUTURE_INTERACTION_PROOF".into());
    } else {
        evidence.push("OUTCOMES_CONSISTENT_DOMAIN_REQUIRED".into());
    }
    FutureInteractionReport {
        assessment,
        evidence,
        domain: None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn preserved_outcome(label: &str) -> SupportedProbeOutcome {
        SupportedProbeOutcome {
            label: label.into(),
            object_supported: Some(true),
            inside_reachable_workspace: Some(true),
            joint_margin_rad: Some(0.2),
            collision_admissible: Some(true),
            motion_within_declared_bound: Some(true),
            belief_outcome_bounded: Some(true),
            contact_persists: Some(false),
            return_contact_witness_digest: Some(format!("witness:{label}")),
        }
    }

    fn inputs(outcomes: Vec<SupportedProbeOutcome>) -> ProbeFutureInputs {
        ProbeFutureInputs {
            consumed_stroke_m: 0.01,
            observed_displacement_m: Some(0.008),
            geometry_residual_m: Some(0.002),
            outcomes,
            domain: None,
        }
    }

    fn centered_coverage(radius: f64, collision: bool) -> CommonWitnessCoverage {
        CommonWitnessCoverage {
            label: "stay".into(),
            witness_digest: "witness:stay".into(),
            contact_point: [-0.025, 0.0, 0.025],
            object_center: [0.0, 0.0, 0.025],
            object_quat: [1.0, 0.0, 0.0, 0.0],
            object_half: [0.025, 0.025, 0.025],
            push_direction: [1.0, 0.0, 0.0],
            support_normal: [0.0, 0.0, 1.0],
            face_gap_m: 0.0,
            translation_radius_m: radius,
            yaw_abs_rad: 0.0,
            geometry_residual_m: 0.002,
            support_clearance_m: 0.01,
            min_support_clearance_m: 0.004,
            joint_margin_rad: 0.2,
            collision_admissible_for_grown_object: collision,
            model_applicable: true,
            model_applicability: "planar push, fixed witness, yaw bound 0".into(),
        }
    }

    #[test]
    fn displacement_before_any_consumed_stroke_stays_unknown() {
        let report = assess_probe_future(&ProbeFutureInputs {
            consumed_stroke_m: 0.0,
            observed_displacement_m: Some(0.013),
            geometry_residual_m: Some(0.0),
            outcomes: vec![preserved_outcome("current-contact")],
            domain: None,
        });
        assert_eq!(report.assessment, ProbeRecoverabilityAssessment::Unknown);
        assert!(report
            .evidence
            .iter()
            .any(|line| line == "DISPLACEMENT_WITHOUT_CONSUMED_STROKE"));
    }

    #[test]
    fn missing_geometry_residual_stays_unknown() {
        let mut sample = inputs(vec![preserved_outcome("corner")]);
        sample.geometry_residual_m = None;
        let report = assess_probe_future(&sample);
        assert_eq!(report.assessment, ProbeRecoverabilityAssessment::Unknown);
        assert!(report
            .evidence
            .iter()
            .any(|line| line == "GEOMETRY_RESIDUAL_UNMEASURED"));
    }

    #[test]
    fn one_failed_corner_stays_at_risk_and_is_not_replaced() {
        let mut failed = preserved_outcome("far");
        failed.collision_admissible = Some(false);
        failed.return_contact_witness_digest = None;
        let mut risk = inputs(vec![preserved_outcome("near"), failed]);
        risk.domain = domain_from_common_witness(&centered_coverage(0.004, true)).ok();
        let risk = assess_probe_future(&risk);
        assert_eq!(risk.assessment, ProbeRecoverabilityAssessment::AtRisk);
        assert!(risk
            .evidence
            .iter()
            .any(|line| line == "AT_RISK_OUTCOME_NOT_REPLACED"));
        assert_ne!(risk.assessment, ProbeRecoverabilityAssessment::Preserved);
    }

    #[test]
    fn duplicated_optimistic_continuation_is_not_preserved() {
        let one = SupportedProbeOutcome {
            label: "witness-prefix".into(),
            object_supported: Some(true),
            inside_reachable_workspace: Some(true),
            joint_margin_rad: Some(0.2),
            collision_admissible: Some(true),
            motion_within_declared_bound: Some(true),
            belief_outcome_bounded: Some(true),
            contact_persists: Some(false),
            return_contact_witness_digest: Some("witness:prefix".into()),
        };
        let report = assess_probe_future(&ProbeFutureInputs {
            consumed_stroke_m: 0.004,
            observed_displacement_m: Some(0.001),
            geometry_residual_m: Some(0.0),
            outcomes: vec![one.clone(), one],
            domain: None,
        });
        assert_eq!(report.assessment, ProbeRecoverabilityAssessment::Unknown);
        assert_ne!(report.assessment, ProbeRecoverabilityAssessment::Preserved);
        assert!(report
            .evidence
            .iter()
            .any(|line| line == "DUPLICATED_SINGLE_WITNESS_IS_NOT_A_DOMAIN"));
    }

    #[test]
    fn uncovered_sample_and_missing_grown_collision_stay_unknown() {
        let mut sampled = inputs(vec![preserved_outcome("stay"), preserved_outcome("stroke")]);
        sampled.domain = Some(SupportedOutcomeDomain {
            derivation: "three corners".into(),
            bounds: UncertaintyBounds {
                translation_radius_m: 0.004,
                yaw_abs_rad: 0.0,
                residual_high_m: 0.002,
            },
            coverage: CoverageMethod::SampledStates {
                count: 2,
                sufficiency_property: None,
            },
            obligations: vec!["SAMPLED".into()],
            witnesses: vec![FutureInteractionWitnessRecord {
                label: "stay".into(),
                digest: "witness:stay".into(),
                clearance_m: None,
                obligations_met: false,
            }],
            model_applicable: true,
            model_applicability: "sampled".into(),
            unresolved: vec!["interior".into()],
            failure_reasons: Vec::new(),
        });
        let sampled = assess_probe_future(&sampled);
        assert_eq!(sampled.assessment, ProbeRecoverabilityAssessment::Unknown);
        assert!(domain_from_common_witness(&centered_coverage(0.004, false)).is_err());
        assert!(domain_from_common_witness(&centered_coverage(0.05, true)).is_err());
    }

    #[test]
    fn common_witness_ball_is_preserved_only_with_the_domain_record() {
        let domain = domain_from_common_witness(&centered_coverage(0.004, true))
            .expect("centered 4 mm ball fits the face");
        assert!(matches!(
            domain.coverage,
            CoverageMethod::UncertaintyBallInsideWitnessClearance
        ));
        assert!(domain.witnesses[0]
            .clearance_m
            .is_some_and(|clearance| clearance + 1e-12 >= 0.004));
        let mut covered = inputs(vec![preserved_outcome("stay")]);
        covered.domain = Some(domain);
        let preserved = assess_probe_future(&covered);
        assert_eq!(
            preserved.assessment,
            ProbeRecoverabilityAssessment::Preserved
        );
        let record = preserved.domain.expect("preserved carries the domain");
        assert!(!record.derivation.is_empty());
        assert!(!record.obligations.is_empty());
        assert!(record.unresolved.is_empty());
        assert!(record.model_applicable);
        let mut bare = inputs(vec![preserved_outcome("stay")]);
        bare.domain = None;
        assert_eq!(
            assess_probe_future(&bare).assessment,
            ProbeRecoverabilityAssessment::Unknown
        );
    }
}
