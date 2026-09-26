//! Bounded future-interaction argument for one physical probe.
//!
//! A contact that is reachable now is not an input. Nominal goal progress is
//! not an input. Missing bounds stay `Unknown`.

use serde::{Deserialize, Serialize};

use crate::physical_decision::ProbeRecoverabilityAssessment;

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
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct FutureInteractionReport {
    pub assessment: ProbeRecoverabilityAssessment,
    pub evidence: Vec<String>,
}

fn unknown(evidence: &[&str]) -> FutureInteractionReport {
    FutureInteractionReport {
        assessment: ProbeRecoverabilityAssessment::Unknown,
        evidence: evidence.iter().map(|line| (*line).to_string()).collect(),
    }
}

/// Assess whether every supported post-probe outcome still has a safe,
/// executable future interaction. Incomplete inputs stay `Unknown`.
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
    assess_future_interaction(&inputs.outcomes)
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
    let assessment = if unmeasured {
        ProbeRecoverabilityAssessment::Unknown
    } else if at_risk {
        ProbeRecoverabilityAssessment::AtRisk
    } else {
        ProbeRecoverabilityAssessment::Preserved
    };
    if assessment != ProbeRecoverabilityAssessment::Preserved {
        evidence.push("CURRENT_REACHABLE_CONTACT_IS_NOT_FUTURE_INTERACTION_PROOF".into());
    }
    FutureInteractionReport {
        assessment,
        evidence,
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
        }
    }

    #[test]
    fn displacement_before_any_consumed_stroke_stays_unknown() {
        let report = assess_probe_future(&ProbeFutureInputs {
            consumed_stroke_m: 0.0,
            observed_displacement_m: Some(0.013),
            geometry_residual_m: Some(0.0),
            outcomes: vec![preserved_outcome("current-contact")],
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
    fn one_failed_corner_is_at_risk_and_a_complete_set_is_preserved() {
        let mut failed = preserved_outcome("far");
        failed.collision_admissible = Some(false);
        failed.return_contact_witness_digest = None;
        let risk = assess_probe_future(&inputs(vec![preserved_outcome("near"), failed]));
        assert_eq!(risk.assessment, ProbeRecoverabilityAssessment::AtRisk);
        let preserved = assess_probe_future(&inputs(vec![
            preserved_outcome("stay"),
            preserved_outcome("stroke"),
        ]));
        assert_eq!(
            preserved.assessment,
            ProbeRecoverabilityAssessment::Preserved
        );
        assert!(preserved
            .evidence
            .iter()
            .all(|line| !line.contains("FAILED")));
    }
}
