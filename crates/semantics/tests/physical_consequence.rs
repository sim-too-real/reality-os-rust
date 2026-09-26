use realityos_semantics::physical_consequence::{
    assess_consequence, ConsequenceStatus, FrozenPrediction, ObservedConsequence,
};

fn prediction() -> FrozenPrediction {
    FrozenPrediction {
        action_id: "action:1".into(),
        witness_id: "witness:1".into(),
        stroke_m: 0.02,
        predicted_displacement_m: Some(0.018),
        predicted_yaw_change_rad: Some(0.0),
        predicted_contact_persists: true,
        quasi_static_stroke_limit_m: 0.05,
    }
}

fn observed() -> ObservedConsequence {
    ObservedConsequence {
        action_id: "action:1".into(),
        witness_id: "witness:1".into(),
        observation_id: "observation:1".into(),
        freshness_ok: true,
        execution_aborted: false,
        stroke_consumed_m: Some(0.02),
        displacement_m: Some(0.018),
        yaw_change_rad: Some(0.0),
        contact_persisted: Some(true),
        tracking_error_m: Some(0.0),
        geometry_residual_m: Some(0.0),
        quasi_static_applicable: Some(true),
    }
}

#[test]
fn nominal_consequence_is_assessed_as_consistent() {
    assert_eq!(
        assess_consequence(&prediction(), &observed()).status,
        ConsequenceStatus::Consistent
    );
}

#[test]
fn contradicted_measurement_is_not_hidden_by_the_nominal_prediction() {
    let mut obs = observed();
    obs.displacement_m = Some(0.08);
    assert_eq!(
        assess_consequence(&prediction(), &obs).status,
        ConsequenceStatus::Contradicted
    );
}

#[test]
fn competing_live_explanations_remain_underdetermined() {
    let mut pred = prediction();
    pred.predicted_displacement_m = None;
    pred.predicted_yaw_change_rad = None;
    let assessment = assess_consequence(&pred, &observed());
    assert_eq!(assessment.status, ConsequenceStatus::Underdetermined);
    assert!(assessment.hypotheses.len() > 1);
}

#[test]
fn missing_motion_tracking_contact_or_geometry_stays_unknown() {
    let mut obs = observed();
    obs.displacement_m = None;
    obs.tracking_error_m = None;
    obs.contact_persisted = None;
    obs.geometry_residual_m = None;
    let assessment = assess_consequence(&prediction(), &obs);
    assert_eq!(assessment.status, ConsequenceStatus::InsufficientEvidence);
    assert_eq!(assessment.observation.displacement_m, None);
    assert_eq!(assessment.observation.tracking_error_m, None);
    assert_eq!(assessment.observation.contact_persisted, None);
    assert_eq!(assessment.observation.geometry_residual_m, None);
}

#[test]
fn outside_regime_and_aborted_execution_have_distinct_statuses() {
    let mut obs = observed();
    obs.quasi_static_applicable = Some(false);
    assert_eq!(
        assess_consequence(&prediction(), &obs).status,
        ConsequenceStatus::OutsideModelRegime
    );
    obs.execution_aborted = true;
    assert_eq!(
        assess_consequence(&prediction(), &obs).status,
        ConsequenceStatus::ExecutionDiverged
    );
}

#[test]
fn aborted_execution_retains_policy_visible_contact_discrepancy() {
    let mut obs = observed();
    obs.execution_aborted = true;
    obs.contact_persisted = Some(false);

    let assessment = assess_consequence(&prediction(), &obs);

    assert_eq!(assessment.status, ConsequenceStatus::ExecutionDiverged);
    assert!(assessment
        .hypotheses
        .contains(&realityos_semantics::discrepancy::DiscrepancyKind::ExecutionTrackingDivergence));
    assert!(assessment.hypotheses.contains(
        &realityos_semantics::discrepancy::DiscrepancyKind::ToolContactFrictionInconsistent
    ));
}
