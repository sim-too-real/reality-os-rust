//! Decision-function checks for the self-correction loop. MuJoCo is not required.

use realityos_semantics::discrepancy::{
    apply_probe_observation, classify_reasoning_outcome, hypothesize, DiscrepancyKind,
    DiscrepancyObservation, Identifiability, ObservationTag, PhysicalReasoningOutcome, Stimulus,
    TaxonomyFacts,
};
use realityos_semantics::execution_envelope::{
    check_execution_envelope, EnvelopeVerdict, ExecutionEnvelope, RuntimeExecutionObservation,
};
use realityos_semantics::goal_loop::{
    goal_status_if_no_admissible_interaction, receding_horizon_step, GoalLoopOutcome, LoopState,
};
use realityos_semantics::kinematics::IK_ACCEPT_M;
use realityos_semantics::physical_belief::{
    BeliefEpistemicStatus, PhysicalParameter, PhysicalParameterBelief,
    DECLARED_MODEL_INCONSISTENT_WITH_OBSERVATION,
};
use realityos_semantics::physical_experience::{
    authorize_from_experience, initial_decision, ApplicabilityQuery, ExperienceLog,
    PhysicalExperienceRecord, APPLICABILITY_UNKNOWN,
};
use realityos_semantics::physical_interaction::{
    evaluate_all, generate_planar_push_candidates, EvaluationContext,
};
use realityos_semantics::probe_selection::{
    candidates_for_uncertainty, rank_goal_or_probe, DecisionCandidate, DecisionClass,
};
use realityos_semantics::recoverability::{
    classify_recoverability, select_recoverable_progress, InteractionRegion, RecoverabilityChoice,
    RecoverabilityClass, RecoverabilityInput,
};
use realityos_semantics::transform::Se3;

fn scratch() -> std::path::PathBuf {
    std::path::PathBuf::from(
        r"C:\Users\moram\AppData\Local\Temp\grok-goal-cfa27dcadf88\implementer",
    )
}

fn save(name: &str, body: &str) {
    let dir = scratch();
    std::fs::create_dir_all(&dir).expect("scratch");
    std::fs::write(dir.join(name), body).expect(name);
}

#[test]
fn envelope_and_recoverability_use_the_shipped_functions() {
    assert_eq!(IK_ACCEPT_M, 1e-3);
    let mut lines = Vec::new();
    let commanded = 0.04;
    let envelope = ExecutionEnvelope::for_quasi_static_stroke(commanded, commanded);
    let mut sample = RuntimeExecutionObservation {
        stroke_consumed_m: 0.01,
        commanded_stroke_m: commanded,
        object_displacement_m: 0.009,
        yaw_change_rad: 0.01,
        intended_contact_persists: true,
        goal_error_before: 1.0,
        goal_error_now: 0.8,
        robot_tracking_error_m: Some(0.001),
        reachability_margin_m: 0.05,
        quasi_static_applicable: Some(true),
        authority_ok: true,
    };
    let kept = check_execution_envelope(&envelope, &sample);
    assert_eq!(kept.verdict, EnvelopeVerdict::Continue);
    lines.push(format!("continue={:?}", kept.verdict));
    sample.stroke_consumed_m = 0.02;
    sample.object_displacement_m = 0.09;
    let abort = check_execution_envelope(&envelope, &sample);
    assert_eq!(abort.verdict, EnvelopeVerdict::AbortAndReobserve);
    assert!(abort.early);
    assert!(abort.displacement_m < 0.2);
    lines.push(format!(
        "abort={:?} guard={:?} disp={}",
        abort.verdict, abort.failed_guard, abort.displacement_m
    ));
    let text = serde_json::to_string(&sample).unwrap();
    for needle in ["cfrc_ext", "qacc", "actuator_force", "hidden_friction"] {
        assert!(!text.contains(needle), "{needle}");
    }
    let mut already = sample.clone();
    already.stroke_consumed_m = 0.0;
    already.reachability_margin_m = -0.02;
    let lost = check_execution_envelope(&envelope, &already);
    assert!(lost.prevention_impossible);
    assert_ne!(
        goal_status_if_no_admissible_interaction(0),
        GoalLoopOutcome::GoalReached
    );
    assert_eq!(
        goal_status_if_no_admissible_interaction(0),
        GoalLoopOutcome::GoalCurrentlyUnachievable
    );
    let labels = [
        classify_recoverability(&RecoverabilityInput {
            physically_feasible: true,
            makes_progress: true,
            current_xy: [0.0, 0.0],
            nominal_dxy: Some([0.01, 0.0]),
            uncertainty_radius_m: Some(0.001),
            region: Some(InteractionRegion {
                center_xy: [0.0, 0.0],
                radius_m: 0.1,
            }),
            next_contact_admissible: Some(true),
        }),
        classify_recoverability(&RecoverabilityInput {
            physically_feasible: true,
            makes_progress: true,
            current_xy: [0.0, 0.0],
            nominal_dxy: None,
            uncertainty_radius_m: None,
            region: None,
            next_contact_admissible: None,
        }),
        classify_recoverability(&RecoverabilityInput {
            physically_feasible: true,
            makes_progress: true,
            current_xy: [0.09, 0.0],
            nominal_dxy: Some([0.05, 0.0]),
            uncertainty_radius_m: Some(0.02),
            region: Some(InteractionRegion {
                center_xy: [0.0, 0.0],
                radius_m: 0.1,
            }),
            next_contact_admissible: Some(true),
        }),
        classify_recoverability(&RecoverabilityInput {
            physically_feasible: true,
            makes_progress: false,
            current_xy: [0.0, 0.0],
            nominal_dxy: Some([0.0, 0.0]),
            uncertainty_radius_m: Some(0.0),
            region: Some(InteractionRegion {
                center_xy: [0.0, 0.0],
                radius_m: 0.1,
            }),
            next_contact_admissible: Some(true),
        }),
        classify_recoverability(&RecoverabilityInput {
            physically_feasible: false,
            makes_progress: true,
            current_xy: [0.0, 0.0],
            nominal_dxy: Some([0.01, 0.0]),
            uncertainty_radius_m: Some(0.0),
            region: Some(InteractionRegion {
                center_xy: [0.0, 0.0],
                radius_m: 0.1,
            }),
            next_contact_admissible: Some(false),
        }),
    ];
    assert_eq!(
        labels,
        [
            RecoverabilityClass::ProgressAndRecoverable,
            RecoverabilityClass::ProgressButRecoverabilityUnknown,
            RecoverabilityClass::ProgressButCanEnterUnrecoverableState,
            RecoverabilityClass::NoProgress,
            RecoverabilityClass::PhysicallyInfeasible,
        ]
    );
    let picked = select_recoverable_progress(&[
        RecoverabilityChoice {
            id: "eject".into(),
            class: RecoverabilityClass::ProgressButCanEnterUnrecoverableState,
            progress: 0.95,
        },
        RecoverabilityChoice {
            id: "stay".into(),
            class: RecoverabilityClass::ProgressAndRecoverable,
            progress: 0.15,
        },
    ])
    .unwrap();
    assert_eq!(picked, 1);
    lines.push(format!("recoverability={labels:?} preferred=stay"));
    save("envelope-recoverability.log", &lines.join("\n"));
    println!("{}", lines.join("\n"));
}

#[test]
fn belief_and_hypotheses_keep_lineage_and_distinct_outcomes() {
    let mu = 0.37;
    let mut belief =
        PhysicalParameterBelief::declared_point(PhysicalParameter::SupportFriction, mu, "scene.mu")
            .with_unknown(PhysicalParameter::ObjectMassKg, "mass");
    belief.contradict_declared(PhysicalParameter::SupportFriction, "ratio=3");
    let entry = belief.entry(PhysicalParameter::SupportFriction).unwrap();
    assert_eq!(entry.declared.value, Some(mu));
    let line = &entry.lineage[0];
    assert_eq!(line.inference, DECLARED_MODEL_INCONSISTENT_WITH_OBSERVATION);
    assert!(line.belief_before.contains("0.37"));
    assert_eq!(line.observation, "ratio=3");
    assert!(line.belief_after.contains("Contradicted"));
    assert_eq!(
        belief
            .entry(PhysicalParameter::ObjectMassKg)
            .unwrap()
            .status,
        BeliefEpistemicStatus::Unknown
    );
    belief.contradictory_measurements(PhysicalParameter::SupportFriction, "mu_a mu_b");
    assert_eq!(
        belief
            .entry(PhysicalParameter::SupportFriction)
            .unwrap()
            .status,
        BeliefEpistemicStatus::Contradicted
    );
    assert!(belief
        .entry(PhysicalParameter::SupportFriction)
        .unwrap()
        .empirical_interval
        .is_none());

    let limit = 0.015;
    let report = hypothesize(&DiscrepancyObservation {
        displacement_ratio: Some(3.0),
        yaw_change_rad: Some(0.0),
        predicted_yaw_sign: Some(0),
        observed_yaw_sign: Some(0),
        contact_persisted: Some(true),
        tracking_error_m: Some(0.0),
        geometry_residual_m: Some(0.0),
        freshness_ok: true,
        stroke_m: 0.04,
        quasi_static_stroke_limit_m: limit,
        contradictory: false,
        reachable: true,
    });
    assert_eq!(report.status, Identifiability::Underdetermined);
    assert!(report
        .kinds
        .contains(&DiscrepancyKind::SupportFrictionInconsistent));
    assert!(report
        .kinds
        .contains(&DiscrepancyKind::QuasiStaticAssumptionBroken));
    let stale = hypothesize(&DiscrepancyObservation {
        displacement_ratio: None,
        yaw_change_rad: None,
        predicted_yaw_sign: None,
        observed_yaw_sign: None,
        contact_persisted: None,
        tracking_error_m: None,
        geometry_residual_m: None,
        freshness_ok: false,
        stroke_m: 0.0,
        quasi_static_stroke_limit_m: limit,
        contradictory: false,
        reachable: true,
    });
    assert_eq!(stale.status, Identifiability::Unknown);
    let mut contradictory = DiscrepancyObservation {
        displacement_ratio: Some(3.0),
        yaw_change_rad: Some(0.0),
        predicted_yaw_sign: None,
        observed_yaw_sign: None,
        contact_persisted: Some(true),
        tracking_error_m: Some(0.0),
        geometry_residual_m: Some(0.0),
        freshness_ok: true,
        stroke_m: 0.04,
        quasi_static_stroke_limit_m: limit,
        contradictory: true,
        reachable: true,
    };
    let contradicted = hypothesize(&contradictory);
    assert_eq!(contradicted.status, Identifiability::Contradicted);
    contradictory.reachable = false;
    contradictory.contradictory = false;
    let unreachable = hypothesize(&contradictory);
    assert_eq!(unreachable.status, Identifiability::Unknown);
    assert_ne!(stale.kinds, contradicted.kinds);

    let cases = [
        (
            TaxonomyFacts {
                action_failed: true,
                ..TaxonomyFacts::none()
            },
            PhysicalReasoningOutcome::ActionFailed,
        ),
        (
            TaxonomyFacts {
                model_contradicted: true,
                ..TaxonomyFacts::none()
            },
            PhysicalReasoningOutcome::ModelContradicted,
        ),
        (
            TaxonomyFacts {
                parameter_uncertain: true,
                ..TaxonomyFacts::none()
            },
            PhysicalReasoningOutcome::ParameterUncertain,
        ),
        (
            TaxonomyFacts {
                regime_changed: true,
                ..TaxonomyFacts::none()
            },
            PhysicalReasoningOutcome::PhysicalRegimeChanged,
        ),
        (
            TaxonomyFacts {
                execution_diverged: true,
                ..TaxonomyFacts::none()
            },
            PhysicalReasoningOutcome::ExecutionDiverged,
        ),
        (
            TaxonomyFacts {
                recoverability_at_risk: true,
                ..TaxonomyFacts::none()
            },
            PhysicalReasoningOutcome::RecoverabilityAtRisk,
        ),
        (
            TaxonomyFacts {
                already_unrecoverable: true,
                ..TaxonomyFacts::none()
            },
            PhysicalReasoningOutcome::StateAlreadyUnrecoverable,
        ),
        (
            TaxonomyFacts {
                safe_probe: Some(true),
                ..TaxonomyFacts::none()
            },
            PhysicalReasoningOutcome::SafeProbeAvailable,
        ),
        (
            TaxonomyFacts {
                safe_probe: Some(false),
                ..TaxonomyFacts::none()
            },
            PhysicalReasoningOutcome::NoSafeProbe,
        ),
        (
            TaxonomyFacts {
                goal_unachievable: true,
                ..TaxonomyFacts::none()
            },
            PhysicalReasoningOutcome::GoalCurrentlyUnachievable,
        ),
    ];
    let mut seen = Vec::new();
    for (facts, expected) in cases {
        let got = classify_reasoning_outcome(&facts);
        assert_eq!(got, expected);
        let label = serde_json::to_value(got).unwrap();
        assert_ne!(label, "RECOVER");
        seen.push(label);
    }
    for (i, label) in seen.iter().enumerate() {
        assert!(!seen[i + 1..].contains(label));
    }
    let body = format!(
        "declared_mu={mu}\ninference={DECLARED_MODEL_INCONSISTENT_WITH_OBSERVATION}\nstatus={report:?}\noutcomes={seen:?}\n"
    );
    save("belief-hypotheses.log", &body);
    println!("{body}");
}

#[test]
fn probe_ranking_changes_after_the_cited_observation() {
    let limit = 0.015;
    let candidates = candidates_for_uncertainty(limit);
    let belief = PhysicalParameterBelief::declared_point(
        PhysicalParameter::SupportFriction,
        0.5,
        "scene.mu",
    );
    let report = hypothesize(&DiscrepancyObservation {
        displacement_ratio: Some(3.0),
        yaw_change_rad: Some(0.0),
        predicted_yaw_sign: Some(0),
        observed_yaw_sign: Some(0),
        contact_persisted: Some(true),
        tracking_error_m: Some(0.0),
        geometry_residual_m: Some(0.0),
        freshness_ok: true,
        stroke_m: limit * 2.5,
        quasi_static_stroke_limit_m: limit,
        contradictory: false,
        reachable: true,
    });
    let before = rank_goal_or_probe(&candidates, &report.kinds, limit);
    assert_eq!(before.selected_class, Some(DecisionClass::PhysicalProbe));
    assert_eq!(before.selected_id.as_deref(), Some("probe_separating"));
    assert!(before
        .refused
        .iter()
        .any(|(id, why)| id == "probe_unsafe" && why == "UNSAFE"));
    assert!(before
        .refused
        .iter()
        .any(|(id, why)| id == "probe_repeat" && why == "NO_INFORMATION"));
    let goal_large = candidates.iter().find(|c| c.id == "goal_large").unwrap();
    assert!(goal_large.immediate_progress > 0.0);
    let stimulus = Stimulus {
        stroke_m: limit * 0.4,
        quasi_static_stroke_limit_m: limit,
    };
    let low_friction = apply_probe_observation(
        &belief,
        &report.kinds,
        stimulus,
        ObservationTag::HighDisplacementRatio,
        "probe-low-friction",
    );
    let dynamic = apply_probe_observation(
        &belief,
        &report.kinds,
        stimulus,
        ObservationTag::NominalDisplacementRatio,
        "probe-dynamic",
    );
    let after_friction = rank_goal_or_probe(&candidates, &low_friction.remaining, limit);
    let after_dynamic = rank_goal_or_probe(&candidates, &dynamic.remaining, limit);
    assert_eq!(
        rank_goal_or_probe(&candidates, &report.kinds, limit).selected_id,
        before.selected_id
    );
    assert_ne!(after_friction.selected_id, after_dynamic.selected_id);
    assert_ne!(after_friction.selected_id, before.selected_id);
    assert_ne!(after_dynamic.selected_id, before.selected_id);
    assert_eq!(
        low_friction
            .belief
            .declared_value(PhysicalParameter::SupportFriction),
        Some(0.5)
    );
    assert!(low_friction
        .belief
        .entry(PhysicalParameter::SupportFriction)
        .unwrap()
        .lineage
        .iter()
        .any(|line| line.observation == "probe-low-friction"));
    assert!(dynamic
        .belief
        .entry(PhysicalParameter::SupportFriction)
        .unwrap()
        .lineage
        .iter()
        .any(|line| line.observation == "probe-dynamic"));
    let mut optimistic = candidates.clone();
    optimistic.push(DecisionCandidate {
        id: "goal_optimistic".into(),
        class: DecisionClass::GoalAction,
        stroke_m: limit * 0.5,
        immediate_progress: 4.0,
        safe: true,
        recoverability: RecoverabilityClass::ProgressAndRecoverable,
        recoverability_if_low_friction: RecoverabilityClass::ProgressAndRecoverable,
        progress_declared: realityos_semantics::planar_goal::GoalProgressClass::StrictProgress,
        progress_if_low_friction: realityos_semantics::planar_goal::GoalProgressClass::Regression,
        shrinks_interval: false,
        determines_contact_regime: false,
        determines_quasi_static: false,
        resolves_unknown_predicate: false,
    });
    let optimistic_rank = rank_goal_or_probe(
        &optimistic,
        &[DiscrepancyKind::SupportFrictionInconsistent],
        limit,
    );
    assert_ne!(
        optimistic_rank.selected_id.as_deref(),
        Some("goal_optimistic")
    );
    let body = format!(
        "before={:?}\nfriction={:?}\ndynamic={:?}\ngain={}\n",
        before.selected_id,
        after_friction.selected_id,
        after_dynamic.selected_id,
        before.information_gain
    );
    save("probe-decision.log", &body);
    println!("{body}");
}

#[test]
fn experience_informs_the_next_episode_and_cannot_authorize() {
    let limit = 0.015;
    let candidates = candidates_for_uncertainty(limit);
    let declared = PhysicalParameterBelief::declared_point(
        PhysicalParameter::SupportFriction,
        0.5,
        "scene.mu",
    );
    let regime = ApplicabilityQuery {
        world_model_id: "world-a".into(),
        embodiment_applicability: "serial-planar".into(),
        object_support_geometry: "box-on-plane".into(),
    };
    let empty = initial_decision(
        &ExperienceLog::default(),
        &regime,
        &candidates,
        &declared,
        limit,
    );
    assert_eq!(empty.ranking.selected_id.as_deref(), Some("goal_large"));
    let stored = PhysicalExperienceRecord {
        world_model_id: regime.world_model_id.clone(),
        embodiment_applicability: regime.embodiment_applicability.clone(),
        object_support_geometry: regime.object_support_geometry.clone(),
        belief_before: declared.clone(),
        selected_action: "probe_separating".into(),
        frozen_prediction: "sticking".into(),
        authority_result: "AUTHORIZE".into(),
        execution_envelope: "quasi_static".into(),
        observed_consequence: "nominal".into(),
        first_divergence: Some("MODEL_DISAGREEMENT".into()),
        hypotheses_before: vec![
            DiscrepancyKind::SupportFrictionInconsistent,
            DiscrepancyKind::QuasiStaticAssumptionBroken,
        ],
        hypotheses_after: vec![DiscrepancyKind::QuasiStaticAssumptionBroken],
        belief_after: declared.clone(),
        goal_effect: "probe".into(),
        recoverability_result: "inside".into(),
        provenance: "episode-1".into(),
    };
    let auth = authorize_from_experience(&stored);
    assert!(!auth.allow);
    assert_eq!(auth.unauthorized_writes, 0);
    let mut log = ExperienceLog::default();
    log.append(stored);
    let later = initial_decision(&log, &regime, &candidates, &declared, limit);
    assert_ne!(later.ranking.selected_id, empty.ranking.selected_id);
    let mut other = regime.clone();
    other.embodiment_applicability = "different-serial".into();
    let blocked = initial_decision(&log, &other, &candidates, &declared, limit);
    assert_eq!(blocked.applicability, APPLICABILITY_UNKNOWN);
    assert_eq!(blocked.ranking.selected_id, empty.ranking.selected_id);

    let goal = realityos_semantics::planar_goal::PlanarObjectGoal {
        object_id: "obj0".into(),
        world_id: "w".into(),
        model_id: "m".into(),
        target_xy: Some([0.2, 0.0]),
        target_xy_region: None,
        target_yaw: None,
        target_yaw_interval: None,
        translation_tolerance_m: 0.01,
        orientation_tolerance_rad: 0.1,
        freshness_s: 1.0,
        allowed_interaction_family: realityos_semantics::planar_goal::InteractionFamily::PlanarPush,
        safety: realityos_semantics::planar_goal::SafetyConstraints::default(),
        max_bounded_attempts: 4,
    };
    let mass = 0.1;
    let mu_s = 0.2;
    let tau = 20.0;
    let normal = mass * 9.80665;
    let f_max = mu_s * normal;
    let mechanics = realityos_semantics::effect_feasibility::PlanarPushInitiation {
        mass_kg: realityos_semantics::provenance::Provenanced::declared(mass, "t", 0.0),
        object_com_world: realityos_semantics::provenance::Provenanced::declared(
            [0.0, 0.0, 0.03],
            "t",
            0.0,
        ),
        gravity_m_s2: realityos_semantics::provenance::Provenanced::declared(
            [0.0, 0.0, -9.80665],
            "t",
            0.0,
        ),
        support_normal: realityos_semantics::provenance::Provenanced::declared(
            [0.0, 0.0, 1.0],
            "t",
            0.0,
        ),
        object_support_friction: realityos_semantics::pair_friction::PairFriction::coulomb(
            "object",
            "support",
            realityos_semantics::provenance::Provenanced::declared(mu_s, "t", 0.0),
        ),
        tool_object_friction: realityos_semantics::pair_friction::PairFriction::coulomb(
            "tool",
            "object",
            realityos_semantics::provenance::Provenanced::declared(0.8, "t", 0.0),
        ),
        contact_point_world: realityos_semantics::provenance::Provenanced::unknown("c", 0.0),
        contact_normal_world: realityos_semantics::provenance::Provenanced::unknown("n", 0.0),
        push_direction_world: realityos_semantics::provenance::Provenanced::unknown("d", 0.0),
        contact_force_direction_world: realityos_semantics::provenance::Provenanced::unknown(
            "f", 0.0,
        ),
        pusher_velocity_world: realityos_semantics::provenance::Provenanced::unknown("v", 0.0),
        joint_names: vec!["j0".into(), "j1".into(), "j2".into()],
        translational_jacobian_3xn: vec![
            vec![1.0, 0.0, 0.0],
            vec![0.0, 1.0, 0.0],
            vec![0.0, 0.0, 1.0],
        ],
        jacobian_residual: Some(0.0),
        joint_effort_abs: vec![
            realityos_semantics::provenance::Provenanced::declared(tau, "t", 0.0),
            realityos_semantics::provenance::Provenanced::declared(tau, "t", 0.0),
            realityos_semantics::provenance::Provenanced::declared(tau, "t", 0.0),
        ],
        joint_effort_min: vec![
            realityos_semantics::provenance::Provenanced::declared(-tau, "t", 0.0),
            realityos_semantics::provenance::Provenanced::declared(-tau, "t", 0.0),
            realityos_semantics::provenance::Provenanced::declared(-tau, "t", 0.0),
        ],
        joint_effort_max: vec![
            realityos_semantics::provenance::Provenanced::declared(tau, "t", 0.0),
            realityos_semantics::provenance::Provenanced::declared(tau, "t", 0.0),
            realityos_semantics::provenance::Provenanced::declared(tau, "t", 0.0),
        ],
        self_load_torque_nm: vec![
            realityos_semantics::provenance::Provenanced::declared(0.0, "s", 0.0),
            realityos_semantics::provenance::Provenanced::declared(0.0, "s", 0.0),
            realityos_semantics::provenance::Provenanced::declared(0.0, "s", 0.0),
        ],
        link_com_known: true,
        object_supported: true,
        approximately_planar: true,
        quasi_static: true,
        single_intended_contact: true,
        no_significant_impact: true,
        object_characteristic_length_m: Some(0.05),
        support_friction_model: realityos_physics::SupportFrictionModel::Ellipsoidal {
            f_max,
            tau_max: f_max * (2.0 / 3.0) * 0.05,
            pressure: realityos_physics::PressureDistribution::DeclaredUniform,
        },
        object_yaw_rad: realityos_semantics::provenance::Provenanced::declared(0.0, "yaw", 0.0),
        stale_object_evidence: false,
        intended_contact_lost: false,
        authority_ok: true,
    };
    let mut cands = generate_planar_push_candidates(
        "obj0",
        Se3 {
            xyz: [0.0, 0.0, 0.03],
            quat_wxyz: [1.0, 0.0, 0.0, 0.0],
        },
        [0.04, 0.03, 0.03],
        [0.0, 0.0, 1.0],
        0.01,
        0.02,
    )
    .expect("valid fixture support geometry");
    let ctx = EvaluationContext {
        goal: goal.clone(),
        object_xy: [0.0, 0.0],
        object_yaw: 0.0,
        object_com_world: [0.0, 0.0, 0.03],
        mechanics_template: Some(mechanics),
        authority_ok: true,
        robot_provided: false,
        robot_reachable: None,
        collision_admissible: None,
        executable_witness: None,
        robot_reject_reason: None,
    };
    evaluate_all(&mut cands, &ctx);
    crate::test_support::attach_test_execution_proofs(&mut cands);
    let obs = realityos_semantics::goal_loop::WorldObservation {
        object_id: "obj0".into(),
        xy: [0.0, 0.0],
        yaw: 0.0,
        robot_q: vec![0.0],
        freshness_ok: true,
        intended_contact_face: None,
        authority_ok: true,
        observed_at_s: 0.0,
    };
    let step = receding_horizon_step(&obs, &goal, &cands, LoopState::default(), None);
    assert!(step.record.selection_rationale.contains("STRICT_PROGRESS"));
    assert_eq!(step.record.authority_decision, "AUTHORIZE");
    assert_eq!(step.record.unauthorized_writes, 0);
    assert_eq!(step.state.unauthorized_writes, 0);
    let body = format!(
        "empty={:?}\nlater={:?}\nblocked={}\nauthorize_allow={} writes={}\nstep_authority={} writes={}\n",
        empty.ranking.selected_id,
        later.ranking.selected_id,
        blocked.applicability,
        auth.allow,
        auth.unauthorized_writes,
        step.record.authority_decision,
        step.record.unauthorized_writes
    );
    save("experience-authority.log", &body);
    println!("{body}");
}
