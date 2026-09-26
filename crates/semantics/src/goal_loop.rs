//! Receding-horizon PREDICT → ACT → OBSERVE → CORRECT. No plant I/O.

use realityos_physics::{PlanarTwist, RotationSign};
use serde::{Deserialize, Serialize};

use crate::execution_envelope::SupervisorDecision;
use crate::physical_decision::ContactTransitionPhase;
use crate::physical_interaction::{
    select_interaction_with_context, FunnelStage, PhysicalInteractionCandidate, SelectionOutcome,
};
use crate::planar_goal::{
    evaluate_goal_error, twist_in_world, wrap_pi, GoalError, GoalProgressClass, PlanarObjectGoal,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum ModelValidity {
    Consistent,
    WeaklyConsistent,
    Contradicted,
    InsufficientEvidence,
    OutsideRegime,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum GoalLoopOutcome {
    GoalReached,
    GoalProgress,
    NoProgress,
    GoalRegression,
    GoalPhysicallyInfeasible,
    InsufficientEvidence,
    ModelNotApplicable,
    AuthorityRefusal,
    GoalCurrentlyUnachievable,
    InsufficientPhysicalEvidence,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum LoopDecision {
    Continue,
    Replan,
    SwitchContact,
    Recover,
    Refuse,
    Halt,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WorldObservation {
    pub object_id: String,
    pub xy: [f64; 2],
    pub yaw: f64,
    pub robot_q: Vec<f64>,
    pub freshness_ok: bool,
    pub intended_contact_face: Option<String>,
    pub authority_ok: bool,
    pub observed_at_s: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ContactSwitch {
    pub phases: Vec<ContactTransitionPhase>,
    pub leave_current: bool,
    pub reobserve_required: bool,
    pub new_approach_required: bool,
    pub collision_admissible_required: bool,
    pub executable_witness_required: bool,
    pub from_face: Option<String>,
    pub to_face: String,
    /// Must stay None: switching never copies a predicted pose.
    pub assumed_object_pose: Option<[f64; 2]>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CausalActionRecord {
    pub object_xy: [f64; 2],
    pub object_yaw: f64,
    pub goal_object_id: String,
    pub robot_q: Vec<f64>,
    pub candidate_count: usize,
    pub rejection_reasons: Vec<String>,
    pub selected_id: Option<String>,
    pub selected_face: Option<String>,
    pub selection_rationale: String,
    pub predicted_contact_mode: Option<String>,
    pub predicted_twist: Option<PlanarTwist>,
    pub predicted_rotation_sign: Option<RotationSign>,
    pub predicted_goal_progress: Option<GoalProgressClass>,
    pub predicted_error_derivative: Option<f64>,
    pub model_validity: ModelValidity,
    pub authority_decision: String,
    pub goal_error_before: Option<GoalError>,
    pub goal_error_after: Option<GoalError>,
    pub prediction_residual: Option<f64>,
    pub first_divergence: Option<String>,
    pub decision: LoopDecision,
    pub outcome: GoalLoopOutcome,
    pub contact_switch: Option<ContactSwitch>,
    pub unauthorized_writes: u64,
    pub reasoning: Option<ReasoningNote>,
}

/// Causal notes from the self-correction loop. Absent on an unchanged decision.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct ReasoningNote {
    pub envelope_verdict: Option<String>,
    pub failed_guard: Option<String>,
    pub prevention_impossible: bool,
    pub hypotheses: Vec<String>,
    pub hypothesis_status: Option<String>,
    pub belief_before: Option<String>,
    pub belief_after: Option<String>,
    pub ranking_before: Option<String>,
    pub ranking_after: Option<String>,
    pub selected_kind: Option<String>,
    pub information_gain: Option<u32>,
    pub recoverability: Option<String>,
    pub taxonomy: Option<String>,
    pub displacement_m: Option<f64>,
    pub interactable_after_abort: Option<bool>,
    #[serde(default)]
    pub probe_displacement_m: Option<f64>,
    #[serde(default)]
    pub probe_contact_persisted: Option<bool>,
    #[serde(default)]
    pub admissible_contact_count: Option<u32>,
    #[serde(default)]
    pub consequence_assessment: Option<crate::physical_consequence::ConsequenceAssessment>,
    #[serde(default)]
    pub supervisor_decisions: Vec<String>,
    #[serde(default)]
    pub supervisor_events: Vec<SupervisorDecision>,
    #[serde(default)]
    pub probe_decision: Option<crate::physical_decision::DecisionKind>,
    #[serde(default)]
    pub executed_quanta: u32,
    #[serde(default)]
    pub frozen_action_id: Option<String>,
    #[serde(default)]
    pub frozen_witness_id: Option<String>,
    #[serde(default)]
    pub policy_observation_source: Option<String>,
    #[serde(default)]
    pub work_counters: Option<DecisionWorkCounters>,
}

/// Typed development counters. `None` means the owning subsystem did not expose
/// a trustworthy count in this runtime slice.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct DecisionWorkCounters {
    pub candidates: u64,
    pub candidate_evaluations: u64,
    pub ik_attempts: Option<u64>,
    pub fk_evaluations: Option<u64>,
    pub collision_evaluations: Option<u64>,
    pub jacobian_evaluations: Option<u64>,
    pub mechanics_evaluations: u64,
    pub recoverability_evaluations: u64,
    pub belief_domain_evaluations: u64,
    pub probe_evaluations: u64,
    pub decision_wall_time_ns: u64,
    pub execution_quanta: u32,
    pub observations: u32,
    pub replans: u32,
    pub aborts: u32,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LoopState {
    pub model_validity: ModelValidity,
    pub forbidden_action_keys: Vec<String>,
    pub last_record: Option<CausalActionRecord>,
    pub attempts: u32,
    pub contact_switches: u32,
    pub unauthorized_writes: u64,
    pub last_predicted_pose: Option<[f64; 2]>,
}

impl Default for LoopState {
    fn default() -> Self {
        Self {
            model_validity: ModelValidity::InsufficientEvidence,
            forbidden_action_keys: Vec::new(),
            last_record: None,
            attempts: 0,
            contact_switches: 0,
            unauthorized_writes: 0,
            last_predicted_pose: None,
        }
    }
}

impl LoopState {
    /// Blacklist one physical identity after contradiction or an aborted
    /// witness. Idempotence keeps repeated reports from changing the state.
    pub fn forbid_action_key(&mut self, action_key: impl Into<String>) -> bool {
        let action_key = action_key.into();
        if action_key.is_empty() || self.forbidden_action_keys.contains(&action_key) {
            return false;
        }
        self.forbidden_action_keys.push(action_key);
        true
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RecedingHorizonResult {
    pub record: CausalActionRecord,
    pub state: LoopState,
    pub selected: Option<PhysicalInteractionCandidate>,
}

pub fn observed_rotation_sign(yaw_change: f64, translation_xy: [f64; 2]) -> RotationSign {
    let yaw_change = wrap_pi(yaw_change);
    let t = (translation_xy[0] * translation_xy[0] + translation_xy[1] * translation_xy[1]).sqrt();
    if !yaw_change.is_finite() {
        return RotationSign::Unknown;
    }
    if yaw_change.abs() < 1e-4 && t < 1e-4 {
        return RotationSign::Unknown;
    }
    if yaw_change.abs() * 0.05 < 0.2 * t.max(1e-9) {
        return RotationSign::TranslationOnly;
    }
    if yaw_change > 0.0 {
        RotationSign::Counterclockwise
    } else {
        RotationSign::Clockwise
    }
}

fn opposite_sign(a: RotationSign, b: RotationSign) -> bool {
    matches!(
        (a, b),
        (RotationSign::Clockwise, RotationSign::Counterclockwise)
            | (RotationSign::Counterclockwise, RotationSign::Clockwise)
    )
}

pub fn classify_model_validity(
    predicted_sign: Option<RotationSign>,
    observed_sign: RotationSign,
    predicted_xy: Option<[f64; 2]>,
    observed_dxy: Option<[f64; 2]>,
) -> ModelValidity {
    match predicted_sign {
        None => ModelValidity::InsufficientEvidence,
        Some(RotationSign::Unknown) | Some(RotationSign::Ambiguous) => {
            ModelValidity::InsufficientEvidence
        }
        Some(p) => {
            if opposite_sign(p, observed_sign) {
                return ModelValidity::Contradicted;
            }
            if observed_sign == RotationSign::Unknown {
                return ModelValidity::InsufficientEvidence;
            }
            if let (Some(a), Some(b)) = (predicted_xy, observed_dxy) {
                let na = (a[0] * a[0] + a[1] * a[1]).sqrt();
                let nb = (b[0] * b[0] + b[1] * b[1]).sqrt();
                if na > 1e-9 && nb > 1e-9 {
                    let cos = ((a[0] * b[0] + a[1] * b[1]) / (na * nb)).clamp(-1.0, 1.0);
                    if cos < -0.5 {
                        return ModelValidity::Contradicted;
                    }
                    if cos < 0.0 {
                        return ModelValidity::WeaklyConsistent;
                    }
                }
            }
            if p == observed_sign
                || (p == RotationSign::TranslationOnly
                    && observed_sign == RotationSign::TranslationOnly)
            {
                ModelValidity::Consistent
            } else {
                ModelValidity::WeaklyConsistent
            }
        }
    }
}

pub fn classify_goal_outcome(before: &GoalError, after: &GoalError) -> GoalLoopOutcome {
    if after.reached {
        GoalLoopOutcome::GoalReached
    } else if after.combined + 1e-9 < before.combined {
        GoalLoopOutcome::GoalProgress
    } else if after.combined > before.combined + 1e-9 {
        GoalLoopOutcome::GoalRegression
    } else {
        GoalLoopOutcome::NoProgress
    }
}

fn rejection_reasons(cands: &[PhysicalInteractionCandidate]) -> Vec<String> {
    cands
        .iter()
        .filter_map(|c| c.funnel.reason.as_ref().map(|r| format!("{}:{r}", c.id)))
        .collect()
}

/// One receding-horizon decision from the current observation. Does not execute.
pub fn receding_horizon_step(
    observation: &WorldObservation,
    goal: &PlanarObjectGoal,
    candidates: &[PhysicalInteractionCandidate],
    mut state: LoopState,
    observed_consequence: Option<(&WorldObservation, Option<RotationSign>)>,
) -> RecedingHorizonResult {
    let err_now = evaluate_goal_error(observation.xy, observation.yaw, goal);
    if let Some((after, obs_sign)) = observed_consequence {
        if let Some(prev) = state.last_record.clone() {
            let dxy = [
                after.xy[0] - prev.object_xy[0],
                after.xy[1] - prev.object_xy[1],
            ];
            let sign = obs_sign.unwrap_or_else(|| {
                observed_rotation_sign(
                    crate::planar_goal::wrap_pi(after.yaw - prev.object_yaw),
                    dxy,
                )
            });
            let pred_xy = prev.predicted_twist.map(|t| {
                let w = twist_in_world(t, prev.object_yaw);
                [w.vx, w.vy]
            });
            state.model_validity =
                classify_model_validity(prev.predicted_rotation_sign, sign, pred_xy, Some(dxy));
            if state.model_validity == ModelValidity::Contradicted {
                let k = candidates
                    .iter()
                    .find(|c| prev.selected_id.as_ref().is_some_and(|id| &c.id == id))
                    .map(|c| c.action_key())
                    .or_else(|| prev.selected_id.clone())
                    .unwrap_or_default();
                state.forbid_action_key(k);
                if let Some(rec) = state.last_record.as_mut() {
                    rec.first_divergence = Some("MODEL_DISAGREEMENT".into());
                }
            }
        }
    }

    if !observation.freshness_ok && goal.safety.require_fresh_observation {
        let record = CausalActionRecord {
            object_xy: observation.xy,
            object_yaw: observation.yaw,
            goal_object_id: goal.object_id.clone(),
            robot_q: observation.robot_q.clone(),
            candidate_count: candidates.len(),
            rejection_reasons: vec!["STALE_OBJECT_EVIDENCE".into()],
            selected_id: None,
            selected_face: None,
            selection_rationale: "freshness lost; discard stale plan".into(),
            predicted_contact_mode: None,
            predicted_twist: None,
            predicted_rotation_sign: None,
            predicted_goal_progress: None,
            predicted_error_derivative: None,
            model_validity: state.model_validity,
            authority_decision: "REPLAN".into(),
            goal_error_before: Some(err_now),
            goal_error_after: None,
            prediction_residual: None,
            first_divergence: Some("STALE_OBJECT_EVIDENCE".into()),
            decision: LoopDecision::Recover,
            outcome: GoalLoopOutcome::InsufficientPhysicalEvidence,
            contact_switch: None,
            unauthorized_writes: 0,
            reasoning: None,
        };
        state.last_record = Some(record.clone());
        return RecedingHorizonResult {
            record,
            state,
            selected: None,
        };
    }

    if err_now.reached {
        let record = CausalActionRecord {
            object_xy: observation.xy,
            object_yaw: observation.yaw,
            goal_object_id: goal.object_id.clone(),
            robot_q: observation.robot_q.clone(),
            candidate_count: candidates.len(),
            rejection_reasons: rejection_reasons(candidates),
            selected_id: None,
            selected_face: None,
            selection_rationale: "GOAL_REACHED".into(),
            predicted_contact_mode: None,
            predicted_twist: None,
            predicted_rotation_sign: None,
            predicted_goal_progress: None,
            predicted_error_derivative: None,
            model_validity: state.model_validity,
            authority_decision: "HALT".into(),
            goal_error_before: Some(err_now),
            goal_error_after: None,
            prediction_residual: None,
            first_divergence: None,
            decision: LoopDecision::Halt,
            outcome: GoalLoopOutcome::GoalReached,
            contact_switch: None,
            unauthorized_writes: 0,
            reasoning: None,
        };
        state.last_record = Some(record.clone());
        return RecedingHorizonResult {
            record,
            state,
            selected: None,
        };
    }

    if state.attempts >= goal.max_bounded_attempts {
        let record = CausalActionRecord {
            object_xy: observation.xy,
            object_yaw: observation.yaw,
            goal_object_id: goal.object_id.clone(),
            robot_q: observation.robot_q.clone(),
            candidate_count: candidates.len(),
            rejection_reasons: rejection_reasons(candidates),
            selected_id: None,
            selected_face: None,
            selection_rationale: "PHYSICAL_INTERACTION_BUDGET_EXHAUSTED".into(),
            predicted_contact_mode: None,
            predicted_twist: None,
            predicted_rotation_sign: None,
            predicted_goal_progress: None,
            predicted_error_derivative: None,
            model_validity: state.model_validity,
            authority_decision: "REFUSE".into(),
            goal_error_before: Some(err_now),
            goal_error_after: None,
            prediction_residual: None,
            first_divergence: Some("BUDGET".into()),
            decision: LoopDecision::Refuse,
            outcome: GoalLoopOutcome::GoalCurrentlyUnachievable,
            contact_switch: None,
            unauthorized_writes: 0,
            reasoning: None,
        };
        state.last_record = Some(record.clone());
        return RecedingHorizonResult {
            record,
            state,
            selected: None,
        };
    }

    if state.model_validity == ModelValidity::Contradicted {
        // A contradicted model is not left authoritative: forbidden keys already set.
    }

    let sel = select_interaction_with_context(
        candidates,
        &state.forbidden_action_keys,
        observation.intended_contact_face.as_deref(),
        observation.freshness_ok,
        goal.max_bounded_attempts.saturating_sub(state.attempts),
    );
    let (outcome, decision, authority, selected_idx, switch) = match &sel {
        SelectionOutcome::Selected { index, .. } => (
            GoalLoopOutcome::GoalProgress,
            LoopDecision::Continue,
            "AUTHORIZE".to_string(),
            Some(*index),
            None,
        ),
        SelectionOutcome::ContactTransition { index, phases, .. } => {
            let candidate = &candidates[*index];
            state.contact_switches += 1;
            let contains = |phase| phases.contains(&phase);
            let switch = ContactSwitch {
                phases: phases.clone(),
                leave_current: contains(ContactTransitionPhase::LeaveCurrentContact),
                reobserve_required: contains(ContactTransitionPhase::Reobserve),
                new_approach_required: contains(ContactTransitionPhase::ApproachNewContact),
                collision_admissible_required: contains(
                    ContactTransitionPhase::RecheckCollisionAndWitness,
                ),
                executable_witness_required: contains(
                    ContactTransitionPhase::RecheckCollisionAndWitness,
                ),
                from_face: observation.intended_contact_face.clone(),
                to_face: candidate.face_id.clone(),
                assumed_object_pose: None,
            };
            (
                GoalLoopOutcome::GoalProgress,
                LoopDecision::SwitchContact,
                "AUTHORIZE".to_string(),
                Some(*index),
                Some(switch),
            )
        }
        SelectionOutcome::AuthorityRefusal { .. } => (
            GoalLoopOutcome::AuthorityRefusal,
            LoopDecision::Refuse,
            "REFUSE".into(),
            None,
            None,
        ),
        SelectionOutcome::MechanicsUnknown { .. } => (
            GoalLoopOutcome::InsufficientEvidence,
            LoopDecision::Refuse,
            "REFUSE".into(),
            None,
            None,
        ),
        SelectionOutcome::InsufficientEvidence { .. } => (
            GoalLoopOutcome::InsufficientEvidence,
            LoopDecision::Refuse,
            "REFUSE".into(),
            None,
            None,
        ),
        SelectionOutcome::NoUsefulPhysicalAction { .. }
        | SelectionOutcome::PlannerFailedToFind { .. }
        | SelectionOutcome::GeometricallyUsefulRobotCannotExecute { .. } => (
            GoalLoopOutcome::GoalCurrentlyUnachievable,
            LoopDecision::Refuse,
            "REFUSE".into(),
            None,
            None,
        ),
    };

    if matches!(sel, SelectionOutcome::AuthorityRefusal { .. }) {
        // Planner does not write. Unauthorized stays 0.
        state.unauthorized_writes = 0;
    }

    let selected = selected_idx.map(|i| candidates[i].clone());
    let rationale = match &sel {
        SelectionOutcome::Selected { reason, .. } => reason.clone(),
        SelectionOutcome::ContactTransition { reason, .. } => reason.clone(),
        SelectionOutcome::AuthorityRefusal { reason, .. } => reason.clone(),
        SelectionOutcome::NoUsefulPhysicalAction { reason }
        | SelectionOutcome::PlannerFailedToFind { reason }
        | SelectionOutcome::GeometricallyUsefulRobotCannotExecute { reason }
        | SelectionOutcome::MechanicsUnknown { reason }
        | SelectionOutcome::InsufficientEvidence { reason } => reason.clone(),
    };

    if selected.is_some() {
        state.attempts += 1;
    }

    let record = CausalActionRecord {
        object_xy: observation.xy,
        object_yaw: observation.yaw,
        goal_object_id: goal.object_id.clone(),
        robot_q: observation.robot_q.clone(),
        candidate_count: candidates.len(),
        rejection_reasons: rejection_reasons(candidates),
        selected_id: selected.as_ref().map(|c| c.id.clone()),
        selected_face: selected.as_ref().map(|c| c.face_id.clone()),
        selection_rationale: rationale,
        predicted_contact_mode: selected
            .as_ref()
            .and_then(|c| c.predicted_contact_mode)
            .map(|m| format!("{m:?}")),
        predicted_twist: selected.as_ref().and_then(|c| c.predicted_twist),
        predicted_rotation_sign: selected.as_ref().and_then(|c| c.predicted_rotation_sign),
        predicted_goal_progress: selected.as_ref().and_then(|c| c.goal_progress),
        predicted_error_derivative: selected.as_ref().and_then(|c| c.predicted_error_derivative),
        model_validity: state.model_validity,
        authority_decision: authority,
        goal_error_before: Some(err_now),
        goal_error_after: None,
        prediction_residual: None,
        first_divergence: if state.model_validity == ModelValidity::Contradicted {
            Some("MODEL_DISAGREEMENT".into())
        } else {
            None
        },
        decision,
        outcome,
        contact_switch: switch,
        unauthorized_writes: 0,
        reasoning: None,
    };
    // Never carry a predicted world as the next planning state.
    state.last_predicted_pose = None;
    state.last_record = Some(record.clone());
    RecedingHorizonResult {
        record,
        state,
        selected,
    }
}

/// No admissible interaction means the goal is currently unachievable.
/// An unreachable object is not reported as recoverable.
pub fn goal_status_if_no_admissible_interaction(admissible: usize) -> GoalLoopOutcome {
    if admissible == 0 {
        GoalLoopOutcome::GoalCurrentlyUnachievable
    } else {
        GoalLoopOutcome::GoalProgress
    }
}

/// Record the observed consequence against the same goal used to plan.
pub fn record_after_with_goal(
    mut result: RecedingHorizonResult,
    after: &WorldObservation,
    goal: &PlanarObjectGoal,
) -> RecedingHorizonResult {
    let after_err = evaluate_goal_error(after.xy, after.yaw, goal);
    if let Some(before) = result.record.goal_error_before.as_ref() {
        result.record.outcome = classify_goal_outcome(before, &after_err);
        let dxy = [
            after.xy[0] - result.record.object_xy[0],
            after.xy[1] - result.record.object_xy[1],
        ];
        let dyaw = crate::planar_goal::wrap_pi(after.yaw - result.record.object_yaw);
        result.record.prediction_residual = result.record.predicted_twist.map(|t| {
            let nv = (t.vx * t.vx + t.vy * t.vy).sqrt().max(1e-9);
            let nd = (dxy[0] * dxy[0] + dxy[1] * dxy[1]).sqrt();
            let dir = if nd < 1e-9 {
                0.0
            } else {
                1.0 - ((t.vx * dxy[0] + t.vy * dxy[1]) / (nv * nd)).clamp(-1.0, 1.0)
            };
            dir + (t.omega_z - dyaw).abs()
        });
        let obs_sign = observed_rotation_sign(dyaw, dxy);
        let pred_xy = result.record.predicted_twist.map(|t| {
            let w = twist_in_world(t, result.record.object_yaw);
            [w.vx, w.vy]
        });
        let validity = classify_model_validity(
            result.record.predicted_rotation_sign,
            obs_sign,
            pred_xy,
            Some(dxy),
        );
        result.state.model_validity = validity;
        if validity == ModelValidity::Contradicted {
            result.record.first_divergence = Some("MODEL_DISAGREEMENT".into());
            if let Some(id) = result.record.selected_id.clone() {
                let key = result
                    .selected
                    .as_ref()
                    .map(|c| c.action_key())
                    .unwrap_or(id);
                result.state.forbid_action_key(key);
            }
        }
    }
    result.record.goal_error_after = Some(after_err);
    result.state.last_predicted_pose = None;
    result
}

pub fn funnel_stage_name(s: FunnelStage) -> &'static str {
    match s {
        FunnelStage::Generated => "generated",
        FunnelStage::GeometryValid => "geometry_valid",
        FunnelStage::RobotReachable => "robot_reachable",
        FunnelStage::CollisionAdmissible => "collision_admissible",
        FunnelStage::ExecutableWitness => "executable_witness",
        FunnelStage::AvailableEffort => "available_effort",
        FunnelStage::MotionInitiation => "motion_initiation",
        FunnelStage::InstantaneousMotion => "instantaneous_motion",
        FunnelStage::ContactMode => "contact_mode",
        FunnelStage::GoalUseful => "goal_useful",
        FunnelStage::Selected => "selected",
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::effect_feasibility::PlanarPushInitiation;
    use crate::pair_friction::PairFriction;
    use crate::physical_interaction::{
        evaluate_all, generate_planar_push_candidates, EvaluationContext,
    };
    use crate::planar_goal::{InteractionFamily, SafetyConstraints};
    use crate::provenance::Provenanced;
    use crate::transform::Se3;
    use realityos_physics::{PressureDistribution, SupportFrictionModel};

    fn mechanics() -> PlanarPushInitiation {
        let mass = 0.1;
        let mu_s = 0.2;
        let tau = 20.0;
        let n = mass * 9.80665;
        let f_max = mu_s * n;
        PlanarPushInitiation {
            mass_kg: Provenanced::declared(mass, "t", 0.0),
            object_com_world: Provenanced::declared([0.0, 0.0, 0.03], "t", 0.0),
            gravity_m_s2: Provenanced::declared([0.0, 0.0, -9.80665], "t", 0.0),
            support_normal: Provenanced::declared([0.0, 0.0, 1.0], "t", 0.0),
            object_support_friction: PairFriction::coulomb(
                "object",
                "support",
                Provenanced::declared(mu_s, "t", 0.0),
            ),
            tool_object_friction: PairFriction::coulomb(
                "tool",
                "object",
                Provenanced::declared(0.8, "t", 0.0),
            ),
            contact_point_world: Provenanced::unknown("c", 0.0),
            contact_normal_world: Provenanced::unknown("n", 0.0),
            push_direction_world: Provenanced::unknown("d", 0.0),
            contact_force_direction_world: Provenanced::unknown("f", 0.0),
            pusher_velocity_world: Provenanced::unknown("v", 0.0),
            joint_names: vec!["j0".into(), "j1".into(), "j2".into()],
            translational_jacobian_3xn: vec![
                vec![1.0, 0.0, 0.0],
                vec![0.0, 1.0, 0.0],
                vec![0.0, 0.0, 1.0],
            ],
            jacobian_residual: Some(0.0),
            joint_effort_abs: vec![
                Provenanced::declared(tau, "t", 0.0),
                Provenanced::declared(tau, "t", 0.0),
                Provenanced::declared(tau, "t", 0.0),
            ],
            joint_effort_min: vec![
                Provenanced::declared(-tau, "t", 0.0),
                Provenanced::declared(-tau, "t", 0.0),
                Provenanced::declared(-tau, "t", 0.0),
            ],
            joint_effort_max: vec![
                Provenanced::declared(tau, "t", 0.0),
                Provenanced::declared(tau, "t", 0.0),
                Provenanced::declared(tau, "t", 0.0),
            ],
            self_load_torque_nm: vec![
                Provenanced::declared(0.0, "s", 0.0),
                Provenanced::declared(0.0, "s", 0.0),
                Provenanced::declared(0.0, "s", 0.0),
            ],
            link_com_known: true,
            object_supported: true,
            approximately_planar: true,
            quasi_static: true,
            single_intended_contact: true,
            no_significant_impact: true,
            object_characteristic_length_m: Some(0.05),
            support_friction_model: SupportFrictionModel::Ellipsoidal {
                f_max,
                tau_max: f_max * (2.0 / 3.0) * 0.05,
                pressure: PressureDistribution::DeclaredUniform,
            },
            object_yaw_rad: Provenanced::declared(0.0, "yaw", 0.0),
            stale_object_evidence: false,
            intended_contact_lost: false,
            authority_ok: true,
        }
    }

    fn goal_plus_x() -> PlanarObjectGoal {
        PlanarObjectGoal {
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
            allowed_interaction_family: InteractionFamily::PlanarPush,
            safety: SafetyConstraints::default(),
            max_bounded_attempts: 8,
        }
    }

    fn obs(xy: [f64; 2], face: Option<&str>) -> WorldObservation {
        WorldObservation {
            object_id: "obj0".into(),
            xy,
            yaw: 0.0,
            robot_q: vec![0.0, 0.0],
            freshness_ok: true,
            intended_contact_face: face.map(str::to_string),
            authority_ok: true,
            observed_at_s: 1.0,
        }
    }

    fn evaluated(xy: [f64; 2], g: &PlanarObjectGoal) -> Vec<PhysicalInteractionCandidate> {
        let mut cands = generate_planar_push_candidates(
            "obj0",
            Se3 {
                xyz: [xy[0], xy[1], 0.03],
                quat_wxyz: [1.0, 0.0, 0.0, 0.0],
            },
            [0.04, 0.03, 0.03],
            [0.0, 0.0, 1.0],
            0.01,
            0.02,
        )
        .expect("valid fixture support geometry");
        let ctx = EvaluationContext {
            goal: g.clone(),
            object_xy: xy,
            object_yaw: 0.0,
            object_com_world: [xy[0], xy[1], 0.03],
            mechanics_template: Some(mechanics()),
            authority_ok: true,
            robot_provided: false,
            robot_reachable: None,
            collision_admissible: None,
            executable_witness: None,
            robot_reject_reason: None,
        };
        evaluate_all(&mut cands, &ctx);
        crate::physical_interaction::test_support::attach_executable_witnesses(&mut cands);
        cands
    }

    #[test]
    fn forbidden_action_keys_are_nonempty_and_idempotent() {
        let mut state = LoopState::default();
        assert!(!state.forbid_action_key(""));
        assert!(state.forbid_action_key("face:+x:0.0"));
        assert!(!state.forbid_action_key("face:+x:0.0"));
        assert_eq!(state.forbidden_action_keys, vec!["face:+x:0.0"]);
    }

    #[test]
    fn model_disagreement_forbids_repeating_the_same_action() {
        let g = goal_plus_x();
        let start = obs([0.0, 0.0], None);
        let cands = evaluated(start.xy, &g);
        let step1 = receding_horizon_step(&start, &g, &cands, LoopState::default(), None);
        assert!(step1.selected.is_some());
        let first_key = step1.selected.as_ref().unwrap().action_key();
        let mut predicted_ccw = step1;
        predicted_ccw.record.predicted_rotation_sign = Some(RotationSign::Counterclockwise);
        predicted_ccw.state.last_record = Some(predicted_ccw.record.clone());

        let after = obs(
            [0.01, 0.0],
            Some(first_key.split(':').next().unwrap_or("-x")),
        );
        let step2 = receding_horizon_step(
            &after,
            &g,
            &cands,
            predicted_ccw.state.clone(),
            Some((&after, Some(RotationSign::Clockwise))),
        );
        assert_eq!(step2.state.model_validity, ModelValidity::Contradicted);
        assert!(
            step2
                .state
                .forbidden_action_keys
                .iter()
                .any(|k| k == &first_key || k.contains(&first_key)),
            "forbidden={:?}",
            step2.state.forbidden_action_keys
        );
        if let Some(sel) = step2.selected.as_ref() {
            assert_ne!(sel.action_key(), first_key);
        }
        assert_eq!(
            step2.record.first_divergence.as_deref(),
            Some("MODEL_DISAGREEMENT")
        );
    }

    #[test]
    fn opposite_translation_is_model_disagreement_and_forbids_repeat() {
        let g = goal_plus_x();
        let start = obs([0.0, 0.0], None);
        let cands = evaluated(start.xy, &g);
        let step1 = receding_horizon_step(&start, &g, &cands, LoopState::default(), None);
        let first_key = step1.selected.as_ref().unwrap().action_key();
        let mut predicted = step1;
        predicted.record.predicted_twist = Some(realityos_physics::PlanarTwist {
            vx: 1.0,
            vy: 0.0,
            omega_z: 0.0,
            frame: realityos_physics::PlanarFrameKind::World,
        });
        predicted.record.predicted_rotation_sign = Some(RotationSign::TranslationOnly);
        predicted.state.last_record = Some(predicted.record.clone());
        let after = obs([-0.04, 0.0], None);
        let rec = record_after_with_goal(predicted, &after, &g);
        assert_eq!(rec.state.model_validity, ModelValidity::Contradicted);
        assert_eq!(
            rec.record.first_divergence.as_deref(),
            Some("MODEL_DISAGREEMENT")
        );
        let step2 = receding_horizon_step(
            &after,
            &g,
            &cands,
            rec.state,
            Some((&after, Some(RotationSign::TranslationOnly))),
        );
        assert!(
            step2
                .state
                .forbidden_action_keys
                .iter()
                .any(|k| k == &first_key),
            "forbidden={:?}",
            step2.state.forbidden_action_keys
        );
        if let Some(sel) = step2.selected.as_ref() {
            assert_ne!(sel.action_key(), first_key);
        }
    }

    #[test]
    fn contact_switch_does_not_teleport() {
        let g = goal_plus_x();
        let start = obs([0.0, 0.0], Some("+y"));
        let cands = evaluated(start.xy, &g);
        let step = receding_horizon_step(&start, &g, &cands, LoopState::default(), None);
        let sw = step
            .record
            .contact_switch
            .expect("expected a contact switch event");
        assert!(sw.leave_current);
        assert_eq!(
            sw.phases.first(),
            Some(&ContactTransitionPhase::LeaveCurrentContact)
        );
        assert!(sw.reobserve_required);
        assert!(sw.new_approach_required);
        assert!(sw.collision_admissible_required);
        assert!(sw.executable_witness_required);
        assert_eq!(sw.from_face.as_deref(), Some("+y"));
        assert_ne!(sw.to_face, "+y");
        assert!(sw.assumed_object_pose.is_none());
        assert!(step.state.last_predicted_pose.is_none());
        assert_eq!(step.record.decision, LoopDecision::SwitchContact);
    }

    #[test]
    fn no_current_contact_preserves_reobserve_recheck_and_approach_phases() {
        let g = goal_plus_x();
        let start = obs([0.0, 0.0], None);
        let cands = evaluated(start.xy, &g);
        let step = receding_horizon_step(&start, &g, &cands, LoopState::default(), None);
        let transition = step
            .record
            .contact_switch
            .expect("new contact still needs a witnessed transition");

        assert!(!transition.leave_current);
        assert_eq!(
            transition.phases,
            vec![
                ContactTransitionPhase::Reobserve,
                ContactTransitionPhase::RecheckCollisionAndWitness,
                ContactTransitionPhase::ApproachNewContact,
            ]
        );
        assert_eq!(step.record.decision, LoopDecision::SwitchContact);
        assert!(transition.from_face.is_none());
        assert!(transition.assumed_object_pose.is_none());
    }

    #[test]
    fn unauthorized_best_action_refuses_without_writes() {
        let g = goal_plus_x();
        let start = obs([0.0, 0.0], None);
        let mut cands = evaluated(start.xy, &g);
        for c in &mut cands {
            c.authority_ok = false;
        }
        let step = receding_horizon_step(&start, &g, &cands, LoopState::default(), None);
        assert_eq!(step.record.outcome, GoalLoopOutcome::AuthorityRefusal);
        assert_eq!(step.record.decision, LoopDecision::Refuse);
        assert_eq!(step.record.unauthorized_writes, 0);
        assert_eq!(step.state.unauthorized_writes, 0);
        assert!(step.selected.is_none());
    }

    #[test]
    fn observation_replaces_predicted_world() {
        let g = goal_plus_x();
        let start = obs([0.0, 0.0], None);
        let cands = evaluated(start.xy, &g);
        let step = receding_horizon_step(&start, &g, &cands, LoopState::default(), None);
        let after = obs([0.03, 0.0], step.record.selected_face.as_deref());
        let rec = record_after_with_goal(step, &after, &g);
        assert!(rec.record.goal_error_after.is_some());
        assert!(rec.state.last_predicted_pose.is_none());
        assert_ne!(rec.record.outcome, GoalLoopOutcome::AuthorityRefusal);
    }

    #[test]
    fn typed_model_validity_is_not_a_probability() {
        assert_eq!(
            classify_model_validity(
                Some(RotationSign::Clockwise),
                RotationSign::Counterclockwise,
                None,
                None
            ),
            ModelValidity::Contradicted
        );
        assert_eq!(
            classify_model_validity(
                Some(RotationSign::Clockwise),
                RotationSign::Clockwise,
                None,
                None
            ),
            ModelValidity::Consistent
        );
        assert_eq!(
            classify_model_validity(None, RotationSign::Clockwise, None, None),
            ModelValidity::InsufficientEvidence
        );
    }

    #[test]
    fn consistent_model_still_authorizes_strict_progress() {
        let g = goal_plus_x();
        let start = obs([0.0, 0.0], None);
        let cands = evaluated(start.xy, &g);
        let step = receding_horizon_step(&start, &g, &cands, LoopState::default(), None);
        assert!(step.record.selection_rationale.contains("STRICT_PROGRESS"));
        assert_eq!(step.record.authority_decision, "AUTHORIZE");
        assert_eq!(step.record.unauthorized_writes, 0);
        assert_eq!(step.state.unauthorized_writes, 0);
        assert!(step.selected.is_some());
        assert!(step.record.reasoning.is_none());
    }

    #[test]
    fn authorized_stroke_aborts_on_the_first_excess_increment() {
        use crate::execution_envelope::{
            check_execution_envelope, EnvelopeVerdict, ExecutionEnvelope,
            RuntimeExecutionObservation,
        };
        use crate::kinematics::IK_ACCEPT_M;
        use crate::recoverability::{
            classify_recoverability, select_recoverable_progress, InteractionRegion,
            RecoverabilityChoice, RecoverabilityClass, RecoverabilityInput,
        };

        assert_eq!(IK_ACCEPT_M, 1e-3);
        let g = goal_plus_x();
        let start = obs([0.0, 0.0], None);
        let cands = evaluated(start.xy, &g);
        let step = receding_horizon_step(&start, &g, &cands, LoopState::default(), None);
        let selected = step.selected.expect("authorized action");
        let commanded = selected.stroke_m;
        let envelope = ExecutionEnvelope::for_quasi_static_stroke(commanded, commanded);
        let mut sample = RuntimeExecutionObservation {
            stroke_consumed_m: commanded * 0.25,
            commanded_stroke_m: commanded,
            object_displacement_m: commanded * 0.2,
            yaw_change_rad: 0.0,
            intended_contact_persists: true,
            goal_error_before: 1.0,
            goal_error_now: 0.8,
            robot_tracking_error_m: Some(0.0),
            reachability_margin_m: 0.05,
            quasi_static_applicable: Some(true),
            authority_ok: true,
        };
        let continued = check_execution_envelope(&envelope, &sample);
        assert_eq!(continued.verdict, EnvelopeVerdict::Continue);
        sample.stroke_consumed_m = commanded * 0.5;
        sample.object_displacement_m = commanded * 4.0;
        let abort = check_execution_envelope(&envelope, &sample);
        assert_eq!(abort.verdict, EnvelopeVerdict::AbortAndReobserve);
        assert!(abort.early);
        assert!(sample.stroke_consumed_m < commanded);
        let unguarded_displacement = commanded * 8.0;
        assert!(abort.displacement_m < unguarded_displacement);

        let outside = classify_recoverability(&RecoverabilityInput {
            physically_feasible: false,
            makes_progress: false,
            current_xy: [1.0, 1.0],
            nominal_dxy: None,
            uncertainty_radius_m: None,
            region: Some(InteractionRegion {
                center_xy: [0.0, 0.0],
                radius_m: 0.1,
            }),
            next_contact_admissible: Some(false),
        });
        assert_eq!(outside, RecoverabilityClass::PhysicallyInfeasible);
        assert_ne!(outside, RecoverabilityClass::ProgressAndRecoverable);
        assert_eq!(
            goal_status_if_no_admissible_interaction(0),
            GoalLoopOutcome::GoalCurrentlyUnachievable
        );
        let preferred = select_recoverable_progress(&[
            RecoverabilityChoice {
                id: "eject".into(),
                class: RecoverabilityClass::ProgressButCanEnterUnrecoverableState,
                progress: 0.8,
            },
            RecoverabilityChoice {
                id: "stay".into(),
                class: RecoverabilityClass::ProgressAndRecoverable,
                progress: 0.2,
            },
        ])
        .unwrap();
        assert_eq!(preferred, 1);
    }
}
