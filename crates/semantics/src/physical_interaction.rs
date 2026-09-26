//! Generate planar-push candidates from declared geometry and evaluate them
//! through an explicit funnel. Generation never silently drops failures.

use realityos_physics::{ContactMode, PlanarTwist, RotationSign};
use serde::{Deserialize, Serialize};

use crate::contact_maneuver::ContactManeuver;
use crate::contact_manifold::box_vertical_face_manifolds_posed;
use crate::effect_feasibility::{
    evaluate_planar_twist_direction, EffectFeasibility, EffectFeasibilityWitness,
    PlanarPushInitiation,
};
use crate::physical_decision::{
    decide_physical_action, CandidateEvidence, CandidateRole, ContactTransitionPhase,
    DecisionContext, DecisionKind, PredictedPhysicalEffect, ProbeEvidence,
    ProbeRecoverabilityAssessment,
};
use crate::planar_goal::{
    classify_predicted_progress, evaluate_goal_error, predicted_error_derivative, GoalError,
    GoalProgressClass, PlanarObjectGoal,
};
use crate::probe_selection::BeliefRobustness;
use crate::provenance::Provenanced;
use crate::push::{push_approach_standoff_m, PushCandidate};
use crate::recoverability::RecoverabilityClass;
use crate::transform::{add3, scale3, Se3};

const OFFSET_FRACTIONS: [f64; 3] = [-0.55, 0.0, 0.55];

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum FunnelStage {
    Generated,
    GeometryValid,
    RobotReachable,
    CollisionAdmissible,
    ExecutableWitness,
    AvailableEffort,
    MotionInitiation,
    InstantaneousMotion,
    ContactMode,
    GoalUseful,
    Selected,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FunnelTransition {
    pub stage: FunnelStage,
    pub passed: bool,
    pub reason: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FunnelRecord {
    pub stage: FunnelStage,
    pub reason: Option<String>,
    pub transitions: Vec<FunnelTransition>,
}

impl FunnelRecord {
    fn new() -> Self {
        Self {
            stage: FunnelStage::Generated,
            reason: None,
            transitions: vec![FunnelTransition {
                stage: FunnelStage::Generated,
                passed: true,
                reason: None,
            }],
        }
    }

    fn record(&mut self, stage: FunnelStage, passed: bool, reason: Option<String>) {
        self.stage = stage;
        self.reason = reason.clone();
        self.transitions.push(FunnelTransition {
            stage,
            passed,
            reason,
        });
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PhysicalInteractionCandidate {
    pub id: String,
    pub object_id: String,
    pub face_id: String,
    pub contact_point_world: [f64; 3],
    pub contact_normal_world: [f64; 3],
    pub push_direction_world: [f64; 3],
    pub approach_point_world: [f64; 3],
    pub stroke_m: f64,
    pub contact_offset_u: f64,
    pub funnel: FunnelRecord,
    pub predicted_twist: Option<PlanarTwist>,
    pub predicted_contact_mode: Option<ContactMode>,
    pub predicted_rotation_sign: Option<RotationSign>,
    pub predicted_error_derivative: Option<f64>,
    pub goal_progress: Option<GoalProgressClass>,
    pub goal_error_before: Option<GoalError>,
    pub witness: Option<EffectFeasibilityWitness>,
    pub authority_ok: bool,
    pub executable_for_plant: bool,
    /// Set only when a caller supplies an explicit belief-domain assessment.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub decision_robustness: Option<BeliefRobustness>,
    /// Set only when a caller supplies an explicit bounded-state assessment.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub decision_recoverability: Option<RecoverabilityClass>,
    /// Proven Mode B maneuver. Not serialized; execution-only.
    #[serde(skip)]
    pub maneuver: Option<ContactManeuver>,
}

impl PhysicalInteractionCandidate {
    pub fn action_key(&self) -> String {
        format!(
            "{}:{:.5}:{:.4}:{:.4}",
            self.face_id,
            self.contact_offset_u,
            self.push_direction_world[0],
            self.push_direction_world[1]
        )
    }

    pub fn to_push_candidate(&self) -> PushCandidate {
        PushCandidate {
            object_id: self.object_id.clone(),
            contact: Se3 {
                xyz: self.contact_point_world,
                quat_wxyz: [1.0, 0.0, 0.0, 0.0],
            },
            approach: Se3 {
                xyz: self.approach_point_world,
                quat_wxyz: [1.0, 0.0, 0.0, 0.0],
            },
            direction: self.push_direction_world,
            distance_m: self.stroke_m,
            target_region: None,
            reference_frame: "world".into(),
        }
    }
}

#[derive(Debug, Clone)]
pub struct EvaluationContext {
    pub goal: PlanarObjectGoal,
    pub object_xy: [f64; 2],
    pub object_yaw: f64,
    pub object_com_world: [f64; 3],
    pub mechanics_template: Option<PlanarPushInitiation>,
    pub authority_ok: bool,
    pub robot_provided: bool,
    pub robot_reachable: Option<bool>,
    pub collision_admissible: Option<bool>,
    pub executable_witness: Option<bool>,
    pub robot_reject_reason: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum SelectionOutcome {
    Selected {
        index: usize,
        reason: String,
    },
    ContactTransition {
        index: usize,
        reason: String,
        phases: Vec<ContactTransitionPhase>,
    },
    NoUsefulPhysicalAction {
        reason: String,
    },
    InsufficientEvidence {
        reason: String,
    },
    PlannerFailedToFind {
        reason: String,
    },
    GeometricallyUsefulRobotCannotExecute {
        reason: String,
    },
    MechanicsUnknown {
        reason: String,
    },
    AuthorityRefusal {
        reason: String,
        best_index: usize,
    },
}

pub fn generate_planar_push_candidates(
    object_id: &str,
    object_pose: Se3,
    half_extents: [f64; 3],
    support_normal: [f64; 3],
    face_gap: f64,
    stroke_m: f64,
) -> Result<Vec<PhysicalInteractionCandidate>, crate::contact_manifold::ManifoldReject> {
    let faces =
        box_vertical_face_manifolds_posed(object_pose, half_extents, support_normal, face_gap)?;
    let standoff = push_approach_standoff_m();
    let stroke = stroke_m.max(1e-4);
    let mut out = Vec::new();
    for (face_id, man) in faces {
        for frac in OFFSET_FRACTIONS {
            let offset_u = frac * man.u_half;
            let contact = add3(
                add3(man.origin, scale3(man.u, offset_u)),
                scale3(man.normal, man.face_gap),
            );
            let approach = add3(contact, scale3(man.normal, standoff));
            let id = format!("{face_id}:{frac:.2}");
            out.push(PhysicalInteractionCandidate {
                id,
                object_id: object_id.into(),
                face_id: face_id.clone(),
                contact_point_world: contact,
                contact_normal_world: man.normal,
                push_direction_world: man.push,
                approach_point_world: approach,
                stroke_m: stroke,
                contact_offset_u: offset_u,
                funnel: FunnelRecord::new(),
                predicted_twist: None,
                predicted_contact_mode: None,
                predicted_rotation_sign: None,
                predicted_error_derivative: None,
                goal_progress: None,
                goal_error_before: None,
                witness: None,
                authority_ok: true,
                executable_for_plant: false,
                decision_robustness: None,
                decision_recoverability: None,
                maneuver: None,
            });
        }
    }
    Ok(out)
}

fn finite3(v: [f64; 3]) -> bool {
    v.iter().all(|x| x.is_finite())
}

pub fn initiation_from_candidate(
    template: &PlanarPushInitiation,
    cand: &PhysicalInteractionCandidate,
    object_com: [f64; 3],
    object_yaw: f64,
    authority_ok: bool,
) -> PlanarPushInitiation {
    let mut p = template.clone();
    p.contact_point_world =
        Provenanced::declared(cand.contact_point_world, "candidate.contact", 0.0);
    // Planar-push initiation uses the inward (into-object) frame, same as
    // freeze-before-execute. Outward face normal stays on the candidate for
    // approach geometry.
    p.contact_normal_world =
        Provenanced::declared(cand.push_direction_world, "candidate.normal_into", 0.0);
    p.push_direction_world =
        Provenanced::declared(cand.push_direction_world, "candidate.push", 0.0);
    p.contact_force_direction_world =
        Provenanced::declared(cand.push_direction_world, "candidate.force", 0.0);
    p.pusher_velocity_world = Provenanced::declared(cand.push_direction_world, "candidate.vp", 0.0);
    p.object_com_world = Provenanced::declared(object_com, "candidate.com", 0.0);
    p.object_yaw_rad = Provenanced::declared(object_yaw, "candidate.yaw", 0.0);
    p.authority_ok = authority_ok;
    p
}

fn halt(cand: &mut PhysicalInteractionCandidate, stage: FunnelStage, reason: impl Into<String>) {
    cand.funnel.record(stage, false, Some(reason.into()));
}

fn pass(cand: &mut PhysicalInteractionCandidate, stage: FunnelStage, reason: Option<String>) {
    cand.funnel.record(stage, true, reason);
}

pub fn evaluate_candidate(cand: &mut PhysicalInteractionCandidate, ctx: &EvaluationContext) {
    cand.authority_ok = ctx.authority_ok;
    cand.executable_for_plant = false;
    let geom_ok = finite3(cand.contact_point_world)
        && finite3(cand.contact_normal_world)
        && finite3(cand.push_direction_world)
        && cand.stroke_m > 0.0
        && cand.stroke_m <= ctx.goal.safety.max_stroke_m + 1e-9;
    if !geom_ok {
        halt(cand, FunnelStage::GeometryValid, "GEOMETRY_INVALID");
        return;
    }
    pass(cand, FunnelStage::GeometryValid, None);

    match (ctx.robot_provided, ctx.robot_reachable) {
        (true, Some(true)) => pass(cand, FunnelStage::RobotReachable, None),
        (true, Some(false)) => {
            halt(
                cand,
                FunnelStage::RobotReachable,
                ctx.robot_reject_reason
                    .clone()
                    .unwrap_or_else(|| "UNREACHABLE".into()),
            );
            return;
        }
        (true, None) => {
            halt(
                cand,
                FunnelStage::RobotReachable,
                "INSUFFICIENT_REACHABILITY_EVIDENCE",
            );
            return;
        }
        (false, _) => pass(
            cand,
            FunnelStage::RobotReachable,
            Some("NO_EMBODIMENT".into()),
        ),
    }

    match (ctx.robot_provided, ctx.collision_admissible) {
        (true, Some(true)) => pass(cand, FunnelStage::CollisionAdmissible, None),
        (true, Some(false)) => {
            halt(
                cand,
                FunnelStage::CollisionAdmissible,
                ctx.robot_reject_reason
                    .clone()
                    .unwrap_or_else(|| "COLLISION_INADMISSIBLE".into()),
            );
            return;
        }
        (true, None) => {
            halt(
                cand,
                FunnelStage::CollisionAdmissible,
                "INSUFFICIENT_COLLISION_EVIDENCE",
            );
            return;
        }
        (false, _) => pass(
            cand,
            FunnelStage::CollisionAdmissible,
            Some("NO_EMBODIMENT".into()),
        ),
    }

    match (ctx.robot_provided, ctx.executable_witness) {
        (true, Some(true)) => {
            pass(cand, FunnelStage::ExecutableWitness, None);
            cand.executable_for_plant = true;
        }
        (true, Some(false)) => {
            halt(
                cand,
                FunnelStage::ExecutableWitness,
                ctx.robot_reject_reason
                    .clone()
                    .unwrap_or_else(|| "NON_EXECUTABLE".into()),
            );
            return;
        }
        (true, None) => {
            halt(
                cand,
                FunnelStage::ExecutableWitness,
                "INSUFFICIENT_WITNESS_EVIDENCE",
            );
            return;
        }
        (false, _) => pass(
            cand,
            FunnelStage::ExecutableWitness,
            Some("NO_EMBODIMENT".into()),
        ),
    }

    let Some(template) = ctx.mechanics_template.as_ref() else {
        halt(cand, FunnelStage::AvailableEffort, "MECHANICS_UNKNOWN");
        return;
    };
    let init = initiation_from_candidate(
        template,
        cand,
        ctx.object_com_world,
        ctx.object_yaw,
        ctx.authority_ok,
    );
    let w = evaluate_planar_twist_direction(&init);
    cand.witness = Some(w.clone());
    cand.predicted_twist = w.planar_twist;
    cand.predicted_contact_mode = w.contact_mode;
    cand.predicted_rotation_sign = w.rotation_sign;

    match w.effort_bound_kind {
        crate::effect_feasibility::EffortBoundKind::AvailableContactEffortBound => {
            if w.feasibility == EffectFeasibility::Infeasible
                && w.infeasible_reason.as_deref() == Some("INSUFFICIENT_ACTUATOR_EFFORT")
            {
                halt(cand, FunnelStage::AvailableEffort, "INSUFFICIENT_EFFORT");
                return;
            }
            pass(cand, FunnelStage::AvailableEffort, None);
        }
        crate::effect_feasibility::EffortBoundKind::GrossEffortBound => {
            if w.feasibility == EffectFeasibility::Infeasible
                && w.infeasible_reason.as_deref() == Some("INSUFFICIENT_ACTUATOR_EFFORT")
            {
                halt(cand, FunnelStage::AvailableEffort, "INSUFFICIENT_EFFORT");
                return;
            }
            if w.feasibility == EffectFeasibility::Unknown {
                halt(cand, FunnelStage::AvailableEffort, "MECHANICS_UNKNOWN");
                return;
            }
            pass(cand, FunnelStage::AvailableEffort, w.unknown_reason.clone());
        }
    }

    match w.feasibility {
        EffectFeasibility::Feasible => pass(cand, FunnelStage::MotionInitiation, None),
        EffectFeasibility::Infeasible => {
            let r = w
                .infeasible_reason
                .clone()
                .unwrap_or_else(|| "MOTION_INITIATION_INFEASIBLE".into());
            halt(cand, FunnelStage::MotionInitiation, r);
            return;
        }
        EffectFeasibility::Unknown => {
            halt(cand, FunnelStage::MotionInitiation, "MECHANICS_UNKNOWN");
            return;
        }
    }

    match w.physical_levels.instantaneous_motion {
        EffectFeasibility::Feasible => pass(cand, FunnelStage::InstantaneousMotion, None),
        EffectFeasibility::Infeasible => {
            halt(
                cand,
                FunnelStage::InstantaneousMotion,
                "INSTANTANEOUS_INFEASIBLE",
            );
            return;
        }
        EffectFeasibility::Unknown => {
            halt(cand, FunnelStage::InstantaneousMotion, "MECHANICS_UNKNOWN");
            return;
        }
    }

    match w.contact_mode {
        Some(mode) => pass(cand, FunnelStage::ContactMode, Some(format!("{mode:?}"))),
        None => pass(
            cand,
            FunnelStage::ContactMode,
            Some("CONTACT_MODE_UNDECLARED".into()),
        ),
    }

    let err = evaluate_goal_error(ctx.object_xy, ctx.object_yaw, &ctx.goal);
    cand.goal_error_before = Some(err.clone());
    cand.predicted_error_derivative = predicted_error_derivative(
        &err,
        w.planar_twist.unwrap_or(PlanarTwist {
            vx: 0.0,
            vy: 0.0,
            omega_z: 0.0,
            frame: realityos_physics::PlanarFrameKind::World,
        }),
        &ctx.goal,
        ctx.object_yaw,
    );
    let progress = classify_predicted_progress(&err, w.planar_twist, &ctx.goal, ctx.object_yaw);
    cand.goal_progress = Some(progress);
    if progress == GoalProgressClass::StrictProgress {
        pass(cand, FunnelStage::GoalUseful, None);
    } else {
        halt(cand, FunnelStage::GoalUseful, "NOT_GOAL_USEFUL");
    }
}

pub fn evaluate_all(cands: &mut [PhysicalInteractionCandidate], ctx: &EvaluationContext) {
    for c in cands.iter_mut() {
        evaluate_candidate(c, ctx);
    }
}

fn is_goal_useful(c: &PhysicalInteractionCandidate) -> bool {
    c.funnel.stage == FunnelStage::GoalUseful
        && c.goal_progress == Some(GoalProgressClass::StrictProgress)
}

fn rejected_at(c: &PhysicalInteractionCandidate, stage: FunnelStage) -> bool {
    c.funnel
        .transitions
        .iter()
        .any(|t| t.stage == stage && !t.passed)
}

/// Return whether this physical action identity has already been refused after
/// contradictory execution evidence. Keep this predicate shared by the
/// canonical selector and any selector that narrows its candidate set.
pub fn action_key_is_forbidden(action_key: &str, forbidden_keys: &[String]) -> bool {
    forbidden_keys.iter().any(|key| key == action_key)
}

fn select_with_physical_decision(
    cands: &[PhysicalInteractionCandidate],
    forbidden_keys: &[String],
    current_contact_id: Option<&str>,
    evidence_fresh: bool,
    remaining_attempts: u32,
) -> SelectionOutcome {
    let evidence: Vec<_> = cands
        .iter()
        .map(|candidate| {
            let candidate_contents = serde_json::to_string(candidate).unwrap_or_default();
            let witness_contents = candidate
                .witness
                .as_ref()
                .zip(
                    candidate
                        .maneuver
                        .as_ref()
                        .and_then(|maneuver| maneuver.executable.as_ref()),
                )
                .and_then(|(mechanics, execution)| {
                    serde_json::to_string(&(mechanics, execution)).ok()
                });
            let witness_digest = witness_contents.as_ref().map(|contents| {
                use sha2::Digest;
                format!("{:x}", sha2::Sha256::digest(contents.as_bytes()))
            });
            let executable = candidate.executable_for_plant
                && candidate
                    .maneuver
                    .as_ref()
                    .and_then(|maneuver| maneuver.executable.as_ref())
                    .is_some_and(|witness| witness.is_executable());
            let mut hard_rejections = Vec::new();
            if !is_goal_useful(candidate) {
                hard_rejections.push(crate::physical_decision::CandidateRejection::Other(
                    "NOT_STRICT_GOAL_USEFUL".into(),
                ));
            }
            if !executable {
                hard_rejections
                    .push(crate::physical_decision::CandidateRejection::WitnessUnavailable);
            }
            CandidateEvidence {
                candidate_id: candidate.id.clone(),
                candidate_contents,
                action_key: candidate.action_key(),
                contact_id: candidate.face_id.clone(),
                role: CandidateRole::GoalAction,
                strict_goal_progress: candidate.goal_progress
                    == Some(GoalProgressClass::StrictProgress),
                authority_ok: candidate.authority_ok,
                executable_witness_id: executable.then(|| format!("{}:witness", candidate.id)),
                witness_digest,
                witness_contents,
                robustness: candidate
                    .decision_robustness
                    .unwrap_or(BeliefRobustness::Ambiguous),
                recoverability: candidate
                    .decision_recoverability
                    .unwrap_or(RecoverabilityClass::ProgressButRecoverabilityUnknown),
                predicted_effect: PredictedPhysicalEffect {
                    object_translation_world_m: None,
                    yaw_change_rad: None,
                    contact_persists: None,
                    goal_error_derivative: candidate.predicted_error_derivative,
                    goal_progress: candidate.goal_progress,
                },
                probe: ProbeEvidence {
                    decision_relevant_distinctions: 0,
                    observable_distinctions: 0,
                    future_interaction: ProbeRecoverabilityAssessment::Unknown,
                },
                preference: crate::physical_decision::LexicographicPreference {
                    error_derivative: candidate.predicted_error_derivative,
                    angular_rate_abs: candidate.predicted_twist.map(|twist| twist.omega_z.abs()),
                    contact_offset_abs_m: Some(candidate.contact_offset_u.abs()),
                    stroke_m: candidate.stroke_m,
                },
                hard_rejections,
            }
        })
        .collect();
    let decision = decide_physical_action(&DecisionContext {
        goal_id: cands
            .first()
            .map(|candidate| candidate.object_id.clone())
            .unwrap_or_default(),
        goal_reached: false,
        evidence_fresh,
        remaining_attempts,
        current_contact_id: current_contact_id.map(str::to_string),
        forbidden_action_keys: forbidden_keys.to_vec(),
        candidates: evidence,
    });
    match decision {
        DecisionKind::GoalInteraction { action } => {
            let Some(index) = cands
                .iter()
                .position(|candidate| candidate.id == action.candidate_id)
            else {
                return SelectionOutcome::PlannerFailedToFind {
                    reason: "CANONICAL_DECISION_ID_NOT_IN_CANDIDATE_SET".into(),
                };
            };
            SelectionOutcome::Selected {
                index,
                reason: action.rationale,
            }
        }
        DecisionKind::ContactTransition { action, phases } => {
            let Some(index) = cands
                .iter()
                .position(|candidate| candidate.id == action.candidate_id)
            else {
                return SelectionOutcome::PlannerFailedToFind {
                    reason: "CANONICAL_DECISION_ID_NOT_IN_CANDIDATE_SET".into(),
                };
            };
            SelectionOutcome::ContactTransition {
                index,
                reason: action.rationale,
                phases,
            }
        }
        DecisionKind::Refuse { reason } => SelectionOutcome::AuthorityRefusal {
            reason,
            best_index: cands.iter().position(is_goal_useful).unwrap_or(0),
        },
        DecisionKind::InsufficientEvidence { reason } => {
            SelectionOutcome::InsufficientEvidence { reason }
        }
        DecisionKind::PhysicallyInfeasible { reason }
        | DecisionKind::CurrentlyUnachievable { reason } => {
            SelectionOutcome::NoUsefulPhysicalAction { reason }
        }
        DecisionKind::PhysicalProbe { .. } | DecisionKind::GoalReached { .. } => {
            SelectionOutcome::NoUsefulPhysicalAction {
                reason: "UNEXPECTED_DECISION_CLASS_FOR_GOAL_INTERACTION".into(),
            }
        }
    }
}

pub fn select_interaction(
    cands: &[PhysicalInteractionCandidate],
    forbidden_keys: &[String],
) -> SelectionOutcome {
    select_interaction_with_context(cands, forbidden_keys, None, true, 1)
}

/// Select with the live policy-contact identity and remaining decision budget.
/// The canonical decision remains the only authority for ranking and gating.
pub fn select_interaction_with_context(
    cands: &[PhysicalInteractionCandidate],
    forbidden_keys: &[String],
    current_contact_id: Option<&str>,
    evidence_fresh: bool,
    remaining_attempts: u32,
) -> SelectionOutcome {
    if cands.is_empty() {
        return SelectionOutcome::PlannerFailedToFind {
            reason: "NO_CANDIDATES_GENERATED".into(),
        };
    }
    select_with_physical_decision(
        cands,
        forbidden_keys,
        current_contact_id,
        evidence_fresh,
        remaining_attempts,
    )
}

pub fn funnel_reject_counts(cands: &[PhysicalInteractionCandidate]) -> Vec<(FunnelStage, usize)> {
    let stages = [
        FunnelStage::GeometryValid,
        FunnelStage::RobotReachable,
        FunnelStage::CollisionAdmissible,
        FunnelStage::ExecutableWitness,
        FunnelStage::AvailableEffort,
        FunnelStage::MotionInitiation,
        FunnelStage::InstantaneousMotion,
        FunnelStage::ContactMode,
        FunnelStage::GoalUseful,
    ];
    stages
        .iter()
        .map(|s| (*s, cands.iter().filter(|c| rejected_at(c, *s)).count()))
        .collect()
}

#[cfg(test)]
pub(crate) mod test_support {
    use super::*;
    use crate::maneuver_witness::{
        ExecutableContactManeuver, ManeuverPhase, PhaseTransition, TransitionKind,
    };

    /// Explicit synthetic proof used only by selector and goal-loop unit tests.
    pub(crate) fn attach_executable_witnesses(candidates: &mut [PhysicalInteractionCandidate]) {
        for candidate in candidates {
            let strict = is_goal_useful(candidate);
            candidate.decision_robustness = Some(if strict {
                BeliefRobustness::RobustStrictProgress
            } else {
                BeliefRobustness::Ambiguous
            });
            candidate.decision_recoverability = Some(if strict {
                RecoverabilityClass::ProgressAndRecoverable
            } else {
                RecoverabilityClass::ProgressButRecoverabilityUnknown
            });
            if !strict {
                continue;
            }

            let pose = |xyz| Se3 {
                xyz,
                quat_wxyz: [1.0, 0.0, 0.0, 0.0],
            };
            let contact = candidate.contact_point_world;
            let approach = candidate.approach_point_world;
            let mid_stroke = [
                contact[0] + candidate.push_direction_world[0] * candidate.stroke_m * 0.5,
                contact[1] + candidate.push_direction_world[1] * candidate.stroke_m * 0.5,
                contact[2] + candidate.push_direction_world[2] * candidate.stroke_m * 0.5,
            ];
            let end_stroke = [
                contact[0] + candidate.push_direction_world[0] * candidate.stroke_m,
                contact[1] + candidate.push_direction_world[1] * candidate.stroke_m,
                contact[2] + candidate.push_direction_world[2] * candidate.stroke_m,
            ];
            let phase = |xyz| ManeuverPhase::positional(pose(xyz), vec![0.0], vec![0.0], 0.0, 0.0);
            let transition = |kind| PhaseTransition::feasible(kind, 1, 0.5);
            candidate.executable_for_plant = true;
            candidate.maneuver = Some(ContactManeuver {
                contact_pose: pose(contact),
                approach_pose: pose(approach),
                contact_point: contact,
                contact_normal: candidate.contact_normal_world,
                push_direction: candidate.push_direction_world,
                requested_stroke: candidate.stroke_m,
                available_stroke: candidate.stroke_m,
                support_clearance: 0.01,
                joint_margin: 0.5,
                orientation_error: 0.0,
                object_center: [0.0, 0.0, 0.03],
                support_top_z: 0.0,
                sampled_q: vec![0.0],
                executable: Some(ExecutableContactManeuver {
                    joint_names: vec!["test_joint".into()],
                    start_q: vec![0.0],
                    approach: phase(approach),
                    contact: phase(contact),
                    mid_stroke: phase(mid_stroke),
                    end_stroke: phase(end_stroke),
                    current_to_approach: transition(TransitionKind::CurrentToApproach),
                    approach_to_contact: transition(TransitionKind::ApproachToContact),
                    contact_to_mid: transition(TransitionKind::ContactToMidStroke),
                    mid_to_end: transition(TransitionKind::MidToEndStroke),
                }),
            });
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::effect_feasibility::PlanarPushInitiation;
    use crate::pair_friction::PairFriction;
    use crate::planar_goal::{InteractionFamily, SafetyConstraints};
    use realityos_physics::{PressureDistribution, SupportFrictionModel};

    fn identity_j() -> Vec<Vec<f64>> {
        vec![
            vec![1.0, 0.0, 0.0],
            vec![0.0, 1.0, 0.0],
            vec![0.0, 0.0, 1.0],
        ]
    }

    fn mechanics(mass: f64, mu_s: f64, tau: f64, known_mass: bool) -> PlanarPushInitiation {
        let n = mass * 9.80665;
        let f_max = mu_s * n;
        PlanarPushInitiation {
            mass_kg: if known_mass {
                Provenanced::declared(mass, "test.mass", 0.0)
            } else {
                Provenanced::unknown("mass", 0.0)
            },
            object_com_world: Provenanced::declared([0.0, 0.0, 0.03], "test.com", 0.0),
            gravity_m_s2: Provenanced::declared([0.0, 0.0, -9.80665], "test.g", 0.0),
            support_normal: Provenanced::declared([0.0, 0.0, 1.0], "test.n", 0.0),
            object_support_friction: PairFriction::coulomb(
                "object",
                "support",
                Provenanced::declared(mu_s, "test.mu_s", 0.0),
            ),
            tool_object_friction: PairFriction::coulomb(
                "tool",
                "object",
                Provenanced::declared(0.8, "test.mu_t", 0.0),
            ),
            contact_point_world: Provenanced::unknown("c", 0.0),
            contact_normal_world: Provenanced::unknown("n", 0.0),
            push_direction_world: Provenanced::unknown("d", 0.0),
            contact_force_direction_world: Provenanced::unknown("f", 0.0),
            pusher_velocity_world: Provenanced::unknown("v", 0.0),
            joint_names: vec!["j0".into(), "j1".into(), "j2".into()],
            translational_jacobian_3xn: identity_j(),
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

    fn trans_goal(target: [f64; 2]) -> PlanarObjectGoal {
        PlanarObjectGoal {
            object_id: "obj0".into(),
            world_id: "w".into(),
            model_id: "m".into(),
            target_xy: Some(target),
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

    fn pose_at(xy: [f64; 2]) -> Se3 {
        Se3 {
            xyz: [xy[0], xy[1], 0.03],
            quat_wxyz: [1.0, 0.0, 0.0, 0.0],
        }
    }

    fn ctx_for(
        goal: PlanarObjectGoal,
        xy: [f64; 2],
        mech: PlanarPushInitiation,
        robot: bool,
        reachable: Option<bool>,
        collision: Option<bool>,
        exec: Option<bool>,
    ) -> EvaluationContext {
        EvaluationContext {
            goal,
            object_xy: xy,
            object_yaw: 0.0,
            object_com_world: [xy[0], xy[1], 0.03],
            mechanics_template: Some(mech),
            authority_ok: true,
            robot_provided: robot,
            robot_reachable: reachable,
            collision_admissible: collision,
            executable_witness: exec,
            robot_reject_reason: None,
        }
    }

    #[test]
    fn generation_keeps_every_face_offset_and_does_not_drop() {
        let cands = generate_planar_push_candidates(
            "obj0",
            pose_at([0.0, 0.0]),
            [0.04, 0.03, 0.03],
            [0.0, 0.0, 1.0],
            0.01,
            0.02,
        )
        .unwrap();
        assert_eq!(cands.len(), 12);
        assert!(cands
            .iter()
            .all(|c| c.funnel.stage == FunnelStage::Generated));
        let faces: Vec<_> = cands.iter().map(|c| c.face_id.as_str()).collect();
        assert!(faces.contains(&"-x") && faces.contains(&"+x"));
        assert!(faces.contains(&"-y") && faces.contains(&"+y"));
    }

    #[test]
    fn removing_physical_geometry_evidence_never_creates_push_candidates() {
        let pose = pose_at([0.0, 0.0]);
        let valid = generate_planar_push_candidates(
            "obj0",
            pose,
            [0.04, 0.03, 0.03],
            [0.0, 0.0, 1.0],
            0.01,
            0.02,
        )
        .unwrap();
        assert_eq!(valid.len(), 12, "valid explicit support is the control");

        for missing_support in [[0.0; 3], [f64::NAN, 0.0, 1.0], [0.0, f64::INFINITY, 1.0]] {
            assert_eq!(
                generate_planar_push_candidates(
                    "obj0",
                    pose,
                    [0.04, 0.03, 0.03],
                    missing_support,
                    0.01,
                    0.02,
                )
                .unwrap_err(),
                crate::contact_manifold::ManifoldReject::InvalidSupportPlane,
                "removing support evidence ({missing_support:?}) must not produce candidates"
            );
        }
        let large_finite_direction = generate_planar_push_candidates(
            "obj0",
            pose,
            [0.04, 0.03, 0.03],
            [f64::MAX, 0.0, 0.0],
            0.01,
            0.02,
        )
        .unwrap();
        assert_eq!(large_finite_direction.len(), 6);
        assert!(large_finite_direction
            .iter()
            .all(|candidate| candidate.contact_point_world.iter().all(|v| v.is_finite())));

        let malformed_pose = Se3 {
            xyz: [f64::NAN, 0.0, 0.03],
            quat_wxyz: [1.0, 0.0, 0.0, 0.0],
        };
        assert_eq!(
            generate_planar_push_candidates(
                "obj0",
                malformed_pose,
                [0.04, 0.03, 0.03],
                [0.0, 0.0, 1.0],
                0.01,
                0.02,
            )
            .unwrap_err(),
            crate::contact_manifold::ManifoldReject::InvalidObjectPose
        );
    }

    #[test]
    fn funnel_records_each_named_rejection() {
        let goal = trans_goal([0.2, 0.0]);
        let xy = [0.0, 0.0];
        let mut cands = generate_planar_push_candidates(
            "obj0",
            pose_at(xy),
            [0.04, 0.03, 0.03],
            [0.0, 0.0, 1.0],
            0.01,
            0.02,
        )
        .unwrap();
        let one = cands.remove(0);
        let mut geom = one.clone();
        geom.stroke_m = 0.0;
        evaluate_candidate(
            &mut geom,
            &ctx_for(
                goal.clone(),
                xy,
                mechanics(0.1, 0.2, 20.0, true),
                false,
                None,
                None,
                None,
            ),
        );
        assert_eq!(geom.funnel.reason.as_deref(), Some("GEOMETRY_INVALID"));
        let cases: [(&str, EvaluationContext); 6] = [
            (
                "UNREACHABLE",
                ctx_for(
                    goal.clone(),
                    xy,
                    mechanics(0.1, 0.2, 20.0, true),
                    true,
                    Some(false),
                    Some(true),
                    Some(true),
                ),
            ),
            (
                "COLLISION_INADMISSIBLE",
                ctx_for(
                    goal.clone(),
                    xy,
                    mechanics(0.1, 0.2, 20.0, true),
                    true,
                    Some(true),
                    Some(false),
                    Some(true),
                ),
            ),
            (
                "NON_EXECUTABLE",
                ctx_for(
                    goal.clone(),
                    xy,
                    mechanics(0.1, 0.2, 20.0, true),
                    true,
                    Some(true),
                    Some(true),
                    Some(false),
                ),
            ),
            (
                "INSUFFICIENT_EFFORT",
                ctx_for(
                    goal.clone(),
                    xy,
                    mechanics(3.0, 2.0, 0.01, true),
                    false,
                    None,
                    None,
                    None,
                ),
            ),
            (
                "MECHANICS_UNKNOWN",
                ctx_for(
                    goal.clone(),
                    xy,
                    mechanics(0.1, 0.2, 20.0, false),
                    false,
                    None,
                    None,
                    None,
                ),
            ),
            (
                "NOT_GOAL_USEFUL",
                ctx_for(
                    trans_goal([0.2, 0.0]),
                    xy,
                    mechanics(0.1, 0.2, 20.0, true),
                    false,
                    None,
                    None,
                    None,
                ),
            ),
        ];
        for (reason, ctx) in cases {
            let mut c = one.clone();
            evaluate_candidate(&mut c, &ctx);
            assert!(
                c.funnel.reason.as_deref() == Some(reason)
                    || c.funnel
                        .transitions
                        .iter()
                        .any(|t| !t.passed && t.reason.as_deref() == Some(reason)),
                "expected {reason}, got {:?} stage={:?}",
                c.funnel.reason,
                c.funnel.stage
            );
            assert_ne!(c.funnel.stage, FunnelStage::Selected);
        }
    }

    #[test]
    fn goal_usefulness_without_execution_proof_does_not_select_an_action() {
        let goal = trans_goal([0.2, 0.0]);
        let xy = [0.0, 0.0];
        let mut candidates = generate_planar_push_candidates(
            "obj0",
            pose_at(xy),
            [0.04, 0.03, 0.03],
            [0.0, 0.0, 1.0],
            0.01,
            0.02,
        )
        .unwrap();
        evaluate_all(
            &mut candidates,
            &ctx_for(
                goal,
                xy,
                mechanics(0.1, 0.2, 20.0, true),
                false,
                None,
                None,
                None,
            ),
        );

        assert!(candidates.iter().any(is_goal_useful));
        assert!(matches!(
            select_interaction(&candidates, &[]),
            SelectionOutcome::InsufficientEvidence { .. }
        ));
    }

    #[test]
    fn executable_witness_without_belief_and_recoverability_assessments_is_not_selected() {
        let goal = trans_goal([0.2, 0.0]);
        let xy = [0.0, 0.0];
        let mut candidates = generate_planar_push_candidates(
            "obj0",
            pose_at(xy),
            [0.04, 0.03, 0.03],
            [0.0, 0.0, 1.0],
            0.01,
            0.02,
        )
        .unwrap();
        evaluate_all(
            &mut candidates,
            &ctx_for(
                goal,
                xy,
                mechanics(0.1, 0.2, 20.0, true),
                false,
                None,
                None,
                None,
            ),
        );
        test_support::attach_executable_witnesses(&mut candidates);
        let mut executable: Vec<_> = candidates.into_iter().filter(is_goal_useful).collect();
        assert!(executable
            .iter()
            .all(|candidate| candidate.executable_for_plant));
        for candidate in &mut executable {
            candidate.decision_robustness = None;
            candidate.decision_recoverability = None;
        }

        assert!(matches!(
            select_interaction(&executable, &[]),
            SelectionOutcome::InsufficientEvidence { .. }
        ));
    }

    #[test]
    fn counterfactual_prefers_progress_then_flips_with_target() {
        let xy = [0.0, 0.0];
        let mut toward = generate_planar_push_candidates(
            "obj0",
            pose_at(xy),
            [0.04, 0.03, 0.03],
            [0.0, 0.0, 1.0],
            0.01,
            0.02,
        )
        .unwrap();
        let mech = mechanics(0.1, 0.2, 20.0, true);
        let ctx_plus = ctx_for(
            trans_goal([0.2, 0.0]),
            xy,
            mech.clone(),
            false,
            None,
            None,
            None,
        );
        evaluate_all(&mut toward, &ctx_plus);
        test_support::attach_executable_witnesses(&mut toward);
        let sel = select_interaction(&toward, &[]);
        let (ia, ra) = match sel {
            SelectionOutcome::Selected { index, reason }
            | SelectionOutcome::ContactTransition { index, reason, .. } => (index, reason),
            other => panic!("expected A selected, got {other:?}"),
        };
        assert_eq!(
            toward[ia].goal_progress,
            Some(GoalProgressClass::StrictProgress)
        );
        assert!(ra.contains("STRICT_PROGRESS"));
        let face_a = toward[ia].face_id.clone();
        let key_a = toward[ia].action_key();

        let mut away = generate_planar_push_candidates(
            "obj0",
            pose_at(xy),
            [0.04, 0.03, 0.03],
            [0.0, 0.0, 1.0],
            0.01,
            0.02,
        )
        .unwrap();
        let ctx_minus = ctx_for(trans_goal([-0.2, 0.0]), xy, mech, false, None, None, None);
        evaluate_all(&mut away, &ctx_minus);
        test_support::attach_executable_witnesses(&mut away);
        let sel_b = select_interaction(&away, &[]);
        let ib = match sel_b {
            SelectionOutcome::Selected { index, .. }
            | SelectionOutcome::ContactTransition { index, .. } => index,
            other => panic!("expected B selected after target flip, got {other:?}"),
        };
        assert_eq!(
            away[ib].goal_progress,
            Some(GoalProgressClass::StrictProgress)
        );
        assert_ne!(
            away[ib].face_id, face_a,
            "selection must change with the target"
        );
        assert_ne!(away[ib].action_key(), key_a);

        let still = select_interaction(&toward, &[]);
        assert!(matches!(
            still,
            SelectionOutcome::Selected { index, .. }
                | SelectionOutcome::ContactTransition { index, .. } if index == ia
        ));

        // A progress, B regression, C infeasible, D unknown — prefer A.
        let mut abcd = toward.clone();
        let mut saw_b = false;
        let mut saw_c = false;
        let mut saw_d = false;
        for c in &mut abcd {
            if c.id == toward[ia].id {
                continue;
            }
            if !saw_b
                && (c.goal_progress == Some(GoalProgressClass::Regression)
                    || c.funnel.reason.as_deref() == Some("NOT_GOAL_USEFUL"))
            {
                saw_b = true;
                continue;
            }
            if !saw_c {
                c.funnel.record(
                    FunnelStage::MotionInitiation,
                    false,
                    Some("INFEASIBLE".into()),
                );
                c.goal_progress = None;
                saw_c = true;
                continue;
            }
            if !saw_d {
                c.funnel.record(
                    FunnelStage::InstantaneousMotion,
                    false,
                    Some("MECHANICS_UNKNOWN".into()),
                );
                c.goal_progress = None;
                saw_d = true;
            }
        }
        assert!(saw_b && saw_c && saw_d);
        let abcd_sel = select_interaction(&abcd, &[]);
        match abcd_sel {
            SelectionOutcome::Selected { index, reason }
            | SelectionOutcome::ContactTransition { index, reason, .. } => {
                assert_eq!(abcd[index].id, toward[ia].id);
                assert!(reason.contains("STRICT_PROGRESS"));
            }
            other => panic!("ABCD must prefer A, got {other:?}"),
        }
    }

    #[test]
    fn source_has_no_identity_or_seed_policy() {
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("src");
        for name in ["planar_goal.rs", "physical_interaction.rs", "goal_loop.rs"] {
            let t = std::fs::read_to_string(root.join(name)).unwrap();
            let lower = t.to_ascii_lowercase();
            let seed_policy = ["if ", "seed"].concat();
            let name_policy = ["if ", "object_name"].concat();
            let side_policy = ["push from left when ", "target is right"].concat();
            assert!(!lower.contains(&seed_policy), "{name} contains seed policy");
            assert!(
                !lower.contains(&name_policy),
                "{name} contains object-name policy"
            );
            assert!(
                !lower.contains(&side_policy),
                "{name} hard-codes a left/right policy"
            );
        }
    }
}
