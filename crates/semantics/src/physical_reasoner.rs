//! Production physical reasoner. No plant writes, no MuJoCo state, no grant issuance.
//!
//! Push and pinch-grasp are two consumers of the same candidate, proof, decision,
//! belief, and authorization boundary. The decision engine does not contain a
//! push-specific branch.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

use crate::adapter::synth_planar_two_link;
use crate::allowed_contact::ContactPhase;
use crate::contact_collision::{
    policy_from_scene_model, scene_from_collision_world, CollisionWorld, NamedBox,
};
use crate::embodiment::EmbodimentModel;
use crate::execution_envelope::{
    contents_digest, ExecutionAuthorization, ExecutionEnvelope, FrozenAction, ObservationContract,
    ObservationField,
};
use crate::future_interaction::{
    assess_probe_future, domain_from_exact_axis_sweep, CommonWitnessCoverage, ProbeFutureInputs,
    SupportedProbeOutcome,
};
use crate::grasp_hold::{evaluate_pinch_hold, HoldFeasibility, PinchHoldInput};
use crate::kinematics::{forward_kinematics, ik_residual_is_precise, solve_ik, with_ik_q_seed};
use crate::pair_friction::PairFriction;
use crate::physical_belief::{BeliefEpistemicStatus, PhysicalParameter, PhysicalParameterBelief};
use crate::physical_consequence::FrozenPrediction;
use crate::physical_decision::CandidateRole::{GoalAction, PhysicalProbe};
use crate::physical_decision::{
    decide_physical_action, CandidateEvidence, CandidateRole, DecisionContext, DecisionKind,
    LexicographicPreference, PredictedPhysicalEffect, ProbeEvidence, ProbeRecoverabilityAssessment,
    ScopedGrantBinding,
};
use crate::planar_goal::GoalProgressClass;
use crate::probe_selection::BeliefRobustness;
use crate::provenance::Provenanced;
use crate::recoverability::RecoverabilityClass;
use crate::transform::{add3, norm3, scale3, sub3};
use crate::transition_validity::validate_transition;
use crate::work_counters;

pub const EVIDENCE_STATUS: &str = "SIMULATION_ONLY";
pub const BENCHMARK_SCHEMA: &str = "realityos.physical_benchmark/1";

const PUSH_FAMILY: &str = "planar_push";
const GRASP_FAMILY: &str = "pinch_grasp";

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum PhysicalObjective {
    PlanarTranslation { target_xy: [f64; 2] },
    AcquireObject { object_id: String },
}

#[derive(Debug, Clone, PartialEq)]
pub struct TableScene {
    pub object_id: String,
    pub object_center: [f64; 3],
    pub object_half: [f64; 3],
    pub object_quat: [f64; 4],
    pub support_origin: [f64; 3],
    pub support_normal: [f64; 3],
    pub obstacles: Vec<NamedBox>,
    pub arm_q: Vec<f64>,
    pub joint_names: Vec<String>,
    pub ee: String,
    pub tool_radius_m: f64,
    pub observation_epoch: String,
    pub held: bool,
    pub finger_names: Vec<String>,
    pub finger_q: Vec<f64>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct ReasoningRequest {
    pub scene: TableScene,
    pub belief: PhysicalParameterBelief,
    pub objective: PhysicalObjective,
    pub mu_required: f64,
    pub goal_stroke_m: f64,
    pub probe_stroke_m: f64,
    pub grants: Vec<ScopedGrantBinding>,
    pub now_s: f64,
    pub evidence_fresh: bool,
    pub mass_kg: Option<f64>,
    pub gripper_force_n: Option<f64>,
    pub gravity_m_s2: [f64; 3],
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum MechanicsStatus {
    Feasible,
    Infeasible,
    Unknown,
}

#[derive(Debug, Clone, PartialEq)]
pub struct CandidateSummary {
    pub family: String,
    pub action_key: String,
    pub status: String,
    pub mechanics: MechanicsStatus,
    pub immediate_progress: bool,
    pub error_derivative: Option<f64>,
    pub witness_digest: Option<String>,
    pub witness_contents: Option<String>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct PlanStep {
    pub family: String,
    pub action_key: String,
    pub candidate_id: String,
    pub witness_digest: String,
    pub witness_contents: String,
    pub stroke_m: f64,
    pub immediate_progress: bool,
    pub predicted_translation_m: f64,
    pub error_after: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct PhysicalPlan {
    pub steps: Vec<PlanStep>,
    pub greedy_failed: bool,
    /// A plan predicts later actions. It does not authorize them.
    pub conditional: bool,
    pub expansions: u32,
}

#[derive(Debug, Clone, PartialEq)]
pub struct ReasoningReport {
    pub candidates: Vec<CandidateSummary>,
    pub decision: DecisionKind,
    /// Conditional prediction. Later steps are not executable yet.
    pub plan: Option<PhysicalPlan>,
    /// The single next action justified by the current observation.
    pub executable: Option<PlanStep>,
    pub families: Vec<String>,
    pub evidence_status: &'static str,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ProofCacheStats {
    pub executable_hits: u64,
    pub executable_misses: u64,
    pub geometric_hits: u64,
    pub geometric_misses: u64,
    pub ik_avoided: u64,
    pub fk_avoided: u64,
    pub collision_avoided: u64,
    pub proof_latency_avoided_ns: u64,
    pub last_proof_latency_ns: u64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct AuthorizedAction {
    frozen: FrozenAction,
}

impl AuthorizedAction {
    pub fn frozen(&self) -> &FrozenAction {
        &self.frozen
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ExperienceReceipt {
    pub belief_before: PhysicalParameterBelief,
    pub belief_after: PhysicalParameterBelief,
    pub observation: String,
    pub information_gain: bool,
    pub evidence_status: &'static str,
    pub parameter: PhysicalParameter,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BenchmarkReport {
    pub schema: &'static str,
    pub evidence_status: &'static str,
    pub cold_ik_attempts: u64,
    pub warm_ik_attempts: u64,
    pub cold_fk_evaluations: u64,
    pub warm_fk_evaluations: u64,
    pub cold_collision_queries: u64,
    pub warm_collision_queries: u64,
    pub cold_jacobian_evaluations: u64,
    pub warm_jacobian_evaluations: u64,
    pub cold_mechanics_evaluations: u64,
    pub warm_mechanics_evaluations: u64,
    pub tasks_attempted: u32,
    pub tasks_solved: u32,
    pub tasks_correctly_refused: u32,
    pub greedy_failed_on_solved_task: bool,
    pub parallel_evaluation: bool,
    pub cache_hits: u64,
    pub cache_misses: u64,
    pub ik_avoided: u64,
    pub collision_avoided: u64,
    pub notes: Vec<String>,
}

#[derive(Clone)]
struct ProvedAction {
    summary: CandidateSummary,
    evidence: Option<CandidateEvidence>,
    step: Option<PlanStep>,
    successor_center: [f64; 3],
    successor_held: bool,
}

struct CachedProof {
    action: ProvedAction,
    ik: u64,
    fk: u64,
    collision: u64,
    latency_ns: u64,
}

pub struct PhysicalReasoner {
    model: EmbodimentModel,
    proofs: BTreeMap<String, CachedProof>,
    geometric: BTreeMap<String, String>,
    stats: ProofCacheStats,
    aborted_grants: std::collections::BTreeSet<String>,
}

impl PhysicalReasoner {
    pub fn new(model: EmbodimentModel) -> Self {
        Self {
            model,
            proofs: BTreeMap::new(),
            geometric: BTreeMap::new(),
            stats: ProofCacheStats::default(),
            aborted_grants: std::collections::BTreeSet::new(),
        }
    }

    pub fn cache_stats(&self) -> ProofCacheStats {
        self.stats
    }

    pub fn model_hash(&self) -> &str {
        &self.model.model_hash
    }

    pub fn robot_id(&self) -> &str {
        &self.model.robot_id
    }

    /// An aborted grant cannot be resumed. A later action needs a new proof and a new grant.
    pub fn abort_grant(&mut self, action: &AuthorizedAction) {
        if let Some(grant) = action.frozen().execution_authorization.as_ref() {
            self.aborted_grants.insert(grant.grant_id.clone());
        }
    }

    pub fn resume_aborted(&self, action: &AuthorizedAction) -> Result<(), &'static str> {
        let Some(grant) = action.frozen().execution_authorization.as_ref() else {
            return Err("NO_GRANT");
        };
        if self.aborted_grants.contains(&grant.grant_id) {
            return Err("ABORTED_GRANT_CANNOT_RESUME");
        }
        Ok(())
    }

    pub fn families(&self) -> [&'static str; 2] {
        [PUSH_FAMILY, GRASP_FAMILY]
    }

    pub fn consider(&mut self, request: &ReasoningRequest) -> ReasoningReport {
        let proved = self.actions_at(request);
        let summaries = proved.iter().map(|action| action.summary.clone()).collect();
        let decision = decide_physical_action(&decision_context(request, &proved));
        self.note_geometric_fact();
        let plan = search_plan(self, request);
        let greedy_key = decision_action_key(&decision);
        let greedy_failed = plan.as_ref().is_some_and(|plan| {
            plan.steps.first().is_some_and(|step| {
                !step.immediate_progress && Some(step.action_key.as_str()) != greedy_key
            })
        });
        let plan = plan.map(|mut plan| {
            plan.greedy_failed = greedy_failed;
            plan.conditional = true;
            plan
        });
        let executable = match &decision {
            DecisionKind::GoalInteraction { action }
            | DecisionKind::PhysicalProbe { action, .. }
            | DecisionKind::ContactTransition { action, .. } => self
                .proofs
                .values()
                .find(|proof| {
                    proof.action.summary.action_key == action.action_key
                        && proof.action.summary.status == "PROVED"
                        && proof.action.summary.witness_digest.as_deref()
                            == Some(action.witness_digest.as_str())
                })
                .and_then(|proof| proof.action.step.clone()),
            _ => None,
        };
        ReasoningReport {
            candidates: summaries,
            decision,
            plan,
            executable,
            families: self.families().map(str::to_string).to_vec(),
            evidence_status: EVIDENCE_STATUS,
        }
    }

    pub fn authorize_step(
        &mut self,
        request: &ReasoningRequest,
        step: &PlanStep,
        authorization: ExecutionAuthorization,
    ) -> Result<AuthorizedAction, &'static str> {
        if self.aborted_grants.contains(&authorization.grant_id) {
            return Err("ABORTED_GRANT_CANNOT_RESUME");
        }
        let fresh = self.prove_action_key(request, &step.action_key);
        if fresh.summary.status != "PROVED" {
            return Err("NOT_PROVED");
        }
        let digest = fresh.summary.witness_digest.as_deref().unwrap_or("");
        let contents = fresh.summary.witness_contents.as_deref().unwrap_or("");
        if digest != step.witness_digest || contents != step.witness_contents {
            return Err("STALE_OR_UNPROVED_WITNESS");
        }
        if !initial_configuration_matches(request, contents) {
            return Err("INITIAL_CONFIGURATION_MISMATCH");
        }
        let frozen = frozen_from_step(&self.model, request, step);
        let frozen = frozen.bind_issued_authorization(authorization, request.now_s)?;
        Ok(AuthorizedAction { frozen })
    }

    fn note_geometric_fact(&mut self) {
        let key = geometric_key(&self.model);
        if self.geometric.contains_key(&key) {
            self.stats.geometric_hits = self.stats.geometric_hits.saturating_add(1);
        } else {
            self.stats.geometric_misses = self.stats.geometric_misses.saturating_add(1);
            self.geometric.insert(
                key,
                format!(
                    "joints={};version={PROOF_ALGORITHM}",
                    self.model.joints.len()
                ),
            );
        }
    }

    fn prove_action_key(&mut self, request: &ReasoningRequest, action_key: &str) -> ProvedAction {
        if let Some(candidate) = push_candidates(request)
            .into_iter()
            .find(|candidate| candidate.action_key == action_key)
        {
            return self.prove_cached(PUSH_FAMILY, request, &candidate, true);
        }
        if let Some(candidate) = grasp_candidates(request)
            .into_iter()
            .find(|candidate| candidate.action_key == action_key)
        {
            return self.prove_cached(GRASP_FAMILY, request, &candidate, false);
        }
        rejected(
            "unknown",
            &RawCandidate {
                action_key: action_key.into(),
                direction: [0.0, 0.0, 0.0],
                stroke_m: 0.0,
                target_xyz: [0.0, 0.0, 0.0],
                probe: false,
            },
            "UNKNOWN_ACTION",
            MechanicsStatus::Unknown,
        )
    }

    fn actions_at(&mut self, request: &ReasoningRequest) -> Vec<ProvedAction> {
        let mut actions = Vec::new();
        for candidate in push_candidates(request) {
            actions.push(self.prove_cached(PUSH_FAMILY, request, &candidate, true));
        }
        for candidate in grasp_candidates(request) {
            actions.push(self.prove_cached(GRASP_FAMILY, request, &candidate, false));
        }
        actions
    }

    fn prove_cached(
        &mut self,
        family: &str,
        request: &ReasoningRequest,
        candidate: &RawCandidate,
        is_push: bool,
    ) -> ProvedAction {
        let key = proof_dependency_digest(family, &self.model, request, candidate);
        if let Some(hit) = self.proofs.get(&key) {
            self.stats.executable_hits = self.stats.executable_hits.saturating_add(1);
            self.stats.ik_avoided = self.stats.ik_avoided.saturating_add(hit.ik);
            self.stats.fk_avoided = self.stats.fk_avoided.saturating_add(hit.fk);
            self.stats.collision_avoided =
                self.stats.collision_avoided.saturating_add(hit.collision);
            self.stats.proof_latency_avoided_ns = self
                .stats
                .proof_latency_avoided_ns
                .saturating_add(hit.latency_ns);
            return hit.action.clone();
        }
        self.stats.executable_misses = self.stats.executable_misses.saturating_add(1);
        let before = work_counters::work_snapshot();
        let started = std::time::Instant::now();
        let proved = if is_push {
            prove_push(&self.model, request, candidate)
        } else {
            prove_grasp(&self.model, request, candidate)
        };
        let latency_ns = u64::try_from(started.elapsed().as_nanos()).unwrap_or(u64::MAX);
        self.stats.last_proof_latency_ns = latency_ns;
        let after = work_counters::work_snapshot();
        self.proofs.insert(
            key,
            CachedProof {
                action: proved.clone(),
                ik: after.ik_attempts.saturating_sub(before.ik_attempts),
                fk: after.fk_evaluations.saturating_sub(before.fk_evaluations),
                collision: after
                    .collision_queries
                    .saturating_sub(before.collision_queries),
                latency_ns,
            },
        );
        proved
    }
}

pub fn integrate_support_friction(
    belief: &PhysicalParameterBelief,
    displacement_over_stroke: f64,
    mu_stick_min: f64,
    observation: &str,
) -> ExperienceReceipt {
    let before = belief.clone();
    let mut after = belief.clone();
    let mut gain = false;
    if let Some([lo, hi]) = friction_interval(&after, PhysicalParameter::SupportFriction) {
        let narrowed = if (0.7..=1.35).contains(&displacement_over_stroke) {
            Some([lo.max(mu_stick_min), hi])
        } else if displacement_over_stroke > 1.75 {
            Some([lo, hi.min(mu_stick_min - 1e-3)])
        } else {
            None
        };
        if let Some(next) = narrowed {
            if next[0] <= next[1] && (next[0] - lo).abs() + (next[1] - hi).abs() > 1e-12 {
                after.narrow_interval(PhysicalParameter::SupportFriction, next, observation);
                gain = true;
            }
        }
    }
    let declared_before = before.declared_value(PhysicalParameter::SupportFriction);
    let declared_after = after.declared_value(PhysicalParameter::SupportFriction);
    debug_assert_eq!(declared_before, declared_after);
    ExperienceReceipt {
        belief_before: before,
        belief_after: after,
        observation: observation.to_string(),
        information_gain: gain,
        evidence_status: EVIDENCE_STATUS,
        parameter: PhysicalParameter::SupportFriction,
    }
}

pub fn development_benchmark() -> BenchmarkReport {
    let (model, mut request) = benchmark_scene();
    let mut reasoner = PhysicalReasoner::new(model);
    work_counters::reset_work_counters();
    let cold = reasoner.consider(&request);
    let cold_work = work_counters::work_snapshot();
    work_counters::reset_work_counters();
    let _warm = reasoner.consider(&request);
    let warm_work = work_counters::work_snapshot();
    let probe = cold
        .candidates
        .iter()
        .find(|candidate| candidate.action_key.starts_with("probe:"))
        .expect("probe candidate");
    let binding = fixture_binding(probe, &request);
    request.grants = vec![binding];
    let probed = reasoner.consider(&request);
    let solved_plan = probed.plan.is_some();
    let receipt =
        integrate_support_friction(&request.belief, 1.0, request.mu_required, "benchmark-stick");
    request.belief = receipt.belief_after;
    request.grants.clear();
    let after = reasoner.consider(&request);
    let solved = after.plan.as_ref().is_some_and(|plan| {
        plan.steps.len() >= 2 && plan.greedy_failed && plan.steps[1].immediate_progress
    });
    let mut refused_request = request.clone();
    refused_request.objective = PhysicalObjective::AcquireObject {
        object_id: refused_request.scene.object_id.clone(),
    };
    refused_request.mass_kg = None;
    refused_request.belief = PhysicalParameterBelief { parameters: vec![] };
    let refused = reasoner.consider(&refused_request);
    let correctly_refused = !matches!(
        refused.decision,
        DecisionKind::GoalInteraction { .. } | DecisionKind::ContactTransition { .. }
    ) && refused.plan.is_none();
    let _ = (solved_plan, probed);
    BenchmarkReport {
        schema: BENCHMARK_SCHEMA,
        evidence_status: EVIDENCE_STATUS,
        cold_ik_attempts: cold_work.ik_attempts,
        warm_ik_attempts: warm_work.ik_attempts,
        cold_fk_evaluations: cold_work.fk_evaluations,
        warm_fk_evaluations: warm_work.fk_evaluations,
        cold_collision_queries: cold_work.collision_queries,
        warm_collision_queries: warm_work.collision_queries,
        cold_jacobian_evaluations: cold_work.jacobian_evaluations,
        warm_jacobian_evaluations: warm_work.jacobian_evaluations,
        cold_mechanics_evaluations: cold_work.mechanics_evaluations,
        warm_mechanics_evaluations: warm_work.mechanics_evaluations,
        tasks_attempted: 2,
        tasks_solved: u32::from(solved),
        tasks_correctly_refused: u32::from(correctly_refused),
        greedy_failed_on_solved_task: solved,
        parallel_evaluation: false,
        cache_hits: reasoner.cache_stats().executable_hits,
        cache_misses: reasoner.cache_stats().executable_misses,
        ik_avoided: reasoner.cache_stats().ik_avoided,
        collision_avoided: reasoner.cache_stats().collision_avoided,
        notes: vec![
            "population: one blocked planar translation and one acquire task with missing mass".into(),
            "warm counts are a second identical consider() served from the proof cache".into(),
            "candidates are proved sequentially so authority selection stays ordered; the cache removes repeated IK".into(),
            "jacobian counts are those performed inside IK, not an extra linearization".into(),
            EVIDENCE_STATUS.into(),
        ],
    }
}

pub fn benchmark_scene() -> (EmbodimentModel, ReasoningRequest) {
    let mut model = synth_planar_two_link();
    model.model_hash = "hash-table".into();
    model.calibration_epoch = "epoch:table".into();
    for joint in &mut model.joints {
        joint.q_min = Provenanced::declared(-3.0, "bench.limits", 0.0);
        joint.q_max = Provenanced::declared(3.0, "bench.limits", 0.0);
    }
    add_bench_fingers(&mut model);
    let chain = model.ee_joint_chain("ee").expect("planar chain");
    let (home, _) = with_ik_q_seed(Some(&[0.4, -0.6]), || {
        solve_ik(&model, &chain, "ee", [0.146, 0.0, 0.0], &[0.4, -0.6])
    })
    .expect("home pose");
    let mut belief = PhysicalParameterBelief::declared_point(
        PhysicalParameter::SupportFriction,
        0.5,
        "scene.prior",
    );
    belief.narrow_interval(
        PhysicalParameter::SupportFriction,
        [0.05, 0.8],
        "prior_domain",
    );
    belief = belief.with_unknown(PhysicalParameter::ToolObjectFriction, "finger.mu");
    let request = ReasoningRequest {
        scene: TableScene {
            object_id: "block".into(),
            object_center: [0.20, 0.0, 0.0],
            object_half: [0.02, 0.02, 0.02],
            object_quat: [1.0, 0.0, 0.0, 0.0],
            support_origin: [0.20, 0.0, -0.05],
            support_normal: [0.0, 0.0, 1.0],
            obstacles: vec![NamedBox::aabb(
                "south-wall",
                [0.20, -0.030, 0.0],
                [0.012, 0.008, 0.02],
            )],
            arm_q: home,
            joint_names: chain,
            ee: "ee".into(),
            tool_radius_m: 0.004,
            observation_epoch: "epoch:table".into(),
            held: false,
            finger_names: vec!["finger_l".into(), "finger_r".into()],
            finger_q: vec![0.02, 0.02],
        },
        belief,
        objective: PhysicalObjective::PlanarTranslation {
            target_xy: [0.20, 0.10],
        },
        mu_required: 0.2,
        goal_stroke_m: 0.04,
        probe_stroke_m: 0.008,
        grants: Vec::new(),
        now_s: 10.0,
        evidence_fresh: true,
        mass_kg: Some(0.05),
        gripper_force_n: Some(2.0),
        gravity_m_s2: [0.0, 0.0, -9.81],
    };
    (model, request)
}

fn add_bench_fingers(model: &mut EmbodimentModel) {
    for (name, axis) in [
        ("finger_l", [0.0, 1.0, 0.0]),
        ("finger_r", [0.0, -1.0, 0.0]),
    ] {
        model.joints.push(crate::embodiment::Joint {
            name: name.into(),
            kind: crate::embodiment::JointKind::Slide,
            axis: Provenanced::declared(axis, "bench.finger", 0.0),
            qpos_dim: 1,
            dof_dim: 1,
            parent_body: "link2".into(),
            child_body: name.into(),
            q_min: Provenanced::declared(0.0, "bench.finger", 0.0),
            q_max: Provenanced::declared(0.04, "bench.finger", 0.0),
            dq_max: Provenanced::unknown("bench.finger", 0.0),
            effort_max: Provenanced::declared(2.0, "bench.finger", 0.0),
            origin_in_child: Provenanced::declared([0.0, 0.0, 0.0], "bench.finger", 0.0),
            parent_to_joint: crate::embodiment::unknown_se3("bench.finger"),
            joint_to_child: crate::embodiment::unknown_se3("bench.finger"),
            qpos_adr: None,
            dof_adr: None,
        });
        model.actuators.push(crate::embodiment::Actuator {
            name: format!("act-{name}"),
            target_joint: name.into(),
            control_mode: "position".into(),
            transmission_kind: "joint".into(),
            ctrlrange: Provenanced::unknown("bench.finger", 0.0),
            forcerange: Provenanced::unknown("bench.finger", 0.0),
            gear: Provenanced::unknown("bench.finger", 0.0),
        });
    }
}

struct RawCandidate {
    action_key: String,
    direction: [f64; 3],
    stroke_m: f64,
    target_xyz: [f64; 3],
    probe: bool,
}

fn push_candidates(request: &ReasoningRequest) -> Vec<RawCandidate> {
    let directions = [
        [1.0, 0.0, 0.0],
        [-1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, -1.0, 0.0],
    ];
    let mut out = Vec::new();
    for direction in directions {
        let label = axis_label(direction);
        out.push(push_raw(
            request,
            &format!("push:{label}"),
            direction,
            request.goal_stroke_m,
            false,
        ));
    }
    if friction_straddles(
        &request.belief,
        PhysicalParameter::SupportFriction,
        request.mu_required,
    ) {
        out.push(push_raw(
            request,
            "probe:+x",
            [1.0, 0.0, 0.0],
            request.probe_stroke_m,
            true,
        ));
    }
    out
}

fn push_raw(
    request: &ReasoningRequest,
    key: &str,
    direction: [f64; 3],
    stroke: f64,
    probe: bool,
) -> RawCandidate {
    let extent = if direction[0].abs() > 0.5 {
        request.scene.object_half[0]
    } else {
        request.scene.object_half[1]
    };
    let target = sub3(
        request.scene.object_center,
        scale3(direction, extent + request.scene.tool_radius_m),
    );
    RawCandidate {
        action_key: key.into(),
        direction,
        stroke_m: stroke,
        target_xyz: target,
        probe,
    }
}

fn grasp_candidates(request: &ReasoningRequest) -> Vec<RawCandidate> {
    let direction = [0.0, -1.0, 0.0];
    let target = add3(
        request.scene.object_center,
        scale3(
            [0.0, 1.0, 0.0],
            request.scene.object_half[1] + request.scene.tool_radius_m,
        ),
    );
    vec![RawCandidate {
        action_key: "grasp:y".into(),
        direction,
        stroke_m: 0.0,
        target_xyz: target,
        probe: false,
    }]
}

fn prove_push(
    model: &EmbodimentModel,
    request: &ReasoningRequest,
    candidate: &RawCandidate,
) -> ProvedAction {
    work_counters::note_mechanics_evaluation();
    if cheap_obstacle_hit(request, candidate.target_xyz) {
        return rejected(
            PUSH_FAMILY,
            candidate,
            "COLLISION_NECESSARY_CONDITION",
            MechanicsStatus::Unknown,
        );
    }
    let mechanics = mechanics_from_interval(
        friction_interval(&request.belief, PhysicalParameter::SupportFriction),
        request.mu_required,
    );
    let solved = match prove_arm_path(model, request, candidate, false) {
        Ok(solved) => solved,
        Err(reason) => return rejected(PUSH_FAMILY, candidate, reason, mechanics),
    };
    let end = add3(
        request.scene.object_center,
        scale3(candidate.direction, candidate.stroke_m),
    );
    if aabb_hits_obstacle(end, request.scene.object_half, &request.scene.obstacles) {
        return rejected(PUSH_FAMILY, candidate, "EFFECT_HITS_OBSTACLE", mechanics);
    }
    let (before, after) = translation_errors(request, end);
    let derivative = after - before;
    let immediate = !candidate.probe && derivative < -1e-4;
    let progress_for_objective = matches!(
        request.objective,
        PhysicalObjective::PlanarTranslation { .. }
    ) && immediate;
    if candidate.probe {
        return finish_probe(model, request, candidate, &solved, mechanics, before);
    }
    if mechanics != MechanicsStatus::Feasible {
        return rejected(PUSH_FAMILY, candidate, "MECHANICS_UNKNOWN", mechanics);
    }
    if !progress_for_objective
        && !matches!(
            request.objective,
            PhysicalObjective::PlanarTranslation { .. }
        )
    {
        return rejected(PUSH_FAMILY, candidate, "NOT_AN_ACQUIRE_EFFECT", mechanics);
    }
    let recoverable = !aabb_hits_obstacle(end, request.scene.object_half, &request.scene.obstacles);
    finish_action(
        model,
        request,
        candidate,
        PUSH_FAMILY,
        &solved,
        mechanics,
        progress_for_objective,
        Some(derivative),
        end,
        false,
        if progress_for_objective && recoverable {
            RecoverabilityClass::ProgressAndRecoverable
        } else if progress_for_objective {
            RecoverabilityClass::ProgressButCanEnterUnrecoverableState
        } else {
            RecoverabilityClass::NoProgress
        },
        CandidateRole::GoalAction,
        before,
    )
}

fn finish_probe(
    model: &EmbodimentModel,
    request: &ReasoningRequest,
    candidate: &RawCandidate,
    solved: &SolvedPose,
    mechanics: MechanicsStatus,
    error_before: f64,
) -> ProvedAction {
    let margin = joint_margin(model, &request.scene.joint_names, &solved.q_contact);
    let gap = sweep_gap(
        request.scene.object_center,
        request.scene.object_half,
        candidate.direction,
        candidate.stroke_m,
        &request.scene.obstacles,
    );
    let Some(gap) = gap else {
        return rejected(PUSH_FAMILY, candidate, "PROBE_SWEEP_BLOCKED", mechanics);
    };
    if margin <= 0.0 {
        return rejected(
            PUSH_FAMILY,
            candidate,
            "JOINT_MARGIN_UNAVAILABLE",
            mechanics,
        );
    }
    let contents = witness_text(PUSH_FAMILY, model, request, candidate, solved);
    let digest = contents_digest(&contents);
    let coverage = CommonWitnessCoverage {
        label: candidate.action_key.clone(),
        witness_digest: digest.clone(),
        contact_point: candidate.target_xyz,
        object_center: request.scene.object_center,
        object_quat: request.scene.object_quat,
        object_half: request.scene.object_half,
        push_direction: candidate.direction,
        support_normal: request.scene.support_normal,
        face_gap_m: request.scene.tool_radius_m,
        translation_radius_m: candidate.stroke_m,
        yaw_abs_rad: 0.0,
        geometry_residual_m: 0.0,
        support_clearance_m: 0.02,
        min_support_clearance_m: 0.004,
        joint_margin_rad: margin,
        collision_admissible_for_grown_object: false,
        model_applicable: true,
        model_applicability: "axis-aligned tool-driven sweep of at most the probe stroke".into(),
        observation_epoch: request.scene.observation_epoch.clone(),
    };
    let Ok(domain) = domain_from_exact_axis_sweep(&coverage, &request.scene.obstacles, gap) else {
        return rejected(PUSH_FAMILY, candidate, "PROBE_DOMAIN_UNPROVED", mechanics);
    };
    let outcome = SupportedProbeOutcome {
        label: candidate.action_key.clone(),
        object_supported: Some(true),
        inside_reachable_workspace: Some(true),
        joint_margin_rad: Some(margin),
        collision_admissible: Some(true),
        motion_within_declared_bound: Some(true),
        belief_outcome_bounded: Some(true),
        contact_persists: Some(false),
        return_contact_witness_digest: Some(digest.clone()),
    };
    let future = assess_probe_future(&ProbeFutureInputs {
        consumed_stroke_m: 0.0,
        observed_displacement_m: Some(0.0),
        geometry_residual_m: Some(0.0),
        outcomes: vec![outcome],
        domain: Some(domain),
    });
    if future.assessment != ProbeRecoverabilityAssessment::Preserved {
        return rejected(PUSH_FAMILY, candidate, "PROBE_FUTURE_UNPROVED", mechanics);
    }
    let mut action = finish_action(
        model,
        request,
        candidate,
        PUSH_FAMILY,
        solved,
        mechanics,
        false,
        None,
        request.scene.object_center,
        false,
        RecoverabilityClass::NoProgress,
        PhysicalProbe,
        error_before,
    );
    if let Some(evidence) = action.evidence.as_mut() {
        evidence.role = PhysicalProbe;
        evidence.strict_goal_progress = false;
        evidence.robustness = BeliefRobustness::Ambiguous;
        evidence.probe.future_interaction = ProbeRecoverabilityAssessment::Preserved;
        evidence.probe.decision_relevant_distinctions = 1;
        evidence.probe.observable_distinctions = 1;
        evidence.preference.stroke_m = candidate.stroke_m;
    }
    action.summary.immediate_progress = false;
    action
}

fn prove_grasp(
    model: &EmbodimentModel,
    request: &ReasoningRequest,
    candidate: &RawCandidate,
) -> ProvedAction {
    work_counters::note_mechanics_evaluation();
    if cheap_obstacle_hit(request, candidate.target_xyz) {
        return rejected(
            GRASP_FAMILY,
            candidate,
            "COLLISION_NECESSARY_CONDITION",
            MechanicsStatus::Unknown,
        );
    }
    let mechanics = grasp_mechanics(request);
    let solved = match prove_arm_path(model, request, candidate, true) {
        Ok(solved) => solved,
        Err(reason) => return rejected(GRASP_FAMILY, candidate, reason, mechanics),
    };
    let acquire = matches!(request.objective, PhysicalObjective::AcquireObject { .. });
    if mechanics != MechanicsStatus::Feasible {
        return rejected(GRASP_FAMILY, candidate, "HOLD_MECHANICS_UNKNOWN", mechanics);
    }
    if !acquire {
        return rejected(
            GRASP_FAMILY,
            candidate,
            "GRASP_DOES_NOT_REDUCE_TRANSLATION_ERROR",
            mechanics,
        );
    }
    if request.scene.held {
        return rejected(GRASP_FAMILY, candidate, "ALREADY_HELD", mechanics);
    }
    finish_action(
        model,
        request,
        candidate,
        GRASP_FAMILY,
        &solved,
        mechanics,
        true,
        Some(-1.0),
        request.scene.object_center,
        true,
        RecoverabilityClass::ProgressAndRecoverable,
        GoalAction,
        1.0,
    )
}

fn grasp_mechanics(request: &ReasoningRequest) -> MechanicsStatus {
    let (Some(mass), Some(force)) = (request.mass_kg, request.gripper_force_n) else {
        return MechanicsStatus::Unknown;
    };
    if !(mass > 0.0 && force > 0.0) {
        return MechanicsStatus::Unknown;
    }
    let Some(interval) = friction_interval(&request.belief, PhysicalParameter::ToolObjectFriction)
    else {
        return MechanicsStatus::Unknown;
    };
    let weight = mass * norm3(request.gravity_m_s2);
    let required_mu = (weight / 2.0) / force;
    if interval[1] < required_mu {
        return MechanicsStatus::Infeasible;
    }
    if interval[0] < required_mu {
        return MechanicsStatus::Unknown;
    }
    let mu = Provenanced::assumed(interval[0], "conservative_interval_lower_bound", 0.0);
    let witness = evaluate_pinch_hold(&PinchHoldInput {
        mass_kg: Provenanced::declared(mass, "request.mass", 0.0),
        gravity_m_s2: Provenanced::declared(request.gravity_m_s2, "request.gravity", 0.0),
        finger_a_inward_normal: [0.0, 1.0, 0.0],
        finger_b_inward_normal: [0.0, -1.0, 0.0],
        finger_object_friction: PairFriction::coulomb("finger", &request.scene.object_id, mu),
        gripper_force_bound_n: Provenanced::declared(force, "request.gripper_force", 0.0),
    });
    match witness.feasibility {
        HoldFeasibility::Feasible => MechanicsStatus::Feasible,
        HoldFeasibility::Infeasible => MechanicsStatus::Infeasible,
        HoldFeasibility::Unknown => MechanicsStatus::Unknown,
    }
}

struct SolvedPose {
    q0: Vec<f64>,
    q_transit: Vec<f64>,
    q_contact: Vec<f64>,
    q_end: Vec<f64>,
    finger_q: Vec<f64>,
}

fn prove_arm_path(
    model: &EmbodimentModel,
    request: &ReasoningRequest,
    candidate: &RawCandidate,
    grasp: bool,
) -> Result<SolvedPose, &'static str> {
    let standoff_xyz = if grasp {
        add3(candidate.target_xyz, scale3([0.0, 1.0, 0.0], 0.02))
    } else {
        sub3(candidate.target_xyz, scale3(candidate.direction, 0.02))
    };
    let q_transit = solve_precise(model, &request.scene, standoff_xyz)?;
    let transit_phase = ContactPhase::CurrentToApproach;
    arm_motion_clear(
        model,
        request,
        &request.scene.arm_q,
        &q_transit,
        transit_phase,
    )?;
    let mut contact_scene = request.scene.clone();
    contact_scene.arm_q = q_transit.clone();
    let q_contact = solve_precise(model, &contact_scene, candidate.target_xyz)?;
    let approach_phase = if grasp {
        ContactPhase::GraspApproach
    } else {
        ContactPhase::ApproachToContact
    };
    arm_motion_clear(model, request, &q_transit, &q_contact, approach_phase)?;
    let mut q_end = q_contact.clone();
    if !grasp && candidate.stroke_m > 1e-9 {
        let end_xyz = add3(
            candidate.target_xyz,
            scale3(candidate.direction, candidate.stroke_m),
        );
        let mut stroke_scene = contact_scene;
        stroke_scene.arm_q = q_contact.clone();
        q_end = solve_precise(model, &stroke_scene, end_xyz)?;
        arm_motion_clear(
            model,
            request,
            &q_contact,
            &q_end,
            ContactPhase::ContactStroke,
        )?;
    }
    let finger_q = if grasp {
        close_fingers(model, request)?
    } else {
        request.scene.finger_q.clone()
    };
    Ok(SolvedPose {
        q0: request.scene.arm_q.clone(),
        q_transit,
        q_contact,
        q_end,
        finger_q,
    })
}

fn close_fingers(
    model: &EmbodimentModel,
    request: &ReasoningRequest,
) -> Result<Vec<f64>, &'static str> {
    if request.scene.finger_names.len() < 2
        || request.scene.finger_q.len() != request.scene.finger_names.len()
    {
        return Err("FINGERS_UNDECLARED");
    }
    let mut closed = Vec::new();
    for (name, open) in request
        .scene
        .finger_names
        .iter()
        .zip(request.scene.finger_q.iter())
    {
        let Some(joint) = model.joints.iter().find(|joint| joint.name == *name) else {
            return Err("FINGERS_UNDECLARED");
        };
        if joint.kind != crate::embodiment::JointKind::Slide {
            return Err("FINGER_IS_NOT_A_SLIDE");
        }
        let (Some(lo), Some(hi)) = (joint.q_min.value, joint.q_max.value) else {
            return Err("FINGER_LIMITS_UNKNOWN");
        };
        // Positive slide ranges in the supported grippers open outward. The
        // closed stop is the lower limit. A command already at that stop has
        // no close travel to prove.
        let commanded = lo;
        if !commanded.is_finite() || commanded < lo - 1e-9 || commanded > hi + 1e-9 {
            return Err("FINGER_CLOSE_OUTSIDE_LIMITS");
        }
        if (commanded - open).abs() <= 1e-6 {
            return Err("FINGER_CLOSE_HAS_NO_TRAVEL");
        }
        closed.push(commanded);
    }
    Ok(closed)
}

fn finish_action(
    model: &EmbodimentModel,
    request: &ReasoningRequest,
    candidate: &RawCandidate,
    family: &str,
    solved: &SolvedPose,
    mechanics: MechanicsStatus,
    immediate: bool,
    derivative: Option<f64>,
    successor_center: [f64; 3],
    successor_held: bool,
    recoverability: RecoverabilityClass,
    role: CandidateRole,
    error_before: f64,
) -> ProvedAction {
    let _ = error_before;
    let contents = witness_text(family, model, request, candidate, solved);
    let digest = contents_digest(&contents);
    let robust = if immediate && mechanics == MechanicsStatus::Feasible {
        BeliefRobustness::RobustStrictProgress
    } else {
        BeliefRobustness::Ambiguous
    };
    let evidence = CandidateEvidence {
        candidate_id: candidate.action_key.clone(),
        candidate_contents: contents.clone(),
        action_key: candidate.action_key.clone(),
        contact_id: format!("{family}:{}", candidate.action_key),
        role,
        strict_goal_progress: immediate,
        authority_ok: true,
        scoped_grant: None,
        executable_witness_id: Some(format!("witness:{}", candidate.action_key)),
        witness_digest: Some(digest.clone()),
        witness_contents: Some(contents.clone()),
        robustness: robust,
        recoverability,
        predicted_effect: PredictedPhysicalEffect {
            object_translation_world_m: Some([
                successor_center[0] - request.scene.object_center[0],
                successor_center[1] - request.scene.object_center[1],
            ]),
            yaw_change_rad: Some(0.0),
            contact_persists: Some(family == GRASP_FAMILY || !candidate.probe),
            goal_error_derivative: derivative,
            goal_progress: derivative.map(|value| {
                if value < -1e-4 {
                    GoalProgressClass::StrictProgress
                } else if value > 1e-4 {
                    GoalProgressClass::Regression
                } else {
                    GoalProgressClass::Neutral
                }
            }),
        },
        probe: ProbeEvidence {
            decision_relevant_distinctions: 0,
            observable_distinctions: 0,
            future_interaction: ProbeRecoverabilityAssessment::Unknown,
        },
        preference: LexicographicPreference {
            error_derivative: derivative,
            angular_rate_abs: Some(0.0),
            contact_offset_abs_m: Some(0.0),
            stroke_m: candidate.stroke_m,
        },
        hard_rejections: Vec::new(),
    };
    let translation = norm3(sub3(successor_center, request.scene.object_center));
    let step = if mechanics == MechanicsStatus::Feasible
        && (immediate
            || matches!(
                request.objective,
                PhysicalObjective::PlanarTranslation { .. }
            )) {
        Some(PlanStep {
            family: family.into(),
            action_key: candidate.action_key.clone(),
            candidate_id: candidate.action_key.clone(),
            witness_digest: digest.clone(),
            witness_contents: contents.clone(),
            stroke_m: candidate.stroke_m,
            immediate_progress: immediate,
            predicted_translation_m: translation,
            error_after: translation_errors(request, successor_center).1,
        })
    } else {
        None
    };
    ProvedAction {
        summary: CandidateSummary {
            family: family.into(),
            action_key: candidate.action_key.clone(),
            status: "PROVED".into(),
            mechanics,
            immediate_progress: immediate,
            error_derivative: derivative,
            witness_digest: Some(digest),
            witness_contents: Some(contents),
        },
        evidence: Some(evidence),
        step,
        successor_center,
        successor_held,
    }
}

fn rejected(
    family: &str,
    candidate: &RawCandidate,
    reason: &str,
    mechanics: MechanicsStatus,
) -> ProvedAction {
    ProvedAction {
        summary: CandidateSummary {
            family: family.into(),
            action_key: candidate.action_key.clone(),
            status: reason.into(),
            mechanics,
            immediate_progress: false,
            error_derivative: None,
            witness_digest: None,
            witness_contents: None,
        },
        evidence: None,
        step: None,
        successor_center: [0.0; 3],
        successor_held: false,
    }
}

fn decision_context(request: &ReasoningRequest, actions: &[ProvedAction]) -> DecisionContext {
    let mut candidates = Vec::new();
    for action in actions {
        let Some(mut evidence) = action.evidence.clone() else {
            continue;
        };
        let keep = match evidence.role {
            GoalAction => {
                action.summary.immediate_progress
                    && action.summary.mechanics == MechanicsStatus::Feasible
            }
            PhysicalProbe => evidence.probe.decision_relevant_distinctions > 0,
        };
        if keep {
            if evidence.role == PhysicalProbe {
                attach_grant(request, &mut evidence);
            }
            candidates.push(evidence);
        }
    }
    DecisionContext {
        goal_id: match &request.objective {
            PhysicalObjective::PlanarTranslation { target_xy } => {
                format!("translate:{:.3},{:.3}", target_xy[0], target_xy[1])
            }
            PhysicalObjective::AcquireObject { object_id } => format!("acquire:{object_id}"),
        },
        goal_reached: false,
        evidence_fresh: request.evidence_fresh,
        now_s: request.now_s,
        observation_epoch: request.scene.observation_epoch.clone(),
        remaining_attempts: 4,
        current_contact_id: None,
        forbidden_action_keys: Vec::new(),
        candidates,
    }
}

fn search_plan(
    reasoner: &mut PhysicalReasoner,
    request: &ReasoningRequest,
) -> Option<PhysicalPlan> {
    let root_error = translation_errors(request, request.scene.object_center).0;
    let mut budget = 48u32;
    for depth in 1..=3 {
        if budget == 0 {
            break;
        }
        if let Some(plan) = bounded_plan(reasoner, request, depth, root_error, &mut budget) {
            let improved = plan
                .steps
                .last()
                .is_some_and(|step| step.error_after + 1e-4 < root_error);
            if improved {
                return Some(plan);
            }
        }
    }
    None
}

fn bounded_plan(
    reasoner: &mut PhysicalReasoner,
    request: &ReasoningRequest,
    depth: u32,
    root_error: f64,
    budget: &mut u32,
) -> Option<PhysicalPlan> {
    if *budget == 0 {
        return None;
    }
    *budget = budget.saturating_sub(1);
    let actions = reasoner.actions_at(request);
    let goal = actions
        .iter()
        .filter(|action| {
            action.summary.immediate_progress
                && action.summary.mechanics == MechanicsStatus::Feasible
                && action
                    .step
                    .as_ref()
                    .is_some_and(|step| step.error_after + 1e-4 < root_error)
        })
        .min_by(|left, right| {
            left.summary
                .error_derivative
                .unwrap_or(0.0)
                .total_cmp(&right.summary.error_derivative.unwrap_or(0.0))
                .then_with(|| left.summary.action_key.cmp(&right.summary.action_key))
        });
    if let Some(goal) = goal {
        return goal.step.clone().map(|step| PhysicalPlan {
            steps: vec![step],
            greedy_failed: false,
            conditional: true,
            expansions: 0,
        });
    }
    if depth == 0 {
        return None;
    }
    let mut intermediates: Vec<ProvedAction> = actions
        .into_iter()
        .filter(|action| {
            action.summary.mechanics == MechanicsStatus::Feasible
                && !action.summary.immediate_progress
                && action
                    .step
                    .as_ref()
                    .is_some_and(|step| step.predicted_translation_m > 1e-4)
        })
        .collect();
    intermediates.sort_by(|a, b| a.summary.action_key.cmp(&b.summary.action_key));
    for action in intermediates {
        let mut next = request.clone();
        next.scene.object_center = action.successor_center;
        next.scene.held = action.successor_held;
        next.scene.arm_q =
            q_from_contents(action.summary.witness_contents.as_deref().unwrap_or(""));
        if let Some(mut rest) = bounded_plan(reasoner, &next, depth - 1, root_error, budget) {
            if let Some(step) = action.step.clone() {
                rest.steps.insert(0, step);
                rest.expansions = rest.expansions.saturating_add(1);
                rest.conditional = true;
                return Some(rest);
            }
        }
    }
    None
}

fn frozen_from_step(
    model: &EmbodimentModel,
    request: &ReasoningRequest,
    step: &PlanStep,
) -> FrozenAction {
    let prediction = FrozenPrediction {
        action_id: format!("action:{}", step.candidate_id),
        witness_id: format!("witness:{}", step.action_key),
        stroke_m: step.stroke_m,
        predicted_displacement_m: Some(step.predicted_translation_m),
        predicted_yaw_change_rad: Some(0.0),
        predicted_contact_persists: step.family == GRASP_FAMILY,
        quasi_static_stroke_limit_m: step.stroke_m.max(request.goal_stroke_m),
    };
    FrozenAction {
        action_id: prediction.action_id.clone(),
        action_key: step.action_key.clone(),
        candidate_id: step.candidate_id.clone(),
        contact_id: format!("{}:{}", step.family, step.action_key),
        witness_id: prediction.witness_id.clone(),
        witness_contents: step.witness_contents.clone(),
        witness_digest: step.witness_digest.clone(),
        requested_stroke_m: step.stroke_m,
        prediction,
        belief_snapshot: request.belief.clone(),
        recoverability: if step.immediate_progress {
            RecoverabilityClass::ProgressAndRecoverable
        } else {
            RecoverabilityClass::NoProgress
        },
        envelope: ExecutionEnvelope::for_quasi_static_stroke(
            step.stroke_m.max(1e-4),
            step.predicted_translation_m,
        ),
        observation_contract: ObservationContract {
            source: "policy-observation".into(),
            model_epoch: model.calibration_epoch.clone(),
            calibration_epoch: model.calibration_epoch.clone(),
            max_age_s: 1.0,
            required_units: BTreeMap::new(),
            required_fields: vec![
                ObservationField::StrokeConsumed,
                ObservationField::Displacement,
                ObservationField::Yaw,
                ObservationField::Contact,
                ObservationField::GoalError,
                ObservationField::Tracking,
                ObservationField::Reachability,
                ObservationField::QuasiStaticApplicability,
            ],
        },
        model_id: model.model_hash.clone(),
        embodiment_id: model.robot_id.clone(),
        observation_epoch: request.scene.observation_epoch.clone(),
        actuator_id: "tool".into(),
        observation_contract_id: format!("sensors:{}", request.scene.observation_epoch),
        abort_contract_id: "abort-and-reobserve".into(),
        authority_granted: false,
        execution_authorization: None,
    }
}

fn solve_precise(
    model: &EmbodimentModel,
    scene: &TableScene,
    target: [f64; 3],
) -> Result<Vec<f64>, &'static str> {
    let chain = model.ee_joint_chain(&scene.ee).ok_or("NO_CHAIN")?;
    if scene.arm_q.len() != chain.len() {
        return Err("Q_LENGTH");
    }
    let solved = with_ik_q_seed(Some(&scene.arm_q), || {
        solve_ik(model, &chain, &scene.ee, target, &scene.arm_q)
    });
    let (q, trace) = solved.map_err(|_| "IK_FAILED")?;
    let fk = forward_kinematics(model, &chain, &scene.ee, &q).map_err(|_| "FK_FAILED")?;
    let residual = trace.residual.max(norm3(sub3(fk.ee.xyz, target)));
    if !ik_residual_is_precise(residual) {
        return Err("IK_RESIDUAL");
    }
    Ok(q)
}

fn arm_motion_clear(
    model: &EmbodimentModel,
    request: &ReasoningRequest,
    qa: &[f64],
    qb: &[f64],
    phase: ContactPhase,
) -> Result<(), &'static str> {
    let world = collision_world(request);
    let scene = scene_from_collision_world(model, &request.scene.ee, &world)
        .map_err(|_| "COLLISION_WORLD")?;
    let policy = policy_from_scene_model(Some(model), &scene);
    let report = validate_transition(
        model,
        &request.scene.joint_names,
        qa,
        qb,
        &scene,
        &policy,
        phase,
    );
    if let Some(reason) = report.execution_refusal() {
        Err(reason)
    } else {
        Ok(())
    }
}

fn collision_world(request: &ReasoningRequest) -> CollisionWorld {
    CollisionWorld {
        object_id: request.scene.object_id.clone(),
        object_center: request.scene.object_center,
        object_half: request.scene.object_half,
        object_quat: request.scene.object_quat,
        support_id: "table".into(),
        support_origin: request.scene.support_origin,
        support_normal: request.scene.support_normal,
        intended_tool_bodies: vec!["tool".into(), "link2".into()],
        robot_volumes: Vec::new(),
        obstacles: request.scene.obstacles.clone(),
        ee_radius: request.scene.tool_radius_m,
        object_probe_radius: request.scene.tool_radius_m,
        tool_offset_ee: [0.0, 0.0, 0.0],
        declared_geoms: Vec::new(),
    }
}

fn cheap_obstacle_hit(request: &ReasoningRequest, point: [f64; 3]) -> bool {
    let radius = request.scene.tool_radius_m;
    request.scene.obstacles.iter().any(|obstacle| {
        let grown = [
            obstacle.half_extents[0] + radius,
            obstacle.half_extents[1] + radius,
            obstacle.half_extents[2] + radius,
        ];
        aabb_contains(obstacle.center, grown, point)
    })
}

fn aabb_contains(center: [f64; 3], half: [f64; 3], point: [f64; 3]) -> bool {
    (0..3).all(|axis| (point[axis] - center[axis]).abs() <= half[axis] + 1e-9)
}

fn aabb_hits_obstacle(center: [f64; 3], half: [f64; 3], obstacles: &[NamedBox]) -> bool {
    obstacles
        .iter()
        .any(|obstacle| !aabb_separated(center, half, obstacle.center, obstacle.half_extents))
}

fn aabb_separated(a_c: [f64; 3], a_h: [f64; 3], b_c: [f64; 3], b_h: [f64; 3]) -> bool {
    (0..3).any(|axis| (a_c[axis] - b_c[axis]).abs() > a_h[axis] + b_h[axis] + 1e-9)
}

fn sweep_gap(
    center: [f64; 3],
    half: [f64; 3],
    direction: [f64; 3],
    distance: f64,
    obstacles: &[NamedBox],
) -> Option<f64> {
    if direction.iter().filter(|value| value.abs() > 1e-6).count() != 1 {
        return None;
    }
    let mut swept_center = center;
    let mut swept_half = half;
    for axis in 0..3 {
        let delta = direction[axis] * distance;
        swept_center[axis] += 0.5 * delta;
        swept_half[axis] += 0.5 * delta.abs();
    }
    let mut gap = f64::INFINITY;
    for obstacle in obstacles {
        if !aabb_separated(
            swept_center,
            swept_half,
            obstacle.center,
            obstacle.half_extents,
        ) {
            return None;
        }
        for axis in 0..3 {
            let separation = (swept_center[axis] - obstacle.center[axis]).abs()
                - swept_half[axis]
                - obstacle.half_extents[axis];
            if separation >= 0.0 {
                gap = gap.min(separation);
            }
        }
    }
    if gap.is_finite() {
        Some(gap)
    } else {
        Some(distance.max(0.0))
    }
}

fn friction_interval(
    belief: &PhysicalParameterBelief,
    parameter: PhysicalParameter,
) -> Option<[f64; 2]> {
    let entry = belief.entry(parameter)?;
    if let Some(interval) = entry.empirical_interval {
        if interval[0].is_finite() && interval[1].is_finite() && interval[0] <= interval[1] {
            return Some(interval);
        }
    }
    if entry.status == BeliefEpistemicStatus::DeclaredFact {
        return entry.declared.value.map(|value| [value, value]);
    }
    None
}

fn friction_straddles(
    belief: &PhysicalParameterBelief,
    parameter: PhysicalParameter,
    required: f64,
) -> bool {
    friction_interval(belief, parameter)
        .is_some_and(|[lo, hi]| lo + 1e-12 < required && hi + 1e-12 >= required)
}

fn mechanics_from_interval(interval: Option<[f64; 2]>, required: f64) -> MechanicsStatus {
    match interval {
        Some([lo, _hi]) if lo + 1e-12 >= required => MechanicsStatus::Feasible,
        Some([_, hi]) if hi < required => MechanicsStatus::Infeasible,
        Some(_) => MechanicsStatus::Unknown,
        None => MechanicsStatus::Unknown,
    }
}

fn translation_errors(request: &ReasoningRequest, center: [f64; 3]) -> (f64, f64) {
    let target = match request.objective {
        PhysicalObjective::PlanarTranslation { target_xy } => target_xy,
        PhysicalObjective::AcquireObject { .. } => return (1.0, 1.0),
    };
    let before = ((request.scene.object_center[0] - target[0]).powi(2)
        + (request.scene.object_center[1] - target[1]).powi(2))
    .sqrt();
    let after = ((center[0] - target[0]).powi(2) + (center[1] - target[1]).powi(2)).sqrt();
    (before, after)
}

fn joint_margin(model: &EmbodimentModel, names: &[String], q: &[f64]) -> f64 {
    let mut margin = f64::INFINITY;
    for (name, value) in names.iter().zip(q.iter()) {
        let Some(joint) = model.joints.iter().find(|joint| joint.name == *name) else {
            return 0.0;
        };
        let (Some(lo), Some(hi)) = (joint.q_min.value, joint.q_max.value) else {
            return 0.0;
        };
        margin = margin.min((value - lo).min(hi - value));
    }
    if margin.is_finite() {
        margin
    } else {
        0.0
    }
}

const PROOF_ALGORITHM: &str = "physical-proof/3";

fn witness_text(
    family: &str,
    model: &EmbodimentModel,
    request: &ReasoningRequest,
    candidate: &RawCandidate,
    solved: &SolvedPose,
) -> String {
    let dependency = proof_dependency_digest(family, model, request, candidate);
    format!(
        "algorithm={PROOF_ALGORITHM}\nfamily={family}\nkey={}\npush_bits={}\nstroke_bits={}\nq0_bits={}\nq_transit_bits={}\nq_contact_bits={}\nq_end_bits={}\nfinger_bits={}\ndependency={dependency}\nsegments=transit,approach,contact,interaction\ncontact_observed=false\n",
        candidate.action_key,
        bits3(candidate.direction),
        candidate.stroke_m.to_bits(),
        bits_q(&solved.q0),
        bits_q(&solved.q_transit),
        bits_q(&solved.q_contact),
        bits_q(&solved.q_end),
        bits_q(&solved.finger_q),
    )
}

fn bits3(value: [f64; 3]) -> String {
    format!(
        "{},{},{}",
        value[0].to_bits(),
        value[1].to_bits(),
        value[2].to_bits()
    )
}

fn bits_q(q: &[f64]) -> String {
    q.iter()
        .map(|value| value.to_bits().to_string())
        .collect::<Vec<_>>()
        .join(",")
}

fn q_from_bit_line(contents: &str, prefix: &str) -> Vec<f64> {
    contents
        .lines()
        .find_map(|line| line.strip_prefix(prefix))
        .map(|text| {
            text.split(',')
                .filter_map(|value| value.parse::<u64>().ok())
                .map(f64::from_bits)
                .collect()
        })
        .unwrap_or_default()
}

fn q_from_contents(contents: &str) -> Vec<f64> {
    q_from_bit_line(contents, "q_end_bits=")
}

fn initial_configuration_matches(request: &ReasoningRequest, contents: &str) -> bool {
    let q0 = q_from_bit_line(contents, "q0_bits=");
    q0.len() == request.scene.arm_q.len()
        && q0
            .iter()
            .zip(request.scene.arm_q.iter())
            .all(|(left, right)| left.to_bits() == right.to_bits())
}

fn geometric_key(model: &EmbodimentModel) -> String {
    let mut bytes = Vec::new();
    push_str(&mut bytes, PROOF_ALGORITHM);
    push_str(&mut bytes, &model.model_hash);
    push_str(&mut bytes, &model.calibration_epoch);
    push_str(&mut bytes, &model.robot_id);
    for joint in &model.joints {
        push_str(&mut bytes, &joint.name);
        push_opt_f64(&mut bytes, joint.q_min.value);
        push_opt_f64(&mut bytes, joint.q_max.value);
    }
    digest_bytes(&bytes)
}

fn proof_dependency_digest(
    family: &str,
    model: &EmbodimentModel,
    request: &ReasoningRequest,
    candidate: &RawCandidate,
) -> String {
    let mut bytes = Vec::new();
    push_str(&mut bytes, PROOF_ALGORITHM);
    push_str(&mut bytes, family);
    push_str(&mut bytes, &candidate.action_key);
    push_str(&mut bytes, &model.model_hash);
    push_str(&mut bytes, &model.calibration_epoch);
    push_str(&mut bytes, &model.robot_id);
    push_str(&mut bytes, &request.scene.observation_epoch);
    for name in &request.scene.joint_names {
        push_str(&mut bytes, name);
    }
    push_q(&mut bytes, &request.scene.arm_q);
    for name in &request.scene.finger_names {
        push_str(&mut bytes, name);
    }
    push_q(&mut bytes, &request.scene.finger_q);
    push_str(&mut bytes, &request.scene.object_id);
    push_q(&mut bytes, &request.scene.object_center);
    push_q(&mut bytes, &request.scene.object_half);
    push_q(&mut bytes, &request.scene.object_quat);
    push_q(&mut bytes, &request.scene.support_origin);
    push_q(&mut bytes, &request.scene.support_normal);
    let mut obstacles = request.scene.obstacles.clone();
    obstacles.sort_by(|left, right| left.name.cmp(&right.name));
    for obstacle in obstacles {
        push_str(&mut bytes, &obstacle.name);
        push_q(&mut bytes, &obstacle.center);
        push_q(&mut bytes, &obstacle.half_extents);
        push_q(&mut bytes, &obstacle.quat_wxyz);
    }
    push_f64(&mut bytes, request.scene.tool_radius_m);
    push_str(&mut bytes, &request.scene.ee);
    bytes.push(u8::from(request.scene.held));
    push_q(&mut bytes, &candidate.direction);
    push_f64(&mut bytes, candidate.stroke_m);
    push_q(&mut bytes, &candidate.target_xyz);
    bytes.push(u8::from(candidate.probe));
    push_f64(&mut bytes, request.mu_required);
    push_f64(&mut bytes, request.goal_stroke_m);
    push_f64(&mut bytes, request.probe_stroke_m);
    push_opt_f64(&mut bytes, request.mass_kg);
    push_opt_f64(&mut bytes, request.gripper_force_n);
    push_q(&mut bytes, &request.gravity_m_s2);
    match request.objective {
        PhysicalObjective::PlanarTranslation { target_xy } => {
            bytes.push(1);
            push_f64(&mut bytes, target_xy[0]);
            push_f64(&mut bytes, target_xy[1]);
        }
        PhysicalObjective::AcquireObject { ref object_id } => {
            bytes.push(2);
            push_str(&mut bytes, object_id);
        }
    }
    let mut belief = request.belief.parameters.clone();
    belief.sort_by_key(|entry| format!("{:?}", entry.parameter));
    for entry in belief {
        push_str(
            &mut bytes,
            &format!("{:?}:{:?}", entry.parameter, entry.status),
        );
        push_opt_f64(&mut bytes, entry.declared.value);
        match entry.empirical_interval {
            Some(interval) => {
                push_f64(&mut bytes, interval[0]);
                push_f64(&mut bytes, interval[1]);
            }
            None => bytes.push(0),
        }
    }
    push_str(
        &mut bytes,
        "applicability:declared-quasi-static-contact-model",
    );
    digest_bytes(&bytes)
}

fn digest_bytes(bytes: &[u8]) -> String {
    use sha2::{Digest, Sha256};
    hex::encode(Sha256::digest(bytes))
}

fn push_str(bytes: &mut Vec<u8>, text: &str) {
    bytes.extend_from_slice(&(text.len() as u64).to_le_bytes());
    bytes.extend_from_slice(text.as_bytes());
}

fn push_f64(bytes: &mut Vec<u8>, value: f64) {
    bytes.extend_from_slice(&value.to_bits().to_le_bytes());
}

fn push_opt_f64(bytes: &mut Vec<u8>, value: Option<f64>) {
    match value {
        Some(value) => {
            bytes.push(1);
            push_f64(bytes, value);
        }
        None => bytes.push(0),
    }
}

fn push_q(bytes: &mut Vec<u8>, values: &[f64]) {
    bytes.extend_from_slice(&(values.len() as u64).to_le_bytes());
    for value in values {
        push_f64(bytes, *value);
    }
}

fn attach_grant(request: &ReasoningRequest, evidence: &mut CandidateEvidence) {
    if let Some(grant) = request.grants.iter().find(|grant| {
        grant.matches_probe(evidence, request.now_s, &request.scene.observation_epoch)
    }) {
        evidence.scoped_grant = Some(grant.clone());
    }
}

fn decision_action_key(decision: &DecisionKind) -> Option<&str> {
    match decision {
        DecisionKind::GoalInteraction { action }
        | DecisionKind::PhysicalProbe { action, .. }
        | DecisionKind::ContactTransition { action, .. } => Some(action.action_key.as_str()),
        _ => None,
    }
}

fn axis_label(direction: [f64; 3]) -> &'static str {
    if direction[0] > 0.5 {
        "+x"
    } else if direction[0] < -0.5 {
        "-x"
    } else if direction[1] > 0.5 {
        "+y"
    } else {
        "-y"
    }
}

fn fixture_binding(candidate: &CandidateSummary, request: &ReasoningRequest) -> ScopedGrantBinding {
    let digest = candidate.witness_digest.clone().unwrap_or_default();
    ScopedGrantBinding {
        grant_id: format!("sim-scope:{digest}"),
        scope_digest: digest.clone(),
        action_key: candidate.action_key.clone(),
        candidate_id: candidate.action_key.clone(),
        witness_digest: digest,
        observation_epoch: request.scene.observation_epoch.clone(),
        requested_stroke_m: request.probe_stroke_m,
        expires_at_s: 40.0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::contact_collision::{forbidden_class_on_interpolation, CollisionWorldError};
    use crate::execution_envelope::{
        supervise_execution, ExecutionAuthorization, ExecutionProgress, RuntimePolicyObservation,
        SupervisorDecision,
    };
    use crate::hardware_boundary::{HardwareActionScope, HardwareAuthority, UnconfiguredHardware};
    use crate::maneuver_witness::TransitionKind;
    use std::collections::BTreeMap;

    #[test]
    fn interpolation_failure_is_not_absence_of_collision() {
        let model = synth_planar_two_link();
        let world = CollisionWorld {
            object_id: "object".into(),
            object_center: [0.2, 0.0, 0.0],
            object_half: [0.02, 0.02, 0.02],
            object_quat: [1.0, 0.0, 0.0, 0.0],
            support_id: "table".into(),
            support_origin: [0.2, 0.0, -0.05],
            support_normal: [0.0, 0.0, 1.0],
            intended_tool_bodies: vec!["tool".into()],
            robot_volumes: Vec::new(),
            obstacles: Vec::new(),
            ee_radius: 0.012,
            object_probe_radius: 0.012,
            tool_offset_ee: [0.0, 0.0, 0.0],
            declared_geoms: Vec::new(),
        };
        let hit = forbidden_class_on_interpolation(
            &model,
            "ee",
            &["j0".into(), "j1".into()],
            &[0.0],
            &[0.2, 0.2],
            &world,
            TransitionKind::CurrentToApproach,
        );
        assert!(matches!(
            hit,
            Err(CollisionWorldError::InterpolationUnavailable)
        ));
    }

    #[test]
    fn sampled_stroke_domain_cannot_be_preserved() {
        let (_model, request) = benchmark_scene();
        let coverage = CommonWitnessCoverage {
            label: "stay".into(),
            witness_digest: "witness".into(),
            contact_point: request.scene.object_center,
            object_center: request.scene.object_center,
            object_quat: request.scene.object_quat,
            object_half: request.scene.object_half,
            push_direction: [1.0, 0.0, 0.0],
            support_normal: request.scene.support_normal,
            face_gap_m: 0.0,
            translation_radius_m: 0.01,
            yaw_abs_rad: 0.0,
            geometry_residual_m: 0.0,
            support_clearance_m: 0.02,
            min_support_clearance_m: 0.004,
            joint_margin_rad: 0.2,
            collision_admissible_for_grown_object: false,
            model_applicable: true,
            model_applicability: "samples".into(),
            observation_epoch: request.scene.observation_epoch.clone(),
        };
        assert!(crate::future_interaction::domain_from_sticking_stroke(&coverage, true).is_err());
    }

    #[test]
    fn blocked_translation_needs_a_probe_then_an_intermediate_push() {
        let (model, mut request) = benchmark_scene();
        let mut reasoner = PhysicalReasoner::new(model);
        assert_eq!(reasoner.families(), [PUSH_FAMILY, GRASP_FAMILY]);
        let first = reasoner.consider(&request);
        assert!(
            first.plan.is_none(),
            "unknown friction must not invent a sticking plan, got {:?}",
            first.plan.as_ref().map(|plan| plan
                .steps
                .iter()
                .map(|step| step.action_key.clone())
                .collect::<Vec<_>>())
        );
        let probe = first
            .candidates
            .iter()
            .find(|candidate| candidate.action_key == "probe:+x")
            .expect("probe");
        assert_eq!(probe.status, "PROVED");
        assert!(matches!(
            first.decision,
            DecisionKind::Refuse { .. } | DecisionKind::InsufficientEvidence { .. }
        ));
        request.grants = vec![fixture_binding(probe, &request)];
        let selected = reasoner.consider(&request);
        let action = match &selected.decision {
            DecisionKind::PhysicalProbe { action, .. }
            | DecisionKind::ContactTransition { action, .. } => action,
            other => panic!("granted probe should be selected, got {other:?}"),
        };
        assert_eq!(action.action_key, "probe:+x");
        let grant = covering_authorization(&reasoner, &request, &selected_step(action));
        let authorized = reasoner
            .authorize_step(&request, &selected_step(action), grant.clone())
            .expect("issued grant binds");
        let mut env = HiddenTable {
            center: request.scene.object_center,
            true_mu: 0.6,
            held: false,
            unauthorized_writes: 0,
        };
        assert!(env.apply_without_grant().is_err());
        assert_eq!(env.unauthorized_writes, 0);
        let observed = env.apply(&authorized).expect("authorized probe");
        let ratio = observed.displacement_m / request.probe_stroke_m;
        let before = request
            .belief
            .declared_value(PhysicalParameter::SupportFriction);
        let receipt = integrate_support_friction(
            &request.belief,
            ratio,
            request.mu_required,
            "stick-observation",
        );
        assert!(receipt.information_gain);
        assert_eq!(
            receipt
                .belief_after
                .declared_value(PhysicalParameter::SupportFriction),
            before
        );
        assert!(
            receipt
                .belief_after
                .entry(PhysicalParameter::ToolObjectFriction)
                .unwrap()
                .status
                == BeliefEpistemicStatus::Unknown
        );
        let interval = receipt
            .belief_after
            .entry(PhysicalParameter::SupportFriction)
            .unwrap()
            .empirical_interval
            .unwrap();
        assert!(interval[0] + 1e-12 >= request.mu_required);
        request.belief = receipt.belief_after.clone();
        request.grants.clear();
        request.scene.object_center = observed.center;
        let after = reasoner.consider(&request);
        let plan = after.plan.clone().unwrap_or_else(|| {
            let lines: Vec<String> = after
                .candidates
                .iter()
                .map(|candidate| {
                    format!(
                        "{} {} {} mech={:?} progress={} deriv={:?}",
                        candidate.family,
                        candidate.action_key,
                        candidate.status,
                        candidate.mechanics,
                        candidate.immediate_progress,
                        candidate.error_derivative
                    )
                })
                .collect();
            panic!("no plan\n{}", lines.join("\n"));
        });
        assert!(plan.conditional);
        assert!(plan.greedy_failed);
        assert!(!plan.steps[0].immediate_progress);
        assert!(plan.steps[1].immediate_progress);
        assert!(matches!(
            after.decision,
            DecisionKind::CurrentlyUnachievable { .. }
                | DecisionKind::InsufficientEvidence { .. }
                | DecisionKind::Refuse { .. }
        ));
        let lateral_grant = covering_authorization(&reasoner, &request, &plan.steps[0]);
        let lateral = reasoner
            .authorize_step(&request, &plan.steps[0], lateral_grant.clone())
            .unwrap();
        assert!(reasoner
            .authorize_step(&request, &plan.steps[1], lateral_grant)
            .is_err());
        let moved = env.apply(&lateral).unwrap();
        let stale_second = plan.steps[1].clone();
        request.scene.arm_q = q_from_contents(&lateral.frozen().witness_contents);
        let mut disagreed = request.clone();
        disagreed.scene.object_center = moved.center;
        disagreed.scene.object_center[1] += 0.004;
        assert!(
            reasoner
                .authorize_step(
                    &disagreed,
                    &stale_second,
                    covering_authorization(&reasoner, &disagreed, &stale_second)
                )
                .is_err(),
            "a predicted second step is stale when the object is not where the plan assumed"
        );
        let mut lost = request.clone();
        lost.scene.object_center = moved.center;
        lost.scene.held = true;
        assert!(reasoner
            .authorize_step(
                &lost,
                &stale_second,
                covering_authorization(&reasoner, &lost, &stale_second)
            )
            .is_err());
        let mut blocked = request.clone();
        blocked.scene.object_center = moved.center;
        blocked.scene.obstacles.push(NamedBox::aabb(
            "surprise",
            blocked.scene.object_center,
            [0.05, 0.05, 0.05],
        ));
        assert!(reasoner
            .authorize_step(
                &blocked,
                &stale_second,
                covering_authorization(&reasoner, &blocked, &stale_second)
            )
            .is_err());
        request.scene.object_center = moved.center;
        let mut shifted = request.clone();
        shifted.scene.observation_epoch = "epoch:other".into();
        assert!(reasoner
            .authorize_step(
                &shifted,
                &plan.steps[0],
                covering_authorization(&reasoner, &shifted, &plan.steps[0])
            )
            .is_err());
        let mut joints = request.clone();
        joints.scene.arm_q[0] += 0.05;
        assert!(reasoner
            .authorize_step(
                &joints,
                &plan.steps[0],
                covering_authorization(&reasoner, &joints, &plan.steps[0])
            )
            .is_err());
        let replanned = reasoner.consider(&request);
        let next = replanned
            .executable
            .clone()
            .or_else(|| {
                replanned
                    .plan
                    .as_ref()
                    .and_then(|plan| plan.steps.first().cloned())
            })
            .expect("fresh action from the observed state");
        let goal_grant = covering_authorization(&reasoner, &request, &next);
        let goal = reasoner
            .authorize_step(&request, &next, goal_grant)
            .unwrap();
        let finished = env.apply(&goal).unwrap();
        let start_error = 0.10;
        let end_error =
            ((finished.center[0] - 0.20).powi(2) + (finished.center[1] - 0.10).powi(2)).sqrt();
        assert!(
            end_error < start_error - 1e-3,
            "end {end_error} center {:?} moved {:?} steps {}/{} strokes {}/{} keys {} {}",
            finished.center,
            moved.center,
            plan.steps[0].action_key,
            plan.steps[1].action_key,
            plan.steps[0].stroke_m,
            plan.steps[1].stroke_m,
            lateral.frozen().requested_stroke_m,
            goal.frozen().requested_stroke_m
        );
        let mut divergent = policy_observation(&goal, 0.001, 0.2);
        divergent.object_displacement_m = Some(0.2);
        assert!(matches!(
            supervise_execution(&goal.frozen, &progress(&goal), &divergent),
            SupervisorDecision::AbortAndReobserve { .. }
        ));
        reasoner.abort_grant(&goal);
        assert_eq!(
            reasoner.resume_aborted(&goal),
            Err("ABORTED_GRANT_CANNOT_RESUME")
        );
        assert!(reasoner
            .authorize_step(
                &request,
                &next,
                goal.frozen().execution_authorization.clone().unwrap()
            )
            .is_err());
        request.belief = belief_with_finger(request.belief.clone());
        request.objective = PhysicalObjective::AcquireObject {
            object_id: "block".into(),
        };
        let acquired = reasoner.consider(&request);
        let grasp = acquired
            .candidates
            .iter()
            .find(|candidate| candidate.family == GRASP_FAMILY && candidate.status == "PROVED")
            .unwrap_or_else(|| {
                let lines: Vec<String> = acquired
                    .candidates
                    .iter()
                    .map(|candidate| {
                        format!(
                            "{} {} {}",
                            candidate.family, candidate.action_key, candidate.status
                        )
                    })
                    .collect();
                panic!(
                    "grasp proved after finger friction is declared\n{}",
                    lines.join("\n")
                );
            });
        assert_eq!(grasp.mechanics, MechanicsStatus::Feasible);
        let mut missing = request.clone();
        missing.belief = PhysicalParameterBelief { parameters: vec![] };
        let unknown = reasoner.consider(&missing);
        let grasp_unknown = unknown
            .candidates
            .iter()
            .find(|candidate| candidate.action_key == "grasp:y")
            .unwrap();
        assert_ne!(grasp_unknown.mechanics, MechanicsStatus::Feasible);
        assert_eq!(env.unauthorized_writes, 0);
        let mut hardware = UnconfiguredHardware;
        assert!(hardware
            .issue(&HardwareActionScope {
                action_key: "grasp:y".into(),
                witness_digest: "none".into(),
                observation_epoch: "epoch:table".into(),
            })
            .is_err());
    }

    #[test]
    fn benchmark_cache_cuts_repeated_ik_and_records_the_task() {
        let report = development_benchmark();
        assert_eq!(report.schema, BENCHMARK_SCHEMA);
        assert_eq!(report.evidence_status, EVIDENCE_STATUS);
        assert!(report.cold_ik_attempts > 0, "{report:?}");
        assert!(report.warm_ik_attempts < report.cold_ik_attempts);
        assert_eq!(report.tasks_solved, 1);
        assert_eq!(report.tasks_correctly_refused, 1);
        assert!(report.greedy_failed_on_solved_task);
        assert!(!report.parallel_evaluation);
        println!(
            "benchmark schema={} cold_ik={} warm_ik={} cold_collision={} warm_collision={} cold_fk={} warm_fk={} jacobian_cold={} mechanics_cold={} solved={} refused={}",
            report.schema,
            report.cold_ik_attempts,
            report.warm_ik_attempts,
            report.cold_collision_queries,
            report.warm_collision_queries,
            report.cold_fk_evaluations,
            report.warm_fk_evaluations,
            report.cold_jacobian_evaluations,
            report.cold_mechanics_evaluations,
            report.tasks_solved,
            report.tasks_correctly_refused
        );
        assert!(report.cache_hits > 0, "{report:?}");
        assert!(report.ik_avoided > 0, "{report:?}");
    }

    #[test]
    fn one_dependency_change_misses_the_proof_cache() {
        let (model, request) = benchmark_scene();
        let mut reasoner = PhysicalReasoner::new(model);
        let _ = reasoner.consider(&request);
        let cold = reasoner.cache_stats();
        assert!(cold.executable_misses > 0);
        let _ = reasoner.consider(&request);
        let warm = reasoner.cache_stats();
        assert!(warm.executable_hits > cold.executable_hits);
        assert!(warm.geometric_hits > 0);
        assert!(warm.ik_avoided > 0);
        for index in 0..8 {
            let mut changed = request.clone();
            let before = reasoner.cache_stats().executable_misses;
            match index {
                0 => {
                    changed.scene.object_center[0] =
                        f64::from_bits(changed.scene.object_center[0].to_bits() + 1);
                }
                1 => changed.scene.object_quat = [0.98, 0.0, 0.0, 0.199],
                2 => changed.scene.observation_epoch = "epoch:next".into(),
                3 => changed.scene.arm_q[0] += 0.01,
                4 => changed.scene.obstacles[0].center[1] += 0.001,
                5 => changed.mass_kg = Some(0.07),
                6 => changed.gravity_m_s2[2] = -9.80,
                _ => changed.belief.narrow_interval(
                    PhysicalParameter::SupportFriction,
                    [0.06, 0.8],
                    "shift",
                ),
            }
            let _ = reasoner.consider(&changed);
            assert!(
                reasoner.cache_stats().executable_misses > before,
                "dependency change reused a proof"
            );
        }
        let mut granted = request.clone();
        granted.grants = vec![fixture_binding(
            &reasoner.consider(&request).candidates[0],
            &request,
        )];
        let hits = reasoner.cache_stats().executable_hits;
        let _ = reasoner.consider(&granted);
        assert!(
            reasoner.cache_stats().executable_hits > hits,
            "a grant must not change proof identity"
        );
    }

    #[test]
    fn three_laterals_need_more_than_the_original_two_expansion_search() {
        let (model, mut request) = benchmark_scene();
        request.goal_stroke_m = 0.015;
        request.scene.object_center[0] = 0.18;
        request.objective = PhysicalObjective::PlanarTranslation {
            target_xy: [0.18, 0.10],
        };
        request.scene.obstacles = vec![NamedBox::aabb(
            "long-wall",
            [0.165, -0.030, 0.0],
            [0.045, 0.008, 0.02],
        )];
        request
            .belief
            .narrow_interval(PhysicalParameter::SupportFriction, [0.4, 0.8], "known");
        let mut shallow = PhysicalReasoner::new(model.clone());
        let root_error = translation_errors(&request, request.scene.object_center).0;
        let mut budget = 48u32;
        let depth_two = bounded_plan(&mut shallow, &request, 2, root_error, &mut budget);
        let solved_in_two = depth_two.as_ref().is_some_and(|plan| {
            plan.steps
                .last()
                .is_some_and(|step| step.error_after + 1e-4 < root_error)
        });
        assert!(
            !solved_in_two,
            "this wall must survive a two-expansion search, got {:?}",
            depth_two.as_ref().map(|plan| plan
                .steps
                .iter()
                .map(|step| step.action_key.clone())
                .collect::<Vec<_>>())
        );
        let mut reasoner = PhysicalReasoner::new(model);
        let report = reasoner.consider(&request);
        let plan = report.plan.unwrap_or_else(|| {
            let lines: Vec<String> = report
                .candidates
                .iter()
                .map(|candidate| {
                    format!(
                        "{} {} mech={:?}",
                        candidate.action_key, candidate.status, candidate.mechanics
                    )
                })
                .collect();
            panic!(
                "depth-three search clears the long wall\n{}",
                lines.join("\n")
            );
        });
        assert!(plan.expansions >= 3, "{plan:?}");
        assert!(plan
            .steps
            .last()
            .is_some_and(|step| step.immediate_progress));
        assert!(plan.conditional);
    }

    struct HiddenTable {
        center: [f64; 3],
        true_mu: f64,
        held: bool,
        unauthorized_writes: u64,
    }

    struct EnvObs {
        displacement_m: f64,
        center: [f64; 3],
    }

    impl HiddenTable {
        fn apply_without_grant(&mut self) -> Result<(), ()> {
            Err(())
        }

        fn apply(&mut self, action: &AuthorizedAction) -> Result<EnvObs, ()> {
            let frozen = action.frozen();
            let Some(grant) = frozen.execution_authorization.as_ref() else {
                return Err(());
            };
            if !grant.covers(frozen, 10.1) {
                return Err(());
            }
            let contents = &frozen.witness_contents;
            let family = contents
                .lines()
                .find_map(|line| line.strip_prefix("family="))
                .unwrap_or("");
            if family == GRASP_FAMILY {
                self.held = true;
                return Ok(EnvObs {
                    displacement_m: 0.0,
                    center: self.center,
                });
            }
            let push = parse_push(contents).ok_or(())?;
            let stroke = frozen.requested_stroke_m;
            let travel = if self.true_mu >= 0.2 {
                stroke
            } else {
                stroke * 0.2
            };
            self.center = add3(self.center, scale3(push, travel));
            Ok(EnvObs {
                displacement_m: travel,
                center: self.center,
            })
        }
    }

    fn parse_push(contents: &str) -> Option<[f64; 3]> {
        let text = contents
            .lines()
            .find_map(|line| line.strip_prefix("push_bits="))?;
        let mut values = text
            .split(',')
            .filter_map(|value| value.parse::<u64>().ok())
            .map(f64::from_bits);
        Some([values.next()?, values.next()?, values.next()?])
    }

    fn selected_step(action: &crate::physical_decision::SelectedPhysicalAction) -> PlanStep {
        PlanStep {
            family: PUSH_FAMILY.into(),
            action_key: action.action_key.clone(),
            candidate_id: action.candidate_id.clone(),
            witness_digest: action.witness_digest.clone(),
            witness_contents: action.witness_contents.clone(),
            stroke_m: 0.008,
            immediate_progress: false,
            predicted_translation_m: 0.008,
            error_after: 0.1,
        }
    }

    fn covering_authorization(
        reasoner: &PhysicalReasoner,
        request: &ReasoningRequest,
        step: &PlanStep,
    ) -> ExecutionAuthorization {
        let frozen = frozen_from_step(&reasoner.model, request, step);
        let digest = frozen.witness_digest.clone();
        ExecutionAuthorization {
            grant_id: format!("sim-scope:{digest}"),
            scope_digest: digest.clone(),
            model_id: frozen.model_id.clone(),
            embodiment_id: frozen.embodiment_id.clone(),
            observation_epoch: frozen.observation_epoch.clone(),
            candidate_id: frozen.candidate_id.clone(),
            action_key: frozen.action_key.clone(),
            witness_digest: digest,
            actuator_id: frozen.actuator_id.clone(),
            requested_stroke_m: frozen.requested_stroke_m,
            execution_bound_m: frozen.requested_stroke_m.max(0.02),
            issued_at_s: 10.0,
            expires_at_s: 40.0,
            observation_contract_id: frozen.observation_contract_id.clone(),
            abort_contract_id: frozen.abort_contract_id.clone(),
        }
    }

    fn progress(action: &AuthorizedAction) -> ExecutionProgress {
        ExecutionProgress {
            action_id: action.frozen().action_id.clone(),
            witness_id: action.frozen().witness_id.clone(),
            completed_quanta: 0,
            total_quanta: 2,
            stroke_consumed_m: Some(0.001),
            contact_guard_active: true,
            now_s: 10.1,
            remainder_invalidated: false,
        }
    }

    fn policy_observation(
        action: &AuthorizedAction,
        stroke: f64,
        displacement: f64,
    ) -> RuntimePolicyObservation {
        RuntimePolicyObservation {
            action_id: action.frozen().action_id.clone(),
            witness_id: action.frozen().witness_id.clone(),
            observation_id: "obs:divergence".into(),
            source: "policy-observation".into(),
            timestamp_s: 10.1,
            model_epoch: action.frozen().observation_contract.model_epoch.clone(),
            calibration_epoch: action
                .frozen()
                .observation_contract
                .calibration_epoch
                .clone(),
            units: BTreeMap::new(),
            stroke_consumed_m: Some(stroke),
            object_displacement_m: Some(displacement),
            yaw_change_rad: Some(0.0),
            intended_contact_persists: Some(true),
            goal_error_before: Some(0.1),
            goal_error_now: Some(0.1),
            robot_tracking_error_m: Some(0.0),
            reachability_margin_m: Some(0.05),
            quasi_static_applicable: Some(true),
            authority_ok: Some(true),
        }
    }

    fn belief_with_finger(mut belief: PhysicalParameterBelief) -> PhysicalParameterBelief {
        belief
            .parameters
            .retain(|entry| entry.parameter != PhysicalParameter::ToolObjectFriction);
        let mut declared = PhysicalParameterBelief::declared_point(
            PhysicalParameter::ToolObjectFriction,
            0.5,
            "finger.declared",
        );
        declared.parameters.extend(belief.parameters);
        declared
    }
}
