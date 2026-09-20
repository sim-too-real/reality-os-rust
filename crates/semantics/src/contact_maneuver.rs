//! PUSH contact as a feasible maneuver, not a reachable XYZ sample.
//!
//! No robot name, vendor, URDF filename, bundle ID, or dataset origin.

use crate::command_domain::{named_joint_limit_margin, MIN_NAMED_JOINT_MARGIN_FRAC};
use crate::contact::declared_manipulation_contact_bodies;
use crate::contact_collision::{
    apply_collision_admissibility, is_collision_block_reason, AttachedSphere, CollisionWorld,
    NamedBox,
};
use crate::contact_manifold::{
    contact_constraint_residual, evaluate_box_face_contact, ContactGeometryTolerance,
    ContactManifoldTarget, ContactingBodyKind, ExecutionPoseTolerance, IkResidual, ManifoldReject,
    PositionTarget,
};
use crate::embodiment::{EmbodimentModel, Joint};
use crate::kinematics::{
    forward_kinematics, ik_residual_is_precise, solve_ik, with_ik_q_seed, IK_ACCEPT_M,
};
use crate::maneuver_witness::{
    execution_block_reason, witness_from_continuing_phases, witness_min_joint_margin,
    ExecutableContactManeuver, ManeuverPhase, TransitionVerdict,
};
use crate::push::{
    effective_push_distance, push_approach_standoff_m, push_contact_success_radius,
    support_plane_direction,
};
use crate::transform::{add3, norm3, normalize3, quat_conj, rotate_by_quat, scale3, sub3, Se3};
use serde::{Deserialize, Serialize};
use std::cell::{Cell, RefCell};

/// How contact targets are generated. Stroke compile mode is independent.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ContactEstablishmentMode {
    /// Object-face + sampled EE pose + support/stroke checks.
    #[default]
    ManeuverV2,
    /// Legacy: nearest/random reachable XYZ, object along push_dir.
    XyzSampleBaseline,
}

thread_local! {
    static CONTACT_ESTABLISHMENT_MODE: Cell<ContactEstablishmentMode> =
        const { Cell::new(ContactEstablishmentMode::ManeuverV2) };
}

pub fn with_contact_establishment_mode<R>(
    mode: ContactEstablishmentMode,
    f: impl FnOnce() -> R,
) -> R {
    CONTACT_ESTABLISHMENT_MODE.with(|c| {
        let prev = c.replace(mode);
        let out = f();
        c.set(prev);
        out
    })
}

pub fn current_contact_establishment_mode() -> ContactEstablishmentMode {
    CONTACT_ESTABLISHMENT_MODE.with(Cell::get)
}

/// How the world was constructed for an episode.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum WorldConstructionMode {
    /// Object/support poses are immutable. The robot adapts.
    #[default]
    FixedWorld,
    /// Object/support may be generated to match a sampled configuration.
    /// Labeled as generated test conditions, not a planning result.
    CapabilitySynthesis,
}

thread_local! {
    static WORLD_CONSTRUCTION_MODE: Cell<WorldConstructionMode> =
        const { Cell::new(WorldConstructionMode::FixedWorld) };
}

pub fn with_world_construction_mode<R>(mode: WorldConstructionMode, f: impl FnOnce() -> R) -> R {
    WORLD_CONSTRUCTION_MODE.with(|c| {
        let prev = c.replace(mode);
        let out = f();
        c.set(prev);
        out
    })
}

pub fn current_world_construction_mode() -> WorldConstructionMode {
    WORLD_CONSTRUCTION_MODE.with(Cell::get)
}

/// FK sample that includes orientation and the joint vector that produced it.
#[derive(Debug, Clone, PartialEq)]
pub struct SampledEePose {
    pub xyz: [f64; 3],
    pub quat_wxyz: [f64; 4],
    pub q: Vec<f64>,
    pub joint_names: Vec<String>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SupportPlane {
    pub origin: [f64; 3],
    pub normal: [f64; 3],
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BoxObject {
    pub center: [f64; 3],
    pub half_extents: [f64; 3],
    /// Object-frame orientation (wxyz). Identity means world-aligned.
    pub quat_wxyz: [f64; 4],
}

impl BoxObject {
    pub fn new(center: [f64; 3], half_extents: [f64; 3]) -> Self {
        Self {
            center,
            half_extents,
            quat_wxyz: [1.0, 0.0, 0.0, 0.0],
        }
    }

    pub fn pose(self) -> Se3 {
        Se3::try_new(self.center, self.quat_wxyz)
            .unwrap_or_else(|_| Se3::translation(self.center).unwrap_or_else(|_| Se3::identity()))
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct ContactManeuverSpec {
    pub push_direction: [f64; 3],
    pub requested_stroke: f64,
    pub approach_standoff: f64,
    pub tool_offset_ee: [f64; 3],
    pub ee_radius: f64,
    pub current_ee_xyz: [f64; 3],
    pub min_align: f64,
    pub min_stroke: f64,
    pub min_support_clearance: f64,
    pub face_gap: f64,
    pub max_approach_match: f64,
    pub object_id: String,
    pub support_body: String,
    pub intended_tool_bodies: Vec<String>,
    pub robot_body_volumes: Vec<AttachedSphere>,
    pub obstacle_boxes: Vec<NamedBox>,
    pub object_probe_radius: f64,
    /// Declared tool approach axis in the EE frame. None = not declared.
    pub declared_tool_axis_ee: Option<[f64; 3]>,
}

impl ContactManeuverSpec {
    pub fn table_push(
        push_direction: [f64; 3],
        requested_stroke: f64,
        current_ee_xyz: [f64; 3],
        tool_offset_ee: [f64; 3],
    ) -> Self {
        let stroke = effective_push_distance(requested_stroke);
        Self {
            push_direction,
            requested_stroke,
            approach_standoff: push_approach_standoff_m(),
            tool_offset_ee,
            ee_radius: 0.015,
            current_ee_xyz,
            min_align: 0.5,
            min_stroke: (stroke * 0.5).max(0.02),
            min_support_clearance: 0.004,
            face_gap: 0.015,
            // Must be < approach standoff so the contact pose itself cannot
            // satisfy approach feasibility.
            max_approach_match: (push_contact_success_radius() * 2.0).clamp(0.025, 0.04),
            object_id: "object".into(),
            support_body: "table".into(),
            intended_tool_bodies: Vec::new(),
            robot_body_volumes: Vec::new(),
            obstacle_boxes: Vec::new(),
            object_probe_radius: 0.015,
            declared_tool_axis_ee: None,
        }
    }

    pub fn contact_geometry_tolerance(&self) -> ContactGeometryTolerance {
        ContactGeometryTolerance::for_face(self.max_approach_match)
    }

    pub fn execution_pose_tolerance(&self) -> ExecutionPoseTolerance {
        let _ = self;
        ExecutionPoseTolerance::from_precise_ik()
    }
}

/// Geometric rank features. These are the only ranking inputs.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RankInputs {
    pub approach_distance: f64,
    pub joint_margin: f64,
    pub remaining_stroke: f64,
    pub support_clearance: f64,
    pub orientation_error: f64,
    pub collision_risk: f64,
}

pub fn rank_inputs_field_names() -> &'static [&'static str] {
    &[
        "approach_distance",
        "joint_margin",
        "remaining_stroke",
        "support_clearance",
        "orientation_error",
        "collision_risk",
    ]
}

#[derive(Debug, Clone, PartialEq)]
pub struct RankWhy {
    pub score: i64,
    pub inputs: RankInputs,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ContactInfeasible {
    NoFeasibleContactPose,
    NoIkSolution,
    InsufficientJointMargin,
    NoExecutableContactManeuver,
    OrientationInfeasible,
    ApproachCollidesBeforeContact,
    SupportPlaneBlocksEe,
    ContactPoseUnreachableFromApproach,
    WrongContactGeometry,
    InsufficientRemainingStroke,
    CollisionInadmissible,
}

impl ContactInfeasible {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::NoFeasibleContactPose => "NO_FEASIBLE_CONTACT_POSE",
            Self::NoIkSolution => "NO_IK_SOLUTION",
            Self::InsufficientJointMargin => "INSUFFICIENT_JOINT_MARGIN",
            Self::NoExecutableContactManeuver => "NO_EXECUTABLE_CONTACT_MANEUVER",
            Self::OrientationInfeasible => "ORIENTATION_INFEASIBLE",
            Self::ApproachCollidesBeforeContact => "APPROACH_COLLIDES_BEFORE_CONTACT",
            Self::SupportPlaneBlocksEe => "SUPPORT_PLANE_BLOCKS_EE",
            Self::ContactPoseUnreachableFromApproach => "CONTACT_POSE_UNREACHABLE_FROM_APPROACH",
            Self::WrongContactGeometry => "WRONG_CONTACT_GEOMETRY",
            Self::InsufficientRemainingStroke => "INSUFFICIENT_REMAINING_STROKE",
            Self::CollisionInadmissible => "COLLISION_INADMISSIBLE",
        }
    }
}

/// Proven layer of Mode B contact selection. Do not call every layer `feasible`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum ContactFeasibilityLayer {
    GeometricCandidate,
    IkPositionFeasible,
    WitnessConstructed,
    WitnessTransitionsFeasible,
    ExecutableCandidate,
    SelectedExecutableManeuver,
}

impl ContactFeasibilityLayer {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::GeometricCandidate => "GEOMETRIC_CANDIDATE",
            Self::IkPositionFeasible => "IK_POSITION_FEASIBLE",
            Self::WitnessConstructed => "WITNESS_CONSTRUCTED",
            Self::WitnessTransitionsFeasible => "WITNESS_TRANSITIONS_FEASIBLE",
            Self::ExecutableCandidate => "EXECUTABLE_CANDIDATE",
            Self::SelectedExecutableManeuver => "SELECTED_EXECUTABLE_MANEUVER",
        }
    }
}

/// Honest Mode B selection funnel. Counts are proven program facts.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Default)]
pub struct ContactSelectFunnel {
    pub n_geometric_candidates: u64,
    /// Complete 4-phase IK chains. Not per-phase successes.
    pub n_ik_solutions: u64,
    #[serde(default)]
    pub n_phase_ik_attempts: u64,
    #[serde(default)]
    pub n_phase_ik_successes: u64,
    #[serde(default)]
    pub n_complete_ik_chains: u64,
    pub n_complete_witnesses: u64,
    pub n_joint_margin_valid: u64,
    pub n_transition_valid: u64,
    pub n_executable_candidates: u64,
    pub n_selected_executable: u64,
    #[serde(default)]
    pub last_block_reason: String,
}

impl ContactSelectFunnel {
    pub fn deepest_proven_layer(&self) -> Option<ContactFeasibilityLayer> {
        if self.n_selected_executable > 0 {
            Some(ContactFeasibilityLayer::SelectedExecutableManeuver)
        } else if self.n_executable_candidates > 0 {
            Some(ContactFeasibilityLayer::ExecutableCandidate)
        } else if self.n_transition_valid > 0 {
            Some(ContactFeasibilityLayer::WitnessTransitionsFeasible)
        } else if self.n_complete_witnesses > 0 {
            Some(ContactFeasibilityLayer::WitnessConstructed)
        } else if self.n_complete_ik_chains > 0 || self.n_ik_solutions > 0 {
            Some(ContactFeasibilityLayer::IkPositionFeasible)
        } else if self.n_geometric_candidates > 0 {
            Some(ContactFeasibilityLayer::GeometricCandidate)
        } else {
            None
        }
    }
}

/// One approach/contact/mid/end translational IK evaluation on the shipped path.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PhaseIkRecord {
    pub phase: String,
    pub residual_m: f64,
    pub precise: bool,
    pub accepted: bool,
}

thread_local! {
    static PHASE_IK_RECORDS: RefCell<Option<Vec<PhaseIkRecord>>> = const { RefCell::new(None) };
}

/// Observe every phase IK residual Mode B actually computes.
pub fn with_phase_ik_records<R>(f: impl FnOnce() -> R) -> (R, Vec<PhaseIkRecord>) {
    PHASE_IK_RECORDS.with(|c| {
        let prev = c.replace(Some(Vec::new()));
        let out = f();
        let recs = c.replace(prev).unwrap_or_default();
        (out, recs)
    })
}

fn record_phase_ik(phase: &str, residual_m: f64, accepted: bool) {
    PHASE_IK_RECORDS.with(|c| {
        if let Some(v) = c.borrow_mut().as_mut() {
            v.push(PhaseIkRecord {
                phase: phase.into(),
                residual_m,
                precise: IkResidual {
                    translational_m: residual_m,
                }
                .is_precise(),
                accepted,
            });
        }
    });
}

fn manifold_to_infeasible(r: ManifoldReject) -> ContactInfeasible {
    match r {
        ManifoldReject::WrongOrientation => ContactInfeasible::OrientationInfeasible,
        ManifoldReject::InsideSupport => ContactInfeasible::SupportPlaneBlocksEe,
        ManifoldReject::WrongContactingBody
        | ManifoldReject::TangentUOutside
        | ManifoldReject::TangentVOutside
        | ManifoldReject::NormalSeparation
        | ManifoldReject::OppositeFace => ContactInfeasible::WrongContactGeometry,
    }
}

/// Public finite-face evaluator used by Mode B. No robot identity.
pub fn evaluate_contact_candidate(
    tool_point_world: [f64; 3],
    tool_axis_world: Option<[f64; 3]>,
    contacting: ContactingBodyKind,
    object: BoxObject,
    support: SupportPlane,
    spec: &ContactManeuverSpec,
) -> Result<(), ContactInfeasible> {
    let push = plane_push(spec.push_direction).ok_or(ContactInfeasible::WrongContactGeometry)?;
    evaluate_box_face_contact(
        tool_point_world,
        tool_axis_world,
        contacting,
        object.pose(),
        object.half_extents,
        push,
        support.origin,
        support.normal,
        spec.face_gap,
        spec.min_align,
        spec.contact_geometry_tolerance(),
    )
    .map(|_| ())
    .map_err(manifold_to_infeasible)
}

#[derive(Debug, Clone, PartialEq)]
pub struct ContactManeuver {
    pub contact_pose: Se3,
    pub approach_pose: Se3,
    pub contact_point: [f64; 3],
    pub contact_normal: [f64; 3],
    pub push_direction: [f64; 3],
    pub requested_stroke: f64,
    pub available_stroke: f64,
    pub support_clearance: f64,
    pub joint_margin: f64,
    pub orientation_error: f64,
    pub object_center: [f64; 3],
    pub support_top_z: f64,
    pub sampled_q: Vec<f64>,
    pub executable: Option<ExecutableContactManeuver>,
}

const TOOL_LEN_MIN: f64 = 0.008;
const STROKE_CORRIDOR_M: f64 = 0.08;
/// Neighborhood in which a sampled q can seed IK to a geometric contact pose.
const IK_SEED_RADIUS_M: f64 = 0.15;
/// Current q + high-margin nearby + nearest. Bound-resting does not end search.
const MAX_IK_SEEDS_PER_GEOMETRY: usize = 8;
const MAX_PERTURB_SOURCES: usize = 4;
const MAX_PERTURBATIONS_PER_SOURCE: usize = 6;
const MAX_UNIQUE_GEOMETRIES: usize = 6;

fn dot3(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn dist3(a: [f64; 3], b: [f64; 3]) -> f64 {
    norm3(sub3(a, b))
}

fn plane_push(dir: [f64; 3]) -> Option<[f64; 3]> {
    support_plane_direction(dir).or_else(|| normalize3(dir))
}

fn tool_world_offset(sample: &SampledEePose, tool_offset_ee: [f64; 3]) -> [f64; 3] {
    rotate_by_quat(sample.quat_wxyz, tool_offset_ee)
}

fn half_along_push(half: [f64; 3], push: [f64; 3]) -> f64 {
    (half[0] * push[0].abs() + half[1] * push[1].abs() + half[2] * push[2].abs()).max(1e-6)
}

/// Object center so the tool contact sits on the near face plus `face_gap`.
pub fn object_center_for_sample(
    sample: &SampledEePose,
    tool_offset_ee: [f64; 3],
    push_dir: [f64; 3],
    object_half: [f64; 3],
    face_gap: f64,
) -> Option<[f64; 3]> {
    let push = plane_push(push_dir)?;
    let tool = add3(sample.xyz, tool_world_offset(sample, tool_offset_ee));
    let along = half_along_push(object_half, push) + face_gap.max(0.0);
    Some(add3(tool, scale3(push, along)))
}

fn support_clearance_of(point: [f64; 3], support: SupportPlane, ee_radius: f64) -> f64 {
    let n = normalize3(support.normal).unwrap_or([0.0, 0.0, 1.0]);
    dot3(sub3(point, support.origin), n) - ee_radius
}

fn joint_margin_of(sample: &SampledEePose, joints: &[Joint]) -> f64 {
    crate::command_domain::named_joint_limit_margin(&sample.q, &sample.joint_names, joints)
}

fn available_stroke_along(cloud: &[SampledEePose], contact_xyz: [f64; 3], push: [f64; 3]) -> f64 {
    let mut best = 0.0_f64;
    for s in cloud {
        let d = sub3(s.xyz, contact_xyz);
        let along = dot3(d, push);
        if along <= 0.0 {
            continue;
        }
        let perp = dist3(d, scale3(push, along));
        if perp > STROKE_CORRIDOR_M {
            continue;
        }
        if along > best {
            best = along;
        }
    }
    best
}

fn orientation_error_of(tool_axis_world: Option<[f64; 3]>, push: [f64; 3]) -> f64 {
    match tool_axis_world.and_then(normalize3) {
        Some(axis) => (1.0 - dot3(axis, push).clamp(-1.0, 1.0)).clamp(0.0, 2.0),
        None => 1.0,
    }
}

fn pose(xyz: [f64; 3], quat: [f64; 4]) -> Result<Se3, ContactInfeasible> {
    Se3::try_new(xyz, quat).map_err(|_| ContactInfeasible::NoFeasibleContactPose)
}

/// Evaluate one sampled EE pose against a box sitting on a support.
pub fn evaluate_sampled_push(
    sample: &SampledEePose,
    cloud: &[SampledEePose],
    object: BoxObject,
    support: SupportPlane,
    spec: &ContactManeuverSpec,
    joints: &[Joint],
) -> Result<ContactManeuver, ContactInfeasible> {
    let push = plane_push(spec.push_direction).ok_or(ContactInfeasible::WrongContactGeometry)?;
    let tool_off = tool_world_offset(sample, spec.tool_offset_ee);
    let tool_axis = tool_axis_of(sample, spec).ok_or(ContactInfeasible::OrientationInfeasible)?;
    if dot3(tool_axis, push) < spec.min_align {
        return Err(ContactInfeasible::OrientationInfeasible);
    }

    let tool_contact = add3(sample.xyz, tool_off);
    evaluate_contact_candidate(
        tool_contact,
        Some(tool_axis),
        ContactingBodyKind::DeclaredTool,
        object,
        support,
        spec,
    )?;

    let clearance = support_clearance_of(sample.xyz, support, spec.ee_radius);
    if clearance < spec.min_support_clearance {
        return Err(ContactInfeasible::SupportPlaneBlocksEe);
    }

    let standoff = spec
        .approach_standoff
        .max(push_contact_success_radius() + 1e-4);
    let approach_xyz = [
        sample.xyz[0] - push[0] * standoff,
        sample.xyz[1] - push[1] * standoff,
        sample.xyz[2],
    ];
    let approach_clear = support_clearance_of(approach_xyz, support, spec.ee_radius);
    if approach_clear < spec.min_support_clearance {
        return Err(ContactInfeasible::ApproachCollidesBeforeContact);
    }

    let travel = dist3(approach_xyz, sample.xyz);
    if travel <= push_contact_success_radius() {
        return Err(ContactInfeasible::ContactPoseUnreachableFromApproach);
    }
    let near_approach = cloud
        .iter()
        .any(|s| dist3(s.xyz, approach_xyz) <= spec.max_approach_match)
        || dist3(spec.current_ee_xyz, approach_xyz) <= spec.max_approach_match;
    if !near_approach {
        return Err(ContactInfeasible::ContactPoseUnreachableFromApproach);
    }

    let available = available_stroke_along(cloud, sample.xyz, push);
    let mid = spec.min_stroke * 0.5;
    if available + 1e-6 < mid || available + 1e-6 < spec.min_stroke {
        return Err(ContactInfeasible::InsufficientRemainingStroke);
    }

    let support_top_z = support.origin[2];
    let contact_pose = pose(sample.xyz, sample.quat_wxyz)?;
    let approach_pose = pose(approach_xyz, sample.quat_wxyz)?;
    let orientation_error = orientation_error_of(Some(tool_axis), push);
    Ok(ContactManeuver {
        contact_pose,
        approach_pose,
        contact_point: tool_contact,
        contact_normal: scale3(push, -1.0),
        push_direction: push,
        requested_stroke: spec.requested_stroke,
        available_stroke: available,
        support_clearance: clearance,
        joint_margin: joint_margin_of(sample, joints),
        orientation_error,
        object_center: object.center,
        support_top_z,
        sampled_q: sample.q.clone(),
        executable: None,
    })
}

pub fn rank_inputs_of(m: &ContactManeuver, spec: &ContactManeuverSpec) -> RankInputs {
    let approach_distance = dist3(spec.current_ee_xyz, m.approach_pose.xyz);
    let collision_risk = if m.support_clearance <= 0.0 {
        1.0
    } else if m.support_clearance >= 0.04 {
        0.0
    } else {
        (0.04 - m.support_clearance) / 0.04
    };
    let joint_margin = m
        .executable
        .as_ref()
        .filter(|w| execution_block_reason(w).is_none())
        .map(witness_min_joint_margin)
        .unwrap_or(m.joint_margin);
    RankInputs {
        approach_distance,
        joint_margin,
        remaining_stroke: m.available_stroke,
        support_clearance: m.support_clearance,
        orientation_error: m.orientation_error,
        collision_risk,
    }
}

/// Deterministic integer score. Higher is better. No identity features.
pub fn rank_score(inputs: &RankInputs) -> i64 {
    let approach = (inputs.approach_distance * 1000.0).round() as i64;
    let stroke = (inputs.remaining_stroke * 1000.0).round() as i64;
    let clear = (inputs.support_clearance * 1000.0).round() as i64;
    let margin = (inputs.joint_margin * 1000.0).round() as i64;
    let orien = (inputs.orientation_error * 1000.0).round() as i64;
    let coll = (inputs.collision_risk * 1000.0).round() as i64;
    stroke * 2 + clear * 2 + margin - approach * 6 - orien * 4 - coll * 3
}

pub fn select_contact_maneuver(
    cands: &[ContactManeuver],
    spec: &ContactManeuverSpec,
) -> Option<(usize, RankWhy)> {
    let mut best: Option<(usize, i64, RankInputs)> = None;
    for (i, m) in cands.iter().enumerate() {
        if let Some(w) = m.executable.as_ref() {
            if execution_block_reason(w).is_some() {
                continue;
            }
        }
        let inputs = rank_inputs_of(m, spec);
        let score = rank_score(&inputs);
        let better = match &best {
            None => true,
            Some((j, s, prev)) => match score.cmp(s) {
                std::cmp::Ordering::Greater => true,
                std::cmp::Ordering::Less => false,
                std::cmp::Ordering::Equal => {
                    i < *j && {
                        let _ = prev;
                        true
                    }
                }
            },
        };
        if better {
            best = Some((i, score, inputs));
        }
    }
    best.map(|(i, score, inputs)| (i, RankWhy { score, inputs }))
}

/// Place-and-select: each sample proposes an object pose on its tool face.
pub fn select_push_maneuver(
    cloud: &[SampledEePose],
    spec: &ContactManeuverSpec,
    object_half: [f64; 3],
    support_normal: [f64; 3],
    joints: &[Joint],
) -> Result<(ContactManeuver, RankWhy), ContactInfeasible> {
    if cloud.is_empty() {
        return Err(ContactInfeasible::NoFeasibleContactPose);
    }
    let mut feasible = Vec::new();
    let mut last_err = ContactInfeasible::NoFeasibleContactPose;
    for sample in cloud {
        let Some(center) = object_center_for_sample(
            sample,
            spec.tool_offset_ee,
            spec.push_direction,
            object_half,
            spec.face_gap,
        ) else {
            last_err = ContactInfeasible::WrongContactGeometry;
            continue;
        };
        let support = SupportPlane {
            origin: [center[0], center[1], center[2] - object_half[2]],
            normal: support_normal,
        };
        let object = BoxObject {
            center,
            half_extents: object_half,
            quat_wxyz: [1.0, 0.0, 0.0, 0.0],
        };
        match evaluate_sampled_push(sample, cloud, object, support, spec, joints) {
            Ok(m) => feasible.push(m),
            Err(e) => last_err = e,
        }
    }
    let (idx, why) = select_contact_maneuver(&feasible, spec).ok_or(last_err)?;
    Ok((feasible[idx].clone(), why))
}

struct GeometricContact {
    ee: [f64; 3],
    tool: [f64; 3],
    push: [f64; 3],
}

fn geometric_contact_ee(
    object: BoxObject,
    spec: &ContactManeuverSpec,
    seed: &SampledEePose,
    support: SupportPlane,
    cloud: &[SampledEePose],
) -> Result<GeometricContact, ContactInfeasible> {
    let _ = cloud;
    let push = plane_push(spec.push_direction).ok_or(ContactInfeasible::WrongContactGeometry)?;
    let Some(manifold) = crate::contact_manifold::box_push_face_manifold_posed(
        object.pose(),
        object.half_extents,
        push,
        support.normal,
        spec.face_gap,
    ) else {
        return Err(ContactInfeasible::WrongContactGeometry);
    };
    let tool_at_face = add3(manifold.origin, scale3(manifold.normal, manifold.face_gap));
    let tool_off = tool_world_offset(seed, spec.tool_offset_ee);
    let ee = sub3(tool_at_face, tool_off);
    Ok(GeometricContact {
        ee,
        tool: tool_at_face,
        push,
    })
}

/// Mode B: object and support are immutable. Generate the contact pose from
/// object geometry, then seed IK from the nearest sampled configuration.
pub fn select_fixed_world_push(
    cloud: &[SampledEePose],
    object: BoxObject,
    support: SupportPlane,
    spec: &ContactManeuverSpec,
    joints: &[Joint],
) -> Result<(ContactManeuver, RankWhy), ContactInfeasible> {
    if cloud.is_empty() {
        return Err(ContactInfeasible::NoFeasibleContactPose);
    }
    let mut feasible = Vec::new();
    let mut last_err = ContactInfeasible::NoFeasibleContactPose;
    for sample in cloud {
        match evaluate_sampled_push(sample, cloud, object, support, spec, joints) {
            Ok(m) => {
                debug_assert_eq!(m.object_center, object.center);
                feasible.push(m);
            }
            Err(e) => last_err = e,
        }
    }
    let (idx, why) = select_contact_maneuver(&feasible, spec).ok_or(last_err)?;
    Ok((feasible[idx].clone(), why))
}

fn ik_sample_to_target(
    model: &EmbodimentModel,
    ee: &str,
    seed: &SampledEePose,
    target: PositionTarget,
    phase: &str,
) -> Result<(SampledEePose, f64), ContactInfeasible> {
    let chain = model
        .ee_joint_chain(ee)
        .ok_or(ContactInfeasible::NoFeasibleContactPose)?;
    if seed.q.len() != chain.len() || !seed.q.iter().all(|v| v.is_finite()) {
        return Err(ContactInfeasible::NoFeasibleContactPose);
    }
    let solved = with_ik_q_seed(Some(seed.q.as_slice()), || {
        solve_ik(model, &chain, ee, target.xyz, &seed.q)
    });
    let (q, trace) = match solved {
        Ok(v) => v,
        Err(_) => {
            record_phase_ik(phase, f64::INFINITY, false);
            return Err(ContactInfeasible::NoIkSolution);
        }
    };
    let fk =
        forward_kinematics(model, &chain, ee, &q).map_err(|_| ContactInfeasible::NoIkSolution)?;
    let residual = trace.residual.max(dist3(fk.ee.xyz, target.xyz));
    let precise = ik_residual_is_precise(residual);
    record_phase_ik(phase, residual, precise);
    if !precise {
        return Err(ContactInfeasible::NoIkSolution);
    }
    Ok((
        SampledEePose {
            xyz: fk.ee.xyz,
            quat_wxyz: fk.ee.quat_wxyz,
            q,
            joint_names: chain,
        },
        residual,
    ))
}

/// Declared semantic axis, mechanically derived tool offset, or UNKNOWN.
/// Never invents EE-local +X.
pub fn declared_or_derived_tool_axis(
    sample: &SampledEePose,
    spec: &ContactManeuverSpec,
) -> Option<[f64; 3]> {
    if let Some(ax) = spec.declared_tool_axis_ee {
        return normalize3(rotate_by_quat(sample.quat_wxyz, ax));
    }
    let off = tool_world_offset(sample, spec.tool_offset_ee);
    if norm3(off) >= TOOL_LEN_MIN {
        return normalize3(off);
    }
    None
}

fn tool_axis_of(sample: &SampledEePose, spec: &ContactManeuverSpec) -> Option<[f64; 3]> {
    declared_or_derived_tool_axis(sample, spec)
}

fn prove_contact_manifold(
    sample: &SampledEePose,
    spec: &ContactManeuverSpec,
    object: BoxObject,
    support: SupportPlane,
) -> Result<(), ContactInfeasible> {
    let tool = add3(sample.xyz, tool_world_offset(sample, spec.tool_offset_ee));
    let push = plane_push(spec.push_direction).ok_or(ContactInfeasible::WrongContactGeometry)?;
    let Some(manifold) = crate::contact_manifold::box_push_face_manifold_posed(
        object.pose(),
        object.half_extents,
        push,
        support.normal,
        spec.face_gap,
    ) else {
        return Err(ContactInfeasible::WrongContactGeometry);
    };
    let designed = add3(manifold.origin, scale3(manifold.normal, manifold.face_gap));
    let contact_target = ContactManifoldTarget {
        tool_point: designed,
        min_axis_align: spec.min_align,
    };
    let axis = tool_axis_of(sample, spec);
    let (n_err, u_err, v_err, axis_err) =
        contact_constraint_residual(tool, axis, &manifold, contact_target.min_axis_align);
    if axis_err > 1e-12 {
        return Err(ContactInfeasible::OrientationInfeasible);
    }
    if u_err > 1e-4 || v_err > 1e-4 || n_err > IK_ACCEPT_M {
        return Err(ContactInfeasible::WrongContactGeometry);
    }
    let tight = ContactGeometryTolerance {
        normal_m: IK_ACCEPT_M,
        tangent_slack_m: 1e-4,
    };
    evaluate_box_face_contact(
        tool,
        axis,
        ContactingBodyKind::DeclaredTool,
        object.pose(),
        object.half_extents,
        push,
        support.origin,
        support.normal,
        spec.face_gap,
        spec.min_align,
        tight,
    )
    .map(|_| ())
    .map_err(manifold_to_infeasible)
}

fn ik_sample_contact(
    model: &EmbodimentModel,
    ee: &str,
    seed: &SampledEePose,
    geo: &GeometricContact,
    spec: &ContactManeuverSpec,
    object: BoxObject,
    support: SupportPlane,
) -> Result<(SampledEePose, f64), ContactInfeasible> {
    let mut seed_i = seed.clone();
    let mut target_ee = geo.ee;
    let mut last_constraint = ContactInfeasible::OrientationInfeasible;
    for _ in 0..4 {
        let (solved, residual) = ik_sample_to_target(
            model,
            ee,
            &seed_i,
            PositionTarget { xyz: target_ee },
            "contact",
        )?;
        match prove_contact_manifold(&solved, spec, object, support) {
            Ok(()) => return Ok((solved, residual)),
            Err(e) => {
                last_constraint = e;
                let off = tool_world_offset(&solved, spec.tool_offset_ee);
                target_ee = sub3(geo.tool, off);
                seed_i = solved;
            }
        }
    }
    Err(last_constraint)
}

fn collision_world_of(
    model: &EmbodimentModel,
    ee: &str,
    object: BoxObject,
    support: SupportPlane,
    spec: &ContactManeuverSpec,
) -> CollisionWorld {
    let resource = model.resources.first();
    let mut intended = spec.intended_tool_bodies.clone();
    if intended.is_empty() {
        intended = declared_manipulation_contact_bodies(model, resource, ee);
    }
    if intended.is_empty() {
        intended.push("tool".into());
    }
    if !intended.iter().any(|n| n == "tool") {
        intended.push("tool".into());
    }
    let mut robot_volumes = spec.robot_body_volumes.clone();
    if robot_volumes.is_empty() {
        if let Some(chain) = model.ee_joint_chain(ee) {
            for jn in &chain {
                let Some(j) = model.joints.iter().find(|j| j.name == *jn) else {
                    continue;
                };
                if intended.iter().any(|n| n == &j.child_body) {
                    continue;
                }
                robot_volumes.push(AttachedSphere {
                    body: j.child_body.clone(),
                    radius: spec.ee_radius,
                    offset: [0.0, 0.0, 0.0],
                });
            }
        }
    }
    CollisionWorld {
        object_id: spec.object_id.clone(),
        object_center: object.center,
        object_half: object.half_extents,
        object_quat: object.quat_wxyz,
        support_id: spec.support_body.clone(),
        support_origin: support.origin,
        support_normal: support.normal,
        intended_tool_bodies: intended,
        robot_volumes,
        obstacles: spec.obstacle_boxes.clone(),
        ee_radius: spec.ee_radius,
        object_probe_radius: spec.object_probe_radius.max(spec.ee_radius),
        tool_offset_ee: spec.tool_offset_ee,
        declared_geoms: model.collision_geoms.clone(),
    }
}

fn seed_matches_chain(seed: &SampledEePose, names: &[String]) -> bool {
    seed.joint_names == names && seed.q.len() == names.len() && seed.q.iter().all(|v| v.is_finite())
}

fn q_quantized(q: &[f64]) -> Vec<i64> {
    q.iter().map(|v| (v * 1e5).round() as i64).collect()
}

fn xyz_quantized(p: [f64; 3]) -> (i64, i64, i64) {
    (
        (p[0] * 1000.0).round() as i64,
        (p[1] * 1000.0).round() as i64,
        (p[2] * 1000.0).round() as i64,
    )
}

fn sample_from_q(
    model: &EmbodimentModel,
    ee: &str,
    names: &[String],
    q: Vec<f64>,
) -> Option<SampledEePose> {
    let fk = forward_kinematics(model, names, ee, &q).ok()?;
    Some(SampledEePose {
        xyz: fk.ee.xyz,
        quat_wxyz: fk.ee.quat_wxyz,
        q,
        joint_names: names.to_vec(),
    })
}

/// Current q first, then nearby workspace samples ordered by named-joint margin.
fn ordered_ik_seeds<'a>(
    cloud: &'a [SampledEePose],
    names: &[String],
    geo_ee: [f64; 3],
    joints: &[Joint],
) -> Vec<&'a SampledEePose> {
    let mut out: Vec<&SampledEePose> = Vec::new();
    if let Some(cur) = cloud.first().filter(|s| seed_matches_chain(s, names)) {
        out.push(cur);
    }
    let mut nearby: Vec<&SampledEePose> = cloud
        .iter()
        .filter(|s| seed_matches_chain(s, names) && dist3(s.xyz, geo_ee) <= IK_SEED_RADIUS_M)
        .collect();
    nearby.sort_by(|a, b| {
        let ma = named_joint_limit_margin(&a.q, &a.joint_names, joints);
        let mb = named_joint_limit_margin(&b.q, &b.joint_names, joints);
        mb.partial_cmp(&ma)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then_with(|| {
                dist3(a.xyz, geo_ee)
                    .partial_cmp(&dist3(b.xyz, geo_ee))
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
    });
    for s in nearby {
        if out.len() >= MAX_IK_SEEDS_PER_GEOMETRY {
            break;
        }
        if !out.iter().any(|e| e.q == s.q) {
            out.push(s);
        }
    }
    out
}

/// Bounded deterministic perturbations of a valid seed. Seeds are clamped
/// to declared limits; solved q is never interior-clamped.
fn bounded_seed_perturbations(seed: &SampledEePose, joints: &[Joint]) -> Vec<Vec<f64>> {
    const FRACS: [f64; 4] = [-0.16, -0.08, 0.08, 0.16];
    let n = seed.q.len();
    let mut out = Vec::new();
    for i in 0..n {
        let Some(name) = seed.joint_names.get(i) else {
            continue;
        };
        let Some(j) = joints.iter().find(|j| j.name == *name) else {
            continue;
        };
        let (Some(lo), Some(hi)) = (j.q_min.value, j.q_max.value) else {
            continue;
        };
        let span = (hi - lo).abs();
        if span < 1e-6 {
            continue;
        }
        for f in FRACS {
            let mut q = seed.q.clone();
            q[i] = (seed.q[i] + f * span).clamp(lo, hi);
            if q.iter()
                .zip(seed.q.iter())
                .any(|(a, b)| (a - b).abs() > 1e-12)
            {
                out.push(q);
            }
        }
    }
    for sign in [1.0, -1.0] {
        let mut q = seed.q.clone();
        let mut changed = false;
        for (i, qi) in seed.q.iter().copied().take(3).enumerate() {
            let Some(name) = seed.joint_names.get(i) else {
                continue;
            };
            let Some(j) = joints.iter().find(|j| j.name == *name) else {
                continue;
            };
            let (Some(lo), Some(hi)) = (j.q_min.value, j.q_max.value) else {
                continue;
            };
            let span = (hi - lo).abs();
            if span < 1e-6 {
                continue;
            }
            q[i] = (qi + sign * 0.12 * span).clamp(lo, hi);
            changed = true;
        }
        if changed {
            out.push(q);
        }
    }
    out.truncate(MAX_PERTURBATIONS_PER_SOURCE);
    out
}

struct WitnessAttempt {
    maneuver: Option<ContactManeuver>,
    err: ContactInfeasible,
    n_ik: u64,
    n_ik_attempts: u64,
    complete_chain: bool,
}

fn fail_attempt(err: ContactInfeasible, n_ik: u64, n_ik_attempts: u64) -> WitnessAttempt {
    WitnessAttempt {
        maneuver: None,
        err,
        n_ik,
        n_ik_attempts,
        complete_chain: false,
    }
}

fn attempt_continuing_witness(
    model: &EmbodimentModel,
    ee: &str,
    ik_seed: &SampledEePose,
    geo: &GeometricContact,
    object: BoxObject,
    support: SupportPlane,
    spec: &ContactManeuverSpec,
    cloud: &[SampledEePose],
    start_q: &[f64],
    standoff: f64,
) -> WitnessAttempt {
    let mut n_ik = 0;
    let mut n_ik_attempts = 0;
    let approach_xyz = [
        geo.ee[0] - geo.push[0] * standoff,
        geo.ee[1] - geo.push[1] * standoff,
        geo.ee[2],
    ];
    let mid_xyz = [
        geo.ee[0] + geo.push[0] * spec.min_stroke * 0.5,
        geo.ee[1] + geo.push[1] * spec.min_stroke * 0.5,
        geo.ee[2],
    ];
    let end_xyz = [
        geo.ee[0] + geo.push[0] * spec.min_stroke,
        geo.ee[1] + geo.push[1] * spec.min_stroke,
        geo.ee[2],
    ];
    n_ik_attempts += 1;
    let (contact_s, contact_res) =
        match ik_sample_contact(model, ee, ik_seed, geo, spec, object, support) {
            Ok(s) => {
                n_ik += 1;
                s
            }
            Err(e) => return fail_attempt(e, n_ik, n_ik_attempts),
        };
    n_ik_attempts += 1;
    let (approach, approach_res) = match ik_sample_to_target(
        model,
        ee,
        &contact_s,
        PositionTarget { xyz: approach_xyz },
        "approach",
    ) {
        Ok(s) => {
            n_ik += 1;
            s
        }
        Err(e) => return fail_attempt(e, n_ik, n_ik_attempts),
    };
    n_ik_attempts += 1;
    let (mid, mid_res) = match ik_sample_to_target(
        model,
        ee,
        &contact_s,
        PositionTarget { xyz: mid_xyz },
        "mid",
    ) {
        Ok(s) => {
            n_ik += 1;
            s
        }
        Err(e) => return fail_attempt(e, n_ik, n_ik_attempts),
    };
    n_ik_attempts += 1;
    let (end, end_res) =
        match ik_sample_to_target(model, ee, &mid, PositionTarget { xyz: end_xyz }, "end") {
            Ok(s) => {
                n_ik += 1;
                s
            }
            Err(e) => return fail_attempt(e, n_ik, n_ik_attempts),
        };
    let complete_chain = n_ik == 4;
    let mut local = cloud.to_vec();
    local.push(approach.clone());
    local.push(contact_s.clone());
    local.push(mid.clone());
    local.push(end.clone());
    let planned = |xyz: [f64; 3], q: &[f64]| SampledEePose {
        xyz,
        quat_wxyz: contact_s.quat_wxyz,
        q: q.to_vec(),
        joint_names: contact_s.joint_names.clone(),
    };
    local.push(planned(approach_xyz, &approach.q));
    local.push(planned(mid_xyz, &mid.q));
    local.push(planned(end_xyz, &end.q));
    match evaluate_sampled_push(&contact_s, &local, object, support, spec, &model.joints) {
        Ok(mut m) => {
            let mut contact_phase = ManeuverPhase::positional(
                pose(contact_s.xyz, contact_s.quat_wxyz).unwrap_or(m.contact_pose),
                contact_s.q.clone(),
                approach.q.clone(),
                contact_res,
                m.orientation_error,
            );
            let axis_ok = tool_axis_of(&contact_s, spec).is_some_and(|ax| {
                plane_push(spec.push_direction)
                    .is_some_and(|p| dot3(ax, p) + 1e-12 >= spec.min_align)
            });
            contact_phase.orientation_checked = true;
            contact_phase.full_pose_feasible = axis_ok;
            let mut w = witness_from_continuing_phases(
                contact_s.joint_names.clone(),
                start_q.to_vec(),
                ManeuverPhase::positional(
                    pose(approach.xyz, approach.quat_wxyz).unwrap_or(m.approach_pose),
                    approach.q.clone(),
                    ik_seed.q.clone(),
                    approach_res,
                    m.orientation_error,
                ),
                contact_phase,
                ManeuverPhase::positional(
                    pose(mid.xyz, mid.quat_wxyz).unwrap_or(m.contact_pose),
                    mid.q.clone(),
                    contact_s.q.clone(),
                    mid_res,
                    m.orientation_error,
                ),
                ManeuverPhase::positional(
                    pose(end.xyz, end.quat_wxyz).unwrap_or(m.contact_pose),
                    end.q.clone(),
                    mid.q.clone(),
                    end_res,
                    m.orientation_error,
                ),
                &model.joints,
            );
            apply_collision_admissibility(
                &mut w,
                model,
                ee,
                &collision_world_of(model, ee, object, support, spec),
            );
            m.executable = Some(w);
            m.sampled_q = contact_s.q.clone();
            WitnessAttempt {
                maneuver: Some(m),
                err: ContactInfeasible::NoFeasibleContactPose,
                n_ik,
                n_ik_attempts,
                complete_chain,
            }
        }
        Err(e) => WitnessAttempt {
            maneuver: None,
            err: e,
            n_ik,
            n_ik_attempts,
            complete_chain,
        },
    }
}

fn record_constructed_witness(funnel: &mut ContactSelectFunnel, w: &ExecutableContactManeuver) {
    funnel.n_complete_witnesses += 1;
    let transitions = [
        &w.current_to_approach,
        &w.approach_to_contact,
        &w.contact_to_mid,
        &w.mid_to_end,
    ];
    let margin_ok = transitions.iter().all(|t| {
        t.reason != "JOINT_LIMIT" && t.min_joint_margin + 1e-12 >= MIN_NAMED_JOINT_MARGIN_FRAC
    });
    if margin_ok {
        funnel.n_joint_margin_valid += 1;
    }
    let trans_ok = transitions
        .iter()
        .all(|t| t.verdict == TransitionVerdict::Feasible);
    if trans_ok {
        funnel.n_transition_valid += 1;
    }
    if let Some(reason) = execution_block_reason(w) {
        funnel.last_block_reason = reason;
    } else {
        funnel.n_executable_candidates += 1;
    }
}

fn select_reject_from_funnel(
    funnel: &ContactSelectFunnel,
    last_err: ContactInfeasible,
) -> ContactInfeasible {
    if funnel.n_complete_witnesses > 0 {
        if is_collision_block_reason(&funnel.last_block_reason) {
            return ContactInfeasible::CollisionInadmissible;
        }
        if funnel.last_block_reason == "JOINT_LIMIT" && funnel.n_joint_margin_valid == 0 {
            return ContactInfeasible::InsufficientJointMargin;
        }
        return ContactInfeasible::NoExecutableContactManeuver;
    }
    if last_err == ContactInfeasible::OrientationInfeasible
        || last_err == ContactInfeasible::WrongContactGeometry
    {
        return last_err;
    }
    if funnel.n_geometric_candidates > 0 && funnel.n_complete_ik_chains == 0 {
        return ContactInfeasible::NoIkSolution;
    }
    last_err
}

/// Mode B with seeded IK: refine a nearby `sampled_q` onto the object face.
pub fn select_fixed_world_push_seeded(
    model: &EmbodimentModel,
    ee: &str,
    cloud: &[SampledEePose],
    object: BoxObject,
    support: SupportPlane,
    spec: &ContactManeuverSpec,
) -> Result<(ContactManeuver, RankWhy), ContactInfeasible> {
    select_fixed_world_push_seeded_with_funnel(model, ee, cloud, object, support, spec).0
}

/// Same path as [`select_fixed_world_push_seeded`], with proven funnel counts.
pub fn select_fixed_world_push_seeded_with_funnel(
    model: &EmbodimentModel,
    ee: &str,
    cloud: &[SampledEePose],
    object: BoxObject,
    support: SupportPlane,
    spec: &ContactManeuverSpec,
) -> (
    Result<(ContactManeuver, RankWhy), ContactInfeasible>,
    ContactSelectFunnel,
) {
    let mut funnel = ContactSelectFunnel::default();
    if cloud.is_empty() {
        return (Err(ContactInfeasible::NoFeasibleContactPose), funnel);
    }
    let Some(chain) = model.ee_joint_chain(ee) else {
        return (Err(ContactInfeasible::NoFeasibleContactPose), funnel);
    };
    let standoff = spec
        .approach_standoff
        .max(push_contact_success_radius() + 1e-4);
    let start = cloud
        .first()
        .filter(|s| seed_matches_chain(s, &chain))
        .cloned();
    let mut last_err = ContactInfeasible::NoFeasibleContactPose;
    let mut seen_geo = Vec::new();
    let mut geometries: Vec<(GeometricContact, SampledEePose)> = Vec::new();
    for seed in cloud {
        if !seed_matches_chain(seed, &chain) {
            continue;
        }
        let Ok(geo) = geometric_contact_ee(object, spec, seed, support, cloud) else {
            last_err = ContactInfeasible::WrongContactGeometry;
            continue;
        };
        if dist3(seed.xyz, geo.ee) > IK_SEED_RADIUS_M {
            continue;
        }
        let key = xyz_quantized(geo.ee);
        if seen_geo.contains(&key) {
            continue;
        }
        seen_geo.push(key);
        geometries.push((geo, seed.clone()));
        if geometries.len() >= MAX_UNIQUE_GEOMETRIES {
            break;
        }
    }
    funnel.n_geometric_candidates = geometries.len() as u64;

    let mut executable: Vec<ContactManeuver> = Vec::new();
    let mut seen_witness: Vec<Vec<i64>> = Vec::new();
    let mut perturbation_sources: Vec<SampledEePose> = Vec::new();

    for (geo, _orient) in &geometries {
        let seeds = ordered_ik_seeds(cloud, &chain, geo.ee, &model.joints);
        for seed in seeds {
            let start_q = start.as_ref().map(|s| s.q.as_slice()).unwrap_or(&seed.q);
            let attempt = attempt_continuing_witness(
                model, ee, seed, geo, object, support, spec, cloud, start_q, standoff,
            );
            funnel.n_phase_ik_attempts += attempt.n_ik_attempts;
            funnel.n_phase_ik_successes += attempt.n_ik;
            if attempt.complete_chain {
                funnel.n_complete_ik_chains += 1;
                funnel.n_ik_solutions += 1;
            }
            if attempt.n_ik > 0
                && !perturbation_sources
                    .iter()
                    .any(|s| q_quantized(&s.q) == q_quantized(&seed.q))
            {
                perturbation_sources.push(seed.clone());
            }
            if let Some(m) = attempt.maneuver {
                if let Some(w) = m.executable.as_ref() {
                    let key = q_quantized(&w.contact.q);
                    if seen_witness.iter().any(|k| k == &key) {
                        continue;
                    }
                    seen_witness.push(key);
                    record_constructed_witness(&mut funnel, w);
                    if execution_block_reason(w).is_none() {
                        executable.push(m);
                    }
                }
            } else {
                last_err = attempt.err;
            }
        }
    }

    if executable.is_empty() {
        for (geo, _) in &geometries {
            for src in perturbation_sources.iter().take(MAX_PERTURB_SOURCES) {
                for pq in bounded_seed_perturbations(src, &model.joints) {
                    let Some(pseed) = sample_from_q(model, ee, &chain, pq) else {
                        continue;
                    };
                    if perturbation_sources
                        .iter()
                        .chain(cloud.iter())
                        .any(|s| q_quantized(&s.q) == q_quantized(&pseed.q))
                    {
                        continue;
                    }
                    let start_q = start.as_ref().map(|s| s.q.as_slice()).unwrap_or(&pseed.q);
                    let attempt = attempt_continuing_witness(
                        model, ee, &pseed, geo, object, support, spec, cloud, start_q, standoff,
                    );
                    funnel.n_phase_ik_attempts += attempt.n_ik_attempts;
                    funnel.n_phase_ik_successes += attempt.n_ik;
                    if attempt.complete_chain {
                        funnel.n_complete_ik_chains += 1;
                        funnel.n_ik_solutions += 1;
                    }
                    if let Some(m) = attempt.maneuver {
                        if let Some(w) = m.executable.as_ref() {
                            let key = q_quantized(&w.contact.q);
                            if seen_witness.iter().any(|k| k == &key) {
                                continue;
                            }
                            seen_witness.push(key);
                            record_constructed_witness(&mut funnel, w);
                            if execution_block_reason(w).is_none() {
                                executable.push(m);
                            }
                        }
                    } else {
                        last_err = attempt.err;
                    }
                }
            }
        }
    }

    match select_contact_maneuver(&executable, spec) {
        Some((idx, why)) => {
            funnel.n_selected_executable = 1;
            (Ok((executable[idx].clone(), why)), funnel)
        }
        None => (Err(select_reject_from_funnel(&funnel, last_err)), funnel),
    }
}

/// Evidence-backed pre-contact label. Privileged collision flags are
/// evaluation-only inputs to this classifier, never ranking inputs.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum PreContactTaxonomy {
    NoFeasibleContactPose,
    NoExecutableContactManeuver,
    InsufficientJointMargin,
    OrientationInfeasible,
    InsufficientStrokeWorkspace,
    SupportCollision,
    ApproachNotReached,
    ContactPoseReachedNoPhysicalContact,
    ContactGeometryMismatch,
    Unknown,
}

impl PreContactTaxonomy {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::NoFeasibleContactPose => "NO_FEASIBLE_CONTACT_POSE",
            Self::NoExecutableContactManeuver => "NO_EXECUTABLE_CONTACT_MANEUVER",
            Self::InsufficientJointMargin => "INSUFFICIENT_JOINT_MARGIN",
            Self::OrientationInfeasible => "ORIENTATION_INFEASIBLE",
            Self::InsufficientStrokeWorkspace => "INSUFFICIENT_STROKE_WORKSPACE",
            Self::SupportCollision => "SUPPORT_COLLISION",
            Self::ApproachNotReached => "APPROACH_NOT_REACHED",
            Self::ContactPoseReachedNoPhysicalContact => "CONTACT_POSE_REACHED_NO_PHYSICAL_CONTACT",
            Self::ContactGeometryMismatch => "CONTACT_GEOMETRY_MISMATCH",
            Self::Unknown => "UNKNOWN",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct PreContactEvidence {
    pub positive_reachable: bool,
    pub had_feasible_candidate: bool,
    pub reject: Option<ContactInfeasible>,
    pub approach_reached: bool,
    pub contact_pose_reached: bool,
    pub ee_object_contact: bool,
    pub support_contact_with_ee: bool,
}

pub fn classify_pre_contact(ev: &PreContactEvidence) -> PreContactTaxonomy {
    if ev.ee_object_contact {
        return PreContactTaxonomy::Unknown;
    }
    if !ev.positive_reachable {
        return PreContactTaxonomy::Unknown;
    }
    if !ev.had_feasible_candidate {
        return match ev.reject {
            Some(ContactInfeasible::OrientationInfeasible) => {
                PreContactTaxonomy::OrientationInfeasible
            }
            Some(ContactInfeasible::InsufficientRemainingStroke) => {
                PreContactTaxonomy::InsufficientStrokeWorkspace
            }
            Some(ContactInfeasible::SupportPlaneBlocksEe)
            | Some(ContactInfeasible::ApproachCollidesBeforeContact) => {
                PreContactTaxonomy::SupportCollision
            }
            Some(ContactInfeasible::WrongContactGeometry) => {
                PreContactTaxonomy::ContactGeometryMismatch
            }
            Some(ContactInfeasible::InsufficientJointMargin) => {
                PreContactTaxonomy::InsufficientJointMargin
            }
            Some(ContactInfeasible::NoExecutableContactManeuver)
            | Some(ContactInfeasible::CollisionInadmissible) => {
                PreContactTaxonomy::NoExecutableContactManeuver
            }
            Some(ContactInfeasible::NoIkSolution)
            | Some(ContactInfeasible::NoFeasibleContactPose)
            | Some(ContactInfeasible::ContactPoseUnreachableFromApproach)
            | None => PreContactTaxonomy::NoFeasibleContactPose,
        };
    }
    if !ev.approach_reached {
        return PreContactTaxonomy::ApproachNotReached;
    }
    if ev.contact_pose_reached && !ev.ee_object_contact {
        if ev.support_contact_with_ee {
            return PreContactTaxonomy::SupportCollision;
        }
        return PreContactTaxonomy::ContactPoseReachedNoPhysicalContact;
    }
    PreContactTaxonomy::Unknown
}

/// Tool offset in the EE frame from a world-frame EE→tool vector.
pub fn tool_offset_in_ee(
    ee_quat_wxyz: [f64; 4],
    tool_world: [f64; 3],
    ee_xyz: [f64; 3],
) -> [f64; 3] {
    rotate_by_quat(quat_conj(ee_quat_wxyz), sub3(tool_world, ee_xyz))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample(xyz: [f64; 3], quat: [f64; 4]) -> SampledEePose {
        SampledEePose {
            xyz,
            quat_wxyz: quat,
            q: vec![0.1, 0.2],
            joint_names: vec!["j0".into(), "j1".into()],
        }
    }

    fn identity_quat() -> [f64; 4] {
        [1.0, 0.0, 0.0, 0.0]
    }

    #[test]
    fn short_tool_offset_without_declared_axis_is_unknown_not_aligned() {
        let s = sample([0.2, 0.0, 0.16], identity_quat());
        let spec = ContactManeuverSpec::table_push([1.0, 0.0, 0.0], 0.05, s.xyz, [0.0, 0.0, 0.0]);
        assert!(
            declared_or_derived_tool_axis(&s, &spec).is_none(),
            "missing axis must stay UNKNOWN"
        );
        let object = BoxObject {
            center: [0.30, 0.0, 0.16],
            half_extents: [0.03, 0.03, 0.03],
            quat_wxyz: identity_quat(),
        };
        let support = SupportPlane {
            origin: [0.30, 0.0, 0.13],
            normal: [0.0, 0.0, 1.0],
        };
        assert_eq!(
            prove_contact_manifold(&s, &spec, object, support).unwrap_err(),
            ContactInfeasible::OrientationInfeasible
        );
    }

    #[test]
    fn prove_contact_manifold_accepts_posed_face_and_rejects_aabb_support() {
        let yaw = Se3::from_axis_angle([0.0, 0.0, 1.0], std::f64::consts::FRAC_PI_4).unwrap();
        let object = BoxObject {
            center: [0.30, 0.0, 0.16],
            half_extents: [0.03, 0.03, 0.03],
            quat_wxyz: yaw.quat_wxyz,
        };
        let support = SupportPlane {
            origin: [0.30, 0.0, 0.13],
            normal: [0.0, 0.0, 1.0],
        };
        let spec = spec_at([0.20, 0.0, 0.16], [0.0, 0.0, 0.0]);
        let posed = crate::contact_manifold::box_push_face_manifold_posed(
            object.pose(),
            object.half_extents,
            [1.0, 0.0, 0.0],
            support.normal,
            spec.face_gap,
        )
        .unwrap();
        let posed_tool = add3(posed.origin, scale3(posed.normal, spec.face_gap));
        let aabb_tool = sub3(object.center, scale3([1.0, 0.0, 0.0], 0.03 + spec.face_gap));
        assert!(
            norm3(sub3(posed_tool, aabb_tool)) > 1e-3,
            "precondition: posed face and AABB support must differ"
        );
        let posed_sample = sample(posed_tool, identity_quat());
        prove_contact_manifold(&posed_sample, &spec, object, support)
            .expect("tool on the object-local face must be accepted");
        let aabb_sample = sample(aabb_tool, identity_quat());
        assert!(
            prove_contact_manifold(&aabb_sample, &spec, object, support).is_err(),
            "world-AABB support point must not prove contact on a rotated box"
        );
    }

    #[test]
    fn geometric_contact_ee_aims_at_posed_face_not_world_aabb() {
        let yaw = Se3::from_axis_angle([0.0, 0.0, 1.0], std::f64::consts::FRAC_PI_4).unwrap();
        let object = BoxObject {
            center: [0.30, 0.0, 0.16],
            half_extents: [0.03, 0.03, 0.03],
            quat_wxyz: yaw.quat_wxyz,
        };
        let support = SupportPlane {
            origin: [0.30, 0.0, 0.13],
            normal: [0.0, 0.0, 1.0],
        };
        let spec = spec_at([0.20, 0.0, 0.16], [0.0, 0.0, 0.0]);
        let seed = sample([0.20, 0.0, 0.16], identity_quat());
        let geo = geometric_contact_ee(object, &spec, &seed, support, &[]).unwrap();
        let posed = crate::contact_manifold::box_push_face_manifold_posed(
            object.pose(),
            object.half_extents,
            geo.push,
            support.normal,
            spec.face_gap,
        )
        .unwrap();
        let designed = add3(posed.origin, scale3(posed.normal, spec.face_gap));
        let d = norm3(sub3(geo.tool, designed));
        assert!(
            d < 1e-9,
            "Mode B target must be posed face+gap, tool={:?} designed={:?} d={d}",
            geo.tool,
            designed
        );
        let aabb = sub3(object.center, scale3(geo.push, 0.03 + spec.face_gap));
        assert!(
            norm3(sub3(geo.tool, aabb)) > 1e-3,
            "Mode B must not aim at the world-AABB support"
        );
    }

    fn rot_z_90() -> [f64; 4] {
        Se3::from_axis_angle([0.0, 0.0, 1.0], std::f64::consts::FRAC_PI_2)
            .unwrap()
            .quat_wxyz
    }

    fn spec_at(current: [f64; 3], tool_ee: [f64; 3]) -> ContactManeuverSpec {
        let mut spec = ContactManeuverSpec::table_push([1.0, 0.0, 0.0], 0.05, current, tool_ee);
        // Tests that are not about a missing axis declare the EE-frame approach.
        spec.declared_tool_axis_ee = Some([1.0, 0.0, 0.0]);
        spec
    }

    fn support_under(center: [f64; 3], half_z: f64) -> SupportPlane {
        SupportPlane {
            origin: [center[0], center[1], center[2] - half_z],
            normal: [0.0, 0.0, 1.0],
        }
    }

    #[test]
    fn xyz_reachable_but_orientation_infeasible_is_rejected() {
        let xyz = [0.25, 0.0, 0.16];
        let bad = sample(xyz, rot_z_90());
        let ahead = sample([0.36, 0.0, 0.16], identity_quat());
        let tool = [0.04, 0.0, 0.0];
        let spec = spec_at([0.20, 0.0, 0.16], tool);
        let object = BoxObject {
            center: [0.335, 0.0, 0.16],
            half_extents: [0.03, 0.03, 0.03],
            quat_wxyz: [1.0, 0.0, 0.0, 0.0],
        };
        let support = support_under(object.center, 0.03);
        let err = evaluate_sampled_push(&bad, &[bad.clone(), ahead], object, support, &spec, &[])
            .unwrap_err();
        assert_eq!(err, ContactInfeasible::OrientationInfeasible);
    }

    #[test]
    fn support_plane_blocking_ee_is_rejected() {
        let s = sample([0.25, 0.0, 0.16], identity_quat());
        let ahead = sample([0.36, 0.0, 0.16], identity_quat());
        let mut spec = spec_at([0.20, 0.0, 0.16], [0.0, 0.0, 0.0]);
        spec.ee_radius = 0.04;
        spec.min_support_clearance = 0.01;
        let object = BoxObject {
            center: [0.29, 0.0, 0.16],
            half_extents: [0.03, 0.03, 0.03],
            quat_wxyz: [1.0, 0.0, 0.0, 0.0],
        };
        let support = SupportPlane {
            origin: [0.29, 0.0, 0.13],
            normal: [0.0, 0.0, 1.0],
        };
        let err = evaluate_sampled_push(&s, &[s.clone(), ahead], object, support, &spec, &[])
            .unwrap_err();
        assert_eq!(err, ContactInfeasible::SupportPlaneBlocksEe);
    }

    #[test]
    fn insufficient_remaining_stroke_is_rejected() {
        let s = sample([0.25, 0.0, 0.16], identity_quat());
        let spec = spec_at([0.20, 0.0, 0.16], [0.0, 0.0, 0.0]);
        let object = BoxObject {
            center: [0.29, 0.0, 0.16],
            half_extents: [0.03, 0.03, 0.03],
            quat_wxyz: [1.0, 0.0, 0.0, 0.0],
        };
        let support = support_under(object.center, 0.03);
        let err = evaluate_sampled_push(&s, std::slice::from_ref(&s), object, support, &spec, &[])
            .unwrap_err();
        assert_eq!(err, ContactInfeasible::InsufficientRemainingStroke);
    }

    #[test]
    fn valid_approach_contact_stroke_is_accepted() {
        let contact = sample([0.25, 0.0, 0.16], identity_quat());
        let ahead = sample([0.36, 0.0, 0.16], identity_quat());
        let spec = spec_at([0.22, 0.0, 0.16], [0.04, 0.0, 0.0]);
        let center = object_center_for_sample(
            &contact,
            spec.tool_offset_ee,
            spec.push_direction,
            [0.03, 0.03, 0.03],
            spec.face_gap,
        )
        .unwrap();
        let object = BoxObject {
            center,
            half_extents: [0.03, 0.03, 0.03],
            quat_wxyz: [1.0, 0.0, 0.0, 0.0],
        };
        let support = support_under(center, 0.03);
        let m = evaluate_sampled_push(
            &contact,
            &[contact.clone(), ahead],
            object,
            support,
            &spec,
            &[],
        )
        .expect("valid maneuver");
        assert!(m.available_stroke + 1e-9 >= spec.min_stroke);
        let travel = dist3(m.approach_pose.xyz, m.contact_pose.xyz);
        assert!(
            travel > push_contact_success_radius(),
            "approach must travel to contact, travel={travel}"
        );
        assert!(m.support_clearance + 1e-12 >= spec.min_support_clearance);
        assert!((m.contact_pose.xyz[0] - 0.25).abs() < 1e-12);
        assert!(m.push_direction[2].abs() < 1e-12);
    }

    #[test]
    fn ranking_is_deterministic_and_identity_free() {
        let a = sample([0.24, 0.01, 0.16], identity_quat());
        let b = sample([0.26, -0.01, 0.16], identity_quat());
        let ahead = sample([0.40, 0.0, 0.16], identity_quat());
        let spec = spec_at([0.20, 0.0, 0.16], [0.0, 0.0, 0.0]);
        let half = [0.03, 0.03, 0.03];
        let cloud = [a, b, ahead];
        let (first, why1) =
            select_push_maneuver(&cloud, &spec, half, [0.0, 0.0, 1.0], &[]).unwrap();
        let (second, why2) =
            select_push_maneuver(&cloud, &spec, half, [0.0, 0.0, 1.0], &[]).unwrap();
        assert_eq!(first.contact_pose.xyz, second.contact_pose.xyz);
        assert_eq!(why1.score, why2.score);
        let keys: Vec<_> = serde_json::to_value(&why1.inputs)
            .unwrap()
            .as_object()
            .unwrap()
            .keys()
            .cloned()
            .collect();
        for k in &keys {
            let lower = k.to_ascii_lowercase();
            assert!(
                rank_inputs_field_names().contains(&k.as_str()),
                "unexpected rank key {k}"
            );
            assert!(!lower.contains("robot"));
            assert!(!lower.contains("vendor"));
            assert!(!lower.contains("urdf"));
            assert!(!lower.contains("bundle"));
            assert!(!lower.contains("dataset"));
        }
        assert_eq!(keys.len(), rank_inputs_field_names().len());
    }

    #[test]
    fn select_prefers_orientation_feasible_over_xyz_only() {
        let bad = sample([0.22, 0.0, 0.16], rot_z_90());
        let good = sample([0.28, 0.0, 0.16], identity_quat());
        let ahead = sample([0.40, 0.0, 0.16], identity_quat());
        let spec = spec_at([0.20, 0.0, 0.16], [0.04, 0.0, 0.0]);
        let (m, _) = select_push_maneuver(
            &[bad, good.clone(), ahead],
            &spec,
            [0.03, 0.03, 0.03],
            [0.0, 0.0, 1.0],
            &[],
        )
        .unwrap();
        assert!((m.contact_pose.xyz[0] - good.xyz[0]).abs() < 1e-12);
    }

    #[test]
    fn pre_contact_unknown_when_evidence_cannot_prove_cause() {
        let ev = PreContactEvidence {
            positive_reachable: true,
            had_feasible_candidate: true,
            reject: None,
            approach_reached: true,
            contact_pose_reached: false,
            ee_object_contact: false,
            support_contact_with_ee: false,
        };
        assert_eq!(classify_pre_contact(&ev), PreContactTaxonomy::Unknown);
    }

    #[test]
    fn pre_contact_uses_reject_when_no_feasible_candidate() {
        let ev = PreContactEvidence {
            positive_reachable: true,
            had_feasible_candidate: false,
            reject: Some(ContactInfeasible::OrientationInfeasible),
            approach_reached: false,
            contact_pose_reached: false,
            ee_object_contact: false,
            support_contact_with_ee: false,
        };
        assert_eq!(
            classify_pre_contact(&ev),
            PreContactTaxonomy::OrientationInfeasible
        );
    }

    #[test]
    fn approach_half_meter_match_is_not_feasibility() {
        let contact = sample([0.25, 0.0, 0.16], identity_quat());
        let ahead = sample([0.36, 0.0, 0.16], identity_quat());
        let spec = spec_at([0.75, 0.0, 0.16], [0.0, 0.0, 0.0]);
        let object = BoxObject {
            center: [0.29, 0.0, 0.16],
            half_extents: [0.03, 0.03, 0.03],
            quat_wxyz: [1.0, 0.0, 0.0, 0.0],
        };
        let support = support_under(object.center, 0.03);
        let err = evaluate_sampled_push(
            &contact,
            &[contact.clone(), ahead],
            object,
            support,
            &spec,
            &[],
        )
        .unwrap_err();
        assert_eq!(
            err,
            ContactInfeasible::ContactPoseUnreachableFromApproach,
            "0.55 m current-EE proximity must not count as approach feasibility"
        );
    }

    #[test]
    fn xyz_contact_without_approach_or_stroke_is_rejected() {
        let contact = sample([0.25, 0.0, 0.16], identity_quat());
        let spec = spec_at([0.90, 0.0, 0.16], [0.0, 0.0, 0.0]);
        let object = BoxObject {
            center: [0.29, 0.0, 0.16],
            half_extents: [0.03, 0.03, 0.03],
            quat_wxyz: [1.0, 0.0, 0.0, 0.0],
        };
        let support = support_under(object.center, 0.03);
        let err = evaluate_sampled_push(
            &contact,
            std::slice::from_ref(&contact),
            object,
            support,
            &spec,
            &[],
        )
        .unwrap_err();
        assert!(
            matches!(
                err,
                ContactInfeasible::ContactPoseUnreachableFromApproach
                    | ContactInfeasible::InsufficientRemainingStroke
            ),
            "XYZ-only reachability is not a feasible maneuver, got {err:?}"
        );
    }

    fn test_joint(name: &str, lo: f64, hi: f64) -> Joint {
        use crate::embodiment::{unknown_se3, JointKind};
        use crate::provenance::Provenanced;
        Joint {
            name: name.into(),
            kind: JointKind::Hinge,
            axis: Provenanced::declared([0.0, 0.0, 1.0], "test", 0.0),
            qpos_dim: 1,
            dof_dim: 1,
            parent_body: "p".into(),
            child_body: name.into(),
            q_min: Provenanced::declared(lo, "test", 0.0),
            q_max: Provenanced::declared(hi, "test", 0.0),
            dq_max: Provenanced::unknown("test", 0.0),
            effort_max: Provenanced::unknown("test", 0.0),
            origin_in_child: Provenanced::declared([0.0, 0.0, 0.0], "test", 0.0),
            parent_to_joint: unknown_se3("test"),
            joint_to_child: unknown_se3("test"),
            qpos_adr: None,
            dof_adr: None,
        }
    }

    #[test]
    fn joint_margin_uses_named_joints_not_model_order() {
        let joints = vec![
            test_joint("finger_a", 0.0, 0.04),
            test_joint("j0", -2.0, 2.0),
            test_joint("j1", -2.0, 2.0),
        ];
        let contact = SampledEePose {
            xyz: [0.25, 0.0, 0.16],
            quat_wxyz: identity_quat(),
            q: vec![0.5, 0.4],
            joint_names: vec!["j0".into(), "j1".into()],
        };
        let ahead = sample([0.36, 0.0, 0.16], identity_quat());
        let spec = spec_at([0.22, 0.0, 0.16], [0.04, 0.0, 0.0]);
        let center = object_center_for_sample(
            &contact,
            spec.tool_offset_ee,
            spec.push_direction,
            [0.03, 0.03, 0.03],
            spec.face_gap,
        )
        .unwrap();
        let object = BoxObject {
            center,
            half_extents: [0.03, 0.03, 0.03],
            quat_wxyz: [1.0, 0.0, 0.0, 0.0],
        };
        let support = support_under(center, 0.03);
        let m = evaluate_sampled_push(
            &contact,
            &[contact.clone(), ahead],
            object,
            support,
            &spec,
            &joints,
        )
        .expect("feasible");
        assert!(
            m.joint_margin > 0.2,
            "chain q must not be scored against finger [0,0.04] by index, margin={}",
            m.joint_margin
        );
    }

    #[test]
    fn fixed_world_select_does_not_move_object() {
        let contact = sample([0.25, 0.0, 0.16], identity_quat());
        let ahead = sample([0.36, 0.0, 0.16], identity_quat());
        let spec = spec_at([0.22, 0.0, 0.16], [0.04, 0.0, 0.0]);
        let object = BoxObject {
            center: [0.305, 0.0, 0.16],
            half_extents: [0.03, 0.03, 0.03],
            quat_wxyz: [1.0, 0.0, 0.0, 0.0],
        };
        let support = support_under(object.center, 0.03);
        let (m, _) = select_fixed_world_push(&[contact, ahead], object, support, &spec, &[])
            .expect("fixed-world feasible");
        assert!(
            (m.object_center[0] - 0.305).abs() < 1e-12,
            "Mode B must keep object center, got {:?}",
            m.object_center
        );
        assert!((m.object_center[1] - 0.0).abs() < 1e-12);
        assert!((m.object_center[2] - 0.16).abs() < 1e-12);
    }

    #[test]
    fn fixed_world_tool_not_on_face_is_rejected_without_ik() {
        let near = sample([0.34, 0.0, 0.16], identity_quat());
        let ahead = sample([0.46, 0.0, 0.16], identity_quat());
        let spec = spec_at([0.22, 0.0, 0.16], [0.04, 0.0, 0.0]);
        let object = BoxObject {
            center: [0.52, 0.0, 0.16],
            half_extents: [0.03, 0.03, 0.03],
            quat_wxyz: [1.0, 0.0, 0.0, 0.0],
        };
        let support = support_under(object.center, 0.03);
        let err = select_fixed_world_push(&[near, ahead], object, support, &spec, &[]).unwrap_err();
        assert!(
            matches!(
                err,
                ContactInfeasible::WrongContactGeometry
                    | ContactInfeasible::NoFeasibleContactPose
                    | ContactInfeasible::ContactPoseUnreachableFromApproach
            ),
            "a sample 0.14 m off the face is not a feasible contact, got {err:?}"
        );
    }

    #[test]
    fn seeded_ik_moves_tool_onto_the_object_face() {
        use crate::adapter::synth_planar_two_link;
        use crate::kinematics::forward_kinematics;

        let model = synth_planar_two_link();
        let chain = model.ee_joint_chain("ee").unwrap();
        let q_contact = vec![0.5, 0.4];
        let fk_c = forward_kinematics(&model, &chain, "ee", &q_contact).unwrap();
        let q0 = vec![0.35, 0.25];
        let fk0 = forward_kinematics(&model, &chain, "ee", &q0).unwrap();
        let seed = SampledEePose {
            xyz: fk0.ee.xyz,
            quat_wxyz: fk0.ee.quat_wxyz,
            q: q0,
            joint_names: chain.clone(),
        };
        let ahead_q = vec![0.0, 0.0];
        let fka = forward_kinematics(&model, &chain, "ee", &ahead_q).unwrap();
        let ahead = SampledEePose {
            xyz: fka.ee.xyz,
            quat_wxyz: fka.ee.quat_wxyz,
            q: ahead_q,
            joint_names: chain,
        };
        let push = [1.0, 0.0, 0.0];
        let half = [0.03, 0.03, 0.03];
        let face = half_along_push(half, push) + 0.015;
        let object = BoxObject {
            center: [fk_c.ee.xyz[0] + face, fk_c.ee.xyz[1], fk_c.ee.xyz[2]],
            half_extents: half,
            quat_wxyz: [1.0, 0.0, 0.0, 0.0],
        };
        let mut spec = spec_at(seed.xyz, [0.0, 0.0, 0.0]);
        spec.min_stroke = 0.005;
        spec.requested_stroke = 0.01;
        let support = support_under(object.center, 0.03);
        let cloud = [seed.clone(), ahead];
        let no_ik = select_fixed_world_push(&cloud, object, support, &spec, &model.joints);
        assert!(
            no_ik.is_err(),
            "without IK the off-face seed must not count as contact, got {no_ik:?}"
        );
        let (m, _) = select_fixed_world_push_seeded(&model, "ee", &cloud, object, support, &spec)
            .expect("seeded IK must put the tool on the fixed object face");
        assert!(
            (m.object_center[0] - object.center[0]).abs() < 1e-12,
            "object must stay put"
        );
        let expected_tool_x = object.center[0] - face;
        assert!(
            (m.contact_point[0] - expected_tool_x).abs() < 0.02,
            "tool must land on the face, tool={} expected={} seed_ee={}",
            m.contact_point[0],
            expected_tool_x,
            seed.xyz[0],
        );
        assert!(
            (m.contact_pose.xyz[0] - seed.xyz[0]).abs() > 0.02,
            "IK must move off the seed, seed={} contact={}",
            seed.xyz[0],
            m.contact_pose.xyz[0]
        );
        let w = m.executable.expect("phase q must be stored on the witness");
        assert_eq!(w.start_q, seed.q, "current q is the interpolation start");
        assert_eq!(w.approach.seed_q, seed.q, "approach IK prefers current q");
        assert_eq!(
            w.contact.seed_q, w.approach.q,
            "contact IK must continue from approach q"
        );
        assert_eq!(w.mid_stroke.seed_q, w.contact.q);
        assert_eq!(w.end_stroke.seed_q, w.mid_stroke.q);
        assert!(!w.approach.q.is_empty());
        assert!(!w.approach.full_pose_feasible);
        assert!(w.approach.position_feasible);
        assert!(w.contact.orientation_checked);
        assert!(w.contact.full_pose_feasible);
        assert!(ik_residual_is_precise(w.contact.residual));
    }

    #[test]
    fn synthesis_mode_is_distinct_from_fixed_world() {
        assert_ne!(
            WorldConstructionMode::CapabilitySynthesis,
            WorldConstructionMode::FixedWorld
        );
        let saw = with_world_construction_mode(WorldConstructionMode::CapabilitySynthesis, || {
            current_world_construction_mode()
        });
        assert_eq!(saw, WorldConstructionMode::CapabilitySynthesis);
        assert_eq!(
            current_world_construction_mode(),
            WorldConstructionMode::FixedWorld
        );
    }

    fn planar_limited(j0: [f64; 2], j1: [f64; 2]) -> crate::embodiment::EmbodimentModel {
        use crate::provenance::Provenanced;
        let mut m = crate::adapter::synth_planar_two_link();
        for j in &mut m.joints {
            let (lo, hi) = if j.name == "j0" {
                (j0[0], j0[1])
            } else if j.name == "j1" {
                (j1[0], j1[1])
            } else {
                continue;
            };
            j.q_min = Provenanced::declared(lo, "test", 0.0);
            j.q_max = Provenanced::declared(hi, "test", 0.0);
        }
        m
    }

    fn seeded_push_world(
        model: &crate::embodiment::EmbodimentModel,
        seed_qs: &[Vec<f64>],
    ) -> (
        Vec<SampledEePose>,
        BoxObject,
        SupportPlane,
        ContactManeuverSpec,
    ) {
        use crate::kinematics::forward_kinematics;
        let chain = model.ee_joint_chain("ee").unwrap();
        let q_contact = vec![0.5, 0.4];
        let fk_c = forward_kinematics(model, &chain, "ee", &q_contact).unwrap();
        let mut cloud = Vec::new();
        for q in seed_qs {
            let fk = forward_kinematics(model, &chain, "ee", q).unwrap();
            cloud.push(SampledEePose {
                xyz: fk.ee.xyz,
                quat_wxyz: fk.ee.quat_wxyz,
                q: q.clone(),
                joint_names: chain.clone(),
            });
        }
        let ahead_q = vec![0.0, 0.0];
        let fka = forward_kinematics(model, &chain, "ee", &ahead_q).unwrap();
        cloud.push(SampledEePose {
            xyz: fka.ee.xyz,
            quat_wxyz: fka.ee.quat_wxyz,
            q: ahead_q,
            joint_names: chain,
        });
        let push = [1.0, 0.0, 0.0];
        let half = [0.03, 0.03, 0.03];
        let face = half_along_push(half, push) + 0.015;
        let object = BoxObject {
            center: [fk_c.ee.xyz[0] + face, fk_c.ee.xyz[1], fk_c.ee.xyz[2]],
            half_extents: half,
            quat_wxyz: [1.0, 0.0, 0.0, 0.0],
        };
        let mut spec = spec_at(cloud[0].xyz, [0.0, 0.0, 0.0]);
        spec.min_stroke = 0.005;
        spec.requested_stroke = 0.01;
        let support = support_under(object.center, 0.03);
        (cloud, object, support, spec)
    }

    fn maneuver_with_witness(
        sampled_margin: f64,
        w: crate::maneuver_witness::ExecutableContactManeuver,
    ) -> ContactManeuver {
        ContactManeuver {
            contact_pose: pose([0.25, 0.0, 0.16], identity_quat()).unwrap(),
            approach_pose: pose([0.20, 0.0, 0.16], identity_quat()).unwrap(),
            contact_point: [0.25, 0.0, 0.16],
            contact_normal: [-1.0, 0.0, 0.0],
            push_direction: [1.0, 0.0, 0.0],
            requested_stroke: 0.05,
            available_stroke: 0.05,
            support_clearance: 0.04,
            joint_margin: sampled_margin,
            orientation_error: 0.0,
            object_center: [0.30, 0.0, 0.16],
            support_top_z: 0.13,
            sampled_q: w.contact.q.clone(),
            executable: Some(w),
        }
    }

    fn named_phase(q: Vec<f64>, seed: Vec<f64>) -> crate::maneuver_witness::ManeuverPhase {
        crate::maneuver_witness::ManeuverPhase::positional(Se3::identity(), q, seed, 1e-4, 0.0)
    }

    #[test]
    fn complete_witness_failing_joint_limit_is_not_selected_executable() {
        use crate::maneuver_witness::execution_block_reason;
        let model = planar_limited([-2.5, 2.5], [0.38, 1.0]);
        let (cloud, object, support, spec) = seeded_push_world(&model, &[vec![0.5, 0.4]]);
        let (res, funnel) = select_fixed_world_push_seeded_with_funnel(
            &model, "ee", &cloud, object, support, &spec,
        );
        match res {
            Ok((m, _)) => {
                if let Some(w) = m.executable.as_ref() {
                    assert!(
                        execution_block_reason(w).is_none(),
                        "selected witness must be executable, block={:?}",
                        execution_block_reason(w)
                    );
                    let min_m = crate::maneuver_witness::witness_min_joint_margin(w);
                    assert!(
                        min_m + 1e-12 >= crate::command_domain::MIN_NAMED_JOINT_MARGIN_FRAC,
                        "selected executable must meet interior margin, min={min_m}"
                    );
                } else {
                    panic!("selected maneuver must carry an executable witness");
                }
            }
            Err(e) => {
                assert_ne!(
                    e,
                    ContactInfeasible::NoFeasibleContactPose,
                    "geometry/IK existence must not collapse to NO_FEASIBLE_CONTACT_POSE"
                );
                if funnel.n_complete_witnesses >= 1 {
                    assert!(
                        e == ContactInfeasible::InsufficientJointMargin
                            || e == ContactInfeasible::NoExecutableContactManeuver
                            || e == ContactInfeasible::CollisionInadmissible,
                        "distinct execution-feasibility label, got {e:?}"
                    );
                    assert_eq!(funnel.n_executable_candidates, 0);
                    assert_eq!(funnel.n_selected_executable, 0);
                } else {
                    assert!(
                        e == ContactInfeasible::NoIkSolution
                            || e == ContactInfeasible::InsufficientJointMargin
                            || e == ContactInfeasible::NoExecutableContactManeuver,
                        "distinct infeasibility label, got {e:?} funnel={funnel:?}"
                    );
                }
            }
        }
    }

    #[test]
    fn bound_resting_seed_does_not_hide_interior_executable_branch() {
        use crate::command_domain::{named_joint_limit_margin, MIN_NAMED_JOINT_MARGIN_FRAC};
        use crate::kinematics::forward_kinematics;
        use crate::maneuver_witness::{execution_block_reason, witness_min_joint_margin};
        let model_bound = planar_limited([-2.5, 2.5], [-2.0, 1.0]);
        let model = planar_limited([-2.5, 2.5], [-2.0, 2.0]);
        let chain = model_bound.ee_joint_chain("ee").unwrap();
        let q_bound = vec![0.9, 1.0];
        let fk_bound = forward_kinematics(&model_bound, &chain, "ee", &q_bound).unwrap();
        let (q_alt, trace) = crate::kinematics::solve_ik(
            &model_bound,
            &chain,
            "ee",
            fk_bound.ee.xyz,
            &[1.85, -0.95],
        )
        .expect("alternate basin must solve");
        assert!(
            trace.residual <= 0.08,
            "alternate seed must meet Cartesian residual, residual={}",
            trace.residual
        );
        let bound_margin = named_joint_limit_margin(&q_bound, &chain, &model_bound.joints);
        let alt_margin = named_joint_limit_margin(&q_alt, &chain, &model_bound.joints);
        assert!(
            bound_margin + 1e-12 < MIN_NAMED_JOINT_MARGIN_FRAC,
            "bound seed must rest at/near a limit, margin={bound_margin}"
        );
        assert!(
            alt_margin + 1e-12 >= MIN_NAMED_JOINT_MARGIN_FRAC,
            "alternate seed must be interior, margin={alt_margin}"
        );
        let (cloud, object, support, spec) =
            seeded_push_world(&model, &[vec![0.5, 0.4], q_bound.clone()]);
        let (res, funnel) = select_fixed_world_push_seeded_with_funnel(
            &model, "ee", &cloud, object, support, &spec,
        );
        let (m, _) = res.unwrap_or_else(|e| {
            panic!("interior executable witness must be selected: {e:?} funnel={funnel:?}")
        });
        let w = m
            .executable
            .as_ref()
            .expect("selected maneuver must store a witness");
        assert!(
            execution_block_reason(w).is_none(),
            "selected witness must be executable, block={:?}",
            execution_block_reason(w)
        );
        let min_m = witness_min_joint_margin(w);
        assert!(
            min_m + 1e-12 >= MIN_NAMED_JOINT_MARGIN_FRAC,
            "selected witness min margin {min_m} must be interior"
        );
        assert!(
            funnel.n_complete_witnesses >= 1,
            "branch search must construct witnesses, funnel={funnel:?}"
        );
        assert!(
            funnel.n_executable_candidates >= 1,
            "interior branch must enter the executable pool, funnel={funnel:?}"
        );
        assert_eq!(funnel.n_selected_executable, 1);
        assert_eq!(
            funnel.deepest_proven_layer(),
            Some(ContactFeasibilityLayer::SelectedExecutableManeuver)
        );
    }

    #[test]
    fn ranking_uses_actual_witness_min_margin_not_sampled_contact() {
        use crate::maneuver_witness::{
            witness_from_continuing_phases, witness_min_joint_margin, MIN_NAMED_JOINT_MARGIN_FRAC,
        };
        let names = vec!["arm0".into()];
        let joints = vec![test_joint("arm0", -1.0, 1.0)];
        let w_low = witness_from_continuing_phases(
            names.clone(),
            vec![-0.96],
            named_phase(vec![-0.96], vec![-0.96]),
            named_phase(vec![-0.96], vec![-0.96]),
            named_phase(vec![-0.96], vec![-0.96]),
            named_phase(vec![-0.96], vec![-0.96]),
            &joints,
        );
        let w_high = witness_from_continuing_phases(
            names,
            vec![0.10],
            named_phase(vec![0.10], vec![0.10]),
            named_phase(vec![0.10], vec![0.10]),
            named_phase(vec![0.10], vec![0.10]),
            named_phase(vec![0.10], vec![0.10]),
            &joints,
        );
        assert!(w_low.is_executable() && w_high.is_executable());
        let low_w = witness_min_joint_margin(&w_low);
        let high_w = witness_min_joint_margin(&w_high);
        assert!(low_w + 1e-12 >= MIN_NAMED_JOINT_MARGIN_FRAC);
        assert!(high_w > low_w + 0.1);
        let worse_sampled = maneuver_with_witness(0.40, w_low);
        let better_witness = maneuver_with_witness(0.05, w_high);
        let spec = spec_at([0.20, 0.0, 0.16], [0.0, 0.0, 0.0]);
        let sampled_prefers_first = rank_score(&rank_inputs_of(
            &ContactManeuver {
                executable: None,
                ..worse_sampled.clone()
            },
            &spec,
        )) > rank_score(&rank_inputs_of(
            &ContactManeuver {
                executable: None,
                ..better_witness.clone()
            },
            &spec,
        ));
        assert!(
            sampled_prefers_first,
            "precondition: sampled-contact margin would prefer the worse witness"
        );
        let (idx, why) =
            select_contact_maneuver(&[worse_sampled, better_witness], &spec).expect("rank");
        assert_eq!(idx, 1, "selection must follow actual witness min margin");
        assert!((why.inputs.joint_margin - high_w).abs() < 1e-9);
    }

    #[test]
    fn all_complete_witnesses_failing_execution_are_not_no_feasible_contact_pose() {
        let model = planar_limited([-2.5, 2.5], [0.38, 0.42]);
        let (cloud, object, support, spec) = seeded_push_world(&model, &[vec![0.5, 0.4]]);
        let (res, funnel) = select_fixed_world_push_seeded_with_funnel(
            &model, "ee", &cloud, object, support, &spec,
        );
        assert!(res.is_err(), "must not select a non-executable witness");
        let e = res.unwrap_err();
        assert_ne!(e, ContactInfeasible::NoFeasibleContactPose);
        assert!(
            e == ContactInfeasible::InsufficientJointMargin
                || e == ContactInfeasible::NoExecutableContactManeuver
                || e == ContactInfeasible::NoIkSolution
                || e == ContactInfeasible::CollisionInadmissible,
            "proven non-executable must keep a distinct label, got {e:?} funnel={funnel:?}"
        );
        if funnel.n_geometric_candidates > 0 && funnel.n_complete_witnesses > 0 {
            assert_eq!(funnel.n_executable_candidates, 0);
            assert_eq!(funnel.n_selected_executable, 0);
        }
        assert!(
            funnel.n_geometric_candidates > 0 || funnel.n_ik_solutions > 0,
            "must prove geometry or IK existed, funnel={funnel:?}"
        );
    }

    #[test]
    fn select_skips_complete_witness_with_joint_limit_when_executable_exists() {
        use crate::maneuver_witness::witness_from_continuing_phases;
        let names = vec!["arm0".into()];
        let joints = vec![test_joint("arm0", -1.0, 1.0)];
        let blocked = witness_from_continuing_phases(
            names.clone(),
            vec![0.0],
            named_phase(vec![1.0], vec![0.0]),
            named_phase(vec![1.0], vec![1.0]),
            named_phase(vec![1.0], vec![1.0]),
            named_phase(vec![1.0], vec![1.0]),
            &joints,
        );
        assert!(!blocked.is_executable());
        let ok = witness_from_continuing_phases(
            names,
            vec![0.0],
            named_phase(vec![0.1], vec![0.0]),
            named_phase(vec![0.1], vec![0.1]),
            named_phase(vec![0.1], vec![0.1]),
            named_phase(vec![0.1], vec![0.1]),
            &joints,
        );
        assert!(ok.is_executable());
        let spec = spec_at([0.20, 0.0, 0.16], [0.0, 0.0, 0.0]);
        let (idx, _) = select_contact_maneuver(
            &[
                maneuver_with_witness(0.9, blocked),
                maneuver_with_witness(0.1, ok),
            ],
            &spec,
        )
        .expect("executable remains");
        assert_eq!(idx, 1);
    }

    #[test]
    fn loose_best_effort_ik_is_not_contact_phase_success() {
        use crate::kinematics::solve_ik;
        let model = crate::adapter::synth_planar_two_link();
        let chain = model.ee_joint_chain("ee").unwrap();
        let q_seed = vec![0.0, 0.0];
        let fk = crate::kinematics::forward_kinematics(&model, &chain, "ee", &q_seed).unwrap();
        let far = [fk.ee.xyz[0] + 0.04, fk.ee.xyz[1], fk.ee.xyz[2]];
        let (_q, trace) = solve_ik(&model, &chain, "ee", far, &q_seed).expect("best-effort exists");
        assert!(
            trace.residual > IK_ACCEPT_M && trace.residual <= 0.08,
            "precondition: residual in (1e-3, 0.08], got {}",
            trace.residual
        );
        let seed = SampledEePose {
            xyz: fk.ee.xyz,
            quat_wxyz: fk.ee.quat_wxyz,
            q: q_seed,
            joint_names: chain.clone(),
        };
        let ahead_q = vec![0.2, 0.2];
        let fka = crate::kinematics::forward_kinematics(&model, &chain, "ee", &ahead_q).unwrap();
        let ahead = SampledEePose {
            xyz: fka.ee.xyz,
            quat_wxyz: fka.ee.quat_wxyz,
            q: ahead_q,
            joint_names: chain,
        };
        let push = [1.0, 0.0, 0.0];
        let half = [0.03, 0.03, 0.03];
        let face = half_along_push(half, push) + 0.015;
        let object = BoxObject {
            center: [far[0] + face, far[1], far[2]],
            half_extents: half,
            quat_wxyz: [1.0, 0.0, 0.0, 0.0],
        };
        let mut spec = spec_at(seed.xyz, [0.0, 0.0, 0.0]);
        spec.min_stroke = 0.005;
        spec.requested_stroke = 0.01;
        let support = support_under(object.center, 0.03);
        let ((res, funnel), recs) = with_phase_ik_records(|| {
            select_fixed_world_push_seeded_with_funnel(
                &model,
                "ee",
                &[seed, ahead],
                object,
                support,
                &spec,
            )
        });
        let contact_recs: Vec<_> = recs.iter().filter(|r| r.phase == "contact").collect();
        assert!(
            contact_recs.iter().all(|r| !r.accepted || r.precise),
            "a residual between precise IK and 0.08 must not be accepted, recs={contact_recs:?}"
        );
        if let Ok((m, _)) = &res {
            if let Some(w) = m.executable.as_ref() {
                assert!(
                    ik_residual_is_precise(w.contact.residual),
                    "selected contact residual must be precise, got {}",
                    w.contact.residual
                );
            }
        } else {
            assert_ne!(
                res.clone().unwrap_err(),
                ContactInfeasible::NoFeasibleContactPose
            );
            assert_eq!(funnel.n_selected_executable, 0);
        }
        assert!(
            funnel.n_phase_ik_attempts >= funnel.n_phase_ik_successes,
            "funnel={funnel:?}"
        );
        assert!(
            funnel.n_complete_ik_chains <= funnel.n_phase_ik_successes,
            "complete chains must not be counted as per-phase successes, funnel={funnel:?}"
        );
        assert_eq!(
            funnel.n_ik_solutions, funnel.n_complete_ik_chains,
            "n_ik_solutions must mean complete chains, funnel={funnel:?}"
        );
    }

    #[test]
    fn funnel_does_not_treat_phase_ik_as_complete_maneuver() {
        let funnel = ContactSelectFunnel {
            n_geometric_candidates: 2,
            n_phase_ik_attempts: 8,
            n_phase_ik_successes: 3,
            n_complete_ik_chains: 0,
            n_ik_solutions: 0,
            n_complete_witnesses: 0,
            ..ContactSelectFunnel::default()
        };
        assert_eq!(
            funnel.deepest_proven_layer(),
            Some(ContactFeasibilityLayer::GeometricCandidate)
        );
        assert_ne!(funnel.n_phase_ik_successes, 0);
        assert_eq!(funnel.n_ik_solutions, 0);
    }

    #[test]
    fn phase2_adversarial_contacts_are_rejected_by_shipped_evaluator() {
        let object = BoxObject {
            center: [0.30, 0.0, 0.16],
            half_extents: [0.03, 0.03, 0.03],
            quat_wxyz: [1.0, 0.0, 0.0, 0.0],
        };
        let support = support_under(object.center, 0.03);
        let spec = spec_at([0.20, 0.0, 0.16], [0.04, 0.0, 0.0]);
        let face_tool = [0.30 - 0.03 - 0.015, 0.0, 0.16];
        assert!(evaluate_contact_candidate(
            face_tool,
            Some([1.0, 0.0, 0.0]),
            ContactingBodyKind::DeclaredTool,
            object,
            support,
            &spec,
        )
        .is_ok());
        let past_edge = [face_tool[0], 0.08, face_tool[2]];
        assert_eq!(
            evaluate_contact_candidate(
                past_edge,
                Some([1.0, 0.0, 0.0]),
                ContactingBodyKind::DeclaredTool,
                object,
                support,
                &spec
            )
            .unwrap_err(),
            ContactInfeasible::WrongContactGeometry
        );
        assert_eq!(
            evaluate_contact_candidate(
                face_tool,
                Some([0.0, 1.0, 0.0]),
                ContactingBodyKind::DeclaredTool,
                object,
                support,
                &spec
            )
            .unwrap_err(),
            ContactInfeasible::OrientationInfeasible
        );
        let ee_on_face = face_tool;
        let wrong_offset_tool = [ee_on_face[0] + 0.04, ee_on_face[1] + 0.05, ee_on_face[2]];
        assert_eq!(
            evaluate_contact_candidate(
                wrong_offset_tool,
                Some([1.0, 0.0, 0.0]),
                ContactingBodyKind::DeclaredTool,
                object,
                support,
                &spec
            )
            .unwrap_err(),
            ContactInfeasible::WrongContactGeometry
        );
        let above = [face_tool[0], 0.0, face_tool[2] + 0.08];
        assert_eq!(
            evaluate_contact_candidate(
                above,
                Some([1.0, 0.0, 0.0]),
                ContactingBodyKind::DeclaredTool,
                object,
                support,
                &spec
            )
            .unwrap_err(),
            ContactInfeasible::WrongContactGeometry
        );
        assert_eq!(
            evaluate_contact_candidate(
                face_tool,
                Some([1.0, 0.0, 0.0]),
                ContactingBodyKind::Forearm,
                object,
                support,
                &spec
            )
            .unwrap_err(),
            ContactInfeasible::WrongContactGeometry
        );
        let in_support = [face_tool[0], 0.0, 0.10];
        assert_eq!(
            evaluate_contact_candidate(
                in_support,
                Some([1.0, 0.0, 0.0]),
                ContactingBodyKind::DeclaredTool,
                object,
                support,
                &spec
            )
            .unwrap_err(),
            ContactInfeasible::SupportPlaneBlocksEe
        );
        let opposite = [0.30 + 0.03 + 0.015, 0.0, 0.16];
        assert_eq!(
            evaluate_contact_candidate(
                opposite,
                Some([1.0, 0.0, 0.0]),
                ContactingBodyKind::DeclaredTool,
                object,
                support,
                &spec
            )
            .unwrap_err(),
            ContactInfeasible::WrongContactGeometry
        );
    }

    fn fk_axis_dot_push(fk_quat: [f64; 4], push: [f64; 3]) -> f64 {
        let axis = rotate_by_quat(fk_quat, [1.0, 0.0, 0.0]);
        dot3(axis, push)
    }

    fn assert_collision_inadmissible(
        res: Result<(ContactManeuver, RankWhy), ContactInfeasible>,
        funnel: &ContactSelectFunnel,
    ) {
        assert!(
            funnel.n_complete_witnesses >= 1,
            "collision must run on a constructed witness, funnel={funnel:?}"
        );
        assert_eq!(funnel.n_selected_executable, 0);
        assert!(
            is_collision_block_reason(&funnel.last_block_reason),
            "last_block_reason must be a collision class, funnel={funnel:?}"
        );
        let e = res.expect_err("colliding witness must not be selected");
        assert_eq!(
            e,
            ContactInfeasible::CollisionInadmissible,
            "got {e:?} funnel={funnel:?}"
        );
    }

    #[test]
    fn fk_consistent_xyz_hit_with_bad_axis_is_not_contact_proof() {
        let model = crate::adapter::synth_planar_two_link();
        let chain = model.ee_joint_chain("ee").unwrap();
        let q_bad = vec![0.9, 1.0];
        let fk = crate::kinematics::forward_kinematics(&model, &chain, "ee", &q_bad).unwrap();
        let push = [1.0, 0.0, 0.0];
        assert!(
            fk_axis_dot_push(fk.ee.quat_wxyz, push) < 0.5,
            "precondition: FK tool axis must fail min_align, dot={}",
            fk_axis_dot_push(fk.ee.quat_wxyz, push)
        );
        let spec = spec_at(fk.ee.xyz, [0.0, 0.0, 0.0]);
        let half = [0.03, 0.03, 0.03];
        let face = half_along_push(half, push) + spec.face_gap;
        let object = BoxObject {
            center: [fk.ee.xyz[0] + face, fk.ee.xyz[1], fk.ee.xyz[2]],
            half_extents: half,
            quat_wxyz: [1.0, 0.0, 0.0, 0.0],
        };
        let support = support_under(object.center, 0.03);
        let seed = SampledEePose {
            xyz: fk.ee.xyz,
            quat_wxyz: fk.ee.quat_wxyz,
            q: q_bad,
            joint_names: chain.clone(),
        };
        let ahead_q = vec![0.0, 0.0];
        let fka = crate::kinematics::forward_kinematics(&model, &chain, "ee", &ahead_q).unwrap();
        let ahead = SampledEePose {
            xyz: fka.ee.xyz,
            quat_wxyz: fka.ee.quat_wxyz,
            q: ahead_q,
            joint_names: chain,
        };
        let (res, funnel) = select_fixed_world_push_seeded_with_funnel(
            &model,
            "ee",
            &[seed, ahead],
            object,
            support,
            &spec,
        );
        assert_eq!(funnel.n_selected_executable, 0);
        let e = res.expect_err("XYZ hit with FK-incompatible axis is not contact proof");
        assert_eq!(
            e,
            ContactInfeasible::OrientationInfeasible,
            "got {e:?} funnel={funnel:?}"
        );
    }

    #[test]
    fn fk_consistent_aligned_axis_is_contact_proof() {
        let model = crate::adapter::synth_planar_two_link();
        let chain = model.ee_joint_chain("ee").unwrap();
        let q_good = vec![0.5, 0.4];
        let fk = crate::kinematics::forward_kinematics(&model, &chain, "ee", &q_good).unwrap();
        let push = [1.0, 0.0, 0.0];
        assert!(
            fk_axis_dot_push(fk.ee.quat_wxyz, push) + 1e-12 >= 0.5,
            "precondition: FK tool axis must meet min_align, dot={}",
            fk_axis_dot_push(fk.ee.quat_wxyz, push)
        );
        let mut spec = spec_at(fk.ee.xyz, [0.0, 0.0, 0.0]);
        spec.min_stroke = 0.005;
        spec.requested_stroke = 0.01;
        let half = [0.03, 0.03, 0.03];
        let face = half_along_push(half, push) + spec.face_gap;
        let object = BoxObject {
            center: [fk.ee.xyz[0] + face, fk.ee.xyz[1], fk.ee.xyz[2]],
            half_extents: half,
            quat_wxyz: [1.0, 0.0, 0.0, 0.0],
        };
        let support = support_under(object.center, 0.03);
        let seed = SampledEePose {
            xyz: fk.ee.xyz,
            quat_wxyz: fk.ee.quat_wxyz,
            q: q_good,
            joint_names: chain.clone(),
        };
        let ahead_q = vec![0.0, 0.0];
        let fka = crate::kinematics::forward_kinematics(&model, &chain, "ee", &ahead_q).unwrap();
        let ahead = SampledEePose {
            xyz: fka.ee.xyz,
            quat_wxyz: fka.ee.quat_wxyz,
            q: ahead_q,
            joint_names: chain,
        };
        let (res, funnel) = select_fixed_world_push_seeded_with_funnel(
            &model,
            "ee",
            &[seed, ahead],
            object,
            support,
            &spec,
        );
        let (m, _) = res.unwrap_or_else(|e| {
            panic!("FK-consistent aligned contact must be selectable: {e:?} funnel={funnel:?}")
        });
        let w = m
            .executable
            .as_ref()
            .expect("selected maneuver stores a witness");
        assert!(w.contact.orientation_checked);
        assert!(w.contact.full_pose_feasible);
        assert!(ik_residual_is_precise(w.contact.residual));
        assert_eq!(funnel.n_selected_executable, 1);
    }

    #[test]
    fn colliding_interpolation_is_not_selected_executable() {
        let model = planar_limited([-2.5, 2.5], [-2.0, 2.0]);
        let (cloud, object, support, mut spec) = seeded_push_world(&model, &[vec![0.5, 0.4]]);
        spec.obstacle_boxes
            .push(crate::contact_collision::NamedBox::aabb(
                "obstacle",
                cloud[0].xyz,
                [0.20, 0.20, 0.20],
            ));
        let (res, funnel) = select_fixed_world_push_seeded_with_funnel(
            &model, "ee", &cloud, object, support, &spec,
        );
        assert_collision_inadmissible(res, &funnel);
    }

    #[test]
    fn select_self_collision_is_collision_inadmissible() {
        let mut model = crate::adapter::synth_planar_two_link();
        model.bodies.push(crate::embodiment::Body {
            name: "stub".into(),
            parent: Some("base".into()),
            mass_kg: crate::provenance::Provenanced::unknown("test", 0.0),
            com: crate::provenance::Provenanced::unknown("test", 0.0),
            inertia: crate::provenance::Provenanced::unknown("test", 0.0),
            local_pose: crate::provenance::Provenanced::declared(Se3::identity(), "test", 0.0),
        });
        let (cloud, object, mut support, mut spec) = seeded_push_world(&model, &[vec![0.5, 0.4]]);
        support.origin = [object.center[0], object.center[1], object.center[2] - 1.0];
        spec.robot_body_volumes = vec![
            crate::contact_collision::AttachedSphere {
                body: "stub".into(),
                radius: 0.20,
                offset: [0.0, 0.0, 0.0],
            },
            crate::contact_collision::AttachedSphere {
                body: "link2".into(),
                radius: 0.20,
                offset: [0.0, 0.0, 0.0],
            },
        ];
        let (res, funnel) = select_fixed_world_push_seeded_with_funnel(
            &model, "ee", &cloud, object, support, &spec,
        );
        assert_collision_inadmissible(res, &funnel);
        assert_eq!(funnel.last_block_reason, "SELF_COLLISION");
    }

    #[test]
    fn select_unintended_forearm_is_collision_inadmissible() {
        let model = crate::adapter::synth_planar_two_link();
        let (cloud, object, support, mut spec) = seeded_push_world(&model, &[vec![0.5, 0.4]]);
        spec.intended_tool_bodies = vec!["tool".into()];
        spec.robot_body_volumes = vec![crate::contact_collision::AttachedSphere {
            body: "link1".into(),
            radius: 0.40,
            offset: [0.0, 0.0, 0.0],
        }];
        let (res, funnel) = select_fixed_world_push_seeded_with_funnel(
            &model, "ee", &cloud, object, support, &spec,
        );
        assert_collision_inadmissible(res, &funnel);
        assert!(
            funnel.last_block_reason == "UNINTENDED_ROBOT_OBJECT_CONTACT"
                || funnel.last_block_reason == "WRONG_PHASE_OBJECT_CONTACT",
            "funnel={funnel:?}"
        );
    }

    #[test]
    fn select_wrong_phase_object_contact_is_collision_inadmissible() {
        let model = crate::adapter::synth_planar_two_link();
        let (cloud, object, mut support, mut spec) = seeded_push_world(&model, &[vec![0.5, 0.4]]);
        spec.ee_radius = 0.08;
        spec.object_probe_radius = 0.08;
        support.origin = [object.center[0], object.center[1], object.center[2] - 0.30];
        let (res, funnel) = select_fixed_world_push_seeded_with_funnel(
            &model, "ee", &cloud, object, support, &spec,
        );
        assert_collision_inadmissible(res, &funnel);
        assert_eq!(funnel.last_block_reason, "WRONG_PHASE_OBJECT_CONTACT");
    }

    #[test]
    fn geometry_without_executable_witness_is_not_no_feasible_contact_pose() {
        let model = planar_limited([-2.5, 2.5], [0.38, 0.42]);
        let (cloud, object, support, spec) = seeded_push_world(&model, &[vec![0.5, 0.4]]);
        let (res, funnel) = select_fixed_world_push_seeded_with_funnel(
            &model, "ee", &cloud, object, support, &spec,
        );
        assert!(res.is_err());
        assert_ne!(res.unwrap_err(), ContactInfeasible::NoFeasibleContactPose);
        assert!(
            funnel.n_geometric_candidates > 0 || funnel.n_phase_ik_attempts > 0,
            "funnel={funnel:?}"
        );
    }
}
