//! PUSH contact as a feasible maneuver, not a reachable XYZ sample.
//!
//! No robot name, vendor, URDF filename, bundle ID, or dataset origin.

use crate::embodiment::Joint;
use crate::push::{
    effective_push_distance, push_approach_standoff_m, push_contact_success_radius,
    support_plane_direction,
};
use crate::transform::{add3, norm3, normalize3, quat_conj, rotate_by_quat, scale3, sub3, Se3};
use serde::{Deserialize, Serialize};
use std::cell::Cell;

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

/// FK sample that includes orientation and the joint vector that produced it.
#[derive(Debug, Clone, PartialEq)]
pub struct SampledEePose {
    pub xyz: [f64; 3],
    pub quat_wxyz: [f64; 4],
    pub q: Vec<f64>,
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
            max_approach_match: 0.55,
        }
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
    OrientationInfeasible,
    ApproachCollidesBeforeContact,
    SupportPlaneBlocksEe,
    ContactPoseUnreachableFromApproach,
    WrongContactGeometry,
    InsufficientRemainingStroke,
}

impl ContactInfeasible {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::NoFeasibleContactPose => "NO_FEASIBLE_CONTACT_POSE",
            Self::OrientationInfeasible => "ORIENTATION_INFEASIBLE",
            Self::ApproachCollidesBeforeContact => "APPROACH_COLLIDES_BEFORE_CONTACT",
            Self::SupportPlaneBlocksEe => "SUPPORT_PLANE_BLOCKS_EE",
            Self::ContactPoseUnreachableFromApproach => "CONTACT_POSE_UNREACHABLE_FROM_APPROACH",
            Self::WrongContactGeometry => "WRONG_CONTACT_GEOMETRY",
            Self::InsufficientRemainingStroke => "INSUFFICIENT_REMAINING_STROKE",
        }
    }
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
}

const TOOL_LEN_MIN: f64 = 0.008;
const STROKE_CORRIDOR_M: f64 = 0.08;

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

fn joint_margin_of(q: &[f64], joints: &[Joint]) -> f64 {
    if q.is_empty() {
        return 1.0;
    }
    let mut worst = 1.0_f64;
    let n = q.len().min(joints.len());
    for i in 0..n {
        let (Some(lo), Some(hi)) = (joints[i].q_min.value, joints[i].q_max.value) else {
            continue;
        };
        let span = (hi - lo).abs().max(1e-6);
        let d = (q[i] - lo).min(hi - q[i]).max(0.0);
        worst = worst.min(d / span);
    }
    worst.clamp(0.0, 1.0)
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
        None => 0.0,
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
    let tool_len = norm3(tool_off);
    let tool_axis = if tool_len >= TOOL_LEN_MIN {
        let axis = normalize3(tool_off).ok_or(ContactInfeasible::OrientationInfeasible)?;
        if dot3(axis, push) < spec.min_align {
            return Err(ContactInfeasible::OrientationInfeasible);
        }
        Some(axis)
    } else {
        None
    };

    let tool_contact = add3(sample.xyz, tool_off);
    let face_offset = sub3(tool_contact, object.center);
    let up = normalize3(support.normal).unwrap_or([0.0, 0.0, 1.0]);
    if dot3(face_offset, up).abs() > object.half_extents[2] * 0.9 {
        return Err(ContactInfeasible::WrongContactGeometry);
    }
    if dot3(face_offset, push) > 1e-4 {
        return Err(ContactInfeasible::WrongContactGeometry);
    }

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
    if available + 1e-9 < spec.min_stroke {
        return Err(ContactInfeasible::InsufficientRemainingStroke);
    }

    let support_top_z = support.origin[2];
    let contact_pose = pose(sample.xyz, sample.quat_wxyz)?;
    let approach_pose = pose(approach_xyz, sample.quat_wxyz)?;
    let orientation_error = orientation_error_of(tool_axis, push);
    Ok(ContactManeuver {
        contact_pose,
        approach_pose,
        contact_point: tool_contact,
        contact_normal: scale3(push, -1.0),
        push_direction: push,
        requested_stroke: spec.requested_stroke,
        available_stroke: available,
        support_clearance: clearance,
        joint_margin: joint_margin_of(&sample.q, joints),
        orientation_error,
        object_center: object.center,
        support_top_z,
        sampled_q: sample.q.clone(),
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
    RankInputs {
        approach_distance,
        joint_margin: m.joint_margin,
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
        };
        match evaluate_sampled_push(sample, cloud, object, support, spec, joints) {
            Ok(m) => feasible.push(m),
            Err(e) => last_err = e,
        }
    }
    let (idx, why) = select_contact_maneuver(&feasible, spec).ok_or(last_err)?;
    Ok((feasible[idx].clone(), why))
}

/// Evidence-backed pre-contact label. Privileged collision flags are
/// evaluation-only inputs to this classifier, never ranking inputs.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum PreContactTaxonomy {
    NoFeasibleContactPose,
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
            Some(ContactInfeasible::NoFeasibleContactPose)
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
        }
    }

    fn identity_quat() -> [f64; 4] {
        [1.0, 0.0, 0.0, 0.0]
    }

    fn rot_z_90() -> [f64; 4] {
        Se3::from_axis_angle([0.0, 0.0, 1.0], std::f64::consts::FRAC_PI_2)
            .unwrap()
            .quat_wxyz
    }

    fn spec_at(current: [f64; 3], tool_ee: [f64; 3]) -> ContactManeuverSpec {
        ContactManeuverSpec::table_push([1.0, 0.0, 0.0], 0.05, current, tool_ee)
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
        };
        let support = support_under(object.center, 0.03);
        let err = evaluate_sampled_push(&s, &[s.clone()], object, support, &spec, &[]).unwrap_err();
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
}
