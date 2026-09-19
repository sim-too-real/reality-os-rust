use crate::capability::{CapName, CapStatus, CapabilityGraph};
use crate::embodiment::EmbodimentModel;
use crate::interaction::{world_pose_of, InteractionFrameKind};
use crate::object::ObjectState;
use crate::observation::ObservationFrame;
use crate::plan::{SkillPlan, SkillStep};
use crate::provenance::Provenanced;
use crate::skill::{SkillContract, SkillName, SkillRefuse};
use crate::transform::{Se3, TransformGraph};
use crate::world::PoseEvidence;
use std::cell::Cell;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum PushCompileMode {
    DirectStroke,
    #[default]
    ContactMaintaining,
}

thread_local! {
    static PUSH_COMPILE_MODE: Cell<PushCompileMode> =
        const { Cell::new(PushCompileMode::ContactMaintaining) };
}

pub fn with_push_compile_mode<R>(mode: PushCompileMode, f: impl FnOnce() -> R) -> R {
    PUSH_COMPILE_MODE.with(|c| {
        let prev = c.replace(mode);
        let out = f();
        c.set(prev);
        out
    })
}

pub fn current_push_compile_mode() -> PushCompileMode {
    PUSH_COMPILE_MODE.with(Cell::get)
}

/// Approach standoff along the support-plane push direction.
pub fn push_approach_standoff_m() -> f64 {
    0.05
}

/// Contact Reach must travel from the approach pose. A radius ≥ standoff
/// lets the controller succeed contact without leaving the approach pose.
pub fn push_contact_success_radius() -> f64 {
    (push_approach_standoff_m() * 0.35).clamp(0.012, 0.02)
}

#[derive(Debug, Clone, PartialEq)]
pub struct PushCandidate {
    pub object_id: String,
    pub contact: Se3,
    pub approach: Se3,
    pub direction: [f64; 3],
    pub distance_m: f64,
    pub target_region: Option<[f64; 3]>,
    pub reference_frame: String,
}

fn cap_usable(caps: &CapabilityGraph, name: CapName) -> bool {
    caps.get_opt(name)
        .map(|n| {
            matches!(
                n.status,
                CapStatus::Proven | CapStatus::Supported | CapStatus::PartiallySupported
            )
        })
        .unwrap_or(false)
}

impl SkillContract {
    pub fn push() -> Self {
        Self {
            id: "skill.push".into(),
            name: SkillName::Push,
            required: vec![
                CapName::CartesianPositionControl,
                CapName::ContactManipulation,
            ],
            required_world: vec![
                "target_object".into(),
                "push_contact".into(),
                "push_direction".into(),
            ],
            success_evidence: vec![
                "object_displaced_along_direction".into(),
                "controlled_contact_established".into(),
            ],
            failure_evidence: vec![
                "MISS".into(),
                "SLIP_AROUND_OBJECT".into(),
                "OBJECT_NOT_MOVABLE".into(),
                "EXCESS_FORCE".into(),
                "UNEXPECTED_CONTACT".into(),
                "ROBOT_BLOCKED".into(),
                "TARGET_STALE".into(),
            ],
            authority_ceiling: "allow".into(),
            exploration_allowance: false,
        }
    }
}

fn pose_evidence(pose: Se3, now_s: f64, expires: f64) -> PoseEvidence {
    PoseEvidence {
        frame_id: "world".into(),
        pose: Provenanced {
            value: Some(pose),
            provenance: crate::provenance::Provenance::UserDeclared,
            source: "perfect_perception.scenario".into(),
            as_of_s: now_s,
            uncertainty: None,
        },
        expires_at_s: expires,
    }
}

#[allow(clippy::too_many_arguments)]
pub fn compile_push(
    model: &EmbodimentModel,
    caps: &CapabilityGraph,
    object: &ObjectState,
    candidate: &PushCandidate,
    obs: &ObservationFrame,
    transforms: &TransformGraph,
    ee: &str,
    expected_model_hash: &str,
    now_s: f64,
    freshness_s: f64,
) -> Result<SkillPlan, SkillRefuse> {
    if expected_model_hash != model.model_hash {
        return Err(SkillRefuse::WrongModelHash);
    }
    if !object.fresh(now_s, freshness_s) {
        return Err(SkillRefuse::StaleObject);
    }
    if object.object_id != candidate.object_id {
        return Err(SkillRefuse::MissingTarget);
    }
    let cartesian_ok = cap_usable(caps, CapName::CartesianPositionControl);
    let joint_ok = cap_usable(caps, CapName::JointPositionControl);
    let chain_ok = model.ee_joint_chain(ee).is_some_and(|c| !c.is_empty())
        || model
            .end_effectors
            .iter()
            .any(|e| !e.joint_chain.is_empty());
    if !(cartesian_ok || (joint_ok && chain_ok)) {
        if model.position_actuators().next().is_none() {
            return Err(SkillRefuse::MissingActuator);
        }
        return Err(SkillRefuse::Unsupported);
    }
    if !cap_usable(caps, CapName::ContactManipulation)
        && !cap_usable(caps, CapName::Pushing)
        && !cartesian_ok
    {
        return Err(SkillRefuse::Unsupported);
    }
    if obs.required_stale(now_s, freshness_s) {
        return Err(SkillRefuse::StaleEvidence);
    }
    if model.end_effectors.is_empty() {
        return Err(SkillRefuse::Unsupported);
    }

    let contact_id = InteractionFrameKind::PushContact.frame_id(&candidate.object_id);
    let approach_id = InteractionFrameKind::Approach.frame_id(&candidate.object_id);
    let contact =
        world_pose_of(transforms, &contact_id, now_s, freshness_s).unwrap_or(candidate.contact);
    let approach =
        world_pose_of(transforms, &approach_id, now_s, freshness_s).unwrap_or(candidate.approach);
    let n = (candidate.direction[0].powi(2)
        + candidate.direction[1].powi(2)
        + candidate.direction[2].powi(2))
    .sqrt();
    if n < 1e-8 {
        return Err(SkillRefuse::Unsupported);
    }
    let dir3 = [
        candidate.direction[0] / n,
        candidate.direction[1] / n,
        candidate.direction[2] / n,
    ];
    let (approach_pose, contact_or_press, waypoints, contact_radius) =
        match current_push_compile_mode() {
            PushCompileMode::ContactMaintaining => {
                let dir = support_plane_direction(dir3).unwrap_or(dir3);
                let standoff = push_approach_standoff_m();
                let approach_pose = Se3::try_new(
                    [
                        contact.xyz[0] - dir[0] * standoff,
                        contact.xyz[1] - dir[1] * standoff,
                        contact.xyz[2],
                    ],
                    contact.quat_wxyz,
                )
                .map_err(|_| SkillRefuse::Unsupported)?;
                let press_xyz = contact_press_xyz(contact.xyz, dir, candidate.distance_m);
                let press = Se3::try_new(press_xyz, contact.quat_wxyz)
                    .map_err(|_| SkillRefuse::Unsupported)?;
                (
                    approach_pose,
                    press,
                    contact_maintaining_stroke_xyz(contact.xyz, dir, candidate.distance_m),
                    push_contact_success_radius(),
                )
            }
            PushCompileMode::DirectStroke => {
                let stroke = effective_push_distance(candidate.distance_m);
                let end = Se3::try_new(
                    [
                        contact.xyz[0] + dir3[0] * stroke,
                        contact.xyz[1] + dir3[1] * stroke,
                        contact.xyz[2] + dir3[2] * stroke,
                    ],
                    contact.quat_wxyz,
                )
                .map_err(|_| SkillRefuse::Unsupported)?;
                (approach, contact, vec![end.xyz], 0.06)
            }
        };

    let expires = now_s + freshness_s;
    let mut plan = SkillPlan::empty(SkillName::Push, "skill.push");
    plan.success_evidence = SkillContract::push().success_evidence;
    plan.failure_evidence = SkillContract::push().failure_evidence;
    plan.steps.push(SkillStep::Reach {
        end_effector: ee.into(),
        target: pose_evidence(approach_pose, now_s, expires),
        success_radius: 0.08,
    });
    plan.steps.push(SkillStep::Reach {
        end_effector: ee.into(),
        target: pose_evidence(contact_or_press, now_s, expires),
        success_radius: contact_radius,
    });
    for xyz in waypoints {
        let pose = Se3::try_new(xyz, contact.quat_wxyz).map_err(|_| SkillRefuse::Unsupported)?;
        let travel = ((xyz[0] - contact.xyz[0]).powi(2)
            + (xyz[1] - contact.xyz[1]).powi(2)
            + (xyz[2] - contact.xyz[2]).powi(2))
        .sqrt();
        plan.steps.push(SkillStep::Reach {
            end_effector: ee.into(),
            target: pose_evidence(pose, now_s, expires),
            success_radius: push_stroke_success_radius(travel),
        });
    }
    plan.steps.push(SkillStep::Stop);
    Ok(plan)
}

/// Floor a commanded push so the stroke is not smaller than contact slack.
/// Embodiment-agnostic: no robot-identity branch.
pub fn effective_push_distance(distance_m: f64) -> f64 {
    distance_m.max(0.02)
}

/// Final Reach must actually travel. A radius ≥ stroke lets the controller
/// succeed at the contact pose without displacing the object.
pub fn push_stroke_success_radius(distance_m: f64) -> f64 {
    (distance_m * 0.35).clamp(0.008, 0.03)
}

/// Project a push direction onto the support plane (world +Z up).
/// Vertical remainder-only directions cannot maintain table contact.
pub fn support_plane_direction(direction: [f64; 3]) -> Option<[f64; 3]> {
    let n = (direction[0] * direction[0] + direction[1] * direction[1]).sqrt();
    if n < 1e-8 {
        return None;
    }
    Some([direction[0] / n, direction[1] / n, 0.0])
}

fn along_support_plane(contact_xyz: [f64; 3], plane_dir: [f64; 3], s: f64) -> [f64; 3] {
    [
        contact_xyz[0] + plane_dir[0] * s,
        contact_xyz[1] + plane_dir[1] * s,
        contact_xyz[2],
    ]
}

/// Press the contact target into the object along the support plane so the
/// following stroke starts from established face contact, not a gap.
pub fn contact_press_xyz(contact_xyz: [f64; 3], plane_dir: [f64; 3], distance_m: f64) -> [f64; 3] {
    let stroke = effective_push_distance(distance_m);
    let press = (stroke * 0.35).clamp(0.012, 0.025);
    along_support_plane(contact_xyz, plane_dir, press)
}

/// Terminal stroke at contact height with follow-through. z is locked so the
/// controller cannot arc off the support plane.
pub fn contact_maintaining_stroke_xyz(
    contact_xyz: [f64; 3],
    plane_dir: [f64; 3],
    distance_m: f64,
) -> Vec<[f64; 3]> {
    let stroke = effective_push_distance(distance_m);
    vec![along_support_plane(contact_xyz, plane_dir, stroke * 1.1)]
}

pub fn push_unexpected_recovery() -> Vec<SkillStep> {
    vec![SkillStep::Stop]
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapter::{identity_base_graph, zero_joint_obs};
    use crate::capability::derive_capabilities;
    use crate::object::{GeometryClass, GraspOccupancy, ObjectGeometry};
    use crate::provenance::Provenance;

    #[test]
    fn stale_target_refuses_push() {
        let m = crate::adapter::synth_planar_two_link();
        let caps = derive_capabilities(&m, None);
        let mut object = ObjectState::unknown("obj", 1.0);
        object.pose = Provenanced {
            value: Se3::try_new([0.2, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]).ok(),
            provenance: Provenance::UserDeclared,
            source: "perfect_perception".into(),
            as_of_s: 1.0,
            uncertainty: None,
        };
        object.reference_frame = "world".into();
        object.expires_at_s = 0.1;
        object.timestamp_s = 0.0;
        object.geometry = ObjectGeometry {
            class: GeometryClass::Box,
            bounds: Provenanced::declared([0.03, 0.03, 0.03], "t", 1.0),
        };
        object.grasp_state = GraspOccupancy::Free;
        let cand = PushCandidate {
            object_id: "obj".into(),
            contact: Se3::try_new([0.18, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]).unwrap(),
            approach: Se3::try_new([0.14, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]).unwrap(),
            direction: [1.0, 0.0, 0.0],
            distance_m: 0.05,
            target_region: None,
            reference_frame: "world".into(),
        };
        let obs = zero_joint_obs(&m, "e0", 1.0);
        let g = identity_base_graph("e0");
        let err = compile_push(
            &m,
            &caps,
            &object,
            &cand,
            &obs,
            &g,
            "ee",
            &m.model_hash,
            1.0,
            0.25,
        )
        .unwrap_err();
        assert_eq!(err, SkillRefuse::StaleObject);
    }

    #[test]
    fn stroke_success_radius_is_stricter_than_distance() {
        for d in [0.012, 0.02, 0.03, 0.05, 0.08] {
            let r = push_stroke_success_radius(effective_push_distance(d));
            assert!(
                r < effective_push_distance(d),
                "radius {r} must be < stroke {}",
                effective_push_distance(d)
            );
        }
        assert_eq!(effective_push_distance(0.005), 0.02);
        assert_eq!(effective_push_distance(0.05), 0.05);
    }

    fn fresh_object() -> ObjectState {
        let mut object = ObjectState::unknown("obj", 1.0);
        object.pose = Provenanced {
            value: Se3::try_new([0.2, 0.0, 0.1], [1.0, 0.0, 0.0, 0.0]).ok(),
            provenance: Provenance::UserDeclared,
            source: "perfect_perception".into(),
            as_of_s: 1.0,
            uncertainty: None,
        };
        object.reference_frame = "world".into();
        object.expires_at_s = 10.0;
        object.timestamp_s = 1.0;
        object.geometry = ObjectGeometry {
            class: GeometryClass::Box,
            bounds: Provenanced::declared([0.03, 0.03, 0.03], "t", 1.0),
        };
        object.grasp_state = GraspOccupancy::Free;
        object
    }

    fn compile_fresh_push(direction: [f64; 3], distance_m: f64) -> SkillPlan {
        let m = crate::adapter::synth_planar_two_link();
        let caps = derive_capabilities(&m, None);
        let object = fresh_object();
        let contact = Se3::try_new([0.18, 0.0, 0.1], [1.0, 0.0, 0.0, 0.0]).unwrap();
        let cand = PushCandidate {
            object_id: "obj".into(),
            contact,
            approach: Se3::try_new([0.14, 0.0, 0.1], [1.0, 0.0, 0.0, 0.0]).unwrap(),
            direction,
            distance_m,
            target_region: None,
            reference_frame: "world".into(),
        };
        let obs = zero_joint_obs(&m, "epoch0", 1.0);
        let g = identity_base_graph("epoch0");
        compile_push(
            &m,
            &caps,
            &object,
            &cand,
            &obs,
            &g,
            "ee",
            &m.model_hash,
            1.0,
            0.25,
        )
        .expect("fresh push must compile")
    }

    fn reach_targets(plan: &SkillPlan) -> Vec<([f64; 3], f64)> {
        plan.steps
            .iter()
            .filter_map(|s| match s {
                SkillStep::Reach {
                    target,
                    success_radius,
                    ..
                } => Some((target.xyz()?, *success_radius)),
                _ => None,
            })
            .collect()
    }

    #[test]
    fn support_plane_direction_zeros_world_up() {
        let d = support_plane_direction([1.0, 0.2, 0.8]).expect("horizontal remainder");
        assert!((d[2]).abs() < 1e-12);
        let n = (d[0] * d[0] + d[1] * d[1]).sqrt();
        assert!((n - 1.0).abs() < 1e-12);
        assert!(d[0] > 0.0);
        assert!(support_plane_direction([0.0, 0.0, 1.0]).is_none());
    }

    #[test]
    fn contact_maintaining_stroke_locks_height_and_travels() {
        let contact = [0.18, 0.04, 0.22];
        let dir = support_plane_direction([0.4, 0.1, 0.9]).unwrap();
        let press = contact_press_xyz(contact, dir, 0.05);
        let pts = contact_maintaining_stroke_xyz(contact, dir, 0.05);
        assert!(!pts.is_empty(), "need a terminal stroke waypoint");
        let stroke = effective_push_distance(0.05);
        assert!((press[2] - contact[2]).abs() < 1e-12);
        let press_along = (press[0] - contact[0]) * dir[0] + (press[1] - contact[1]) * dir[1];
        assert!(press_along > 0.0);
        for p in &pts {
            assert!(
                (p[2] - contact[2]).abs() < 1e-12,
                "stroke must stay on the contact support plane, z={} vs {}",
                p[2],
                contact[2]
            );
        }
        let end = *pts.last().unwrap();
        let along = (end[0] - contact[0]) * dir[0] + (end[1] - contact[1]) * dir[1];
        assert!(
            along + 1e-12 >= stroke,
            "terminal waypoint must cover the stroke, along={along} stroke={stroke}"
        );
        assert!(press_along < along);
    }

    #[test]
    fn compile_push_stroke_stays_on_support_plane() {
        let contact_z = 0.1;
        let plan = compile_fresh_push([1.0, 0.0, 0.7], 0.05);
        let reaches = reach_targets(&plan);
        assert!(
            reaches.len() >= 3,
            "approach, pressed contact, terminal stroke; got {}",
            reaches.len()
        );
        let contact_xyz = reaches[1].0;
        assert!((contact_xyz[2] - contact_z).abs() < 1e-12);
        assert!(contact_xyz[0] > 0.18);
        let stroke_pts = &reaches[2..];
        for (xyz, radius) in stroke_pts {
            assert!(
                (xyz[2] - contact_z).abs() < 1e-12,
                "stroke z {} must equal contact z {contact_z}",
                xyz[2]
            );
            let travel = ((xyz[0] - 0.18).powi(2) + (xyz[1] - 0.0).powi(2)).sqrt();
            assert!(
                *radius < travel,
                "stroke radius {radius} must be < travel {travel}"
            );
        }
        let end = stroke_pts.last().unwrap().0;
        assert!(end[0] > contact_xyz[0]);
        let along = end[0] - 0.18;
        assert!(along + 1e-12 >= effective_push_distance(0.05));
    }

    #[test]
    fn contact_reach_must_travel_from_approach() {
        assert!(push_contact_success_radius() < push_approach_standoff_m());
        let plan = compile_fresh_push([1.0, 0.0, 0.0], 0.05);
        let reaches = reach_targets(&plan);
        let approach = reaches[0].0;
        let (contact_xyz, contact_r) = reaches[1];
        let d = ((contact_xyz[0] - approach[0]).powi(2)
            + (contact_xyz[1] - approach[1]).powi(2)
            + (contact_xyz[2] - approach[2]).powi(2))
        .sqrt();
        assert!(
            contact_r < d,
            "contact radius {contact_r} must be < approach travel {d} or the contact Reach succeeds without moving"
        );
    }

    #[test]
    fn direct_stroke_compile_follows_commanded_direction_including_z() {
        let plan = with_push_compile_mode(PushCompileMode::DirectStroke, || {
            compile_fresh_push([1.0, 0.0, 0.7], 0.05)
        });
        let reaches = reach_targets(&plan);
        let end = reaches.last().expect("stroke reach").0;
        assert!(
            end[2] > 0.1,
            "direct stroke keeps the commanded vertical component, z={}",
            end[2]
        );
    }
}
