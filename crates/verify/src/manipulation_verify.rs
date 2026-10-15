//! Privileged verifier for manipulation. Never reads a policy success flag.

use crate::normalize::RobotManifest;
use crate::observation::{ContactTruth, VerifierTruth};
use realityos_kernel::physical_action::PhysicalActionEnvelope;
use realityos_semantics::contact::{SupportKind, SupportRelation, FORCE_BOUND_UNAVAILABLE};
use realityos_semantics::failure::ManipulationFailure;
use realityos_semantics::provenance::Provenance;
use realityos_semantics::resource::{
    ControlledResource, QualificationStatus, ResourceKind, ResourceTopology,
};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PrivilegedVerdict {
    pub success: bool,
    pub failure: Option<String>,
    pub evidence_used: Vec<String>,
    pub support: Vec<SupportRelation>,
    pub unexpected_contact: bool,
    pub force_bound_unavailable: bool,
    pub object_followed: Option<bool>,
    pub opening_ok: Option<bool>,
    #[serde(default)]
    pub grasp_acquired: bool,
    #[serde(default)]
    pub verified_hold: bool,
    pub notes: Vec<String>,
}

pub fn support_of(
    truth: &VerifierTruth,
    object_id: &str,
    table_names: &[&str],
    finger_bodies: &[String],
    now_s: f64,
) -> SupportRelation {
    let on_table = truth
        .contacts
        .iter()
        .any(|c| pair_has(c, object_id, table_names));
    let in_fingers = truth
        .contacts
        .iter()
        .any(|c| finger_bodies.iter().any(|f| c.pair_has(object_id, f)));
    let kind = match (on_table, in_fingers) {
        (true, true) => SupportKind::Shared,
        (true, false) => SupportKind::SupportedBy,
        (false, true) => SupportKind::Unsupported,
        (false, false) => SupportKind::Airborne,
    };
    SupportRelation {
        object_id: object_id.into(),
        surface_id: table_names.first().copied().unwrap_or("table").into(),
        kind,
        evidence: vec![
            format!("table_contact:{on_table}"),
            format!("finger_contact:{in_fingers}"),
        ],
        provenance: Provenance::SimulatorDerived,
        timestamp_s: now_s,
    }
}

fn pair_has(c: &ContactTruth, object: &str, surfaces: &[&str]) -> bool {
    surfaces.iter().any(|s| c.pair_has(object, s))
}

trait Pair {
    fn pair_has(&self, a: &str, b: &str) -> bool;
}

impl Pair for ContactTruth {
    fn pair_has(&self, a: &str, b: &str) -> bool {
        (self.body1.contains(a) && self.body2.contains(b))
            || (self.body1.contains(b) && self.body2.contains(a))
            || (self.body1 == a && self.body2 == b)
            || (self.body1 == b && self.body2 == a)
    }
}

pub fn verify_release(
    truth: &VerifierTruth,
    resource: &ControlledResource,
    opening_01: Option<f64>,
    held_object: Option<&str>,
    object_before: Option<[f64; 3]>,
    ee_delta: Option<[f64; 3]>,
    now_s: f64,
) -> PrivilegedVerdict {
    let mut evidence = Vec::new();
    let mut notes = Vec::new();
    let opening_ok = opening_01.map(|o| o >= 0.7);
    if let Some(ok) = opening_ok {
        evidence.push("gripper_opening_threshold".into());
        if !ok {
            return fail(ManipulationFailure::Blocked, evidence, notes);
        }
    } else {
        notes.push("opening unobserved".into());
    }
    let trapped = held_object
        .map(|o| {
            resource
                .finger_bodies
                .iter()
                .any(|f| truth.contacts.iter().any(|c| c.pair_has(o, f)))
        })
        .unwrap_or(false);
    if trapped && opening_ok == Some(true) {
        evidence.push("unexpected_trapped_contact".into());
        return fail(ManipulationFailure::UnexpectedContact, evidence, notes);
    }
    evidence.push("no_unexpected_trapped_contact".into());
    if let (Some(obj), Some(before), Some(d)) = (held_object, object_before, ee_delta) {
        if let Some(after) = body_xyz(truth, obj) {
            let follow = dist(after, add(before, d)) < 0.02 && mag(d) > 0.01;
            if follow {
                evidence.push("held_object_still_following".into());
                return fail(ManipulationFailure::Blocked, evidence, notes);
            }
            evidence.push("held_object_stops_following".into());
        }
    }
    let _ = now_s;
    PrivilegedVerdict {
        success: true,
        failure: None,
        evidence_used: evidence,
        support: vec![],
        unexpected_contact: false,
        force_bound_unavailable: resource.force_bound.value.is_none(),
        object_followed: Some(false),
        opening_ok,
        grasp_acquired: false,
        verified_hold: false,
        notes,
    }
}

pub fn verify_grasp(
    before: &VerifierTruth,
    after: &VerifierTruth,
    resource: &ControlledResource,
    object_id: &str,
    table_names: &[&str],
    unexpected_bodies: &[String],
    opening_01: Option<f64>,
    now_s: f64,
    ee_names: &[&str],
    authorized_envelope: Option<&PhysicalActionEnvelope>,
) -> PrivilegedVerdict {
    let mut evidence = Vec::new();
    let mut notes = Vec::new();
    let two_finger_resource =
        resource.finger_bodies.len() == 2 && resource.finger_bodies[0] != resource.finger_bodies[1];
    let left_contact = two_finger_resource
        && after
            .contacts
            .iter()
            .any(|c| c.pair_has(object_id, &resource.finger_bodies[0]));
    let right_contact = two_finger_resource
        && after
            .contacts
            .iter()
            .any(|c| c.pair_has(object_id, &resource.finger_bodies[1]));
    let grasp_acquired = left_contact && right_contact;
    if left_contact {
        evidence.push("left_finger_object_contact".into());
    }
    if right_contact {
        evidence.push("right_finger_object_contact".into());
    }
    if grasp_acquired {
        evidence.push("grasp_acquired".into());
    }
    let any_finger_contact = left_contact || right_contact;
    if opening_01.is_some_and(|opening| opening < 0.08) && !any_finger_contact {
        evidence.push("gripper_empty_close".into());
        return fail_with_acquisition(
            ManipulationFailure::GripperEmptyClose,
            evidence,
            notes,
            grasp_acquired,
        );
    }
    if !grasp_acquired {
        return fail_with_acquisition(ManipulationFailure::Miss, evidence, notes, false);
    }

    let unexpected = after.contacts.iter().any(|contact| {
        unexpected_bodies.iter().any(|body| {
            contact.body1.contains(body.as_str()) || contact.body2.contains(body.as_str())
        })
    });
    if unexpected {
        evidence.push("unexpected_contact".into());
        return fail_with_acquisition(
            ManipulationFailure::UnexpectedContact,
            evidence,
            notes,
            grasp_acquired,
        );
    }
    evidence.push("no_forbidden_contact".into());

    let envelope = match authorized_envelope {
        Some(PhysicalActionEnvelope::PinchGrasp {
            resource_id,
            finger_joint_names,
            per_finger_force_ceiling_n,
            maximum_lift_m,
            ..
        }) if authorized_envelope.is_some_and(|envelope| envelope.validate().is_ok())
            && resource_id == &resource.id
            && finger_joint_names == &resource.affected_joints
            && resource.kind == ResourceKind::Gripper
            && resource.topology == ResourceTopology::DirectJointGripper
            && resource.qualification == QualificationStatus::Qualified
            && resource.force_bound.known_value().is_some_and(|bound| {
                bound[1].is_finite() && bound[1] >= *per_finger_force_ceiling_n
            }) =>
        {
            Some((*per_finger_force_ceiling_n, *maximum_lift_m))
        }
        _ => None,
    };
    let Some((force_ceiling, maximum_lift_m)) = envelope else {
        notes.push("typed pinch-grasp authority unavailable or does not match the resource".into());
        return fail_with_acquisition(
            ManipulationFailure::Blocked,
            evidence,
            notes,
            grasp_acquired,
        );
    };
    evidence.push("typed_grasp_authority_matches_resource".into());

    let mut force_unavailable = false;
    for finger in &resource.finger_bodies {
        match measured_finger_force(after, object_id, finger) {
            Some(force) if force <= force_ceiling => {}
            Some(_) => {
                evidence.push("per_finger_force_exceeds_authorized_ceiling".into());
                return fail_with_acquisition(
                    ManipulationFailure::ExcessForce,
                    evidence,
                    notes,
                    grasp_acquired,
                );
            }
            None => {
                force_unavailable = true;
                break;
            }
        }
    }
    if force_unavailable {
        evidence.push("per_finger_force_unavailable".into());
        notes.push(FORCE_BOUND_UNAVAILABLE.into());
        let mut verdict = fail_with_acquisition(
            ManipulationFailure::ForceBoundUnavailable,
            evidence,
            notes,
            grasp_acquired,
        );
        verdict.force_bound_unavailable = true;
        return verdict;
    }
    evidence.push("per_finger_force_within_authorized_ceiling".into());

    let support = support_of(
        after,
        object_id,
        table_names,
        &resource.finger_bodies,
        now_s,
    );
    let support_ok = matches!(support.kind, SupportKind::Unsupported);
    if support_ok {
        evidence.push("object_separated_from_table_support".into());
    } else {
        evidence.push("object_retains_table_support".into());
    }

    let lift = match (ee_xyz(before, ee_names), ee_xyz(after, ee_names)) {
        (Some(ee_before), Some(ee_after)) => sub(ee_after, ee_before),
        _ => [f64::NAN; 3],
    };
    let lift_vertical_m = lift[2];
    let lateral_m = (lift[0] * lift[0] + lift[1] * lift[1]).sqrt();
    let lift_ok = lift.iter().all(|value| value.is_finite())
        && (0.02..=maximum_lift_m).contains(&lift_vertical_m)
        && lateral_m <= 0.005 + 1e-12;
    if lift_ok {
        evidence.push("bounded_vertical_lift_verified".into());
    } else {
        evidence.push("bounded_vertical_lift_not_verified".into());
    }

    let object_delta = match (body_xyz(before, object_id), body_xyz(after, object_id)) {
        (Some(object_before), Some(object_after)) => sub(object_after, object_before),
        _ => [f64::NAN; 3],
    };
    let follow_residual_m = dist(object_delta, lift);
    let followed = follow_residual_m.is_finite() && follow_residual_m <= 0.005 + 1e-12;
    if followed {
        evidence.push("object_follows_ee".into());
    } else {
        evidence.push("object_follow_residual_exceeds_five_mm".into());
    }

    // A held object can prevent the jaws from reaching the exact zero target.
    // Bilateral measured contact and bounded force are the evidence of closure;
    // this ratio only rejects a materially open gripper.
    let opening_ok = opening_01.map(|opening| opening <= 0.15);
    if opening_ok == Some(true) {
        evidence.push("gripper_closed_during_hold_check".into());
    } else if opening_ok.is_none() {
        notes.push("gripper opening unobserved during hold check".into());
    }
    let verified_hold = lift_ok && followed && support_ok && opening_ok == Some(true);
    if verified_hold {
        evidence.push("verified_hold".into());
        PrivilegedVerdict {
            success: true,
            failure: None,
            evidence_used: evidence,
            support: vec![support],
            unexpected_contact: false,
            force_bound_unavailable: false,
            object_followed: Some(true),
            opening_ok,
            grasp_acquired,
            verified_hold: true,
            notes,
        }
    } else {
        let mut verdict =
            fail_with_acquisition(ManipulationFailure::Slip, evidence, notes, grasp_acquired);
        verdict.support = vec![support];
        verdict.object_followed = Some(followed);
        verdict.opening_ok = opening_ok;
        verdict
    }
}

fn measured_finger_force(truth: &VerifierTruth, object_id: &str, finger_body: &str) -> Option<f64> {
    let contacts = truth
        .contacts
        .iter()
        .filter(|contact| contact.pair_has(object_id, finger_body))
        .collect::<Vec<_>>();
    if contacts.is_empty() {
        return None;
    }
    let mut total = 0.0;
    for contact in contacts {
        if !contact.force_available {
            return None;
        }
        let force = contact.normal_force?;
        if !force.is_finite() || force < 0.0 {
            return None;
        }
        total += force;
    }
    total.is_finite().then_some(total)
}
pub fn verify_push(
    before: &VerifierTruth,
    after: &VerifierTruth,
    object_id: &str,
    direction: [f64; 3],
    min_dist: f64,
    unexpected_bodies: &[String],
    immovable: bool,
    intended_bodies: &[String],
) -> PrivilegedVerdict {
    let mut evidence = Vec::new();
    let notes = Vec::new();
    if immovable {
        if let (Some(b), Some(a)) = (body_xyz(before, object_id), body_xyz(after, object_id)) {
            if dist(a, b) < min_dist * 0.25 {
                evidence.push("object_not_moved".into());
                return fail(ManipulationFailure::ObjectNotMovable, evidence, notes);
            }
        }
    }
    let unexpected = after.contacts.iter().any(|c| {
        unexpected_bodies
            .iter()
            .any(|b| c.body1.contains(b.as_str()) || c.body2.contains(b.as_str()))
    });
    if unexpected {
        return fail(ManipulationFailure::UnexpectedContact, evidence, notes);
    }
    let contact = after.contacts.iter().any(|c| {
        crate::push_pipeline::names_are_ee_object_contact(
            &c.body1,
            &c.body2,
            object_id,
            intended_bodies,
        )
    });
    if contact {
        evidence.push("controlled_contact_established".into());
    }
    let displaced = match (body_xyz(before, object_id), body_xyz(after, object_id)) {
        (Some(b), Some(a)) => {
            let d = sub(a, b);
            let n = mag(direction).max(1e-9);
            let along = (d[0] * direction[0] + d[1] * direction[1] + d[2] * direction[2]) / n;
            along >= min_dist.clamp(0.012, 0.03)
        }
        _ => false,
    };
    if displaced {
        evidence.push("object_displaced_along_direction".into());
        return PrivilegedVerdict {
            success: true,
            failure: None,
            evidence_used: evidence,
            support: vec![],
            unexpected_contact: false,
            force_bound_unavailable: after.contacts.iter().all(|c| !c.force_available),
            object_followed: None,
            opening_ok: None,
            grasp_acquired: false,
            verified_hold: false,
            notes,
        };
    }
    if contact {
        return fail(ManipulationFailure::SlipAroundObject, evidence, notes);
    }
    fail(ManipulationFailure::Miss, evidence, notes)
}

fn fail(f: ManipulationFailure, evidence: Vec<String>, notes: Vec<String>) -> PrivilegedVerdict {
    fail_with_acquisition(f, evidence, notes, false)
}

fn fail_with_acquisition(
    f: ManipulationFailure,
    evidence: Vec<String>,
    notes: Vec<String>,
    grasp_acquired: bool,
) -> PrivilegedVerdict {
    PrivilegedVerdict {
        success: false,
        failure: Some(f.as_str().into()),
        evidence_used: evidence,
        support: vec![],
        unexpected_contact: matches!(f, ManipulationFailure::UnexpectedContact),
        force_bound_unavailable: false,
        object_followed: None,
        opening_ok: None,
        grasp_acquired,
        verified_hold: false,
        notes,
    }
}

pub fn body_xyz(truth: &VerifierTruth, name: &str) -> Option<[f64; 3]> {
    truth
        .xpos
        .get(name)
        .or_else(|| {
            truth
                .xpos
                .iter()
                .find(|(k, _)| k.contains(name))
                .map(|(_, v)| v)
        })
        .filter(|p| p.len() >= 3)
        .map(|p| [p[0], p[1], p[2]])
}

fn ee_xyz(truth: &VerifierTruth, names: &[&str]) -> Option<[f64; 3]> {
    for k in names {
        if let Some(p) = truth.named_pos.get(*k).or_else(|| truth.xpos.get(*k)) {
            if p.len() >= 3 {
                return Some([p[0], p[1], p[2]]);
            }
        }
    }
    None
}

fn dist(a: [f64; 3], b: [f64; 3]) -> f64 {
    mag(sub(a, b))
}
fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn add(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}
fn mag(a: [f64; 3]) -> f64 {
    (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt()
}

pub fn opening_from_truth(
    truth: &VerifierTruth,
    resource: &ControlledResource,
    manifest: &RobotManifest,
) -> Option<f64> {
    let mut qs = Vec::new();
    for jn in &resource.affected_joints {
        let j = manifest.joints.iter().find(|j| j.name == *jn)?;
        let q = *truth.qpos.get(j.qpos_address as usize)?;
        if let Some([lo, hi]) = resource.opening_range.value {
            let span = hi - lo;
            if span.abs() > 1e-9 {
                qs.push(((q - lo) / span).clamp(0.0, 1.0));
            }
        }
    }
    if qs.is_empty() {
        let act = resource.actuator_inputs.first()?;
        let i = manifest.actuators.iter().position(|a| a.name == *act)?;
        let cmd = *truth.ctrl.get(i)?;
        resource.opening_from_command(cmd)
    } else {
        Some(qs.iter().sum::<f64>() / qs.len() as f64)
    }
}
