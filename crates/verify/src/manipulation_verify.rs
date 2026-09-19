//! Privileged verifier for manipulation. Never reads a policy success flag.

use crate::normalize::RobotManifest;
use crate::observation::{ContactTruth, VerifierTruth};
use realityos_semantics::contact::{SupportKind, SupportRelation, FORCE_BOUND_UNAVAILABLE};
use realityos_semantics::failure::ManipulationFailure;
use realityos_semantics::provenance::Provenance;
use realityos_semantics::resource::ControlledResource;
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
) -> PrivilegedVerdict {
    let mut evidence = Vec::new();
    let mut notes = Vec::new();
    let finger_contact = after.contacts.iter().any(|c| {
        resource
            .finger_bodies
            .iter()
            .any(|f| c.pair_has(object_id, f))
            || ((c.body1.contains("finger") || c.body2.contains("finger"))
                && (c.body1.contains(object_id) || c.body2.contains(object_id)))
    });
    if finger_contact {
        evidence.push("expected_finger_object_contact".into());
    }
    let empty_close = opening_01.map(|o| o < 0.08).unwrap_or(false) && !finger_contact;
    if empty_close {
        evidence.push("gripper_empty_close".into());
        return fail(ManipulationFailure::GripperEmptyClose, evidence, notes);
    }
    if opening_01.unwrap_or(1.0) > 0.08 && finger_contact {
        evidence.push("closure_blocked_before_empty".into());
    }
    let unexpected = after.contacts.iter().any(|c| {
        unexpected_bodies
            .iter()
            .any(|b| c.body1.contains(b.as_str()) || c.body2.contains(b.as_str()))
    });
    if unexpected {
        evidence.push("unexpected_contact".into());
        return fail(ManipulationFailure::UnexpectedContact, evidence, notes);
    }
    evidence.push("no_forbidden_contact".into());

    let force_unavailable = after.contacts.iter().all(|c| !c.force_available);
    if force_unavailable {
        notes.push(FORCE_BOUND_UNAVAILABLE.into());
    }

    let obj_before = body_xyz(before, object_id);
    let obj_after = body_xyz(after, object_id);
    let ee_before = ee_xyz(before, ee_names);
    let ee_after = ee_xyz(after, ee_names);
    let followed = match (obj_before, obj_after, ee_before, ee_after) {
        (Some(ob), Some(oa), Some(eb), Some(ea)) => {
            let od = sub(oa, ob);
            let ed = sub(ea, eb);
            mag(ed) > 0.012 && dist(od, ed) < 0.025
        }
        _ => false,
    };
    if followed {
        evidence.push("object_follows_ee".into());
    }

    let support = support_of(
        after,
        object_id,
        table_names,
        &resource.finger_bodies,
        now_s,
    );
    let support_changed = !matches!(support.kind, SupportKind::SupportedBy);
    if support_changed {
        evidence.push("support_relation_change".into());
    }

    let used_enough =
        finger_contact && (followed || support_changed || opening_01.unwrap_or(1.0) > 0.08);
    if !used_enough {
        if !finger_contact {
            return fail(ManipulationFailure::Miss, evidence, notes);
        }
        return fail(ManipulationFailure::Slip, evidence, notes);
    }
    PrivilegedVerdict {
        success: true,
        failure: None,
        evidence_used: evidence,
        support: vec![support],
        unexpected_contact: false,
        force_bound_unavailable: force_unavailable,
        object_followed: Some(followed),
        opening_ok: opening_01.map(|o| o < 0.95),
        notes,
    }
}

pub fn verify_push(
    before: &VerifierTruth,
    after: &VerifierTruth,
    object_id: &str,
    direction: [f64; 3],
    min_dist: f64,
    unexpected_bodies: &[String],
    immovable: bool,
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
    let contact = after
        .contacts
        .iter()
        .any(|c| crate::push_pipeline::names_are_ee_object_contact(&c.body1, &c.body2, object_id));
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
            notes,
        };
    }
    if contact {
        return fail(ManipulationFailure::SlipAroundObject, evidence, notes);
    }
    fail(ManipulationFailure::Miss, evidence, notes)
}

fn fail(f: ManipulationFailure, evidence: Vec<String>, notes: Vec<String>) -> PrivilegedVerdict {
    PrivilegedVerdict {
        success: false,
        failure: Some(f.as_str().into()),
        evidence_used: evidence,
        support: vec![],
        unexpected_contact: matches!(f, ManipulationFailure::UnexpectedContact),
        force_bound_unavailable: false,
        object_followed: None,
        opening_ok: None,
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
