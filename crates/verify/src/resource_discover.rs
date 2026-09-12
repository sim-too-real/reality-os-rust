//! Discover manipulation resources from inspected model structure. No robot-name branches.

use crate::bundle::RobotBundle;
use crate::normalize::RobotManifest;
use realityos_semantics::provenance::Provenanced;
use realityos_semantics::resource::{
    ClosingDirection, ControlledResource, CouplingModel, JointEquality, QualificationStatus,
    ResourceKind, ResourceTopology, TendonWrap,
};
use serde_json::Value;
use std::collections::{BTreeSet, HashMap};

pub fn discover_resources(
    bundle: &RobotBundle,
    manifest: &RobotManifest,
    inspect: &Value,
) -> Vec<ControlledResource> {
    let tendons = parse_tendons(inspect);
    let equalities = parse_equalities(inspect);
    let children = body_children(manifest);
    let mut out = Vec::new();
    if bundle.manifest.grippers.is_empty() {
        return out;
    }
    for g in &bundle.manifest.grippers {
        out.push(discover_one(g, manifest, &tendons, &equalities, &children));
    }
    out
}

fn discover_one(
    g: &crate::bundle::NamedRef,
    manifest: &RobotManifest,
    tendons: &[TendonWrap],
    equalities: &[JointEquality],
    children: &HashMap<String, Vec<String>>,
) -> ControlledResource {
    let named_act = g.joint.clone().unwrap_or_default();
    let mut actuator_inputs = Vec::new();
    let mut affected = BTreeSet::new();

    if !named_act.is_empty() {
        if let Some(a) = manifest.actuators.iter().find(|a| a.name == named_act) {
            actuator_inputs.push(a.name.clone());
            collect_affected(a, tendons, &mut affected);
        }
    }

    let ee_joints: BTreeSet<String> = manifest
        .derived
        .end_effector_joint_chains
        .iter()
        .flatten()
        .cloned()
        .collect();

    if let Some(body) = &g.body {
        let subtree = subtree_of(body, children);
        for j in &manifest.joints {
            if subtree.contains(&j.child_body)
                && j.joint_type != "free"
                && !ee_joints.contains(&j.name)
            {
                affected.insert(j.name.clone());
            }
        }
        for a in &manifest.actuators {
            if a.transmission_kind == "joint"
                && affected.contains(&a.transmission_target)
                && !actuator_inputs.contains(&a.name)
            {
                actuator_inputs.push(a.name.clone());
            }
            if a.transmission_kind == "tendon" {
                if let Some(t) = tendons.iter().find(|t| t.tendon == a.transmission_target) {
                    if t.joints.iter().any(|(jn, _)| affected.contains(jn))
                        && !actuator_inputs.contains(&a.name)
                    {
                        actuator_inputs.push(a.name.clone());
                    }
                }
            }
        }
    }

    if actuator_inputs.is_empty() {
        for a in &manifest.actuators {
            if a.transmission_kind == "tendon" {
                actuator_inputs.push(a.name.clone());
                collect_affected(a, tendons, &mut affected);
            }
        }
    }

    let affected_joints: Vec<String> = affected.iter().cloned().collect();
    let mut coupling = CouplingModel::none();
    for t in tendons {
        if t.joints.iter().any(|(jn, _)| affected.contains(jn)) {
            coupling.tendon = Some(t.clone());
        }
    }
    coupling.equalities = equalities
        .iter()
        .filter(|e| affected.contains(&e.joint_a) || affected.contains(&e.joint_b))
        .cloned()
        .collect();

    let tendon_act = actuator_inputs.iter().any(|n| {
        manifest
            .actuators
            .iter()
            .any(|a| a.name == *n && a.transmission_kind == "tendon")
    });
    let (topology, unsupported_detail) = classify(
        &actuator_inputs,
        &affected_joints,
        &coupling,
        tendon_act,
        manifest,
    );

    let finger_bodies: Vec<String> = manifest
        .joints
        .iter()
        .filter(|j| affected.contains(&j.name))
        .map(|j| j.child_body.clone())
        .collect();

    let command_range = actuator_inputs
        .first()
        .and_then(|n| manifest.actuators.iter().find(|a| a.name == *n))
        .map(|a| a.ctrlrange);
    let opening_range = affected_joints
        .first()
        .and_then(|n| manifest.joints.iter().find(|j| j.name == *n))
        .filter(|j| j.limited)
        .map(|j| j.range);

    let force_bound = actuator_inputs
        .first()
        .and_then(|n| manifest.actuators.iter().find(|a| a.name == *n))
        .and_then(|a| a.force_range);

    ControlledResource {
        id: g.name.clone(),
        kind: ResourceKind::Gripper,
        topology,
        actuator_inputs,
        affected_joints,
        finger_bodies,
        coupling,
        command_coordinate: if tendon_act {
            "tendon_ctrl".into()
        } else {
            "opening".into()
        },
        opening_range: match opening_range {
            Some(r) => Provenanced::simulator_derived(r, "verify.inspect", 0.0),
            None => Provenanced::unknown("verify.inspect", 0.0),
        },
        command_range: match command_range {
            Some(r) => Provenanced::simulator_derived(r, "verify.inspect", 0.0),
            None => Provenanced::unknown("verify.inspect", 0.0),
        },
        closing_direction: ClosingDirection::TowardMin,
        force_bound: match force_bound {
            Some(r) => Provenanced::declared(r, "verify.inspect", 0.0),
            None => Provenanced::unknown("verify.inspect", 0.0),
        },
        qualification: QualificationStatus::Unverified,
        unsupported_detail,
    }
}

fn classify(
    actuators: &[String],
    joints: &[String],
    coupling: &CouplingModel,
    tendon_act: bool,
    manifest: &RobotManifest,
) -> (ResourceTopology, Option<String>) {
    if actuators.is_empty() || joints.is_empty() {
        return (
            ResourceTopology::UnsupportedResourceTopology,
            Some("no_actuator_or_joint_for_gripper".into()),
        );
    }
    if tendon_act {
        if coupling.tendon.is_none() {
            return (
                ResourceTopology::UnsupportedResourceTopology,
                Some("ACTUATOR_TARGETS_TENDON without wrap joints".into()),
            );
        }
        return (ResourceTopology::TendonDrivenGripper, None);
    }
    if !coupling.equalities.is_empty() && actuators.len() == 1 && joints.len() > 1 {
        return (ResourceTopology::CoupledJointGripper, None);
    }
    let each_joint_has_act = joints.iter().all(|j| {
        manifest
            .actuators
            .iter()
            .any(|a| a.transmission_kind != "tendon" && a.transmission_target == *j)
    });
    if each_joint_has_act {
        return (ResourceTopology::DirectJointGripper, None);
    }
    (
        ResourceTopology::UnsupportedResourceTopology,
        Some("MODEL_FEATURE_UNSUPPORTED:unclassified_gripper_coupling".into()),
    )
}

fn collect_affected(a: &crate::normalize::ActuatorRecord, tendons: &[TendonWrap], out: &mut BTreeSet<String>) {
    if a.transmission_kind == "tendon" {
        if let Some(t) = tendons.iter().find(|t| t.tendon == a.transmission_target) {
            for (j, _) in &t.joints {
                out.insert(j.clone());
            }
        }
    } else if !a.transmission_target.is_empty() {
        out.insert(a.transmission_target.clone());
    }
}

fn parse_tendons(inspect: &Value) -> Vec<TendonWrap> {
    inspect
        .get("tendons")
        .and_then(|v| v.as_array())
        .map(|arr| {
            arr.iter()
                .map(|t| TendonWrap {
                    tendon: t["name"].as_str().unwrap_or("").into(),
                    joints: t
                        .get("joints")
                        .and_then(|v| v.as_array())
                        .map(|js| {
                            js.iter()
                                .filter_map(|j| {
                                    Some((
                                        j["name"].as_str()?.to_string(),
                                        j["coef"].as_f64().unwrap_or(0.0),
                                    ))
                                })
                                .collect()
                        })
                        .unwrap_or_default(),
                })
                .collect()
        })
        .unwrap_or_default()
}

fn parse_equalities(inspect: &Value) -> Vec<JointEquality> {
    inspect
        .get("equalities")
        .and_then(|v| v.as_array())
        .map(|arr| {
            arr.iter()
                .filter(|e| e["type"].as_str() == Some("joint"))
                .map(|e| JointEquality {
                    joint_a: e["obj1"].as_str().unwrap_or("").into(),
                    joint_b: e["obj2"].as_str().unwrap_or("").into(),
                })
                .collect()
        })
        .unwrap_or_default()
}

fn body_children(manifest: &RobotManifest) -> HashMap<String, Vec<String>> {
    let mut m = HashMap::new();
    for b in &manifest.bodies {
        m.entry(b.parent.clone()).or_insert_with(Vec::new).push(b.name.clone());
    }
    m
}

fn subtree_of(root: &str, children: &HashMap<String, Vec<String>>) -> BTreeSet<String> {
    let mut out = BTreeSet::new();
    let mut stack = vec![root.to_string()];
    while let Some(n) = stack.pop() {
        if !out.insert(n.clone()) {
            continue;
        }
        if let Some(ch) = children.get(&n) {
            stack.extend(ch.iter().cloned());
        }
    }
    out
}

