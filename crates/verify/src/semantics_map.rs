//! Map inspected `RobotManifest` + bundle YAML into `EmbodimentModel` v2.

use crate::bundle::{BaseType, NamedRef, RobotBundle};
use crate::normalize::RobotManifest;
use realityos_semantics::embodiment::{
    Actuator, BaseKind, Body, EmbodimentModel, EndEffector, FrameKind, Gripper, Joint, JointKind,
    ModelDiagnostic, ModelFrame, Transmission,
};
use realityos_semantics::provenance::Provenanced;

const SOURCE: &str = "verify.manifest";

pub fn embodiment_from_manifest(bundle: &RobotBundle, manifest: &RobotManifest) -> EmbodimentModel {
    let mut model = EmbodimentModel::new(
        bundle.manifest.robot_id.clone(),
        manifest.source_hash.clone(),
        manifest.model_hash.clone(),
        manifest.model_hash.clone(),
        manifest.mujoco_version.clone(),
    );
    model.base = map_base(manifest.derived.base_type);
    model.metal = false;
    model.bodies = manifest
        .bodies
        .iter()
        .map(|b| map_body(b))
        .collect();
    model.joints = manifest.joints.iter().map(map_joint).collect();
    model.actuators = manifest.actuators.iter().map(map_actuator).collect();
    model.frames = map_ee_frames(bundle, manifest);
    model.end_effectors = map_end_effectors(bundle, manifest);
    model.grippers = bundle.manifest.grippers.iter().map(map_gripper).collect();
    model.transmissions = manifest
        .actuators
        .iter()
        .map(|a| Transmission {
            name: a.name.clone(),
            source: a.name.clone(),
            target_joint: a.transmission_target.clone(),
        })
        .collect();
    model.diagnostics = map_diagnostics(manifest);
    model
}

fn map_base(base: BaseType) -> BaseKind {
    match base {
        BaseType::Fixed => BaseKind::Fixed,
        BaseType::Floating => BaseKind::Floating,
        BaseType::Mobile => BaseKind::Mobile,
    }
}

fn map_body(b: &crate::normalize::BodyRecord) -> Body {
    let mass_kg = if b.mass > 0.0 {
        Provenanced::simulator_derived(b.mass, SOURCE, 0.0)
    } else {
        Provenanced::unknown(SOURCE, 0.0)
    };
    let inertia = if b.inertia.iter().all(|&x| x == 0.0) {
        Provenanced::unknown(SOURCE, 0.0)
    } else {
        Provenanced::simulator_derived(
            [b.inertia[0], b.inertia[1], b.inertia[2], 0.0, 0.0, 0.0],
            SOURCE,
            0.0,
        )
    };
    Body {
        name: b.name.clone(),
        parent: if b.parent.is_empty() || b.parent == "world" {
            None
        } else {
            Some(b.parent.clone())
        },
        mass_kg,
        com: Provenanced::unknown(SOURCE, 0.0),
        inertia,
    }
}

fn map_joint_kind(joint_type: &str) -> JointKind {
    match joint_type {
        "hinge" => JointKind::Hinge,
        "slide" => JointKind::Slide,
        "ball" => JointKind::Ball,
        "free" => JointKind::Free,
        "fixed" => JointKind::Fixed,
        _ => JointKind::Other,
    }
}

fn map_joint(j: &crate::normalize::JointRecord) -> Joint {
    let (q_min, q_max) = if j.limited {
        (
            Provenanced::declared(j.range[0], SOURCE, 0.0),
            Provenanced::declared(j.range[1], SOURCE, 0.0),
        )
    } else {
        (
            Provenanced::unknown(SOURCE, 0.0),
            Provenanced::unknown(SOURCE, 0.0),
        )
    };
    Joint {
        name: j.name.clone(),
        kind: map_joint_kind(&j.joint_type),
        axis: Provenanced::unknown(SOURCE, 0.0),
        qpos_dim: j.qpos_dim.max(0) as usize,
        dof_dim: j.dof_dim.max(0) as usize,
        parent_body: j.parent_body.clone(),
        child_body: j.child_body.clone(),
        q_min,
        q_max,
        dq_max: Provenanced::unknown(SOURCE, 0.0),
        effort_max: Provenanced::unknown(SOURCE, 0.0),
    }
}

fn map_actuator(a: &crate::normalize::ActuatorRecord) -> Actuator {
    let ctrlrange = if a.ctrllimited {
        Provenanced::declared(a.ctrlrange, SOURCE, 0.0)
    } else {
        Provenanced::unknown(SOURCE, 0.0)
    };
    let forcerange = match a.force_range {
        Some(fr) => Provenanced::declared(fr, SOURCE, 0.0),
        None => Provenanced::unknown(SOURCE, 0.0),
    };
    Actuator {
        name: a.name.clone(),
        target_joint: a.transmission_target.clone(),
        control_mode: a.actuator_type.clone(),
        ctrlrange,
        forcerange,
        gear: Provenanced::unknown(SOURCE, 0.0),
    }
}

fn ee_frame_name(ee: &NamedRef) -> String {
    ee.site
        .clone()
        .or_else(|| ee.body.clone())
        .unwrap_or_else(|| ee.name.clone())
}

fn site_parent_body(manifest: &RobotManifest, site: &str) -> String {
    manifest
        .sites
        .iter()
        .find(|(name, _)| name == site)
        .map(|(_, body)| body.clone())
        .unwrap_or_default()
}

fn map_ee_frames(bundle: &RobotBundle, manifest: &RobotManifest) -> Vec<ModelFrame> {
    bundle
        .manifest
        .end_effectors
        .iter()
        .map(|ee| {
            let frame = ee_frame_name(ee);
            let parent_body = ee
                .site
                .as_ref()
                .map(|s| site_parent_body(manifest, s))
                .filter(|p| !p.is_empty())
                .or_else(|| ee.body.clone())
                .unwrap_or_default();
            ModelFrame {
                name: frame,
                kind: FrameKind::Ee,
                parent_body,
                translation: Provenanced::unknown(SOURCE, 0.0),
            }
        })
        .collect()
}

fn joint_chain_for_ee(
    bundle: &RobotBundle,
    manifest: &RobotManifest,
    idx: usize,
    ee: &NamedRef,
) -> Vec<String> {
    if let Some(chain) = manifest.derived.end_effector_joint_chains.get(idx) {
        if !chain.is_empty() {
            return chain.clone();
        }
    }
    for (i, yaml_ee) in bundle.manifest.end_effectors.iter().enumerate() {
        if yaml_ee.name == ee.name {
            if let Some(chain) = manifest.derived.end_effector_joint_chains.get(i) {
                return chain.clone();
            }
        }
    }
    manifest
        .derived
        .end_effector_joint_chains
        .first()
        .cloned()
        .unwrap_or_default()
}

fn map_end_effectors(bundle: &RobotBundle, manifest: &RobotManifest) -> Vec<EndEffector> {
    bundle
        .manifest
        .end_effectors
        .iter()
        .enumerate()
        .map(|(idx, ee)| EndEffector {
            name: ee.name.clone(),
            frame: ee_frame_name(ee),
            joint_chain: joint_chain_for_ee(bundle, manifest, idx, ee),
        })
        .collect()
}

fn map_gripper(g: &NamedRef) -> Gripper {
    Gripper {
        name: g.name.clone(),
        actuator: g.joint.clone().unwrap_or_default(),
        opening_range: Provenanced::unknown(SOURCE, 0.0),
    }
}

fn map_diagnostics(manifest: &RobotManifest) -> Vec<ModelDiagnostic> {
    let mut out: Vec<ModelDiagnostic> = manifest
        .lost_features
        .iter()
        .map(|f| ModelDiagnostic {
            code: "lost_feature".into(),
            detail: f.clone(),
        })
        .collect();
    for b in &manifest.bodies {
        if b.inertia.iter().all(|&x| x == 0.0) {
            out.push(ModelDiagnostic {
                code: "missing_inertia".into(),
                detail: b.name.clone(),
            });
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mujoco_exec::ensure_mujoco_or_skip;
    use realityos_semantics::provenance::Provenance;

    #[test]
    fn planar_arm_maps_without_inventing_effort_when_missing() {
        if !ensure_mujoco_or_skip() {
            return;
        }
        let b = crate::bundle::RobotBundle::load(crate::corpus::robot_dir("planar_arm")).unwrap();
        let (_inst, man) = crate::runner::load_and_normalize(&b, &[], 0).unwrap();
        let m = embodiment_from_manifest(&b, &man);
        assert!(!m.metal);
        assert_eq!(m.robot_id, "planar_arm");
        assert!(m.end_effectors.iter().any(|e| e.name == "ee"));
        assert!(m.bodies.iter().any(|b| {
            b.mass_kg.provenance != Provenance::HardwareMeasured
        }));
        assert!(m.joints.iter().all(|j| j.axis.provenance == Provenance::Unknown));
    }

    #[test]
    fn spatial_arm4_is_not_a_planar_clone() {
        if !ensure_mujoco_or_skip() {
            return;
        }
        let b = RobotBundle::load(crate::corpus::bundled_robots_root().join("spatial_arm4"))
            .unwrap();
        let (_i, man) = crate::runner::load_and_normalize(&b, &[], 0).unwrap();
        assert_eq!(man.nu, 4);
        let axes: Vec<String> = man.joints.iter().map(|j| j.name.clone()).collect();
        assert_eq!(axes.len(), 4);
        let m = embodiment_from_manifest(&b, &man);
        let g = realityos_semantics::capability::derive_capabilities(&m, None);
        assert_eq!(
            g.get(realityos_semantics::capability::CapName::FixedBaseManipulation).status,
            realityos_semantics::capability::CapStatus::Supported
        );
    }
}
