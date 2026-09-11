//! Map inspected `RobotManifest` + bundle YAML into `EmbodimentModel` v2.

use crate::bundle::{BaseType, FrameReference, NamedRef, RobotBundle};
use crate::normalize::RobotManifest;
use realityos_semantics::embodiment::{
    unknown_se3, Actuator, BaseKind, Body, EmbodimentModel, EndEffector, FrameKind, Gripper, Joint,
    JointKind, ModelDiagnostic, ModelFrame, Transmission,
};
use realityos_semantics::provenance::Provenanced;
use realityos_semantics::transform::Se3;

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
    model.bodies = manifest.bodies.iter().map(map_body).collect();
    model.joints = manifest.joints.iter().map(map_joint).collect();
    model.actuators = manifest.actuators.iter().map(map_actuator).collect();
    model.frames = map_frames(bundle, manifest);
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
        .diagnostics
        .extend(frame_diagnostics(bundle, manifest));
    model.diagnostics.extend(model.validate_transforms());
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
        local_pose: pose_from_parts(b.pos, b.quat),
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
        axis: match j.axis {
            Some(a) if a.iter().any(|v| *v != 0.0) && a.iter().all(|v| v.is_finite()) => {
                Provenanced::simulator_derived(a, SOURCE, 0.0)
            }
            _ => Provenanced::unknown(SOURCE, 0.0),
        },
        qpos_dim: j.qpos_dim.max(0) as usize,
        dof_dim: j.dof_dim.max(0) as usize,
        parent_body: j.parent_body.clone(),
        child_body: j.child_body.clone(),
        q_min,
        q_max,
        dq_max: Provenanced::unknown(SOURCE, 0.0),
        effort_max: Provenanced::unknown(SOURCE, 0.0),
        origin_in_child: match j.pos {
            Some(p) if p.iter().all(|v| v.is_finite()) => {
                Provenanced::simulator_derived(p, SOURCE, 0.0)
            }
            _ => Provenanced::unknown(SOURCE, 0.0),
        },
        parent_to_joint: unknown_se3(SOURCE),
        joint_to_child: unknown_se3(SOURCE),
        qpos_adr: Some(j.qpos_address),
        dof_adr: Some(j.velocity_address),
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
        transmission_kind: if a.transmission_kind.is_empty() {
            "joint".into()
        } else {
            a.transmission_kind.clone()
        },
        ctrlrange,
        forcerange,
        gear: Provenanced::unknown(SOURCE, 0.0),
    }
}

fn site_parent_body(manifest: &RobotManifest, site: &str) -> String {
    manifest
        .sites
        .iter()
        .find(|(name, _)| name == site)
        .map(|(_, body)| body.clone())
        .unwrap_or_default()
}

fn pose_from_parts(pos: Option<[f64; 3]>, quat: Option<[f64; 4]>) -> Provenanced<Se3> {
    match (pos, quat) {
        (Some(xyz), Some(q)) => match Se3::try_new(xyz, q) {
            Ok(pose) => Provenanced::simulator_derived(pose, SOURCE, 0.0),
            Err(_) => Provenanced::unknown(SOURCE, 0.0),
        },
        (Some(xyz), None) => match Se3::translation(xyz) {
            Ok(pose) => Provenanced::simulator_derived(pose, SOURCE, 0.0),
            Err(_) => Provenanced::unknown(SOURCE, 0.0),
        },
        _ => unknown_se3(SOURCE),
    }
}

const BODY_FRAME_SOURCE: &str = "BUNDLE_DECLARED_BODY_FRAME";
const BODY_OFFSET_SOURCE: &str = "BUNDLE_DECLARED_BODY_OFFSET";

fn identity_body_frame_pose() -> (Provenanced<[f64; 3]>, Provenanced<[f64; 4]>) {
    (
        Provenanced::user_declared([0.0, 0.0, 0.0], BODY_FRAME_SOURCE, 0.0),
        Provenanced::user_declared([1.0, 0.0, 0.0, 0.0], BODY_FRAME_SOURCE, 0.0),
    )
}

fn map_ee_frame(ee: &NamedRef, manifest: &RobotManifest) -> ModelFrame {
    let name = ee.semantic_frame_name();
    match ee.frame_reference() {
        Some(FrameReference::Site { site }) => {
            let parent_body = site_parent_body(manifest, &site);
            let rec = manifest.site_records.iter().find(|s| s.name == site);
            let (translation, rotation) = match rec {
                Some(s) => (
                    match s.pos {
                        Some(p) => Provenanced::simulator_derived(p, SOURCE, 0.0),
                        None => Provenanced::unknown(SOURCE, 0.0),
                    },
                    match s.quat {
                        Some(q) => Provenanced::simulator_derived(q, SOURCE, 0.0),
                        None => Provenanced::unknown(SOURCE, 0.0),
                    },
                ),
                None => (
                    Provenanced::unknown(SOURCE, 0.0),
                    Provenanced::unknown(SOURCE, 0.0),
                ),
            };
            ModelFrame {
                name,
                kind: FrameKind::Ee,
                parent_body,
                translation,
                rotation,
            }
        }
        Some(FrameReference::Body { body }) => {
            let (translation, rotation) = identity_body_frame_pose();
            ModelFrame {
                name,
                kind: FrameKind::Ee,
                parent_body: body,
                translation,
                rotation,
            }
        }
        Some(FrameReference::BodyOffset {
            body,
            xyz,
            quat_wxyz,
        }) => ModelFrame {
            name,
            kind: FrameKind::Ee,
            parent_body: body,
            translation: Provenanced::user_declared(xyz, BODY_OFFSET_SOURCE, 0.0),
            rotation: Provenanced::user_declared(quat_wxyz, BODY_OFFSET_SOURCE, 0.0),
        },
        None => ModelFrame {
            name,
            kind: FrameKind::Ee,
            parent_body: String::new(),
            translation: Provenanced::unknown(SOURCE, 0.0),
            rotation: Provenanced::unknown(SOURCE, 0.0),
        },
    }
}

fn map_frames(bundle: &RobotBundle, manifest: &RobotManifest) -> Vec<ModelFrame> {
    let mut frames = Vec::new();
    for ee in &bundle.manifest.end_effectors {
        frames.push(map_ee_frame(ee, manifest));
    }
    for cam in &manifest.cameras {
        frames.push(ModelFrame {
            name: cam.name.clone(),
            kind: FrameKind::Camera,
            parent_body: cam.parent_body.clone(),
            translation: match cam.pos {
                Some(p) => Provenanced::simulator_derived(p, SOURCE, 0.0),
                None => Provenanced::unknown(SOURCE, 0.0),
            },
            rotation: match cam.quat {
                Some(q) => Provenanced::simulator_derived(q, SOURCE, 0.0),
                None => Provenanced::unknown(SOURCE, 0.0),
            },
        });
    }
    frames
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
            frame: ee.semantic_frame_name(),
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

fn frame_diagnostics(bundle: &RobotBundle, manifest: &RobotManifest) -> Vec<ModelDiagnostic> {
    let mut out = Vec::new();
    for ee in &bundle.manifest.end_effectors {
        match ee.frame_reference() {
            None => out.push(ModelDiagnostic {
                code: "MODEL_FEATURE_UNSUPPORTED".into(),
                detail: format!("unknown_frame_reference:{}", ee.name),
            }),
            Some(FrameReference::Site { site }) => {
                if !manifest.site_records.iter().any(|s| s.name == site) {
                    out.push(ModelDiagnostic {
                        code: "MODEL_FEATURE_UNSUPPORTED".into(),
                        detail: format!("missing_site:{}", site),
                    });
                }
            }
            Some(FrameReference::Body { body }) | Some(FrameReference::BodyOffset { body, .. }) => {
                if !manifest.bodies.iter().any(|b| b.name == body) {
                    out.push(ModelDiagnostic {
                        code: "MODEL_FEATURE_UNSUPPORTED".into(),
                        detail: format!("missing_body:{}", body),
                    });
                }
            }
        }
    }
    out
}

fn typed_lost_feature(f: &str) -> ModelDiagnostic {
    const TYPED: &[&str] = &[
        "TENDON_PRESENT",
        "ACTUATOR_TARGETS_TENDON",
        "JOINT_EQUALITY_CONSTRAINT",
        "COUPLED_JOINTS",
    ];
    if let Some((code, detail)) = f.split_once(':') {
        if TYPED.contains(&code) {
            return ModelDiagnostic {
                code: code.into(),
                detail: detail.into(),
            };
        }
    } else if TYPED.contains(&f) {
        return ModelDiagnostic {
            code: f.into(),
            detail: String::new(),
        };
    }
    ModelDiagnostic {
        code: "lost_feature".into(),
        detail: f.into(),
    }
}

fn map_diagnostics(manifest: &RobotManifest) -> Vec<ModelDiagnostic> {
    let mut out: Vec<ModelDiagnostic> = manifest
        .lost_features
        .iter()
        .map(|f| typed_lost_feature(f))
        .collect();
    for b in &manifest.bodies {
        if b.inertia.iter().all(|&x| x == 0.0) {
            out.push(ModelDiagnostic {
                code: "missing_inertia".into(),
                detail: b.name.clone(),
            });
        }
    }
    for j in &manifest.joints {
        if matches!(j.joint_type.as_str(), "ball" | "free") {
            out.push(ModelDiagnostic {
                code: "MODEL_FEATURE_UNSUPPORTED".into(),
                detail: format!("{}:{}", j.joint_type, j.name),
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
        assert!(m
            .bodies
            .iter()
            .any(|b| { b.mass_kg.provenance != Provenance::HardwareMeasured }));
        assert!(m
            .joints
            .iter()
            .all(|j| j.axis.value.is_some() && j.axis.provenance == Provenance::SimulatorDerived));
        assert!(m.bodies.iter().all(|b| b.local_pose.value.is_some()));
        assert!(m
            .frames
            .iter()
            .any(|f| f.name == "ee:ee" && f.pose().is_some()));
    }

    fn synth_named(name: &str, site: Option<&str>, body: Option<&str>) -> NamedRef {
        NamedRef {
            name: name.into(),
            site: site.map(|s| s.into()),
            body: body.map(|s| s.into()),
            joint: None,
            xyz: None,
            quat: None,
        }
    }

    fn synth_bundle(ees: Vec<NamedRef>) -> RobotBundle {
        use crate::format::{FormatDiagnosis, FormatDisposition, ModelFormat};
        RobotBundle {
            root: std::path::PathBuf::from("synth"),
            manifest: crate::bundle::RobotYaml {
                robot_id: "synth_body_ee".into(),
                model_format: "mjcf".into(),
                model_file: "model.xml".into(),
                asset_roots: vec![],
                expected_base_type: BaseType::Fixed,
                joint_aliases: Default::default(),
                actuator_aliases: Default::default(),
                end_effectors: ees,
                grippers: vec![],
                feet: vec![],
                cameras: vec![],
                task_frames: vec![],
                collision_groups: Default::default(),
                default_controller_profile: None,
                effort_limit: None,
                effort_units: None,
                source: None,
            },
            model_path: std::path::PathBuf::from("synth/model.xml"),
            model_text: String::new(),
            model_bytes: vec![],
            format: FormatDiagnosis {
                format: ModelFormat::Mjcf,
                disposition: FormatDisposition::Supported,
                path: "synth".into(),
                detail: String::new(),
                lost_or_unreliable: vec![],
            },
            asset_files: vec![],
            source_hash: "synth".into(),
        }
    }

    fn synth_manifest(body: &str, site: Option<(&str, &str, [f64; 3], [f64; 4])>) -> RobotManifest {
        let mut man = RobotManifest {
            robot_id: "synth_body_ee".into(),
            nq: 1,
            nv: 1,
            nu: 1,
            nbody: 2,
            njoint: 1,
            nactuator: 1,
            nsensor: 0,
            ncamera: 0,
            timestep: 0.002,
            joints: vec![crate::normalize::JointRecord {
                name: "j0".into(),
                joint_type: "hinge".into(),
                qpos_address: 0,
                velocity_address: 0,
                qpos_dim: 1,
                dof_dim: 1,
                range: [-1.0, 1.0],
                limited: true,
                parent_body: "world".into(),
                child_body: body.into(),
                unsupported_reason: None,
                axis: Some([0.0, 0.0, 1.0]),
                pos: Some([0.0, 0.0, 0.0]),
            }],
            actuators: vec![],
            sensors: vec![],
            cameras: vec![],
            bodies: vec![crate::normalize::BodyRecord {
                name: body.into(),
                mass: 1.0,
                inertia: [1.0, 1.0, 1.0],
                parent: "world".into(),
                pos: Some([0.0, 0.0, 0.0]),
                quat: Some([1.0, 0.0, 0.0, 0.0]),
            }],
            sites: vec![],
            site_records: vec![],
            derived: crate::normalize::DerivedInterface::default(),
            model_hash: "h".into(),
            source_hash: "s".into(),
            mujoco_version: "3.7.0".into(),
            source_format: "mjcf".into(),
            lost_features: vec![],
            support_bodies: vec![],
            collision_groups: Default::default(),
            metal: false,
            evidence_status: crate::honesty::SIMULATION_ONLY.into(),
        };
        if let Some((name, parent, pos, quat)) = site {
            man.sites.push((name.into(), parent.into()));
            man.site_records.push(crate::normalize::SiteRecord {
                name: name.into(),
                body: parent.into(),
                pos: Some(pos),
                quat: Some(quat),
            });
        }
        man
    }

    #[test]
    fn body_backed_ee_without_site_no_longer_has_unknown_pose() {
        // Characterization (pre-fix): body-only EE pose was None and FK was Unsupported.
        let bundle = synth_bundle(vec![synth_named("tool0", None, Some("wrist"))]);
        let man = synth_manifest("wrist", None);
        let m = embodiment_from_manifest(&bundle, &man);
        let ee = m.end_effectors.iter().find(|e| e.name == "tool0").unwrap();
        let frame = m.frames.iter().find(|f| f.name == ee.frame).unwrap();
        assert!(
            frame.pose().is_some(),
            "body-backed EE must carry identity pose"
        );
        realityos_semantics::kinematics::forward_kinematics(&m, &["j0".into()], "tool0", &[0.0])
            .expect("body-backed EE FK");
    }

    #[test]
    fn bundle_declared_body_frame_is_identity_not_assumed() {
        let bundle = synth_bundle(vec![synth_named("tool0", None, Some("wrist"))]);
        let man = synth_manifest("wrist", None);
        let m = embodiment_from_manifest(&bundle, &man);
        let ee = m
            .end_effectors
            .iter()
            .find(|e| e.name == "tool0")
            .expect("ee");
        assert_eq!(ee.frame, "ee:tool0");
        let frame = m
            .frames
            .iter()
            .find(|f| f.name == "ee:tool0")
            .expect("frame");
        assert_eq!(frame.parent_body, "wrist");
        let pose = frame.pose().expect("declared body frame has identity pose");
        assert_eq!(pose.xyz, [0.0, 0.0, 0.0]);
        assert_eq!(pose.quat_wxyz, [1.0, 0.0, 0.0, 0.0]);
        assert_eq!(frame.translation.provenance, Provenance::UserDeclared);
        assert_eq!(frame.translation.source, "BUNDLE_DECLARED_BODY_FRAME");
        assert_ne!(frame.translation.provenance, Provenance::Assumed);
        realityos_semantics::kinematics::forward_kinematics(&m, &["j0".into()], "tool0", &[0.0])
            .expect("body-backed EE FK must not be unsupported");
    }

    #[test]
    fn site_backed_ee_preserves_site_local_se3() {
        let bundle = synth_bundle(vec![synth_named("ee", Some("tip"), Some("wrist"))]);
        let man = synth_manifest(
            "wrist",
            Some(("tip", "wrist", [0.0, 0.1, 0.0], [-0.707, 0.707, 0.0, 0.0])),
        );
        let m = embodiment_from_manifest(&bundle, &man);
        let ee = m.end_effectors.iter().find(|e| e.name == "ee").unwrap();
        assert_eq!(ee.frame, "ee:ee");
        let frame = m.frames.iter().find(|f| f.name == "ee:ee").unwrap();
        let pose = frame.pose().expect("site pose");
        assert!((pose.xyz[1] - 0.1).abs() < 1e-12);
        assert_eq!(frame.translation.provenance, Provenance::SimulatorDerived);
    }

    #[test]
    fn body_offset_ee_preserves_explicit_offset() {
        let mut ee = synth_named("tool0", None, Some("wrist"));
        ee.xyz = Some([0.0, 0.0, 0.05]);
        ee.quat = Some([1.0, 0.0, 0.0, 0.0]);
        let bundle = synth_bundle(vec![ee]);
        let man = synth_manifest("wrist", None);
        let m = embodiment_from_manifest(&bundle, &man);
        let frame = m.frames.iter().find(|f| f.name == "ee:tool0").unwrap();
        let pose = frame.pose().expect("offset pose");
        assert!((pose.xyz[2] - 0.05).abs() < 1e-12);
        assert_eq!(frame.translation.source, "BUNDLE_DECLARED_BODY_OFFSET");
        assert_ne!(frame.translation.provenance, Provenance::Assumed);
    }

    #[test]
    fn missing_body_or_site_is_model_feature_unsupported() {
        let bundle = synth_bundle(vec![synth_named("ghost", None, None)]);
        let man = synth_manifest("wrist", None);
        let m = embodiment_from_manifest(&bundle, &man);
        assert!(m
            .diagnostics
            .iter()
            .any(|d| d.code == "MODEL_FEATURE_UNSUPPORTED"));
        let err = realityos_semantics::kinematics::forward_kinematics(
            &m,
            &["j0".into()],
            "ghost",
            &[0.0],
        )
        .unwrap_err();
        assert_eq!(
            err,
            realityos_semantics::skill::SkillRefuse::ModelFeatureUnsupported
        );
    }

    #[test]
    fn spatial_arm4_is_not_a_planar_clone() {
        if !ensure_mujoco_or_skip() {
            return;
        }
        let b =
            RobotBundle::load(crate::corpus::bundled_robots_root().join("spatial_arm4")).unwrap();
        let (_i, man) = crate::runner::load_and_normalize(&b, &[], 0).unwrap();
        assert_eq!(man.nu, 4);
        let axes: Vec<String> = man.joints.iter().map(|j| j.name.clone()).collect();
        assert_eq!(axes.len(), 4);
        let m = embodiment_from_manifest(&b, &man);
        let g = realityos_semantics::capability::derive_capabilities(&m, None);
        assert_eq!(
            g.get(realityos_semantics::capability::CapName::FixedBaseManipulation)
                .status,
            realityos_semantics::capability::CapStatus::Supported
        );
    }
}
