//! Embodiment-agnostic MuJoCo verification harness for Reality OS.
//!
//! Layers stay distinct. Simulation never becomes metal proof. SIM ≠ METAL.

pub mod authority;
pub mod bundle;
pub mod corpus;
pub mod driver;
pub mod evidence;
pub mod families;
pub mod format;
pub mod honesty;
pub mod mujoco_exec;
pub mod normalize;
pub mod observation;
pub mod policy;
pub mod qualify;
pub mod reduce;
pub mod runner;
pub mod scenario;
pub mod task;
pub mod validate;
pub mod verifier;

pub use honesty::{
    refuse_physical_proof_origin, VerificationClass, EVIDENCE_SCHEMA, SIMULATION_ONLY,
    SIM_VERIFY_NOT_METAL,
};
pub use runner::{load_and_normalize, qualify_bundle, run_episode, run_matrix, run_resolved};

#[cfg(test)]
mod integration_tests {
    use super::*;
    use crate::bundle::RobotBundle;
    use crate::format::{detect_format, FormatDisposition, ModelFormat};
    use crate::mujoco_exec::mujoco_available;
    use crate::validate::validate_bundle;
    use std::path::Path;

    fn fixture(name: &str) -> std::path::PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("fixtures")
            .join(name)
    }

    #[test]
    fn mjcf_load_three_robots() {
        if !mujoco_available() {
            return;
        }
        for id in corpus::corpus_ids() {
            let b = RobotBundle::load(corpus::robot_dir(id)).expect(id);
            assert_eq!(b.format.format, ModelFormat::Mjcf);
            let (inst, man) = load_and_normalize(&b, &[], 1).expect(id);
            drop(inst);
            assert!(man.nu >= 1, "{id}");
            assert!(!man.model_hash.is_empty());
            assert!(!man.metal);
            assert_eq!(man.evidence_status, SIMULATION_ONLY);
        }
    }

    #[test]
    fn urdf_load() {
        if !mujoco_available() {
            return;
        }
        let b = RobotBundle::load(fixture("urdf_slider")).unwrap();
        assert_eq!(b.format.format, ModelFormat::Urdf);
        let (_i, man) = load_and_normalize(&b, &[], 0).unwrap();
        assert!(man.nq >= 1);
        assert_eq!(man.source_format, "urdf");
    }

    #[test]
    fn sdf_and_step_rejected() {
        let sdf = detect_format(Path::new("a.sdf"), "<sdf version='1.8'/>");
        assert_eq!(
            sdf.disposition,
            FormatDisposition::UnsupportedRequiresConversion
        );
        assert!(RobotBundle::load(fixture("cad_step")).is_err());
    }

    #[test]
    fn missing_asset_and_bad_mapping() {
        let miss = RobotBundle::load(fixture("missing_mesh")).unwrap();
        let r = validate_bundle(&miss, None);
        assert!(!r.ok());
        if mujoco_available() {
            let bad = RobotBundle::load(fixture("bad_actuator"));
            if let Ok(b) = bad {
                let loaded = load_and_normalize(&b, &[], 0);
                assert!(
                    loaded.is_err()
                        || {
                            let man = &loaded.as_ref().unwrap().1;
                            crate::validate::validate_bundle(&b, Some(man))
                                .errors
                                .iter()
                                .any(|e| e.contains("transmission"))
                        }
                        || loaded.is_err()
                );
            }
        }
    }

    #[test]
    fn passive_joint_and_position_limited() {
        if !mujoco_available() {
            return;
        }
        let b = RobotBundle::load(corpus::robot_dir("cartpole")).unwrap();
        let (_i, man) = load_and_normalize(&b, &[], 0).unwrap();
        assert!(
            !man.derived.passive_dofs.is_empty(),
            "pole should be passive"
        );
        assert!(man.actuators.iter().any(|a| a.ctrllimited));
        let (mut inst2, man2) = load_and_normalize(&b, &[], 0).unwrap();
        let q = crate::qualify::qualify(&mut inst2, &man2).unwrap();
        assert!(matches!(
            q.class,
            crate::qualify::ControlClass::Underactuated
                | crate::qualify::ControlClass::PartiallyActuated
        ));
        assert_eq!(q.evidence_status, SIMULATION_ONLY);
        assert!(q.local_linear_controllability.is_some());
        assert_eq!(
            q.local_linear_controllability.unwrap().label,
            "LOCAL_LINEAR_CONTROLLABILITY"
        );
    }

    #[test]
    fn authority_negative_families_do_not_actuate() {
        if !mujoco_available() {
            return;
        }
        let b = RobotBundle::load(corpus::robot_dir("planar_arm")).unwrap();
        for family in [
            "WRONG_ROBOT_IDENTITY",
            "WRONG_TASK_AUTHORITY",
            "POLICY_CRASH",
            "COMMAND_REPLAY",
            "DUPLICATE_COMMAND",
            "STALE_OBSERVATION",
        ] {
            let ep = run_episode(&b, family, 3, "pd").unwrap();
            assert_eq!(ep.schema, EVIDENCE_SCHEMA);
            assert!(!ep.metal);
            if family == "WRONG_ROBOT_IDENTITY"
                || family == "WRONG_TASK_AUTHORITY"
                || family == "POLICY_CRASH"
                || family == "STALE_OBSERVATION"
            {
                assert_eq!(ep.ctrl_writes, 0, "{family} actuated");
            }
            refuse_physical_proof_origin(&ep.to_json_value()).unwrap();
        }
    }

    #[test]
    fn authorized_motion_changes_state() {
        if !mujoco_available() {
            return;
        }
        let b = RobotBundle::load(corpus::robot_dir("planar_arm")).unwrap();
        let ep = run_episode(&b, "JOINT_TRACKING", 1, "pd").unwrap();
        assert!(ep.ctrl_writes > 0, "authorized tracking should actuate");
        assert_eq!(ep.verification_class, "SIMULATION_VERIFIED");
    }

    #[test]
    fn seeds_are_deterministic() {
        if !mujoco_available() {
            return;
        }
        let b = RobotBundle::load(corpus::robot_dir("planar_arm")).unwrap();
        let a = run_episode(&b, "JOINT_TRACKING", 11, "pd").unwrap();
        let c = run_episode(&b, "JOINT_TRACKING", 11, "pd").unwrap();
        let d = run_episode(&b, "JOINT_TRACKING", 12, "pd").unwrap();
        assert_eq!(a.resolved_parameters, c.resolved_parameters);
        assert_ne!(a.resolved_parameters, d.resolved_parameters);
        assert_eq!(a.robot_hash, c.robot_hash);
    }

    #[test]
    fn three_robots_pass_control_qualification() {
        if !mujoco_available() {
            return;
        }
        for id in corpus::corpus_ids() {
            let b = RobotBundle::load(corpus::robot_dir(id)).unwrap();
            let q = qualify_bundle(&b).expect(id);
            assert_ne!(
                q.class,
                crate::qualify::ControlClass::ControlMappingInvalid,
                "{id}"
            );
            assert_ne!(
                q.class,
                crate::qualify::ControlClass::ControllerUnstable,
                "{id}"
            );
            assert!(!q.probes.is_empty(), "{id}");
            assert!(q.rest_recovery_ok, "{id}");
        }
    }

    #[test]
    fn contact_violation_is_detected() {
        let mut env = crate::scenario::EnvelopeSpec::default();
        env.forbidden_contact_pairs
            .push(("link1".into(), "cube_1".into()));
        let man = crate::normalize::RobotManifest {
            robot_id: "t".into(),
            nq: 1,
            nv: 1,
            nu: 1,
            nbody: 2,
            njoint: 0,
            nactuator: 0,
            nsensor: 0,
            ncamera: 0,
            timestep: 0.002,
            joints: vec![],
            actuators: vec![],
            sensors: vec![],
            cameras: vec![],
            bodies: vec![],
            sites: vec![],
            derived: crate::normalize::DerivedInterface {
                base_type: crate::bundle::BaseType::Fixed,
                actuated_dofs: vec![],
                passive_dofs: vec![],
                end_effector_chains: vec![],
                actuator_coverage: 0.0,
                potentially_uncontrollable_joints: vec![],
            },
            model_hash: "h".into(),
            source_hash: "s".into(),
            mujoco_version: "3".into(),
            source_format: "mjcf".into(),
            lost_features: vec![],
            metal: false,
            evidence_status: SIMULATION_ONLY.into(),
        };
        let mut truth = crate::observation::VerifierTruth {
            contacts: vec![crate::observation::ContactTruth {
                body1: "link1".into(),
                body2: "cube_1".into(),
                dist: -0.001,
                force: 3.0,
            }],
            ..crate::observation::VerifierTruth::default()
        };
        let v = crate::verifier::inspect_step(&man, &env, &mut truth, &["link1".into()]);
        assert!(v.iter().any(|x| x.code == "FORBIDDEN_CONTACT"));
    }

    #[test]
    fn pty_metal_cannot_originate_here() {
        let v = serde_json::json!({
            "schema": crate::honesty::METAL_PROOF_SCHEMA,
            "evidence_status": crate::honesty::PTY_STAND_IN,
            "metal": false
        });
        assert!(refuse_physical_proof_origin(&v).is_err());
    }

    #[test]
    fn observation_freshness_refuses_before_ctrl() {
        if !mujoco_available() {
            return;
        }
        let b = RobotBundle::load(corpus::robot_dir("planar_arm")).unwrap();
        let ep = run_episode(&b, "STALE_OBSERVATION", 4, "pd").unwrap();
        assert_eq!(ep.ctrl_writes, 0);
        assert!(ep.authority_refusals > 0);
        let ctrl = crate::mujoco_exec::json_f64_vec(&ep.final_state["ctrl"]);
        assert!(ctrl.iter().all(|x| x.abs() < 1e-9) || ctrl.is_empty());
    }

    #[test]
    fn command_replay_and_wrong_identity_refuse() {
        if !mujoco_available() {
            return;
        }
        let b = RobotBundle::load(corpus::robot_dir("planar_arm")).unwrap();
        let replay = run_episode(&b, "COMMAND_REPLAY", 5, "pd").unwrap();
        assert!(replay.task_success);
        assert!(replay.authority_refusals > 0);
        let ident = run_episode(&b, "WRONG_ROBOT_IDENTITY", 5, "pd").unwrap();
        assert_eq!(ident.ctrl_writes, 0);
        assert!(ident.authority_refusals > 0);
        assert!(ident.violations.iter().any(|v| v.code.contains("WRONG")
            || v.kind == crate::verifier::ViolationKind::AuthorityViolation));
    }

    #[test]
    fn policy_crash_produces_no_actuation() {
        if !mujoco_available() {
            return;
        }
        let b = RobotBundle::load(corpus::robot_dir("planar_arm")).unwrap();
        let ep = run_episode(&b, "POLICY_CRASH", 2, "crash").unwrap();
        assert_eq!(ep.ctrl_writes, 0);
        assert_eq!(ep.termination_reason, "POLICY_CRASH");
    }

    #[test]
    fn simulator_nan_terminates() {
        if !mujoco_available() {
            return;
        }
        let b = RobotBundle::load(corpus::robot_dir("planar_arm")).unwrap();
        let (mut inst, man) = load_and_normalize(&b, &[], 0).unwrap();
        let injected = inst.inject_nan().expect("inject_nan");
        assert_eq!(injected["nan"], true);
        let mut truth = crate::observation::VerifierTruth::from_mujoco_state(&injected["state"]);
        let env = crate::scenario::EnvelopeSpec::default();
        let v = crate::verifier::inspect_step(&man, &env, &mut truth, &[]);
        assert!(v.iter().any(|x| x.code == "NAN_STATE" && x.critical));
        assert!(v.iter().any(|x| x.code == "DIVERGENT_SIMULATION"));
        crate::mujoco_exec::checkin_worker(inst);
    }

    #[test]
    fn evidence_serialization_roundtrip() {
        if !mujoco_available() {
            return;
        }
        let b = RobotBundle::load(corpus::robot_dir("planar_arm")).unwrap();
        let ep = run_episode(&b, "JOINT_TRACKING", 3, "pd").unwrap();
        let dir = std::env::temp_dir().join(format!("ros-ep-{}", std::process::id()));
        let _ = std::fs::create_dir_all(&dir);
        let path = dir.join("episode.json");
        ep.write(&path).unwrap();
        let raw = std::fs::read_to_string(&path).unwrap();
        let v: serde_json::Value = serde_json::from_str(&raw).unwrap();
        assert_eq!(v["schema"], EVIDENCE_SCHEMA);
        assert_eq!(v["metal"], false);
        assert_eq!(v["evidence_status"], SIMULATION_ONLY);
        assert_ne!(v["schema"], crate::honesty::METAL_PROOF_SCHEMA);
        refuse_physical_proof_origin(&v).unwrap();
    }
}
