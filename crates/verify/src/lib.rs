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
pub mod foundation_report;
pub mod held_out;
pub mod honesty;
pub mod mujoco_exec;
pub mod normalize;
pub mod observation;
pub mod policy;
pub mod qualify;
pub mod reach_foundation;
pub mod reduce;
pub mod runner;
pub mod scenario;
pub mod semantics_map;
pub mod task;
pub mod validate;
pub mod verifier;

pub use honesty::{
    refuse_physical_proof_origin, VerificationClass, EVIDENCE_SCHEMA, SIMULATION_ONLY,
    SIM_VERIFY_NOT_METAL,
};
pub use reach_foundation::{run_foundation_reach, FoundationReachReport};
pub use runner::{load_and_normalize, qualify_bundle, run_episode, run_matrix, run_resolved};

#[cfg(test)]
mod integration_tests {
    use super::*;
    use crate::bundle::RobotBundle;
    use crate::format::{detect_format, FormatDisposition, ModelFormat};
    use crate::mujoco_exec::ensure_mujoco_or_skip;
    use crate::validate::validate_bundle;
    use std::path::Path;

    fn fixture(name: &str) -> std::path::PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("fixtures")
            .join(name)
    }

    #[test]
    fn mjcf_load_three_robots() {
        if !ensure_mujoco_or_skip() {
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
        if !ensure_mujoco_or_skip() {
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
        if ensure_mujoco_or_skip() {
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
        if !ensure_mujoco_or_skip() {
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
        match q.local_linear_controllability {
            Some(llc) => {
                assert_eq!(llc.label, "LOCAL_LINEAR_CONTROLLABILITY");
                assert_eq!(llc.method, "mjd_transitionFD");
                assert!(llc.state_dim > 0);
                assert_eq!(llc.a_shape[0], llc.state_dim);
                assert_eq!(llc.b_shape[1], llc.input_dim);
            }
            None => assert!(q.note.contains("NOT_EVALUATED")),
        }
    }

    #[test]
    fn authority_negative_families_do_not_actuate() {
        if !ensure_mujoco_or_skip() {
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
        if !ensure_mujoco_or_skip() {
            return;
        }
        let b = RobotBundle::load(corpus::robot_dir("planar_arm")).unwrap();
        let ep = run_episode(&b, "JOINT_TRACKING", 1, "pd").unwrap();
        assert!(ep.ctrl_writes > 0, "authorized tracking should actuate");
        assert_eq!(ep.verification_class, "SIMULATION_VERIFIED");
    }

    #[test]
    fn seeds_are_deterministic() {
        if !ensure_mujoco_or_skip() {
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
        if !ensure_mujoco_or_skip() {
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
                end_effector_joint_chains: vec![],
                actuator_coverage: 0.0,
                potentially_uncontrollable_joints: vec![],
            },
            model_hash: "h".into(),
            source_hash: "s".into(),
            mujoco_version: "3".into(),
            source_format: "mjcf".into(),
            lost_features: vec![],
            support_bodies: vec![],
            collision_groups: Default::default(),
            metal: false,
            evidence_status: SIMULATION_ONLY.into(),
        };
        let mut truth = crate::observation::VerifierTruth {
            contacts: vec![crate::observation::ContactTruth {
                body1: "link1".into(),
                body2: "cube_1".into(),
                dist: -0.001,
                force: 3.0,
                group1: 0,
                group2: 0,
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
        if !ensure_mujoco_or_skip() {
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
        if !ensure_mujoco_or_skip() {
            return;
        }
        let b = RobotBundle::load(corpus::robot_dir("planar_arm")).unwrap();
        let replay = run_episode(&b, "COMMAND_REPLAY", 5, "pd").unwrap();
        assert!(
            replay.task_success,
            "status={} writes={} delta={:?} refusals={} decisions={:?}",
            replay.episode_status,
            replay.policy_ctrl_writes,
            replay.replay_write_delta,
            replay.authority_refusals,
            replay
                .authority_decisions
                .iter()
                .map(|d| (d.executed, d.violations.clone()))
                .collect::<Vec<_>>()
        );
        assert!(replay.authority_refusals > 0);
        let ident = run_episode(&b, "WRONG_ROBOT_IDENTITY", 5, "pd").unwrap();
        assert_eq!(ident.ctrl_writes, 0);
        assert!(ident.authority_refusals > 0);
        assert!(ident.violations.iter().any(|v| v.code.contains("WRONG")
            || v.kind == crate::verifier::ViolationKind::AuthorityViolation));
    }

    #[test]
    fn policy_crash_produces_no_actuation() {
        if !ensure_mujoco_or_skip() {
            return;
        }
        let b = RobotBundle::load(corpus::robot_dir("planar_arm")).unwrap();
        let ep = run_episode(&b, "POLICY_CRASH", 2, "crash").unwrap();
        assert_eq!(ep.ctrl_writes, 0);
        assert_eq!(ep.termination_reason, "POLICY_CRASH");
    }

    #[test]
    fn simulator_nan_terminates() {
        if !ensure_mujoco_or_skip() {
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
        if !ensure_mujoco_or_skip() {
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

    #[test]
    fn mesh_bundle_loads_from_unrelated_cwd() {
        if !ensure_mujoco_or_skip() {
            return;
        }
        let b = RobotBundle::load(fixture("mesh_cube")).unwrap();
        let cwd = std::env::current_dir().unwrap();
        let tmp = std::env::temp_dir().join(format!("ros-cwd-{}", std::process::id()));
        let _ = std::fs::create_dir_all(&tmp);
        std::env::set_current_dir(&tmp).unwrap();
        let (inst, man) = load_and_normalize(&b, &[], 1).expect("compile from other cwd");
        let hash1 = man.model_hash.clone();
        let src1 = man.source_hash.clone();
        crate::mujoco_exec::checkin_worker(inst);
        std::env::set_current_dir(&cwd).unwrap();
        let (inst2, man2) = load_and_normalize(&b, &[], 1).unwrap();
        assert_eq!(hash1, man2.model_hash);
        assert_eq!(src1, man2.source_hash);
        crate::mujoco_exec::checkin_worker(inst2);
    }

    #[test]
    fn urdf_with_scenario_object_steps() {
        if !ensure_mujoco_or_skip() {
            return;
        }
        let b = RobotBundle::load(fixture("urdf_slider")).unwrap();
        let objects = vec![
            serde_json::json!({"name":"cube_1","type":"box","size":[0.03,0.03,0.03],"pos":[0.15,0.0,0.05]}),
        ];
        let (mut inst, man) = load_and_normalize(&b, &objects, 2).unwrap();
        assert_eq!(man.source_format, "urdf");
        assert!(man.nbody >= 3, "scenario body must be compiled in");
        let j = man
            .joints
            .iter()
            .find(|j| j.name == "slide")
            .expect("slide");
        assert_ne!(j.parent_body, j.child_body);
        let stepped = inst.step(5).unwrap();
        assert_eq!(stepped["ok"], true);
        crate::mujoco_exec::checkin_worker(inst);
    }

    #[test]
    fn hinge_slide_free_ball_topology() {
        if !ensure_mujoco_or_skip() {
            return;
        }
        let arm = RobotBundle::load(corpus::robot_dir("planar_arm")).unwrap();
        let (_i, man) = load_and_normalize(&arm, &[], 0).unwrap();
        assert!(man
            .joints
            .iter()
            .any(|j| j.joint_type == "hinge" && j.parent_body != j.child_body));
        assert!(!man.derived.end_effector_joint_chains.is_empty());
        assert_eq!(man.q_min().len(), man.nq as usize);
        assert_eq!(man.q_max().len(), man.nq as usize);
        crate::mujoco_exec::checkin_worker(_i);

        let slide = RobotBundle::load(fixture("urdf_slider")).unwrap();
        let (_i, man) = load_and_normalize(&slide, &[], 0).unwrap();
        assert!(man.joints.iter().any(|j| j.joint_type == "slide"));
        crate::mujoco_exec::checkin_worker(_i);

        let free = RobotBundle::load(fixture("free_base")).unwrap();
        let (_i, man) = load_and_normalize(&free, &[], 0).unwrap();
        let root = man.joints.iter().find(|j| j.joint_type == "free").unwrap();
        assert_eq!(root.qpos_dim, 7);
        assert_eq!(man.q_min().len(), man.nq as usize);
        crate::mujoco_exec::checkin_worker(_i);

        let ball = RobotBundle::load(fixture("ball_joint")).unwrap();
        let (_i, man) = load_and_normalize(&ball, &[], 0).unwrap();
        let bj = man.joints.iter().find(|j| j.joint_type == "ball").unwrap();
        assert_eq!(bj.qpos_dim, 4);
        assert!(bj.unsupported_reason.as_deref() == Some("ball_joint_multi_dof"));
        crate::mujoco_exec::checkin_worker(_i);
    }

    #[test]
    fn authority_restart_replay_has_zero_write_delta() {
        if !ensure_mujoco_or_skip() {
            return;
        }
        let b = RobotBundle::load(corpus::robot_dir("planar_arm")).unwrap();
        let ep = run_episode(&b, "AUTHORITY_RESTART", 7, "pd").unwrap();
        assert_eq!(
            ep.replay_write_delta,
            Some(0),
            "{:?}",
            ep.replay_write_delta
        );
        assert_eq!(ep.ctrl_writes_after_first_command, Some(1));
        assert_eq!(ep.policy_ctrl_writes, 1);
        assert!(ep.task_success);
    }

    #[test]
    fn command_lease_expires_to_safe_state() {
        if !ensure_mujoco_or_skip() {
            return;
        }
        let b = RobotBundle::load(corpus::robot_dir("planar_arm")).unwrap();
        let ep = run_episode(&b, "JOINT_TRACKING", 2, "pd").unwrap();
        assert!(ep.total_ctrl_writes >= ep.policy_ctrl_writes);
        assert_eq!(
            ep.total_ctrl_writes,
            ep.policy_ctrl_writes + ep.authority_safe_state_writes
        );
    }

    #[test]
    fn worker_hang_is_infra_error() {
        if !ensure_mujoco_or_skip() {
            return;
        }
        let ep = crate::runner::run_hang_matrix_job();
        assert_eq!(ep.episode_status, "INFRA_ERROR");
        assert!(ep.infra_error.as_ref().unwrap().contains("timeout"));
    }

    #[test]
    fn matrix_never_drops_jobs() {
        if !ensure_mujoco_or_skip() {
            return;
        }
        let robots = vec![corpus::robot_dir("cartpole")];
        let eps = run_matrix(
            &robots,
            &["JOINT_TRACKING", "POLICY_CRASH"],
            0..2,
            &["pd"],
            false,
        );
        assert_eq!(eps.len(), 4);
        let report = crate::evidence::aggregate(&eps);
        assert_eq!(report.scheduled_jobs, 4);
        assert_eq!(
            report.scheduled_jobs,
            report.completed_jobs + report.infra_error_jobs
        );
        assert!(eps.iter().all(|e| !e.episode_status.is_empty()));
        assert!(eps
            .iter()
            .all(|e| !e.metal && e.evidence_status == SIMULATION_ONLY));
    }

    #[test]
    fn historical_keep_out_fails_even_if_final_frame_is_clear() {
        let mut env = crate::scenario::EnvelopeSpec::default();
        env.keep_out.push(crate::scenario::Region {
            name: "keep_out".into(),
            center: [0.0, 0.0, 0.0],
            half: [0.1, 0.1, 0.1],
            ..crate::scenario::Region::default()
        });
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
            derived: crate::normalize::DerivedInterface::default(),
            model_hash: "h".into(),
            source_hash: "s".into(),
            mujoco_version: "3".into(),
            source_format: "mjcf".into(),
            lost_features: vec![],
            support_bodies: vec![],
            collision_groups: Default::default(),
            metal: false,
            evidence_status: SIMULATION_ONLY.into(),
        };
        let mut inside = crate::observation::VerifierTruth {
            named_pos: std::collections::BTreeMap::from([("ee".into(), vec![0.0, 0.0, 0.0])]),
            ..crate::observation::VerifierTruth::default()
        };
        let v1 = crate::verifier::inspect_step(&man, &env, &mut inside, &[]);
        assert!(v1.iter().any(|v| v.code == "KEEP_OUT_ZONE_ENTRY"));
        let mut hist = inside.zone_entries.clone();
        let mut outside = crate::observation::VerifierTruth {
            named_pos: std::collections::BTreeMap::from([("ee".into(), vec![2.0, 2.0, 2.0])]),
            ..crate::observation::VerifierTruth::default()
        };
        let v2 = crate::verifier::inspect_step(&man, &env, &mut outside, &[]);
        assert!(!v2.iter().any(|v| v.code == "KEEP_OUT_ZONE_ENTRY"));
        hist.extend(outside.zone_entries);
        outside.zone_entries = hist;
        assert!(!crate::task::TaskSpec::KeepOut {
            name: "keep_out".into()
        }
        .evaluate(&outside, &man));
    }

    #[test]
    fn camera_mode_is_not_a_working_pipeline() {
        let m = crate::normalize::RobotManifest {
            robot_id: "x".into(),
            nq: 1,
            nv: 1,
            nu: 1,
            nbody: 1,
            njoint: 1,
            nactuator: 1,
            nsensor: 0,
            ncamera: 0,
            timestep: 0.002,
            joints: vec![],
            actuators: vec![],
            sensors: vec![],
            cameras: vec![],
            bodies: vec![],
            sites: vec![],
            derived: crate::normalize::DerivedInterface::default(),
            model_hash: "h".into(),
            source_hash: "s".into(),
            mujoco_version: "3".into(),
            source_format: "mjcf".into(),
            lost_features: vec![],
            support_bodies: vec![],
            collision_groups: Default::default(),
            metal: false,
            evidence_status: SIMULATION_ONLY.into(),
        };
        let truth = crate::observation::VerifierTruth::default();
        let obs = crate::observation::policy_observation(
            &m,
            "e",
            "o",
            &crate::task::TaskSpec::Hold { duration_s: 0.1 },
            crate::observation::VisionMode::Camera,
            &truth,
            0.0,
            false,
        );
        assert_eq!(
            obs.camera_status.as_deref(),
            Some("NOT_IMPLEMENTED_IN_VERIFY_V1")
        );
        assert!(obs.rgb.is_none());
    }
}
