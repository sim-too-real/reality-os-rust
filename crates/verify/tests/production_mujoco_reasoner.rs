//! Production PhysicalReasoner driving MuJoCo contact. SIMULATION_ONLY.
//!
//! Hidden friction is applied only on the simulator body. The reasoner sees the
//! uncertain belief, the measured joint configuration, and the perceived object
//! pose. It does not see the simulator friction coefficient.

use realityos_core::RealityOs;
use realityos_kernel::physical_action::{
    AbortHoldMode, PhysicalActionEnvelope, WrenchAbortContract, WrenchLimitPolicy, WrenchLimitScope,
};
use realityos_reasoner_bridge::{authorize_current_action, BridgeRefusal, SimulationAdmission};
use realityos_semantics::contact::SupportKind;
use realityos_semantics::contact_collision::NamedBox;
use realityos_semantics::physical_belief::{PhysicalParameter, PhysicalParameterBelief};
use realityos_semantics::physical_decision::DecisionKind;
use realityos_semantics::physical_reasoner::{
    integrate_support_friction, PhysicalObjective, PhysicalReasoner, ReasoningRequest, TableScene,
    EVIDENCE_STATUS,
};
use realityos_semantics::provenance::Provenanced;
use realityos_semantics::resource::{
    ClosingDirection, ControlledResource, CouplingModel, QualificationStatus, ResourceKind,
    ResourceTopology,
};
use realityos_session::grant_covers;
use realityos_verify::bundle::RobotBundle;
use realityos_verify::corpus;
use realityos_verify::manipulation_verify::verify_grasp;
use realityos_verify::mujoco_exec::{checkin_worker, ensure_mujoco_or_skip, MujocoInstance};
use realityos_verify::observation::VerifierTruth;
use realityos_verify::runner::load_and_normalize;
use realityos_verify::semantics_map::embodiment_from_manifest;
use serde_json::{json, Value};
use std::path::PathBuf;

const TRUE_FRICTION: f64 = 0.85;
const MU_REQUIRED: f64 = 0.3;

fn simulation_admission(expires_at_s: f64) -> SimulationAdmission {
    let mut wrench_limit_policy = WrenchLimitPolicy {
        scope: WrenchLimitScope {
            sensor_id: "ft-tool".into(),
            sensor_frame: "tool0".into(),
            caps: [20.0, 20.0, 30.0, 2.0, 2.0, 2.0],
            policy_digest: String::new(),
            abort_contract_id: "abort-and-reobserve".into(),
        },
        sensor_calibration_hash: "test-sensor-calibration-v1".into(),
        sensor_sample_rate_hz: 125.0,
        max_position_tracking_correction_rad: 0.0,
        position_tracking_correction_gain: 0.0,
        max_rise_rate: [10.0; 6],
        max_sensing_latency_s: 0.01,
        max_abort_latency_s: 0.03,
        max_sample_age_s: 0.02,
        max_inter_sample_gap_s: 0.01,
        max_cross_sensor_skew_s: 0.005,
        monitor_id: "test-wrench-monitor-v1".into(),
        compensation_digest: "c".repeat(64),
        abort_contract: WrenchAbortContract {
            contract_id: "abort-and-reobserve".into(),
            hold_mode: AbortHoldMode::HoldCurrentJointPosition,
            allowed_stop_actuator_ids: vec!["tool".into()],
            max_time_to_stop_s: 0.03,
            max_stop_writes: 1,
        },
    };
    wrench_limit_policy.scope.policy_digest = wrench_limit_policy.digest();
    SimulationAdmission {
        execution_bound_m: 0.1,
        expires_at_s,
        wrench_limit_policy,
    }
}

#[test]
fn menagerie_ur5e_declared_body_transforms_match_mujoco_geometry() {
    if !ensure_mujoco_or_skip() {
        return;
    }
    let bundle_path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../robots/external/ur5e");
    let bundle = RobotBundle::load(&bundle_path).expect("UR5e bundle");
    let (mut worker, manifest) = load_and_normalize(&bundle, &[], 17).expect("load UR5e");
    let model = embodiment_from_manifest(&bundle, &manifest);
    let chain = model.ee_joint_chain("tool0").expect("tool chain");
    let q = [-6.0626, 4.6069, -1.4816, 1.2369, -2.5153, -2.213];
    let mut qpos = manifest.qpos0.clone();
    for (name, value) in chain.iter().zip(q) {
        let joint = manifest
            .joints
            .iter()
            .find(|joint| joint.name == *name)
            .expect("inspected chain joint");
        qpos[joint.qpos_address as usize] = value;
    }
    worker
        .reset(Some(&qpos), None)
        .expect("set diagnostic pose");
    let snapshot = worker.step(0).expect("MuJoCo state");
    let state = snapshot.get("state").unwrap_or(&snapshot);
    let joint_state = chain.iter().cloned().zip(q).collect::<Vec<_>>();
    let bodies = realityos_semantics::geometry_fk::body_world_transforms(&model, &joint_state)
        .expect("semantic body transforms");

    for body_name in [
        "shoulder_link",
        "upper_arm_link",
        "forearm_link",
        "wrist_1_link",
        "wrist_2_link",
        "wrist_3_link",
    ] {
        let expected = state["xpos"][body_name]
            .as_array()
            .map(|xyz| {
                [
                    xyz[0].as_f64().unwrap(),
                    xyz[1].as_f64().unwrap(),
                    xyz[2].as_f64().unwrap(),
                ]
            })
            .expect("MuJoCo body position");
        let actual = bodies.get(body_name).expect("semantic body position").xyz;
        let error = ((expected[0] - actual[0]).powi(2)
            + (expected[1] - actual[1]).powi(2)
            + (expected[2] - actual[2]).powi(2))
        .sqrt();
        assert!(
            error < 1e-5,
            "{body_name} FK error {error}: expected {expected:?}, actual {actual:?}"
        );
    }
    let actual_site = state["sites"]["attachment_site"]
        .as_array()
        .map(|xyz| {
            [
                xyz[0].as_f64().unwrap(),
                xyz[1].as_f64().unwrap(),
                xyz[2].as_f64().unwrap(),
            ]
        })
        .expect("MuJoCo tool site position");
    let semantic_site =
        realityos_semantics::kinematics::forward_kinematics(&model, &chain, "tool0", &q)
            .expect("semantic tool site")
            .ee
            .xyz;
    let site_error = ((actual_site[0] - semantic_site[0]).powi(2)
        + (actual_site[1] - semantic_site[1]).powi(2)
        + (actual_site[2] - semantic_site[2]).powi(2))
    .sqrt();
    assert!(
        site_error < 1e-5,
        "tool site FK error {site_error}: MuJoCo {actual_site:?}, semantic {semantic_site:?}"
    );
    checkin_worker(worker);
}

#[test]
fn ur5e_finds_a_collision_checked_push_from_a_side_precontact_pose() {
    if !ensure_mujoco_or_skip() {
        return;
    }
    let bundle_path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../robots/external/ur5e");
    let bundle = RobotBundle::load(&bundle_path).expect("UR5e bundle");
    let (worker, manifest) = load_and_normalize(&bundle, &[], 17).expect("load UR5e");
    checkin_worker(worker);
    let model = embodiment_from_manifest(&bundle, &manifest);
    let chain = model.ee_joint_chain("tool0").expect("tool chain");
    let mut reasoner = PhysicalReasoner::new(model.clone());
    let request = ReasoningRequest {
        scene: TableScene {
            object_id: "obj0".into(),
            object_center: [
                0.4049284358754865,
                0.018913550624106795,
                0.45849492823038374,
            ],
            object_half: [0.03849492823038377; 3],
            object_quat: [1.0, 0.0, 0.0, 0.0],
            support_origin: [0.45, 0.0, 0.42],
            support_normal: [0.0, 0.0, 1.0],
            support_half_extents: Some([0.25, 0.25, 0.02]),
            obstacles: Vec::new(),
            arm_q: vec![2.56, 4.1, 2.04, 4.83, -1.04, -1.88],
            joint_names: chain,
            ee: "tool0".into(),
            tool_radius_m: 0.04,
            tool_contact_extent_m: 0.04,
            observation_epoch: "ur5e-safe-standoff".into(),
            held: false,
            finger_names: Vec::new(),
            finger_q: Vec::new(),
        },
        belief: PhysicalParameterBelief::declared_point(
            PhysicalParameter::SupportFriction,
            0.8,
            "test_prior",
        ),
        objective: PhysicalObjective::PlanarTranslation {
            target_xy: [0.48, 0.0286],
        },
        mu_required: MU_REQUIRED,
        goal_stroke_m: 0.025,
        probe_stroke_m: 0.008,
        grants: Vec::new(),
        now_s: 10.0,
        evidence_fresh: true,
        safety_faults: Vec::new(),
        mass_kg: Some(0.238493207014167),
        gripper_force_n: None,
        gripper_resource: None,
        linear_execution_bound_m: None,
        linear_wrench_limit: None,
        gravity_m_s2: [0.0, 0.0, -9.81],
    };

    let report = reasoner.consider(&request);
    let push = report
        .candidates
        .iter()
        .find(|candidate| candidate.action_key == "push:+x")
        .expect("positive-x push candidate");
    assert_eq!(
        push.status, "PROVED",
        "{push:?}; decision={:?}",
        report.decision
    );
}

#[test]
fn production_reasoner_push_probe_replans_and_refuses_after_overstroke() {
    if !ensure_mujoco_or_skip() {
        return;
    }
    let mut world = World::load("planar_arm");
    let model = world.model.clone();
    // The planar arm's interaction plane is z=0.12. Put the table top at
    // z=0.08 and the block center at z=0.10 so the block is supported while
    // the tool sphere clears the support at the planned contact height.
    world
        .inst()
        .rpc(&json!({"cmd":"set_body_pos","body":"table","pos":[0.4,0.0,0.07]}))
        .expect("place table at the planar-arm support height");
    world.place_for_contact(&model, [0.30, 0.0, 0.10]);
    let chain = model.ee_joint_chain("ee").expect("planar ee chain");
    let seed = world.arm_q();
    let (standoff_q, trace) = realityos_semantics::kinematics::with_ik_q_seed(Some(&seed), || {
        realityos_semantics::kinematics::solve_ik(&model, &chain, "ee", [0.215, 0.0, 0.12], &seed)
    })
    .expect("solve reachable probe standoff");
    assert!(realityos_semantics::kinematics::ik_residual_is_precise(
        trace.residual
    ));
    world.set_arm(&standoff_q);
    let mut request = world.policy_request(&model, uncertain_friction());
    request.scene.support_origin = [0.30, 0.0, 0.08];
    request.linear_execution_bound_m = Some(0.1);
    request.linear_wrench_limit =
        Some(simulation_admission(40.0).wrench_limit_policy.scope.clone());
    assert!(
        !format!("{request:?}").contains(&format!("{TRUE_FRICTION}")),
        "the policy request must not carry the hidden friction"
    );
    let mut reasoner = PhysicalReasoner::new(model);
    let mut os = RealityOs::new();
    let first = reasoner.consider(&request);
    assert_eq!(first.evidence_status, EVIDENCE_STATUS);
    assert!(
        !matches!(first.decision, DecisionKind::GoalInteraction { .. }),
        "uncertain friction must not authorize an immediate goal push: {:?}",
        first.decision
    );
    let probe_proposal = first
        .authority_request
        .as_ref()
        .unwrap_or_else(|| panic!("probe not proposed: {}", candidate_lines(&first.candidates)));
    assert_eq!(probe_proposal.action_key, "probe:+x");
    let probe_grant = authorize_planar_current(&mut os, &mut reasoner, &request);
    assert_eq!(probe_grant.frozen().action_key, probe_proposal.action_key);
    assert_eq!(
        probe_grant.frozen().witness_digest,
        probe_proposal.witness_digest
    );
    let before_writes = world.ctrl_writes();
    let probed = world.execute(&probe_grant);
    assert!(world.ctrl_writes() > before_writes);
    let probe_retract = q_line(&probe_grant.frozen().witness_contents, "q_retract_bits=");
    assert!(
        q_tracks(&world.arm_q(), &probe_retract, 0.03),
        "MuJoCo probe executor must reach the frozen collision-checked retreat before re-observation"
    );
    let ratio = if request.probe_stroke_m > 1e-9 {
        probed.displacement / request.probe_stroke_m
    } else {
        0.0
    };
    let receipt = integrate_support_friction(
        &request.belief,
        ratio,
        request.mu_required,
        "mujoco-probe-displacement",
    );
    request.belief = receipt.belief_after;
    request.scene.object_center = world.object_xyz();
    request.scene.arm_q = world.arm_q();
    request.scene.observation_epoch = "epoch:after-probe".into();
    request.grants.clear();
    let planned = reasoner.consider(&request);
    assert!(ratio > 1.75, "expected measured overstroke, got {ratio}");
    assert!(receipt.information_gain);
    assert!(
        planned.plan.is_none(),
        "overstroke must not justify a follow-up plan: {planned:?}"
    );
    assert!(planned.executable.is_none());
    assert!(planned.authority_request.is_none());
    let writes_after_probe = world.ctrl_writes();
    assert!(writes_after_probe > before_writes);
    let evidence = json!({
        "evidence_status": EVIDENCE_STATUS,
        "hidden_friction_not_in_policy": true,
        "probe_displacement_m": probed.displacement,
        "probe_ratio": ratio,
        "belief_gain": receipt.information_gain,
        "belief_interval": request.belief.entry(PhysicalParameter::SupportFriction).and_then(|e| e.empirical_interval),
        "post_probe_decision": format!("{:?}", planned.decision),
        "follow_up_action_authorized": false,
        "ctrl_writes": writes_after_probe,
        "unauthorized_writes": 0,
        "cache": {
            "hits": reasoner.cache_stats().executable_hits,
            "misses": reasoner.cache_stats().executable_misses,
            "ik_avoided": reasoner.cache_stats().ik_avoided,
        }
    });
    let path = evidence_path("production_mujoco_push.json");
    std::fs::write(&path, serde_json::to_string_pretty(&evidence).unwrap()).unwrap();
    assert!(
        world.ctrl_writes() == writes_after_probe,
        "the fresh refusal must not add a control write\n{evidence}"
    );
    world.finish();
}

#[test]
fn production_reasoner_grasp_refuses_without_qualified_resource() {
    if !ensure_mujoco_or_skip() {
        return;
    }
    let mut world = World::load("arm_gripper");
    let model = world.model.clone();
    let ee = world.site("ee");
    world
        .inst()
        .configure_body(
            "obj0",
            None,
            Some(0.05),
            Some(TRUE_FRICTION),
            Some(vec![0.008, 0.008, 0.008]),
            false,
        )
        .expect("shrink object");
    world
        .inst()
        .rpc(&json!({"cmd":"set_body_pos","body":"obj0","pos": ee}))
        .expect("place grasp object");
    let mut request = world.policy_request(&model, finger_friction());
    request.scene.object_half = [0.008, 0.008, 0.008];
    request.scene.object_center = world.object_xyz();
    request.objective = PhysicalObjective::AcquireObject {
        object_id: "obj0".into(),
    };
    request.scene.finger_names = vec!["finger_l".into(), "finger_r".into()];
    request.scene.finger_q = world.finger_q();
    request.gripper_force_n = Some(2.0);
    request.mass_kg = Some(0.05);
    request.belief.narrow_interval(
        PhysicalParameter::SupportFriction,
        [0.5, 0.5],
        "known-support-friction",
    );
    let mut reasoner = PhysicalReasoner::new(model);
    let mut os = RealityOs::new();
    let report = reasoner.consider(&request);
    let grasp = report
        .candidates
        .iter()
        .find(|candidate| candidate.action_key == "grasp:y")
        .expect("the reasoner reports the grasp refusal");
    assert_eq!(grasp.status, "GRIPPER_RESOURCE_UNKNOWN");
    assert!(report.executable.is_none());
    assert!(report.authority_request.is_none());
    let before_writes = world.ctrl_writes();
    assert_eq!(
        authorize_current_action(
            &mut os,
            &mut reasoner,
            &request,
            &simulation_admission(40.0),
            &[],
        ),
        Err(BridgeRefusal::NoJustifiedAction)
    );
    assert_eq!(world.ctrl_writes(), before_writes);
    world.finish();
}

#[test]
fn production_reasoner_authorized_grasp_is_verified_in_mujoco() {
    if !ensure_mujoco_or_skip() {
        return;
    }
    let mut world = World::load("arm_gripper_lift");
    let (model, request, resource) = prepare_grasp(&mut world);
    let mut reasoner = PhysicalReasoner::new(model);
    let mut os = RealityOs::new();
    let authorized = authorize_current_action(
        &mut os,
        &mut reasoner,
        &request,
        &simulation_admission(40.0),
        &[],
    )
    .unwrap_or_else(|error| {
        panic!(
            "exactly proved grasp receives simulation-only authority: {error}; candidates={} finger_q={:?}",
            candidate_lines(&reasoner.consider(&request).candidates),
            request.scene.finger_q
        )
    });
    let frozen = authorized.action.frozen();
    assert!(frozen.authority_granted);
    assert_eq!(authorized.grant.evidence_status, EVIDENCE_STATUS);
    assert!(!authorized.grant.metal);
    let PhysicalActionEnvelope::PinchGrasp {
        per_finger_force_ceiling_n,
        ..
    } = &frozen.action_envelope
    else {
        panic!("authorized grasp must carry typed pinch authority");
    };
    assert_eq!(*per_finger_force_ceiling_n, 1.95);
    assert!(
        resource
            .force_bound
            .known_value()
            .is_some_and(|bound| bound[1] <= 1.95),
        "resource force bound exceeds the authorized limit: {:?}",
        resource.force_bound
    );

    let before_writes = world.ctrl_writes();
    let execution = world.execute_grasp(&authorized, &request, 10.0);
    assert!(
        !execution.aborted,
        "grasp phases aborted: {:?}",
        execution.phases
    );
    assert!(!execution.runtime_divergence);
    assert_eq!(execution.phases, ["approach", "contact", "close", "lift"]);
    assert_eq!(execution.unauthorized_writes, 0);
    let after = world.current_truth();
    let opening = world.measured_opening(&resource);
    let verdict = verify_grasp(
        execution
            .hold_start
            .as_ref()
            .expect("measured state after close"),
        &after,
        &resource,
        "obj0",
        &["table", "floor"],
        &["wall".into()],
        opening,
        after.time_s,
        &["ee"],
        Some(&frozen.action_envelope),
    );
    assert!(verdict.grasp_acquired, "{:?}", verdict.evidence_used);
    assert!(
        verdict.verified_hold,
        "failure={:?} evidence={:?} notes={:?} phases={:?} hold_q={:?} hold_obj={:?} hold_ee={:?} object_after={:?} ee_after={:?} envelope={:?} left_force={:?} right_force={:?}",
        verdict.failure,
        verdict.evidence_used,
        verdict.notes,
        execution.phases,
        execution.hold_start.as_ref().map(|state| &state.qpos),
        execution.hold_start.as_ref().and_then(|state| state.xpos.get("obj0")),
        execution.hold_start.as_ref().and_then(|state| state.named_pos.get("ee")),
        after.xpos.get("obj0"),
        after.named_pos.get("ee"),
        frozen.action_envelope,
        normal_force_sum(&after, "obj0", "finger_l"),
        normal_force_sum(&after, "obj0", "finger_r"),
    );
    assert!(verdict.success);
    assert_eq!(verdict.support[0].kind, SupportKind::Unsupported);
    assert!(world.ctrl_writes() > before_writes);
    assert_eq!(execution.unauthorized_writes, 0);

    let hold_start = execution
        .hold_start
        .as_ref()
        .expect("verified hold has a measured close state");
    let object_before = hold_start.xpos["obj0"].as_slice();
    let object_after = after.xpos["obj0"].as_slice();
    let ee_before = hold_start.named_pos["ee"].as_slice();
    let ee_after = after.named_pos["ee"].as_slice();
    let object_lift_m = object_after[2] - object_before[2];
    let ee_lift_m = ee_after[2] - ee_before[2];
    let follow_residual_m = vector_delta_distance(
        [
            object_after[0] - object_before[0],
            object_after[1] - object_before[1],
            object_lift_m,
        ],
        [
            ee_after[0] - ee_before[0],
            ee_after[1] - ee_before[1],
            ee_lift_m,
        ],
    );
    let left_force_n = normal_force_sum(&after, "obj0", "finger_l");
    let right_force_n = normal_force_sum(&after, "obj0", "finger_r");
    let force_ceiling = match &frozen.action_envelope {
        PhysicalActionEnvelope::PinchGrasp {
            per_finger_force_ceiling_n,
            ..
        } => *per_finger_force_ceiling_n,
        _ => unreachable!("verified grasp has a typed pinch envelope"),
    };
    assert!(left_force_n.is_some_and(|force| force <= force_ceiling + 1e-9));
    assert!(right_force_n.is_some_and(|force| force <= force_ceiling + 1e-9));
    let evidence = json!({
        "evidence_status": EVIDENCE_STATUS,
        "authorized_action": frozen.action_key,
        "action_envelope": frozen.action_envelope,
        "phases": execution.phases,
        "grasp_acquired": verdict.grasp_acquired,
        "verified_hold": verdict.verified_hold,
        "support": format!("{:?}", verdict.support[0].kind),
        "measured_object_lift_m": object_lift_m,
        "measured_end_effector_lift_m": ee_lift_m,
        "object_follow_residual_m": follow_residual_m,
        "measured_per_finger_normal_force_n": {
            "finger_l": left_force_n,
            "finger_r": right_force_n,
        },
        "measured_opening_fraction": opening,
        "evidence": verdict.evidence_used,
        "ctrl_writes": world.ctrl_writes() - before_writes,
        "unauthorized_writes": execution.unauthorized_writes,
    });
    let path = evidence_path("production_mujoco_grasp.json");
    std::fs::write(&path, serde_json::to_string_pretty(&evidence).unwrap()).unwrap();
    world.finish();
}

#[test]
fn production_reasoner_grasp_expiry_stops_before_unverified_lift() {
    if !ensure_mujoco_or_skip() {
        return;
    }
    let mut world = World::load("arm_gripper_lift");
    let (model, request, resource) = prepare_grasp(&mut world);
    let mut reasoner = PhysicalReasoner::new(model);
    let mut os = RealityOs::new();
    let authorized = authorize_current_action(
        &mut os,
        &mut reasoner,
        &request,
        &simulation_admission(10.95),
        &[],
    )
    .expect("grant is valid at action start");
    let before = world.current_truth();
    let before_writes = world.ctrl_writes();
    let execution = world.execute_grasp(&authorized, &request, 10.0);
    assert!(execution.aborted);
    assert!(!execution.runtime_divergence);
    assert_eq!(execution.phases, ["approach", "contact", "close"]);
    assert_eq!(execution.unauthorized_writes, 0);
    let after = world.current_truth();
    let opening = world.measured_opening(&resource);
    let verdict = verify_grasp(
        execution.hold_start.as_ref().unwrap_or(&before),
        &after,
        &resource,
        "obj0",
        &["table", "floor"],
        &["wall".into()],
        opening,
        after.time_s,
        &["ee"],
        Some(&authorized.action.frozen().action_envelope),
    );
    assert!(!verdict.verified_hold);
    assert!(!verdict.success);
    assert!(world.ctrl_writes() > before_writes);
    assert_eq!(execution.unauthorized_writes, 0);
    world.finish();
}

#[test]
fn production_reasoner_grasp_runtime_divergence_aborts_before_control_write() {
    if !ensure_mujoco_or_skip() {
        return;
    }
    let mut world = World::load("arm_gripper_lift");
    let (model, request, _) = prepare_grasp(&mut world);
    let mut reasoner = PhysicalReasoner::new(model);
    let mut os = RealityOs::new();
    let authorized = authorize_current_action(
        &mut os,
        &mut reasoner,
        &request,
        &simulation_admission(40.0),
        &[],
    )
    .expect("grasp starts with a current exact proof");
    let mut divergent = q_line(&authorized.action.frozen().witness_contents, "q0_bits=");
    divergent[0] += 0.1;
    world.set_arm(&divergent);
    let before_writes = world.ctrl_writes();

    let execution = world.execute_grasp(&authorized, &request, 10.0);

    assert!(execution.aborted);
    assert!(execution.runtime_divergence);
    assert!(execution.phases.is_empty());
    assert_eq!(execution.unauthorized_writes, 0);
    assert_eq!(world.ctrl_writes(), before_writes);
    world.finish();
}

fn prepare_grasp(
    world: &mut World,
) -> (
    realityos_semantics::embodiment::EmbodimentModel,
    ReasoningRequest,
    ControlledResource,
) {
    let mut model = world.model.clone();
    let mut current_arm = world.arm_q();
    let lift_index = world
        .arm_names
        .iter()
        .position(|name| name == "lift_z")
        .expect("fixture declares its vertical lift joint");
    // The default qpos leaves the open fingers 8 micrometers into the table.
    // Start 2 mm higher so full-geometry admission evaluates a clear approach.
    current_arm[lift_index] = 0.002;
    world.set_arm(&current_arm);
    world.command_q(&current_arm, &[0.029, 0.029], 200);
    let resource = ControlledResource {
        id: "gripper:direct-two-finger".into(),
        kind: ResourceKind::Gripper,
        topology: ResourceTopology::DirectJointGripper,
        actuator_inputs: vec!["grip_l".into(), "grip_r".into()],
        affected_joints: vec!["finger_l".into(), "finger_r".into()],
        finger_bodies: vec!["finger_l".into(), "finger_r".into()],
        coupling: CouplingModel::none(),
        command_coordinate: "joint_position".into(),
        opening_range: Provenanced::declared([0.0, 0.03], "mujoco_fixture", 10.0),
        command_range: Provenanced::declared([0.0, 0.03], "mujoco_fixture", 10.0),
        closing_direction: ClosingDirection::TowardMin,
        force_bound: Provenanced::declared([0.0, 1.95], "mujoco_fixture", 10.0),
        qualification: QualificationStatus::Qualified,
        unsupported_detail: None,
    };
    model.resources = vec![resource.clone()];
    world.model = model.clone();
    let ee = world.site("ee");
    world
        .inst()
        .configure_body(
            "obj0",
            None,
            Some(0.04),
            Some(TRUE_FRICTION),
            Some(vec![0.01, 0.01, 0.025]),
            false,
        )
        .expect("configure grasp object");
    // A 50 mm tall supported box leaves the complete 40 mm finger collision
    // geometry above the table throughout the centered pinch approach.
    let object_pos = [ee[0], ee[1], 0.125];
    world
        .inst()
        .rpc(&json!({"cmd":"set_body_pos","body":"obj0","pos":object_pos}))
        .expect("place object on the table between fingers");
    let mut request = world.policy_request(&model, finger_friction());
    request.scene.object_half = [0.01, 0.01, 0.025];
    request.scene.object_center = world.object_xyz();
    request.scene.support_origin = [
        request.scene.object_center[0],
        request.scene.object_center[1],
        request.scene.object_center[2] - request.scene.object_half[2],
    ];
    request.objective = PhysicalObjective::AcquireObject {
        object_id: "obj0".into(),
    };
    request.scene.finger_names = resource.affected_joints.clone();
    request.scene.finger_q = world.finger_q();
    request.gripper_force_n = Some(1.95);
    request.mass_kg = Some(0.04);
    request.gripper_resource = Some(resource.clone());
    request.belief.narrow_interval(
        PhysicalParameter::SupportFriction,
        [0.5, 0.5],
        "known-support-friction",
    );
    (model, request, resource)
}

struct World {
    inst: Option<MujocoInstance>,
    model: realityos_semantics::embodiment::EmbodimentModel,
    inspect: Value,
    arm_names: Vec<String>,
    object_z: f64,
}

struct Executed {
    displacement: f64,
}

#[derive(Debug, Default)]
struct GraspExecution {
    phases: Vec<&'static str>,
    aborted: bool,
    runtime_divergence: bool,
    unauthorized_writes: u64,
    hold_start: Option<VerifierTruth>,
}

impl World {
    fn load(robot: &str) -> Self {
        let bundle = RobotBundle::load(corpus::robot_dir(robot)).expect(robot);
        let objects = vec![
            json!({"name":"table","type":"box","pos":[0.25,0.0,0.09],"size":[0.30,0.22,0.01],"mass":8.0,"movable":false}),
            json!({"name":"obj0","type":"box","pos":[1.4,0.8,0.12],"size":[0.02,0.02,0.02],"mass":0.05,"friction":0.2,"movable":true}),
            json!({"name":"wall","type":"box","pos":[1.4,0.7,0.12],"size":[0.03,0.008,0.02],"mass":2.0,"movable":false}),
        ];
        let (mut inst, manifest) = load_and_normalize(&bundle, &objects, 7).expect("load");
        inst.configure_body("obj0", None, Some(0.05), Some(TRUE_FRICTION), None, false)
            .expect("hidden friction");
        let inspect = inst.inspect.clone();
        let model = embodiment_from_manifest(&bundle, &manifest);
        let arm_names = model.ee_joint_chain("ee").expect("ee chain");
        let mut world = Self {
            inst: Some(inst),
            model,
            inspect,
            arm_names,
            object_z: 0.13,
        };
        world.set_arm(&[0.6, -1.0, 0.6]);
        let settled = world.arm_q();
        world.command_q(&settled, &[0.02, 0.02], 20);
        let ee = world.site("ee");
        world.object_z = ee[2];
        world
            .inst()
            .rpc(&json!({
                "cmd": "set_body_pos",
                "body": "obj0",
                "pos": [ee[0] + 0.015, ee[1], ee[2]]
            }))
            .expect("place object");
        let object = world.object_xyz();
        world
            .inst()
            .rpc(&json!({
                "cmd": "set_body_pos",
                "body": "wall",
                "pos": [object[0], object[1] - 0.034, object[2]]
            }))
            .ok();
        world
    }

    fn inst(&mut self) -> &mut MujocoInstance {
        self.inst.as_mut().expect("worker")
    }

    fn finish(&mut self) {
        if let Some(inst) = self.inst.take() {
            checkin_worker(inst);
        }
    }

    fn current_truth(&mut self) -> VerifierTruth {
        let snapshot = self.inst().step(0).expect("privileged truth snapshot");
        let state = snapshot.get("state").unwrap_or(&snapshot);
        VerifierTruth::from_mujoco_state(state)
    }

    fn measured_opening(&mut self, resource: &ControlledResource) -> Option<f64> {
        let [minimum, maximum] = resource.opening_range.value?;
        let span = maximum - minimum;
        if !span.is_finite() || span <= 0.0 {
            return None;
        }
        let positions = self.finger_q();
        (positions.len() == resource.affected_joints.len()).then(|| {
            positions
                .iter()
                .map(|position| ((position - minimum) / span).clamp(0.0, 1.0))
                .sum::<f64>()
                / positions.len() as f64
        })
    }

    fn execute_grasp(
        &mut self,
        authorized: &realityos_reasoner_bridge::AuthorizedReasonerAction,
        request: &ReasoningRequest,
        start_time_s: f64,
    ) -> GraspExecution {
        let frozen = authorized.action.frozen();
        let Some(PhysicalActionEnvelope::PinchGrasp {
            resource_id,
            finger_joint_names,
            open_positions,
            close_positions,
            lift_trajectory_digest,
            ..
        }) = Some(&frozen.action_envelope)
        else {
            return GraspExecution {
                aborted: true,
                ..GraspExecution::default()
            };
        };
        let Some(resource) = request.gripper_resource.as_ref() else {
            return GraspExecution {
                aborted: true,
                ..GraspExecution::default()
            };
        };
        let contents = &frozen.witness_contents;
        let q0 = q_line(contents, "q0_bits=");
        let transit = q_line(contents, "q_transit_bits=");
        let contact = q_line(contents, "q_contact_bits=");
        let frozen_close = q_line(contents, "finger_bits=");
        let lift_digest = contents
            .lines()
            .find_map(|line| line.strip_prefix("lift_trajectory_digest="))
            .unwrap_or_default();
        let lift_states = contents
            .lines()
            .filter(|line| line.starts_with("lift_state_"))
            .filter_map(parse_lift_state)
            .collect::<Vec<_>>();
        let Some(PhysicalActionEnvelope::PinchGrasp {
            per_finger_force_ceiling_n,
            maximum_lift_m,
            ..
        }) = Some(&frozen.action_envelope)
        else {
            unreachable!()
        };
        let envelope_matches = resource_id == &resource.id
            && finger_joint_names == &request.scene.finger_names
            && open_positions.len() == 2
            && close_positions.len() == 2
            && open_positions.len() == request.scene.finger_q.len()
            && exact_values(open_positions, &request.scene.finger_q)
            && exact_values(close_positions, &frozen_close)
            && resource.force_bound.known_value().is_some_and(|bound| {
                bound[1].is_finite() && bound[1] >= *per_finger_force_ceiling_n
            })
            && !lift_trajectory_digest.is_empty()
            && lift_digest == lift_trajectory_digest
            && q0.len() == request.scene.joint_names.len()
            && transit.len() == q0.len()
            && contact.len() == q0.len()
            && lift_states.len() >= 2
            && lift_states[0].0.len() == q0.len()
            && lift_states
                .iter()
                .all(|state| state.0.len() == q0.len() && state.1.len() == 2)
            && exact_values(&lift_states[0].0, &contact)
            && exact_values(&lift_states[0].1, close_positions)
            && exact_values(&lift_states[1].1, close_positions)
            && *maximum_lift_m >= 0.02;
        if !envelope_matches || !grasp_action_covers(authorized, start_time_s) {
            return GraspExecution {
                aborted: true,
                ..GraspExecution::default()
            };
        }
        let measured_start = self.arm_q();
        if !q_tracks(&measured_start, &q0, 0.03) {
            return GraspExecution {
                aborted: true,
                runtime_divergence: true,
                ..GraspExecution::default()
            };
        }

        let phases = [
            (
                "approach",
                transit.as_slice(),
                open_positions.as_slice(),
                160_u32,
            ),
            (
                "contact",
                contact.as_slice(),
                open_positions.as_slice(),
                160_u32,
            ),
            (
                "close",
                contact.as_slice(),
                close_positions.as_slice(),
                160_u32,
            ),
        ];
        let mut result = GraspExecution::default();
        let mut now_s = start_time_s;
        for (phase, arm_target, fingers, steps) in phases {
            if !grasp_action_covers(authorized, now_s) {
                result.aborted = true;
                return result;
            }
            self.command_q(arm_target, fingers, steps);
            result.phases.push(phase);
            now_s += f64::from(steps) * 0.002;
            let measured = self.arm_q();
            if !q_tracks(&measured, arm_target, 0.025) {
                result.aborted = true;
                result.runtime_divergence = true;
                return result;
            }
            if phase == "close" {
                result.hold_start = Some(self.current_truth());
            }
        }

        let lift_steps = 5_u32;
        let steps_per_lift = 160_u32;
        for step in 1..=lift_steps {
            if !grasp_action_covers(authorized, now_s) {
                result.aborted = true;
                return result;
            }
            if step == 1 {
                result.phases.push("lift");
            }
            let fraction = f64::from(step) / f64::from(lift_steps);
            let target = lift_states[0]
                .0
                .iter()
                .zip(&lift_states[1].0)
                .map(|(start, end)| start + fraction * (end - start))
                .collect::<Vec<_>>();
            self.command_q(&target, close_positions, steps_per_lift);
            now_s += f64::from(steps_per_lift) * 0.002;
            let measured = self.arm_q();
            if !q_tracks(&measured, &target, 0.025) {
                result.aborted = true;
                result.runtime_divergence = true;
                return result;
            }
        }
        result
    }

    fn policy_request(
        &mut self,
        model: &realityos_semantics::embodiment::EmbodimentModel,
        belief: PhysicalParameterBelief,
    ) -> ReasoningRequest {
        let object = self.object_xyz();
        let wall = NamedBox::aabb(
            "wall",
            [object[0], object[1] - 0.034, object[2]],
            [0.01, 0.008, 0.02],
        );
        ReasoningRequest {
            scene: TableScene {
                object_id: "obj0".into(),
                object_center: object,
                object_half: [0.02, 0.02, 0.02],
                object_quat: [1.0, 0.0, 0.0, 0.0],
                support_origin: [object[0], object[1], object[2] - 0.03],
                support_normal: [0.0, 0.0, 1.0],
                support_half_extents: None,
                obstacles: vec![wall],
                arm_q: self.arm_q(),
                joint_names: self.arm_names.clone(),
                ee: "ee".into(),
                tool_radius_m: declared_ee_cross_section_radius(model, "ee")
                    .expect("declared end-effector collision geometry"),
                tool_contact_extent_m: declared_ee_cross_section_radius(model, "ee")
                    .expect("declared end-effector collision geometry"),
                observation_epoch: "epoch:mujoco-0".into(),
                held: false,
                finger_names: vec!["finger_l".into(), "finger_r".into()],
                finger_q: self.finger_q(),
            },
            belief,
            objective: PhysicalObjective::PlanarTranslation {
                target_xy: [object[0] + 0.08, object[1]],
            },
            mu_required: MU_REQUIRED,
            goal_stroke_m: 0.025,
            probe_stroke_m: 0.008,
            grants: Vec::new(),
            now_s: 10.0,
            evidence_fresh: true,
            safety_faults: Vec::new(),
            mass_kg: Some(0.05),
            gripper_force_n: Some(2.0),
            gripper_resource: None,
            linear_execution_bound_m: None,
            linear_wrench_limit: None,
            gravity_m_s2: [0.0, 0.0, -9.81],
        }
    }

    fn execute(
        &mut self,
        action: &realityos_semantics::physical_reasoner::AuthorizedAction,
    ) -> Executed {
        let before = self.object_xyz();
        let contents = &action.frozen().witness_contents;
        let segments = [
            q_line(contents, "q0_bits="),
            q_line(contents, "q_transit_bits="),
            q_line(contents, "q_contact_bits="),
            q_line(contents, "q_end_bits="),
        ];
        let retract = q_line(contents, "q_retract_bits=");
        let fingers = q_line(contents, "finger_bits=");
        for q in segments.iter().take(3) {
            if q.len() == self.arm_names.len() {
                self.command_q(q, &fingers, 60);
            }
        }
        self.track_stroke(contents, &fingers);
        if retract.len() == self.arm_names.len() {
            self.command_q(&retract, &fingers, 60);
        }
        if fingers.len() == 2 {
            let arm = self.arm_q();
            self.command_q(&arm, &fingers, 80);
        }
        let after = self.object_xyz();
        Executed {
            displacement: ((after[0] - before[0]).powi(2)
                + (after[1] - before[1]).powi(2)
                + (after[2] - before[2]).powi(2))
            .sqrt(),
        }
    }

    fn place_for_contact(
        &mut self,
        model: &realityos_semantics::embodiment::EmbodimentModel,
        object: [f64; 3],
    ) {
        let Some(chain) = model.ee_joint_chain("ee") else {
            return;
        };
        let contact = [object[0] - 0.028, object[1], object[2]];
        let standoff = [contact[0] - 0.02, contact[1], contact[2]];
        let seed = self.arm_q();
        if let Ok((q, _)) = realityos_semantics::kinematics::with_ik_q_seed(Some(&seed), || {
            realityos_semantics::kinematics::solve_ik(model, &chain, "ee", standoff, &seed)
        }) {
            self.set_arm(&q);
        }
        self.inst()
            .rpc(&json!({"cmd":"set_body_pos","body":"obj0","pos": object}))
            .expect("place object");
    }

    fn track_stroke(&mut self, contents: &str, fingers: &[f64]) {
        let direction = q_line(contents, "push_bits=");
        let stroke = contents
            .lines()
            .find_map(|line| line.strip_prefix("stroke_bits="))
            .and_then(|text| text.parse::<u64>().ok())
            .map(f64::from_bits)
            .unwrap_or(0.0);
        let contact_q = q_line(contents, "q_contact_bits=");
        if direction.len() != 3 || stroke <= 1e-6 || contact_q.len() != self.arm_names.len() {
            if contact_q.len() == self.arm_names.len() {
                self.command_q(&contact_q, fingers, 40);
            }
            return;
        }
        let Some(chain) = self.model.ee_joint_chain("ee") else {
            return;
        };
        let Ok(start) = realityos_semantics::kinematics::forward_kinematics(
            &self.model,
            &chain,
            "ee",
            &contact_q,
        ) else {
            return;
        };
        let mut seed = contact_q;
        for step in 1..=8 {
            let fraction = f64::from(step) / 8.0;
            let target = [
                start.ee.xyz[0] + direction[0] * stroke * fraction,
                start.ee.xyz[1] + direction[1] * stroke * fraction,
                start.ee.xyz[2] + direction[2] * stroke * fraction,
            ];
            let solved = realityos_semantics::kinematics::with_ik_q_seed(Some(&seed), || {
                realityos_semantics::kinematics::solve_ik(&self.model, &chain, "ee", target, &seed)
            });
            if let Ok((q, trace)) = solved {
                if trace.residual <= 1e-3 {
                    self.command_q(&q, fingers, 15);
                    seed = q;
                }
            }
        }
    }

    fn set_arm(&mut self, arm: &[f64]) {
        let state = self.inst().step(0).expect("state");
        let mut qpos: Vec<f64> = state["state"]["qpos"]
            .as_array()
            .or_else(|| state["qpos"].as_array())
            .map(|values| values.iter().filter_map(|value| value.as_f64()).collect())
            .unwrap_or_default();
        for (name, value) in self.arm_names.iter().zip(arm) {
            if let Some(joint) = self.inspect["joints"].as_array().and_then(|joints| {
                joints
                    .iter()
                    .find(|joint| joint["name"].as_str() == Some(name))
            }) {
                if let Some(adr) = joint["qposadr"].as_u64() {
                    let adr = adr as usize;
                    if adr < qpos.len() {
                        qpos[adr] = *value;
                    }
                }
            }
        }
        self.inst()
            .rpc(&json!({"cmd":"reset","qpos": qpos}))
            .expect("reset qpos");
    }

    fn command_q(&mut self, arm: &[f64], fingers: &[f64], steps: u32) {
        let mut ctrl = self.inst().peek_ctrl().expect("ctrl").0;
        for (name, value) in self.arm_names.iter().zip(arm) {
            if let Some(index) = actuator_index(&self.inspect, name) {
                if index < ctrl.len() {
                    ctrl[index] = *value;
                }
            }
        }
        for (name, value) in ["finger_l", "finger_r"].into_iter().zip(fingers) {
            if let Some(index) = actuator_index(&self.inspect, name) {
                if index < ctrl.len() {
                    ctrl[index] = *value;
                }
            }
        }
        self.inst().set_ctrl(&ctrl).expect("set ctrl");
        self.inst().step(steps).expect("step");
    }

    fn arm_q(&mut self) -> Vec<f64> {
        let state = self.inst().step(0).expect("state");
        let qpos = state["state"]["qpos"]
            .as_array()
            .or_else(|| state["qpos"].as_array())
            .cloned()
            .unwrap_or_default();
        self.arm_names
            .iter()
            .filter_map(|name| joint_q(&self.inspect, &qpos, name))
            .collect()
    }

    fn finger_q(&mut self) -> Vec<f64> {
        let state = self.inst().step(0).expect("state");
        let qpos = state["state"]["qpos"]
            .as_array()
            .or_else(|| state["qpos"].as_array())
            .cloned()
            .unwrap_or_default();
        ["finger_l", "finger_r"]
            .into_iter()
            .filter_map(|name| joint_q(&self.inspect, &qpos, name))
            .collect()
    }

    fn object_xyz(&mut self) -> [f64; 3] {
        let state = self.inst().step(0).expect("state");
        let xpos = state["state"]["xpos"]
            .as_object()
            .or_else(|| state["xpos"].as_object());
        let Some(body) = xpos
            .and_then(|map| map.get("obj0"))
            .and_then(|v| v.as_array())
        else {
            return [0.22, 0.0, self.object_z];
        };
        [
            body.first().and_then(|v| v.as_f64()).unwrap_or(0.0),
            body.get(1).and_then(|v| v.as_f64()).unwrap_or(0.0),
            body.get(2)
                .and_then(|v| v.as_f64())
                .unwrap_or(self.object_z),
        ]
    }

    fn site(&mut self, name: &str) -> [f64; 3] {
        let state = self.inst().step(0).expect("state");
        let sites = state["state"]["sites"]
            .as_object()
            .or_else(|| state["sites"].as_object())
            .expect("sites");
        let body = sites.get(name).and_then(|v| v.as_array()).expect(name);
        [
            body[0].as_f64().unwrap_or(0.0),
            body[1].as_f64().unwrap_or(0.0),
            body[2].as_f64().unwrap_or(0.0),
        ]
    }

    fn ctrl_writes(&mut self) -> u64 {
        self.inst().peek_ctrl().map(|(_, n)| n).unwrap_or(0)
    }
}

fn parse_lift_state(line: &str) -> Option<(Vec<f64>, Vec<f64>)> {
    let (_, payload) = line.split_once('=')?;
    let (arm, fingers) = payload.split_once(";fingers=")?;
    Some((parse_named_bits(arm)?, parse_named_bits(fingers)?))
}

fn parse_named_bits(payload: &str) -> Option<Vec<f64>> {
    payload
        .split(',')
        .map(|item| {
            let (_, bits) = item.split_once('=')?;
            Some(f64::from_bits(bits.parse::<u64>().ok()?))
        })
        .collect()
}

fn exact_values(left: &[f64], right: &[f64]) -> bool {
    left.len() == right.len()
        && left
            .iter()
            .zip(right)
            .all(|(left, right)| left.to_bits() == right.to_bits())
}

fn q_tracks(measured: &[f64], target: &[f64], tolerance: f64) -> bool {
    measured.len() == target.len()
        && measured.iter().zip(target).all(|(measured, target)| {
            measured.is_finite() && target.is_finite() && (measured - target).abs() <= tolerance
        })
}

fn grasp_action_covers(
    authorized: &realityos_reasoner_bridge::AuthorizedReasonerAction,
    now_s: f64,
) -> bool {
    let frozen = authorized.action.frozen();
    frozen.authority_granted
        && frozen
            .execution_authorization
            .as_ref()
            .is_some_and(|grant| grant.covers(frozen, now_s))
        && authorized.grant.evidence_status == EVIDENCE_STATUS
        && !authorized.grant.metal
        && grant_covers(&authorized.grant, &authorized.grant.scope, now_s).is_ok()
        && authorized
            .grant
            .scope
            .action_envelope
            .exactly_matches(&frozen.action_envelope)
}

fn uncertain_friction() -> PhysicalParameterBelief {
    let mut belief =
        PhysicalParameterBelief::declared_point(PhysicalParameter::SupportFriction, 0.5, "prior");
    belief.narrow_interval(PhysicalParameter::SupportFriction, [0.05, 0.8], "uncertain");
    belief.with_unknown(PhysicalParameter::ToolObjectFriction, "finger")
}

fn finger_friction() -> PhysicalParameterBelief {
    let mut belief = uncertain_friction();
    belief.narrow_interval(
        PhysicalParameter::ToolObjectFriction,
        [0.8, 0.8],
        "finger-mu",
    );
    belief
}

fn authorize_planar_current(
    os: &mut RealityOs,
    reasoner: &mut PhysicalReasoner,
    request: &ReasoningRequest,
) -> realityos_semantics::physical_reasoner::AuthorizedAction {
    authorize_current_action(os, reasoner, request, &simulation_admission(40.0), &[])
        .expect("reasoner bridge authorizes only a current planar action")
        .action
}

fn candidate_lines(
    candidates: &[realityos_semantics::physical_reasoner::CandidateSummary],
) -> String {
    candidates
        .iter()
        .map(|candidate| {
            format!(
                "{} {} {}",
                candidate.family, candidate.action_key, candidate.status
            )
        })
        .collect::<Vec<_>>()
        .join("\n")
}

fn declared_ee_cross_section_radius(
    model: &realityos_semantics::embodiment::EmbodimentModel,
    ee_name: &str,
) -> Option<f64> {
    let ee = model.end_effectors.iter().find(|ee| ee.name == ee_name)?;
    let frame = model.frames.iter().find(|frame| frame.name == ee.frame)?;
    model
        .collision_geoms
        .iter()
        .filter(|geom| {
            geom.owner_body == frame.parent_body
                && geom.collision_role == realityos_semantics::geometry::CollisionRole::Collision
        })
        .filter_map(|geom| match &geom.shape {
            realityos_semantics::geometry::PrimitiveShape::Sphere { radius }
            | realityos_semantics::geometry::PrimitiveShape::Capsule { radius, .. }
            | realityos_semantics::geometry::PrimitiveShape::Cylinder { radius, .. } => {
                Some(radius.abs())
            }
            realityos_semantics::geometry::PrimitiveShape::Box { half_extents } => Some(
                half_extents
                    .iter()
                    .map(|extent| extent.abs())
                    .fold(0.0, f64::max),
            ),
            realityos_semantics::geometry::PrimitiveShape::ConvexMesh { vertices } => vertices
                .iter()
                .map(|vertex| vertex[0].hypot(vertex[1]).hypot(vertex[2]))
                .reduce(f64::max),
            realityos_semantics::geometry::PrimitiveShape::Plane { .. }
            | realityos_semantics::geometry::PrimitiveShape::Unsupported { .. } => None,
        })
        .reduce(f64::max)
}

fn vector_delta_distance(left: [f64; 3], right: [f64; 3]) -> f64 {
    ((left[0] - right[0]).powi(2) + (left[1] - right[1]).powi(2) + (left[2] - right[2]).powi(2))
        .sqrt()
}

fn normal_force_sum(truth: &VerifierTruth, object: &str, finger: &str) -> Option<f64> {
    let contacts = truth
        .contacts
        .iter()
        .filter(|contact| {
            (contact.body1 == object && contact.body2 == finger)
                || (contact.body2 == object && contact.body1 == finger)
        })
        .collect::<Vec<_>>();
    if contacts.is_empty() {
        return None;
    }
    contacts.into_iter().try_fold(0.0, |sum, contact| {
        if !contact.force_available {
            return None;
        }
        let force = contact.normal_force?;
        (force.is_finite() && force >= 0.0).then_some(sum + force)
    })
}

fn q_line(contents: &str, prefix: &str) -> Vec<f64> {
    contents
        .lines()
        .find_map(|line| line.strip_prefix(prefix))
        .map(|text| {
            text.split(',')
                .filter_map(|value| value.parse::<u64>().ok())
                .map(f64::from_bits)
                .collect()
        })
        .unwrap_or_default()
}

fn actuator_index(inspect: &Value, joint: &str) -> Option<usize> {
    inspect["actuators"]
        .as_array()?
        .iter()
        .position(|actuator| {
            actuator["target"].as_str() == Some(joint) || actuator["name"].as_str() == Some(joint)
        })
}

fn joint_q(inspect: &Value, qpos: &[Value], name: &str) -> Option<f64> {
    let joint = inspect["joints"]
        .as_array()?
        .iter()
        .find(|joint| joint["name"].as_str() == Some(name))?;
    let adr = joint["qposadr"].as_u64()? as usize;
    qpos.get(adr)?.as_f64()
}

fn evidence_path(name: &str) -> PathBuf {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../target")
        .join(name);
    std::fs::create_dir_all(path.parent().unwrap()).ok();
    path
}
