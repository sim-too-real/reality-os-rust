use realityos_core::RealityOs;
use realityos_kernel::physical_action::PhysicalActionEnvelope;
use realityos_reasoner_bridge::{
    authorize_current_action, BridgeRefusal, SimulationAdmission, EVIDENCE_STATUS,
};
use realityos_semantics::physical_belief::{PhysicalParameter, PhysicalParameterBelief};
use realityos_semantics::physical_reasoner::{
    benchmark_scene, PhysicalObjective, PhysicalReasoner,
};
use realityos_session::{grant_covers, PhysicalActionScope};

fn fixture() -> (
    RealityOs,
    PhysicalReasoner,
    realityos_semantics::physical_reasoner::ReasoningRequest,
) {
    let (model, request) = benchmark_scene();
    (RealityOs::new(), PhysicalReasoner::new(model), request)
}

fn grasp_fixture() -> (
    RealityOs,
    PhysicalReasoner,
    realityos_semantics::physical_reasoner::ReasoningRequest,
) {
    use realityos_semantics::embodiment::{Actuator, Body, Joint, JointKind};
    use realityos_semantics::provenance::Provenanced;
    use realityos_semantics::resource::{
        ClosingDirection, ControlledResource, CouplingModel, QualificationStatus, ResourceKind,
        ResourceTopology,
    };
    use realityos_semantics::transform::Se3;

    let (mut model, mut request) = benchmark_scene();
    model.model_hash = "hash-grasp".into();
    model.bodies.push(Body {
        name: "lift_link".into(),
        parent: Some("link2".into()),
        mass_kg: Provenanced::unknown("bridge.grasp.fixture", 0.0),
        com: Provenanced::unknown("bridge.grasp.fixture", 0.0),
        inertia: Provenanced::unknown("bridge.grasp.fixture", 0.0),
        local_pose: Provenanced::declared(Se3::identity(), "bridge.grasp.fixture", 0.0),
    });
    model.joints.push(Joint {
        name: "lift_z".into(),
        kind: JointKind::Slide,
        axis: Provenanced::declared([0.0, 0.0, 1.0], "bridge.grasp.fixture", 0.0),
        qpos_dim: 1,
        dof_dim: 1,
        parent_body: "link2".into(),
        child_body: "lift_link".into(),
        q_min: Provenanced::declared(0.0, "bridge.grasp.fixture", 0.0),
        q_max: Provenanced::declared(0.05, "bridge.grasp.fixture", 0.0),
        dq_max: Provenanced::declared(0.1, "bridge.grasp.fixture", 0.0),
        effort_max: Provenanced::declared(10.0, "bridge.grasp.fixture", 0.0),
        origin_in_child: Provenanced::declared([0.0, 0.0, 0.0], "bridge.grasp.fixture", 0.0),
        parent_to_joint: Provenanced::declared(Se3::identity(), "bridge.grasp.fixture", 0.0),
        joint_to_child: Provenanced::declared(Se3::identity(), "bridge.grasp.fixture", 0.0),
        qpos_adr: Some(2),
        dof_adr: Some(2),
    });
    model
        .frames
        .iter_mut()
        .find(|frame| frame.name == "ee")
        .unwrap()
        .parent_body = "lift_link".into();
    model.end_effectors[0].joint_chain.push("lift_z".into());
    model.actuators.push(Actuator {
        name: "act-lift-z".into(),
        target_joint: "lift_z".into(),
        control_mode: "position".into(),
        transmission_kind: "joint".into(),
        ctrlrange: Provenanced::declared([0.0, 0.05], "bridge.grasp.fixture", 0.0),
        forcerange: Provenanced::declared([-10.0, 10.0], "bridge.grasp.fixture", 0.0),
        gear: Provenanced::declared(1.0, "bridge.grasp.fixture", 0.0),
    });
    for actuator in model
        .actuators
        .iter_mut()
        .filter(|actuator| actuator.target_joint.starts_with("finger_"))
    {
        actuator.ctrlrange = Provenanced::declared([0.0, 0.04], "bridge.grasp.fixture", 0.0);
        actuator.forcerange = Provenanced::declared([-2.0, 2.0], "bridge.grasp.fixture", 0.0);
        actuator.gear = Provenanced::declared(1.0, "bridge.grasp.fixture", 0.0);
    }
    let resource = ControlledResource {
        id: "gripper:direct-two-finger".into(),
        kind: ResourceKind::Gripper,
        topology: ResourceTopology::DirectJointGripper,
        actuator_inputs: vec!["act-finger_l".into(), "act-finger_r".into()],
        affected_joints: vec!["finger_l".into(), "finger_r".into()],
        finger_bodies: vec!["finger_l".into(), "finger_r".into()],
        coupling: CouplingModel::none(),
        command_coordinate: "joint_position".into(),
        opening_range: Provenanced::declared([0.0, 0.04], "bridge.grasp.fixture", 0.0),
        command_range: Provenanced::declared([0.0, 0.04], "bridge.grasp.fixture", 0.0),
        closing_direction: ClosingDirection::TowardMin,
        force_bound: Provenanced::declared([0.0, 2.0], "bridge.grasp.fixture", 0.0),
        qualification: QualificationStatus::Qualified,
        unsupported_detail: None,
    };
    model.resources.push(resource.clone());
    request.scene.joint_names.push("lift_z".into());
    request.scene.arm_q.push(0.0);
    request.scene.obstacles.clear();
    request.objective = PhysicalObjective::AcquireObject {
        object_id: "block".into(),
    };
    request.belief = PhysicalParameterBelief::declared_point(
        PhysicalParameter::ToolObjectFriction,
        0.8,
        "bridge.grasp.fixture",
    );
    request.gripper_force_n = Some(2.0);
    request.gripper_resource = Some(resource);
    request.linear_execution_bound_m = None;
    (RealityOs::new(), PhysicalReasoner::new(model), request)
}

fn admission() -> SimulationAdmission {
    SimulationAdmission {
        execution_bound_m: 0.05,
        expires_at_s: 40.0,
    }
}

#[test]
fn safe_probe_is_scoped_then_reselected_before_authorization() {
    let (mut os, mut reasoner, mut request) = fixture();
    request.linear_execution_bound_m = Some(admission().execution_bound_m);
    let proposed = reasoner.consider(&request);
    assert!(proposed.executable.is_none());
    assert!(proposed.authority_request.is_some());

    let admitted = authorize_current_action(&mut os, &mut reasoner, &request, &admission(), &[])
        .expect("safe probe should be admitted through the simulation gate");
    assert!(admitted.report.authority_request.is_none());
    let selected = admitted
        .report
        .executable
        .as_ref()
        .expect("final selection must be executable");
    let frozen = admitted.action.frozen();
    assert_eq!(selected.action_key, frozen.action_key);
    assert_eq!(selected.candidate_id, frozen.candidate_id);
    assert_eq!(selected.witness_digest, frozen.witness_digest);
    assert_eq!(selected.witness_contents, frozen.witness_contents);
    assert_eq!(selected.stroke_m, frozen.linear_stroke_bounds().unwrap().0);
    assert!(frozen.authority_granted);
    assert_eq!(admitted.grant.evidence_status, "SIMULATION_ONLY");
    assert_eq!(EVIDENCE_STATUS, "SIMULATION_ONLY");
    assert!(!admitted.grant.metal);

    let scope = PhysicalActionScope {
        model_id: frozen.model_id.clone(),
        embodiment_id: frozen.embodiment_id.clone(),
        observation_epoch: frozen.observation_epoch.clone(),
        candidate_id: frozen.candidate_id.clone(),
        action_key: frozen.action_key.clone(),
        witness_digest: frozen.witness_digest.clone(),
        action_envelope: PhysicalActionEnvelope::LinearStroke {
            requested_m: frozen.linear_stroke_bounds().unwrap().0,
            maximum_m: admission().execution_bound_m,
        },
        issued_at_s: request.now_s,
        expires_at_s: admission().expires_at_s,
        actuator_id: frozen.actuator_id.clone(),
        observation_contract_id: frozen.observation_contract_id.clone(),
        abort_contract_id: frozen.abort_contract_id.clone(),
    };
    assert!(grant_covers(&admitted.grant, &scope, request.now_s).is_ok());
    assert!(frozen
        .execution_authorization
        .as_ref()
        .is_some_and(|authorization| authorization.covers(frozen, request.now_s)));
}

#[test]
fn supported_grasp_is_granted_only_after_exact_reselection() {
    let (mut os, mut reasoner, request) = grasp_fixture();
    let result = authorize_current_action(&mut os, &mut reasoner, &request, &admission(), &[]);
    assert!(
        result.is_ok(),
        "supported grasp should be admitted: {result:?}"
    );
    let admitted = result.unwrap();
    assert_eq!(
        admitted.report.executable.as_ref().unwrap().action_envelope,
        admitted.action.frozen().action_envelope
    );
    assert!(
        admitted
            .action
            .frozen()
            .execution_authorization
            .as_ref()
            .is_some_and(
                |authorization| authorization.covers(admitted.action.frozen(), request.now_s)
            )
    );
    assert!(!admitted.grant.metal);
    assert_eq!(admitted.grant.evidence_status, "SIMULATION_ONLY");
}

#[test]
fn grasp_resource_force_or_lift_mismatch_refuses() {
    let (mut os, mut reasoner, request) = grasp_fixture();
    let admitted = authorize_current_action(&mut os, &mut reasoner, &request, &admission(), &[])
        .expect("fixture grasp authorization");
    let frozen = admitted.action.frozen();
    let grant = frozen.execution_authorization.as_ref().unwrap();
    let mut changed = grant.clone();
    if let PhysicalActionEnvelope::PinchGrasp { resource_id, .. } = &mut changed.action_envelope {
        *resource_id = "gripper:other-resource".into();
    }
    assert!(!changed.covers(frozen, request.now_s));
    let mut changed = grant.clone();
    if let PhysicalActionEnvelope::PinchGrasp {
        per_finger_force_ceiling_n,
        ..
    } = &mut changed.action_envelope
    {
        *per_finger_force_ceiling_n += 0.1;
    }
    assert!(!changed.covers(frozen, request.now_s));
    let mut changed = grant.clone();
    if let PhysicalActionEnvelope::PinchGrasp { maximum_lift_m, .. } = &mut changed.action_envelope
    {
        *maximum_lift_m += 0.001;
    }
    assert!(!changed.covers(frozen, request.now_s));
    let mut changed = grant.clone();
    if let PhysicalActionEnvelope::PinchGrasp {
        lift_trajectory_digest,
        ..
    } = &mut changed.action_envelope
    {
        lift_trajectory_digest.replace_range(
            0..1,
            if lift_trajectory_digest.starts_with('0') {
                "1"
            } else {
                "0"
            },
        );
    }
    assert!(!changed.covers(frozen, request.now_s));

    let mut unqualified = request.clone();
    unqualified.gripper_resource.as_mut().unwrap().qualification =
        realityos_semantics::resource::QualificationStatus::Unverified;
    assert!(
        authorize_current_action(&mut os, &mut reasoner, &unqualified, &admission(), &[]).is_err()
    );
    let mut force_out_of_range = request;
    force_out_of_range.gripper_force_n = Some(3.0);
    assert!(authorize_current_action(
        &mut os,
        &mut reasoner,
        &force_out_of_range,
        &admission(),
        &[]
    )
    .is_err());
}

#[test]
fn unsupported_topology_stale_evidence_expiry_and_forbidden_grasp_refuse() {
    let (mut os, mut reasoner, request) = grasp_fixture();
    let mut unsupported = request.clone();
    unsupported.gripper_resource.as_mut().unwrap().topology =
        realityos_semantics::resource::ResourceTopology::TendonDrivenGripper;
    assert!(
        authorize_current_action(&mut os, &mut reasoner, &unsupported, &admission(), &[]).is_err()
    );
    let mut stale = request.clone();
    stale.evidence_fresh = false;
    assert_eq!(
        authorize_current_action(&mut os, &mut reasoner, &stale, &admission(), &[]),
        Err(BridgeRefusal::SensorEvidenceMissing)
    );
    assert!(authorize_current_action(
        &mut os,
        &mut reasoner,
        &request,
        &SimulationAdmission {
            execution_bound_m: 0.1,
            expires_at_s: 9.0
        },
        &[]
    )
    .is_err());
    assert!(authorize_current_action(
        &mut os,
        &mut reasoner,
        &request,
        &admission(),
        &["grasp:y".into()]
    )
    .is_err());
}

#[test]
fn stale_sensor_evidence_is_refused_before_grant_issuance() {
    let (mut os, mut reasoner, mut request) = fixture();
    request.evidence_fresh = false;
    assert_eq!(
        authorize_current_action(&mut os, &mut reasoner, &request, &admission(), &[]),
        Err(BridgeRefusal::SensorEvidenceMissing)
    );
}

#[test]
fn undersized_execution_bound_is_refused_by_scoped_authority() {
    let (mut os, mut reasoner, request) = fixture();
    let small = SimulationAdmission {
        execution_bound_m: 0.001,
        expires_at_s: 40.0,
    };
    assert_eq!(
        authorize_current_action(&mut os, &mut reasoner, &request, &small, &[]),
        Err(BridgeRefusal::Scope(
            realityos_session::ScopeRefusal::ScopeIncomplete("execution_bound")
        ))
    );
}

#[test]
fn expired_admission_is_refused() {
    let (mut os, mut reasoner, request) = fixture();
    let expired = SimulationAdmission {
        execution_bound_m: 0.05,
        expires_at_s: request.now_s,
    };
    assert!(matches!(
        authorize_current_action(&mut os, &mut reasoner, &request, &expired, &[]),
        Err(BridgeRefusal::Scope(_))
    ));
}

#[test]
fn no_justified_current_action_is_refused() {
    let (mut os, mut reasoner, mut request) = fixture();
    request.scene.obstacles.extend([
        realityos_semantics::contact_collision::NamedBox::aabb(
            "north-block",
            [0.20, 0.030, 0.0],
            [0.05, 0.008, 0.02],
        ),
        realityos_semantics::contact_collision::NamedBox::aabb(
            "east-block",
            [0.230, 0.0, 0.0],
            [0.008, 0.05, 0.02],
        ),
        realityos_semantics::contact_collision::NamedBox::aabb(
            "west-block",
            [0.170, 0.0, 0.0],
            [0.008, 0.05, 0.02],
        ),
    ]);
    assert_eq!(
        authorize_current_action(&mut os, &mut reasoner, &request, &admission(), &[]),
        Err(BridgeRefusal::NoJustifiedAction)
    );
}

#[test]
fn grasp_family_refuses_without_a_typed_gripper_resource() {
    let (mut os, mut reasoner, mut request) = grasp_fixture();
    request.gripper_resource = None;
    let report = reasoner.consider(&request);
    assert!(report.executable.is_none());
    assert_eq!(
        report
            .candidates
            .iter()
            .find(|candidate| candidate.action_key == "grasp:y")
            .unwrap()
            .status,
        "GRIPPER_RESOURCE_UNKNOWN"
    );
    assert_eq!(
        authorize_current_action(&mut os, &mut reasoner, &request, &admission(), &[]),
        Err(BridgeRefusal::NoJustifiedAction)
    );
}
