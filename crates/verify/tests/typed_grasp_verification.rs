use realityos_kernel::physical_action::PhysicalActionEnvelope;
use realityos_semantics::provenance::Provenanced;
use realityos_semantics::resource::{
    ClosingDirection, ControlledResource, CouplingModel, QualificationStatus, ResourceKind,
    ResourceTopology,
};
use realityos_verify::manipulation::{ManipulationEpisode, ManipulationMetrics};
use realityos_verify::manipulation_verify::verify_grasp;
use realityos_verify::observation::{ContactTruth, VerifierTruth};
use serde_json::json;
use std::collections::BTreeMap;

fn resource() -> ControlledResource {
    ControlledResource {
        id: "two_finger_gripper".into(),
        kind: ResourceKind::Gripper,
        topology: ResourceTopology::DirectJointGripper,
        actuator_inputs: vec!["left_actuator".into(), "right_actuator".into()],
        affected_joints: vec!["left_finger_joint".into(), "right_finger_joint".into()],
        finger_bodies: vec!["left_finger".into(), "right_finger".into()],
        coupling: CouplingModel::none(),
        command_coordinate: "joint_position".into(),
        opening_range: Provenanced::declared([0.0, 0.04], "test", 0.0),
        command_range: Provenanced::declared([0.0, 0.04], "test", 0.0),
        closing_direction: ClosingDirection::TowardMin,
        force_bound: Provenanced::declared([0.0, 4.0], "test", 0.0),
        qualification: QualificationStatus::Qualified,
        unsupported_detail: None,
    }
}

fn envelope() -> PhysicalActionEnvelope {
    PhysicalActionEnvelope::PinchGrasp {
        resource_id: "two_finger_gripper".into(),
        finger_joint_names: vec!["left_finger_joint".into(), "right_finger_joint".into()],
        open_positions: vec![0.04, 0.04],
        close_positions: vec![0.005, 0.005],
        per_finger_force_ceiling_n: 3.0,
        lift_trajectory_digest: "a".repeat(64),
        maximum_lift_m: 0.04,
    }
}

fn contact(finger: &str, force: Option<f64>) -> ContactTruth {
    ContactTruth {
        body1: "object".into(),
        body2: finger.into(),
        force: force.unwrap_or_default(),
        normal_force: force,
        force_available: force.is_some(),
        ..ContactTruth::default()
    }
}

fn before() -> VerifierTruth {
    let mut xpos = BTreeMap::new();
    xpos.insert("object".into(), vec![0.0, 0.0, 0.1]);
    xpos.insert("ee_link".into(), vec![0.0, 0.0, 0.2]);
    VerifierTruth {
        xpos,
        ..VerifierTruth::default()
    }
}

fn after() -> VerifierTruth {
    let mut xpos = BTreeMap::new();
    xpos.insert("object".into(), vec![0.0, 0.0, 0.125]);
    xpos.insert("ee_link".into(), vec![0.0, 0.0, 0.225]);
    VerifierTruth {
        xpos,
        contacts: vec![
            contact("left_finger", Some(1.0)),
            contact("right_finger", Some(1.0)),
        ],
        ..VerifierTruth::default()
    }
}

fn verify(
    after: &VerifierTruth,
    authority: Option<&PhysicalActionEnvelope>,
) -> realityos_verify::manipulation_verify::PrivilegedVerdict {
    verify_with_opening(after, authority, Some(0.02))
}

fn verify_with_opening(
    after: &VerifierTruth,
    authority: Option<&PhysicalActionEnvelope>,
    opening: Option<f64>,
) -> realityos_verify::manipulation_verify::PrivilegedVerdict {
    verify_grasp(
        &before(),
        after,
        &resource(),
        "object",
        &["table"],
        &["obstacle".into()],
        opening,
        1.0,
        &["ee_link"],
        authority,
    )
}

#[test]
fn one_sided_contact_is_not_acquisition_or_verified_hold() {
    let mut state = after();
    state.contacts.pop();
    let verdict = verify(&state, Some(&envelope()));
    assert!(!verdict.grasp_acquired);
    assert!(!verdict.verified_hold);
    assert!(!verdict.success);
}

#[test]
fn missing_or_over_limit_force_on_either_finger_rejects_hold() {
    for index in 0..2 {
        let mut missing = after();
        missing.contacts[index].normal_force = None;
        missing.contacts[index].force_available = false;
        let verdict = verify(&missing, Some(&envelope()));
        assert!(verdict.grasp_acquired);
        assert!(!verdict.verified_hold);
        assert!(verdict.force_bound_unavailable);

        let mut excessive = after();
        excessive.contacts[index].normal_force = Some(3.01);
        let verdict = verify(&excessive, Some(&envelope()));
        assert!(verdict.grasp_acquired);
        assert!(!verdict.verified_hold);
        assert_eq!(verdict.failure.as_deref(), Some("EXCESS_FORCE"));
    }
}

#[test]
fn minimum_lift_is_vertical_and_within_the_authorized_bound() {
    for (z, x) in [(0.119, 0.0), (0.125, 0.006), (0.241, 0.0)] {
        let mut state = after();
        state
            .xpos
            .insert("ee_link".into(), vec![x, 0.0, 0.2 + z - 0.1]);
        state.xpos.insert("object".into(), vec![x, 0.0, z]);
        let verdict = verify(&state, Some(&envelope()));
        assert!(!verdict.verified_hold, "z={z}, x={x}");
        assert!(!verdict.success);
    }
}

#[test]
fn object_follow_tolerance_is_five_millimeters() {
    for (residual, expected) in [(0.005, true), (0.0051, false)] {
        let mut state = after();
        state
            .xpos
            .insert("object".into(), vec![residual, 0.0, 0.125]);
        let verdict = verify(&state, Some(&envelope()));
        assert_eq!(verdict.verified_hold, expected, "residual={residual}");
        assert_eq!(verdict.success, expected, "residual={residual}");
    }
}

#[test]
fn table_support_and_forbidden_contact_reject_verified_hold() {
    let mut supported = after();
    supported.contacts.push(ContactTruth {
        body1: "object".into(),
        body2: "table".into(),
        ..ContactTruth::default()
    });
    assert!(!verify(&supported, Some(&envelope())).verified_hold);

    let mut forbidden = after();
    forbidden.contacts.push(ContactTruth {
        body1: "robot_arm".into(),
        body2: "obstacle".into(),
        ..ContactTruth::default()
    });
    assert!(!verify(&forbidden, Some(&envelope())).verified_hold);
}

#[test]
fn acquisition_is_counted_when_later_hold_verification_fails() {
    let mut state = after();
    state.contacts[0].normal_force = None;
    state.contacts[0].force_available = false;
    let verdict = verify(&state, Some(&envelope()));
    assert!(verdict.grasp_acquired);
    assert!(!verdict.verified_hold);

    let episode: ManipulationEpisode = serde_json::from_value(json!({
        "software_sha":"test", "robot_id":"test", "model_hash":"test",
        "object_definitions":[], "world_seed":1, "skill_contract":"skill.grasp",
        "joint_state":[], "object_evidence":{}, "commands":[],
        "authority_decisions":[], "contacts":[], "support_relations":[],
        "task_result":"fail", "failure_taxonomy":verdict.failure,
        "evidence_used":verdict.evidence_used, "physical_violations":[],
        "ctrl_writes":0, "unauthorized_writes":0, "metal":false,
        "evidence_status":"SIMULATION_ONLY", "perception":"PERFECT_PERCEPTION",
        "simulation_only":true, "expected_refusal":false
    }))
    .unwrap();
    let mut metrics = ManipulationMetrics::default();
    metrics.absorb(&episode);
    assert_eq!(metrics.grasp_acquisition_success, 1);
    assert_eq!(metrics.grasp_verified_hold_success, 0);
}

#[test]
fn object_limited_closure_allows_verified_hold_but_open_jaw_does_not() {
    let held = after();
    let verdict = verify_with_opening(&held, Some(&envelope()), Some(0.115));
    assert!(verdict.verified_hold);
    assert!(verdict.success);

    let verdict = verify_with_opening(&held, Some(&envelope()), Some(0.5));
    assert!(!verdict.verified_hold);
    assert!(!verdict.success);
}

#[test]
fn hold_requires_matching_typed_authority() {
    let verdict = verify(&after(), None);
    assert!(verdict.grasp_acquired);
    assert!(!verdict.verified_hold);
    assert!(!verdict.success);

    let mut mismatch = envelope();
    if let PhysicalActionEnvelope::PinchGrasp { resource_id, .. } = &mut mismatch {
        *resource_id = "different_gripper".into();
    }
    assert!(!verify(&after(), Some(&mismatch)).verified_hold);

    let mut overdeclared_force = envelope();
    if let PhysicalActionEnvelope::PinchGrasp {
        per_finger_force_ceiling_n,
        ..
    } = &mut overdeclared_force
    {
        *per_finger_force_ceiling_n = 4.1;
    }
    let verdict = verify(&after(), Some(&overdeclared_force));
    assert!(verdict.grasp_acquired);
    assert!(!verdict.verified_hold);
}
