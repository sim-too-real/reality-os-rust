//! The production reasoner consumes a grant from the independent issuer.
//! It does not mint one. Evidence stays SIMULATION_ONLY.

use realityos_core::{PolicyProposal, RealityOs};
use realityos_kernel::physical_action::WrenchLimitScope;
use realityos_semantics::execution_envelope::ExecutionAuthorization;
use realityos_semantics::physical_reasoner::{benchmark_scene, PhysicalReasoner, EVIDENCE_STATUS};
use realityos_session::{issue_scoped_simulation_grant, PhysicalActionScope, ScopeRefusal};

#[test]
fn production_reasoner_binds_only_an_independently_issued_grant() {
    let (model, mut request) = benchmark_scene();
    request.linear_execution_bound_m = Some(0.02);
    request.linear_wrench_limit = Some(WrenchLimitScope {
        sensor_id: "test-ft-tool".into(),
        sensor_frame: "tool0".into(),
        caps: [20.0, 20.0, 30.0, 2.0, 2.0, 2.0],
        policy_digest: "a".repeat(64),
        abort_contract_id: "abort-and-reobserve".into(),
    });
    let mut reasoner = PhysicalReasoner::new(model);
    let report = reasoner.consider(&request);
    assert_eq!(report.evidence_status, EVIDENCE_STATUS);
    let probe = report
        .candidates
        .iter()
        .find(|candidate| candidate.action_key == "probe:+x" && candidate.status == "PROVED")
        .expect("probe");
    let step = report
        .authority_request
        .clone()
        .expect("reasoner selected the proved probe for authority");
    assert_eq!(step.action_key, probe.action_key);
    let scope = PhysicalActionScope {
        model_id: "hash-table".into(),
        embodiment_id: "synth_planar".into(),
        observation_epoch: request.scene.observation_epoch.clone(),
        candidate_id: step.candidate_id.clone(),
        action_key: step.action_key.clone(),
        witness_digest: step.witness_digest.clone(),
        action_envelope: step.action_envelope.clone(),
        issued_at_s: request.now_s,
        expires_at_s: 40.0,
        actuator_id: "tool".into(),
        observation_contract_id: format!("sensors:{}", request.scene.observation_epoch),
        abort_contract_id: "abort-and-reobserve".into(),
    };
    let mut os = RealityOs::new();
    let learned = PolicyProposal::learned(vec![1.0], "experience_log");
    assert_eq!(
        issue_scoped_simulation_grant(&mut os, &scope, &learned, request.now_s, &[], true),
        Err(ScopeRefusal::LearnedSourceCannotAuthorize)
    );
    let grant = issue_scoped_simulation_grant(
        &mut os,
        &scope,
        &PolicyProposal::external_deterministic(vec![1.0], "canonical-physical-decision"),
        request.now_s,
        &[],
        true,
    )
    .expect("independent simulation grant");
    assert!(!grant.metal);
    assert_eq!(grant.evidence_status, EVIDENCE_STATUS);
    let authorization = ExecutionAuthorization {
        grant_id: grant.command_id.clone(),
        scope_digest: grant.scope_digest.clone(),
        model_id: grant.scope.model_id.clone(),
        embodiment_id: grant.scope.embodiment_id.clone(),
        observation_epoch: grant.scope.observation_epoch.clone(),
        candidate_id: grant.scope.candidate_id.clone(),
        action_key: grant.scope.action_key.clone(),
        witness_digest: grant.scope.witness_digest.clone(),
        actuator_id: grant.scope.actuator_id.clone(),
        action_envelope: grant.scope.action_envelope.clone(),
        issued_at_s: grant.scope.issued_at_s,
        expires_at_s: grant.scope.expires_at_s,
        observation_contract_id: grant.scope.observation_contract_id.clone(),
        abort_contract_id: grant.scope.abort_contract_id.clone(),
    };
    let authorized = reasoner
        .authorize_step(&request, &step, authorization.clone())
        .expect("issued grant covers the probe");
    assert!(authorized.frozen().authority_granted);
    assert_eq!(
        authorized
            .frozen()
            .execution_authorization
            .as_ref()
            .map(|grant| grant.grant_id.as_str()),
        Some(grant.command_id.as_str())
    );
    let mut other = authorization;
    other.action_key = "push:+y".into();
    assert!(reasoner.authorize_step(&request, &step, other).is_err());
}
