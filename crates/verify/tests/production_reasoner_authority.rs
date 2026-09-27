//! The production reasoner consumes a grant from the independent issuer.
//! It does not mint one. Evidence stays SIMULATION_ONLY.

use realityos_core::{PolicyProposal, RealityOs};
use realityos_semantics::execution_envelope::ExecutionAuthorization;
use realityos_semantics::physical_reasoner::{
    benchmark_scene, PhysicalReasoner, PlanStep, EVIDENCE_STATUS,
};
use realityos_session::{issue_scoped_simulation_grant, PhysicalActionScope, ScopeRefusal};

#[test]
fn production_reasoner_binds_only_an_independently_issued_grant() {
    let (model, request) = benchmark_scene();
    let mut reasoner = PhysicalReasoner::new(model);
    let report = reasoner.consider(&request);
    assert_eq!(report.evidence_status, EVIDENCE_STATUS);
    let probe = report
        .candidates
        .iter()
        .find(|candidate| candidate.action_key == "probe:+x" && candidate.status == "PROVED")
        .expect("probe");
    let step = PlanStep {
        family: probe.family.clone(),
        action_key: probe.action_key.clone(),
        candidate_id: probe.action_key.clone(),
        witness_digest: probe.witness_digest.clone().expect("digest"),
        witness_contents: probe.witness_contents.clone().expect("contents"),
        stroke_m: request.probe_stroke_m,
        immediate_progress: false,
        predicted_translation_m: request.probe_stroke_m,
        error_after: 0.1,
    };
    let scope = PhysicalActionScope {
        model_id: "hash-table".into(),
        embodiment_id: "synth_planar".into(),
        observation_epoch: request.scene.observation_epoch.clone(),
        candidate_id: step.candidate_id.clone(),
        action_key: step.action_key.clone(),
        witness_digest: step.witness_digest.clone(),
        requested_stroke_m: step.stroke_m,
        execution_bound_m: step.stroke_m.max(0.02),
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
        requested_stroke_m: grant.scope.requested_stroke_m,
        execution_bound_m: grant.scope.execution_bound_m,
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
