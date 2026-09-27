//! Scoped simulation permission for one exact physical action.
//!
//! Eligibility is a policy reading. The grant is an `IssuedCommand` that has
//! also passed the simulation governor gate. This path does not construct
//! `OnlineWrite` and does not claim hardware evidence.

use realityos_core::domains::BodyRegion;
use realityos_core::{
    is_learned_source, DecideRequest, Intent, PolicyProposal, ProposalClass, RealityOs, WorldView,
};
use realityos_governor::{admit_from_parts, RuntimeGovernor, RuntimeIdentity};

use crate::packages::GovernorPackage;
use realityos_plant::{ActuationCommand, SimPlant};
use sha2::{Digest, Sha256};

pub const SIMULATION_ONLY: &str = "SIMULATION_ONLY";

#[derive(Debug, Clone, PartialEq)]
pub struct PhysicalActionScope {
    pub model_id: String,
    pub embodiment_id: String,
    pub observation_epoch: String,
    pub candidate_id: String,
    pub action_key: String,
    pub witness_digest: String,
    pub requested_stroke_m: f64,
    pub execution_bound_m: f64,
    pub issued_at_s: f64,
    pub expires_at_s: f64,
    pub actuator_id: String,
    pub observation_contract_id: String,
    pub abort_contract_id: String,
}

#[derive(Debug, Clone, PartialEq)]
pub struct ScopedSimulationGrant {
    pub command_id: String,
    pub sequence: i64,
    pub scope_digest: String,
    pub scope: PhysicalActionScope,
    pub gate_event: String,
    pub evidence_status: &'static str,
    pub metal: bool,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ScopeRefusal {
    LearnedSourceCannotAuthorize,
    ActionBlacklisted,
    SensorEvidenceMissing,
    ScopeIncomplete(&'static str),
    KernelRefused(String),
    GovernorRefused(Vec<String>),
    GateRefused(String),
    BindingMismatch,
    Expired,
}

impl std::fmt::Display for ScopeRefusal {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::LearnedSourceCannotAuthorize => {
                write!(f, "LEARNED_OR_EXPERIENCE_CANNOT_AUTHORIZE")
            }
            Self::ActionBlacklisted => write!(f, "ACTION_BLACKLISTED"),
            Self::SensorEvidenceMissing => write!(f, "SENSOR_EVIDENCE_MISSING"),
            Self::ScopeIncomplete(reason) => write!(f, "SCOPE_INCOMPLETE:{reason}"),
            Self::KernelRefused(reason) => write!(f, "KERNEL_REFUSED:{reason}"),
            Self::GovernorRefused(reasons) => write!(f, "GOVERNOR_REFUSED:{}", reasons.join(",")),
            Self::GateRefused(reason) => write!(f, "GATE_REFUSED:{reason}"),
            Self::BindingMismatch => write!(f, "GRANT_BINDING_MISMATCH"),
            Self::Expired => write!(f, "GRANT_EXPIRED"),
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AuthorityEligibility {
    pub eligible: bool,
    pub reason: String,
}

pub fn scope_digest(scope: &PhysicalActionScope) -> String {
    let canonical = format!(
        "m={}\ne={}\no={}\nc={}\na={}\nw={}\ns={:.9}\nb={:.9}\ni={:.6}\nx={:.6}\nu={}\nv={}\nz={}\n",
        scope.model_id,
        scope.embodiment_id,
        scope.observation_epoch,
        scope.candidate_id,
        scope.action_key,
        scope.witness_digest,
        scope.requested_stroke_m,
        scope.execution_bound_m,
        scope.issued_at_s,
        scope.expires_at_s,
        scope.actuator_id,
        scope.observation_contract_id,
        scope.abort_contract_id,
    );
    hex::encode(Sha256::digest(canonical.as_bytes()))
}

fn nonempty(value: &str) -> bool {
    !value.trim().is_empty()
}

fn experience_or_learned(proposal: &PolicyProposal) -> bool {
    proposal.class == ProposalClass::UntrustedLearned
        || proposal.is_learned()
        || is_learned_source(&proposal.source)
        || is_learned_source(&proposal.policy_id)
        || proposal.source.to_ascii_lowercase().contains("experience")
        || proposal
            .policy_id
            .to_ascii_lowercase()
            .contains("experience")
}

fn scope_problem(scope: &PhysicalActionScope) -> Option<&'static str> {
    if !nonempty(&scope.model_id) {
        return Some("model_id");
    }
    if !nonempty(&scope.embodiment_id) {
        return Some("embodiment_id");
    }
    if !nonempty(&scope.observation_epoch) {
        return Some("observation_epoch");
    }
    if !nonempty(&scope.candidate_id) {
        return Some("candidate_id");
    }
    if !nonempty(&scope.action_key) {
        return Some("action_key");
    }
    if !nonempty(&scope.witness_digest) {
        return Some("witness_digest");
    }
    if !scope.requested_stroke_m.is_finite() || scope.requested_stroke_m <= 0.0 {
        return Some("requested_stroke");
    }
    if !scope.execution_bound_m.is_finite()
        || scope.execution_bound_m + 1e-12 < scope.requested_stroke_m
    {
        return Some("execution_bound");
    }
    if !scope.issued_at_s.is_finite() || scope.issued_at_s <= 0.0 {
        return Some("issued_at");
    }
    if !scope.expires_at_s.is_finite() || scope.expires_at_s <= scope.issued_at_s {
        return Some("expires_at");
    }
    if !nonempty(&scope.actuator_id) {
        return Some("actuator_id");
    }
    if !nonempty(&scope.observation_contract_id) {
        return Some("observation_contract");
    }
    if !nonempty(&scope.abort_contract_id) {
        return Some("abort_contract");
    }
    None
}

/// Policy eligibility for an exact action. This does not issue a command.
pub fn evaluate_action_eligibility(
    scope: &PhysicalActionScope,
    proposal: &PolicyProposal,
    forbidden_action_keys: &[String],
    sensor_evidence_present: bool,
) -> AuthorityEligibility {
    if experience_or_learned(proposal) {
        return AuthorityEligibility {
            eligible: false,
            reason: "LEARNED_OR_EXPERIENCE_CANNOT_AUTHORIZE".into(),
        };
    }
    if forbidden_action_keys
        .iter()
        .any(|key| key == &scope.action_key)
    {
        return AuthorityEligibility {
            eligible: false,
            reason: "ACTION_BLACKLISTED".into(),
        };
    }
    if !sensor_evidence_present {
        return AuthorityEligibility {
            eligible: false,
            reason: "SENSOR_EVIDENCE_MISSING".into(),
        };
    }
    if let Some(problem) = scope_problem(scope) {
        return AuthorityEligibility {
            eligible: false,
            reason: format!("SCOPE_INCOMPLETE:{problem}"),
        };
    }
    AuthorityEligibility {
        eligible: true,
        reason: "ELIGIBLE_FOR_EXACT_ACTION".into(),
    }
}

/// Issue a simulation-scoped grant for this exact action through `RealityOs::decide`
/// and the simulation governor gate. A previous grant is not an input.
pub fn issue_scoped_simulation_grant(
    os: &mut RealityOs,
    scope: &PhysicalActionScope,
    proposal: &PolicyProposal,
    now_s: f64,
    forbidden_action_keys: &[String],
    sensor_evidence_present: bool,
) -> Result<ScopedSimulationGrant, ScopeRefusal> {
    let eligibility = evaluate_action_eligibility(
        scope,
        proposal,
        forbidden_action_keys,
        sensor_evidence_present,
    );
    if !eligibility.eligible {
        return Err(match eligibility.reason.as_str() {
            "LEARNED_OR_EXPERIENCE_CANNOT_AUTHORIZE" => ScopeRefusal::LearnedSourceCannotAuthorize,
            "ACTION_BLACKLISTED" => ScopeRefusal::ActionBlacklisted,
            "SENSOR_EVIDENCE_MISSING" => ScopeRefusal::SensorEvidenceMissing,
            other => ScopeRefusal::ScopeIncomplete(incomplete_label(other)),
        });
    }
    let digest = scope_digest(scope);
    let mut request = DecideRequest::new(
        Intent {
            text: scope.action_key.clone(),
            verb: "push".into(),
            target_object: scope.candidate_id.clone(),
            source: "external_deterministic".into(),
            require_scene: false,
        },
        WorldView {
            contact_force_n: Some(1.0),
            body_region: Some(BodyRegion::Hand),
            ..WorldView::default()
        },
        now_s,
    );
    request.proposal = Some(PolicyProposal::external_deterministic(
        vec![1.0],
        "canonical-physical-decision",
    ));
    request.command_id = format!("sim-scope:{digest}");
    request.sequence = (now_s * 1000.0) as i64;
    request.ttl_s = scope.expires_at_s - now_s;
    let decision = os.decide(request);
    if !decision.allowed {
        return Err(ScopeRefusal::KernelRefused(decision.physical_reason));
    }
    let issued = decision
        .command
        .ok_or_else(|| ScopeRefusal::KernelRefused("decide_allowed_without_command".into()))?;
    let identity = RuntimeIdentity::sim(&scope.model_id)
        .map_err(|_| ScopeRefusal::ScopeIncomplete("model_id"))?;
    let mut governor = RuntimeGovernor::new(identity, SimPlant::new("sim-scope", 1, 10.0));
    governor.heartbeat(now_s);
    governor.mark_sensor(now_s, Some(scope.observation_epoch.clone()));
    let pre = governor.pre_actuation_check(now_s);
    if !pre.is_empty() {
        return Err(ScopeRefusal::GovernorRefused(pre));
    }
    let gate_request = admit_from_parts(
        issued.command_id(),
        issued.sequence_value(),
        issued.as_command().allowed_action(),
        issued.as_command().certificate_status(),
        scope.model_id.as_str(),
        issued.as_command().expires_at_s(),
    )
    .map_err(|error| ScopeRefusal::GateRefused(error.to_string()))?;
    let verdict = GovernorPackage::gate_only(&gate_request);
    if !verdict.ok {
        return Err(ScopeRefusal::GateRefused(verdict.violations.join(",")));
    }
    if verdict.command_id.as_deref() != Some(issued.command_id()) {
        return Err(ScopeRefusal::BindingMismatch);
    }
    Ok(ScopedSimulationGrant {
        command_id: issued.command_id().to_string(),
        sequence: issued.sequence_value(),
        scope_digest: digest,
        scope: scope.clone(),
        gate_event: verdict.event,
        evidence_status: SIMULATION_ONLY,
        metal: false,
    })
}

fn incomplete_label(reason: &str) -> &'static str {
    if reason.contains("model_id") {
        "model_id"
    } else if reason.contains("embodiment") {
        "embodiment_id"
    } else if reason.contains("observation") {
        "observation_epoch"
    } else if reason.contains("candidate") {
        "candidate_id"
    } else if reason.contains("action_key") {
        "action_key"
    } else if reason.contains("witness") {
        "witness_digest"
    } else if reason.contains("requested_stroke") {
        "requested_stroke"
    } else if reason.contains("execution_bound") {
        "execution_bound"
    } else if reason.contains("issued_at") {
        "issued_at"
    } else if reason.contains("expires") {
        "expires_at"
    } else if reason.contains("actuator") {
        "actuator_id"
    } else if reason.contains("observation_contract") {
        "observation_contract"
    } else {
        "abort_contract"
    }
}

fn same_text(left: &str, right: &str) -> bool {
    left == right
}

fn same_quantity(left: f64, right: f64) -> bool {
    left.is_finite() && right.is_finite() && (left - right).abs() <= 1e-9
}

/// A grant covers only the scope it was issued for, and only while it is fresh.
pub fn grant_covers(
    grant: &ScopedSimulationGrant,
    scope: &PhysicalActionScope,
    now_s: f64,
) -> Result<(), ScopeRefusal> {
    if grant.metal || grant.evidence_status != SIMULATION_ONLY {
        return Err(ScopeRefusal::BindingMismatch);
    }
    if !now_s.is_finite() || now_s < scope.issued_at_s || now_s >= scope.expires_at_s {
        return Err(ScopeRefusal::Expired);
    }
    let fields_match = same_text(&grant.scope.model_id, &scope.model_id)
        && same_text(&grant.scope.embodiment_id, &scope.embodiment_id)
        && same_text(&grant.scope.observation_epoch, &scope.observation_epoch)
        && same_text(&grant.scope.candidate_id, &scope.candidate_id)
        && same_text(&grant.scope.action_key, &scope.action_key)
        && same_text(&grant.scope.witness_digest, &scope.witness_digest)
        && same_quantity(grant.scope.requested_stroke_m, scope.requested_stroke_m)
        && same_quantity(grant.scope.execution_bound_m, scope.execution_bound_m)
        && same_quantity(grant.scope.issued_at_s, scope.issued_at_s)
        && same_quantity(grant.scope.expires_at_s, scope.expires_at_s)
        && same_text(&grant.scope.actuator_id, &scope.actuator_id)
        && same_text(
            &grant.scope.observation_contract_id,
            &scope.observation_contract_id,
        )
        && same_text(&grant.scope.abort_contract_id, &scope.abort_contract_id)
        && grant.scope_digest == scope_digest(scope)
        && grant.command_id.contains(&grant.scope_digest);
    if !fields_match {
        return Err(ScopeRefusal::BindingMismatch);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scope(action: &str, witness: &str) -> PhysicalActionScope {
        PhysicalActionScope {
            model_id: "model-hash".into(),
            embodiment_id: "arm".into(),
            observation_epoch: "epoch:1".into(),
            candidate_id: format!("candidate:{action}"),
            action_key: action.into(),
            witness_digest: witness.into(),
            requested_stroke_m: 0.012,
            execution_bound_m: 0.02,
            issued_at_s: 10.0,
            expires_at_s: 40.0,
            actuator_id: "tool".into(),
            observation_contract_id: "policy-sensors:epoch:1".into(),
            abort_contract_id: "abort-and-reobserve".into(),
        }
    }

    #[test]
    fn learned_experience_blacklist_and_missing_sensors_do_not_issue_grants() {
        let mut os = RealityOs::new();
        let sample = scope("probe", "digest-a");
        let learned = PolicyProposal::learned(vec![1.0], "experience_log");
        assert_eq!(
            issue_scoped_simulation_grant(&mut os, &sample, &learned, 10.0, &[], true),
            Err(ScopeRefusal::LearnedSourceCannotAuthorize)
        );
        let ordinary =
            PolicyProposal::external_deterministic(vec![1.0], "canonical-physical-decision");
        assert_eq!(
            issue_scoped_simulation_grant(
                &mut os,
                &sample,
                &ordinary,
                10.0,
                std::slice::from_ref(&sample.action_key),
                true
            ),
            Err(ScopeRefusal::ActionBlacklisted)
        );
        assert_eq!(
            issue_scoped_simulation_grant(&mut os, &sample, &ordinary, 10.0, &[], false),
            Err(ScopeRefusal::SensorEvidenceMissing)
        );
    }

    #[test]
    fn a_grant_covers_only_its_own_scope() {
        let mut os = RealityOs::new();
        let goal = scope("goal-action", "digest-goal");
        let probe = scope("probe-action", "digest-probe");
        let proposal =
            PolicyProposal::external_deterministic(vec![1.0], "canonical-physical-decision");
        let grant = issue_scoped_simulation_grant(&mut os, &goal, &proposal, 10.0, &[], true)
            .expect("goal grant");
        assert!(!grant.metal);
        assert_eq!(grant.evidence_status, SIMULATION_ONLY);
        assert!(grant_covers(&grant, &goal, 10.0).is_ok());
        assert_eq!(
            grant_covers(&grant, &probe, 10.0),
            Err(ScopeRefusal::BindingMismatch)
        );
        let mut changed = goal.clone();
        changed.witness_digest = "digest-other".into();
        assert_eq!(
            grant_covers(&grant, &changed, 10.0),
            Err(ScopeRefusal::BindingMismatch)
        );
        changed = goal.clone();
        changed.observation_epoch = "epoch:2".into();
        assert_eq!(
            grant_covers(&grant, &changed, 10.0),
            Err(ScopeRefusal::BindingMismatch)
        );
        assert_eq!(
            grant_covers(&grant, &goal, 40.0),
            Err(ScopeRefusal::Expired)
        );
        let probe_grant =
            issue_scoped_simulation_grant(&mut os, &probe, &proposal, 11.0, &[], true)
                .expect("probe grant");
        assert_ne!(probe_grant.command_id, grant.command_id);
        assert!(grant_covers(&probe_grant, &probe, 11.0).is_ok());
        assert_eq!(
            grant_covers(&grant, &probe, 11.0),
            Err(ScopeRefusal::BindingMismatch)
        );
    }
}
