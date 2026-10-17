//! Simulation-only bridge from a proved reasoner selection to a scoped grant.

use realityos_core::{PolicyProposal, RealityOs};
use realityos_kernel::physical_action::{PhysicalActionEnvelope, WrenchLimitPolicy};
use realityos_semantics::execution_envelope::ExecutionAuthorization;
use realityos_semantics::physical_decision::ScopedGrantBinding;
use realityos_semantics::physical_reasoner::{
    AuthorizedAction, PhysicalReasoner, PlanStep, ReasoningReport, ReasoningRequest,
};
use realityos_session::{
    grant_covers, issue_scoped_simulation_grant, PhysicalActionScope, ScopeRefusal,
    ScopedSimulationGrant,
};

pub const EVIDENCE_STATUS: &str = "SIMULATION_ONLY";

#[derive(Debug, Clone, PartialEq)]
pub struct SimulationAdmission {
    pub execution_bound_m: f64,
    pub expires_at_s: f64,
    pub wrench_limit_policy: WrenchLimitPolicy,
}

#[derive(Debug, Clone, PartialEq)]
pub struct AuthorizedReasonerAction {
    pub report: ReasoningReport,
    pub action: AuthorizedAction,
    pub grant: ScopedSimulationGrant,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum BridgeRefusal {
    SensorEvidenceMissing,
    NoJustifiedAction,
    UnsupportedActionFamily(String),
    SelectionChanged,
    Scope(ScopeRefusal),
    InvalidWrenchPolicy(&'static str),
    Reasoner(&'static str),
}

impl std::fmt::Display for BridgeRefusal {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::SensorEvidenceMissing => write!(f, "SENSOR_EVIDENCE_MISSING"),
            Self::NoJustifiedAction => write!(f, "NO_JUSTIFIED_CURRENT_ACTION"),
            Self::UnsupportedActionFamily(family) => {
                write!(f, "UNSUPPORTED_ACTION_FAMILY:{family}")
            }
            Self::SelectionChanged => write!(f, "SELECTION_CHANGED_AFTER_SCOPING"),
            Self::Scope(error) => error.fmt(f),
            Self::InvalidWrenchPolicy(reason) => {
                write!(f, "INVALID_WRENCH_LIMIT_POLICY:{reason}")
            }
            Self::Reasoner(error) => write!(f, "REASONER_REFUSED:{error}"),
        }
    }
}

impl std::error::Error for BridgeRefusal {}

/// Select, scope, and independently re-select one current supported physical action.
///
/// The bridge only issues simulation grants. It has no writer, plant, or
/// hardware transport and admits only linear push/probe and the reasoner's
/// exact qualified two-finger pinch envelope.
pub fn authorize_current_action(
    os: &mut RealityOs,
    reasoner: &mut PhysicalReasoner,
    request: &ReasoningRequest,
    admission: &SimulationAdmission,
    forbidden_action_keys: &[String],
) -> Result<AuthorizedReasonerAction, BridgeRefusal> {
    if !request.evidence_fresh {
        return Err(BridgeRefusal::SensorEvidenceMissing);
    }
    let linear_objective = matches!(
        request.objective,
        realityos_semantics::physical_reasoner::PhysicalObjective::PlanarTranslation { .. }
    );
    if linear_objective {
        admission
            .wrench_limit_policy
            .validate()
            .map_err(|error| BridgeRefusal::InvalidWrenchPolicy(error.0))?;
    }

    // Install the admission cap before proof selection. This exact cloned
    // request is then used for preparation and the post-grant reselection.
    let mut effective_request = request.clone();
    effective_request.linear_execution_bound_m = Some(admission.execution_bound_m);
    effective_request.linear_wrench_limit = if linear_objective {
        Some(admission.wrench_limit_policy.scope.clone())
    } else {
        None
    };
    let report = reasoner.consider_excluding(&effective_request, forbidden_action_keys);
    let step = report
        .executable
        .clone()
        .or(report.authority_request.clone())
        .ok_or_else(|| {
            if report.candidates.iter().any(|candidate| {
                candidate.status == "LINEAR_EXECUTION_BOUND_TOO_SMALL"
                    || candidate.status == "LINEAR_EXECUTION_BOUND_INVALID"
            }) {
                BridgeRefusal::Scope(ScopeRefusal::ScopeIncomplete("execution_bound"))
            } else {
                BridgeRefusal::NoJustifiedAction
            }
        })?;
    match (step.family.as_str(), &step.action_envelope) {
        ("planar_push", PhysicalActionEnvelope::LinearStroke { .. })
        | ("pinch_grasp", PhysicalActionEnvelope::PinchGrasp { .. }) => {}
        (family, _) => return Err(BridgeRefusal::UnsupportedActionFamily(family.into())),
    }

    let prepared = reasoner
        .prepare_step(&effective_request, &step)
        .map_err(BridgeRefusal::Reasoner)?;
    if !prepared
        .action_envelope
        .exactly_matches(&step.action_envelope)
        || prepared.action_envelope.validate_for_execution().is_err()
    {
        return Err(BridgeRefusal::SelectionChanged);
    }
    match (&*step.family, &prepared.action_envelope) {
        ("planar_push", PhysicalActionEnvelope::LinearStroke { .. })
        | ("pinch_grasp", PhysicalActionEnvelope::PinchGrasp { .. }) => {}
        (family, _) => return Err(BridgeRefusal::UnsupportedActionFamily(family.into())),
    }
    if let PhysicalActionEnvelope::LinearStroke {
        wrench_limit: Some(wrench_limit),
        ..
    } = &prepared.action_envelope
    {
        if wrench_limit.abort_contract_id != prepared.abort_contract_id {
            return Err(BridgeRefusal::InvalidWrenchPolicy(
                "abort_contract_scope_mismatch",
            ));
        }
    }
    let scope = PhysicalActionScope {
        model_id: prepared.model_id.clone(),
        embodiment_id: prepared.embodiment_id.clone(),
        observation_epoch: prepared.observation_epoch.clone(),
        candidate_id: prepared.candidate_id.clone(),
        action_key: prepared.action_key.clone(),
        witness_digest: prepared.witness_digest.clone(),
        action_envelope: prepared.action_envelope.clone(),
        issued_at_s: effective_request.now_s,
        expires_at_s: admission.expires_at_s,
        actuator_id: prepared.actuator_id.clone(),
        observation_contract_id: prepared.observation_contract_id.clone(),
        abort_contract_id: prepared.abort_contract_id.clone(),
    };
    let proposal = PolicyProposal::external_deterministic(vec![1.0], "physical-reasoner-bridge");
    let grant = issue_scoped_simulation_grant(
        os,
        &scope,
        &proposal,
        effective_request.now_s,
        forbidden_action_keys,
        effective_request.evidence_fresh,
    )
    .map_err(BridgeRefusal::Scope)?;
    grant_covers(&grant, &scope, effective_request.now_s).map_err(BridgeRefusal::Scope)?;

    let mut granted_request = effective_request;
    granted_request.grants.push(ScopedGrantBinding {
        grant_id: grant.command_id.clone(),
        scope_digest: grant.scope_digest.clone(),
        action_key: grant.scope.action_key.clone(),
        candidate_id: grant.scope.candidate_id.clone(),
        witness_digest: grant.scope.witness_digest.clone(),
        observation_epoch: grant.scope.observation_epoch.clone(),
        action_envelope: grant.scope.action_envelope.clone(),
        expires_at_s: grant.scope.expires_at_s,
    });
    let report = reasoner.consider_excluding(&granted_request, forbidden_action_keys);
    let selected = report
        .executable
        .as_ref()
        .ok_or(BridgeRefusal::SelectionChanged)?;
    if !same_step(&step, selected) {
        return Err(BridgeRefusal::SelectionChanged);
    }

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
    let action = reasoner
        .authorize_step(&granted_request, selected, authorization)
        .map_err(BridgeRefusal::Reasoner)?;
    Ok(AuthorizedReasonerAction {
        report,
        action,
        grant,
    })
}

fn same_step(left: &PlanStep, right: &PlanStep) -> bool {
    left.family == right.family
        && left.action_key == right.action_key
        && left.candidate_id == right.candidate_id
        && left.witness_digest == right.witness_digest
        && left.witness_contents == right.witness_contents
        && left.action_envelope.exactly_matches(&right.action_envelope)
        && left.stroke_m.to_bits() == right.stroke_m.to_bits()
        && left.immediate_progress == right.immediate_progress
        && left.predicted_translation_m.to_bits() == right.predicted_translation_m.to_bits()
        && left.error_after.to_bits() == right.error_after.to_bits()
}
