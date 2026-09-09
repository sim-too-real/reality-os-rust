use realityos_kernel::{DecisionStatus, UnifiedDecision};
use serde::{Deserialize, Serialize};

use crate::attestation::CertificateLedger;
use crate::authority::{is_forbidden_tool, screen_external_proposal};
use crate::certificate::Certificate;
use crate::command::CertifiedCommand;
use crate::domains::{DomainRegistry, WorldView};
use crate::plan::{Intent, PhysicalPlan, PolicyProposal};
use crate::see::evaluate_manip_observation_gate;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct KernelDecision {
    pub status: DecisionStatus,
    pub allowed: bool,
    pub action: Vec<f64>,
    pub physical_reason: String,
    pub plan: Option<PhysicalPlan>,
    pub certificate: Certificate,
    pub n_probes: u32,
    pub command: Option<CertifiedCommand>,
    pub unified: UnifiedDecision,
    pub metal: bool,
}

impl KernelDecision {
    fn from_cert(
        cert: Certificate,
        plan: Option<PhysicalPlan>,
        action: Vec<f64>,
        n_probes: u32,
        command: Option<CertifiedCommand>,
    ) -> Self {
        let status = cert.status;
        let physical_reason = cert.physical_reason.clone();
        let unified = cert.to_unified();
        Self {
            status,
            allowed: status.allowed(),
            action,
            physical_reason,
            plan,
            certificate: cert,
            n_probes,
            command,
            unified,
            metal: false,
        }
    }
}

#[derive(Debug, Clone)]
pub struct DecideRequest {
    pub intent: Intent,
    pub proposal: Option<PolicyProposal>,
    pub world: WorldView,
    pub now_s: f64,
    pub ttl_s: f64,
    pub sequence: i64,
    pub command_id: String,
    pub max_pose_std_m: f64,
}

impl DecideRequest {
    pub fn new(intent: Intent, world: WorldView, now_s: f64) -> Self {
        Self {
            intent,
            proposal: None,
            world,
            now_s,
            ttl_s: 30.0,
            sequence: 1,
            command_id: format!("cmd-{now_s}"),
            max_pose_std_m: 0.05,
        }
    }
}

pub struct RealityOs {
    pub registry: DomainRegistry,
    pub attestation: CertificateLedger,
    pub last_sequence: i64,
}

impl Default for RealityOs {
    fn default() -> Self {
        Self::new()
    }
}

impl RealityOs {
    pub fn new() -> Self {
        Self {
            registry: DomainRegistry::product_defaults(),
            attestation: CertificateLedger::new(),
            last_sequence: 0,
        }
    }

    pub fn decide(&mut self, req: DecideRequest) -> KernelDecision {
        if is_forbidden_tool(&req.intent.verb) {
            return self.finalize(
                Certificate::new(DecisionStatus::Refuse, "forbidden_tool_name")
                    .with_reasons(["forbidden_tool"]),
                None,
                vec![],
                0,
                req.now_s,
                req.ttl_s,
                &req.command_id,
                req.sequence,
            );
        }

        if let Some(p) = &req.proposal {
            let screen = screen_external_proposal(&p.source);
            if screen.learned_actuator_authority {
                return self.finalize(
                    Certificate::new(DecisionStatus::Abort, "learned_actuator_authority")
                        .with_reasons(["learned_actuator_authority"]),
                    None,
                    vec![],
                    0,
                    req.now_s,
                    req.ttl_s,
                    &req.command_id,
                    req.sequence,
                );
            }
        }

        let mog = evaluate_manip_observation_gate(
            &req.intent,
            req.world.observation.as_ref(),
            req.world.pose_std_m,
            req.max_pose_std_m,
            req.now_s,
        );
        if !mog.ok {
            return self.finalize(
                Certificate::new(mog.status, mog.physical_reason).with_reasons(mog.reasons),
                None,
                vec![],
                0,
                req.now_s,
                req.ttl_s,
                &req.command_id,
                req.sequence,
            );
        }

        let kind = match self.registry.infer_kind(&req.intent) {
            Some(k) => k,
            None => {
                return self.finalize(
                    Certificate::new(DecisionStatus::Refuse, "no domain for intent")
                        .with_reasons(["unsupported_plan_kind"]),
                    None,
                    vec![],
                    0,
                    req.now_s,
                    req.ttl_s,
                    &req.command_id,
                    req.sequence,
                );
            }
        };

        let plan = if let Some(p) = &req.proposal {
            PhysicalPlan::new(kind, p.action.clone())
        } else {
            self.registry
                .plan(kind, &req.intent, &req.world)
                .unwrap_or_else(|| PhysicalPlan::new(kind, vec![]))
        };

        if plan.lacks_explicit_target() {
            return self.finalize(
                Certificate::new(DecisionStatus::Refuse, "plan_lacks_explicit_target")
                    .with_reasons(["plan_lacks_explicit_target"]),
                Some(plan),
                vec![],
                0,
                req.now_s,
                req.ttl_s,
                &req.command_id,
                req.sequence,
            );
        }

        if !plan.finite() {
            return self.finalize(
                Certificate::new(DecisionStatus::Abort, "non_finite_plan_action")
                    .with_reasons(["non_finite_plan_action"]),
                Some(plan),
                vec![],
                0,
                req.now_s,
                req.ttl_s,
                &req.command_id,
                req.sequence,
            );
        }

        let cert = self.registry.certify(&plan, &req.world);
        let action = if cert.status.allowed() {
            plan.action.clone()
        } else {
            vec![0.0; plan.action.len()]
        };
        self.finalize(
            cert,
            Some(plan),
            action,
            0,
            req.now_s,
            req.ttl_s,
            &req.command_id,
            req.sequence,
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn finalize(
        &mut self,
        cert: Certificate,
        plan: Option<PhysicalPlan>,
        action: Vec<f64>,
        n_probes: u32,
        now_s: f64,
        ttl_s: f64,
        command_id: &str,
        sequence: i64,
    ) -> KernelDecision {
        self.attestation
            .append(cert.status.as_str(), &cert.physical_reason);
        let command = if cert.status.allowed() {
            self.last_sequence = sequence;
            CertifiedCommand::issue(
                command_id,
                sequence,
                now_s,
                ttl_s,
                cert.clone(),
                action.clone(),
            )
            .ok()
        } else {
            None
        };
        KernelDecision::from_cert(cert, plan, action, n_probes, command)
    }
}
