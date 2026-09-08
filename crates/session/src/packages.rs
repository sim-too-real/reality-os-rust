//! Sealed customer surface. Command in → decide → gate → HMAC-shaped report out.
//! No Foundry. No motor write from this module itself.

use realityos_core::{DecideRequest, Intent, KernelDecision, RealityOs, WorldView};
use realityos_governor::{GovernorGateRequest, GovernorGateVerdict};
use realityos_kernel::{DecisionStatus, HonestyStamp, GATE_EVIDENCE, KERNEL_EVIDENCE};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PackageReport {
    pub schema: String,
    pub ok: bool,
    pub status: String,
    pub reason: String,
    pub metal: bool,
    pub online: bool,
    pub learned_actuator_authority: bool,
    pub invent_authority: bool,
    pub evidence_status: String,
}

impl PackageReport {
    fn sim(schema: &str, ok: bool, status: DecisionStatus, reason: &str, evidence: &str) -> Self {
        Self {
            schema: schema.into(),
            ok,
            status: status.as_str().into(),
            reason: reason.into(),
            metal: false,
            online: false,
            learned_actuator_authority: false,
            invent_authority: false,
            evidence_status: evidence.into(),
        }
    }
}

pub struct RealityOsPackage {
    ros: RealityOs,
}

impl Default for RealityOsPackage {
    fn default() -> Self {
        Self {
            ros: RealityOs::new(),
        }
    }
}

impl RealityOsPackage {
    pub fn decide(
        &mut self,
        intent: Intent,
        world: WorldView,
        now_s: f64,
    ) -> (KernelDecision, PackageReport) {
        let d = self.ros.decide(DecideRequest::new(intent, world, now_s));
        let report = PackageReport::sim(
            "realityos.packages.reality_os/1",
            d.allowed,
            d.status,
            &d.physical_reason,
            KERNEL_EVIDENCE,
        );
        (d, report)
    }
}

pub struct GovernorPackage;

impl GovernorPackage {
    pub fn gate_only(req: &GovernorGateRequest) -> GovernorGateVerdict {
        if !req.certificate_status.allowed() {
            return GovernorGateVerdict::refused(
                "gate_refused",
                req.release_hash.clone(),
                vec!["command_certificate_not_executable".into()],
            );
        }
        let mut v = GovernorGateVerdict::ok_event("gate_admit", req.release_hash.clone());
        v.command_id = Some(req.command_id.clone());
        v
    }
}

pub struct SafetyEdge {
    ros: RealityOsPackage,
}

impl Default for SafetyEdge {
    fn default() -> Self {
        Self {
            ros: RealityOsPackage::default(),
        }
    }
}

impl SafetyEdge {
    pub fn decide_and_gate(
        &mut self,
        intent: Intent,
        world: WorldView,
        now_s: f64,
        release_hash: &str,
    ) -> PackageReport {
        let (d, _) = self.ros.decide(intent, world, now_s);
        let honesty = HonestyStamp::sim(GATE_EVIDENCE).expect("stamp");
        let _ = honesty;
        if !d.allowed {
            return PackageReport::sim(
                "realityos.packages.safety_edge/1",
                false,
                d.status,
                &d.physical_reason,
                GATE_EVIDENCE,
            );
        }
        let req = GovernorGateRequest::new(
            d.command
                .as_ref()
                .map(|c| c.command_id.as_str())
                .unwrap_or("none"),
            d.command.as_ref().map(|c| c.sequence).unwrap_or(0),
            d.action.clone(),
            d.status,
            release_hash,
        );
        match req {
            Ok(r) => {
                let g = GovernorPackage::gate_only(&r);
                PackageReport::sim(
                    "realityos.packages.safety_edge/1",
                    g.ok,
                    d.status,
                    &d.physical_reason,
                    GATE_EVIDENCE,
                )
            }
            Err(e) => PackageReport::sim(
                "realityos.packages.safety_edge/1",
                false,
                DecisionStatus::Abort,
                &e.to_string(),
                GATE_EVIDENCE,
            ),
        }
    }
}
