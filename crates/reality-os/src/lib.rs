//! Reality OS decide kernel.
//!
//! See → identify → certify → issue [`CertifiedCommand`]. Does not write motors.
//! VLAs propose only. Domain physics is a plugin screen, not MEASURED.

pub mod attestation;
pub mod authority;
pub mod bounded_trust;
pub mod certificate;
pub mod command;
pub mod decide;
pub mod domains;
pub mod plan;
pub mod see;

pub use attestation::CertificateLedger;
pub use authority::{
    assert_no_learned_actuator_authority, is_forbidden_tool, is_learned_source,
    screen_external_proposal,
};
pub use bounded_trust::{certify_dispose_step, BoundedTrustEnvelope, DisposeStatus, ExecutionMode};
pub use certificate::Certificate;
pub use command::{narrow_certified_command, CertifiedCommand};
pub use decide::{DecideRequest, KernelDecision, RealityOs};
pub use domains::{DomainPlugin, DomainRegistry, WorldView};
pub use plan::{Intent, PhysicalPlan, PolicyProposal};

pub const SCHEMA: &str = "realityos.kernel_loop/1";

#[cfg(test)]
mod tests {
    use super::*;
    use realityos_kernel::DecisionStatus;

    #[test]
    fn gifted_pose_without_pixels_refuses_place() {
        let mut ros = RealityOs::new();
        let intent = Intent::language("place the part", "place");
        let world = WorldView {
            pixels_present: false,
            scene_compiled: false,
            ..WorldView::default()
        };
        let d = ros.decide(DecideRequest::new(intent, world, 1.0));
        assert_eq!(d.status, DecisionStatus::Refuse);
        assert!(d.command.is_none());
        assert!(!d.allowed);
    }

    #[test]
    fn actuator_envelope_allow_issues_command() {
        let mut ros = RealityOs::new();
        let intent = Intent::language("hold", "hold");
        let world = WorldView {
            tau_max: vec![10.0],
            ..WorldView::default()
        };
        let d = ros.decide(DecideRequest::new(intent, world, 1.0));
        assert_eq!(d.status, DecisionStatus::Allow);
        assert!(d.command.is_some());
        assert!(!d.command.unwrap().acknowledged);
    }

    #[test]
    fn cannot_upgrade_refuse_to_allow() {
        let cert = Certificate::new(DecisionStatus::Refuse, "no");
        let cmd = CertifiedCommand::issue("c", 1, 0.0, 10.0, cert, vec![0.1]).unwrap();
        let err = narrow_certified_command(cmd, DecisionStatus::Allow, None, None).unwrap_err();
        assert!(err.to_string().contains("cannot upgrade"));
    }

    #[test]
    fn cannot_widen_on_modify() {
        let cert = Certificate::new(DecisionStatus::Allow, "ok");
        let cmd = CertifiedCommand::issue("c", 1, 0.0, 10.0, cert, vec![0.1]).unwrap();
        let err = narrow_certified_command(
            cmd,
            DecisionStatus::Modify,
            Some(vec![0.5]),
            Some("wider".into()),
        )
        .unwrap_err();
        assert!(err.to_string().contains("only narrow"));
    }

    #[test]
    fn vla_proposal_still_certifies() {
        let mut ros = RealityOs::new();
        let mut req = DecideRequest::new(
            Intent::language("hold", "hold"),
            WorldView {
                tau_max: vec![1.0],
                ..WorldView::default()
            },
            1.0,
        );
        req.proposal = Some(PolicyProposal {
            action: vec![50.0],
            source: "openvla".into(),
            policy_id: "x".into(),
        });
        let d = ros.decide(req);
        assert_eq!(d.status, DecisionStatus::Refuse);
        assert!(d.command.is_none());
    }
}
