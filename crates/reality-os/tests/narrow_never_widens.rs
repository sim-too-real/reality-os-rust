use proptest::prelude::*;
use realityos_core::{narrow_certified_command, Certificate, CertifiedCommand};
use realityos_kernel::DecisionStatus;

proptest! {
    #[test]
    fn modify_never_widens_or_reverses(
        mag in 0.01f64..5.0,
        scale in 0.0f64..=1.0,
    ) {
        let issuer = vec![mag];
        let cert = Certificate::new(DecisionStatus::Allow, "ok");
        let cmd = CertifiedCommand::issue("c", 1, 0.0, 10.0, cert, issuer.clone()).unwrap();
        let narrowed = vec![mag * scale];
        let next = narrow_certified_command(
            cmd,
            DecisionStatus::Modify,
            Some(narrowed.clone()),
            None,
        )
        .unwrap();
        prop_assert!(next.allowed_action_vec()[0].abs() <= mag + 1e-12);
        let widen = narrow_certified_command(
            next,
            DecisionStatus::Modify,
            Some(vec![mag + 0.5]),
            None,
        );
        prop_assert!(widen.is_err());
    }
}
