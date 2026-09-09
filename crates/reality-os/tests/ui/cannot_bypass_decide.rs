//! Certificate::new(ALLOW) + issue + sign + acknowledge is not a public path.
use realityos_core::{Certificate, CertifiedCommand};
use realityos_kernel::DecisionStatus;

fn main() {
    let cert = Certificate::new(DecisionStatus::Allow, "forged");
    let _ = CertifiedCommand::issue("forged", 1, 0.0, 10.0, cert, vec![0.1])
        .unwrap()
        .sign(b"k")
        .acknowledge();
}
