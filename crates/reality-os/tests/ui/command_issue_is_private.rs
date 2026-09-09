use realityos_core::{Certificate, CertifiedCommand};
use realityos_kernel::DecisionStatus;

fn main() {
    let cert = Certificate::new(DecisionStatus::Allow, "ok");
    let _ = CertifiedCommand::issue("c", 1, 0.0, 10.0, cert, vec![0.1]);
}
