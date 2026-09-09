use realityos_core::CertifiedCommand;

fn ack(cmd: CertifiedCommand) {
    let _ = cmd.acknowledge();
}

fn main() {}
