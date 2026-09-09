use realityos_core::{CertifiedCommand, IssuedCommand};

fn forge(cmd: CertifiedCommand) -> IssuedCommand {
    IssuedCommand { command: cmd }
}

fn main() {}
