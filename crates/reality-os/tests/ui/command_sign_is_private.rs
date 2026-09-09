use realityos_core::CertifiedCommand;

fn sign(cmd: CertifiedCommand) {
    let _ = cmd.sign(b"stolen-key");
}

fn main() {}
