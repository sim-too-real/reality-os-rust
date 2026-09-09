use realityos_governor::{RuntimeIdentity, ValidatedRuntimeIdentity};

fn forge() -> ValidatedRuntimeIdentity {
    ValidatedRuntimeIdentity {
        identity: RuntimeIdentity::sim("x").unwrap(),
    }
}

fn main() {}
