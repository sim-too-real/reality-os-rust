# Trusted computing base (authority kernel)

This is conceptual ownership, not a crate deletion list. Fixture and demo
crates stay; they are not inside the write-uniqueness kernel.

## Inside the write-uniqueness kernel

| Crate / module | Why it stays |
|----------------|--------------|
| `realityos-kernel` honesty, ids, `CommandOutcome`, evidence contract | Honesty floor |
| `realityos-plant` write_guard, execute, ledger+seal, signing, sealed `Plant` | Only certified write token |
| `realityos-governor` typestate rails, envelope, identity, `write_driver` | Last gate |
| `realityos-session` bind, ingest, ONLINE start, acknowledge | Session binder / ack |
| `realityos-core` `decide`, `Certificate`, `CertifiedCommand`, lifecycle, typed provenance | Semantic certifier |

Physics **screens** used by `certify()` affect ALLOW/REFUSE. They are in the
*semantic* TCB for predicates, not in the *write-token* TCB.

## Outside the write-uniqueness kernel

| Crate | Role |
|-------|------|
| `vision` | Pixel fixture; not trusted evidence minting |
| `embodiment` | Body catalogs for gauntlets |
| `physics` | Closed-form screens, not MEASURED |
| `reality-os` trajectory / control / skills / bounded_trust | Demo / screens |
| `agent` | Untrusted proposer |
| `ros2` | Transport adapter; hardware writes default off |
| `rate` | SIM rate budgets |
| `data` | Debug event store |
| `gauntlet` | Scenario matrix |
| `apps/ros-governor` | SIM CLI |

Do not put a proposer, ROS adapter, or vendor SDK inside the kernel.
