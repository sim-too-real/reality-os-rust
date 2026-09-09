# Trusted computing base (authority kernel)

This is conceptual ownership, not a crate deletion list. Fixture and demo
crates stay; they are not inside the write-uniqueness kernel.

## Inside the write-uniqueness kernel

| Crate / module | Why it stays |
|----------------|--------------|
| `realityos-kernel` honesty, ids, `CommandOutcome`, evidence contract | Honesty floor |
| `realityos-plant` write_guard, execute, ledger+seal, signing, sealed `Plant` | Only certified write token |
| `realityos-governor` typestate rails, `OnlineWrite`, private signing key | Last ONLINE gate |
| `realityos-session` ingest, ONLINE start, SIM `write_driver` | Composition root |
| `realityos-core` `decide` → `IssuedCommand`, `Certificate`, lifecycle | Semantic certifier |

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
| `vport` | Exclusive virtual serial port (`HardwareDriverPort`) |
| `hil` | Two-process HIL harness + proof report |

Do not put a proposer, ROS adapter, or vendor SDK inside the kernel.
