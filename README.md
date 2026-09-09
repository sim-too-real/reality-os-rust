# reality-os-rust

Rust last-gate for **Reality OS + Governor**. This is `theworld-runtime`, not Foundry invent.

Python in `C:\projects\theworld` remains the living product. This repo is a grouped, fail-closed port of the **authority kernel**: five-word verdicts, `CertifiedCommand`, identity, e-stop latch, envelope, command ledger, `RuntimeSession.bind_and_dispatch`.

**SIM ≠ metal.** Learned systems never write motors. Governor may only **allow / narrow / abort**. It cannot invent a plan or upgrade a refuse.

## Why this exists

ROS 2, Isaac ROS, Apex.OS, NVIDIA Halos, and QNX are middleware, perception, or safety *operating systems*. They do not own a typed choke point:

```
intent / VLA caption
        ↓  proposal only
   Reality OS.decide   → IssuedCommand
        ↓  ONLINE authorize (identity/evidence/actuators/sign/ack)
   OnlineWrite
        ↓
   RuntimeGovernor.write_online     (SIM/HIL: write_driver)
        ↓  ledger + envelope + e-stop
   plant.act            (SIM today)
```

That choke point is the wedge. See [docs/COMPETITOR_WEDGE.md](docs/COMPETITOR_WEDGE.md).

## Workspace (grouped by domain, not a file dump)

| Crate | Owns | Must not own |
|-------|------|----------------|
| `realityos-kernel` | Verdicts, honesty, typed `Violation`/`Layer` | Plants, ROS, invent |
| `realityos-physics` | First-principles SI (stop, energy, motor, Nyquist) | MEASURED dyno |
| `realityos-data` | Event store, query, snapshots | Actuation |
| `realityos-plant` | `Plant`, backed plant, harness, refuse egress | Task planning |
| `realityos-governor` | Identity, e-stop, envelope, `write_driver` | `decide()`, Foundry |
| `realityos-core` | `decide` + domain plugins | `plant.act` |
| `realityos-session` | Session + `HardwareControlBridge` | Invent modules |
| `realityos-ros2` | Codecs, veto topics, connection map | Veto *logic* |
| `realityos-vport` | Exclusive virtual serial `HardwareDriverPort` | Plant, metal |
| `realityos-hil` | Two-process HIL + proof campaign | Kernel APIs |
| `ros-governor` | CLI (`chain`, `debug`) | Metal / ONLINE motion |

Physics formula domains are **plugins**. EtherCAT/metal stay **named holes**.

## Build

```
cargo test --workspace
cargo run -p ros-governor -- status
cargo run -p ros-governor -- decide --verb hold
cargo run -p ros-governor -- dispatch --verb hold
cargo run -p ros-governor -- chain
cargo run -p ros-governor -- debug
cargo run -p ros-governor -- gauntlet
cargo run -p ros-governor -- rates
cargo run -p ros-governor -- propose --prompt "pick and place"
```

Robot connection (holes named): `docs/ROBOT_CONNECTION.md`. Formulas: `docs/FIRST_PRINCIPLES.md`.

## Honesty

| Claim | Status |
|-------|--------|
| SIM last-gate uniqueness (software fence) | yes — sealed write token; foreign `Plant` impls and public guard entry compile-fail |
| ONLINE capability bound to one runtime instance | yes — `runtime_instance_hash` in the HMAC; cross-instance writes refuse |
| Process-separated HIL + exclusive virtual endpoint | yes as tested — ordinary same-UID open/flock/write without chmod; not vs root |
| ONLINE metal / MEASURED PFL | **no** |
| ISO 13850 / 10218 / 26262 / SIL | **no** — analogs only |
| Independent hardware e-stop | **no** |
| Invent any machine | **no** |
| VLA last-write | **no** |

Living Python doctrine: `docs/GOVERNOR_CAPABILITY_LEDGER.md` and `CLAIM_LEDGER.md` in TheWorld.

## Port map (Python → crate)

| Python | Rust |
|--------|------|
| `theworld.core.decision` | `realityos-kernel::DecisionStatus` |
| `kernel.contracts.primitives.HonestyStamp` | `realityos-kernel::HonestyStamp` |
| `governor.models.input.session_identity` | `realityos-governor::RuntimeIdentity` |
| `control_plane.runtime.runtime_governor` | `realityos-governor::RuntimeGovernor` |
| `control_plane.runtime.write_guard` | `realityos-plant::write_guard` |
| `adapters.plant_interface.CommandLedger` | `realityos-plant::CommandLedger` |
| `reality_os.models.output.certified_command` | `realityos-core::CertifiedCommand` |
| `sdk.reality_layer.RealityOS.decide` | `realityos-core::RealityOs::decide` |
| `control_plane.runtime.runtime_session` | `realityos-session::RuntimeSession` |
| `ros2/governor_node.py` | adapter only (`realityos-ros2`); **not** product write path |
