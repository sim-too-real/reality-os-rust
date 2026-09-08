# Architecture

Layered so a crate can change internals without breaking the write uniqueness invariant.

```
                    ┌─────────────────────────────┐
                    │  apps/ros-governor (CLI)    │
                    │  realityos-ros2 (adapter)   │
                    └─────────────┬───────────────┘
                                  │
                    ┌─────────────▼───────────────┐
                    │  realityos-session          │
                    │  bind_and_dispatch          │
                    │  sealed packages            │
                    └──────┬──────────────┬───────┘
           CertifiedCommand│              │ identity ack
              ┌────────────▼──┐    ┌──────▼──────────┐
              │ realityos-core│    │ realityos-governor│
              │ decide/certify│    │ write_driver      │
              └──────┬────────┘    └──────┬───────────┘
                     │                    │ ActuationCommand
                     └────────┬───────────┘
                              ▼
                      realityos-plant
                      ledger + write_guard + Plant
                              │
                      realityos-kernel
                      Decision / Honesty / IDs
```

## SOLID mapping

- **S** — `RealityOs` certifies; `RuntimeGovernor` gates; `Plant` writes; ROS 2 only transports.
- **O** — new machine classes register `DomainPlugin`; they do not edit `decide()`.
- **L** — any `Plant` (SIM stub, later hardware port) is substitutable; ONLINE still requires certified scope.
- **I** — `ActuationCommand` is the driver-boundary trait. Governor does not import `CertifiedCommand`.
- **D** — `HardwareDriverPort` is for robots. Governor depends on `Plant`, never on a vendor SDK.

## Control loop (ported)

1. Intent (language / VLA caption). Not torques.
2. Forbidden tool names refuse.
3. Learned sources screened (`executable=false`).
4. See-before-act when the verb needs a scene.
5. Domain `plan` + `certify`. Router miss → refuse, never silent slide-wedge.
6. Non-finite action → abort.
7. ALLOW/MODIFY issues `CertifiedCommand` (`acknowledged=false`).
8. Session binds empty identity fields only (foreign hashes refuse).
9. Session stamps `acknowledged=true` after preflight.
10. Governor `pre_actuation_check` + envelope + ledger + `execute_certified_command`.
11. ONLINE `Plant::act` refuses unless the certified-write guard is entered.

## Two journals (do not collapse)

| Chain | Type | Consumes command ids? |
|-------|------|------------------------|
| Driver | `CommandLedger` | yes (`kind=consume`) |
| Decision audit | `CertificateLedger` | no |

Governor events (`estop`, `driver_write_refused`) share the **driver** file so e-stop survives restart. They do not mark a command spent.

## What we did not port

- Foundry invent / CAD / shop
- `PhysicsGovernor` slide loop
- 100+ closed-form `plan_*` / `certify_*` formulas (stay Python SIM)
- MuJoCo studio / ONNX policy last-write
- `ros2/governor_node.py` as production write path (legacy; hardware writes disabled)
