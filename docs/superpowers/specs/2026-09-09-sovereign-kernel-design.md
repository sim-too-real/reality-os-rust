# Sovereign kernel design

Date: 2026-09-09  
Repo: `reality-os-rust`  
Binding: Approach B — sovereign Rust kernel, replaceable backends, in-place rewrite.

This spec is the durable record of the rewrite. It does not claim ISO/SIL, metal readiness, or an independent safety MCU/PLC. SIM ≠ metal.

## Authority path

One path exists from proposal to plant:

1. An untrusted proposer emits `GoalIR` / `SkillIR` / `PolicyProposal`.
2. `RealityOs::decide` issues an immutable `CertifiedCommand` or refuses.
3. `RuntimeSession` binds identity and acknowledges.
4. `RuntimeGovernor::write_driver` is the only admission into the plant.
5. `execute_certified_command` is the only public write. `with_certified_write` stays crate-private. `Plant` and `HardwareDriverPort` are sealed.

No parallel command type. Narrowing yields a derived command with `parent_payload_hash`. Sign reversal is widening. Unknown verbs refuse. Empty or unqualified plans do not synthesize `0.5 * tau_max`.

## Honesty rules

- `WorldView` observation is `Option<ObservationEvidence>`, not caller booleans.
- Ledger `prepare` → write → `ack`. Write-then-error is `CommandOutcome::Unknown`. No auto-resend.
- Journal load recomputes every `prev_hash` / `chain_hash`. A missing file is empty genesis. Tamper is unreadable.
- Session start may attach a journal. Continuity restores e-stop / abort. Replay of a prepared id cannot write twice.
- Software watchdog is a monotonic supervisor, not a hardware island. Clock rollback latches.
- Independent safety MCU/PLC remains a named hole until a product ODD exists.

## Contracts

| Object | Owner crate | Role |
|---|---|---|
| Units, `MonoTime` / `SimTime` / `SyncTime`, `Capability`, `BeliefState`, adapter handshake | `kernel` | Honesty floor. No plant I/O. |
| `EmbodimentGraph`, URDF/MJCF import, world belief | `embodiment` | Body and world data. Diagnostics for dropped features. |
| `CertifiedCommand`, `SkillIR` catalog, `decide` | `reality-os` | Issue only. |
| `RuntimeGovernor` | `governor` | Admit only. Private plant/envelope/key. |
| `Plant`, ledger, fieldbus hole | `plant` | Sole write + journal. |
| `RuntimeSession` | `session` | Bind, ingest, dispatch, journal attach. |
| `SkillIR` admission + evidence memory | `agent` | Proposal only. Prose is not motion. |
| Serial FK/RNEA ABI | `physics` | Screens + narrow dynamics math. Not safety authority. |
| Bounded mailbox, rate bands | `rate` | Overload → hold. SIM deadlines. |
| `McapWriter`, event store | `data` | Audit. Never on the write path. |
| ROS codecs | `ros2` | Transport adapter. Never last write. |

Capabilities come from morphology `kind` (`biped`, `quadruped`, `serial_arm`, `aerial`, …), never from a product name. H1, arm6, wheeled, quadruped, and aerial are fixtures.

## Backends

- Default plant is `SimPlant`. MuJoCo is a feature-gated `DynamicsBackend` and is unavailable without the feature.
- FK/RNEA is a pure-Rust `RigidBodyBackend`. Pinocchio/Drake are not runtime deps.
- Adapter handshake is versioned: capability, deadline, cancel, deterministic errors.
- MCAP and mailbox sit beside the write path. Overload latches `HOLD` and cannot widen an envelope.

## Skills and memory

`SkillIR::catalog()` is the sole verb admission list. `ALLOWED_VERBS` is deleted. Free text never becomes `allowed_action`. `EvidenceMemory` is digest-keyed and expiring; `recall_prose` is always empty.

## Safety protocol

`SafeTransition` / `SafetyFrame` are the software protocol types. Fieldbus is a named hole. This kernel must be unable to energize metal without an independent island. That island is product work, not a completion criterion of this spec.

## Verification

- `cargo test --workspace --all-targets`
- `cargo clippy --workspace --all-targets -- -D warnings`
- trybuild: foreign crates cannot construct a write token or impl `Plant`
- Kill/restart: replay refused; unrestored e-stop stays latched; tampered `prev_hash` fails closed
- Property coverage: NaN/Inf, dim mismatch, clock rollback, mutation-after-sign, replay, sequence gaps, unknown verbs
- Conformance: same capability queries on every fixture; zero robot-name branches in control/planning APIs
