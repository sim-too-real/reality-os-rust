# Authority kernel — reconstruction, claims, and target architecture

Work is against `main` at the rewrite point plus this hardening. This document
states only what source and deployment topology actually enforce.

## 1. Current authority graph (from source)

```text
untrusted proposer
  Intent / PolicyProposal (ProposalClass is typed; source string is diagnostic)
        │
        ▼
RealityOs::decide
  forbidden-tool refuse
  see-before-act screen (manip verbs)
  DomainPlugin::plan + certify     ← semantic TCB (physics screens)
  Certificate + CertifiedCommand::issue   (acknowledged=false)
        │
        ▼
RuntimeSession::bind_and_dispatch
  SafeState latch
  bind_identity (empty fields only; foreign hashes refuse)
  ONLINE: bind actuator allow-list, bind_evidence, sign AFTER bind, acknowledge
  SIM: optional sensor fill, acknowledge
        │
        ▼
RuntimeGovernor<R>::write_driver
  pre_actuation_check (identity, heartbeat, sensor, estop, ONLINE time rollback)
  envelope.check_action
  ExecuteBind (OnlineLocked sets force_online_rails)
        │
        ▼
execute_certified_command          ← only with_certified_write entry
  allow+ack, optional/forced signature, ledger.check, ledger.prepare
  plant.act / follow_waypoints
  ledger.ack  or  ledger.mark_unknown
        │
        ▼
Plant (sealed) → HardwareBackedPlant → HardwareDriverPort::write_action
```

Two journals: driver `CommandLedger` (consumes ids) and `CertificateLedger`
(does not). Governor events share the driver file.

## 2. Remaining bypass / weaken paths (ranked)

### P0 — closed or structurally reduced in this change

| Path | Before | After |
|------|--------|-------|
| `config_mut` disables ONLINE rails after start | Public on all governors | Only `UnlockedRail` (SIM/HIL). Compile-fail on `OnlineLocked`. |
| Public `identity` swap | Field was `pub` | Read-only `identity()`. |
| `envelope_mut` / `set_envelope` widen ONLINE | Public | Unlocked rails only. ONLINE envelope set at construction. |
| `plant_mut` / `ledger_mut` | Public | Unlocked rails only. |
| `set_signing_key(None)` | Public | Unlocked rails only. ONLINE key set at `new_online`. |
| Silent in-memory ONLINE ledger | `CommandLedger::new()` default | `start_online` requires a journal; `first_online=false` refuses missing pair. |
| Journal deletion = empty replay | Missing file → genesis | Seal present + journal absent → `JournalDeleted`. Both missing + not first → `JournalMissing`. |
| Journal truncation / rollback | Prefix reload as truth | Seal event_count ahead → `JournalRollback`. |
| Journal replacement | New valid chain accepted | Prefix hash ≠ seal → `JournalReplaced`. |
| `execute` opt-out on production plants | Bind flags honored | `production_locked` or `force_online_rails` ignore weakening. Key hash must match plant lock. |
| Sign-then-rebind | Silent stale HMAC | Bind after sign errors; mutations clear signature; ONLINE re-signs after bind. |
| Deserialize `acknowledged=true` | Serde restored ack | `skip_deserializing` on `acknowledged`. |
| String “learned” as security | Dead `learned_actuator_authority` branch (always false) | Typed `ProposalClass`. No class can mint/ack/sign/execute. Dead branch removed. |
| `HardwareDriverPort` sealed (no vendors) / `Plant` implementable | Port sealed | Port unsealed. `Plant` remains sealed. Bounds checked at egress. |
| Waypoint shortcut on ONLINE | `follow_waypoints` skipped envelope | ONLINE rails refuse waypoints. |
| Caller `now_s` rollback on ONLINE | Unchecked | `time_rollback` refuse + abort latch. |
| Journal `t_s` f64 not JSON-stable | Hash mismatch on reload under fail_closed | Integer `t_ms` in records. |

### P1 — still open (honest)

| Path | Why it remains |
|------|----------------|
| Sibling crate calls `CertifiedCommand::issue` | Certifier API is in-process and public. Feature `fixtures` marks the test mint. Rust cannot stop a dependent crate from constructing an ALLOW command. ONLINE still requires session sign + rails. |
| Sibling calls `execute_certified_command` on an *unlocked* plant | SIM/harness plants are not production-locked. That is the fixture write path. |
| Sibling with `&mut RuntimeGovernor<OnlineLocked>` calls `write_driver` with a homemade command | Must pass session signature (key never exported as replaceable). Semantic certify is not re-checked as a capability token. |
| Dual-delete of journal **and** seal + `first_online=true` | Looks like first boot. Operator flag is privileged, not proposer-facing. |
| Same-filesystem seal | Tamper-evident vs the seal, not authenticated, not ransomware-resistant. |
| `now_s` far-future / clock-domain mix | Rollback is checked; there is no trusted timestamping. |
| Observation digest is recomputed at ingest, but samples still come from the caller | Contract: perception stack must be the acquirer. No perception implementation here. |
| Thread-local certified-write counter | Process/thread uniqueness, not machine-wide. Reentrant `act` during the guard can write. |
| `ActuationCommand` is unsealed | DIP for governor/plant. Homemade impls exist for tests. ONLINE rails + production key bind the production path. |

### P2

| Path | Notes |
|------|-------|
| Hash-chain canonicalization | `serde_json::Map` is sorted; still not a second preimage-resistant design. |
| `CommandLedger::consume` without a write | Marks spent (fail-closed). No `ledger_mut` on ONLINE. |
| `apply_journal_continuity(operator_ack=true)` skips identity mismatch | Operator privilege. |
| Certificate public fields | Status can be constructed; production path is decide + session sign. |
| Sequence integers chosen by caller | ONLINE monotonic vs ledger; not a global machine counter. |

## 3. Exact claims the current code can honestly make

1. **Rust API uniqueness of `Plant`:** only this crate can `impl Plant` (sealed trait). Compile-fail test.
2. **Process-level certified-write uniqueness:** ONLINE/`caps.online` plants refuse `act` unless the crate-private TLS guard is entered from `execute_certified_command`.
3. **ONLINE typestate lock:** `RuntimeGovernor<OnlineLocked>` does not provide `config_mut`, `envelope_mut`, `plant_mut`, `ledger_mut`, or `set_signing_key`. Compile-fail tests.
4. **ONLINE start is fail-closed on rail opt-out, SIM identity, missing journal/key/actuators, and `first_online` mismatch.**
5. **Durable consume:** prepare/ack/unknown persist to the journal+seal when constructed with `with_online_journal`. Restart will not retry a prepared/consumed/unknown id.
6. **Narrowing:** governor/kernel modify cannot upgrade REFUSE→ALLOW or widen/reverse the issuer envelope (unit + property tests).
7. **Proposal class cannot execute:** every `ProposalClass` returns `can_execute()==false`. Learned strings are not the boundary.
8. **HonestyStamp** cannot construct metal / MEASURED / invent_authority.
9. **Vendor ports** may implement `HardwareDriverPort`. They cannot implement `Plant` or enter the write guard.

These are **not** machine-wide single-writer, authenticated journals, or ISO PL/SIL.

## 4. Claims the code still cannot make

- Physical machine-level single-writer (root or another process can open the bus).
- Authenticated / ransomware-resistant / anti-rollback against an adversary who replaces journal **and** seal and is allowed `first_online`.
- Trusted time, external anchoring, or firmware-rooted identity.
- That evidence hashes prove a real sensor (they prove the session hashed *some* samples).
- That `CertifiedCommand::issue` is unreachable from an untrusted crate in the same process.
- Independent safety PLC / STO / SS1 / fieldbus.
- MEASURED, metal, or ISO certification.

## 5. Minimal target architecture

Keep one authority kernel: typed proposal → certify → typestate bind/sign/ack → locked ONLINE governor → sealed Plant → limited driver port. Move perception, planning, WBC, ROS, agent, embodiment catalogs, and rate out of that kernel (they already are, conceptually; see `docs/TCB.md`).

## 6. Proposed Rust typestate / capability design (implemented)

**Runtime**

- `Simulation` / `Hil` implement `UnlockedRail` (test hooks remain).
- `OnlineLocked` implements `Rail` only.
- `RuntimeGovernor<P, R = Simulation>` and `RuntimeSession<P, R = Simulation>`.

**Command**

```text
UntrustedProposal (Intent / PolicyProposal)
  → CertifiedIntent          decide()
  → SessionBound             bind_identity
  → EvidenceBound            bind_evidence
  → SignedCommand            sign (after bind)
  → AcknowledgedCommand      session only on the ONLINE path
  → Prepared / Consumed / Unknown   ledger
```

SIM may `acknowledge_sim` without a signature. ONLINE must sign after evidence bind.

**Capabilities**

- Certified-write token: crate-private TLS, not a public type.
- Production signing key: hashed into the plant at `lock_production`; execute checks the hash.
- Session binder is the only ONLINE acknowledger/signer in the composition root.

## 7. Public API changes

- `RuntimeGovernor.identity` is no longer a public field; use `identity()`.
- ONLINE construction: `RuntimeGovernor::new_online`, `RuntimeSession::start_online` (journal, key, `first_online`, actuator ids).
- `RuntimeSession::start` refuses ONLINE/HIL (use typestate constructors).
- `GovernorConfig::online_locked()`.
- `CommandLedger::with_online_journal`.
- `ProposalClass` on `PolicyProposal`; `fixture` feature for test minting.
- `HardwareDriverPort` unsealed; `Plant` still sealed.
- `execute` `ExecuteBind.force_online_rails`.
- `CertifiedCommand` deserialize cannot restore `acknowledged`.
- Removed dead `assert_no_learned_actuator_authority` / never-true learned-authority abort.

Compatibility was not a goal.

## 8. Modules that should leave the trusted core

See `docs/TCB.md`. Do not delete fixtures. Vision, embodiment, agent, ros2, rate, data, gauntlet, trajectory/control remain useful and untrusted for write uniqueness.

## 9. Command lifecycle state machine

See `crates/reality-os/src/lifecycle.rs` and §6. Invalid order (sign then bind, ack via serde, ONLINE without evidence) is refused.

## 10. Ledger crash / restart state machine

See `docs/models/consume_write.tla` and `crates/plant/src/consume.rs`.

| Crash point | Persisted | Restart |
|-------------|-----------|---------|
| Before prepare | unseen | May retry (no physical write) |
| After prepare, before write | prepared | Must **not** retry (id spent; plant untouched) |
| After write, before ack | prepared (or unknown if marked) | Must **not** retry (outcome may have occurred) |
| After ack | consumed | Must **not** retry |

Missing journal (not first): refuse start. Deleted journal (seal remains): refuse. Truncation: rollback. New chain: replaced. Corrupt/partial line: unreadable, fail-closed. In-memory ledger is SIM-only.

**Trust model for cryptography:** HMAC-SHA256 is a shared-key capability envelope (session/governor holds the key). It is not non-repudiation and not Ed25519. Asymmetric signatures would not fix “who holds the process key.” Journal SHA-256 chain is tamper-**evidence** given a stored tip (the seal), not authenticity against an adversary who writes both files.

Separated: tamper evidence (chain+seal), authenticity (not provided), anti-rollback (seal vs prefix; not WORM), trusted time (not provided), external anchoring (not provided), retention (not provided).

## 11. Provenance model

| Class | Who constructs | May certify | May bind | May sign | May execute |
|-------|----------------|-------------|----------|----------|-------------|
| `UntrustedLearned` | agent / VLA | no | no | no | no |
| `ExternalDeterministic` | external planner | no | no | no | no |
| `Operator` | CLI / language intent | no | no | no | no |
| Certifier | `RealityOs::decide` | yes | no | no | no |
| Session binder | `RuntimeSession` | no | yes | ONLINE yes | no |
| Signer | session after bind | no | no | yes | no |
| Execution authority | `write_driver` → execute | no | no | no | yes |

Source strings (`grok`, `vla`, `policy`) are notes. They are not a security boundary.

## 12. Hardware-driver extension boundary

- `Plant` sealed; certified-write private; `HardwareBackedPlant<P>` is the production wrapper.
- `HardwareDriverPort` is implementable by vendor crates.
- A port cannot certify, acknowledge, widen policy, or mint the write token.
- `write_action` on a port **you own** is transport ownership. Exclusive `/dev` or EtherCAT master is an OS property.
- Dimension and hard action bounds are checked again in `check_hard_action_bounds` immediately before egress.

## 13. ONLINE deployment topology

```text
untrusted autonomy process     (agent, VLA, ROS talker)
        │ messages only
        ▼
semantic execution authority   (this kernel, one process)
        │ certified write only
        ▼
hardware-driver process/lib    (HardwareDriverPort impl)
        │
        ▼
OS permissions                 (cgroup, device node, capabilities)
        │
        ▼
bus ownership                  (one master; not enforced by Rust)
        │
        ▼
safety controller              (NAMED HOLE — independent PLC/STO)
        │
        ▼
drive → physical energy
```

**Machine-wide single-writer** requires: exactly one process may open the actuator bus; udev/ACL/cgroup enforce it; no root/debug tool on the same node during operation; an independent safety controller can remove energy. Rust privacy does not provide this. If root can `open()` the device, the software guarantee is only process-local.

## 14. Verification plan

| Method | Used? | Why |
|--------|-------|-----|
| Compile-fail (trybuild) | yes | `impl Plant`, `with_certified_write`, ONLINE `config_mut`/`plant_mut`/`envelope_mut`/`ledger_mut`/`set_signing_key` |
| proptest | yes | narrowing never widens; non-finite never writes |
| Crash/restart + journal tests | yes | deletion, rollback, prepare not retryable, ONLINE restart |
| TLA+ | yes, one model | consume/write + crash (`docs/models/consume_write.tla`) |
| Kani | no | would re-prove `may_retry` / envelope predicates already covered by tests; no CI harness |
| Loom | no | no lock-free concurrency in the kernel |
| cargo-fuzz | later | useful for serde/journal parsers; not added for appearance |

## 15. Concrete implementation sequence (done / remaining)

1. Reconstruct path from source — done.
2. Typestate rails + freeze ONLINE config — done.
3. Durable journal+seal + `first_online` — done.
4. Sign-after-bind + typed provenance + unseal port — done.
5. Adversarial tests + TLA+ model — done.
6. Remaining P1: capability token on `issue`, separate seal media, trusted time, OS device exclusive open.

## 16. Tests that must pass before any real actuator is connected

- Workspace `cargo test --workspace --all-targets`
- Compile-fail suite (plant + governor)
- `clippy -D warnings`, `fmt --check`
- ONLINE: no rail opt-out; journal required; restart does not replay
- Journal missing/deleted/rollback/replaced fail closed
- Unknown outcome not retryable
- Production lock refuses unsigned / wrong key
- External-style port cannot `impl Plant` and cannot `act` without the guard
- Narrowing / non-finite properties
- Gauntlet matrices still pass (SIM fixtures)

Do **not** connect energy until deployment topology in §13 is true on the machine.

## 17. Judgment: one-robot hardware experiment?

**No.** The architecture is now a *small, auditable authority kernel* whose process-level guarantees can be stated precisely and tested adversarially. It is **not** strong enough to justify energizing a real actuator:

- Machine-wide single-writer is unproven.
- Fieldbus / safety PLC are named holes.
- Journal+seal is not authenticated storage.
- Evidence is still caller-supplied samples plus a recomputed hash.
- Semantic certification is still in-process and sibling-callable.

Use the kernel as the last software gate in SIM/HIL, with an independent hardware safety channel, exclusive bus ownership, and a stored seal on media the autonomy process cannot rewrite, before any one-robot powered experiment.
