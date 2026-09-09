# Authority kernel — reconstruction, claims, and ONLINE capability seal

Work is against `main` after the typestate/journal pass, plus this capability-boundary
pass. This document states only what source and deployment topology actually enforce.

## 1. Current ONLINE authority graph (from source)

```text
untrusted request / proposal
    Intent / PolicyProposal
        │
        ▼
RealityOs::decide                         semantic certification
    Certificate + IssuedCommand           (private ctor; issue is pub(crate))
        │
        ▼
RuntimeGovernor<OnlineLocked>::authorize_issued
    session identity bind                 (governor-owned RuntimeIdentity)
    evidence bind                         (governor-owned sensor hash)
    actuator bind                         (governor-owned allow-list)
    HMAC sign                             (governor-private key)
    acknowledge                           (authority transition, not metadata)
        │
        ▼
OnlineWrite                               unforgeable executable capability
        │
        ▼
RuntimeGovernor<OnlineLocked>::write_online
    pre_actuation_check + envelope
    execute_certified_command
        │
        ▼
sealed Plant → HardwareDriverPort
```

SIM/HIL (`UnlockedRail`) keep `write_driver(&dyn ActuationCommand)` and may
acknowledge via `lifecycle::acknowledge_sim` / `fixtures`. That interface is
intentionally looser and is not the ONLINE path.

## 2. Bypass attempts against public APIs (proven from source)

Attacks were reconstructed from the pre-seal surface, then re-checked after the
redesign. “Succeeds” means a same-process dependent crate can reach
`Plant::act` on an `OnlineLocked` governor’s plant without `RealityOs::decide`.

| Attack | Pre-seal | After this pass |
|--------|----------|-----------------|
| `Certificate::new(ALLOW, …)` | Succeeds (public) | Still public. Certificate alone is not `IssuedCommand` / `OnlineWrite`. |
| `CertifiedCommand::issue` | **Succeeded (P0)** | Fails: `pub(crate)`. Compile-fail. |
| Deserialize `CertifiedCommand` | Succeeds as a struct | Still possible. Cannot wrap as `IssuedCommand`. Serde cannot set `acknowledged`. |
| Deserialize `IssuedCommand` | n/a | No `Deserialize`. |
| `bind_identity` / `with_*` / evidence / actuator ids | **Succeeded (P0)** as prep | `bind_*` are `pub(crate)`. `with_*` still shape a `CertifiedCommand` you already hold. ONLINE authorize ignores caller actuator lists and uses governor-owned ids + hash. |
| Read `signing_key()` | **Succeeded (P0)** | Fails: method removed. Compile-fail. |
| `CertifiedCommand::sign` | **Succeeded (P0)** given the key | Fails: `pub(crate)`. |
| `CertifiedCommand::acknowledge` | **Succeeded (P0)** | Fails: `pub(crate)`. Compile-fail. |
| `write_driver(&dyn ActuationCommand)` on `OnlineLocked` | **Succeeded (P0)** | Fails: `UnlockedRail` only. Compile-fail. |
| Homemade `impl ActuationCommand` | **Succeeded (P0)** via `write_driver` | Trait stays unsealed (DIP). ONLINE `write_online` takes `&OnlineWrite` only. Compile-fail. |
| Construct `OnlineWrite { … }` | n/a | Fails: private field. Compile-fail. |
| Construct `IssuedCommand { … }` | n/a | Fails: private field. Compile-fail. |
| Bypass `RuntimeSession` and call the governor | **Succeeded (P0)** | `authorize_issued` + `write_online` is the sanctioned governor path. It still requires an `IssuedCommand` from `decide` plus governor-owned key/evidence/actuators. Session HOLD is mirrored on the governor (`tighten` only). |
| `execute_certified_command` on the ONLINE plant | Needs `&mut Plant` | `plant_mut` is `UnlockedRail` only. Sibling cannot get the locked plant. |
| `fixtures` feature mint | Intentional SIM | Still the test mint. Production dependents must not enable `fixtures`. |

### Ranked findings

**P0 — closed in this pass**

- Public `issue` / `sign` / `acknowledge` minting a command indistinguishable from a kernel-issued, executable ONLINE write.
- Public export of the ONLINE HMAC secret.
- `RuntimeGovernor<OnlineLocked>::write_driver` accepting an arbitrary `ActuationCommand`.
- Homemade `ActuationCommand` satisfying ONLINE execution.
- Public tuple-struct wrapping (`AcknowledgedCommand(cmd)`) as a substitute for `OnlineWrite`.
- ONLINE safe-state unlock back to `Running`.

**P1 — still open (honest)**

- Sibling calls `execute_certified_command` on an *unlocked* SIM plant (fixture write path).
- Dual-delete of journal **and** seal + `first_online=true` looks like first boot.
- Same-filesystem seal is tamper-evident, not authenticated.
- `record_sensor` still hashes caller-supplied samples.
- Thread-local certified-write counter is process/thread uniqueness, not machine-wide.
- Caller who *retained a copy* of the `Vec<u8>` they passed into `start_online` still has the secret. The governor no longer hands it back.
- `CertifiedCommand::seal_online` is public and will HMAC+ack if the caller already has a key. That value is still not `OnlineWrite`; `write_online` verifies against the governor key.

**P2**

- Hash-chain canonicalization is not second-preimage design.
- `Certificate` public fields (status can be constructed; it cannot become `IssuedCommand`).
- Sequence integers chosen by `DecideRequest` (ONLINE monotonic vs ledger only).

## 3. Final authority-capability architecture

```text
untrusted request/proposal
  → semantic certification          RealityOs::decide → IssuedCommand
  → session identity binding        authorize_issued (governor identity)
  → evidence binding                authorize_issued (governor sensor hash)
  → actuator binding                authorize_issued (governor allow-list)
  → signature                       governor-private HMAC key
  → acknowledgement                 same transition; not a public method
  → executable authority            OnlineWrite
  → ONLINE Governor                 write_online(&OnlineWrite)
  → execute                         execute_certified_command
  → sealed Plant
```

`RuntimeGovernor<OnlineLocked>` does **not** treat a public `ActuationCommand` as
sufficient authority.

## 4. Signing-key ownership

- The raw ONLINE key is stored only on `RuntimeGovernor`.
- There is no getter. `set_signing_key` remains `UnlockedRail` only.
- The governor needs the key to (1) seal in `authorize_issued` and (2) verify in
  `execute_certified_command`.
- Session no longer reads the key. It calls `authorize_issued`.
- HMAC-SHA256 is a shared-key capability envelope, not a hardware root of trust
  and not non-repudiation.

## 5. Command / certificate construction

| API | Ordinary consumer | Notes |
|-----|-------------------|--------|
| `Certificate::new` | public | Domain plugins. Not `IssuedCommand`. |
| `CertifiedCommand::issue` | `pub(crate)` | Kernel + fixtures. |
| `sign` / `acknowledge` / `bind_*` | `pub(crate)` | Lifecycle + authorize. |
| `with_*` shaping | public | Invalidates signature. Cannot mint `IssuedCommand`. |
| `IssuedCommand` | decide only | Private constructor. No `Deserialize`. |
| `seal_online` | public | Needs a key; result is not `OnlineWrite`. |
| `fixture::issue` / `fixture::acknowledge` | `fixtures` / tests | SIM/gauntlet. |
| `narrow_certified_command` | public | Cannot upgrade refuse; returns `CertifiedCommand`. |

Acknowledgement **is** an authority transition on the ONLINE path: it happens
only inside `authorize_issued` while producing `OnlineWrite`. Serde cannot
restore it (`skip_deserializing`).

## 6. Governor ONLINE interface

Public on `RuntimeGovernor<P, OnlineLocked>` (authority-relevant):

- `new_online(..., signing_key, first_online, actuator_ids, clock)`
  (probes `Plant::probe_identity`, exact-match vs expected, then hashes)
- `authorize_issued(IssuedCommand) -> Result<OnlineWrite, _>`
  (instance digest from `ValidatedRuntimeIdentity` only)
- `write_online_now` (authority clock). Caller-time `write_online` is crate-private.
- `heartbeat_now` / `watchdog_tick_now` / `engage_estop_now` /
  `clear_estop_requires_recovery_now` / `latch_abort_now`
- `latch_safe_state` — tighten only
- Observation / recovery: `plant`, `ledger`, `config`, `identity`,
  `ingest_sensor_packet` / `acquire_sensor`, traces, `safe_state`

Not public on ONLINE: `write_driver`, caller-time `heartbeat` / `watchdog_tick` /
`record_sensor` / `engage_estop` / `write_online`, `signing_key`, `config_mut`,
`plant_mut`, `envelope_mut`, `ledger_mut`, `set_signing_key`, `set_envelope`,
`mark_sensor`. Those remain on unlocked / test / HIL rails.

`ActuationCommand` stays unsealed as a dependency-inversion shape.
ONLINE execution consumes `OnlineWrite` (authority), not the trait (shape).

## 7. Public API changes (this pass)

- `KernelDecision.command: Option<IssuedCommand>`
- `IssuedCommand`, `OnlineWrite`
- Removed `RuntimeGovernor::signing_key()`
- `write_driver` moved to `UnlockedRail`
- `new_online` requires `actuator_ids`
- `CertifiedCommand::{issue,sign,acknowledge,bind_*}` are crate-private
- Lifecycle tuple fields are private
- SIM `bind_and_dispatch` still takes `CertifiedCommand`
- ONLINE `dispatch_issued` takes `IssuedCommand`
- `HardwareControlBridge::dispatch` takes `IssuedCommand`
- Minimal GitHub Actions workflow (fmt / clippy / test)

Compatibility was not a goal. SIM/HIL/gauntlet fixture paths remain.

## 8. Compile-fail / adversarial tests

1. Cannot read the ONLINE signing key — `online_no_signing_key.rs`
2. Cannot produce `OnlineWrite` / `IssuedCommand` — private-ctor UI tests
3. Cannot `acknowledge()` into ONLINE executability — `command_acknowledge_is_private.rs`
4. Cannot `write_driver` / `write_online(&dyn ActuationCommand)` on ONLINE
5. Cannot `Certificate(ALLOW) + issue + sign` bypass `decide` — `cannot_bypass_decide.rs`
6. SIM/HIL/gauntlet fixtures still run
7. Valid ONLINE path is exactly one actuation — governor + session tests
8. Restart/replay still holds — `online_dispatch_signs_after_bind_and_survives_restart`

## 9. Claims now justified (do not promote)

See `CLAIM_LEDGER.md` for the level of each claim.

This pass justifies, at **Rust type/API** and **same-process** level only:

- No public same-process sequence fabricates semantic authorization and reaches
  ONLINE actuation on `RuntimeGovernor<OnlineLocked>`.
- ONLINE signing secret is not readable back from the governor.
- ONLINE execution requires an `OnlineWrite` produced from an `IssuedCommand`.

## 10. Remaining gaps

Not provided by this patch (and not claimed):

- Exclusive bus ownership (machine-wide)
- Process isolation against root or same-UID `chmod` / `/proc/<pid>/fd`
- Hardware root of trust / TPM / HSM
- Authenticated journal storage, anti-rollback secure storage, or WORM
- Independent safety / STO / SS1 / PL / SIL / ISO 10218
- Metal / MEASURED validation
- Perception authenticity (HIL samples may be synthetic; receive time is authority-owned)
- Branch-protection / merge policy (see `docs/BRANCH_PROTECTION.md`)

## 11. CI / repository integrity

`.github/workflows/authority.yml` runs:

```text
cargo fmt --all -- --check
cargo clippy --workspace --all-targets -- -D warnings
cargo test --workspace --all-targets
```

That is required CI **if** the workflow is enabled on the default branch. It does
not enforce merge policy. Requiring this check on `main`, forbidding skip, and
restricting who can dismiss it are **operational** GitHub settings, not repository
code.

## 12. Another Rust architecture pass?

**No.** Do not propose another speculative Rust hardening pass.

The next milestone is one real low-energy actuator + real `HardwareDriverPort`
+ separate autonomy/authority OS identities + exclusive device ownership +
independent physical power cutoff + the same adversarial campaign.

The software is **not** enough to put energy on a robot.

## 13. Modules that should leave the trusted core

See `docs/TCB.md`. Do not delete fixtures.

## 14. Ledger crash / restart

See `docs/models/consume_write.tla` and `crates/plant/src/consume.rs`.

| Crash point | Persisted | Restart |
|-------------|-----------|---------|
| Before prepare | unseen | May retry (no physical write) |
| After prepare, before write | prepared | Must **not** retry |
| After write, before ack | prepared (or unknown) | Must **not** retry |
| After ack | consumed | Must **not** retry |

**Trust model for cryptography:** HMAC-SHA256 is a shared-key capability envelope
(the governor holds the key). It is not non-repudiation and not Ed25519.
Journal SHA-256 chain is tamper-**evidence** given a stored tip (the seal), not
authenticity against an adversary who writes both files.

## 15. Provenance model

| Class | Who constructs | May certify | May bind | May sign | May execute |
|-------|----------------|-------------|----------|----------|-------------|
| `UntrustedLearned` | agent / VLA | no | no | no | no |
| `ExternalDeterministic` | external planner | no | no | no | no |
| `Operator` | CLI / language intent | no | no | no | no |
| Certifier | `RealityOs::decide` | yes → `IssuedCommand` | no | no | no |
| ONLINE governor | `authorize_issued` | no | yes | yes | no |
| Execution authority | `OnlineWrite` → `write_online` | no | no | no | yes |

Source strings (`grok`, `vla`, `policy`) are notes. They are not a security boundary.

## 16. ONLINE deployment topology

```text
untrusted autonomy process     (agent, VLA, ROS talker)
        │ messages only
        ▼
semantic execution authority   (this kernel, one process)
        │ OnlineWrite only
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

**Machine-wide single-writer** requires OS exclusivity plus an independent safety
controller. Rust privacy does not provide this.

## 17. Frozen software authority kernel

The following public types/APIs are **frozen**. Change them only for a
demonstrated failed invariant, not speculative features. SIM/fixture surfaces
are outside this freeze.

* `IssuedCommand`
* `OnlineWrite`
* `RuntimeGovernor<OnlineLocked>` (`new_online`, `authorize_issued`, `write_online`)
* certified-write scope (`execute_certified_command`, `with_certified_write`)
* consume ledger (`CommandLedger::with_online_journal`, prepare/ack/unknown)
* signing payload (`command_payload_for_sign` / `realityos.command_signing/1`)
* runtime identity (`RuntimeIdentity` as expected, `ValidatedRuntimeIdentity` after
  probe match, `instance_hash`, `realityos.runtime_instance/2` length-prefixed)

## 18. Identity-binding audit (from signed bytes)

Traced values, not field names:

| Runtime concept | Bound into signed payload? | How |
|-----------------|----------------------------|-----|
| release | yes | `release_hash` |
| design | yes | copied into `as_built_hash` (name is historical) |
| serial / as-built | **yes after this pass** | `runtime_instance_hash` |
| firmware | **yes after this pass** | `runtime_instance_hash` |
| calibration | yes | `calibration_ids` + digest |
| authorized actuators | yes | `actuator_ids` + digest |

ONLINE startup path:

```text
StartArgs / ExpectedRuntimeIdentity
        │
        ▼
RuntimeGovernor::new_online
        │
        ▼
Plant::probe_identity  (HardwareBackedPlant → HardwareDriverPort)
        │
        ▼
exact match (design, serial, firmware, calibration, connected,
             non-placeholder, actuator topology if reported)
        │
        ▼
ValidatedRuntimeIdentity
        │
        ▼
instance_hash (length-prefixed realityos.runtime_instance/2)
        │
        ▼
OnlineWrite
```

Configured values are never overwritten by the probe. Mismatch fails closed.

`instance_hash` = SHA-256 of length-prefixed little-endian fields:
schema, release, design, serial, firmware, calibration, sorted unique actuators.

`write_online` re-probes. If the physical identity changes, or the device
disconnects, this runtime instance FAULT/ABORTs: zero further writes.
`clear_estop_requires_recovery` cannot resurrect it. Recovery is a complete
ONLINE restart (`new_online`). Reconnect to the same or a different device
under the previously authorized instance is refused.

`write_online` independently checks digest equality and actuator-scope subset.

A capability from Governor A is not transferable to Governor B merely because
they share a release, design, or signing key.

## 19. Authority clock and sensor freshness

* `AuthorityClock::monotonic_now` is the issue / expiry / heartbeat / watchdog /
  write-time / freshness anchor. Production uses `OsMonotonicClock` (OS
  monotonic, not Unix wall time). Tests/HIL inject `FakeClock`.
* `unix_now_s` is audit / CLI wall time only.
* `RuntimeSession::start_online(args, plant)` uses `OsMonotonicClock`. Tests
  inject time only via `start_online_with_clock`.
* Production IPC (`ProductionProposal`) may carry verb, action, command_id,
  proposer, optional intent metadata. It cannot set `now_s`, `write_now_s`, or
  safety TTL. Those exist only on `HilFaultInjectionRequest` (HIL serve, not
  `--production`).
* Sensor trust model:
  * `device_capture_time` (`SensorPacket.timestamp_s`) — informative / validated
    when synchronized later. Not a freshness anchor.
  * `authority_receive_monotonic` — stamped internally on ingest; freshness =
    `now_monotonic - last_sensor_s`.
  * PTP / hardware timestamping is not implemented.

## 20. Judgment: powered physical testing?

**No.** See `docs/HIL.md`. Process separation and exclusive virtual I/O are
demonstrated. Independent physical energy-stop is still a named hole. Do not
energize a real actuator. This pass earns the right to plan the first
controlled physical experiment; it does not authorize one.
