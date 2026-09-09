# Claim ledger — reality-os-rust

SIM software. Never metal. Never invent.

Guarantee levels used below:

| Level | Meaning |
|-------|---------|
| **Rust type/API** | Invalid states are unrepresentable or uncallable on the public surface. |
| **Same-process** | Ordinary dependent crates in this process cannot take the path. `unsafe`, process memory, or a `fixtures` feature are out of scope. |
| **Process-isolation** | Another process on the machine cannot take the path. **Not claimed** unless an OS policy is named. |
| **Machine-wide** | Exclusive bus / device node / single writer. **Not claimed.** |
| **Physical safety** | STO/SS1, PL/SIL, independent energy removal. **Not claimed.** |

Never promote one level into another.

| Claim | Level | Status | Evidence |
|-------|-------|--------|----------|
| Five-word verdicts `allow/modify/probe/refuse/abort`; probe is not abort | Rust type/API | ACTIVE | `crates/kernel` `test_probe_is_not_abort` |
| HonestyStamp cannot construct metal / MEASURED / invent_authority | Rust type/API | ACTIVE | `HonestyStamp::sim` + serde skip on claim fields |
| ONLINE plant `act` refuses outside certified write scope | Same-process | ACTIVE | `crates/plant` `online_act_without_scope_refuses` |
| `execute_certified_command` is the only sanctioned plant write | Same-process | ACTIVE | `crates/plant/src/execute.rs` |
| Replay / expiry fail closed before `act` | Same-process | ACTIVE | `CommandLedger` tests |
| Governor cannot upgrade REFUSE→ALLOW or widen action | Rust type/API | ACTIVE | `crates/reality-os` narrow tests |
| `complete_online` refuses `SIM_*` identity | Rust type/API | ACTIVE | `crates/governor` identity tests |
| `start_online(..., require_*=false)` → `online_refuses_safety_rail_opt_out` | Rust type/API | ACTIVE | `crates/session` |
| `RuntimeGovernor<OnlineLocked>` has no `config_mut` / `plant_mut` / `envelope_mut` / `ledger_mut` / `set_signing_key` / `signing_key` / `write_driver` | Rust type/API | ACTIVE | `crates/governor/tests/ui` compile-fail |
| ONLINE write consumes `OnlineWrite` only, not `&dyn ActuationCommand` | Rust type/API | ACTIVE | `write_online`; compile-fail `online_write_rejects_actuation_command` |
| `IssuedCommand` is minted only by `RealityOs::decide` (or `fixtures`) | Rust type/API + same-process | ACTIVE | private ctor; `issue` is `pub(crate)` |
| ONLINE signing key is not publicly readable after `new_online` | Same-process | ACTIVE | no `signing_key()`; compile-fail `online_no_signing_key` |
| Acknowledgement is not a public authority transition into ONLINE executability | Rust type/API | ACTIVE | `acknowledge` is `pub(crate)`; serde skip; `OnlineWrite` private ctor |
| ONLINE default is a durable journal+seal, not an in-memory ledger | Same-process | ACTIVE | `CommandLedger::with_online_journal`; `start_online` |
| Journal missing/deleted/rollback/replaced fail closed | Same-process | ACTIVE | `crates/plant` ledger tests |
| Prepare/unknown/consume are not retryable | Same-process | ACTIVE | `ConsumePhase`; `docs/models/consume_write.tla` |
| Safe-state HOLD/FREEZE/FAULT blocks dispatch in SIM too | Same-process | ACTIVE | `crates/session` `hold_blocks_all_modes` |
| ONLINE safe-state can only tighten | Rust type/API | ACTIVE | `SafeState::tighten`; `online_hold_cannot_return_to_running` |
| ProposalClass cannot certify/ack/sign/execute; source strings are diagnostic | Rust type/API | ACTIVE | `crates/reality-os` `authority.rs` |
| `HardwareDriverPort` is vendor-implementable; `Plant` stays sealed | Rust type/API | ACTIVE | `crates/plant` `external_port_wraps_but_cannot_skip_certified_write` |
| See-before-act: gifted pose without pixels cannot ALLOW place | Same-process | ACTIVE | `gifted_pose_without_pixels_refuses_place` |
| Bounded-trust modes: fastpath / box project / passive | Same-process | ACTIVE | `crates/reality-os` bounded_trust tests |
| PFL table is a SIM screen, not ISO 10218 / TS 15066 certified | — | ACTIVE | `domains/pfl.rs` |
| ROS2 node is adapter; hardware writes default off | Same-process | ACTIVE | `crates/ros2` |
| Attestation chain ≠ driver consume journal | Rust type/API | ACTIVE | two types: `CertificateLedger` vs `CommandLedger` |
| First-principles stop/energy/motor screens (SIM formulas) | Same-process | ACTIVE | `crates/physics` + domain plugins |
| Typed `Violation`/`Layer` + event store | Rust type/API | ACTIVE | `crates/kernel`, `crates/data` |
| HardwareDriverPort → BackedPlant → Governor | Same-process | ACTIVE | `crates/plant` harness; metal clamped false |
| Robot-agnostic bodies (H1 19-DoF from xml ranges, arm6, wheeled) | — | ACTIVE | `robots/*.json` + `crates/embodiment` |
| Env-agnostic g/μ (earth/moon/ice/high_g) | — | ACTIVE | `Environment::catalog` |
| Pixel see-before-act (pinhole, no gifted pose) | Same-process | ACTIVE | `crates/vision` |
| Multi-Hz bands 30–1000 + deadline miss | — | ACTIVE | `crates/rate` SIM budgets, not PREEMPT_RT |
| Grok/LLM propose-only + forbid metal keys | Same-process | ACTIVE | `crates/agent`; live `XAI_API_KEY` + feature `live-grok` |
| Governor gauntlet 160 + Reality OS 131 | — | ACTIVE | `crates/gauntlet`; CLI `gauntlet` |
| Exclusive bus ownership | Machine-wide | **NOT_CLAIMED** | OS/udev; not Rust |
| Process isolation of the authority kernel | Process-isolation | **NOT_CLAIMED** | deployment topology |
| Hardware root of trust / authenticated journal | Machine-wide | **NOT_CLAIMED** | HMAC is a process shared secret |
| Independent safety / STO / SS1 / PL / SIL | Physical safety | **NOT_CLAIMED** | named hole |
| ONLINE `OnlineWrite` is instance-bound (serial/firmware/cal/actuators in digest) | Rust type/API + same-process | ACTIVE | `RuntimeIdentity::instance_hash`; governor cross-instance tests |
| Untrusted HIL process cannot submit authority objects | Process-isolation (HIL IPC) | ACTIVE | `crates/hil` protocol refuse; campaign |
| Exclusive virtual endpoint: second process cannot `flock`/`open` log without chmod | Process-isolation (same-UID, no chmod) | ACTIVE | `try_hostile_open`; `docs/HIL.md` |
| Fieldbus / metal robot | Physical safety | **NAMED_HOLE** | `FieldbusLink::named_hole`; `docs/ROBOT_CONNECTION.md` |
| ONLINE metal / MEASURED / ISO PL/SIL | Physical safety | **NOT_EVIDENCE** | type system + this ledger |
