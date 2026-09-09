# Claim ledger — reality-os-rust

SIM software. Never metal. Never invent.

| Claim | Status | Evidence |
|-------|--------|----------|
| Five-word verdicts `allow/modify/probe/refuse/abort`; probe is not abort | ACTIVE | `crates/kernel` `test_probe_is_not_abort` |
| HonestyStamp cannot construct metal / MEASURED / invent_authority | ACTIVE | `HonestyStamp::sim` + serde skip on claim fields |
| ONLINE plant `act` refuses outside certified write scope | ACTIVE | `crates/plant` `online_act_without_scope_refuses` |
| `execute_certified_command` is the only sanctioned plant write | ACTIVE | `crates/plant/src/execute.rs` |
| Replay / expiry fail closed before `act` | ACTIVE | `CommandLedger` tests |
| Governor cannot upgrade REFUSE→ALLOW or widen action | ACTIVE | `crates/reality-os` narrow tests |
| `complete_online` refuses `SIM_*` identity | ACTIVE | `crates/governor` identity tests |
| `start_online(..., require_*=false)` → `online_refuses_safety_rail_opt_out` | ACTIVE | `crates/session` |
| `RuntimeGovernor<OnlineLocked>` has no `config_mut` / `plant_mut` / `envelope_mut` / `ledger_mut` | ACTIVE | `crates/governor/tests/ui` compile-fail |
| ONLINE default is a durable journal+seal, not an in-memory ledger | ACTIVE | `CommandLedger::with_online_journal`; `start_online` |
| Journal missing/deleted/rollback/replaced fail closed | ACTIVE | `crates/plant` ledger tests |
| Prepare/unknown/consume are not retryable | ACTIVE | `ConsumePhase`; `docs/models/consume_write.tla` |
| Safe-state HOLD/FREEZE/FAULT blocks dispatch in SIM too | ACTIVE | `crates/session` `hold_blocks_all_modes` |
| ProposalClass cannot certify/ack/sign/execute; source strings are diagnostic | ACTIVE | `crates/reality-os` `authority.rs` |
| `HardwareDriverPort` is vendor-implementable; `Plant` stays sealed | ACTIVE | `crates/plant` `external_port_wraps_but_cannot_skip_certified_write` |
| See-before-act: gifted pose without pixels cannot ALLOW place | ACTIVE | `gifted_pose_without_pixels_refuses_place` |
| Bounded-trust modes: fastpath / box project / passive | ACTIVE | `crates/reality-os` bounded_trust tests |
| PFL table is a SIM screen, not ISO 10218 / TS 15066 certified | ACTIVE | `domains/pfl.rs` |
| ROS2 node is adapter; hardware writes default off | ACTIVE | `crates/ros2` |
| Attestation chain ≠ driver consume journal | ACTIVE | two types: `CertificateLedger` vs `CommandLedger` |
| First-principles stop/energy/motor screens (SIM formulas) | ACTIVE | `crates/physics` + domain plugins |
| Typed `Violation`/`Layer` + event store | ACTIVE | `crates/kernel`, `crates/data` |
| HardwareDriverPort → BackedPlant → Governor | ACTIVE | `crates/plant` harness; metal clamped false |
| Robot-agnostic bodies (H1 19-DoF from xml ranges, arm6, wheeled) | ACTIVE | `robots/*.json` + `crates/embodiment` |
| Env-agnostic g/μ (earth/moon/ice/high_g) | ACTIVE | `Environment::catalog` |
| Pixel see-before-act (pinhole, no gifted pose) | ACTIVE | `crates/vision` |
| Multi-Hz bands 30–1000 + deadline miss | ACTIVE | `crates/rate` SIM budgets, not PREEMPT_RT |
| Grok/LLM propose-only + forbid metal keys | ACTIVE | `crates/agent`; live `XAI_API_KEY` + feature `live-grok` |
| Governor gauntlet 160 + Reality OS 131 | ACTIVE | `crates/gauntlet`; CLI `gauntlet` |
| Fieldbus / metal robot | **NAMED_HOLE** | `FieldbusLink::named_hole`; `docs/ROBOT_CONNECTION.md` |
| ONLINE metal / MEASURED / ISO PL/SIL | **NOT_EVIDENCE** | type system + this ledger |
