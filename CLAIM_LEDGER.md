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
| `start(ONLINE, require_*=false)` → `online_refuses_safety_rail_opt_out` | ACTIVE | `crates/session` |
| Safe-state HOLD/FREEZE/FAULT blocks dispatch in SIM too | ACTIVE | `crates/session` `hold_blocks_all_modes` |
| VLA / learned source is proposal-only | ACTIVE | `authority::screen_external_proposal` |
| See-before-act: gifted pose without pixels cannot ALLOW place | ACTIVE | `gifted_pose_without_pixels_refuses_place` |
| Bounded-trust modes: fastpath / box project / passive | ACTIVE | `crates/reality-os` bounded_trust tests |
| PFL table is a SIM screen, not ISO 10218 / TS 15066 certified | ACTIVE | `domains/pfl.rs` |
| ROS2 node is adapter; hardware writes default off | ACTIVE | `crates/ros2` |
| Attestation chain ≠ driver consume journal | ACTIVE | two types: `CertificateLedger` vs `CommandLedger` |
| ONLINE metal / MEASURED / ISO PL/SIL | **NOT_EVIDENCE** | type system + this ledger |
