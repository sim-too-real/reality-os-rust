# Integrity-transition inventory

**Tree:** descendant of `709a0793fcf5e1706b059c27f1ed5a8dba0ca1f1` (landed latch).
**Rule:** production integrity mutations go through `EstopLatch::latch_abort` only. Ordinary ESTOP uses `engage()`. Direct `abort_latched =` exists only inside `latch.rs`.

Search (production Rust, tests excluded from classification): `abort_latched`, `integrity_aborted`, `latch_abort(`.

| Condition | Classification | State-transition API | Recoverability | Test coverage |
|---|---|---|---|---|
| Operator / watchdog / bus ESTOP | `RECOVERABLE_ESTOP` | `EstopLatch::engage` via `engage_estop_at` / `kill_hardware_session` (also sets session-dead) | Recoverable on a live session that is not hardware-session-dead. `engage` does **not** set `integrity_aborted`. | `recover_after_ordinary_estop_clears_and_allows_new_command`; `estop_never_auto_clears`; PTY campaign restart |
| Software watchdog miss | `RECOVERABLE_ESTOP` | `watchdog_tick_at` → `engage_estop_at("software_watchdog_miss")` | Same as ESTOP. Metal `pet_watchdog` may refuse IPC before `recover()` is reached; integrity, if already set, stays. | `successful_watchdog_pets_do_not_journal…`; PTY downgrade composition |
| Unknown physical outcome | `INTEGRITY_ABORT` | `write_driver_inner` → `latch.latch_abort("unknown_outcome")` | Refused: `integrity_abort_requires_online_restart` **before** `Plant::clear_estop` | `recover_after_unknown_outcome_refuses_before_clear_estop` |
| Replay (`replayed command_id`) | `INTEGRITY_ABORT` | `LATCHING_PREFIXES` → `latch.latch_abort` | Refused as above | `recover_after_replay_refuses_and_fresh_id_cannot_execute`; `xl330_pty_untrusted_recover_after_replay_does_not_write` |
| Time rollback | `INTEGRITY_ABORT` | pre-check `time_rollback` → `latch.latch_abort` | Refused as above | `recover_after_time_rollback_integrity_refuses` |
| Release-hash mismatch (pre-check) | `INTEGRITY_ABORT` | pre-check `release_hash` → `latch.latch_abort` | Refused as above | ONLINE write refuse tests (instance mismatch / command hash) |
| `LATCHING_PREFIXES` (missing command_id, command expired, calibration_ids, sensor_packet_hash, missing_allowed_action / non-finite, runtime_instance_mismatch, actuator_scope_not_authorized) | `INTEGRITY_ABORT` (as currently classified) | execute result → `latch.latch_abort` | Refused as above | replay + instance-mismatch write tests. **Expiry is currently integrity** because it is in `LATCHING_PREFIXES`; not reclassified without a live hole. |
| Identity continuity mismatch at start | `STARTUP_REFUSE` | `apply_journal_continuity` → `latch_abort` + `start_refused: true` → `new_online` Err | No live IPC. Do not rewrite. | `new_online_refuses_configured_vs_probed_mismatches` family |
| Repeated identity refuse count | `INTEGRITY_ABORT` if start still succeeds | `apply_journal_continuity` `latch_abort` without `start_refused` | Live instance: recover refused. | continuity / identity tests |
| `journal_unreadable` | `STARTUP_REFUSE` | `engage("journal_unreadable")` + `start_refused` | No live process that accepts recover. **Not converted to integrity.** | ledger unreadable tests |
| Journaled ESTOP restored on `--restart` | `RECOVERABLE_ESTOP` | `state.estop` → `engage` | Must remain recoverable so campaign hold after restart can write | `journaled_estop_after_restart_still_recovers`; `xl330_pty_campaign_restarts_are_live_after_identity_and_disconnect` |
| Identity change / disconnect after ONLINE | `HARDWARE_SESSION_DEAD` | `kill_hardware_session` → `engage` + `hardware_session_dead` | Recover refused `hardware_session_requires_online_restart` **before** integrity check | `identity_change_after_online_faults_and_writes_zero`; PTY identity recover |
| `bus_lost` | `HARDWARE_SESSION_DEAD` | metal `refuse_bus_lost` (does not clear integrity) | Recover refused `bus_lost` / session-dead tokens | PTY disconnect path |
| Ordinary pre-check refuse (heartbeat stale, sensor stale, incomplete identity, ESTOP already engaged) | `ORDINARY_COMMAND_REFUSE` | no `latch_abort` (ESTOP string no longer integrity-latches) | ESTOP recoverable; others not integrity | existing precheck tests |
| Integrity then later ESTOP/watchdog | `INTEGRITY_ABORT` (monotonic) | `engage` cannot clear `integrity_aborted`; recover still refused | Not recoverable | `recover_after_integrity_then_engage_estop_still_refused`; PTY watchdog composition |

## Journal over-correction

Inspected `journal_unreadable`, continuity, identity continuity, restart ESTOP restoration.

No constructed live path where those conditions leave a process that incorrectly **accepts recover and then actuates**. Startup-refused paths never bind IPC. Journaled ESTOP after legitimate `--restart` is required by the campaign. **No journal behavior change in this task.**

## Remaining named holes (not defects of the latch)

- Production `op=recover` is still an untrusted ESTOP-ack analog (`AUTHENTICATED OPERATOR RECOVERY` = `NAMED_HOLE`).
- Integrity abort is not rehydrated across process restart (by design, so campaign ESTOP recover still works).
- Command expiry currently shares the integrity API via `LATCHING_PREFIXES`; documented, not widened.
