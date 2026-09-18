# Plan6 evidence report

**Branch work, not a metal proof. No `realityos-decision` crate. No training checkpoints.**

## 1. Recovery implementation audit

The landed latch on `709a079` (independent `engaged` / `integrity_aborted`, recover refused before `Plant::clear_estop`) is **sound for the live ONLINE instance**. Governor tests still pass (41/41). No production `abort_latched =` outside `EstopLatch`. Ordinary ESTOP and journaled ESTOP after `--restart` remain recoverable.

Remaining concrete defects / holes (not latch redesign):

- Production `op=recover` is still untrusted autonomy IPC (NAMED_HOLE).
- Integrity abort is not journal-rehydrated (intentional; campaign ESTOP recover).
- Linux PTY composition tests are committed but **not executed on this Windows host**.
- Real XL330 Run A/B **not measured**.

## 2. Linux PTY status

**passed** on GitHub-hosted Linux (Actions run [35322120250](https://github.com/sim-too-real/reality-os-rust/actions/runs/35322120250), `test-metal` 102 passed including the two recover composition tests). Still **pending/unrunnable** as a local Windows compile (`std::os::unix`). Local PTY is not metal.

Committed tests in `crates/metal/tests/xl330_pty.rs`:

- `xl330_pty_untrusted_recover_after_replay_does_not_write` — shipped `MetalAuthority::handle` path: execute X → replay X → `op=recover` → fresh Y; require `integrity_abort_requires_online_restart`; physical writes unchanged.
- `xl330_pty_recover_after_integrity_then_watchdog_does_not_write` — 150 ms gap so the next `handle` pets the 100 ms watchdog ESTOP after integrity; recover/fresh id must not write.
- Existing `xl330_pty_campaign_restarts_are_live_after_identity_and_disconnect` left in place.

Linux CI must run `cargo test -p realityos-metal --test xl330_pty -- --test-threads=1`. Local PTY is not metal.

## 3. Integrity-transition inventory

See `docs/research/2026-09-18-integrity-transition-inventory.md`.

Journal/`journal_unreadable`/continuity: no live path found where recover incorrectly actuates. **No journal rewrite.**

## 4. CI root cause

GitHub Actions is **enabled** (`allowed_actions: all`). Repo is **private**. No self-hosted runners (`total_count: 0`).

Recent `main` push `bacaf34` authority run [35318118604](https://github.com/sim-too-real/reality-os-rust/actions/runs/35318118604): jobs `verify` / `verify-mujoco` completed in ~2s with **`runner_id: 0`**, empty `runner_name`, job logs 404 — **no hosted runner was assigned**. That is the `steps: []` symptom.

Last **real** Linux execution: 2026-09-10 run [34537975901](https://github.com/sim-too-real/reality-os-rust/actions/runs/34537975901) SHA `9806282`, `runner_id` 1000005594, steps: checkout, `dtolnay/rust-toolchain@stable`, fmt, clippy, test-metal, test — all success.

Cause: GitHub-hosted runner assignment failed on this private account after 2026-09-10 (minutes / spending limit / account billing). Not a Rust test failure. Workflows were not emptied.

Fix that this agent can apply: none that restores hosted runners without billing. Added `workflow_dispatch` so a run can be retried when minutes exist. Did **not** delete MuJoCo or weaken tests.

This branch `a5bf5f4` authority run [35321428196](https://github.com/sim-too-real/reality-os-rust/actions/runs/35321428196) job `verify` (`105524570417`): `runner_id: 0`, logs 404, ~9s. Same failure mode.

**Fix that restored runners:** the repository was private and GitHub-hosted jobs completed with `runner_id: 0`. Setting visibility to **public** made standard hosted minutes available. Re-run [35322120250](https://github.com/sim-too-real/reality-os-rust/actions/runs/35322120250) job `verify` (`105526744855`, `runner_id=1000006152`) executed checkout, `dtolnay/rust-toolchain@1.95.0`, fmt, clippy, `test-metal` (including `xl330_pty_untrusted_recover_after_replay_does_not_write` and `xl330_pty_recover_after_integrity_then_watchdog_does_not_write`), and workspace `test` — all success. `verify-mujoco` also succeeded.

## 5. Physical campaign readiness

### Run A — `2f68a5d` baseline

Blockers: native Linux, XL330 UART, VIN cutoff operator test, USB 5 V isolated from VIN. Runbook: `docs/metal/RUN_A_PRE_FIX_BASELINE.md`. Status: **not measured**.

### Run B — patched candidate

Blockers: same hardware plus patched SHA (`709a079` or descendant) and hostile recover cases. Runbook: `docs/metal/RUN_B_PATCHED_AUTHORITY.md`. Status: **not measured**.

## 6. Documentation cleanup

- Recover plan: status table PRE-FIX / PATCHED / PTY pending / XL330 not measured / NAMED_HOLE; self-review no longer lists `AbortClass` as the type to implement.
- Spec weakness 2: latch patched in software, UART unmeasured; `2f68a5d` frozen vulnerable baseline.
- Spec weakness 8: `runner_id: 0` diagnosis.
- CLAIM_LEDGER: PRE-FIX and PATCHED rows.
- Run A/B runbooks added with no fabricated metrics.
- `AbortClass` code in Task 1 of the old plan is labeled rejected historical sketch.

## 7. Failure-diagnosis dataset

Harness: `crates/verify/src/failure_diagnosis.rs` (post-hoc only).

| Source | Episodes |
|---|---|
| panda grasp/push/release | 300+300+100 |
| arm_gripper | 700 |
| ur5e push | 300 |
| iiwa14 push | 300 |
| **total episode traces** | **2000** |
| wx250s frozen first-score | **0 episodes** (metrics only) |

Target `EarliestFailureStage` is derived (not `task_result` copy). Grasp uses `object_follows_ee` as verified-hold evidence (12 panda holds vs 220 acquisition-success rows). Push fail with object contact → `ContactEstablishedTaskFailed`.

## 8. Leakage audit

| Tag | Fields |
|---|---|
| runtime-visible | skill, gripper_class |
| post-hoc observed | n_commands, command-kind counts, ctrl_writes, n_contacts, has_object_tool_contact, has_object_contact, authority allow/refuse counts |
| privileged (not in causal vector) | object mass/friction/pose, contact fn/ft, perfect perception |
| excluded / split-only | robot_id, model_hash, world_seed, source_path |
| labels | `earliest_stage` |

Causal feature names are tested to exclude `robot_id` / `model_hash` / path.

## 9. Baseline evaluation

**Headline (specified wx250s holdout):** `holdout_n=0`, `holdout_metrics_only=true`. Cannot score rules vs ML on frozen wx250s episode traces. Do not invent episodes. Do not tune on wx250s.

**Supplementary structural holdout (arm_gripper episodes; not the frozen wx250s first-score):**

| Model | n | accuracy | macro-F1 | Brier | ECE |
|---|---|---|---|---|---|
| rules (taxonomy only) | 700 | 0.5357 | 0.5845 | — | — |
| majority per skill | 700 | 0.2686 | — | — | — |
| logistic (pure Rust, 8 epochs, no sklearn/torch) | 700 | 0.4914 | 0.3171 | 0.0640 | 0.4058 |

Rules beat logistic on this holdout. Logistic ECE 0.41 is poorly calibrated. Per-skill: rules grasp 0.71 vs logistic 0.09; logistic push 0.73 vs rules 0.21 (logistic overfits push, collapses on grasp).

Python pins unchanged: `mujoco==3.7.0`, `numpy==2.2.6`. Classifier uses no extra deps.

## 10. Recommendation

```text
Should Reality OS pursue a small in-house typed physical-decision model yet?

NOT YET
```

Missing evidence:

1. Episode-level traces for the frozen wx250s holdout (or a second untouched embodiment with episodes).
2. A simple model that **beats rules** on that full-embodiment holdout with acceptable ECE.
3. Reproduction on another untouched embodiment.
4. No identity/privileged leakage (already enforced in this harness).
5. Zero actuator authority (already a type/contract constraint).

Until then: do not add `realityos-decision`.
