# Per-Step Probe Contact Force Diagnostics Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add digest-verified MuJoCo contact-force snapshots to time-aligned probe evidence, then compare them with the tool-wrench inference on one fresh held-out case.

**Architecture:** Extend `ProbeObjectPoseSample` with an optional, versioned contact snapshot list. Capture existing MuJoCo contact values in the worker and parse them in Rust. Keep inference unchanged; extend the verifier and offline analyzer to show support-contact force around motion onset.

**Tech Stack:** Rust, Python, serde, existing MuJoCo worker, existing Reality OS sidecar verifier.

**Spec:** `docs/superpowers/specs/2026-11-04-probe-contact-force-diagnostics-design.md`

## Global Constraints

- Local canonical `main` only; no worktrees or copied repository checkouts.
- Preserve inference thresholds at 1 mm planar motion and 0.5 N tangential-force lower bound.
- Diagnostic fields do not influence inference, authority, or control.
- Old frozen sidecars without contact-force samples remain readable and verifiable.
- Freeze each replay before execution; at most one simulation run for the new seed.
- No full-lift, physical actuation, purchasing, fabrication, or external contact.

## Review Focus

- Legacy pose samples without contact-force fields retain their old digest and verification behavior; test old sample compatibility.
- A force value or friction coefficient changed after capture invalidates the digest; test force/friction digest binding.
- Contact-force and contact-pair lists with different length or identities fail validation; test alignment rejection.
- Empty contact lists remain distinguishable from missing legacy data; test `Some([])` versus `None`.
- Missing contact force on only some new samples fails the trace-completeness check; test partial-trace rejection.

---

### Task 1: Versioned contact-force evidence types

**Files:**
- Modify: `crates/semantics/src/push_probe.rs`
- Test: unit tests in `crates/semantics/src/push_probe.rs`

**Interfaces:** Add `ProbeContactForceSample` and an optional `contact_force_samples` field on `ProbeObjectPoseSample`. Each contact sample carries the corresponding `ProbeContactPair`, `position_world_m`, `frame: [f64; 9]`, `friction: [f64; 5]`, `force_local: [f64; 3]`, `torque_local: [f64; 3]`, `normal_force_n`, `tangential_force_n`, and `force_available`. Preserve existing digest selection when the field is absent; select the new sample/trace digest versions when it is present.

- [x] Add inline tests `probe_object_pose_digest_binds_contact_force_samples` and `probe_object_pose_trace_rejects_partial_contact_force_samples` in a new `probe_object_pose_trace_tests` module for force/friction digest binding, identity alignment, empty-versus-missing data, and partial-trace rejection.
- [x] Run the focused semantics tests and confirm they fail on the missing field/digest behavior.
- [x] Implement the optional type, validation, and digest versioning.
- [x] Run the focused semantics tests and confirm they pass, including legacy compatibility.

### Task 2: Capture contact snapshots from the existing worker stream

**Files:**
- Modify: `crates/verify/mujoco_worker.py`
- Modify: `crates/verify/src/manipulation_benchmark/wrench_sensor.rs`
- Test: inline unit tests in `crates/verify/src/manipulation_benchmark/wrench_sensor.rs` and `crates/verify/tests/test_mujoco_wrench_gates.py`

**Interfaces:** Emit `contact_force_samples` beside each `object_pose_trace` record, from the same `MjData.contact` entries and simulation step. Parse each record into the semantic type from Task 1; require ordering and identity alignment with `contact_pairs` when the field is present.

- [x] Add `probe_pose_trace_tests::parses_and_seals_contact_force_samples`, proving contact force, friction, position, frame, and availability survive into a pose sample.
- [x] Run the focused parser test and confirm it fails before the field is parsed.
- [x] Add one worker helper that captures contact force and torque without advancing physics; populate the trace field.
- [x] Parse and validate the new field in Rust.
- [x] Run focused capture/parse/validation tests.

### Task 3: Verify and analyze the new evidence

**Files:**
- Modify: `crates/verify/src/bin/realityos-verify.rs`
- Create: `crates/verify/probe_contact_force_analysis.py`
- Test: offline compatibility checks against the committed seed 21003 sidecar and analyzer output (no simulator replay).

- [x] Before editing the CLI, verify the committed seed 21003 sidecar with `target/debug/realityos-verify.exe`; it must pass.
- [x] Extend CLI output with `contact_force_trace_status`; create the analyzer to report per-contact force, friction, normal/tangential magnitudes, and the paired tool-wrench interval at slip onset.
- [x] Re-run the verifier and analyzer on seed 21003; confirm the old sidecar still verifies and reports `absent_legacy`, then run formatting and `git diff --check`.
- [x] Run `cargo check -p realityos-verify`, `cargo fmt --all -- --check`, and `git diff --check`.

### Task 4: Freeze and run one fresh held-out case

**Files:**
- Create: `docs/superpowers/evidence/rotated-probe-heldout-seed-21004-20261104/`

- [x] Generate seed 21004 trial and zero-step preflight on the current canonical `main`.
- [ ] Freeze trial, preflight, exact source/model hashes, unchanged thresholds, and one-run budget; verify and commit/push the freeze before replay.
- [ ] Run the frozen simulator trial once; preserve its episode and contact/wrench sidecar whether the task passes or fails. One run completed and the episode was preserved, but no measurement sidecar was emitted because the monitor aborted before inference.
- [ ] Independently verify sidecar hashes and recompute support-contact force versus inferred friction at the slip onset. Not possible for this seed: there is no sidecar and no slip onset.
- [x] Record classification, findings, limitations, and next justified experiment; commit/push the result.
- [x] Confirm all four repositories remain on `main` and synchronized with their remotes (all report 0 ahead / 0 behind after fetching; `first-principles` has local working-tree changes that were left untouched).

### Task 5: Preserve diagnostics when the monitor aborts

**Reason:** Seed 21004 stopped on `unexpected_contact` before a finalized measurement existed. The first abort-preservation patch was insufficient: seed 21005 stopped during probe transit, before the formal motion phase where pose/contact-force capture was enabled.

**Files:**
- Modify: `crates/verify/src/manipulation_benchmark/execution.rs`
- Modify: `crates/verify/tests/manipulation_benchmark.rs`
- Evidence: `docs/superpowers/evidence/rotated-probe-heldout-seed-21005-20261104/`

- [x] Record a probe measurement after `WrenchAborted` when pose and wrench samples are available; keep `UnexpectedContact` ambiguous so inference remains fail-closed. Commit/push: `3258af0`.
- [x] Freeze seed 21005 and its zero-step preflight; `FREEZE_VERIFY.json` passed. The freeze files were not committed before replay.
- [x] Run seed 21005 once and preserve its episode. It aborted during probe transit with 0.252 mm displacement and no measurement records or contact-force trace; see `RESULT.json` and `CHECKPOINT.md`. Do not replay seed 21005.
- [x] Record the root cause: contact-force pose capture was tied to the formal motion phase, so the transit abort had no trace to seal.

### Task 6: Capture contact evidence independently of the contact gate

**Files:**
- Modify: `crates/verify/mujoco_worker.py`
- Modify: `crates/verify/src/mujoco_exec.rs`
- Modify: `crates/verify/src/manipulation_benchmark/execution.rs`
- Test: `crates/verify/tests/test_mujoco_wrench_gates.py`
- Evidence: `docs/superpowers/evidence/rotated-probe-heldout-seed-21006-20261104/`

- [x] Add a read-only per-step probe trace request that records contact forces during transit without changing `expected_contact_body` or the unexpected-contact stop.
- [x] Preserve a trace origin before the probe action so an abort before the formal motion phase can still produce a time-aligned trace; a trace-capture error cannot bypass the stop gate.
- [x] Run the worker wrench-gate suite (8 passed), `cargo check -p realityos-verify --tests`, formatting, and diff checks.
- [x] Freeze and run seed 21006 once. Its freeze verified before the reservation; the working-tree freeze was not committed before replay and changed the expected-contact gate for probe transit.
- [x] Verify and analyze the seed 21006 sidecar: 6,130 pose samples, 25,912 force samples, onset bracket 23.978–23.980 s, and simulated μ=0.631634 inside inferred [0.335121, 0.8].
- [x] Record that the seed 21006 run does not validate the final source change: the final code restores the original transit gate and decouples evidence capture. Do not replay seed 21006; its one-run budget is consumed.
- [x] Commit/push the final source and evidence in `7f239943fcf3d68dbdd54ebe601acee738b002e2`, then confirm all four canonical repositories are on `main` and synchronized.

Do not replay seeds 21004, 21005, or 21006. Any further simulator evidence needs a new seed frozen against a committed source snapshot.
