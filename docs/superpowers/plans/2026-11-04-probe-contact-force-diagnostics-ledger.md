# Probe Contact Force Diagnostics Execution Ledger

Base commit: `90e78dba5a7b3b71c9318172b4c7be54c6d426af`
Branch: `main`
Checkout: `C:\projects\reality-os-rust` (canonical repository, no worktree)
Plan: `docs/superpowers/plans/2026-11-04-probe-contact-force-diagnostics.md`
Spec: `docs/superpowers/specs/2026-11-04-probe-contact-force-diagnostics-design.md`

## Rulings

- Ruling: Work on the canonical `main` checkout without creating a worktree — the user explicitly requires local main and forbids worktrees/copies — cost if wrong: less isolation from shared-checkout edits; mitigate by checking status and staging only plan-owned files before commits.
- Ruling: Push each completed plan commit to `origin/main` — the user explicitly requested commit/push/sync to main — cost if wrong: work becomes visible on shared main before a later review.
- Ruling: Use author self-review rather than a review subagent — developer instructions prohibit spawning subagents — cost if wrong: no independent-context review of this branch.

## Task status

- Task 1: complete. The three RED tests failed on absent digest binding, `Some([])` collapsing to null, and missing partial-trace rejection; all three pass with versioned sample/trace digests and semantic validation. Focused command: `cargo test -p realityos-semantics probe_object_pose_ -- --nocapture` (3 passed).
- Task 2: complete. Parser RED test first observed missing `contact_force_samples`; worker RED test first observed the missing capture helper. The worker now snapshots MuJoCo contacts in-place and the parser preserves and validates pair order. Focused checks: `cargo test -p realityos-verify probe_pose_trace_tests -- --nocapture` (4 passed) and `python crates/verify/tests/test_mujoco_wrench_gates.py` (6 passed). The legacy parser test explicitly asserts force samples remain absent.
- Task 3: complete. Added `contact_force_trace_status` to the CLI and the offline analyzer. Reverified seed 21003 after the CLI change (`status: PASS`, both force traces `absent_legacy`); the analyzer reports both as legacy while preserving onset and tool-wrench brackets. `cargo check -p realityos-verify`, `cargo fmt --all -- --check`, and `git diff --check` passed. A synthetic in-memory analyzer check also exercised new force-bearing onset data and exact wrench-sequence pairing; it is a code-path check, not simulator evidence.
- Task 4: one held-out replay completed. `EPISODE.json` reports `ActionFailed` / `force_monitor_abort` after `unexpected_contact` at 12.834 s, before measurable object motion or inference. The stop was verified, unauthorized writes were 0, and the episode was preserved. The runtime emitted no measurement records or `PROBE_MEASUREMENT_SIDECAR.json`, so independent sidecar verification and slip-onset contact-force comparison are unavailable; the 1-run budget is exhausted and seed 21004 will not be replayed. The zero-step preflight recorded four `obj0`–`table` contacts at μ=0.5725908634 and 2.61718 N normal force per point. A second concurrent prepare-only invocation completed zero-step and collided on exclusive artifact creation; `PREPARATION_AUDIT.json` records the count without mutating the freeze. The freeze files were present and verified before the runner used them, but the freeze Git commit was created while the test process was active, so commit-before-first-simulator-step ordering is not established. Results and limitations are in `CHECKPOINT.md`, `RESULT.json`, `OFFLINE_ANALYSIS.json`, and `OFFLINE_VERIFY.json`.
- Task 5: in progress. The post-run change in `execution.rs` seals a probe measurement when an unexpected-contact monitor aborts after wrench/pose capture. It marks the evidence `runtime_contact_ambiguous`, which the existing inference path rejects without changing the friction belief. The diagnostic test now requires a sealed contact-force trace for `CONTACT_FORCE` freezes. `cargo check -p realityos-verify --tests` passed; no second seed 21004 replay was performed. A new seed 21005 trial and zero-step preflight exist; no freeze or simulator trial exists yet.

## Evidence and implementation notes

- Contact-force snapshots use the same `MjData.contact` ordering and step as each pose sample. The worker helper calls `mj_contactForce` but never advances physics; its fixture test checks simulation time and joint state remain unchanged.
- New sample digests use v3 and force-bearing trace digests use v4. Existing digest selection is preserved when the optional field is absent.
- Rust parsing rejects contact-force pair identity mismatches, and semantics validation rejects force/pair length mismatches and partial traces. Empty arrays remain distinct from legacy omission.
