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
- Task 3: in progress. Before editing the CLI, the seed 21003 sidecar verified with `status: PASS`; the first analyzer run reports `absent_legacy` for both measurements while retaining motion and wrench onset brackets.
- Task 4: pending.

## Evidence and implementation notes

- Contact-force snapshots use the same `MjData.contact` ordering and step as each pose sample. The worker helper calls `mj_contactForce` but never advances physics; its fixture test checks simulation time and joint state remain unchanged.
- New sample digests use v3 and force-bearing trace digests use v4. Existing digest selection is preserved when the optional field is absent.
- Rust parsing rejects contact-force pair identity mismatches, and semantics validation rejects force/pair length mismatches and partial traces. Empty arrays remain distinct from legacy omission.
