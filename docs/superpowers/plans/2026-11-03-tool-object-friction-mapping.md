# Tool–Object Friction Mapping Implementation Plan

> **For agentic workers:** Use the native inline execution path in the canonical `C:/projects/reality-os-rust` `main` checkout.

**Goal:** Make the grasp fixture's scenario friction coefficient control target object–finger sliding friction in MuJoCo.

**Architecture:** The Rust grasp runner marks the target scenario object with the selected gripper finger bodies. The Python worker adds explicit MuJoCo geom pairs for those bodies and applies the scalar to both tangential friction coefficients. Geom scalar mapping uses the three MuJoCo geom slots in sliding/torsional/rolling order.

**Tech Stack:** Rust, Python, MuJoCo, Python `unittest`.

**Spec:** `docs/superpowers/specs/2026-11-03-tool-object-friction-mapping-v1.md`

## Global Constraints

- Use only canonical local `main`; no worktrees or repository copies.
- Change no control, lift, threshold, Governor, admission, or release behavior.
- Do not run a full-lift replay.
- Add no dependency and make no physical or hardware claim.

## Review Focus

- Target object–finger pairs use the scenario value on both tangential axes.
- Unrelated object contacts retain their existing friction mixing.
- Missing target/finger body names fail the diagnostic load instead of silently keeping the wrong friction.
- Geom friction updates use sliding/torsional/rolling order.

### Task 1: Reproduce the friction mismatch

**Files:**
- Create: `crates/verify/tests/test_mujoco_contact_friction.py`

- [x] Add a real worker-load test with a default-friction finger, a target object at μ=0.4, and an unrelated support geom.
- [x] Assert target object–finger contacts have `condim=3`, tangential friction `[0.4, 0.4]`, and torsional friction `0.01`; assert the unrelated pair remains unaffected.
- [x] Run the focused Python test first against the old worker and observe the expected `[1.0, 1.0]` failure.

### Task 2: Apply pair-specific friction

**Files:**
- Modify: `crates/verify/src/manipulation_benchmark/execution.rs`
- Modify: `crates/verify/mujoco_worker.py`
- Test: the Rust worker-object preparation helper and the Python worker-load test.

- [x] Add a Rust helper that clones scenario objects and marks only the target object with finger bodies from `qualified_resources.first()` when a friction value is present.
- [x] Add deterministic explicit MuJoCo geom pairs for those bodies and set their friction to `[μ, μ, 0.01, 0.01, 0.01]` while preserving the geom-derived contact dimension.
- [x] Map scalar geom friction to `[μ, 0.01, 0.01]` in scenario-object construction and the existing body-friction update path.
- [x] Run focused Rust and Python tests; verify target and unrelated contact coefficients, runtime updates, fail-closed body resolution, and nondefault contact dimension.

### Task 3: Review and synchronize

- [x] Review the diff for control-path changes, unrelated files, and changed safety logic.
- [x] Run formatting checks and `git diff --check`.
- [x] Commit and push directly to the existing `main` branch.

## Checkpoint

No full-grasp or lift replay is part of this plan. After the coefficient mapping is independently tested, return to the frozen offline predictor work before considering a new full-lift case.

## Follow-up regression and evidence correction

- [x] Reserve pair names created earlier in the same `MjSpec` load; the actual two-finger bundle regression verifies both target pairs compile with the requested friction. The diagnostic performs model compilation only.
- [x] Record that the archived frozen traces used sliding coefficients `[1.0, 1.0]` and scenario μ in the torsional slot. Preserve `FREEZE.json` and `RESULT.json`; do not use those runs to validate corrected-μ predictor performance.
- [ ] Continue offline predictor work with corrected-model evidence. Keep full-lift replay gated until independent predictor evaluation; no lift was run in this follow-up.
