# Exact owner decision — 21008 freeze authorize (do not invent approval)

## Decision required

**Authorize a git commit** on `reality-os-rust` that lands the 21008 prepare/freeze
tooling and related offline failure-class work on a **clean tree**, then re-run
authorize-on-clean-tree so `FREEZE.json` can set `replay_authorized=true`.

This agent must **not** invent that authorization.

## Why replay is blocked now

`FREEZE.json` status: `FROZEN_WORKING_TREE_AWAITING_COMMIT_BEFORE_REPLAY`  
`tools/assert_rotated_probe_replay_authorized_21008.py` exits **3** on the live freeze.

Dirty / untracked paths at last inspection (may drift; re-check `git status`):

- `tools/write_rotated_probe_freeze_21008.py` (modified)
- `tools/test_write_rotated_probe_freeze_21008.py` (modified)
- `tools/assert_rotated_probe_replay_authorized_21008.py` (untracked)
- `tools/test_assert_rotated_probe_replay_authorized_21008.py` (untracked)
- `docs/superpowers/evidence/rotated-probe-heldout-seed-21008-20261104/PREPARE_CHECKPOINT.md` (modified)
- Plus previously frozen diagnostic paths listed inside `FREEZE.json`

## After owner authorizes commit

1. Commit the authorize/assert tooling + freeze evidence on `main` (or named branch).
2. Re-run clean-tree freeze authorize so `replay_authorized=true` and
   `replay_block_reason` is null **without** changing motion/force thresholds.
3. Assert: `py -3 tools/assert_rotated_probe_replay_authorized_21008.py --json` → exit 0.
4. One FREEZE replay only (see `docs/superpowers/evidence/rotated-probe-next-seed-21008-command-20261104.md`).
5. Never weaken Governor expected-contact / force thresholds to pass.

## Explicit non-actions until authorized

- No purchase / shop load / metal admission.
- No invented `replay_authorized=true` on a dirty tree.
- No threshold lowering.
