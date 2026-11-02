# Independent Probe Interval Reference Implementation Plan

> **For agentic workers:** Execute inline on canonical `main`; do not create or enter a worktree or another branch.

**Goal:** Independently recompute supported v1 friction-probe inferences from archived JSON sidecars using Python standard library only.

**Architecture:** A separate Python module validates sidecar, sample, measurement, and pose-trace hashes, then implements the common contact gates and slip-onset interval arithmetic. It supports the validated no-intended-contact refusal and slip-onset update paths; other results fail closed as unsupported.

**Tech Stack:** Python 3 standard library (`json`, `hashlib`, `struct`, `math`, `argparse`, `unittest`).

**Spec:** `docs/superpowers/specs/2026-11-03-independent-probe-interval-reference.md`

## Global Constraints

- Work only on canonical local `main` and push only to `origin/main`.
- Do not use any worktree or other branch.
- No simulator, physics step, physical actuation, purchase, fabrication, or external contact.
- Preserve `NOT_ADMITTED`, `metal=false`, `shop_release=false`, and `viewport_admits=false`.
- Reuse the full existing `realityos.probe_measurement_sidecar` v1 records.
- Python reference errors on unsupported input; it never treats an unsupported record as a pass.

## Review Focus

- Float packing must match Rust f64 little-endian digest inputs: test sidecar digest acceptance and single-wrench mutation rejection.
- Contact list ordering and body-pair symmetry affect identity/gates: test complete-trace absence and incomplete-trace rejection.
- Interval arithmetic must not copy the stored result: assert against frozen literal interval values.
- Stored output tampering must fail: mutate the recorded interval while preserving measurements.
- Unsupported formats and inference branches must fail closed: test wrong schema version and unsupported result.

---

### Task 1: Independent reference verifier

**Files:**
- Create: `crates/verify/probe_interval_reference.py`
- Test: `crates/verify/tests/test_probe_interval_reference.py`

**Interfaces:**
- `verify_sidecar(sidecar: dict) -> dict` returns a summary for supported records or raises `ReplayError`.
- CLI: `python crates/verify/probe_interval_reference.py SIDECAR.json` prints JSON; invalid/unsupported evidence exits nonzero.

- [x] Write tests for both archived sidecars, raw sample tampering, stored interval tampering, and incomplete contact trace.
- [x] Run `python -m unittest discover -s crates/verify/tests -p test_probe_interval_reference.py` and confirm the missing implementation fails.
- [x] Implement independent digest packing, trace/contact validation, common gate checks, and slip-onset force/normal interval arithmetic.
- [x] Re-run `python -m unittest discover -s crates/verify/tests -p test_probe_interval_reference.py` and the CLI for `23023:17` and `23024:17`; compare with frozen Rust outputs and literal acceptance values.
- [x] Run `git diff --check`; inspect that the Python implementation neither imports nor calls the Rust verifier or simulator.

### Task 2: Evidence and mission checkpoint

**Files:**
- Modify: `docs/superpowers/specs/2026-11-03-probe-interval-replay-evidence-contract.md`
- Modify: `docs/superpowers/evidence/grasp-lift-margin-20261102/RESULTS.md`
- Modify: `C:/projects/first-principles/docs/research/mission_state.json`
- Update: `docs/superpowers/evidence/probe-interval-reference-replay-20261103/RESULTS.json`

- [x] Record exact inputs, independent outputs, artifact hashes, verification commands, and limitations.
- [x] Confirm all safety and release flags remain false in the freeze.
- [x] Commit and push the finished Reality OS implementation, then commit and push the First Principles mission-state update.
- [x] Fetch both canonical repos and confirm local `main` equals `origin/main` with clean worktrees.

## Review ledger

- Read-only self-review found and fixed malformed-field exception leakage, invalid pose-array exception leakage, and start-pose validation drift from the Rust trace contract. Regression tests cover each case. No blocking issue remains within the frozen v1 scope.
- Final focused suite on implementation commit `ab8039b6aba387ac52111ddffbb436e292ede6e3`: 11 passed; syntax compilation and both CLI replays passed.
- The one final fresh reviewer checkpoint is covered by a read-only self-review because this run has no subagent reviewer available.
