# Grasp Contact Wrench Trace Implementation Plan

> **For agentic workers:** Use the native inline execution path task by task. Keep all work in the canonical `C:/projects/reality-os-rust` `main` checkout.

**Goal:** Preserve post-close and lift contact wrench plus actuator telemetry for offline diagnosis of the frozen grasp-lift failures.

**Architecture:** The MuJoCo worker already emits contact position, contact frame, and force data. Extend its state payload with body inertial centers and full contact torque, parse these into verifier truth, and append backward-compatible per-contact snapshots to existing grasp audits. Keep derived values alongside raw fields and leave control decisions unchanged.

**Tech Stack:** Rust, Python, MuJoCo, Cargo.

**Spec:** `docs/superpowers/specs/2026-11-03-grasp-contact-wrench-trace-v1.md`

## Global Constraints

- Telemetry remains post-hoc and simulation-only.
- Do not modify grasp control, thresholds, Governor behavior, admission, release authority, or frozen candidate controls.
- Keep prior episode JSON readable through serde defaults.
- Run only the two frozen primary trials named in the spec.
- Use the existing canonical `main` checkout; no worktrees, clones, or alternate copies.

## Review Focus

- Contact body ordering: test both geom 1 and geom 2 as the object.
- Contact frame layout: test row-axis rotation with non-axis-aligned values.
- Torsional contact contribution: test nonzero local contact torque.
- Missing or non-finite inputs: test that derived wrench values remain absent.
- Legacy saved episodes: deserialize a fixture without the new fields.

---

### Task 1: Parse complete post-hoc MuJoCo state

**Files:**
- Modify: `crates/verify/mujoco_worker.py`
- Modify and test: `crates/verify/src/observation.rs`

**Interfaces:**
- `VerifierTruth` gains `xipos: BTreeMap<String, Vec<f64>>`.
- `ContactTruth` gains optional world `position` and optional `torque_local: [f64; 3]`.
- Existing `ctrl` and `actuator_force` remain in verifier truth.

- [x] Add `verifier_truth_keeps_contact_geometry_and_actuator_state` using a MuJoCo-shaped JSON state with one contact, nonzero six-axis wrench, object `xipos`, `ctrl`, and `actuator_force`.
- [x] Run `cargo test -p realityos-verify --lib verifier_truth_keeps_contact_geometry_and_actuator_state` and confirm it fails because the parser drops `position`, `torque_local`, and `xipos`.
- [x] Emit body `xipos` and contact `torque_local` from the worker; parse those plus existing contact `pos`.
- [x] Run the focused parser test and confirm it passes; the same test checks missing optional fields still default safely.

### Task 2: Preserve auditable object contact wrenches

**Files:**
- Modify and test: `crates/verify/src/manipulation_benchmark/execution.rs`

**Interfaces:**
- Add a serde-defaulted `contact_wrenches` collection to `GraspContactAudit`.
- Each contact record retains raw body names, position, local force/torque, frame, friction and dimension, plus optional world force and moment about object `xipos`.
- Each audit also retains optional object `xipos`, `ctrl`, and `actuator_force` for the sampled state.

- [x] Add `grasp_contact_audits_keep_signed_world_wrench` for both contact orderings and nonzero contact torque, plus `grasp_contact_audits_omit_derived_wrench_without_finite_inputs` and a legacy-audit deserialization test; confirm they fail before implementation.
- [x] Run `cargo test -p realityos-verify --lib grasp_contact_audits` and confirm the new behavior fails before implementation.
- [x] Implement the conversion using row-stored MuJoCo contact-frame axes and the geom-2 force convention documented in the spec.
- [x] Add a `post_close` audit at the existing close-settle snapshot and retain the existing lift-waypoint audits.
- [x] Run `cargo test -p realityos-verify --lib grasp_contact_wrench_tests` and benchmark serialization compatibility tests.

### Task 3: Replay and verify the frozen diagnostic cases

**Files:**
- Create: `docs/superpowers/evidence/grasp-lift-wrench-trace-20261103/`
- Modify: the evidence record for the active Reality OS campaign only if a decision-relevant result is established.

- [x] Replay the two frozen primary trials from the spec without changing controls or acceptance criteria.
- [x] Independently verify source hashes, sample phases, raw-to-derived sign conversion, finite-value behavior, and unchanged outcome/guard behavior.
- [x] Record exact limits and the next diagnostic question; do not present solver forces as hardware capacity.
- [x] Compare the prior scalar contact-capacity proxy with signed world-z finger-contact force and COM moment at the first lift waypoint, offline from the frozen trace; retain all existing caps and guards.

### Task 4: Verify and synchronize

- [x] Run focused observation and manipulation benchmark tests plus the two-case deterministic replay verifier.
- [x] Review the final diff and confirm no unrelated files or safety behavior changed.
- [x] Commit and push the validated changes directly to the existing `main` branch.

## Current Checkpoint

Tasks 1 through 4 for the telemetry capture are synchronized to `main`. The offline support comparison is also complete and is being synced as a follow-up evidence increment. The full crate run produced 192/192 library tests passing and 105/106 integration tests passing (6 ignored). The one integration failure was reproduced in isolation and traced to the existing conservative axis-aligned sweep guard rejecting a slightly rotated probe direction; its integration success expectation conflicts with an existing unit-test contract. No safety guard was changed. See `docs/superpowers/evidence/grasp-lift-wrench-trace-20261103/CHECKPOINT.md` for replay findings and the next diagnostic question.
