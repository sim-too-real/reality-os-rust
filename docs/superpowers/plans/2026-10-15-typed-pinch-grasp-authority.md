# Typed Pinch-Grasp Authority Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Admit one exact simulation-only two-finger pinch-grasp through the reasoner and authority bridge, then count verified hold only when independent simulator evidence proves the bounded lift succeeded.

**Architecture:** Define the serializable action envelope in `realityos-kernel`, the shared dependency already used by both semantics and session. Hash scopes with a versioned, length-delimited binary encoding; have the reasoner freeze gripper targets and the lift in one witness; and have the bridge grant and reselect that same envelope. Tighten the privileged verifier to separate contact acquisition from a measured, force-bounded hold.

**Tech Stack:** Rust workspace, Serde, SHA-256, existing MuJoCo verification harness. No new external dependencies.

**Spec:** [`docs/superpowers/specs/2026-10-15-typed-pinch-grasp-authority-design.md`](../specs/2026-10-15-typed-pinch-grasp-authority-design.md)

## Global Constraints

- The grant and result stay `SIMULATION_ONLY`; this work adds no hardware transport or hardware qualification.
- Admit only one qualified `DirectJointGripper` with exactly two independently actuated slide-joint fingers in this variant.
- Require a finite positive explicit per-finger force ceiling equal to the mechanics proof and no greater than the resource maximum; unknown force, friction, mass, pose, topology, geometry, joint state, or freshness refuses.
- Bind the same typed envelope through `PlanStep`, `FrozenAction`, `PhysicalActionScope`, and `ExecutionAuthorization`; all required exact matches use structured values and exact float bits.
- The scope digest is versioned, domain-separated, length-delimited, and encodes each `f64` using `to_bits()`; decimal formatting and free-form serialization are forbidden for authority digests.
- Verified hold requires bilateral declared-finger contact, at least 0.02 m measured vertical lift within the authorized maximum, object-follow residual at most 0.005 m, `UNSUPPORTED` support, available finite in-bound normal force at both fingers, and no forbidden contact.
- A refusal before execution produces no executable action and no control writes. Runtime divergence stops the remaining frozen motion and requires fresh observation and replanning.
- Keep carried-object prediction inside collision checking; only post-execution simulator measurements can establish acquisition or verified hold.

## Review Focus

- Very large but finite finger positions whose close-minus-open range overflows must refuse; pin this in Task 1.
- Reordered or duplicate finger names must not silently bind a different actuator mapping; pin this in Task 2.
- A lift witness with the right endpoint but reordered intermediate states must not retain the same authority; pin this in Task 2.
- One finger with missing force data must fail verified hold even if the other finger reports a valid force; pin this in Task 4.
- Lateral end-effector movement must not satisfy the vertical-lift minimum; pin this in Task 4.

---

### Task 1: Shared action envelope and unambiguous scoped-grant digest

**Files:**
- Create: `crates/kernel/src/physical_action.rs`
- Modify: `crates/kernel/src/lib.rs`
- Modify: `crates/session/src/scoped_grant.rs`
- Modify: `crates/reasoner-bridge/src/lib.rs`
- Modify: `crates/reasoner-bridge/Cargo.toml` (direct dependency on the existing kernel crate)
- Modify: `crates/verify/src/goal_directed.rs`
- Test: `crates/session/src/scoped_grant.rs`
- Test: `crates/reasoner-bridge/tests/bridge.rs`
- Test: `crates/verify/tests/production_reasoner_authority.rs`
- Test: `crates/verify/tests/physical_experiment_decision.rs`

**Interfaces:**
- Produce `realityos_kernel::physical_action::PhysicalActionEnvelope` with variants `LinearStroke { requested_m: f64, maximum_m: f64 }` and `PinchGrasp { resource_id: String, finger_joint_names: Vec<String>, open_positions: Vec<f64>, close_positions: Vec<f64>, per_finger_force_ceiling_n: f64, lift_trajectory_digest: String, maximum_lift_m: f64 }`.
- Produce `PhysicalActionEnvelope::validate(&self) -> Result<(), &'static str>` for finite values, positive required bounds, exactly two distinct names, matching vector lengths, and a 64-character hexadecimal lift digest. Resource/model agreement remains the reasoner's responsibility.
- Produce `PhysicalActionEnvelope::exactly_matches(&self, other: &Self) -> bool`, comparing every float by `to_bits()` and every string/vector in order.
- Produce `PhysicalActionEnvelope::linear_stroke_bounds(&self) -> Option<(f64, f64)>` so legacy linear-only callers can extract bounds and refuse other variants.
- Change `PhysicalActionScope` to carry `action_envelope: PhysicalActionEnvelope` instead of `requested_stroke_m` and `execution_bound_m`. Keep the old bridge behavior by constructing `LinearStroke` from its prepared request and admission bound.

- [ ] **Step 1: Add red tests** named `scope_digest_uses_exact_float_bits_and_unambiguous_string_lengths` and `action_envelope_rejects_malformed_and_overflowing_bounds`. Assert that sub-nanometer stroke changes, signed-zero position changes, newline-bearing strings, duplicate fingers, mismatched vectors, non-finite values, and overflow-prone finite ranges cannot alias or validate.
- [ ] **Step 2: Run the tests and confirm they fail** with missing typed-envelope support or the current decimal/delimiter collision.
  Run: `cargo test -p realityos-session scope_digest_uses_exact_float_bits_and_unambiguous_string_lengths`
  Expected: FAIL before the implementation.
- [ ] **Step 3: Add the kernel type and validator**, export it from `crates/kernel/src/lib.rs`, migrate the session scope, and implement a SHA-256 encoder with a `realityos.physical_action_scope/v2` domain tag. Encode scope fields in declaration order, use a fixed one-byte action-variant tag and variant field order, encode each string as a big-endian u64 byte length plus UTF-8 bytes, each vector with a big-endian u64 element count, and each float as big-endian `to_bits()`.
- [ ] **Step 4: Update grant coverage and the existing bridge's linear scope construction** to compare the typed envelope and refuse invalid envelopes before grant issuance.
- [ ] **Step 5: Run the focused session and bridge suites.**
  Run: `cargo test -p realityos-session -p realityos-reasoner-bridge`
  Expected: PASS, including the old push/probe exact-scope tests.
- [ ] **Step 6: Run legacy verification consumers of the migrated scope.**
  Run: `cargo test -p realityos-verify --test production_reasoner_authority --test physical_experiment_decision`
  Expected: PASS with their existing scalar authorization behavior represented by typed linear envelopes.

### Task 2: Freeze the resource-bound grasp and lift in the physical proof

**Files:**
- Modify: `crates/semantics/src/physical_reasoner.rs`
- Modify: `crates/semantics/src/physical_decision.rs`
- Modify: `crates/semantics/src/execution_envelope.rs`
- Modify: `crates/reasoner-bridge/src/lib.rs`
- Modify: `crates/reasoner-bridge/Cargo.toml` (direct dependency on the existing kernel crate)
- Modify: `crates/verify/src/goal_directed.rs`
- Test: `crates/semantics/tests/physical_decision_cycle.rs`
- Test: `crates/verify/tests/production_mujoco_reasoner.rs`
- Modify: `crates/semantics/src/allowed_contact.rs` (grasp-lift phase policy)
- Modify: `crates/semantics/src/physical_interaction.rs` (evidence-only candidate migration)
- Test: `crates/semantics/tests/physical_decision.rs`
- Test: `crates/semantics/tests/physical_decision_cycle.rs`
- Test: `crates/semantics/tests/execution_supervision.rs`
- Test: `crates/verify/tests/production_reasoner_authority.rs`

**Interfaces:**
- Add `ReasoningRequest.gripper_resource: Option<ControlledResource>` and `ReasoningRequest.linear_execution_bound_m: Option<f64>`. The bridge copies its admission cap into a cloned request before selection, preparation, and reselection; those three phases use the same effective request.
- Add `PlanStep.action_envelope: PhysicalActionEnvelope`, `FrozenAction.action_envelope: PhysicalActionEnvelope`, and `ExecutionAuthorization.action_envelope: PhysicalActionEnvelope`. Keep `PlanStep.stroke_m` only as a ranking metric; derive executed stroke from `LinearStroke`.
- Add `ScopedGrantBinding.action_envelope` so reasoner reselection binds the whole envelope, not just a stroke.

- [ ] **Step 1: Add red reasoner tests** named `grasp_requires_exact_qualified_direct_two_finger_resource`, `prepare_step_rejects_changed_gripper_force_target_and_lift`, and `lift_witness_commits_to_ordered_joint_states`. Assert that missing/unknown resource data yields no executable grasp, the supported fixture proves approach/contact/close/lift with at least 0.02 m vertical travel, and any resource, force, target, lift-state ordering, or observation change makes preparation fail.
- [ ] **Step 2: Run the focused tests to verify failure** against the current scalar-only scope and contact-only grasp witness.
  Run: `cargo test -p realityos-semantics grasp_requires_exact_qualified_direct_two_finger_resource`
  Expected: FAIL because the request and frozen step do not yet bind a typed resource or lift.
- [ ] **Step 3: Move the shared enum into semantics use and migrate its authority structs.** Make `ExecutionAuthorization::covers` require `action_envelope.exactly_matches(...)` and retain no duplicate scalar stroke/bound authority fields.
- [ ] **Step 4: Extend grasp proof construction** to validate the resource's direct actuator-to-slide-joint mapping, exact finger order and limits, measured open and proved close coordinates, equal mechanics/resource force bound, and a vertical lift path. Commit the ordered named lift states into the witness and lift digest; include the carried-object swept volume in collision checks.
- [ ] **Step 5: Make `prepare_step` re-prove the complete envelope**, and make every current/future `PlanStep` carry the same envelope as its witness. Keep a ranking stroke separate from the typed authority values.
- [ ] **Step 6: Run semantics and existing production reasoner tests.**
  Run: `cargo test -p realityos-semantics`
  Expected: PASS; the existing push/probe selection remains unchanged except for typed linear envelopes.

### Task 3: Admit exact typed grasps through the simulation bridge

**Files:**
- Modify: `crates/reasoner-bridge/src/lib.rs`
- Modify: `crates/reasoner-bridge/Cargo.toml` (direct dependency on the existing kernel crate)
- Modify: `crates/verify/src/goal_directed.rs`
- Modify: `crates/reasoner-bridge/tests/bridge.rs`
- Modify: `crates/session/src/scoped_grant.rs`
- Modify: `crates/semantics/src/physical_reasoner.rs` (enforce the exact linear cap before generating a grantable step)

**Interfaces:**
- `authorize_current_action` admits only `planar_push`, safe `planar_probe`, and the supported `pinch_grasp` envelope.
- Scope, grant binding, authorization, and selected step all carry the same `PhysicalActionEnvelope`; `same_step` uses `PhysicalActionEnvelope::exactly_matches` to compare all envelope members by exact float bits.
- Keep the bridge free of plant and hardware writers.

- [ ] **Step 1: Add red bridge tests** named `supported_grasp_is_granted_only_after_exact_reselection`, `grasp_resource_force_or_lift_mismatch_refuses`, and `unsupported_topology_stale_evidence_expiry_and_forbidden_grasp_refuse`. Assert exact envelope equality and no executable result/grant on every refusal.
- [ ] **Step 2: Run the new tests and verify they fail** because the bridge currently rejects every `pinch_grasp`.
  Run: `cargo test -p realityos-reasoner-bridge supported_grasp_is_granted_only_after_exact_reselection`
  Expected: FAIL with `UnsupportedActionFamily("pinch_grasp")`.
- [ ] **Step 3: Build scope only from the freshly prepared action**, bind the exact action envelope, issue only `SIMULATION_ONLY` authority, attach that same envelope to the cloned grant request, and reselect the exact candidate. Refuse any changed envelope or unsupported family before the executor can receive an action.
- [ ] **Step 4: Run the bridge suite**, including old push/probe, expiry, undersized-bound, and forbidden-action cases.
  Run: `cargo test -p realityos-reasoner-bridge`
  Expected: PASS with zero writer APIs added to the bridge.

### Task 4: Make privileged grasp evidence prove acquisition and hold separately

**Files:**
- Modify: `crates/verify/src/manipulation_verify.rs`
- Modify: `crates/verify/src/manipulation.rs`
- Test: `crates/verify/tests/typed_grasp_verification.rs` (create)

**Interfaces:**
- Extend `PrivilegedVerdict` with `grasp_acquired` and `verified_hold`.
- Change `verify_grasp` to accept the exact optional authorized envelope. Without a matching `PinchGrasp` envelope, it may record acquisition but cannot report verified hold.
- Emit `grasp_acquired` and `verified_hold` evidence markers independently. Count acquisition from its marker even when task result is failure; count hold only from its marker and successful task result.

- [ ] **Step 1: Add red verifier tests** named `one_sided_contact_is_not_acquisition`, `missing_or_over_limit_force_on_either_finger_rejects_hold`, `minimum_lift_is_vertical_and_bounded`, `object_follow_tolerance_is_five_millimeters`, `table_support_and_forbidden_contact_reject_hold`, and `acquisition_is_counted_when_hold_fails`. Assert every failed hold has `verified_hold == false`, and no single-sided or unknown-force case is successful.
- [ ] **Step 2: Run the verifier tests to verify failure** against current one-contact acceptance and 25 mm follow tolerance.
  Run: `cargo test -p realityos-verify --test typed_grasp_verification`
  Expected: FAIL until the strict verifier and independent metric markers exist.
- [ ] **Step 3: Require a contact for each exact declared finger body** and sum that finger's contact normal forces only when every contributing measurement is available, finite, and nonnegative; each sum must be at or below the envelope ceiling. Measure EE vertical displacement, enforce the 0.02 m minimum and envelope maximum, require 3D object-follow residual at most 0.005 m, require `SupportKind::Unsupported`, and reject any forbidden contact.
- [ ] **Step 4: Return acquisition and hold independently**, update `ManipulationMetrics::absorb`, and make grasp task success mean `verified_hold`. Legacy verification without an exact authority envelope can contribute acquisition evidence only.
- [ ] **Step 5: Run focused verifier tests and the existing manipulation verification suite.**
  Run: `cargo test -p realityos-verify --test typed_grasp_verification`
  Expected: PASS, including separate acquisition and verified-hold counters.

### Task 5: Prove the complete authorized grasp in MuJoCo

**Files:**
- Modify: `crates/verify/tests/production_mujoco_reasoner.rs`
- Modify: `crates/verify/Cargo.toml` only if a test-only dependency is needed.

**Interfaces:**
- Use `authorize_current_action` to obtain the only action the MuJoCo harness may execute; do not hand-build a plan, scope, grant, or authorization.
- The harness checks grant coverage and each frozen approach/contact/close/lift phase against the envelope before writing controls, and stops on divergence.

- [ ] **Step 1: Replace the current refusal-only grasp integration assertion** with a supported exact-resource fixture and add a failure variant for insufficient force or failed acquisition.
- [ ] **Step 2: Execute only the witness's ordered approach/contact/close/lift joint states** in the simulator harness. Check the same action envelope and grant before every phase; record control writes and unauthorized writes.
- [ ] **Step 3: Capture before/after privileged truth** and call the strict verifier with the exact action envelope. Assert bilateral contact, measured vertical lift at least 0.02 m and within the cap, object-follow residual at most 0.005 m, unsupported support, in-bound force at each finger, no forbidden contact, and `verified_hold` only for the passing episode.
- [ ] **Step 4: Assert the failed episode is never a verified hold**, pre-execution refusals have no executable action and zero writes, runtime divergence stops remaining phases, and both episodes leave unauthorized writes at zero.
- [ ] **Step 5: Run the focused integration and package checks.**
  Run: `cargo test -p realityos-verify --test production_mujoco_reasoner -- --nocapture`
  Expected: PASS for push/probe/replan and authorized grasp, with evidence still labeled `SIMULATION_ONLY`.

## Final verification

Run these after all tasks:

- `cargo fmt --all -- --check`
- `cargo test -p realityos-kernel -p realityos-session -p realityos-semantics -p realityos-reasoner-bridge`
- `cargo test -p realityos-verify --test production_reasoner_authority --test typed_grasp_verification --test production_mujoco_reasoner -- --nocapture`
- `cargo clippy -p realityos-kernel -p realityos-session -p realityos-semantics -p realityos-reasoner-bridge -p realityos-verify --all-targets -- -D warnings`
- `git diff --check`

The full workspace verifier includes long simulator loops and `realityos-vport` has existing Windows-only incompatibilities; report those separately if they limit broad validation. Preserve existing uncommitted bridge changes and frozen evidence artifacts; do not stage or commit unrelated files.
