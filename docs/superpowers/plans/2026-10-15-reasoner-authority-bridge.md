# Reasoner-to-authority bridge implementation plan

**Execution:** local `main`, as directed. No commit is part of this task.

## Task 1: Separate probe authority requests from executable actions

**Files:**
- Modify `crates/semantics/src/physical_reasoner.rs`
- Test in the existing `physical_reasoner::tests` module

**Interfaces:**
- Add `ReasoningReport::authority_request: Option<PlanStep>`; this is a proposal only.
- Add `PhysicalReasoner::prepare_step(&mut self, request: &ReasoningRequest, step: &PlanStep) -> Result<FrozenAction, &'static str>`; it re-proves the witness and returns a frozen action without authority.

- [x] Add regression assertions that an unscoped safe probe creates `authority_request` while `executable` stays `None`, and that an attached exact grant makes the selected probe executable with identical witness and stroke.
- [x] Run `cargo test -p realityos-semantics physical_reasoner::tests::blocked_translation_needs_a_probe_then_an_intermediate_push` and confirm the new assertion fails because the report currently has no authority-request channel and probe step.
- [x] Implement the internal scope-preview selection using the canonical `decide_physical_action` function; only expose a proposal if it selects a proved probe whose future interaction is preserved.
- [x] Preserve a `PlanStep` for such probe proofs even when goal mechanics are unknown; keep actual `executable` dependent on a real grant.
- [x] Extract the existing proof/witness checks into `prepare_step`; make `authorize_step` call it before binding the issuer's authorization.
- [x] Run the focused test and `cargo test -p realityos-semantics` (330 unit tests and all semantics integration tests passed).

## Task 2: Add a reusable simulation-only bridge crate

**Files:**
- Create `crates/reasoner-bridge/Cargo.toml`
- Create `crates/reasoner-bridge/src/lib.rs`
- Create `crates/reasoner-bridge/tests/bridge.rs`
- Modify workspace members/dependencies in root `Cargo.toml`
- Modify `crates/verify/Cargo.toml`

**Interfaces:**
- `SimulationAdmission { execution_bound_m: f64, expires_at_s: f64 }`
- `AuthorizedReasonerAction { report: ReasoningReport, action: AuthorizedAction, grant: ScopedSimulationGrant }`
- `BridgeRefusal::{SensorEvidenceMissing, NoJustifiedAction, UnsupportedActionFamily, SelectionChanged, Scope(ScopeRefusal), Reasoner(&'static str)}`
- `authorize_current_action(os, reasoner, request, admission, forbidden_action_keys) -> Result<AuthorizedReasonerAction, BridgeRefusal>`

- [x] Add bridge tests for a safe initial probe, exact scope/frozen-action identity, `SIMULATION_ONLY` evidence, stale sensing, invalid bounds, and refusing when no justified action exists.
- [x] Run `cargo test -p realityos-reasoner-bridge` and confirm the expected missing bridge behavior fails.
- [x] Implement scope derivation from `PhysicalReasoner::prepare_step`, simulation grant issuance, grant-to-authorization conversion, re-selection against the same request, and exact-step equality checks.
- [x] Ensure the bridge has no direct plant or hardware transport dependency and rejects non-planar-push families.
- [x] Run bridge tests and `cargo test -p realityos-session` (all 15 session tests passed).

## Task 3: Replace the MuJoCo test's hand-built push/probe grant path

**Files:**
- Modify `crates/verify/tests/production_mujoco_reasoner.rs`

- [x] Route the push/probe/replan scenario through `authorize_current_action`; remove its manual `PlanStep`, scope, grant, and authorization copying for push-family actions.
- [x] Keep assertions that each executed action matches the selected witness and that evidence remains simulation-only.
- [x] Run `cargo test -p realityos-verify --test production_mujoco_reasoner`; both MuJoCo scenarios ran and passed.
- [x] Run `cargo test --workspace`; on this Windows host it stops at the existing Unix-only `realityos-vport` permission APIs. The full verifier suite had 156 passes and three simulator resource failures under parallel load; all three passed when rerun serially.

## Review focus

- A preview binding must remain local to reasoner selection and never escape as an issued grant.
- The second selector run must match the first request's exact proposal; a changed action, epoch, witness, or stroke must refuse.
- Probe authority requests must not turn unknown future interaction into permission.
- A failed bridge call must not return or expose an executable action.
- Only the supported linear-stroke family is admitted until a typed gripper envelope exists.
