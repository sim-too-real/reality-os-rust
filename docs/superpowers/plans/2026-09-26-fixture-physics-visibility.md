# Test Fixture Physics Visibility Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Keep closed-loop simulator mass/friction declarations explicit while preventing hidden fixture friction from becoming planner input.

**Architecture:** Introduce one named PUSH scenario-physics configuration that owns simulator truth and explicitly states which values are disclosed to the reasoning path. Simulator setup consumes truth; mechanics/belief construction consumes only disclosed values and returns unknown/refusal when friction is hidden.

**Tech Stack:** Rust 2021 workspace; `realityos-verify` test-only closed-loop module; `realityos-semantics::physical_interaction`; `cargo test`.

**Spec:** `docs/superpowers/specs/2026-09-26-physical-input-truth-boundary-design.md`

## Global Constraints

- Keep simulator ground truth separate from what the reasoning path is told.
- A hidden friction value with no inference must yield mechanics unknown/refusal, never `0.3` by fallback.
- Keep explicit fixture parameters as test data; do not promote them to production defaults.
- Do not extract the `#[cfg(test)]` closed loop or change selection/experience behavior in this plan.
- Do not run the verifier's full suite or frozen holdout evaluations in this slice; run only the named focused tests.
- Preserve valid explicitly disclosed fixture behavior and keep diagnostic text honest about unknown values.

## Review Focus

- Simulator scenario receives mass/friction from one named configuration rather than repeated literals.
- Explicitly disclosed friction is carried to the mechanics template with declared provenance.
- Hidden support friction creates an unknown belief and no mechanics template; candidate selection reports `MechanicsUnknown`.
- No `.unwrap_or(0.3)` or fresh declared-point `0.3` remains as an unknown-value recovery in the closed-loop path.
- Diagnostic/rationale text for unknown friction does not print a made-up declared coefficient.

---

### Task 1: Separate simulator truth from disclosed PUSH physics

**Files:**
- Modify: `crates/verify/src/goal_directed.rs` (`#[cfg(test)] mod tests` only)
- Test: unit tests in `crates/verify/src/goal_directed.rs`

**Interfaces:**
- Consumes: `push_scenario`, `mechanics_template`, `fill_mechanics_from_q`, `PhysicalParameterBelief`, `prediction_regime`, and `EvaluationContext`.
- Produces: `PushScenarioPhysics { simulator_mass_kg: f64, simulator_support_friction: f64, reasoner_mass_kg: Option<f64>, reasoner_support_friction: Option<f64> }`; `push_scenario(obj: [f64; 3], size: f64, physics: PushScenarioPhysics, push_dir: [f64; 3], push_dist: f64, seed: u64) -> ManipulationScenario`; and `fill_mechanics_from_q(model: &EmbodimentModel, ee: &str, qpos: &[f64], contact_world: [f64; 3], yaw: f64, physics: &PushScenarioPhysics) -> Option<PlanarPushInitiation>`. A fresh-belief helper must create a declared friction point only from `reasoner_support_friction: Some(mu)` and `with_unknown(SupportFriction, ...)` for `None`.

- [ ] **Step 1: Add failing fixture-visibility tests.** Assert the configured truth is serialized into simulator JSON once; an explicitly disclosed scenario produces a mechanics template with declared provenance; a friction-hidden scenario produces unknown belief, no mechanics template, and `SelectionOutcome::MechanicsUnknown`; rationale text for that case says unknown. Name the tests `push_fixture_truth_and_disclosure_are_separate` and `hidden_fixture_friction_refuses_unknown_mechanics`.
- [ ] **Step 2: Run the focused tests and confirm failure.** Run `cargo test -p realityos-verify push_fixture_truth_and_disclosure_are_separate` and `cargo test -p realityos-verify hidden_fixture_friction_refuses_unknown_mechanics`. Expected: current code leaks the fixture value through fallback/fresh belief and does not refuse for unknown mechanics.
- [ ] **Step 3: Implement explicit scenario physics and disclosure.** Replace the duplicated `.05`/`.3` scenario literals with the named configuration at action/probe simulator setup; remove friction `.unwrap_or(0.3)` fallbacks and the fresh `declared_point(..., 0.3)` initialization; pass only disclosed model inputs into mechanics construction; make missing friction return `None` so `evaluate_candidate` records `MECHANICS_UNKNOWN`; render missing friction as unknown in rationale text.
- [ ] **Step 4: Verify fixtures and closed-loop behavior without holdouts.** Run `cargo test -p realityos-verify goal_directed::tests::push_fixture_truth_and_disclosure_are_separate`, `cargo test -p realityos-verify goal_directed::tests::hidden_fixture_friction_refuses_unknown_mechanics`, and `cargo test -p realityos-verify goal_directed::tests::analytic_selection_and_loop_twice`. Do not run the whole verifier test suite. Also run `rg -n -U 'unwrap_or\(0\.3\)|declared_point\(\s*PhysicalParameter::SupportFriction,\s*0\.3' crates/verify/src/goal_directed.rs`; expected: no fallback/fresh-belief matches in this module.
- [ ] **Step 5: Commit Task 1.** Stage only `goal_directed.rs`; commit as `test(verify): separate fixture truth from planner inputs`.
