# Feasible Public Probe Benchmark Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use `superpowers:executing-plans` to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the failing public push/probe distribution with a versioned, randomized, physically reachable benchmark suite for UR5e and IIWA-14, then freeze a new baseline.

**Architecture:** Load a checked-in scenario-suite manifest into the existing benchmark resolver and bind suite identity and stratum into resolved trials and paired keys. Align the shared push approach gap with the existing bounded probe stroke, then run the strict MuJoCo pilot before freezing and executing the new public baseline.

**Tech Stack:** Rust workspace, serde/serde_json, SHA-256, existing seeded RNG and IK, MuJoCo Python worker, Cargo integration tests.

**Spec:** `docs/superpowers/specs/2026-10-17-feasible-public-probe-benchmark-design.md`

## Global Constraints

- Do not relax any force/torque, collision, authority, geometry, lateral-drift, or refusal threshold.
- Every scheduled trial must run under MuJoCo or be recorded as an infrastructure failure; missing MuJoCo never skips a trial.
- Positive trials that fail pre-execution feasibility resolution remain invalid and fail campaign accounting; they are never relabeled as refusals.
- Every simulation episode remains `SIMULATION_ONLY`; the WX250S holdout is not run or inspected.
- Preserve historical schema-v5/v6 records byte-for-byte and in their existing evidence directories.
- The shared task, reasoner, bridge, executor, and scorer do not branch on robot name, seed, or fixture identity.

## Review Focus

- Invalid or overlapping pilot/public/holdout seed ranges; manifest validation tests reject overlaps.
- Positive trial feasibility failures being relabeled as refusal; resolver tests require an invalid trial instead.
- Tool beginning in contact or probe ending short of contact; geometry tests assert a 4 mm initial gap and at least 4 mm post-contact stroke.
- End-effector radius proxies omitting frame/geom offsets or ignoring unknown collision shapes; geometry tests require offset-inclusive bounds and fail-closed behavior.
- Model-specific target retries changing with random iteration order; replay tests assert deterministic resolved JSON and paired keys.
- Historical schema-v5/v6 records losing compatibility; fixture tests deserialize old records without adding wrench or suite claims.

---

### Task 1: Versioned scenario suite and resolved trial identity

**Files:**
- Create: `robots/benchmarks/manipulation/scenario_suite_v2.json`
- Modify: `crates/verify/src/manipulation_benchmark/scenario.rs`
- Modify: `crates/verify/src/manipulation_benchmark/report.rs`
- Modify: `crates/verify/src/bin/realityos-verify.rs`
- Test: `crates/verify/tests/manipulation_benchmark.rs`

**Interfaces:**
- Add `ScenarioSuiteManifest`, `PushTrialStratum`, and a validated suite loader with suite ID/version, pilot/public seeds, 3:2 interleaved stratum, object pose/plan dimensions/mass/friction/obstacle ranges, resample bound, 4 mm approach gap, and 8 mm probe stroke.
- Add `suite_id`, `suite_version`, and `stratum` to schema-v7 `ResolvedBenchmarkTrial`; add the suite digest to `BenchmarkConfig` with a serde default for v5/v6 compatibility.
- Add `resolve_benchmark_trial_with_suite(...)`; retain the existing resolver wrapper for existing call sites and tests that intentionally exercise legacy fixtures.
- Bind suite ID/version and stratum into `paired_trial_key`; add version 7 readers without changing legacy record content.

- [ ] Write failing tests `scenario_suite_rejects_overlapping_seed_ranges`, `scenario_suite_rejects_invalid_strata_or_probe_gap`, `v7_push_strata_are_interleaved_and_balanced`, and `suite_identity_changes_paired_key`.
- [ ] Run `cargo test -p realityos-verify --test manipulation_benchmark scenario_suite` and confirm failures.
- [ ] Implement strict manifest validation, deterministic index-to-stratum mapping, suite digest binding, and schema-v7 serialization.
- [ ] Run the focused tests and existing v5/v6 deserialization and paired-key tests.

### Task 2: Physically reachable probe scenario generation and contact gap

**Files:**
- Modify: `crates/semantics/src/physical_reasoner.rs`
- Modify: `crates/verify/src/manipulation_benchmark/scenario.rs`
- Modify: `crates/verify/src/manipulation_benchmark/execution.rs`
- Test: `crates/semantics` physical reasoner tests
- Test: `crates/verify/tests/manipulation_benchmark.rs`

**Interfaces:**
- Use the manifest's shared 4 mm approach gap and existing 8 mm probe stroke for both planner standoff and randomized initial `qpos`.
- Positive scenario generation returns a validated expected-success trial or a typed invalid-trial error after the fixed retry bound; it never mutates a positive label into refusal.
- Positive scenarios use predeclared parameter ranges, including a 70–90 mm half-height that keeps side-probe contact near the support-load centerline for the registered tool geometry, and complete-geometry initial-configuration and IK checks. Obstacle samples must remain clear of the declared approach and goal corridor.

- [ ] Write failing tests `push_probe_standoff_fits_authorized_stroke`, `positive_trials_are_not_downgraded_after_failed_feasibility`, `positive_trials_reach_both_development_embodiments`, and `positive_probe_crosses_contact_plane_with_four_mm_remaining`.
- [ ] Run the focused tests and confirm the old 50 mm gap and IIWA positive-label fallback fail.
- [ ] Implement shared gap use, deterministic bounded scenario resampling, explicit refusal-stratum generation, and positive feasibility admission.
- [ ] Run resolver and reasoner tests; verify repeated seed resolution is byte-identical and no robot-name branch is introduced.

### Task 3: Strict v2 pilot and frozen public baseline

**Files:**
- Modify: `crates/verify/src/bin/realityos-verify.rs`
- Modify: `docs/superpowers/specs/2026-10-17-feasible-public-probe-benchmark-design.md`
- Create: `docs/superpowers/evidence/feasible-probe-v2-pilot-<candidate-short>/`
- Create: `docs/superpowers/evidence/feasible-probe-v2-public-<candidate-short>/`
- Test: `crates/verify/tests/manipulation_benchmark.rs`

**Interfaces:**
- Campaign scheduling consumes the manifest seed ranges and records the suite digest in each report.
- Pilot and public reports preserve the complete scheduled denominator and every existing safety, wrench, refusal, success, runtime, and failure-cause metric.

- [ ] Add an integration test that schedules both development arms from the manifest and asserts the exact 12/8 pilot split and nonzero positive denominators.
- [ ] Run `cargo fmt --check`, focused reasoner/benchmark tests, `REALITYOS_REQUIRE_MUJOCO=1 MUJOCO_GL=disable cargo test -p realityos-verify --test manipulation_benchmark -- --test-threads=1`, all changed-package tests, and strict Clippy for changed packages.
- [ ] Freeze the candidate commit and run the MuJoCo-required 20-trial-per-arm pilot on suite-v2 pilot seeds; record report and raw trials. Require zero infrastructure/invalid/unaccounted records and all hard safety gates.
- [ ] If the pilot passes its predeclared gate, run the 100-trial-per-arm suite-v2 public baseline plus the registered grasp-fixture baseline on the new public seed range; do not run or inspect holdout.
- [ ] Record the exact candidate SHA, suite digest, command, completeness, gates, denominators, outcomes, failure causes, and limits in the spec and evidence directories.
- [ ] Commit and push the implementation and reviewed evidence to the current local `main`.
