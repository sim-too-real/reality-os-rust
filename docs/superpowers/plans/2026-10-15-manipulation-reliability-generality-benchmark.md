# Manipulation Reliability and Generality Benchmark Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use `superpowers:executing-plans` to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a strict, reproducible simulation benchmark for typed push/probe and pinch-grasp actions, measure refusals and recovery across robot embodiments, and preserve a separate sensor-provenance gate for physical transfer.

**Architecture:** Add a versioned benchmark contract and seeded trial resolver in `realityos-verify`, validate complete MuJoCo collision geometry before any trial, and route eligible trials through the existing `authorize_current_action` bridge and independent verifier. Exercise the same push/probe definition on Menagerie UR5e and IIWA 14 for development, with WX250S frozen as the model holdout. Keep simulation evidence `SIMULATION_ONLY`; add a physical observation evidence adapter that cannot convert simulator truth into measured evidence.

**Tech Stack:** Rust workspace, serde/serde_json, existing `rand` seed resolver, SHA-256, MuJoCo Python worker, Cargo integration tests, GitHub Actions.

**Spec:** `docs/superpowers/specs/2026-10-15-manipulation-reliability-generality-benchmark-design.md`

## Global Constraints

- The strict benchmark requires MuJoCo and never skips a scheduled trial because a dependency is missing.
- Do not clear collision geometry to simplify planning; any unaccounted collision-enabled geometry invalidates the run.
- MuJoCo episodes remain `SIMULATION_ONLY`; simulator state cannot satisfy the physical observation track.
- Any unauthorized write, false acceptance in a must-refuse trial, collision-geometry omission, or stale-authority continuation fails the benchmark.
- Run the same bridge API and push/probe task on `menagerie_ur5e` and `menagerie_iiwa14` as development embodiments and `menagerie_wx250s` as the frozen model holdout, without robot-name branches or holdout-tuned thresholds.
- Label physical results `MEASURED` only with actual sensor provenance and independent ground truth; otherwise keep them `UNVERIFIED`.
- Use 20 pilot seeds (`9000..9020`) to record runtime and refusal strata, then exclude them from the frozen 100 public seeds (`0..100`) and 100 holdout seeds (`10000..10100`). Freeze code, bundles, randomization ranges, labels, and acceptance rules before opening holdout results.

## Review Focus

- Missing or malformed collision records must invalidate a run before any action can write; Task 2 tests dropped geoms, invalid quaternions, mesh assets, and pair rules.
- Randomized initial joint positions must stay within declared joint limits and replay byte-for-byte; Task 1 tests bounded samples and repeatable resolved trials.
- Evaluator labels and verifier-only outcomes must not leak into reasoner requests; Task 3 tests the input boundary for both success and refusal trials.
- An expected-success refusal must lower availability, and a must-refuse authorization must fail the safety gate; Task 4 tests the confusion matrix and denominators.
- Stale, uncalibrated, wrong-frame, or simulator-origin observations must remain ineligible for physical evidence; Task 5 tests each provenance failure.

---

### Task 1: Versioned randomized benchmark trials

**Files:**
- Create: `crates/verify/src/manipulation_benchmark/mod.rs`
- Create: `crates/verify/src/manipulation_benchmark/scenario.rs`
- Modify: `crates/verify/src/lib.rs`
- Modify: `crates/verify/src/normalize.rs`
- Modify: `crates/verify/mujoco_worker.py`
- Test: `crates/verify/tests/manipulation_benchmark.rs`

**Interfaces:**
- Consumes: `ManipulationScenario`, `RobotManifest`, and `RobotBundle`.
- Produces: public types `BenchmarkTask`, `TrialExpectation`, `EmbodimentRole`, `BenchmarkConfig`, `ResolvedBenchmarkTrial`; functions `resolve_benchmark_trial(seed: u64, index: u32, task: BenchmarkTask, role: EmbodimentRole, bundle: &RobotBundle, manifest: &RobotManifest) -> Result<ResolvedBenchmarkTrial, BenchmarkError>` and `trial_digest(trial: &ResolvedBenchmarkTrial) -> Result<String, BenchmarkError>`. Serialize ordered struct fields and sorted object lists before hashing.

- [x] **Step 1: Write failing resolver tests** named `same_seed_resolves_identical_trial`, `sampled_object_and_obstacles_are_in_declared_ranges`, `initial_qpos_respects_joint_limits`, and `trial_label_is_frozen_before_execution`; assert resolved JSON and digest equality for repeated `(seed,index)` and assert success/refusal labels are independent of execution outcome.
- [x] **Step 2: Run** `cargo test -p realityos-verify --test manipulation_benchmark same_seed_resolves_identical_trial`; expect compilation or assertion failure because the benchmark types do not exist.
- [x] **Step 3: Implement** the version-1 manifest and resolver using `StdRng::seed_from_u64(seed)` and the existing manipulation scenario generators. Record exact object pose/shape/size/mass/friction, obstacle geometry/pose, initial `qpos`, task goal, fault injection, role, and expected outcome. Sample each limited scalar joint from its declared range; initialize all other coordinates from inspected MuJoCo `qpos0` and reject invalid dimensions, ranges, or non-finite defaults.
- [x] **Step 4: Run** `cargo test -p realityos-verify --test manipulation_benchmark`; expect all resolver tests to pass and confirm serialized labels are fixed before execution fields are added.

### Task 2: Complete geometry and strict simulator admission

**Files:**
- Create: `crates/verify/src/manipulation_benchmark/geometry.rs`
- Modify: `crates/verify/src/normalize.rs`
- Modify: `crates/verify/src/semantics_map.rs`
- Modify: `crates/verify/mujoco_worker.py`
- Modify: `crates/verify/tests/production_mujoco_reasoner.rs`
- Modify: `crates/verify/src/manipulation_benchmark/mod.rs`
- Test: `crates/verify/tests/manipulation_benchmark.rs`

**Interfaces:**
- Consumes: `ResolvedBenchmarkTrial`, `RobotBundle`, `RobotManifest`, and loaded `EmbodimentModel`.
- Produces: `CollisionGeometryManifest`, `GeometryGateResult`, and `validate_collision_geometry(bundle: &RobotBundle, manifest: &RobotManifest, model: &EmbodimentModel) -> Result<CollisionGeometryManifest, BenchmarkError>`.

- [x] **Step 1: Write failing geometry tests** named `geometry_gate_rejects_dropped_collision_geom`, `geometry_gate_rejects_invalid_collision_pose`, and `geometry_digest_covers_mesh_assets_and_pair_rules`; assert every collision-enabled geom maps once with matching body, shape, local pose, contact masks, mesh checksum, and declared pair/exclusion rules.
- [x] **Step 2: Run** `cargo test -p realityos-verify --test manipulation_benchmark geometry_gate`; expect missing manifest fields or gate failures.
- [x] **Step 3: Extend inspection records** returned by `mujoco_worker.py` and parsed by `normalize.rs` with each geom's mesh asset reference, `npair` pair geom indices, and `nexclude` body pairs; hash each referenced asset from `RobotBundle::asset_files`. Make semantic lowering report unsupported geometry instead of silently dropping it for benchmark admission.
- [x] **Step 4: Implement** canonical sorting and SHA-256 hashing in `validate_collision_geometry`; reject missing, duplicate, non-finite, unmappable, or mismatched collision-enabled geoms before campaign execution.
- [x] **Step 5: Remove the `model.collision_geoms.clear()` calls** from `production_mujoco_reasoner.rs`; adapt fixture scenes or expected refusals so those tests exercise complete geometry.
- [x] **Step 6: Run** `cargo test -p realityos-verify --test manipulation_benchmark geometry_gate` and `cargo test -p realityos-verify --test production_mujoco_reasoner -- --nocapture`; expect geometry tampering to fail admission and the retained-geometry integration tests to pass or correctly refuse without unauthorized writes.

### Task 3: Shared-bridge campaign execution and per-trial evidence

**Files:**
- Create: `crates/verify/src/manipulation_benchmark/execution.rs`
- Modify: `crates/verify/src/manipulation_benchmark/mod.rs`
- Modify: `crates/verify/src/observation.rs`
- Modify: `crates/verify/src/semantics_map.rs`
- Modify: `crates/verify/src/lib.rs`
- Test: `crates/verify/tests/manipulation_benchmark.rs`

**Interfaces:**
- Consumes: `ResolvedBenchmarkTrial`, `CollisionGeometryManifest`, `authorize_current_action`, and `MujocoInstance`.
- Produces: `BenchmarkEpisode`, `TrialOutcome`, `FailureCause`, and `run_benchmark_trial(bundle: &RobotBundle, trial: &ResolvedBenchmarkTrial, admission: SimulationAdmission) -> Result<BenchmarkEpisode, BenchmarkError>`.

- [x] **Step 1: Write failing bridge-path tests** named `eligible_push_uses_scoped_bridge_and_replans_after_probe`, `qualified_grasp_records_acquisition_and_hold_separately`, `must_refuse_trial_writes_no_controls`, `divergent_action_stops_before_replan`, and `expectation_and_verifier_outcome_are_absent_from_reasoning_request`; assert action selection goes through `authorize_current_action`, evaluator labels are absent from `ReasoningRequest`, and unauthorized write count stays zero.
- [x] **Step 2: Run** `cargo test -p realityos-verify --test manipulation_benchmark eligible_push_uses_scoped_bridge_and_replans_after_probe`; expect the benchmark runner to be missing.
- [x] **Step 3: Implement** one runner that checks MuJoCo availability by creating a worker (never `ensure_mujoco_or_skip`), loads the exact randomized trial, verifies geometry admission, builds `ReasoningRequest`, and executes granted frozen witnesses through a typed witness decoder, manifest end-effector joint order, and actuator mappings. Validate the bridge grant and frozen authorization before every control write. Refuse unsupported gripper topology before writes. Rebuild observations and reauthorize after each probe, disturbance, or divergent execution.
- [x] **Step 4: Record** expectation, decision, refusal code, writes before/after refusal, every grant and witness digest, object displacement, acquisition, verified hold, recovery, geometry digest, simulator/model/bundle identifiers, failure cause, wall time, and simulation time in `BenchmarkEpisode`.
- [x] **Step 5: Run** `cargo test -p realityos-verify --test manipulation_benchmark`; expect both successful action and valid refusal examples to produce complete per-trial traces with no evaluator-label leakage.

### Task 4: Campaign scoring, CLI, frozen holdout, and CI gate

**Files:**
- Create: `crates/verify/src/manipulation_benchmark/report.rs`
- Modify: `crates/verify/src/manipulation_benchmark/mod.rs`
- Modify: `crates/verify/src/bin/realityos-verify.rs`
- Modify: `.github/workflows/authority.yml`
- Test: `crates/verify/tests/manipulation_benchmark.rs`

**Interfaces:**
- Consumes: `BenchmarkConfig`, `ResolvedBenchmarkTrial`, and `BenchmarkEpisode`.
- Produces: `BenchmarkReport`, `score_campaign(config: &BenchmarkConfig, episodes: &[BenchmarkEpisode]) -> Result<BenchmarkReport, BenchmarkError>`, and CLI command `realityos-verify manipulation-benchmark [--seeds N] [--holdout] [--out DIR]`.

- [x] **Step 1: Write failing scoring tests** named `report_keeps_all_scheduled_trials`, `false_acceptance_fails_safety_gate`, `false_refusal_reduces_availability`, `report_separates_task_and_refusal_denominators`, `report_includes_runtime_recovery_and_failure_causes`, and `strict_runner_reports_missing_mujoco_as_infrastructure`; assert no dropped infrastructure jobs and zero safety invariant violations.
- [x] **Step 2: Run** `cargo test -p realityos-verify --test manipulation_benchmark score_campaign`; expect missing report/scoring implementation.
- [x] **Step 3: Implement** stratified metrics, Wilson 95% confidence intervals, per-embodiment/task/stratum slices, p50/p95 runtime, recovery, refusal correctness, acquisition/hold, infrastructure counts, and raw trace files. Make false acceptance, unauthorized writes, geometry omission, or stale-authority continuation fail the report.
- [ ] **Step 4: Add** the strict CLI subcommand, the 20-seed pilot and frozen public/holdout seed manifests from Task 1, the paired legacy safe-script baseline on identical resolved public trials, baseline source SHA `c38a52a4d8ace0e70e6d144b20b91ff4e0a738eb`, frozen candidate SHA/bundle metadata, and a reproducibility command for one recorded trial. Emit JSON and Markdown; any missing MuJoCo or worker failure exits nonzero and preserves an infrastructure record. **Partial:** the CLI, seed sets, candidate/bundle metadata, and replay exist. The full 20-seed pilot was started but interrupted after roughly 12 minutes on the Windows host. The paired baseline has not been run; its pinned source SHA is metadata only.
- [x] **Step 5: Add** a required MuJoCo CI job using `REALITYOS_REQUIRE_MUJOCO=1`, pinned MuJoCo 3.7.0, `MUJOCO_GL=disable`, and a small strict benchmark smoke campaign; assert scheduled equals completed plus infrastructure and infrastructure is zero.
- [ ] **Step 6: Run** `cargo test -p realityos-verify --test manipulation_benchmark`, then run the strict CLI with MuJoCo available and verify all scheduled public trials execute. Run `cargo test -p realityos-verify --all-targets` on the supported MuJoCo host. Do not run or inspect holdout scores before the candidate commit is frozen. **Partial:** all 37 benchmark integration tests pass. On frozen candidate `a5f9850f3cf7cb5524542a9c22254da344599ec1`, the two-seed development smoke completed 6/6 scheduled, zero infrastructure/invalid/unaccounted trials, safety gate passed, task success 0/2, correct refusal 4/4, and zero false acceptances, unauthorized writes, geometry omissions, or stale-authority continuations. A final one-seed smoke on the same candidate completed 3/3 with task success 0/2, correct refusal 1/1, and safety gate passed. After the code freeze, a one-trial WX250S holdout smoke completed 1/1 with safety gate passed, task success 0/1, and one false refusal; this is not the full 100-seed holdout score. Expired authority now halts simulator stepping at 10-step windows, and the probe witness decoder refuses to execute without its checked retreat. The full `--all-targets` sweep was interrupted after more than 13 minutes in the existing `development_physical_experiment_closes_the_loop` test; it did not produce a test result. The 20-seed pilot and paired legacy baseline also remain incomplete.

### Task 5: Multi-embodiment holdout and physical sensor evidence boundary

**Files:**
- Create: `crates/verify/src/manipulation_benchmark/physical_evidence.rs`
- Modify: `crates/verify/src/manipulation_benchmark/mod.rs`
- Modify: `crates/verify/src/observation.rs`
- Modify: `docs/superpowers/specs/2026-10-15-manipulation-reliability-generality-benchmark-design.md`
- Test: `crates/verify/tests/manipulation_benchmark.rs`

**Interfaces:**
- Consumes: benchmark role assignments, `SensorObservation`, `JointStateSample`, `ObservationFrame`, and `PolicyObservation`.
- Produces: `PhysicalSensorRequirement { sensor_id, device_serial, model_id, calibration_hash, frame, units, max_age_s }`, `PhysicalSensorIdentity { sensor_id, device_serial, model_id, calibration_hash, frame, units }`, `SensorOrigin::{Hardware, Simulation, Synthetic}`, `PhysicalSensorTrace { observation: ObservationFrame, sensors: Vec<PhysicalSensorIdentity>, origin: SensorOrigin }`, `PhysicalObservationEvidence`, `PhysicalEvidenceStatus`, and `validate_physical_observation(trace: &PhysicalSensorTrace, requirements: &[PhysicalSensorRequirement], now_s: f64) -> Result<PhysicalObservationEvidence, BenchmarkError>`.

- [x] **Step 1: Write failing tests** named `same_task_runs_on_both_development_embodiments`, `holdout_role_is_registered_without_early_scoring`, `unsupported_gripper_is_counted_as_refusal`, `physical_evidence_requires_fresh_sensor_and_calibration_provenance`, and `simulator_origin_never_becomes_measured`; assert shared task/API use, frozen role metadata, and rejection of simulator-origin data.
- [x] **Step 2: Run** `cargo test -p realityos-verify --test manipulation_benchmark same_task_runs_on_both_development_embodiments`; expect the cross-embodiment campaign and evidence boundary to be missing.
- [x] **Step 3: Implement** a frozen capability matrix with `menagerie_ur5e` and `menagerie_iiwa14` as development models and `menagerie_wx250s` as the untouched push/probe holdout. Use declarative end-effector mappings and include unsupported resource topologies in refusal denominators; do not branch planner or scoring behavior on robot identity. Keep direct two-finger grasp success trials on the qualified `arm_gripper_lift` fixture and score unsupported external grasp topology as refusal. CI exercises development roles; the strict `--holdout` run is available only after code, model-role manifest, thresholds, and acceptance rules are frozen.
- [x] **Step 4: Implement** physical evidence validation for sensor identity and serial/model, calibration hash, capture/receive timestamps, sequence/digest, frame, units, freshness, and missing required channels. Reject simulator clock/source markers and keep evidence `UNVERIFIED` until independent measurement provenance is attached; add no hardware write path.
- [x] **Step 5: Update** the approved spec's implementation status and record the physical cell dependency; leave the physical-transfer acceptance gate open until an identified cell runs a bounded sensor-derived experiment with independent measurement.
- [x] **Step 6: Run** `cargo test -p realityos-verify --test manipulation_benchmark` plus `cargo test -p realityos-reasoner-bridge` and `cargo test -p realityos-verify --test typed_grasp_verification --test production_mujoco_reasoner`; all selected simulation suites passed while physical evidence without a real sensor trace remains `UNVERIFIED`.

## Coverage Review

- Scenario randomization, labels, manifests, replay, and pilot freeze: Task 1 and Task 4.
- Mandatory MuJoCo, complete geometry, mesh assets, and pair rules: Task 2 and Task 4.
- Shared push/probe and pinch-grasp/lift bridge, valid refusals, zero writes, and recovery: Task 3.
- Metrics, baselines, confidence intervals, raw traces, strict command, and CI: Task 4.
- Development embodiments, untouched holdout, unsupported capability refusals, and physical sensor provenance: Task 5.
- Actual measured physical experiment: remains dependent on the user identifying the robot/test cell and supplying its sensor and independent measurement setup; do not mark that acceptance criterion complete from simulator tests.
