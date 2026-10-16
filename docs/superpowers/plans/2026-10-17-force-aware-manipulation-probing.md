# Force-Aware Manipulation Probing Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use `superpowers:executing-plans` to implement this plan task-by-task. The user selected inline work on the current local `main`; do not switch checkouts. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add provenance-carrying six-axis wrench observations, conservative force-based push-probe inference, and scoped force/torque stopping while keeping simulator evidence separate from physical evidence.

**Architecture:** Add a typed wrench sample and provenance validator, then bind a complete wrench policy into linear action authority and the proof cache. A pure estimator updates support-friction belief only from synchronized, bounded wrench and motion evidence. A simulation-only MuJoCo sensor adapter and per-embodiment policy exercise the same path; the strict pilot compares those trials to the existing 8 mm records by a schema-independent key.

**Tech Stack:** Rust workspace, serde/serde_json, SHA-256, MuJoCo Python worker, Cargo unit/integration tests, existing typed reasoner-authority bridge.

**Spec:** `docs/superpowers/specs/2026-10-17-force-aware-manipulation-probing-design.md`

## Global Constraints

- Work on the current local `main` because the user explicitly selected it; no alternate checkout.
- Keep every MuJoCo sample and derived belief labeled `SIMULATION`; neither can satisfy physical evidence validation.
- Do not add a hardware writer. Physical evidence remains `UNVERIFIED` until a trusted hardware adapter and independent outcome measurement exist.
- Wrench axes are ordered `[Fx,Fy,Fz,Tx,Ty,Tz]`; force caps use `[N]`, torque caps use `[N·m]`, and all six values bind the named sensor frame.
- Use `g = 9.80665 m/s²`, per-axis absolute error bounds, and conservative interval arithmetic; do not use covariance estimates.
- No action executes without finite positive wrench caps, a policy digest, fresh qualified monitoring, and a pre-authorized abort contract. No limit or monitor default is inferred.
- Safety monitoring uses the complete measured tool wrench, including unexpected contacts. Estimation receives no MuJoCo contact identity, material/friction field, or evaluator label.
- Keep the 8 mm stroke unchanged. Preserve complete geometry, collision proofs, lateral-drift checks, authority expiry, fresh-observation reauthorization, and zero unauthorized writes.
- Emit schema-v6 records and preserve schema-v5 reports as immutable history. Paired comparisons require an identical `paired_trial_key` for every scheduled record.
- Run the 20-seed pilot against the same 60 randomized scenarios as the 8 mm/16 mm pilot. Do not inspect holdout results before a clean candidate commit and passing public safety gate.

## Review Focus

- Contact-frame force sign, orientation, and moment arm: the adapter must produce the equal-and-opposite wrench on the instrumented tool and aggregate moments at the named sensor origin; Task 5 includes a force/moment balance test.
- A second or ambiguous contact: the stop monitor must include its load while the estimator refuses friction inference; Tasks 4–6 test each boundary separately.
- A missing, delayed, replayed, or stale sample: the monitor must invoke only the scoped abort within its latency bound and prevent further motion; Tasks 2 and 6 cover these inputs.
- Evidence lineage changes with unchanged friction bounds: the proof cache must invalidate when the measurement digest or origin changes; Task 4 tests the cache dependency.
- Existing v5 records and schema-v6 pairing: old reports remain readable, while a missing or mismatched pair key invalidates comparison instead of dropping a trial; Task 7 tests both cases.

---

### Task 1: Typed wrench observations and canonical sample digests

**Files:**
- Modify: `crates/semantics/src/sensor.rs`
- Modify: `crates/semantics/src/observation.rs`
- Modify: all `ObservationFrame` constructors in `crates/semantics/src/adapter.rs`, `crates/verify/src/manipulation.rs`, `crates/verify/src/reach_foundation.rs`, and tests
- Test: `crates/semantics/tests/wrench_observation.rs`

**Interfaces:**
- Consumes: existing `SensorClass`, `SensorObservation`, and `ObservationFrame`.
- Produces: `WrenchSampleOrigin::{Hardware, Simulation, Synthetic}` and `WrenchSample { sensor_id: String, sensor_frame: String, transform_epoch: String, units: String, wrench: [f64; 6], error_bounds: [f64; 6], capture_s: f64, receive_s: f64, sequence: u64, clock_domain: String, origin: WrenchSampleOrigin, digest: String }`; `wrench_sample_digest(sample: &WrenchSample) -> String`; and `ObservationFrame.wrench_samples: Vec<WrenchSample>` with serde default for old frames. Sample origin is included in the digest and must agree with the enclosing trace/measurement origin.

- [ ] **Step 1: Write failing tests** named `wrench_digest_binds_every_sample_field`, `invalid_wrench_or_error_bound_is_rejected`, `wrench_samples_require_monotonic_sequence_and_matching_epoch`, and `observation_frame_round_trips_wrench_samples`. Assert a change to any metadata, origin, value, or error bound changes the digest; reject non-finite values, negative error bounds, empty identities, wrong units, and stale/out-of-order samples.
- [ ] **Step 2: Run** `cargo test -p realityos-semantics --test wrench_observation`; expect missing `WrenchSample` or the new frame field.
- [ ] **Step 3: Implement** the struct, validator, and digest. Hash the domain tag `realityos.wrench_sample/1\0`, fixed-order length-prefixed UTF-8 strings including the stable origin name, little-endian finite `f64` bit patterns, and little-endian sequence; exclude only the digest field itself. Use fixed axis order `[Fx,Fy,Fz,Tx,Ty,Tz]`.
- [ ] **Step 4: Update** existing frame constructors with an empty wrench vector and retain deserialization of frames without the field.
- [ ] **Step 5: Run** `cargo test -p realityos-semantics --test wrench_observation`; expect all four tests to pass.
- [ ] **Step 6: Commit** as `feat(semantics): add typed wrench observations`.

### Task 2: Force/torque provenance validation

**Files:**
- Modify: `crates/semantics/src/observation.rs`
- Modify: `crates/verify/src/manipulation_benchmark/physical_evidence.rs`
- Modify: `crates/verify/tests/manipulation_benchmark.rs`

**Interfaces:**
- Consumes: Task 1 `WrenchSample` and current `PhysicalSensorTrace` validation.
- Produces: `PhysicalSensorRequirement` fields `class: SensorClass`, `min_sample_rate_hz: f64`, `max_inter_sample_gap_s: f64`, `max_age_s: f64`, and `max_cross_sensor_skew_s: f64`; `PhysicalSensorIdentity.class`; and optional `SensorObservation.device_serial`, `model_id`, and `units` fields, required for `ForceTorque` observations and cross-checked against the identity and requirement. Require each declared channel exactly once; support multiple declared force/torque channels with explicit cross-sensor skew checks.

- [ ] **Step 1: Write failing tests** named `force_torque_trace_requires_declared_class_and_identity`, `physical_wrench_trace_rejects_simulation_and_synthetic_origins`, `wrench_trace_rejects_bad_digest_or_replayed_sequence`, `wrench_trace_rejects_rate_gap_age_and_cross_sensor_skew`, and `valid_hardware_wrench_trace_stays_unverified_without_outcome_measurement`. Assert that valid sensor provenance alone never changes status from `UNVERIFIED`.
- [ ] **Step 2: Run** `cargo test -p realityos-verify --test manipulation_benchmark physical_evidence`; expect the new requirement fields and sample checks to be missing.
- [ ] **Step 3: Implement** validation requiring every declared channel to be matched exactly once, exact serial/model/calibration/frame/units identity, valid sample digests, increasing sequence and timestamps, age/gap/rate/skew bounds, `WrenchSampleOrigin::Hardware` on every wrench sample, and no simulation clock or non-hardware trace origin. Reject missing, duplicate, and undeclared channels. Hash ordered wrench sample digests into `PhysicalObservationEvidence.trace_digest`.
- [ ] **Step 4: Preserve** existing joint-encoder validation; non-wrench `SensorObservation` constructors set the optional force/torque identity fields to `None`.
- [ ] **Step 5: Run** `cargo test -p realityos-verify --test manipulation_benchmark physical_evidence`; expect all provenance tests to pass and simulator-origin traces to remain rejected.
- [ ] **Step 6: Commit** as `feat(verify): validate force torque evidence provenance`.

### Task 3: Wrench policy in exact linear-action authority

**Files:**
- Modify: `crates/kernel/src/physical_action.rs`
- Modify: `crates/semantics/src/physical_reasoner.rs`
- Modify: `crates/session/src/scoped_grant.rs`
- Modify: `crates/reasoner-bridge/src/lib.rs`
- Modify: `crates/reasoner-bridge/tests/bridge.rs`
- Test: kernel physical-action unit tests and bridge integration tests

**Interfaces:**
- Consumes: existing `LinearStroke`, `ReasoningRequest`, `PhysicalActionScope`, and `SimulationAdmission`.
- Produces: `WrenchLimitScope { sensor_id: String, sensor_frame: String, caps: [f64; 6], policy_digest: String, abort_contract_id: String }`; `WrenchAbortContract { contract_id: String, hold_mode: AbortHoldMode, allowed_stop_actuator_ids: Vec<String>, max_time_to_stop_s: f64, max_stop_writes: u32 }`, with `AbortHoldMode::HoldCurrentJointPosition` meaning capture the measured joint position at trip as the only hold target; and `WrenchLimitPolicy { scope: WrenchLimitScope, max_rise_rate: [f64; 6], max_sensing_latency_s: f64, max_abort_latency_s: f64, max_sample_age_s: f64, max_inter_sample_gap_s: f64, max_cross_sensor_skew_s: f64, monitor_id: String, compensation_digest: String, abort_contract: WrenchAbortContract }`.
- `WrenchLimitPolicy::digest(&self) -> String` hashes every policy field, excluding only `scope.policy_digest` to prevent recursive hashing; `validate(&self) -> Result<(), WrenchPolicyError>` rejects empty identities, non-positive/non-finite caps, negative/non-finite rates or timing values, mismatched abort IDs, and a scope digest mismatch.
- Extend `PhysicalActionEnvelope::LinearStroke` with `wrench_limit: Option<WrenchLimitScope>`; exact matching compares all six cap bit patterns and every identity/digest field. Preserve unscoped planning candidates if existing reasoner flows need them, but add `validate_for_execution()` that rejects a missing or invalid scope; grant issuance and execution must use that validation.
- Extend `ReasoningRequest` with `linear_wrench_limit: Option<WrenchLimitScope>`; `SimulationAdmission` carries a validated `WrenchLimitPolicy`. The bridge copies its scope into the request before selection and refuses an invalid policy before grant issuance.

- [ ] **Step 1: Write failing tests** named `linear_stroke_scope_binds_all_six_wrench_caps`, `linear_stroke_rejects_missing_or_invalid_wrench_scope`, `policy_digest_binds_sensor_latency_rise_rates_and_abort_contract`, `abort_contract_rejects_unregistered_stop_actuator`, `bridge_refuses_invalid_policy_before_grant`, and `grant_revalidation_rejects_changed_force_cap`. Assert a one-bit change in any cap or digest invalidates exact authority matching.
- [ ] **Step 2: Run** `cargo test -p realityos-kernel physical_action` and `cargo test -p realityos-reasoner-bridge --test bridge`; expect missing policy types or failing exact-scope checks.
- [ ] **Step 3: Implement** the policy and scope types in the kernel, add the scope to the linear envelope, then propagate the same scope through reasoner selection, prepared action, and scoped grant. Hash the policy with nested `scope.policy_digest` omitted, then verify that field equals the computed digest. Never construct a default policy.
- [ ] **Step 4: Update** every linear-action request and envelope fixture with an explicit test policy or an intentional missing-policy refusal case; leave pinch-grasp authority unchanged.
- [ ] **Step 5: Run** `cargo test -p realityos-kernel physical_action` and `cargo test -p realityos-reasoner-bridge --test bridge`; expect all exact-scope and refusal tests to pass.
- [ ] **Step 6: Commit** as `feat(authority): scope linear actions to wrench limits`.

### Task 4: Conservative probe inference and evidence lineage

**Files:**
- Create: `crates/semantics/src/push_probe.rs`
- Modify: `crates/semantics/src/lib.rs`
- Modify: `crates/semantics/src/physical_belief.rs`
- Modify: `crates/semantics/src/physical_reasoner.rs`
- Test: `crates/semantics/tests/push_probe_inference.rs`

**Interfaces:**
- Uses `WrenchEvidenceOrigin` as the semantics-layer `WrenchSampleOrigin` (or a lossless type alias), `WrenchTransform { rotation_to_support: [[f64; 3]; 3], translation_to_support_m: [f64; 3] }`, `ProbeMotionWindow { start_s: f64, end_s: f64, displacement_m: [f64; 3], max_speed_m_s: f64, pose_resolution_m: f64, first_motion_s: Option<f64> }`, and `WrenchCompensation { bias: [f64; 6], gravity_wrench: [f64; 6], residual_error: [f64; 6], digest: String }`.
- Produces `IntendedContactEvidence { observation_epoch: String, collision_model_digest: String, tool_body_id: String, target_body_id: String, plausible_contact_sets: Vec<Vec<String>> }` and `PushProbeMeasurement { samples: Vec<WrenchSample>, sensor_to_support: WrenchTransform, compensation: WrenchCompensation, mass_kg: [f64; 2], motion: ProbeMotionWindow, prior_support_friction: [f64; 2], intended_contact: IntendedContactEvidence, saturated: bool, origin: WrenchEvidenceOrigin }` and `FrictionInferenceResult::{Updated { interval: [f64; 2], mode: FrictionInferenceMode }, NoInference(FrictionNoInferenceReason)}`. Eligibility requires fresh matching observation/collision-model digests and exactly one plausible intended tool-to-target contact set; the estimator does not accept a raw boolean.
- Produces `infer_support_friction(measurement: &PushProbeMeasurement) -> FrictionInferenceResult`, `BeliefEvidenceReference { measurement_digest: String, origin: WrenchEvidenceOrigin }`, and `PhysicalParameterBelief::intersect_interval_with_evidence(parameter, interval, reference) -> Result<[f64; 2], BeliefUpdateError>`. Store structured evidence references in a new `#[serde(default)]` field on `ParameterBelief`; leave existing `BeliefLineage` semantics intact.

- [ ] **Step 1: Write failing tests** named `sticking_probe_adds_only_a_conservative_lower_bound`, `slip_onset_interval_intersects_prior`, `zero_or_uncertain_normal_load_returns_no_inference`, `unknown_mass_or_dynamic_sliding_returns_no_inference`, `alternate_contact_or_saturation_returns_no_inference`, and `measurement_origin_and_digest_are_appended_to_belief`. In the internal `physical_reasoner.rs` unit tests, add `probe_lineage_changes_proof_dependency_digest` because the dependency digest is private.
- [ ] **Step 2: Run** `cargo test -p realityos-semantics --test push_probe_inference`; expect the estimator and lineage API to be missing.
- [ ] **Step 3: Implement** rigid wrench transformation including `r × F`, action-reaction sign conversion, declared compensation, conservative per-axis interval propagation, `F_t = sqrt(F_x² + F_y²)`, `N = m*9.80665 - F_z`, and the spec's stick/slip formulas. Accept consistent `Hardware` or `Simulation` samples and preserve that origin on every derived belief; return typed no-inference for `Synthetic` or mismatched sample/measurement origins. Reject non-planar support, non-quasi-static windows, unknown mass, and empty prior intersections. Verify the typed contact evidence references the current observation epoch and collision-model digest and has exactly one plausible set for the intended tool/target pair.
- [ ] **Step 4: Implement** belief intersection only against an existing finite empirical prior interval; append the measurement digest and origin without changing the declared value. Include ordered structured evidence references in `proof_dependency_digest` so identical interval/status values from different evidence cannot share a cached proof. Test cache invalidation in `physical_reasoner.rs`'s internal unit tests, where the dependency digest is visible.
- [ ] **Step 5: Run** `cargo test -p realityos-semantics --test push_probe_inference`; expect conservative intervals and cache invalidation tests to pass.
- [ ] **Step 6: Commit** as `feat(semantics): infer support friction from bounded wrench evidence`.

### Task 5: Simulation-only wrench sensor profile and MuJoCo adapter

**Files:**
- Create: `crates/verify/src/manipulation_benchmark/wrench_sensor.rs`
- Modify: `crates/verify/src/manipulation_benchmark/mod.rs`
- Modify: `crates/verify/src/manipulation_benchmark/scenario.rs`
- Modify: `crates/verify/src/mujoco_exec.rs`
- Modify: `crates/verify/mujoco_worker.py`
- Modify: `robots/benchmarks/manipulation/embodiments.json`
- Test: `crates/verify/tests/manipulation_benchmark.rs`

**Interfaces:**
- Produces `SimulationWrenchSensorProfile { sensor_id: String, model_id: String, calibration_hash: String, sensor_frame: String, units: String, sample_rate_hz: f64, latency_s: f64, resolution: [f64; 6], bias: [f64; 6], error_bounds: [f64; 6], seeded_noise_bound: [f64; 6], limit_policy: WrenchLimitPolicy }` and `SimulationWrenchWindow { samples: Vec<WrenchSample>, max_gap_s: f64, compensation_digest: String }`. `seeded_noise_bound` bounds reproducible trial-seeded noise; each realized noise value must remain inside this bound and the reported sample error bound.
- Extend every push/probe `EmbodimentRegistration` with an explicit simulation profile and policy; every generated sample is stamped `WrenchSampleOrigin::Simulation`. The pinch-grasp fixture may omit the push profile. Bump embodiment registry schema from 4 to 5. No inherited values or profile defaults.
- Add `MujocoInstance::step_with_wrench_profile(n: u32, profile: &SimulationWrenchSensorProfile) -> Result<Value, ExecError>` over the worker JSON protocol; return one simulated sensor sample at each declared sampling instant.

- [ ] **Step 1: Write failing tests** named `simulation_wrench_profile_requires_all_six_limits_and_sensor_fields`, `contact_wrench_transform_matches_force_and_moment_balance`, `contact_force_sign_matches_geom_order`, `simulation_wrench_samples_replay_for_same_trial_seed`, and `sensor_adapter_aggregates_unexpected_tool_contacts`.
- [ ] **Step 2: Run** `cargo test -p realityos-verify --test manipulation_benchmark simulation_wrench`; expect missing profile parsing or adapter output.
- [ ] **Step 3: Implement** the profile parser, explicit registered values for each push embodiment, profile digest validation, and deterministic trial-seeded noise bounded by `seeded_noise_bound`. Reject a policy whose declared sensor rate, max gap, or latency cannot be represented by the loaded model timestep.
- [ ] **Step 4: Implement** worker contact-wrench aggregation at every MuJoCo integration step: call `mj_contactForce`, rotate its contact-frame force/torque into world coordinates, apply equal-and-opposite sign for the instrumented body, shift each moment to the named sensor origin, and sum all loads on the instrumented tool. Keep contact pair identities internal to the adapter; emit only typed `Simulation` sensor samples to Rust. Include sensor bias, bounded error, rate, delay, resolution, and seeded noise; never read object friction/material labels.
- [ ] **Step 5: Run** `cargo test -p realityos-verify --test manipulation_benchmark simulation_wrench`; expect force/moment balance, sign, full-load, and deterministic replay tests to pass.
- [ ] **Step 6: Commit** as `feat(verify): simulate provenance-bound wrench sensing`.

The worker conversion follows MuJoCo's `mj_contactForce` API, which returns a six-dimensional wrench in the contact frame; its contact convention defines the normal from geom 1 toward geom 2. The implementation tests the sign and frame conversion rather than relying on a presumed convention: [MuJoCo API reference](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html), [MuJoCo contact direction convention](https://mujoco.readthedocs.io/en/latest/XMLreference.html).

### Task 6: Per-step force monitor, authorized abort, and trial audit

**Files:**
- Modify: `crates/verify/src/mujoco_exec.rs`
- Modify: `crates/verify/mujoco_worker.py`
- Modify: `crates/verify/src/manipulation_benchmark/execution.rs`
- Modify: `crates/verify/src/manipulation_benchmark/report.rs`
- Modify: `crates/verify/tests/manipulation_benchmark.rs`
- Modify: `crates/verify/tests/production_mujoco_reasoner.rs`

**Interfaces:**
- Produces `WrenchAbortReason::{Limit { axis: usize, measured_upper_bound: f64, cap: f64 }, MissingSample, StaleSample, GapExceeded, DigestMismatch, MonitorFault}`, `WrenchAbortEvent { reason: WrenchAbortReason, triggered_at_s: f64, stopped_at_s: f64, stop_write_count: u32, contract_id: String }`, and `MonitoredStepResult { state: Value, wrench_window: SimulationWrenchWindow, abort: Option<WrenchAbortEvent> }`. Telemetry faults carry a typed reason without fabricating a force-axis measurement.
- Add `MujocoInstance::step_monitored(n: u32, profile: &SimulationWrenchSensorProfile, policy: &WrenchLimitPolicy, grant: &ScopedSimulationGrant) -> Result<MonitoredStepResult, ExecError>`; Rust validates that policy, grant, and action scope are exactly equal before invoking the worker. The worker checks every physics step, enforces `abs(measured_i) + error_i + max_rise_rate_i * (max_sensing_latency_s + max_abort_latency_s) >= caps[i]`, and may write only the registered actuator IDs to hold their measured joint positions at trip through the exact pre-authorized abort contract.
- Extend `BenchmarkEpisode` with serde-defaulted fields for optional paired key, sensor/compensation digests, sample count, age/gap extrema, six peak conservative wrench bounds, six caps, minimum margin, typed trip reason/axis, trip/stop timestamps, abort result, inference interval/reason, and prior/posterior friction interval. Defaults preserve reading of historical v5 episodes; they do not authorize v6 execution.

- [ ] **Step 1: Write failing tests** named `force_monitor_trips_each_of_six_axes_before_cap`, `force_monitor_uses_absolute_values_for_both_signs`, `wrench_threshold_uses_error_and_latency_rise_margin`, `missing_or_late_wrench_sample_uses_pre_authorized_abort`, `abort_contract_is_the_only_stop_write`, and `no_motion_or_reauthorization_follows_unverified_stop`.
- [ ] **Step 2: Run** `REALITYOS_REQUIRE_MUJOCO=1 MUJOCO_GL=disable cargo test -p realityos-verify --test manipulation_benchmark force_monitor -- --test-threads=1`; expect the worker guard and event fields to be missing.
- [ ] **Step 3: Implement** monitored stepping in the Python worker. It evaluates the total sensor wrench every `mj_step`, applies the pre-authorized hold contract immediately when any axis reaches the conservative predictive threshold, stops the batch, and returns an event whose contract ID matches the grant and whose stop-write count is within the contract. Rust refuses to call a further motion step after a trip; if stop verification fails or write accounting is inconsistent, latch a fault and refuse reauthorization.
- [ ] **Step 4: Integrate** the typed sample window into `execute_push_witness`; pass only the validated action grant's exact policy, bind each measurement to the current observation epoch, update belief through Task 4, then rebuild observation and request a fresh grant before any follow-up.
- [ ] **Step 5: Record** all force, timing, freshness, inference, abort, grant, and write fields in `BenchmarkEpisode`; evaluator truth and contact identities remain absent from `ReasoningRequest`.
- [ ] **Step 6: Run** `REALITYOS_REQUIRE_MUJOCO=1 MUJOCO_GL=disable cargo test -p realityos-verify --test manipulation_benchmark -- --test-threads=1` and `REALITYOS_REQUIRE_MUJOCO=1 MUJOCO_GL=disable cargo test -p realityos-verify --test production_mujoco_reasoner -- --test-threads=1`; expect all tests to pass with zero unauthorized writes and no post-abort motion.
- [ ] **Step 7: Commit** as `feat(verify): enforce wrench limits during push probes`.

### Task 7: Schema-v6 pairing, report metrics, and frozen pilot

**Files:**
- Modify: `crates/verify/src/manipulation_benchmark/scenario.rs`
- Modify: `crates/verify/src/manipulation_benchmark/report.rs`
- Modify: `crates/verify/src/manipulation_benchmark/mod.rs`
- Modify: `crates/verify/src/bin/realityos-verify.rs`
- Modify: `crates/verify/tests/manipulation_benchmark.rs`
- Modify: `docs/superpowers/specs/2026-10-17-force-aware-manipulation-probing-design.md`
- Create: `docs/superpowers/evidence/force-aware-probe-<candidate-short>/`

**Interfaces:**
- Bump `BENCHMARK_SCHEMA_VERSION` from 5 to 6 and registry schema from 4 to 5. Add `paired_trial_key: Option<String>` to `ResolvedBenchmarkTrial` with `#[serde(default)]`; v6 resolved trials must contain a key, while existing v5 records remain deserializable. Compute the key as SHA-256 over canonical role, robot ID, model hash, task, seed, scenario index/stratum, expectation, normalized resolved scenario, exact goal, and initial `qpos`, excluding record schema, candidate SHA, sensor trace, action trace, and outcome.
- Produce `paired_campaign_delta(baseline: &[BenchmarkTrialRecord], candidate: &[BenchmarkTrialRecord]) -> Result<PairedCampaignDelta, BenchmarkError>`; require identical, unique key sets and report paired changes in expected-success rate, information-bearing probe rate, and justified-follow-up rate with 95% paired bootstrap intervals from 10,000 resamples seeded by the scenario-manifest digest. Define each rate per embodiment over all scheduled expected-success `PushProbeReplan` keys, including trials where no probe or follow-up was attempted; information-bearing means a valid strict friction-interval narrowing, and justified follow-up means a fresh goal-action grant after that valid probe. Report all numerators and denominators.
- Add a schema-aware v5 compatibility reader: new fields use explicit `#[serde(default)]` or an adapter so existing v5 reports deserialize unchanged. Legacy trials have no persisted paired key; compute a transient key from their stored resolved inputs without modifying the artifact. Require a persisted key for v6 execution/scoring inputs.
- Extend `BenchmarkReport` with serde-defaulted per-trial wrench events and aggregate inference, cap-trip, stale/missing-sample, and paired-delta metrics. Continue reading and reporting stored v5 records without upgrading their provenance or execution schema.

- [ ] **Step 1: Write failing tests** named `paired_trial_key_ignores_schema_but_binds_randomized_inputs`, `v5_records_remain_readable_without_wrench_claims`, `paired_comparison_rejects_missing_or_mismatched_keys`, `paired_delta_bootstrap_is_deterministic`, and `report_includes_wrench_inference_and_abort_metrics`.
- [ ] **Step 2: Run** `cargo test -p realityos-verify --test manipulation_benchmark paired_trial_key`; expect the schema-v6 key and paired scorer to be missing.
- [ ] **Step 3: Implement** canonical schema-independent key generation and strict paired scoring. Load the existing 8 mm pilot records from `docs/superpowers/evidence/probe-stroke-paired-198b5de/probe8mm/manipulation_benchmark.json`; recompute transient keys from the stored v5 resolved inputs without modifying the artifact, and fail the comparison if any of its 60 trial keys cannot be paired exactly.
- [ ] **Step 4: Implement** report serialization/rendering for sample provenance, friction intervals, cap trips, abort behavior, paired denominators, and confidence intervals. Keep v5 records historical and readable; reject any attempt to use them as v6 execution input.
- [ ] **Step 5: Run** `cargo test -p realityos-verify --test manipulation_benchmark paired_trial_key` and `cargo test -p realityos-verify --test manipulation_benchmark -- --test-threads=1`; expect v5 compatibility and all report tests to pass.
- [ ] **Step 6: Run full local checks**: `cargo fmt --all -- --check`; `cargo test -p realityos-semantics`; `cargo test -p realityos-kernel`; `cargo test -p realityos-session`; `cargo test -p realityos-reasoner-bridge`; `REALITYOS_REQUIRE_MUJOCO=1 MUJOCO_GL=disable cargo test -p realityos-verify --all-targets -- --test-threads=1`; and `cargo clippy --workspace --all-targets -- -D warnings`. The required MuJoCo environment makes unavailable simulator tests fail rather than skip.
- [ ] **Step 7: Commit the candidate** with all code and policy manifests, confirm clean `main`, then run `REALITYOS_REQUIRE_MUJOCO=1 MUJOCO_GL=disable cargo run -p realityos-verify -- manipulation-benchmark --pilot --seeds 20 --out docs/superpowers/evidence/force-aware-probe-<candidate-short>`; expect 60/60 scheduled trials, zero infrastructure/invalid/unaccounted records, and zero false acceptances, unauthorized writes, geometry omissions, or stale-authority continuations.
- [ ] **Step 8: Compare** every pilot trial against the schema-v5 8 mm record by exact `paired_trial_key`. Compute availability measures per embodiment over all scheduled expected-success push trials, including trials with no probe/follow-up; use the same paired keys in each candidate-baseline delta. The candidate-screening gate requires at least a 10-percentage-point point-estimate gain in one declared availability measure and at most a 5-point regression in every other measure; record paired 95% intervals and all denominators. If the gate fails, report it without claiming improvement and do not run or inspect holdout results.
- [ ] **Step 9: Update** the spec with the pilot SHA, paired outcome, limitations, and physical-transfer status; commit evidence and documentation, then push `main` once the full evidence set is reviewed.

## Coverage Review

- Sample schema, canonical digest, frame freshness, identity, sensor class, calibration, units, and hardware/simulation/synthetic boundary: Tasks 1–2.
- Exact caps, policy digest, monitor latencies, stop contract, and grant equality: Task 3.
- Conservative stick/slip formulas, prior intersection, evidence origin, lineage, and proof-cache dependencies: Task 4.
- Complete contact wrench, frame/sign/moment conversion, declared sensor rate/delay/noise, and no contact-ID leakage: Task 5.
- Per-step safety stop, abort timing, no stale-authority continuation, and trial audit: Task 6.
- Schema v6, v5 immutability, exact paired 60-trial comparison, strict campaign accounting, and no premature holdout: Task 7.
- The physical-transfer gate remains open pending a named cell, real sensor adapter, and independent outcome measurement; no simulation task can close it.
