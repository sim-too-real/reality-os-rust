# Manipulation Reliability and Generality Benchmark Design

**Date:** 2026-10-15

**Status:** approved; implementation advances the benchmark, while baseline, full campaign, holdout, and physical-transfer gates remain open

**Implementation status (2026-10-16):** The strict MuJoCo runner, complete-geometry admission, shared bridge campaign, report/CLI, development embodiment registry, physical sensor-provenance validator, collision-aware planning, checked probe retreat, bounded authority-expiry handling, and exact finite tabletop footprint checks are implemented. The reasoner and authorization bridge now accept a set of excluded action keys; after an executed push probe yields no bounded information gain, the runner excludes that exact action, refreshes observations, then lets the shared selector choose another justified action or refuse. A regression test exercises this path on reachable geometry. The fresh strict 20-seed development pilot on dirty worktree base `8a2adce4035997909392b6ed182c071ffdb6ecd3` completed all 60 scheduled trials with zero infrastructure, invalid, or unaccounted records and passed the hard safety gate: task success 0/16 (95% Wilson interval 0–0.194), correct refusal 44/44 (0.920–1.000), 14 false refusals, zero false acceptances, zero unauthorized writes, zero geometry omissions, and zero stale-authority continuations. Positive push trials were refused before a probe because complete-geometry checks found support collision, collision necessary-condition, self-collision, or IK residual blockers, so the new probe-exclusion path was not exercised by this randomized pilot. The two eligible positive grasp trials both reached bilateral acquisition, but neither passed hold verification; the independent verifier measured a per-finger contact force above the authorized 1.95 N ceiling. Median/p95 wall time was 6.65/9.47 seconds. The pilot is not a frozen candidate result because the worktree was dirty. The paired legacy safe-script baseline, 100-seed public run, current-candidate 100-seed holdout, and physical experiment remain open. The full strict `realityos-verify --all-targets` regression sweep and Clippy check have since passed. The previous 100-trial WX250S score pertains to an earlier frozen candidate and is not evidence for this implementation. Physical evidence remains `UNVERIFIED` until an identified robot cell supplies real sensor data and independent outcome measurements.

**Holdout integrity note (2026-10-16):** WX250S was scored in an earlier 100-trial holdout run, and the present development work used its geometry and seed `10000`, index `10` to investigate a colliding initial configuration. The full holdout seed range is also present in this repository. Its existing `holdout` registration is therefore historical compatibility; any new WX250S score is a known-model regression result, not an untouched holdout. A fresh model bundle and a seed set kept private until candidate freeze are required for an independent model-generalization claim.

**Evidence characterization update (2026-10-16):** A later 300-trial strict public run at candidate `4e46c570cf45eec0e31e03702dffbd70930ee3b9` completed but failed the safety gate with 12 false acceptances; the benchmark evaluator blocked every one before a control write, so unauthorized writes remained zero. The follow-up adds typed, provenance-carrying fault alerts to the reasoner and regression cases for those failures. MuJoCo trials exercise response to alerts deliberately supplied by the fault-injection adapter; they do not measure autonomous fault detection. Grasp friction samples are derived from the configured simulator friction with seeded noise and are recorded as simulator-oracle inputs, not tactile or sensor measurements.

**Scoring correction (2026-10-16):** The 300-trial run on candidate `06cefd55cd2066525b7d403ce152ff59a5980219` completed but reported four false acceptances and one stale-authority continuation. Review traced two false-acceptance cases to unexpected-contact and slip negatives that lacked typed fault observations, and two to fault-free ambiguous push cases that the shared selector safely treated as probe-capable. The scenario resolver now places ambiguous cases without an explicit expected-refusal cause in the expected-success cohort; explicit negative fault cases remain expected refusals and receive typed, simulator-derived contact/slip alerts. The continuation metric now treats a post-divergence action as stale only if it lacks reauthorization or reuses the prior grant or observation epoch. The corrected candidate must pass a fresh strict public campaign before any result is accepted; no holdout scoring is authorized by this update.

**Frozen candidate score (2026-10-16):** Commit `9ad56f8239449e764bbf541f752a6852ebf2b0aa` passed the strict MuJoCo public campaign (300/300 scheduled; zero infrastructure, invalid, or unaccounted trials). The safety gate passed with zero false acceptances, unauthorized writes, geometry omissions, and stale-authority continuations. Task success was 0/113 (95% Wilson interval 0–0.0329), correct refusal was 187/187 (0.9799–1.000), false refusals were 78, recovery was 0/4, and wall time p50/p95 was 6.00/8.14 seconds. All 78 expected-success push trials refused before action selection with `NO_JUSTIFIED_CURRENT_ACTION`; their candidate action proofs were blocked by support collision, collision necessary-condition, IK residual, or self-collision checks. All 35 expected-success grasp trials acquired the object, but none passed hold verification: 32 exceeded the authorized per-finger force ceiling and 3 slipped. The same frozen candidate passed the 100-trial WX250S holdout safety gate (100/100; zero infrastructure, invalid, or unaccounted trials; zero false acceptances, unauthorized writes, geometry omissions, and stale-authority continuations). Holdout task success was 0/39 (95% Wilson interval 0–0.0897), correct refusal was 61/61 (0.9408–1.000), false refusals were 39, and recovery was 0/2; all 39 positive holdout trials refused because ten loaded collision geoms could not be lowered into supported semantic geometry. These results establish safe refusal on this simulation protocol, not useful task execution or superiority: the paired legacy baseline remains unexecuted, and physical transfer remains `UNVERIFIED` pending real sensor data and independent measurements. The next capability work should make collision-geometry admission complete for the holdout embodiment, resolve the development push proof blockers without relaxing safety invariants, and make grasp closure/force control land inside the already authorized force envelope.

**Repository baseline:** `c38a52a4d8ace0e70e6d144b20b91ff4e0a738eb` (`main`)

**Evidence boundary:** MuJoCo episodes are `SIMULATION_ONLY`. External episodes may be labeled measured only when their policy inputs are traceable to actual sensors and evaluation truth comes from independent measurement.

For simulation-only grasp cases, friction inputs come from the scenario's MuJoCo material configuration and carry seeded noise; they are oracle inputs to test reasoning with known simulator parameters. They are not tactile measurements and cannot establish sensing or autonomous slip-detection capability. Similarly, injected safety-fault alerts test safe reasoner response to an explicitly supplied alert. Any physical sensing claim requires the separate hardware-origin observation and independent-measurement track below.

## Goal

Create one reproducible manipulation benchmark that demonstrates what Reality OS can safely accomplish, what it correctly refuses, how it recovers, and whether the same reasoning-and-authority API transfers across embodiments. The benchmark must be falsifiable: unavailable simulation, incomplete geometry, missing observations, infrastructure failure, and unsupported cases cannot silently become passes or disappear from denominators.

This is an evaluation milestone, not a claim that Reality OS will be impossible to compete with. Competitive claims require published results against fixed baselines and independent holdouts.

## Current repository evidence

- `crates/verify/tests/production_mujoco_reasoner.rs` calls `ensure_mujoco_or_skip()` and returns when MuJoCo is unavailable under the default test environment. The runtime already has a `REALITYOS_REQUIRE_MUJOCO` fail-fast switch; the benchmark must set it and verify every scheduled trial ran. The same integration tests clear `model.collision_geoms` before giving the model to the reasoner. MuJoCo still simulates its loaded model, but the reasoner's collision proof no longer sees the full semantic geometry.
- `crates/verify/src/manipulation_scenarios.rs` already generates deterministic variation in object pose, size, mass, and friction, plus obstacles in selected negative cases. `crates/verify/src/scenario.rs` has a separate generic resolver. Neither currently supplies a single frozen campaign that combines randomized obstacle layouts and initial robot configurations with expected-outcome labels through the new typed bridge.
- `EpisodeEvidence` already records robot and model identity, simulator version, seed, action decisions, violations, writes, termination, simulation time, and wall-clock time. A benchmark wrapper can build on this record while adding expected-outcome labels, refusal correctness, geometry integrity, and aggregate statistics.
- `crates/verify/src/observation.rs` constructs simulator policy observations from `VerifierTruth` (including a mode explicitly named perfect perception from simulation truth). The actual-sensor physical track therefore needs a separate observation adapter and evidence provenance; simulator observations cannot satisfy it.
- Existing holdout and generality work demonstrates useful freeze-SHA and provenance patterns, but the manipulation simulator integration and external robot holdouts are not yet one strict, full-geometry campaign over the same typed bridge.
- The last implementation added the shared `PhysicalActionEnvelope`, exact scoped authority for push/probe and direct two-finger pinch grasp, and simulation-only post-lift verification. Those are the first benchmarked action families.

## Recommended architecture

Add a benchmark campaign module and strict command under `realityos-verify`, reusing the existing scenario resolver, MuJoCo worker, `authorize_current_action`, and privileged outcome verifier. Do not make a second planner or an external script that reconstructs plans, grants, or action envelopes. Extend the shared scenario and evidence interfaces only where the benchmark needs typed randomized fields and labels.

The campaign has two linked tracks with separate evidence and scorecards:

1. **Simulation reliability and generality:** mandatory MuJoCo execution with complete loaded and semantic collision geometry, recorded seeded randomization, repeated runs, and frozen robot holdouts.
2. **External physical transfer:** a bounded real-robot experiment whose policy input is the actual timestamped sensor stream. The benchmark records its sensor provenance and independently measured outcome. This track cannot be marked complete from simulator data, synthetic observations, or simulator privileged state.

The physical track needs an identified robot, cell, sensors, calibration procedure, and operator safety process. No hardware trial is inferred from the simulation authorization introduced in the prior milestone.

## Benchmark task set

Version 1 covers the manipulation capabilities that already pass through the shared authority bridge:

- push/probe/replan toward a declared object goal, when its mechanics and collision proof support the action;
- qualified direct two-finger pinch grasp followed by the bounded vertical lift and independent hold verification.

Use identical task definitions and success/refusal rules across compatible embodiments. The benchmark may use embodiment-specific declarative resource mappings, but policy or authority selection must not branch on robot names, benchmark seed, or fixture identity. Unsupported resource topologies remain in the refusal set; they are not silently excluded.

Each trial is labeled before execution as one of:

- **Expected success:** inputs and constraints establish an admissible task, so the system should authorize a proved action and achieve the measured objective. A fault-free ambiguous case with no explicit expected-refusal cause belongs here because a safe, scoped probe is an admissible way to resolve uncertainty.
- **Expected refusal:** evidence is unknown, stale, malformed, unsupported, unsafe, out of bounds, or the requested action lacks a valid proof; the system must refuse before unauthorized control writes.

Scenario labels and evaluator-only state are not visible to the reasoner. A refusal on an expected-success trial counts as a false refusal and reduces task availability. Authorization on an expected-refusal trial counts as a false acceptance and fails the safety gate.

## Scenario generation and reproducibility

Resolve every trial from a versioned scenario specification and an explicit seed. The resolved record must include the exact sampled values and resulting scene, not only the seed. Randomize within predeclared, physically valid ranges:

- object pose, shape/size where supported, mass, and surface friction;
- obstacle count, geometry, pose, and clearance from the task path;
- initial robot joint configuration within declared limits;
- sensor delay, dropout, stale epoch, and missing or unknown required parameters for refusal trials;
- selected contact, authority, expiry, and runtime-divergence fault injections.

Fault and sensor-like variations in simulation are explicit inputs. Score them as response-to-injected-condition cases; do not interpret them as evidence that a perception, tactile, or health-monitoring system autonomously detected the condition.

Use separate public regression seeds and a frozen holdout seed set. Freeze code, bundles, parameter ranges, trial labels, and acceptance rules before opening holdout results. Publish the holdout seeds and resolved episodes after scoring so every reported result can be reproduced. Choose the final sample size from a pilot and a predeclared uncertainty target; do not change it after seeing holdout outcomes.

Every report includes repository SHA, benchmark schema/version, policy/API version, robot bundle and all asset checksums, MuJoCo version, machine/runtime information, scenario-spec hash, seed, and complete resolved parameters. Repeating a seed with the same declared versions must produce the same resolved input and an auditable episode trace.

## Geometry and simulator integrity gates

- The strict benchmark entry point requires MuJoCo and fails with an infrastructure error when it is unavailable. It never returns success or skips the trial because a dependency is missing.
- The CI job for the strict benchmark runs on a supported MuJoCo host. Ordinary unit tests may remain simulator-independent, but they are not substitutes for this gate.
- Load the full model and assets used by MuJoCo and the reasoner. Do not clear collision geometry to simplify planning. Record a canonical geometry manifest and hash, including collision-enabled geoms, their shape and dimensions or mesh asset hash, body attachment, pose, contact masks, and relevant contact-pair rules.
- Assert that semantic collision geometry corresponds to the loaded simulator geometry for all benchmark-relevant bodies. Any unaccounted collision-enabled geom invalidates the run.
- Infrastructure errors, invalid manifests, or failed geometry checks invalidate a benchmark run. They are reported separately and are never counted as valid refusal or robot failure.

## Metrics and scoring

Report per embodiment, task family, scenario stratum, and aggregate:

- eligible-task success rate and confidence interval;
- expected-refusal rate, correct refusal rate, false refusal rate, and false acceptance rate;
- pre-execution refusal writes and total unauthorized writes (both must equal zero);
- recovery rate after an injected disturbance, including whether the old frozen action stopped and fresh observations/replanning preceded any continuation;
- grasp acquisition and verified hold as separate outcomes;
- wall-clock runtime percentiles and simulation time, reported separately;
- failure counts by a versioned cause taxonomy: perception/evidence, unsupported capability, planning/proof, collision, authority, execution divergence, recovery, task miss, verifier, and infrastructure;
- raw per-trial records so aggregate scores cannot hide a dangerous or systematic failure.

Safety invariants are hard gates, not weighted score components: any unauthorized write, false acceptance in a must-refuse trial, collision-geometry omission, or stale-authority continuation fails the benchmark regardless of task success. Task success is computed only over pre-labeled expected-success trials, while refusal quality is computed over expected-refusal trials. This prevents an always-refuse policy from appearing strong.

After runtime divergence, continuation is valid only after a fresh authorization under a different grant and observation epoch. The report counts missing reauthorization and reuse of either the prior grant or prior observation epoch as stale-authority continuation; a correctly refreshed action is not a violation.

Compare the candidate with a frozen prior Reality OS revision and a clearly described safe scripted baseline on identical public seeds. A holdout score must be produced after the implementation SHA is frozen. Report confidence intervals and denominators; do not announce superiority from raw pass counts alone.

## Multi-embodiment protocol

Run the same bridge API and benchmark task definitions on at least two development embodiments with qualified resources and one untouched holdout embodiment. The holdout model and seed set are not used for tuning. The model loader may map physical names through each robot's declarative manifest; planner, authorization, verifier, and scoring behavior stay shared.

Record every embodiment-specific conditional and fixture-derived value. Any task-specific robot-name branch, omitted geometry, manually reconstructed grant, or holdout-tuned threshold invalidates the generality claim. Embodiment-specific resource limits may come from the frozen robot manifest; the task-level outcome and safety rules remain shared. Unsupported topology is scored under expected refusal rather than removed from the report.

## External physical experiment protocol

Before claiming physical transfer, run at least one bounded task from the benchmark on an identified external robot. The reasoner consumes only that robot's actual observations: sensor source and serial/model identity, timestamp, calibration version, frame, units, freshness, and missing-data status are recorded with the decision. Simulator state, fixture constants, and evaluator-only measurement cannot enter policy observations or authority scope.

Use independent measurement for the result (for example, calibrated external pose tracking and a separate object/force measurement appropriate to the task). Preserve raw sensor and reference traces, clock alignment, calibration records, all action-authority decisions, and operator interventions. Report every attempt, including refusals and aborted trials. The cell owner must define and follow the applicable risk controls and emergency-stop procedure before energizing the robot; the benchmark itself does not certify functional safety. ISO 10218-1 addresses industrial robot safety requirements, with integration and application requirements covered separately by ISO 10218-2.

Physical results are labeled `MEASURED` only when acquisition provenance and independent ground truth are present and reviewed; otherwise they remain `UNVERIFIED` and cannot substantiate physical transfer. The user has not yet identified the target robot or test cell, so that setup remains an explicit dependency for the physical track.

## Deliverables

- a versioned benchmark specification and machine-readable scenario/seed manifest;
- a strict command and CI job that cannot silently skip MuJoCo;
- randomized push/probe and grasp/lift trials on complete models;
- a structured per-trial record and aggregate report with refusal, recovery, runtime, geometry, and failure-cause metrics;
- public regression scores and a frozen multi-embodiment holdout score;
- a physical-transfer protocol and sensor-evidence adapter, followed by an actual measured trial on the identified cell;
- a reproducibility command that replays a recorded trial from its bundle hashes and resolved scenario.

## Acceptance criteria

1. The strict campaign fails, rather than skips, when MuJoCo is unavailable; the designated CI campaign must execute all required trials.
2. The geometry gate proves that no benchmark-relevant collision geom or asset was removed between loaded model and reasoning input.
3. All generated scenarios and trial labels replay exactly from the published manifest, version identifiers, and seeds.
4. Both expected-success and expected-refusal cases appear in every scorecard, with complete denominators and per-trial traces.
5. Zero unauthorized writes, zero false acceptance on the safety refusal set, and zero continuation of stale or divergent authority are mandatory.
6. Success, refusal correctness, acquisition, verified hold, recovery, runtime, and failure causes are independently reported.
7. The same typed reasoner-authority API runs on at least two qualified development embodiments and one frozen holdout without task-specific robot branches or tuned per-robot thresholds.
8. The physical-transfer gate remains open until an actual sensor-derived experiment is run with independent measurement and complete provenance on an identified robot/test cell.

## Out of scope

- claiming the system is unbeatable or production-certified from one benchmark;
- replacing the existing reasoner with a benchmark-specific planner;
- broad new robot-control features not needed to execute and measure these tasks;
- counting MuJoCo truth as a real sensor observation;
- actuating an unspecified physical robot or treating a simulated grant as hardware authorization.

## References

- [MuJoCo XML reference](https://mujoco.readthedocs.io/en/3.13.0/XMLreference.html) for model geometry and contact-pair configuration.
- [NIST Robotics Test Facility](https://www.nist.gov/laboratories/tools-instruments/robotics-test-facility) for performance measurement using repeatable task artifacts and defined metrics.
- [NIST grasping metrics and test methods](https://www.nist.gov/el/intelligent-systems-division-73500/robotic-grasping-and-manipulation-assembly/grasping) for independent measurement in comparative manipulation tests.
- [ISO 10218-1:2025](https://www.iso.org/standard/73933.html); robot integration and application requirements are addressed in ISO 10218-2:2025.
