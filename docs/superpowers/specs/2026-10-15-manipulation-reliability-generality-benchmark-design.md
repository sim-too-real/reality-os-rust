# Manipulation Reliability and Generality Benchmark Design

**Date:** 2026-10-15

**Status:** approved; benchmark implementation complete with baseline, full holdout, and physical-transfer gates open

**Implementation status:** The strict MuJoCo runner, complete-geometry admission, shared bridge campaign, report/CLI, development embodiment registry, physical sensor-provenance validator, collision-aware manipulation planning, checked probe retreat, and bounded authority-expiry handling are implemented. Candidate `a5f9850f3cf7cb5524542a9c22254da344599ec1` is the frozen implementation commit. Its two-seed development smoke completed 6/6 trials with zero infrastructure, invalid, or unaccounted trials and passed the safety gate; task success was 0/2 and correct refusal was 4/4, with two false refusals and no false acceptances or unauthorized writes. A final one-seed smoke on this commit completed 3/3 with task success 0/2 and correct refusal 1/1. A one-trial post-freeze WX250S holdout smoke completed 1/1 with the safety gate passed, task success 0/1, and one false refusal; this is not the planned 100-seed holdout score. Positive grasp labels require the same conservative worst-case hold-force proof used by the reasoner. The 20-seed pilot was started but interrupted after roughly 12 minutes on the Windows host. The paired legacy baseline has not been executed, and the full `--all-targets` test sweep did not finish on the Windows host. The full baseline, pilot, and holdout scorecard remain open. Physical evidence remains `UNVERIFIED` until an identified robot cell supplies real sensor data and independent outcome measurements.

**Repository baseline:** `c38a52a4d8ace0e70e6d144b20b91ff4e0a738eb` (`main`)

**Evidence boundary:** MuJoCo episodes are `SIMULATION_ONLY`. External episodes may be labeled measured only when their policy inputs are traceable to actual sensors and evaluation truth comes from independent measurement.

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

- **Expected success:** inputs and constraints establish an admissible task, so the system should authorize a proved action and achieve the measured objective.
- **Expected refusal:** evidence is unknown, stale, malformed, unsupported, unsafe, out of bounds, or the requested action lacks a valid proof; the system must refuse before unauthorized control writes.

Scenario labels and evaluator-only state are not visible to the reasoner. A refusal on an expected-success trial counts as a false refusal and reduces task availability. Authorization on an expected-refusal trial counts as a false acceptance and fails the safety gate.

## Scenario generation and reproducibility

Resolve every trial from a versioned scenario specification and an explicit seed. The resolved record must include the exact sampled values and resulting scene, not only the seed. Randomize within predeclared, physically valid ranges:

- object pose, shape/size where supported, mass, and surface friction;
- obstacle count, geometry, pose, and clearance from the task path;
- initial robot joint configuration within declared limits;
- sensor delay, dropout, stale epoch, and missing or unknown required parameters for refusal trials;
- selected contact, authority, expiry, and runtime-divergence fault injections.

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
