# Grasp lift margin screen

**Evidence status:** `SIMULATION_ONLY`  
**Classification:** Foundational advance in failure diagnostics; lift-target sweep produced a negative generalization result
**Fixture:** `arm_gripper_lift` pinch-grasp/lift

**Generalization status:** The six-case gain below did not improve success rate on a fresh seven-case expected-success grasp subset. Targets of 24.3, 24.5, and 25 mm each triggered a different measured guard failure on fresh inputs. Keep the lift result classified as a narrow screen result.

## Question and intervention

Four archived expected-success trials previously acquired the object and kept bilateral contact, force, support, and object-follow checks, but the measured end-effector lift stopped below the verifier's 20 mm minimum. Their frozen witness targeted 23 mm while the action envelope and collision proof already allowed at most 25 mm.

The candidate uses the full collision-cleared 25 mm lift target. The 20–25 mm measured verifier window, 1.95 N per-finger force cap, authority checks, collision proof, and expected labels are unchanged. Two previously successful trials serve as controls.

The original schema-v5 inputs were upgraded by the replay command to schema v7 without resampling or changing scenario, goal, fault, or initial joint-position fields. Each baseline/candidate pair has the same v7 trial ID and paired key.

## Paired results

| Frozen trial | Baseline, 23 mm target | Candidate, 25 mm target | Candidate follow residual | Candidate peak finger force |
|---|---|---|---:|---:|
| 6:21, threshold failure | Hold not verified; EE lift 18.77 mm | Verified hold; EE lift 20.75 mm | 4.58 mm | 1.839 N |
| 26:41, threshold failure | Hold not verified; EE lift 18.72 mm | Verified hold; EE lift 20.70 mm | 4.72 mm | 1.836 N |
| 30:45, threshold failure | Hold not verified; EE lift 18.76 mm | Verified hold; EE lift 20.75 mm | 4.42 mm | 1.800 N |
| 50:65, threshold failure | Hold not verified; EE lift 19.97 mm | Verified hold; EE lift 21.94 mm | 2.90 mm | 1.817 N |
| 0:15, prior-success control | Verified hold; EE lift 20.09 mm | Verified hold; EE lift 22.06 mm | 3.30 mm | 1.348 N |
| 21:36, prior-success control | Verified hold; EE lift 20.38 mm | Verified hold; EE lift 22.35 mm | 2.48 mm | 1.814 N |

Baseline source: `67c93b11f7645c9702f7e4ca548164f725b1905d` (23 mm target). Candidate source: `09e4c49ae72f612208006c3f308d70a8b272eb2a` (25 mm target). Candidate reports are clean. The 6:21 baseline report is clean; the other five baseline reports record `candidate_worktree_dirty=true` because the regression test was uncommitted during those replays. The engine source for those baseline runs matched `67c93b1`.

All six candidate episodes completed with `safety_gate_passed=true`, no unauthorized writes, verified acquisition and hold, end-effector lift within 20–25 mm, object-follow residual within 5 mm, and measured per-finger force below 1.95 N. Four of four selected threshold failures changed to success; both controls remained successful. Paired keys match for all six inputs.

## Fresh-suite and target-isolated controls

The complete suite-v3 pilot at the 25 mm target finished 60/60 trials with no infrastructure, invalid, or unaccounted trials. Its hard safety gate passed; false acceptances, unauthorized writes, geometry omissions, stale-authority continuations, unverified-wrench stops, and invalid compensation audits were all zero. Overall task success was 14/31. The grasp slice remained 3/7 expected-success trials, with 6/6 acquisition and 3/6 verified holds. One grasp episode ended in `runtime_divergence`.

This full-suite report compares candidate source `09e4c49` with the older `18eac301` baseline, so its overall paired-screening failure (worst change −8.3 percentage points in the UR5e information-bearing-probe slice) is not attributable to the lift-target change. The candidate worktree dirty flag reflects uncommitted documentation at campaign time; the engine source was `09e4c49`.

To isolate the lift target, the seven fresh expected-success grasp inputs were replayed again on current engine code, changing only the target between 23 mm and 25 mm. All paired keys match. The 25 mm candidate engine code is from `09e4c49`; its current-main descendant is `0725e76`. The 23 mm control reports identify `0725e76` and mark the worktree dirty because only the lift constant was temporarily changed for those control runs.

| Trial | 23 mm result | EE lift / follow residual | 25 mm result | EE lift / follow residual |
|---|---|---:|---|---:|
| 21000:15 | Success | 21.64 / 1.26 mm | Success | 23.60 / 1.26 mm |
| 21005:20 | Success | 21.00 / 2.29 mm | Success | 22.96 / 2.32 mm |
| 21006:21 | Hold not verified | 18.44 / 8.62 mm | Hold not verified | 20.42 / 7.36 mm |
| 21010:25 | Hold not verified | 20.07 / 18.73 mm | Runtime divergence during lift | No completed lift / follow measurement |
| 21011:26 | Success | 20.89 / 2.00 mm | Success | 22.85 / 2.01 mm |
| 21015:30 | Hold not verified | 19.91 / 7.91 mm | Hold not verified | 21.90 / 8.15 mm |
| 21016:31 | Hold not verified | 20.89 / 19.05 mm | Hold not verified | 22.86 / 19.80 mm |

The 25 mm target raised measured EE lift by about 1.97 mm in the six episodes that completed the lift, but neither target succeeded on more than three of the seven inputs. The three follow-residual failures remained failures. Trial 21010:25 changed from a completed `hold_not_verified` refusal to `runtime_divergence`; both outcomes had zero unauthorized writes. This is an execution-continuity regression, not evidence of an unsafe acceptance. These results show no fresh-cohort task-success gain from the target change.

## Target margin follow-up and guard diagnosis

The same 13 frozen inputs (six selected historical cases and seven fresh expected-success cases) were replayed at 24.5 mm. All 13 paired keys matched both the 23 mm and 25 mm records. All six historical cases were successful, including the four selected lift-threshold recoveries. Fresh success remained 3/7. Trial 21006:21 stopped at its first lift waypoint with a finger-force ceiling violation (`finger_l=1.978583 N`, limit `1.950000 N`); trial 21010:25 completed the lift but remained `hold_not_verified` with 19.83 mm object-follow residual. No unauthorized writes occurred and the individual replay safety gates passed.

A targeted 24.3 mm replay of 21006:21 also stopped at the first lift waypoint for the same guard, at `1.994177 N` on `finger_l`. This was a single-case bracket, not a full 13-case campaign. At 25 mm, 21006:21 remained a completed hold-verifier refusal with 7.36 mm follow residual, while 21010:25 diverged at lift waypoint 4 because object-to-tool relative-pose error reached `0.020099 m`, just above the unchanged 20 mm guard. Its sampled peak finger force was 1.109279 N.

These paired traces reject the scalar-target hypothesis: lowering the target from 25 mm avoids the 21010:25 relative-pose stop but selects a path that exceeds the per-finger force cap on 21006:21; lowering again to 24.3 mm did not remove that force violation. No tested target improves fresh task success beyond 3/7, and the safety thresholds remain unchanged. Further constant tuning is stopped pending a trajectory-level explanation.

Grasp-lift runtime divergence now appends its first failed guard to the episode's existing `decision_audits` as a stable `grasp_lift_guard_failure=...` entry, including waypoint and measured value. This makes the two previously generic `runtime_divergence` outcomes distinguishable and independently reviewable. For the 25 mm replays of 21006:21 and 21010:25, 29 execution and verification fields plus all pre-existing decision-audit strings match the earlier records; the only addition is the guard diagnostic. Admission, force limits, and verifier thresholds are unchanged.

The full suite-v3 candidate report and all 60 raw trial records are in `suite-v3-candidate-25mm/`. The seven current-code 23 mm control reports and raw inputs are in `suite-v3-control-23mm/`. The full-suite baseline used the preserved report at `docs/superpowers/evidence/suite-v3-paired-after-label-fix/manipulation_benchmark.json`.

The 13 saved 24.5 mm replays are in `suite-v3-candidate-24_5mm/`. Structured 24.3, 24.5, and 25 mm guard-diagnostic replays are in `suite-v3-lift-guard-diagnostics-20261102/`.

## Trajectory branch and dwell follow-up

The saved 24.5 mm and 25 mm lift witnesses for 21006:21 and 21010:25 use the same contact configuration and arm-joint coordinates at lift, to numerical precision. Only the lift-z target differs by 0.5 mm. This rules out an IK branch switch or arm-joint path discontinuity for those pairs, so the proposed branch-ranking change was not run.

A bounded timing check doubled the lift waypoint dwell from 50 to 100 simulation steps on 21010:25 at the fixed 25 mm target. The unchanged 50-step baseline diverged at waypoint 4 with 20.099 mm relative-pose error; the 100-step candidate diverged at the same waypoint with 22.896 mm error, 2.797 mm worse. The 20 mm guard and all safety checks remained fixed; unauthorized writes were zero. Since the primary case failed, the 13-case cohort was not started. Production code was restored to 50 steps. The frozen inputs and negative result are in `suite-v3-lift-time-scale-20261102/FREEZE.json` and `suite-v3-lift-time-scale-20261102/RESULT.json`.

## Suite v3/v4 refusal-label audit

The full 25 mm v3 report had two evaluation-label mismatches on the low-load `unidentifiable_friction` refusal stratum. In v3, the scenario generator made the case positive with `expected_refusal = None`; the trial still inherited the suite-level expected-refusal label. The v3 scenario test explicitly preserves that legacy shape. Suite v4 adds the scenario reason `UNIDENTIFIABLE_FRICTION` while preserving the physical randomization. A targeted v4 audit covered the first 14 official pilot seeds so the target cases retained indices 3 and 13. The attempted two-seed manifest was rejected before simulation because the manifest contract requires exactly 20 pilot and 100 public seeds; `SETUP_ATTEMPT.json` records that setup failure.

The follow-up campaign completed and parsed 42/42 trial records with no infrastructure, invalid, or unaccounted trials. Its hard safety gate failed: task success was 13/23, correct refusal 17/19, false refusals 6, false acceptances 2, evaluation-label mismatches 0, unauthorized writes 0, and friction narrowings 10/15. The runner returned exit code 1 because the hard gate failed, not because the campaign was incomplete.

For both target records—UR5e 21003:3 and iiwa14 21013:13—the expectation, object randomization, initial joint state, and goal matched the v3 records. The scenario reason changed from absent to `UNIDENTIFIABLE_FRICTION`; the paired keys changed accordingly. Both outcomes changed from `evaluation_label_mismatch` to `false_acceptance`. The decision audit remained `Refuse { reason: "SCOPED_GRANT_DOES_NOT_COVER_PROBE" }`, the probe candidate was marked `PROVED`, and unauthorized writes remained zero. Thus v4 fixes the attribution error but reveals no capability gain and does not pass the benchmark gate.

A behavior-preserving audit trace now records the pre-grant source and key plus the action key issued by the bridge. Replays of both exact target records show `pregrant_action_source=authority_request`, `pregrant_action_key=probe:+x`, and `authorized_action_key=probe:+x`; each remains `false_acceptance` with zero control writes before or after and zero unauthorized writes. Three focused audit-metadata tests passed, the release binary built, and formatting and diff checks passed. The full result, campaign report, traced replay reports, target trial hashes, and freeze are in `suite-v4-refusal-label-audit-20261102/`.

The v4 audit is only a 14-seed screen, not a full pilot or holdout. The CLI still defaults to suite v3; this result alone is not sufficient to promote v4 as the default. The unit test `v4_names_the_unidentifiable_friction_refusal_and_preserves_v3` passed. All results remain `SIMULATION_ONLY`.

## SIMULATION_ONLY probe-feasibility experiment

The frozen v4 UR5e trial 21003:3 was replayed once from its saved resolved input (SHA-256 `A375E62223D6AEEFCD409AF3E831A1F5E0D14D7C51DC4099740626D98D928191`). Only the exploratory runner's evaluator expectation was changed to `expected_success`; the official record remains `expected_refusal`, and the scenario still carries `UNIDENTIFIABLE_FRICTION`. The runner verified that neither expectation label entered either reasoning-request audit. The registered 20/20/30 N and 2/2/2 N·m wrench caps and abort-and-reobserve contract were unchanged.

Exactly one `probe:+x` action ran. It produced a simulation wrench measurement (`updated:Sticking`) and narrowed the support-friction interval from `[0.05, 0.8]` to `[0.0919516, 0.8]`, a width reduction of about 5.6%. The inferred interval still includes friction below the required `0.3`, so the reasoner selected no follow-up physical action (`NO_PHYSICAL_CANDIDATES`). The episode ended as `unexpected_refusal` under the counterfactual expectation; that is not benchmark success and does not revise the official refusal label. Object displacement was 3.84 mm, there was no wrench abort, and unauthorized writes were zero. The one complete episode, original official record, measurement digests, and decision audits are preserved in `unidentifiable-probe-feasibility-20261102/RESULT.json`; the run protocol is in `FREEZE.json`.

The lift-target screen remains limited to one simulated gripper fixture; the target change did not improve fresh success above 3/7. The v4 label audit covered 14/20 pilot seeds and failed its hard safety gate. These experiments support no physical claims.

## Probe measurement sidecar

The selected UR5e probe audit now retains the full inference input in `unidentifiable-probe-feasibility-20261102/PROBE_MEASUREMENT_SIDECAR.json`, separate from the compact episode. It contains the 3,889 samples used by the friction inference plus their sample digests, per-sample sensor transforms and free-space model wrenches with bounds, compensation, motion window, timing limits, contact identity, and friction prior. Its measurement digest matches the episode's `wrench_inference_measurement_digest` (`4b341f3c93252ba3aa2a38c4e22da7d6da7b2929d1fefb8920654642fb355352`); the sidecar SHA-256 is `22768e93f0dec9a5a532230840aaa4a931ccbc78c5cdb99ddd1903a705947f0c`.

Verify the sidecar offline without starting MuJoCo:

```powershell
cargo run --release -p realityos-verify --bin realityos-verify -- verify-probe-measurement-sidecar "docs/superpowers/evidence/grasp-lift-margin-20261102/unidentifiable-probe-feasibility-20261102/PROBE_MEASUREMENT_SIDECAR.json"
```

The command checks each sample digest, recomputes the complete measurement digest, reruns support-friction inference, and prints the interval and sample count. This remains simulation-only evidence and does not change the official expected-refusal label or admission state.

The sidecar writer never overwrites an existing artifact. For another explicit exploratory replay, set `REALITYOS_EXPLORATORY_SIDECAR_OUTPUT` to a new output path before running the frozen test.

New opt-in captures also include a digest-sealed target-body pose at every monitored physics step, paired to the wrench sequence and simulation-clock timestamp whenever both were captured on the same step. The offline verifier checks pose ordering, pose and trace digests, endpoint displacement against the compact motion summary, and one-to-one wrench/pose sample pairing; it reports the first XY threshold-crossing bracket at the configured pose resolution. The historical sidecar above predates this field, so its verifier output marks pose-trace coverage as `absent_legacy`; it still reproduces the stored friction calculation but cannot independently audit the old motion summary.

## Offline calculation and motion-tracking follow-up

The interval's lower bound comes from the final sample. Its capture time matches the motion-window end, so the rise-rate time-skew allowance is zero. The sensor error (`0.05 N` per force axis) plus precontact residual bound (`0.05 N`) gives `0.10 N` per sensor-force axis; the same-state free-space model error is zero. Applying the saved per-sample transform yields a conservative tangential-force lower bound of `0.1859044 N` and normal-load upper bound of `2.0217637 N`. Their ratio is `0.09195159957`; intersecting with `[0.05, 0.8]` reduces interval width by 5.59% and leaves the upper bound unchanged. The lower bound remains well below the required `0.3`, so the probe justifies no follow-up action.

This audit also found a motion-summary limitation in the original replay. The complete episode reports `3.8388 mm` net object displacement, while its stored probe window reports zero displacement and no `first_motion_s`. The runner had been resetting the motion origin whenever the force lower bound was below `0.5 N`; a sequence of gradual movements could therefore be reported as `Sticking`. The current runner now preserves the original origin and flags motion that crosses the pose threshold before force evidence resolves; inference fails closed with `UnbracketedMotion`. Focused unit tests cover cumulative drift and a later force rise. The frozen simulation was not rerun after this code fix, so the historical sidecar still reproduces the original stored `Sticking` calculation and should not be read as proof that the object remained motionless.

During sidecar integration, the frozen trial was invoked twice with the same saved input and unchanged intervention: the first invocation completed simulation and wrote the sidecar, then failed while writing `RESULT.json` because its output path was relative; the second used an absolute output path and completed. Both produced the same sidecar digest. No other seed or embodiment was run. The new pose-trace contract was not exercised by rerunning that trial; the artifact remains historical.

Next selection: run a newly frozen, bounded probe case through the opt-in path, then independently compare its raw time-aligned pose trace against the compact motion summary and verify the first threshold-crossing bracket. Keep the prior frozen case closed; do not relax the probe, force, or authorization limits.

## Replay

The full baseline and candidate JSON reports, Markdown summaries, and raw trial records are stored under `baseline/` and `candidate/`, one directory per trial. A candidate replay from current `main` is:

```powershell
cargo run --release -p realityos-verify --bin realityos-verify -- manipulation-benchmark --replay "docs/superpowers/evidence/manipulation-reliability-generality-8f090bc/trials/v5_grasp_fixture_arm_gripper_lift_pinch_grasp_lift_6_21.json" --out "$env:TEMP/grasp-lift-candidate"
```

The replay reports the schema upgrade and rejects the trial if the current robot model/source identity no longer matches the saved input.

## Verification

- `cargo build --release -p realityos-verify --bin realityos-verify` — passed at the retained 25 mm target.
- `cargo test -p realityos-verify --test manipulation_benchmark --quiet` — 104 passed, including the new guard-diagnostic test and fresh 21006:21 refusal check.
- `cargo test -p realityos-verify --lib --quiet -- --test-threads=1` — 178 passed.
- Remaining Rust integration targets (`pair_key_roundtrip`, `physical_experiment_decision`, `production_mujoco_reasoner`, `production_reasoner_authority`, `typed_grasp_verification`) — 20 passed.
- `cargo test -p realityos-semantics --quiet` — 412 passed across unit, integration, and documentation targets.
- The three failures reported by an initial default-parallel crate run passed individually; the complete serial library run passed 178/178. The cause of the parallel-only failures is unconfirmed.
- `cargo fmt --all -- --check` and `git diff --check` — passed.

## Time-aligned probe pose trace: bootstrap pairing follow-up

Three frozen runs isolate and then repair a missing evidence link. The first two are retained negative results; their structurally valid traces had one fewer pose sample than wrench sample and both independently recomputed as `NoInference(InvalidMotionTrace)`. In seed 21016, wrench sequence `26000` occurred at `start_s = 52.00200000001759`, while the first pose was sequence `26001` at `52.00400000001759`. In seed 21017, wrench sequence `8000` occurred at `start_s = 16.00200000000201`, while the first pose was sequence `8001` at `16.00400000000201`. The latter also showed that making the trace filter inclusive at `start_s` cannot restore a pose the worker never emitted. The worker's bootstrap wrench is sampled before its per-physics-step pose loop.

The correction adds an optional `start_wrench_sequence` to `ProbeObjectPoseTrace`. Legacy traces deserialize with no sequence and retain their `/1` digest. Traces with an explicit start pair use the `/2` digest, sealing the sequence alongside the already captured start pose and time. Inference accepts that pairing only when one wrench sample has both the exact sequence and exact `start_s`; the builder records it only for a unique start-time wrench that lacks a per-step pose pair. All later sample pairing stays one-to-one. The old 21016 and 21017 artifacts were not edited, and the new release verifier still reports their motion traces invalid while reproducing their fail-closed inference.

One new exploratory v4 UR5e input, seed/index 23017:17, was frozen outside both v4 configured seed cohorts (pilot 21000–21019; public 22000–22099) and run once. Its bootstrap wrench sequence `8000` is explicitly sealed at `start_s = 16.00200000000201`; the trace has 5,590 subsequent pose samples for 5,591 wrench samples. The replay test passed. Independent offline verification reports `motion_trace_status: verified`, recomputes the same `Updated(SlipOnset)` friction interval `[0.3509380, 0.4567707]`, and calculates the first 1 mm XY bracket `[23.062, 23.064]` s. The endpoint displacement is `[4.8335, 0.4054, -0.0017]` mm. The episode outcome was success, with zero unauthorized writes and no wrench abort. This was a trace-contract validation with no historical exact-seed baseline, not a benchmark comparison or physical result.

Each freeze, input, result, sidecar, offline verification, and per-trial interpretation is preserved in `probe-pose-trace-v4-ur5e-21016-20261102/`, `probe-pose-trace-v4-ur5e-21017-20261102/`, and `probe-pose-trace-v4-ur5e-23017-20261102/`. The old 21003 trial remains closed. No probe, force, or authorization limits changed; no physical loads, purchases, fabrication, or external contacts occurred.

Verification for this correction: `cargo test -p realityos-semantics` passed (420 total across unit and integration targets); all 29 `push_probe_inference` tests passed; the verifier pose-trace, gradual-motion, and force-resolved-onset tests passed; `cargo test -p realityos-verify --test manipulation_benchmark --no-run` passed; the one frozen exploratory replay passed; and the release offline verifier independently accepted the new sidecar and reproduced its inference. No full `realityos-verify` suite pass is claimed.

The prior checkpoint proposed another frozen exploratory replay; the follow-up selection diagnostic below supersedes that proposal after testing showed that an expected-success label alone does not guarantee a probe action.

## Follow-up selection diagnostic

The second exploratory selection exposed that an `expected_success` label does not guarantee the episode will execute a probe. Seed/index 23018:18 resolved as `expected_refusal` / `UNIDENTIFIABLE_FRICTION`; it was retained as a freeze-only selection and not simulated. Seed/index 23018:17 resolved as `expected_success` and was frozen with an explicit bootstrap-pair criterion, but its one simulation produced no probe measurement records. The sidecar verifier returned `InvalidTrial("invalid probe measurement sidecar identity or empty records")` before writing either sidecar or episode result. Because records are appended only inside the executed `probe:` branch, no probe reached the measurement path. The episode outcome and action sequence were not persisted and remain unknown. The frozen input was not rerun; details are in `probe-pose-trace-v4-ur5e-23018-17-20261102/RUN_FAILURE.json`.

This is a trial-selection failure, not evidence against the bootstrap-pair fix: the run did not exercise the trace path. A regression now distinguishes an empty measurement sidecar from malformed sidecar identity (`probe measurement sidecar has no measurement records`); its test passed after first reproducing the old ambiguous error. The next action is a source-backed audit of archived v4 action/measurement records and the candidate selection path to identify a pre-run criterion for an expected-success case that actually executes `probe:+x`. Do not spend another simulation on an expected-success label alone. Resume trace replay only after a new frozen input satisfies that criterion; retain all prior trial inputs unchanged.


## Runtime first-action preflight and fresh trace replay (2026-11-02)

The runtime now exposes a non-actuating first-action preflight for expected-success, un-faulted push/probe trials. It reuses the normal initial observation, reasoner request/ranking, and authorization path, then returns before control writes or physics steps. Its receipt binds the resolved trial, observation digest, request/decision audits, selected action, grant/witness/epoch, and before/after simulator counters. The implementation and test are at Reality OS source commit 22aa6cb1f489e064741a8bd2a2eeeaa9d574fd99 on canonical local main.

A fresh v4 UR5e trial, seed/index 23019:17, was outside the frozen, pilot, and public cohorts. Its preflight selected and authorized probe:+x; controls were 0 -> 0, simulation time 0 -> 0, and steps 0. The frozen input, preflight receipt, and exact runner/suite sources are retained in probe-action-selection-preflight-v1-ur5e-23019-20261102/.

Exactly one simulation replay was run after freezing. The actual first action matched the preflight (probe:+x). The replay produced one measurement with 5,606 wrench samples and 5,605 pose samples. The sidecar digest and motion-trace contract verify. The offline verifier independently recomputed Updated(Sticking) with interval [0.11775341918041364, 0.8] from prior interval [0.05, 0.8]; it returned no first 1 mm XY motion bracket. The trace endpoint displacement from its start is approximately [0.026370, -0.001116, -0.000235] mm, well below the 1 mm threshold.

The episode ended in unexpected_refusal with NO_JUSTIFIED_CURRENT_ACTION; its only action was the initial probe:+x. The follow-on decision audit reports probe:+x:PROBE_SWEEP_BLOCKED and no justified physical candidate. A subsequent offline source/trace audit narrowed the cause: the final recorded object quaternion implies yaw -7.5232924e-5 rad, rotating local +X to [0.99999999717, -7.52329258e-5, 0]. The sweep checker requires exactly one direction component above 1e-6; this direction has two, so sweep_gap returns None before checking obstacles, and finish_probe maps None to PROBE_SWEEP_BLOCKED. This explains the status as an unsupported slightly rotated sweep, not an observed obstacle collision. The audit is retained in OFFLINE_GEOMETRY_AUDIT.json; it used no new simulation. There were zero unauthorized writes. The replay test failed at the predeclared assertion requiring a 1 mm threshold bracket; this is a bounded negative result, not a trace-contract or sidecar-integrity failure. The trial was not rerun.

The preflight demonstrates that the runtime's first authorized action can be identified before stepping the simulator and matches the subsequent episode. It does not establish task completion, benchmark improvement, or physical performance; no exact-seed baseline was available. The interval is simulation evidence only. No physical loads, purchases, fabrication, or external contacts occurred; admission and release flags remain unchanged.

Artifact SHA-256 values (raw files):

- TRIAL_INPUT.json: 3101BB40DDEA3123ED671FD07B1A06A308E9AC3DFCA199275AB9C015B9185EF5
- PREFLIGHT.json: 66826E9E19C12225FCA4B40CAE85EAC1F6E7BCE10E45B6FA4795309B02ACD329
- FREEZE.json: 13BE85B8CA4D96FDE3D7467103A3BF2939B8E46997791B855DC713FF921BB716
- PROBE_MEASUREMENT_SIDECAR.json: B6FACB3DF06A3F7545C7BB5A7FF503F9FF5D128D1519EC731AD8AAAB0822B928
- RESULT.json: C8A17B1D78DD96FB71635B3129F510EABA58AC718A631A3D0A83F308DC1C5F9C
- OFFLINE_VERIFY.json: AA27F849E42798AF13F5D1812202A6B63A8D189DE889049333F64D32E3BD4B0B
- OFFLINE_GEOMETRY_AUDIT.json: 9D71B5BFEE7604375DF6CB13BB21F619D9600E5C7AAB983E53DF3ADAA7F906C8

Verification:

- cargo test -p realityos-verify --test manipulation_benchmark first_action_preflight_selects_without_control_write_or_simulation_step -- --ignored --exact --nocapture — 1 passed.
- cargo run --quiet -p realityos-verify --bin realityos-verify -- verify-probe-measurement-sidecar <sidecar> — passed; trace verified, interval recomputed, no XY bracket.
- cargo test -p realityos-verify --test manipulation_benchmark probe_pose_trace_replays_frozen_v4_ur5e_trial -- --ignored --exact --nocapture — failed at the frozen 1 mm bracket assertion after the one permitted simulation. Do not rerun.
- Serial verifier library suite — 184 passed. The broader manipulation integration target was 104 passed, 1 failed, 4 ignored; its sole failure was the restricted pilot 21000:0, which must not be rerun. No full integration-suite pass is claimed.
- cargo fmt --all -- --check and git diff --check — passed before commit.

Next action: add an offline regression for the retained slightly rotated object axis, then make the sweep checker either conservatively validate rotated swept bounds or return a distinct unsupported-direction refusal. Preserve fail-closed collision handling. Validate the chosen rule offline before freezing any new out-of-cohort trial. Do not rerun 23019:17 or any frozen/pilot/public input; do not loosen probe, wrench, authorization, or evaluator-label gates.

## Receipt verifier hardening after the frozen replay

A read-only review identified two gaps in the reusable receipt API: equal nonzero control-write counters could pass, and the self-only verifier could not bind the receipt to a resolved trial or later episode. Follow-up code commit f96ffc70 strengthens the contract: verification requires both control-write counts to be zero and simulator times finite, nonnegative, and bitwise equal; verify_for_trial binds trial ID, trial digest, and paired key; verify_matches_episode binds the initial request and decision audit plus the first action, grant, witness, and observation epoch.

The focused offline regression reads the already frozen TRIAL_INPUT.json, PREFLIGHT.json, and RESULT.json. It confirms the archived artifacts cross-bind, then rejects a nonzero-but-equal write count, a different resolved trial, a modified request, and mismatched replay authority. The test passed (1/1); formatting and diff checks passed. This hardening was committed after the frozen replay and changed no frozen artifacts. The replay remains bound to its original runner commit 22aa6cb1f489e064741a8bd2a2eeeaa9d574fd99; no replay or simulation was repeated.

## Remaining observation-to-request verification limit

The post-run review found one boundary that the current receipt API does not independently close. The verifier hashes the embedded preflight observation and matches the request audit, decision, and authority fields against the saved replay, but the episode does not contain a separate digest of its initial raw observation. The source path constructs the request and receipt from the same current observation, and the frozen source hashes preserve that implementation; however, this is source-backed linkage rather than a replay-side observation-to-request recomputation. No stronger claim is made. Add a replay-side initial-observation digest or an equivalent explicit binding, with an offline observation/time mutation test, before treating this receipt format as fully portable proof of that relationship.

## Offline follow-up: rotated sweep classification and replay observation binding

**Classification:** Foundational verification improvement; no benchmark gain or new task result. **Status:** `SIMULATION_ONLY`; admission and release flags remain unchanged.

The saved `-7.5232924e-5 rad` yaw produces a non-axis-aligned probe direction. `sweep_gap` now distinguishes `UnsupportedDirection` from an actual `Blocked` intersection. `finish_probe` reports `PROBE_SWEEP_DIRECTION_UNSUPPORTED` for the saved direction and retains `PROBE_SWEEP_BLOCKED` for a swept-volume intersection. The regression uses the saved yaw and directly verifies an intersecting AABB remains blocked; it does not claim rotated geometry is collision-checked.

New episodes record `initial_observation_evidence` v1: compact observation JSON in `observation_json`, lowercase SHA-256 of those exact UTF-8 bytes, and `simulator_time_s`. The verifier parses the JSON, recomputes the digest, checks the time against the embedded observation, then matches the parsed observation, digest, and time to preflight before checking the first request and authority. Preflight verification also requires its simulator time to match its observation. This is hash consistency and record binding, not a signature or an authenticity claim.

The existing raw wrench/pose measurement sidecar remains unchanged and continues to carry the samples used to recompute friction intervals. The new record binds the episode’s first request to its replay-side starting observation.

The archived 23019:17 `RESULT.json` has no replay-side observation record. It remains unchanged and the new strict `verify_matches_episode` rejects it for this missing evidence. A synthetic contract fixture exercises a matching record and a self-consistent altered-time record; it is not presented as evidence from the archived run. No frozen, pilot, or public trial was rerun, and no new benchmark experiment was executed.

Verification:

- Red-green rotated sweep regression: failed before the change with `PROBE_SWEEP_BLOCKED`; passed after the change (1/1), including the actual-collision block assertion.
- `cargo test -p realityos-verify --lib manipulation_benchmark::execution::replay_initial_observation_evidence_tests::replay_observation_record_hashes_exact_json_bytes_and_binds_time -- --exact --nocapture` — 1 passed.
- `cargo test -p realityos-verify --test manipulation_benchmark first_action_preflight_verifier_rejects_legacy_and_binds_replay_observation -- --exact --nocapture` — 1 passed after an expected RED against the legacy verifier.
- `cargo test -p realityos-semantics --lib` — 366 passed.
- `cargo check -p realityos-verify`, `cargo fmt --all -- --check`, and `git diff --check` — passed.
- The parallel `cargo test -p realityos-verify --lib` run was not clean: it reported failures in `authority_restart_replay_has_zero_write_delta`, `command_lease_expires_to_safe_state`, `command_replay_and_wrong_identity_refuse`, `geometry_cross_exam_against_mujoco_or_skip`, `mode_b_select_on_arm_gripper_is_precise_or_distinct_refusal`, `mesh_bundle_loads_from_unrelated_cwd`, `body_backed_ee_fk_matches_mujoco`, `hinge_slide_free_ball_topology`, `evidence_serialization_roundtrip`, and `matrix_never_drops_jobs`, followed by a stack overflow in `mode_b_v2_without_maneuver_does_not_invent_xyz_targets`. Isolated reruns of the geometry cross-exam, IK residual, serialization, and stack-overflow test all passed; causes for the remaining parallel-run failures are unclassified. A serial full-library rerun was stopped during a simulation-backed goal-directed case and has no final suite result.

That full library run also rewrote `external_holdout_postfix_score.json` with a different random sample. The file was restored to `HEAD` and is not part of this change.

Next: prepare a separate out-of-cohort freeze for one replay using this record format, only after choosing and verifying a new input. Keep all listed frozen, pilot, and public inputs untouched.

## Fresh out-of-cohort replay with observation binding: 23020:17

**Classification:** Foundational verification evidence; no benchmark gain. **Status:** `SIMULATION_ONLY`; the predeclared 1 mm motion-bracket gate failed. The rotated-sweep classifier was not exercised.

Seed/index `23020:17` resolved to `v7:development:menagerie_ur5e:push_probe_replan:23020:17`. It is outside the v4 pilot (`21000..=21019`), v4 public (`22000..=22099`), and manifest historical seed ranges. No exact trial ID or frozen-trial manifest matched before this input was created. The input, zero-step preflight, and freeze were committed and pushed before the single replay; the freeze binds runner source commit `cf17d77c344c1b94e6c064566a7edd93b3d33a71`.

Preflight selected and authorized `probe:+x` with zero control writes, zero simulator-time advance, and zero steps. The one replay's first action matched. The episode then ended `unexpected_refusal` with `NO_JUSTIFIED_CURRENT_ACTION` (`bridge_refusal`) and zero unauthorized writes. It did not produce a justified follow-on action.

The sidecar contains 7,251 wrench samples and 7,250 pose samples. Offline verification passed for the preflight-to-trial and preflight-to-replay observation bindings, exact UTF-8 observation digest, observation time, sidecar measurement digest/inference, and pose trace. The recorded friction interval stayed `[0.05, 0.8]`; inference was `NoInference(Unidentifiable)`. Maximum planar object displacement from the trace start was 0.3452 mm, below the unchanged 1 mm motion threshold; no first motion bracket was present. Peak absolute yaw in the trace was `3.3234e-8 rad`, and no rotated-sweep decision was reached.

The replay test exited at its predeclared first-1-mm-bracket assertion after it had written `RESULT.json` and `PROBE_MEASUREMENT_SIDECAR.json`. This is the one allowed run; do not rerun it. `OFFLINE_VERIFY.json` records the preserved artifact hashes and offline checks. Admission remains `NOT_ADMITTED`; `metal`, `shop_release`, and `viewport_admits` remain false. There were no physical actions, loads, purchases, fabrication, or external contacts.

Verification:

- The parameterized zero-step preflight test failed against the prior hardcoded seed as expected, then passed for `23020:17`; it recorded `probe:+x`, zero writes, zero time advance, and zero steps.
- `cargo fmt --all -- --check` and `git diff --check` passed for the preflight-test change.
- The single frozen replay ran once and failed only the predeclared 1 mm motion-bracket assertion. No retry or second simulation occurred.
- `cargo run --quiet -p realityos-verify --bin realityos-verify -- verify-probe-measurement-sidecar <sidecar>` returned `PASS`; it recomputed `NoInference(Unidentifiable)`, verified the trace, and reported no XY bracket.
- A separate offline evidence consumer passed `verify_for_trial`, `verify_matches_episode`, sidecar verification, and pose-trace validation on the saved artifacts; it launched no simulator.
- Code review of `c6f1ad5d18adabdc8b34305fbfe9f778a6ce6a69..cf17d77c344c1b94e6c064566a7edd93b3d33a71` found no blocking issues. The code commit and pre-run freeze commit are pushed on canonical `main`.

## Per-step runtime-contact evidence contract

**Classification:** Foundational verification improvement; no benchmark gain. **Status:** `SIMULATION_ONLY`; admission and all release flags remain unchanged.

The 23020:17 episode retained only a unique episode-wide contact-pair list. It showed an object-table pair and no tool-object pair, but it could not independently align contacts to each pose in the measured probe interval. Its old sidecar remains intact and verifies with the prior `NoInference(Unidentifiable)` interpretation; the new contact claim is not retroactively added to it.

New probe pose samples carry structured body and geometry identities for the contacts observed at that simulator step, with a separate list at the exact window start. `Some([])` means the simulator observed no contacts at that step; absent contact fields mean a legacy record. The per-sample and trace digests bind these values. A new trace must include contact evidence at the start and every step; partial evidence is invalid. After validating the trace and wrench alignment, friction inference returns `NoInference(NoIntendedContact)` when the complete trace contains no authorized tool-target pair. Legacy traces retain their previous interpretation.

The simulator worker now emits contact pairs for every monitored probe step, the parser requires that field for each new pose sample, and the replay freeze binds the semantics, execution, parser, worker, and test source hashes. This makes the per-step contact classification recomputable from the sidecar without trusting an episode summary or an inference boolean.

Verification before the next replay:

- `cargo test -p realityos-semantics --test push_probe_inference` — 32 passed, including absent contact, positive contact, partial trace rejection, digest binding, and legacy traces.
- `cargo test -p realityos-verify --lib probe_` — 14 passed, including start-state identity capture and worker parser requirements.
- `cargo test -p realityos-verify --test manipulation_benchmark --no-run` — integration target compiled.
- `cargo fmt --all -- --check`, `git diff --check`, and `python -m py_compile crates/verify/mujoco_worker.py` — passed.
- The unchanged 23020:17 sidecar verifier returned `PASS`, recomputing its historical `NoInference(Unidentifiable)` result.

That selected experiment was executed once as `23021:17` and is reported below. Do not rerun it or any frozen, pilot, or public input.

## Fresh out-of-cohort per-step contact trace replay: 23021:17

**Classification:** Foundational verification improvement; no benchmark gain. **Status:** `SIMULATION_ONLY`; admission and all release flags remain unchanged.

The exact trial ID `v7:development:menagerie_ur5e:push_probe_replan:23021:17` was absent from the prior repository archive at candidate audit, and the suite preflight confirmed exclusion from pilot, public, and historical seed ranges. The resolved input SHA-256 is `46AC4D7C2B3A093D9564BD8755583667747493EF139FAFFAE6E9E53F717E67C6`. The zero-step preflight selected `probe:+x` with zero control writes, zero time advance, and zero simulator steps. Source code is frozen at `a22d944e0ef51017a535f303c86e51a72beb571b`; the freeze and preflight artifacts were committed and pushed as `edd865e` before the only simulation.

The replay test passed in one run (166.09 seconds). Its first action matched preflight, the episode outcome was `success`, and unauthorized writes were zero. The sidecar contains 5,124 wrench samples and 5,123 pose samples with complete start and per-step contact evidence. The authorized tool-target pair (`wrist_3_link/geom_28` with `obj0/geom_29`) appeared in 1,281 pose samples, first at simulator step 10,952 and time 21.904 s. The trace independently brackets the 1 mm motion threshold at `[23.556, 23.558]` s.

The offline sidecar verifier returned `PASS`, validated the trace, and recomputed `Updated(SlipOnset)` with friction interval `[0.4788557402, 0.7211370850]` from prior `[0.05, 0.8]`. This is one model-conditioned simulation result. It establishes no physical friction measurement, held-out performance, broader reliability, or product claim. `OFFLINE_VERIFY.json` records the exact artifact hashes and contact summary; no artifact from `23020:17` was changed.

Verification:

- Export test for the resolved 23021:17 input — 1 passed.
- Zero-step preflight for 23021:17 — 1 passed; `probe:+x`, zero writes, zero time advance, zero steps.
- Frozen simulation replay — 1 passed; exactly one simulation, first action matched, sidecar and complete contact trace valid.
- `cargo run -p realityos-verify --bin realityos-verify -- verify-probe-measurement-sidecar <sidecar>` — `PASS`; independently recomputed the SlipOnset interval and first motion bracket.
- The offline regression suite covers absent intended contact, observed intended contact, partial-trace rejection, digest binding, and legacy trace compatibility.
- Code verification: semantics probe tests 32/32; verify probe tests 14/14; replay integration target compiled; rustfmt, diff check, and Python syntax check passed.

Artifact SHA-256 values (raw files):

- `FREEZE.json`: `6424E2F72D599418EEB292460A20E1B0FDB964BE73F16C24284A39E009BF82F3`
- `TRIAL_INPUT.json`: `46AC4D7C2B3A093D9564BD8755583667747493EF139FAFFAE6E9E53F717E67C6`
- `PREFLIGHT.json`: `033E7A98919540A3FEA4FC57A2C32E0F5BBC56018079AB1B22068BED085BDEAD`
- `RESULT.json`: `997090CC4CFCA8A26687B1D41561D64D3441CE5F3DB7ACC5C98317C608003E27`
- `PROBE_MEASUREMENT_SIDECAR.json`: `BD45EB08908C91E9B0EF1DED67AF152302AFD55F56B4D58FCCE9320BC88B5E5F`
- `OFFLINE_VERIFY.json`: `085F511F95ECD60C6A7D9087B29271A69ECF79C952A43A30E05681515299CA9F`

Candidate `23022:17` was rejected before freeze: its expected-success label did not produce an authorized action. The zero-step preflight returned `NO_JUSTIFIED_CURRENT_ACTION`; the reasoner reported `NO_PHYSICAL_CANDIDATES`, with `probe:+x` classified as `UNINTENDED_ROBOT_OBJECT_CONTACT`. No replay occurred. The failure and resolved input are preserved in `fresh-out-of-cohort-per-step-contact-trace-v1-23022-20261103/`.

The next fresh candidate, `23023:17`, passed the zero-step preflight and was frozen and pushed before its one replay. Its result is recorded below. Do not rerun either candidate or any frozen, pilot, or public input.

## Fresh out-of-cohort per-step contact trace replay: 23023:17

**Classification:** Foundational verification evidence; no benchmark gain. **Status:** `SIMULATION_ONLY`; admission and all release flags remain unchanged.

The exact trial ID `v7:development:menagerie_ur5e:push_probe_replan:23023:17` was absent from the prior archive. Seed 23023 is outside the checked-in v4 pilot, public, and historical seed ranges; index 17 resolves to the expected-success stratum. The first candidate, `23022:17`, was retained as a fail-closed selection refusal and was not simulated. `CANDIDATE_SELECTION.json` records both decisions and their inputs.

For `23023:17`, the resolved input SHA-256 is `774AB38FDA184379DFBB0F561846326328606D15F2D5EFA216FF6BF590710FE2`. Zero-step preflight authorized `probe:+x`, matching the trial digest, with zero control writes, zero simulator-time advance, and zero steps. The freeze binds the preflight, candidate selection, suite, and runner source hashes. It was pushed to Reality OS `main` as `6193ec346e0e3664facc60d485bd8dccc6599c2c` before the only replay.

The frozen replay test passed in one run (56.24 seconds). Its first action matched preflight; the episode outcome was `success`, with zero unauthorized writes. The sidecar contains 5,232 wrench samples and 5,231 pose samples, with complete start and per-step contact evidence. The intended pair (`wrist_3_link/geom_28` with `obj0/geom_31`) appeared in 1,535 pose samples, first at simulator step 9,613 and time 19.226 s. The offline verifier recomputed the first 1 mm motion bracket at `[22.712, 22.714]` s and `Updated(SlipOnset)` with friction interval `[0.4711696053, 0.5484442443]` from prior `[0.05, 0.8]`.

The sidecar CLI verifier returned `PASS`, validated the trace, and independently recomputed the inference from saved samples. This is one model-conditioned simulation result; it establishes no physical friction measurement, held-out performance, broader reliability, or product claim. `OFFLINE_VERIFY.json` contains the raw artifact hashes and contact summary. Admission remains `NOT_ADMITTED`; `metal`, `shop_release`, and `viewport_admits` remain false.

Verification:

- Export test for `23023:17` — 1 passed; exact trial identity and expected-success label recorded.
- Zero-step preflight for `23023:17` — 1 passed; `probe:+x`, zero writes, zero time advance, zero steps.
- Pre-run freeze commit `6193ec346e0e3664facc60d485bd8dccc6599c2c` was pushed before replay.
- Frozen simulation replay — 1 passed; exactly one simulation, preflight action matched, complete per-step contact trace validated.
- `cargo run --quiet -p realityos-verify --bin realityos-verify -- verify-probe-measurement-sidecar <sidecar>` — `PASS`; interval and motion bracket recomputed.
- The 23022:17 preflight failure was diagnosed and preserved; it was not replayed. The temporary stdout-only diagnostic instrumentation was reverted before the freeze and replay.
- The offline regression suite continues to cover absent intended contact, observed intended contact, partial-trace rejection, digest binding, and legacy trace compatibility.

Artifact SHA-256 values (raw files):

- `23022:17/PREFLIGHT_FAILURE.json`: `681246C8DA21366C7C876E26F77F76D5C31A34FB46CBE77A79184D5C51BF739C`
- `FREEZE.json`: `A3CBAEDC54B1F1B13AB423C51B81EF28E544CB44DDFA4C65FFCCCDF7AC9F79C3`
- `CANDIDATE_SELECTION.json`: `F59D6A8CED487909FC44C5961B867D585E3F8594E384CDC32C283E111F7AE7D4`
- `TRIAL_INPUT.json`: `774AB38FDA184379DFBB0F561846326328606D15F2D5EFA216FF6BF590710FE2`
- `PREFLIGHT.json`: `BDBC7609E5388417B1B50CF7AAF1C73760C2414C1CC8E269CD3BED4C5EBAAC0B`
- `RESULT.json`: `831738562F4709ECCD0ECF6472C75D693F32FAC17D4434DCBF238DBBB9694A9E`
- `PROBE_MEASUREMENT_SIDECAR.json`: `2BB3A8FA881AD41AEECE6BC7D573BCD3C640A91E30B438BBE08D33CC70D39E62`
- `OFFLINE_VERIFY.json`: `02AE04942CB3540F450F97E57072160F37100CB035BB0684E33C5B1CF7424836`

## Fresh out-of-cohort per-step contact trace replay: 23024:17

**Classification:** Foundational verification evidence for the no-intended-contact branch; no benchmark gain. **Status:** SIMULATION_ONLY; admission and release flags remain unchanged.

The exact trial ID v7:development:menagerie_ur5e:push_probe_replan:23024:17 was absent from the repository archive and Git history before export. Seed 23024 is outside the v4 pilot, public, and historical ranges; index 17 resolves to expected_success. The freeze and selection audit preserve the cohort checks.

The candidate export contains 2,600 bytes with SHA-256 530C92DF9EECC96820DFB24F1FE90BFE5ACF74B90BDDA114B8F0FA385CCD6A60. The preflight test wrote the same serialized trial plus a final LF to TRIAL_INPUT.json; the frozen on-disk file SHA-256 is 48344027D8EB2AF1AF738800EF3795B761D23D7BC59C51CBCEEC3193148DC191. Zero-step preflight selected probe:+x, bound trial digest 78173999d1cecf77d1e26dc661b4b0433a3d4c2cd5a360afa2c92cd166c8feae and initial-observation SHA-256 45049b8e39f839c88c6759a24da36cb8bf65e4b9d0f93321d3540c34d5c7e8e1, and recorded zero control writes, zero simulator-time advance, and zero steps.

The source-bound freeze (EFDAA928AD0D5E011812E517FBD305F23916CB6BFE92C0800A4E812522084294) and candidate audit (42356BBA89F7F6F11C6A36C368A14F51ACA3584E137A3EC1FC82127F7E8B0BB1) were pushed to RealityOS main in commit 52b425ccf5eda717871650b6d71117c287f8cab4 before replay.

The frozen replay test passed once in 40.83 seconds. The actual first action matched preflight (probe:+x), and unauthorized writes were zero. The episode ended unexpected_refusal with bridge_refusal; the later decision audit was NO_JUSTIFIED_GOAL_ACTION_OR_SAFE_OBSERVABLE_PROBE. The sidecar contains 7,893 wrench samples and 7,892 pose samples with a complete start and per-step contact trace. The four start pairs and every observed per-step pair were object-table contacts; the intended wrist_3_link/obj0 pair appeared in zero pose samples. No first 1 mm XY-motion bracket was present.

The offline sidecar verifier returned PASS, validated the trace, and recomputed NoInference(NoIntendedContact). The support-friction interval remained [0.05, 0.8]. This is a single model-conditioned negative observation; it does not establish a physical friction measurement, benchmark gain, or broader performance claim. No physical actuation, load, purchase, fabrication, or external contact occurred.

A read-only contrast with 23023:17 found both zero-step preflights selected probe:+x. In 23023, the intended wrist_3_link/obj0 pair appeared in 1,535 samples and the runtime then selected push:+x; in 23024, the pair appeared in none and the runtime refused a follow-on action. Source review confirms finish_probe proves a bounded candidate using joint margin, sweep clearance, a feasible domain, and future recoverability; the measurement gate separately requires observed tool-target contact in the complete trace before inference. The contrast is consistent with that separation. Two different seeds do not estimate a contact success rate or establish a causal reliability claim.

Verification:

- Exact-ID archive and Git-history checks — no prior 23024:17 match.
- Freeze-only input export — 1 passed.
- Zero-step preflight — 1 passed; selected probe:+x, with zero writes, zero time advance, and zero steps.
- Frozen replay — 1 passed; exactly one simulation, first action matched preflight, complete per-step contact trace validated.
- Offline verify-probe-measurement-sidecar — PASS; trace verified and no-contact inference recomputed.
- Admission remains NOT_ADMITTED; metal, shop_release, and viewport_admits remain false.

Artifact SHA-256 values (raw files):

- CANDIDATE_EXPORT.json: 530C92DF9EECC96820DFB24F1FE90BFE5ACF74B90BDDA114B8F0FA385CCD6A60
- CANDIDATE_SELECTION.json: 42356BBA89F7F6F11C6A36C368A14F51ACA3584E137A3EC1FC82127F7E8B0BB1
- FREEZE.json: EFDAA928AD0D5E011812E517FBD305F23916CB6BFE92C0800A4E812522084294
- TRIAL_INPUT.json: 48344027D8EB2AF1AF738800EF3795B761D23D7BC59C51CBCEEC3193148DC191
- PREFLIGHT.json: C1EA4FF0BD9A73939DA7748F98F74077286B34F4F1E8C2143F177F0050EF31E7
- RESULT.json: 0BAF96A6D9683FD77A79C486F107A373D69290D45E388F74A3213961F157CC46
- PROBE_MEASUREMENT_SIDECAR.json: 7A1AE0F8300ED5690A2E58185238C3F0B4D0D0ECD803D1DA521194D3C50D2462
- OFFLINE_VERIFY.json: B8583E3FF1102220BD8949BD3891B7A5CEED12D963717F4EA3DB5AD88BEDA2B3

## Reusable probe interval replay format

Use the versioned `realityos.probe_measurement_sidecar` JSON together with the
frozen source and offline verification records to recompute future friction
intervals from the exact typed measurement inputs. The field contract, replay
command, versioning rule, and limits are documented in
[`2026-11-03-probe-interval-replay-evidence-contract.md`](../../specs/2026-11-03-probe-interval-replay-evidence-contract.md).
The offline CLI was rerun against the preserved `23023:17` and `23024:17`
sidecars: it recomputed the recorded `SlipOnset` interval for the former and
`NoInference(NoIntendedContact)` for the latter. No simulator was started.
Both freeze records' `push_probe.rs` SHA-256 matched the source file used for
this replay (`6444cab400e6407ddf4ff7a6cd019fbc3719b43f29c73a24fe92c95d0abbd80c`).

Next: select a distinct bottleneck for the next cycle; do not run another seed under the same probe setup. Any follow-up replay must test a new falsifiable intervention chosen from this contrast and be frozen before one run. Never rerun 23024:17, 23023:17, 23022:17, or any frozen, pilot, or public input.
