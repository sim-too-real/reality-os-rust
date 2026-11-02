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