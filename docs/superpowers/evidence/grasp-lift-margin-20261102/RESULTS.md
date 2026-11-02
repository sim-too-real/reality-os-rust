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

## Limits and next experiment

The lift-target screen remains limited to one simulated gripper fixture; the target change did not improve fresh success above 3/7. The v4 label audit covered 14/20 pilot seeds and failed its hard safety gate. No physical claims are supported by either experiment.

Next: implement a separately named, one-shot SIMULATION_ONLY probe-feasibility experiment using the exact v4 low-load UR5e 21003:3 physical input. Keep the official expected-refusal record, scoring, physical inputs, collision proof, registered 20 N wrench caps, and abort-and-reobserve behavior unchanged. The exploratory harness may authorize only the planned 8 mm probe under a simulation-scoped grant and must stop after the first measurement or any guard abort. Record whether wrench evidence narrows the support-friction interval and the object-motion residual. Keep benchmark labels out of the reasoning request, do not count this as benchmark success, and do not reclassify the v4 refusal from this single screen.

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

