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

## Limits and next experiment

The selected six-trial screen is on one simulated gripper fixture. The four recovery cases were selected because their saved verifier evidence isolated the lift threshold while object-follow passed; several other archived grasp failures have different causes and were not expected to improve from this change. The fresh seven-input control did not show higher success at any tested target and exposed force or relative-pose stops. These results do not establish performance on randomized mass, friction, and object dimensions, another embodiment, or physical hardware.

Next: compare the prepared joint-space lift witnesses and deterministic IK branches for 21006:21 and 21010:25, then test one path-continuity candidate-selection change against the same 13 inputs. Keep collision checking, the 1.95 N per-finger limit, the 20–25 mm verifier window, and the 20 mm relative-pose stop fixed. If the branch traces do not support a deterministic candidate-ranking hypothesis, pivot away from the arm-gripper lift fixture before expanding the cohort. Keep all results `SIMULATION_ONLY` until there is independent physical evidence.

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

