# Grasp lift margin screen

**Evidence status:** `SIMULATION_ONLY`  
**Classification:** Meaningful advance on a bounded, frozen simulation screen  
**Fixture:** `arm_gripper_lift` pinch-grasp/lift

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

## Limits and next experiment

This is a six-trial screen on one simulated gripper fixture. The four recovery cases were selected because their saved verifier evidence isolated the lift threshold while object-follow passed; several other archived grasp failures have different causes and were not expected to improve from this change. These results do not establish performance on a fresh seed set, another embodiment, or physical hardware.

Next: freeze a new schema-v7 grasp cohort across randomized mass, friction, and object dimensions, then measure hold success, force margin, and false acceptance without changing labels or verifier limits. Keep all results `SIMULATION_ONLY` until there is independent physical evidence.

## Replay

The full baseline and candidate JSON reports, Markdown summaries, and raw trial records are stored under `baseline/` and `candidate/`, one directory per trial. A candidate replay from current `main` is:

```powershell
cargo run --release -p realityos-verify --bin realityos-verify -- manipulation-benchmark --replay "docs/superpowers/evidence/manipulation-reliability-generality-8f090bc/trials/v5_grasp_fixture_arm_gripper_lift_pinch_grasp_lift_6_21.json" --out "$env:TEMP/grasp-lift-candidate"
```

The replay reports the schema upgrade and rejects the trial if the current robot model/source identity no longer matches the saved input.

## Verification

- `cargo test -p realityos-verify --test manipulation_benchmark historical_grasp_failure_reaches_verified_hold_within_frozen_limits -- --nocapture` — passed; includes four prior failures and two success controls.
- `cargo test -p realityos-verify --test manipulation_benchmark grasp --quiet` — 8 passed.
- `cargo test -p realityos-semantics --quiet` — 412 tests passed across unit, integration, and documentation targets.
- `cargo fmt --all -- --check` and `git diff --check` — passed.

