# Cartesian grasp-lift waypoint experiment — 2026-11-03

**Classification:** `VALIDATED_LEARNING; CARTESIAN_PATH_HYPOTHESIS_REJECTED; SIMULATION_ONLY; NOT_ADMITTED`

**Candidate source and executor correction commit:**
`0f14e9b073fcadf11ed69c6643cc7bbbfb36daf5` (Cartesian planner implementation
is preserved in history and removed from the final production reasoning code;
the multi-waypoint executor validation fix remains).

## Hypothesis and frozen gates

The candidate replaced endpoint-only joint interpolation with sequential
translation-IK states at no more than 1 mm Cartesian spacing. Each adjacent
joint-state transition retained the existing collision check. The endpoint
remained 25 mm above contact. The 1.95 N per-finger ceiling, 20 mm runtime
relative-pose stop, 5 mm independent hold limit, simulator, task inputs, and
authority checks stayed fixed.

The paired uniform baseline had 4 successes in its frozen seven-case cohort.
The candidate primary controls had to clear before the seven-case candidate
cohort could run.

## Execution and result

The first execution attempt was rejected by the executor after the first
multi-waypoint segment because its contact-state check incorrectly required
every segment to start at the original contact state. That run is preserved in
`CANDIDATE_PRIMARY.json` as an integration rejection. A regression test now
checks a multi-waypoint path's initial contact state, finite joint states, and
unchanged finger closure.

After correcting that executor gate, both frozen primary controls reached the
lift path and hit the unchanged 20 mm relative-pose stop:

| Trial | Relative-pose error | Object vertical change | Net vertical contact force at last sample | Object weight |
|---|---:|---:|---:|---:|
| 21010:25 | 20.032 mm | -0.104 mm | 0.135 N | 1.419 N |
| 21006:21 | 20.128 mm | +0.026 mm | 0.501 N | 2.192 N |

The candidate cohort was skipped under the frozen decision rule. There is no
candidate success rate to compare with the 4/7 baseline or historical 3/7.
No success, admission, physical, or release claim follows.

The measured simulation contact forces and near-zero object rise support a
contact-load distribution mismatch with the planner's scalar friction/force
hold screen. They do not identify whether contact orientation, contact
geometry, or actuator force realization is the cause. The Cartesian path did
not resolve the failures, so the path-planning candidate is rejected and has
been removed from production reasoning code. The executor's independent
multi-waypoint validation correction remains.

## Independent verification

`verify_candidate.py` independently recomputes the freeze, baseline, input,
and output digests; verifies both fixed trial identities and refusal outcomes;
and compares the recorded vertical contact support with request weight. Its
output is `INDEPENDENT_VERIFICATION.json`.

Run the offline check with:

```powershell
python docs/superpowers/evidence/grasp-lift-cartesian-waypoints-20261103/verify_candidate.py
```

The next justified software question is whether a source-grounded contact
wrench or load-direction screen can explain and predict these false-positive
hold decisions before a full lift. Any follow-up must preserve the current
force ceiling and verifier limits, use a separately frozen evaluation, and
remain explicitly simulation-only.

## Authority

`admission_status=NOT_ADMITTED`, `metal=false`, `shop_release=false`, and
`viewport_admits=false`. No physical actuation, purchase, fabrication, or
external contact was performed.
