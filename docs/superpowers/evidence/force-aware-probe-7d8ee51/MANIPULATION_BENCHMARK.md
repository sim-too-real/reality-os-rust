# Manipulation benchmark

- Schema: 6
- Campaign complete: true
- Safety gate: true
- Scheduled / completed / infrastructure / invalid / unaccounted: 60 / 60 / 0 / 0 / 0
- Task success: 3/17 (17.6%, 95% Wilson 6.2–41.0%)
- Correct refusal: 43/43 (100.0%, 95% Wilson 91.8–100.0%)
- False refusals / false acceptances: 9 / 0
- Unauthorized writes / geometry omissions / stale-authority continuations / unverified wrench stops: 0 / 0 / 0 / 0
- Recovery rate: 0/9 (0.0%, 95% Wilson 0.0–29.9%)
- Acquisition / verified hold: 6/6 (100.0%, 95% Wilson 61.0–100.0%) / 3/6 (50.0%, 95% Wilson 18.8–81.2%)
- Wall time p50 / p95: Some(11.33766445) / Some(55.22870958999996) s
- Simulation time total: 157.550 s
- Candidate source SHA / dirty worktree: Some("7d8ee51582f2e6718ad1dbdc40a871b2cc5f424d") / false
- Baseline source SHA/status: Some("198b5de17b0066b08f7ceb359a3e21a9a812d94e") / paired_schema_v5_exact_keys

## Failure causes

- bridge_refusal: 52
- force_monitor_abort: 2
- hold_not_verified: 3

## Embodiment and task slices

| Slice | Task success | Correct refusal | False refusals | False acceptances | Acquisition | Verified hold |
|---|---:|---:|---:|---:|---:|---:|
| development/menagerie_iiwa14/push_probe_replan | 0/0 (n/a) | 20/20 (100.0%, 95% Wilson 83.9–100.0%) | 0 | 0 | 0/0 (n/a) | 0/0 (n/a) |
| development/menagerie_ur5e/push_probe_replan | 0/11 (0.0%, 95% Wilson 0.0–25.9%) | 9/9 (100.0%, 95% Wilson 70.1–100.0%) | 9 | 0 | 0/0 (n/a) | 0/0 (n/a) |
| grasp_fixture/arm_gripper_lift/pinch_grasp_lift | 3/6 (50.0%, 95% Wilson 18.8–81.2%) | 14/14 (100.0%, 95% Wilson 78.5–100.0%) | 0 | 0 | 6/6 (100.0%, 95% Wilson 61.0–100.0%) | 3/6 (50.0%, 95% Wilson 18.8–81.2%) |

Every simulator episode is labeled SIMULATION_ONLY. A missing physical sensor trace does not establish transfer.

## Force/torque evidence

- Push episodes / episodes with samples / samples: 40 / 9 / 16852
- Strict friction narrowings / inference refusals: 0 / 7
- Force-cap trips / stale-or-missing-sample aborts: 2 / 0
- Strict narrowing rate among sampled episodes: 0/9 (0.0%, 95% Wilson 0.0–29.9%)

## Paired campaign delta

- Matched randomized-input keys: 60
- Scenario manifest digest: `7edf551e136fb4912cccbcd1c964ef4a4aae26b6d0b7796c4b2d1a980725cf67`
- Paired bootstrap resamples: 10000

| Embodiment | Expected success | Information-bearing probe | Justified follow-up |
|---|---|---|---|
| development:menagerie_ur5e (n=11) | 0.0% → 0.0% (+0.0 pp, paired 95% bootstrap 0.0–0.0 pp) | 0.0% → 0.0% (+0.0 pp, paired 95% bootstrap 0.0–0.0 pp) | 0.0% → 0.0% (+0.0 pp, paired 95% bootstrap 0.0–0.0 pp) |

Screening gate: **FAIL**; gain at least 10 pp: false; no metric regressed more than 5 pp: true; best gain: +0.0 pp; worst regression: +0.0 pp.
