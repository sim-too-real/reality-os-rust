# Manipulation benchmark

- Schema: 7
- Campaign complete: true
- Safety gate: true
- Scheduled / completed / infrastructure / invalid / unaccounted: 60 / 60 / 0 / 0 / 0
- Task success: 14/31 (45.2%, 95% Wilson 29.2–62.2%)
- Correct refusal: 27/29 (93.1%, 95% Wilson 78.0–98.1%)
- False refusals / false acceptances / evaluation-label mismatches: 9 / 0 / 2
- Unauthorized writes / geometry omissions / stale-authority continuations / unverified wrench stops / invalid compensation-audit trials: 0 / 0 / 0 / 0 / 0
- Recovery rate: 11/20 (55.0%, 95% Wilson 34.2–74.2%)
- Acquisition / verified hold: 6/6 (100.0%, 95% Wilson 61.0–100.0%) / 3/6 (50.0%, 95% Wilson 18.8–81.2%)
- Wall time p50 / p95: Some(14.1195418) / Some(328.616757745) s
- Simulation time total: 1058.908 s
- Execution build profile: debug_assertions_disabled
- Candidate source SHA / dirty worktree: Some("09e4c49ae72f612208006c3f308d70a8b272eb2a") / true
- Baseline source SHA/status: Some("18eac3011d82a33a49e0cdf4a3303ed5d222d143") / suite_v3_paired_preserved_pilot

## Failure causes

- bridge_refusal: 35
- evaluation_label_mismatch: 2
- force_monitor_abort: 5
- hold_not_verified: 3
- runtime_divergence: 1

## Embodiment and task slices

| Slice | Task success | Correct refusal | False refusals | False acceptances | Acquisition | Verified hold |
|---|---:|---:|---:|---:|---:|---:|
| development/menagerie_iiwa14/push_probe_replan | 7/12 (58.3%, 95% Wilson 32.0–80.7%) | 7/8 (87.5%, 95% Wilson 52.9–97.8%) | 1 | 0 | 0/0 (n/a) | 0/0 (n/a) |
| development/menagerie_ur5e/push_probe_replan | 4/12 (33.3%, 95% Wilson 13.8–60.9%) | 7/8 (87.5%, 95% Wilson 52.9–97.8%) | 7 | 0 | 0/0 (n/a) | 0/0 (n/a) |
| grasp_fixture/arm_gripper_lift/pinch_grasp_lift | 3/7 (42.9%, 95% Wilson 15.8–75.0%) | 13/13 (100.0%, 95% Wilson 77.2–100.0%) | 1 | 0 | 6/6 (100.0%, 95% Wilson 61.0–100.0%) | 3/6 (50.0%, 95% Wilson 18.8–81.2%) |

Every simulator episode is labeled SIMULATION_ONLY. A missing physical sensor trace does not establish transfer.

## Force/torque evidence

- Push episodes / episodes with samples / samples: 40 / 20 / 516150
- Compensation-audit invalid trials / samples: 0 / 0
- Strict friction narrowings / inference refusals: 12 / 3
- Force-cap trips / stale-or-missing-sample aborts: 1 / 0
- Unexpected-contact / actuator-saturation aborts: 4 / 0
- Strict narrowing rate among sampled episodes: 12/20 (60.0%, 95% Wilson 38.7–78.1%)

## Paired campaign delta

- Matched randomized-input keys: 60
- Scenario manifest digest: `8e7755a27b7c340d2104d6419ff8717e94f1372d74a81b438aa8211f9a706bd4`
- Paired bootstrap resamples: 10000

| Embodiment | Expected success | Information-bearing probe | Justified follow-up |
|---|---|---|---|
| development:menagerie_iiwa14 (n=12) | 58.3% → 58.3% (+0.0 pp, paired 95% bootstrap 0.0–0.0 pp) | 58.3% → 58.3% (+0.0 pp, paired 95% bootstrap 0.0–0.0 pp) | 58.3% → 58.3% (+0.0 pp, paired 95% bootstrap 0.0–0.0 pp) |
| development:menagerie_ur5e (n=12) | 33.3% → 33.3% (+0.0 pp, paired 95% bootstrap 0.0–0.0 pp) | 50.0% → 41.7% (-8.3 pp, paired 95% bootstrap -25.0–0.0 pp) | 33.3% → 33.3% (+0.0 pp, paired 95% bootstrap 0.0–0.0 pp) |

Screening gate: **FAIL**; gain at least 10 pp: false; no metric regressed more than 5 pp: false; best gain: +0.0 pp; worst regression: -8.3 pp.
