# Manipulation benchmark

- Schema: 7
- Campaign complete: true
- Safety gate: false
- Scheduled / completed / infrastructure / invalid / unaccounted: 42 / 42 / 0 / 0 / 0
- Task success: 13/23 (56.5%, 95% Wilson 36.8–74.4%)
- Correct refusal: 17/19 (89.5%, 95% Wilson 68.6–97.1%)
- False refusals / false acceptances / evaluation-label mismatches: 6 / 2 / 0
- Unauthorized writes / geometry omissions / stale-authority continuations / unverified wrench stops / invalid compensation-audit trials: 0 / 0 / 0 / 0 / 0
- Recovery rate: 10/15 (66.7%, 95% Wilson 41.7–84.8%)
- Acquisition / verified hold: 4/4 (100.0%, 95% Wilson 51.0–100.0%) / 3/4 (75.0%, 95% Wilson 30.1–95.4%)
- Wall time p50 / p95: Some(16.8579926) / Some(194.00503077499974) s
- Simulation time total: 868.278 s
- Execution build profile: debug_assertions_disabled
- Candidate source SHA / dirty worktree: Some("32b038de19bdf7d886b8bfb8d0322ebb86a2b5c1") / true
- Baseline source SHA/status: None / suite_v4_pilot_screening_not_compared

## Failure causes

- bridge_refusal: 22
- false_acceptance: 2
- force_monitor_abort: 3
- hold_not_verified: 1
- runtime_divergence: 1

## Embodiment and task slices

| Slice | Task success | Correct refusal | False refusals | False acceptances | Acquisition | Verified hold |
|---|---:|---:|---:|---:|---:|---:|
| development/menagerie_iiwa14/push_probe_replan | 7/9 (77.8%, 95% Wilson 45.3–93.7%) | 4/5 (80.0%, 95% Wilson 37.6–96.4%) | 0 | 1 | 0/0 (n/a) | 0/0 (n/a) |
| development/menagerie_ur5e/push_probe_replan | 3/9 (33.3%, 95% Wilson 12.1–64.6%) | 4/5 (80.0%, 95% Wilson 37.6–96.4%) | 5 | 1 | 0/0 (n/a) | 0/0 (n/a) |
| grasp_fixture/arm_gripper_lift/pinch_grasp_lift | 3/5 (60.0%, 95% Wilson 23.1–88.2%) | 9/9 (100.0%, 95% Wilson 70.1–100.0%) | 1 | 0 | 4/4 (100.0%, 95% Wilson 51.0–100.0%) | 3/4 (75.0%, 95% Wilson 30.1–95.4%) |

Every simulator episode is labeled SIMULATION_ONLY. A missing physical sensor trace does not establish transfer.

## Force/torque evidence

- Push episodes / episodes with samples / samples: 28 / 15 / 424637
- Compensation-audit invalid trials / samples: 0 / 0
- Strict friction narrowings / inference refusals: 10 / 2
- Force-cap trips / stale-or-missing-sample aborts: 1 / 0
- Unexpected-contact / actuator-saturation aborts: 2 / 0
- Strict narrowing rate among sampled episodes: 10/15 (66.7%, 95% Wilson 41.7–84.8%)
