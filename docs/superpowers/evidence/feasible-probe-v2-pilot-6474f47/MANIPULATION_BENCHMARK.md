# Manipulation benchmark

- Schema: 7
- Campaign complete: true
- Safety gate: true
- Scheduled / completed / infrastructure / invalid / unaccounted: 60 / 60 / 0 / 0 / 0
- Task success: 2/29 (6.9%, 95% Wilson 1.9–22.0%)
- Correct refusal: 31/31 (100.0%, 95% Wilson 89.0–100.0%)
- False refusals / false acceptances: 20 / 0
- Unauthorized writes / geometry omissions / stale-authority continuations / unverified wrench stops: 0 / 0 / 0 / 0
- Recovery rate: 0/24 (0.0%, 95% Wilson 0.0–13.8%)
- Acquisition / verified hold: 5/5 (100.0%, 95% Wilson 56.6–100.0%) / 2/5 (40.0%, 95% Wilson 11.8–76.9%)
- Wall time p50 / p95: Some(16.15734645) / Some(296.1936413599999) s
- Simulation time total: 679.326 s
- Candidate source SHA / dirty worktree: Some("6474f4765f426f864669a0cf04bb27c4eb88c2bd") / true
- Baseline source SHA/status: None / suite_v2_pilot_screening_not_compared

## Failure causes

- bridge_refusal: 48
- force_monitor_abort: 4
- hold_not_verified: 3
- runtime_divergence: 3

## Embodiment and task slices

| Slice | Task success | Correct refusal | False refusals | False acceptances | Acquisition | Verified hold |
|---|---:|---:|---:|---:|---:|---:|
| development/menagerie_iiwa14/push_probe_replan | 0/12 (0.0%, 95% Wilson 0.0–24.2%) | 8/8 (100.0%, 95% Wilson 67.6–100.0%) | 8 | 0 | 0/0 (n/a) | 0/0 (n/a) |
| development/menagerie_ur5e/push_probe_replan | 0/12 (0.0%, 95% Wilson 0.0–24.2%) | 8/8 (100.0%, 95% Wilson 67.6–100.0%) | 12 | 0 | 0/0 (n/a) | 0/0 (n/a) |
| grasp_fixture/arm_gripper_lift/pinch_grasp_lift | 2/5 (40.0%, 95% Wilson 11.8–76.9%) | 15/15 (100.0%, 95% Wilson 79.6–100.0%) | 0 | 0 | 5/5 (100.0%, 95% Wilson 56.6–100.0%) | 2/5 (40.0%, 95% Wilson 11.8–76.9%) |

Every simulator episode is labeled SIMULATION_ONLY. A missing physical sensor trace does not establish transfer.

## Force/torque evidence

- Push episodes / episodes with samples / samples: 40 / 24 / 82539
- Strict friction narrowings / inference refusals: 0 / 17
- Force-cap trips / stale-or-missing-sample aborts: 4 / 0
- Strict narrowing rate among sampled episodes: 0/24 (0.0%, 95% Wilson 0.0–13.8%)
