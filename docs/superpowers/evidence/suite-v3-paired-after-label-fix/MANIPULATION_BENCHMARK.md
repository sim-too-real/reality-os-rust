# Manipulation benchmark

- Schema: 7
- Campaign complete: true
- Safety gate: true
- Scheduled / completed / infrastructure / invalid / unaccounted: 60 / 60 / 0 / 0 / 0
- Task success: 14/31 (45.2%, 95% Wilson 29.2–62.2%)
- Correct refusal: 27/29 (93.1%, 95% Wilson 78.0–98.1%)
- False refusals / false acceptances / evaluation-label mismatches: 8 / 0 / 2
- Unauthorized writes / geometry omissions / stale-authority continuations / unverified wrench stops: 0 / 0 / 0 / 0
- Recovery rate: 11/20 (55.0%, 95% Wilson 34.2–74.2%)
- Acquisition / verified hold: 7/7 (100.0%, 95% Wilson 64.6–100.0%) / 3/7 (42.9%, 95% Wilson 15.8–75.0%)
- Wall time p50 / p95: Some(16.657905550000002) / Some(466.39029139999985) s
- Simulation time total: 1059.944 s
- Candidate source SHA / dirty worktree: Some("18eac3011d82a33a49e0cdf4a3303ed5d222d143") / false
- Baseline source SHA/status: Some("4e5e6fc35ca7260974a2b2f5e82b1617a8eeb071") / suite_v3_pairing_failed: invalid benchmark trial: schema-v7 trial is missing or has a mismatched paired_trial_key

## Failure causes

- bridge_refusal: 35
- evaluation_label_mismatch: 2
- force_monitor_abort: 5
- hold_not_verified: 4

## Embodiment and task slices

| Slice | Task success | Correct refusal | False refusals | False acceptances | Acquisition | Verified hold |
|---|---:|---:|---:|---:|---:|---:|
| development/menagerie_iiwa14/push_probe_replan | 7/12 (58.3%, 95% Wilson 32.0–80.7%) | 7/8 (87.5%, 95% Wilson 52.9–97.8%) | 1 | 0 | 0/0 (n/a) | 0/0 (n/a) |
| development/menagerie_ur5e/push_probe_replan | 4/12 (33.3%, 95% Wilson 13.8–60.9%) | 7/8 (87.5%, 95% Wilson 52.9–97.8%) | 7 | 0 | 0/0 (n/a) | 0/0 (n/a) |
| grasp_fixture/arm_gripper_lift/pinch_grasp_lift | 3/7 (42.9%, 95% Wilson 15.8–75.0%) | 13/13 (100.0%, 95% Wilson 77.2–100.0%) | 0 | 0 | 7/7 (100.0%, 95% Wilson 64.6–100.0%) | 3/7 (42.9%, 95% Wilson 15.8–75.0%) |

Every simulator episode is labeled SIMULATION_ONLY. A missing physical sensor trace does not establish transfer.

## Force/torque evidence

- Push episodes / episodes with samples / samples: 40 / 20 / 516668
- Strict friction narrowings / inference refusals: 13 / 2
- Force-cap trips / stale-or-missing-sample aborts: 1 / 0
- Unexpected-contact / actuator-saturation aborts: 4 / 0
- Strict narrowing rate among sampled episodes: 13/20 (65.0%, 95% Wilson 43.3–81.9%)
