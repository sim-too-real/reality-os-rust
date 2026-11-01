# Manipulation benchmark

- Schema: 7
- Campaign complete: true
- Safety gate: true
- Scheduled / completed / infrastructure / invalid / unaccounted: 1 / 1 / 0 / 0 / 0
- Task success: 1/1 (100.0%, 95% Wilson 20.7–100.0%)
- Correct refusal: 0/0 (n/a)
- False refusals / false acceptances / evaluation-label mismatches: 0 / 0 / 0
- Unauthorized writes / geometry omissions / stale-authority continuations / unverified wrench stops / invalid compensation-audit trials: 0 / 0 / 0 / 0 / 0
- Recovery rate: 1/1 (100.0%, 95% Wilson 20.7–100.0%)
- Acquisition / verified hold: 0/0 (n/a) / 0/0 (n/a)
- Wall time p50 / p95: Some(182.3573142) / Some(182.3573142) s
- Simulation time total: 58.684 s
- Execution build profile: debug_assertions_disabled
- Candidate source SHA / dirty worktree: Some("3023a0e5ebaebc9445ca4c147ab9dee657cf688f") / false
- Baseline source SHA/status: None / historical_replay_not_compared

## Embodiment and task slices

| Slice | Task success | Correct refusal | False refusals | False acceptances | Acquisition | Verified hold |
|---|---:|---:|---:|---:|---:|---:|
| development/menagerie_iiwa14/push_probe_replan | 1/1 (100.0%, 95% Wilson 20.7–100.0%) | 0/0 (n/a) | 0 | 0 | 0/0 (n/a) | 0/0 (n/a) |

Every simulator episode is labeled SIMULATION_ONLY. A missing physical sensor trace does not establish transfer.

## Force/torque evidence

- Push episodes / episodes with samples / samples: 1 / 1 / 29342
- Compensation-audit invalid trials / samples: 0 / 0
- Strict friction narrowings / inference refusals: 1 / 0
- Force-cap trips / stale-or-missing-sample aborts: 0 / 0
- Unexpected-contact / actuator-saturation aborts: 0 / 0
- Strict narrowing rate among sampled episodes: 1/1 (100.0%, 95% Wilson 20.7–100.0%)
