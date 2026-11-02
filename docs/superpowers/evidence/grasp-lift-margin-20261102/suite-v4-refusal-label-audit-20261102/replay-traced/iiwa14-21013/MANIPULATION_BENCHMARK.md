# Manipulation benchmark

- Schema: 7
- Campaign complete: true
- Safety gate: false
- Scheduled / completed / infrastructure / invalid / unaccounted: 1 / 1 / 0 / 0 / 0
- Task success: 0/0 (n/a)
- Correct refusal: 0/1 (0.0%, 95% Wilson 0.0–79.3%)
- False refusals / false acceptances / evaluation-label mismatches: 0 / 1 / 0
- Unauthorized writes / geometry omissions / stale-authority continuations / unverified wrench stops / invalid compensation-audit trials: 0 / 0 / 0 / 0 / 0
- Recovery rate: 0/0 (n/a)
- Acquisition / verified hold: 0/0 (n/a) / 0/0 (n/a)
- Wall time p50 / p95: Some(20.5583604) / Some(20.5583604) s
- Simulation time total: 0.000 s
- Execution build profile: debug_assertions_disabled
- Candidate source SHA / dirty worktree: Some("32b038de19bdf7d886b8bfb8d0322ebb86a2b5c1") / true
- Baseline source SHA/status: None / historical_replay_not_compared

## Failure causes

- false_acceptance: 1

## Embodiment and task slices

| Slice | Task success | Correct refusal | False refusals | False acceptances | Acquisition | Verified hold |
|---|---:|---:|---:|---:|---:|---:|
| development/menagerie_iiwa14/push_probe_replan | 0/0 (n/a) | 0/1 (0.0%, 95% Wilson 0.0–79.3%) | 0 | 1 | 0/0 (n/a) | 0/0 (n/a) |

Every simulator episode is labeled SIMULATION_ONLY. A missing physical sensor trace does not establish transfer.

## Force/torque evidence

- Push episodes / episodes with samples / samples: 1 / 0 / 0
- Compensation-audit invalid trials / samples: 0 / 0
- Strict friction narrowings / inference refusals: 0 / 0
- Force-cap trips / stale-or-missing-sample aborts: 0 / 0
- Unexpected-contact / actuator-saturation aborts: 0 / 0
- Strict narrowing rate among sampled episodes: 0/0 (n/a)
