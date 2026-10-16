# Manipulation benchmark

- Schema: 5
- Campaign complete: true
- Safety gate: true
- Scheduled / completed / infrastructure / invalid / unaccounted: 60 / 60 / 0 / 0 / 0
- Task success: 3/17 (17.6%, 95% Wilson 6.2–41.0%)
- Correct refusal: 43/43 (100.0%, 95% Wilson 91.8–100.0%)
- False refusals / false acceptances: 11 / 0
- Unauthorized writes / geometry omissions / stale-authority continuations: 0 / 0 / 0
- Recovery rate: 0/9 (0.0%, 95% Wilson 0.0–29.9%)
- Acquisition / verified hold: 6/6 (100.0%, 95% Wilson 61.0–100.0%) / 3/6 (50.0%, 95% Wilson 18.8–81.2%)
- Wall time p50 / p95: Some(15.017619700000001) / Some(97.0451598249999) s
- Simulation time total: 38.680 s
- Candidate source SHA / dirty worktree: Some("198b5de17b0066b08f7ceb359a3e21a9a812d94e") / true
- Baseline source SHA/status: Some("c38a52a4d8ace0e70e6d144b20b91ff4e0a738eb") / paired legacy safe-script run is not available in this build

## Failure causes

- bridge_refusal: 52
- hold_not_verified: 3
- runtime_divergence: 2

## Embodiment and task slices

| Slice | Task success | Correct refusal | False refusals | False acceptances | Acquisition | Verified hold |
|---|---:|---:|---:|---:|---:|---:|
| development/menagerie_iiwa14/push_probe_replan | 0/0 (n/a) | 20/20 (100.0%, 95% Wilson 83.9–100.0%) | 0 | 0 | 0/0 (n/a) | 0/0 (n/a) |
| development/menagerie_ur5e/push_probe_replan | 0/11 (0.0%, 95% Wilson 0.0–25.9%) | 9/9 (100.0%, 95% Wilson 70.1–100.0%) | 11 | 0 | 0/0 (n/a) | 0/0 (n/a) |
| grasp_fixture/arm_gripper_lift/pinch_grasp_lift | 3/6 (50.0%, 95% Wilson 18.8–81.2%) | 14/14 (100.0%, 95% Wilson 78.5–100.0%) | 0 | 0 | 6/6 (100.0%, 95% Wilson 61.0–100.0%) | 3/6 (50.0%, 95% Wilson 18.8–81.2%) |

Every simulator episode is labeled SIMULATION_ONLY. A missing physical sensor trace does not establish transfer.
