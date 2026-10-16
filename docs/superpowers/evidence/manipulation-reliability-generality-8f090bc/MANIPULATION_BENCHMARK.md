# Manipulation benchmark

- Schema: 5
- Campaign complete: true
- Safety gate: true
- Scheduled / completed / infrastructure / invalid / unaccounted: 300 / 300 / 0 / 0 / 0
- Task success: 15/94 (16.0%, 95% Wilson 9.9–24.7%)
- Correct refusal: 206/206 (100.0%, 95% Wilson 98.2–100.0%)
- False refusals / false acceptances: 57 / 0
- Unauthorized writes / geometry omissions / stale-authority continuations: 0 / 0 / 0
- Recovery rate: 1/46 (2.2%, 95% Wilson 0.4–11.3%)
- Acquisition / verified hold: 35/35 (100.0%, 95% Wilson 90.1–100.0%) / 14/35 (40.0%, 95% Wilson 25.6–56.4%)
- Wall time p50 / p95: Some(14.4989504) / Some(105.26930358500009) s
- Simulation time total: 199.890 s
- Candidate source SHA / dirty worktree: Some("8f090bce8bd3f0041919c1bca1f37245a74d5f41") / false
- Baseline source SHA/status: Some("c38a52a4d8ace0e70e6d144b20b91ff4e0a738eb") / paired legacy safe-script run is not available in this build

## Failure causes

- bridge_refusal: 256
- hold_not_verified: 21
- runtime_divergence: 7
- unsupported_control_mapping: 1

## Embodiment and task slices

| Slice | Task success | Correct refusal | False refusals | False acceptances | Acquisition | Verified hold |
|---|---:|---:|---:|---:|---:|---:|
| development/menagerie_iiwa14/push_probe_replan | 0/0 (n/a) | 100/100 (100.0%, 95% Wilson 96.3–100.0%) | 0 | 0 | 0/0 (n/a) | 0/0 (n/a) |
| development/menagerie_ur5e/push_probe_replan | 1/59 (1.7%, 95% Wilson 0.3–9.0%) | 41/41 (100.0%, 95% Wilson 91.4–100.0%) | 57 | 0 | 0/0 (n/a) | 0/0 (n/a) |
| grasp_fixture/arm_gripper_lift/pinch_grasp_lift | 14/35 (40.0%, 95% Wilson 25.6–56.4%) | 65/65 (100.0%, 95% Wilson 94.4–100.0%) | 0 | 0 | 35/35 (100.0%, 95% Wilson 90.1–100.0%) | 14/35 (40.0%, 95% Wilson 25.6–56.4%) |

Every simulator episode is labeled SIMULATION_ONLY. A missing physical sensor trace does not establish transfer.
