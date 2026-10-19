# Force-observable v3 pilot analysis

Candidate: `4e5e6fc35ca7260974a2b2f5e82b1617a8eeb071` (clean worktree at run time)

The 20-seed campaign completed all 60 scheduled MuJoCo trials: 60 completed, zero infrastructure failures, zero invalid trials, and zero unaccounted records. The safety gate **failed** and the command exited with status 1. The paired screening gate was **not evaluated** because this is the first v3 baseline; the report records `suite_v3_pilot_screening_not_compared`, so these results do not establish a paired improvement.

| Slice | Task success | Correct refusal | False refusals | False acceptances | Acquisition | Verified hold |
|---|---:|---:|---:|---:|---:|---:|
| UR5e push/probe | 4/12 | 7/8 | 7 | 1 | n/a | n/a |
| IIWA14 push/probe | 7/12 | 7/8 | 1 | 1 | n/a | n/a |
| Gripper lift | 3/7 | 13/13 | 0 | 0 | 7/7 | 3/7 |
| **Overall** | **14/31** | **27/29** | **8** | **2** | **7/7** | **3/7** |

All four grasp hold failures followed verified acquisition and were classified as `SLIP`. The two false acceptances were low-mass expected-refusal cases in which the reasoner proposed `probe:+x`, but the scoped grant refused it with `SCOPED_GRANT_DOES_NOT_COVER_PROBE`; both had zero control writes. The eight false refusals were expected-success cases rejected as `NO_JUSTIFIED_CURRENT_ACTION`. Decision audits identify unresolved contact extents/support collision, unstable tool orientation or unintended contact, and post-probe grant/goal-action coverage as recurring causes.

Five expected-success push trials stopped under the monitor: four on unexpected tool-subtree contact and one at a 20 N force cap (measured conservative upper bound 22.42 N). The stops were verified. Across the campaign there were zero unauthorized writes, geometry omissions, stale-authority continuations, or unverified wrench stops. Thus the safety stop path remained effective, while availability and task completion remained below the screening gate.

Force evidence was present in 20 of 40 push episodes and contained 516,668 samples. Thirteen of those 20 sampled episodes narrowed the support-friction interval; two inference attempts returned no inference. Recovery was 11/20 (55%). Overall wall-time p50 was 35.86 s and p95 was 555.45 s; the slowest episode took 1,768.40 s (29.5 min). These runtimes make the current positive-push path impractical for a real-time robot loop even when the episode eventually completes.

The next engineering pass should preserve the force caps and fail-closed behavior while addressing, in order: real-time planning budgets and the multi-minute tail; post-probe authorization and positive-push geometric availability on both arms; force-safe approach/trajectory generation to avoid monitor trips without relaxing limits; and grasp force/geometry that improves verified hold rather than weakening the slip check. Add a pre-probe observability check so low-load cases refuse before asking for authority the current grant cannot provide. Re-run the public screening campaign after those changes. Do not run or inspect holdout data until the public safety and availability gates pass.

Every record is labeled `SIMULATION_ONLY`. This pilot provides no evidence of physical sensor performance or transfer, and the physical-transfer gate remains open.
