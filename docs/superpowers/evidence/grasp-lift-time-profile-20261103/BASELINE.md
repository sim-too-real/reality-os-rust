# Grasp-lift trapezoidal profile baseline

**Status:** `BASELINE_COMPLETE; SIMULATION_ONLY; NOT_ADMITTED`  
**Source:** `77b5b9d1e545fa4fab601649d473e886ae2a1d2a` (`main`)  
**Pre-run freeze:** `96ad0115836be21ae142cfcfe06530439045da43`  
**Freeze SHA-256:** `AAC36CB177DEA7F7523DDB371372F770409856F3C6B0D7A327D9A41BE14BFB82`  
**Raw baseline SHA-256:** `6877DEDA6D344973D6C28196E5BB076EFE85F6C63AA37BAB3D9317ACEC48CDD1`

The seven out-of-archive inputs were replayed once with the unchanged uniform
profile, 25 mm endpoint, and 50 simulator steps per interpolation waypoint.
All inputs matched their frozen SHA-256, trial digest, and paired key. All
seven episodes recorded zero unauthorized writes. The maximum observed
per-finger lift force remained below the unchanged 1.95 N ceiling.

| Trial | Outcome | Hold verified | Object-follow residual | Peak lift force | First runtime guard |
|---|---|---:|---:|---:|---|
| 22000:16 | `action_failed` | no | 18.883 mm | 0.310 N | — |
| 22000:20 | `success` | yes | 1.080 mm | 1.682 N | — |
| 22000:21 | `success` | yes | 1.685 mm | 1.779 N | — |
| 22000:26 | `action_failed` | no | 5.026 mm | 1.801 N | — |
| 22000:30 | `success` | yes | 2.065 mm | 1.760 N | — |
| 22000:31 | `unexpected_refusal` | no | not completed | 1.310 N | `object_tool_relative_pose_exceeded_20mm`, 20.057 mm at waypoint 4 |
| 22001:15 | `success` | yes | 3.727 mm | 1.758 N | — |

The uniform baseline is **4/7**. Under the pre-registered decision rule, the
candidate must therefore reach at least **5/7** verified successes, exceed
the 3/7 historical rate, and complete the frozen 21010:25 control without
runtime divergence. A claimed success must still pass the independent hold
verifier and unchanged force ceiling.

The one baseline test replayed the cohort in seed/index order and completed in
82.92 seconds:

```powershell
cargo test -p realityos-verify --test manipulation_benchmark replay_grasp_time_profile_frozen_inputs -- --ignored --nocapture
```

`BASELINE.json` contains the complete seven episode records and their witness,
decision, contact, and verifier evidence. No physical hardware, external
service, or paid compute was used.
