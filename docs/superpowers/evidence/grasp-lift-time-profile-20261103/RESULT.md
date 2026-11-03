# Grasp-lift trapezoidal profile result

**Status:** `NEGATIVE_PRIMARY_GATE; CANDIDATE_REJECTED; SIMULATION_ONLY; NOT_ADMITTED`
**Candidate source:** `d11ad829206b1cd29856255e73da6371fa707b97` (`main`)
**Candidate freeze commit:** `412a5b253992215a85c9460ac2525c959e29af20`
**Candidate freeze SHA-256:** `B88D369DE46DB27A615F68E528FC5FAF39BA12C9C74D0D50C6743218F5B23D00`
**Primary result SHA-256:** `45C61F5B93CD5349900F78F8EB009A97CA8D5A5B85A693BEBA97004F4B6B435C`

The profile was present in the candidate grasp witnesses and both frozen
primary controls executed with zero unauthorized writes. The primary gate
failed, so the seven-case candidate cohort was not replayed.

| Trial | Uniform profile | Trapezoidal profile | Candidate evidence |
|---|---|---|---|
| 21010:25 | Runtime divergence at waypoint 4; relative-pose error 20.099 mm | Runtime divergence at waypoint 4; relative-pose error 20.508 mm | Same 20 mm guard remained active; error increased 0.409 mm; peak finger force 1.100 N |
| 21006:21 | Hold refusal; object-follow residual 7.362 mm | Hold refusal; object-follow residual 10.137 mm | Residual increased 2.774 mm; peak finger force 1.9483 N against the unchanged 1.95 N ceiling |

The candidate did not clear its pre-registered 21010:25 gate. The seven
candidate replays were skipped. The frozen uniform baseline remains 4/7, but
there is no candidate cohort rate to compare with it. These two controls do
not establish a broader performance estimate.

The replay command completed one test invocation in 24.10 seconds:

```powershell
cargo test -p realityos-verify --test manipulation_benchmark replay_grasp_time_profile_frozen_inputs -- --ignored --nocapture
```

`CANDIDATE_PRIMARY.json` contains the full records for both controls,
including the profile-bound witnesses, force audits, failure guards, and
verifier results. The candidate is rejected and the uniform production
profile is restored in the following source commit. All results remain
`SIMULATION_ONLY`; no physical actuation, admission, or release claim follows.
