# Suite v3 paired comparison after exact float parsing

This report compares the preserved 60-trial baseline and candidate episodes after enabling `serde_json`'s `float_roundtrip` feature for `realityos-verify`. The old report could not validate a persisted paired key after JSON reload because one `f64` input changed by one ULP. Both preserved campaigns already have matching stamped keys; exact parsing validates all 60 without rerunning MuJoCo.

## Reproduction

```powershell
cargo test -p realityos-verify --test pair_key_roundtrip -- --nocapture
```

The focused test loads both preserved trial directories, validates their saved keys against the JSON-round-tripped inputs, computes the paired delta, and checks that changing a serialized pose invalidates the key.

## Result

- Matched randomized-input keys: 60/60.
- Expected-success, information-bearing-probe, and justified-follow-up deltas: 0.0 percentage points on both development embodiments.
- Paired screening gate: `false` (no gain of at least 10 percentage points); no metric regressed by more than 5 points.
- Simulations rerun: no. This is an analysis of the two preserved complete campaigns.
- Safety accounting in the candidate report remains: 0 false acceptances and 0 unauthorized writes. Two evaluation-label mismatches remain and are described in `result.json`.
- Evidence remains `SIMULATION_ONLY`; this is not physical transfer or hardware validation.

The comparison implementation and round-trip regression test are present in the current local worktree; this report does not claim those dirty source changes are committed.
