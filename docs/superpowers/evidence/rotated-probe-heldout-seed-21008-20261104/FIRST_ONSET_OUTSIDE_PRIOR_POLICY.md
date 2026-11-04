# 21008 first-onset-outside-prior policy comparison — 2026-11-04

## Question

For sealed 21008 bounds entirely above the declared prior, which research-only
first-onset policies unlock a push-proving friction interval without changing
Governor contact/force thresholds (1 mm / 0.5 N)?

## Measured (offline numbers)

| Policy | Outcome | Proves μ≥0.3? |
|---|---|---|
| `RetainDeclaredPrior` (live status quo) | `Rejected(PriorMeasurementIncompatible)` | **no** |
| `PreferHigherLoadFamilyOnIncompatible` | `Rejected` (no first-onset posterior) | **no** |
| `RefusePriorEnvelopeAsNotCredible` | named hole; no Updated interval | **no** |
| `NarrowToMeasurementOnEntirelyAbovePrior` | replace prior with ≈[0.841, 0.905] | **yes** |

Live `record_probe_measurement` stays on `RetainDeclaredPrior`. Narrow-to-measurement
is research-only and discards the declared prior envelope; it is **not** wired
into the Governor path in this experiment.

`NOT_ADMITTED`. Thresholds unchanged.

## Evidence

```powershell
cargo test -p realityos-semantics --test push_probe_inference first_onset_outside_prior -- --nocapture
py -3.10 -B -m unittest crates.verify.tests.test_first_onset_policy_comparison_21008
```

Machine-readable seal:
`FIRST_ONSET_POLICY_COMPARISON.json` (bound to `FRICTION_BOUND_DIAGNOSIS.json` and
`PRIOR_ENVELOPE_MISS_MARGIN.json`).
