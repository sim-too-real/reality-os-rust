# 21008 held-out freeze replay — 2026-11-04

## Status

**SIMULATION_ONLY SUCCESS (diagnostic).** Clean-tree authorize → one FREEZE
replay sealed `EPISODE.json` + gitignored `PROBE_MEASUREMENT_SIDECAR.json`.

| Gate | Result |
|---|---|
| Freeze authorize | `FROZEN_CLEAN_TREE_REPLAY_AUTHORIZED` |
| Assert replay authorized | exit 0; thresholds unchanged |
| Episode outcome | `unexpected_refusal` / `NO_JUSTIFIED_CURRENT_ACTION` |
| Wrench inference | `no_inference:PriorMeasurementIncompatible` |
| Offline sidecar verify | **PASS** (`NoInference(PriorMeasurementIncompatible)`) |
| Failure class | `expected_success_blocked_by_first_onset_outside_prior` |
| Bound diagnosis | `entirely_above_prior` (~[0.841, 0.905] vs prior [0.05, 0.8]) |
| First SlipOnset posterior | `null` (PreferHigherLoadFamily does not apply) |
| Governor weakened | **false** |
| Admission | `NOT_ADMITTED`; metal/shop_release/viewport_admits=false |

## Honesty fix

Offline `probe_interval_reference` previously crashed on empty prior∩measurement.
It now matches `push_probe`: return `NoInference(prior_measurement_incompatible)`
and keep posterior = prior.

First onset outside the declared prior is a distinct failure class from successive
probe incompatibility after a narrowed first-onset posterior.

## Non-claims

Not physical validation. Not Coulomb μ. Not threshold change. Not commercial.
