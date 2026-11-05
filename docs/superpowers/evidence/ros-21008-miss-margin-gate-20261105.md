# ROS 21008 miss-margin gate vs Narrow — 2026-11-05

## Question

Does a relative prior-envelope miss gate change complete-task continuation vs bare Narrow on sealed 21008 numbers without weakening Governor?

## Competing interventions (offline)

| Policy | 21008 (rel≈0.12) | Held-out small miss (rel≈0.0125) |
|---|---|---|
| NarrowToMeasurementOnEntirelyAbovePrior | ContinueProbeWithBelief (proves μ) | ContinueProbeWithBelief |
| NarrowWhenRelativeMissAtMost(0.05) | CorrectRefusePriorEnvelopeNotCredible | ContinueProbeWithBelief |
| RetainDeclaredPrior | CorrectRefuseIncompatiblePrior | (not primary compare) |

Governor thresholds unchanged. Live `record_probe_measurement` still Retain.

## Classification

**VALIDATED_LEARNING** — offline matched trials. **SIMULATION_ONLY**. Not live Narrow authorization.

## Verify

```
cargo test --offline -p realityos-semantics --test push_probe_inference matched_trials_miss_margin
```
