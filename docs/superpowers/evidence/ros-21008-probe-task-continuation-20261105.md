# ROS 21008 probe-task continuation — 2026-11-05

## Question

Do Retain vs Narrow first-onset policies change a complete probe-task
continuation decision on sealed 21008 numbers without weakening Governor?

## Result

| Policy | Task continuation | Proves μ≥0.3? |
|---|---|---|
| RetainDeclaredPrior | `CorrectRefuseIncompatiblePrior` | n/a (refused) |
| NarrowToMeasurementOnEntirelyAbovePrior | `ContinueProbeWithBelief` | **yes** |

Governor thresholds unchanged. Live `record_probe_measurement` still Retain.
Classification: **VALIDATED_LEARNING** (offline task-continuation mapping).
**SIMULATION_ONLY** / **NOT** PHYSICALLY VALIDATED.

## Verify

```powershell
cargo test --offline -p realityos-semantics --test push_probe_inference probe_task_continuation
```
