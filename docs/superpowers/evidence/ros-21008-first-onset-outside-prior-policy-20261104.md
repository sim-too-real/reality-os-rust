# ROS 21008 first-onset-outside-prior policy — 2026-11-04

## Question

Does an offline first-onset policy split RetainDeclaredPrior vs
NarrowToMeasurementOnEntirelyAbovePrior vs RefusePriorEnvelope, on sealed 21008
numbers, without PreferHigherLoadFamily and without weakening Governor thresholds?

## Measured

| Arm | Result |
|---|---|
| PreferHigherLoadFamily | rejects (null first SlipOnset) |
| RetainDeclaredPrior | rejects PriorMeasurementIncompatible |
| RefusePriorEnvelope | named hole; no Updated |
| NarrowToMeasurement | accepts ≈[0.841, 0.905]; proves μ≥0.3 offline |
| Live wire | **not** changed; retain declared prior |

`NOT_ADMITTED`; metal/shop_release/viewport_admits=false.

## Classification

**VALIDATED_LEARNING** — named policy space for first-onset-above-prior; live
Governor path unchanged.
