# ROS 21008 after-reprobe feedback — 2026-11-05

## Question

After `RequestReprobeAfterPriorEnvelopeRefuse`, does a typed feedback step that
incorporates a **new** measurement improve complete-task sequencing without
unsafe Continue, inventing μ, or changing Governor thresholds?

## Competing paths (offline)

| Pending | New measurement | Decision |
|---|---|---|
| Stop refuse (no RequestReprobe) | any | RejectedMissingPendingReprobe |
| RequestReprobe | still large miss above prior | CorrectRefusePriorEnvelopeStillNotCredible |
| RequestReprobe | intersects prior | ContinueProbeWithBelief |
| RequestReprobe | NaN bounds | RejectedInvalidReprobeMeasurement |

Second refuse does **not** auto-loop another RequestReprobe. Live writes still unauthorized.

## Classification

**VALIDATED_LEARNING** — offline matched trials. **SIMULATION_ONLY**.

## Verify

```
cargo test --offline -p realityos-semantics --test push_probe_inference matched_trials_after_reprobe
```
