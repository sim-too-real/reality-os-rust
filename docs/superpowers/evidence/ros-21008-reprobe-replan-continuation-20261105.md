# ROS 21008 reprobe/replan continuation — 2026-11-05

## Question

After a correct prior-envelope refuse (miss-margin gate), does a typed
**RequestReprobe** continuation improve complete-task handling vs stop-only
refuse, without inventing μ or weakening Governor?

## Competing interventions (offline)

| Path | 21008 gated refuse | Held-out small miss |
|---|---|---|
| StopAsCorrectRefuse | CorrectRefusePriorEnvelopeNotCredible | ContinueProbeWithBelief |
| RequestReprobe | RequestReprobeAfterPriorEnvelopeRefuse | ContinueProbeWithBelief |

RequestReprobe does **not** become ContinueProbeWithBelief (no unsafe acceptance).
Governor thresholds unchanged. Live `record_probe_measurement` still Retain.

## Classification

**VALIDATED_LEARNING** — offline matched trials. **SIMULATION_ONLY**. Not live replan authorization.

## Verify

```
cargo test --offline -p realityos-semantics --test push_probe_inference matched_trials_reprobe_replan
```
