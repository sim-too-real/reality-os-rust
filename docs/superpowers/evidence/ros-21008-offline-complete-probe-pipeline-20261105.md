# ROS 21008 offline complete-probe pipeline — 2026-11-05

## Question

Does one offline pipeline composing miss-margin first-onset → RequestReprobe →
after-reprobe feedback → belief next-action improve complete-task sequencing
on sealed 21008 and held-out scenes without inventing μ, changing Governor
thresholds, or authorizing live writes?

## Competing paths (offline)

| Path | Outcome |
|---|---|
| Miss-margin refuse + RequestReprobe + recovered reprobe | `ProceedCompleteTaskWithBelief` |
| Miss-margin refuse + StopAsCorrectRefuse (+ unused reprobe) | `HoldAfterCorrectRefuse` (ablation) |
| RequestReprobe without measurement | `RejectedNotContinuing` |
| Held-out small miss (≤ margin) | Proceed without reprobe |
| Still-miss after reprobe | Hold after correct refuse |
| Claim live write from pipeline complete | **refused** |

Live writes still unauthorized. Governor thresholds unchanged. **SIMULATION_ONLY**.

## Classification

**VALIDATED_LEARNING** — offline matched trials. **SIMULATION_ONLY**.

## Verify

```
cargo test --offline -p realityos-semantics --test push_probe_inference matched_trials_offline_complete_probe_pipeline
```
