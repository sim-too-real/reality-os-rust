# ROS 21008 next action from belief after reprob — 2026-11-05

## Question

After `ContinueProbeWithBelief` from after-reprobe feedback, does a typed
next-action gate that requires belief coverage of `mu_required` improve
complete-task sequencing without inventing μ, changing Governor thresholds, or
unsafe proceed?

## Competing paths (offline)

| Feedback | Next action |
|---|---|
| Continue + `proves_mu_required` | `ProceedCompleteTaskWithBelief` |
| Continue + not proves | `HoldRefuseInsufficientBeliefCoverage` |
| Correct refuse (still miss) | `HoldAfterCorrectRefuse` |
| Invalid / missing pending | `RejectedNotContinuing` |
| Blind proceed when not proves | `RejectedBlindProceedWithoutBeliefProof` |

Live writes still unauthorized. Governor thresholds unchanged.

## Classification

**VALIDATED_LEARNING** — offline matched trials. **SIMULATION_ONLY**.

## Verify

```
cargo test --offline -p realityos-semantics --test push_probe_inference matched_trials_next_action
```
