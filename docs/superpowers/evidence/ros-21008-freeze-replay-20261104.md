# ROS 21008 freeze replay — 2026-11-04

## Question

After clean-tree authorize, does one FREEZE replay of seed 21008 seal a
probe-measurement sidecar and classify PriorMeasurementIncompatible without
weakening Governor contact/force thresholds?

## Measured

| Arm | Result |
|---|---|
| Dirty-tree invent authorize | forbidden / not used |
| Clean-tree freeze | `replay_authorized=true`; 1 mm / 0.5 N unchanged |
| Simulator run | 1 (budget consumed) |
| Sidecar | sealed (gitignored locally; offline verify PASS) |
| Failure class | `expected_success_blocked_by_successive_prior_incompatibility` |

Harness outcome label `preserve_sealed_sidecar_or_typed_abort` accepted alongside
`preserve_sealed_sidecar`.

`NOT_ADMITTED`.

## Classification

**VALIDATED_LEARNING** — E2E authorize→replay→sealed diagnostic without Governor
weakening; successive prior incompatibility remains the research hole (not
threshold lowering).
