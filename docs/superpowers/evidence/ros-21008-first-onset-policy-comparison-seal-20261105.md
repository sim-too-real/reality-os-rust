# ROS 21008 first-onset policy comparison seal — 2026-11-05

## Question

Can sealed 21008 numbers carry a machine-readable offline comparison of
Retain / PreferHigherLoad / Refuse / Narrow first-onset policies, including
whether each proves μ≥0.3, without wiring live belief or weakening Governor?

## Measured

| Policy | Proves μ≥0.3? | Live? |
|---|---|---|
| RetainDeclaredPrior | no | **yes (status quo)** |
| PreferHigherLoadFamily | no | no (N/A) |
| RefusePriorEnvelope | no | no |
| NarrowToMeasurement | **yes** | **no** |

`FIRST_ONSET_POLICY_COMPARISON.json` sealed beside miss-margin and bound
diagnosis. Governor thresholds 1 mm / 0.5 N unchanged.

## Classification

**VALIDATED_LEARNING** — offline policy space sealed for transfer/audit; live
path retains declared prior.

## Evidence

```powershell
py -3.10 -B -m unittest crates.verify.tests.test_first_onset_policy_comparison_21008
cargo test --offline -p realityos-semantics --test push_probe_inference first_onset_outside_prior
```
