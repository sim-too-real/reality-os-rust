# ROS 21008 prior-envelope miss margin — 2026-11-05

## Question

Can sealed first-onset-above-prior geometry report a quantitative prior-envelope
miss margin offline, without inventing measurements or weakening Governor
thresholds?

## Model

When `measurement.lower > prior.upper`:

`absolute_miss = measurement.lower − prior.upper`

`relative_miss = absolute_miss / prior.upper`

## Measured

| Arm | Result |
|---|---|
| Sealed 21008 numbers | `EntirelyAbovePrior` with relative miss > 5% |
| Held-out numeric family | same kind |
| Intersecting bounds | `NotEntirelyAbovePrior` |
| Live Governor path | **unchanged** |

`NOT_ADMITTED`; metal/shop_release/viewport_admits=false.

## Classification

**VALIDATED_LEARNING** — prior-envelope honesty metric; does not select live belief.

## Evidence

```powershell
cargo test --offline -p realityos-semantics --test push_probe_inference prior_envelope_miss_margin
py -3.10 -B -m unittest crates.verify.tests.test_prior_envelope_miss_margin_21008
```

Sealed seed artifact:
`docs/superpowers/evidence/rotated-probe-heldout-seed-21008-20261104/PRIOR_ENVELOPE_MISS_MARGIN.json`.
