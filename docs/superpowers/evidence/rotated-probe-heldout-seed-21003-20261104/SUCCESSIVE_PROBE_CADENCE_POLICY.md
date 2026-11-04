# 21003 successive-probe cadence policy comparison — 2026-11-04

## Inputs (sealed offline numbers; thresholds unchanged)

| Quantity | Value |
|---|---|
| First SlipOnset posterior | `[0.2590453156056787, 0.33705505290410587]` |
| Second measurement bounds | `[0.4173207930536931, 0.491342106128889]` |
| Prior before first onset | `[0.05, 0.8]` |
| `mu_required` | `0.3` |
| Force / contact gates | 1 mm / 0.5 N (unchanged) |

## Competing policies

| Policy | Outcome on 21003 numbers | Push-proving (`lo ≥ 0.3`)? |
|---|---|---|
| `StrictIntersect` | `Rejected(PriorMeasurementIncompatible)` | no (belief stuck at first) |
| `RetainFirstOnPriorIncompatible` | retain first; quarantine second | **no** (first straddles 0.3) |
| `PreferHigherLoadFamilyOnIncompatible` | escalate to second∩original prior; quarantine first | **yes** (`[0.417…, 0.491…]`) |

## Decision

Wire `PreferHigherLoadFamilyOnIncompatible` into `record_probe_measurement`.

Retain-first was evaluated and **does not** change the E2E justified-action outcome for this sealed case: after blocking `probe:+x`, push remains `MECHANICS_UNKNOWN`.

Higher-load-family escalation is the justified corrective hypothesis: later onset entirely above the first posterior is treated as a different load family, first onset is quarantined, belief replaces with later∩original_prior — without lowering force/contact/admission thresholds.

## Verification

- Unit: `successive_probe_cadence_policies_on_21003_numbers` (PASS).
- Fresh paired MuJoCo replay:
  `rotated-probe-heldout-seed-21003-cadence-escalation-20261104/` →
  `outcome=success`, escalate audit, `push:+x`, unauthorized_writes=0 (PASS).

## Classification

**SIGNIFICANT ADVANCE** (simulation-only) on this seed geometry: sealed baseline
`unexpected_refusal` → treatment `success` via higher-load-family escalation,
without weakening force/contact/Governor thresholds.

Transfer to an independent held-out seed remains open.
