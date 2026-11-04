# Episode-diagnostic failure-class refinement — 2026-11-04

## Question

Do sealed 21004/21005 transit aborts that already have
`*.episode-diagnostic.json` but no verified probe-measurement sidecar deserve a
finer failure class than `transit_unexpected_contact_without_measurement_trace`?

## Change

Classifier + portfolio now detect episode diagnostics under the evidence dir.

| Condition | Class |
|---|---|
| unexpected_contact, no sidecar, **no** episode diagnostic | `transit_unexpected_contact_without_measurement_trace` |
| unexpected_contact, no sidecar, **with** episode diagnostic | `unexpected_contact_with_episode_diagnostic_only` |
| unexpected_contact, sidecar PASS | `unexpected_contact_with_preserved_episode` (unchanged) |

`governor_weakened=false`. Contact/force thresholds unchanged.

## Measured

Portfolio reclassify: 21004 and 21005 → `unexpected_contact_with_episode_diagnostic_only`.
Sidecar verification portfolio (same day): 21004/21005 → measured `FAIL_NO_SIDECAR`
with episode-diagnostic `INTEGRITY_PASS`; do not invent measurements or weaken gates.
Next: seal non-empty probe-measurement sidecar on an authorized fresh capture, or
owner-gated 21008 replay.
21008 replay still blocked on owner commit.

## Classification

**SIGNIFICANT_ADVANCE** for diagnostics/research prioritization (not task-success rate).

## Evidence

```powershell
py -3.10 -m unittest crates.verify.tests.test_probe_transit_abort_failure_class crates.verify.tests.test_probe_failure_class_portfolio -v
```
