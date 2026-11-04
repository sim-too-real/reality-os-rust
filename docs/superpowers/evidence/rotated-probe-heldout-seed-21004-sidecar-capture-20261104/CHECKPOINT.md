# 21004 sidecar capture — 2026-11-04

## Result

**SIGNIFICANT ADVANCE** (simulation-only, NOT_ADMITTED). Fresh MuJoCo capture of
seed 21004 into a new evidence directory sealed a non-empty
`PROBE_MEASUREMENT_SIDECAR.json` (~12.5 MB, 1 measurement record) under the
fail-closed contact-class seal gate. Unauthorized writes: 0. Force/contact
gates unchanged (1 mm / 0.5 N). Sealed historical 21004 was not overwritten.

## Causal evidence vs sealed baseline 21004

| | Sealed 21004 | Sidecar capture |
|---|---|---|
| Sidecar | empty / absent after failure | **non-empty sealed** |
| Seal gate | n/a (pre-gate) | **PASS_SIDECAR_PRESENT** |
| Offline verifier | NOT_RUN_NO_SIDECAR | **FAIL** (`ambiguous runtime contact unsupported`) |
| Episode outcome | `action_failed` / force_monitor_abort | same |
| Failure class | `unexpected_contact_with_episode_diagnostic_only` | **`unexpected_contact_with_preserved_episode`** |

## Next bottleneck

Observation completeness for this seed is closed enough to preserve a probe
measurement sidecar. The independent offline interval verifier does not yet
support this ambiguous runtime contact pattern.

Do not invent measurements. Do not weaken Governor thresholds.
