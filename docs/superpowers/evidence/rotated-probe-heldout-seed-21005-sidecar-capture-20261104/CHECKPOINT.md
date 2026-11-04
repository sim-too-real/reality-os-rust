# 21005 sidecar capture — 2026-11-04

## Result

**SIGNIFICANT ADVANCE** (simulation-only, NOT_ADMITTED). Fresh MuJoCo capture of
seed 21005 into a new evidence directory sealed a non-empty
`PROBE_MEASUREMENT_SIDECAR.json` (~12.9 MB, 1 measurement record) under the
fail-closed contact-class seal gate. Unauthorized writes: 0. Force/contact
gates unchanged. Sealed historical 21005 was not overwritten.

## Causal evidence vs sealed baseline 21005

| | Sealed 21005 | Sidecar capture |
|---|---|---|
| Sidecar | empty / absent after failure | **non-empty sealed** |
| Seal gate | n/a (pre-gate) | **PASS_SIDECAR_PRESENT** |
| Offline verifier | NOT_RUN_NO_SIDECAR | **PASS_STRUCTURAL_AMBIGUOUS_CONTACT** |
| Episode outcome | `action_failed` / force_monitor_abort | same |
| Failure class | `unexpected_contact_with_episode_diagnostic_only` | **`unexpected_contact_with_preserved_episode`** |

## Transfer claim

Same capture+seal+structural-verify path that closed 21004's empty-sidecar hole
also closes 21005 without inventing measurements or weakening Governor gates.

Do not invent measurements. Do not weaken Governor thresholds.
