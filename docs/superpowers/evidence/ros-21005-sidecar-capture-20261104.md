# ROS 21005 fresh sidecar capture — 2026-11-04

## Question

Does the fail-closed contact-class seal gate + structural ambiguous-contact
offline verifier transfer from held-out 21004 to held-out 21005 without
inventing measurements or weakening Governor gates?

## Measured

| Metric | Value |
|---|---|
| Seal gate | `PASS_SIDECAR_PRESENT` (`seal_allowed=true`) |
| Sidecar bytes | 12,885,817 |
| Measurement records | 1 |
| Episode | `action_failed` / `force_monitor_abort` / `unexpected_contact` |
| Unauthorized writes | 0 |
| Offline verifier | `PASS_STRUCTURAL_AMBIGUOUS_CONTACT` (interval replay unsupported) |
| Failure class | `unexpected_contact_with_preserved_episode` |
| Governor weakened | false |

Evidence dir:
`docs/superpowers/evidence/rotated-probe-heldout-seed-21005-sidecar-capture-20261104/`

## Classification

**SIGNIFICANT_ADVANCE** — transfer of 21004 sidecar-capture + structural-verify
discovery to a second held-out unexpected-contact seed.
