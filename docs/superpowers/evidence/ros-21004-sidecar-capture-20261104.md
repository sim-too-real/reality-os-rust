# ROS 21004 fresh sidecar capture — 2026-11-04

## Question

Under the fail-closed contact-class seal gate, can a fresh authorized MuJoCo
capture of held-out seed 21004 seal a non-empty probe-measurement sidecar
without inventing measurements or weakening Governor gates, and how does that
change the failure class vs sealed EMPTY_SIDECAR 21004?

## Measured

| Metric | Value |
|---|---|
| Seal gate | `PASS_SIDECAR_PRESENT` (`seal_allowed=true`) |
| Sidecar bytes | 12,486,099 |
| Measurement records | 1 |
| Episode | `action_failed` / `force_monitor_abort` / `unexpected_contact` |
| Unauthorized writes | 0 |
| Offline verifier | `PASS_STRUCTURAL_AMBIGUOUS_CONTACT` (interval replay unsupported) |
| Failure class | `unexpected_contact_with_preserved_episode` |
| Governor weakened | false |

## Classification

**SIGNIFICANT_ADVANCE** vs sealed 21004 empty-sidecar failure mode.
Follow-on: `ros-21004-ambiguous-contact-verify-20261104.md` closed the offline
structural hole without inventing friction intervals.

Evidence dir:
`docs/superpowers/evidence/rotated-probe-heldout-seed-21004-sidecar-capture-20261104/`
