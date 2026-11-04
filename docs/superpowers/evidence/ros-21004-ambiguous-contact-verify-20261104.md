# ROS 21004 ambiguous-contact offline verify — 2026-11-04

## Question

Can the independent offline probe-interval verifier structurally accept a sealed
`runtime_contact_ambiguous` sidecar (force-trace digests +
`NoInference(unexpected_contact)`) without inventing friction-interval replay
or weakening Governor gates?

## Measured

| Metric | Value |
|---|---|
| Offline verifier | `PASS_STRUCTURAL_AMBIGUOUS_CONTACT` |
| Interval replay supported | false (by design) |
| Inference | `NoInference(UnexpectedContact)` |
| Pose/trace digests | force-aware `/3` sample + `/4` trace match sealed hashes |
| Unauthorized writes | 0 |
| Governor weakened | false |

Evidence dir:
`docs/superpowers/evidence/rotated-probe-heldout-seed-21004-sidecar-capture-20261104/`

## Classification

**SIGNIFICANT_ADVANCE** — closes the offline-verifier ambiguous-contact hole for
sealed 21004-class captures. Friction-interval math remains refused while contact
identity is ambiguous.
