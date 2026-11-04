# ROS sidecar verification portfolio — 2026-11-04 (updated)

## Question

Can offline verification of sealed probe sidecars complete without inventing
measurements or weakening Governor gates, including after fresh 21004/21005
captures?

## Measured

| Seed | Status | Notes |
|---|---|---|
| 21004 historical | `FAIL_NO_SIDECAR` | episode diagnostic `INTEGRITY_PASS` |
| 21005 historical | `FAIL_NO_SIDECAR` | episode diagnostic `INTEGRITY_PASS` |
| 21004 sidecar capture | `PASS_RECORDED_AT_SEAL` | `PASS_STRUCTURAL_AMBIGUOUS_CONTACT` |
| 21005 sidecar capture | `PASS_RECORDED_AT_SEAL` | `PASS_STRUCTURAL_AMBIGUOUS_CONTACT` |
| 21006 | `PASS_STRUCTURAL_NONEMPTY` | success path |
| 21007 | `PASS_STRUCTURAL_NONEMPTY` | success path |

Portfolio: 6 seeds; fail_no_sidecar=2; pass=4; governor_weakened_any=false;
invented_measurements_any=false.

## Classification

**SIGNIFICANT_ADVANCE** for capture transfer; historical empty-sidecar seeds
remain explicit measured negatives.
