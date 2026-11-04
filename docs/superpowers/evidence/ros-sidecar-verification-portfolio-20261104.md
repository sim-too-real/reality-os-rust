# ROS sidecar verification portfolio — 2026-11-04 (updated)

## Question

Can offline verification of sealed probe sidecars complete without inventing
measurements or weakening Governor gates, including after 21008 FREEZE replay?

## Measured

| Seed | Status | Notes |
|---|---|---|
| 21004 historical | `FAIL_NO_SIDECAR` | episode diagnostic `INTEGRITY_PASS` |
| 21005 historical | `FAIL_NO_SIDECAR` | episode diagnostic `INTEGRITY_PASS` |
| 21004 sidecar capture | `PASS_RECORDED_AT_SEAL` | `PASS_STRUCTURAL_AMBIGUOUS_CONTACT` |
| 21005 sidecar capture | `PASS_RECORDED_AT_SEAL` | `PASS_STRUCTURAL_AMBIGUOUS_CONTACT` |
| 21006 | `PASS_STRUCTURAL_NONEMPTY` | success path |
| 21007 | `PASS_STRUCTURAL_NONEMPTY` | success path |
| 21008 FREEZE replay | `PASS_RECORDED_AT_SEAL` | offline `PASS`; first_onset_outside_prior; miss-margin sealed |

Portfolio: 7 seeds; fail_no_sidecar=2; pass=5; governor_weakened_any=false;
invented_measurements_any=false.

21008 also carries `PRIOR_ENVELOPE_MISS_MARGIN.json` (relative miss > 5% vs
declared prior). Live Governor path unchanged; NarrowToMeasurement unwired.

## Classification

**SIGNIFICANT_ADVANCE** for capture transfer + 21008 authorize→replay; historical
empty-sidecar seeds remain explicit measured negatives.
