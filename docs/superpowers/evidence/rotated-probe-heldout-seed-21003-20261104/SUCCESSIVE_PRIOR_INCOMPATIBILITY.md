# 21003 successive prior incompatibility — 2026-11-04

## Root cause (sealed offline diagnosis)

First probe updates prior to ≈`[0.259, 0.337]` (SlipOnset).  
Second probe measurement bounds ≈`[0.417, 0.491]` sit **entirely above** that prior → empty intersection.

Offline label: `measurement_bounds_do_not_overlap_prior` in `INFERENCE_BOUND_DIAGNOSIS.json`.

## Software change (no Governor weaken)

1. Semantics: split `EmptyIntersection` (measurement lo>hi) from
   `PriorMeasurementIncompatible` (non-empty bounds outside prior).
2. Offline portfolio class for sealed 21003:
   `expected_success_blocked_by_successive_prior_incompatibility`.

## What success looks like next (authorized)

Offline cadence comparison is recorded in `SUCCESSIVE_PROBE_CADENCE_POLICY.md`.

- Retain-first: validated as insufficient for push-proving on these numbers.
- Higher-load-family escalation: wired into `record_probe_measurement`.

**Next authorized experiment:** fresh paired MuJoCo replay of sealed 21003 (or a new freeze) under the wired policy; measure task completion / refusal classes without changing 1 mm / 0.5 N gates.
