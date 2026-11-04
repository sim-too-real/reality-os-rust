# Seed 21007 — inferred vs simulator friction (sim-only)

| Seed | Preflight / sim μ (Coulomb param) | Inferred interval | vs μ_sim |
|---|---:|---|---|
| 21006 | 0.6316 | [0.335, 0.8] | overlap (source-identity caveat) |
| 21007 | 0.7517 | [0.406, 0.492] | **disjoint** |

## What is sealed

- Freeze commit before replay: `9e90acc`
- Sidecar offline verify: PASS
- Frozen source hashes match current sources at RESULT time
- Contact-force samples present (6451 poses / 27266 forces)
- Unauthorized writes: 0

## Offline onset comparison (no new sim)

Artifact: `ONSET_WRENCH_VS_CONTACT_FORCE.json`

At `first_motion_s ≈ 25.788`:

| Quantity | Value |
|---|---|
| Runtime inferred interval | [0.406, 0.492] |
| obj0/table contact T/N (before) | min/mean/max ≈ 0.437 / 0.490 / 0.544 |
| obj0/table contact T/N (after) | min/mean/max ≈ 0.437 / 0.491 / 0.545 |
| Simulator Coulomb μ (preflight / analysis) | ≈ 0.752 |

Contact T/N at the detected onset **overlaps** the wrench-derived posterior. Both sit **below** the Coulomb parameter.

## Named hypothesis (software-only; not yet patched)

1. Slip-onset inference constructs bounds from **tool wrench** force/normal envelopes (`push_probe.rs`), not from sealed obj/table `contact_force_samples`.
2. For seed 21007, those wrench-derived bounds agree with instantaneous contact T/N at the motion threshold (~0.49).
3. The motion threshold fires while T/N < μ_coulomb, so intersecting the estimator with μ_sim as a “transfer success” metric is the wrong claim: the estimator tracks **onset force ratio**, not the Coulomb parameter, under this detection rule.
4. Seed 21006’s wider interval overlapped μ_sim by prior coverage, not by proving Coulomb identification.

## What this does **not** prove

- No physical transfer.
- No calibrated Coulomb identifier.
- No admission (`NOT_ADMITTED`).

## Offline evaluator (landed)

`crates/verify/probe_contact_force_analysis.py` now emits
`claim_separation` under `inference_vs_simulator_friction`:

- `onset_force_ratio_vs_contact_tn`: **overlap** on 21007
- `coulomb_parameter_in_inferred_interval`: **no** on 21007

Unit coverage: `crates/verify/tests/test_probe_contact_force_claim_separation.py`.
Refreshed analysis: `OFFLINE_ANALYSIS.json`. Keep seed 21007 sealed.

## Runtime quantity label (landed after seal)

`FrictionInferenceResult::Updated` now carries
`inferred_quantity = onset_or_sticking_force_ratio` (serde-defaulted for old
records). Episode reasons include `quantity=OnsetOrStickingForceRatio`. This
does **not** rewrite sealed 21007 artifacts; it prevents future runs from
treating the wrench-derived interval as identified Coulomb μ.
