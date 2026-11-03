# Retrospective pre-lift contact-capacity screen — 2026-11-03

**Classification:** `RETROSPECTIVE_PARTIAL_DISCRIMINATION; NOT_A_VALIDATED_PREDICTOR; SIMULATION_ONLY; NOT_ADMITTED`

## Question

Could measured post-close finger normal loads, combined with the lower frozen
simulated friction interval, reject the two primary controls before the
25 mm lift?

The screen proxy is `lower_mu × (finger_l normal + finger_r normal)`. It
rejects if that proxy is below object weight. It does not solve the full
contact friction cones, moments, actuator limits, or dynamic response.

## Results

| Trial | Proxy capacity | Object weight | Proxy decision | 1 mm candidate result |
|---|---:|---:|---|---|
| 21010:25 | 0.595 N | 1.419 N | Reject before lift (0.419× weight) | Runtime divergence; 0.135 N last recorded vertical contact force; object Δz −0.104 mm |
| 21006:21 | 2.628 N | 2.192 N | Pass to lift (1.199× weight) | Runtime divergence; 0.501 N last recorded vertical contact force; object Δz +0.026 mm |

The proxy agrees with the later failure on one control and misses the other.
That is 1/2 agreement on a two-control retrospective check; it is not a
predictor-performance estimate. The Cartesian candidate cohort remains
skipped under its original rule.

## Evidence limits

Post-close observations come from historical runs whose manifests mark the
source worktree dirty. Their exact trial digests, scenario mass, and scenario
friction match the current frozen primary controls, but their lift targets
differ (24.5 mm and 23 mm). Treat these observations as exploratory evidence.

The candidate execution result independently records both controls below the
object weight at the end of the lift attempt. It does not show why the
available friction was not mobilized on trial 21006. No contact geometry,
actuator-force realization, or dynamic stability cause is established.

Saved MuJoCo contact-friction tuples have five entries. MuJoCo documents the
first two as tangential coefficients and the third as torsional, so the third
entry must not be read as sliding friction. See [MuJoCo modeling
documentation](https://mujoco.readthedocs.io/en/latest/modeling.html) and the
[mjContact type reference](https://mujoco.readthedocs.io/en/latest/APIreference/APItypes.html).

The next justified software question is what per-contact frame, normal,
moment-arm, and actuator evidence a pre-lift diagnostic must retain to
distinguish geometric capacity from failure to realize that capacity. No
trajectory, force threshold, or authority gate changed in this analysis.

## Reproduction

Run `python verify_retrospective.py`. It checks all frozen evidence and source
hashes, verifies exact trial-digest matches, and regenerates `RESULTS.json`.

Admission remains `NOT_ADMITTED`; metal, shop release, and viewport admission
remain false. No physical action, purchase, fabrication, or external contact
occurred.
