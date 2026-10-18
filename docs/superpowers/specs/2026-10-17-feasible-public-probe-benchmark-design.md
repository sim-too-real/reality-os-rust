# Feasible Public Probe Benchmark Design

**Date:** 2026-10-17  
**Status:** Approved objective; implementation in progress  
**Scope:** Replace the public push/probe scenario distribution after the schema-v6 pilot exposed unreachable positive trials and probes that could not contact the object.

## Evidence and problem

The frozen v6 force-aware pilot at `docs/superpowers/evidence/force-aware-probe-7d8ee51/` passed its accounting and safety gates, but did not demonstrate probe availability. Its 11 expected-success UR5e push trials had zero task successes, zero information-bearing probes, and zero justified follow-ups. IIWA-14 had zero expected-success trials because all sampled positive/ambiguous scenarios were relabeled as refusals after standoff IK failed. Seven of nine inference attempts had no pre-contact baseline.

The raw UR5e episode for seed `9000`, index `10` places the tool approximately 50 mm behind the intended side-contact pose while authority limits the probe stroke to 8 mm. The tool therefore cannot reach contact during that probe. The force-aware estimator and safety monitor correctly refuse to claim friction information without contact and a valid baseline. This benchmark revision must address scenario feasibility and action geometry before another public comparison.

A full-geometry audit found the original shared x-range placed positive targets deep over the IIWA-14 table footprint, where its arm could not reach the authorized side contact without support collision. Suite-v2 now randomizes targets along the near table edge and keeps obstacle centers and extents within the support footprint. The IIWA spatial fixture is classified from world-space joint axes evaluated at the inspected reference pose; raw MJCF local axes are insufficient for this distinction.

Follow-up full-geometry witnesses showed the first shifted fixture still placed IIWA wrist/link geometry into the table at its left edge (`SUPPORT_COLLISION` on `wrist_1_link` and `link6`). The shared spatial fixture now uses a table centered at x=0.90 m with 0.40 m half-width and positive target centers randomized from x=0.56–0.63 m, keeping the target footprint supported. Direct joint interpolation can still intersect that edge; the reasoner rejects those paths and uses a raised, collision-checked transfer route with certified waypoints. No collision or contact rule was relaxed.

The first full IIWA execution of suite-v2 also showed that a 20 mm half-height block is too short for the inspected 60 mm wrist collision sphere to push near its vertical center. The conservative friction estimator then observed an invalid normal-load interval and correctly refused. Positive suite-v2 objects now randomize half-height from 70–90 mm, placing the authorized side probe near the support-load centerline while preserving the robot's complete collision geometry and all force limits. A geometry audit also found the contact-radius proxy omitted each collision shape's local offset; it now bounds geometry relative to the declared end-effector frame and refuses unknown or unsupported collision geometry. With that correction, the strict MuJoCo preflight resolver admits the positive suite-v2 trial for both UR5e and IIWA-14 without relaxing contact or collision policy.

## Goals

- Define a new deterministic, machine-readable public scenario suite with expected-success and expected-refusal strata for both development embodiments.
- Randomize target pose, size, mass, friction, obstacle layout, and initial joint configuration within predeclared ranges while screening positive trials for complete-geometry reachability and a clear initial configuration.
- Ensure the first authorized 8 mm probe can physically reach the target from its resolved initial pose.
- Preserve separate, explicit expected-refusal cases and count all scheduled trials; never convert an unresolved expected-success case into a refusal after seeing execution results.
- Establish a new baseline on new public seeds and preserve all v5/v6 evidence unchanged. Do not run or inspect the WX250S holdout.

## Non-goals

- No force, collision, authority, lateral-drift, geometry, or refusal threshold is relaxed.
- No robot-name branch is added to the reasoner, planner, authority bridge, executor, or scorer. Embodiment manifests may declare physical resource limits; the task and outcome rules remain shared.
- No simulator evidence is labeled physical or hardware-derived. The external physical-transfer gate remains `UNVERIFIED`.
- This work does not claim that the current candidate is improved or competitive. The first new public run is a baseline for the revised input distribution.

## Design

### Suite and strata

Add scenario-suite version 2 as a checked-in manifest. It declares 100 public trials per registered development task, 20 pilot trials, the frozen seed ranges, all randomization ranges, the push contact gap, and an interleaved 3:2 expected-success/expected-refusal schedule. Thus a complete 20-trial pilot schedules 12 expected-success and 8 expected-refusal push cases per development embodiment; a 100-trial public run schedules 60 and 40. Stratum selection is derived only from the scheduled index and is independent of execution outcome.

Positive trials randomize a movable target's pose and plan dimensions, including a 70–90 mm half-height, a bounded mass/friction range, a short push goal, and zero or more obstacles whose sampled geometry is outside the declared approach and goal corridors. The shared spatial fixture keeps the object near the table edge facing the arm and declares obstacle forward/lateral offsets that remain on the tabletop; both pose ranges remain randomized and shared across embodiments. The height range keeps the contact band near the block's vertical center for the inspected tool cross-sections, so the measured push can retain a positive support-normal load. The resolver uses the same complete robot geometry and deterministic IK/standoff checks already used for the benchmark. It may deterministically resample scenario parameters within the manifest's fixed attempt limit. If it cannot resolve a positive trial that passes those pre-execution checks, it records an invalid trial and fails the run; it must not relabel the trial as an expected refusal.

Refusal trials use explicit, typed unsupported or unsafe conditions from the existing scenario taxonomy. Their labels and fault inputs are resolved before execution and remain hidden from the reasoner except for ordinary evidence/fault observations already part of the task contract.

### Contact and authority geometry

The shared reasoner currently places the push approach 50 mm behind contact. Reduce the push approach gap to 4 mm, leaving at least 4 mm of the existing 8 mm probe stroke after first contact. Use the same named constant in scenario initialization and reasoner standoff construction. Preserve the 8 mm action stroke and every existing force/torque cap. The resolved-trial feasibility check must assert that the initial tool pose is outside contact, inside the declared 4 mm gap tolerance, and that the probe stroke reaches through the contact plane.

### Versioning and evidence

Bump benchmark trial schema to 7 and add the suite identifier/version and resolved stratum to each trial and the suite digest to the report configuration. The schema-independent paired key must bind the suite identifier/version and all resolved randomized inputs. Keep schema-v5/v6 readers and evidence immutable. New pilot/public seed ranges must not overlap prior v5/v6 pilot, public, or historical holdout ranges.

Run the strict MuJoCo-required 20-trial-per-arm pilot first. It must complete with zero infrastructure, invalid, or unaccounted trials; zero false acceptances, unauthorized writes, geometry omissions, stale-authority continuations, and unverified wrench stops; and nonzero expected-success denominators for both arms. The pilot is a screening run and cannot be called the baseline. After candidate freeze and pilot review, run and freeze the new 100-trial-per-arm public baseline (plus the existing registered grasp-fixture trials) under a new evidence directory. Do not run the holdout.

## Acceptance criteria

1. The manifest loads only when suite version, seed ranges, strata, randomization bounds, and contact-gap/stroke relationship are valid.
2. Repeated `(suite, seed, index, robot model)` resolution is byte-identical and produces the same paired key.
3. Both development embodiments schedule the declared 12/8 pilot split and 60/40 public split; no positive scenario is silently downgraded to refusal.
4. Every positive resolved scenario passes complete-geometry initial-configuration and standoff reachability checks; its target height is sampled within the declared range; its 8 mm probe traverses the contact plane with at least 4 mm of post-contact stroke; the end-effector contact-radius bound includes declared frame and geometry offsets; and the measured friction interval has a positive conservative normal-load bound.
5. Existing v5/v6 records remain readable and unchanged; paired keys differ when suite ID/version or randomized inputs differ.
6. Strict pilot and public baseline campaigns require MuJoCo and report every scheduled outcome, safety invariant, friction inference, abort, runtime, and failure cause.
7. No WX250S holdout is run or inspected, and simulation evidence remains `SIMULATION_ONLY`.
