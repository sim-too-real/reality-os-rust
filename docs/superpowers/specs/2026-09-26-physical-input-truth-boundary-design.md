# Physical-input truth-boundary design

**Date:** 2026-09-26
**Status:** proposed; initial slice approved, implementation awaits spec review
**Baseline:** `main` @ `9cd656fc835a3284ca264c3e1f10edb6fda0ce1d`

## Goal and invariant

Close the narrowest set of physical-input boundaries where invalid or absent evidence currently becomes a plausible physical fact. Keep the existing useful planner and verifier structure, but make the affected paths preserve uncertainty or return a typed refusal.

The acceptance invariant is: removing physical evidence from an otherwise identical scenario must never strengthen its physical claim. `UNKNOWN`, `INVALID`, and `MISSING` must remain unknown, produce an explicit typed refusal, or be used only by a clearly non-authoritative search heuristic. They must not become plausible geometry or mechanics and then yield `FEASIBLE`.

This is a truth-boundary slice, not the production physical reasoner, a selector rewrite, or an authority change.

## Chosen approach

Validate inputs at the reusable semantics boundary and propagate the failure to the existing refusal/unknown vocabulary. Keep valid explicit values unchanged. Do not repair bad physical inputs with identity transforms, world-up normals, zero residuals, or a fixture parameter. Do not replace these with new implicit “safe defaults.”

For collision-scene construction, use a typed `Result` so invalid world geometry is distinguishable from a valid scene with no collision. Preserve the existing `apply_collision_admissibility` call shape if practical: when scene construction fails, refuse each still-feasible phase with one stable invalid-scene reason, retaining any prior refusal. For contact manifold generation and contact feasibility, propagate a typed invalid-support/object-pose reason instead of returning positive candidates or a plausible transform. This is conservative refusal, not a claim that collision was observed.

## In-scope changes

1. **Support-plane validity in reusable push/contact paths**

   Remove the `+Z` recovery of an invalid support normal from `contact_manifold.rs` (including the tangent-basis, vertical-face, and signed-height paths), `contact_maneuver.rs` clearance checks, and collision support-plane checks. Normalize with a finite, nonzero result check so norm overflow cannot turn a direction into `[0, 0, 0]`; reject zero or non-finite normals and non-finite support origins. A rejected plane must not generate push candidates or pass a contact/support-clearance test.

   Add specific invalid-support and invalid-object-pose results to the existing manifold/contact refusal vocabulary. The posed manifold APIs must validate their `Se3` input as well as support data. `physical_interaction` must propagate invalid plane/pose input as an error/outcome distinguishable from an ordinary empty candidate set. `evaluate_sampled_push` must return an invalid-input refusal before making a clearance claim.

2. **Malformed object and obstacle poses**

   In `contact_collision.rs`, remove object/obstacle pose fallbacks to translation or identity and the invalid-normal fallback used to place the support slab. Validate positions and quaternions through `Se3::try_new` before materializing a planner-visible scene. Ensure `Se3::try_new` cannot return a pose whose normalized quaternion is zero/non-finite after norm overflow (use scale-safe normalization or reject the result). Infallible iterator construction must become fallible collection so any malformed obstacle invalidates the scene rather than disappearing or being repaired. Invalid support data follows the same typed-scene failure path.

   `apply_collision_admissibility` must fail closed on invalid scene input: no transition remains `Feasible` solely because collision geometry was fabricated from a malformed pose. Valid finite nonzero quaternions may continue to be normalized by `Se3::try_new` when the normalized result is valid.

   In `contact_maneuver.rs`, make `BoxObject::pose` and its callers preserve `Se3::try_new` failure instead of substituting translation/identity. Contact evaluation must stop with a typed invalid-pose result. This applies only to the physical object pose entering the push/contact proof, not a general transform or kinematics redesign.

3. **Missing discrepancy discriminators**

   In `discrepancy::hypothesize`, stop interpreting absent `tracking_error_m` and `geometry_residual_m` as measured zero. Keep positive hypotheses supported by present evidence, but add an insufficient/unknown discriminator when a required alternative explanation was not measured. A high displacement ratio alone, when tracking or contact-geometry evidence is absent, must not identify `SupportFrictionInconsistent` or report `Identified` friction.

   Friction inconsistency may be emitted only when the ratio is high and the relevant alternative discriminators are explicitly present and low; otherwise the report remains `Unknown` or `Underdetermined` with missing evidence represented in its typed status/kinds. Existing freshness, reachability, contradiction, and independent positive-evidence gates remain intact.

4. **Explicit test-fixture physics; no unknown-friction substitution**

   The closed-loop physical scenario in `verify/src/goal_directed.rs` is test-only. Consolidate its literal simulation mass/friction inputs into one named fixture/scenario-physics value and pass that value explicitly to the simulator setup and any predictor input that is intentionally allowed to know it. Keep simulator ground truth separate from what the reasoning path is told: if friction is not exposed as declared/observed input or supported by an inference, mechanics remain unknown and selection refuses for unknown mechanics. Remove `.unwrap_or(0.3)` and equivalent fresh-belief substitution; diagnostic text must say unknown rather than print a made-up declared value.

   This preserves explicit fixture parameters for controlled tests; it does not promote them to production defaults or leak hidden simulator truth into a physical claim.

## Compatibility and boundaries

- Public Rust APIs that currently cannot express invalid support or scene construction may gain `Result`/typed error returns. Update their in-repository call sites and tests in the same implementation slice; do not preserve a misleading infallible wrapper that silently discards the error.
- Keep established valid-input behavior and valid explicit `+Z` support fixtures unchanged.
- Keep `contact_collision` refusal distinct from authority authorization. No command minting, online write, governor, metal, or hardware path changes.
- Do not sweep the repository for every numeric default. Missing joint `q`/origin kinematics are deferred unless a direct dependency blocks these tests; IK search bounds such as `[-2, 2]` remain acceptable only as explicitly non-evidentiary search heuristics.
- Do not extract the test-only closed loop into production, change selector/experience logic, alter GRASP/HOLD/RELEASE, or claim robot generalization in this slice.

## Tests and acceptance evidence

Add focused regression tests alongside the affected modules:

- zero, NaN, and infinite support normals (plus a non-finite origin) return a typed invalid-input result; they do not produce vertical-face push candidates, a contact manifold, positive support clearance, or a collision-free claim. Include a normalization-overflow case so it cannot silently become a zero vector;
- malformed object poses (zero/non-finite quaternion and non-finite translation) and a malformed obstacle pose return a typed scene/contact refusal; collision admissibility refuses every otherwise-feasible phase and never substitutes translation/identity;
- valid normalized and non-unit finite quaternions, and valid declared support normals, retain their current geometric results;
- high displacement with tracking or geometry removed never becomes an identified friction explanation. Use a deterministic property-style test across subsets of the required discriminator fields: deleting any required measurement cannot add a friction-identification claim or increase identifiability;
- fixture tests demonstrate that a declared fixture value is passed explicitly, while the same scenario with friction hidden and no inference returns mechanics unknown/refusal rather than consuming fixture truth as a fallback.

Acceptance requires focused crate tests and the repository's applicable verification checks to pass, plus a diff review confirming only this P0 surface changed. Do not run frozen holdouts as part of the implementation slice; after it lands, the next milestone is the already-approved decision-consistency work, then production-reasoner extraction, followed by GRASP/HOLD/RELEASE proof.

## Risks and containment

- Fallible API changes can touch several internal callers. Keep propagation local to contact/manifold/collision evaluation and update callers in one compile-checked change.
- Conservative refusal reduces apparent planner coverage on malformed input. That is intended: invalid evidence must not certify a physical action.
- Discrepancy reports may become less specific when observations are incomplete. Preserve every independently supported explanation and report exactly which discriminator is missing; do not turn incomplete evidence into a false contradiction.
- Test outcomes may change where closed-loop tests previously relied on friction truth leaking from the simulator. Make visibility explicit per fixture and assert the unknown case rather than weakening the refusal.
