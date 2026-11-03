# Tool–Object Friction Mapping v1

## Intent

Make the simulated `ToolObjectFriction` value control sliding friction at the target object–finger contacts. This restores correspondence between the friction value supplied to the reasoner and the contact model used by MuJoCo.

## Evidence and current behavior

The worker writes a scenario scalar as `[μ, μ, 0.01]` to a MuJoCo geom. Geom friction has three entries: sliding, torsional, and rolling. The second `μ` therefore controls torsional friction. In the grasp fixture, finger geoms have default sliding friction 1.0; MuJoCo combines equal-priority geom friction by taking the elementwise maximum. The resulting 3D contact has sliding coefficients `[1, 1]` and places the scenario value in the unused torsional entry. A minimal MuJoCo reproduction with scenario μ=0.4 produced exactly that result.

## Goals

- Apply the scenario coefficient to both tangential directions of explicit target-object–finger contact pairs.
- Keep the coefficient limited to the target object and the gripper finger bodies used by the grasp reasoner.
- Map scalar geom friction to `[μ, 0.01, 0.01]` for sliding, torsional, and rolling entries.
- Preserve unrelated contact-pair friction and the existing contact dimensionality.
- Fail closed if a requested target body or finger body cannot be resolved.

## Non-goals

- No grasp-control, lift-trajectory, threshold, Governor, admission, or release-authority changes.
- No physical friction claim or hardware inference.
- No change to support-surface friction; the table/support model remains a separate limitation.
- No full-lift replay until the offline predictor is independently evaluated.

## Approaches

1. **Correct only the geom vector.** This fixes the torsional-slot error, but finger geoms with sliding friction 1.0 still mask lower scenario values under MuJoCo's max mixing rule.
2. **Raise object priority or lower finger friction globally.** This would change object–table or finger–other-body contacts that are outside the target pair and could alter unrelated behavior.
3. **Add explicit target object–finger pair friction.** This applies μ only to the contacts represented as `ToolObjectFriction`, leaves other geom mixing intact, and preserves their existing contact dimensions. This is selected.

## Constraints and implementation calls

- The scenario coefficient remains simulator ground truth, not a sensor measurement.
- The executor passes the target object's finger-body names from the same qualified gripper resource already used by the grasp reasoner.
- MuJoCo pair friction receives `[μ, μ, 0.01, 0.01, 0.01]`; an existing pair keeps its `condim`, and a new pair uses the geom-derived contact dimension.
- Scenarios without a target friction value do not receive an override.

## Acceptance

1. A real MuJoCo worker-load regression with μ=0.4 and a default-friction finger reports target pair `condim=3` with tangential friction `[0.4, 0.4]` and torsional friction `0.01`; a second check preserves a nondefault `condim=4`.
2. An unrelated object contact keeps its own effective friction.
3. Rust preparation attaches the override only to the target object and the selected gripper's finger bodies.
4. Runtime object-friction updates retain sliding/torsional/rolling order and update only the explicit target–finger pairs.
5. Focused Python and Rust tests pass; no grasp/lift replay is run.
6. The actual two-finger `arm_gripper_lift` bundle compiles two uniquely named target–finger pairs, each with the requested friction vector; pair-name allocation remains finite when the spec does not enumerate pairs added earlier in the same load.

## Archived evidence limit

The two frozen lift traces predate this correction. Their recorded contact vectors were `(1.0, 1.0, μ, 0.01, 0.01)`, so they do not validate outcomes under the corrected scenario sliding friction. Keep them immutable and exclude them from corrected-μ predictor validation.
