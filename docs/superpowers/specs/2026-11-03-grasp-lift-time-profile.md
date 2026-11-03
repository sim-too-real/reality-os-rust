# Grasp-lift time-profile experiment

## Intent

Test whether a bounded trapezoidal time profile reduces grasp slip or lift
force excursions on the current pinch-grasp task. The current controller
linearly changes joint targets along one collision-checked path. The frozen
25 mm target improved four historical cases but left the fresh cohort at
3/7; target and dwell follow-ups did not improve that cohort.

## Approaches

- More target or dwell tuning is rejected because the saved paired runs show
  no fresh-cohort gain and the longer dwell worsened one case.
- Cartesian lift sub-waypoints could alter the checked geometric path and add
  execution time, making it harder to isolate the trajectory effect.
- **Selected:** keep the existing joint-space path and 25 mm endpoint, and
  replace uniform progress with a bounded trapezoidal time profile. This
  isolates setpoint timing while keeping the collision-checked path image and
  simulation step budget unchanged.

## Contract

Use a 25% acceleration ramp and a symmetric 25% deceleration ramp. The
normalized position is `t² / (2 r (1-r))` for `t < r`, is linear at constant
normalized velocity for `r <= t <= 1-r`, and is `1 - (1-t)² / (2 r (1-r))`
for `t > 1-r`, where `r = 0.25`. This gives a continuous velocity profile
with peak speed `4/3` of the uniform profile. Start and end poses, collision
proof, 50 simulator steps per interpolation waypoint, 25 mm maximum lift,
1.95 N per-finger force ceiling, verifier thresholds, and authority checks
remain fixed.

The witness and lift-trajectory digest must identify the profile. The
executor accepts only the supported profile identifier and applies the same
profile the reasoner bound. The independent hold verifier still requires
bilateral contact, a 20–25 mm end-effector lift, object-follow residual at
most 5 mm, and per-finger force at most 1.95 N.

## Evaluation and limits

Freeze the source identity, the two known failing grasp/lift cases, and seven
expected-success inputs absent from the local evidence archive before changing
production code. Select the first seven resolved `ExpectedSuccess` inputs in
ascending seed order from seeds 22000–22100 and fixed positive scenario
indices `[15, 16, 20, 21, 25, 26, 30, 31]`; selection reads labels only and
does not run simulation. Run the new cohort once with the uniform profile, bind that
baseline to the same exact input digests, and commit it before the candidate
run. Run candidate controls 21010:25 and 21006:21 first; if 21010:25 still
diverges during lift, stop before replaying the candidate cohort. Otherwise,
compare paired outcomes and per-waypoint forces. Keep the intervention only if
the candidate fresh success rate exceeds both the frozen uniform-profile
cohort and the historical 3/7 rate, every claimed success passes the
independent hold verifier, the known 21010:25 divergence is removed, and no
refusal control becomes a false acceptance. Any failure is a simulation-only
result; no physical transfer, friction measurement, safety approval,
admission, or release claim is implied. If it does not pass this rule, restore
the prior profile and record the negative result.
