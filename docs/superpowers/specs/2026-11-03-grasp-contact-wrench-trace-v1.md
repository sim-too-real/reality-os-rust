# Grasp Contact Wrench Trace v1

## Goal

Retain enough simulator evidence from the two frozen failed grasp-lift trials to separate contact geometry and load distribution from actuator realization in a later offline diagnostic.

## Trace contract

For the post-close state and each observed lift waypoint, retain the object inertial center in world coordinates, `ctrl`, raw `actuator_force`, and each object-finger contact's body names, world contact position, local force and torque, contact frame, friction tuple, contact dimension, and force-availability flag. Also retain the derived world force and moment about the object inertial center, with sign applied to the object body.

MuJoCo contact frame axes are stored as rows and the normal points from geom 1 to geom 2. `mj_contactForce` reports the contact wrench on geom 2, so negate the wrench when the object is geom 1. Preserve raw values alongside derived values. Missing or non-finite geometry/wrench inputs produce absent derived values; they must not become zero-valued evidence.

## Scope and limits

- Telemetry only: do not change grasp control, lift trajectories, thresholds, Governor behavior, admission, or release authority.
- Keep old episode JSON readable with defaults for the new fields.
- Replay only frozen trials `v7:grasp_fixture:arm_gripper_lift:pinch_grasp_lift:21010:25` and `...:21006:21` using their already frozen controls.
- The output is simulation-only diagnostic evidence. It is not a physical capacity proof, prediction, or validation of hardware.
- Use only the canonical local `main` checkout; do not create or use worktrees or repository copies.

## Acceptance

1. Rust tests prove state parsing retains contact position, contact torque, body inertial center, control, and actuator force.
2. Rust tests prove world-frame force and moment conversion for both geom orderings, including a nonzero contact torque, and verify missing inputs remain absent.
3. Both frozen trials produce post-close and lift samples with raw and derived fields; the replay leaves control and safety behavior unchanged.
4. A separate verifier checks contact-wrench sign, finite-value handling, sample phases, and source/result hashes.
