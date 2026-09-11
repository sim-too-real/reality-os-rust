# External embodiment generality (not Phase B)

**Date:** 2026-09-11  
**Status:** implementation spec for the generality proof  
**Baseline `main`:** `9806282cf4864bd06f90750b1d5880843f039ddb` (PR #12 merged)  
**Linux CI on that SHA:** `authority` (`verify` + `verify-mujoco`) and `os-users` green  

This pass tests whether Foundation REACH semantics generalize to realistic externally authored robots. It is **not** Phase B. Do not implement GRASP, PUSH, PLACE, vision, VLA, behavior planning, tactile, or locomotion.

## Frozen

Do not redesign or edit:

- kernel authority semantics
- governor / session authority
- metal path
- consume / replay
- `RealityOs::decide`
- SIM ≠ METAL honesty

Preserve Foundation V1 fixtures and evidence: `planar_arm`, `spatial_arm4`, `wrist_offset_arm`. They remain internal fixtures, not external-generalization evidence.

## Hypothesis

A realistic robot model not authored for Reality OS can be ingested, mechanically understood, and controlled through the same semantic REACH pathway **without product-name-specific semantic logic**.

Honest outcomes:

- A: `EXTERNAL EMBODIMENT GENERALIZATION DEMONSTRATED`
- B: `GENERALIZATION LIMIT FOUND` (smallest missing abstraction preserved)

## Pipeline

```
compiled MuJoCo model
  → complete available RobotManifest (axes, SE3, sites, actuators)
  → EmbodimentModel
  → diagnostics
  → capability derivation
  → skill compilation
```

No compile-fail-then-hydrate. Unknown stays unknown. No identity rotations invented.

## Hold-out

1. Stage A: Menagerie UR5e only. Generic semantics until REACH works or an honest unsupported diagnostic.
2. Commit semantics as `GENERALITY_FREEZE_SHA`.
3. Stage B: import Panda, record first REACH score **before** any further semantic edit.
4. First Panda artifact is never rewritten.

## External source pin

- Repository: `https://github.com/google-deepmind/mujoco_menagerie`
- Commit: `8161bba264d7fa7c99ca301e91e7fb44737676ad`
- UR5e path: `universal_robots_ur5e/ur5e.xml`
- UR5e license: BSD-3-Clause (ROS Industrial Consortium)
- Panda path (Stage B only): `franka_emika_panda/panda.xml`

Pinned fetch/cache with provenance. Do not flatten or rewrite vendor kinematics.

## Semantic upgrades (generic)

- Full `Se3` (translation + normalized wxyz) on bodies, joints, sites, cameras.
- Joint types: hinge / slide / fixed implemented; ball / free → `KINEMATICS_UNSUPPORTED_FOR_ADAPTER`.
- `JointStateSample` separate from kernel citation; REACH requires fresh current q.
- IK seeds from current chain q; prefer near current valid solution.
- `JointTargetSet` / named lowering; no invented gripper commands; no index==joint.
- Capabilities scoped to chain / EE; grasping stays unproven.
- `ReachGoal { end_effector, target: PoseEvidence }` — EE ≠ target frame.
- Operational `TransformGraph` (lookup, inverse, path, compose, cycle/disconnect, freshness, epoch).
- No manual base-position subtraction. Rotated-base world targets must resolve through SE3.

## Forbidden semantic identifiers

`ur5`, `ur5e`, `panda`, `franka` may appear only in fixture registry, licenses, bundle selection, test names, or hardware adapter boundaries — never in capability / FK / IK / SkillContract / world / transform logic.
