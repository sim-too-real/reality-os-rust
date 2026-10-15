# Typed pinch-grasp authority design

**Date:** 2026-10-15

**Status:** approved by user on 2026-10-15

**Repository baseline:** local `main`, `54c7e243486ffe3c97d704e506fdbb6012616790`, with the previously approved uncommitted reasoner-authority bridge changes in the working tree
**Evidence class:** simulation only

## Goal

Extend the reasoner-to-authority bridge from planar push/probe to one deliberately narrow pinch-grasp action, while making the gripper commands and hold-verification motion part of the same frozen, scoped action. A grasp is successful only when an independent simulator oracle observes two-sided contact and the object follows the end effector through the bounded lift.

## Current gap

`realityos-reasoner-bridge` accepts only `planar_push`. It correctly refuses a proved `pinch_grasp` because `PhysicalActionScope` only expresses a scalar linear stroke. The existing grasp reasoner can prove static pinch mechanics and a contact path, but it does not freeze a lift trajectory or bind named gripper positions and force limits into authority. The existing privileged `verify_grasp` also treats any one finger contact as acquisition and may report success from contact plus a partly open gripper, without requiring that the object follow the end effector. Historical manipulation evidence separates acquisition from verified hold: the Panda report records 220 acquisition successes and 12 verified holds. Neither the reasoner's candidate nor a close command is sufficient evidence of holding an object.

## Chosen approach

Add a small, typed action-envelope enum that distinguishes a linear stroke from a two-finger pinch-grasp transaction. Keep the current `planar_push` behavior and bind its exact stroke through the linear variant. The grasp variant binds the named gripper resource, the exact measured opening and proved close targets, a finite force ceiling, the arm lift witness, and a maximum lift distance. It does not accept arbitrary string labels as a substitute for a resource or witness.

The physical reasoner will construct this envelope only when its current proof establishes the complete approach, contact, close, and bounded lift path for a supported two-finger resource. `prepare_step` will re-prove the complete step and envelope against the current request. The bridge will issue a `SIMULATION_ONLY` grant for that typed envelope, attach it to a copy of the same request, run canonical selection again, and authorize only an exact match. The bridge remains free of plant and hardware writers.

## Action and evidence flow

1. The reasoner receives a fresh request containing the measured arm and finger states, object pose, named two-finger resource, and explicit grasp-force bound. Unknown or non-finite required values do not receive fixture defaults.
2. The reasoner proves the arm approach and contact, the named finger close targets within declared joint limits, and a short vertical lift path. The same witness commits to every arm and finger position used by execution. The lift stays below the contract's maximum distance.
3. The selected `PlanStep` carries a typed grasp envelope as well as the exact witness digest and contents. `prepare_step` checks the entire step, including the typed envelope, against a fresh proof before the bridge can request authority.
4. The bridge derives scope only from the prepared action, issues a simulation grant, and reselects against the identical observation. A changed resource, target, close position, force ceiling, witness, observation epoch, or lift distance refuses.
5. The simulator executes only the frozen approach/contact/close/lift path. The executor checks the action-specific envelope before each phase and stops on a bound, freshness, contact-policy, or tracking violation.
6. The existing privileged verifier reads simulator state unavailable to the policy. It reports acquisition and verified hold independently. Acquisition requires contact with each of the two declared finger bodies. Verified hold requires both contacts, at least 0.02 m of measured vertical end-effector lift within the authorized maximum, object-follow residual no greater than 0.005 m, `UNSUPPORTED` object support relation with both finger contacts still present, available finite per-finger normal-force measurements no greater than the authorized ceiling, and no forbidden contact. The result remains `SIMULATION_ONLY`.

## Typed contract

Use one public serializable `PhysicalActionEnvelope` enum shared by the semantics, session, and bridge layers:

- `LinearStroke { requested_m, maximum_m }` for the already-supported push and probe path.
- `PinchGrasp { resource_id, finger_joint_names, open_positions, close_positions, per_finger_force_ceiling_n, lift_trajectory_digest, maximum_lift_m }` for the new simulation-only grasp path.

The reasoner request must carry the existing typed `ControlledResource` for the gripper, and the resource must match the embodiment's direct actuator-to-slide-joint mapping. The resource id, affected joint names, topology, explicit force bound, and command range must all agree with the current model. `open_positions` and `close_positions` are measured and commanded joint coordinates in each joint's model coordinate; do not assume radians for slide joints. Every numeric member must be finite and satisfy its physical range checks. The name and position vectors must have equal lengths and exactly two distinct fingers for this initial variant. Close targets must be within declared limits and move from the measured open coordinates toward closure. The per-finger force ceiling must be positive, explicitly supplied, equal to the force bound used in the mechanics proof, and no larger than the resource's declared maximum force. `maximum_lift_m` must be at least 0.02 m and equal to the vertical distance established by the lift witness. The lift trajectory digest commits to the ordered named joint states for the lift phase. No tendon, coupled, multi-finger, suction, or unqualified resource is admitted by this variant.

`PlanStep`, `FrozenAction`, `PhysicalActionScope`, and `ExecutionAuthorization` carry this same enum as the canonical action authority. Replace the scope and authorization scalar stroke/bound fields with the enum; keep any planner ranking metric separate from authority and never interpret it as a grasp command. The frozen action's grant-coverage check requires exact envelope equality. The scope digest uses a versioned, domain-separated, length-delimited encoding of strings, enum tags, and exact `f64::to_bits()` values; formatted decimal text and free-form serialization are not used for the authority digest. Equality uses the exact structured values selected by the reasoner; it does not reconstruct a grasp envelope from a candidate summary or action name.

## Proof and outcome boundaries

The reasoner must prove the lift trajectory as part of the same composite action. It may use the predicted grasp attachment to check the carried object's swept volume, but this prediction cannot establish success. The verifier independently compares measured object motion with measured end-effector motion after execution. A commanded close, geometric proof, static friction calculation, one-sided contact, or a simulator contact by itself is not verified hold. The verifier's `grasp_acquired` evidence is counted independently of task success; `verified_hold` is recorded only when every hold condition passes. A task result of success for this grasp objective means verified hold.

Unknown force, friction, object mass, object pose, joint state, resource topology, collision geometry, or sensor freshness remains unknown and refuses grasp execution. Refusal happens before control writes. This milestone consumes the existing simulation request freshness contract; it does not claim that the boolean freshness marker authenticates a physical camera or encoder. Runtime divergence during any phase invalidates the remaining frozen motion and returns an abort requiring fresh observation and re-planning; it cannot resume the old lift.

## Scope limits

- Simulation only: the typed grasp grant is never a metal grant and adds no hardware transport or hardware qualification.
- One supported two-independent-finger position resource with explicit limits; support for tendon, coupled, suction, dexterous, or compliant grippers is deferred.
- One bounded vertical lift in a static scene; placement, regrasp, in-hand manipulation, dynamic obstacles, tactile servoing, and whole-body motion are out of scope.
- The simulator oracle stays isolated from policy selection and authority. Camera calibration, physical sensor authentication, fleet management, and safety certification remain separate milestones.
- No new dependency is required.

## Failure behavior

Use explicit refusals for unsupported resource topology, missing force or friction, stale or mismatched observation, malformed bounds, invalid lift proof, exact-step mismatch, unsupported action family, expiry, and runtime divergence. A pre-execution refusal must not expose an executable action or increase simulator control-write counts. A post-execution failure may report contact acquisition without verified hold; it must never promote that result to successful grasp, and it must leave unauthorized writes at zero.

## Acceptance tests

1. Unit tests prove a typed `PinchGrasp` scope digest changes when any resource, finger position, force, witness, epoch, or lift bound changes; delimiter-containing strings cannot alias another scope; malformed/non-finite and dimension-mismatched envelopes refuse.
2. Reasoner tests prove exact preparation accepts the current full grasp step and rejects a widened force ceiling, changed finger target, altered lift path, or stale initial configuration.
3. Bridge tests authorize a current supported two-finger grasp only after exact re-selection; unknown force/friction, unsupported topology, stale evidence, expiry, and forbidden action all refuse with zero control writes.
4. Tests for the existing privileged verifier show that one-sided contact, unavailable/over-limit force, insufficient lift, excessive object-follow residual, remaining table support, or forbidden contact cannot produce a verified hold. They separately count bilateral contact acquisition when a later hold check fails.
5. A MuJoCo integration test uses no hand-built plan, scope, grant, or authorization. It executes the frozen composite grasp and requires bilateral finger contacts, at least 0.02 m of measured object-following lift within 0.005 m tracking residual, support separation, available in-bound contact force, no forbidden contact, and zero unauthorized writes. It also tests a failed acquisition or insufficient force case and confirms that the result is not labeled a verified hold.
6. Existing push/probe bridge behavior and its exact-scope tests remain passing.

## Risks and containment

The principal risk is that the simulator's grasp attachment assumption could be mistaken for proof that the object was actually acquired. Keep that assumption inside collision prediction only; success comes from independent post-execution measurements. Another risk is that a nominal force value could be mistaken for a hardware capability. The bridge and verifier remain simulation-only, and this design does not qualify the force bound for hardware. A failed lift-path collision proof must refuse rather than omit the carried object from collision checking.

## Success statement

Reality OS can authorize one exact, bounded, simulation-only pinch-grasp transaction through the reusable bridge, then distinguish contact acquisition from verified hold using independent simulator measurements. It makes no hardware-readiness or general-grasping claim.
