# Foundation design: physical-intelligence schemas + REACH

**Date:** 2026-09-10  
**Status:** design for implementation planning; not implemented  
**Baseline `main` at inspection:** `5cd9e602cb89d4241176eeffc08d1bbb86902f3d`  
**Baseline `main` after CI-only toolchain pin (PR #10):** `1141179f254536c856f328142236632a4602078c`  
**Scope:** Phase A / Foundation only. Stop after evidence review. Do not start vision, VLA, tactile, behavior planning, locomotion, or metal transfer.

This document grounds the Foundation milestone in the current `sim-too-real/reality-os-rust` tree. It does not redesign the authority kernel.

## 0. Non-negotiables

- Authority kernel, metal ONLINE path, consume/replay, and SIM ≠ METAL are **frozen**.
- Intelligence may propose. It may never write actuators or confer authority on itself.
- Unknown physical values stay `UNKNOWN`. Assumed stays `ASSUMED`. Measured stays distinguishable from inferred. Simulation stays distinguishable from metal.
- No core branch on product or fixture identity (`if robot_id == "…"`) except explicit test fixtures and external hardware adapters.
- Do not claim robot-agnostic because three similar fixture IDs share an interface.
- Do not implement Phases B–I in this milestone.

## 1. Current tree (evidence)

The repo is a physical-action authority kernel plus a MuJoCo verification harness. It is not yet a general physical-intelligence runtime.

| Wanted concept | What exists | Honest status |
|---|---|---|
| `EmbodimentModel` | `EmbodimentGraph` (joint limits, invented serial frames, `mass_kg: None`) | limit table |
| `CapabilityGraph` | `Capability` + `capabilities_for_kind` | morphology label |
| Kernel `ObservationEvidence` | sensor id, cal hash, times, digest, transform epoch, quality, OOD, expiry | authority handle |
| `WorldState` | `WorldBelief` (epoch, surfaces, object name strings) | shallow |
| `SkillContract` | `SkillIR::hold()` | stub |
| REACH | `TaskSpec::Reach`; runner IK + `PdPolicy` → `SimAuthority` | task + scripted policy, not a semantic skill |
| Camera | `VisionMode::Camera` = `NOT_IMPLEMENTED_IN_VERIFY_V1` | leave unimplemented |

Executable MJCF corpus today: `planar_arm`, `arm_gripper`, `cartpole`.  
`planar_arm` and `arm_gripper` are near-isomorphic 3R planar Z-hinge arms. `cartpole` is 1-actuator underactuated. `arm6` / `unitree_h1` are JSON limit tables, not MuJoCo execution models.

Existing write path (keep):

```
ActionProposal → RealityOs::decide → session/governor → plant → MuJoCo port → privileged verifier
```

`reach` already maps to the `workspace_boundary` domain plugin. Do not edit `decide()`.

`verify-mujoco` rustup/clippy bootstrap on `@stable` vs `rust-toolchain.toml` `1.95.0` was **not** fixed on `5cd9e60`. It was fixed only in CI plumbing (PR #10) and is green on Linux. Do not treat that as a product change.

## 2. Architecture

```
GOAL / WorldState target
        ↓
SkillContract(REACH)          realityos-semantics   NEW; no actuator I/O
        ↓
CapabilityGraph check
        ↓
ControlAdapter → ActionProposal
        ↓
RealityOs::decide             FROZEN
        ↓
session / governor / plant    FROZEN
        ↓
MuJoCo via existing verify port
        ↓
privileged verifier           verify only
        ↓
OutcomeEvidence               metal=false, SIMULATION_ONLY
```

### 2.1 New crate `realityos-semantics`

Path: `crates/semantics`.

Owns: `EmbodimentModel` v2, `SensorModel`, `SensorObservation` (ObservationEvidence v2), `ObservationFrame`, `TransformGraph`, `WorldState` v1, `CapabilityGraph`, `SkillContract` v1, `ControlAdapter` trait, REACH compiler, refuse/probe reasons, `ExplorationBudget` type (bounds only).

Must not own: `Plant::act`, ledgers, ONLINE metal, MuJoCo stepping, privileged simulator truth.

Depends on: `realityos-kernel` only (status words, finite checks, citation of kernel `ObservationEvidence`).  
Does not depend on: plant, governor, session, metal, verify.

### 2.2 `realityos-verify`

Changed, not redesigned. Maps inspected `RobotManifest` + bundle YAML into `EmbodimentModel` v2 with provenance. Derives `CapabilityGraph` (may call existing `qualify` for SIM evidence). Compiles REACH through semantics. Writes only through existing `SimAuthority`. Privilege and MuJoCo stay here.

### 2.3 Frozen / do not edit except a concrete bug

`kernel`, `plant`, `governor`, `session`, `metal`, `RealityOs::decide`, consume/replay journals, SIM ≠ METAL honesty tokens.

### 2.4 Leave alone

- `EmbodimentGraph` and `gauntlet` (old limit-table fixtures).
- `SkillIR::hold()` in `reality-os`.
- `reality-os::Controller` (named clamp, not a ControlAdapter).
- Camera policy path.

### 2.5 Approach rejected

- Growing kernel types in place (unfreezes the kernel).
- Hosting Foundation schemas only inside `verify` (traps the product in the harness).
- A parallel runtime that writes the plant beside `decide()` (intelligence confers authority).

## 3. Provenance and value rule

Every physical quantity is:

```text
Provenanced<T> {
  value: Option<T>       // None ⇒ UNKNOWN; never a guessed default
  provenance: Provenance
  source: String
  as_of_s: f64
  uncertainty: Option<T> // absent ≠ zero
}
```

`Provenance`: `MODEL_DECLARED | CALIBRATION_MEASURED | HARDWARE_MEASURED | SIMULATOR_DERIVED | USER_DECLARED | LEARNED_ESTIMATE | ASSUMED | UNKNOWN`

Illegal: collapsing provenance into a float. Illegal: filling `None` with typical mass, friction, stiffness, calibration, payload, or effort. `ASSUMED` requires an explicit recorded assumption. `SIMULATOR_DERIVED` cannot become `HARDWARE_MEASURED` and cannot set `metal: true`.

## 4. Schemas

Schema family: `realityos.semantics/1`. Canonical JSON forms are hashed. Unknown fields stay explicit.

### 4.1 `EmbodimentModel` v2

Tree, not a forced serial chain.

- Identity: `robot_id` (fixture key; never switched on in semantics), `source_bundle_hash`, `model_hash`, `calibration_epoch`, `model_version`.
- `Body`: name, parent, mass/COM/inertia as `Provenanced`.
- `Joint`: name, type (`hinge | slide | ball | free | fixed | other`), axis, qpos/dof dims, position/velocity/acceleration/effort limits as `Provenanced`, parent/child bodies.
- `Actuator`: name, target joint, control mode, ctrlrange, forcerange, gear if declared — all `Provenanced`.
- `Frame`: EE, tool, camera, sensor, task; parent body/site.
- `Geometry`: collision / visual / contact, each optional, provenance required if present.
- `EndEffector`, `Gripper`, `Transmission`.
- `diagnostics: ModelDiagnostic[]` (malformed topology, missing inertia, questionable collision, unsupported transmission, actuator/joint mismatch, missing semantic frames, unsupported/uncertain features).
- `metal: false` for all Foundation ingestions.

Ingestion: MJCF and URDF via existing verify compile + inspect. USD remains experimental (`FormatDisposition` already says so). CAD/STEP is not a robot. Do not auto-repair uncertain physics without recording the transformation as a diagnostic.

Mapping source: `verify::RobotManifest` fields already include joints (type, dims, range, parent/child), actuators (target, ctrl/force ranges, type), bodies (mass, inertia), sites, EE chains, hashes, lost features. Those become `MODEL_DECLARED` or `SIMULATOR_DERIVED`. Missing → `UNKNOWN`.

The mapper lives in `verify` (`semantics_map`). `realityos-semantics` exposes constructors/`TryFrom` parts and never imports MuJoCo or `RobotManifest`.

### 4.2 `SensorModel`

`sensor_id`, semantic class (`RgbCamera | DepthCamera | JointEncoder | JointVelocity | ActuatorTorque | ForceTorque | Imu | …`), parent frame, calibration id/hash, rate, latency as `Provenanced`, units, shape. Absence of a class is `NOT_APPLICABLE`, not a dummy sensor.

### 4.3 `SensorObservation` (ObservationEvidence v2)

New type. Do not mutate kernel `ObservationEvidence`.

Fields: sensor id/class, robot frame, calibration hash, capture_s, receive_s, sequence, clock_domain, units, shape, validity, latency, provenance, content digest, expires_at, failure flags, optional covariance. Payload is digest + typed reference, not a naked array used as truth.

When `decide()` needs a kernel handle, construct kernel `ObservationEvidence` from digest, calibration hash, times, transform epoch, quality/OOD, expiry. Do not pass pixels into the kernel.

### 4.4 `ObservationFrame`

Synchronized bundle: `frame_id`, `transform_epoch`, per-sensor `SensorObservation`, stale/missing/late flags. Authority may refuse when required evidence is stale.

### 4.5 `TransformGraph`

Edges: parent, child, SE3 as `Provenanced`, timestamp, uncertainty, `calibration_epoch`, source. Mixing calibration epochs is a hard error. Foundation frames: `world`, `base`, links, EE, declared cameras/task frames.

### 4.6 `WorldState` v1

Enough for REACH, not a scene-graph product.

- `transform_epoch`, `as_of_s`
- target pose belief: `Provenanced<[f64; 3]>` + freshness + frame id
- optional object hypotheses (id, pose, provenance); empty is valid
- free-space / occlusion / friction: `UNKNOWN` unless evidenced
- no `floor = concrete`

### 4.7 `CapabilityGraph`

Nodes are semantic capabilities. Each node:

- `status: PROVEN | SUPPORTED | PARTIALLY_SUPPORTED | UNVERIFIED | NOT_APPLICABLE | UNSUPPORTED`
- evidence ids, confidence only when evidence exists, dependencies, operating envelope, `unsupported_reason`

Not a boolean. `capabilities_for_kind("serial_arm")` is not this graph.

Foundation capability set that must be *derived* (others may exist as `UNVERIFIED` / `NOT_APPLICABLE`):

- `JOINT_POSITION_CONTROL`
- `JOINT_VELOCITY_CONTROL`
- `JOINT_EFFORT_CONTROL`
- `CARTESIAN_POSITION_CONTROL`
- `FIXED_BASE_MANIPULATION`
- `MOBILE_BASE`
- `FLOATING_BASE`
- `GRASPING` / `PARALLEL_GRIPPER` (derive; do not qualify GRASP)

### 4.8 `SkillContract` v1

Data, not a controller call.

Fields: id, semantic intent, required capabilities, required world evidence, preconditions, parameters, expected observations, authority requirements, motion/contact envelope, success evidence, failure evidence, recovery names, termination, exploration allowance.

IR names defined: `OBSERVE, LOOK_AT, REACH, GRASP, RELEASE, PUSH, PULL, MOVE_BASE, HOLD, PLACE, PRESS, TURN, INSERT, RETRACT, VERIFY_STATE`.

Foundation *qualifies* only `REACH`. Any other IR name returns `UNSUPPORTED` and must not produce a write. A string that is not in the IR cannot appear as a skill.

### 4.9 `ControlAdapter`

```text
fn advertise(&self) -> AdapterContract
fn compile(skill, model, caps, world, obs_frame) -> Result<ActionProposal, SkillRefuse>
```

`AdapterContract`: input semantic space, output control mode, required rate, latency assumptions, required sensors, required capabilities, failure signals, stopping semantics, envelope compatibility.

Foundation adapter: `IkPositionPdAdapter` — IK on the declared EE chain to joint position setpoints, then the existing position-PD proposal shape. Selected by capabilities (`JOINT_POSITION_CONTROL` + EE chain), not `robot_id`. Output is always `verify`-compatible `ActionProposal` (or an identical DTO mapped at the verify boundary).

Do not add `FrankaController` / `UR5Skill`. Do not use `reality-os::Controller` (clamp) as this trait.

### 4.10 `OutcomeEvidence`

Per episode: software SHA, model hash, calibration epoch, scenario seed, sensor configuration, skill id, single-node plan (`REACH`), authority decisions, adapter id, command ids, probes, world-state hashes, privileged task result, physical violations, uncertainty, counterexample on failure, `metal: false`, `evidence_status: SIMULATION_ONLY`, write counts, replay write delta.

Reuse verify evidence schema family where it already exists; extend rather than mint metal-capable fields.

## 5. Capability derivation (mechanical)

Inputs: `EmbodimentModel` + optional `ControlQualification` (SIM, already in verify).  
No `robot_id` switch.

| Capability | SUPPORTED when | else |
|---|---|---|
| `JOINT_POSITION_CONTROL` | ≥1 position actuator targeting a 1-DoF hinge/slide | `UNSUPPORTED` if no such actuator; `UNVERIFIED` until qualify if mapping exists but unprobed |
| `JOINT_VELOCITY_CONTROL` | velocity/motor actuator targeting a 1-DoF joint | `NOT_APPLICABLE` / `UNSUPPORTED` |
| `JOINT_EFFORT_CONTROL` | effort/motor actuator with declared force range | missing force range → `UNVERIFIED`, do not invent |
| `CARTESIAN_POSITION_CONTROL` | EE chain + `JOINT_POSITION_CONTROL` + IK-capable adapter | `PARTIALLY_SUPPORTED` without qualify; `UNSUPPORTED` if no EE frame |
| `FIXED_BASE_MANIPULATION` | `base_type == Fixed` and EE chain length ≥ 1 | `NOT_APPLICABLE` if floating/mobile |
| `MOBILE_BASE` | declared mobile base or prismatic/wheeled base joints with drive actuators | `NOT_APPLICABLE` on fixed manipulators |
| `FLOATING_BASE` | free/floating joint in tree | `NOT_APPLICABLE` on fixed base |
| `GRASPING` / `PARALLEL_GRIPPER` | gripper actuator + opening range declared | `UNSUPPORTED` without gripper; **not** PROVEN in Foundation |

Promotion to `PROVEN` requires stored `ControlQualification` evidence (tracking/stability probes). Local linear controllability may be attached as evidence. It is **not** a substitute for task-level REACH success.

`cartpole`: `JOINT_POSITION_CONTROL` may be `UNSUPPORTED` (motor on slider); `CARTESIAN_POSITION_CONTROL` `UNSUPPORTED`; REACH `NOT_APPLICABLE`. That is a capability-discovery win, not a REACH success.

## 6. REACH compile

`REACH(target_frame, target_xyz, radius)` approximately requires:

- Capability: `CARTESIAN_POSITION_CONTROL` status ∈ {`SUPPORTED`, `PARTIALLY_SUPPORTED`, `PROVEN`} **or** (`JOINT_POSITION_CONTROL` ∈ those statuses **and** EE joint chain exists).
- World: target pose `value` is `Some`, finite, not expired, same `transform_epoch` as the observation frame.
- Identity: proposal/model hash equals current `EmbodimentModel.model_hash`.
- EE frame exists on the model.
- Adapter can produce a finite joint/ctrl proposal.

Authority envelope (Foundation): command lifetime, observation freshness, workspace implied by joint limits, no invented effort if effort is `UNKNOWN`.

**Success** (privileged verifier only): EE site/body within `radius` of target. Adapter `ok` is not success.

**Failure / refuse** (zero additional policy writes when listed):

| Condition | Outcome |
|---|---|
| missing target pose | `PROBE` (status only; no LOOK_AT motion in Foundation), 0 writes |
| stale target / stale joints | `REFUSE`, 0 writes |
| wrong model hash | `REFUSE`, 0 writes |
| unreachable IK / no candidate | `UNREACHABLE`, 0 writes |
| missing relevant actuator | skill `UNSUPPORTED`, 0 writes |
| calibration/transform epoch mismatch | `REFUSE`, 0 writes |
| command replay / authority restart replay | 0 additional writes |
| controller/policy crash | safe hold/zero via existing authority, 0 or safe-state only |
| unknown effort bound | remains `UNVERIFIED`; do not invent; do not claim PROVEN |

`decide()` verb remains `reach` → `workspace_boundary`. Semantics never calls `plant.act`.

## 7. Robot corpus

Do not use `{planar_arm, arm_gripper, cartpole}` as three REACH embodiments.

| Role | Bundle | Morphology | Notes |
|---|---|---|---|
| Development A | `robots/bundles/planar_arm` | 3R planar, all hinges `axis=0 0 1` | existing original MIT fixture |
| Development B | `robots/bundles/spatial_arm4` | 4-DoF spatial: Z hinge, Y hinge, Y hinge, Z hinge | **new original** MJCF; different joint axes than A |
| Held-out | `robots/bundles/wrist_offset_arm` | 5-DoF, non-intersecting wrist offset, mixed axes | **new original** MJCF; first opened at evaluation |
| Not REACH-dev | `arm_gripper` | planar 3R + fingers | later GRASP; not a REACH twin |
| Discovery only | `cartpole` | underactuated slider+hinge | REACH `NOT_APPLICABLE` |

Both new models: original geometry, MIT license block in `robot.yaml`, no third-party meshes, `end_effectors.site: ee`, position actuators, fixed base. No vendor names.

**Quarantine:** `crates/semantics` must not contain the string `wrist_offset_arm`. Held-out is loaded by path/hash from verify integration only. First held-out score is recorded even if it fails.

Adaptation report for Foundation: `CONFIGURED` (bundle + derived caps). Do not claim `ZERO_SHOT` if we authored the MJCF. Do not claim `METAL_ADAPTED`.

## 8. Tests

Unit tests in `realityos-semantics` (development robots and synthetic models only):

- Provenance: missing mass stays `UNKNOWN`; no default friction.
- Tree ingest: ball/free/fixed joints do not crash; unsupported features become diagnostics.
- Capability derivation on synthetic trees (fixed arm, floating base, wheeled, gripperless, no EE).
- REACH refuse: missing pose, stale pose, wrong hash, unreachable, missing actuator, epoch mismatch.
- No `robot_id` match arms in semantics sources (`rg` gate in test).

Verify integration (MuJoCo, `REALITYOS_REQUIRE_MUJOCO` in CI):

- Same `SkillContract::REACH` on `planar_arm` and `spatial_arm4` through `IkPositionPdAdapter` → `SimAuthority` → privileged EE success metric.
- Held-out `wrist_offset_arm` first-run evaluation; no semantics edits afterward to make it pass silently.
- Authority-negative: replay, restart, crash, wrong hash → write counts / replay delta = 0 extra policy writes.
- Report `metal: false`, `SIMULATION_ONLY`.
- Failed seeds retained; reduction uses existing `verify::reduce` where applicable.

Anti-cheat: policy does not receive privileged truth as if it were camera; success is not hardcoded; unknown effort is not replaced; held-out first result is recorded.

## 9. Foundation acceptance

The phase passes only if all of the following are true:

1. `planar_arm` and `spatial_arm4` execute the same `REACH` SkillContract via capability-derived adapter mapping.
2. `wrist_offset_arm` is evaluated as held-out without semantics robot-name branches; first result recorded.
3. Robot-specific differences exist only in model/adapter *data* (chains, limits, hashes), not in SkillContract logic.
4. All physical/world observations used for the skill have freshness and provenance.
5. Unknown properties remain explicit (`UNKNOWN` / `UNVERIFIED`).
6. Missing/stale evidence causes `PROBE`/`REFUSE`, not guessed certainty, and no physical ctrl write.
7. Reality OS (`decide` + session/governor/plant) remains the only actuation authority.
8. No artifact claims metal verification.
9. Failures are reproducible (seed + hashes).
10. `authority` CI green on Linux, including `verify-mujoco`.

## 10. Required output package

After implementation (not this document):

- exact SHA
- files changed
- architecture diagram (this §2)
- schema definitions (this §4)
- robot corpus and held-out identity
- derived CapabilityGraphs (machine-readable)
- REACH SkillContract
- ControlAdapters used
- scenario counts, task success, safe completion, refusal/probe behavior
- authority write counts, replay write deltas
- counterexamples
- known limitations
- explicit **NOT IMPLEMENTED** list (below)

## 11. NOT IMPLEMENTED in Foundation

Advanced vision / RGB-D policy, VLA, tactile, F/T servoing, behavior planner, PROBE motion skills (`LOOK_AT`, `CHANGE_VIEWPOINT`) beyond refuse/probe *status*, GRASP/PUSH/PLACE qualification, locomotion, world-estimator fusion, human tracking, first-principles merge, Sim2Real training, 100 robots, metal transfer, safety certification, global controllability claims, `world` / `sensors` crates, USD as a supported ingest path.

## 12. Implementation order (for the later plan)

1. Workspace member `crates/semantics` with provenance + schemas + tests.
2. Ingest mapper in `verify::semantics_map`: `RobotManifest` → `EmbodimentModel`.
3. Capability derivation + tests on synthetic + development bundles.
4. `SkillContract::REACH` + `IkPositionPdAdapter` + refuse matrix.
5. Author `spatial_arm4` and `wrist_offset_arm` original MJCF.
6. Wire verify episode path: Goal → WorldState → contract → caps → adapter → existing `SimAuthority` → privileged `OutcomeEvidence`.
7. Held-out first evaluation; record; do not silent-tune.
8. Stop. Review evidence.

No kernel/plant/governor/session/metal edits unless a concrete compile bug appears.
