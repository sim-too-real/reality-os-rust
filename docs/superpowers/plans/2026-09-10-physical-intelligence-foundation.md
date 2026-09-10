# Physical Intelligence Foundation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement Foundation only: `realityos-semantics` schemas plus one semantic `REACH` path on `planar_arm`, `spatial_arm4`, and held-out `wrist_offset_arm`, writing solely through frozen Reality OS / `SimAuthority`, then stop.

**Architecture:** New crate `realityos-semantics` owns typed models, capability derivation, `SkillContract`, and `ControlAdapter`. `verify` maps `RobotManifest` → `EmbodimentModel`, implements MuJoCo-backed IK, and remains the only privileged stepper. Kernel, plant, governor, session, metal, and `RealityOs::decide` stay frozen.

**Tech Stack:** Rust 1.95.0 (`rust-toolchain.toml`), workspace edition 2021, `realityos-kernel`, existing `realityos-verify` MuJoCo worker, serde/thiserror. No new Python packages. No VLA/vision/planner crates.

**Spec:** `docs/superpowers/specs/2026-09-10-physical-intelligence-foundation-design.md`

## Global Constraints

- Frozen: `crates/kernel`, `crates/plant`, `crates/governor`, `crates/session`, `crates/metal`, `RealityOs::decide`, consume/replay, SIM ≠ METAL.
- `realityos-semantics` depends on `realityos-kernel` only. It must not depend on plant, governor, session, metal, or verify.
- No `if robot_id == "…"` in `crates/semantics`. The string `wrist_offset_arm` must not appear anywhere under `crates/semantics`.
- Unknown physical values stay `Provenance::Unknown` with `value: None`. Do not invent mass, friction, effort, or calibration.
- Every Foundation episode: `metal: false`, `evidence_status: "SIMULATION_ONLY"`. Never emit `METAL_VERIFIED`.
- Do not add robots to `milestone_robots()` / `corpus_ids()`. CI `verify-mujoco` asserts `scheduled == 3 * 10 * 2 * 1`. Breaking that is a plan failure.
- `reach` verb still maps to existing `workspace_boundary`. Do not register a new domain plugin.
- ControlAdapter output is `CompiledCtrl`. `verify` wraps it into existing `ActionProposal`. Semantics never calls `Plant::act`.
- Stop after Foundation evidence. Do not start GRASP, vision, VLA, planner, tactile, or metal.
- Adaptation label is `CONFIGURED`, never `ZERO_SHOT` or `METAL_ADAPTED`.

---

## File map

| File | Responsibility |
|---|---|
| `crates/semantics/Cargo.toml` | New crate manifest |
| `crates/semantics/src/lib.rs` | Module root, `SCHEMA_FAMILY` |
| `crates/semantics/src/provenance.rs` | `Provenance`, `Provenanced<T>` |
| `crates/semantics/src/embodiment.rs` | `EmbodimentModel` v2 tree |
| `crates/semantics/src/sensor.rs` | `SensorModel`, `SensorClass` |
| `crates/semantics/src/observation.rs` | `SensorObservation`, `ObservationFrame`, kernel citation |
| `crates/semantics/src/transform.rs` | `TransformGraph`, epoch check |
| `crates/semantics/src/world.rs` | `WorldState` v1 |
| `crates/semantics/src/capability.rs` | `CapabilityGraph` + `derive_capabilities` |
| `crates/semantics/src/skill.rs` | `SkillContract`, `SkillName`, `SkillRefuse` |
| `crates/semantics/src/adapter.rs` | `ControlAdapter`, `CompiledCtrl`, `ChainIkPositionPdAdapter` |
| `crates/semantics/src/reach.rs` | `compile_reach` |
| `Cargo.toml` | workspace member + `realityos-semantics` dep |
| `crates/verify/Cargo.toml` | depend on `realityos-semantics` |
| `crates/verify/src/semantics_map.rs` | `RobotManifest` → `EmbodimentModel` |
| `crates/verify/src/reach_foundation.rs` | end-to-end REACH via `SimAuthority` |
| `crates/verify/src/lib.rs` | `mod semantics_map; mod reach_foundation;` |
| `crates/verify/src/bin/realityos-verify.rs` | `foundation-reach` subcommand |
| `robots/bundles/spatial_arm4/*` | development robot B |
| `robots/bundles/wrist_offset_arm/*` | held-out robot (verify only) |
| `docs/superpowers/specs/2026-09-10-foundation-output.md` | written after implementation, not now |

Do not modify: `crates/kernel/**`, `crates/plant/**`, `crates/governor/**`, `crates/session/**`, `crates/metal/**`, `crates/reality-os/src/decide.rs`, `crates/verify/src/corpus.rs` robot lists, `.github/workflows/authority.yml` smoke arithmetic.

---

### Task 1: Workspace crate + provenance

**Files:**
- Create: `crates/semantics/Cargo.toml`
- Create: `crates/semantics/src/lib.rs`
- Create: `crates/semantics/src/provenance.rs`
- Modify: `Cargo.toml` (members, default-members, workspace.dependencies)

**Interfaces:**
- Consumes: nothing
- Produces: `Provenance` enum; `Provenanced<T> { value: Option<T>, provenance: Provenance, source: String, as_of_s: f64, uncertainty: Option<T> }`; constructors `unknown`, `declared`, `assumed`; `SCHEMA_FAMILY = "realityos.semantics/1"`

- [ ] **Step 1: Write the failing test**

Create `crates/semantics/src/provenance.rs` with only the test module first (types commented or absent so the test fails to compile), then immediately add the test that encodes the law:

```rust
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn missing_mass_is_unknown_not_a_default() {
        let m: Provenanced<f64> = Provenanced::unknown("body.mass_kg", 0.0);
        assert!(m.value.is_none());
        assert_eq!(m.provenance, Provenance::Unknown);
        assert!(m.uncertainty.is_none());
    }

    #[test]
    fn assumed_requires_an_explicit_note_source() {
        let mu = Provenanced::assumed(0.6, "user:earth_indoor_screen", 0.0);
        assert_eq!(mu.provenance, Provenance::Assumed);
        assert_eq!(mu.value, Some(0.6));
        assert!(mu.source.contains("user:"));
    }
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cargo test -p realityos-semantics --lib provenance -- --nocapture`

Expected: FAIL — package `realityos-semantics` not in workspace.

- [ ] **Step 3: Write minimal implementation**

`crates/semantics/Cargo.toml`:

```toml
[package]
name = "realityos-semantics"
version.workspace = true
edition.workspace = true
license.workspace = true
rust-version.workspace = true
description = "Robot-agnostic physical semantics. Proposals only. Not metal. Not authority."
readme = false

[dependencies]
realityos-kernel.workspace = true
serde.workspace = true
serde_json.workspace = true
thiserror.workspace = true

[lints]
workspace = true
```

Add `"crates/semantics"` to both `members` and `default-members` in root `Cargo.toml`, and:

```toml
realityos-semantics = { path = "crates/semantics" }
```

`crates/semantics/src/lib.rs`:

```rust
//! Physical-intelligence schemas. No plant I/O. SIM ≠ METAL.

pub mod provenance;

pub const SCHEMA_FAMILY: &str = "realityos.semantics/1";
```

`crates/semantics/src/provenance.rs` (full):

```rust
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum Provenance {
    ModelDeclared,
    CalibrationMeasured,
    HardwareMeasured,
    SimulatorDerived,
    UserDeclared,
    LearnedEstimate,
    Assumed,
    Unknown,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Provenanced<T> {
    pub value: Option<T>,
    pub provenance: Provenance,
    pub source: String,
    pub as_of_s: f64,
    pub uncertainty: Option<T>,
}

impl<T> Provenanced<T> {
    pub fn unknown(source: impl Into<String>, as_of_s: f64) -> Self {
        Self {
            value: None,
            provenance: Provenance::Unknown,
            source: source.into(),
            as_of_s,
            uncertainty: None,
        }
    }

    pub fn declared(value: T, source: impl Into<String>, as_of_s: f64) -> Self {
        Self {
            value: Some(value),
            provenance: Provenance::ModelDeclared,
            source: source.into(),
            as_of_s,
            uncertainty: None,
        }
    }

    pub fn simulator_derived(value: T, source: impl Into<String>, as_of_s: f64) -> Self {
        Self {
            value: Some(value),
            provenance: Provenance::SimulatorDerived,
            source: source.into(),
            as_of_s,
            uncertainty: None,
        }
    }

    pub fn assumed(value: T, source: impl Into<String>, as_of_s: f64) -> Self {
        Self {
            value: Some(value),
            provenance: Provenance::Assumed,
            source: source.into(),
            as_of_s,
            uncertainty: None,
        }
    }
}
```

Keep the tests at the bottom of the same file.

- [ ] **Step 4: Run test to verify it passes**

Run: `cargo test -p realityos-semantics --lib`

Expected: PASS, 2 tests.

- [ ] **Step 5: Commit**

```bash
git add Cargo.toml crates/semantics/Cargo.toml crates/semantics/src/lib.rs crates/semantics/src/provenance.rs
git commit -m "Add realityos-semantics crate with explicit provenance values."
```

---

### Task 2: EmbodimentModel v2 tree

**Files:**
- Create: `crates/semantics/src/embodiment.rs`
- Modify: `crates/semantics/src/lib.rs`

**Interfaces:**
- Consumes: `Provenanced`, `Provenance`
- Produces: `JointKind { Hinge, Slide, Ball, Free, Fixed, Other }`, `BaseKind { Fixed, Floating, Mobile }`, `Body`, `Joint`, `Actuator`, `ModelFrame`, `EndEffector`, `Gripper`, `Transmission`, `ModelDiagnostic`, `EmbodimentModel { robot_id, source_bundle_hash, model_hash, calibration_epoch, model_version, base, bodies, joints, actuators, frames, end_effectors, grippers, transmissions, diagnostics, metal: bool }` with `metal` always `false` on `EmbodimentModel::new`. Helpers: `ee_joint_chain(&self, ee: &str) -> Option<Vec<String>>`, `position_actuators(&self) -> impl Iterator`.

- [ ] **Step 1: Write the failing test**

In `embodiment.rs` tests:

```rust
#[test]
fn tree_accepts_ball_and_fixed_without_inventing_mass() {
    let mut m = EmbodimentModel::new("synth", "src", "hash", "epoch0", "1");
    m.bodies.push(Body {
        name: "link".into(),
        parent: Some("base".into()),
        mass_kg: Provenanced::unknown("bundle", 0.0),
        com: Provenanced::unknown("bundle", 0.0),
        inertia: Provenanced::unknown("bundle", 0.0),
    });
    m.joints.push(Joint {
        name: "j_ball".into(),
        kind: JointKind::Ball,
        axis: Provenanced::unknown("bundle", 0.0),
        qpos_dim: 4,
        dof_dim: 3,
        parent_body: "base".into(),
        child_body: "link".into(),
        q_min: Provenanced::unknown("bundle", 0.0),
        q_max: Provenanced::unknown("bundle", 0.0),
        dq_max: Provenanced::unknown("bundle", 0.0),
        effort_max: Provenanced::unknown("bundle", 0.0),
    });
    m.diagnostics.push(ModelDiagnostic {
        code: "unsupported_joint".into(),
        detail: "ball not used by REACH adapter".into(),
    });
    assert!(m.bodies[0].mass_kg.value.is_none());
    assert!(!m.metal);
    assert_eq!(m.joints[0].kind, JointKind::Ball);
}

#[test]
fn new_model_is_never_metal() {
    let m = EmbodimentModel::new("x", "s", "h", "e", "1");
    assert!(!m.metal);
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cargo test -p realityos-semantics --lib embodiment -- --nocapture`

Expected: FAIL compile (`EmbodimentModel` missing).

- [ ] **Step 3: Write minimal implementation**

Implement the structs with `#[derive(Debug, Clone, Serialize, Deserialize)]`. `EmbodimentModel::new` sets `metal: false`, empty vecs, `base: BaseKind::Fixed`. `ee_joint_chain` walks `end_effectors` then listed joint names (do not invent a serial chain if none declared).

Export from `lib.rs`: `pub mod embodiment;`

- [ ] **Step 4: Run test to verify it passes**

Run: `cargo test -p realityos-semantics --lib`

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add crates/semantics/src/embodiment.rs crates/semantics/src/lib.rs
git commit -m "Add EmbodimentModel v2 as a kinematic tree with unknown-safe fields."
```

---

### Task 3: SensorModel, SensorObservation, ObservationFrame, kernel citation

**Files:**
- Create: `crates/semantics/src/sensor.rs`
- Create: `crates/semantics/src/observation.rs`
- Modify: `crates/semantics/src/lib.rs`

**Interfaces:**
- Consumes: `Provenanced`, `realityos_kernel::ObservationEvidence`
- Produces: `SensorClass` (`RgbCamera, DepthCamera, Rgbd, JointEncoder, JointVelocity, ActuatorTorque, ForceTorque, Imu, Other`); `SensorModel`; `SensorObservation`; `ObservationFrame { frame_id, transform_epoch, observations: Vec<SensorObservation>, as_of_s }`; `ObservationFrame::joint_stale(&self, now_s: f64) -> bool`; `SensorObservation::to_kernel_handle(&self, transform_epoch: &str, quality: f64, ood: f64) -> KernelResult<ObservationEvidence>`

- [ ] **Step 1: Write the failing test**

```rust
#[test]
fn stale_joint_observation_is_visible() {
    let obs = SensorObservation::joint_encoder("enc0", "j1", "cal", 1.0, 1.0, 1, "sim", "digest", 1.2);
    let frame = ObservationFrame {
        frame_id: "f1".into(),
        transform_epoch: "e0".into(),
        observations: vec![obs],
        as_of_s: 1.0,
    };
    assert!(frame.required_stale(2.0, 0.25));
    assert!(!frame.required_stale(1.1, 0.25));
}

#[test]
fn kernel_citation_does_not_carry_payload_arrays() {
    let obs = SensorObservation::joint_encoder("enc0", "j1", "cal", 1.0, 1.0, 1, "sim", "abc", 3.0);
    let k = obs.to_kernel_handle("e0", 0.9, 0.1).unwrap();
    assert_eq!(k.digest(), "abc");
    assert_eq!(k.transform_epoch(), "e0");
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cargo test -p realityos-semantics --lib observation -- --nocapture`

Expected: FAIL compile.

- [ ] **Step 3: Write minimal implementation**

`SensorObservation` stores metadata + digest + `expires_at_s`. No `Vec<f64>` payload field. `required_stale(now, freshness)` is true if any `JointEncoder` observation has `now - receive_s > freshness` or `now > expires_at_s`.

`to_kernel_handle` calls `ObservationEvidence::new(sensor_id, calibration_hash, capture_s, receive_s, digest, transform_epoch, quality, ood, expires_at_s)`.

- [ ] **Step 4: Run tests**

Run: `cargo test -p realityos-semantics --lib`

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add crates/semantics/src/sensor.rs crates/semantics/src/observation.rs crates/semantics/src/lib.rs
git commit -m "Add sensor observations that cite kernel evidence without naked arrays."
```

---

### Task 4: TransformGraph + WorldState v1

**Files:**
- Create: `crates/semantics/src/transform.rs`
- Create: `crates/semantics/src/world.rs`
- Modify: `crates/semantics/src/lib.rs`

**Interfaces:**
- Consumes: `Provenanced`
- Produces: `Se3 { xyz: [f64; 3], quat_wxyz: [f64; 4] }`; `TransformEdge { parent, child, pose: Provenanced<Se3>, timestamp_s, calibration_epoch, source }`; `TransformGraph::insert` returns `Err(TransformError::EpochMismatch)` if an existing edge’s epoch differs; `WorldState { transform_epoch, as_of_s, target_frame, target_xyz: Provenanced<[f64; 3]>, target_expires_at_s, objects: Vec<ObjectHypothesis> }`; `WorldState::target_fresh(now, freshness) -> bool`

- [ ] **Step 1: Write the failing test**

```rust
#[test]
fn refuses_to_mix_calibration_epochs() {
    let mut g = TransformGraph::new("e0");
    g.insert(TransformEdge::identity("world", "base", "e0", 1.0)).unwrap();
    let err = g
        .insert(TransformEdge::identity("base", "ee", "e1", 1.0))
        .unwrap_err();
    assert_eq!(err, TransformError::EpochMismatch);
}

#[test]
fn missing_target_is_unknown_not_origin() {
    let w = WorldState::empty("e0", 1.0);
    assert!(w.target_xyz.value.is_none());
    assert_eq!(w.target_xyz.provenance, Provenance::Unknown);
    assert!(!w.target_fresh(1.0, 0.25));
}
```

- [ ] **Step 2: Run to verify fail**

Run: `cargo test -p realityos-semantics --lib transform -- --nocapture`

Expected: FAIL compile.

- [ ] **Step 3: Implement**

`WorldState::empty` sets `target_xyz = Provenanced::unknown("world.target", as_of_s)`, `target_expires_at_s = as_of_s` (already expired unless later set). `with_target(frame, xyz, expires, epoch, now)` sets `ModelDeclared` or `UserDeclared` as the caller specifies (Foundation uses `UserDeclared` for the goal).

`target_fresh`: `value.is_some()` AND `now <= target_expires_at_s` AND `now - as_of_s <= freshness` AND all finite.

- [ ] **Step 4: Run tests**

Run: `cargo test -p realityos-semantics --lib`

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add crates/semantics/src/transform.rs crates/semantics/src/world.rs crates/semantics/src/lib.rs
git commit -m "Add transform-epoch isolation and WorldState v1 with unknown targets."
```

---

### Task 5: CapabilityGraph derivation

**Files:**
- Create: `crates/semantics/src/capability.rs`
- Modify: `crates/semantics/src/lib.rs`

**Interfaces:**
- Consumes: `EmbodimentModel`, `Actuator.control_mode`, `Joint.kind`, `BaseKind`, grippers
- Produces: `CapName` (`JointPositionControl`, `JointVelocityControl`, `JointEffortControl`, `CartesianPositionControl`, `FixedBaseManipulation`, `MobileBase`, `FloatingBase`, `Grasping`, `ParallelGripper`); `CapStatus` (`Proven, Supported, PartiallySupported, Unverified, NotApplicable, Unsupported`); `CapNode { name, status, evidence: Vec<String>, confidence: Option<f64>, dependencies: Vec<CapName>, unsupported_reason: Option<String> }`; `CapabilityGraph`; `pub fn derive_capabilities(model: &EmbodimentModel, qualify_ok: Option<bool>) -> CapabilityGraph`

Derivation rules (no `robot_id`):

1. `JointPositionControl`: `Supported` if any actuator `control_mode == "position"` targeting a 1-DoF `Hinge` or `Slide`. Else `Unsupported`.
2. If `qualify_ok == Some(true)` and rule 1, promote that node to `Proven` with evidence `"sim_qualify"`. If `qualify_ok == Some(false)`, keep `Supported` (do not invent Proven). If `None`, stay `Supported` (unverified qualify).
3. `JointVelocityControl`: `Supported` if any actuator mode is `"velocity"` or `"motor"` on 1-DoF joint; else `Unsupported`.
4. `JointEffortControl`: `Supported` if mode `"motor"`/`"effort"` AND `effort_max.value.is_some()`; if such actuator exists but `effort_max.value.is_none()` → `Unverified` reason `"effort_bound_unknown"`; else `Unsupported`.
5. `CartesianPositionControl`: `PartiallySupported` if EE exists AND `JointPositionControl` is Supported/Proven; `Unsupported` if no EE; `NotApplicable` never for a fixed arm with EE.
6. `FixedBaseManipulation`: `Supported` if `base == Fixed` and ≥1 EE; `NotApplicable` if Floating or Mobile.
7. `FloatingBase`: `Supported` if `base == Floating` or any `JointKind::Free`; else `NotApplicable`.
8. `MobileBase`: `Supported` if `base == Mobile`; else `NotApplicable`.
9. `Grasping` / `ParallelGripper`: `Supported` if a gripper has an actuator and opening range `value.is_some()`; else `Unsupported`. Never `Proven` in Foundation (`qualify_ok` must not promote these).

- [ ] **Step 1: Write the failing test**

```rust
#[test]
fn fixed_position_arm_gets_cartesian_partial_not_by_robot_name() {
    let m = synth_fixed_position_arm();
    let g = derive_capabilities(&m, None);
    assert_eq!(g.get(CapName::JointPositionControl).status, CapStatus::Supported);
    assert_eq!(g.get(CapName::CartesianPositionControl).status, CapStatus::PartiallySupported);
    assert_eq!(g.get(CapName::FloatingBase).status, CapStatus::NotApplicable);
    assert_eq!(g.get(CapName::Grasping).status, CapStatus::Unsupported);
}

#[test]
fn motor_without_effort_bound_stays_unverified() {
    let m = synth_motor_no_effort();
    let g = derive_capabilities(&m, None);
    let n = g.get(CapName::JointEffortControl);
    assert_eq!(n.status, CapStatus::Unverified);
    assert_eq!(n.unsupported_reason.as_deref(), Some("effort_bound_unknown"));
}

#[test]
fn derive_does_not_read_robot_id() {
    let mut a = synth_fixed_position_arm();
    let mut b = a.clone();
    a.robot_id = "alice".into();
    b.robot_id = "bob".into();
    assert_eq!(derive_capabilities(&a, None), derive_capabilities(&b, None));
}
```

`synth_fixed_position_arm` / `synth_motor_no_effort` are test helpers in the same file building `EmbodimentModel` without product names.

- [ ] **Step 2: Run to verify fail**

Run: `cargo test -p realityos-semantics --lib capability -- --nocapture`

Expected: FAIL compile.

- [ ] **Step 3: Implement `derive_capabilities` exactly as the nine rules above**

`CapabilityGraph::get` panics only in tests; production uses `get_opt`. Implement `get` for tests via `expect`.

- [ ] **Step 4: Run tests**

Run: `cargo test -p realityos-semantics --lib`

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add crates/semantics/src/capability.rs crates/semantics/src/lib.rs
git commit -m "Derive CapabilityGraph from actuators and topology, not robot names."
```

---

### Task 6: SkillContract IR + SkillRefuse

**Files:**
- Create: `crates/semantics/src/skill.rs`
- Modify: `crates/semantics/src/lib.rs`

**Interfaces:**
- Consumes: `CapName`
- Produces: `SkillName` enum with exactly `Observe, LookAt, Reach, Grasp, Release, Push, Pull, MoveBase, Hold, Place, Press, Turn, Insert, Retract, VerifyState`. `SkillName::parse(raw: &str) -> Result<Self, SkillRefuse>` errors `NotInIr` on unknown strings. `SkillContract` struct. `SkillContract::reach() -> Self` is the only fully populated Foundation contract. `SkillContract::is_qualified(&self) -> bool` true only for `Reach`. `SkillRefuse` enum: `Probe, Refuse, Unreachable, Unsupported, NotInIr, WrongModelHash, StaleEvidence, EpochMismatch, MissingActuator, MissingTarget` with `writes_allowed(&self) -> bool` always `false`.

- [ ] **Step 1: Write the failing test**

```rust
#[test]
fn llm_string_is_not_a_skill() {
    assert!(matches!(SkillName::parse("do_a_flip"), Err(SkillRefuse::NotInIr)));
}

#[test]
fn only_reach_is_qualified() {
    assert!(SkillContract::reach().is_qualified());
    assert!(!SkillContract::named(SkillName::Grasp).is_qualified());
}

#[test]
fn refuse_never_authorizes_writes() {
    for r in [
        SkillRefuse::Probe,
        SkillRefuse::Refuse,
        SkillRefuse::Unreachable,
        SkillRefuse::Unsupported,
        SkillRefuse::WrongModelHash,
        SkillRefuse::StaleEvidence,
        SkillRefuse::EpochMismatch,
        SkillRefuse::MissingActuator,
        SkillRefuse::MissingTarget,
        SkillRefuse::NotInIr,
    ] {
        assert!(!r.writes_allowed());
    }
}
```

- [ ] **Step 2: Run to verify fail**

Run: `cargo test -p realityos-semantics --lib skill -- --nocapture`

Expected: FAIL compile.

- [ ] **Step 3: Implement**

`SkillContract::reach()`:

- `id = "skill.reach"`
- `name = SkillName::Reach`
- `required = [CapName::CartesianPositionControl]` (compiler also accepts the JointPosition+EE alternative; the contract lists Cartesian as the semantic requirement)
- `required_world = ["target_xyz", "transform_epoch"]`
- `success_evidence = ["privileged_ee_within_radius"]`
- `failure_evidence = ["UNREACHABLE", "STALE_OBJECT", "WRONG_ROBOT_HASH", "MISSING_ACTUATOR", "STALE_EVIDENCE"]`
- `authority_ceiling = "allow"`
- `exploration_allowance = false`

`SkillContract::named` fills empty required lists and `is_qualified == false`.

- [ ] **Step 4: Run tests**

Run: `cargo test -p realityos-semantics --lib`

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add crates/semantics/src/skill.rs crates/semantics/src/lib.rs
git commit -m "Define SkillContract IR and qualify only REACH."
```

---

### Task 7: ControlAdapter + chain IK (no MuJoCo)

**Files:**
- Create: `crates/semantics/src/adapter.rs`
- Modify: `crates/semantics/src/lib.rs`

**Interfaces:**
- Consumes: `EmbodimentModel`, `CapabilityGraph`, `WorldState`, `ObservationFrame`, `SkillContract`
- Produces:

```rust
pub struct CompiledCtrl {
    pub action: Vec<f64>,          // joint position setpoints, actuator order
    pub control_mode: String,      // "position"
    pub adapter_id: String,        // "chain_ik_position_pd"
    pub adapter_version: String,   // "1"
}

pub struct AdapterContract {
    pub input: &'static str,       // "ee_xyz"
    pub output_mode: &'static str, // "position"
    pub required_caps: &'static [CapName],
    pub stop: &'static str,        // "hold_last_position"
}

pub trait ControlAdapter {
    fn advertise(&self) -> AdapterContract;
    fn compile(
        &self,
        skill: &SkillContract,
        model: &EmbodimentModel,
        caps: &CapabilityGraph,
        world: &WorldState,
        obs: &ObservationFrame,
    ) -> Result<CompiledCtrl, SkillRefuse>;
}

pub struct ChainIkPositionPdAdapter;

pub(crate) fn synth_planar_two_link() -> EmbodimentModel;
```

Selection: `ChainIkPositionPdAdapter` is usable when `JointPositionControl` is Supported/Proven/PartiallySupported AND `ee_joint_chain` is `Some` and non-empty. It does **not** read `robot_id`.

IK: iterative damped Jacobian on hinge axes stored in the model (`axis.value` must be `Some`; if any EE-chain axis is `Unknown`, return `Unreachable`). Foundation unit tests use a 2-link planar synthetic chain with declared axes `[0,0,1]`.

- [ ] **Step 1: Write the failing test**

```rust
#[test]
fn adapter_selected_by_caps_not_name() {
    let m = synth_planar_two_link();
    let caps = derive_capabilities(&m, None);
    let world = WorldState::empty("e0", 1.0).with_target("ee", [0.2, 0.0, 0.0], 5.0, "e0", 1.0, Provenance::UserDeclared);
    let obs = ObservationFrame { frame_id: "f".into(), transform_epoch: "e0".into(), observations: vec![], as_of_s: 1.0 };
    let a = ChainIkPositionPdAdapter;
    let out = a.compile(&SkillContract::reach(), &m, &caps, &world, &obs).unwrap();
    assert_eq!(out.control_mode, "position");
    assert_eq!(out.action.len(), m.actuators.len());
    assert!(out.action.iter().all(|x| x.is_finite()));
}

#[test]
fn unknown_axis_is_unreachable_not_invented() {
    let mut m = synth_planar_two_link();
    m.joints[0].axis = Provenanced::unknown("bundle", 0.0);
    let caps = derive_capabilities(&m, None);
    let world = WorldState::empty("e0", 1.0).with_target("ee", [0.2, 0.0, 0.0], 5.0, "e0", 1.0, Provenance::UserDeclared);
    let obs = ObservationFrame { frame_id: "f".into(), transform_epoch: "e0".into(), observations: vec![], as_of_s: 1.0 };
    let err = ChainIkPositionPdAdapter
        .compile(&SkillContract::reach(), &m, &caps, &world, &obs)
        .unwrap_err();
    assert_eq!(err, SkillRefuse::Unreachable);
}
```

- [ ] **Step 2: Run to verify fail**

Run: `cargo test -p realityos-semantics --lib adapter -- --nocapture`

Expected: FAIL compile.

- [ ] **Step 3: Implement**

`synth_planar_two_link` in the test module: two hinge Z joints, two position actuators, EE frame `ee`, link lengths stored as `ModelFrame` translations (`Provenanced::declared`).

IK loop (max 40 iters, damp 1e-3): 2D/3D cross-product Jacobian from stored axes and link translations. Clamp setpoints to `q_min`/`q_max` when those values are `Some`; if limits are `Unknown`, still return the finite iterate (do not invent limits).

If `skill.name != Reach`, return `Unsupported`.

- [ ] **Step 4: Run tests**

Run: `cargo test -p realityos-semantics --lib`

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add crates/semantics/src/adapter.rs crates/semantics/src/lib.rs
git commit -m "Add capability-selected chain IK adapter that refuses unknown axes."
```

---

### Task 8: `compile_reach` refuse matrix

**Files:**
- Create: `crates/semantics/src/reach.rs`
- Modify: `crates/semantics/src/lib.rs`

**Interfaces:**
- Consumes: all prior types
- Produces:

```rust
pub fn compile_reach(
    model: &EmbodimentModel,
    caps: &CapabilityGraph,
    world: &WorldState,
    obs: &ObservationFrame,
    expected_model_hash: &str,
    now_s: f64,
    freshness_s: f64,
    adapter: &dyn ControlAdapter,
) -> Result<CompiledCtrl, SkillRefuse>
```

Order of checks (stop at first):

1. If `expected_model_hash != model.model_hash` → `WrongModelHash`
2. If `world.transform_epoch != obs.transform_epoch` → `EpochMismatch`
3. If `world.target_xyz.value.is_none()` → `MissingTarget` (caller maps this to Probe status)
4. If `!world.target_fresh(now_s, freshness_s)` or `obs.required_stale(now_s, freshness_s)` → `StaleEvidence`
5. Cartesian usable OR (JointPosition usable AND `ee_joint_chain` nonempty). Else if no position actuator → `MissingActuator`, else `Unsupported`
6. `adapter.compile(...)`

`usable` means status ∈ {Proven, Supported, PartiallySupported}.

- [ ] **Step 1: Write the failing test**

```rust
fn base() -> (EmbodimentModel, CapabilityGraph, WorldState, ObservationFrame) {
    let m = crate::adapter::synth_planar_two_link();
    let caps = derive_capabilities(&m, None);
    let world = WorldState::empty("e0", 1.0).with_target("ee", [0.2, 0.0, 0.0], 5.0, "e0", 1.0, Provenance::UserDeclared);
    let obs = ObservationFrame { frame_id: "f".into(), transform_epoch: "e0".into(), observations: vec![], as_of_s: 1.0 };
    (m, caps, world, obs)
}

#[test]
fn missing_target_is_probe_path() {
    let (m, caps, _, obs) = base();
    let world = WorldState::empty("e0", 1.0);
    let err = compile_reach(&m, &caps, &world, &obs, &m.model_hash, 1.0, 0.25, &ChainIkPositionPdAdapter).unwrap_err();
    assert_eq!(err, SkillRefuse::MissingTarget);
}

#[test]
fn stale_target_refuses() {
    let (m, caps, mut world, obs) = base();
    world.target_expires_at_s = 0.5;
    let err = compile_reach(&m, &caps, &world, &obs, &m.model_hash, 1.0, 0.25, &ChainIkPositionPdAdapter).unwrap_err();
    assert_eq!(err, SkillRefuse::StaleEvidence);
}

#[test]
fn wrong_hash_refuses() {
    let (m, caps, world, obs) = base();
    let err = compile_reach(&m, &caps, &world, &obs, "other", 1.0, 0.25, &ChainIkPositionPdAdapter).unwrap_err();
    assert_eq!(err, SkillRefuse::WrongModelHash);
}

#[test]
fn epoch_mismatch_refuses() {
    let (m, caps, mut world, obs) = base();
    world.transform_epoch = "e1".into();
    let err = compile_reach(&m, &caps, &world, &obs, &m.model_hash, 1.0, 0.25, &ChainIkPositionPdAdapter).unwrap_err();
    assert_eq!(err, SkillRefuse::EpochMismatch);
}

#[test]
fn missing_actuator_refuses() {
    let (mut m, _, world, obs) = base();
    m.actuators.clear();
    let caps = derive_capabilities(&m, None);
    let err = compile_reach(&m, &caps, &world, &obs, &m.model_hash, 1.0, 0.25, &ChainIkPositionPdAdapter).unwrap_err();
    assert_eq!(err, SkillRefuse::MissingActuator);
}
```

Make `synth_planar_two_link` `pub(crate)` in `adapter.rs` so reach tests reuse it. Do not name real robots.

- [ ] **Step 2: Run to verify fail**

Run: `cargo test -p realityos-semantics --lib reach -- --nocapture`

Expected: FAIL compile.

- [ ] **Step 3: Implement `compile_reach` in the exact check order**

- [ ] **Step 4: Run tests**

Run: `cargo test -p realityos-semantics --lib`

Expected: PASS.

- [ ] **Step 5: Add quarantine test and commit**

In `lib.rs` tests:

```rust
#[test]
fn semantics_sources_do_not_name_held_out_robot() {
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("src");
    for ent in std::fs::read_dir(root).unwrap() {
        let p = ent.unwrap().path();
        if p.extension().and_then(|e| e.to_str()) == Some("rs") {
            let t = std::fs::read_to_string(&p).unwrap();
            assert!(!t.contains("wrist_offset_arm"), "{}", p.display());
        }
    }
}
```

```bash
git add crates/semantics/src/reach.rs crates/semantics/src/adapter.rs crates/semantics/src/lib.rs
git commit -m "Compile REACH with a refuse matrix and hold out the held-out name."
```

---

### Task 9: `verify::semantics_map`

**Files:**
- Modify: `crates/verify/Cargo.toml` — add `realityos-semantics.workspace = true`
- Create: `crates/verify/src/semantics_map.rs`
- Modify: `crates/verify/src/lib.rs` — `pub mod semantics_map;`

**Interfaces:**
- Consumes: `RobotBundle`, `RobotManifest`
- Produces: `pub fn embodiment_from_manifest(bundle: &RobotBundle, manifest: &RobotManifest) -> EmbodimentModel`

Mapping:

- `robot_id` from bundle (stored, never switched on in semantics).
- hashes from manifest/bundle.
- `calibration_epoch` = `manifest.model_hash` (model identity is the epoch until a real cal exists).
- joints: `kind` from `joint_type` string; limits `declared` if `limited`, else `unknown`; axis `unknown` unless later filled from inspect JSON (if inspect lacks axis, stay unknown — then REACH IK in semantics will Unreachable; MuJoCo adapter in Task 11 supplies setpoints instead).
- bodies: mass `simulator_derived` if `mass > 0`, else `unknown`.
- actuators: `control_mode` from `actuator_type` (`position` / `motor`); force range `declared` if `Some`, else `unknown`.
- EE from `bundle.manifest.end_effectors` + `manifest.derived.end_effector_joint_chains`.
- grippers from YAML.
- `metal = false`.
- diagnostics from `lost_features` and missing inertia.

- [ ] **Step 1: Write the failing test** in `semantics_map.rs`

```rust
#[test]
fn planar_arm_maps_without_inventing_effort_when_missing() {
    let b = crate::bundle::RobotBundle::load(crate::corpus::robot_dir("planar_arm")).unwrap();
    let (_inst, man) = crate::runner::load_and_normalize(&b, &[], 0).unwrap_or_else(|_| {
        // allow skip if no mujoco: load yaml-only partial
        panic!("need mujoco or fixture inspect");
    });
    let m = embodiment_from_manifest(&b, &man);
    assert!(!m.metal);
    assert_eq!(m.robot_id, "planar_arm");
    assert!(m.end_effectors.iter().any(|e| e.name == "ee"));
    assert!(m.bodies.iter().any(|b| b.mass_kg.provenance != realityos_semantics::provenance::Provenance::HardwareMeasured));
}
```

If `ensure_mujoco_or_skip()` is false, the test must `return` (skip), not fake a model.

- [ ] **Step 2: Run to verify fail**

Run: `cargo test -p realityos-verify semantics_map -- --nocapture`

Expected: FAIL compile (`semantics_map` missing).

- [ ] **Step 3: Implement mapper**

Do not write axis unit vectors unless `inspect` JSON actually contains them. Prefer unknown over guessed `[0,0,1]`.

- [ ] **Step 4: Run tests**

Run: `cargo test -p realityos-verify semantics_map`

Expected: PASS or skip if MuJoCo absent locally. CI `verify-mujoco` will run it.

- [ ] **Step 5: Commit**

```bash
git add crates/verify/Cargo.toml crates/verify/src/lib.rs crates/verify/src/semantics_map.rs Cargo.toml
git commit -m "Map RobotManifest into EmbodimentModel v2 without inventing axes."
```

---

### Task 10: Development robot `spatial_arm4`

**Files:**
- Create: `robots/bundles/spatial_arm4/model.xml`
- Create: `robots/bundles/spatial_arm4/robot.yaml`
- Create: `robots/bundles/spatial_arm4/LICENSE`
- Test: add `#[test] fn spatial_arm4_ingests_as_fixed_position_arm` in `semantics_map.rs` (allowed: this is a development robot)

**Do not** add this id to `corpus.rs` `milestone_robots` / `corpus_ids`.

- [ ] **Step 1: Write the failing test**

```rust
#[test]
fn spatial_arm4_is_not_a_planar_clone() {
    if !crate::mujoco_exec::ensure_mujoco_or_skip() { return; }
    let b = RobotBundle::load(crate::corpus::bundled_robots_root().join("spatial_arm4")).unwrap();
    let (_i, man) = crate::runner::load_and_normalize(&b, &[], 0).unwrap();
    assert_eq!(man.nu, 4);
    let axes: Vec<String> = man.joints.iter().map(|j| j.name.clone()).collect();
    assert_eq!(axes.len(), 4);
    let m = embodiment_from_manifest(&b, &man);
    let g = realityos_semantics::capability::derive_capabilities(&m, None);
    assert_eq!(g.get(realityos_semantics::capability::CapName::FixedBaseManipulation).status,
               realityos_semantics::capability::CapStatus::Supported);
}
```

- [ ] **Step 2: Run to verify fail**

Run: `cargo test -p realityos-verify spatial_arm4_is_not_a_planar_clone -- --nocapture`

Expected: FAIL (bundle missing).

- [ ] **Step 3: Author original MJCF**

`model.xml`: fixed base; joints `j1` hinge axis `0 0 1`, `j2` hinge `0 1 0`, `j3` hinge `0 1 0`, `j4` hinge `0 0 1`; link lengths 0.18 / 0.16 / 0.12 / 0.08; site `ee`; four `<position>` actuators; MIT comment header; no vendor names.

`robot.yaml`: `robot_id: spatial_arm4`, `expected_base_type: fixed`, EE site `ee`, license MIT, original fixture notes.

- [ ] **Step 4: Run test**

Run: `cargo test -p realityos-verify spatial_arm4_is_not_a_planar_clone`

Expected: PASS when MuJoCo present.

- [ ] **Step 5: Commit**

```bash
git add robots/bundles/spatial_arm4 crates/verify/src/semantics_map.rs
git commit -m "Add original spatial_arm4 development fixture with mixed hinge axes."
```

---

### Task 11: Held-out bundle + verify-only loader

**Files:**
- Create: `robots/bundles/wrist_offset_arm/model.xml`
- Create: `robots/bundles/wrist_offset_arm/robot.yaml`
- Create: `robots/bundles/wrist_offset_arm/LICENSE`
- Create: `crates/verify/src/held_out.rs` — `pub fn held_out_bundle() -> PathBuf` returning `bundled_robots_root().join("wrist_offset_arm")` (the string may live only in verify)
- Modify: `crates/verify/src/lib.rs`

**Files that must stay clean:** every `crates/semantics/src/*.rs`

- [ ] **Step 1: Write the failing test** (in verify)

```rust
#[test]
fn held_out_bundle_exists_and_semantics_crate_does_not_name_it() {
    let p = held_out_bundle();
    assert!(p.join("robot.yaml").exists());
    let sem = Path::new(env!("CARGO_MANIFEST_DIR")).join("../semantics/src");
    for ent in std::fs::read_dir(sem).unwrap() {
        let path = ent.unwrap().path();
        if path.extension().and_then(|e| e.to_str()) == Some("rs") {
            let t = std::fs::read_to_string(&path).unwrap();
            assert!(!t.contains("wrist_offset_arm"), "{}", path.display());
        }
    }
}
```

- [ ] **Step 2: Run to verify fail**

Expected: FAIL (directory missing).

- [ ] **Step 3: Author 5-DoF original MJCF**

Joints: `j1` hinge `0 0 1`; `j2` hinge `0 1 0`; `j3` hinge `0 1 0`; `j4` hinge `1 0 0`; `j5` hinge `0 0 1`. Insert a 0.04 m Y-offset body between `j4` and `j5` so wrist axes do not intersect. Five position actuators. Site `ee`. No vendor names.

- [ ] **Step 4: Run tests**

Run: `cargo test -p realityos-verify held_out -- --nocapture` and `cargo test -p realityos-semantics semantics_sources_do_not_name_held_out_robot`

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add robots/bundles/wrist_offset_arm crates/verify/src/held_out.rs crates/verify/src/lib.rs
git commit -m "Add held-out wrist-offset arm fixture isolated from semantics sources."
```

---

### Task 12: Foundation REACH loop through SimAuthority

**Files:**
- Create: `crates/verify/src/reach_foundation.rs`
- Modify: `crates/verify/src/lib.rs`

**Interfaces:**
- Consumes: `compile_reach`, `embodiment_from_manifest`, `SimAuthority`, existing `PdPolicy` / `ActionProposal`, `TaskSpec::Reach`, privileged `VerifierTruth::ee_pos`
- Produces:

```rust
pub struct FoundationReachReport {
    pub robot_id: String,
    pub model_hash: String,
    pub skill: String,                 // "REACH"
    pub adapter_id: String,
    pub adaptation: String,            // "CONFIGURED"
    pub metal: bool,                   // false
    pub evidence_status: String,       // SIMULATION_ONLY
    pub task_success: bool,
    pub skill_refuse: Option<String>,
    pub ctrl_writes: u64,
    pub replay_write_delta: Option<i64>,
    pub authority_decisions: Vec<String>,
    pub seed: u64,
}

pub fn run_foundation_reach(
    bundle: &RobotBundle,
    target: [f64; 3],
    radius: f64,
    now_s: f64,
    freshness_s: f64,
    expected_hash: Option<&str>,
    force_stale: bool,
    replay: bool,
) -> Result<FoundationReachReport, String>

pub fn run_foundation_reach_missing_target(
    bundle: &RobotBundle,
) -> Result<FoundationReachReport, String>
```

`run_foundation_reach_missing_target` builds `WorldState::empty` and calls `compile_reach` only (no `decide_and_maybe_write`).

Algorithm:

1. `load_and_normalize` + `embodiment_from_manifest` + `derive_capabilities`.
2. Build `WorldState` / `ObservationFrame` (joint encoder obs from qpos digest; expires `now+freshness` unless `force_stale`).
3. `hash = expected_hash.unwrap_or(model.model_hash)`.
4. `compile_reach(...)`.
5. On `Err(e)`: return report with `task_success=false`, `skill_refuse=Some(...)`, `ctrl_writes=0`, `metal=false`. Map `MissingTarget` → refuse string `"PROBE"`.
6. On `Ok(ctrl)`: wrap into `ActionProposal` (copy fields from episode ids), `SimAuthority::decide_and_maybe_write` with `force_verb: Some("reach")`.
7. Step MuJoCo with authorized ctrl for a short horizon (reuse runner step pattern; 0.5 s at control_hz 50).
8. Privileged success: EE within `radius`. Adapter success is ignored.
9. If `replay`: call `decide_and_maybe_write` again with the same command id; set `replay_write_delta`.

Do not implement a second plant write path.

- [ ] **Step 1: Write the failing tests**

```rust
#[test]
fn planar_and_spatial_use_the_same_reach_contract() {
    if !ensure_mujoco_or_skip() { return; }
    for id in ["planar_arm", "spatial_arm4"] {
        let b = RobotBundle::load(crate::corpus::bundled_robots_root().join(id)).unwrap();
        let r = run_foundation_reach(&b, [0.22, 0.0, 0.12], 0.20, 10.0, 0.25, None, false, false).unwrap();
        assert_eq!(r.skill, "REACH");
        assert_eq!(r.adapter_id, "chain_ik_position_pd");
        assert!(!r.metal);
        assert_eq!(r.evidence_status, SIMULATION_ONLY);
        assert_eq!(r.adaptation, "CONFIGURED");
        assert!(r.skill_refuse.is_none(), "{id} {:?}", r.skill_refuse);
        assert!(r.ctrl_writes > 0, "{id}");
        assert!(r.task_success, "{id} must reach under privileged verifier");
    }
}

#[test]
fn missing_target_probes_without_writes() {
    if !ensure_mujoco_or_skip() { return; }
    let b = RobotBundle::load(crate::corpus::robot_dir("planar_arm")).unwrap();
    // run_foundation_reach_without_target helper or target None via a flag
    let r = run_foundation_reach_missing_target(&b).unwrap();
    assert_eq!(r.skill_refuse.as_deref(), Some("PROBE"));
    assert_eq!(r.ctrl_writes, 0);
}

#[test]
fn replay_does_not_add_policy_writes() {
    if !ensure_mujoco_or_skip() { return; }
    let b = RobotBundle::load(crate::corpus::robot_dir("planar_arm")).unwrap();
    let r = run_foundation_reach(&b, [0.22, 0.0, 0.12], 0.20, 10.0, 0.25, None, false, true).unwrap();
    assert_eq!(r.replay_write_delta, Some(0));
}
```

- [ ] **Step 2: Run to verify fail**

Expected: FAIL compile.

- [ ] **Step 3: Implement `run_foundation_reach` using existing `SimAuthority` only**

For MuJoCo IK: if `ChainIkPositionPdAdapter` returns `Unreachable` because axes were unknown after mapping, the verify loop may fill axes from the compiled mjModel inspect **as `SimulatorDerived`** (recorded) and retry compile once. Do not fill as `ModelDeclared` if the XML axis was not parsed. Do not fill effort.

If still unreachable, report `UNREACHABLE` and 0 writes (honest failure).

- [ ] **Step 4: Run tests**

Run: `cargo test -p realityos-verify reach_foundation -- --nocapture`

Expected: PASS with MuJoCo.

- [ ] **Step 5: Commit**

```bash
git add crates/verify/src/reach_foundation.rs crates/verify/src/lib.rs
git commit -m "Execute REACH through SkillContract and SimAuthority, not a side plant."
```

---

### Task 13: Adversarial + held-out first score + CLI + report

**Files:**
- Modify: `crates/verify/src/reach_foundation.rs` (more tests)
- Modify: `crates/verify/src/bin/realityos-verify.rs` add `foundation-reach`
- Create: writer `crates/verify/src/foundation_report.rs` that writes `verify-out/foundation_report.json` + `.md`

**Interfaces:**
- `foundation-reach` loads planar_arm, spatial_arm4, then `held_out_bundle()`. Records first held-out result unconditionally. `adaptation: CONFIGURED`. Prints counts.

CLI tests / integration:

```rust
#[test]
fn wrong_hash_zero_writes() { ... expected_hash: Some("deadbeef") ... }

#[test]
fn stale_zero_writes() { force_stale: true }

#[test]
fn held_out_first_evaluation_is_recorded() {
    if !ensure_mujoco_or_skip() { return; }
    let b = RobotBundle::load(held_out_bundle()).unwrap();
    let r = run_foundation_reach(&b, [0.20, 0.0, 0.12], 0.25, 10.0, 0.25, None, false, false).unwrap();
    assert_eq!(r.adaptation, "CONFIGURED");
    assert!(!r.metal);
    // do not assert success; assert the report fields exist
    assert!(!r.model_hash.is_empty());
}
```

`foundation_report.json` fields: `software_sha` (env `GITHUB_SHA` or `"local"`), robots[], capability graphs, skill id, adapter id, scenario counts, successes, refusals, probes, write counts, replay deltas, `metal: false`, `not_implemented: [ ... spec §11 list ... ]`.

Do not change `milestone` job arithmetic.

- [ ] **Step 1: Write failing tests** for wrong hash, stale, held-out record

- [ ] **Step 2: Run to verify fail**

- [ ] **Step 3: Implement CLI + report**

`realityos-verify.rs` match arm:

```rust
"foundation-reach" => foundation_reach(),
```

- [ ] **Step 4: Run**

Run: `cargo test -p realityos-verify --all-targets`  
Run: `cargo test -p realityos-semantics --lib`  
Run: `cargo clippy -p realityos-semantics -p realityos-verify --all-targets -- -D warnings`

Expected: PASS. Then locally (if MuJoCo): `cargo run -p realityos-verify -- foundation-reach`

- [ ] **Step 5: Commit**

```bash
git add crates/verify/src/reach_foundation.rs crates/verify/src/foundation_report.rs crates/verify/src/bin/realityos-verify.rs crates/verify/src/lib.rs
git commit -m "Record Foundation REACH evidence including the held-out first score."
```

---

### Task 14: Stop condition and output note

**Files:**
- Create: `docs/superpowers/specs/2026-09-10-foundation-output.md` only after Task 13 has numbers from a real run (CI or local MuJoCo). Fill every output-package bullet from the spec §10 with measured values, not estimates.

- [ ] **Step 1: Write the document from `verify-out/foundation_report.json`**

If held-out failed, write the failure and the counterexample. Do not retune `wrist_offset_arm` or `compile_reach` to erase the first score.

- [ ] **Step 2: Confirm CI smoke still `3 * 10 * 2`**

Read `.github/workflows/authority.yml` and `corpus_ids()`. If either gained extra robots, revert that part.

- [ ] **Step 3: Commit the output note**

```bash
git add docs/superpowers/specs/2026-09-10-foundation-output.md
git commit -m "Record Foundation measurements and explicitly list what is not implemented."
```

- [ ] **Step 4: Stop**

Do not start Phase B.

---

## Self-review

**1. Spec coverage**

| Spec section | Task |
|---|---|
| §0 freeze / no robot_id | Global + Task 5/8 quarantine |
| §2 crate split | Task 1 |
| §3 provenance | Task 1 |
| §4.1 EmbodimentModel | Task 2 |
| §4.2–4.4 sensors/obs | Task 3 |
| §4.5–4.6 transform/world | Task 4 |
| §4.7 / §5 capabilities | Task 5 |
| §4.8 / IR | Task 6 |
| §4.9 adapter | Task 7 |
| §6 REACH compile | Task 8 |
| mapper in verify | Task 9 |
| corpus spatial_arm4 | Task 10 |
| held-out + quarantine | Task 11 |
| end-to-end + SimAuthority | Task 12 |
| adversarial, report, CLI | Task 13 |
| §9–11 output / stop | Task 14 |
| CI 3×10×2 | Global + Task 14 |
| Camera / VLA / planner | Explicitly not tasked |

**2. Placeholder scan:** no TBD. `CompiledCtrl` is the locked DTO (spec allowed verify-boundary mapping).

**3. Type consistency:** `Provenanced`, `CapName`, `CapStatus`, `SkillRefuse`, `CompiledCtrl`, `compile_reach`, `embodiment_from_manifest`, `run_foundation_reach`, `FoundationReachReport` used under those names in later tasks.

**Clarification vs spec §4.9:** adapter returns `CompiledCtrl`, not `verify::ActionProposal`, so semantics does not depend on verify. Verify wraps `CompiledCtrl` into `ActionProposal` in Task 12.
