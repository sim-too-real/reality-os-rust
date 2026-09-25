# Physical Geometry Input Truth Boundary Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Ensure malformed support planes and object/obstacle poses cannot become plausible contact or collision geometry.

**Architecture:** Validate support vectors and poses at the reusable semantics boundary. Preserve invalid-input reasons through manifold generation, maneuver evaluation, and collision admissibility; valid explicit geometry retains existing behavior.

**Tech Stack:** Rust 2021 workspace; `realityos-semantics`; local unit tests; `cargo test`.

**Spec:** `docs/superpowers/specs/2026-09-26-physical-input-truth-boundary-design.md`

## Global Constraints

- `UNKNOWN`, `INVALID`, and `MISSING` must remain unknown or produce an explicit typed refusal; they must not become plausible geometry and then yield `FEASIBLE`.
- Do not repair bad physical inputs with identity transforms, world-up normals, zero residuals, or a fixture parameter.
- Keep valid explicit values unchanged, including valid declared `+Z` support fixtures.
- Keep collision refusal distinct from authority authorization; do not change command minting, online write, governor, metal, or hardware paths.
- Do not rewrite missing joint `q`/origin handling or IK search bounds in this plan.

## Review Focus

- Zero, NaN, or infinite support normal and non-finite support origin: typed invalid-support result, never contact/collision-free. Pin in Task 1 and Task 2.
- Finite support direction whose naïve norm overflows: normalize safely or reject; never produce `[0,0,0]`. Pin in Task 1.
- Zero/non-finite object quaternion or non-finite translation: typed pose failure, not translation/identity substitution. Pin in Task 1 and Task 2.
- Malformed obstacle pose among valid obstacles: invalidate the whole collision scene and refuse every otherwise-feasible phase. Pin in Task 2.
- Valid non-unit finite quaternion and valid explicit `+Z` normal: preserve normalized pose and existing contact result. Pin in Task 1 and Task 2.

---

### Task 1: Checked support and object inputs in contact geometry

**Files:**
- Modify: `crates/semantics/src/transform.rs`
- Modify: `crates/semantics/src/contact_manifold.rs`
- Modify: `crates/semantics/src/contact_maneuver.rs`
- Modify: `crates/semantics/src/physical_interaction.rs`
- Test: unit tests in those same files

**Interfaces:**
- Consumes: `Se3::try_new`, `normalize3`, `ManifoldReject`, `ContactInfeasible`.
- Produces: `box_push_face_manifold(object_center: [f64; 3], half_extents: [f64; 3], push: [f64; 3], support_normal: [f64; 3], face_gap: f64) -> Result<Option<BoxFaceManifold>, ManifoldReject>`; `box_push_face_manifold_posed(object_pose: Se3, half_extents: [f64; 3], push_world: [f64; 3], support_normal: [f64; 3], face_gap: f64) -> Result<Option<BoxFaceManifold>, ManifoldReject>`; `box_vertical_face_manifolds_posed(object_pose: Se3, half_extents: [f64; 3], support_normal: [f64; 3], face_gap: f64) -> Result<Vec<(String, BoxFaceManifold)>, ManifoldReject>`; `generate_planar_push_candidates(object_id: &str, object_pose: Se3, half_extents: [f64; 3], support_normal: [f64; 3], face_gap: f64, stroke_m: f64) -> Result<Vec<PhysicalInteractionCandidate>, ManifoldReject>`; `BoxObject::pose(self) -> Result<Se3, TransformError>`; and `ContactInfeasible::{InvalidSupportPlane, InvalidObjectPose}`.

- [ ] **Step 1: Add failing geometry-boundary tests.** In `contact_manifold` cover zero/NaN/infinite and norm-overflow support normals, non-finite support origin, invalid posed `Se3`, plus valid non-unit quaternion and explicit `+Z` controls. In `contact_maneuver` and `physical_interaction`, assert typed refusal/error rather than positive clearance, contact, or candidates. Name the tests `invalid_support_inputs_are_rejected`, `invalid_pose_is_not_used_for_contact`, and `valid_explicit_geometry_is_preserved`.
- [ ] **Step 2: Run the focused tests and confirm the current fallbacks violate them.** Run `cargo test -p realityos-semantics invalid_support_inputs_are_rejected` and `cargo test -p realityos-semantics invalid_pose_is_not_used_for_contact`. Expected: failures show invalid inputs currently produce a normal, pose, clearance, or candidate result instead of the asserted typed error.
- [ ] **Step 3: Implement checked normalization and fallible manifold/contact APIs.** Make quaternion/vector normalization scale-safe (or reject an invalid normalized result); add `ManifoldReject::InvalidSupportPlane` and `ManifoldReject::InvalidObjectPose`; validate finite support origin and `Se3` before transformation; remove the `+Z` tangent-basis, vertical-face, and signed-height recoveries; propagate errors through `physical_interaction` and `evaluate_sampled_push`; remove `BoxObject::pose` translation/identity fallback and add the two `ContactInfeasible` variants with stable `as_str()` values.
- [ ] **Step 4: Run the full semantics crate tests.** Run `cargo test -p realityos-semantics`. Expected: exit 0; all existing valid-geometry tests and the new invalid-input regressions pass.
- [ ] **Step 5: Commit Task 1.** Stage only the four semantics source files and tests; commit as `fix(semantics): reject invalid contact geometry inputs`.

### Task 2: Fail-closed collision-scene construction

**Files:**
- Modify: `crates/semantics/src/contact_collision.rs`
- Test: `crates/semantics/src/contact_collision.rs` unit tests

**Interfaces:**
- Consumes: checked `Se3` and support validation from Task 1.
- Produces: `CollisionWorldError::{InvalidObjectPose, InvalidObstaclePose { name: String }, InvalidSupportPlane}`; `scene_from_collision_world(model: &EmbodimentModel, ee: &str, world: &CollisionWorld) -> Result<CollisionScene, CollisionWorldError>`; `forbidden_class_on_interpolation(model: &EmbodimentModel, ee: &str, names: &[String], qa: &[f64], qb: &[f64], world: &CollisionWorld, kind: TransitionKind) -> Result<Option<(&'static str, ContactEvidenceClass)>, CollisionWorldError>`; stable block reason `INVALID_COLLISION_WORLD` recognized by `is_collision_block_reason`.

- [ ] **Step 1: Add failing collision tests.** Assert invalid object pose, invalid obstacle pose, and invalid support plane return typed errors; assert `apply_collision_admissibility` changes every still-feasible phase to `Refused` with `INVALID_COLLISION_WORLD`; assert a malformed obstacle is not silently dropped. Name the tests `invalid_collision_world_refuses_feasible_phases` and `malformed_obstacle_invalidates_scene`.
- [ ] **Step 2: Run the focused tests and confirm failure.** Run `cargo test -p realityos-semantics invalid_collision_world_refuses_feasible_phases` and `cargo test -p realityos-semantics malformed_obstacle_invalidates_scene`. Expected: current code builds a fallback scene and leaves at least one phase feasible.
- [ ] **Step 3: Make collision input validation fallible.** Remove object/obstacle pose repair and support-normal `+Z` fallback; validate all entries before returning the scene; collect obstacle construction as `Result`; propagate invalid support through `sphere_plane`/`forbidden_class_on_interpolation`; on scene error, refuse each still-feasible transition without overwriting a prior refusal.
- [ ] **Step 4: Run collision and full semantics tests.** Run `cargo test -p realityos-semantics contact_collision::tests`, then `cargo test -p realityos-semantics`. Expected: exit 0 for both, with valid-scene collision classifications unchanged.
- [ ] **Step 5: Commit Task 2.** Stage only `contact_collision.rs`; commit as `fix(semantics): refuse malformed collision worlds`.
