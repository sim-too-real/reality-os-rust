# Foundation output package (Phase A stop)

**Date:** 2026-09-11  
**Design spec:** [2026-09-10-physical-intelligence-foundation-design.md](./2026-09-10-physical-intelligence-foundation-design.md)  
**Branch:** `docs/foundation-design`  
**Baseline (CI-fixed main at branch start):** `1141179f254536c856f328142236632a4602078c`  
**Implementation SHA:** `da89af7d3a0de480762697f3a2e1e5f0384e37ee`  
**Evidence source:** `verify-out/foundation_report.json` (produced by `cargo run -p realityos-verify -- foundation-reach`)

---

## Exact SHA

| Field | Value |
|---|---|
| Implementation commit | `da89af7d3a0de480762697f3a2e1e5f0384e37ee` |
| Branch baseline (`1141179…` → HEAD) | `1141179f254536c856f328142236632a4602078c` |
| Report `software_sha` | `local` (`GITHUB_SHA` unset at run time) |
| Schema family | `realityos.semantics/1` |

---

## Files changed

Measured from `git diff --stat 1141179f254536c856f328142236632a4602078c...da89af7d3a0de480762697f3a2e1e5f0384e37ee`:

**30 files changed, 5127 insertions(+), 26 deletions(-)**

| Area | Paths |
|---|---|
| Workspace | `Cargo.toml`, `Cargo.lock` |
| New crate `realityos-semantics` | `crates/semantics/Cargo.toml`, `src/{lib,adapter,capability,embodiment,observation,provenance,reach,sensor,skill,transform,world}.rs` |
| Verify integration | `crates/verify/Cargo.toml`, `src/{foundation_report,held_out,reach_foundation,semantics_map,lib}.rs`, `src/bin/realityos-verify.rs`, `mujoco_worker.py` |
| New robot bundles | `robots/bundles/spatial_arm4/{LICENSE,model.xml,robot.yaml}`, `robots/bundles/wrist_offset_arm/{LICENSE,model.xml,robot.yaml}` |
| Design docs | `docs/superpowers/specs/2026-09-10-physical-intelligence-foundation-design.md`, `docs/superpowers/specs/2026-09-10-physical-intelligence-foundation.md` |

Frozen crates (`kernel`, `plant`, `governor`, `session`, `metal`) and milestone corpus robots (`planar_arm`, `arm_gripper`, `cartpole`) were not edited.

---

## Architecture diagram

Use the design spec §2 diagram and crate split — no alternate architecture was introduced:

- [§2 Architecture](./2026-09-10-physical-intelligence-foundation-design.md#2-architecture)
- [§2.1 `realityos-semantics`](./2026-09-10-physical-intelligence-foundation-design.md#21-new-crate-realityos-semantics)
- [§2.2 `realityos-verify`](./2026-09-10-physical-intelligence-foundation-design.md#22-realityos-verify)

Flow executed in this milestone: `Goal / WorldState` → `SkillContract(REACH)` → `CapabilityGraph` → `ChainIkPositionPdAdapter` → `CompiledCtrl` → verify maps to `ActionProposal` → `RealityOs::decide` → `SimAuthority` → MuJoCo → privileged EE verifier.

---

## Schema definitions

Canonical definitions: design spec [§4 Schemas](./2026-09-10-physical-intelligence-foundation-design.md#4-schemas).

Rust implementations in `crates/semantics/src/`:

| Schema | Module |
|---|---|
| `EmbodimentModel` v2 | `embodiment.rs` |
| `SensorModel` | `sensor.rs` |
| `SensorObservation` / `ObservationFrame` | `observation.rs` |
| `TransformGraph` | `transform.rs` |
| `WorldState` v1 | `world.rs` |
| `CapabilityGraph` | `capability.rs` |
| `SkillContract` v1 | `skill.rs` |
| `ControlAdapter` / `CompiledCtrl` | `adapter.rs` |
| `Provenance` / `Provenanced<T>` | `provenance.rs` |
| REACH compiler | `reach.rs` |
| Schema family constant | `lib.rs` (`SCHEMA_FAMILY = "realityos.semantics/1"`) |

Ingest mapper (`RobotManifest` → `EmbodimentModel`): `crates/verify/src/semantics_map.rs`.

---

## Robot corpus and held-out identity

Per design spec [§7 Robot corpus](./2026-09-10-physical-intelligence-foundation-design.md#7-robot-corpus). REACH development and held-out bundles are **not** the milestone smoke corpus `{planar_arm, arm_gripper, cartpole}`.

| Role | `robot_id` | Model hash (measured) | Notes |
|---|---|---|---|
| Development A | `planar_arm` | `d1ae9d767848cae4d0b5259c71e30a5acc4db7a705e29d46db4f688e8329a648` | Existing 3R planar MIT fixture |
| Development B | `spatial_arm4` | `552907cb6275c21d379f879ce350d7a382a5c62ed992db5c0f01e3766a072252` | New original 4-DoF spatial arm (mixed hinge axes) |
| Held-out (first score) | `wrist_offset_arm` | `13608b356bc6fe7e0f317a1efa7afd079783e7c24e70df5b1ed060585724c6ff` | New original 5-DoF wrist-offset arm; **first score recorded below, not retuned** |
| Not REACH-dev | `arm_gripper` | (unchanged) | Later GRASP; milestone smoke only |
| Discovery only | `cartpole` | (unchanged) | REACH `NOT_APPLICABLE`; milestone smoke only |

Held-out quarantine: `wrist_offset_arm` is loaded from `crates/verify/src/held_out.rs` by path/hash only. The string `wrist_offset_arm` does not appear in `crates/semantics/src/` (enforced by `quarantine` test in `lib.rs` and `held_out_bundle_exists_and_semantics_crate_does_not_name_it` in verify).

---

## Derived CapabilityGraphs (machine-readable)

Source: `verify-out/foundation_report.json` → `robots[].capability_graph`.

All three REACH robots share the same derived statuses:

| Capability | Status (all three robots) |
|---|---|
| `joint_position_control` | `supported` |
| `joint_velocity_control` | `unsupported` |
| `joint_effort_control` | `unsupported` |
| `cartesian_position_control` | `partially_supported` (depends on `joint_position_control`) |
| `fixed_base_manipulation` | `supported` |
| `mobile_base` | `not_applicable` |
| `floating_base` | `not_applicable` |
| `grasping` | `unsupported` |
| `parallel_gripper` | `unsupported` |

No capability node reached `proven` in Foundation (no `ControlQualification` promotion path exercised for REACH).

---

## REACH SkillContract

From `SkillContract::reach()` in `crates/semantics/src/skill.rs` (design spec §4.8, §6):

| Field | Value |
|---|---|
| `id` | `skill.reach` |
| `name` | `Reach` |
| `required` | `CartesianPositionControl` |
| `required_world` | `target_xyz`, `transform_epoch` |
| `success_evidence` | `privileged_ee_within_radius` |
| `authority_ceiling` | `allow` |
| `exploration_allowance` | `false` |

Success is judged only by the privileged verifier (EE site within `radius` of target). Adapter `ok` is not success.

---

## ControlAdapters used

| Adapter ID | Version | Implementation | Selection |
|---|---|---|---|
| `chain_ik_position_pd` | `1` | `ChainIkPositionPdAdapter` in `crates/semantics/src/adapter.rs` | Capability-derived (`joint_position_control` usable + EE chain); not `robot_id` |

Output: `CompiledCtrl` with `control_mode: "position"`. Verify maps to `ActionProposal` at the boundary (`reach_foundation.rs`).

---

## Scenario counts, task success, safe completion, refusal/probe behavior

Measured campaign totals from `verify-out/foundation_report.json`:

| Metric | Value |
|---|---|
| `scenario_count` | **3** |
| `successes` | **3** |
| `refusals` | **0** |
| `probes` | **0** |
| `evidence_status` | `SIMULATION_ONLY` |
| `metal` | `false` |
| `adaptation` | `CONFIGURED` |
| `skill` | `REACH` |

Per-robot REACH results:

| Robot | Role | `task_success` | `skill_refuse` | Safe completion |
|---|---|---|---|---|
| `planar_arm` | development | `true` | `null` | Privileged EE within radius; seed `0` |
| `spatial_arm4` | development | `true` | `null` | Privileged EE within radius; seed `0` |
| `wrist_offset_arm` | `held_out_first_score` | `true` | `null` | **First held-out score** — privileged EE within radius; seed `0`; no post-hoc retune |

Refuse/probe matrix (design spec §6): missing target → `PROBE` (0 writes); stale target / wrong hash / unreachable / missing actuator / epoch mismatch → refuse variants (0 writes). Campaign runs had no refuses or probes.

---

## Authority write counts, replay write deltas

| Robot | `ctrl_writes` | `replay_write_delta` | `authority_decisions` |
|---|---|---|---|
| `planar_arm` | **1** | **0** | `Allowed:allow`, `Refused:refuse` |
| `spatial_arm4` | **1** | `null` | `Allowed:allow` |
| `wrist_offset_arm` | **1** | `null` | `Allowed:allow` |
| **Campaign total** | **3** | **`replay_deltas: [0]`** | — |

Replay test (`replay_does_not_add_policy_writes` in `reach_foundation.rs`) confirms `replay_write_delta == 0` on `planar_arm`.

---

## Counterexamples

**REACH campaign:** None. All three scenarios succeeded (`successes: 3`, no failure counterexamples in `foundation_report.json`).

**Adversarial authority-negative paths** (0-write refuses/probes; not counted in campaign `scenario_count`):

| Condition | Expected | Test |
|---|---|---|
| Missing target | `PROBE`, 0 writes | `missing_target_probes_without_writes` (`reach_foundation.rs`); `missing_target_is_probe_path` (`reach.rs`) |
| Wrong model hash | refuse, 0 writes | `wrong_hash_zero_writes` (`reach_foundation.rs`); `wrong_hash_refuses` (`reach.rs`) |
| Stale evidence | refuse, 0 writes | `stale_zero_writes` (`reach_foundation.rs`); `stale_target_refuses` (`reach.rs`) |
| Replay | 0 additional policy writes | `replay_does_not_add_policy_writes` (`reach_foundation.rs`) |

---

## Known limitations

- **Simulation only:** `metal: false`, `evidence_status: SIMULATION_ONLY` on all runs; no metal transfer or hardware evidence.
- **Single qualified skill:** Only `REACH` is compiled; all other `SkillName` IR entries return `UNSUPPORTED` with 0 writes.
- **Single adapter:** `chain_ik_position_pd` v1 only; no velocity/effort adapters, no vendor-specific controllers.
- **Cartesian capability partial:** `cartesian_position_control` is `partially_supported` (IK + position actuators, no `PROVEN` qualification promotion).
- **Adaptation honesty:** `CONFIGURED` (authored MJCF bundles); not `ZERO_SHOT` or `METAL_ADAPTED`.
- **PROBE is status-only:** Missing target returns `PROBE` status; no `LOOK_AT` / `CHANGE_VIEWPOINT` motion skills.
- **Milestone smoke corpus unchanged:** `corpus_ids()` remains `{planar_arm, arm_gripper, cartpole}` for the existing `3×10×2` matrix; REACH uses a separate development + held-out set.
- **Held-out first score only:** `wrist_offset_arm` evaluated once; score recorded even on failure (this run: success).
- **No world/sensors crates:** Schemas live in `realityos-semantics`; dedicated `world` / `sensors` workspace crates not created.
- **USD ingest:** Not a supported path (experimental disposition unchanged).

---

## NOT IMPLEMENTED (explicit)

From design spec [§11](./2026-09-10-physical-intelligence-foundation-design.md#11-not-implemented-in-foundation) and `foundation_report.json` → `not_implemented`:

1. Advanced vision / RGB-D policy
2. VLA
3. tactile
4. F/T servoing
5. behavior planner
6. PROBE motion skills (`LOOK_AT`, `CHANGE_VIEWPOINT`) beyond refuse/probe *status*
7. GRASP/PUSH/PLACE qualification
8. locomotion
9. world-estimator fusion
10. human tracking
11. first-principles merge
12. Sim2Real training
13. 100 robots
14. metal transfer
15. safety certification
16. global controllability claims
17. `world` / `sensors` crates
18. USD as a supported ingest path

**Phase B and beyond were not started.** Authority kernel, plant, governor, session, and metal paths remain frozen.

---

## CI smoke confirmation (Step 2)

Verified at implementation SHA `da89af7d3a0de480762697f3a2e1e5f0384e37ee`:

| Check | Result |
|---|---|
| `corpus_ids()` in `crates/verify/src/corpus.rs` | `["planar_arm", "arm_gripper", "cartpole"]` — **unchanged** (3 robots) |
| `authority.yml` smoke assertion | `scheduled == 3 * 10 * 2 * 1` (60 jobs) in `verify-mujoco` → `smoke matrix 3x10x2` step |
| Extra robots in corpus or workflow | **None added** — no revert required |

Foundation REACH (`foundation-reach` CLI) is a separate 3-scenario campaign on `{planar_arm, spatial_arm4, wrist_offset_arm}` and does not alter the milestone smoke matrix.
