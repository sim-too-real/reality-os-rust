# Phase B design: robot-independent manipulation semantics

**Date:** 2026-09-11  
**Status:** implementation design for Phase B only  
**Baseline `main`:** `53dfb0eef73e14a9413cca202fd268e895520258` (PR #15 merged; contains PR #14)  
**`GENERALITY_V2_FREEZE_SHA`:** `b580c6275acc797c7433faa490cc712107b0adb5`  
**Scope:** RELEASE, GRASP, PUSH. Stop after hold-out first score. No vision, VLA, planner, PLACE, insertion, tactile, locomotion.

## 0. Frozen

Do not redesign: authority kernel, governor, session, consume/replay, SIM ≠ METAL, REACH compile (except a concrete bug Phase B tests expose), TransformGraph architecture, EmbodimentModel architecture for aesthetics.

REACH evidence remains required: UR5e development, Panda postfix regression, KUKA iiwa14 untouched V2 hold-out.

`crates/semantics` must not contain product/fixture identifiers (`panda`, `ur5e`, `kuka`, `iiwa`, `franka`).

## 1. Write path (unchanged authority)

```
SkillContract(RELEASE|GRASP|PUSH)
    → semantic resource / REACH subgoals
    → ActionProposal
    → RealityOs::decide          FROZEN
    → session / governor / plant FROZEN
    → MuJoCo port
    → privileged verifier        verify only
```

Gripper/contact commands use existing verbs (`drive` / `reach`). No new kernel verbs.

Policy observation never receives privileged MuJoCo contacts unless the embodiment actually declares an equivalent sensor. Verifier truth is separate.

## 2. New semantics modules

Owned by `realityos-semantics`:

| Module | Owns |
|---|---|
| `object` | `ObjectState` — physical identity, not product class |
| `contact` | `ContactState`, `SupportRelation` |
| `resource` | `ControlledResource`, topologies, lowering |
| `gripper_state` | OPEN / CLOSED_EMPTY / CLOSED_ON_OBJECT / … |
| `interaction` | object / grasp / approach / push / surface frames via TransformGraph |
| `failure` | grasp/push/release taxonomy |
| `plan` | ordered skill steps (REACH, resource command, verify motion, stop) |
| `release` / `grasp` / `push` | SkillContract compile |

`WorldState` keeps the REACH goal. Manipulation adds object states, support (policy-visible only when perception provider supplies it), and `PERFECT_PERCEPTION` marking. Unknown stays unknown.

## 3. Resource topologies

Exactly:

- `DIRECT_JOINT_GRIPPER` — each affected joint has its own actuator
- `COUPLED_JOINT_GRIPPER` — joint equality, no tendon
- `TENDON_DRIVEN_GRIPPER` — actuator targets a tendon; tendon wrap + equality recorded as declared
- `UNSUPPORTED_RESOURCE_TOPOLOGY` → `MODEL_FEATURE_UNSUPPORTED` with details

Panda-class models: keep tendon name, wrap coefficients, equality pair. Do **not** flatten `actuator → finger_joint` unless the model states that transmission.

Skill logic commands an opening coordinate in `[0, 1]` (0 closed, 1 open). Lowering maps through `command_range` and `closing_direction`.

## 4. Evidence rules

- Close command sent ≠ grasp success
- `CLOSED_ON_OBJECT` requires contact + blocked closure (or equivalent), not command issuance
- Grasp success uses an explicit subset of: expected finger/object contact; blocked empty-close; object follows EE on bounded verify motion (2–5 cm); support change; no forbidden/excess-force contact
- Force: use `mj_contactForce` when present. Unknown physical bound → `FORCE_BOUND_UNAVAILABLE`. Do not invent Newtons from ctrlrange
- Object poses for policy: `PERFECT_PERCEPTION` scenario provider. Not robot perception

## 5. Embodiments

| Role | Bundle | Resource |
|---|---|---|
| Dev gripper A | official Menagerie Panda (already imported) | tendon + equality |
| Dev gripper B | original `arm_gripper` | two direct finger actuators |
| PUSH extra | UR5e (no gripper), KUKA if EE contact exists | NOT_APPLICABLE for GRASP |
| Hold-out | new official Menagerie manipulator **after** `MANIPULATION_V1_FREEZE_SHA` | first score once |

## 6. Acceptance

Minimum episode counts (not success targets): RELEASE ≥100 / gripper embodiment; GRASP ≥300 mixed; PUSH ≥300 / applicable arm. Failures stay visible. Then freeze semantics, import hold-out, first score, stop.
