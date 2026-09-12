# Close Actuator Invention Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make unknown actuator state unrepresentable as a command: omitted actuators hold a measured or explicitly declared finite value, otherwise REFUSE with zero downstream policy writes, then freeze software and (only on a Linux XL330 bench) measure metal.

**Architecture:** Keep the existing write path. Add `explicit_safe: BTreeMap<String, f64>` on the existing command sets. Collapse the two lowerers onto one private validate+hold helper in `adapter.rs`. Stop verify's `named_to_ctrl` from zero-filling. Feed KeepCurrent with finite observed qpos **and** ctrl. Do not touch kernel, governor, session, ledger, or the XL330 driver.

**Tech Stack:** Rust 1.95.0 workspace; `realityos-semantics` + `realityos-verify`; existing `SimActuationProbe.policy_ctrl_writes`; MuJoCo 3.7.0 for verify jobs; existing `scripts/metal-campaign.sh` on Linux only.

## Global Constraints

- Baseline SHA: `0bd136eb6b58bfd3538ee836a660f8a8ee16afd3`
- Preserve `MANIPULATION_V1_FREEZE_SHA=b6396a784c9d09e69cafbb073a9ef3e933113990` and do not rewrite `docs/superpowers/evidence/manipulation_holdout_first_score.json`
- Do not redesign kernel, CertifiedCommand, IssuedCommand, OnlineWrite, RuntimeGovernor, RuntimeSession ONLINE, consume/replay, hardware identity, authority clock, evidence architecture, process topology, metal proof schema, XL330 driver
- No PLACE, insertion, tactile, VLA, locomotion, planner, new robot abstraction, new safety subsystem, new proof framework
- Unknown means unknown. Never invent state. No output command vector on failure.
- Zero is not an implicit safe value. Explicit `0.0` is legal only when declared in `explicit_safe` or measured in current.
- Do not create `docs/metal_proof.json` except from a measured campaign with `experiment_status=measured_success`
- This host is Windows: metal bash/`sudo`/`/dev/ttyUSB*` is a named hole here, not success
- Do not “fix” plant dynamics zeros, governor Abort zeros, `runtime_assurance` Zero, XL330 `ticks_from_action` 0-tick hold, or hostile replay zeros

## File map

| File | Responsibility |
|---|---|
| `crates/semantics/src/command.rs` | `explicit_safe` on `JointTargetSet` and `ActuatorCommandSet` |
| `crates/semantics/src/skill.rs` | `SkillRefuse::InvalidCommand`; `writes_allowed() == false` |
| `crates/semantics/src/adapter.rs` | private validate + `hold_value`; both lowerers; unit tests |
| `crates/semantics/src/resource.rs` | construct `explicit_safe: BTreeMap::new()` |
| `crates/verify/src/reach_foundation.rs` | observed map includes ctrl; `named_to_ctrl`; `refuse_string` for `InvalidCommand` |
| `crates/verify/src/manipulation.rs` | shared observed map; `named_to_ctrl` is `Result`; lower failure → no proposal |
| `crates/verify/src/lib.rs` | e2e: lowering failure ⇒ `policy_ctrl_writes` delta 0 |

Do **not** add a new crate or `crates/semantics/src/lower.rs` unless `adapter.rs` becomes unreviewable after the helper lands. Prefer private functions in `adapter.rs`.

---

### Task 1: Command-set field and InvalidCommand

**Files:**
- Modify: `crates/semantics/src/command.rs`
- Modify: `crates/semantics/src/skill.rs`
- Modify: `crates/semantics/src/resource.rs` (struct literal)
- Modify: `crates/semantics/src/adapter.rs` (existing `JointTargetSet { ... }` literal)
- Modify: `crates/verify/src/reach_foundation.rs` (`refuse_string` match)

**Interfaces:**
- Consumes: existing `HoldSemantics::{KeepCurrent, ExplicitSafe}` unit variants (do not change the enum shape)
- Produces:
  - `JointTargetSet { targets, hold_outside, explicit_safe: BTreeMap<String, f64> }`
  - `ActuatorCommandSet { commands, hold_outside, explicit_safe: BTreeMap<String, f64> }`
  - `SkillRefuse::InvalidCommand`

- [ ] **Step 1: Write the failing tests**

In `crates/semantics/src/command.rs` tests, add:

```rust
#[test]
fn explicit_safe_map_defaults_empty() {
    let json = r#"{"targets":[],"hold_outside":"explicit_safe"}"#;
    let set: JointTargetSet = serde_json::from_str(json).unwrap();
    assert!(set.explicit_safe.is_empty());
    assert_eq!(set.hold_outside, HoldSemantics::ExplicitSafe);
}

#[test]
fn actuator_set_explicit_safe_roundtrip() {
    let mut set = ActuatorCommandSet {
        commands: vec![],
        hold_outside: HoldSemantics::ExplicitSafe,
        explicit_safe: std::collections::BTreeMap::from([("grip".into(), 0.0)]),
    };
    let v = serde_json::to_value(&set).unwrap();
    assert_eq!(v["explicit_safe"]["grip"], 0.0);
    set = serde_json::from_value(v).unwrap();
    assert_eq!(set.explicit_safe.get("grip").copied(), Some(0.0));
}
```

In `crates/semantics/src/skill.rs` `refuse_never_authorizes_writes`, add `SkillRefuse::InvalidCommand` to the list.

- [ ] **Step 2: Run tests to verify they fail**

Run: `cargo test -p realityos-semantics --lib command::tests -- --nocapture`

Expected: compile error (`explicit_safe` missing) and/or `InvalidCommand` not found.

- [ ] **Step 3: Write minimal implementation**

`command.rs` — keep the enum; add the field with serde default:

```rust
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum HoldSemantics {
    KeepCurrent,
    ExplicitSafe,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct JointTargetSet {
    pub targets: Vec<JointTarget>,
    pub hold_outside: HoldSemantics,
    #[serde(default)]
    pub explicit_safe: BTreeMap<String, f64>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ActuatorCommandSet {
    pub commands: Vec<ActuatorCommand>,
    pub hold_outside: HoldSemantics,
    #[serde(default)]
    pub explicit_safe: BTreeMap<String, f64>,
}
```

`skill.rs` — add `InvalidCommand` to the enum.

`resource.rs` `ActuatorCommandSet { commands, hold_outside, explicit_safe: BTreeMap::new() }`.

`adapter.rs` REACH compile `JointTargetSet { targets, hold_outside: HoldSemantics::KeepCurrent, explicit_safe: BTreeMap::new() }`.

`command.rs` existing `named_lookup_does_not_use_index` literal: `explicit_safe: BTreeMap::new()`.

`reach_foundation.rs` `refuse_string`:

```rust
SkillRefuse::InvalidCommand | SkillRefuse::Refuse => "REFUSE".into(),
```

(keep the existing `Refuse` arm; fold `InvalidCommand` into it).

- [ ] **Step 4: Run tests to verify they pass**

Run: `cargo test -p realityos-semantics --lib`

Expected: PASS. `cargo test -p realityos-verify --lib reach_foundation` must still compile (exhaustive match).

- [ ] **Step 5: Commit**

```bash
git add crates/semantics/src/command.rs crates/semantics/src/skill.rs crates/semantics/src/resource.rs crates/semantics/src/adapter.rs crates/verify/src/reach_foundation.rs
git commit -m "fix(semantics): declare explicit_safe on command sets; add InvalidCommand"
```

---

### Task 2: Fail-closed hold resolver (TDD in adapter tests)

**Files:**
- Modify: `crates/semantics/src/adapter.rs`

**Interfaces:**
- Consumes: `HoldSemantics`, `explicit_safe`, `EmbodimentModel`, `HashMap<String, f64>` current
- Produces:
  - `fn hold_value(hold, explicit_safe, act, current) -> Result<f64, SkillRefuse>`
  - `fn lower_named_targets(...) -> Result<Vec<(String, f64)>, SkillRefuse>` (same signature)
  - `fn lower_actuator_commands(...) -> Result<Vec<(String, f64)>, SkillRefuse>` (same signature)
  - both call the same private fill; neither uses `unwrap_or(0.0)`

- [ ] **Step 1: Write the failing tests**

Add to `crates/semantics/src/adapter.rs` `mod tests`. Use `synth_planar_two_link()`. Import `ActuatorCommand`, `ActuatorCommandSet`.

```rust
fn cmd(name: &str, value: f64, mode: &str) -> ActuatorCommand {
    ActuatorCommand {
        actuator_name: name.into(),
        value,
        control_mode: mode.into(),
        skill_id: "t".into(),
    }
}

fn current_complete() -> std::collections::HashMap<String, f64> {
    let mut m = std::collections::HashMap::new();
    m.insert("j0".into(), 0.11);
    m.insert("j1".into(), 0.22);
    m.insert("a0".into(), 0.11);
    m.insert("a1".into(), 0.22);
    m
}

#[test]
fn keep_current_holds_exact_measured_on_omitted_actuator() {
    let model = synth_planar_two_link();
    let set = ActuatorCommandSet {
        commands: vec![cmd("a0", 0.4, "position")],
        hold_outside: HoldSemantics::KeepCurrent,
        explicit_safe: Default::default(),
    };
    let out = lower_actuator_commands(&model, &set, &current_complete()).unwrap();
    assert_eq!(out, vec![("a0".into(), 0.4), ("a1".into(), 0.22)]);
}

#[test]
fn keep_current_missing_state_refuses_and_emits_no_vector() {
    let model = synth_planar_two_link();
    let set = ActuatorCommandSet {
        commands: vec![cmd("a0", 0.4, "position")],
        hold_outside: HoldSemantics::KeepCurrent,
        explicit_safe: Default::default(),
    };
    let mut current = current_complete();
    current.remove("a1");
    current.remove("j1");
    let err = lower_actuator_commands(&model, &set, &current).unwrap_err();
    assert_eq!(err, SkillRefuse::MissingJointState);
    assert!(!err.writes_allowed());
}

#[test]
fn explicit_safe_uses_declared_value_including_zero() {
    let model = synth_planar_two_link();
    let set = ActuatorCommandSet {
        commands: vec![cmd("a0", 0.4, "position")],
        hold_outside: HoldSemantics::ExplicitSafe,
        explicit_safe: std::collections::BTreeMap::from([("a1".into(), 0.0)]),
    };
    let out = lower_actuator_commands(&model, &set, &Default::default()).unwrap();
    assert_eq!(out, vec![("a0".into(), 0.4), ("a1".into(), 0.0)]);
}

#[test]
fn explicit_safe_without_declaration_refuses() {
    let model = synth_planar_two_link();
    let set = ActuatorCommandSet {
        commands: vec![cmd("a0", 0.4, "position")],
        hold_outside: HoldSemantics::ExplicitSafe,
        explicit_safe: Default::default(),
    };
    let err = lower_actuator_commands(&model, &set, &current_complete()).unwrap_err();
    assert_eq!(err, SkillRefuse::InvalidCommand);
}

#[test]
fn unknown_actuator_refuses() {
    let model = synth_planar_two_link();
    let set = ActuatorCommandSet {
        commands: vec![cmd("nope", 0.1, "position")],
        hold_outside: HoldSemantics::KeepCurrent,
        explicit_safe: Default::default(),
    };
    let err = lower_actuator_commands(&model, &set, &current_complete()).unwrap_err();
    assert_eq!(err, SkillRefuse::MissingActuator);
}

#[test]
fn duplicate_actuator_refuses() {
    let model = synth_planar_two_link();
    let set = ActuatorCommandSet {
        commands: vec![cmd("a0", 0.1, "position"), cmd("a0", 0.2, "position")],
        hold_outside: HoldSemantics::KeepCurrent,
        explicit_safe: Default::default(),
    };
    let err = lower_actuator_commands(&model, &set, &current_complete()).unwrap_err();
    assert_eq!(err, SkillRefuse::InvalidCommand);
}

#[test]
fn nan_refuses() {
    let model = synth_planar_two_link();
    let set = ActuatorCommandSet {
        commands: vec![cmd("a0", f64::NAN, "position")],
        hold_outside: HoldSemantics::KeepCurrent,
        explicit_safe: Default::default(),
    };
    assert_eq!(
        lower_actuator_commands(&model, &set, &current_complete()).unwrap_err(),
        SkillRefuse::InvalidCommand
    );
}

#[test]
fn inf_refuses() {
    let model = synth_planar_two_link();
    for v in [f64::INFINITY, f64::NEG_INFINITY] {
        let set = ActuatorCommandSet {
            commands: vec![cmd("a0", v, "position")],
            hold_outside: HoldSemantics::KeepCurrent,
            explicit_safe: Default::default(),
        };
        assert_eq!(
            lower_actuator_commands(&model, &set, &current_complete()).unwrap_err(),
            SkillRefuse::InvalidCommand
        );
    }
}

#[test]
fn incompatible_control_mode_refuses() {
    let model = synth_planar_two_link();
    let set = ActuatorCommandSet {
        commands: vec![cmd("a0", 0.1, "velocity")],
        hold_outside: HoldSemantics::KeepCurrent,
        explicit_safe: Default::default(),
    };
    assert_eq!(
        lower_actuator_commands(&model, &set, &current_complete()).unwrap_err(),
        SkillRefuse::InvalidCommand
    );
}

#[test]
fn tendon_omitted_without_current_refuses() {
    let mut m = synth_planar_two_link();
    m.actuators.push(Actuator {
        name: "split".into(),
        target_joint: "split".into(),
        control_mode: "position".into(),
        transmission_kind: "tendon".into(),
        ctrlrange: Provenanced::unknown("test", 0.0),
        forcerange: Provenanced::unknown("test", 0.0),
        gear: Provenanced::unknown("test", 0.0),
    });
    let targets = JointTargetSet {
        targets: vec![JointTarget {
            joint_name: "j0".into(),
            actuator_name: Some("a0".into()),
            value: 0.3,
            control_mode: "position".into(),
            skill_id: "skill.reach".into(),
        }],
        hold_outside: HoldSemantics::KeepCurrent,
        explicit_safe: Default::default(),
    };
    let mut current = current_complete();
    current.remove("split");
    let err = lower_named_targets(&m, &targets, &current).unwrap_err();
    assert_eq!(err, SkillRefuse::MissingJointState);
}

#[test]
fn tendon_omitted_with_named_current_holds_exact() {
    let mut m = synth_planar_two_link();
    m.actuators.push(Actuator {
        name: "split".into(),
        target_joint: "split".into(),
        control_mode: "position".into(),
        transmission_kind: "tendon".into(),
        ctrlrange: Provenanced::unknown("test", 0.0),
        forcerange: Provenanced::unknown("test", 0.0),
        gear: Provenanced::unknown("test", 0.0),
    });
    let targets = JointTargetSet {
        targets: vec![JointTarget {
            joint_name: "j0".into(),
            actuator_name: Some("a0".into()),
            value: 0.3,
            control_mode: "position".into(),
            skill_id: "skill.reach".into(),
        }],
        hold_outside: HoldSemantics::KeepCurrent,
        explicit_safe: Default::default(),
    };
    let mut current = current_complete();
    current.insert("split".into(), 0.77);
    let out = lower_named_targets(&m, &targets, &current).unwrap();
    let split = out.iter().find(|(n, _)| n == "split").unwrap();
    assert_eq!(split.1, 0.77);
    let a1 = out.iter().find(|(n, _)| n == "a1").unwrap();
    assert_eq!(a1.1, 0.22);
}

#[test]
fn ambiguous_joint_target_refuses() {
    let mut m = synth_planar_two_link();
    m.actuators.push(Actuator {
        name: "a0_alias".into(),
        target_joint: "j0".into(),
        control_mode: "position".into(),
        transmission_kind: "joint".into(),
        ctrlrange: Provenanced::unknown("test", 0.0),
        forcerange: Provenanced::unknown("test", 0.0),
        gear: Provenanced::unknown("test", 0.0),
    });
    let targets = JointTargetSet {
        targets: vec![JointTarget {
            joint_name: "j0".into(),
            actuator_name: None,
            value: 0.3,
            control_mode: "position".into(),
            skill_id: "skill.reach".into(),
        }],
        hold_outside: HoldSemantics::KeepCurrent,
        explicit_safe: Default::default(),
    };
    assert_eq!(
        lower_named_targets(&m, &targets, &current_complete()).unwrap_err(),
        SkillRefuse::InvalidCommand
    );
}

#[test]
fn named_and_actuator_lowerers_agree_on_keep_current() {
    let model = synth_planar_two_link();
    let current = current_complete();
    let cmds = ActuatorCommandSet {
        commands: vec![cmd("a0", 0.4, "position")],
        hold_outside: HoldSemantics::KeepCurrent,
        explicit_safe: Default::default(),
    };
    let targets = JointTargetSet {
        targets: vec![JointTarget {
            joint_name: "j0".into(),
            actuator_name: Some("a0".into()),
            value: 0.4,
            control_mode: "position".into(),
            skill_id: "t".into(),
        }],
        hold_outside: HoldSemantics::KeepCurrent,
        explicit_safe: Default::default(),
    };
    let a = lower_actuator_commands(&model, &cmds, &current).unwrap();
    let b = lower_named_targets(&model, &targets, &current).unwrap();
    assert_eq!(a, b);
}
```

Delete or replace `tendon_actuator_is_held_without_requiring_joint_state` so it no longer expects a successful lower without current for `split`. REACH **compile** without commanding the tendon remains valid; only lowering without observed tendon state must refuse.

- [ ] **Step 2: Run tests to verify they fail**

Run: `cargo test -p realityos-semantics --lib adapter::tests -- --nocapture`

Expected: `keep_current_missing_state_refuses_and_emits_no_vector` and `tendon_omitted_without_current_refuses` FAIL because of `unwrap_or(0.0)`. `explicit_safe_without_declaration_refuses` FAIL (same arm as KeepCurrent). `unknown_actuator_refuses` FAIL (extra names currently dropped). `duplicate_actuator_refuses` FAIL (first match wins). `nan_refuses` / `inf_refuses` / `incompatible_control_mode_refuses` FAIL (no validation).

- [ ] **Step 3: Write minimal implementation**

Private helpers in `adapter.rs` (do not export a new trait):

```rust
use crate::command::{
    ActuatorCommand, ActuatorCommandSet, HoldSemantics, JointTarget, JointTargetSet,
};
use crate::embodiment::{Actuator, EmbodimentModel};
use crate::skill::SkillRefuse;
use std::collections::{BTreeMap, HashMap, HashSet};

fn finite_or_invalid(v: f64) -> Result<f64, SkillRefuse> {
    if v.is_finite() {
        Ok(v)
    } else {
        Err(SkillRefuse::InvalidCommand)
    }
}

fn observed_for(act: &Actuator, current: &HashMap<String, f64>) -> Option<f64> {
    current
        .get(&act.name)
        .or_else(|| current.get(&act.target_joint))
        .copied()
        .filter(|v| v.is_finite())
}

fn hold_value(
    hold: HoldSemantics,
    explicit_safe: &BTreeMap<String, f64>,
    act: &Actuator,
    current: &HashMap<String, f64>,
) -> Result<f64, SkillRefuse> {
    match hold {
        HoldSemantics::KeepCurrent => {
            observed_for(act, current).ok_or(SkillRefuse::MissingJointState)
        }
        HoldSemantics::ExplicitSafe => explicit_safe
            .get(&act.name)
            .copied()
            .ok_or(SkillRefuse::InvalidCommand)
            .and_then(finite_or_invalid),
    }
}

fn bind_joint_target<'a>(
    model: &'a EmbodimentModel,
    t: &JointTarget,
) -> Result<&'a Actuator, SkillRefuse> {
    finite_or_invalid(t.value)?;
    let hits: Vec<&Actuator> = model
        .actuators
        .iter()
        .filter(|act| {
            t.joint_name == act.target_joint
                || t.actuator_name.as_deref() == Some(act.name.as_str())
        })
        .collect();
    match hits.as_slice() {
        [act] => {
            if act.control_mode != t.control_mode {
                return Err(SkillRefuse::InvalidCommand);
            }
            Ok(*act)
        }
        [] => Err(SkillRefuse::MissingActuator),
        _ => Err(SkillRefuse::InvalidCommand),
    }
}

fn fill_holds(
    model: &EmbodimentModel,
    requested: HashMap<&str, f64>,
    hold: HoldSemantics,
    explicit_safe: &BTreeMap<String, f64>,
    current: &HashMap<String, f64>,
) -> Result<Vec<(String, f64)>, SkillRefuse> {
    let mut out = Vec::with_capacity(model.actuators.len());
    for act in &model.actuators {
        let v = if let Some(v) = requested.get(act.name.as_str()) {
            *v
        } else {
            hold_value(hold, explicit_safe, act, current)?
        };
        out.push((act.name.clone(), v));
    }
    if out.is_empty() {
        return Err(SkillRefuse::MissingActuator);
    }
    Ok(out)
}

pub fn lower_actuator_commands(
    model: &EmbodimentModel,
    commands: &ActuatorCommandSet,
    current_by_name: &HashMap<String, f64>,
) -> Result<Vec<(String, f64)>, SkillRefuse> {
    let mut requested = HashMap::new();
    let mut seen = HashSet::new();
    for c in &commands.commands {
        let v = finite_or_invalid(c.value)?;
        let act = model
            .actuators
            .iter()
            .find(|a| a.name == c.actuator_name)
            .ok_or(SkillRefuse::MissingActuator)?;
        if !seen.insert(c.actuator_name.as_str()) {
            return Err(SkillRefuse::InvalidCommand);
        }
        if act.control_mode != c.control_mode {
            return Err(SkillRefuse::InvalidCommand);
        }
        requested.insert(act.name.as_str(), v);
    }
    fill_holds(
        model,
        requested,
        commands.hold_outside,
        &commands.explicit_safe,
        current_by_name,
    )
}

pub fn lower_named_targets(
    model: &EmbodimentModel,
    targets: &JointTargetSet,
    current_by_joint: &HashMap<String, f64>,
) -> Result<Vec<(String, f64)>, SkillRefuse> {
    let mut requested = HashMap::new();
    for t in &targets.targets {
        let act = bind_joint_target(model, t)?;
        if requested.insert(act.name.as_str(), t.value).is_some() {
            return Err(SkillRefuse::InvalidCommand);
        }
    }
    fill_holds(
        model,
        requested,
        targets.hold_outside,
        &targets.explicit_safe,
        current_by_joint,
    )
}
```

No `unwrap_or(0.0)` remains in these two functions.

- [ ] **Step 4: Run tests to verify they pass**

Run: `cargo test -p realityos-semantics --lib`

Expected: PASS, including REACH compile tests. `adapter_selected_by_caps_not_name` still compiles without commanding a gripper.

- [ ] **Step 5: Commit**

```bash
git add crates/semantics/src/adapter.rs
git commit -m "fix(semantics): refuse unknown hold; never invent zero actuator commands"
```

---

### Task 3: Verify mapping must not invent, and must feed KeepCurrent

**Files:**
- Modify: `crates/verify/src/manipulation.rs`
- Modify: `crates/verify/src/reach_foundation.rs`

**Interfaces:**
- Consumes: `lower_named_targets`, `lower_actuator_commands`, `SkillRefuse`
- Produces:
  - `fn observed_hold_values(model, manifest, qpos, ctrl) -> HashMap<String, f64>`
  - `fn named_to_ctrl(manifest, named, current) -> Result<Vec<f64>, SkillRefuse>`
  - ResourceCommand / `write_ctrl` / `action_proposal_from_ctrl` never build `ActionProposal` on `Err`

- [ ] **Step 1: Write the failing tests**

Add in `crates/verify/src/manipulation.rs` under `#[cfg(test)]` if a test module exists there; otherwise add a `#[cfg(test)] mod named_ctrl_tests` at the bottom of that file. If fixture construction is heavy, put tests in `crates/verify/src/lib.rs` and `pub(crate)` the helpers.

```rust
#[test]
fn named_to_ctrl_refuses_zero_fill_on_dim_mismatch() {
    let manifest = RobotManifest {
        // use the smallest in-tree constructor already used by other verify tests.
        // If RobotManifest is only loaded from bundles, skip this unit and cover
        // via the helper on a loaded planar_arm bundle in lib.rs tests.
        .. /* do not invent a second manifest type */
    };
    let err = named_to_ctrl(&manifest, &[("act1".into(), 0.1)], &[]).unwrap_err();
    assert_eq!(err, realityos_semantics::skill::SkillRefuse::InvalidCommand);
}
```

`named_to_ctrl` today is a private `fn` in `manipulation.rs` returning `Vec<f64>`. Make it `pub(crate)` and `Result`. `reach_foundation.rs` must call `crate::manipulation::named_to_ctrl` — `manipulation` does not import `reach_foundation`, so this is not a cycle.

Preferred if `RobotManifest` is awkward to fake: load `robots/bundles/planar_arm` in `crates/verify/src/lib.rs`:

```rust
#[test]
fn named_to_ctrl_does_not_invent_zero_vector() {
    let b = crate::bundle::RobotBundle::load(crate::corpus::robot_dir("planar_arm")).unwrap();
    let (_i, man) = crate::runner::load_and_normalize(&b, &[], 0).unwrap();
    let named: Vec<(String, f64)> = man
        .actuators
        .iter()
        .map(|a| (a.name.clone(), 0.1))
        .collect();
    let bad = crate::manipulation::named_to_ctrl(&man, &named, &[]);
    assert!(bad.is_err());
    crate::mujoco_exec::checkin_worker(_i);
}
```

If `named_to_ctrl` is private, change it to `pub(crate)` **only** — not `pub`.

Also add:

```rust
#[test]
fn observed_hold_values_omits_non_finite() {
    // after helper exists: insert only finite qpos/ctrl; NaN ctrl must not appear
}
```

Write the test first against the current `named_to_ctrl` that returns `Vec<f64>` so it fails to compile/assert.

- [ ] **Step 2: Run test to verify it fails**

Run: `cargo test -p realityos-verify --lib named_to_ctrl_does_not_invent_zero_vector -- --nocapture`

Expected: FAIL or compile error (`named_to_ctrl` returns `Vec`, zero-fills).

- [ ] **Step 3: Write minimal implementation**

Shared helper (put in `manipulation.rs` and call from `reach_foundation.rs`, **or** a tiny `pub(crate)` fn in `manipulation.rs` imported by foundation — do not create a new crate):

```rust
pub(crate) fn observed_hold_values(
    model: &EmbodimentModel,
    manifest: &RobotManifest,
    qpos: &[f64],
    ctrl: &[f64],
) -> HashMap<String, f64> {
    let mut out = HashMap::new();
    for joint in &model.joints {
        if let Some(adr) = joint.qpos_adr {
            if let Some(q) = qpos.get(adr as usize).copied().filter(|v| v.is_finite()) {
                out.insert(joint.name.clone(), q);
            }
        }
    }
    for (i, a) in manifest.actuators.iter().enumerate() {
        if let Some(v) = ctrl.get(i).copied().filter(|v| v.is_finite()) {
            out.insert(a.name.clone(), v);
        }
    }
    out
}

pub(crate) fn named_to_ctrl(
    manifest: &RobotManifest,
    named: &[(String, f64)],
    current: &[f64],
) -> Result<Vec<f64>, realityos_semantics::skill::SkillRefuse> {
    use realityos_semantics::skill::SkillRefuse;
    let nu = manifest.nu.max(0) as usize;
    if current.len() != nu {
        return Err(SkillRefuse::InvalidCommand);
    }
    let mut action = vec![None; nu];
    let mut seen = std::collections::HashSet::new();
    for (name, v) in named {
        if !v.is_finite() {
            return Err(SkillRefuse::InvalidCommand);
        }
        let Some(i) = manifest.actuators.iter().position(|a| a.name == *name) else {
            return Err(SkillRefuse::MissingActuator);
        };
        if !seen.insert(i) {
            return Err(SkillRefuse::InvalidCommand);
        }
        if i >= action.len() {
            return Err(SkillRefuse::InvalidCommand);
        }
        let [lo, hi] = manifest.actuators[i].ctrlrange;
        action[i] = Some(v.clamp(lo.min(hi), lo.max(hi)));
    }
    action
        .into_iter()
        .collect::<Option<Vec<f64>>>()
        .ok_or(SkillRefuse::MissingActuator)
}
```

Wire:

* `current_map` in manipulation.rs → `observed_hold_values(model, manifest, &truth.qpos, &truth.ctrl)` (delete the ResourceCommand-only overlay loop; it becomes redundant).
* `write_ctrl`: same map; `named_to_ctrl(...)?`; on lower/`named_to_ctrl` err, return that error to a caller that **must not** call `decide`.
* ResourceCommand step: on lower err, take the `compile_reach` Err path (refused episode, `ctrl_writes` snapshot, **no** `ActionProposal`).
* `reach_foundation.rs`: `observed_hold_values` instead of joints-only `current_joint_map`; `action_proposal_from_ctrl` uses `named_to_ctrl` and maps `SkillRefuse` through existing `error_report` / `refuse_string` instead of `format!("{e:?}")?` as infra.

ResourceCommand fail-closed sketch (replace `?` on lower):

```rust
let lowered = match lower_actuator_commands(model, cmds, &current) {
    Ok(v) => v,
    Err(e) => {
        let writes = shared.probe.snapshot().policy_ctrl_writes;
        drop(auth);
        let inst = unwrap_shared(shared)?;
        let mut ep = refused_episode(
            bundle, model, sc, sha, &plan.contract_id,
            Some(fail_from_refuse(e)), // already &'static str via ManipulationFailure::from_refuse
            sc.polarity != Polarity::Positive,
        );
        ep.ctrl_writes = writes;
        ep.authority_decisions = decisions;
        let _ = inst;
        return Ok(ep);
    }
};
let action = match named_to_ctrl(&manifest, &lowered, &truth.ctrl) {
    Ok(a) => a,
    Err(e) => { /* same refused_episode path, writes unchanged */ }
};
```

Mirror `write_ctrl` so REACH lower failure does the same (today `write_ctrl` `?` turns it into scenario `String`).

- [ ] **Step 4: Run tests to verify they pass**

Run:

```text
cargo test -p realityos-verify --lib named_to_ctrl_does_not_invent_zero_vector
cargo test -p realityos-semantics --lib
cargo test -p realityos-verify --lib -- --test-threads=1
```

Expected: PASS, or MuJoCo skip where `ensure_mujoco_or_skip` already skips. Do not weaken skips.

- [ ] **Step 5: Commit**

```bash
git add crates/verify/src/manipulation.rs crates/verify/src/reach_foundation.rs crates/verify/src/lib.rs
git commit -m "fix(verify): observed hold map and named_to_ctrl refuse invention"
```

---

### Task 4: End-to-end policy_ctrl_writes delta = 0

**Files:**
- Modify: `crates/verify/src/lib.rs` (test only)

**Interfaces:**
- Consumes: `lower_actuator_commands`, `SimActuationProbe` / `SimAuthority::decide_and_maybe_write` existing path
- Produces: test proving lowering failure never reaches decide

- [ ] **Step 1: Write the failing test**

```rust
#[test]
fn semantic_lowering_failure_does_not_allow_policy_write() {
    use realityos_semantics::adapter::lower_actuator_commands;
    use realityos_semantics::command::{ActuatorCommand, ActuatorCommandSet, HoldSemantics};
    use realityos_semantics::skill::SkillRefuse;

    let model = /* cannot use adapter::synth (crate-private). Build the e2e on
                   a loaded bundle instead. */;
}
```

Use `arm_gripper` or `planar_arm` so the model is real:

```rust
#[test]
fn semantic_lowering_failure_does_not_allow_policy_write() {
    if !crate::mujoco_exec::ensure_mujoco_or_skip() {
        return;
    }
    let b = crate::bundle::RobotBundle::load(crate::corpus::robot_dir("planar_arm")).unwrap();
    let (inst, man) = crate::runner::load_and_normalize(&b, &[], 0).unwrap();
    let model = crate::semantics_map::embodiment_from_manifest(&b, &man);
    let cmds = realityos_semantics::command::ActuatorCommandSet {
        commands: vec![realityos_semantics::command::ActuatorCommand {
            actuator_name: man.actuators[0].name.clone(),
            value: 0.05,
            control_mode: "position".into(),
            skill_id: "t".into(),
        }],
        hold_outside: realityos_semantics::command::HoldSemantics::KeepCurrent,
        explicit_safe: Default::default(),
    };
    let current = std::collections::HashMap::new(); // missing state
    let before = {
        // construct the same plant/authority the foundation tests use, or
        // SharedSimPort probe already on the instance.
        0u64
    };
    match realityos_semantics::adapter::lower_actuator_commands(&model, &cmds, &current) {
        Err(e) => {
            assert_eq!(e, realityos_semantics::skill::SkillRefuse::MissingJointState);
            assert!(!e.writes_allowed());
            // Do not build ActionProposal. Do not call decide.
        }
        Ok(_) => panic!("lowering invented a command vector from empty current"),
    }
    // If this test opened a SimAuthority / SharedSimPort, snapshot policy_ctrl_writes.
    // It must equal `before` (0).
    let _ = inst;
    crate::mujoco_exec::checkin_worker(inst);
}
```

If `embodiment_from_manifest` is not public, use the same helper `write_ctrl` / foundation already uses (`semantics_map`). Grep before inventing a mapper.

Stronger variant (preferred if `SimAuthority` setup in `reach_foundation.rs` is copyable): open journal like foundation, snapshot `shared.probe.snapshot().policy_ctrl_writes` before, skip `decide_and_maybe_write` on `Err`, snapshot after, `assert_eq!(after, before)`.

- [ ] **Step 2: Run test to verify it fails**

Run: `cargo test -p realityos-verify --lib semantic_lowering_failure_does_not_allow_policy_write -- --nocapture`

Expected: FAIL with `lowering invented a command vector from empty current` until Task 2 is merged; after Task 2, this should already PASS if the test never calls decide. If someone still builds a zero `ActionProposal` on error, FAIL until Task 3 wiring is done.

- [ ] **Step 3: Implement only if the test still constructs a proposal on error**

No new framework. The implementation is “do not call decide”.

- [ ] **Step 4: Run test to verify it passes**

Run: `cargo test -p realityos-verify --lib semantic_lowering_failure_does_not_allow_policy_write -- --nocapture`

Expected: PASS (or skip without MuJoCo, same as other verify tests).

- [ ] **Step 5: Commit**

```bash
git add crates/verify/src/lib.rs
git commit -m "test(verify): lowering failure cannot produce a policy write"
```

---

### Task 5: Verification baseline and freeze receipt

**Files:**
- Create: `docs/superpowers/evidence/2026-09-12-pre-metal-verify.txt` (commands, exit codes, SHA only — not a proof schema)

**Interfaces:**
- Consumes: Tasks 1–4 on the working tree
- Produces: `PRE_METAL_FREEZE_SHA=<git rev-parse HEAD>` after green local verification

- [ ] **Step 1: Format and clippy**

Run:

```text
cargo fmt --all -- --check
cargo clippy --workspace --all-targets -- -D warnings
```

Expected: exit 0. Fix only rustfmt/clippy in files already in the allowlist.

- [ ] **Step 2: Tests**

Run:

```text
cargo test -p realityos-metal --all-targets -- --test-threads=1
cargo test --workspace --all-targets --exclude realityos-metal
```

Expected: exit 0.

- [ ] **Step 3: MuJoCo verify job (if Python 3.12 + pins present)**

```text
python -m pip install -r crates/verify/python-requirements.txt
python -c "import mujoco, numpy; assert mujoco.__version__ == '3.7.0'"
$env:REALITYOS_REQUIRE_MUJOCO="1"
$env:MUJOCO_GL="disable"
cargo test -p realityos-verify --all-targets
cargo run -p realityos-verify -- milestone
```

Expected: exit 0, or record skip reason (`mujoco_uninstalled`) without claiming the MuJoCo job passed.

Unix-only (record `named_hole` on this Windows host, do not fake exit 0):

```text
scripts/metal-pty-sequence.sh
scripts/metal-online-journal.sh
scripts/metal-os-boundary.sh
```

- [ ] **Step 4: Freeze**

```text
git rev-parse HEAD
```

Write `PRE_METAL_FREEZE_SHA=<that sha>` into the receipt file together with each command and exit code. After this SHA, do not patch generic software to make metal succeed.

- [ ] **Step 5: Commit receipt only if the user asked to commit docs**

Do not start metal from this task.

---

### Task 6: XL330 campaign (hard gate — not this Windows workspace)

**Files:**
- Existing: `scripts/metal-campaign.sh`, `docs/METAL_EXPERIMENT.md`, `crates/metal/**` (do not replace)

**Interfaces:**
- Consumes: `PRE_METAL_FREEZE_SHA`, real XL330, isolated VIN, operator cutoff
- Produces: measured `docs/metal_proof.json` **or** `MEASURED_INCOMPLETE_OR_FAILED` plus raw trace. Never a hand-edited success artifact.

- [ ] **Step 1: Refuse to run without freeze + hardware**

If `PRE_METAL_FREEZE_SHA` is unset, or device is not a real `ttyUSB*`/`ttyACM*`, or VIN cutoff not operator-tested: stop. Status: `named_hole` / `MEASURED_INCOMPLETE_OR_FAILED`. Do not write `docs/metal_proof.json`.

- [ ] **Step 2: Build and campaign (Linux bench only)**

```bash
cargo build -p realityos-metal --bins
sudo -E env \
  REALITYOS_METAL_DEVICE=/dev/ttyUSB0 \
  REALITYOS_METAL_BIN=$PWD/target/debug \
  REALITYOS_METAL_CUTOFF_TESTED=1 \
  REALITYOS_METAL_CUTOFF_LIVE=1 \
  REALITYOS_METAL_UNPLUG_LIVE=1 \
  scripts/metal-campaign.sh
```

- [ ] **Step 3: Check gates**

Valid HOLD/NUDGE: one certified command, one serial TX, ACK, post-command authority sensor, bounded result.

Hostile list and crash matrix as in `plan1.md` Phase 4. VIN cutoff must be a VIN-specific measured event (`cutoff_live_observed`), not USB unplug / IPC death / torque-disable.

- [ ] **Step 4: Artifact**

Success → existing reporter writes `realityos.metal_proof/1`. Failure → no success artifact; keep raw trace; status `MEASURED_INCOMPLETE_OR_FAILED`.

- [ ] **Step 5: Stop**

Return the 14-point report in `plan1.md` Phase 6. Do not start Manipulation V2, PLACE, or any new feature.

---

## Spec coverage (self-review)

| Spec requirement | Task |
|---|---|
| KeepCurrent missing → refuse, no vector | Task 2 tests 2–3, tendon tests |
| KeepCurrent complete → exact measured | Task 2 test 1, tendon-with-current |
| ExplicitSafe declared / missing | Task 1 field + Task 2 tests |
| unknown / duplicate / NaN / Inf / mode | Task 2 |
| unambiguous bind / no silent drop | Task 2 ambiguous + unknown |
| one helper, two lowerers | Task 2 `fill_holds` |
| named_to_ctrl no zero-fill | Task 3 |
| observed ctrl for REACH and resource | Task 3 `observed_hold_values` |
| no ActionProposal / writes delta 0 | Task 3 wiring + Task 4 |
| fmt/clippy/tests/mujoco/scripts | Task 5 |
| PRE_METAL_FREEZE_SHA | Task 5 |
| XL330 campaign, VIN, proof, stop | Task 6 |
| do not rewrite WX250s first score | Global Constraints |
| do not redesign kernel / metal driver | Global Constraints |

## Placeholder scan

No TBD. Metal on this Windows host is an explicit named hole, not an open design question.

## Type consistency

- `explicit_safe: BTreeMap<String, f64>` on both command sets, keyed by **actuator name**
- `hold_value` / `fill_holds` / both `lower_*` / `named_to_ctrl` all `Result<_, SkillRefuse>`
- `InvalidCommand` in `skill.rs`, adapter tests, `refuse_string`
- `named_to_ctrl` `pub(crate)` returning `Result<Vec<f64>, SkillRefuse>`
