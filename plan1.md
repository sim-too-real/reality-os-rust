# Close actuator invention, then freeze, then measure XL330

Repository: `sim-too-real/reality-os-rust`

Work from current `main`:

`0bd136eb6b58bfd3538ee836a660f8a8ee16afd3`

Execution plan with TDD tasks:

`docs/superpowers/plans/2026-09-12-close-actuator-invention.md`

## Goal

Close the remaining software path that can invent an actuator command from unknown state. Verify the frozen system. Only then run the first genuine measured XL330 metal proof.

Unknown means unknown. Never invent state. Never emit a command vector on failure.

THIS IS NOT AN ARCHITECTURE PASS.

## Hard freeze (do not redesign)

Do not add new robotics capabilities.

Do not redesign:

* kernel
* CertifiedCommand
* IssuedCommand
* OnlineWrite
* RuntimeGovernor
* RuntimeSession ONLINE semantics
* consume/replay ledger
* hardware identity architecture
* authority clock
* evidence architecture
* process topology
* metal proof schema
* XL330 metal implementation (use it; do not replace it)

No PLACE. No insertion. No tactile stack. No VLA. No locomotion. No planner framework. No new robot abstraction. No new generic safety subsystem. No new proof framework.

Preserve:

`MANIPULATION_V1_FREEZE_SHA=b6396a784c9d09e69cafbb073a9ef3e933113990`

Do not rewrite `docs/superpowers/evidence/manipulation_holdout_first_score.json`.

The recorded first-score result `MANIPULATION GENERALIZATION LIMIT FOUND` is historical evidence.

`docs/metal_proof.json` does not exist and must not be created except from a measured campaign with `experiment_status=measured_success`.

## The actual hole (measured in code, not assumed)

`HoldSemantics::KeepCurrent` and `HoldSemantics::ExplicitSafe` are collapsed into one match arm. `ExplicitSafe` has no declared-value field. Two lowerers disagree, and verify has a third zero-fill.

```172:237:crates/semantics/src/adapter.rs
pub fn lower_named_targets(...) {
    // joint-targeting omitted actuators: MissingJointState (honest)
    // non-joint / tendon omitted actuators: hold.unwrap_or(0.0)  // INVENT
}

pub fn lower_actuator_commands(...) {
    // ALL omitted actuators, both KeepCurrent and ExplicitSafe:
    out.push((act.name.clone(), hold.unwrap_or(0.0)));  // INVENT
}
```

```1608:1632:crates/verify/src/manipulation.rs
fn named_to_ctrl(...) {
    let mut action = if current.len() == manifest.nu ... {
        current.to_vec()
    } else {
        vec![0.0; manifest.nu.max(0) as usize]  // INVENT
    };
    // overlay named; clamp
}
```

```199:208:crates/verify/src/reach_foundation.rs
let current_by_joint = current_joint_map(&model, &initial.qpos); // joints only
let proposal = action_proposal_from_ctrl(..., &current_by_joint, ...)?;
// action = lowered values in model.actuators order; no named_to_ctrl
```

Connected write path (unchanged authority):

```
Skill compile (REACH / resource opening)
    → JointTargetSet / ActuatorCommandSet   (partial on purpose)
    → lower_*                               (must fill hold OR REFUSE)
    → named pairs (actuator, value)
    → named_to_ctrl / action vec            (must not invent)
    → ActionProposal
    → SimAuthority / RealityOs::decide
    → session / governor / plant
    → policy_ctrl_writes
```

If lowering fails today, verify uses `?` as a `String` infra error instead of “no proposal, no write”. That is fail-open in the test harness, not fail-closed at the gate.

Existing test `tendon_actuator_is_held_without_requiring_joint_state` encodes the bug: a tendon omitted from REACH is filled with `0.0`. On a tendon gripper, invented `0.0` can mean close.

### Why this is not “just adapter.rs”

| Site | What it does today | After this work |
|---|---|---|
| `lower_named_targets` | joint omit → refuse; tendon omit → `0.0` | both omit → KeepCurrent requires finite observed, else refuse |
| `lower_actuator_commands` | every omit → `0.0` | same hold policy as named targets |
| `HoldSemantics::ExplicitSafe` | identical to KeepCurrent; no values | omitted actuator requires declared finite safe-by-name, else refuse |
| `named_to_ctrl` | dim mismatch → zero vector | refuse; never zero-fill |
| `current_map` / `current_joint_map` | joints only | plus finite observed `ctrl` by actuator name (KeepCurrent fuel) |
| REACH `write_ctrl` | no ctrl overlay | same observed map as resource commands |
| ResourceCommand / `write_ctrl` on lower Err | scenario `?` infra error | no `ActionProposal`, no decide, writes delta = 0 |
| `tendon_actuator_is_held_without_requiring_joint_state` | asserts invented hold | replace: missing tendon current refuses; observed tendon current holds exactly |

### Out of scope (do not “fix” as invention)

These `unwrap_or(0.0)` / zero vectors are **not** this invariant:

* `crates/plant/src/dynamics.rs` — sim integration default
* `crates/verify/src/{observation,normalize,qualify,policy}.rs` — telemetry parse
* `crates/reality-os/src/command.rs` Abort `vec![0.0; len]` — frozen governor abort
* `runtime_assurance` → `AssuranceAction::Zero` on non-finite — frozen
* `crates/metal/src/xl330.rs` `ticks_from_action` empty/`~0` → 0 ticks means **no motion** on this 1-DoF bench, not an invented goal position. Do not replace the XL330 driver.
* Hostile replay in `maybe_replay_or_restart` that **intentionally** proposes zeros — attack traffic, not hold fill

Plant already refuses non-finite writes. Invented **finite** `0.0` is the hole that passes that screen.

## Software design (minimal, existing types)

### 1. Declare ExplicitSafe values on the command set, not in the kernel

Keep `HoldSemantics` as unit variants (`KeepCurrent`, `ExplicitSafe`) so existing serde tags stay valid.

Add to both `JointTargetSet` and `ActuatorCommandSet`:

```rust
#[serde(default)]
pub explicit_safe: BTreeMap<String, f64>,  // keyed by actuator name
```

* `KeepCurrent` ignores the map.
* `ExplicitSafe` requires a **finite** entry for every omitted actuator.
* Empty map + omitted actuator → REFUSE.
* Zero is a legal declared value only when it is **in the map**. Zero is not a default.

Do not add “safe” onto `Actuator` in the embodiment model. That would invent a robot-level default. Safe is a command-time declaration.

### 2. One hold resolver, two public lowerers

Do not add a new crate, trait, or proof system. Private helpers in `crates/semantics/src/adapter.rs`:

* validate requested commands/targets (exist, unique bind, finite, mode match, no silent drop, no silent extra)
* resolve omitted actuators through `hold_value`
* return `Ok(Vec<(String, f64)>)` only when every model actuator has a determined finite value
* on any failure, drop the local vector and `Err` — no partial output

`SkillRefuse` mapping (add one variant, not a subsystem):

| Failure | Refuse |
|---|---|
| missing observed hold (`KeepCurrent`) | `MissingJointState` |
| unknown requested actuator / target binds none | `MissingActuator` |
| duplicate, ambiguous bind, non-finite, mode mismatch, missing/non-finite ExplicitSafe | `InvalidCommand` |

`InvalidCommand` must be added to:

* `crates/semantics/src/skill.rs` (`writes_allowed` stays `false`; extend the exhaustive test)
* `crates/verify/src/reach_foundation.rs` `refuse_string` (exhaustive) → `"REFUSE"`
* `crates/semantics/src/failure.rs` already maps unknown refuses to `ResourceUnsupported` via `_`

### 3. Validate before fill

Requested set:

* every named actuator exists on the model
* no duplicate actuator
* every value finite (`NaN`, `+Inf`, `-Inf` refuse)
* `control_mode` equals `Actuator.control_mode` (string, existing field)
* each `JointTarget` binds **exactly one** model actuator (`joint_name == target_joint` OR `actuator_name == act.name`)
* two targets must not bind the same actuator
* a requested name that matches nothing is refuse, not drop

Hold fill:

* iterate **model** actuators only (output scope = declared model, never extra)
* commanded → use requested finite value
* omitted + `KeepCurrent` → finite observed at `act.name` else `act.target_joint`, else refuse
* omitted + `ExplicitSafe` → finite `explicit_safe[act.name]`, else refuse
* observed `Some(NaN)` is missing, not a hold

“Command scope must never silently widen” means: do not invent values, do not apply one target to two actuators, do not keep extra unknown names by ignoring them. KeepCurrent fill of **measured** omitted actuators is the declared hold policy, not silent widening.

### 4. Verify boundary: mapping must not be a second inventor

* `observed_hold_values(model, manifest, qpos, ctrl)` inserts only **finite** joint qpos and actuator ctrl. Absence stays absence.
* ResourceCommand already overlays ctrl; REACH `write_ctrl` and foundation `current_joint_map` must use the same helper.
* `named_to_ctrl` returns `Result`. Refuse if observed ctrl length ≠ `nu`, if a named actuator is missing from the manifest, if a manifest actuator is missing from named, if duplicate names, if non-finite. **Never** `vec![0.0; nu]`.
* Foundation `action_proposal_from_ctrl` must use `named_to_ctrl`, not “values in model order”.
* On lower/`named_to_ctrl` `Err`: do not build `ActionProposal`, do not call `decide_and_maybe_write`, leave `policy_ctrl_writes` unchanged. Record a refuse episode the same way `compile_reach` `Err` already does. Do not promote lowering failure to an infra `String` `?`.

### 5. Tests (semantics first, then one existing-instrumentation e2e)

Semantics (`crates/semantics/src/adapter.rs`):

1. Partial command + complete current → omitted actuator holds **exact** measured value (not a clamped neighbor).
2. Partial command + missing current → `MissingJointState`.
3. That refusal returns `Err`; no `Ok` vector.
4. `ExplicitSafe` + declared finite (including explicit `0.0`) → exact declared value.
5. `ExplicitSafe` + no declared value → `InvalidCommand`.
6. Unknown actuator → `MissingActuator`.
7. Duplicate actuator → `InvalidCommand`.
8. `NaN` → `InvalidCommand`.
9. `+Inf` / `-Inf` → `InvalidCommand`.
10. Incompatible `control_mode` → `InvalidCommand`.
11. Tendon/non-joint omitted + missing current → refuse (replaces the old “hold without joint state” test).
12. Tendon omitted + current keyed by **actuator name** → exact measured value.
13. Ambiguous joint/actuator bind → `InvalidCommand`.
14. Both public lowerers share the same hold outcome for the same inputs.

Verify e2e (existing `SimActuationProbe.policy_ctrl_writes`, no new proof crate):

```
semantic lowering failure
    → no ActionProposal
    → decide never called
    → RealityOs cannot ALLOW a write
    → policy_ctrl_writes delta = 0
```

Do not rewrite holdout JSON. Do not re-score WX250s as a new untouched holdout.

## Phase 1 — software invariant (this workspace)

Implement the design above. Files that **may** change before freeze:

* `crates/semantics/src/command.rs`
* `crates/semantics/src/skill.rs`
* `crates/semantics/src/adapter.rs`
* `crates/semantics/src/resource.rs` (pass empty `explicit_safe` at construction)
* `crates/verify/src/manipulation.rs`
* `crates/verify/src/reach_foundation.rs`
* `crates/verify/src/lib.rs` only if the e2e test lives there

Files that **must not** change before freeze unless a test in the list above fails for a concrete reason in that file:

* `crates/kernel/**`
* `crates/governor/**`
* `crates/session/**`
* `crates/plant/src/{execute,ledger,write_guard,consume}.rs`
* `crates/metal/**` (except if a software test already in-tree fails for an unrelated compile)
* `docs/superpowers/evidence/manipulation_holdout_first_score.json`
* `docs/metal_proof.json`

## Phase 2 — verification baseline

Do not weaken CI. GitHub Actions may be unavailable; unexecuted local verification is not.

Pinned toolchain: `rust-toolchain.toml` → `1.95.0`.

Pinned MuJoCo: `crates/verify/python-requirements.txt` → `mujoco==3.7.0`, `numpy==2.2.6`.

Execute and record **exact command, exit code, commit SHA**:

```text
cargo fmt --all -- --check
cargo clippy --workspace --all-targets -- -D warnings
cargo test -p realityos-metal --all-targets -- --test-threads=1
cargo test --workspace --all-targets --exclude realityos-metal
```

Then the MuJoCo job equivalent to `.github/workflows/authority.yml` `verify-mujoco` (Python 3.12, those pins, `REALITYOS_REQUIRE_MUJOCO=1`, `MUJOCO_GL=disable`):

```text
cargo test -p realityos-verify --all-targets
cargo run -p realityos-verify -- milestone
```

Also, on a Unix host with sudo (this Windows workspace cannot run these scripts):

```text
scripts/metal-pty-sequence.sh
scripts/metal-online-journal.sh
scripts/metal-os-boundary.sh          # two-UID HIL/metal OS boundary
```

Manipulation semantics regression: `cargo test -p realityos-semantics --all-targets` plus verify manipulation tests already in `cargo test -p realityos-verify`.

Frozen REACH external-generalization tests: existing `realityos-verify` tests that pin `GENERALITY_FREEZE_SHA` / `GENERALITY_V2_FREEZE_SHA`. Do not change those SHAs.

Record a short verification receipt (commands, exit codes, SHA). Do not invent a second proof schema.

## Phase 3 — freeze software again

After Phase 1 and Phase 2 succeed, record:

`PRE_METAL_FREEZE_SHA=<exact SHA>`

Do not modify generic software after this point merely to make the physical experiment succeed.

Any later software patch must correspond to a concrete measured bench failure.

## Phase 4 — first real XL330 campaign (hard gate)

**Do not start Phase 4 without `PRE_METAL_FREEZE_SHA` and the hardware listed below.**

This workspace is Windows. The campaign is bash + `sudo` + `/dev/ttyUSB*`. Treat missing Linux bench / missing XL330 as a **named hole**, not as success, not as a reason to hand-edit proof JSON.

Required hardware (existing `docs/METAL_EXPERIMENT.md`):

* supported XL330
* isolated servo VIN (USB 5 V must not back-feed the center pin)
* USB data/GND may remain
* operator-accessible independent VIN cutoff
* low-energy no-load fixture
* conservative PWM / velocity / excursion as already configured

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

Do not fabricate or hand-edit proof results.

### Required physical cases

Valid: valid HOLD; valid tiny NUDGE. Each must show one certified command, one command serial TX, successful device ACK, post-command authority-owned sensor acquisition, observed bounded result.

Hostile / invalid (unauthorized certified serial TX delta = 0, unauthorized successful ACK delta = 0):

* unsupported action; oversized action; replay same command ID; stale evidence; missing sensor; malformed proposal; caller timestamp injection; autonomy direct-device attempt; identity mismatch; hot swap; live USB disconnect; reconnect after disconnect; repeated valid nudges attempting cumulative cage escape

Crash matrix (restart must never retransmit an ambiguous command ID):

* before prepare
* after prepare / before serial TX
* after serial TX / before device status
* after ACK / before final ledger completion

### Physical cutoff

Live VIN test must measure servo power loss. Do not count USB disconnect, IPC death, authority crash, or software torque-disable as VIN-cutoff evidence.

Do not call the bench disconnect STO / SS1 / PL / SIL unless certified hardware independently establishes those claims.

## Phase 5 — proof artifact

Only if all proof gates succeed, generate existing `docs/metal_proof.json` schema `realityos.metal_proof/1` from measured cases. No manually populated success fields.

Required headline fields: real actuator present; valid command / TX / ACK counts equal; unauthorized TX = 0; unauthorized ACK = 0; duplicate retransmissions after restart = 0; autonomy direct-device attempts > 0 and successes = 0; identity swap detected; live disconnect detected; live VIN cutoff observed; clock `OsMonotonicClock`; evidence status `MEASURED`.

If any gate fails: do **not** create a success artifact. Record `MEASURED_INCOMPLETE_OR_FAILED` and retain the raw trace.

## Phase 6 — stop

Once the metal result exists (success **or** measured incomplete), STOP. Do not start the next feature.

Report:

1. PRE_METAL_FREEZE_SHA
2. exact files changed before freeze
3. tests executed (command + exit code)
4. real hardware identity, or `named_hole: no XL330 bench on this host`
5. valid command measurements
6. hostile case measurements
7. crash/restart results
8. VIN-cutoff result
9. unauthorized TX count
10. unauthorized ACK count
11. duplicate retransmission count
12. resulting metal-proof status
13. any concrete physical failure discovered
14. whether frozen authority/kernel files changed (`must be no` for Phase 1)

The only justified software work after freeze is work caused by a measured physical failure.

No speculative hardening. No new architecture.

## Manipulation work after metal

Do not modify the historical WX250s first-score artifact. It is no longer an untouched holdout.

After metal is complete (separate later effort, not this plan):

* classify the WX250s failure modes
* use WX250s as development evidence if desired
* fix only robot-independent causes
* freeze a Manipulation V2 SHA
* import a NEW untouched external manipulator
* score it exactly once

Never claim post-tuning performance on WX250s as untouched generalization.
