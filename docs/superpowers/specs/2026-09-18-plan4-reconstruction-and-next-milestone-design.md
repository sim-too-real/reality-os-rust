# Plan4 reconstruction and next-milestone design

**Date:** 2026-09-18  
**Status:** reconstruction complete; one next milestone justified; **do not implement on this checkout**  
**Inspected tree:** `main` @ `c82c78c4053bd9047f14fe0261380cbba93b58dc`  
**Frozen metal candidate:** `2f68a5d82fec5e7e2c0b78ca88fe65d8a8a4acfc` (`PRE_METAL_V2_QUALIFIED`, branch `fix/metal-authority-lifecycle`)  
**Research worktree:** `.worktrees/research-adversarial-evidence-v1` @ `604e2af` (parent `2f68a5d`; **do not merge**)  
**This host:** Windows. `realityos-metal` / `realityos-hil` / `realityos-vport` are Unix-only. No `docs/metal_proof.json`.

This document is the required `plan4.md` output plus the design for the single next milestone. It was reconstructed from code, tests, evidence JSON, CI, and Git — not from README claims.

---

## 0. Approaches considered

**A — Measure first (recommended).** Immediate milestone is the already-designed XL330 campaign at **exactly** `2f68a5d` on a native Linux bench. This session writes reconstruction + plans only. No kernel change. No merge of research.

**B — Narrow recover on a research worktree.** Production `op=recover` must not unlatch integrity `abort_latched` on a live ONLINE instance. ESTOP after `--restart` remains recoverable. Tests first. **Must not** replace the metal SHA.

**C — Split the latch into typestates / rewrite authority.** Cleaner long-term, high architectural risk, delays Gate E. Rejected as the next step.

Recommendation: **A now**. **B only after metal measurement, or in parallel on `research/adversarial-evidence-v1` if an operator explicitly wants software work while the bench is unavailable.** Never C before the first measured campaign.

---

## 1. Repository reconstruction

Reality OS is a **fail-closed last-write kernel** for physical commands, plus a **MuJoCo semantic/manipulation verifier** that is a second product sitting beside that kernel. It is not a robot SDK, not a planner, not certified safety, and not measured metal.

Two live stacks:

1. **Production ONLINE (metal):** untrusted JSON `MetalRequest` → `RealityOs::decide` → `IssuedCommand` → `OnlineWrite` → ledger prepare → `HardwareBackedPlant` → `Xl330Driver`. No `SkillContract`. World is essentially `{ tau_max }`.
2. **Verify/SIM:** `SkillContract` + `CapabilityGraph` + `ControlAdapter` → `ActionProposal` → the same decide path on a simulation rail. Evidence is `SIMULATION_ONLY`.

The workspace is 18 crates. The living choke point is `IssuedCommand` (minted only by `decide`) → `OnlineWrite` (minted only by `RuntimeGovernor<OnlineLocked>::authorize_issued`) → `execute_certified_command` (the only sanctioned `Plant::act`). Learned crates (`agent`) cannot write motors.

`plan1.md` (close actuator invention) is implemented in semantics/verify. `plan2.md`/`plan3.md` are operator metal plans and have **not** produced `docs/metal_proof.json`. Older plan freeze `e360649` is superseded for metal by `2f68a5d`. Current `main` is one merge commit past that freeze (`c82c78c`).

---

## 2. Actual architecture

```text
untrusted JSON (propose|sensor|heartbeat|status|recover)
        │
        ▼
MetalAuthority::handle          production_ops_only; refuses now_s / hil_fault / sensor_samples
        │
        ▼
acquire_sensor (device)         authority receive time, not proposer timestamps
        │
        ▼
RealityOs::decide               Intent + optional PolicyProposal (metal labels it operator)
        │                       screen_proposal() result is discarded; numeric action still certifies
        ▼
IssuedCommand                   private ctor; pub(crate) issue
        │
        ▼
RuntimeSession::dispatch_issued
        │
        ▼
authorize_issued                identity + sensor hash + actuator allow-list + HMAC
        │
        ▼
OnlineWrite                     capability token; no public constructor
        │
        ▼
write_online_now
        │
        ▼
execute_certified_command       CommandLedger prepare → Plant::act (certified-write TLS) → ack/unknown
        │
        ▼
HardwareBackedPlant → HardwareDriverPort → Xl330Driver (Goal Position)
        │
        ▼
observations / cage / ACK / present   metal restamps metal=true from tty, not session
```

Verify stack (not on the metal write path):

```text
WorldState (ReachGoal-shaped) + ObservationFrame
  → SkillContract (REACH/GRASP/RELEASE/PUSH only are qualified)
  → CapabilityGraph (structure, not robot_id)
  → ControlAdapter / named hold lowering
  → ActionProposal
  → SimAuthority → decide → SIM write_driver
  → privileged MuJoCo verifier
```

Cargo is acyclic. Conceptual cycles exist: governor **does** import `CertifiedCommand`/`IssuedCommand` from core (docs that say otherwise are false); two rails (`kernel::RailStatus` vs `governor::Rail`); two skill IRs; two embodiment models; two capability enums.

---

## 3. Strengths

- Typed last-write: `IssuedCommand` / `OnlineWrite` / sealed `Plant` / ONLINE typestate. Same-process dependents cannot mint an executable write. Trybuild enforces this.
- Honesty in types: `HonestyStamp` cannot construct metal/MEASURED. PTY cannot install `docs/metal_proof.json`. Claim ledger refuses to promote SIM into physical safety.
- Fail-closed vocabulary: allow/modify/probe/refuse/abort; probe is not abort; missing scene evidence refuses place; unknown hold no longer invents zeros on the semantic lowerer.
- Identity bind + live re-probe + two-UID HIL topology (HIL proof exists; metal-on-UART does not).
- Consume uniqueness per `command_id` (TLA `docs/models/consume_write.tla`). Spent IDs stay spent even under the recover hole.
- XL330 campaign is **complete as a measurement program**: TX/ACK/present, cage, crash matrix including `after_serial_tx_before_status`, live unplug, live VIN, direct-device attacks. Remaining work is running it.
- Semantics crate forbids held-out robot names. Caps come from structure.

---

## 4. Weaknesses (ranked)

1. **No measured physical evidence.** Gate E not started on hardware. This is the credibility bottleneck.
2. **Untrusted production `recover` is still an ESTOP-ack analog** (NAMED_HOLE: no authenticated operator channel). Software latch on PATCHED `709a079` refuses integrity recover before `Plant::clear_estop`. PRE-FIX `2f68a5d` is intentionally vulnerable and retained only as the frozen pre-fix physical baseline. UART / real XL330 still **not measured**.
3. **`abort_latched` / `unknown_outcome` are not rehydrated from journal.** A crash after unknown can authorize a new id on restart without recover.
4. **Two products glued by verify.** Semantics/manipulation are a SIM research OS. Metal is verb+action. An external roboticist cannot add a robot without both internals (Gate J fail).
5. **Policy is scored on privileged sim truth.** `graph_from_truth`, `PerfectPerception`, candidate fallback. Grasp “acquisition” OR-chain (panda 220 acq vs 12 verified holds).
6. **wx250s manipulation hold-out failed at resource + approach**, not at authority (`MANIPULATION GENERALIZATION LIMIT FOUND`: release 0/20, grasp 2/20, push task 0/20).
7. **Error strings are the authority protocol.** Ledger phrases ↔ `LATCHING_PREFIXES` ↔ metal `op: String`. `ViolationCode` is unused on the wire.
8. **CI jobs on recent private-repo SHAs complete in ~2s with empty steps and `runner_id: 0`.** Last hosted-runner success: 2026-09-10 (`9806282`, authority verify ran checkout/toolchain/fmt/clippy/test). Not a demonstrated Rust test failure. Root cause is GitHub-hosted runner assignment on this private account (Actions enabled; no self-hosted runners), not workflow step deletion.
9. **Windows cannot build default-members** (unix-only metal/hil/vport).
10. **`screen_proposal` is telemetry.** A numeric proposal that passes envelope becomes `IssuedCommand`. Metal always labels autonomy as operator.

---

## 5. Accidental complexity (simplify / freeze / do not grow)

- Parallel `EmbodimentGraph` vs `EmbodimentModel`, `WorldBelief` vs `WorldState`, kernel `Capability` vs `CapName`, `SkillIR` vs `SkillContract`.
- `SkillName` lists Place/Insert/Pull/… with empty `named()`.
- Lifecycle typestates (`CertifiedIntent` → `AcknowledgedCommand`) unused on ONLINE.
- `SensorModel` defined, unused.
- `vision`, `rate`, `gauntlet`, `agent`, `data` are early Python-port crates; not the product path.
- `scripts/metal-campaign.sh` is huge because first-contact Wizard/UART folklore is real; do not rewrite it before the first measured run.
- `xl330.rs` vendor protocol mixed into the only hardware port.
- README / `PUBLIC_SURFACE.md` / `ARCHITECTURE.md` stale (CLI extra commands; “Governor does not import CertifiedCommand” is false).
- SHA freeze zoo: `5c4c6e5` / `e360649` / `2f68a5d` / `c82c78c` / `b6396a7`. **One metal SHA: `2f68a5d`.**

---

## 6. Missing foundational concepts (only those justified now)

- **Operator vs proposer recovery.** Campaign needs ESTOP recover after `--restart`. Integrity abort must not be the same op.
- **World sufficiency** as `Unknown | Partial | Actionable`, not a default 5 cm `ReachGoal`.
- **Policy-visible observation that cannot contain privileged FK** unless a real `SensorModel` exists.
- **Resource contract measured once and hashed** (wx250s `resource_qualification: {}`).
- **Conjunctive grasp success** (contact ∧ follow ∧ support), not OR.

Do **not** invent InformationNeed/PROBE-as-motion until Gate E exists and privileged perception is cut. Probe today is a kernel status, not a sensing skill (`SkillRefuse::Probe` is never constructed by compile_reach/grasp/release/push).

---

## 7. Gate status A–J

| Gate | Status | Evidence |
|---|---|---|
| **A Software authority** | **Partial.** In-process minting closed (trybuild). Production IPC `recover` is an authorization hole. | `metal/src/authority.rs:214-222`; `governor/src/latch.rs:110-114`; HIL production **refuses** recover (`hil/src/lib.rs:279-292`) |
| **B Process isolation** | **Partial.** HIL two-UID measured (`docs/hil_os_users_proof.json`). Metal-on-UART unmeasured. Socket `0660`, no peer cred. Same-UID `/proc/fd` named non-claim. | CI `os-users.yml`; `CLAIM_LEDGER.md` process-isolation claims |
| **C Physical identity** | **Designed, not physically closed.** Exact bind + live EEPROM re-read. XL330 has **no factory serial**; bind is adapter+id+model+fw+deployment cal. Identical spare on same adapter/id can pass. | `governor` identity tests; `METAL_EXPERIMENT.md`; `proof.rs` named hole |
| **D Crash/replay** | **Partial.** Spent IDs + prepare-before-write + TLA. Recover wipes abort; abort not journaled; older matching journal+seal can treat never-seen ids as Unseen. | ledger tests; research `red_team.md`; `ledger.rs` seal comment |
| **E Physical evidence** | **Not passed.** No `docs/metal_proof.json`. Ledger NOT_EVIDENCE. | tree listing; reporter refuses `hardware_present=false` |
| **F Semantic generality** | **Partial.** REACH transfers after a post-hoc semantic change (panda first 0/10 then 10/10 with `semantic_code_changed_after_first_score: true`). iiwa V2 10/10. Verify YAML still name-heuristic. | evidence JSONs |
| **G Manipulation generality** | **Failed (recorded).** wx250s `MANIPULATION GENERALIZATION LIMIT FOUND`. | `manipulation_holdout_first_score.json` |
| **H Closed-loop real task** | **Not started.** | every verify JSON `metal: false` |
| **I Active uncertainty** | **Not built.** Probe is a status. Observe/LookAt unqualified. Camera `NOT_IMPLEMENTED_IN_VERIFY_V1`. | `kernel/src/decision.rs`; `semantics/src/skill.rs` |
| **J Product usability** | **Fail.** No `Robot::load`. Dual embodiment. Domain verb table. Linux metal soup. | `PUBLIC_SURFACE.md` vs `apps/ros-governor/src/main.rs` |

Do not claim a gate passed without the artifact named above.

---

## 8. Physical readiness

The campaign is **sufficiently designed**. The milestone is **running and measuring it**, not adding metal infrastructure.

**Blockers only:**

1. Wrong SHA if measured from this tree (`c82c78c`). Detach at `2f68a5d`.
2. This Windows workspace: no USB-UART assumed, Unix sockets, bash/`sudo`/`udevadm`/`fuser`. `ENVIRONMENT_SETUP_FAILURE`, not a kernel task.
3. Hardware: XL330-M288-T or M077-T; 5 V ≤0.5 A; VIN isolated from USB 5 V; operator disconnect; 32-tick mechanical stop; operator for live unplug and live VIN.
4. Do not merge research `604e2af`.

**Not blockers:** new HAL, second CLI, rewriting `HardwareDriverPort` before the first XL330, faking the proof, treating PTY/HIL as metal.

`HardwareDriverPort` is enough for this experiment. A second target that would **falsify** it: multi-drop Dynamixel (wx250s already in-repo) or any EtherCAT/CiA 402 drive — not a second identical XL330.

---

## 9. Manipulation diagnosis

wx250s hold-out (freeze `b6396a7`, `SIMULATION_ONLY`):

| Skill | n | Result |
|---|---|---|
| RELEASE | 20 | success **0**, expected refusals 2, empty `resource_qualification` |
| GRASP | 20 | acquisition **2**, verified hold **2**, expected refusals 16 |
| PUSH | 20 | contact **14**, task success **0** |

Earliest stage: **resource/world sufficiency** (coupled gripper never qualified), then **approach generation** (hardcoded offsets from privileged pose), then **contact maintenance** (push contacts without displacement). Authority refusals are not the bottleneck (`unauthorized_writes: 0`).

In-distribution panda (PerfectPerception): release 86/100, grasp acq 220/300 vs hold 12/300, push 107/300. Grasp success can fire without object-follows-EE.

Do not patch wx250s. Do not add PLACE. Treat the limit as evidence.

---

## 10. Product direction (12–24 months)

Become the **last-write execution substrate** that turns a small set of embodiment-described skills into **measured** physical outcomes, with SIM≠metal enforced in types and in artifacts.

Year-1 credibility: one real XL330 proof, then a recover/latch composition fix, then one embodiment+capability surface so a second robot is data not a fork. Year-2: one contact skill that transfers without privileged perception; ROS 2 as adapter only; sit on QNX/Apex later rather than out-certifying them.

Public surface should collapse toward: load bundle → observe (authority-owned) → capabilities → submit task → allow/probe/refuse/abort → execute through OnlineWrite → explain from the two ledgers. Names are not frozen.

---

## 11. Competitor check and what NOT to build

Independent `/deep-research` (2026-09-18, **Partial**): session workflow `deep-research`. Aligns with `docs/COMPETITOR_WEDGE.md`. Not a certificate.

**Own in-kernel:** `IssuedCommand` / `OnlineWrite` / sealed Plant, identity bind, spent-id consume journal, honesty stamps. GoalIR/SkillIR stay non-executable.

**Integrate, do not rebuild:** ROS 2 as adapter (hardware writes default off); ros2_control-class drivers behind `HardwareDriverPort`; MoveIt / Nav2 / cuRobo / cuMotion as planners to validate *behind* the gate; Pinocchio / Drake / MuJoCo as screens or SIM, never metal evidence; LeRobot / GR00T / OpenVLA / Gemini as proposers of goals, skills, or bounded references.

**Sit on later, do not replace:** drive STO/SS1 (IEC 61800-5-2), fieldbus FSCP (IEC 61784-3), QNX/Apex SEooC OS. Rust privacy is not ISO 10218. ros2_control is an I/O mutex, not a spent-id journal. Nav2 Collision Monitor is a wiring-dependent `cmd_vel` filter, not certified stop. Isaac ROS/Sim licenses disclaim Critical Applications. Drake orange ports are simulator ground-truth. `consume_write.tla` is a spec, not a TLC/TLAPS proof in this tree.

**Do not build:**

- Ansys / Isaac Sim / NITROS / MoveIt / Nav2 / cuRobo / Pinocchio clones
- ONNX / VLA / LeRobot `robot.send_action` as `plant.act`
- Foundry / CAD / invent in this repo
- PLACE, insert, tactile, locomotion, planner framework
- Per-robot `if panda` / `if wx250s` in semantics
- Tuning IK/radii until wx250s “passes”
- Flattening tendons
- Promoting PerfectPerception into metal
- Merging `research/adversarial-evidence-v1` or `sovereign-kernel-rewrite` into the metal freeze
- Hand-written `docs/metal_proof.json`
- Generic HAL expansion before a second real actuator
- Another capability enum / World type / skill IR
- ISO/SIL/MEASURED claims, or treating the ledger HMAC as a hardware root of trust
- Weakening tests to green CI
- Rewriting the authority kernel “for cleanliness”
- PROFIsafe/FSCP inside CommandLedger (belongs on the bus)

---

## 12. Immediate milestone

**Hypothesis:** Frozen Reality OS at `2f68a5d` can execute one authorized XL330 hold and one authorized 32-tick nudge exactly once each, with zero unauthorized certified serial TX / physical writes / device ACKs, under crash/restart, identity change, process attack, live USB unplug, and live VIN cutoff.

**Reason:** Gate E is the unique credibility gap. The campaign is already written. Further kernel hardening without a concrete merge-blocking hole on the freeze SHA is diminishing returns **except** the recover P0, which must **not** change the SHA being measured.

**Files involved (run, do not edit):** `scripts/metal-campaign.sh`, `crates/metal/**`, `docs/METAL_EXPERIMENT.md`, `crates/metal/src/proof.rs`.

**Frozen files:** entire tree at `2f68a5d`. No fmt, no recover patch, no docs-only commits on that SHA.

**Implementation tasks:** none. Operator sequence only (see implementation plan).

**Experiment:** native Linux, real tty, isolated VIN, two UIDs, `scripts/metal-campaign.sh` as documented.

**Acceptance:** `docs/metal_proof.json` with `schema=realityos.metal_proof/1`, `experiment_status=measured_success`, `software_commit_sha=2f68a5d82fec5e7e2c0b78ca88fe65d8a8a4acfc`, `hardware_present=true`, live unplug + live VIN observed.

**Evidence artifact:** `docs/metal_proof.json` + metal-root `METAL_PROOF_REPORT.md`. PTY/CI must not install it.

**Stopping condition:** any unexplained functional failure of the frozen software; or this host remaining Windows/WSL without USB-UART → record `MEASURED_INCOMPLETE_OR_FAILED` and stop. Do not invent the file.

**This Windows agent cannot execute this milestone.**

---

## 13. Next three milestones (after immediate)

1. **Untrusted recover must not clear integrity abort** (Approach B). Reproducer tests from `red_team.md` land on a branch that is **not** the metal SHA. Campaign `--restart` ESTOP recover still works. `abort_latched` from `unknown_outcome` / `replayed command_id` survives `op=recover` on the live instance.
2. **One embodiment + capability surface.** Delete or hide `EmbodimentGraph` toys from the product path; verify ingest is the only robot load. Required for Gate J.
3. **Manipulation resource qualification without privileged perception.** Fix the wx250s failure at resource/approach generically; conjunctive grasp success; do not add skills.

Dependency: 12 (metal) does not depend on 13.1. 13.1 must not merge into the tree that produced the first proof. 13.2–13.3 depend on not expanding skills until F/G are honest.

---

## 14. Architecture changes (only if 13.1 is authorized)

**Current problem:** production IPC treats autonomy as operator for latch reset. `EstopLatch::engage` sets both `engaged` and `abort_latched`. `clear()` wipes both. Campaign **requires** recover after `--restart` to clear journaled ESTOP so the next hold can write. Integrity abort (unknown_outcome, replay) must not use that path.

**Evidence:** `crates/metal/src/authority.rs:214-222`; `crates/governor/src/latch.rs:110-114`; `crates/governor/src/governor.rs:647-664`; research `docs/research/adversarial-v1/red_team.md`; campaign `scripts/metal-campaign.sh:1919-1926`.

**New abstraction:** distinguish **ESTOP** (`engaged`, recoverable by recover on a live session that is not `hardware_session_dead` / `bus_lost`) from **integrity abort** (independent `integrity_aborted` fact set only through `latch_abort()`, covering unknown physical outcome, replay/`LATCHING_PREFIXES`, release/binding mismatch, time rollback, and existing identity/journal continuity integrity). Do **not** encode this as a single `AbortClass` enum whose `engage()` assignment can overwrite Integrity. Recover must refuse with `integrity_abort_requires_online_restart` **before** `Plant::clear_estop`. Do not add a new crate. Production `op=recover` is still an untrusted ESTOP-ack analog.

**Why more general:** operator ack is a role, not a JSON op available to every IPC peer. Matches HIL production (already refuses recover) and `PHILOSOPHY.md` (“integrity failures do latch”).

**Migration:** keep `op=recover` for campaign ESTOP. Change `EstopLatch::clear` / `clear_estop_requires_recovery_at` so integrity reasons stay latched. PTY tests that recover after bus-loss still refuse via `bus_lost` / `hardware_session_dead`. Add tests equivalent to `rt1`/`rt2` **in the metal/governor tree**, not by merging the whole adversarial crate.

**Tests:** see implementation plan. Expected fail: recover after unknown_outcome currently returns ok and a new id writes.

**Deletion:** none required. Do not delete campaign recover.

---

## 15. Implementation plan pointer

Coding-agent tasks for 13.1: `docs/superpowers/plans/2026-09-18-untrusted-recover-integrity-latch.md`.

Operator tasks for 12: same plan, Part 0. Do not start Part 1 on `main` or on `2f68a5d`.

---

## 16. Final judgment

**What this repository is uniquely becoming:** a typed last-gate that will not let a learned system, a planner, or a sibling crate own motors, and that refuses to call simulation metal.

**What prevents it from becoming much stronger:** no measured XL330 proof; a composition hole where autonomy can unlatch integrity abort; a second SIM-OS (verify/semantics) that still eats privileged truth.

**Overengineering:** authority typestate theater beyond the choke point (lifecycle wrappers, unused `SensorModel`, skill verb enum, gauntlet/vision/rate as product surface); further kernel rewrites without Gate E.

**Underengineering:** recover vs ESTOP role split; journaled abort; one robot load path; policy/privileged split; CI runners actually executing.

**Single achievement that most increases credibility:** `docs/metal_proof.json` from `2f68a5d` on a real XL330 with live VIN and live unplug.

**Next month:** run that campaign on a Linux bench. If the bench is unavailable, either wait or do **only** the recover/latch fix on a research worktree. Do not expand skills, HALs, or product APIs.

---

## Spec self-review

- No TBD/TODO left in the milestone definition.
- Metal SHA is `2f68a5d`, not `e360649` (plan3 superseded) and not `c82c78c`.
- Recover fix is explicitly **not** the metal SHA.
- Probe is documented as status, not a sensing skill — no implementation of Gate I in this milestone.
- Scope is one immediate milestone (measure) plus one optional software follow-up (recover). Not a platform rewrite.
