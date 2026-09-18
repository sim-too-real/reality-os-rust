# Typed physical-decision model — investigation only

**Date:** 2026-09-18
**Status:** investigation / experiment design. **No model was trained. No `realityos-decision` crate was added.**
**Not Jev. Not RLCD.** Jev is cited only as evidence that non-generative typed decision models exist as a system category.

Question answered from Reality OS artifacts, not from architecture fashion:

> Does Reality OS contain repeated probabilistic physical judgments where deterministic rules are too brittle but open-ended generative models are unnecessarily unsafe, slow, or unconstrained?

Short answer: **yes, in manipulation failure staging** — contact/acquisition metrics diverge from verified task success across embodiments, while authority refusals are not the bottleneck (`unauthorized_writes: 0`). Rules already name some failures (`ManipulationFailure`, episode `failure_taxonomy`) and miss the contact-established-but-task-failed cases. A learned model is **not** justified until baselines on a held-out embodiment beat those rules on calibration and transfer. Prefer no model over an unjustified model.

---

## Hard safety boundary (applies to any future learned component)

A learned Reality model may **never** directly:

- construct `OnlineWrite`
- produce `IssuedCommand`
- call `Plant::act`
- acquire hardware authority
- clear ESTOP
- clear integrity abort
- bypass capability requirements
- declare MEASURED evidence
- upgrade UNKNOWN to known without evidence
- widen a safety envelope
- invent actuator state
- invent calibration
- assert physical identity
- declare a physical task successful without verifier evidence

It may initially only classify, score, rank, diagnose, identify missing evidence, recommend PROBE, recommend REFUSE, and estimate uncertainty.

If later allowed to influence execution, influence may only **narrow** capability, request information, or select among options independently proven safe. No learned component gets last-write authority.

Existing type hook: `Provenance::LearnedEstimate` in `crates/semantics/src/provenance.rs` is already distinct from `HardwareMeasured` / `SimulatorDerived` / `Unknown`. Predictions must stay in that bucket.

---

## A. Decision-task inventory

Every plausible typed-judgment problem currently present in this repository.

### A1. Manipulation earliest-failure-stage / failure diagnosis

- **Input:** policy-visible episode traces (skill contract, commands issued, resource topology, evidence_used, authority_decisions, expected_refusal, ctrl_writes, coarse contact presence if treated as a sensor analog) plus optional privileged simulator fields for *labels only*.
- **Answer space:** bounded enum extending `crates/semantics/src/failure.rs` `ManipulationFailure` (MISS, UNREACHABLE, STALE_OBJECT, GRIPPER_EMPTY_CLOSE, SLIP, RESOURCE_UNSUPPORTED, CONTROLLER_FAILURE, …) plus explicit stages the current taxonomy does not isolate well: `ContactNotEstablished`, `ContactEstablishedTaskFailed`, `GraspAcquiredHoldUnverified`, `VerifierEvidenceInsufficient`, `AuthorityRefusal`, `Unknown`.
- **Existing labels:** episode `failure_taxonomy` (rule-generated; examples in `docs/superpowers/evidence/manipulation_panda_grasp.json`: `UNREACHABLE`, `STALE_OBJECT`, `GRIPPER_EMPTY_CLOSE`, `RESOURCE_UNSUPPORTED`, `CONTROLLER_FAILURE`, or `null` on many executed episodes); `task_result` ∈ {success, fail, refuse}; metric counters `grasp_acquisition_success` vs `grasp_verified_hold_success`, `push_contact_establishment` vs `push_task_success`.
- **Data volume:** Panda 700 episodes (100 release / 300 grasp / 300 push); arm_gripper 700; UR5e 300 push; iiwa14 300 push; wx250s holdout 60 (20 each skill). Contact/command traces are per-episode in the JSON artifacts.
- **Safety impact:** diagnostic only if the model cannot write. Mis-diagnosis could later cause a bad PROBE recommendation; it must not authorize motion.
- **Rule baseline:** `ManipulationFailure::from_refuse` maps `SkillRefuse` deterministically; verify already emits `failure_taxonomy` and `expected_refusal`. Rules do **not** explain Panda grasp 220/300 acquisition vs 12/300 verified hold, or wx250s push 14/20 contact vs 0/20 task success.
- **Value if solved:** operators and the verify loop would know *which physical stage* failed instead of a binary task_result. That is the only candidate with both volume and a product gap.

### A2. Evidence sufficiency / CanAttempt

- **Input:** `SkillContract`, WorldState, SensorModel, Provenance, CapabilityGraph (`crates/semantics/src/skill.rs`, `world.rs`, `provenance.rs`).
- **Answer space:** `CanAttempt = { Yes, No, Unknown }` and `MissingEvidence` (SurfaceNormal, ObjectPose, ContactState, GripperState, FrictionEstimate, Calibration, Other) plus confidence.
- **Existing labels:** `SkillRefuse::StaleEvidence`, `StaleObject`, `StaleGripperState`, `ForceBoundUnavailable`, `MissingTarget`, `MissingJointState`; REACH tests `missing_target_is_probe_path` / `missing_joint_state_is_probe` (`crates/semantics/src/reach.rs`).
- **Data volume:** refusal traces inside verify/semantics tests plus expected_refusals in holdout JSON (wx250s grasp 16/20 expected refusals; Panda grasp 74/300).
- **Safety impact:** kernel must remain the legality authority. A model saying Yes cannot mint a write.
- **Rule baseline:** already strong — probe/refuse is a typed verdict (`SkillRefuse::Probe`), not a sensing skill. CLAIM_LEDGER: “Five-word verdicts allow/modify/probe/refuse/abort; probe is not abort.”
- **Value if solved:** low until rules miss a sufficiency class that still executes. Today the miss is *after* attempt, not before.

### A3. PROBE selection

- **Input:** missing evidence, available sensors, risk envelope, candidate observation actions.
- **Answer space:** `NoProbe | ObserveAgain | ChangeViewpoint | TouchProbe | Recalibrate | Refuse` plus probabilities.
- **Existing labels:** `SkillRefuse::Probe`; observe/look_at skill names exist; no ranked probe dataset.
- **Data volume:** insufficient. Probe is a verdict, not a logged alternative set.
- **Safety impact:** TouchProbe would be a motion. Ranking must not execute.
- **Rule baseline:** missing evidence → PROBE. No competing probe catalog.
- **Value if solved:** none until a probe catalog and labels exist.

### A4. Skill / capability routing

- **Input:** goal, world state, available capabilities, skill contracts, embodiment.
- **Answer space:** ranking over an *explicit* caller-supplied skill set (`SkillName` in `skill.rs`: observe, reach, grasp, release, push, …). No generated skill names.
- **Existing labels:** which `skill_contract` was run (`skill.grasp` / `skill.push` / `skill.release`); verify chooses the skill, it is not a learned choice.
- **Data volume:** the traces are conditioned on the skill already chosen — they do not label “which skill should have been selected.”
- **Safety impact:** routing into PLACE/VLA would expand the product without a second real actuator (plan4 non-goal).
- **Rule baseline:** skill is selected by the verify/campaign harness.
- **Value if solved:** low; would mostly memorize the harness schedule.

### A5. Manipulation risk estimates (contact / slip / displacement / hold)

- **Input:** same traces as A1, possibly with privileged contact `fn`/`ft` as training labels only.
- **Answer space:** calibrated probabilities: contact will establish; object will slip; push displacement succeeds; grasp has acquired; verifier evidence is insufficient.
- **Existing labels:** metric splits above; `grasp_slip` is 0 in every checked artifact (Panda, wx250s, arm_gripper, UR5e, iiwa14) — slip is not currently a useful label.
- **Data volume:** same as A1. Contact-vs-task is the informative split (Panda push 252/300 contact, 107/300 task; UR5e 259/300 contact, 188/300 task; iiwa14 249/300 contact, 252/300 task; arm_gripper 246/300 contact, 14/300 task).
- **Safety impact:** scoring only.
- **Rule baseline:** none for “contact established ⇒ task success.”
- **Value if solved:** high as a *head* on A1, not as a first standalone product.

### A6. Authority / recover / integrity classification

- **Input:** governor traces, latch facts, journal events.
- **Answer space:** recoverable ESTOP vs integrity abort vs identity-dead vs bus-lost.
- **Existing labels:** now a type fact (`EstopLatch::is_integrity_abort`), not a judgment. Tokens `integrity_abort_requires_online_restart`, `hardware_session_requires_online_restart`.
- **Data volume:** unit tests + metal PTY (Linux) + campaign. Not an ML dataset.
- **Safety impact:** catastrophic if a model could clear ESTOP/integrity or construct `OnlineWrite`.
- **Rule baseline:** this milestone’s latch. Deterministic.
- **Value if solved:** **negative.** Do not put a model on the authority path.

### A7. Embodiment / resource qualification

- **Input:** Menagerie XML, resource discovery (`resource_topology`, `semantic_resource`).
- **Answer space:** supported vs unsupported gripper/coupling; required world features.
- **Existing labels:** wx250s `conclusion: "MANIPULATION GENERALIZATION LIMIT FOUND"`; `RESOURCE_UNSUPPORTED`; plan4 notes coupled gripper never qualified.
- **Data volume:** small number of embodiments (Panda, UR5e, iiwa14, wx250s, arm_gripper).
- **Safety impact:** wrong qualification could skip a required refuse.
- **Rule baseline:** resource discovery already runs; failures are structural (topology), not fuzzy.
- **Value if solved:** a table beats a model at this n.

### A8. REACH target feasibility

- **Input:** REACH matrices in `docs/superpowers/evidence/external_holdout_*.json`.
- **Answer space:** reachable / unreachable / probe-missing-target.
- **Existing labels:** per-target REACH rows; `SkillRefuse::Unreachable`.
- **Data volume:** holdout JSON REACH blocks (Panda postfix, iiwa v2, UR5e matrix).
- **Safety impact:** low if it only ranks targets the kinematics already screens.
- **Rule baseline:** kinematics + missing-target probe. Likely sufficient.
- **Value if solved:** only if a held-out body systematically disagrees with the IK/rule screen; not observed as the primary gap.

---

## B. Recommended first model experiment

**Exactly one:** **A1 — earliest failure-stage classification** on policy-visible manipulation episodes.

Why this one, from repository evidence, not from Jev:

1. **Product gap is measured.** wx250s holdout (`manipulation_holdout_first_score.json`): release 0/20, grasp acquisition 2/20, push task 0/20 with 14/20 contact. Panda (`manipulation_panda.json`): grasp acquisition 220/300 vs verified hold 12/300; push contact 252/300 vs task 107/300. arm_gripper: grasp verified hold 0/300 after 88/300 acquisition; push 14/300 task after 246/300 contact. Authority is not the failure (`unauthorized_writes: 0` everywhere cited).
2. **Bounded outputs already exist** (`ManipulationFailure`, `failure_taxonomy`, `task_result`) and can be extended with an earliest-stage enum without generating text.
3. **Cross-embodiment evaluation is already the house rule:** train on Panda + development bodies, hold out wx250s (frozen first-score; do not rewrite that JSON).
4. **No actuator-authority implications** if the model only classifies. `IssuedCommand` / `OnlineWrite` stay kernel-minted.
5. Rejected alternatives: A2/A3 are already typed probe/refuse; A4 has no counterfactual skill labels; A6 would be a safety regression; A7 has n≈5 embodiments.

Do **not** train in this goal. Next step after this note, if authorized, is a dataset/evaluation harness and rule/statistical baselines — not a neural architecture.

---

## C. Dataset specification

Working name: `EarliestFailureEpisode` (name is local to this note).

### Schema (conceptual)

```text
episode_id
skill: {grasp, push, release, reach}     # from skill_contract; not a free string
embodiment_features: {topology, dof, gripper_class}  # NOT robot_id / vendor name
world_features_policy: {n_objects, object_size_bins, support_present}
evidence_features_policy: {evidence_used[], expected_refusal, ctrl_writes,
                           n_commands, command_kinds[], authority_decision_kinds[]}
outcome: {task_result, failure_taxonomy_rule}
label_earliest_stage: enum               # derived; see below
provenance_split: {policy_visible, privileged_label, rule_generated}
```

### Policy-visible fields (may enter a runtime model)

- `skill_contract`, command `kind` sequence, `ctrl_writes`, `expected_refusal`
- `authority_decisions` kinds (allow/refuse/probe) — not payloads that could be forged into writes
- `evidence_used`, `resource_topology` class (e.g. TendonDrivenGripper vs Coupler), `semantic_resource` class
- `task_result` is an *outcome label*, not a runtime input
- embodiment graph structure (joint/link counts, gripper class) without `robot_id` / `"menagerie_panda"` / vendor strings

### Privileged simulator truth (labels / evaluation only; must not silently enter policy input)

- `object_definitions.{mass, friction, pos, size}` (`PERFECT_PERCEPTION`)
- MuJoCo contact `fn`/`ft` and exact body names
- full `joint_state` as simulator oracle (a future real encoder stream would be policy-visible *measured*)
- `world_seed`, `model_hash` as identity (exclude from features; use only for split hygiene)
- `perception: PERFECT_PERCEPTION` flag — if present, pose is privileged

### Derived labels

- Rule-generated: existing `failure_taxonomy`
- Derived from metrics/trace: `ContactEstablishedTaskFailed` when push contacts exist and `task_result=fail`; `GraspAcquiredHoldUnverified` when acquisition counter would fire but verified hold does not (episode-level analog of the 220 vs 12 split)
- Human labels: none in-repo
- Physical measured labels: none (`evidence_status: SIMULATION_ONLY`; no `docs/metal_proof.json`)

### Split

- Train: Panda + arm_gripper (+ UR5e/iiwa push if the answer space is shared)
- Validation: different seeds/scenes of the same train embodiments
- Holdout: **entire** wx250s (`manipulation_holdout_first_score.json`). Do not rewrite that artifact. Do not one-hot `robot_id`.

### Generation opportunity (harness later, not this goal)

Randomize pose, dimensions, mass, friction, support, gripper, gains, contact approach, sensor noise, missing/delayed observations, calibration offsets, embodiment topology. Record success/failure and earliest causal stage. Useful even if no ML model is trained. Do not encode robot names as causal features.

---

## D. Baselines

In this order. Do not jump to transformers.

1. **Rules:** `ManipulationFailure::from_refuse` + existing `failure_taxonomy`. Map `task_result=refuse` to the refuse code; map executed-fail with empty contacts to `ContactNotEstablished`; map executed-fail with contacts to `ContactEstablishedTaskFailed`; map grasp acquisition-without-hold analog if reconstructible from the episode. This is the bar a model must beat.
2. **Simple statistical:** per-skill empirical stage frequencies on train embodiments; majority class; optional logistic regression on a handful of counts (`ctrl_writes`, `n_contacts>0`, `expected_refusal`, gripper class). Calibrate with temperature scaling.
3. **Small ML:** gradient-boosted trees or a ≤2 MB MLP on the policy-visible numeric/categorical vector. Same holdout protocol. No robot-name features.

Only if (3) beats (1) and (2) on holdout Brier/ECE *and* does not collapse to embodiment identity, consider a small encoder. Custom neural architecture is out of scope until that happens.

---

## E. Generalization protocol

- Never evaluate only on random train/test episode splits of the same robot.
- Hold out an entire embodiment (wx250s first-score). Optional second holdout: a later real XL330 set, labeled SIMULATION_ONLY vs MEASURED honestly.
- Primary question: did the model learn physical decision structure or memorize embodiment identity?
- Exclude robot IDs and vendor names from inputs unless a specific experiment demonstrates a legitimate reason (default: exclude).
- Report train embodiment accuracy *and* holdout accuracy; a large gap is a kill signal, not a tuning target.

---

## F. Calibration protocol

If probabilities are used, they must mean something. Measure:

- accuracy, precision/recall per stage
- Brier score, expected calibration error, reliability diagrams
- selective risk and coverage-vs-error (abstention)
- OOD: wx250s and any later unseen gripper class
- cross-robot transfer

The model must be able to say UNKNOWN / low confidence. A 95% accurate model that is confidently wrong on the remaining 5% is less useful than a weaker calibrated one. Use proper scoring rules; temperature scaling / isotonic / conformal prediction are evaluation tools, not a claim of “RLCD.”

---

## G. Runtime contract

Conceptual API (do not adopt names blindly; do not ship this crate now):

```text
DecisionQuery<State, AnswerSpace> → Decision<Answer> {
    answer, probability, confidence, provenance, model_version
}
```

Answer space is a Rust enum supplied by the caller. The model cannot construct a value outside that enum. UNKNOWN/abstention is first-class.

**May:** classify, score, rank, diagnose, identify missing evidence, recommend PROBE, recommend REFUSE, estimate uncertainty. Predictions are `Provenance::LearnedEstimate` with `model_hash`, `training_dataset_hash`, `calibration_version`, probability, OOD/abstention.

**Must not:** construct `OnlineWrite`; produce `IssuedCommand`; call `Plant::act`; acquire hardware authority; clear ESTOP; clear integrity abort; bypass capabilities; declare MEASURED; upgrade UNKNOWN without evidence; widen envelopes; invent actuator/calibration/identity; declare task success without verifier evidence; last-write authority.

If later allowed to influence execution: only narrow capability, request information, or select among independently proven-safe options. The deterministic kernel remains responsible for whether execution is legal.

---

## H. Kill criteria

Abandon a custom in-house model if any of:

- rules (baseline 1) match small-ML holdout Brier/ECE within a small margin
- wx250s (or a later untouched embodiment) transfer is near chance after robot-id ablation
- probabilities cannot be calibrated (ECE stays high; selective risk does not improve with abstention)
- training labels are circular (rule taxonomy in, rule taxonomy out) with no extra causal stage recovered
- `grasp_slip` remains 0 and the model invents slip
- latency or footprint exceeds the verify-loop value of a confusion matrix
- the model’s top features are embodiment identity / `model_hash` / vendor strings
- any prototype attempts to write, clear ESTOP/integrity, or declare MEASURED
- no measurable change in how verify/debug decides the next experiment

---

## I. Go criteria

Move from this investigation toward a real `realityos-decision` component only if **all** of:

1. A dataset/eval harness exists that reconstructs earliest-stage labels without leaking privileged fields into policy input.
2. Rules, statistical, and small-ML baselines are reported on Panda/arm_gripper train and **frozen wx250s holdout**.
3. Small ML improves holdout Brier (or selective risk at fixed coverage) over rules **and** over the statistical baseline, with ECE not worse.
4. Robot-id ablation does not explain the gain (same holdout after stripping identity).
5. Runtime wiring is evidence-only (`LearnedEstimate`); compile-fail or review proves it cannot construct `OnlineWrite` / `IssuedCommand` / `Plant::act`.
6. No metal MEASURED claim is attached to the model. Physical transfer is a later experiment with its own ledger row.

Until those hold: **do not train, do not add a crate, do not copy Jev/RLCD.**

---

## Evidence cited

| Artifact | What it shows |
|---|---|
| `docs/superpowers/evidence/manipulation_holdout_first_score.json` | wx250s: release 0/20, grasp acq 2/20, push contact 14/20 task 0/20, unauthorized_writes 0 |
| `docs/superpowers/evidence/manipulation_panda.json` (+ grasp/push/release JSON) | 220/300 acq vs 12/300 verified hold; 252/300 contact vs 107/300 push task |
| `docs/superpowers/evidence/manipulation_arm_gripper.json` | 88/300 acq vs 0/300 verified hold; 246/300 contact vs 14/300 push task |
| `docs/superpowers/evidence/manipulation_ur5e_push.json` | 259/300 contact, 188/300 task |
| `docs/superpowers/evidence/manipulation_iiwa14_push.json` | 249/300 contact, 252/300 task |
| `crates/semantics/src/failure.rs` | deterministic `ManipulationFailure`; every variant `writes_allowed = false` |
| `crates/semantics/src/skill.rs` | `SkillRefuse::Probe` is a verdict; `SkillName` is a closed enum |
| `crates/semantics/src/provenance.rs` | `LearnedEstimate` already distinct from measured/sim/unknown |
| `CLAIM_LEDGER.md` | probe is not abort; HonestyStamp cannot construct metal |
| this branch’s governor tests | integrity abort is a type fact, not a place for a classifier |

**Not trained:** no checkpoints, no new learned crate, no MuJoCo counterexample generator in this goal.
