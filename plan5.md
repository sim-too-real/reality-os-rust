Work exclusively in the current `reality-os-rust` repository.

Do not use TheWorld or any stale predecessor repository.

This task has TWO objectives:

1. Correct and implement the narrowly scoped recovery/integrity defect discovered in the current authority kernel.
2. Investigate whether Reality OS should eventually contain a small in-house, Jev-like typed decision model specialized for physical intelligence.

The first objective is an immediate correctness task.

The second objective is an investigation and experiment-design task only.

Do not train a model merely because the idea sounds attractive.

---

# PART A — FIX THE RECOVERY / INTEGRITY MODEL CORRECTLY

Current production behavior contains a real authority defect:

```text
integrity abort
→ autonomy sends op=recover
→ authority supplies operator_ack=true
→ plant.clear_estop(...)
→ EstopLatch::clear()
→ abort_latched cleared
→ fresh command_id can potentially execute
```

This violates the intended fail-closed ONLINE-instance invariant.

The existing proposed `AbortClass { None, Estop, Integrity }` design also has two additional hazards that must be fixed before implementation.

---

## INVARIANT A1 — INTEGRITY IS MONOTONIC

An ONLINE instance that has entered an integrity-aborted state must NEVER be downgraded back into a recoverable ESTOP by a later watchdog, ESTOP, identity event, journal event, or another call to `engage()`.

Conceptually:

```text
None < RecoverableEstop < IntegrityAbort
```

Transitions may move right.

They must not move left during the same ONLINE instance.

Specifically this sequence must remain permanently integrity-aborted:

```text
unknown_outcome
→ integrity abort
→ watchdog / engage_estop
→ recover
```

`recover` must still fail.

Do not implement a state representation in which:

```rust
latch_abort() -> Integrity
engage()     -> Estop
```

can overwrite the stronger state.

Prefer representing independent facts if that produces stronger semantics:

```rust
estop_engaged: bool
integrity_aborted: bool
```

rather than forcing fundamentally different conditions into one mutable enum.

Derive the minimal representation from the current code.

Do not perform a broad latch rewrite.

---

# INVARIANT A2 — FORBIDDEN RECOVERY HAS ZERO PHYSICAL SIDE EFFECTS

Current recovery performs:

```rust
plant.clear_estop(true)?;
latch.clear();
```

It is insufficient for software to preserve `abort_latched` AFTER calling `plant.clear_estop()`.

For an integrity-aborted instance:

```text
op=recover
```

must be refused BEFORE touching the plant.

Required behavior:

```text
integrity_aborted == true
        ↓
recover()
        ↓
recovery_refused:
integrity_abort_requires_online_restart
        ↓
Plant::clear_estop NOT called
        ↓
software latch unchanged
        ↓
zero new physical writes
```

Add instrumentation/test support if needed to prove that `clear_estop` was never invoked.

Do not merely assert that subsequent motor writes remain zero.

The recovery attempt itself must have zero recovery side effects.

---

# INVARIANT A3 — ONE SEMANTIC PATH CREATES INTEGRITY ABORTS

Search the entire workspace for:

```text
abort_latched =
reason =
latch_abort(
```

Identify every direct mutation of the integrity state.

No production path should be able to produce:

```text
abort_latched = true
integrity_classification = false
```

because a forgotten direct assignment would make the recovery policy unsound.

Route every genuine integrity-abort transition through one authoritative API.

Examples include, where semantically applicable:

- unknown physical outcome
- replay
- release / binding integrity mismatch
- time rollback
- identity continuity violation
- repeated identity failures
- journal/continuity integrity violation

Do NOT blindly classify every ESTOP as an integrity failure.

Maintain the distinction between:

```text
recoverable operational ESTOP
```

and:

```text
current ONLINE instance is no longer trustworthy
```

---

# INVARIANT A4 — OPERATOR RECOVERY CLAIM MUST REMAIN HONEST

Production metal currently accepts:

```text
op=recover
```

from the IPC surface and internally invokes:

```rust
clear_estop_requires_recovery_now(true)
```

The caller does not currently present a separately authenticated operator acknowledgement.

The metal campaign itself invokes recover through the autonomy user.

Therefore do NOT describe the current mechanism as a cryptographically or OS-authenticated operator recovery channel.

Call it what it actually is.

For this milestone:

- preserve the campaign's ability to recover journaled ESTOP after a legitimate new-process `--restart`
- prevent integrity abort from being cleared
- do not build a large authentication system
- do not redesign IPC

But record the architectural hole:

```text
future production recovery should have an authority/operator trust boundary
separate from ordinary autonomy IPC
```

Possible future directions to evaluate, NOT implement unless required:

```text
separate operator Unix socket
separate OS group
root/operator control fd
one-shot recovery capability
authenticated recovery token
physical/operator control surface
```

This is not required for the first metal experiment unless current tests prove it is necessary.

---

# REQUIRED REGRESSION TESTS

Tests first.

At minimum demonstrate:

### Recoverable ESTOP

```text
ordinary recoverable ESTOP
→ authorized recovery
→ ESTOP clears
→ integrity state false
→ subsequent legitimate command may execute
```

### Integrity abort

```text
unknown_outcome
→ integrity abort
→ recover
→ refused
→ integrity still latched
→ Plant::clear_estop call count unchanged
→ fresh command_id cannot execute
```

### Replay

```text
execute command_id X
→ replay X
→ integrity abort
→ recover
→ refused
→ fresh command_id Y still cannot execute
→ physical write count unchanged
```

### Downgrade attack

```text
integrity abort
→ later engage ESTOP/watchdog
→ recover
→ integrity MUST remain latched
→ zero plant recovery side effects
```

### Multiple integrity sources

Do not make the implementation depend specifically on the string:

```text
"unknown_outcome"
```

Exercise more than one integrity origin.

The security property belongs to the state type, not a reason-string special case.

---

# FIRST METAL BASELINE

Preserve:

```text
2f68a5d82fec5e7e2c0b78ca88fe65d8a8a4acfc
```

as an immutable historical/pre-fix metal baseline.

Do NOT rewrite it.

The first real XL330 run against this SHA remains scientifically useful.

However, because the recovery defect is already known, classify the resulting evidence honestly as:

```text
pre-fix frozen physical baseline
```

rather than treating it as the final authority milestone.

Then run the same complete experiment on the corrected SHA.

The corrected campaign must additionally contain the hostile recovery cases above.

The patched SHA becomes the serious first physical authority candidate only if:

- legitimate hold/nudge works
- unauthorized physical writes remain zero
- crash/replay passes
- live USB removal passes
- VIN cutoff passes
- identity behavior passes
- integrity recovery attack fails closed
- downgrade attack fails closed
- forbidden recover produces zero plant recovery side effects

Never manually write `docs/metal_proof.json`.

---

# DO NOT EXPAND AUTHORITY AFTER THIS WITHOUT EVIDENCE

If this defect is closed and physical metal succeeds:

FREEZE the authority architecture again.

Do not perform another authority-kernel architecture pass merely to make the code look cleaner.

Only reopen it when a concrete physical experiment, counterexample, security attack, or correctness failure demands it.

---

# PART B — INVESTIGATE A REALITY-NATIVE TYPED DECISION MODEL

Investigate whether Reality OS would benefit from an internal model inspired by the DESIGN PRINCIPLE behind systems such as TypeSafe AI Jev:

```text
state
+
predeclared questions
        ↓
typed decisions
+
probabilities / confidence
```

Do NOT attempt to copy Jev's architecture or claim that we reproduce RLCD.

Treat Jev only as evidence that non-generative typed decision models are a potentially useful system category.

The question is:

> Does Reality OS contain repeated probabilistic physical judgments where deterministic rules are too brittle but open-ended generative models are unnecessarily unsafe, slow, or unconstrained?

Answer from actual Reality OS workflows and evidence.

---

# HARD SAFETY BOUNDARY FOR ANY LEARNED MODEL

A learned Reality model may NEVER directly:

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

A model may initially only:

```text
classify
score
rank
diagnose
identify missing evidence
recommend PROBE
recommend REFUSE
estimate uncertainty
```

If eventually allowed to influence execution:

```text
model influence may only NARROW capability,
request information,
or select among options independently proven safe.
```

No learned component gets last-write authority.

---

# SEARCH FOR DECISION-SHAPED PROBLEMS

Inspect current Reality OS and identify all places resembling:

```text
large set of state/evidence
        ↓
fuzzy judgment
        ↓
small bounded answer space
```

Candidate areas to investigate include, but are not limited to:

### Failure diagnosis

Input:

```text
execution traces
contact observations
world state
capability state
controller output
verification results
```

Output:

```rust
enum FailureCause {
    MissingEvidence,
    Reachability,
    ApproachGeometry,
    ContactNotEstablished,
    ContactLost,
    FrictionMismatch,
    ControllerMismatch,
    GraspAcquisitionFailure,
    Slip,
    VerificationInsufficient,
    AuthorityRefusal,
    Unknown,
}
```

with probabilities.

---

### Evidence sufficiency

Input:

```text
SkillContract
WorldState
SensorModel
Provenance
CapabilityGraph
```

Outputs such as:

```text
CanAttempt = { Yes, No, Unknown }

MissingEvidence =
    SurfaceNormal |
    ObjectPose |
    ContactState |
    GripperState |
    FrictionEstimate |
    Calibration |
    Other

confidence
```

The deterministic kernel remains responsible for whether execution is actually legal.

---

### PROBE selection

Input:

```text
missing evidence
available sensors
risk envelope
candidate observation actions
```

Output:

```text
NoProbe
ObserveAgain
ChangeViewpoint
TouchProbe
Recalibrate
Refuse
```

plus probabilities.

The model may rank candidate probes but may not execute one unless the ordinary Reality OS authority path certifies it.

---

### Skill / capability routing

Input:

```text
goal
world state
available capabilities
skill contracts
embodiment
```

Output:

```text
candidate skill ranking
confidence
```

It must select only from an explicit set supplied by code.

No generated skill names.

---

### Manipulation risk

Examples:

```text
likelihood contact will establish
likelihood object will slip
likelihood push direction will achieve displacement
likelihood grasp has genuinely acquired object
likelihood verifier evidence is insufficient
```

These are potentially far more useful than generating control commands.

---

# IMPORTANT QUESTION: DO WE NEED A NEURAL MODEL AT ALL?

For every candidate decision task, establish progressively stronger baselines:

```text
1. deterministic rules
2. simple statistical baseline
3. logistic regression / calibrated linear model
4. gradient-boosted trees
5. small MLP
6. small encoder model
7. custom neural architecture
```

Do not jump directly to transformers.

If a 2 MB model performs as well as a 200 MB model, prefer the 2 MB model.

If deterministic rules are superior, use deterministic rules.

The point is capability, not saying Reality OS has its own AI model.

---

# DATA INVENTORY

Determine what labeled data already exists or can be generated from:

- REACH matrices
- PUSH episodes
- GRASP episodes
- RELEASE episodes
- wx250s holdout
- Panda
- UR5e
- iiwa14
- arm_gripper
- manipulation traces
- authority traces
- refusals
- contact observations
- MuJoCo privileged verification
- capability graphs
- embodiment descriptors
- future XL330 evidence

Determine which labels are trustworthy.

Explicitly distinguish:

```text
policy-visible evidence
privileged simulator truth
derived labels
human labels
rule-generated labels
physical measured labels
```

Privileged simulator truth may label training/evaluation data.

It must not silently enter policy input at runtime.

---

# DATA GENERATION OPPORTUNITY

Investigate whether MuJoCo can cheaply generate a large counterexample dataset.

Instead of merely running successful benchmark episodes, deliberately randomize:

- object pose
- dimensions
- mass
- friction
- support geometry
- gripper geometry
- controller gains
- contact approach
- sensor noise
- actuator limits
- missing observations
- delayed observations
- calibration offsets
- embodiment topology

Generate both:

```text
success
failure
```

and record the earliest causal failure stage.

The dataset should be useful even if no ML model is ultimately trained.

A strong target schema might resemble:

```text
EpisodeState
EmbodimentFeatures
WorldFeatures
EvidenceFeatures
Skill
ActionCandidate
Outcome
FailureStage
MissingEvidence
VerifierResult
Provenance
```

Do not encode robot names as causal features.

---

# GENERALIZATION REQUIREMENT

Never evaluate only on random train/test episode splits from the same robot.

Use structural holdouts:

```text
train:
Panda + development embodiments

validation:
different scenes/configurations

holdout:
WX250s or another untouched embodiment
```

Eventually:

```text
simulation-trained
        ↓
real-hardware evaluation
```

The primary question is:

> Did the model learn physical decision structure or memorize embodiment identity?

Exclude robot IDs/vendor names from model inputs unless an experiment specifically demonstrates a legitimate reason.

---

# CALIBRATION IS MORE IMPORTANT THAN RAW ACCURACY

If Reality OS uses probabilities, measure whether they mean anything.

Evaluate:

```text
accuracy
precision / recall
Brier score
expected calibration error
reliability diagrams
selective risk
coverage vs error
OOD behavior
abstention behavior
cross-robot transfer
```

The model MUST be able to say:

```text
UNKNOWN / low confidence
```

A model that is 95% accurate but confidently wrong on the remaining 5% may be less useful than a weaker calibrated model.

---

# POSSIBLE REALITY DECISION MODEL API

Investigate an interface conceptually like:

```rust
DecisionQuery<State, AnswerSpace>
        ↓
Decision<Answer> {
    answer,
    probability,
    confidence,
    provenance,
    model_version,
}
```

or:

```rust
FailureDiagnosis
EvidenceAssessment
ProbeRanking
SkillRanking
RiskEstimate
```

Do not adopt these names blindly.

The output space must be represented by Rust types supplied by the calling code.

Unknown/abstention must be first-class.

The model cannot create a value outside the declared answer space.

---

# MODEL SIZE / ARCHITECTURE INVESTIGATION

Only if the data demonstrates a need, investigate a small in-house model.

Do not assume an LLM architecture.

Compare:

### Typed numerical / graph state

Potential:

```text
MLP
DeepSets
small graph neural network
small transformer encoder
```

### Mixed structured + textual metadata

Potential:

```text
small pretrained encoder
+
typed classification heads
```

### Embodiment graph

Potential:

```text
graph encoder
joint/link/resource nodes
+
world/contact nodes
+
typed decision heads
```

An interesting longer-term hypothesis to evaluate is:

```text
Embodiment graph
+
WorldState
+
SkillContract
+
Evidence
        ↓
shared physical-state encoder
        ↓
many independent typed heads
```

For example:

```text
CanExecute
MissingEvidence
FailureCause
ContactLikelihood
SlipLikelihood
SkillSuitability
ProbeValue
```

This is much closer to a Reality-native System-One model than training a small chatbot.

Do not build it until simpler baselines justify it.

---

# TRAINING PRINCIPLE

If eventually training an in-house model, optimize for:

```text
correctness
calibration
abstention
cross-embodiment generality
low latency
small footprint
reproducibility
```

not prose quality.

Potential training objective should support calibrated bounded decisions.

Research appropriate objectives such as:

```text
cross entropy
Brier loss
proper scoring rules
temperature scaling
isotonic calibration
selective prediction
conformal prediction
```

Do not invent the label “RLCD” for our work unless we independently implement and define such a method.

---

# MODEL OUTPUT IS EVIDENCE, NOT TRUTH

Any learned prediction becomes evidence with provenance such as:

```text
source = learned_model
model_hash
training_dataset_hash
calibration_version
prediction_probability
OOD/abstention state
```

Reality OS must be able to distinguish:

```text
measured physical observation
deterministic derivation
simulator privileged truth
learned prediction
human declaration
```

These must never collapse into one generic `known=true`.

---

# FIRST EXPERIMENT TO RECOMMEND

Do NOT propose full model development.

Identify ONE decision task having:

- existing data
- clear bounded outputs
- trustworthy labels
- actual current product value
- cross-robot evaluation
- no actuator-authority implications

Strongly consider:

```text
manipulation failure diagnosis
```

because Reality OS already has counterexamples where:

```text
contact established
but push failed

grasp acquisition triggered
but verified hold failed
```

A candidate first experiment is:

```text
Input:
policy-visible episode state + execution evidence

Output:
typed earliest-failure-stage distribution

Baseline:
existing deterministic taxonomy

Evaluation:
development robots vs frozen wx250s holdout

Question:
does learned classification improve causal diagnosis
without robot-specific identity?
```

But derive the choice from repository evidence.

---

# REQUIRED OUTPUT

Before coding anything for Part B, produce:

## A. Decision-task inventory

Every plausible typed-judgment problem currently present in Reality OS.

For each:

```text
input
answer space
existing labels
data volume
safety impact
rule baseline
value if solved
```

## B. Recommended first model experiment

Select exactly ONE.

Explain why.

## C. Dataset specification

Exact schema.

Policy-visible vs privileged fields must be explicit.

## D. Baselines

At least:

```text
rules
simple statistical model
small ML model
```

## E. Generalization protocol

No robot-name memorization.

Hold out an entire embodiment.

## F. Calibration protocol

How probabilities will be validated.

## G. Runtime contract

Exactly what the model may and may not influence.

## H. Kill criteria

Define conditions under which Reality OS should abandon the custom-model idea.

Examples:

```text
rules perform equivalently
poor cross-robot transfer
probabilities cannot calibrate
training labels are unreliable
latency exceeds value
model mostly learns robot identity
no measurable workflow improvement
```

## I. Go criteria

Define what evidence would justify moving from experiment to a real `realityos-decision` component.

---

# EXECUTION ORDER

Follow this order strictly:

```text
1. Correct the recover plan design.
2. Implement/tests the narrow integrity fix on an isolated branch.
3. Do not touch frozen 2f68a5d.
4. Prepare/run the real XL330 baseline when Linux hardware exists.
5. Run the corrected SHA against the same metal campaign + recovery attacks.
6. Freeze authority if evidence passes.
7. Investigate typed physical-decision tasks.
8. Build dataset/evaluation harness.
9. Establish deterministic/simple ML baselines.
10. Only then decide whether an in-house neural model is justified.
```

The purpose is NOT to create an AI feature.

The purpose is to discover whether Reality OS has a domain where a small, calibrated, typed physical-decision model creates a genuine capability leap without compromising deterministic authority.

Prefer no model over an unjustified model.

Prefer a tiny calibrated model over a large generative model.

Prefer UNKNOWN over confident invention.

Prefer measured evidence over model prediction.

Prefer Reality OS remaining in control of reality.