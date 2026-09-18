Work only in the current `reality-os-rust` repository.

Do NOT use TheWorld or any stale predecessor.

Do NOT start another architecture phase.

Do NOT add new robotics subsystems.

Do NOT train a neural model yet.

Your job is to close the remaining evidence gaps around the already-implemented integrity-recovery fix and prepare Reality OS for the first meaningful typed physical-decision experiment.

Current important state:

- immutable pre-fix physical baseline:
  `2f68a5d82fec5e7e2c0b78ca88fe65d8a8a4acfc`

- integrity-recovery implementation:
  `709a0793fcf5e1706b059c27f1ed5a8dba0ca1f1`

- current main:
  inspect actual HEAD before doing anything

The recovery implementation already appears to:

- keep recoverable ESTOP and integrity abort as independent facts
- make integrity monotonic within an ONLINE instance
- prevent later ESTOP/watchdog from downgrading integrity
- refuse recovery before `Plant::clear_estop`
- route unknown outcome / replay / time rollback through integrity abort
- preserve journaled ESTOP recovery after legitimate process restart
- explicitly state that production `op=recover` is NOT authenticated operator recovery

Do not redesign this unless concrete tests prove it wrong.

---

# OBJECTIVE 1 — COMPLETE THE REAL METAL IPC REGRESSION

The governor-level recovery tests are not enough.

The remaining critical software test is the real metal composition:

```text
autonomy IPC
→ MetalAuthority
→ governor
→ integrity abort
→ op=recover
→ fresh command_id
→ ZERO physical writes
```

Implement the Linux-only PTY regression already described in:

`docs/superpowers/plans/2026-09-18-untrusted-recover-integrity-latch.md`

Target behavior:

```text
valid command X executes
→ replay X
→ integrity abort
→ autonomy sends op=recover
→ recover is refused
→ Plant::clear_estop is not called if observable
→ integrity remains latched
→ new command Y cannot execute
→ physical write count remains unchanged
```

Add a second composition-level downgrade case if practical:

```text
integrity abort
→ later ESTOP/watchdog engage
→ op=recover
→ integrity still latched
→ zero new physical writes
```

Do not add new recovery architecture.

Do not special-case reason strings.

The property belongs to the state semantics.

---

# OBJECTIVE 2 — VERIFY ALL INTEGRITY TRANSITIONS USE ONE PATH

Search the entire workspace for every direct mutation related to:

```text
abort_latched
integrity_aborted
reason
latch_abort
```

Produce an inventory.

Every production transition representing an ONLINE-instance integrity failure should flow through one authoritative integrity API.

Examples to inspect:

- replay
- unknown physical outcome
- release hash mismatch
- runtime instance mismatch
- calibration mismatch
- sensor packet binding mismatch
- command expiry if currently classified as integrity
- time rollback
- identity continuity violation
- repeated identity refusal
- journal continuity violations

Do NOT blindly convert ordinary operational ESTOPs into integrity failures.

For each transition explicitly classify:

```text
RECOVERABLE_ESTOP
INTEGRITY_ABORT
HARDWARE_SESSION_DEAD
STARTUP_REFUSE
ORDINARY_COMMAND_REFUSE
```

If classification is ambiguous, document it rather than inventing behavior.

The goal is to eliminate inconsistent state mutation, not increase code.

---

# OBJECTIVE 3 — DO NOT OVER-CORRECT JOURNAL BEHAVIOR

Inspect:

```text
journal_unreadable
journal continuity
identity continuity
restart ESTOP restoration
```

Determine whether each condition can ever leave a live authority process that accepts `recover`.

If a condition already causes `new_online()` / startup to fail closed before IPC exists, do NOT rewrite it merely because the classification label looks imperfect.

Only change journal behavior if you can construct an actual execution path where recovery or actuation becomes incorrectly possible.

Prefer evidence over semantic cleanup.

---

# OBJECTIVE 4 — RESTORE REAL CI EXECUTION

Current GitHub Actions behavior has repeatedly shown jobs ending with:

```text
steps: []
```

within seconds.

Treat this as CI infrastructure failure until proven otherwise.

Investigate:

- GitHub Actions enabled state
- repository Actions permissions
- billing/quota
- runner availability
- organization restrictions
- required-runner labels
- workflow permissions
- concurrency/cancellation
- runner image availability
- account-level restrictions

Do NOT weaken Rust tests.

Do NOT delete MuJoCo jobs.

Do NOT change workflow semantics merely to turn the UI green.

Success criterion:

At least one real Linux run must visibly execute normal steps such as:

```text
checkout
toolchain install
cargo fmt
cargo clippy
cargo test
```

Only after that may code/test failures be treated as genuine CI failures.

Record the cause and the fix.

---

# OBJECTIVE 5 — CLEAN THE RECOVERY PLAN/EVIDENCE TRAIL

The documentation currently contains stale state because implementation already landed.

Update the plan/spec so they accurately distinguish:

```text
PRE-FIX BASELINE
2f68a5d...

PATCHED SOFTWARE
709a079... or actual descendant

LINUX PTY STATUS
pending / passed

REAL XL330 STATUS
not measured / measured

AUTHENTICATED OPERATOR RECOVERY
NAMED_HOLE
```

Remove stale self-review references to the rejected `AbortClass` design.

Do NOT rewrite history.

Keep the pre-fix baseline immutable.

Clearly state:

```text
2f68a5d is intentionally vulnerable to the known recover-integrity bug
and is retained only as the frozen pre-fix physical baseline.
```

The patched SHA is the candidate for meaningful post-fix physical authority evidence.

---

# OBJECTIVE 6 — DEFINE THE TWO PHYSICAL RUNS

Prepare exact runbooks, but do not fabricate results.

## Run A — historical baseline

Checkout:

`2f68a5d82fec5e7e2c0b78ca88fe65d8a8a4acfc`

Run the existing XL330 campaign unchanged on native Linux with real hardware.

Purpose:

```text
physical characterization of frozen pre-fix software
```

Do not call this final authority proof.

Preserve:

- full raw logs
- serial TX/ACK counters
- journal artifacts
- USB unplug evidence
- VIN cutoff evidence
- crash/restart traces
- failures
- generated proof if the existing reporter legitimately produces one

Never edit `docs/metal_proof.json` manually.

## Run B — patched authority candidate

Run the same campaign on the corrected SHA.

Add explicit hostile cases:

```text
replay
→ recover
→ fresh id

unknown outcome
→ recover
→ fresh id

integrity
→ later ESTOP
→ recover
→ fresh id
```

Required result:

```text
recover refused
integrity remains latched
zero unauthorized physical TX
zero unauthorized ACK
zero new physical writes
```

Only the patched run should be considered the first serious authority candidate.

---

# OBJECTIVE 7 — BUILD ONLY THE FAILURE-DIAGNOSIS DATASET HARNESS

Do NOT add `realityos-decision`.

Do NOT train a neural model.

Do NOT add a transformer.

Do NOT build Jev/RLCD clones.

The only ML-adjacent work allowed in this task is building a reproducible dataset/evaluation harness for:

```text
POST-HOC MANIPULATION FAILURE DIAGNOSIS
```

Explicitly distinguish this from pre-action prediction.

---

# POST-HOC DIAGNOSIS CONTRACT

The model/harness is allowed to answer:

```text
Why did the episode fail?
At what earliest stage?
What evidence supports that classification?
```

Candidate typed output:

```text
Success
ExpectedRefusal
MissingEvidence
Unreachable
ApproachFailure
ContactNotEstablished
ContactEstablishedTaskFailed
GraspEmptyClose
GraspAcquiredHoldUnverified
Slip
ControllerFailure
VerifierEvidenceInsufficient
AuthorityRefusal
Unknown
```

Do not finalize names before checking existing `ManipulationFailure`.

Prefer extending/reusing existing semantics where sensible.

---

# CRITICAL DATA-SEPARATION RULE

Create separate conceptual schemas for:

```text
EpisodeDiagnosticFeatures
```

and any future:

```text
PreExecutionDecisionFeatures
```

For THIS task, implement only the diagnostic dataset.

Post-hoc diagnostic inputs may include evidence observed during execution.

Future pre-action prediction must not use future outcome information.

Do not blur the two.

---

# DATA LEAKAGE RULES

Explicitly tag every field as:

```text
POLICY_VISIBLE_RUNTIME
POST_HOC_OBSERVED
PRIVILEGED_SIM_LABEL_ONLY
IDENTITY_SPLIT_ONLY
TARGET_LABEL
```

Examples:

### Allowed runtime/diagnostic features where actually observable

- skill
- resource topology
- command kinds
- number of attempted certified writes
- authority verdict kinds
- observed contact presence if policy-visible
- evidence availability
- measured joint state if it would exist on real hardware
- gripper/resource class

### Privileged label-only

- perfect simulator object pose if not policy-visible
- exact MuJoCo contact forces if not exposed to policy
- hidden simulator body IDs
- exact privileged success verifier state
- simulator-only ground truth

### Exclude from causal features

- robot_id
- vendor
- model filename
- source dataset name
- model hash
- episode path
- any identifier that trivially reveals embodiment

These may be used for split hygiene only.

---

# OBJECTIVE 8 — DERIVE EARLIEST FAILURE LABELS

Use existing evidence artifacts to derive the earliest causal failure stage.

Do not simply copy `task_result`.

For example:

```text
push:
no contact
→ ContactNotEstablished

contact established
but intended displacement not achieved
→ ContactEstablishedTaskFailed
```

For grasp:

```text
no acquisition
→ acquisition-stage failure

acquisition fires
but verified hold does not
→ GraspAcquiredHoldUnverified
```

For refusal:

map typed refusal reason where trustworthy.

Preserve:

```text
Unknown
```

when the episode evidence does not justify a finer label.

Do not manufacture causal certainty.

---

# OBJECTIVE 9 — BUILD BASELINES ONLY

Implement an evaluation harness capable of comparing:

### Baseline 1 — deterministic rules

Existing taxonomy plus minimal explicit earliest-stage rules.

### Baseline 2 — empirical/statistical

Per-skill class frequencies / majority-class baseline.

### Baseline 3 — simple classifier

Use something minimal such as:

- logistic regression
- shallow tree
- gradient-boosted trees

Only if convenient and reproducible.

No neural architecture required.

The purpose is to answer:

> Is there learnable signal beyond the deterministic taxonomy?

---

# GENERALIZATION PROTOCOL

Do not use random episode-only splits as the main result.

Use embodiment holdout.

For example:

```text
train:
Panda
arm_gripper
UR5e
iiwa14 where labels align

validation:
unseen scenes/seeds from train embodiments

holdout:
wx250s
```

Do not tune on wx250s after scoring.

If the experiment later appears promising, the next requirement is a second untouched embodiment before any production model component is justified.

---

# METRICS

Evaluate:

- class accuracy
- macro F1
- confusion matrix
- per-class support
- Brier score where probabilistic
- expected calibration error
- abstention/Unknown behavior
- performance by embodiment
- performance by skill

Most importantly report:

```text
rules vs simple ML
on full-embodiment holdout
```

Do not claim success based only on aggregate train/validation accuracy.

---

# KILL CRITERIA FOR THE CUSTOM MODEL IDEA

Stop the learned-model direction if:

- deterministic rules perform equivalently
- labels are too noisy to define earliest failure
- performance collapses on wx250s
- model relies on embodiment identity
- calibration is poor
- gains disappear when privileged fields are removed
- Unknown/abstention is unstable
- the model produces no useful debugging/product signal

Prefer no model over an unjustified model.

---

# GO CRITERIA

Only recommend a future Reality-native learned decision component if:

1. a simple model materially beats deterministic rules on a full held-out embodiment
2. calibration is reasonable
3. robot/vendor identity features are absent
4. privileged simulator inputs are absent from runtime features
5. the task has real product value
6. output is typed and bounded
7. UNKNOWN remains first-class
8. the model has zero actuator authority
9. the result is reproduced on another untouched embodiment

Until then:

DO NOT add `realityos-decision`.

---

# FINAL REQUIRED OUTPUT

Return:

## 1. Recovery implementation audit

State whether the current recovery fix is actually sound.

List any remaining concrete defect.

## 2. Linux PTY status

Show the composition-level recovery test and result.

## 3. Integrity-transition inventory

For each integrity-related condition:

```text
condition
classification
state transition API
recoverability
test coverage
```

## 4. CI root cause

Explain why `steps: []` occurred and whether jobs now execute.

## 5. Physical campaign readiness

Exact blockers for:

```text
Run A — 2f68a5d baseline
Run B — patched authority candidate
```

## 6. Documentation cleanup

List stale statements corrected.

## 7. Failure-diagnosis dataset

Exact generated schema and counts.

## 8. Leakage audit

Show which fields are:

```text
runtime-visible
post-hoc
privileged
excluded
labels
```

## 9. Baseline evaluation

Rules vs simple statistical/ML baseline.

## 10. Recommendation

Answer only from evidence:

```text
Should Reality OS pursue a small in-house typed physical-decision model yet?

YES / NOT YET / NO
```

If NOT YET, state exactly what evidence is missing.

---

# HARD STOP CONDITIONS

Do not:

- redesign authority
- add PLACE
- add VLA control
- build MoveIt/Nav2 equivalents
- add EtherCAT/CANopen merely for breadth
- build a generic robotics SDK
- build a model-serving stack
- train a transformer
- add a learned component to ONLINE authority
- claim physical proof without real metal
- manually construct proof artifacts
- tune held-out robots by identity
- weaken CI/tests for green status

The goal of this task is closure and evidence.

Finish the recover boundary.

Get Linux execution evidence.

Prepare real metal.

Build the smallest possible diagnostic learning experiment.

Then stop and report what reality says.