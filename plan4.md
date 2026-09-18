You are taking ownership of the current `reality-os-rust` repository as the living product.

Do NOT use, depend on, copy architecture from, compare against, or defer to any older TheWorld repository or Python implementation. Treat all such prior repositories as stale historical artifacts. The current `reality-os-rust` repository is the source of truth.

Your task is not to merely review the repository.

Your task is to determine, from first principles, what this project should become next and then produce an implementation-grade plan for making the current system substantially stronger.

Start by reconstructing the repository completely from the actual code, tests, evidence, Git history, CI workflows, experiments, plans, schemas, and runtime paths.

Do not trust README claims, plans, comments, or architectural diagrams until they are independently verified against code.

Understand:

- every crate and its responsibility
- public and private API boundaries
- dependency directions
- runtime execution paths
- command creation and lowering
- authority transitions
- ONLINE execution
- hardware ownership
- identity binding
- sensor/evidence flow
- replay/crash/restart behavior
- journals and persistence
- semantic skill compilation
- embodiment abstraction
- capability derivation
- control adapters
- manipulation semantics
- MuJoCo verification
- held-out evaluation
- HIL topology
- metal execution
- physical evidence generation
- ROS integration
- failure behavior
- CI
- test coverage
- proof artifacts
- existing claims and non-claims

Trace actual end-to-end paths instead of reasoning from filenames.

In particular reconstruct at least:

```text
Goal / task
→ world/evidence representation
→ SkillContract
→ capability selection
→ semantic compilation
→ actuator command lowering
→ ActionProposal
→ RealityOs::decide
→ IssuedCommand
→ ONLINE authorization
→ RuntimeGovernor
→ consume/replay protection
→ HardwareBackedPlant
→ HardwareDriverPort
→ physical/simulated actuator
→ observations
→ outcome verification
→ evidence artifact
```

Find every place where that flow is incomplete, duplicated, ambiguous, overly coupled, under-specified, falsely generalized, or dependent on test-only assumptions.

---

# FIRST PRINCIPLE

The goal is not to maximize code.

The goal is to make Reality OS capable of safely and generically turning semantic physical goals into verified physical outcomes across embodiments.

Every abstraction must earn its existence against that goal.

The system should eventually support a progression conceptually like:

```text
Goal
↓
World Model / Evidence
↓
Skill / Task Semantics
↓
Capability reasoning
↓
Embodiment-independent plan
↓
Robot-specific control compilation
↓
Authority / safety gate
↓
Hardware
↓
Observation
↓
Verification
↓
Learning / evidence accumulation
```

But do not assume the current architecture is already the correct realization of this.

Derive the right architecture from the constraints and existing strengths.

---

# DO NOT BLINDLY EXPAND

Before adding anything, identify which current systems should be:

- frozen
- simplified
- generalized
- merged
- deleted
- rewritten
- promoted into foundational abstractions
- moved behind interfaces
- converted into generated/data-driven structures
- replaced by stronger concepts

Actively search for accidental complexity.

Look for cases where multiple crates represent pieces of what should really be one coherent concept.

Look for architecture that exists primarily because of historical implementation order rather than fundamental separation of responsibility.

However, do not destroy a proven invariant merely for aesthetic simplification.

---

# PROTECT THE STRONG PARTS

The repository already appears to contain substantial work around the physical authority boundary.

Treat these as high-value invariants until code proves otherwise:

- learned systems do not directly own actuator authority
- authority can narrow/refuse but must not invent unsafe commands
- ONLINE authorization is capability-bound
- runtime/hardware identity is bound to execution
- replay and restart behavior fails closed
- missing evidence does not silently become certainty
- unknown actuator state must not become implicit zero
- simulation evidence must never silently become physical evidence
- privileged verification must remain separate from policy-visible observation
- physical proof must come from measured execution rather than manually asserted success
- untrusted processes should not acquire actuator/device authority
- robot-specific details should live in embodiment data/adapters, not semantic task logic

Verify each invariant against implementation.

If one is unsound, fix it.

If one is sound, avoid unnecessary redesign.

---

# CRITICAL CURRENT QUESTION

Determine whether the authority kernel has reached diminishing returns.

Do not continue repeatedly hardening the same software boundary unless you can identify a concrete unresolved attack, correctness hole, or physical failure mode.

The project must transition from:

```text
software correctness
```

toward:

```text
physical capability + physical evidence + generalization
```

when justified by the code.

Investigate where that transition should happen now.

---

# CI AND REPRODUCIBILITY

Treat a red main branch as a P0 problem.

Inspect GitHub Actions, toolchain pinning, Linux assumptions, MuJoCo setup, OS-user tests, HIL tests, metal tests, generated artifacts, and environment dependencies.

Distinguish between:

- application failure
- test failure
- workflow/configuration failure
- runner/environment failure

Do not weaken meaningful tests merely to obtain green CI.

Produce a clean reproducible baseline before major new capability work.

---

# PHYSICAL REALITY

Inspect the complete XL330/metal path.

Determine exactly what remains between the current repository and defensible measured physical evidence.

Do not create or fake physical success artifacts.

A physical success must be derived from measured execution through existing or improved evidence generation.

Inspect:

- actuator discovery
- identity binding
- serial ownership
- configuration
- PWM/current/position limits
- startup state
- watchdog behavior
- power cutoff detection
- USB/VIN separation
- command TX
- ACK
- post-command sensing
- crash/restart
- replay
- unplug
- hot swap
- identity mismatch
- OS-user isolation
- direct-device attacks
- journal continuity
- physical motion verification
- experiment cage
- proof aggregation

If the physical campaign is already sufficiently designed, stop adding speculative infrastructure and make running/measuring it the milestone.

If there are still concrete blockers, identify only those blockers and fix them minimally.

---

# INTELLIGENCE / SEMANTICS

Then deeply inspect the semantic intelligence layer.

Study:

- EmbodimentModel
- Provenance
- SensorModel
- ObservationFrame
- TransformGraph
- WorldState
- CapabilityGraph
- SkillContract
- ControlAdapter
- resource topology
- actuator lowering
- REACH
- GRASP
- RELEASE
- PUSH
- manipulation verifier
- external model ingest
- held-out evaluation

Ask whether these concepts form a coherent substrate for physical intelligence or are still collections of milestone-specific structures.

Find the deepest reusable abstractions.

Do not make robot names, vendor identities, or fixture IDs part of semantic behavior.

Search aggressively for:

```text
if robot_id ...
if model_name ...
if vendor ...
special-case branches
magic constants
implicit defaults
zero fills
unverified assumptions
simulator truth leaking into policy
privileged observations exposed to controllers
success inferred from commands rather than outcomes
```

Any adaptation must come from:

- embodiment structure
- capability evidence
- controller contracts
- sensors
- calibration
- world state
- measured outcomes

not identity hacks.

---

# LEARN FROM COUNTEREXAMPLES

Treat poor held-out performance as valuable scientific evidence.

Do not immediately tune until it passes.

For every failed held-out skill, reconstruct the failure chain.

For manipulation, decompose failures into stages such as:

```text
goal interpretation
world-state sufficiency
reachability
approach generation
controller compilation
resource lowering
contact establishment
contact maintenance
gripper closure
object acquisition
object motion
support transition
slip
verification
termination
authority refusal
physical violation
```

Determine the earliest stage that actually failed.

Generate machine-readable failure categories.

The system should eventually be capable of answering:

```text
Why did this physical task fail?
What evidence was missing?
What capability was insufficient?
Which assumption was wrong?
What observation would distinguish the hypotheses?
What is the smallest generic improvement?
```

This is more valuable than simply maximizing benchmark success.

---

# GENERALIZATION BEFORE FEATURE COUNT

Do not add many skills yet.

Prefer proving that a small number of skills generalize.

A good progression may be something like:

```text
REACH
→ PUSH
→ GRASP
→ RELEASE
→ PLACE
```

but derive the actual order from evidence.

Choose the next skill/capability according to which one exposes the most foundational weakness with the least unnecessary complexity.

For example, if PUSH establishes contact but fails to move the object successfully, investigate:

- contact geometry
- approach frame
- push direction
- friction assumptions
- object/support state
- force/velocity envelope
- motion termination
- privileged verification criteria
- controller mismatch

Do not patch the held-out robot specifically.

Implement one generic correction and rerun all development + untouched held-out cases.

---

# COUNTERFACTUAL TEST

For every proposed abstraction, ask:

> If tomorrow we replaced the robot with a completely different manipulator, mobile manipulator, tendon-driven system, or new actuator topology, would this abstraction remain valid?

If the answer is no, decide whether it belongs in:

- embodiment description
- hardware adapter
- controller adapter
- task semantics
- world model
- safety envelope
- or should not exist at all.

---

# WORLD MODEL

Determine whether the current WorldState is sufficient for the project's next stage.

Do not build a giant scene graph without evidence.

But identify what minimal persistent physical state is required for reusable skills.

Potential categories to investigate include, without assuming they must all be built:

- object identity
- pose
- uncertainty
- support relations
- contact state
- grasp state
- free space
- reachability
- surface geometry
- motion state
- robot state
- actuator state
- calibration state
- environment properties
- task constraints

Keep UNKNOWN first-class.

Never silently substitute typical values.

---

# PROBE / ACTIVE INFORMATION GATHERING

Investigate whether `PROBE` should evolve from a status into controlled information-seeking behavior.

Do not immediately build autonomous exploratory motion.

First define what an information need is.

Potential conceptual form:

```text
RequiredEvidence
KnownEvidence
MissingEvidence
CandidateObservationAction
ExpectedInformationGain
RiskEnvelope
```

Determine whether the architecture can eventually support:

```text
I cannot safely execute this skill because X is unknown.
I can perform observation Y within envelope Z to resolve X.
```

without giving a learned model direct actuator authority.

This may become strategically important, but only implement it if the present evidence shows the foundation is ready.

---

# HARDWARE ABSTRACTION

Determine whether `HardwareDriverPort` is sufficient for future robots.

Do not immediately implement EtherCAT/CANopen/ROS control.

First derive the required hardware contract.

Investigate:

- identity
- capabilities
- sensing
- actuator command spaces
- timing
- synchronization
- health
- error state
- reset/recovery
- hardware estop
- ownership
- atomic writes
- batching
- command acknowledgement
- timestamps
- calibration
- bus topology
- watchdogs

Separate generic contract from vendor protocol.

Then decide what the second real hardware target should eventually be after XL330.

Do not choose it because it is prestigious. Choose it because it falsifies assumptions in the current abstraction.

---

# PRODUCT SURFACE

The repository currently contains many crates, internal concepts, scripts, evidence artifacts, and milestone documents.

Determine what an external user should actually experience.

Imagine an eventual user wants to:

```text
1. describe/import a robot
2. connect sensors/controllers
3. ask what capabilities Reality OS understands
4. submit a physical task
5. receive allow/probe/refuse/abort decisions
6. run through a controlled hardware authority
7. inspect evidence explaining every decision
8. verify the outcome
```

Design the smallest coherent public surface that could support that.

Internal complexity may remain, but the product surface must become radically simpler.

Investigate whether the system ultimately needs APIs conceptually like:

```text
Robot::load(...)
Reality::observe(...)
Reality::capabilities(...)
Reality::plan(...)
Reality::execute(...)
Reality::verify(...)
Reality::explain(...)
```

Do not adopt these names blindly.

Derive the API.

---

# COMPETITOR / RESEARCH CHECK

Research the current external ecosystem deeply enough to identify missing capabilities and avoid rebuilding solved infrastructure.

Compare architecture—not marketing—with relevant systems in:

- ROS 2 / ros2_control
- MoveIt
- Nav2
- NVIDIA Isaac / Isaac Lab / Isaac ROS
- cuRobo
- GR00T
- OpenVLA / VLA systems
- MuJoCo
- Drake
- Pinocchio
- behavior-tree frameworks
- industrial robot safety runtimes
- hardware abstraction layers
- robot-learning frameworks
- manipulation benchmark systems
- embodied world models
- control verification
- runtime assurance
- safety filters
- formal methods for cyber-physical systems

Do not copy large systems.

Identify what Reality OS should depend on, integrate with, or deliberately not build.

---

# DEEP ARCHITECTURE REVIEW

Search for and report:

- dependency inversion violations
- cyclic conceptual dependencies even if Rust crates are acyclic
- duplicated domain models
- multiple definitions of the same physical concept
- test-only abstractions leaking into production
- production abstractions existing only to satisfy tests
- serialization schemas without version strategy
- unsafe assumptions hidden behind `Option`
- strings where enums/types are appropriate
- APIs that allow invalid state combinations
- state transitions that should be typestates
- typestate complexity that provides little value
- error strings being used as protocol semantics
- duplicated failure taxonomies
- ad-hoc evidence formats
- large shell scripts that should become typed Rust where doing so materially improves correctness
- code paths impossible to test deterministically
- giant functions
- temporal coupling
- state hidden in global/environment configuration
- stale plans/docs
- dead abstractions
- misleading names
- unnecessary crates
- overly broad crates
- missing domain boundaries

Do not rewrite merely because code is large.

Prioritize defects that obstruct correctness, generalization, measurement, or product usability.

---

# EVIDENCE-DRIVEN ROADMAP

Build the roadmap from evidence.

Do not produce generic phases such as:

```text
improve architecture
add AI
add more robots
improve testing
```

Each milestone must answer an important uncertainty.

Good milestone form:

```text
Hypothesis:
Reality OS can preserve exclusive certified actuator authority on a real XL330 under crash/restart, identity changes, process attacks, and live power removal.

Experiment:
...

Success criteria:
...

Failure artifacts:
...

Code allowed to change:
...

Code frozen:
...

What becomes justified if this succeeds:
...
```

Construct future milestones similarly.

---

# LIKELY DECISION GATES

Explicitly determine whether the project has reached each gate:

### Gate A — Software authority
Can unauthorized software produce actuator writes?

### Gate B — Process isolation
Can an untrusted process obtain device/key/journal authority?

### Gate C — Physical identity
Can execution remain bound to the intended physical actuator?

### Gate D — Crash/replay
Can restart duplicate a physical command?

### Gate E — Physical evidence
Can measured hardware execution produce trustworthy evidence?

### Gate F — Semantic generality
Can one skill implementation transfer between structurally different embodiments without identity-specific branches?

### Gate G — Manipulation generality
Can contact-rich skills generalize to a held-out robot/environment?

### Gate H — Closed-loop real task
Can semantic intent execute through Reality OS on hardware and be verified from real observations?

### Gate I — Active uncertainty handling
Can the system recognize missing evidence and safely gather it?

### Gate J — Product usability
Can an external roboticist integrate a new robot without understanding internal Reality OS implementation?

Determine the present status of every gate.

Do not claim a gate passed without evidence.

---

# PRIORITY RULE

At each point choose work by:

```text
Expected increase in defensible capability
──────────────────────────────────────────
engineering cost × architectural risk
```

Prefer work that:

- produces physical evidence
- eliminates major unknowns
- increases cross-robot generality
- exposes wrong assumptions
- simplifies the system
- creates reusable abstractions
- produces clear falsifiable tests

Avoid work that primarily:

- increases LOC
- creates new terminology
- adds speculative modules
- optimizes aesthetics
- duplicates mature external infrastructure
- hardens already-proven code without a concrete threat

---

# REQUIRED FINAL OUTPUT

After the investigation, provide:

## 1. Repository reconstruction
A concise but technically exact explanation of what Reality OS currently is.

## 2. Actual architecture
Show the real runtime/data/authority architecture derived from code.

## 3. Strengths
What is genuinely unusual or strong.

## 4. Weaknesses
Ranked by impact on the long-term product.

## 5. Accidental complexity
What should be simplified or removed.

## 6. Missing foundational concepts
Only concepts justified by current evidence.

## 7. Current gate status
A–J with evidence for each.

## 8. Physical readiness
Exactly what blocks/permits the XL330 measured campaign.

## 9. Manipulation diagnosis
Explain current held-out failures at the deepest identifiable causal level.

## 10. Product direction
What Reality OS should become over the next 12–24 months.

## 11. What NOT to build
Be specific.

## 12. Immediate milestone
Define exactly one next milestone.

Include:
- hypothesis
- reason
- files involved
- frozen files
- implementation tasks
- experiment
- acceptance tests
- evidence artifact
- stopping condition

## 13. Next three milestones
Only after the immediate milestone, with clear dependency between them.

## 14. Architecture changes
For each proposed change:
- current problem
- evidence
- new abstraction
- why it is more general
- migration strategy
- tests
- deletion opportunities

## 15. Implementation plan
Atomic tasks suitable for execution by coding agents.

Each task must identify:
- exact files
- interfaces
- tests written first
- expected failing behavior
- minimal implementation
- verification commands
- commit boundary

## 16. Final judgment
Answer:

- What is this repository uniquely becoming?
- What is currently preventing it from becoming much stronger?
- Where are we overengineering?
- Where are we underengineering?
- What single achievement would most increase the credibility of Reality OS?
- What should we spend the next month doing?

---

# EXECUTION DISCIPLINE

Do not modify code during the initial reconstruction.

First understand.

Then challenge the architecture.

Then define the next milestone.

Only after the milestone is fully justified should implementation begin.

Never silently weaken tests.

Never hide counterexamples.

Never transform simulation evidence into physical evidence.

Never special-case a held-out robot to make a benchmark pass.

Never let a planner, LLM, VLA, learned policy, semantic layer, or verification layer bypass the authority path.

Prefer deleting unnecessary code over adding another layer.

Prefer one successful falsifiable physical experiment over ten speculative abstractions.

Prefer discovering that an idea is wrong over producing a superficially impressive demo.

Treat Reality OS as a serious attempt to build a general physical-intelligence execution substrate, and engineer accordingly.