Work only in the current reality-os-rust repository.

Do NOT use TheWorld or any stale predecessor.

Start by inspecting actual HEAD and Git status.

The current known baseline is:

75f3396188e2ba2010ff18906480d4bb71d5ae1b

Virtual Metal is frozen.

Respect:

docs/virtual_metal/FREEZE.md

Do NOT resume general XL330 simulation development.

The only permitted reopening of Virtual Metal in this task is to close a concrete production-path counterexample already discovered by Tier 2:

A CRC-valid Status Packet carrying the wrong servo ID
can currently be accepted by the production transaction path.

After closing that narrowly, re-freeze Virtual Metal and move the majority of work to generic physical-intelligence PUSH capability.

The mission is:

CLOSE ONE REAL DRIVER CORRECTNESS HOLE
                ↓
RE-FREEZE VIRTUAL METAL
                ↓
TURN CONTACT INTO VERIFIED TASK SUCCESS
                ↓
PROVE THE IMPROVEMENT GENERALIZES

PART 1 — CLOSE WRONG-STATUS-ID ACCEPTANCE

Current production flow roughly contains:

request to cfg.servo_id
        ↓
recv_status()
        ↓
decode_status_scan()
        ↓
first CRC-valid Status Packet
        ↓
accepted

The transaction must additionally establish:

status.id == expected servo ID

A packet from another actuator must NEVER satisfy the directed transaction.

Do not fix this inside VirtualXl330.

Fix the host/production transaction semantics.

The virtual device correctly exposed the problem.

Required behavior

For a directed request to servo ID 1:

status(ID=7, valid CRC)
→ MUST NOT be accepted as response for ID 1

If the stream contains:

wrong-ID valid status
followed by
correct-ID valid status

then:

ignore/skip wrong ID
accept correct ID

If only wrong-ID replies arrive:

timeout / communication refusal

Never count wrong-ID WRITE status as an ACK for the commanded actuator.

Do NOT break broadcast discovery.

These are different operations:

directed transaction
→ exact expected responder

broadcast sniff/discovery
→ multiple responder IDs are legitimate

Keep unique_status_ids() semantics separate.

Required regressions

Add tests for:

1. correct ID only
   → succeeds

2. wrong ID only
   → does not satisfy transaction

3. wrong ID then correct ID
   → correct response accepted

4. correct ID then wrong ID
   → first valid expected response accepted;
     later unrelated packet cannot mutate transaction result

5. wrong-ID WRITE response
   → cannot count as command ACK

6. wrong-ID READ response with plausible payload
   → cannot poison sensor/identity values

7. broadcast discovery
   → still sees multiple valid servo IDs

Run these through production parser/driver paths, not only protocol helpers.

If fixing this exposes assumptions elsewhere, fix the smallest correct layer.

Do not redesign Protocol 2.0 architecture.

PART 2 — RE-RUN A1–A7

After the wrong-ID fix, run the existing production-path Virtual Metal authority suite:

decide
→ ONLINE governor
→ HardwareBackedPlant
→ production Xl330Driver
→ Unix PTY
→ Protocol 2.0
→ VirtualXl330

Require A1–A7 to remain green.

Specifically verify WrongStatusId is no longer categorized as legitimate success for a directed transaction.

Update only the relevant expectation.

Do not loosen other fault expectations to make tests pass.

PART 3 — RE-FREEZE VIRTUAL METAL

Once:

wrong-ID fix passes
A1–A7 passes
Tier 2 passes
honesty passes
CI passes

restore the freeze.

Record the newly discovered counterexample and regression in:

docs/virtual_metal/FREEZE.md

or its appropriate adjacent evidence document.

After that:

NO NEW XL330 WORK

unless one of these happens:

real metal contradicts Virtual Metal

a newly minimized counterexample breaks a claimed invariant

a concrete new hardware target requires an abstraction change

Do not add fidelity merely because it is possible.

PART 4 — MAIN MILESTONE: PUSH TASK SUCCESS GIVEN CONTACT

Shift the project immediately to physical intelligence.

The current important failure is:

contact established
but
object displacement/task success does not follow

Current semantic investigation found a generic issue:

final Reach success radius
was large enough relative to stroke
that the controller could succeed
without actually traveling through the requested push.

The generic correction is:

final push Reach radius < meaningful push stroke

This correction must now be experimentally validated.

Do not assume the semantic change solved PUSH.

The next milestone is:

Convert established contact into verified object displacement across structurally different robot embodiments without robot-specific branches.

PART 5 — RECONSTRUCT PUSH AS A CAUSAL PIPELINE

Do not score PUSH as one Boolean.

Represent the episode as stages:

TARGET_AVAILABLE
        ↓
REACHABLE
        ↓
APPROACH_REACHED
        ↓
CONTACT_ESTABLISHED
        ↓
CONTACT_MAINTAINED
        ↓
PUSH_STROKE_EXECUTED
        ↓
OBJECT_DISPLACED
        ↓
DISPLACEMENT_DIRECTION_VALID
        ↓
DISPLACEMENT_MAGNITUDE_VALID
        ↓
TASK_VERIFIED

For every episode record:

first stage entered
last stage completed
earliest failed stage

The most important metric right now is:

P(task success | contact established)

Also compute:

P(contact | approach)

P(contact maintained | contact)

P(stroke executed | contact)

P(object displaced | stroke)

P(correct direction | displacement)

P(task verified | contact)

PART 6 — DEVELOPMENT EMBODIMENTS

Use multiple development robots.

Prefer existing repository embodiments such as:

Panda
UR5e
iiwa14
arm_gripper

Do not use WX250s for iterative tuning if it is intended as the held-out counterexample.

If WX250s has already influenced the implementation too much, retain it as a historical challenge and choose an additional untouched embodiment.

PART 7 — RANDOMIZE THE PHYSICAL WORLD

Do not test one easy object in one pose.

Generate scenarios varying:

object pose
object dimensions
object mass
support friction
object-contact friction
push direction
push distance
contact point
surface normal
end-effector orientation
approach offset
controller limits
joint limits
workspace proximity
sensor noise
observation freshness

Use public engineering data where available.

Where exact properties are unknown:

use physically defensible ranges

Where not defensibly bounded:

UNKNOWN

Do not invent a nominal value and silently treat it as truth.

Do not wait for hardware.

Use MuJoCo and existing simulation infrastructure aggressively.

PART 8 — ASK WHY CONTACT DOES NOT BECOME MOTION

When:

CONTACT_ESTABLISHED

but:

OBJECT_DISPLACED = false

determine the earliest defensible reason.

Candidate failure classes may include:

PushStrokeNotExecuted
ContactLost
ContactNormalMismatch
WorkspaceSaturated
ControllerSaturated
InsufficientEffectiveStroke
Slip
ObjectNotMovableUnderEnvelope
WrongPushDirection
CollisionBlocked
VerifierInsufficient
Unknown

Do not force an episode into a causal category if the evidence is insufficient.

Use:

Unknown

honestly.

PART 9 — MAKE ONE GENERIC IMPROVEMENT AT A TIME

The currently selected generic improvement is:

final Reach radius smaller than required push stroke

Test it first.

Do NOT simultaneously add:

adaptive force
contact-normal controllers
replanning
slip recovery
new sensors
learned controllers

If the radius correction materially improves contact→displacement, preserve the result.

Then use remaining counterexamples to choose exactly ONE next generic improvement.

Possible families include:

maintain contact through stroke
contact-normal-aware direction
closed-loop displacement
workspace-aware stroke reduction
slip recovery

Choose based on evidence.

Not preference.

PART 10 — NO ROBOT-SPECIFIC FIXES

For every proposed change ask:

Would this rule still make physical sense
if the robot identity string disappeared?

Do not use:

robot name
vendor
URDF filename
dataset name
embodiment ID

as causal branching features.

Absolutely forbid:

if robot == "wx250s"

or equivalent disguised behavior.

PART 11 — HELD-OUT TEST

After tuning only on development embodiments:

freeze the PUSH implementation.

Then evaluate on the held-out embodiment.

Do not inspect failures and immediately tune again.

First produce a frozen report.

Compare:

BEFORE generic fix
vs
AFTER generic fix

for:

contact established
stroke executed
object displaced
direction correct
task success
expected refusals
authority violations
unauthorized writes

The strongest result is not:

100% success

The strongest result is:

One generic semantic/control correction improved task-success-given-contact across multiple structurally different embodiments and transferred to an untouched robot.

PART 12 — GENERATE STRUCTURED FAILURE DATA

Every PUSH episode should emit a machine-readable diagnostic record.

At minimum:

episode_id
skill
embodiment topology features
world features
object features
requested push direction
requested push distance
approach evidence
contact evidence
stroke evidence
object displacement evidence
authority verdict
controller outcome
verifier result
earliest failure stage
final task result
provenance

Every field must be categorized:

POLICY_VISIBLE_RUNTIME
POST_HOC_OBSERVED
PRIVILEGED_SIM_LABEL_ONLY
IDENTITY_SPLIT_ONLY
TARGET_LABEL

Never allow privileged MuJoCo truth to silently become runtime input.

PART 13 — DIAGNOSTIC MODEL REMAINS NOT YET

Do NOT build a learned model yet.

First ask whether deterministic diagnosis is already sufficient.

Build a deterministic earliest-failure classifier from explicit evidence.

Measure:

coverage
Unknown rate
consistency
cross-robot applicability

Only reconsider a learned model after enough structured episodes exist.

Minimum reasons to reconsider:

failure labels are stable

there are enough examples per class

deterministic rules leave ambiguous but learnable cases

development/held-out protocol exists

Until then:

TYPED MODEL = NOT YET

PART 14 — DO NOT GET BLOCKED

A missing hardware arm, sensor or measurement is not automatically a blocker.

Before stopping:

1. Search existing repository data.
2. Search authoritative public engineering data if required.
3. Use URDF/MJCF/CAD parameters where trustworthy.
4. Use MuJoCo.
5. Use bounded uncertainty.
6. Use domain randomization.
7. Use synthetic counterexamples.
8. Use another embodiment.
9. Use CI/Linux for OS-specific tests.
10. Mark remaining physical questions honestly.

Only stop for physical hardware if the question truly cannot be answered in software.

Classify blockers as:

NOT_BLOCKED_SIMULATION_AVAILABLE

NOT_BLOCKED_PUBLIC_DATA_AVAILABLE

NOT_BLOCKED_UNCERTAINTY_CAN_BE_BOUNDED

NOT_BLOCKED_CI_ENVIRONMENT_AVAILABLE

PARTIALLY_BLOCKED_ONLY_FOR_CALIBRATION

TRUE_PHYSICAL_BLOCKER

PART 15 — DO NOT DRIFT

Do NOT build:

PLACE
VLA integration
vision foundation model
new authority kernel
new HAL
EtherCAT
CANopen
second Virtual Metal actuator
Nav2 replacement
MoveIt replacement
large world model
large neural model
UI platform
generic digital twin platform

unless a concrete PUSH counterexample directly requires something.

REQUIRED SUCCESS CRITERIA

This milestone is successful when:

1. Wrong Status ID can no longer satisfy directed production transactions.

2. Broadcast discovery still works.

3. A1–A7 remain green.

4. Virtual Metal is re-frozen.

5. PUSH final Reach requires actual stroke travel.

6. PUSH episodes expose explicit causal stages.

7. Development embodiments are evaluated under randomized physical conditions.

8. At least one generic change improves task-success-given-contact.

9. Held-out evaluation is run without robot-specific tuning.

10. Zero authority violations and zero unauthorized writes remain invariant.

11. Failure dataset is emitted with provenance.

12. Typed diagnostic model remains NOT YET unless actual data overturns that conclusion.

REQUIRED FINAL REPORT

Return:

1. Wrong-ID bug

Exact cause, exact production fix and regression coverage.

2. Virtual Metal status

Explicitly state:

FROZEN

or justify why a discovered counterexample prevents freezing.

3. PUSH causal funnel

Report counts and rates for every stage.

4. Before/after result

Show impact of the final-Reach-radius correction.

5. Development embodiment results

Per robot.

6. Held-out result

Without post-hoc tuning.

7. Counterexamples

List the highest-value remaining generic failure cases.

8. Failure taxonomy

Explain which categories are evidence-backed and which remain UNKNOWN.

9. Diagnostic dataset

Counts, schema and provenance separation.

10. Model decision

Answer:

NO
NOT YET
YES

based only on evidence.

11. Exactly one next milestone

Choose the milestone with the greatest:

physical capability gain
× cross-robot generality
× falsification value
──────────────────────────
engineering complexity

Do not create another architecture phase.

The purpose of Reality OS is no longer to prove that we can build more infrastructure.

The next proof is:

The same physical reasoning converts contact into useful, verified action across different machines.