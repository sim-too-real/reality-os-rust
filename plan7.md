Work only in the current reality-os-rust repository.

Do NOT use TheWorld or any stale predecessor.

Current main baseline is:

483c0ee32a026282af427bcf2f075c87de2a96ea

Treat this as the current shipped baseline unless HEAD has advanced; inspect HEAD first.

Current state is already strong:

Virtual Metal V1 exists

production Xl330Driver talks to VirtualXl330 over a real Protocol 2.0 byte/PTTY boundary on Unix

Tier 1 in-process campaigns exist

Tier 2 PTY smoke exists

transport faults exist on the byte path

motion is time-dependent through advance(dt)

published no-load/stall/PWM/position envelopes are respected

Truth Pack uncertainty is sampled

campaign artifacts are reproducible and hashed

hardware_present=false

SIM_VIRTUAL_METAL_NOT_METAL

Virtual Metal cannot mint MEASURED evidence

FACTORY_VELOCITY_P_GAIN=180

stale worktree gitlink is removed

Linux authority CI is green

os-users is green

MuJoCo verification is green

Do NOT start another broad architecture pass.

The immediate objective is:

Close the final gap between Virtual Metal and the actual production authority path, then freeze Virtual Metal and move the project toward physical-intelligence generalization.

PART A — TIER-2 AUTHORITY EQUIVALENCE

Current Tier 2 proves roughly:

Xl330Driver
→ PTY
→ VirtualXl330

and mainly exercises:

open
identity
sensor
hold
close

That is not enough.

Build the production-equivalence path:

RealityOs::decide
        ↓
IssuedCommand
        ↓
RuntimeGovernor<OnlineLocked>
        ↓
HardwareBackedPlant
        ↓
production Xl330Driver
        ↓
Unix PTY
        ↓
VirtualSerialPeer
        ↓
Protocol 2.0 bytes
        ↓
VirtualXl330

No Virtual-Metal-specific bypasses.

No alternate governor.

No direct device manipulation from the production side.

HARD REQUIREMENT — SEPARATE AUTHORITY BELIEF FROM DEVICE TRUTH

Tier 2 must have an independent observation channel from the virtual device.

The production path must only see what a real host would see.

The test harness may separately inspect privileged virtual truth such as:

physical_action_count
actual target applied
actual present position
device torque state
raw TX packets
raw RX packets
fault state
device reset count

This privileged truth must NEVER enter the production authority input.

It is test-only oracle data.

The purpose is to compare:

WHAT REALITY OS BELIEVES
vs
WHAT THE VIRTUAL DEVICE ACTUALLY DID

This is the core value of Tier 2.

INVARIANT A1 — NO FALSE "NO EFFECT"

Define this property explicitly:

If Reality OS concludes that no physical effect occurred,
but VirtualXl330 actually applied an actuator effect,
that is a SEVERE FAILURE
unless Reality OS classified the result as UNKNOWN.

The allowed mapping is:

Known success      ↔ effect confirmed
Known refusal      ↔ no effect
UNKNOWN            ↔ effect may or may not have happened

Forbidden:

"definitely did not happen"
while virtual truth says it happened

Also forbidden:

"definitely happened"
while virtual truth says it did not

Turn these into machine-checkable assertions.

INVARIANT A2 — UNKNOWN MUST POISON THE SAME ONLINE INSTANCE

For:

write applied
→ status/ACK lost

require:

driver returns UnknownOutcome
→ governor integrity-aborts
→ recover cannot clear it
→ fresh command cannot actuate
→ no additional physical_action_count

Run this through:

RuntimeGovernor
→ HardwareBackedPlant<Xl330Driver>
→ PTY
→ VirtualXl330

not through VirtualMetalPort.

INVARIANT A3 — REPLAY THROUGH THE REAL DRIVER STACK

Run:

command X
→ applied
→ command X replayed

Require:

second physical action count = 0
integrity behavior matches current design

Then restart on the same journal.

Require:

spent command_id remains spent
no duplicate action

INVARIANT A4 — RECOVERY ATTACK

Run through full Tier 2:

integrity abort
→ op/recover-equivalent governor recovery path
→ fresh command

Require:

recover refused
integrity remains
fresh command cannot actuate
device action count unchanged

Then test:

integrity abort
→ later ESTOP
→ recover

Integrity must still survive.

INVARIANT A5 — IDENTITY CHANGE

After ONLINE:

change model
change firmware
change device identity

Then issue a command.

Require:

authority refuses or faults
zero physical action

Do not special-case Virtual Metal identity names.

INVARIANT A6 — VOLTAGE FAULT

Use on-wire/real-driver path:

Present Input Voltage
→ outside configured min/max

Require:

sensor path sees actual register value
vin fault is raised
subsequent writes refuse
zero new physical action

Test both documented voltage-error-bit interpretations where the manual conflict remains unresolved.

Do not pretend the conflict is settled.

INVARIANT A7 — REAL TRANSPORT FAULTS

At minimum run these through the production driver:

CRC corruption
truncated Status Packet
wrong Status ID
delayed Status Packet
status after timeout
device silent
disconnect
reconnect
duplicate status
split status across reads
garbage prefix/suffix
reboot during request

For each:

record:

driver result
governor result
integrity state
device truth
physical action delta

Do not collapse all faults into the same expected outcome.

Different failure modes may legitimately produce:

refusal
disconnection
unknown outcome
startup failure

The test should verify the correct class.

TIER-2 ARCHITECTURE RULE

Do NOT make Tier 2 huge or slow.

Use:

Tier 1 = discovery / large-scale search
Tier 2 = production-path confirmation

Tier 2 should contain a compact but adversarial set.

A good target is:

50–500 carefully structured Tier-2 runs

rather than millions.

Use Tier 1 for tens of thousands or more.

COUNTEREXAMPLE PROMOTION

When Tier 1 finds a failure:

seed
fault sequence
parameter realization

provide a mechanism to replay the same minimized case through Tier 2 where possible.

Conceptually:

Tier1Counterexample
        ↓
promote
        ↓
Tier2ProductionReplay

Do not automatically assume every Tier-1 failure maps to a serial/driver issue.

Promote only when production-path equivalence is relevant.

TIER-2 CAMPAIGN RECORD

Do not hardcode:

physical_actions: 0

Record actual privileged VirtualXl330 device truth.

Each row should carry at least:

seed
scenario
fault sequence
truth-pack hash
parameter realization
Reality OS SHA
Virtual Metal version
authority verdict
driver verdict
integrity state
device physical action count before
device physical action count after
device position before
device position after
actual applied target if available
TX count
RX count
verdict

Still:

hardware_present=false
SIM_VIRTUAL_METAL_NOT_METAL

PART B — DO NOT OVERBUILD XL330

Once Tier-2 authority equivalence passes:

FREEZE Virtual Metal XL330 architecture.

Do not immediately add:

another Dynamixel
EtherCAT
CANopen
BLDC
industrial drive
thermal FEA
high-fidelity motor electromagnetics
large digital twin framework

Only reopen Virtual Metal if one of these occurs:

real metal contradicts it
Tier 1 finds a new authority counterexample
Tier 2 exposes a production-path mismatch
a new hardware target requires a concrete abstraction change

Do not improve fidelity merely because more fidelity is possible.

PART C — MOVE THE MAIN R&D TARGET TO PUSH V2

After Tier 2 closes, physical intelligence becomes the primary milestone.

Current important evidence includes the WX250s PUSH failure pattern:

contact established often
task success = 0

The next research target is:

Why does contact not become intended object displacement?

Do not add PLACE yet.

Do not special-case WX250s.

PUSH V2 PIPELINE

Model PUSH as explicit stages:

Target identified
        ↓
Reachability established
        ↓
Approach geometry valid
        ↓
Contact established
        ↓
Useful contact maintained
        ↓
Force/motion direction appropriate
        ↓
Object displacement occurs
        ↓
Displacement matches task goal
        ↓
Verifier confirms outcome

Every episode must identify the earliest stage that failed.

Do not report only:

push_success=false

Report why.

PUSH V2 — PHYSICAL VARIABLES TO MODEL

Investigate generic variables including:

object mass
object dimensions
support surface
friction
surface normal
contact point
contact normal
push direction
end-effector geometry
approach direction
approach velocity
push velocity
contact persistence
slip
object inertia
joint limits
controller limits
reachable workspace
obstacles
sensor uncertainty

Do not use robot/vendor identity as a semantic feature.

SIMULATION WITHOUT HARDWARE BLOCKING

Use MuJoCo and public engineering data aggressively.

Do not wait for a physical arm.

For missing physical properties:

search public data
use manufacturer values
use URDF/MJCF inertials if credible
use estimated ranges
domain-randomize
mark UNKNOWN

Never silently fill UNKNOWN with a nominal constant.

For each uncertain variable ask:

Does PUSH success depend strongly on this?

If yes:

sweep it.

If no:

do not waste time calibrating it.

PUSH V2 — GENERIC CONTROL IMPROVEMENT

Do not add robot-specific branches.

Any proposed improvement must answer:

Would this still make sense on:
Panda
UR5e
iiwa
arm_gripper
WX250s
another unseen arm

Potential generic improvement families to investigate:

contact-normal-aware approach
persistent-contact controller
push-distance controller
slip detection
surface-normal correction
adaptive push velocity
force-envelope estimation
trajectory replan after missed contact
closed-loop object displacement verification

Do not implement all.

Use counterexamples to select the smallest generic change.

PUSH V2 — HELD-OUT GENERALIZATION

Keep at least one embodiment frozen.

Preferred structure:

development:
Panda
UR5e
iiwa
arm_gripper

heldout:
WX250s

If WX250s has already influenced code too much, choose another untouched embodiment too.

Do not tune using the holdout result.

PUSH V2 — COUNTEREXAMPLE DATASET

Every episode should emit a structured record.

Conceptually:

embodiment topology
skill
world-state features
object features
contact evidence
controller evidence
authority result
execution evidence
verifier result
earliest failure stage
success/failure
provenance

Tag fields:

POLICY_VISIBLE
POST_HOC_OBSERVED
PRIVILEGED_SIM_LABEL_ONLY
TARGET_LABEL
IDENTITY_SPLIT_ONLY

Do not leak privileged simulator truth into runtime decision inputs.

PART D — SMALL TYPED MODEL EXPERIMENT, ONLY IF DATA JUSTIFIES IT

Do NOT create a large AI model.

After the PUSH counterexample dataset exists, test exactly one bounded task:

post-hoc earliest failure-stage diagnosis

Input:

policy-visible + post-hoc observable execution evidence

Output:

typed failure-stage probabilities

Potential labels:

Success
ExpectedRefusal
Unreachable
ApproachFailure
ContactNotEstablished
ContactLost
ContactEstablishedNoDisplacement
WrongDirection
InsufficientDisplacement
Slip
ControllerFailure
VerifierInsufficient
Unknown

Reuse existing semantics where possible.

MODEL BASELINES

Compare:

deterministic rules
majority / empirical baseline
logistic regression
small tree / gradient boosting
tiny MLP only if justified

Do not start with a transformer.

Do not build a serving stack.

MODEL SAFETY BOUNDARY

The learned model may only:

classify
score
diagnose
rank
identify missing evidence

It may NEVER:

construct OnlineWrite
mint IssuedCommand
clear ESTOP
clear integrity
grant authority
invent physical state
declare MEASURED evidence
directly write motors

MODEL GO CRITERIA

Only continue with a learned Reality decision component if:

simple ML materially beats deterministic rules
on a held-out embodiment
without robot identity features
with acceptable calibration
without privileged runtime leakage

Otherwise stop.

PART E — PRODUCT SURFACE, BUT ONLY AFTER CAPABILITY

Do not redesign the public API now.

After Tier 2 and PUSH V2 produce stronger evidence, identify the minimum eventual user flow:

load embodiment
connect/simulate hardware
observe capabilities
submit task
get ALLOW / REFUSE / PROBE / ABORT
execute through authority
inspect evidence
verify outcome

Only document the desired surface.

Do not build a large SDK/UI yet.

HARD PRIORITY ORDER

Execute in this order:

1. Inspect current HEAD.
2. Build Tier-2 full authority equivalence.
3. Add independent device-truth assertions.
4. Port only high-value adversarial scenarios.
5. Run Tier-2 CI.
6. Fix discovered bugs.
7. Freeze Virtual Metal.
8. Start PUSH V2 causal failure analysis.
9. Improve one generic push capability.
10. Evaluate across multiple embodiments.
11. Generate failure dataset.
12. Only then test a small typed diagnostic model.

HARD STOP LIST

Do NOT:

rewrite authority kernel
add PLACE
add locomotion
build a VLA
add new hardware protocols
build a generic HAL
replace MuJoCo
replace ROS
add a large world model
train a large model
build another simulator
polish XL330 indefinitely
claim virtual evidence is physical
special-case WX250s

REQUIRED FINAL REPORT

Return:

1. Tier-2 architecture

Show the actual end-to-end path.

2. Authority-vs-device-truth table

For every adversarial scenario show:

authority belief
driver result
device actual effect
integrity state
pass/fail

3. Tier-2 counterexamples

List and minimize every failure.

4. CI evidence

Exact Linux run status.

5. Virtual Metal freeze decision

Answer:

FREEZE
or
DO NOT FREEZE

with concrete evidence.

6. PUSH failure reconstruction

Explain where current PUSH fails.

7. One generic PUSH V2 improvement

Choose exactly ONE based on evidence.

8. Cross-robot evaluation

Development robots + held-out robot.

9. Failure dataset status

Schema, counts, labels, provenance.

10. Typed-model recommendation

Answer:

NO
NOT YET
YES

based only on actual dataset results.

11. Next milestone

Choose exactly ONE next milestone.

Do not propose another infrastructure program unless new evidence forces it.

DECISION RULE

Optimize all work according to:

increase in real physical-intelligence capability
× falsification power
× cross-robot generality
────────────────────────────────────────────
engineering cost
× speculative complexity

Reality OS has enough authority architecture.

Virtual Metal is close to enough hardware falsification infrastructure.

The project should now increasingly prove:

Can the same semantics
under uncertainty
cause useful physical outcomes
across different machines?

Close Tier 2.

Freeze the hardware lab.

Then make Reality OS physically intelligent.