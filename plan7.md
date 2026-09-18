Take ownership of the current `reality-os-rust` repository as the living product.

Do NOT use TheWorld or any stale predecessor.

The objective of this task is to remove real-hardware availability as a development bottleneck WITHOUT pretending software simulation is physical proof.

Build the strongest possible software representation of a real actuator and its surrounding hardware environment using:

- manufacturer datasheets
- official manuals
- protocol specifications
- official SDK/source code
- public engineering measurements
- academic papers
- teardown/bench data where credible
- first-principles physics
- uncertainty ranges
- adversarial parameter variation

The first target is the ROBOTIS Dynamixel XL330 family already used by Reality OS.

The result should become a reusable Reality OS capability for constructing executable hardware surrogates from engineering data.

Call the concept:

```text
Virtual Metal
```

unless the existing repository already has a better term.

Do NOT claim Virtual Metal is physical evidence.

---

# PRIMARY QUESTION

Determine:

> How much of a physical actuator, controller, communication bus, firmware state machine, power system, and failure environment can be reconstructed from public information strongly enough to falsify Reality OS software assumptions before touching real hardware?

Do not answer philosophically.

Research the data.

Build it.

Test Reality OS against it.

Measure what remains unknown.

---

# FIRST PRINCIPLE

The goal is NOT:

```text
make a visually realistic robot simulation
```

The goal is:

```text
construct the strongest executable approximation of the actual hardware contract
+
explicitly represent everything we do not know
+
attack Reality OS across that uncertainty
```

A perfect nominal simulation is less valuable than a broad adversarial family of plausible hardware.

---

# EVIDENCE CLASSES

Introduce or reuse explicit provenance levels.

At minimum distinguish:

```text
MANUFACTURER_SPECIFIED
MANUFACTURER_DERIVED
OFFICIAL_SDK_BEHAVIOR
THIRD_PARTY_MEASURED
ACADEMIC_MEASURED
PHYSICS_DERIVED
CALIBRATED_FROM_REAL_HARDWARE
ESTIMATED
UNKNOWN
```

Every hardware parameter used by Virtual Metal must carry provenance.

Do not allow undocumented constants to silently appear in production simulation logic.

For example:

```text
stall_torque_5v:
    value: 0.52 Nm
    provenance: MANUFACTURER_SPECIFIED
    source: <exact source>

gearbox_efficiency:
    range: [...]
    provenance: ESTIMATED

backlash:
    state: UNKNOWN
```

UNKNOWN must remain explicit.

---

# EVIDENCE STATUS

Virtual Metal results must NEVER appear as real hardware evidence.

Create/reuse an evidence distinction such as:

```text
SIMULATION_ONLY
VIRTUAL_METAL
MEASURED_HARDWARE
```

or an equivalent already compatible with Reality OS provenance.

The distinction must survive serialization, reports, tests and proof aggregation.

A Virtual Metal campaign may increase confidence.

It may not mint:

```text
MEASURED
METAL_MEASURED
hardware_present=true
```

or any equivalent real-hardware claim.

---

# PHASE 1 — RESEARCH THE XL330 COMPLETELY

Research current authoritative public information for XL330-M288 and relevant XL330 variants.

Prefer sources in this order:

1. ROBOTIS official e-Manual
2. ROBOTIS official SDK/source
3. official protocol documentation
4. official electrical/mechanical documentation
5. peer-reviewed / academic measurements
6. credible independent bench measurements
7. community information only where unavoidable

Collect at least:

## Identity

- model number
- firmware behavior
- ID behavior
- baud behavior
- factory defaults
- reboot semantics
- operating mode behavior

## Mechanical

- mass
- dimensions
- gear ratio
- encoder resolution
- torque-speed data
- no-load speed
- stall torque
- position range
- velocity range
- mechanical limits if documented
- gearbox characteristics if documented

## Electrical

- supported voltage range
- recommended voltage
- current limits
- PWM behavior
- voltage sensing
- thermal behavior
- shutdown conditions
- brownout behavior if documented
- startup behavior
- power-loss behavior

## Control

- position loop
- velocity loop
- gains
- feedforward
- acceleration profile
- velocity profile
- Goal PWM
- PWM Limit
- Current Limit
- Velocity Limit
- Position Limits
- Bus Watchdog
- torque enable behavior
- operating modes
- startup configuration

## Communication

- Dynamixel Protocol 2.0
- packet format
- CRC
- status packet format
- instruction set
- errors
- half-duplex behavior
- timeout semantics
- broadcast behavior
- return status behavior
- reboot
- reset
- sync/bulk operations where applicable

## Failure behavior

- watchdog
- hardware error status
- over-temperature
- under/over-voltage
- communication loss
- torque shutdown
- reboot
- EEPROM/RAM persistence rules

Produce a machine-readable inventory.

---

# PHASE 2 — BUILD AN ACTUATOR TRUTH PACK

Design the smallest reusable schema for hardware knowledge.

Conceptually:

```text
actuators/
  robotis/
    xl330-m288/
      identity
      protocol
      control_table
      controller
      mechanical
      electrical
      timing
      faults
      uncertainty
      provenance
```

Do not blindly use YAML if another representation fits the repository better.

Requirements:

- every field has provenance
- units are explicit
- version/schema is explicit
- UNKNOWN is representable
- ranges/distributions are representable
- exact values and estimated intervals are distinguishable
- manufacturer value and measured calibration can coexist
- model/firmware-specific differences can be expressed
- no robot-specific semantic behavior leaks into skill logic

This should eventually be reusable for different actuator families.

Do not over-generalize before XL330 proves the schema.

---

# PHASE 3 — BYTE-ACCURATE DYNAMIXEL VIRTUAL DEVICE

Build a virtual device that communicates through the same conceptual driver boundary as the physical actuator.

Prefer exercising the actual XL330 driver as much as possible.

Implement Protocol 2.0 behavior at packet level.

Support at least the subset Reality OS actually uses:

```text
PING
READ
WRITE
REBOOT
STATUS PACKET
CRC
device ID
baud
error/status codes
```

Expand only where current Reality OS requires it.

The virtual device must contain:

```text
EEPROM state
RAM control table state
firmware state
communication state
power state
torque state
controller state
encoder state
error state
```

The goal is for Reality OS's real driver code to believe it is talking to an XL330-compatible device.

Avoid test-only shortcuts that bypass:

```text
serialization
packet parsing
CRC
timeouts
driver state
control-table semantics
```

---

# PHASE 4 — DIFFERENTIAL TEST AGAINST OFFICIAL SDK

Use the official ROBOTIS Dynamixel SDK as an independent reference where possible.

Construct scenarios where identical request sequences are applied to:

```text
official SDK/reference expectations
vs
Virtual Metal
```

Check:

- packet construction
- CRC
- error handling
- register reads/writes
- invalid packet behavior
- model/ID behavior
- reboot transitions
- status semantics

Do not treat the SDK as infallible firmware truth.

Use it as an independent implementation reference.

Document any ambiguity between:

```text
manual
SDK
Virtual Metal
```

---

# PHASE 5 — IMPLEMENT THE CONTROL-TABLE STATE MACHINE

Do not model registers as a flat dictionary only.

Encode relevant relationships.

Examples:

```text
Torque Enable influences allowed writes

EEPROM vs RAM persistence differs

REBOOT resets RAM according to documented behavior

Bus Watchdog changes write behavior

Operating Mode changes controller semantics

Goal Position interacts with Position Limits

PWM Limit constrains Goal PWM

Voltage/temperature errors affect torque/error status
```

Reproduce only behavior supported by public evidence.

If firmware behavior is undocumented:

```text
UNKNOWN
```

and test multiple plausible implementations if it matters.

---

# PHASE 6 — PHYSICAL ACTUATOR MODEL

Create a physics-backed XL330 surrogate.

Do NOT hardcode one ideal servo.

First fit broad public constraints:

```text
no-load speed
stall torque
voltage dependence
position range
controller limits
```

Then derive a simple actuator model.

Potential physical state:

```text
theta
omega
motor_current
applied_voltage
temperature
load_torque
gearbox_state
```

Potential equation family:

```text
V = I*R + Ke*omega + L*dI/dt

tau_motor = Kt*I

tau_output =
    gear_ratio
    * efficiency
    * tau_motor
    - friction
    - load
```

Use simpler equations if sufficient.

Do not add complexity without data.

The model must respect published performance envelopes.

---

# PHASE 7 — UNCERTAINTY MODEL

This is critical.

Do not assume unknown parameters.

Represent them as uncertainty.

Examples:

```text
gearbox efficiency
static friction
viscous friction
backlash
motor resistance
torque constant
thermal constants
encoder noise
communication delay
firmware scheduling jitter
boot delay
voltage sag
USB latency
```

Where reliable bounds exist:

use them.

Where only rough engineering bounds exist:

record them as ESTIMATED.

Where nothing defensible exists:

keep UNKNOWN.

Then decide:

```text
Does the property being tested actually depend on this unknown?
```

If yes:

randomize or adversarially search it.

---

# PHASE 8 — DOMAIN RANDOMIZATION AS FALSIFICATION

Do not use randomization merely for ML robustness.

Use it to attack Reality OS assumptions.

Generate many plausible actuator instances:

```text
VirtualXL330(seed)
```

with variation across:

- voltage
- inertia
- friction
- backlash
- load
- encoder noise
- communication latency
- boot delay
- packet loss
- EEPROM startup state
- PID parameters where legally variable
- position within allowed range
- temperature
- power quality
- host timing

Run the same Reality OS tests across thousands of instances.

Ask:

> Which assumptions only work for the nominal device?

That is the purpose.

---

# PHASE 9 — FAULT-INJECTION ENGINE

Introduce an explicit fault model.

Do not scatter random `if fault` logic throughout the driver.

Create a schedule/trigger representation conceptually like:

```text
FaultTrigger:
    AtTime
    AfterPacket
    BeforeAck
    AfterAck
    BeforeStateApply
    AfterStateApply
    DuringReboot
    DuringSensorRead
    DuringCommandWrite
```

Faults should include:

```text
USB disconnect
USB reconnect
device rename/rematch
VIN cutoff
VIN brownout
UART read timeout
UART write failure
CRC corruption
partial status packet
wrong ID response
wrong model response
wrong firmware response
missing ACK
late ACK
duplicate ACK
ACK lost after physical effect
ACK returned before physical effect
state changed but ACK lost
ACK returned but state change failed
device reboot
firmware reset
host process crash
authority crash after TX
authority crash before ACK
authority crash after ACK
communication stall
device hot swap
EEPROM corruption
sensor stale
encoder jump
voltage out of range
temperature fault
watchdog expiry
```

Where a scenario is physically impossible, exclude it.

Where plausibility is unknown, tag it as adversarial hypothetical.

---

# PHASE 10 — CRASH-AT-EVERY-BOUNDARY TESTING

Systematically identify all meaningful temporal boundaries:

```text
before serialization
after serialization
before TX
during TX
after TX
before device applies write
after device applies write
before status packet
during status packet
after status packet
before journal append
after journal append
before fsync
after fsync
before response to caller
after response
```

Inject process/device failures at each boundary.

The key question:

```text
Can Reality OS ever incorrectly conclude:
- command not executed when it did execute
- command executed when it did not
- safe restart when outcome is unknown
- recoverable ESTOP when integrity is lost
```

Unknown physical outcome must remain UNKNOWN.

---

# PHASE 11 — PROPERTY-BASED TESTING

Convert key Reality OS claims into invariants.

Examples:

```text
Unauthorized software never causes virtual physical TX.

A replayed command_id never causes a second physical action.

Integrity abort cannot be cleared by autonomy recover.

Unknown outcome prevents subsequent actuation in the same ONLINE instance.

Simulation cannot mint MEASURED hardware evidence.

Identity mismatch prevents actuation.

Restart cannot silently duplicate a previously applied command.

Missing actuator state cannot become implicit zero.

Recovery cannot widen authority.

Learned components cannot mint OnlineWrite.

A command outside the physical safety envelope never reaches the actuator.
```

Use property-based testing / fuzzing where appropriate.

The tool/approach should fit Rust and the existing repository.

The important part is reproducible failing seeds.

---

# PHASE 12 — MASSIVE CAMPAIGN RUNNER

Build a campaign runner capable of executing:

```text
10,000+
100,000+
eventually 1,000,000
```

virtual hardware scenarios cheaply.

Do not optimize prematurely.

First make results reproducible.

Every run must record:

```text
seed
Virtual Metal configuration
fault sequence
Reality OS commit
truth-pack version
input command sequence
device state transitions
physical surrogate state
TX/ACK sequence
authority decisions
final verdict
invariant violations
```

A failing seed should be replayable exactly.

---

# PHASE 13 — SEARCH, NOT JUST RANDOMNESS

Random fuzzing alone is not enough.

Explore methods for finding adversarial states:

- boundary-value generation
- state-machine coverage
- mutation-based fuzzing
- property-based shrinking
- timing sweeps
- exhaustive enumeration of small state spaces
- Monte Carlo
- Latin hypercube / parameter sweeps
- optimization against failure probability
- model checking for discrete state machines where tractable

Use sophisticated methods only if they increase falsification power.

Do not add research complexity for appearance.

---

# PHASE 14 — VIRTUAL METAL EVIDENCE

Create a machine-readable campaign artifact.

Conceptually:

```text
realityos.virtual_metal/1
```

Record:

```text
Reality OS SHA
truth pack SHA/hash
Virtual Metal implementation SHA
source provenance set
parameter bounds
number of campaigns
fault coverage
invariant results
counterexamples
unknown assumptions
```

Example conclusion:

```text
VIRTUAL_METAL_PASS
```

must NEVER mean:

```text
physical hardware verified
```

Make that impossible to confuse.

---

# PHASE 15 — FIND WHERE REAL HARDWARE IS STILL REQUIRED

After building Virtual Metal, explicitly list every property that cannot be established from public data/software.

Examples may include:

```text
actual undocumented firmware behavior
true boot timing distribution
USB adapter quirks
real voltage sag
EMI
actual mechanical backlash
manufacturing variation
real holding torque
real thermal response
actual power-cut behavior
kernel/driver timing
physical hot-swap behavior
```

For each unknown, ask:

> What is the smallest physical experiment required to learn this?

This should produce a minimal calibration plan.

The goal is to reduce hardware experiments from:

```text
development dependency
```

to:

```text
targeted falsification/calibration measurements
```

---

# PHASE 16 — HARDWARE-AS-ORACLE LOOP

Design the future feedback loop:

```text
Virtual Metal predicts behavior
        ↓
real XL330 test
        ↓
compare prediction vs observation
        ↓
simulation gap
        ↓
update truth pack / uncertainty
        ↓
rerun massive campaigns
```

Never overwrite manufacturer data with one bench result.

Store calibration separately with provenance.

A real observation may:

```text
confirm
narrow uncertainty
widen uncertainty
contradict assumption
reveal new state
```

---

# PHASE 17 — CONNECTION TO ENGINEERING DATA

Investigate whether this concept should eventually integrate with the existing `engineering-data` effort.

Do not force integration now.

Ask whether a generalized pipeline could become:

```text
datasheet
protocol manual
CAD/URDF
controller documentation
public measurements
        ↓
Engineering Data normalization
        ↓
Actuator Truth Pack
        ↓
Virtual Metal generator
        ↓
Reality OS adversarial campaigns
```

If the boundary is clean, describe it.

Do not merge repositories merely because they are related.

---

# PHASE 18 — GENERALIZATION BEYOND XL330

Only after XL330 works, determine how much of the abstraction transfers to:

```text
different Dynamixel
CAN motor controller
BLDC servo
industrial EtherCAT drive
ROS-controlled actuator
```

The second target should be selected specifically to falsify XL330 assumptions.

Do not implement the second target yet unless XL330 exposes the correct abstraction.

---

# WHAT NOT TO BUILD

Do NOT:

- build another generic physics simulator
- replace MuJoCo
- replace ROS
- write a general-purpose digital-twin platform
- invent unknown constants
- claim exact servo fidelity without measurements
- special-case Reality OS tests so they pass
- bypass the real XL330 driver
- turn PTY mocks into “physical evidence”
- train an AI model for this problem unless a clear ML need appears
- build fancy visualization before falsification works
- build a giant hardware ontology
- add broad hardware support before XL330 proves the architecture

---

# REQUIRED RESEARCH DISCIPLINE

For every important parameter or behavior report:

```text
VALUE / BEHAVIOR
SOURCE
SOURCE TYPE
CONFIDENCE
USED BY
UNCERTAINTY
WHAT BREAKS IF WRONG
```

Prefer primary sources.

Where sources conflict:

record both.

Do not silently choose one.

---

# REQUIRED EXPERIMENTS

At minimum run Virtual Metal campaigns covering:

## Normal

```text
boot
identity
sensor read
hold
nudge
restart
```

## Replay

```text
execute command X
replay X
```

Required:

zero second physical action.

## Unknown outcome

```text
TX accepted
physical application uncertain
ACK lost
```

Required:

same ONLINE instance becomes non-actuating.

## Crash timing

Crash at all significant write/ACK/journal boundaries.

## Recover attack

```text
integrity abort
→ op=recover
→ fresh command
```

Required:

recovery refused, zero additional physical actions.

## Downgrade attack

```text
integrity abort
→ later ESTOP
→ recover
```

Required:

integrity remains.

## Identity

Swap:

```text
servo ID
model
firmware
calibration
virtual device instance
```

Verify binding/refusal behavior.

## Power

Sweep:

```text
healthy voltage
gradual sag
instant VIN loss
USB alive / VIN dead
VIN alive / UART dead
```

## Communication

Inject:

```text
latency
loss
corruption
truncation
duplicate packets
late packets
```

---

# SUCCESS CRITERIA FOR VIRTUAL METAL V1

Virtual Metal V1 is successful only if:

1. the real Reality OS XL330 driver is exercised rather than bypassed
2. Protocol 2.0 behavior is implemented for the required subset
3. relevant control-table semantics are modeled
4. actuator dynamics respect published hardware envelopes
5. unknown parameters are explicit
6. domain randomization covers uncertain physics
7. failure injection covers critical authority boundaries
8. campaigns are deterministic/replayable by seed
9. at least thousands of scenarios run automatically
10. counterexamples become regression tests
11. Virtual Metal evidence is impossible to confuse with measured hardware
12. the system produces a list of what still requires physical measurement

---

# MOST IMPORTANT OUTPUT

Do not merely say:

```text
Virtual Metal works.
```

Answer:

```text
Which Reality OS assumptions survived?

Which failed?

Which depend on undocumented hardware behavior?

Which public specifications were sufficient?

Which physical measurements are still necessary?

How much of the original XL330 physical campaign can now be pre-falsified entirely in software?
```

---

# FINAL DELIVERABLE

Return:

## 1. Public-data inventory

Exact XL330 facts found, with source/provenance.

## 2. Unknown-data inventory

Everything important that public data does not establish.

## 3. Truth Pack schema

Exact representation.

## 4. Virtual device architecture

Protocol + firmware + physics + power + faults.

## 5. Driver integration

Show that existing production driver paths are used.

## 6. Differential results

Virtual device vs official protocol/SDK expectations.

## 7. Physical model

Equations/assumptions/ranges.

## 8. Fault model

Exact supported fault classes.

## 9. Property suite

Every invariant tested.

## 10. Campaign results

Number of runs, coverage, failures, minimized counterexamples.

## 11. Reality OS defects discovered

Rank by severity.

## 12. Simulation-gap report

Everything still requiring real hardware.

## 13. Minimal hardware calibration plan

Only measurements that materially reduce uncertainty.

## 14. Generalization assessment

What transfers beyond XL330.

## 15. Next milestone

Exactly one next milestone justified by evidence.

---

# PRIORITY RULE

Choose work according to:

```text
increase in falsification power × reusable hardware knowledge
─────────────────────────────────────────────────────────
engineering cost × speculative complexity
```

Prefer:

```text
better hardware contract
better uncertainty
better fault coverage
more reproducible adversarial runs
better counterexamples
```

over:

```text
more architecture
more code
more abstraction
more visualization
more features
```

The objective is to make lack of hardware a minor inconvenience rather than a development blocker.

Use software, public engineering knowledge, first-principles physics and uncertainty aggressively.

Then use real hardware later to attack and improve the model—not to give permission for Reality OS development to continue.