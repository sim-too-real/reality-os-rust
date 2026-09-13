You are the experiment conductor for `sim-too-real/reality-os-rust`.

This task exists to produce external physical evidence.

It is NOT a software-development task.

Do not redesign Reality OS.
Do not add capabilities.
Do not refactor.
Do not improve abstractions.
Do not tune manipulation.
Do not write a new plan.
Do not invent a new success predicate.

The experiment answers one question:

> Does the frozen Reality OS authority path permit the intended physical XL330 action exactly once while preventing hostile, invalid, replayed, ambiguous, identity-invalid, or unauthorized requests from producing unauthorized physical command egress?

## Authoritative identifiers

Experiment software snapshot:

`e3606495e445c8723754b9db8863ad3081740f21`

Last code-changing freeze:

`5c4c6e54489468584e7197c03c19f9f0f750e2c2`

Operator-plan carrier commit:

`6bb29ff086ed13a268cf91099194901d8e992617`

Historical manipulation freeze:

`b6396a784c9d09e69cafbb073a9ef3e933113990`

The experiment MUST execute software from `e360649...`.

Do NOT execute the metal experiment from current `main`.

`6bb29ff...` exists only to carry `plan2.md`; it is not the measured software snapshot.

## Step 0 — externalize the operator plan

Before changing commits, export the operator plan outside the repository.

Example:

```bash
mkdir -p ../realityos-experiment
git show 6bb29ff086ed13a268cf91099194901d8e992617:plan2.md \
  > ../realityos-experiment/plan2.md

sha256sum ../realityos-experiment/plan2.md \
  | tee ../realityos-experiment/plan2.sha256
```

Record the SHA-256.

Do not edit that exported file afterward.

The committed `.worktrees/close-actuator-invention` gitlink on current main is repository housekeeping and is OUT OF SCOPE for this experiment.

Do not repair it now.

## Step 1 — create a clean measurement checkout

Prefer a fresh clone or separate clean working directory.

Checkout exactly:

```bash
git checkout --detach e3606495e445c8723754b9db8863ad3081740f21
git rev-parse HEAD
git status --porcelain=v1
```

Required HEAD:

```text
e3606495e445c8723754b9db8863ad3081740f21
```

Tracked source tree must be clean before qualification.

Record:

```bash
git rev-parse HEAD
git show -s --format=fuller HEAD
uname -a
rustc --version
cargo --version
python3 --version
```

Do not merge, cherry-pick, pull, rebase, or update dependencies.

## Evidence model

Use the frozen repository's existing machinery.

Authoritative sequence:

`scripts/metal-campaign.sh`

Authoritative success predicate:

`MetalProof::from_measured`

Authoritative bench requirements:

`docs/METAL_EXPERIMENT.md`

Lifecycle model:

`docs/models/consume_write.tla`

The operator may collect evidence.

The operator may NOT redefine evidence.

Keep these planes separate:

```text
SOFTWARE != PTY
PTY      != HIL
HIL      != SIM
SIM      != METAL
```

No result from one plane may be promoted into another.

## Linux qualification

Run using Rust 1.95.0.

Run:

```bash
cargo fmt --all -- --check
cargo clippy --workspace --all-targets -- -D warnings

cargo test -p realityos-metal --all-targets -- --test-threads=1

cargo test --workspace --all-targets \
  --exclude realityos-metal \
  -- --test-threads=1
```

Record exact exit codes and logs.

IMPORTANT GATING RULE:

`cargo fmt --check` is CHARACTERIZATION ONLY.

The frozen Windows receipt already recorded pre-existing formatting drift.

A formatting-only failure does NOT authorize changing source and does NOT by itself prohibit the physical experiment.

Likewise, a pure lint-style warning does not authorize code changes.

Hard software stop conditions are:

```text
frozen source does not build
functional test assertion fails
metal crate fails functionally on Linux
journal semantics fail
PTY campaign fails functionally
two-UID metal boundary fails
proof-predicate tests fail
identity binding behaves incorrectly
replay / consume semantics behave incorrectly
```

If one of those occurs reproducibly:

```text
MEASURED_PRECONDITION_FAILURE
```

STOP.

Do not patch the software during this task.

Return the failure evidence.

## Unix authority qualification

Run the existing machinery, not reconstructed equivalents.

Build:

```bash
cargo build -p realityos-metal --bins
cargo build -p realityos-hil --bins
```

Run:

```bash
scripts/metal-online-journal.sh
```

Run the actual metal OS boundary:

```bash
sudo -E env \
  PATH="$PATH" \
  REALITYOS_METAL_BIN="$PWD/target/debug" \
  scripts/metal-os-boundary.sh
```

Autonomy must not gain direct access to:

```text
device
authority key
protected journal
```

IPC is the intended path.

Then run the full Protocol-2 PTY campaign:

```bash
sudo -E env \
  PATH="$PATH" \
  REALITYOS_METAL_BIN="$PWD/target/debug" \
  scripts/metal-pty-sequence.sh
```

Required PTY result:

```text
PTY_STAND_IN_NOT_METAL
experiment_status != measured_success
no repository metal proof installed
```

A PTY run is a campaign-regression measurement only.

It is never physical evidence.

HIL two-UID and MuJoCo qualification may also be executed and recorded, but they belong to independent evidence planes and do not mint or substitute for metal evidence.

## MuJoCo

Where practical, reproduce the pinned environment:

```text
Python 3.12
mujoco==3.7.0
numpy==2.2.6
```

Run the existing required MuJoCo verification.

Never weaken `ensure_mujoco_or_skip`.

If this environment cannot be established:

```text
MUJOCO_QUALIFICATION=INCOMPLETE
```

Record why.

Do not call it success.

Do not call it a metal failure.

## Metal go/no-go

Proceed to the XL330 only if the functional frozen-system gates relevant to the physical campaign are green.

Before connecting power confirm the bench matches `docs/METAL_EXPERIMENT.md`.

One XL330 only.

No meaningful load.

No person or body part in the sweep.

5 V supply.

Current limited as documented.

Existing conservative PWM/velocity/acceleration/excursion limits remain unchanged.

Do NOT increase limits because movement looks small.

USB must not back-power servo VIN.

The independent operator-accessible VIN disconnect must be tested before torque is enabled.

The bench disconnect is NOT STO, SS1, PL, SIL, or certified functional safety.

## Physical campaign

Do NOT manually reproduce:

```text
init
probe
bind
serve
hold
nudge
crash
report
```

The authoritative physical path is:

```bash
scripts/metal-campaign.sh
```

Use the documented environment only.

Example form:

```bash
sudo -E env \
  REALITYOS_METAL_DEVICE=/dev/serial/by-id/<real-adapter> \
  REALITYOS_METAL_BIN="$PWD/target/debug" \
  REALITYOS_METAL_CUTOFF_TESTED=1 \
  REALITYOS_METAL_CUTOFF_LIVE=1 \
  REALITYOS_METAL_UNPLUG_LIVE=1 \
  scripts/metal-campaign.sh
```

Prefer `/dev/serial/by-id/...` when available.

Do NOT set PTY flags on real hardware.

Do NOT force a high baud rate unless the actual servo is already configured for it.

Do NOT bypass identity failures.

Do NOT rewrite expected identity to make a different actuator pass.

## Physical evidence semantics

Never collapse these counters:

```text
egress attempt
serial TX
device ACK
post-command present
```

They are different observations.

Software `ALLOW` is not physical execution evidence.

A write attempt is not a serial TX.

A serial TX is not an ACK.

An ACK without post-command present is not sufficient evidence that the physical outcome occurred.

`physical_writes_*` must correspond to certified `serial_tx_*`.

For every unauthorized/hostile case require:

```text
serial_tx_delta = 0
physical_writes_delta = 0
unauthorized_device_ack_delta = 0
```

Any unauthorized physical TX or ACK is:

```text
UNAUTHORIZED_PHYSICAL_WRITE
```

Immediately stop the campaign, remove servo VIN, and preserve the evidence root.

Do not continue looking for a later success.

## Valid commands

The existing campaign/report predicate decides whether HOLD and NUDGE succeeded.

Do not substitute visual judgement.

A valid HOLD requires the existing measured criteria, including exactly one certified command TX, ACK, authority-owned post-command present, cage compliance, and the documented hold band.

A valid NUDGE requires the existing measured criteria, including exactly one certified TX, ACK, observed post-command motion beyond the hold-hunt band, motion toward the commanded goal, and cage compliance.

Do not loosen those criteria.

## Crash/restart

Use only the campaign's existing crash injection points.

The critical ambiguous physical window is:

```text
after_serial_tx_before_status
```

After restart, the command that may already have affected the plant must not be retransmitted.

Required:

```text
restart serial_tx_delta = 0
```

Do not manually retry the same command because its status is uncertain.

Uncertainty after possible physical effect means refuse/recover, not retry.

The TLA model describes the intended lifecycle.

The actual campaign trace is the implementation evidence.

## Physical USB unplug

The live USB-UART unplug must be physically performed when the campaign requests it.

`force_disconnect`, identity overlays, process death, and IPC failure are not substitutes.

Record actual observation.

## Physical VIN cutoff

Open the independent servo VIN path when requested.

USB data/GND may remain present.

Accepted evidence is the frozen campaign/proof machinery's VIN-specific measurement.

UART death alone is not VIN-cutoff evidence.

Software torque-disable is not VIN-cutoff evidence.

Process death is not VIN-cutoff evidence.

The campaign/report predicate decides whether the observation qualifies.

## No manual proof creation

Never create or hand-edit:

```text
docs/metal_proof.json
```

Never copy a HIL artifact into it.

Never synthesize case JSON to satisfy `report`.

Only the existing campaign/report path may install the repository proof.

If the campaign completes but the predicate returns incomplete:

```text
MEASURED_INCOMPLETE_OR_FAILED
```

That is a legitimate experiment result.

Do not turn it into success.

## Result preservation

Preserve the complete campaign root before doing anything else.

Create an immutable evidence bundle outside the repository containing:

```text
software SHA
operator-plan SHA-256
uname / toolchain versions
qualification command log + exit codes
campaign stdout/stderr
proof_meta
all case records
journal / seal evidence allowed by the experiment
generated proof/report if any
final git status
```

Archive it and compute SHA-256:

```bash
tar -czf ../realityos-experiment/metal-run.tar.gz <campaign-root-and-logs>
sha256sum ../realityos-experiment/metal-run.tar.gz
```

If `docs/metal_proof.json` exists because the frozen campaign legitimately installed it, compute:

```bash
sha256sum docs/metal_proof.json
```

Do NOT commit anything during the experiment.

## Final answer format

Return one concise experiment report.

State:

```text
SOFTWARE_SHA=
CODE_FREEZE_SHA=
OPERATOR_PLAN_SHA256=
HOST=
RUST=
PYTHON=
DEVICE=
MEASURED_IDENTITY=
```

Then report the four planes separately.

For Plane D report at minimum:

```text
valid_hold serial_tx_delta / ack_delta / post-present
valid_nudge serial_tx_delta / ack_delta / post-present

hostile case count
unauthorized serial TX count
unauthorized device ACK count

direct-device attempts / successes

identity mismatch observed
disconnect refusal observed

ambiguous crash restart retransmissions

live USB unplug observed
live VIN cutoff observed

proof experiment_status
```

Finish with exactly one verdict:

```text
METAL_PROOF=MEASURED_SUCCESS
```

or

```text
METAL_PROOF=MEASURED_INCOMPLETE_OR_FAILED
```

or

```text
MEASURED_PRECONDITION_FAILURE
```

or

```text
UNAUTHORIZED_PHYSICAL_WRITE
```

Do not write a new roadmap after the result.

STOP.

If the result is success, the next task is demonstration/customer discovery.

If the result is failure, the next software task may address only the first concrete measured failure.

No speculative hardening is allowed until then.
