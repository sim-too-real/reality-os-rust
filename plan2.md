# First measured XL330 campaign — frozen software

Repository: `sim-too-real/reality-os-rust`

This is an **operator / measurement plan**. It is not a software development plan.

```text
Executable sequence  = scripts/metal-campaign.sh
Success predicate    = MetalProof::from_measured  (crates/metal/src/proof.rs)
Bench physics        = docs/METAL_EXPERIMENT.md
Formal no-retry      = docs/models/consume_write.tla
Why Linux is required = docs/superpowers/evidence/2026-09-12-pre-metal-verify.txt
```

Do not fork a weaker human CLI path. Do not invent success criteria. Do not hand-edit a proof.

---

## Mission

Qualify the frozen Reality OS build on Linux. Only if the frozen software has no unexplained functional failure, run the first genuine measured XL330 metal campaign.

One question, after that campaign:

> Can the frozen Reality OS authority path execute a valid physical command exactly once while preventing hostile, invalid, replayed, ambiguous, or identity-invalid requests from producing unauthorized physical writes?

THIS IS NOT A SOFTWARE DEVELOPMENT TASK.

- Do not add features.
- Do not redesign.
- Do not refactor.
- Do not improve architecture.
- Do not tune manipulation.
- Do not modify the historical WX250s first-score artifact.
- Do not `cargo fmt` the frozen tree to make a check green.
- Do not fabricate successful evidence.
- Do not copy `docs/hil_proof.json` patterns into a metal proof by hand.

The default answer to any temptation to change code is:

```text
NO — MEASURE THE CURRENT SYSTEM FIRST.
```

Only a concrete, reproducible failure observed on the Linux bench, classified as a **software defect** (not environment), may justify reopening software. That reopening is a later task. It is not this plan.

---

## Immutable identifiers

```text
REPO_SNAPSHOT_SHA=e3606495e445c8723754b9db8863ad3081740f21
CODE_FREEZE_SHA=5c4c6e54489468584e7197c03c19f9f0f750e2c2
MANIPULATION_V1_FREEZE_SHA=b6396a784c9d09e69cafbb073a9ef3e933113990
```

Verified ancestry on this tree:

- `CODE_FREEZE_SHA` is an ancestor of `REPO_SNAPSHOT_SHA`.
- Snapshot `e360649` is **docs only**: `docs/superpowers/evidence/2026-09-12-pre-metal-verify.txt`.
- Code freeze `5c4c6e5` is clippy nits on verify/semantics after the hold-invention close. It does not change kernel, governor, session, metal, plant consume/ledger/write_guard.
- `docs/metal_proof.json` does **not** exist and must not exist until `scripts/metal-campaign.sh` installs it after `experiment_status=measured_success`.

Toolchain pins (do not silently accept substitutes):

```text
rust-toolchain.toml          channel = 1.95.0  (rustfmt + clippy)
crates/verify/python-requirements.txt
  mujoco==3.7.0
  numpy==2.2.6
Python for MuJoCo job        3.12
MUJOCO_GL                    disable
```

Proof schemas (do not collapse, do not overwrite):

| Artifact | Schema | What it is allowed to prove |
|---|---|---|
| `docs/metal_proof.json` | `realityos.metal_proof/1` | Physical XL330 campaign. Only from the campaign reporter. |
| `docs/hil_proof.json` | `realityos.hil_proof/2` | Virtual HIL. Not metal. Do not overwrite. |
| `docs/hil_os_users_proof.json` | HIL two-UID | Process topology on virtual endpoint. Not metal. |
| `verify-out/verification_report.json` | SIM | Must stay `metal=false`, `evidence_status=SIMULATION_ONLY`. |

---

## Evidence planes — do not collapse

Four planes. Passing one never mints another.

```text
Plane A  software qualification     cargo fmt / clippy / tests
Plane B  Unix authority topology    PTY campaign + two-UID boundary + journal helper
Plane C  SIM / MuJoCo               verify crate; metal=false forever
Plane D  physical XL330             scripts/metal-campaign.sh on a real UART
```

```text
HIL  ≠  metal
PTY  ≠  metal
SIM  ≠  metal
operator attestation ≠ live VIN evidence
force_disconnect / hot_swap.json ≠ USB unplug
software torque-disable ≠ VIN cutoff
IPC / process death ≠ VIN cutoff
write attempt ≠ serial TX ≠ device ACK ≠ present motion
```

Composition of Plane D (already in the frozen tree; do not redesign):

```text
autonomy UID
  realityos-metal-propose     ProductionProposal (verb, action, command_id)
                              must not send now_s / write_now_s / safety TTL
        │  Unix socket 0660, group realityos-ipc
        ▼
authority UID
  realityos-metal-smoke serve
    RuntimeSession::start_online + OsMonotonicClock   (not HIL Authority / FakeClock)
      RealityOs::decide → IssuedCommand
        RuntimeGovernor<OnlineLocked>::authorize_issued → OnlineWrite
          write_online
            CommandLedger.prepare
              Xl330Driver write_all+flush     = certified serial TX
                Status Packet                 = device ACK
                  consume
                    authority-owned sensor    = post-command present
```

The crash matrix is the physical measurement of `docs/models/consume_write.tla`: a command causes at most one physical actuation attempt; after crash/restart the system never retries a command that may already have affected the plant.

---

## Stop taxonomy

Use these labels. Do not invent success by renaming a failure.

| Label | Meaning | Next action |
|---|---|---|
| `ENVIRONMENT_SETUP_FAILURE` | Missing users, python3, timeout, fuser, udevadm, tmpfs, UART holder, toolchain pin, MuJoCo env | Fix the environment. Retry. Do not patch Reality OS. |
| `MEASURED_PRECONDITION_FAILURE` | Frozen software behaved wrongly on Linux (test, clippy, PTY sequence, identity bind, ledger) | **STOP.** Record command, error, logs, SHA. Do not patch automatically. Do not run Plane D. |
| `MEASURED_INCOMPLETE_OR_FAILED` | Campaign ran; a required gate was not observed | Preserve metal root + logs. Do **not** install `docs/metal_proof.json` as success. |
| `UNAUTHORIZED_PHYSICAL_WRITE` | Hostile/invalid/replay/ambiguous case produced `serial_tx_delta > 0` or unauthorized ACK | **STOP the campaign.** Power-remove VIN. Preserve evidence. This is the invariant failure. |
| `PTY_STAND_IN_NOT_METAL` | Unix dry-run. `experiment_status` must not be `measured_success`. Must not install repo proof. | Expected for Plane B. |
| `MUJOCO_QUALIFICATION=INCOMPLETE` | Pinned MuJoCo env could not be created | Record reason. Does not mint metal. Does not by itself prove a metal defect. |
| `METAL_PROOF=MEASURED_SUCCESS` | Campaign reporter + install path wrote `docs/metal_proof.json` with `experiment_status=measured_success` | Report, then **STOP**. |

Environment vs software (mandatory classification before any patch thought):

- **Environment:** no `/dev/ttyUSB*`, ModemManager holds the UART, `/tmp` noexec, journal not on tmpfs so watchdog misses, wrong rustc, Python 3.10 vs 3.12, USB back-powering VIN, leftover `REALITYOS_METAL_BAUD=1000000`.
- **Software:** PTY sequence fails the same way CI would; identity mismatch is accepted; replay retransmits; `from_measured` would mint success without VIN tokens; autonomy opens the tty.

If unsure: classify as incomplete, not success.

---

## Counter definitions — do not confuse

These are already distinct in `CaseRecord`. Using the wrong one is a plan failure.

| Name | What it is | What it is not |
|---|---|---|
| `egress_attempt_delta` / `command_egress_attempts` | Intent to emit a certified frame (`record_attempt`) | Not a physical write |
| `serial_tx_delta` | Certified command frame passed `write_all`+`flush` | Not setup/sensor traffic |
| `physical_writes_*` | **Copies of** `serial_tx_*` | Not `bus/writes` attempts. Mismatch ⇒ not success |
| `device_ack_delta` | Successful Status Packet | Not “software ALLOW” |
| `observed_present_after` | Authority-owned present after the command | Not Goal Position, not IPC status |
| `unauthorized_device_ack_delta` | ACK on a case with `expected_authorization=false` | Must be 0 |
| `direct_device_open_attempts` / `_successes` / `direct_device_write_successes` | Autonomy UID vs the device node (os-probe) | IPC propose is not a direct-device open |

Hold / nudge experiment acceptance (not certified accuracy):

```text
HOLD_STILL_MAX_ABS_TICKS = 4     (~0.35°)
valid_hold  : |present delta| ≤ 4, inside session cage, TX=1, ACK≥1
valid_nudge : |present delta| > 4, toward commanded goal, present and goal inside cage, TX=1, ACK≥1
certified step = 32 ticks (~2.8°) at tau_max=0.2
session cage  = startup present ± max_total_excursion_ticks (default 48), clamped to XL330 0..4095 and leftover Wizard window
```

---

## Traceability matrix

Every phase of this plan must land on an existing artifact. If it does not, the phase is wrong — not the software.

| This plan | Existing artifact | Must not |
|---|---|---|
| Plane A fmt/clippy/tests | `.github/workflows/authority.yml` `verify` job, with **pinned 1.95.0** | Use `@stable` as the pin; smash metal tests into the workspace race |
| Plane B two-UID metal | `scripts/metal-os-boundary.sh` | Treat as metal evidence; write `metal_proof.json` |
| Plane B PTY campaign | `scripts/metal-pty-sequence.sh` → `metal-campaign.sh` with `REALITYOS_METAL_PTY_SEQUENCE=1` | Install `docs/metal_proof.json`; claim `measured_success` |
| Plane B journal helper | `scripts/metal-online-journal.sh` | Delete journal to fake first boot |
| Plane B HIL two-UID | `scripts/hil-os-users-ci.sh` | Collapse with metal OS boundary |
| Plane C MuJoCo | `.github/workflows/authority.yml` `verify-mujoco` | Relax `ensure_mujoco_or_skip`; call SIM `metal` |
| Plane D bench physics | `docs/METAL_EXPERIMENT.md` | Label the bench switch STO/SS1/PL/SIL |
| Plane D operator path | `scripts/metal-campaign.sh` | Manual `init/probe/serve/report` as the success path |
| Plane D propose | `realityos-metal-propose` as autonomy | Authority proposing as root and calling it two-UID |
| Plane D identity | `probe` + `bind-measured` + `start_online` expected vs measured | Rewrite expected values to match a different servo |
| Plane D HOLD/NUDGE | campaign cases `valid_hold` / `valid_nudge` | Infer motion from Goal; hardcode `+0.2` |
| Plane D hostile | campaign `add_case` list below | Software refuse without TX/ACK deltas |
| Plane D crash | `crash_if` points below + TLA `unknown` | Retransmit after `after_serial_tx_before_status` |
| Plane D unplug | `REALITYOS_METAL_UNPLUG_LIVE=1` live prompt | `force_disconnect` hook as unplug |
| Plane D VIN | `dxl_vin_unreadable` / `dxl_vin_outside_wizard_limits` / `bus/vin` < 2.0 V | UART death, serve death, torque-disable |
| Plane D proof | `realityos-metal-smoke report` + campaign install | Hand-edit JSON; copy HIL proof |
| Non-claims | `CLAIM_LEDGER.md`, `proof.rs` `default_unresolved()` | Promote process-isolation to physical safety |

---

## Phase 0 — establish immutable baseline

Checkout exactly:

```bash
git checkout e3606495e445c8723754b9db8863ad3081740f21
git status --short
git rev-parse HEAD
rustc --version
```

Required:

```text
HEAD=e3606495e445c8723754b9db8863ad3081740f21
working tree clean of source changes
rustc 1.95.0   (rust-toolchain.toml; do not use a random stable)
```

`plan2.md` itself may be untracked in the working tree. That is this operator document. It is not a source change to the freeze. Do not commit it as part of the experiment. Do not modify tracked source.

Optional immutable tag (do not move afterward):

```bash
git tag pre-metal-e360649 e3606495e445c8723754b9db8863ad3081740f21
```

Record Windows named holes from the snapshot receipt so Linux does not “re-prove” them as success:

```text
host=windows cannot compile realityos-metal (nix termios, serialport TTYPort)
cargo fmt EXIT=1 on that Windows host was recorded and NOT applied
Unix metal scripts = named hole on Windows
MuJoCo 3.12 gate = not run on Windows
docs/metal_proof.json was not written
```

---

## Plane A — Linux software qualification

Run on the Linux bench host. Use the repository-pinned toolchain.

Match CI isolation. Metal ONLINE software watchdog misses if a journal+seal fsync exceeds 100 ms; do not smash metal tests inside the workspace run.

```bash
rustc --version | grep -F "1.95.0"

cargo fmt --all -- --check
cargo clippy --workspace --all-targets -- -D warnings

cargo test -p realityos-metal --all-targets -- --test-threads=1
cargo test --workspace --all-targets --exclude realityos-metal -- --test-threads=1
```

Record exact command, exit code, host (`uname -a`), rustc version, and relevant output.

Classification:

- fmt/clippy/test fail because rustc is not 1.95.0 → `ENVIRONMENT_SETUP_FAILURE`
- fmt fails on frozen sources under 1.95.0 → `MEASURED_PRECONDITION_FAILURE`. **Do not** `cargo fmt` to make it pass. The Windows receipt already refused that.
- metal tests fail with Unix compile errors on Linux → `MEASURED_PRECONDITION_FAILURE` (this host is supposed to compile them)
- metal tests fail with watchdog miss and the journal is on a disk → `ENVIRONMENT_SETUP_FAILURE` (put later campaign root on tmpfs; for cargo tests, re-run isolated before calling it a defect)
- genuine assertion failure in frozen tests → `MEASURED_PRECONDITION_FAILURE` → **STOP**

Do not weaken `-D warnings`. Do not `--exclude` metal/vport/hil on Linux the way Windows had to.

---

## Plane B — Unix authority topology (not metal)

These scripts require **root** to switch UIDs. Root is not the tested actor. Do not weaken permissions or run everything as one user.

Build first:

```bash
cargo build -p realityos-metal --bins
cargo build -p realityos-hil --bins
```

### B1. Journal helper (no hardware)

```bash
scripts/metal-online-journal.sh
```

This asserts `--first-online` vs `--restart` from `driver.jsonl` / `.authority-seal`. It must not delete a journal to fake first boot.

### B2. Two-UID metal OS boundary (not physical evidence)

```bash
sudo -E env PATH="$PATH" REALITYOS_METAL_BIN="$PWD/target/debug" scripts/metal-os-boundary.sh
```

Expected security topology:

- authority UID owns authority secrets / journal / device node
- autonomy UID cannot directly open device, signing key, or protected journal
- IPC is the permitted boundary
- dummy device is a regular `0600` file, never a character device
- this script **never** writes `docs/metal_proof.json`

### B3. HIL two-UID (connected topology, different evidence plane)

```bash
sudo -E env PATH="$PATH" REALITYOS_HIL_BIN="$PWD/target/debug" \
  REALITYOS_OS_USERS_PROOF="$PWD/docs/hil_os_users_proof.json" \
  scripts/hil-os-users-ci.sh
```

This is virtual-endpoint process isolation (`docs/HIL.md`). It is **not** metal. Do not treat a green HIL proof as XL330 evidence.

### B4. Full campaign on Protocol 2.0 PTY stand-in (not metal)

```bash
sudo -E env PATH="$PATH" REALITYOS_METAL_BIN="$PWD/target/debug" scripts/metal-pty-sequence.sh
```

Required:

```text
docs/metal_proof.json is still absent in the repo
experiment_status != measured_success
cutoff_live_observed = false
unplug_live_observed = false
identity evidence_status = PTY_STAND_IN_NOT_METAL  (or HardwareIdentity.metal = false)
```

This is the first-contact dry-run of restart/crash-replay, inbound nudge sign, torque-wrap cage recenter. It exists so campaign-script bugs fail before the XL330. It is **not** a substitute for Plane D.

If B4 fails: classify environment vs software. A PTY sequence failure that CI `os-users` would also catch is `MEASURED_PRECONDITION_FAILURE` → **STOP** before the servo.

---

## Plane C — MuJoCo SIM qualification (independent)

This qualifies the **frozen verify crate**, not the actuator.

It must never be called metal. A green MuJoCo job does not authorize Plane D. A missing MuJoCo env does not prove the XL330 path is broken.

Use Python **3.12** and the pinned requirements. Do not use the Windows 3.10 + numpy 1.26.4 substitute.

```bash
python -c "import sys; assert sys.version_info[:2] == (3, 12), sys.version"
python -m pip install -r crates/verify/python-requirements.txt
python -c "import mujoco, numpy; assert mujoco.__version__ == '3.7.0'; print('mujoco', mujoco.__version__, 'numpy', numpy.__version__)"

REALITYOS_REQUIRE_MUJOCO=1 MUJOCO_GL=disable cargo test -p realityos-verify --all-targets

REALITYOS_REQUIRE_MUJOCO=1 REALITYOS_VERIFY_SEEDS=2 MUJOCO_GL=disable cargo run -p realityos-verify -- milestone
```

Then assert the report is simulation only (same checks as CI):

```text
verify-out/verification_report.json
  metal            = false
  metal_verified   = false
  evidence_status  = SIMULATION_ONLY
  scheduled_jobs   = 3 * 10 * 2 * 1
  scheduled        = completed + infra
```

Do not modify `ensure_mujoco_or_skip`. Do not relax pins.

Classification:

- Cannot create Python 3.12 + mujoco 3.7.0 + numpy 2.2.6 → `MUJOCO_QUALIFICATION=INCOMPLETE` with the exact reason. Plane D may still proceed **only if** Planes A and B passed.
- Env exists and tests fail → `MEASURED_PRECONDITION_FAILURE` → **STOP**. This is freeze software, not metal, but this plan qualifies the freeze before energizing a servo.
- Pass → record `SIMULATION_ONLY`. Proceed to the pre-hardware gate.

---

## Phase 3 — pre-hardware stop gate

Fill this table from measured commands. Empty cells are not passes.

```text
HEAD:
rustc:
Linux fmt:
Linux clippy:
Metal tests (isolated):
Workspace tests (exclude metal):
metal-online-journal.sh:
metal-os-boundary.sh:
hil-os-users-ci.sh:
metal-pty-sequence.sh (no repo metal_proof.json):
MuJoCo required job:
MuJoCo milestone + SIMULATION_ONLY:
Working tree source clean:
docs/metal_proof.json absent:
```

Proceed to Plane D only if:

1. Planes A and B have **no** `MEASURED_PRECONDITION_FAILURE`.
2. Plane C is either pass (`SIMULATION_ONLY`) or explicit `MUJOCO_QUALIFICATION=INCOMPLETE` (environment), not a failed required job.
3. `docs/metal_proof.json` is still absent.

If a failure is environment/setup: fix the environment, re-run that plane.

If a failure is frozen software:

```text
MEASURED_PRECONDITION_FAILURE
```

Include: exact command, exact error, relevant logs, SHA, why this is software rather than environment. **Do not patch. Do not connect the XL330.**

---

## Plane D — physical XL330 campaign

### D0. Bench (operator). Not STO.

Use **one** XL330 in a deliberately low-energy configuration. Limits are already in `docs/METAL_EXPERIMENT.md` / `metal.json`; do not raise them because the motion looks unimpressive.

```text
actuator     XL330-M288-T (also accept M077-T)
supply       5.0 V, current limited ≤ 0.5 A
PWM cap      max_pwm_limit_raw default 200  (output/PWM cap, not certified torque)
profile      velocity 20, accel 10
cage         32-tick certified step, 48-tick session excursion
load         none; horn fixture or zip-tie that still allows ~2.8°
cutoff       independent VIN switch on the 5 V rail, ≥ 2 A rated
USB data     must not back-power XL330 VIN (isolate center pin from USB 5 V)
```

The physical switch is an experimental cutoff.

Do **not** describe it as STO, SS1, PL, SIL, or certified functional safety.

Operator VIN pretest (required before any torque):

1. USB data may stay enumerated.
2. Open the independent VIN switch.
3. Observe **lost holding torque**.
4. Close the switch again.
5. Only then set `REALITYOS_METAL_CUTOFF_TESTED=1`.

That flag is **operator attestation**. It cannot mint `measured_success`. Live campaign measurement still has to observe VIN tokens.

Hard physical power removal must remain available for the whole run. Operator can interrupt servo VIN immediately.

### D1. Identity: refuse, do not retune expected values

Before the campaign enables torque, the existing `probe` path must observe:

1. USB adapter identity (sysfs serial, else vid:pid:bus:devpath, else tty+rdev)
2. servo present on the bus (broadcast PING; refuse `dxl_multiple_servos_on_bus`)
3. model number (1200 M288 / 1190 M077)
4. firmware version
5. bus ID
6. present position
7. present input voltage
8. deployment `calibration_id` / `design_content_hash` from `metal.json` (not EEPROM)

`bind-measured` copies measured serial + firmware into expected. ONLINE start refuses when expected ≠ measured.

If the operator’s expected identity differs from what probe measures: **REFUSE**. Do not edit expected values to make the experiment pass. Do not continue.

XL330 EEPROM has **no** factory unique actuator serial. This experiment binds USB adapter + bus ID + model + firmware + deployment calibration/design. That is a named non-claim, not a bug to “fix” by inventing a serial.

### D2. The operator path is the campaign script

Do **not** mint a proof by walking `init → probe → bind-measured → serve → report` by hand. Those subcommands exist for the script and for diagnosis after a measured failure.

Required live flags (campaign exits 2 without them on a real UART):

```text
REALITYOS_METAL_DEVICE          live character device (ttyUSB* / ttyACM* / ttyCH341* or by-id)
REALITYOS_METAL_CUTOFF_TESTED=1 operator saw lost holding torque
REALITYOS_METAL_CUTOFF_LIVE=1   operator will open VIN when prompted
REALITYOS_METAL_UNPLUG_LIVE=1   operator will unplug USB-UART when prompted
```

Forbidden on the live command:

```text
REALITYOS_METAL_BAUD=1000000     (or any 2/3/4 Mbps hint unless the servo is already there)
REALITYOS_METAL_ALLOW_PTY=1      on a real UART
REALITYOS_METAL_PTY_SEQUENCE=1   on a real UART
editing expected identity to match a different servo
```

Build, then run **as root** so UID switching is real:

```bash
cargo build -p realityos-metal --bins

sudo -E env REALITYOS_METAL_DEVICE=/dev/ttyUSB0 \
  REALITYOS_METAL_BIN=$PWD/target/debug \
  REALITYOS_METAL_CUTOFF_TESTED=1 \
  REALITYOS_METAL_CUTOFF_LIVE=1 \
  REALITYOS_METAL_UNPLUG_LIVE=1 \
  scripts/metal-campaign.sh
```

Prefer `/dev/serial/by-id/...` when the adapter has one. The campaign rematches USB identity after udev `change`.

The campaign already:

- creates `realityos-authority` / `realityos-autonomy` / group `realityos-ipc` if missing
- requires `python3`, `timeout`, and on real USB-serial `fuser` + `udevadm`
- mounts tmpfs on `/tmp/realityos-metal` unless already tmpfs
- kills leftover `serve` for that root
- `init` / `probe` / `bind-measured` / `serve` / measured cases / `report`
- installs `docs/metal_proof.json` **only** when `experiment_status=measured_success`

Keep `REALITYOS_METAL_ROOT` on tmpfs. A disk fsync >100 ms latches `software_watchdog_miss` and is an environment failure, not a reason to disable the watchdog.

### D3. Cases the campaign already measures — do not skip or reorder

Authorized physical:

| Case | Required |
|---|---|
| `valid_hold` | ALLOW, `serial_tx_delta=1`, `physical_writes_delta=1`, ACK, present inside hunt band and cage |
| `valid_nudge` | inbound 32-tick step (`scripts/metal-nudge-action.sh`), TX=1, ACK, present moved farther than hunt **toward** goal, present and goal inside cage |

Hostile / invalid (each: `serial_tx_delta=0`, `physical_writes_delta=0`, `unauthorized_device_ack_delta=0`):

```text
unsupported_action
oversized_action
nan_action
inf_action
replay                  (same command_id metal-hold)
malformed_json
hil_fault_refused
caller_time_refused     (now_s injection)
forged_sensor_refused   (autonomy sensor_samples)
missing_sensor
firmware_mismatch       (hot_swap.json hook — identity token required)
recover_after_identity
reconnect_foreign
device_disconnect       (force_disconnect hook — not the live unplug)
recover_after_disconnect
reconnect_after_disconnect
```

Plus os-probe: autonomy direct-device open/write attempts > 0, successes = 0.

Crash / restart (`crash_if` points; restarted process must not retransmit):

```text
before_prepare
after_prepare_before_write
during_write                         (after egress attempt, before transport)
after_serial_tx_before_status        (THE ambiguous case: TX done, status unknown)
after_write_before_ack
after_ack
```

Critical ambiguous case:

```text
TX occurred
status result uncertain
process crashes
system restarts
same command is presented again
→ NO RETRANSMISSION
```

That is TLA `unknown` + ledger consume. Prefer refuse over double execution. Record serial TX before/after every restart. After each measured crash-replay the campaign does one authorized **reset hold** (not a proof case) so the next crash point is a live instance — do not skip that and then call later crash windows unmeasured.

Live physical (not hooks):

```text
usb_unplug_live     REALITYOS_METAL_UNPLUG_LIVE prompt
vin_cutoff_live     REALITYOS_METAL_CUTOFF_LIVE prompt; VIN tokens required
```

### D4. Operator live prompts

**USB-UART unplug** while ONLINE:

- authority detects loss
- current ONLINE session latched unusable
- no further physical writes (`serial_tx_delta=0`)
- reconnect does not silently restore old authority
- full valid ONLINE restart/bind required
- `force_disconnect` is a campaign hook and does **not** satisfy this gate
- if unplug kills serve, record measured serial_tx and the drop evidence file; do not invent `online_hardware_disconnected`

**VIN cutoff** with USB data topology still separate:

Required evidence is one of:

```text
dxl_vin_unreadable
dxl_vin_outside_wizard_limits
measured bus/vin < 2.0 V
```

The measured unauthorized case must **carry** a VIN token with `serial_tx_delta=0`. The boolean `cutoff_live_observed` alone is not enough (`live_vin_cutoff_measured` in `proof.rs`).

Not sufficient: generic UART failure, process death, IPC failure, software torque disable, USB unplug tokens (`metal_serial_closed`, `online_hardware_disconnected`).

### D5. Proof generation

Only the campaign reporter (`realityos-metal-smoke report` → `MetalProof::from_measured`) may produce the artifact. Campaign installs into the **script’s repo** `docs/metal_proof.json`, not `$PWD/docs`.

`measured_success` is the conjunction already implemented — do not weaken it in this plan. Minimum:

```text
hardware_present = true
not PTY stand-in
schema = realityos.metal_proof/1
used_os_monotonic_clock = true          (from bus/sensor_freshness.json, not hardcoded)
used_hardware_driver_port = true        (HardwareDriverPort+Xl330Driver, not hardcoded)
PWM cap requested and read back
absolute position cage active
valid_hold measured
valid_nudge measured
unauthorized serial TX = 0
unauthorized physical_writes_delta = 0  (and physical_writes_* copy serial_tx_*)
unauthorized ACK = 0
duplicate retransmission = 0
crash_restart_after_serial_tx_before_status serial_tx_delta = 0
crash_restart_before_prepare serial_tx_delta = 0
all CrashRecoveryBlocked restart cases serial_tx_delta = 0
direct-device open attempts > 0, open successes = 0, write successes = 0
identity_mismatch_refusals > 0          (violation tokens, not case-name substrings)
disconnect_refusals > 0                 (tokens)
unplug_live_observed = true
cutoff_live_observed = true
live VIN token on a measured unauthorized case
device_capture_s and authority_receive_s present
```

If any required gate fails: **do not** create a success proof. Campaign must leave `experiment_status=measured_incomplete_or_failed` and must **not** install repo success. Preserve `$REALITYOS_METAL_ROOT` (default `/tmp/realityos-metal`) including `os_metal_cases.json`, `proof_meta.json`, `serve.err`, VIN/unplug evidence files.

A measured failure is valuable evidence.

### D6. Diagnosis-only CLI (not the success path)

If the campaign stops, diagnosis may use:

```text
realityos-metal-smoke init | probe | bind-measured | serve | report
realityos-metal-propose
```

Rules:

- `probe` must not write EEPROM or enable torque (except torque-off if Startup Configuration already enabled it; that is not command egress).
- `serve` enables torque and applies bench limits.
- `report` refuses without `proof_meta.json` from the campaign.
- Do not run `report` against hand-built cases.
- Do not set `REALITYOS_METAL_ALLOW_PTY=1` on a real tty to “make it work.”

---

## Phase 13 — final report

Return only measured facts. Map them from campaign artifacts, not from memory.

### Baseline

```text
REPO_SNAPSHOT_SHA=
CODE_FREEZE_SHA=
rustc=
working tree before experiment=
working tree after experiment=
```

Allowed working-tree delta:

```text
docs/metal_proof.json          only if experiment_status=measured_success
docs/METAL_PROOF_REPORT.md     only with that success install
docs/hil_os_users_proof.json   Plane B HIL two-UID may rewrite this; it is not metal
```

Source trees (`crates/`, `scripts/`, `robots/`) must be unchanged. A rustfmt or clippy “fix” is a plan violation. Do not overwrite `docs/hil_proof.json`.

### Linux qualification

Every Plane A/B/C command and exit code. Include the Windows receipt named holes so the two hosts are not mixed.

### Hardware identity

Measured adapter / XL330 model / firmware / bus ID / deployment calibration / design hash / cage min-max / PWM requested vs read back / startup present / VIN.

### Valid physical commands

For `valid_hold` and `valid_nudge`:

```text
command
authority result
certified TX delta
physical_writes delta   (must equal serial_tx)
ACK delta
observed_present_after
observed_motion
commanded_goal
cage min/max
bounded physical result
```

### Hostile matrix

For each case in D3:

```text
case
authority result
violation tokens
serial TX delta
ACK delta
physical result
```

### Crash matrix

For each `crash_if` point: TX before, TX after restart, whether the same command was retransmitted. Explicit line for `after_serial_tx_before_status`.

### USB disconnect

Hook cases vs **live** unplug. Show tokens, TX, whether session resurrected.

### VIN cutoff

Exact physical VIN evidence (token and/or `bus/vin`). State USB data still enumerated or not, as observed.

### Totals

```text
authorized physical writes =
unauthorized serial TX =
unauthorized ACK =
duplicate retransmissions =
direct-device open attempts / successes / write successes =
```

### Proof

One of:

```text
METAL_PROOF=MEASURED_SUCCESS
```

or

```text
METAL_PROOF=MEASURED_INCOMPLETE_OR_FAILED
```

or, if stopped before Plane D:

```text
MEASURED_PRECONDITION_FAILURE
```

If success, the generated `docs/metal_proof.json` and `docs/METAL_PROOF_REPORT.md` (16-point report). If failure, the first concrete violated assumption, with the case name and counters.

---

## Named holes — do not promote claim levels

From `CLAIM_LEDGER.md` and `default_unresolved()`. Reporting success does **not** close these.

- Root can open any endpoint; root is outside this threat model.
- Same-UID `chmod` or `/proc/<pid>/fd` can recover a locked device node.
- Journal+seal is a hash-chain / local seal, not WORM or anti-rollback.
- Paired restore of an older journal+seal looks like that earlier valid tip.
- Filesystem `signing.key` is not a TPM/HSM.
- XL330 torque-disable is a register write, not independent power cutoff.
- No unique factory actuator serial; identical model+fw behind the same adapter+ID may be indistinguishable.
- PWM Limit is an output cap, not a certified torque limit.
- Software watchdog pets are in-memory; not an independent hardware watchdog.
- Hold-still ±4 ticks is experiment acceptance, not certified positioning.
- No STO/SS1/PLC/SIL/ISO is provided or claimed.
- HIL exclusive virtual endpoint is not this experiment.
- Fieldbus generality / other robots remain named holes. One XL330 smoke is not a robot framework.

---

## Absolute stop condition

After reporting the physical result: **STOP.**

Do not start:

- Manipulation V2
- PLACE / insertion / tactile
- VLA integration
- new robot support
- new architecture
- certification claims
- generalized safety framework
- “now that metal worked, raise PWM / cage / load”
- rewriting `docs/ROBOT_CONNECTION.md` to pretend the fieldbus hole is closed for arbitrary robots
- rewriting `CLAIM_LEDGER.md` from NOT_EVIDENCE to ISO/PL/SIL

Measure the one question first. Then stop.
)
