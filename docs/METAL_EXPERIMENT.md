# First metal experiment — Dynamixel XL330

Physical-evidence milestone. Not a kernel redesign. Not certified safety.

## Chosen actuator

| Item | Value |
|------|--------|
| Actuator | Robotis Dynamixel **XL330-M288-T** (also accept XL330-M077-T) |
| Controller | Servo onboard MCU + USB–UART adapter (U2D2 / FTDI / CP2102) |
| Supply | **5.0 V**, current limited to **≤ 0.5 A** at the bench PSU |
| Interface | Dynamixel Protocol 2.0, default **57 600** 8N1 on `/dev/ttyUSB*` (probe also tries 115 200, 1 Mbps, and 2 Mbps) |
| Max configured velocity | Profile Velocity **20** (≈ 4.6 rpm) |
| Max configured effort | Current Limit **200 mA**; Reality OS `tau_max` **0.2** |
| Operating mode | Driver sets EEPROM **position control (3)** if it is not already |
| Mechanical constraint | Horn fixture or zip-tie stop; **no load**, no linkage, no person in the sweep |

Why this is low-energy: stall torque is about **0.52 N·m** at 5 V, plastic gears, no mobile base, no high-voltage bus. Unexpected motion cannot throw a mass or travel.

Do **not** start with a humanoid, industrial arm, high-torque BLDC, or anything that can travel or crush.

## Independent physical power cutoff

The servo **VIN** (not the USB data path) must pass through an operator-accessible disconnect:

* appropriately rated **bench PSU output switch**, or
* **SPST / in-line barrel disconnect** rated **≥ 2 A** on the 5 V rail.

Reality OS torque-disable is a register write. It is **not** the cutoff.

This cutoff is **not** STO, SS1, PL, or SIL unless the chosen hardware’s own documentation says it is and that certification is in force. A bench switch is none of those.

Operator test: open the switch; the servo must lose holding torque while USB/data may stay enumerated. Record `REALITYOS_METAL_CUTOFF_TESTED=1` only after that observation. Optional live measurement during the campaign: `REALITYOS_METAL_CUTOFF_LIVE=1` waits for VIN to drop, then proposes and records `write_delta`.

## Identity mapping

`probe_identity()` reports:

| Field | Source |
|-------|--------|
| `serial` | **Measured:** USB adapter serial (sysfs) + servo bus ID, else USB vid:pid:devpath, else tty name + rdev (UART/GPIO). XL330 EEPROM has **no** factory serial. |
| `firmware_id` | **Measured:** model number register 0 + firmware version register 6 (`xl330-m288:1190:<fw>`). |
| `actuator_ids` | **Measured:** `xl330:<id>`. |
| `calibration_id` | **Deployment:** `metal.json`, not EEPROM. |
| `design_content_hash` | **Deployment:** SHA-256 of `realityos.metal_design/1` (limits). Not EEPROM. |

If USB serial cannot be read, a `usb:<vid>:<pid>:<devpath>` fallback is used if sysfs exposes it. If that is also missing, the char-device name and `rdev` are used (`tty:<name>:<rdev>:id<n>`). That is measured from the OS node, not invented EEPROM. Values are not invented. `/dev/serial/by-id/*` paths are canonicalized to the real tty name before the sysfs walk. The campaign and `serve` refuse `/dev/pts/*` unless `REALITYOS_METAL_PTY_SEQUENCE=1` **and** the node is a PTY (`scripts/metal-pty-sequence.sh`). That dry-run exercises the real campaign script; it must finish with `experiment_status != measured_success`, `cutoff_tested=false`, and must **not** install `docs/metal_proof.json`. Setting the flag on a real `ttyUSB*` / `ttyACM*` is ignored.

Model and firmware are latched at identify time. Later sensor packets do not rewrite `firmware_id`.

## Composition

`realityos-metal-smoke serve` calls `RuntimeSession::start_online` → `OsMonotonicClock`. It does not use the HIL `Authority` object or `FakeClock`. `new_online` heartbeats, then `watchdog_tick_now` stamps the start watchdog after that tick's journal persist so persist latency is not counted as a 100 ms miss (driver/deployment cannot catch that up). Successful ONLINE `watchdog_tick_now` calls stamp completion after persist; a gap >100 ms with no successful tick still latches. Idle serve pets the 50 ms software watchdog about every 40 ms (Instant, scheduled from before the emit) and heartbeats about every 800 ms (each emit fsyncs journal+seal; a 10 ms pet of both was ~400 fsyncs/s and can miss the 100 ms watchdog). The idle loop accepts a waiting IPC client *before* the idle heartbeat; a heartbeat persist in the same iteration as `handle` can consume the 100 ms window (refuse at the first handle pet, empty `serve.err`). After heartbeat persist, serve refreshes the watchdog and pushes the idle Instant out. Serve does not pet again on accept — `handle` pets, and an extra fsync before acquire failed the two-UID PTY hold (sensor ok, write missed). After `handle`, the idle Instant is pushed out so status-then-propose does not add another idle fsync. Propose ticks the watchdog once on IPC accept; it does not heartbeat or re-pet between sensor acquire and the certified write (`dispatch_issued` ticks immediately before `write_online`). After the write, serve refreshes the watchdog (so the next propose does not inherit prepare/consume/emit fsync time) and then persists freshness. After identify, live I/O is one attempt with a **40 ms overall deadline** (dribbled 4×40 ms reads are 160 ms and miss the watchdog). Sensor sampling is a single 26-byte RAM block. Propose pets the watchdog again after acquire so dispatch does not inherit that read. Setup enables torque after the EEPROM settle so the first certified write is only `goal_position` (not torque-on + goal). Goal writes reuse the last sensor present and skip redundant profile/torque pokes. `scripts/metal-kill-serve.sh` kills leftover `serve` processes for a metal root so a zombie cannot rewrite journal+seal after `rm -rf` and poison `--first-online`.

Keep `REALITYOS_METAL_ROOT` on tmpfs. The campaign default is `/tmp/realityos-metal`; if that path is not already tmpfs, `scripts/metal-campaign.sh` mounts a 32 MiB tmpfs there (override with `REALITYOS_METAL_ALLOW_SLOW_DISK=1` only to debug, knowing a journal fsync >100 ms latches the software watchdog). Each watchdog/heartbeat emit fsyncs journal+seal; if a single fsync takes more than 100 ms the software watchdog latches and cannot be caught up. That is a deployment constraint, not a certified hardware watchdog.

Users: `realityos-authority` owns the tty, key, journal, and process. `realityos-autonomy` may use only the Unix socket. The metal root is `0751` so autonomy can traverse to `ipc.sock` (`0660` / `realityos-ipc`); `bus/` stays `0700`. A `0750` root would make IPC and `direct_device_open_attempts` fail closed without measuring the attacks. After each `serve` bind the campaign re-applies `0600` on the tty because udev may restore `0660 dialout`. On `ttyUSB*` / `ttyACM*` it also installs a temporary udev rule (`ID_MM_DEVICE_IGNORE`, `ID_BRLTTY=0`, owner/mode). If `fuser` still shows brltty or ModemManager on that node, the campaign stops those services for the run (and starts them again on exit). Any other holder still fails closed. `os-probe` measures `/proc/<pid>/fd` against the `realityos-metal-smoke` child, not the `sudo -u` wrapper.

Identity/disconnect ESTOP is not a software-watchdog miss. `serve` pets the watchdog on tick success only, so recover-after-identity can return `hardware_session_requires_online_restart` instead of a vacuous `software_watchdog_miss`. Campaign `force_disconnect` is an identity overlay (USB may stay enumerated) so propose reaches `verify_live_hardware` and FAULT/ABORTs the instance; clearing the hook cannot revive it. Live I/O loss (VIN drop, USB-UART timeout) is detected on acquire, before write-time verify; the metal serve latches `bus_lost` and recover cannot resurrect that instance (missing-sensor campaign hook is not a bus loss). Authorized hold/nudge must produce a physical write; hostile cases must include the expected violation token and must not be a watchdog miss. A dead session before those cases fails the campaign instead of minting `docs/metal_proof.json`. `measured_success` also requires measured identity and disconnect refusals plus device-capture and authority-receive freshness fields — hold/nudge writes alone cannot mint the proof.

## Sensor freshness

* Device capture: XL330 Realtime Tick (wrapping 1 ms counter) as `timestamp_s`.
* Freshness: authority monotonic receive time stamped by `ingest_sensor_packet`.
* Threshold: `freshness_threshold_s` in `metal.json` (default 2 s), recorded into the proof from `bus/sensor_freshness.json`.
* Autonomy cannot ingest or refresh evidence (`sensor_samples` is refused).
* Observed motion is measured from `bus/present` / `bus/goal` (device registers), not inferred from IPC status.

## Proof

`docs/metal_proof.json` schema `realityos.metal_proof/1` is written only from measured cases on a real device. `hardware_model` is taken from the probed model number (1190 → XL330-M288-T, 1200 → XL330-M077-T); a missing or unknown model refuses the proof meta. The campaign reporter writes the artifact and a 16-point `METAL_PROOF_REPORT.md` into the metal root (authority-owned). Root installs both into `docs/` only when `experiment_status` is `measured_success` (including `cutoff_tested`). The authority UID does not need write access to the repository. The reporter refuses to emit a success artifact when `hardware_present` is false. It does not overwrite `docs/hil_proof.json`. The PTY Protocol 2.0 stand-in in `crates/metal/tests/` is a driver regression test, not physical evidence. Its `HardwareIdentity.metal` is false and `evidence_status` is `PTY_STAND_IN_NOT_METAL`. `scripts/metal-os-boundary.sh` measures the two-UID filesystem/device-open boundary only; it is not physical evidence and must not write `metal_proof.json`. Serial open uses serialport exclusive mode (TIOCEXCL + flock on the tty) plus an authority-owned sidecar lock.

## How to run (bench host)

```text
# 1) Operator-test the VIN cutoff first (servo loses holding torque).
# 2) Close the cutoff again, then:
cargo build -p realityos-metal --bins
sudo -E env REALITYOS_METAL_DEVICE=/dev/ttyUSB0 \
  REALITYOS_METAL_BIN=$PWD/target/debug \
  REALITYOS_METAL_BAUD=1000000 \
  REALITYOS_METAL_CUTOFF_TESTED=1 \
  scripts/metal-campaign.sh
```

The campaign exits 2 before `init`/`probe` (which enable torque) unless `REALITYOS_METAL_CUTOFF_TESTED=1`. That flag is an operator observation, not a certified STO/SS1 function.

`REALITYOS_METAL_BAUD` and `REALITYOS_METAL_SERVO_ID` are optional. Probe tries the configured pair first, then common XL330 baud/id pairs, waits 100 ms after open before the first ping (U2D2/FTDI often drop a cold first packet), waits 100 ms after a failed pair, retries the configured pair once after the scan, and writes the working pair into `metal.json`. Serial open does **not** assert DTR/RTS: on many FTDI/CP2102 adapters DTR is servo RESET, and a toggle would reboot the XL330 on every serve start and crash-replay. The campaign udev rule is installed and triggered once (then `udevadm settle`); re-triggering on every crash-replay restart can reset `latency_timer` to 16 ms. Each campaign kills leftover `realityos-metal-smoke --root <root>` processes, wipes `REALITYOS_METAL_ROOT`, and mounts tmpfs on that path when the parent filesystem is a disk so `--first-online` is not refused by a leftover journal and idle journal fsyncs stay inside 100 ms. Crash-replay waits at most 5 s for the smoke child to exit; if `crash_if` never fires the campaign fails instead of hanging on `wait`. Set `REALITYOS_METAL_CUTOFF_TESTED=1` only after opening VIN and seeing lost holding torque; the campaign will not torque or write without it. On `ttyUSB*` / `ttyACM*` the campaign sets the USB-serial `latency_timer` to 1 ms when sysfs exposes it (FTDI/U2D2 default 16 ms can miss the 40 ms live I/O deadline), writes a temporary udev rule so ModemManager and brltty ignore that tty, stops brltty/ModemManager if they already hold the node, and exits 2 if another process still has it open. That check runs before every `serve` start (probe→serve, disconnect restart, crash-replay) and waits briefly for our own close so a leftover smoke fd is not mistaken for ModemManager. Protocol 2.0 `STATUS_ALERT` (bit 7) is leftover Hardware Error Status, not an instruction NAK; setup reboots the servo once if that latch is set, then refuses if the error remains (VIN still out of range).

`scripts/metal-os-boundary.sh` (CI `os-users`) proves the 0751 / device-open counting path with a dummy 0600 file, then a two-UID `serve` + hold on the PTY Protocol 2.0 stand-in (`REALITYOS_METAL_ALLOW_PTY=1`). That is **not** a substitute for the XL330 campaign and does not write `metal_proof.json`. `scripts/metal-pty-sequence.sh` runs the full campaign script against the same PTY responder; it is a first-contact dry-run of restart/crash-replay, not metal evidence.

First-contact script invariants (found on the PTY sequence, would fail the first XL330 run):

* Empty `METAL_CMD_ID` must not override `--id`. Autonomy used to export the empty string; propose treated `is_ok()` as a set id, minted `metal-{now}`, and made the `metal-hold` replay look like a new write.
* Planned `stop_auth` writes `$ROOT/stop_serve` so Drop torque-offs and releases the tty. SIGKILL skips Drop; a real XL330 would keep torque and the next open can get `EBUSY`.
* PTY `TIOCEXCL` survives `process::exit` (`crash_if`). Serial open uses `exclusive(false)` only on `/dev/pts/*`; real tty keeps exclusive. The sidecar flock still serializes.
* Crash-replay must not `wait $AUTH_PID` unless the smoke child actually exited. `after_write_before_ack` / `after_ack` fire only after `act()` returns Ok; a refused propose leaves serve up. Propose IPC has a 20 s I/O timeout so a wedged serve cannot hang the campaign. After `crash_if`, `fuser` on a USB-UART can still show a holder; `start_auth` retries that instead of `exit 2` (which skipped the open retry and would abort the first XL330 crash-replay).
* Journal continuity treats `replayed` as an identity marker (`repeated_refuse_n` defaults to 3). The campaign `metal-hold` replay plus two crash-replays latch the next `--restart` as `abort_latched:replayed`, so `after_write_before_ack` never reaches `crash_if`. After each measured crash-replay the campaign does one authorized reset hold (not a proof case) so the next crash point is a live instance.

This Cloud Agent VM has **no** USB/serial actuator and **no** self-hosted worker. Attach a Cursor self-hosted worker (`cursor worker start`) on the bench host that can see `/dev/ttyUSB*` / `/dev/ttyACM*`. Until that happens, the experiment is blocked. That is not a software-architecture remaining task.
