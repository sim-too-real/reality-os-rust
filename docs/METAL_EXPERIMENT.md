# First metal experiment — Dynamixel XL330

Physical-evidence milestone. Not a kernel redesign. Not certified safety.

## Chosen actuator

| Item | Value |
|------|--------|
| Actuator | Robotis Dynamixel **XL330-M288-T** (also accept XL330-M077-T) |
| Controller | Servo onboard MCU + USB–UART adapter (U2D2 / FTDI / CP2102) |
| Supply | **5.0 V**, current limited to **≤ 0.5 A** at the bench PSU |
| Interface | Dynamixel Protocol 2.0, default **57 600** 8N1 on `/dev/ttyUSB*` (probe also tries 115 200 and 1 Mbps) |
| Max configured velocity | Profile Velocity **20** (≈ 4.6 rpm) |
| Max configured effort | Current Limit **200 mA**; Reality OS `tau_max` **0.2** |
| Max position step | **8 ticks** (≈ 0.7°) |
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

If USB serial cannot be read, a `usb:<vid>:<pid>:<devpath>` fallback is used if sysfs exposes it. If that is also missing, the char-device name and `rdev` are used (`tty:<name>:<rdev>:id<n>`). That is measured from the OS node, not invented EEPROM. Values are not invented. `/dev/serial/by-id/*` paths are canonicalized to the real tty name before the sysfs walk. The campaign and `serve` refuse `/dev/pts/*` so the PTY stand-in cannot emit `docs/metal_proof.json`.

Model and firmware are latched at identify time. Later sensor packets do not rewrite `firmware_id`.

## Composition

`realityos-metal-smoke serve` calls `RuntimeSession::start_online` → `OsMonotonicClock`. It does not use the HIL `Authority` object or `FakeClock`.

Users: `realityos-authority` owns the tty, key, journal, and process. `realityos-autonomy` may use only the Unix socket.

## Sensor freshness

* Device capture: XL330 Realtime Tick (wrapping 1 ms counter) as `timestamp_s`.
* Freshness: authority monotonic receive time stamped by `ingest_sensor_packet`.
* Threshold: `freshness_threshold_s` in `metal.json` (default 2 s), recorded into the proof from `bus/sensor_freshness.json`.
* Autonomy cannot ingest or refresh evidence (`sensor_samples` is refused).
* Observed motion is measured from `bus/present` / `bus/goal` (device registers), not inferred from IPC status.

## Proof

`docs/metal_proof.json` schema `docs/metal_proof.json` schema `realityos.metal_proof/1` is written only from measured cases on a real device. The reporter refuses to emit a success artifact when `hardware_present` is false. It does not overwrite `docs/hil_proof.json`. The PTY Protocol 2.0 stand-in in `crates/metal/tests/` is a driver regression test, not physical evidence. The PTY Protocol 2.0 stand-in in `crates/metal/tests/` is a driver regression test, not physical evidence.

## How to run (bench host)

```text
# as a user who can sudo, with the XL330 powered and the cutoff closed
cargo build -p realityos-metal --bins
sudo -E env REALITYOS_METAL_DEVICE=/dev/ttyUSB0 \
  REALITYOS_METAL_BIN=$PWD/target/debug \
  REALITYOS_METAL_BAUD=1000000 \
  REALITYOS_METAL_CUTOFF_TESTED=1 \
  scripts/metal-campaign.sh
```

`REALITYOS_METAL_BAUD` and `REALITYOS_METAL_SERVO_ID` are optional. Probe tries the configured pair first, then common XL330 baud/id pairs, and writes the working pair into `metal.json`. Each campaign wipes `REALITYOS_METAL_ROOT` so `--first-online` is not refused by a leftover journal.

This Cloud Agent VM has **no** USB/serial actuator and **no** self-hosted worker. Attach a Cursor self-hosted worker (`cursor worker start`) on the bench host that can see `/dev/ttyUSB*` / `/dev/ttyACM*`. Until that happens, the experiment is blocked. That is not a software-architecture remaining task.
