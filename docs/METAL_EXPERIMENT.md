# First metal experiment — Dynamixel XL330

Physical-evidence milestone. Not a kernel redesign. Not certified safety.

## Chosen actuator

| Item | Value |
|------|--------|
| Actuator | Robotis Dynamixel **XL330-M288-T** (also accept XL330-M077-T) |
| Controller | Servo onboard MCU + USB–UART adapter (U2D2 / FTDI / CP2102) |
| Supply | **5.0 V**, current limited to **≤ 0.5 A** at the bench PSU |
| Interface | Dynamixel Protocol 2.0, default **57 600** 8N1 on `/dev/ttyUSB*` |
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

Operator test: open the switch; the servo must lose holding torque while USB/data may stay enumerated. Record `REALITYOS_METAL_CUTOFF_TESTED=1` only after that observation.

## Identity mapping

`probe_identity()` reports:

| Field | Source |
|-------|--------|
| `serial` | **Measured:** USB adapter serial (sysfs) + servo bus ID. XL330 EEPROM has **no** factory serial. |
| `firmware_id` | **Measured:** model number register 0 + firmware version register 6 (`xl330-m288:1190:<fw>`). |
| `actuator_ids` | **Measured:** `xl330:<id>`. |
| `calibration_id` | **Deployment:** `metal.json`, not EEPROM. |
| `design_content_hash` | **Deployment:** SHA-256 of `realityos.metal_design/1` (limits). Not EEPROM. |

If USB serial cannot be read, a `usb:<vid>:<pid>:<devpath>` fallback is used if sysfs exposes it. If neither exists, ONLINE start fails closed. Values are not invented.

## Composition

`realityos-metal-smoke serve` calls `RuntimeSession::start_online` → `OsMonotonicClock`. It does not use the HIL `Authority` object or `FakeClock`.

Users: `realityos-authority` owns the tty, key, journal, and process. `realityos-autonomy` may use only the Unix socket.

## Sensor freshness

* Device capture: XL330 Realtime Tick (wrapping 1 ms counter) as `timestamp_s`.
* Freshness: authority monotonic receive time stamped by `ingest_sensor_packet`.
* Threshold: `GovernorConfig.sensor_stale_s` (default 2 s) / `freshness_threshold_s` in `metal.json`.
* Autonomy cannot ingest evidence.

## Proof

`docs/metal_proof.json` schema `realityos.metal_proof/1` is written only from measured cases on a real device. The reporter refuses to emit a success artifact when `hardware_present` is false. It does not overwrite `docs/hil_proof.json`.

## How to run (bench host)

```text
# as a user who can sudo, with the XL330 powered and the cutoff closed
cargo build -p realityos-metal --bins
sudo -E env REALITYOS_METAL_DEVICE=/dev/ttyUSB0 \
  REALITYOS_METAL_BIN=$PWD/target/debug \
  REALITYOS_METAL_CUTOFF_TESTED=1 \
  scripts/metal-campaign.sh
```

This Cloud Agent VM has **no** USB/serial actuator and **no** self-hosted worker. Until a bench host with the XL330 and cutoff is attached, the experiment is blocked. That is not a software-architecture remaining task.
