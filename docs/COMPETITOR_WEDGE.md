# Competitor wedge

Independent `/deep-research` (2026, cited) plus live TheWorld code. Status of that report: **Partial** (some certificates not fetched as PDFs). This crate occupies the hole those sources actually leave.

## Landscape

| Stack | What it actually is | Gap vs this repo |
|-------|---------------------|------------------|
| **ROS 2 + rmw_zenoh / CycloneDDS** | Transport. rclcpp/rmw_zenoh Apache-2.0; Cyclone DDS EPL-2.0/EDL-1.0, ROS 2 tier-1 RMW (REP-2004 QL2 — process maturity, **not** a functional-safety certificate). | No identity-bound `CertifiedCommand`. No five-word `probe`. |
| **Nav2 / MoveIt 2** | Apache-2.0 (mixed SPDX) mobile planner; BSD-3 MoveIt 2 manipulation planner. Nav2 Collision Monitor filters `cmd_vel` from raw sensors **after** the controller server. | Planner/filter, not identity rail or certified last write. Wiring-dependent. |
| **Isaac ROS / NITROS / Perceptor** | CUDA ROS 2 GEMs + GPU type adaptation. NVIDIA Isaac ROS Software License: **not tested or certified for Critical Applications** including navigation and AVs. Perceptor hands costmaps to Nav2. | Fast pixels. Explicitly not a motor gate. |
| **cuRobo V2 / cuMotion** | cuRobo V2 Apache-2.0 research motion lib. cuMotion is the productized MoveIt 2 planner under the Isaac license + Critical Application disclaimer. | Planner, not OS, not last-driver-gate. |
| **DriveOS / DriveWorks / Halos** | DriveOS: proprietary Linux/QNX on DRIVE AGX; NVIDIA states TÜV SÜD ISO 26262 ASIL D (DriveOS 6.0). DriveWorks: sensor/egomotion middleware, **not** a last-write OS. Halos Core: next-gen DriveOS + hypervisor + safety CUDA/TensorRT. Halos Applications: AEB/LDW-style **rule** guardrails, not Isaac GEMs. Outside-In Safety: Apache-2.0 early-access facility cameras; NVIDIA `SAFETY_NOTICE` disclaims FS/cyber compliance and production suitability. TÜV Rheinland is inspecting IGX Thor / Halos OS / Holoscan for **certification readiness** (IEC 61508 / ISO 13849 / ISO 26262) — not stated as completed robotics-stack certificates. | Hardware + SEooC OS + mute. Mute ≠ `probe`. Not robot-agnostic without IGX. |
| **Apex.OS + QNX** | Apex.OS: ROS 2-shaped **application runtime** (Grace+Ida) above a host OS; sold as ISO 26262 ASIL D **SEooC** (TÜV Nord). Customer still certifies the in-context item. QNX OS for Safety 8.0: microkernel RTOS pre-certified SEooC ISO 26262 ASIL D, IEC 61508 SIL 3, IEC 62304 Class C, ISO/SAE 21434. | Certified **substrate**. Not identity + PFL + learned-actuator refuse. We should *sit on* them later, not replace them. |
| **Autoware Safety Island** | Zephyr/FreeRTOS follower (MPC/PID), no ROS 2 on the island. Open code **republishes** `/control/trajectory_follower/control_cmd` into Linux; `vehicle_cmd_gate` still selects the vehicle command. 0.5 s stale timeout is **disabled**; missing inputs **skip** the tick; CAN is a placeholder. MRM stays on Linux `mrm_handler`. | Placement analog only. We refuse skip-on-missing-input (`rate::missing_input_forces_passive`). We do **not** claim an RTOS island. |
| **ros2_control** | Exclusive `command_interface` claim; read–update–`write()` loop; URDF `rw_rate` up to 1 kHz, manager default 100 Hz, faster components **capped** at manager rate. | Exclusive I/O contract, not a certified last-gate. Our `HardwareDriverPort` is the analog; fieldbus still a hole. |
| **Halos Outside-In SIL** | SDM in Linux Docker; “Simple UDP + OPC UA (non-safe)”; SIL **does not stop the vehicle** — integrator acts. Production FSI mute/stop is early-access, not a completed robotics last-write certificate. | Mute/unmute ≠ `CertifiedCommand`. |
| **PX4 / Auterion** | BSD-3 flight stack; enterprise distro. In-stack failsafes (RC/GCS, kill, lockdown). Inspected pages do **not** present ISO 26262 / IEC 61508 certificates. | In-stack failsafe, not last-write identity gate. |
| **micro-ROS** | ROS 2 on MCUs via XRCE-DDS Agent. **Not developed to any safety standard**; Agent not production-ready for ISO 26262. | No Governor. |
| **Open-RMF / Foxglove / Foxlet** | Fleet orchestration (Apache-2.0). Observability (MIT SDK, SOC 2 Type II). Foxglove: visualizes and can publish commands the robot already accepts; **does not replace an E-stop, safety PLC, or certified vehicle controller**. | Complementary. Never authority. |
| **ISO 10218-2:2025 / TS 15066** | Collaborative *application* (TS 15066 folded in). SRMS / HGC / SSM / PFL. PFL must keep quasi-static and transient contact below biomechanical limits using **active** force/torque/velocity/momentum/power/energy limits — not a post-contact trip. ISO 10218-1: single point of control (pendant/teach blocks other motion sources); e-stop independent of protective stop. | Our PFL module is a **SIM screen**. Not SRP/CS PL/SIL. |
| **UL 4600 / IEC 61508-3** | UL 4600: simulation is not a substitute for real-world evidence; relying on sim as the primary validation of a fix is a named risk. IEC 61508-3: AI fault correction **Not Recommended** for SIL 2–4 (1998 table inspected; 2010 table not re-opened here). UL 4600 allows a **fixed-functionality safety checker** that keeps updated ML from acting unsafely. | Matches `HonestyStamp` + `assert_no_learned_actuator_authority`. We implement the checker. We do **not** claim SIL. |

## What a last gate must implement (standards mapping — analogs in this repo)

Vendors do not use the product term “last-driver-gate.” Mapping is architectural:

| Requirement | Standard hook | In this repo (SIM analog, not a certificate) |
|-------------|---------------|-----------------------------------------------|
| Single point of control | ISO 10218-1 | `RuntimeSession.bind_and_dispatch` only; bare `act` refused when online |
| Safety channel independent of planner (FFI) | ISO 26262-1 1.49; ISO 10218 e-stop ≠ protective stop | Governor crate does not import Reality OS types (`ActuationCommand` DIP) |
| Active PFL | ISO/TS 15066 5.5.5 → ISO 10218-2:2025 | `DriverEnvelopePack` + `pfl_contact` screen; **not MEASURED** |
| Fail-closed stop | IEC 60204-1 cat 0/1; ISO 13849 Cat 3 via ISO 10218 | e-stop latch, never auto-clear; journal carry |
| No skip on missing input | Autoware SI gap (skip tick); ros2_control still writes last claim | `missing_input_forces_passive` → PassiveFallback this tick |
| SIM ≠ metal | UL 4600 13.3.2 | `HonestyStamp` cannot construct metal/MEASURED |
| No opaque learned actuator | IEC 61508-3 Table A.2 NR; UL 4600 fixed checker | `screen_external_proposal`; forbidden motor tools |

## Wedge (occupy this, do not duplicate that)

1. **Last-gate crate, not another ROS.** Depend on ROS 2 / Zenoh as an adapter. Never reimplement DDS or Nav2/MoveIt.
2. **Five-word verdicts.** Competitors collapse to go/no-go. `probe` (“go measure”) is the product difference vs VLAs.
3. **Governor may only narrow.** Unique DIP. Halos SDM mutes; Apex schedules; Nav2 Collision Monitor clips `cmd_vel`. None has one-way command narrowing that cannot upgrade a refuse.
4. **Honesty in the type system.** SEooC OS certificates (QNX, Apex, DriveOS) are **substrates**. They do not stamp SIM≠metal on the *application* write.
5. **Learned-actuator refuse.** GR00T / Gemini / OpenVLA / Isaac policies propose. UL 4600’s fixed checker is this fence.
6. **Sit on QNX/Apex/Halos later; do not out-certify them.** Placement analog: Autoware Safety Island — independent channel, fail-closed transfer — implemented as software last-write today.
7. **Rust hot path** — no GC on `pre_actuation_check` + envelope + ledger + write.
8. **Robot-agnostic `Plant` + `HardwareDriverPort`.** Same gate on a SIM stub now and a vendor port later.

## Do not build

- Ansys / Isaac Sim clone
- NITROS GPU perception
- Apex-class certified DDS (eProsima Safe DDS is certified comms, still not an actuator gate)
- Foundry invent inside this repo
- ONNX policy as `plant.act`
- Claims of ISO/SIL/MEASURED / completed Halos robotics certificates

Cited research report: session workflow `deep-research` (Partial). Isaac license mixed across repos; Halos Core public certificate for IGX Thor not inspected; IEC 61508-3:2010 table not re-opened.
