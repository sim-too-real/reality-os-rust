# Competitor wedge

Last-gate vs everything that is *not* a last-gate. Sources checked 2026.

## Landscape

| Stack | What it actually is | Gap vs this repo |
|-------|---------------------|------------------|
| **ROS 2 + rmw_zenoh / CycloneDDS** | Comms + lifecycle. Zenoh is a first-class RMW (Jazzy/Kilted). | No identity-bound `CertifiedCommand`. No five-word probe. Safety is integrator-owned. |
| **Nav2 / MoveIt / cuMotion / cuRobo** | Planners. cuMotion is a MoveIt plugin; cuRoboV2 is GPU motion. | Planning ≠ last write. Collision-free path is not a driver certificate. |
| **NVIDIA Isaac ROS / NITROS** | GPU type adaptation + perception graphs. | Fast pixels, not a motor gate. |
| **NVIDIA Halos (2025–2026)** | Full-stack FS: IGX Thor FSI (IEC 61508 SIL 3 *capability*), Halos Core (DriveOS next-gen, ISO 26262 ASIL D *on NVIDIA hardware*), Outside-In Safety Blueprint (MUTE/UNMUTE from infrastructure cameras). | Hardware + OS + mute. Not a typed software last-gate that VLAs must cross. Mute/unmute is not `probe`. Not robot-agnostic without IGX. |
| **Apex.OS 25-05 + QNX SDP 8.0** | ROS 2-shaped middleware, ISO 26262 ASIL-D SEooC, zero-copy, Grace/Ida. | Certified *transport and execution*. Not identity+PFL+learned-actuator refuse. Sits *on* a certified OS; we sit *under* any RMW. |
| **Autoware** | Open AV stack on ROS 2. | Vehicle autonomy, not cobot last-gate. |
| **micro-ROS** | MCU client. | No Governor. |
| **QNX Neutrino** | Certified RTOS. | We are not an OS. We should *run on* QNX later, not replace it. |
| **Foxglove** | Observe/log. | Complementary. Never authority. |
| **Ames CBF / HJ filters** | Academic safety filters (QP / value function). | Closest cousin to Bounded Trust dispose. We wire filter **output** to `CertifiedCommand` + Governor ack; they often stop at the QP. |
| **VLA gating papers (2026)** | Calibrated uncertainty gates on OpenVLA-class policies. | Improves robustness; still a learned gate unless identity/envelope/journal sit below it. |
| **ISO 10218-2:2025** | Collaborative *application* (TS 15066 absorbed). SRMS / HGC / SSM / PFL. | Standard for cells. Our PFL module is a **SIM screen** with configured body-region limits, not a certified cobot. |

## Wedge (occupy this, do not duplicate that)

1. **Last-gate crate, not another ROS.** Depend on ROS 2 / Zenoh as an adapter. Never reimplement DDS.
2. **Five-word verdicts.** Competitors collapse to go/no-go. `probe` (“go measure”) is the product difference vs VLAs.
3. **Governor may only narrow.** Unique DIP. Halos SDM mutes; Apex schedules; neither has one-way command narrowing that cannot upgrade a refuse.
4. **Honesty in the type system.** `HonestyStamp` cannot be metal. SIM/SIL/production stay distinct — Halos SIL and Apex SEooC blur this in marketing.
5. **Learned-actuator refuse.** GR00T / Gemini Robotics / OpenVLA / Isaac policies propose. This stack is the fence they cannot skip.
6. **Dual-rate Screen/Dispose** as a scheduler that still ends in `CertifiedCommand` (not a second plant write).
7. **Rust hot path** — no GC on `pre_actuation_check` + envelope + ledger + write. Python keeps start/identity issuance and Foundry.
8. **Robot-agnostic `Plant` + `HardwareDriverPort`.** Halos wants IGX Thor; Apex wants QNX. We want the same gate on a SIM stub today and a vendor port later.

## Do not build

- Ansys/Isaac Sim clone
- NITROS GPU perception
- Apex-class certified DDS
- Foundry invent inside this repo
- ONNX policy as `plant.act`
- Claims of ISO/SIL/MEASURED
