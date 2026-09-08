# Philosophies (from live TheWorld code)

## Two products

| Product | Door | Role |
|---------|------|------|
| theworld-foundry | `shop` / invent | Physics-native invent + CAD |
| theworld-runtime | Reality OS + Governor | Control last-gate |

This repo is runtime only. Foundry packages may emit **hints**. They never spend, never ONLINE, never MEASURED.

## Reality OS

From `sdk/reality_layer.py`: VLAs predict what robots tend to do. World models predict what scenes may look like. Reality OS determines **what will happen**, **what to do**, and **whether it is safe**.

Locked loop: see → identify → certify → execute → writeback.

- No end-to-end imitation as the control core
- No video prediction as truth of dynamics
- No action without a physical certificate
- Probe when uncertain (`probe` is not `abort`)
- Intent is semantic, not torques
- External proposals (teleop / VLA) are **candidates** and still certify

## Governor

From `control_plane/runtime/runtime_governor.py`: fail-closed **actuation boundary**. Distinct from `PhysicsGovernor` (slide-task belief filter).

- Identity of this exact built machine
- E-stop never auto-clears; recovery needs operator ack + heartbeat
- Envelope is defense in depth vs ALLOW-alone
- Ordinary kernel REFUSE does **not** latch e-stop (replan must remain possible)
- Integrity failures (replay, identity, journal unreadable) **do** latch
- Governor may narrow; it cannot invent `command_id` or upgrade a refuse

## Authority matrix

| Role | Allowed | Never |
|------|---------|-------|
| VLA / VLM / RL / ER | candidates, parse, tools | `direct_motor_command` |
| Reality OS | plan, probe, certificate | semantic hallucination as truth |
| Governor | identity / safety / driver check | task planner, invent |

Gemini Robotics / GR00T / OpenVLA map to proposers. Motors only after decide → CertifiedCommand → Governor ack.

## Honesty

Typed contracts cannot stamp metal, MEASURED, or invent_authority (`HonestyStamp`). Evidence tokens must be `SIM_*` or contain `NOT_METAL`.

ISO 10218-2:2025 collaborative functions (SRMS, HGC, SSM, PFL) appear here as **software analogs**, not SRP/CS PL/SIL certificates.
