# Virtual Metal V1 — what public data does not establish

Virtual Metal is a software surrogate. `VIRTUAL_METAL_PASS` is not physical hardware verification.

Machine-readable inventory: `docs/virtual_metal/unknowns.json`.

Properties that still require a physical XL330 measurement (smallest experiment in the JSON):

- Undocumented firmware scheduling and exact RAM restore after REBOOT
- Boot-timing distribution after VIN apply / INST_REBOOT
- USB adapter quirks (DTR-RESET, echo, latency)
- Real voltage sag under stall
- EMI / on-wire CRC error rate
- Mechanical backlash and gearbox efficiency
- Unit-to-unit stall / no-load variation
- Holding torque at the conservative PWM cap
- Thermal response and shutdown timing
- Power-cut behavior (USB unplug vs VIN cutoff)
- Kernel/driver timing versus live I/O deadlines
- Physical hot-swap identity
- Hardware Error Status bit for under-voltage (e-Manual Shutdown table 0x01 vs voltage-limit prose 0x10). Campaign samples both; field reports latch 0x01.
- Velocity P Gain factory value is settled at 180 (`CONFIRMED_STALE_CODE` was 100). Remaining physical question is a factory READ of register 78 on a specific unit.
