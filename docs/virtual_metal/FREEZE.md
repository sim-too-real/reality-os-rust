# Virtual Metal freeze

Status: **PENDING_LINUX_A1_A7**

Do not add another Dynamixel, EtherCAT, CANopen, BLDC, thermal FEA, or digital-twin
stack unless:

- real metal contradicts this lab, or
- Tier 1 finds a new authority counterexample, or
- Tier 2 exposes a production-path mismatch, or
- a new hardware target requires a concrete abstraction change.

`VIRTUAL_METAL_PASS` is not MEASURED. `hardware_present` stays false.
`SIM_VIRTUAL_METAL_NOT_METAL` stays the evidence token.

Flip this file to **FREEZE** only after Linux `authority` runs A1–A7 on
`RuntimeGovernor` + production `Xl330Driver` + PTY and those jobs succeed.
