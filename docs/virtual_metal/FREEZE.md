# Virtual Metal freeze

Status: **FREEZE** as of `6c5ac6673b9d8878ccbd186a7abd82e4bd5558be`

Linux `authority` https://github.com/sim-too-real/reality-os-rust/actions/runs/35428615835
succeeded: fmt, clippy `-D warnings`, `cargo test -p realityos-metal --all-targets -- --test-threads=1`,
`cargo test -p realityos-virtual-metal` (A1–A7 on `RuntimeGovernor` + production `Xl330Driver` + PTY),
Tier-2 campaign `--n 64 --tier 2`, honesty (`hardware_present=false`,
`SIM_VIRTUAL_METAL_NOT_METAL`, hold rows record real action counts), workspace tests, MuJoCo.

Do not add another Dynamixel, EtherCAT, CANopen, BLDC, thermal FEA, or digital-twin
stack unless:

- real metal contradicts this lab, or
- Tier 1 finds a new authority counterexample, or
- Tier 2 exposes a production-path mismatch, or
- a new hardware target requires a concrete abstraction change.

`VIRTUAL_METAL_PASS` is not MEASURED. `hardware_present` stays false.
`SIM_VIRTUAL_METAL_NOT_METAL` stays the evidence token.

Known production-path observation (not a freeze blocker): a WrongStatusId
status with a valid CRC is accepted by the current decoder (`success` class).
