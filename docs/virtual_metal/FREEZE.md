# Virtual Metal freeze

Status: **FROZEN** as of `4e591055615943be4af8325a08de68ccd5ab7811`

Linux `authority` https://github.com/sim-too-real/reality-os-rust/actions/runs/35430916712
succeeded: fmt, clippy `-D warnings`, `cargo test -p realityos-metal --all-targets -- --test-threads=1`,
`cargo test -p realityos-virtual-metal` (A1–A7; WrongStatusId is not directed success),
Tier-2 `--n 64 --tier 2`, honesty (`hardware_present=false`, `SIM_VIRTUAL_METAL_NOT_METAL`,
hold rows record real action counts), workspace tests, MuJoCo.

Closed counterexample: a CRC-valid Status Packet with the wrong servo ID no longer
satisfies a directed transaction. Production path is `decode_status_scan_for(buf, expected_id)`
on `Xl330Driver` recv/ACK/READ. Broadcast sniff remains `unique_status_ids`. Seven
protocol regressions: correct-only, wrong-only, wrong-then-correct, correct-then-wrong,
wrong WRITE ACK, wrong READ poison, multi-ID sniff.

Do not add another Dynamixel, EtherCAT, CANopen, BLDC, thermal FEA, or digital-twin
stack unless:

- real metal contradicts this lab, or
- a newly minimized counterexample breaks a claimed invariant, or
- a new hardware target requires a concrete abstraction change.

`VIRTUAL_METAL_PASS` is not MEASURED. `hardware_present` stays false.
`SIM_VIRTUAL_METAL_NOT_METAL` stays the evidence token.
