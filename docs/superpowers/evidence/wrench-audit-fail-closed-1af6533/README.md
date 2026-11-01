# Fail-closed wrench compensation audit replay

Classification: **FOUNDATIONAL_IMPROVEMENT**. This is a one-trial simulation replay confirming a reporting and safety-gate correction. It demonstrates no new mechanism capability.

## Frozen replay

- Source commit: 1af6533405070fe280b68ef47a94f648ba74b322
- Worktree at replay: clean
- Input: docs/superpowers/evidence/suite-v3-paired-after-label-fix/trials/v7_development_menagerie_iiwa14_push_probe_replan_21000_0.json
- Input SHA-256: 3cfc67e027055dd137ac5df8d4e5cbe2b2876519296b2409c120beae5cfe1775
- Command: cargo run --release -p realityos-verify --bin realityos-verify -- manipulation-benchmark --replay docs/superpowers/evidence/suite-v3-paired-after-label-fix/trials/v7_development_menagerie_iiwa14_push_probe_replan_21000_0.json --out verify-out/wrench-audit-fail-closed-clean-1af6533

## Result

The single trial completed and retained task success (1/1). The report now fails the safety gate because the compensation audit was incomplete: 1 invalid audit trial and 145 invalid samples out of 29,342 wrench samples. The episode records monitor_compensation_audit_invalid; conservative peak bounds and minimum margin are absent. The benchmark CLI exited 1, as required by the false safety gate. The campaign is recorded as complete in the sense that its scheduled trial finished; that does not imply a passing safety gate.

This is SIMULATION_ONLY, one trial, and NOT_ADMITTED. It does not establish physical safety or sim-to-real transfer.

## Verification context

- The two focused regression tests passed before this replay.
- cargo fmt --all -- --check passed.
- cargo test -p realityos-verify completed with 168 passed and 9 failed (334.86 s). All nine failures reported MuJoCo/engine allocation errors or an empty engine error under the parallel package run. The cross-action wrench-audit unit regression passed in that run. These package failures remain unresolved; no skip-as-pass interpretation is used.

See RESULT.json for machine-readable fields and SHA256SUMS.json for archived file hashes.
