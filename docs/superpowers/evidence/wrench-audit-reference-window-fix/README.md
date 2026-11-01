# Wrench reference timing audit replay

Classification: **FOUNDATIONAL_IMPROVEMENT**. This one-trial simulation replay verifies a correction to wrench-audit reference association. It demonstrates no new mechanism capability.

## Frozen replay

- Source commit: `3023a0e5ebaebc9445ca4c147ab9dee657cf688f`
- Worktree at replay: clean
- Input: `docs/superpowers/evidence/suite-v3-paired-after-label-fix/trials/v7_development_menagerie_iiwa14_push_probe_replan_21000_0.json`
- Input SHA-256: `3cfc67e027055dd137ac5df8d4e5cbe2b2876519296b2409c120beae5cfe1775`
- Command: `cargo run --release -p realityos-verify --bin realityos-verify -- manipulation-benchmark --replay docs/superpowers/evidence/suite-v3-paired-after-label-fix/trials/v7_development_menagerie_iiwa14_push_probe_replan_21000_0.json --out verify-out/wrench-audit-reference-window-fix-clean`

## Result

The replay completed its one scheduled trial with task success (1/1). The safety gate passed. The audit examined 29,342 wrench samples and found 0 invalid compensation samples; conservative peak bounds and a 1.45278 minimum margin were recorded. The prior replay at source commit `1af6533405070fe280b68ef47a94f648ba74b322` had 145 invalid samples, no conservative peak bounds or minimum margin, and correctly failed the safety gate. See `../wrench-audit-fail-closed-1af6533/README.md` for that baseline record.

The fix associates a gravity reference with a sample only when the reference's sequence, capture time, and receive time are no later than that sample's. Earlier samples remain auditable without a reference that had not yet become available. The fail-closed gate and thresholds were not changed.

## Verification and limits

- `cargo fmt --all -- --check` passed.
- `REALITYOS_REQUIRE_MUJOCO=1 MUJOCO_GL=disable cargo test -p realityos-verify --offline -- --test-threads=2` passed: 178 unit tests, 100 manipulation benchmark tests, 1 pair-key test, 3 physical-decision tests, 7 production MuJoCo tests, 1 production-authority test, and 8 typed-grasp tests (298 passed, 0 failed).
- Focused regression test: `window_reference_only_applies_after_sample_captured_and_received_it` passed.

This is one `SIMULATION_ONLY` replay and remains `NOT_ADMITTED`. It does not establish physical safety or sim-to-real transfer. The correction fixes an invalid audit attribution; it does not show improved task performance.

See `RESULT.json` for machine-readable before/after fields and `SHA256SUMS.json` for archived artifact hashes.
