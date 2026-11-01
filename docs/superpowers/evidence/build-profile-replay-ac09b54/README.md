# Build-profile replay comparison — 2026-11-01

This frozen one-trial replay compares the same Reality OS source commit and preserved IIWA-14 input under debug and optimized Rust builds. Both runs completed successfully with the same trial key, trial digest, outcome, simulated duration, wrench sample count, and control-write count. The optimized build reduced measured wall time from 129.37 s to 77.79 s (39.87%) on this one case.

This is a single simulation replay, not a campaign result or a general runtime claim. The debug run still took 129.37 s for 58.68 s simulated. The profile comparison does not establish that optimization alone resolves the latency tail.

## Reproduction

From clean Reality OS `main` at commit `ac09b54dc368f0474fe61218b291a2be04ebfb00`, run the following twice with the same input and separate output directories:

```powershell
cargo run -p realityos-verify --bin realityos-verify -- manipulation-benchmark --replay docs/superpowers/evidence/suite-v3-paired-after-label-fix/trials/v7_development_menagerie_iiwa14_push_probe_replan_21000_0.json --out verify-out/build-profile-debug
cargo run --release -p realityos-verify --bin realityos-verify -- manipulation-benchmark --replay docs/superpowers/evidence/suite-v3-paired-after-label-fix/trials/v7_development_menagerie_iiwa14_push_probe_replan_21000_0.json --out verify-out/build-profile-optimized
```

The report field `execution_build_profile` distinguishes `debug_assertions_enabled` from `debug_assertions_disabled`. Run reports and raw trial records are preserved in the `debug/` and `optimized/` directories. The frozen input SHA-256 and paired checks are in `RESULT.json`.

All episodes are `SIMULATION_ONLY`. Admission remains `NOT_ADMITTED`; `metal`, `shop_release`, and `viewport_admits` remain false. No physical measurement, hardware execution, or customer use is claimed.
