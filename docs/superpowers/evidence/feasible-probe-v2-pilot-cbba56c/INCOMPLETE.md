# Incomplete suite-v2 pilot attempt

- Candidate source SHA: `cbba56cfc8adf8dcc5dd8e085998246a05521259`
- Command: `cargo run --release -p realityos-verify -- manipulation-benchmark --pilot --out docs/superpowers/evidence/feasible-probe-v2-pilot-cbba56c`
- Required environment: `REALITYOS_REQUIRE_MUJOCO=1`, `MUJOCO_GL=disable`
- Started: 2026-10-18 16:49:09 Asia/Kolkata
- Stopped: 2026-10-18 around 17:47 Asia/Kolkata
- Result: incomplete; no trial records or campaign report were written

The run was stopped after nearly 58 minutes because it produced no intermediate
trial progress or report artifact while the verifier continued consuming CPU.
The peak observed verifier CPU time was approximately 2,800 seconds. This is a
diagnostic interruption, not a pilot result, and must not be included in a
baseline or success-rate analysis.

The likely cost center is positive-trial feasibility resolution: every bounded
IK proof that does not find a collision-clear path falls back to exhaustive IK,
and the resolver may repeat that proof for up to 64 deterministic scenario
samples. The next candidate should add phase-level timing/work diagnostics and
reduce search cost only through bounded, collision-checked witnesses. No IK,
collision, force, authority, geometry, or refusal gate may be weakened.

The WX250S holdout was not run or inspected.
