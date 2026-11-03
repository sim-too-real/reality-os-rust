# Held-Out Probe Follow-Up (2026-11-04)

**Classification: DIAGNOSTIC CAPTURE FAILURE; SIMULATION-ONLY; NOT_ADMITTED.**

The single replay for `v7:development:menagerie_iiwa14:push_probe_replan:21001:0` (seed `21001`) exited after 102.75 seconds with `InvalidTrial("probe measurement sidecar has no measurement records")`. The runtime finalized an episode internally, then rejected the empty measurement sidecar. The caller did not receive the episode, so no `EPISODE.json` or `PROBE_MEASUREMENT_SIDECAR.json` was written. Task outcome, action sequence, and inference are unknown. This is not a pass or a task-failure result. The one-run budget is spent; seed 21001 must not be replayed. Details are in `RUN_FAILURE.json`.

The frozen trial had a valid zero-step first-action preflight for `probe:+x`; that does not guarantee the full replay executes a probe or produces a measurement record. The baseline fix result for seed 21000 remains one successful simulation case with unchanged 1 mm / 0.5 N gates.

**Next:** make the diagnostic path persist the finalized `BenchmarkEpisode` when the sidecar has no measurements, without fabricating a measurement. Then freeze a different held-out seed, verify all source and trial hashes, and run it once. Keep full-lift replay gated on independent held-out predictor evaluation.
