# Time-Aligned Probe Motion Follow-Up (2026-11-04)

**Classification: FOUNDATIONAL RUNTIME FIX; ONE SIMULATOR CASE PASSED; SIMULATION-ONLY; NOT_ADMITTED.**

The baseline sidecar in `rotated-probe-unbracketed-motion-20261104` showed the second probe crossing 1 mm at `43.292 s` with a recomputed tangential-force lower bound of `14.3063 N`. The executor observed only 100-step batch endpoints; at `43.354 s`, the bound was `0 N`, which produced `UnbracketedMotion`.

`execution.rs` now checks each time-aligned object-pose sample against the latest wrench sample captured at or before that pose time. The 1 mm displacement threshold and 0.5 N force lower-bound threshold remain unchanged. The existing batch speed calculation is unchanged. Focused tests passed for per-sample onset, refusal when force is below threshold, and rejection of future wrench samples.

The updated source was frozen before one replay. Freeze commit `a1b20f1` binds the current `main` source, model, trial `v7:development:menagerie_iiwa14:push_probe_replan:21000:0`, and its zero-write, zero-step preflight. The diagnostic integration test passed in 208.25 seconds. The episode was `success`, with `Updated(SlipOnset)` interval `[0.6864261124, 0.7735596184]`, one authorized `probe:+x`, one authorized `push:+x`, and zero unauthorized writes. The sidecar verifier passed and recomputed the motion bracket `[23.904, 23.906] s`.

This is one deterministic simulation case. It establishes no physical friction measurement, independent held-out predictor performance, hardware capacity, broad reliability, or admission. No full-lift replay, physical test, physical actuation, purchase, fabrication, or external contact occurred. The separate offline predictor gate remains closed; do not run a full-lift replay until it has independent held-out evaluation.
