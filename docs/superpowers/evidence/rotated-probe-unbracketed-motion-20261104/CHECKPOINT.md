# Rotated Probe Motion-Onset Diagnostic (2026-11-04)

**Classification: FOUNDATIONAL RUNTIME DIAGNOSTIC; SIMULATION-ONLY; NOT_ADMITTED.**

The input was frozen before one simulator replay. `FREEZE.json` binds source commit `92cab21a531520f3bb50d33c0a8336e9ad095210`, trial `v7:development:menagerie_iiwa14:push_probe_replan:21000:0`, its digest, model hash, resolved trial, zero-step preflight, scenario suite, sensor profile, and relevant source/model hashes. Freeze commit `f63892a` was pushed before the replay.

The episode still ended `UnexpectedRefusal` / `NO_JUSTIFIED_CURRENT_ACTION` after two authorized `probe:+x` actions. It recorded `no_inference:UnbracketedMotion` and zero unauthorized writes. The offline sidecar verifier passed and recomputed `Updated(SlipOnset)` for the first probe and `NoInference(UnbracketedMotion)` for the second.

The second probe’s trace places its first planar 1 mm crossing at `43.292 s`, where the recomputed tangential-force lower bound is `14.3063 N`. The executor’s `PROBE_CONTROL_UPDATE_STEPS` is 100 steps, or 0.2 seconds at this model’s timestep. Its first batch endpoint beyond 1 mm is `43.354 s`; the conservative force lower bound at that endpoint is `0 N`. This matches the unresolved-motion refusal and identifies the batch observation cadence as the cause in this frozen case. The first probe’s trace also contains a brief 1 mm crossing between batch endpoints; a later endpoint sees the motion after the force bound is supported.

The 1 mm displacement threshold and 0.5 N force lower-bound gate remain unchanged. No full-lift replay, physical test, actuation, purchase, fabrication, or external contact occurred. The raw evidence and recomputation are in `PROBE_MEASUREMENT_SIDECAR.json`, `OFFLINE_VERIFY.json`, `analyze_motion_onset.py`, and `OFFLINE_ANALYSIS.json`; `RESULT.json` binds their hashes.

**Next:** process each time-aligned object pose in the monitored wrench window and pair it only with wrench samples captured at or before that pose time. Keep the 1 mm / 0.5 N refusal gate. Freeze the updated source and the same resolved trial before one after-fix replay. Do not run a full-lift replay until the offline predictor has independent held-out evaluation.
