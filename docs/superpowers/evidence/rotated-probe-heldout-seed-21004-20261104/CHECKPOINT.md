# Held-Out Probe Trial 21004 (2026-11-04)

**Classification: HELD-OUT SIMULATOR CASE FORCE-MONITOR ABORT; EMPTY SIDECAR; CONTACT-FORCE TRACE NOT PRESERVED; NOT_ADMITTED.**

The frozen trial `v7:development:menagerie_iiwa14:push_probe_replan:21004:0` ran once under freeze `rotated_probe_heldout_seed_21004_iiwa14_20261104_v1` (source commit `f1db94c`, freeze commit `26eddfa`). The diagnostic test exited 101 after 96.27 s.

## Observed

- Episode outcome: `action_failed` / `force_monitor_abort` / `WRENCH_MONITOR_ABORTED_ACTION`
- Abort reason: `unexpected_contact` at simulation time ≈ 12.834 s (verified stop)
- Object planar displacement ≈ 1.5e-6 m (no useful slip onset)
- Wrench samples collected during the aborted action: 6416
- Unauthorized writes: 0
- `PROBE_MEASUREMENT_SIDECAR.json` was **not** written
- Only `PROBE_MEASUREMENT_SIDECAR.json.episode-diagnostic.json` and `EPISODE.json` were preserved
- Independent contact-force analysis is impossible for this run: no sealed measurement records

## Cause

`WitnessExecution::WrenchAborted` returned before `push_probe_measurement`, so the probe audit (including per-step `object_pose_trace` / `contact_force_samples` collected by the worker) was discarded. The empty-sidecar verifier then correctly refused an empty measurement list.

Freeze text expected contact-force sidecar preservation whether inference passed or failed; `expected_episode_outcome: success` / `updated:SlipOnset` was optimistic and was not met.

## Gates

Unchanged: 1 mm planar motion, 0.5 N tangential-force lower bound. No threshold edits. No physical test, purchase, fabrication, or authority broadening.

## One-run budget

Consumed. Do not replay seed 21004 under the same freeze. Seed 21005 is the next held-out contact-force capture after the abort-path seal fix.

## Next

1. A post-run patch now seals probe measurements on wrench-monitor abort when the audit contains motion and samples. It carries `UnexpectedContact` as `runtime_contact_ambiguous`, so friction inference remains fail-closed. `cargo check -p realityos-verify --tests` passed; no simulator replay was used to validate this patch.
2. A later same-seed runner invocation was stopped before creating the sidecar reservation. No additional trial simulator steps were confirmed.
3. Freeze and run seed 21005 once for contact-force capture.
4. Offline-verify that sidecar and compare obj0–table support force with tool-wrench onset bounds.

The result remains bound to source commit `f1db94c`. The post-run source patch makes the current source tree differ from the frozen run source; `RESULT.json` and `OFFLINE_VERIFY.json` record both hashes and the distinction.
