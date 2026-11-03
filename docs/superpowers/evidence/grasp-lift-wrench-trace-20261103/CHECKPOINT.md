# Grasp Contact Wrench Trace Checkpoint

## State

Telemetry capture, replay evidence, and the offline support comparison are committed on canonical `main`. No worktree or repository copy was used.

The MuJoCo worker emits body inertial centers and all six contact-wrench components. Verifier truth retains those plus contact position. Grasp audits retain raw per-contact data, derived force and moment on the object, `ctrl`, and raw actuator force at post-close and lift samples. Existing grasp control, thresholds, and safety logic were not changed.

## Tool–Object Friction Mapping Correction

The worker previously mapped scenario geom friction to `[μ, μ, 0.01]`. MuJoCo uses the second geom slot for torsional friction, so the target coefficient was not reaching the two sliding directions; default-friction finger geoms kept the target contact at `[1.0, 1.0]`. The worker now maps geom scalar friction to `[μ, 0.01, 0.01]` and adds explicit friction only to the scenario target–selected finger geom pairs. Other object pairs keep their normal mixing. Runtime `configure_body` updates change the geom triplet and only these explicit target–finger pair coefficients.

Focused worker-load checks verified μ=0.4 gives target–finger `condim=3`, tangential friction `[0.4, 0.4]`, torsional friction `0.01`, while an unrelated target–support pair retains `[1.0, 1.0]`. Additional checks verified a `condim=4` finger, a runtime update to μ=0.6, and fail-closed handling of a missing finger body. Rust unit checks verified that only the target scenario object receives the selected gripper's finger-body mapping and that missing gripper metadata is rejected. The old worker failed the regression with tangential friction `[1.0, 1.0]`; the focused checks pass after the fix.

## Verification

- Parser and wrench unit tests were observed failing before implementation and passing after it. Coverage includes both geom body orderings, a rotated contact frame, contact torque, missing/non-finite inputs, and legacy audit deserialization.
- `cargo fmt --all -- --check`: passed.
- `cargo test -p realityos-verify --lib`: 192 passed, 0 failed.
- `cargo test -p realityos-verify`: library passed; integration target had 105 passed, 1 failed, 6 ignored. The failing `iiwa_expected_success_probe_tracks_retract_under_wrench_monitor` was then rerun alone and failed again.
- Root cause of that separate failure: the probe updates support friction, MuJoCo leaves a slight object yaw, and the reasoner rotates the candidate with the object. `sweep_gap` currently accepts only one world-axis direction. An existing unit test explicitly expects `PROBE_SWEEP_DIRECTION_UNSUPPORTED` for the same rotated-sweep class. The integration success expectation and conservative sweep behavior conflict; no reasoner or guard change was made in this telemetry task.
- Five generated tracked evidence snapshots changed during the full run; each was verified as numeric output and restored to `HEAD`. They remain clean.
- The initial local-only freeze/replay was superseded after review found that it omitted the MuJoCo worker/parser hashes and was not committed before execution. Its preliminary artifacts were removed from the final evidence package; the following committed freeze and replay are authoritative.
- Corrected pre-run freeze: `FREEZE.json`, SHA-256 `30A1CC816BA4BA65358152776C04B1EC71809C8DDBA0650BDA73D9A1B7FBD3B9`, committed before replay in commit `65e202804e27cacba402503573b64ae255f64072`. It binds all five source files: worker, observation parser, audit/execution module, reasoner, and integration harness, along with both input digests.
- Replay: `RESULT.json`, SHA-256 `6A920F0544ABAFBE26D366D0CB5D1F228F03D7C70FABB45872A0AEBFE55B00C9`. Exactly the two frozen primary trials ran. Trial `21010:25` retained `unexpected_refusal` / `runtime_divergence`, with 10 audit rows over `post_close` and four lift waypoints. Trial `21006:21` retained `action_failed` / `hold_not_verified`, `hold_verified=false`, 7.362 mm object-follow residual, and zero unauthorized writes; it has 14 audit rows over `post_close`, four lift waypoints, and `post_lift`.
- Independent verification: `verify_trace.ps1` confirmed that the committed freeze predates the replay result, checked freeze/result/all-five-source/baseline/input digests and sample phases, rejects null and non-finite raw vector entries, independently recomputed signed force, torque, moment arms and all 54 available derived contact wrenches, and confirmed unchanged failure outcomes plus zero unauthorized writes. Report: `VERIFICATION.json`.
- Trial `21006:21` reached the existing 1.95 N magnitude cap on both finger actuator-force entries at post-close, then failed the unchanged 5 mm hold-follow limit at 7.362 mm. This supports investigating contact distribution and finger actuation under the current cap; it does not justify raising the cap and is not a hardware-capacity measurement.
- Offline comparison: `compare_upward_support.ps1` binds the same frozen input/result hashes and the checked-in model gravity, then compares the prior scalar proxy against the signed world-z contact resultant at the first lift waypoint. Its source hash is `5E101BE067E8132BC1F230CBB155290BF5FDE38DF9150394FBED33ECDC475F81`; the report hash is `938A76A2507EF40565E01FCA654249302F5668ADB9181425F8561DA4C6B3481A`. The model XML hash is `96638449987EB8CE6D900063F2FCF279D792C2C60F72F44282CF150C61D57904`. Trial `21006:21` has a 2.628 N scalar proxy for 2.192 N weight, but only 1.295 N first-lift finger vertical force (0.591× weight) and a 0.897 N deficit; `grip_l` is at -1.950 N and `grip_r` at -1.897 N. Trial `21010:25` is rejected by the scalar proxy (0.595 N for 1.419 N weight) and shows 0.562 N first-lift finger vertical force. The machine-readable output is `UPWARD_SUPPORT_COMPARISON.json`; the concise result and decision are in `UPWARD_SUPPORT_COMPARISON.md`.

## Limits and Next Diagnostic Question

The audit stores sample phase but not per-sample simulation time. The result contains repeated `post_close` labels, so samples in that phase cannot be ordered precisely from the audit alone. Keep this captured result immutable; add a sample timestamp under a new freeze before any later replay.

The scalar screen's 21006 false pass is now decision-relevant: preserve the current cap and hold/refusal gates, and do not use scalar friction times normal load alone to authorize lift. Next, build an offline predictor using post-close measurements only, including contact direction/moment arms and actuator realization; compare its predictions against first-lift outcomes on independent frozen cases. Align samples by simulation time in any later capture; the current audit has repeated `post_close` phase labels without timestamps. Do not run another full-lift replay until the predictor is independently evaluated. Separately resolve the rotated-sweep contract with conservative geometry and collision tests; do not weaken the existing refusal based on this telemetry.

The friction mapping correction validates simulator contact parameter wiring only. It does not evaluate the predictor, change the two frozen outcomes, justify another full-lift replay, or support a physical friction/capacity claim.

No physical test, actuation, supplier/customer contact, purchase, or fabrication occurred. The traces and force resultants are simulation-only. The two development controls do not establish predictor accuracy, held-out performance, or hardware capacity.
