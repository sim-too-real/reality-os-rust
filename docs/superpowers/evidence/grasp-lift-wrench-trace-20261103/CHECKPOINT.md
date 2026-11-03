# Grasp Contact Wrench Trace Checkpoint

## State

Telemetry code and evidence are on the canonical `main` checkout. The pre-run freeze is committed at `65e202804e27cacba402503573b64ae255f64072`; telemetry source and final evidence are awaiting the final commit and push. No worktree or repository copy was used.

The MuJoCo worker emits body inertial centers and all six contact-wrench components. Verifier truth retains those plus contact position. Grasp audits retain raw per-contact data, derived force and moment on the object, `ctrl`, and raw actuator force at post-close and lift samples. Existing grasp control, thresholds, and safety logic were not changed.

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

## Limits and Next Diagnostic Question

The audit stores sample phase but not per-sample simulation time. The result contains repeated `post_close` labels, so samples in that phase cannot be ordered precisely from the audit alone. Keep this captured result immutable; add a sample timestamp under a new freeze before any later replay.

Next, determine whether the 21006:21 hold residual is driven primarily by contact-point migration and moment about the object COM, or by finger actuator realization under the existing cap. Align contact and actuator traces by simulation time before proposing any control change. Separately resolve the rotated-sweep contract with conservative geometry and collision tests; do not weaken the existing refusal based on this telemetry.

No physical test, actuation, supplier/customer contact, purchase, or fabrication occurred. The trace is simulation-only. Commit and push remain pending final diff review.
