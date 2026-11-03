# Probe Interval Replay Evidence Contract

## Purpose

Preserve enough probe evidence for a later offline verifier to recompute each
support-friction inference and bind the result to the source implementation
that produced it. The format is simulation evidence only; it does not establish
a physical measurement, admission, or deployment authorization.

## Selected format

Use the versioned JSON `realityos.probe_measurement_sidecar` record already
written by the manipulation benchmark. The record is an input-and-result
package: each measurement record contains the complete typed
`PushProbeMeasurement`, its digest, the prior friction interval, the stored
inference, and the posterior interval. Keep this exact measurement payload;
summary-only intervals or rounded CSV samples do not retain enough information
to replay contact, timing, compensation, uncertainty, transform, and trace
gates.

The sidecar is paired with:

- `FREEZE.json`, which records the source revision and hashes of the relevant
  runner and semantics files;
- `OFFLINE_VERIFY.json`, which records the recomputed result and the hashes of
  the result and sidecar artifacts.

These files bind the measurement to its computation and frozen trial. SHA-256
provides integrity checking; it is not a signature or an authenticity claim.

## Version 1 record

The top-level JSON object has `format` equal to
`realityos.probe_measurement_sidecar`, `schema_version` equal to `1`, the exact
`trial_id`, its `trial_digest`, an optional `paired_trial_key`, and a nonempty
`measurements` array. Each array item contains:

- `action_key`;
- `measurement_digest`;
- `prior_support_friction_interval`;
- `inference`, including either the updated interval and mode or the typed
  no-inference reason;
- `posterior_support_friction_interval`;
- the full serialized `PushProbeMeasurement`.

The measurement payload retains all wrench samples and their units, uncertainty,
sensor identity, frame, sequence and timestamps; baseline; compensation;
transforms; mass interval; support normal; motion window and complete pose trace
when available; friction prior; observation and collision-model identities;
timing policy; intended-contact evidence; contact-ambiguity and saturation
flags; and evidence origin. Preserve JSON numeric precision and array ordering.

The Rust serde types are the authoritative field schema:
`ProbeMeasurementSidecarV1`, `ProbeMeasurementRecordV1`, and
`PushProbeMeasurement`. A format or semantic change that cannot be read and
checked under version 1 requires a new schema version and a verifier path that
continues to read archived version 1 records.

## Offline recomputation

Run the verifier from the repository root:

```powershell
cargo run --quiet -p realityos-verify --bin realityos-verify -- verify-probe-measurement-sidecar "<path-to-PROBE_MEASUREMENT_SIDECAR.json>"
```

The command does not start MuJoCo or step a simulator. It validates every
sample, recomputes the measurement digest, reruns `infer_support_friction` from
the saved measurement, and rejects a mismatch with the stored inference. Use
the source revision and semantics-source hash in `FREEZE.json` when selecting
the implementation for a later replay.

For an independent implementation, run the standard-library Python reference
described in
[`2026-11-03-independent-probe-interval-reference.md`](2026-11-03-independent-probe-interval-reference.md):

```powershell
python crates/verify/probe_interval_reference.py "<path-to-PROBE_MEASUREMENT_SIDECAR.json>"
```

It recomputes the archived slip-onset update and no-intended-contact refusal
without importing or invoking the Rust verifier. Other record branches fail
closed until separately implemented and tested.

## Archived examples

The contract was replayed against two already frozen records:

- `23023:17` recomputes `Updated(SlipOnset)` with interval
  `[0.47116960533439084, 0.5484442443281078]` from prior `[0.05, 0.8]`.
  Its freeze names source commit
  `a22d944e0ef51017a535f303c86e51a72beb571b`.
- `23024:17` recomputes `NoInference(NoIntendedContact)` and preserves prior
  `[0.05, 0.8]`. Its freeze names source commit
  `7623fb48c51ef711be770b77c9f7621f27042b6e`.

Both freezes record the `push_probe.rs` SHA-256
`6444cab400e6407ddf4ff7a6cd019fbc3719b43f29c73a24fe92c95d0abbd80c`; it
matches the current source file used for these offline checks.

The two archived verifications are single-trial simulation evidence. They do
not estimate a success rate or establish physical friction.

## Post-hoc supplemental replay

After the original two-case freeze, the independent reference was also run on
the archived `23021:17` complete-contact-trace sidecar. It independently
recomputed `Updated(SlipOnset)` as
`[0.47885574015040633, 0.7211370850171951]`; the Rust verifier returned
`[0.47885574015040633, 0.721137085017195]`, within the existing `1e-12`
absolute tolerance. This retrospective replay is documented in
[`SUPPLEMENT_23021.md`](../evidence/probe-interval-reference-replay-20261103/supplement-23021/SUPPLEMENT_23021.md)
and is not part of the original freeze.

The reference continues to reject the archived `23020:17`
`NoInference(Unidentifiable)` sidecar: all recorded contact-pair fields are
null, so the reference cannot independently establish intended contact or
no-contact. It reports `incomplete contact trace`. The Rust verifier accepts
that historical sidecar under its broader legacy semantics; this difference
is an explicit coverage limit for the independent reference, not a new
inference result.
