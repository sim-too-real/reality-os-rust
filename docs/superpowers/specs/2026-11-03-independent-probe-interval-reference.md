# Independent Probe Interval Reference Replay

## Intent

Add a second implementation that reads the saved v1 probe-measurement JSON and
recomputes the recorded friction inference without importing or invoking the
Rust verifier. This targets the shared-code limit in the existing offline
check: its input replay is useful, but a defect shared with the runtime could
survive.

## Approaches considered

1. Keep the Rust CLI as the only verifier. It is lowest-cost, but it exercises
   the same inference implementation as the run.
2. Add a standard-library Python reference verifier over the existing full JSON
   sidecars. It creates a separate implementation path without simulator,
   network, or third-party package dependencies. **Selected.**
3. Create a second reduced sidecar format for interval arithmetic. This would
   duplicate the existing exact measurements and could omit contact, timing,
   compensation, or trace evidence.

## Scope

The first reference version validates v1 sidecar and record identity, sample
digests, motion-trace completeness and digests, measurement digest, timing and
contact gates, and the supported `SlipOnset` interval calculation. It also
recomputes `NoInference(NoIntendedContact)` when the complete validated trace
contains no intended tool-target contact. Unsupported modes and inference
reasons fail closed with an explicit unsupported error. The tool is offline
evidence analysis only; it cannot authorize actions or change runtime policy.

The checked records are the already frozen `23023:17` and `23024:17` sidecars.
No simulator replay, new trial, physical actuation, purchase, or external
contact is in scope.

Run the independent reference with Python 3.10 or later and no installed
packages:

```powershell
python crates/verify/probe_interval_reference.py "<path-to-PROBE_MEASUREMENT_SIDECAR.json>"
```

It emits JSON with its own format identity. The Rust CLI remains useful for its
broader branch coverage; the Python reference currently supports simulation
records with complete pose/contact traces, a valid slip-onset update, or the
no-intended-contact refusal. Other origins, modes, and refusal reasons are
explicitly unsupported and return a failure status.

## Acceptance

- A standard-library Python command consumes the existing sidecar directly and
  emits a machine-readable recomputation summary.
- It independently recomputes the `23023:17` interval
  `[0.47116960533439084, 0.5484442443281078]` within absolute tolerance
  `1e-12` and reports `Updated(SlipOnset)`.
- It independently recomputes `23024:17` as
  `NoInference(NoIntendedContact)` and preserves `[0.05, 0.8]`.
- Tests reject a changed raw wrench with a stale digest, a changed stored
  interval, and an incomplete or altered contact trace.
- Unknown schema versions, modes, or unsupported no-inference reasons are
  rejected; no input mutation or simulator invocation occurs.
- The Python module imports no Rust verifier and launches no simulator or
  subprocess.

## Maintenance

Keep the verifier implementation separate from the Rust library and use only
Python's standard library. The Rust serde structures remain the field schema.
When a v1 evidence gate or interval rule changes, update the Python reference
independently and retain regression cases for both archived positive and
negative records. A separate implementation is a cross-check, not physical
validation or evidence authenticity.
