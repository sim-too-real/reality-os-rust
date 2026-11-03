# Supplemental archived probe interval replay: 23021:17

This is a retrospective offline check of an already archived sidecar. It was
run after the original two-case freeze and must not be read as a preregistered
third case, a new simulation, or a physical measurement.

The standard-library Python reference recomputed `Updated(SlipOnset)` from
the saved full sidecar as `[0.47885574015040633, 0.7211370850171951]`. The Rust
offline verifier returned `[0.47885574015040633, 0.721137085017195]`; the
difference is below the existing `1e-12` absolute tolerance. Both outputs
report the same trial digest, measurement digest, and first XY motion bracket.

The sidecar contains 5,124 wrench samples and 5,123 pose samples with a
complete per-step contact trace. No simulator was started. The interval is
simulation evidence only; it is not a physical friction measurement,
population estimate, benchmark gain, admission, or release evidence.

The archived `23020:17` `NoInference(Unidentifiable)` sidecar remains outside
the reference's supported cases because its contact-pair fields are null.
The independent verifier rejects that record as an incomplete contact trace
instead of assuming that contact occurred.
