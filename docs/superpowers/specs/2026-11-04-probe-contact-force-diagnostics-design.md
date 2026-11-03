# Per-Step Probe Contact Force Diagnostics

## Intent

The seed 21003 sidecar verifies time-aligned tool-wrench evidence, but it records support contacts only as body and geometry names. Its second probe has a force-supported 1 mm crossing, yet the inferred friction bounds do not overlap the prior. Capture MuJoCo contact force and friction at the same existing pose samples so the next frozen case can compare support response directly with the tool-wrench estimate.

## Goals

- Record each contact's body and geometry identities, contact position and frame, five friction values, local force and torque, normal force, tangential force, and availability alongside every probe pose sample.
- Bind the new values into per-sample and trace digests; validate alignment with the existing contact-pair list.
- Keep old sidecars readable and verifiable when the optional contact-force field is absent.
- Extend offline analysis to report `obj0`–`table` forces at and around each slip onset.

## Non-goals

- Do not change friction inference, action selection, or the 1 mm motion / 0.5 N force gates.
- Do not use contact-force snapshots as inference inputs.
- Do not rerun seeds 21000–21003 or run a full-lift, physical, or commercial experiment.

## Approaches

1. **Extend the existing per-pose evidence sample (chosen).** The worker already emits one pose/contact snapshot per simulator step, and the Rust path binds each pose to a timestamp and wrench sequence. Add optional force data at that boundary, preserving a single aligned record and backward compatibility.
2. **Add a parallel contact log.** This keeps pose schema unchanged but creates a second stream that needs its own alignment, digest, and missing-sample rules. It duplicates work already performed for each pose.
3. **Change the inference bounds.** This would change decisions before identifying whether the discrepancy comes from tool-wrench estimates or simulated support forces. It also risks weakening fail-closed behavior.

## Constraints and success criteria

The worker's existing MuJoCo contact state is the source. The diagnostic field must not influence control or inference. Existing sidecar records without the field must continue to verify. A new frozen trial must show a complete, digest-verified contact-force trace and an offline comparison at both slip onsets; all thresholds and authority checks stay unchanged. Run no more than one simulator replay for the new seed.

## Builder choices

Use a versioned optional field on `ProbeObjectPoseSample` and its digest, with a contact snapshot list that mirrors the exact contact-pair ordering. Capture the fields already produced by MuJoCo's contact state. Keep old digest versions selected when the optional field is absent.
