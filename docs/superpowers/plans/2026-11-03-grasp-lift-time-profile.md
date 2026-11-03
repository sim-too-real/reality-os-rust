# Grasp-lift time-profile experiment plan

> Execution is inline on canonical `main`; do not create a branch, worktree,
> or copied checkout. Tasks use checkbox syntax.

**Goal:** Evaluate a witness-bound trapezoidal time profile against the
unchanged grasp/lift path and safety gates.

**Architecture:** Freeze exact current-source inputs and paired baseline
results, then change only time parameterization. Bind the profile identity
into the witness/digest and have the simulator executor reject unsupported
profile identifiers.

**Tech stack:** Rust workspace, existing MuJoCo integration harness, existing
manipulation verifier.

**Spec:** [Grasp-lift time-profile experiment](../specs/2026-11-03-grasp-lift-time-profile.md)

## Global constraints

- Work in `C:\projects\reality-os-rust` on local `main` only.
- Do not change the 25 mm lift bound, collision proof, force ceiling, verifier,
  authority, or release flags.
- Freeze inputs and baseline results before changing production code.
- No robot actuation, paid compute, purchasing, or external contact.

## Review focus

- A witness cannot claim a time profile that the executor does not apply.
- The same geometric joint path and fixed simulation step budget are retained.
- The per-finger force limit and independent hold criteria remain unchanged.
- Expected-refusal cases remain refusals; no false acceptance is introduced.
- Fresh cohort selection is based on frozen input labels, not treatment results.

## Tasks

1. [x] Freeze current `main` identity, the existing 21006:21 and 21010:25
   inputs, and the first seven resolved expected-success inputs absent from
   the local evidence archive in the fixed seed/index selection order. Run
   and commit their uniform-profile baseline before treatment.
2. [x] Add failing tests for a witness-bound profile identifier, the
   trapezoidal progress function, and executor rejection of unsupported
   profile identifiers.
3. [x] Implement the `r = 0.25` trapezoidal progress function for the
   experiment; bind its stable identifier into the witness and digest; execute
   only that profile in the verifier path. The production prototype was restored
   after the primary gate failed.
4. [x] Replay the known 21010:25 and 21006:21 controls first. The primary
   21010:25 divergence remained and worsened, so the seven candidate cases
   were not replayed; the negative result is recorded in `RESULT.md`.
5. [x] Run focused semantics/verifier tests, formatting, and the relevant
   broader test command; independently replay the saved results. Full semantics
   and verifier-library suites, integration compile, offline uniform controls,
   formatting, and saved-result integrity checks passed.
6. [x] Apply the spec decision rule. The candidate failed; record the negative
   result and do not replay the seven-case candidate cohort. Restore uniform
   production files, commit and push experiment evidence plus restored code to
   `origin/main`, and verify local and remote `main` match.