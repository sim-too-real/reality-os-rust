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
2. [ ] Add failing tests for a witness-bound profile identifier, the
   trapezoidal progress function, and executor rejection of unsupported
   profile identifiers.
3. [ ] Implement the `r = 0.25` trapezoidal progress function; bind its stable
   identifier into the witness and digest; execute only that profile in the
   verifier path.
4. [ ] Replay the known 21010:25 and 21006:21 controls first. If the primary
   21010:25 runtime divergence remains, stop; otherwise replay the seven
   frozen cases once and compare outcomes, force margins, follow residuals,
   false acceptances, and unauthorized writes against the committed baseline.
5. [ ] Run focused semantics/verifier tests, formatting, and the repository's
   relevant broader test command; independently replay the saved results.
6. [ ] Keep the change only if the spec's decision rule passes. Record a
   negative result and restore production code otherwise. Commit and push the
   experiment artifacts and passing implementation to `origin/main`; verify
   that local and remote `main` match.
