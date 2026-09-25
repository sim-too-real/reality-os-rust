# Discrepancy Evidence Truth Boundary Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Prevent absent tracking or geometry measurements from being interpreted as nominal evidence that identifies support friction.

**Architecture:** Keep the observation schema's `Option` fields and existing independent positive hypotheses. In `hypothesize`, gate support-friction attribution on explicitly measured low tracking and geometry residuals; represent missing discriminators in the existing unknown/insufficient vocabulary.

**Tech Stack:** Rust 2021 workspace; `realityos-semantics`; `realityos-verify` integration tests; `cargo test`.

**Spec:** `docs/superpowers/specs/2026-09-26-physical-input-truth-boundary-design.md`

## Global Constraints

- Removing physical evidence from an otherwise identical scenario must never strengthen its physical claim.
- Never interpret absent `tracking_error_m` or `geometry_residual_m` as measured zero.
- Preserve independent positive evidence and existing freshness, reachability, and contradiction gates.
- Keep the change in discrepancy reasoning; do not alter selector ranking or normal-execution integration in this plan.
- Do not add a dependency for property testing; use deterministic enumeration of discriminator subsets.

## Review Focus

- High ratio with `tracking_error_m = None`: do not emit support-friction identification.
- High ratio with `geometry_residual_m = None`: do not emit support-friction identification.
- High ratio with both residuals explicitly present and low: preserve the currently supported friction hypothesis.
- High ratio with present high tracking error: preserve `ExecutionTrackingDivergence` and do not relabel missing geometry as nominal.
- High ratio with present high geometry residual: preserve `ContactGeometryDisagreement` and do not relabel missing tracking as nominal.

---

### Task 1: Preserve missing discrepancy evidence as unknown

**Files:**
- Modify: `crates/semantics/src/discrepancy.rs`
- Test: `crates/semantics/src/discrepancy.rs` unit tests
- Verify: `crates/verify/src/self_correction.rs` existing discrepancy assertions

**Interfaces:**
- Consumes: `DiscrepancyObservation` optional measurements and `DiscrepancyKind::{StaleOrInsufficientObservation, UnidentifiableFromCurrentEvidence}`.
- Produces: unchanged `hypothesize(&DiscrepancyObservation) -> HypothesisReport` signature; reports must include an insufficient/unknown kind when an attribution discriminator is absent and must not emit `SupportFrictionInconsistent` unless ratio is high and tracking and geometry residual are each explicitly present and at/below their current thresholds (`0.03 m`, `0.02 m`).

- [ ] **Step 1: Add failing property-style discrepancy tests.** Starting from the existing `large_slip()` fixture, enumerate all four present/missing combinations of tracking and geometry; assert every missing-required-measurement case has no `SupportFrictionInconsistent`, includes insufficient/unknown evidence, and is not `Identified`. Add positive-control cases for both measured-low and each independently measured-high field. Name the tests `removing_required_discriminators_never_identifies_friction` and `present_positive_discrepancy_evidence_is_preserved`.
- [ ] **Step 2: Run the focused tests and confirm failure.** Run `cargo test -p realityos-semantics removing_required_discriminators_never_identifies_friction`. Expected: failure because the missing fields currently default to zero and support friction is emitted.
- [ ] **Step 3: Implement explicit missingness gates.** Replace the `unwrap_or(0.0)` residual reads with `Option`-aware predicates; add insufficient evidence when tracking or geometry is absent; only add `SupportFrictionInconsistent` when the high ratio and both measured-low alternatives hold; preserve high tracking/geometry and other independent hypotheses.
- [ ] **Step 4: Run discrepancy, probe-selection, and verifier regression tests.** Run `cargo test -p realityos-semantics discrepancy::tests`, `cargo test -p realityos-semantics probe_selection::tests`, and `cargo test -p realityos-verify self_correction`. Expected: exit 0; existing fully observed large-slip behavior remains, and incomplete observations cannot strengthen attribution.
- [ ] **Step 5: Commit Task 1.** Stage only `discrepancy.rs` and any directly updated discrepancy expectations; commit as `fix(semantics): preserve missing discrepancy evidence`.
