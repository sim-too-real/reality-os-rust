# Reasoner-to-authority bridge design

**Date:** 2026-10-15

**Repository baseline:** `54c7e243486ffe3c97d704e506fdbb6012616790` (`main`)
**Evidence class:** Simulation only

## Goal

Close the handoff between the physical reasoner and the existing scoped simulation authority. Today, the MuJoCo scenario manually reconstructs a `PlanStep`, builds a scope, issues a grant, copies grant fields into `ExecutionAuthorization`, and asks the reasoner to bind it. That path is useful as a test, but callers do not have one reusable, checked API.

The first supported action family is planar push, including a bounded information probe. Pinch-grasp admission is deferred because the current action contract measures linear stroke and does not yet carry a gripper-specific width, force, and hold envelope.

## Design

Keep the semantics package independent of session authority and plant I/O. The reasoner will expose two separate outputs: an `authority_request` for a justified but not yet executable probe, and an `executable` action only after a matching scoped grant is present. A request carries the exact proved witness and current observation identity; it never authorizes motion.

When a probe is safe and informative but has no scope grant, the reasoner runs the same canonical selector with an internal, explicitly non-exported scope preview. If the selector chooses that probe and future interaction is proven preserved, the report exposes its exact step as an authority request. The stored decision remains refused until a real grant is attached. Probe steps are frozen even when goal mechanics are unknown, because the probe's safety and return-path evidence are evaluated independently.

Add `realityos-reasoner-bridge` as a composition layer depending on semantics, session, and the existing Reality OS authority. It accepts the current request, obtains the current report, selects only its `executable` or `authority_request`, and rejects unsupported families. It prepares the action from a fresh proof, builds the scope from that frozen contract, issues a `SIMULATION_ONLY` grant, attaches the grant binding to a copy of the same request, and runs the canonical selector again. It returns an authorized action only if the selector still chooses the exact candidate, witness, epoch, and stroke. All grants expire according to caller-supplied limits and remain non-metal.

The bridge exposes no device writer. Its API returns the reasoning report, scoped simulation grant, and authorized frozen action for downstream simulation. It cannot turn a learned proposal into authority, skip fresh evidence, authorize a conditional future plan step, or claim physical qualification.

## Required invariants

1. An authority request is never reported as executable and has no grant.
2. A probe authority request comes only from the canonical selector after proof, freshness, observability, recoverability, and forbidden-action checks.
3. A probe `PlanStep` retains the exact witness and requested stroke even when the goal mechanics remain unknown.
4. The bridge issues grants only for planar-push actions and only from a current `ReasoningReport`.
5. The final selector run must return the same action key, candidate ID, witness digest, witness contents, and stroke as the requested step.
6. `ExecutionAuthorization` is copied only from the issued grant and is rechecked against a fresh proof.
7. Missing/stale observations, incomplete bounds, expired grants, unsupported families, or changed selection fail closed.
8. Every bridge result stays `SIMULATION_ONLY`, `metal == false`; no hardware transport is added.

## Deliberate limits

This slice does not qualify the XL330 or any other hardware, implement camera calibration, create a safety certificate, add a device adapter, change the runtime session boundary, or claim fleet readiness. The next physical gate remains the reviewed XL330 experiment already documented in `docs/METAL_EXPERIMENT.md` and `docs/metal/RUN_B_PATCHED_AUTHORITY.md`. The next manipulation gate is a calibrated arm/camera task with independent outcome measurement.

## Acceptance

The reasoner can describe a safe unscoped probe as a non-executable authority request, then produce the same exact probe as executable after a matching grant is attached. The bridge can carry that request through scope issuance and re-selection without hand-built push/probe steps. Negative cases refuse stale evidence, invalid bounds, unsupported families, no justified action, and expired scope. The MuJoCo push/probe/replan scenario uses the bridge. On the current Windows host, workspace testing is blocked by pre-existing Unix-only permission APIs in `realityos-vport`; the modified crates and MuJoCo integration target are tested independently.
