# Physical Decision Runtime Design

**Date:** 2026-09-26

**Baseline:** `efbf75fca84d4fef26fa1f21ec3437c72a606fc6`

**Scope:** One coherent PUSH decision and supervised execution loop in the current semantics/verifier architecture. No second interaction family or hardware feature is added in this milestone.

## Goal

Replace the verifier loop’s independent goal, recoverability, belief, and probe choices with one typed physical decision. Every selected interaction is frozen with its prediction and execution contract. The same consequence assessor runs after nominal execution, fault-injected execution, and abort. A supervisor observes policy-visible state after bounded witness phases and can discard the remaining frozen witness before it executes.

## Architecture

Add `crates/semantics/src/physical_decision.rs` as the single semantic selector. `PhysicalDecisionContext` carries a policy observation snapshot, goal, evaluated physical candidates, physical belief and live hypotheses, candidate recoverability/robustness evidence, forbidden action keys, current contact state, observation capabilities, and remaining interaction budget. It has no simulator handle, privileged future state, plant write capability, hardware transport, or robot-branch identity.

Each decision candidate contains one concrete contact/action identity and its proof and evidence. It distinguishes goal interactions, physical probes, and explicit contact-transition actions. It includes typed hard-rejection reasons, physical-effect prediction, goal effect, belief robustness, recoverability, and any probe discrimination evidence. A selected outcome contains the exact candidate, action key, contact, executable witness, predicted effect, goal effect, robustness, recoverability, and typed rationale.

The only entry point that selects an action is:

```rust
pub fn decide_physical_action(
    context: &PhysicalDecisionContext<'_>,
) -> PhysicalDecision
```

The decision function applies hard rejection first: malformed or stale required evidence, invalid geometry, unreachable or collision-inadmissible interaction, absent witness, infeasible or unknown required mechanics, forbidden action, authority-ineligible action, unrecoverable action, and actions unsafe for part of the supported belief set cannot be ranked back into eligibility. A contact change is a typed `ContactTransition` decision with its own proved leave/reobserve/approach witness; it is not metadata that can be bypassed when another selector replaces a candidate.

Among eligible choices, it selects a robust strict-progress goal interaction when one exists. Otherwise it may select a physical probe only when uncertainty changes the available goal choice, the probe distinguishes live explanations, a policy-visible sensor can observe the distinction, and the probe is safe and recoverable. It selects a contact transition when the current contact cannot execute the selected interaction without a leave/reobserve/new-approach sequence. If no justified action class exists, it returns a typed refusal, insufficient-evidence, infeasible, or currently-unachievable result. Preferences are lexicographic within an eligible class; they never combine progress, information, or recoverability through arbitrary weights.

`receding_horizon_step` receives the enriched context, calls the canonical selector once, and performs only state/record propagation from that result. The verifier computes proofs and evidence, passes them to the semantic loop, then propagates the returned decision. The recoverability override, belief override, and probe path no longer mutate `step.selected` or launch a separately chosen action. Contact transitions are represented in the decision and must be executed as their own required witness phases.

## Frozen action, consequence assessment, and supervision

Add a typed frozen action contract before the first quantum. It records ActionId, ActionKey, CandidateId, ContactId, witness identity and contents, requested stroke, frozen prediction, belief snapshot, recoverability, execution envelope, required observation contract, and the independently returned authority result. The decision and supervisor have no plant-write capability.

Add one generic consequence function:

```rust
pub fn assess_consequence(
    prediction: &FrozenPrediction,
    observation: &ObservedConsequence,
) -> ConsequenceAssessment
```

It runs after every completed action and every abort, independent of development flags or fault injection. The result reuses existing discrepancy and belief evidence and has typed statuses for consistent, contradicted, underdetermined, insufficient evidence, outside model regime, and execution diverged. Missing tracking, geometry, contact, or motion data remains UNKNOWN; it is never changed to a numeric zero for convenience.

Extend the existing execution-envelope logic into one supervisor API:

```rust
pub fn supervise_execution(
    frozen: &FrozenAction,
    progress: &ExecutionProgress,
    observation: &RuntimePolicyObservation,
) -> SupervisorDecision
```

The supervisor can continue the frozen action, abort and require reobservation, report completion, report unavailable evidence, or report lost authority. It cannot choose a candidate, change a task, widen authority, update belief, or create another command. Every output names the frozen action and any failing observation/evidence.

## Policy observation and PUSH execution quantum

The verifier’s outer decision loop must stop reading `VerifierTruth` for candidate selection, envelope checks, discrepancy hypotheses, probes, and belief updates. It consumes an explicit policy observation with observation ID, source, timestamp, model/calibration epoch, units, and freshness. In this simulation-only scenario the existing perfect-perception channel may provide object pose and joint state, clearly labelled as a simulated policy sensor. Verifier truth remains available only to the independent oracle and physical-consequence score. Missing or invalid policy measurements remain UNKNOWN. In particular, non-finite joint state cannot be sanitized to zero.

One PUSH quantum is one already-proved named witness phase transition, not an arbitrary count of interpolation commands. The existing witness sequence is current-to-approach, approach-to-contact, contact-to-mid-stroke, and mid-stroke-to-end-stroke. The executor independently authorizes bounded control writes inside the phase. After each phase it emits a fresh policy observation and calls the supervisor before dispatching the next phase. A divergence during contact-to-mid-stroke can therefore stop the frozen action before mid-stroke-to-end-stroke; the unexecuted remainder is invalidated and never resumed.

## Development evidence

A deterministic SIMULATION_ONLY scenario uses two otherwise identical worlds and the same original selected action. One executes the complete witness. The supervised run observes an injected physical divergence after a bounded witness phase, aborts before the remaining phase, takes a fresh policy observation, assesses the consequence through the common path, updates hypotheses and belief only from visible evidence, then asks the same canonical selector for a safe informative probe or a justified goal action. The probe receives the same frozen contract, independent simulated authority, and phase supervision. Its observation changes belief and the next canonical decision. The scenario asserts a strict reduction in physical displacement, exact selected witness identity, no post-abort quantum, zero unauthorized writes, no forbidden/unauthorized resurrection, UNKNOWN preservation, no verifier-truth influence on policy result, and SIMULATION_ONLY provenance.

Typed cycle evidence records observation and goal identities, belief identity, all candidate IDs and hard rejections, robustness, recoverability, forbidden state, selected class/action/witness, prediction, envelope, authority result, executed prefix, abort reason, observed consequence, assessment, belief delta, and work counters.

Counters include candidate count/evaluations, IK, FK, collision, Jacobian, mechanics, recoverability, belief-domain and probe evaluations, decision wall time, execution quanta, observations, replans, and aborts. No optimization or speed claim is made in this milestone.

## Verification and boundaries

Add decision-class invariants proving forbidden, authority-ineligible, and non-executable candidates cannot be selected or resurrected; strict goal usefulness cannot be bypassed; expanded uncertainty or loss of evidence cannot strengthen a physical claim; and the selected action identity is the identity executed as a full witness or strict prefix. Add nominal, contradicted, underdetermined, insufficient-evidence, and outside-regime consequence examples. Test the supervisor’s fail-closed observation contract and its inability to create or alter actions.

Run focused development tests and routine CI checks only. The repository workflow skips the four frozen scoring tests during routine runs and gates the full frozen MuJoCo suite behind explicit workflow dispatch; this milestone does not enable that gate or rewrite frozen evidence.

After this milestone, exactly one next milestone is selected: extract the mature semantic decision/consequence/supervision core for reusable production runtime consumers. PUSH plus an existing GRASP/HOLD/RELEASE family remains later work; no interaction family is duplicated here.
