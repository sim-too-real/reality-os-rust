# Physical Decision Runtime Implementation Plan

This plan implements the reviewed milestone in the current checkout. Each task starts with a focused failing regression, applies the smallest generic change, and reruns the same evidence. The execution stays in the current repository; no subagents or worktree are used.

## File map

- Create `crates/semantics/src/physical_decision.rs`: typed candidate evidence, canonical decision context/outcome, hard eligibility funnel, and the only physical selector.
- Create `crates/semantics/src/physical_consequence.rs`: frozen prediction, policy-observed consequence, typed assessment, and shared discrepancy/belief evidence.
- Modify `crates/semantics/src/execution_envelope.rs`: frozen action contract, typed policy observation contract, execution progress, and the single bounded supervisor.
- Modify `crates/semantics/src/goal_loop.rs`: consume and propagate `PhysicalDecision`; retain state update and causal recording without selecting.
- Modify `crates/semantics/src/lib.rs`: export the two reusable modules.
- Modify `crates/verify/src/goal_directed.rs`: build decision evidence, invoke the canonical selector once, propagate its decision, and record typed cycle evidence/work counters.
- Modify `crates/verify/src/manipulation.rs`: expose witness-phase boundaries and policy-visible observation snapshots to the verifier supervisor; stop dispatching later phases after abort.
- Modify `crates/verify/src/observation.rs` only as needed to preserve observation source, freshness, epoch, units, and UNKNOWN values without consulting verifier-only fields.
- Update focused unit tests in the owning semantic modules and development tests in `goal_directed.rs`. Do not change frozen evidence or the four explicitly skipped scoring tests.

## Review focus

1. **Missing or stale policy observation:** supervisor returns evidence unavailable and does not treat any missing motion/tracking/contact value as zero. Pin in the observation and supervisor tests.
2. **Forbidden action offered as a probe:** canonical selection cannot pick it through the probe class. Pin in a decision test using a real action key.
3. **Authority-ineligible or non-executable candidate with higher progress:** it remains rejected through every decision class. Pin in decision tests for both goal and probe candidates.
4. **Larger supported uncertainty or unrecoverable worst case:** candidate robustness/decision status cannot improve. Pin with property-style evidence-subset/domain tests.
5. **No safe, distinguishable, observable probe:** return a typed refusal/evidence outcome and execute nothing. Pin in the canonical decision tests.

## Task 1 — Add the canonical physical decision API

**Files:** create `crates/semantics/src/physical_decision.rs`; modify `crates/semantics/src/lib.rs`; test in the new module.

Define:
- `PhysicalDecisionContext<'a>` with policy observation identity/capabilities, `PlanarObjectGoal`, evaluated decision candidates, `PhysicalParameterBelief`, live `DiscrepancyKind` hypotheses, forbidden keys, contact state, and remaining interaction budget.
- `PhysicalDecisionCandidate` with its `PhysicalInteractionCandidate`, goal/probe/contact-transition role, typed hard rejections, `BeliefRobustness`, `RecoverabilityClass`, probe information/sensor-distinguishability evidence, and lexicographic preference facts.
- `PhysicalDecision` variants `GoalInteraction`, `PhysicalProbe`, `ContactTransition`, `GoalReached`, `Refuse`, `InsufficientEvidence`, `PhysicallyInfeasible`, and `CurrentlyUnachievable`. A selected variant owns the exact candidate and records action key, contact, executable maneuver, physical prediction, goal effect, robustness, recoverability, and typed rationale.
- `decide_physical_action(&PhysicalDecisionContext<'_>) -> PhysicalDecision`.

Write failing tests first: forbidden candidates rejected in every class; authority-ineligible/non-executable candidates never selected; strict goal usefulness cannot be bypassed; unrecoverable/unsafe candidates never selected; contact transitions preserve required leave/reobserve/approach phases; robust goal beats probes; probe requires positive decision-relevant and observable information; preference ranking cannot change hard eligibility; stable ties preserve deterministic candidate identity. Then implement the hard filter, decision tiers, and lexicographic ranking. No weighted combined score.

Run: `cargo test -p realityos-semantics physical_decision::tests -- --nocapture`.
Expected: each new test fails before implementation for its asserted behavior, then passes after the selector is implemented.

## Task 2 — Replace every verifier-side selector and mutation

**Files:** modify `crates/semantics/src/goal_loop.rs`, `crates/verify/src/goal_directed.rs`; tests in both.

Change `receding_horizon_step` to the signature `receding_horizon_step(context: &PhysicalDecisionContext<'_>, state: LoopState, observed_consequence: Option<&ObservedConsequence>) -> RecedingHorizonResult`. It calls `decide_physical_action` once and populates `selected`/records only by propagating that result. Preserve blacklist updates from observed contradiction. Remove `admissible_override_indices`, `select_recoverable_override_index`, direct goal/probe selection branches, and every independent assignment/clear of `step.selected`. Compute probe evidence against actual proved candidate contacts and feed it through the same context; do not launch a probe from a synthetic ranking result. Update contact switching to be a typed selected transition that the verifier must execute through the frozen witness contract.

Add a whole-path identity regression: the candidate/action/witness returned by the canonical decision equals the candidate/action/witness that reaches simulated authority and execution. Add a cross-class forbidden/authority test proving the verifier has no route around the canonical filter.

Run: `cargo test -p realityos-semantics goal_loop::tests -- --nocapture` and focused `cargo test -p realityos-verify goal_directed::tests::decision -- --nocapture`.
Expected: one semantic call returns the final class/candidate; no verifier code selects or replaces a physical action.

## Task 3 — Make consequence assessment universal and typed

**Files:** create `crates/semantics/src/physical_consequence.rs`; modify `crates/semantics/src/lib.rs` and `crates/verify/src/goal_directed.rs`; focused tests in the new semantics module and verifier loop.

Define `FrozenPrediction`, `ObservedConsequence`, `ConsequenceStatus`, and `ConsequenceAssessment`, reusing existing discrepancy and belief types rather than cloning their taxonomies. Implement `assess_consequence(&FrozenPrediction, &ObservedConsequence) -> ConsequenceAssessment`. Preserve each missing sensor field as UNKNOWN. Call it after every completed or aborted physical interaction regardless of `ExecOptions.guard` or injected disturbance. Keep fault injection in the simulated world response only; it must not toggle whether assessment runs.

Write tests first for consistent nominal motion, contradicted prediction, underdetermined competing explanations, insufficient evidence when required fields are missing, outside-model-regime, and execution divergence. Add a monotonic evidence-removal property: removing tracking, geometry, contact, or pose evidence cannot strengthen identifiability or change UNKNOWN into a measured zero. Integrate one nominal and one injected run and assert both use the same assessment function.

Run: `cargo test -p realityos-semantics physical_consequence::tests -- --nocapture` and the focused verifier assessment test.
Expected: typed assessment appears on nominal and fault-injected runs; hypotheses/belief update only from policy-visible evidence.

## Task 4 — Freeze actions and add the sole supervisor

**Files:** modify `crates/semantics/src/execution_envelope.rs`; tests in that module; adapt the verifier trace.

Define `FrozenAction` containing action ID/key, candidate/contact IDs, witness ID and contents, requested stroke, `FrozenPrediction`, belief snapshot, recoverability, execution envelope, and required observation contract. Define provenance-bearing `RuntimePolicyObservation`, `ExecutionProgress`, and typed `SupervisorDecision` variants Continue, AbortAndReobserve, Completed, EvidenceUnavailable, and AuthorityLost. Implement `supervise_execution(&FrozenAction, &ExecutionProgress, &RuntimePolicyObservation) -> SupervisorDecision`. Make all inputs immutable. The supervisor reports whether the already-frozen action may continue; it never chooses a candidate or creates authority.

Test first: missing/stale/wrong-epoch/wrong-unit evidence fails closed; missing tracking stays UNKNOWN; each guard detects its condition; progress belonging to another ActionId/WitnessId refuses; abort invalidates the remainder; supervisor output cannot alter the frozen decision; authority loss stops immediately. Retain the existing missing-tracking fail-closed invariant.

Run: `cargo test -p realityos-semantics execution_envelope::tests -- --nocapture`.
Expected: typed outcomes identify the frozen action, and only the selected witness or a strict prefix is representable.

## Task 5 — Observe and supervise between proved witness phases

**Files:** modify `crates/verify/src/manipulation.rs`, `crates/verify/src/observation.rs`, and `crates/verify/src/goal_directed.rs`; focused tests in the verifier.

Expose each existing proved phase transition as a named execution quantum: current-to-approach, approach-to-contact, contact-to-mid-stroke, and mid-stroke-to-end-stroke. The interpolation commands within a transition remain bounded low-level control writes and are not redefined as action quanta. After each phase, adapt the declared simulated policy observation into the semantics runtime-observation type, preserve source/time/model/calibration epoch/units, and call the supervisor before invoking the next phase. Replace decision/supervision/control-feedback reads from `VerifierTruth` with the policy observation contract; keep verifier truth in the independent oracle and displacement scorer. Remove the non-finite-to-zero sanitization for any value entering the policy path.

Write the early-stop regression before implementation. Use identical deterministic scenes and the same frozen action. In the unsupervised scene execute all witness phases. In the supervised scene inject a physical disturbance during the first stroke quantum, observe it through the policy-visible sensor, abort after contact-to-mid-stroke, and prove mid-stroke-to-end-stroke never executes. Assert exact selected witness identity, a strict physical-displacement reduction, no unauthorized writes, no hidden-field effect on the policy result, UNKNOWN for missing observations, and SIMULATION_ONLY provenance. Replace the old equality assertion only after this strict-prefix evidence passes.

Run: `cargo test -p realityos-verify goal_directed::tests::supervised_prefix_aborts_before_witness_remainder -- --nocapture`.
Expected: the supervisor stops before the full original witness and the remaining phase is absent from the execution trace.

## Task 6 — Complete the canonical probe/replan/belief loop and counters

**Files:** modify `crates/verify/src/goal_directed.rs`; add focused integration tests.

On abort, invalidate the remainder, create a fresh policy observation, run the universal consequence assessment, update only supported hypotheses/belief, regenerate and prove contacts, then call the same canonical selector. Execute any selected probe with the same freeze, independent SimAuthority, policy observations, phase supervision, and consequence assessment. After the probe, record the typed belief delta and call the same selector to choose a recoverable goal interaction. Stop probing when a robust goal action appears, the goal is infeasible, or no safe informative probe remains.

Instrument and report candidate count/evaluations, IK, FK, collision, Jacobian, mechanics, recoverability, belief-domain, probe evaluations, decision wall time, execution quanta, observations, replans, and aborts. Add typed decision-cycle evidence instead of relying on formatted strings for semantics.

The integration test must demonstrate: multiple candidate contacts; one canonical selected action; frozen prediction/contract; independently allowed bounded phases; observable divergence and abort; no stale remainder; underdetermined consequence with several live explanations; safe decision-relevant probe; probe observation changes belief; the same canonical selector then chooses a recoverable goal action; zero unauthorized writes; no forbidden or unauthorized resurrection; no hidden truth leakage; missing evidence stays UNKNOWN; SIMULATION_ONLY.

## Final verification

Run the focused semantic and verifier tests above, then the applicable local authority workflow checks. In particular, use the workflow’s exact four frozen-scoring skips in routine workspace and MuJoCo test commands. Do not dispatch the explicitly gated full frozen MuJoCo suite. Recheck all five original untracked-file SHA-256 values, inspect the complete diff, and report live CI as unavailable if network access still prevents querying the exact commit. Report measured wall time and work counters; make no speed claim.
