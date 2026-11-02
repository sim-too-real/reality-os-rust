# First-Action Preflight Design

## Intent

The suite resolver currently proves that `probe:+x` is feasible, while the runtime separately ranks and authorizes its first action. A read-only preflight should determine the actual authorized first action before the simulation writes controls or advances physics. This prevents spending a full trace run on a resolved trial that will not enter the probe path.

## Approaches

1. **Shared runner preflight (selected).** Reuse the benchmark runner's real initial observation, request construction, reasoner ranking, and authorization. Return immediately after the first authorization decision and record the resolved-trial digest, initial observation, selected action, control-write counts, and simulation time. This keeps prediction aligned with runtime and makes a zero-write, zero-step result directly checkable.
2. **Separate synthetic ranker.** Recreate an initial observation from the suite request and call the reasoner outside the runtime path. This is cheaper, but repeats the current feasibility/runtime mismatch and can drift from the simulator observation.
3. **Audit only full episodes.** Preserve the first-action selection in simulation results and compare it with the probe trace. This is easy to integrate, but it still spends the full simulation when the probe action is not first.

## Goals and limits

- Add an opt-in `PushProbeReplan` preflight that returns the action selected and authorized by the same path used by execution.
- Bind the preflight to the resolved trial and exact initial observation, and expose evidence that control writes and simulation-time advance are both zero.
- Preserve the existing benchmark result schema and all runtime selection, authorization, wrench, and evaluator-label behavior.
- Keep this evidence `SIMULATION_ONLY`; it is not a benchmark outcome or physical validation.
- Do not rerun any previously frozen input. Only a fresh expected-success input outside pilot and public cohorts may be frozen after the preflight reports `probe:+x`.

## Implementation choices

Use an internal preflight capture on the existing runner and stop immediately after the first `authorize_current_action` result, before any witness execution, `set_ctrl`, or simulator step. Reject other task families so the current preflight API cannot enter resource qualification paths that may actuate. Add a focused regression test for the receipt invariants, then use a bounded fresh-input selection and at most one frozen simulation to compare prediction with execution.
