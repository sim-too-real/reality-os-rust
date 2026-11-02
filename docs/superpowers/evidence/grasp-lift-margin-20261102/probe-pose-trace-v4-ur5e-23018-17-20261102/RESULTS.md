# Seed 23018 index 17 selection and run result

**Classification:** failed trace-trial selection; simulation only. No trace result or episode outcome was persisted.

The frozen input resolved as `expected_success`, outside both configured v4 cohorts. One simulation ran, but the sidecar verifier rejected the empty measurement list with `InvalidTrial("invalid probe measurement sidecar identity or empty records")`. The runner did not write a sidecar or `RESULT.json`.

Code inspection confirms measurement records are appended only when an executed action key starts with `probe:`. The empty record list therefore indicates that this run did not reach the probe measurement path. The full episode was not returned to the harness after sidecar verification failed, so its outcome and action sequence are unavailable; they are not inferred. The run is closed and must not be repeated.

The result changes the selection method: an `expected_success` label alone does not establish that a trial will execute `probe:+x`. The adjacent `23018:18` input resolved as `expected_refusal` / `UNIDENTIFIABLE_FRICTION` and was not simulated. The empty-sidecar diagnostic now has a dedicated regression test; see the parent [RESULTS.md](../RESULTS.md).
