# Held-Out Probe Trial 21002 (2026-11-04)

**Classification: VALIDATED LEARNING; ONE HELD-OUT SIMULATOR CASE PASSED; NOT_ADMITTED.**

Frozen trial `v7:development:menagerie_iiwa14:push_probe_replan:21002:0` used seed `21002`, independent of the original seed 21000. One simulation completed the expected outcome: `success` after one authorized `probe:+x` and one authorized `push:+x`; inference was `updated:SlipOnset` with interval `[0.3975340575, 0.4889387859]`, and unauthorized writes were zero. The sidecar verifier passed for one measurement.

Offline recomputation found the first 1 mm crossing at `21.858 s`, with a force lower bound of `7.3536 N`. The first 100-step endpoint was at `22.002 s`, with `5.6676 N`; the old endpoint cadence would also have resolved this sample. It therefore validates one held-out success but does not recreate the original missed-endpoint boundary.

Seed 21001 remains an unclassified simulator capture failure: the old runtime returned an error on an empty sidecar before preserving the episode. That result is not counted as a task pass or task failure and was not replayed. Commit `c49bd40` adds the episode companion artifact on that failure path.

No physical measurement, full-lift replay, purchase, fabrication, or external contact occurred. One held-out success is insufficient to claim broad reliability or authorize full-lift replay.

**Next:** freeze seed 21003 as a third independent trial and run it once. Preserve the 1 mm / 0.5 N gates and sidecar verification. Do not run a full-lift replay until the held-out evaluation covers the cadence-miss boundary independently.
