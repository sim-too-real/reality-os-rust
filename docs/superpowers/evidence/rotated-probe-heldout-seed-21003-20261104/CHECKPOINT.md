# Held-Out Probe Trial 21003 (2026-11-04)

**Classification: HELD-OUT SIMULATOR CASE FAILED EXPECTED INFERENCE; SECOND-PROBE CADENCE-MISS BOUNDARY REPRODUCED; NOT_ADMITTED.**

The frozen trial `v7:development:menagerie_iiwa14:push_probe_replan:21003:0` ran once. The test exited 101 after 393.05 seconds because it expected `updated:SlipOnset` and observed `no_inference:EmptyIntersection`. The captured episode is `unexpected_refusal` (`NO_JUSTIFIED_CURRENT_ACTION`) after two authorized `probe:+x` actions; unauthorized writes were zero. The final belief interval stayed `[0.2590453, 0.3370551]`.

Both sidecar measurements passed the independent verifier. For the second probe, per-pose recomputation found the 1 mm crossing at 65.928 s with a tangential force lower bound of 9.5256 N. The first 100-step endpoint after that crossing was at 66.090 s with a 0 N lower bound, so endpoint-only observation would miss force support at this boundary. The time-aligned observer records the force-supported crossing.

The first probe narrowed the interval to `[0.2590453, 0.3370551]`. The second probe’s recomputed single-measurement bounds were `[0.4173208, 0.4913421]`, which do not intersect that prior interval. Both probe estimates are below the trial’s declared object and table friction coefficient, `0.5136991`. `EmptyIntersection` is the fail-closed result for those recorded bounds; why the inferred bounds sit low and vary between probes remains unresolved. The frozen zero-step preflight recorded four `obj0`–`table` contact points, each with friction vector `[0.5136991, 0.5136991, 0.01, 0.01, 0.01]`, confirming the scenario value reached MuJoCo.

No threshold changed: planar motion remained 1 mm and tangential force lower bound remained 0.5 N. This is one simulator case; it does not establish physical performance or broad reliability. No full-lift replay, physical test, purchase, fabrication, or external contact occurred.

**Next:** capture a bounded per-step `obj0`–`table` contact-force trace alongside each pose and tool-wrench sample, then compare simulator support force with the inferred bounds. Preserve the 1 mm / 0.5 N gates. Do not replay seed 21003 or run a full-lift replay while the held-out inference remains inconsistent.
