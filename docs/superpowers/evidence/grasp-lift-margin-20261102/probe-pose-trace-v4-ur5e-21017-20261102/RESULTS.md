# Seed 21017 trace contract result

**Classification:** negative trace-contract result; simulation only. No benchmark or physical claim.

The frozen v4 UR5e replay wrote its full artifacts before the strengthened acceptance assertion failed. It had 6,000 wrench samples and 5,999 pose samples. The trace was structurally digest-valid and exposed a 1 mm motion bracket, but inference returned `NoInference(InvalidMotionTrace)`. Unauthorized writes were zero and no wrench abort occurred.

The first wrench sample was sequence `8000` at `start_s = 16.00200000000201`; the first per-step pose was sequence `8001` at `16.00400000000201`. The inclusive trace-boundary filter did not create a pose sample because the worker captures the bootstrap wrench before its physics-step pose loop. The independent offline verifier reproduced the refusal.

The freeze, resolved input, failed test result, sidecar, and offline verification are preserved. `OFFLINE_VERIFICATION_POSTFIX.json` records the result under the updated release verifier. Do not rerun this frozen input.
