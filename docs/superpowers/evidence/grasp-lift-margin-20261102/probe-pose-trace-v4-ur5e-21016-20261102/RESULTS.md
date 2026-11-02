# Seed 21016 trace contract result

**Classification:** negative trace-contract result; simulation only. No benchmark or physical claim.

The frozen v4 UR5e replay retained its complete artifacts. It had 7,254 wrench samples and 7,253 pose samples. The trace was structurally digest-valid and exposed a 1 mm motion bracket, but friction inference returned `NoInference(InvalidMotionTrace)`. Unauthorized writes were zero and no wrench abort occurred.

The first wrench sample was sequence `26000` at `start_s = 52.00200000001759`; the first per-step pose was sequence `26001` at `52.00400000001759`. No pose was emitted for the bootstrap wrench, and the historical trace had no start-pair field. The initial replay harness did not reject this invalid inference, so its test exit code was not evidence that the trace-to-inference contract passed. The independent offline verifier recomputed the refusal.

The freeze, resolved input, result, sidecar, and original offline verification are preserved. `OFFLINE_VERIFICATION_POSTFIX.json` records the same result under the updated release verifier. Do not rerun this frozen input.
