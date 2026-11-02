# Seed 23017 trace contract result

**Classification:** positive evidence-contract validation; simulation only. Not benchmark evidence, physical evidence, or admission.

The frozen v4 UR5e exploratory replay passed once. The sidecar contains 5,591 wrench samples and 5,590 per-step pose samples. The initial wrench is sequence `8000` at `start_s = 16.00200000000201`; the trace seals `start_wrench_sequence = 8000` to its recorded start pose using the version-2 trace digest. The first per-step pose is sequence `8001` at `16.00400000000201`.

The sidecar verifier independently reported `motion_trace_status: verified`, recomputed the measurement digest and `Updated(SlipOnset)` interval `[0.3509380227123597, 0.456770725562596]`, and recovered the first 1 mm XY crossing bracket `[23.061999999998097, 23.063999999998096]` s. The compact endpoint displacement was `[0.00483345955111014, 0.00040543725955991, -0.00000174712913741937]` m. The episode outcome was `success`, unauthorized writes were zero, and no wrench abort occurred.

This was an exploratory trace-contract check outside the v4 pilot and public cohorts, with no exact-seed baseline. It does not establish a benchmark gain or real-world transfer. Preserve the freeze, input, result, sidecar, and offline-verification files; do not rerun this frozen input.
