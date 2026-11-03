# Seed 21006 diagnostic checkpoint

- The one-run MuJoCo trial completed with `Success`, zero unauthorized writes, and no lift action.
- The sealed sidecar independently verified `PASS`: 6,130 pose samples and 25,912 per-contact force samples.
- Slip onset was bracketed at 23.978–23.980 s at the 1 mm threshold. The simulated sliding coefficient was 0.631634; it lies inside the inferred support-friction interval [0.335121, 0.8]. The tool-wrench bracket and four obj0/table contact samples are in `OFFLINE_ANALYSIS.json`.
- Freeze verification passed before the run reservation, but the freeze and source changes were not committed before replay. The frozen working tree allowed expected target contact through probe transit. The final source restores the original transit contact gate and separately captures diagnostic pose/contact evidence; it has not been simulator-replayed because the one-run budget is consumed.
- No further trial was run.

See `RESULT.json`, `FREEZE.json`, `FREEZE_VERIFY.json`, `OFFLINE_VERIFY.json`, and `OFFLINE_ANALYSIS.json`.
