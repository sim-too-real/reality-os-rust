# Next held-out probe seed — command preserved

Seeds 21004–21007 one-run budgets are consumed. Transit capture without
weakening the unexpected-contact gate is already in source. Further end-to-end
simulator evidence requires a **new** seed frozen against a committed source.

## Exact next command (after freeze commit)

```powershell
$env:REALITYOS_ROTATED_PROBE_DIAGNOSTIC_DIR='C:\projects\reality-os-rust\docs\superpowers\evidence\rotated-probe-heldout-seed-21008-20261104'
$env:REALITYOS_ROTATED_PROBE_TRIAL_SEED='21008'
$env:REALITYOS_ROTATED_PROBE_TRIAL_INDEX='0'
Remove-Item Env:REALITYOS_ROTATED_PROBE_PREPARE_ONLY -ErrorAction SilentlyContinue
cargo test -p realityos-verify --test manipulation_benchmark iiwa_rotated_probe_unbracketed_motion_diagnostic_capture -- --ignored --exact --nocapture
```

## Prerequisites

1. Create evidence directory and prepare-only preflight.
2. Freeze trial/preflight/source hashes with `simulation_runs_max=1`.
3. Commit freeze **before** first simulator step.
4. Do not replay if budget exhausted.
5. Never weaken Governor expected-contact policy to improve success rate.

## This session (without new simulator seed)

- Claim-separation portfolio: PASS (5 analyses).
- Offline failure-class classifier for transit-abort-without-sidecar: tested on sealed 21005.
