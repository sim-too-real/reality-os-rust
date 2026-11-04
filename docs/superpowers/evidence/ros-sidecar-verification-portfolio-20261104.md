# Probe sidecar verification portfolio — 2026-11-04

## Question

Can offline verification of sealed 21004/21005-class evidence complete without
inventing measurements or weakening Governor gates, and how does that change the
research next-action vs “sidecar still unverified”?

## Measured

| Seed | Sidecar verification | Episode diagnostic | Failure class after verification |
|---|---|---|---|
| 21004 | `FAIL_NO_SIDECAR` | `INTEGRITY_PASS` | `unexpected_contact_with_episode_diagnostic_only` |
| 21005 | `FAIL_NO_SIDECAR` | `INTEGRITY_PASS` | `unexpected_contact_with_episode_diagnostic_only` |
| 21006 | structural / sealed | n/a | unchanged (task_success path) |
| 21007 | `PASS_RECORDED_AT_SEAL` (non-empty) | n/a | positive control |

`governor_weakened=false`. `invented_measurements=false`. Contact/force thresholds unchanged.

## Classification

**VALIDATED_LEARNING**: the open “complete sidecar verification” action is now a
measured negative for sealed 21004/21005 (absent sidecar), with diagnostic
integrity checked. Next capture must seal a non-empty `PROBE_MEASUREMENT_SIDECAR`
or wait for owner-gated 21008 replay — not gate weakening.

## Evidence

```powershell
py -3.10 -m unittest crates.verify.tests.test_probe_sidecar_verification_portfolio -v
py -3.10 -B crates/verify/probe_sidecar_verification_portfolio.py
```

Artifact: `docs/superpowers/evidence/rotated-probe-sidecar-verification-portfolio-20261104.json`
