# Probe freeze sidecar seal gate — 2026-11-04

## Question

Can new contact/transit-abort freeze packages be refused seal-complete when
`PROBE_MEASUREMENT_SIDECAR` is absent or empty, without inventing measurements
or weakening Governor thresholds?

## Measured

| Freeze class | Sidecar | `seal_allowed` |
|---|---|---|
| non-contact | n/a | true (`PASS_NOT_CONTACT_CLASS`) |
| contact/transit | absent | **false** (`FAIL_NO_SIDECAR`) |
| contact | empty `{}` | **false** (`FAIL_EMPTY_SIDECAR`) |
| contact | non-empty JSON | true (`PASS_SIDECAR_PRESENT`) |

`governor_weakened=false`. `invented_measurements=false`. Historical sealed
21004/21005 evidence is unchanged; this gate prevents repeating that failure
mode on **new** freezes.

## Classification

**SIGNIFICANT_ADVANCE** (process): fail-closed observation completeness for
contact-class freezes. Does not itself complete a new MuJoCo episode.

## Evidence

```powershell
py -3.10 -B -m unittest tools.test_assert_probe_freeze_sidecar_required -v
```
