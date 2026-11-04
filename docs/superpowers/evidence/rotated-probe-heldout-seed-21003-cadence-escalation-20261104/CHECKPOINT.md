# 21003 cadence-escalation paired replay — 2026-11-04

## Result

**SUCCESS** (simulation-only, NOT_ADMITTED). Fresh paired MuJoCo run under
`PreferHigherLoadFamilyOnIncompatible` completed authorized `probe:+x`,
`probe:+x`, then `push:+x`. Unauthorized writes: 0. Force/contact gates
unchanged (1 mm / 0.5 N).

## Causal evidence vs sealed baseline

| | Sealed 21003 | Cadence escalation |
|---|---|---|
| Outcome | `unexpected_refusal` | `success` |
| Actions | 2× probe | 2× probe + push:+x |
| After second probe | prior-incompatible; belief stuck `[0.259,0.337]` | escalate; posterior `[0.417,0.491]` |
| Push | not justified | executed |

Cadence audit (episode):
`successive_probe_cadence=escalated_higher_load_family;quarantined_first=[0.259…,0.337…];posterior=[0.417…,0.491…]`

## Classification

**SIGNIFICANT ADVANCE** for this frozen seed geometry under the wired cadence
policy. Still simulation-only; does not transfer to physical hardware or
weaken Governor thresholds.

## Next

Transfer check on an independent seed / held-out requirement; continue
owner-blocked commercial/physical queues unchanged.
