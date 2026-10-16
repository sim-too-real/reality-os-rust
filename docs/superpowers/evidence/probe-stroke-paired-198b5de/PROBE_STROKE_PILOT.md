# Paired push-probe stroke pilot

Date: 2026-10-17

Base commit: `198b5de17b0066b08f7ceb359a3e21a9a812d94e`

Simulator: MuJoCo 3.7.0

Seed set: pilot seeds `9000..9020`, 60 scheduled trials per run

## Question

Does doubling the push-probe stroke from 8 mm to 16 mm improve useful friction evidence or expected-success availability while preserving the existing authority and motion safety gates?

## Paired result

Both runs completed the same 60 trial IDs with no infrastructure, invalid, or unaccounted trials. Both passed the hard safety gate and recorded zero false acceptances, unauthorized writes, geometry omissions, stale-authority continuations, and recovery trials. Both scored task success `3/17`, correct refusal `43/43`, and 11 false refusals. All three successes were pinch-grasp fixture trials; the UR5e push slice had zero successes among 11 expected-success trials in both runs.

| UR5e expected-success measure | 8 mm | 16 mm |
| --- | ---: | ---: |
| Trials | 11 | 11 |
| Probe executed | 9 | 9 |
| Friction interval narrowed | 0 | 0 |
| Follow-up push authorized | 0 | 0 |
| Task success | 0 | 0 |
| Mean object displacement during probe | 2.506 mm | 4.301 mm |
| Mean lateral object displacement | 0.074 mm | 0.228 mm |
| Runtime-divergence refusals | 1 | 2 |

The 16 mm run did not improve probe information or task success. It increased measured object movement, but also increased lateral movement and runtime divergence. One UR5e trial (`v5:development:menagerie_ur5e:push_probe_replan:9011:21`) changed from a bridge refusal at 8 mm to runtime divergence at 16 mm.

The deterministic `informative_probe_reauthorizes_followup_and_stops_on_lateral_drift` fixture also regressed at 16 mm: the object moved 10.787 mm, including 5.562 mm lateral drift, and the post-probe request had no narrowed friction interval. The next probe sweep was blocked by geometry, so the expected fresh push did not occur. The runner still recorded zero unauthorized writes. The same fixture passed with the original 8 mm stroke.

## Decision

Reject the 16 mm change and retain the 8 mm request. Object displacement alone did not provide a useful friction observation for this randomized UR5e slice. Do not lower the inference threshold from these data: the trial does not distinguish weak contact, contact compliance, and high support friction. The next availability design should use a typed, calibrated contact-force observation with explicit source and freshness provenance; simulator-derived contact forces must remain simulation-only and cannot qualify physical transfer.

## Reproduction

Run both commands from the repository root with MuJoCo 3.7.0 installed:

```powershell
$env:REALITYOS_REQUIRE_MUJOCO='1'
$env:MUJOCO_GL='disable'
cargo run -p realityos-verify --bin realityos-verify -- manipulation-benchmark --pilot --seeds 20 --out verify-out/manipulation-probe8-paired
```

For the 16 mm measurement, apply the single temporary candidate delta `probe_stroke_m: 0.008` to `probe_stroke_m: 0.016` in `crates/verify/src/manipulation_benchmark/execution.rs`, then run:

```powershell
$env:REALITYOS_REQUIRE_MUJOCO='1'
$env:MUJOCO_GL='disable'
cargo run -p realityos-verify --bin realityos-verify -- manipulation-benchmark --pilot --seeds 20 --out verify-out/manipulation-probe16-pilot
```

The committed `probe8mm/` and `probe16mm/` directories preserve both JSON reports, Markdown summaries, and all 60 raw trial records. The 16 mm report correctly marks its source as dirty because the one-line experiment was not a frozen candidate.
