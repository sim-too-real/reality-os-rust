# FREEZE replay observation acceptance enforcement — 2026-11-06

**Disposition:** fail-closed verification honesty; `SIMULATION_ONLY`; no new MuJoCo run.

## Hole

Probe-trace `FREEZE.json` packages set
`acceptance.replay_initial_observation_evidence_required: true`, but
`probe_pose_trace_replays_frozen_v4_ur5e_trial` only compared the first action
key to preflight. Replay could pass sidecar/trace checks without binding
`initial_observation_evidence` to the frozen preflight observation bytes.

## After

- Shared helper `enforce_freeze_replay_initial_observation_evidence` calls
  `FirstActionPreflightV1::verify_matches_episode` when the FREEZE flag is true
- The ignored probe-trace replay harness invokes that helper after the episode
- Offline regression on frozen `23020:17` artifacts proves the bind and proves
  stripped evidence fails closed

No frozen RESULT was rewritten. No MuJoCo trial was re-run.

## Reproduce

```powershell
cargo test -p realityos-verify --test manipulation_benchmark freeze_acceptance_requires_replay_initial_observation_evidence_offline -- --exact --nocapture
cargo test -p realityos-verify --test manipulation_benchmark first_action_preflight_verifier_rejects_legacy_and_binds_replay_observation -- --exact --nocapture
```
