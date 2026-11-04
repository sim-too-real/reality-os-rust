# Seed 21008 prepare-only checkpoint — 2026-11-04

**Status:** PREPARE_ONLY complete. **Not frozen for replay.** No simulator motion run.

## Artifacts

| File | Role |
|---|---|
| `TRIAL_INPUT.json` | Resolved held-out trial seed 21008 index 0 |
| `PREFLIGHT.json` | Zero-step first-action preflight (`probe:+x`) |

SHA-256 (bytes as prepared):

- trial: `C310B1498E516D62395A2E3654D21682015B09F3CBFD8D65767C6C14E6BB2C0C`
- preflight: `6ED81421FFA9AAE20B6D6D125B6486D7312196C7BB675A7C3BC373E06DADA4B0`

## Freeze status

`FREEZE.json` + `FREEZE_VERIFY.json` written (`PASS_WORKING_TREE`).
`replay_authorized=false` until freeze is **git-committed**.

Source-hash deltas vs 21007 freeze (expected post-21007 commits):
`execution.rs`, `push_probe.rs`. Model hashes match.

## Remaining before any replay

1. Commit the 21008 evidence freeze (and related verify tooling if desired) on `reality-os-rust`.
2. Re-run freeze verify against the committed tree; set `replay_authorized` only then.
3. Clear `REALITYOS_ROTATED_PROBE_PREPARE_ONLY` and run the command in `FREEZE.json` once.
4. Do not weaken Governor expected-contact policy.

`SIMULATION_ONLY`; `NOT_ADMITTED`; unauthorized writes expected 0.
