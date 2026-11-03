# Seed 21005 diagnostic checkpoint

- The single held-out MuJoCo run stopped at 13.262 s on `unexpected_contact`; the abort hold was verified and unauthorized writes were 0.
- The object moved 0.252 mm, below the 1 mm motion gate. The audit had 6,630 wrench samples and included `link7`–`obj0`, but the runtime wrote no measurement records or probe contact-force trace.
- Root cause: the abort happened during probe transit, before the formal motion phase where pose/contact-force trace capture was enabled.
- The test preserved `EPISODE.json` and `.episode-diagnostic.json`; sidecar verification and onset analysis are unavailable. Seed 21005 was not replayed.
- Freeze verification passed. The freeze artifact itself was not committed before replay; the source change was subsequently committed as `3258af0`.

See `RESULT.json`, `FREEZE.json`, `FREEZE_VERIFY.json`, and `RUN_LOG.txt`.
