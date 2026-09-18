# Run A — historical pre-fix physical baseline

**Do not fabricate results. Do not hand-write `docs/metal_proof.json`.**

## Identity

| Field | Value |
|---|---|
| Role | PRE-FIX BASELINE |
| Checkout | `2f68a5d82fec5e7e2c0b78ca88fe65d8a8a4acfc` |
| Honest label | `pre-fix frozen physical baseline` |
| Vulnerability | This SHA is **intentionally vulnerable** to the known recover-integrity bug and is retained **only** as the frozen pre-fix physical baseline. Do not rewrite it. |
| Authority candidate? | No. Characterization only. |

## Host

Native Linux with real XL330 on USB-UART. This Windows agent cannot run it.

## Checkout

```bash
git fetch origin
git checkout --detach 2f68a5d82fec5e7e2c0b78ca88fe65d8a8a4acfc
git rev-parse HEAD
# must print 2f68a5d82fec5e7e2c0b78ca88fe65d8a8a4acfc
git cat-file -t 2f68a5d82fec5e7e2c0b78ca88fe65d8a8a4acfc
```

## Campaign (unchanged)

Operator-test VIN cutoff (lost holding torque). Isolate USB 5 V from XL330 VIN.

```bash
cargo build -p realityos-metal --bins
sudo -E env REALITYOS_METAL_DEVICE=/dev/ttyUSB0 \
  REALITYOS_METAL_BIN=$PWD/target/debug \
  REALITYOS_METAL_CUTOFF_TESTED=1 \
  REALITYOS_METAL_CUTOFF_LIVE=1 \
  REALITYOS_METAL_UNPLUG_LIVE=1 \
  scripts/metal-campaign.sh
```

## Preserve

- full raw logs
- serial TX/ACK counters
- journal artifacts
- USB unplug evidence
- VIN cutoff evidence
- crash/restart traces
- failures
- generated proof **only if** the existing reporter legitimately produces `docs/metal_proof.json`

## Status here

`REAL XL330 STATUS: not measured` on this Windows checkout. `MEASURED_INCOMPLETE`.
