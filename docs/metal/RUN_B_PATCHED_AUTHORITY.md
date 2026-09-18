# Run B — patched authority candidate

**Do not fabricate results. Do not hand-write `docs/metal_proof.json`.**

## Identity

| Field | Value |
|---|---|
| Role | PATCHED SOFTWARE |
| Checkout | `709a0793fcf5e1706b059c27f1ed5a8dba0ca1f1` or a descendant that still contains the independent-facts latch |
| Honest label | first serious post-fix physical authority **candidate** (only if the campaign plus hostile cases pass) |
| Pre-fix SHA | `2f68a5d82fec5e7e2c0b78ca88fe65d8a8a4acfc` remains the immutable baseline |

## Host

Same native Linux XL330 bench as Run A.

## Campaign

Same `scripts/metal-campaign.sh` invocation as Run A, on the patched SHA.

Then add hostile cases on the **live** ONLINE instance (autonomy IPC, not a new process):

```text
replay X → op=recover → fresh id Y
unknown outcome → op=recover → fresh id
integrity abort → later ESTOP/watchdog → op=recover → fresh id
```

## Required result (hostile cases)

```text
recover refused
integrity remains latched
zero unauthorized physical TX
zero unauthorized ACK
zero new physical writes
```

Ordinary campaign `--restart` ESTOP recover must still work.

## Status here

`REAL XL330 STATUS: not measured`. Linux PTY composition tests are software, not metal.
