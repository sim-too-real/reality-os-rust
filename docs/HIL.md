# HIL / deployment topology

Evidence for process separation and exclusive virtual I/O. Not metal.

## Deployment architecture

```text
hil-untrusted (Process A)
  VLA / LLM / ROS / planner / hostile
  may send: ProductionProposal (verb, optional action, command_id, proposer)
  must never send: now_s / write_now_s / safety TTL as authority
  must never receive: signing key, OnlineWrite, Plant, bus fd, journal
        │ Unix socket IPC (jsonl)
        ▼
hil-authority (Process B)
  RealityOs::decide
  RuntimeGovernor<OnlineLocked>
  HardwareBackedPlant<VirtualSerialPort>
  consume journal + seal
        │ exclusive lock + mode-000 write log
        ▼
virtual actuator endpoint   (file, not a robot)
```

One authority process is enough. Do not split the kernel into microservices.

## OS single-writer guarantee (exact)

On the HIL host:

1. Authority creates `bus/actuator.lock` (`0600`) and takes `flock(LOCK_EX)`.
2. Authority opens `bus/actuator.log` and sets its mode to `000`.
3. Directory is `0700`.

**What this enforces**

* A second ordinary process that only `open`/`flock`/`write` (no `chmod`)
  cannot take the exclusive lock and cannot open the write log (`EACCES`).
* Measured in `crates/hil/tests/campaign.rs` via `hil-untrusted attack-open`.
* After crash, the lock holder may `chmod` the log back to `0600` to reopen,
  then sets `000` again. That is restart of the owner, not the hostile path.

**What this does not enforce**

* Root can open anything.
* A same-UID process that `chmod`s the log, or writes `/proc/<pid>/fd/N`,
  can recover the inode.
* This is not udev, systemd, or a dedicated service user. Those remain
  operational deployment choices on a real machine.

## Two-UID deployment (measurable)

`scripts/hil-os-users-ci.sh` creates `realityos-authority`, `realityos-autonomy`,
and group `realityos-ipc`, then `scripts/hil-os-users-test.sh` runs attacks
**as the autonomy UID**.

* Socket: authority process binds, then sets mode `0660` explicitly (not umask).
  Group is `realityos-ipc`. Autonomy may connect; it cannot open `bus/`.
* `signing.key` is authority `0600`. Filesystem storage is **not** a TPM/HSM.
  `--production` serve refuses to start without that file and ignores
  `hil_fault` / disconnect / shutdown over IPC.
* Cargo unit tests do **not** prove this. CI job `os-users` does.

Root prepares the users, then is outside the threat model.

## Journal attacks (authority process down)

Detected on reopen (`first_online=false`):

* journal deletion (seal remains)
* seal deletion (journal remains)
* both files gone
* truncation / incomplete last record
* new valid-looking JSON that does not extend the seal
* corrupt final record
* older journal against a newer seal (`JournalRollback`)

Not detected (named, not WORM):

* paired restore of an older matching journal+seal (looks like that earlier tip)
* dual-delete plus `first_online=true` (looks like first boot)

Do not call this journal authenticated or WORM. Stronger storage is an
external sealed medium the autonomy host cannot rewrite.

## Frozen kernel vs HIL crates

`crates/vport` and `crates/hil` are **outside** the frozen authority kernel.
They consume public APIs. `hil-faults` on `realityos-plant` is a test-only
`REALITYOS_HIL_CRASH` exit hook, compiled out without the feature.

## Reconnect / hot-swap

If the physical identity changes after ONLINE startup, or the device
disconnects:

* the runtime instance enters FAULT / ABORT
* further writes are zero
* operator `clear_estop` cannot restore the binding
* recovery is a complete ONLINE restart (`new_online`)

Reconnect to a different device under the previously authorized instance is
refused. Root is outside this threat model.

## OS users

See **Two-UID deployment** above. Do not treat a cargo unit test as
multi-user success.

## Physical testing

Not justified. Blocker: no independent STO/SS1 / safety PLC, and exclusive
ownership is not proven against root or same-UID fd escape.

This is semantic execution authority, not independent physical energy safety.

## Measured proof (from `docs/hil_proof.json`)

Headline is only the numbers the campaign **derived** from case write deltas
(`realityos.hil_proof/2`). Re-run: `cargo test -p realityos-hil --test campaign`.

`unauthorized_write` is `!expected_authorized && write_delta > 0`. Aggregates
are recomputed and checked before the JSON is written.
