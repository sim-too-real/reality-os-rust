# HIL / deployment topology

Evidence for process separation and exclusive virtual I/O. Not metal.

## Deployment architecture

```text
hil-untrusted (Process A)
  VLA / LLM / ROS / planner / hostile
  may send: HilRequest (verb, optional action)
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

## Physical testing

Not justified. Blocker: no independent STO/SS1 / safety PLC, and exclusive
ownership is not proven against root or same-UID fd escape.
