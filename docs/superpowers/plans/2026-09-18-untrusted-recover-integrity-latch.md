# Untrusted recover vs integrity abort — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Production `op=recover` still clears journaled ESTOP (campaign `--restart`) but cannot unlatch integrity `abort_latched` (`unknown_outcome`, `replayed command_id`) on a live ONLINE instance.

**Architecture:** Keep `EstopLatch`. Represent ESTOP (`engaged`) and integrity (`integrity_aborted`) as independent monotonic facts. `engage()` is recoverable ESTOP and must **not** clear or overwrite integrity. `latch_abort()` is the only production integrity transition. `clear_estop_requires_recovery_*` refuses integrity abort **before** `Plant::clear_estop` with `integrity_abort_requires_online_restart`. Do not add a crate. Do not change `IssuedCommand` / `OnlineWrite` / XL330 driver / metal proof schema.

> **Plan5 correction (do not implement the Task 1 `AbortClass` sketch as written).**
> `AbortClass { None, Estop, Integrity }` with `engage() -> Estop` can overwrite Integrity and re-open recover. That violates invariant A1 (`None < RecoverableEstop < IntegrityAbort`; transitions may move right, not left). Do not special-case the reason string `"unknown_outcome"`. Production `op=recover` remains an untrusted ESTOP-ack analog, not authenticated operator recovery.

**Status (plan6, do not rewrite history):**

| Token | Value |
|---|---|
| PRE-FIX BASELINE | `2f68a5d82fec5e7e2c0b78ca88fe65d8a8a4acfc` — intentionally vulnerable to the known recover-integrity bug; retained only as the frozen pre-fix physical baseline |
| PATCHED SOFTWARE | `709a0793fcf5e1706b059c27f1ed5a8dba0ca1f1` (and descendants) — candidate for post-fix physical authority evidence |
| LINUX PTY STATUS | passed on GitHub Actions Linux (`35322120250` test-metal: `xl330_pty_untrusted_recover_after_replay_does_not_write`, `xl330_pty_recover_after_integrity_then_watchdog_does_not_write`). Not metal. |
| REAL XL330 STATUS | not measured |
| AUTHENTICATED OPERATOR RECOVERY | NAMED_HOLE |

Task 1's `AbortClass` code block below is a **rejected sketch**, kept as historical plan text. Landed code uses independent facts.

**Tech Stack:** Rust 1.95.0 workspace; `realityos-governor` + `realityos-metal` PTY tests on Linux; existing `SimPlant` ONLINE helpers in `crates/governor/src/lib.rs`.

## Global Constraints

- Frozen metal SHA remains `2f68a5d82fec5e7e2c0b78ca88fe65d8a8a4acfc`. **Do not commit this work on that SHA or on `main`.** Use a worktree on a branch from `c82c78c` or from `2f68a5d` named for recover — never merge into the first XL330 run.
- Do not write `docs/metal_proof.json`.
- Do not merge `.worktrees/research-adversarial-evidence-v1` / crate `adversarial-evidence`.
- Campaign `as_autonomy … recover` after `--restart` must still clear ESTOP so the next hold can write (`scripts/metal-campaign.sh` around the `--restart` recover block).
- Recover after identity mismatch / disconnect / `bus_lost` must still refuse (`hardware_session_requires_online_restart` / `bus_lost`).
- Spent `command_id`s stay spent.
- HIL production already refuses `recover`; do not add recover to HIL.
- This Windows host can run `realityos-governor` tests. `realityos-metal` PTY tests are Linux-only.
- Unknown means unknown. Do not persist a fake operator identity.

### File map

| File | Responsibility |
|---|---|
| `crates/governor/src/latch.rs` | ESTOP vs Integrity kind; `clear()` policy |
| `crates/governor/src/governor.rs` | unknown_outcome uses `latch_abort`; recover keeps integrity latch |
| `crates/governor/src/lib.rs` | ONLINE tests (Windows-runnable) |
| `crates/metal/src/authority.rs` | no behavior change if governor is correct; optional comment |
| `crates/metal/tests/xl330_pty.rs` | IPC recover after replay (Linux) |
| `CLAIM_LEDGER.md` | honest claim: production recover is ESTOP ack, not integrity ack |

---

## Part 0 — Immediate product milestone (no code)

This part is **not** implemented by a coding agent on this Windows checkout.

- [ ] **Step 0.1: Detach the freeze SHA on a native Linux XL330 bench**

```bash
git fetch origin
git checkout --detach 2f68a5d82fec5e7e2c0b78ca88fe65d8a8a4acfc
git rev-parse HEAD
# must print 2f68a5d82fec5e7e2c0b78ca88fe65d8a8a4acfc
```

- [ ] **Step 0.2: Operator-test VIN cutoff (lost holding torque), isolate USB 5 V from XL330 VIN**

- [ ] **Step 0.3: Build and run the campaign**

```bash
cargo build -p realityos-metal --bins
sudo -E env REALITYOS_METAL_DEVICE=/dev/ttyUSB0 \
  REALITYOS_METAL_BIN=$PWD/target/debug \
  REALITYOS_METAL_CUTOFF_TESTED=1 \
  REALITYOS_METAL_CUTOFF_LIVE=1 \
  REALITYOS_METAL_UNPLUG_LIVE=1 \
  scripts/metal-campaign.sh
```

Expected: `docs/metal_proof.json` with `experiment_status=measured_success` and `software_commit_sha=2f68a5d82fec5e7e2c0b78ca88fe65d8a8a4acfc`.  
On this Windows agent: stop. Record incomplete. Do not invent the file.

Do **not** start Part 1 on the tree that will produce that proof.

---

### Task 1: Latch distinguishes ESTOP recover from integrity abort

**Files:**
- Modify: `crates/governor/src/latch.rs`
- Test: same file `#[cfg(test)]`

**Interfaces:**
- Consumes: existing `EstopLatch::{engage, latch_abort, clear}`
- Produces:
  - `EstopLatch::is_integrity_abort() -> bool`
  - `clear()` clears `engaged`; clears `abort_latched` only when the latch is ESTOP, not Integrity

- [ ] **Step 1: Write the failing tests**

Append to `crates/governor/src/latch.rs`:

```rust
#[cfg(test)]
mod tests {
    use super::EstopLatch;

    #[test]
    fn recover_clears_estop_including_abort() {
        let mut l = EstopLatch::default();
        l.engage("operator_estop");
        assert!(l.engaged);
        assert!(l.abort_latched);
        assert!(!l.is_integrity_abort());
        l.clear();
        assert!(!l.engaged);
        assert!(!l.abort_latched);
        assert!(l.reason.is_none());
    }

    #[test]
    fn recover_does_not_clear_integrity_abort() {
        let mut l = EstopLatch::default();
        l.latch_abort("unknown_outcome");
        assert!(!l.engaged);
        assert!(l.abort_latched);
        assert!(l.is_integrity_abort());
        l.clear();
        assert!(!l.engaged);
        assert!(l.abort_latched);
        assert_eq!(l.reason.as_deref(), Some("unknown_outcome"));
        assert!(l.is_integrity_abort());
    }

    #[test]
    fn replay_integrity_survives_clear() {
        let mut l = EstopLatch::default();
        l.latch_abort("replayed command_id");
        l.clear();
        assert!(l.abort_latched);
        assert!(l.is_integrity_abort());
    }
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cargo test -p realityos-governor --lib recover_does_not_clear_integrity_abort recover_clears_estop_including_abort replay_integrity_survives_clear -- --nocapture`

Expected: compile error `no method named is_integrity_abort` and/or `recover_does_not_clear_integrity_abort` FAIL because `clear()` currently sets `abort_latched = false`.

- [ ] **Step 3: Minimal implementation**

In `crates/governor/src/latch.rs`, replace `EstopLatch` with:

```rust
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
enum AbortClass {
    #[default]
    None,
    Estop,
    Integrity,
}

#[derive(Debug, Clone, Default)]
pub struct EstopLatch {
    pub engaged: bool,
    pub abort_latched: bool,
    pub reason: Option<String>,
    class: AbortClass,
}

impl EstopLatch {
    pub fn engage(&mut self, reason: impl Into<String>) {
        self.engaged = true;
        self.abort_latched = true;
        self.reason = Some(reason.into());
        self.class = AbortClass::Estop;
    }

    pub fn latch_abort(&mut self, reason: impl Into<String>) {
        self.abort_latched = true;
        self.reason = Some(reason.into());
        self.class = AbortClass::Integrity;
    }

    pub fn is_integrity_abort(&self) -> bool {
        self.class == AbortClass::Integrity && self.abort_latched
    }

    pub fn clear(&mut self) {
        self.engaged = false;
        if self.class == AbortClass::Integrity {
            return;
        }
        self.abort_latched = false;
        self.reason = None;
        self.class = AbortClass::None;
    }
}
```

Keep `SafeState` / `SafeStateLatch` unchanged.

- [ ] **Step 4: Run tests to verify they pass**

Run: `cargo test -p realityos-governor --lib recover_does_not_clear_integrity_abort recover_clears_estop_including_abort replay_integrity_survives_clear -- --nocapture`

Expected: PASS (3 tests).

- [ ] **Step 5: Commit**

```bash
git add crates/governor/src/latch.rs
git commit -m "fix(governor): recover clears ESTOP, not integrity abort"
```

---

### Task 2: ONLINE recover after unknown/replay cannot authorize a new id

**Files:**
- Modify: `crates/governor/src/governor.rs` (unknown_outcome must call `latch_abort`, not assign fields)
- Modify: `crates/governor/src/lib.rs` (tests next to existing ONLINE helpers)

**Interfaces:**
- Consumes: `EstopLatch::{latch_abort, clear, is_integrity_abort}` from Task 1
- Produces: `clear_estop_requires_recovery_now(true)` after ESTOP allows a later `authorize_issued`; after `unknown_outcome` or `replayed command_id` it does not

- [ ] **Step 1: Write the failing tests**

In `crates/governor/src/lib.rs` test module (same helpers as `online_gov` / `decide_hold`), add:

```rust
#[test]
fn recover_after_estop_allows_new_command() {
    use realityos_plant::ActionParams;
    let mut g = online_gov("rec-estop", online_identity(), b"online-cap-key", vec!["joint-0".into()]);
    let _ = g.engage_estop_now("operator_estop");
    assert!(g.abort_latched());
    let rec = g.clear_estop_requires_recovery_now(true);
    assert!(rec.ok, "{:?}", rec.violations);
    assert!(!g.abort_latched());
    let write = g.authorize_issued(decide_hold(1, 10.0)).expect("authorize after ESTOP recover");
    assert!(g.write_online(&write, &ActionParams::empty(), 10.0).ok);
    assert_eq!(g.plant().write_count(), 1);
}

#[test]
fn recover_after_replay_abort_does_not_allow_new_command() {
    use realityos_plant::ActionParams;
    let mut g = online_gov("rec-replay", online_identity(), b"online-cap-key", vec!["joint-0".into()]);
    let write = g.authorize_issued(decide_hold(1, 10.0)).unwrap();
    assert!(g.write_online(&write, &ActionParams::empty(), 10.0).ok);
    let again = g.write_online(&write, &ActionParams::empty(), 10.1);
    assert!(!again.ok);
    assert!(g.abort_latched());
    let rec = g.clear_estop_requires_recovery_now(true);
    assert!(
        rec.ok || rec.violations.iter().any(|v| v.contains("abort")),
        "recover may ack ESTOP but must not unlatch replay: {rec:?}"
    );
    assert!(g.abort_latched(), "integrity abort must survive recover");
    assert!(g.authorize_issued(decide_hold(2, 10.2)).is_err());
    assert_eq!(g.plant().write_count(), 1);
}

#[test]
fn recover_after_unknown_outcome_does_not_allow_new_command() {
    use realityos_plant::ActionParams;
    let mut g = online_gov("rec-unk", online_identity(), b"online-cap-key", vec!["joint-0".into()]);
    g.latch_abort_now("unknown_outcome");
    assert!(g.abort_latched());
    let rec = g.clear_estop_requires_recovery_now(true);
    let _ = rec;
    assert!(g.abort_latched());
    assert!(g.authorize_issued(decide_hold(1, 10.0)).is_err());
    let issued = decide_hold(1, 10.0);
    if let Ok(write) = g.authorize_issued(issued) {
        let t = g.write_online(&write, &ActionParams::empty(), 10.0);
        assert!(!t.ok, "integrity-latched instance must not write: {:?}", t.violations);
    }
    assert_eq!(g.plant().write_count(), 0);
}
```

If `abort_latched()` is already public (`governor.rs`), use it. If `latch_abort_now` is public, use it for the unknown-outcome stand-in. Do not construct `OnlineWrite` by hand.

- [ ] **Step 2: Run tests to verify they fail**

Run: `cargo test -p realityos-governor --lib recover_after_replay_abort_does_not_allow_new_command recover_after_unknown_outcome_does_not_allow_new_command recover_after_estop_allows_new_command -- --nocapture`

Expected: `recover_after_replay_abort_does_not_allow_new_command` FAIL — after recover, `abort_latched` is false and a new id writes (today's P0). ESTOP test may already pass once Task 1 landed.

- [ ] **Step 3: Minimal implementation**

In `crates/governor/src/governor.rs`, change the unknown-outcome branch to go through `latch_abort` so the class is Integrity:

Find:

```rust
        if result.outcome == realityos_kernel::CommandOutcome::Unknown {
            self.latch.abort_latched = true;
            self.latch.reason = Some("unknown_outcome".into());
```

Replace with:

```rust
        if result.outcome == realityos_kernel::CommandOutcome::Unknown {
            self.latch.latch_abort("unknown_outcome");
```

Find the `LATCHING_PREFIXES` block that assigns `abort_latched` / `reason` and replace with `self.latch.latch_abort(...)`.

Do **not** change `clear_estop_requires_recovery_at` beyond still calling `self.latch.clear()` — Task 1 already made `clear()` preserve integrity.

- [ ] **Step 4: Run tests to verify they pass**

Run: `cargo test -p realityos-governor --lib recover_after_ -- --nocapture`

Expected: PASS. Also run: `cargo test -p realityos-governor --all-targets`

Expected: PASS (do not weaken existing identity-dead recover tests; those still refuse because `hardware_session_dead`).

- [ ] **Step 5: Commit**

```bash
git add crates/governor/src/governor.rs crates/governor/src/lib.rs
git commit -m "test(governor): recover cannot unlatch replay or unknown_outcome"
```

---

### Task 3: Metal production IPC recover after replay (Linux)

**Files:**
- Modify: `crates/metal/tests/xl330_pty.rs`
- Do not change `MetalAuthority::recover` unless a governor-only fix is insufficient (it should be sufficient: recover already calls `clear_estop_requires_recovery_now(true)`).

**Interfaces:**
- Consumes: Task 1–2 latch policy
- Produces: PTY test `xl330_pty_untrusted_recover_after_replay_does_not_write` — new command_id after recover still refused; `physical_writes` unchanged

- [ ] **Step 1: Write the failing test**

In `crates/metal/tests/xl330_pty.rs`, next to `recover_req`, add (reuse existing `spawn_responder` / `MetalAuthority` helpers already in that file):

```rust
#[test]
fn xl330_pty_untrusted_recover_after_replay_does_not_write() {
    let _serial = pty_serial();
    let (_guard, tty) = spawn_responder();
    let root = pty_root("recover-replay");
    let cfg = bind_pty_cfg(&root, &tty);
    let mut auth = MetalAuthority::serve_for_test(cfg, &root).expect("serve");
    let first = auth.handle(MetalRequest::propose("rt-hold", "hold"));
    assert!(first.ok, "{first:?}");
    let writes = auth.physical_writes();
    let replay = auth.handle(MetalRequest::propose("rt-hold", "hold"));
    assert!(!replay.ok, "{replay:?}");
    let rec = auth.handle(recover_req("rt-rec"));
    let _ = rec;
    let second = auth.handle(MetalRequest::propose("rt-hold-2", "hold"));
    assert!(!second.ok, "new id must not write after untrusted recover: {second:?}");
    assert!(
        second.violations.iter().any(|v| v.contains("abort_latched")),
        "{second:?}"
    );
    assert_eq!(auth.physical_writes(), writes);
    let _ = std::fs::remove_dir_all(&root);
}
```

Adapt `MetalAuthority::serve_for_test` / `pty_root` to the names already in the file. Do not invent a new serve path.

- [ ] **Step 2: Run test (Linux only)**

Run: `cargo test -p realityos-metal --test xl330_pty xl330_pty_untrusted_recover_after_replay_does_not_write -- --test-threads=1 --nocapture`

Expected on current main **before** Task 1–2: FAIL (recover ok, second propose writes).  
Expected after Task 1–2: PASS.  
On Windows: compile skip / unix cfg — do not rewrite metal for Windows.

- [ ] **Step 3: Implementation**

If Task 1–2 already make this pass, no metal production change. If the test still writes, then `MetalAuthority::recover` must refuse when `governor.abort_latched()` is an integrity abort **before** calling `clear_estop_requires_recovery_now(true)`:

```rust
        if self.session.governor.abort_latched() {
            return self.refuse(
                "authorize",
                "refuse",
                vec!["abort_latched:integrity_recover_refused".into()],
            );
        }
```

Only add that if the governor-level `clear()` preservation is not visible through IPC (it should be: recover would return ok but the next propose still abort-latches). Prefer: recover may return ok for ESTOP fields while `abort_latched` stays true — the **write** is the invariant, not the recover JSON `ok` bit. The test above asserts the **second propose** fails and writes are unchanged.

- [ ] **Step 4: Run PTY suite subset**

Run: `cargo test -p realityos-metal --test xl330_pty -- --test-threads=1`

Expected: PASS. Campaign restart test `xl330_pty_campaign_restarts_are_live_after_identity_and_disconnect` must still pass (those recovers are after `hardware_session_dead` / new process ESTOP, not same-instance replay).

- [ ] **Step 5: Commit**

```bash
git add crates/metal/tests/xl330_pty.rs crates/metal/src/authority.rs
git commit -m "test(metal): untrusted recover after replay cannot write a new id"
```

---

### Task 4: Claim ledger honesty

**Files:**
- Modify: `CLAIM_LEDGER.md` (one row)

- [ ] **Step 1: Add the claim after the existing e-stop / recover rows**

```markdown
| Production metal `recover` is ESTOP operator-ack analog, not integrity-abort clear | Same-process | ACTIVE | `EstopLatch` ESTOP vs Integrity; `recover_after_replay_abort_does_not_allow_new_command` |
```

Do not claim process-isolation of recover. Do not claim metal MEASURED.

- [ ] **Step 2: No test** (ledger is documentation). Spec coverage: Gate A hole named closed at the latch, still unmeasured on UART.

- [ ] **Step 3: Commit**

```bash
git add CLAIM_LEDGER.md
git commit -m "docs: production recover is ESTOP ack, not integrity unlatch"
```

---

## Self-review

1. **Spec coverage:** Immediate metal milestone is Part 0 (no code). Recover ESTOP-vs-integrity is Tasks 1–4. Privileged perception, WorldState, HAL, PLACE are out of scope.
2. **Placeholders:** none. Helpers (`online_gov`, `decide_hold`, `recover_req`) exist in the cited files; metal serve helper names must be copied from `xl330_pty.rs` at execution time if they differ.
3. **Types (landed):** independent `engaged` + `integrity_aborted` facts; `EstopLatch::latch_abort` is the only integrity API; recover token `integrity_abort_requires_online_restart`. The Task 1 `AbortClass` sketch is **rejected** (plan5 A1) and is not the current design.

**Do not execute Part 1–4 on `main` or on `2f68a5d` without an explicit operator choice to work in a research worktree.**
