#!/usr/bin/env bash
# Exercise the metal campaign script on a Protocol 2.0 PTY stand-in.
# Not physical evidence. Must not install docs/metal_proof.json.

set -euo pipefail

if [[ "$(id -u)" -ne 0 ]]; then
  echo "error: run as root to switch UIDs" >&2
  exit 2
fi

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$SCRIPT_DIR/.." && pwd)"
"$SCRIPT_DIR/metal-unix-mode.sh"
"$SCRIPT_DIR/metal-sensor-drop.sh"
"$SCRIPT_DIR/metal-online-journal.sh"
"$SCRIPT_DIR/metal-nudge-action.sh"
"$SCRIPT_DIR/metal-campaign-case-stdout.sh"
BIN_DIR="${REALITYOS_METAL_BIN:-}"
if [[ ! -x "${BIN_DIR:-}/realityos-metal-smoke" && ! -x "$REPO/target/debug/realityos-metal-smoke" && ! -x "$REPO/target/release/realityos-metal-smoke" ]]; then
  echo "error: build metal bins first (REALITYOS_METAL_BIN or $REPO/target/debug)" >&2
  exit 2
fi

RESPONDER="$REPO/crates/metal/tests/xl330_responder.py"
if [[ ! -f "$RESPONDER" ]]; then
  echo "error: missing $RESPONDER" >&2
  exit 2
fi

ROOT="${REALITYOS_METAL_ROOT:-/tmp/realityos-metal-pty-seq}"
if [[ -f "$REPO/docs/metal_proof.json" ]]; then
  echo "error: docs/metal_proof.json already exists; refuse to run a PTY sequence that could be confused with metal evidence" >&2
  exit 2
fi

RESP_PID=""
stop_responder() {
  if [[ -n "${RESP_PID:-}" ]]; then
    kill "$RESP_PID" 2>/dev/null || true
    wait "$RESP_PID" 2>/dev/null || true
    RESP_PID=""
  fi
}
cleanup() {
  stop_responder
}
trap cleanup EXIT

# Real XL330 does not teleport present, and Moving stays 0 until velocity
# exceeds Moving Threshold. The old campaign treated the first Moving=0 as
# settled and would record nudge delta=0 on a live horn.
start_responder() {
  local out="$1"
  shift
  stop_responder
  : >"$out"
  # Remaining args are extra responder env (KEY=VAL).
  env REALITYOS_METAL_PTY_DELAY_MOTION=1 "$@" python3 "$RESPONDER" \
    >"$out" 2>"${out%.out}.err" &
  RESP_PID=$!
  local tty=""
  local _
  for _ in $(seq 1 50); do
    tty="$(tr -d '[:space:]' <"$out" 2>/dev/null || true)"
    if [[ "$tty" == /dev/pts/* ]]; then
      echo "$tty"
      return 0
    fi
    sleep 0.1
  done
  echo "error: PTY responder did not print a slave tty" >&2
  cat "${out%.out}.err" >&2 || true
  return 1
}

run_campaign() {
  local root="$1"
  local tty="$2"
  "$SCRIPT_DIR/metal-kill-serve.sh" "$root" || true
  # Campaign must install into $REPO/docs, not $PWD/docs, and must
  # find binaries in the script's repo when $PWD/target/debug is
  # empty. A bench run from $HOME used to miss both.
  (cd /tmp && sudo -E env \
    PATH="$PATH" \
    REALITYOS_METAL_BIN="$PWD/target/debug" \
    REALITYOS_METAL_DEVICE="$tty" \
    REALITYOS_METAL_ROOT="$root" \
    REALITYOS_METAL_PTY_SEQUENCE=1 \
    REALITYOS_METAL_ALLOW_PTY=1 \
    REALITYOS_METAL_CUTOFF_TESTED=0 \
    REALITYOS_METAL_CUTOFF_LIVE=0 \
    REALITYOS_METAL_UNPLUG_LIVE=0 \
    ${REALITYOS_METAL_STAGE:+REALITYOS_METAL_STAGE="$REALITYOS_METAL_STAGE"} \
    "$SCRIPT_DIR/metal-campaign.sh")
  if [[ -f "$REPO/docs/metal_proof.json" ]]; then
    echo "error: PTY sequence installed docs/metal_proof.json" >&2
    rm -f "$REPO/docs/metal_proof.json" "$REPO/docs/METAL_PROOF_REPORT.md"
    exit 1
  fi
  python3 - "$root/os_metal_cases.json" <<'PY'
import json, sys
path = sys.argv[1]
cases = json.load(open(path))
by = {c.get("name"): c for c in cases}
need = [
    "crash_restart_before_prepare",
    "crash_restart_after_prepare_before_write",
    "crash_restart_during_write",
    "crash_restart_after_serial_tx_before_status",
    "crash_restart_after_write_before_ack",
    "crash_restart_after_ack",
]
missing = [n for n in need if n not in by]
if missing:
    sys.exit("error: missing crash case records: %s" % (missing,))
for n in need:
    rec = by[n]
    tx = int(rec.get("serial_tx_delta") or 0)
    pw = int(rec.get("physical_writes_delta") or 0)
    if tx != 0 or pw != 0:
        sys.exit("error: %s retransmission serial_tx_delta=%s physical_writes_delta=%s" % (n, tx, pw))
print("pty-crash-matrix-ok")
PY
  leftover="$(ps -eo pid,cmd | grep '[r]ealityos-metal-smoke --root' || true)"
  if [[ -n "$leftover" ]]; then
    echo "error: leftover realityos-metal-smoke after PTY campaign root=$root" >&2
    printf '%s\n' "$leftover" >&2
    exit 1
  fi
}

require_nudge_action() {
  local root="$1"
  local want="$2"
  python3 - "$root/os_metal_cases.json" "$want" <<'PY'
import json, sys
path, want = sys.argv[1], sys.argv[2]
cases = json.load(open(path))
nudge = next((c for c in cases if c.get("name") == "valid_nudge"), None)
if not nudge:
    sys.exit("error: missing valid_nudge case in %s" % (path,))
got = nudge.get("proposal") or ""
if got != "verb=drive action=%s" % (want,):
    sys.exit("error: valid_nudge proposal %r wants action=%s (Wizard-tight cage must flip sign before propose)" % (got, want))
print("pty-nudge-action-ok action=%s" % (want,))
PY
}

require_nudge_wrap_park() {
  local root="$1"
  local want_present="$2"
  local want_min="$3"
  local want_max="$4"
  python3 - "$root/os_metal_cases.json" "$want_present" "$want_min" "$want_max" <<'PY'
import json, re, sys
cases = json.load(open(sys.argv[1]))
want_present, want_min, want_max = map(int, sys.argv[2:5])
nudge = next((c for c in cases if c.get("name") == "valid_nudge"), None)
if not nudge:
    sys.exit("error: missing valid_nudge case in %s" % (sys.argv[1],))
if nudge.get("experiment_min") != want_min or nudge.get("experiment_max") != want_max:
    sys.exit(
        "error: wrap campaign must recenter cage to %s..%s, got %s..%s"
        % (want_min, want_max, nudge.get("experiment_min"), nudge.get("experiment_max"))
    )
motion = nudge.get("observed_motion") or ""
m = re.search(r"present (-?\d+)->", motion)
if not m:
    sys.exit("error: valid_nudge missing present-before in %r" % (motion,))
before = int(m.group(1))
if abs(before - want_present) > 4:
    sys.exit(
        "error: wrap park must be %s±4 before valid_nudge, got %s (%s)"
        % (want_present, before, motion)
    )
print(
    "pty-wrap-park-ok present_before=%s cage=%s..%s"
    % (before, want_min, want_max)
)
PY
}

# valid_nudge records the serve cage and pre-write present. A mid-range
# +16 torque wrap must recenter to 2016..2112 and park at 2064; keeping
# 2000..2096 makes the campaign picker fail slack after hold.
require_nudge_torque_wrap_park() {
  local root="$1"
  python3 - "$root/os_metal_cases.json" <<'PY'
import json, re, sys
cases = json.load(open(sys.argv[1]))
nudge = next((c for c in cases if c.get("name") == "valid_nudge"), None)
if not nudge:
    sys.exit("error: missing valid_nudge case in %s" % (sys.argv[1],))
if nudge.get("experiment_min") != 2016 or nudge.get("experiment_max") != 2112:
    sys.exit(
        "error: torque-wrap campaign must recenter cage to 2016..2112, got %s..%s"
        % (nudge.get("experiment_min"), nudge.get("experiment_max"))
    )
motion = nudge.get("observed_motion") or ""
m = re.search(r"present (-?\d+)->", motion)
if not m:
    sys.exit("error: valid_nudge missing present-before in %r" % (motion,))
before = int(m.group(1))
if abs(before - 2064) > 4:
    sys.exit(
        "error: torque-wrap park must be 2064±4 before valid_nudge, got %s (%s)"
        % (before, motion)
    )
print("pty-torque-wrap-park-ok present_before=%s cage=2016..2112" % (before,))
PY
}

# Robotis Present reset on torque-on. Mid-range +16 wrap used to keep
# cage 2000..2096; after hold at 2064 the picker slack misses 2096.
# Do not set this on AT_MAX: 2064 is past leftover max 2048 and setup refuses.
TTY="$(start_responder "$ROOT.responder.out" REALITYOS_METAL_PTY_TORQUE_JUMP_PRESENT=1)"
run_campaign "$ROOT" "$TTY"
require_nudge_action "$ROOT" "0.2"
require_nudge_torque_wrap_park "$ROOT"

# Wizard leftover max==present: hardcoded +0.2 is experiment_cage_violation
# and abort-latches ONLINE. The campaign must pick -0.2 before propose.
AT_MAX_ROOT="${ROOT}-at-max"
TTY="$(start_responder "$AT_MAX_ROOT.responder.out" REALITYOS_METAL_PTY_AT_MAX=1)"
run_campaign "$AT_MAX_ROOT" "$TTY"
require_nudge_action "$AT_MAX_ROOT" "-0.2"

# Hand-turned first contact (docs: Present 5000 or −16). Driver open tests
# already wrap; the campaign script's hold / inbound picker / crash-replay
# used to run only at mid-range 2048 / AT_MAX. Do not combine with
# TORQUE_JUMP or AT_MAX: those leftover windows refuse a wrap park.
WRAP_ROOT="${ROOT}-multiturn"
TTY="$(start_responder "$WRAP_ROOT.responder.out" REALITYOS_METAL_PTY_PRESENT_MULTITURN=1)"
run_campaign "$WRAP_ROOT" "$TTY"
require_nudge_action "$WRAP_ROOT" "0.2"
require_nudge_wrap_park "$WRAP_ROOT" 904 856 952

NEG_ROOT="${ROOT}-negative"
TTY="$(start_responder "$NEG_ROOT.responder.out" REALITYOS_METAL_PTY_PRESENT_NEGATIVE=1)"
run_campaign "$NEG_ROOT" "$TTY"
require_nudge_action "$NEG_ROOT" "-0.2"
require_nudge_wrap_park "$NEG_ROOT" 4080 4032 4095

# Official e-Manual case 1: leftover Wizard Extended + hand-turned
# wraps on the Position Mode write. Force-disable the reboot wrap so
# this pass cannot hide behind INST_REBOOT. Same park as the reboot
# pass; crash-replay must still pick +0.2 inside 856..952.
MODE_WRAP_ROOT="${ROOT}-mode-wrap"
TTY="$(start_responder "$MODE_WRAP_ROOT.responder.out" \
  REALITYOS_METAL_PTY_EXTENDED=1 \
  REALITYOS_METAL_PTY_PRESENT_MULTITURN=1 \
  REALITYOS_METAL_PTY_NO_REBOOT_PRESENT_WRAP=1)"
run_campaign "$MODE_WRAP_ROOT" "$TTY"
require_nudge_action "$MODE_WRAP_ROOT" "0.2"
require_nudge_wrap_park "$MODE_WRAP_ROOT" 904 856 952

leftover="$(ps -eo pid,cmd | grep '[r]ealityos-metal-smoke --root' || true)"
if [[ -n "$leftover" ]]; then
  echo "error: leftover realityos-metal-smoke after metal-pty-sequence" >&2
  printf '%s\n' "$leftover" >&2
  exit 1
fi
echo "metal-pty-sequence finished (not physical evidence)"
