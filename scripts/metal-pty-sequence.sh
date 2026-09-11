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
"$SCRIPT_DIR/metal-nudge-action.sh"
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

TTY="$(start_responder "$ROOT.responder.out")"
run_campaign "$ROOT" "$TTY"
require_nudge_action "$ROOT" "0.2"

# Wizard leftover max==present: hardcoded +0.2 is experiment_cage_violation
# and abort-latches ONLINE. The campaign must pick -0.2 before propose.
AT_MAX_ROOT="${ROOT}-at-max"
TTY="$(start_responder "$AT_MAX_ROOT.responder.out" REALITYOS_METAL_PTY_AT_MAX=1)"
run_campaign "$AT_MAX_ROOT" "$TTY"
require_nudge_action "$AT_MAX_ROOT" "-0.2"

echo "metal-pty-sequence finished (not physical evidence)"
