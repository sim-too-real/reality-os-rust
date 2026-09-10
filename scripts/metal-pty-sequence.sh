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
"$SCRIPT_DIR/metal-kill-serve.sh" "$ROOT" || true
if [[ -f "$REPO/docs/metal_proof.json" ]]; then
  echo "error: docs/metal_proof.json already exists; refuse to run a PTY sequence that could be confused with metal evidence" >&2
  exit 2
fi

# Real XL330 does not teleport present, and Moving stays 0 until velocity
# exceeds Moving Threshold. The old campaign treated the first Moving=0 as
# settled and would record nudge delta=0 on a live horn.
export REALITYOS_METAL_PTY_DELAY_MOTION=1
python3 "$RESPONDER" >"$ROOT.responder.out" 2>"$ROOT.responder.err" &
RESP_PID=$!
cleanup() {
  kill "$RESP_PID" 2>/dev/null || true
  wait "$RESP_PID" 2>/dev/null || true
}
trap cleanup EXIT

TTY=""
for _ in $(seq 1 50); do
  TTY="$(tr -d '[:space:]' <"$ROOT.responder.out" 2>/dev/null || true)"
  if [[ "$TTY" == /dev/pts/* ]]; then
    break
  fi
  sleep 0.1
done
if [[ "$TTY" != /dev/pts/* ]]; then
  echo "error: PTY responder did not print a slave tty" >&2
  cat "$ROOT.responder.err" >&2 || true
  exit 1
fi

# Campaign must install into $REPO/docs, not $PWD/docs, and must
# find binaries in the script's repo when $PWD/target/debug is
# empty. A bench run from $HOME used to miss both.
(cd /tmp && sudo -E env \
  PATH="$PATH" \
  REALITYOS_METAL_BIN="$PWD/target/debug" \
  REALITYOS_METAL_DEVICE="$TTY" \
  REALITYOS_METAL_ROOT="$ROOT" \
  REALITYOS_METAL_PTY_SEQUENCE=1 \
  REALITYOS_METAL_ALLOW_PTY=1 \
  REALITYOS_METAL_CUTOFF_TESTED=0 \
  ${REALITYOS_METAL_STAGE:+REALITYOS_METAL_STAGE="$REALITYOS_METAL_STAGE"} \
  "$SCRIPT_DIR/metal-campaign.sh")

if [[ -f "$REPO/docs/metal_proof.json" ]]; then
  echo "error: PTY sequence installed docs/metal_proof.json" >&2
  rm -f "$REPO/docs/metal_proof.json" "$REPO/docs/METAL_PROOF_REPORT.md"
  exit 1
fi

echo "metal-pty-sequence finished (not physical evidence)"
