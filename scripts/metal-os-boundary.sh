#!/usr/bin/env bash
# Two-UID metal OS + IPC boundary. Not physical evidence.
# 1) dummy 0600 file: autonomy can traverse 0751 and device-open attempts are counted.
# 2) PTY Protocol 2.0 stand-in: autonomy can use ipc.sock only. ALLOW_PTY is
#    required; this script never writes metal_proof.json.

set -euo pipefail

AUTHORITY_USER="${REALITYOS_AUTHORITY_USER:-realityos-authority}"
AUTONOMY_USER="${REALITYOS_AUTONOMY_USER:-realityos-autonomy}"
IPC_GROUP="${REALITYOS_IPC_GROUP:-realityos-ipc}"
ROOT="${REALITYOS_METAL_ROOT:-/tmp/realityos-metal-boundary}"
IPC_ROOT="${REALITYOS_METAL_IPC_ROOT:-/tmp/realityos-metal-ipc}"
BIN_DIR="${REALITYOS_METAL_BIN:-}"
DUMMY="${REALITYOS_METAL_DUMMY_DEVICE:-/tmp/realityos-metal-dummy-tty}"

if [[ "$(id -u)" -ne 0 ]]; then
  echo "error: run as root to switch UIDs; root is not the tested actor" >&2
  exit 2
fi
if [[ -z "$BIN_DIR" ]]; then
  echo "error: set REALITYOS_METAL_BIN to the directory containing the metal binaries" >&2
  exit 2
fi
if [[ -z "$ROOT" || "$ROOT" == "/" || "$ROOT" == "/tmp" || "$ROOT" == "/var" ]]; then
  echo "error: refusing unexpected REALITYOS_METAL_ROOT=$ROOT" >&2
  exit 2
fi
if [[ -z "$IPC_ROOT" || "$IPC_ROOT" == "/" || "$IPC_ROOT" == "/tmp" || "$IPC_ROOT" == "/var" ]]; then
  echo "error: refusing unexpected REALITYOS_METAL_IPC_ROOT=$IPC_ROOT" >&2
  exit 2
fi
if [[ -c "$DUMMY" ]]; then
  echo "error: dummy path $DUMMY is a character device; refuse to treat it as a stand-in" >&2
  exit 2
fi

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
RESPONDER="$SCRIPT_DIR/../crates/metal/tests/xl330_responder.py"

STAGE="${REALITYOS_METAL_STAGE:-/tmp/realityos-metal-boundary-bin}"
rm -rf "$STAGE"
install -d -m 0755 "$STAGE"
for b in realityos-metal-propose realityos-metal-smoke; do
  if [[ ! -x "$BIN_DIR/$b" ]]; then
    echo "error: missing $BIN_DIR/$b" >&2
    exit 2
  fi
  install -m 0755 "$BIN_DIR/$b" "$STAGE/$b"
done
PROP="$STAGE/realityos-metal-propose"
SMOKE="$STAGE/realityos-metal-smoke"

rm -rf "$ROOT"
rm -f "$DUMMY"
install -m 0600 /dev/null "$DUMMY"
if [[ -c "$DUMMY" || ! -f "$DUMMY" ]]; then
  echo "error: dummy path must be a regular file, not a device node" >&2
  exit 2
fi

export REALITYOS_METAL_ROOT="$ROOT"
export REALITYOS_METAL_DEVICE="$DUMMY"
"$SCRIPT_DIR/metal-deploy.sh"

mode="$(stat -c '%a' "$ROOT")"
if [[ "$mode" != "751" ]]; then
  echo "error: metal root mode is $mode, expected 751" >&2
  exit 1
fi
if ! sudo -u "$AUTONOMY_USER" -- test -x "$ROOT"; then
  echo "error: autonomy cannot traverse 0751 metal root (ipc/os-probe would be unmeasured)" >&2
  exit 1
fi
if sudo -u "$AUTONOMY_USER" -- test -r "$ROOT/bus"; then
  echo "error: autonomy can read authority-owned bus/" >&2
  exit 1
fi

PROBE="$(sudo -u "$AUTONOMY_USER" -- env REALITYOS_METAL_DEVICE="$DUMMY" \
  "$PROP" --root "$ROOT" os-probe)"
echo "os-probe=$PROBE"

python3 - <<'PY' "$PROBE"
import json, sys
p = json.loads(sys.argv[1])
assert p["ran_as_root"] is False, p
assert int(p["direct_device_open_attempts"]) > 0, p
assert int(p["direct_device_open_successes"]) == 0, p
assert int(p.get("direct_device_write_successes") or 0) == 0, p
assert p["read_signing_key"] is False, p
assert p["write_signing_key"] is False, p
assert p["modify_journal"] is False, p
assert p["take_actuator_lock"] is False, p
print("metal-os-boundary-ok")
PY

chmod 0750 "$ROOT"
if sudo -u "$AUTONOMY_USER" -- test -x "$ROOT"; then
  echo "error: 0750 metal root is unexpectedly traversable by autonomy" >&2
  exit 1
fi
chmod 0751 "$ROOT"

if [[ ! -f "$RESPONDER" ]]; then
  echo "error: missing PTY responder $RESPONDER" >&2
  exit 2
fi

RESP_OUT="$(mktemp)"
python3 -u "$RESPONDER" >"$RESP_OUT" &
RESP_PID=$!
TTY=""
for _ in $(seq 1 100); do
  TTY="$(head -n 1 "$RESP_OUT" 2>/dev/null || true)"
  if [[ -n "$TTY" && -e "$TTY" ]]; then
    break
  fi
  sleep 0.05
done
if [[ -z "$TTY" || ! -e "$TTY" ]]; then
  echo "error: PTY responder did not publish a tty" >&2
  kill "$RESP_PID" 2>/dev/null || true
  exit 1
fi
if [[ "$(readlink -f "$TTY")" != /dev/pts/* ]]; then
  echo "error: responder tty $TTY is not a PTY; refuse" >&2
  kill "$RESP_PID" 2>/dev/null || true
  exit 2
fi

AUTH_PID=""
cleanup_ipc() {
  if [[ -n "${AUTH_PID:-}" ]]; then
    kill "$AUTH_PID" 2>/dev/null || true
    wait "$AUTH_PID" 2>/dev/null || true
  fi
  kill "$RESP_PID" 2>/dev/null || true
  wait "$RESP_PID" 2>/dev/null || true
  rm -f "$RESP_OUT"
}
trap cleanup_ipc EXIT

rm -rf "$IPC_ROOT"
export REALITYOS_METAL_ROOT="$IPC_ROOT"
export REALITYOS_METAL_DEVICE="$TTY"
"$SCRIPT_DIR/metal-deploy.sh"
chown "$AUTHORITY_USER:$AUTHORITY_USER" "$TTY" 2>/dev/null || true
chmod 0600 "$TTY" 2>/dev/null || true

as_authority() {
  sudo -u "$AUTHORITY_USER" -- env \
    REALITYOS_METAL_DEVICE="$TTY" \
    REALITYOS_METAL_ALLOW_PTY=1 \
    "$@"
}

as_authority "$SMOKE" --root "$IPC_ROOT" --device "$TTY" init
as_authority "$SMOKE" --root "$IPC_ROOT" --device "$TTY" probe
as_authority "$SMOKE" --root "$IPC_ROOT" bind-measured

rm -f "$IPC_ROOT/ipc.sock"
as_authority "$SMOKE" --root "$IPC_ROOT" --first-online serve \
  >"$IPC_ROOT/authority.out" 2>"$IPC_ROOT/authority.err" &
AUTH_PID=$!
for _ in $(seq 1 400); do
  if [[ -S "$IPC_ROOT/ipc.sock" ]]; then
    break
  fi
  sleep 0.05
done
if [[ ! -S "$IPC_ROOT/ipc.sock" ]]; then
  echo "error: ipc.sock did not appear on PTY stand-in" >&2
  cat "$IPC_ROOT/authority.err" >&2 || true
  exit 1
fi
chmod 0660 "$IPC_ROOT/ipc.sock"
chgrp "$IPC_GROUP" "$IPC_ROOT/ipc.sock"
chmod 0600 "$TTY" 2>/dev/null || true

IPC_PROBE="$(sudo -u "$AUTONOMY_USER" -- env \
  REALITYOS_METAL_DEVICE="$TTY" \
  METAL_AUTHORITY_PID="$AUTH_PID" \
  "$PROP" --root "$IPC_ROOT" --authority-pid "$AUTH_PID" os-probe)"
echo "os-probe-ipc=$IPC_PROBE"

HOLD="$(sudo -u "$AUTONOMY_USER" -- env REALITYOS_METAL_DEVICE="$TTY" \
  "$PROP" --root "$IPC_ROOT" --id pty-boundary-hold --verb hold propose)"
echo "pty-hold=$HOLD"

python3 - <<'PY' "$IPC_PROBE" "$HOLD"
import json, sys
p = json.loads(sys.argv[1])
h = json.loads(sys.argv[2])
assert p["ran_as_root"] is False, p
assert p["ipc_connect"] is True, p
assert p["status_reached_authority"] is True, p
assert int(p["direct_device_open_attempts"]) > 0, p
assert int(p["direct_device_open_successes"]) == 0, p
assert p["read_signing_key"] is False, p
assert p["write_signing_key"] is False, p
assert p["modify_journal"] is False, p
assert p["take_actuator_lock"] is False, p
assert h.get("ok") is True, h
assert h.get("clock") == "OsMonotonicClock", h
assert h.get("metal") is False, h
print("metal-os-ipc-ok")
PY

if [[ -f "$ROOT/metal_proof.json" || -f "$IPC_ROOT/metal_proof.json" || -f "$IPC_ROOT/proof_meta.json" ]]; then
  echo "error: OS-boundary test must not write a metal proof" >&2
  exit 1
fi

echo "metal OS boundary finished (not physical evidence)"
