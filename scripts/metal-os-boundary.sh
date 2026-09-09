#!/usr/bin/env bash
# Two-UID metal OS boundary. Not physical evidence.
# Records that autonomy can traverse 0751 to attempt a device open, and that
# those attempts fail. Does not start ONLINE, does not write metal_proof.json.

set -euo pipefail

AUTHORITY_USER="${REALITYOS_AUTHORITY_USER:-realityos-authority}"
AUTONOMY_USER="${REALITYOS_AUTONOMY_USER:-realityos-autonomy}"
IPC_GROUP="${REALITYOS_IPC_GROUP:-realityos-ipc}"
ROOT="${REALITYOS_METAL_ROOT:-/tmp/realityos-metal-boundary}"
BIN_DIR="${REALITYOS_METAL_BIN:-}"
DUMMY="${REALITYOS_METAL_DUMMY_DEVICE:-/tmp/realityos-metal-dummy-tty}"

if [[ "$(id -u)" -ne 0 ]]; then
  echo "error: run as root to switch UIDs; root is not the tested actor" >&2
  exit 2
fi
if [[ -z "$BIN_DIR" ]]; then
  echo "error: set REALITYOS_METAL_BIN to the directory containing realityos-metal-propose" >&2
  exit 2
fi
if [[ -z "$ROOT" || "$ROOT" == "/" || "$ROOT" == "/tmp" || "$ROOT" == "/var" ]]; then
  echo "error: refusing unexpected REALITYOS_METAL_ROOT=$ROOT" >&2
  exit 2
fi
if [[ -c "$DUMMY" ]]; then
  echo "error: dummy path $DUMMY is a character device; refuse to treat it as a stand-in" >&2
  exit 2
fi

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

STAGE="${REALITYOS_METAL_STAGE:-/tmp/realityos-metal-boundary-bin}"
rm -rf "$STAGE"
install -d -m 0755 "$STAGE"
if [[ ! -x "$BIN_DIR/realityos-metal-propose" ]]; then
  echo "error: missing $BIN_DIR/realityos-metal-propose" >&2
  exit 2
fi
install -m 0755 "$BIN_DIR/realityos-metal-propose" "$STAGE/realityos-metal-propose"
PROP="$STAGE/realityos-metal-propose"

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

if [[ -f "$ROOT/metal_proof.json" || -f "$ROOT/proof_meta.json" ]]; then
  echo "error: OS-boundary test must not write a metal proof" >&2
  exit 1
fi

echo "metal OS boundary finished (not physical evidence)"
