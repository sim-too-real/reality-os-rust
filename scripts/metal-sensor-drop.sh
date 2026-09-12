#!/usr/bin/env bash
# Detect a live bus/VIN drop from an authority sensor JSON body.
# `poll-vin SOCK SAVE [SECONDS]` wraps metal-vin-sensor-poll.py (stdout
# JSON, same dxl_vin_* tokens). Not a PTY proof gate.
#
# `realityos-metal-propose sensor` returns exit 0 for any successful IPC
# round trip, including ok=false. A VIN brownout does not kill serve; it
# refuses acquire and latches bus_lost. Comparing the CLI exit code
# therefore never observes a real cutoff. persist_vin runs after a
# successful motion-block read, including a 0 / brown VIN sample, then
# live acquire refuses. The file freezes only when that read itself
# fails (silent servo). Parse the JSON.

# Keep in sync with metal_sensor_indicates_drop tokens and campaign
# MEASURE_REQUIRE after a live USB-UART drop. VIN cutoff uses the
# narrower metal_sensor_indicates_vin_drop set — UART death is not VIN.
METAL_BUS_DROP_TOKEN_SPEC='dxl_io|driver not connected|online_hardware_disconnected|metal_live_io_deadline|metal_serial_closed|hardware_disconnected|dxl_vin_outside_wizard_limits|dxl_vin_unreadable'
METAL_VIN_DROP_TOKEN_SPEC='dxl_vin_outside_wizard_limits|dxl_vin_unreadable'

# `realityos-metal-propose sensor` exits 0 for any successful IPC round
# trip, including ok=false (VIN refuse, DTR-RESET, bus_lost). A live
# session requires the JSON body, not the CLI status.
metal_sensor_is_live() {
  local path="${1:-}"
  [[ -n "$path" ]] || return 1
  python3 - "$path" <<'PY'
import json, sys

path = sys.argv[1]
try:
    raw = open(path, encoding="utf-8").read().strip()
except OSError:
    sys.exit(1)
if not raw:
    sys.exit(1)
try:
    body = json.loads(raw)
except json.JSONDecodeError:
    sys.exit(1)
sys.exit(0 if isinstance(body, dict) and body.get("ok") is True else 1)
PY
}

metal_sensor_indicates_drop() {
  local path="${1:-}"
  [[ -n "$path" ]] || return 1
  python3 - "$path" <<'PY'
import json, sys

path = sys.argv[1]
tokens = (
    "dxl_io",
    "driver not connected",
    "online_hardware_disconnected",
    "metal_live_io_deadline",
    "metal_serial_closed",
    "hardware_disconnected",
    "dxl_vin_outside_wizard_limits",
    "dxl_vin_unreadable",
)
try:
    raw = open(path, encoding="utf-8").read().strip()
except OSError:
    sys.exit(1)
if not raw:
    sys.exit(1)
try:
    body = json.loads(raw)
except json.JSONDecodeError:
    # Serve died mid-response (USB-UART unplug). Not a healthy sample.
    sys.exit(0)
if not isinstance(body, dict):
    sys.exit(0)
if body.get("ok") is True:
    sys.exit(1)
blob = json.dumps(body).lower()
sys.exit(0 if any(tok in blob for tok in tokens) else 1)
PY
}

# One autonomy process, one connect per sample. Same tokens as
# metal_sensor_indicates_vin_drop. The poller prints JSON to stdout
# and does not write the metal root: ROOT is 0751 and a 0700 clone
# would make autonomy miss both the save file and this script.
# bus/vin < 2.0 V stays a root-side fallback because bus/ is 0700.
_METAL_SENSOR_DROP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
METAL_VIN_SENSOR_POLL_PY="${_METAL_SENSOR_DROP_DIR}/metal-vin-sensor-poll.py"

metal_poll_vin_sensor_ipc() {
  local sock="${1:-}"
  local save="${2:-}"
  local seconds="${3:-60}"
  [[ -n "$sock" && -n "$save" && -f "$METAL_VIN_SENSOR_POLL_PY" ]] || return 1
  python3 "$METAL_VIN_SENSOR_POLL_PY" "$sock" "$seconds" >"$save"
}

# Independent VIN cutoff: USB data may stay enumerated. A USB-UART
# wiggle, truncated IPC, or serve death is the unplug case — not VIN.
metal_sensor_indicates_vin_drop() {
  local path="${1:-}"
  [[ -n "$path" ]] || return 1
  python3 - "$path" <<'PY'
import json, sys

path = sys.argv[1]
tokens = (
    "dxl_vin_outside_wizard_limits",
    "dxl_vin_unreadable",
)
try:
    raw = open(path, encoding="utf-8").read().strip()
except OSError:
    sys.exit(1)
if not raw:
    sys.exit(1)
try:
    body = json.loads(raw)
except json.JSONDecodeError:
    sys.exit(1)
if not isinstance(body, dict):
    sys.exit(1)
if body.get("ok") is True:
    sys.exit(1)
blob = json.dumps(body).lower()
sys.exit(0 if any(tok in blob for tok in tokens) else 1)
PY
}

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  set -euo pipefail
  if [[ "${1:-}" == "poll-vin" ]]; then
    metal_poll_vin_sensor_ipc "${2:-}" "${3:-}" "${4:-60}"
    exit $?
  fi
  tmp="$(mktemp -d)"
  trap 'rm -rf "$tmp"' EXIT
  printf '%s\n' '{"ok":true,"stage":"sensor"}' >"$tmp/live.json"
  metal_sensor_is_live "$tmp/live.json"
  if metal_sensor_indicates_drop "$tmp/live.json"; then
    echo "error: healthy sensor must not count as a drop" >&2
    exit 1
  fi
  printf '%s\n' '{"ok":false,"violations":["dxl_vin_unreadable"]}' >"$tmp/refused.json"
  if metal_sensor_is_live "$tmp/refused.json"; then
    echo "error: ok=false must not count as a live sensor (propose/sensor exits 0)" >&2
    exit 1
  fi
  printf '%s\n' 'not-json{' >"$tmp/dead-live.json"
  if metal_sensor_is_live "$tmp/dead-live.json"; then
    echo "error: truncated IPC must not count as a live sensor" >&2
    exit 1
  fi
  printf '%s\n' '{"ok":false,"violations":["dxl_io:metal_live_io_deadline","online_hardware_disconnected"]}' >"$tmp/vin.json"
  metal_sensor_indicates_drop "$tmp/vin.json"
  printf '%s\n' '{"ok":false,"violations":["metal_serial_closed"]}' >"$tmp/closed.json"
  metal_sensor_indicates_drop "$tmp/closed.json"
  printf '%s\n' '{"ok":false,"violations":["dxl_vin_outside_wizard_limits:vin_0.1v=20:min=35:max=70"]}' >"$tmp/brown.json"
  metal_sensor_indicates_drop "$tmp/brown.json"
  printf '%s\n' '{"ok":false,"violations":["dxl_vin_unreadable"]}' >"$tmp/novin.json"
  metal_sensor_indicates_drop "$tmp/novin.json"
  printf '%s\n' '{"ok":false,"violations":["unsupported_action"]}' >"$tmp/other.json"
  if metal_sensor_indicates_drop "$tmp/other.json"; then
    echo "error: unrelated refuse must not count as a VIN drop" >&2
    exit 1
  fi
  printf '%s\n' '' >"$tmp/empty.json"
  if metal_sensor_indicates_drop "$tmp/empty.json"; then
    echo "error: empty file must not count as a drop" >&2
    exit 1
  fi
  printf '%s\n' 'not-json{' >"$tmp/dead.json"
  metal_sensor_indicates_drop "$tmp/dead.json"
  metal_sensor_indicates_vin_drop "$tmp/brown.json"
  metal_sensor_indicates_vin_drop "$tmp/novin.json"
  printf '%s\n' '{"ok":false,"violations":["metal_serial_closed","dxl_vin_unreadable"]}' >"$tmp/mixed.json"
  metal_sensor_indicates_vin_drop "$tmp/mixed.json"
  if metal_sensor_indicates_vin_drop "$tmp/closed.json"; then
    echo "error: UART serial_closed must not count as a VIN drop" >&2
    exit 1
  fi
  if metal_sensor_indicates_vin_drop "$tmp/vin.json"; then
    echo "error: UART disconnect tokens must not count as a VIN drop" >&2
    exit 1
  fi
  if metal_sensor_indicates_vin_drop "$tmp/dead.json"; then
    echo "error: truncated IPC must not count as a VIN drop" >&2
    exit 1
  fi
  if metal_sensor_indicates_vin_drop "$tmp/live.json"; then
    echo "error: healthy sensor must not count as a VIN drop" >&2
    exit 1
  fi
  python3 - "$tmp/poll-vin-ok.sock" "$tmp/poll-vin-uart.sock" "$tmp/poll-vin-live.sock" <<'PY' &
import json, os, socket, sys, threading, time

ok_path, uart_path, live_path = sys.argv[1:4]


def serve(path, replies):
    try:
        os.unlink(path)
    except OSError:
        pass
    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(path)
    srv.listen(8)
    srv.settimeout(0.2)
    i = 0
    end = time.monotonic() + 4.0
    while time.monotonic() < end:
        try:
            conn, _ = srv.accept()
        except socket.timeout:
            continue
        with conn:
            conn.settimeout(0.2)
            try:
                buf = b""
                while b"\n" not in buf:
                    chunk = conn.recv(4096)
                    if not chunk:
                        break
                    buf += chunk
            except OSError:
                continue
            body = replies[min(i, len(replies) - 1)]
            i += 1
            try:
                conn.sendall((body + "\n").encode())
            except OSError:
                pass
    srv.close()
    try:
        os.unlink(path)
    except OSError:
        pass


threading.Thread(
    target=serve,
    args=(
        ok_path,
        [
            '{"ok":true,"stage":"sensor"}',
            '{"ok":false,"violations":["online_hardware_disconnected"]}',
            "not-json{",
            '{"ok":false,"violations":["dxl_vin_unreadable"]}',
        ],
    ),
    daemon=True,
).start()
threading.Thread(
    target=serve,
    args=(
        uart_path,
        [
            '{"ok":false,"violations":["metal_serial_closed"]}',
            '{"ok":false,"violations":["online_hardware_disconnected","dxl_io"]}',
        ],
    ),
    daemon=True,
).start()
threading.Thread(
    target=serve,
    args=(live_path, ['{"ok":true,"stage":"sensor"}']),
    daemon=True,
).start()
time.sleep(3.5)
PY
  srv_pid=$!
  for _ in $(seq 1 20); do
    if [[ -S "$tmp/poll-vin-ok.sock" && -S "$tmp/poll-vin-uart.sock" && -S "$tmp/poll-vin-live.sock" ]]; then
      break
    fi
    sleep 0.05
  done
  if ! metal_poll_vin_sensor_ipc "$tmp/poll-vin-ok.sock" "$tmp/poll-vin-hit.json" 2; then
    echo "error: poll-vin must catch dxl_vin_unreadable after healthy/UART/truncated samples" >&2
    kill "$srv_pid" 2>/dev/null || true
    exit 1
  fi
  metal_sensor_indicates_vin_drop "$tmp/poll-vin-hit.json"
  # Campaign runs `python3 - SOCK SECONDS < poll.py >save` so autonomy
  # never opens the repo script or writes ROOT.
  if ! python3 - "$tmp/poll-vin-ok.sock" 1 <"$METAL_VIN_SENSOR_POLL_PY" >"$tmp/poll-vin-stdin.json"; then
    echo "error: python3 - < metal-vin-sensor-poll.py must catch VIN (campaign VIN wait)" >&2
    kill "$srv_pid" 2>/dev/null || true
    exit 1
  fi
  metal_sensor_indicates_vin_drop "$tmp/poll-vin-stdin.json"
  if metal_poll_vin_sensor_ipc "$tmp/poll-vin-uart.sock" "$tmp/poll-vin-uart.json" 0.4; then
    echo "error: poll-vin must not treat UART death as VIN" >&2
    kill "$srv_pid" 2>/dev/null || true
    exit 1
  fi
  if metal_poll_vin_sensor_ipc "$tmp/poll-vin-live.sock" "$tmp/poll-vin-live.json" 0.4; then
    echo "error: poll-vin must not treat a healthy sensor as VIN" >&2
    kill "$srv_pid" 2>/dev/null || true
    exit 1
  fi
  kill "$srv_pid" 2>/dev/null || true
  wait "$srv_pid" 2>/dev/null || true
  echo "metal-sensor-drop-ok"
fi
