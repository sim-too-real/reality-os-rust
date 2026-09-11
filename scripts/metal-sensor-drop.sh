#!/usr/bin/env bash
# Detect a live bus/VIN drop from an authority sensor JSON body.
#
# `realityos-metal-propose sensor` returns exit 0 for any successful IPC
# round trip, including ok=false. A VIN brownout does not kill serve; it
# refuses acquire and latches bus_lost. Comparing the CLI exit code
# therefore never observes a real cutoff. `bus/vin` also freezes at the
# last healthy sample because persist_vin runs only after a good motion
# read. Parse the JSON.

# Keep in sync with metal_sensor_indicates_drop tokens and campaign
# MEASURE_REQUIRE after a live VIN / USB-UART drop.
METAL_BUS_DROP_TOKEN_SPEC='dxl_io|driver not connected|online_hardware_disconnected|metal_live_io_deadline|metal_serial_closed|hardware_disconnected|dxl_vin_outside_wizard_limits|dxl_vin_unreadable'

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

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  set -euo pipefail
  tmp="$(mktemp -d)"
  trap 'rm -rf "$tmp"' EXIT
  printf '%s\n' '{"ok":true,"stage":"sensor"}' >"$tmp/live.json"
  if metal_sensor_indicates_drop "$tmp/live.json"; then
    echo "error: healthy sensor must not count as a drop" >&2
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
  echo "metal-sensor-drop-ok"
fi
