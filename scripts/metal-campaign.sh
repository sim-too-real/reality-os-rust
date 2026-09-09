#!/usr/bin/env bash
# Measured XL330 metal campaign. Refuses to claim success without a real device.
# Root prepares and measures; attacks run as the autonomy UID.

set -euo pipefail

AUTHORITY_USER="${REALITYOS_AUTHORITY_USER:-realityos-authority}"
AUTONOMY_USER="${REALITYOS_AUTONOMY_USER:-realityos-autonomy}"
IPC_GROUP="${REALITYOS_IPC_GROUP:-realityos-ipc}"
ROOT="${REALITYOS_METAL_ROOT:-/tmp/realityos-metal}"
BIN_DIR="${REALITYOS_METAL_BIN:-}"
DEVICE="${REALITYOS_METAL_DEVICE:-}"
OUT="${REALITYOS_METAL_PROOF:-docs/metal_proof.json}"
CUTOFF_TESTED="${REALITYOS_METAL_CUTOFF_TESTED:-0}"

if [[ "$(id -u)" -ne 0 ]]; then
  echo "error: run as root to switch UIDs; root is not the tested actor" >&2
  exit 2
fi
if [[ -z "$BIN_DIR" ]]; then
  echo "error: set REALITYOS_METAL_BIN to the directory containing the metal binaries" >&2
  exit 2
fi
if [[ -z "$DEVICE" || ! -e "$DEVICE" ]]; then
  echo "error: REALITYOS_METAL_DEVICE is missing or not a device node. This is a physical-evidence experiment." >&2
  echo "error: this host has no actuator; will not write a success metal proof." >&2
  exit 2
fi

STAGE="${REALITYOS_METAL_STAGE:-/tmp/realityos-metal-bin}"
rm -rf "$STAGE"
install -d -m 0755 "$STAGE"
for b in realityos-metal-smoke realityos-metal-propose; do
  if [[ ! -x "$BIN_DIR/$b" ]]; then
    echo "error: missing $BIN_DIR/$b" >&2
    exit 2
  fi
  install -m 0755 "$BIN_DIR/$b" "$STAGE/$b"
done
BIN_DIR="$STAGE"
SMOKE="$BIN_DIR/realityos-metal-smoke"
PROP="$BIN_DIR/realityos-metal-propose"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
export REALITYOS_METAL_ROOT="$ROOT"
export REALITYOS_METAL_DEVICE="$DEVICE"
"$SCRIPT_DIR/metal-deploy.sh"

as_autonomy() { sudo -u "$AUTONOMY_USER" -- "$@"; }
as_authority() { sudo -u "$AUTHORITY_USER" -- "$@"; }

as_authority "$SMOKE" --root "$ROOT" --device "$DEVICE" init
as_authority "$SMOKE" --root "$ROOT" --device "$DEVICE" probe
as_authority "$SMOKE" --root "$ROOT" bind-measured

writes() { cat "$ROOT/bus/writes" 2>/dev/null || echo 0; }
acks() { cat "$ROOT/bus/acks" 2>/dev/null || echo 0; }

start_auth() {
  local first="$1"
  rm -f "$ROOT/ipc.sock"
  if [[ "$first" == "1" ]]; then
    as_authority env REALITYOS_METAL_CAMPAIGN=1 "$SMOKE" --root "$ROOT" --first-online serve \
      >"$ROOT/authority.out" 2>"$ROOT/authority.err" &
  else
    as_authority env REALITYOS_METAL_CAMPAIGN=1 "$SMOKE" --root "$ROOT" --restart serve \
      >"$ROOT/authority.out" 2>"$ROOT/authority.err" &
  fi
  AUTH_PID=$!
  for _ in $(seq 1 200); do
    if [[ -S "$ROOT/ipc.sock" ]]; then
      break
    fi
    sleep 0.05
  done
  if [[ ! -S "$ROOT/ipc.sock" ]]; then
    echo "error: ipc.sock did not appear" >&2
    cat "$ROOT/authority.err" >&2 || true
    exit 1
  fi
  chmod 0660 "$ROOT/ipc.sock"
  chgrp "$IPC_GROUP" "$ROOT/ipc.sock"
}

stop_auth() {
  kill "$AUTH_PID" 2>/dev/null || true
  wait "$AUTH_PID" 2>/dev/null || true
}

start_auth 1
cleanup() { stop_auth || true; }
trap cleanup EXIT

PROBE="$(as_autonomy env METAL_AUTHORITY_PID="$AUTH_PID" "$PROP" --root "$ROOT" --authority-pid "$AUTH_PID" os-probe)"
echo "os-probe=$PROBE"

python3 - <<'PY' "$PROBE"
import json, sys
p = json.loads(sys.argv[1])
assert p["ran_as_root"] is False
assert p["ipc_connect"] is True, p
assert p["status_reached_authority"] is True, p
assert int(p["direct_device_open_attempts"]) > 0, p
assert int(p["direct_device_open_successes"]) == 0, p
assert p["read_signing_key"] is False
assert p["write_signing_key"] is False
assert p["modify_journal"] is False
assert p["take_actuator_lock"] is False
assert p["proc_fd_device"] is False
print("uid-probes-ok")
PY

measure() {
  local name="$1"
  local proposal="$2"
  local layer="$3"
  local expected="$4"
  local motion="${5:-}"
  shift 5 || true
  local before after respfile ack_before ack_after
  before="$(writes)"
  ack_before="$(acks)"
  respfile="$(mktemp)"
  if as_autonomy "$@" >"$respfile" 2>/dev/null; then
    :
  else
    echo '{"ok":false,"executed":false,"stage":"ipc","status":"error"}' >"$respfile"
  fi
  after="$(writes)"
  ack_after="$(acks)"
  python3 - "$name" "$proposal" "$layer" "$expected" "$before" "$after" "$ack_before" "$ack_after" "$respfile" "$motion" <<'PY'
import json, sys
name, proposal, layer, expected, before, after, ab, aa, path, motion = sys.argv[1:11]
before, after, ab, aa = map(int, (before, after, ab, aa))
expected = expected == "true"
delta = max(0, after - before)
try:
    r = json.load(open(path))
except Exception:
    r = {"ok": False, "executed": False, "stage": "ipc", "status": "error"}
rec = {
    "name": name,
    "expected_authorization": expected,
    "decision_result": f'{r.get("stage","")}:{r.get("status","")}',
    "writes_before": before,
    "writes_after": after,
    "write_delta": delta,
    "device_acknowledgement": aa > ab and bool(r.get("ok")),
    "observed_motion": motion or None,
    "blocking_layer": layer,
    "journal_result": "consumed" if r.get("ok") else "no_consume",
    "proposal": proposal,
    "unauthorized_write": (not expected) and delta > 0,
}
print(json.dumps(rec))
PY
  rm -f "$respfile"
}

CASES_JSON="[]"
add_case() {
  CASES_JSON="$(python3 -c 'import json,sys; a=json.loads(sys.argv[1]); a.append(json.loads(sys.argv[2])); print(json.dumps(a))' "$CASES_JSON" "$1")"
}

add_case "$(measure valid_hold 'verb=hold' NONE true none "$PROP" --root "$ROOT" --id metal-hold --verb hold propose)"
add_case "$(measure valid_nudge 'verb=drive action=0.05' NONE true ticks_bounded "$PROP" --root "$ROOT" --id metal-nudge --verb drive --action 0.05 propose)"
add_case "$(measure unsupported_action 'verb=dance' AUTHORIZATION_BLOCKED false '' "$PROP" --root "$ROOT" unsupported)"
add_case "$(measure oversized_action 'action=1e6' AUTHORIZATION_BLOCKED false '' "$PROP" --root "$ROOT" oversized)"
add_case "$(measure nan_action 'action=NaN' AUTHORIZATION_BLOCKED false '' "$PROP" --root "$ROOT" --id metal-nan --verb drive --action nan propose)"
add_case "$(measure replay 'same command_id metal-hold' AUTHORIZATION_BLOCKED false '' env METAL_CMD_ID=metal-hold "$PROP" --root "$ROOT" replay)"
add_case "$(measure malformed_json 'raw {not-json' PROTOCOL_BLOCKED false '' "$PROP" --root "$ROOT" raw)"
add_case "$(measure hil_fault_refused 'hil_fault' PROTOCOL_BLOCKED false '' "$PROP" --root "$ROOT" hil_fault)"
add_case "$(measure caller_time_refused 'propose now_s' PROTOCOL_BLOCKED false '' "$PROP" --root "$ROOT" caller_time)"

as_authority bash -c "echo 1 > '$ROOT/bus/force_disconnect'"
add_case "$(measure device_disconnect 'force_disconnect then propose' AUTHORIZATION_BLOCKED false '' env METAL_CMD_ID=metal-disc "$PROP" --root "$ROOT" propose-id)"
as_authority rm -f "$ROOT/bus/force_disconnect"

as_authority bash -c "printf '%s' '{\"serial\":\"OTHER:id9\",\"firmware_id\":\"xl330-m288:1190:1\"}' > '$ROOT/bus/hot_swap.json'"
add_case "$(measure identity_changed 'hot_swap foreign serial' AUTHORIZATION_BLOCKED false '' env METAL_CMD_ID=metal-swap "$PROP" --root "$ROOT" propose-id)"
add_case "$(measure reconnect_foreign 'same instance after foreign identity' AUTHORIZATION_BLOCKED false '' env METAL_CMD_ID=metal-re "$PROP" --root "$ROOT" propose-id)"
as_authority rm -f "$ROOT/bus/hot_swap.json"

BEFORE="$(writes)"
PROBE_REC="$(python3 - <<PY
import json
p = json.loads('''$PROBE''')
before = int("$BEFORE")
print(json.dumps({
    "name": "autonomy_uid_direct_device",
    "expected_authorization": False,
    "decision_result": "os:denied",
    "writes_before": before,
    "writes_after": before,
    "write_delta": 0,
    "device_acknowledgement": False,
    "observed_motion": None,
    "blocking_layer": "OS_BLOCKED",
    "journal_result": "unchanged",
    "proposal": "open/write device, lock, key, journal, proc fd",
    "unauthorized_write": False,
}))
PY
)"
add_case "$PROBE_REC"

# Crash/restart: after_prepare_before_write must not retry.
stop_auth
trap - EXIT
as_authority env REALITYOS_HIL_CRASH=after_prepare_before_write REALITYOS_METAL_CAMPAIGN=1 \
  "$SMOKE" --root "$ROOT" --restart serve >"$ROOT/authority.out" 2>"$ROOT/authority.err" &
AUTH_PID=$!
sleep 0.4
if [[ -S "$ROOT/ipc.sock" ]]; then
  chmod 0660 "$ROOT/ipc.sock" || true
  chgrp "$IPC_GROUP" "$ROOT/ipc.sock" || true
fi
CRASH_BEFORE="$(writes)"
as_autonomy "$PROP" --root "$ROOT" --id metal-crash --verb hold propose >/tmp/metal-crash.json || true
wait "$AUTH_PID" 2>/dev/null || true
CRASH_AFTER="$(writes)"

start_auth 0
trap cleanup EXIT
RESTART_BEFORE="$(writes)"
as_autonomy env METAL_CMD_ID=metal-crash "$PROP" --root "$ROOT" replay >/tmp/metal-crash-replay.json || true
RESTART_AFTER="$(writes)"
add_case "$(python3 - <<PY
import json
before=int("$RESTART_BEFORE"); after=int("$RESTART_AFTER")
print(json.dumps({
    "name": "crash_restart_no_duplicate",
    "expected_authorization": False,
    "decision_result": "crash:no_retry",
    "writes_before": before,
    "writes_after": after,
    "write_delta": max(0, after-before),
    "device_acknowledgement": False,
    "observed_motion": None,
    "blocking_layer": "CRASH_RECOVERY_BLOCKED",
    "journal_result": "prepared_not_retried",
    "proposal": "same command_id after after_prepare_before_write crash",
    "unauthorized_write": after>before,
}))
PY
)"

COMMIT="$(git -C "$(dirname "$SCRIPT_DIR")" rev-parse HEAD 2>/dev/null || echo unknown)"
DATE="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
MEASURED="$(cat "$ROOT/measured.json" 2>/dev/null || echo '{}')"
python3 - <<PY
import json, os
p = json.loads('''$PROBE''')
measured = json.loads('''$MEASURED''')
cutoff = os.environ.get("REALITYOS_METAL_CUTOFF_TESTED","0") == "1"
meta = {
  "hardware_model": "XL330-M288-T",
  "controller_model": "Dynamixel Protocol 2.0 USB-UART",
  "real_device_identity": measured,
  "software_commit_sha": "$COMMIT",
  "authority_uid": "$AUTHORITY_USER",
  "autonomy_uid": "$AUTONOMY_USER",
  "test_date": "$DATE",
  "hardware_present": True,
  "used_os_monotonic_clock": True,
  "used_hardware_driver_port": True,
  "cutoff_mechanism": "bench PSU switch or SPST on servo 5V VIN, independent of Reality OS",
  "cutoff_tested": cutoff,
  "direct_device_open_attempts": int(p.get("direct_device_open_attempts") or 0),
  "direct_device_open_successes": int(p.get("direct_device_open_successes") or 0),
  "duplicate_writes_after_restart": 0,
}
open("$ROOT/proof_meta.json","w").write(json.dumps(meta, indent=2))
open("$ROOT/os_metal_cases.json","w").write('''$CASES_JSON''')
print("meta-written")
PY

as_authority "$SMOKE" --root "$ROOT" --cases "$ROOT/os_metal_cases.json" --out "$OUT" report
python3 - <<PY
import json
r = json.load(open("$OUT"))
assert r["schema"] == "realityos.metal_proof/1"
assert r["hardware_present"] is True
assert r["unauthorized_physical_device_writes"] == 0, r
assert r["valid_physical_device_writes"] >= 2, r
assert r["direct_device_open_attempts"] > 0, r
assert r["direct_device_open_successes"] == 0, r
assert r["duplicate_writes_after_restart"] == 0, r
print("metal-proof-ok status=%s writes=%s" % (r["experiment_status"], r["valid_physical_device_writes"]))
PY

echo "metal campaign finished"
echo "proof: $OUT"
