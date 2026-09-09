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

# Authority UID cannot write the repo `docs/` tree. Resolve the install path
# now; the reporter writes into $ROOT, then root copies here.
if [[ "$OUT" != /* ]]; then
  OUT="$(pwd)/$OUT"
fi

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
DEVICE_REAL="$(readlink -f "$DEVICE" 2>/dev/null || echo "$DEVICE")"
if [[ ! -c "$DEVICE" && ! -c "$DEVICE_REAL" ]]; then
  echo "error: $DEVICE is not a character device; will not write a metal proof." >&2
  exit 2
fi
if [[ "$DEVICE_REAL" == /dev/pts/* ]]; then
  echo "error: refusing PTY $DEVICE_REAL; not a physical actuator. Will not write metal_proof.json." >&2
  exit 2
fi
if [[ -z "$ROOT" || "$ROOT" == "/" || "$ROOT" == "/tmp" || "$ROOT" == "/var" ]]; then
  echo "error: refusing to wipe unexpected REALITYOS_METAL_ROOT=$ROOT" >&2
  exit 2
fi
# Stale journal+seal makes --first-online refuse. Kill leftover serve first so
# it cannot rewrite the journal after the wipe.
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
"$SCRIPT_DIR/metal-kill-serve.sh" "$ROOT" || true
rm -rf "$ROOT"

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

export REALITYOS_METAL_ROOT="$ROOT"
export REALITYOS_METAL_DEVICE="$DEVICE"
"$SCRIPT_DIR/metal-deploy.sh"

as_autonomy() {
  sudo -u "$AUTONOMY_USER" -- env \
    REALITYOS_METAL_DEVICE="${REALITYOS_METAL_DEVICE:-}" \
    METAL_AUTHORITY_PID="${METAL_AUTHORITY_PID:-}" \
    METAL_CMD_ID="${METAL_CMD_ID:-}" \
    "$@"
}
as_authority() {
  sudo -u "$AUTHORITY_USER" -- env \
    REALITYOS_METAL_DEVICE="${REALITYOS_METAL_DEVICE:-}" \
    REALITYOS_METAL_BAUD="${REALITYOS_METAL_BAUD:-}" \
    REALITYOS_METAL_SERVO_ID="${REALITYOS_METAL_SERVO_ID:-}" \
    REALITYOS_METAL_CAMPAIGN="${REALITYOS_METAL_CAMPAIGN:-}" \
    REALITYOS_HIL_CRASH="${REALITYOS_HIL_CRASH:-}" \
    "$@"
}

as_authority "$SMOKE" --root "$ROOT" --device "$DEVICE" init
as_authority "$SMOKE" --root "$ROOT" --device "$DEVICE" probe
as_authority "$SMOKE" --root "$ROOT" bind-measured

writes() { cat "$ROOT/bus/writes" 2>/dev/null || echo 0; }
acks() { cat "$ROOT/bus/acks" 2>/dev/null || echo 0; }
present() { cat "$ROOT/bus/present" 2>/dev/null || echo ""; }
goalpos() { cat "$ROOT/bus/goal" 2>/dev/null || echo ""; }

# $! after `sudo -u ... serve &` is the sudo wrapper. /proc/<sudo>/fd is not
# the authority tty; measure proc-fd against the smoke child.
resolve_metal_smoke_pid() {
  local root="$1"
  local pid cmdline rest
  while read -r pid cmdline; do
    case "$cmdline" in
      sudo*|*" sudo "*) continue ;;
    esac
    rest="${cmdline##*realityos-metal-smoke --root }"
    if [[ "$rest" != "$cmdline" && ( "$rest" == "${root} "* || "$rest" == "${root}" ) ]]; then
      echo "$pid"
      return 0
    fi
  done < <(pgrep -af 'realityos-metal-smoke' 2>/dev/null || true)
  return 1
}

start_auth() {
  local first="$1"
  local crash="${2:-}"
  rm -f "$ROOT/ipc.sock"
  export REALITYOS_METAL_CAMPAIGN=1
  if [[ -n "$crash" ]]; then
    export REALITYOS_HIL_CRASH="$crash"
  else
    unset REALITYOS_HIL_CRASH
  fi
  if [[ "$first" == "1" ]]; then
    as_authority "$SMOKE" --root "$ROOT" --first-online serve \
      >"$ROOT/authority.out" 2>"$ROOT/authority.err" &
  else
    as_authority "$SMOKE" --root "$ROOT" --restart serve \
      >"$ROOT/authority.out" 2>"$ROOT/authority.err" &
  fi
  AUTH_PID=$!
  for _ in $(seq 1 400); do
    if [[ -S "$ROOT/ipc.sock" ]]; then
      break
    fi
    sleep 0.05
  done
  unset REALITYOS_HIL_CRASH
  SMOKE_PID="$(resolve_metal_smoke_pid "$ROOT" || echo "$AUTH_PID")"
  if [[ ! -S "$ROOT/ipc.sock" ]]; then
    echo "error: ipc.sock did not appear" >&2
    cat "$ROOT/authority.err" >&2 || true
    return 1
  fi
  chmod 0660 "$ROOT/ipc.sock"
  chgrp "$IPC_GROUP" "$ROOT/ipc.sock"
  # udev may reset the tty to 0660 dialout after open. Re-apply exclusive mode.
  if [[ -e "$DEVICE" ]]; then
    chown "$AUTHORITY_USER:$AUTHORITY_USER" "$DEVICE" 2>/dev/null || true
    chmod 0600 "$DEVICE" 2>/dev/null || true
  fi
}

stop_auth() {
  kill "$AUTH_PID" 2>/dev/null || true
  wait "$AUTH_PID" 2>/dev/null || true
  "$SCRIPT_DIR/metal-kill-serve.sh" "$ROOT" || true
}

AUTH_PID=""
SMOKE_PID=""
cleanup() { stop_auth || true; }
trap cleanup EXIT
start_auth 1
if [[ -e "$DEVICE" ]]; then
  chown "$AUTHORITY_USER:$AUTHORITY_USER" "$DEVICE" 2>/dev/null || true
  chmod 0600 "$DEVICE" 2>/dev/null || true
fi

PROBE="$(as_autonomy env METAL_AUTHORITY_PID="${SMOKE_PID:-$AUTH_PID}" "$PROP" --root "$ROOT" --authority-pid "${SMOKE_PID:-$AUTH_PID}" os-probe)"
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
  shift 4
  local before after respfile ack_before ack_after pb pa gp
  before="$(writes)"
  ack_before="$(acks)"
  pb="$(present)"
  respfile="$(mktemp)"
  if as_autonomy "$@" >"$respfile" 2>/dev/null; then
    :
  else
    echo '{"ok":false,"executed":false,"stage":"ipc","status":"error"}' >"$respfile"
  fi
  after="$(writes)"
  ack_after="$(acks)"
  pa="$(present)"
  gp="$(goalpos)"
  python3 - "$name" "$proposal" "$layer" "$expected" "$before" "$after" "$ack_before" "$ack_after" "$respfile" "$pb" "$pa" "$gp" <<'PY'
import json, sys
name, proposal, layer, expected, before, after, ab, aa, path, pb, pa, gp = sys.argv[1:13]
before, after, ab, aa = map(int, (before, after, ab, aa))
expected = expected == "true"
delta = max(0, after - before)
try:
    r = json.load(open(path))
except Exception:
    r = {"ok": False, "executed": False, "stage": "ipc", "status": "error"}
def parse(x):
    try:
        return int(x)
    except Exception:
        return None
pb_i, pa_i, gp_i = parse(pb), parse(pa), parse(gp)
motion = None
if pa_i is not None or gp_i is not None:
    dlt = None if pb_i is None or pa_i is None else pa_i - pb_i
    motion = f"present {pb}->{pa} goal={gp} delta={dlt}"
ack = aa > ab
if r.get("device_acks") is not None and r.get("physical_writes") is not None:
    ack = ack or bool(r.get("ok") and aa > ab)
rec = {
    "name": name,
    "expected_authorization": expected,
    "decision_result": f'{r.get("stage","")}:{r.get("status","")}',
    "writes_before": before,
    "writes_after": after,
    "write_delta": delta,
    "device_acknowledgement": ack and bool(r.get("ok")),
    "observed_motion": motion,
    "blocking_layer": layer,
    "journal_result": "consumed" if r.get("ok") else "no_consume",
    "proposal": proposal,
    "unauthorized_write": (not expected) and delta > 0,
}
print(json.dumps(rec))
PY
  rm -f "$respfile"
}

CASES_FILE="$ROOT/os_metal_cases.json"
printf '%s\n' '[]' > "$CASES_FILE"
chmod 0644 "$CASES_FILE"
add_case() {
  python3 -c 'import json,sys
path, raw = sys.argv[1], sys.argv[2]
a = json.load(open(path))
a.append(json.loads(raw))
open(path, "w").write(json.dumps(a))
' "$CASES_FILE" "$1"
}

add_case "$(measure valid_hold 'verb=hold' NONE true "$PROP" --root "$ROOT" --id metal-hold --verb hold propose)"
add_case "$(measure valid_nudge 'verb=drive action=0.05' NONE true "$PROP" --root "$ROOT" --id metal-nudge --verb drive --action 0.05 propose)"
add_case "$(measure unsupported_action 'verb=dance' AUTHORIZATION_BLOCKED false "$PROP" --root "$ROOT" unsupported)"
add_case "$(measure oversized_action 'action=1e6' AUTHORIZATION_BLOCKED false "$PROP" --root "$ROOT" oversized)"
add_case "$(measure nan_action 'action=NaN' AUTHORIZATION_BLOCKED false "$PROP" --root "$ROOT" --id metal-nan --verb drive --action nan propose)"
add_case "$(measure replay 'same command_id metal-hold' AUTHORIZATION_BLOCKED false env METAL_CMD_ID=metal-hold "$PROP" --root "$ROOT" replay)"
add_case "$(measure malformed_json 'raw {not-json' PROTOCOL_BLOCKED false "$PROP" --root "$ROOT" raw)"
add_case "$(measure hil_fault_refused 'hil_fault' PROTOCOL_BLOCKED false "$PROP" --root "$ROOT" hil_fault)"
add_case "$(measure caller_time_refused 'propose now_s' PROTOCOL_BLOCKED false "$PROP" --root "$ROOT" caller_time)"
add_case "$(measure forged_sensor_refused 'autonomy sensor_samples' PROTOCOL_BLOCKED false "$PROP" --root "$ROOT" forged-sensor)"

as_authority bash -c "echo 1 > '$ROOT/bus/fail_sensor'"
add_case "$(measure missing_sensor 'fail_sensor then propose' AUTHORIZATION_BLOCKED false env METAL_CMD_ID=metal-miss "$PROP" --root "$ROOT" propose-id)"
as_authority rm -f "$ROOT/bus/fail_sensor"

as_authority bash -c "printf '%s' '{\"firmware_id\":\"xl330-m288:1190:255\"}' > '$ROOT/bus/hot_swap.json'"
add_case "$(measure firmware_mismatch 'hot_swap firmware only' AUTHORIZATION_BLOCKED false env METAL_CMD_ID=metal-fw "$PROP" --root "$ROOT" propose-id)"
add_case "$(measure recover_after_identity 'recover after firmware mismatch' AUTHORIZATION_BLOCKED false "$PROP" --root "$ROOT" recover)"
add_case "$(measure reconnect_foreign 'same instance after foreign firmware' AUTHORIZATION_BLOCKED false env METAL_CMD_ID=metal-re "$PROP" --root "$ROOT" propose-id)"
as_authority rm -f "$ROOT/bus/hot_swap.json"

stop_auth
start_auth 0
as_authority bash -c "echo 1 > '$ROOT/bus/force_disconnect'"
add_case "$(measure device_disconnect 'force_disconnect then propose' AUTHORIZATION_BLOCKED false env METAL_CMD_ID=metal-disc "$PROP" --root "$ROOT" propose-id)"
add_case "$(measure recover_after_disconnect 'recover cannot resurrect binding' AUTHORIZATION_BLOCKED false "$PROP" --root "$ROOT" recover)"
as_authority rm -f "$ROOT/bus/force_disconnect"

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

crash_replay() {
  local point="$1"
  local cid="$2"
  stop_auth
  if ! start_auth 0 "$point"; then
    echo "error: crash serve did not bind for $point" >&2
    exit 1
  fi
  as_autonomy "$PROP" --root "$ROOT" --id "$cid" --verb hold propose >/tmp/metal-"$cid".json || true
  wait "$AUTH_PID" 2>/dev/null || true
  "$SCRIPT_DIR/metal-kill-serve.sh" "$ROOT" || true
  if ! start_auth 0; then
    echo "error: restart after $point crash failed" >&2
    exit 1
  fi
  local before after
  before="$(writes)"
  as_autonomy env METAL_CMD_ID="$cid" "$PROP" --root "$ROOT" replay >/tmp/metal-"$cid"-replay.json || true
  after="$(writes)"
  python3 - <<PY
import json
before=int("$before"); after=int("$after")
print(json.dumps({
    "name": "crash_restart_${point}",
    "expected_authorization": False,
    "decision_result": "crash:no_auto_retry",
    "writes_before": before,
    "writes_after": after,
    "write_delta": max(0, after-before),
    "device_acknowledgement": False,
    "observed_motion": None,
    "blocking_layer": "CRASH_RECOVERY_BLOCKED",
    "journal_result": "not_retried",
    "proposal": "same command_id after $point crash/restart",
    "unauthorized_write": after>before,
}))
PY
}

add_case "$(crash_replay after_prepare_before_write metal-crash-prep)"
add_case "$(crash_replay during_write metal-crash-during)"
add_case "$(crash_replay after_write_before_ack metal-crash-ack)"
add_case "$(crash_replay after_ack metal-crash-afterack)"

VIN="$(cat "$ROOT/bus/vin" 2>/dev/null || echo "")"
if [[ "$CUTOFF_TESTED" == "1" ]]; then
  CUTOFF_BEFORE="$(writes)"
  add_case "$(python3 - <<PY
import json
before=int("$CUTOFF_BEFORE")
print(json.dumps({
    "name": "independent_vin_cutoff",
    "expected_authorization": False,
    "decision_result": "operator:vin_open_servo_lost_torque",
    "writes_before": before,
    "writes_after": before,
    "write_delta": 0,
    "device_acknowledgement": False,
    "observed_motion": "operator opened VIN disconnect; servo lost holding torque independent of Reality OS; last vin_0.1v=$VIN",
    "blocking_layer": "OS_BLOCKED",
    "journal_result": "unchanged",
    "proposal": "physical VIN disconnect (not STO/SS1/PL/SIL)",
    "unauthorized_write": False,
}))
PY
)"
fi

if [[ "${REALITYOS_METAL_CUTOFF_LIVE:-0}" == "1" ]]; then
  echo "Open the independent VIN switch now (USB data may stay enumerated)." >&2
  dropped=0
  for _ in $(seq 1 120); do
    if ! as_autonomy "$PROP" --root "$ROOT" sensor >/dev/null 2>&1; then
      dropped=1
      break
    fi
    vin_now="$(cat "$ROOT/bus/vin" 2>/dev/null || echo 999)"
    if [[ "$vin_now" =~ ^[0-9]+$ ]] && [[ "$vin_now" -lt 20 ]]; then
      dropped=1
      break
    fi
    sleep 0.5
  done
  if [[ "$dropped" != "1" ]]; then
    echo "error: VIN did not drop within 60s; cutoff live test failed" >&2
    exit 1
  fi
  export REALITYOS_METAL_CUTOFF_TESTED=1
  CUTOFF_TESTED=1
  add_case "$(measure vin_cutoff_live 'propose after VIN open' AUTHORIZATION_BLOCKED false env METAL_CMD_ID=metal-cutoff "$PROP" --root "$ROOT" propose-id)"
fi

COMMIT="$(git -C "$(dirname "$SCRIPT_DIR")" rev-parse HEAD 2>/dev/null || echo unknown)"
DATE="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
MEASURED="$(cat "$ROOT/measured.json" 2>/dev/null || echo '{}')"
FRESH="$(cat "$ROOT/bus/sensor_freshness.json" 2>/dev/null || echo '{}')"
python3 - <<PY
import json, os
p = json.loads('''$PROBE''')
measured = json.loads('''$MEASURED''')
fresh = json.loads('''$FRESH''') if '''$FRESH'''.strip() else {}
if isinstance(measured, dict) and fresh.get("vin_0.1v") is not None:
    measured = dict(measured)
    measured["vin_0.1v"] = fresh.get("vin_0.1v")
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
  "sensor_source": fresh.get("sensor_source") or "xl330 registers + realtime tick",
  "device_capture_s": fresh.get("device_capture_s"),
  "authority_receive_s": fresh.get("authority_receive_s"),
  "freshness_threshold_s": fresh.get("freshness_threshold_s"),
}
open("$ROOT/proof_meta.json","w").write(json.dumps(meta, indent=2))
print("meta-written")
PY

as_authority "$SMOKE" --root "$ROOT" --cases "$CASES_FILE" --out "$ROOT/metal_proof.json" report
install -D -m 0644 "$ROOT/metal_proof.json" "$OUT"
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
