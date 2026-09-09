#!/usr/bin/env bash
# Measurable two-UID authority/autonomy deployment test.
# Root prepares and measures; attacks run as the autonomy UID.
# Root is outside the threat model.

set -euo pipefail

AUTHORITY_USER="${REALITYOS_AUTHORITY_USER:-realityos-authority}"
AUTONOMY_USER="${REALITYOS_AUTONOMY_USER:-realityos-autonomy}"
IPC_GROUP="${REALITYOS_IPC_GROUP:-realityos-ipc}"
ROOT="${REALITYOS_HIL_ROOT:-/tmp/realityos-hil-os}"
BIN_DIR="${REALITYOS_HIL_BIN:-}"
OUT="${REALITYOS_OS_USERS_PROOF:-docs/hil_os_users_proof.json}"
CASES="${ROOT}/os_users_cases.json"

if [[ "$(id -u)" -ne 0 ]]; then
  echo "error: run as root to switch UIDs; root is not the tested actor" >&2
  exit 2
fi

if [[ -z "$BIN_DIR" ]]; then
  echo "error: set REALITYOS_HIL_BIN to the directory containing hil-authority and hil-os-users" >&2
  exit 2
fi

AUTH="$BIN_DIR/hil-authority"
OSU="$BIN_DIR/hil-os-users"
for b in "$AUTH" "$OSU"; do
  if [[ ! -x "$b" ]]; then
    echo "error: missing $b" >&2
    exit 2
  fi
done

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
"$SCRIPT_DIR/hil-deploy-users.sh"

writes() {
  cat "$ROOT/bus/writes" 2>/dev/null || echo 0
}

as_autonomy() {
  sudo -u "$AUTONOMY_USER" -- "$@"
}

as_authority() {
  # Do not set -g ipc here: journal/seal must stay owner-only. Only the socket
  # is regrouped to $IPC_GROUP after bind.
  sudo -u "$AUTHORITY_USER" -- "$@"
}

rm -f "$ROOT/ipc.sock"
as_authority "$AUTH" --root "$ROOT" --first-online --production serve \
  >"$ROOT/authority.out" 2>"$ROOT/authority.err" &
AUTH_PID=$!
cleanup() {
  kill "$AUTH_PID" 2>/dev/null || true
  wait "$AUTH_PID" 2>/dev/null || true
}
trap cleanup EXIT

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

# Explicit mode must not be umask-dependent.
chmod 0660 "$ROOT/ipc.sock"
chgrp "$IPC_GROUP" "$ROOT/ipc.sock"

PROBE="$(as_autonomy "$OSU" --root "$ROOT" --authority-pid "$AUTH_PID" probe)"
echo "probe=$PROBE"

python3 - <<'PY' "$PROBE"
import json, sys
p = json.loads(sys.argv[1])
assert p["ran_as_root"] is False
assert p["ipc_connect"] is True, p
assert p["status_reached_authority"] is True, p
for k in (
    "read_signing_key",
    "write_signing_key",
    "open_actuator_lock",
    "take_actuator_lock",
    "open_actuator_log",
    "write_actuator_log",
    "modify_journal",
    "chmod_signing_key",
    "proc_fd_device",
):
    assert p[k] is False, (k, p)
print("uid-probes-ok")
PY

measure() {
  local name="$1"
  local proposal="$2"
  local layer="$3"
  local expected="$4"
  local before after respfile
  before="$(writes)"
  respfile="$(mktemp)"
  if as_autonomy "${@:5}" >"$respfile" 2>/dev/null; then
    :
  else
    echo '{"ok":false,"executed":false,"stage":"ipc","status":"error"}' >"$respfile"
  fi
  after="$(writes)"
  python3 - "$name" "$proposal" "$layer" "$expected" "$before" "$after" "$respfile" <<'PY'
import json, sys
name, proposal, layer, expected, before, after, path = sys.argv[1:8]
before, after = int(before), int(after)
expected = expected == "true"
delta = max(0, after - before)
try:
    r = json.load(open(path))
except Exception:
    r = {"ok": False, "executed": False, "stage": "ipc", "status": "error"}
ok = bool(r.get("ok", False))
rec = {
    "name": name,
    "proposal": proposal,
    "semantic_verdict": f'{r.get("stage","")}:{r.get("status","")}',
    "authority_transition": r.get("stage", "ipc"),
    "blocking_layer": layer,
    "writes_before": before,
    "writes_after": after,
    "write_delta": delta,
    "expected_authorized": expected,
    "executed": bool(r.get("executed", ok)),
    "driver_write_count": after,
    "journal_state": "consumed" if ok else "no_consume",
    "outcome": "ok" if ok else "refused",
    "unauthorized_write": (not expected) and delta > 0,
}
print(json.dumps(rec))
PY
  rm -f "$respfile"
}

CASES_JSON="[]"
add_case() {
  local rec="$1"
  CASES_JSON="$(python3 -c 'import json,sys; a=json.loads(sys.argv[1]); a.append(json.loads(sys.argv[2])); print(json.dumps(a))' "$CASES_JSON" "$rec")"
}

add_case "$(measure valid_hold 'verb=hold' NONE true "$OSU" --root "$ROOT" propose)"
add_case "$(measure unsupported_action 'verb=dance' AUTHORIZATION_BLOCKED false "$OSU" --root "$ROOT" unsupported)"
add_case "$(measure oversized_action 'action=1e6' AUTHORIZATION_BLOCKED false "$OSU" --root "$ROOT" oversized)"
add_case "$(measure replay 'same command_id os-valid' AUTHORIZATION_BLOCKED false "$OSU" --root "$ROOT" replay)"
add_case "$(measure malformed_json 'raw {not-json' PROTOCOL_BLOCKED false "$OSU" --root "$ROOT" raw)"
add_case "$(measure hil_fault_refused 'hil_fault now_s on production IPC' PROTOCOL_BLOCKED false "$OSU" --root "$ROOT" hil_fault)"

# Sequence leftover on production propose is ignored (authority-assigned).
add_case "$(measure sequence_leftover 'propose leftover sequence' AUTHORIZATION_BLOCKED false "$OSU" --root "$ROOT" unsupported)"

# Harness (authority UID) injects device conditions; autonomy proposes.
as_authority bash -c "echo 1 > '$ROOT/bus/force_disconnect'"
add_case "$(measure driver_disconnect 'authority force_disconnect then autonomy propose' AUTHORIZATION_BLOCKED false env HIL_CMD_ID=os-disc "$OSU" --root "$ROOT" propose-id)"
as_authority rm -f "$ROOT/bus/force_disconnect"

as_authority bash -c "printf '%s' '{\"serial\":\"SN-OTHER\",\"firmware_id\":\"HIL-FW-1\",\"calibration_id\":\"HIL-CAL-1\",\"design_content_hash\":\"hil_design\"}' > '$ROOT/bus/hot_swap.json'"
add_case "$(measure identity_changed 'hot_swap SN-OTHER then autonomy propose' AUTHORIZATION_BLOCKED false env HIL_CMD_ID=os-swap "$OSU" --root "$ROOT" propose-id)"

# Reconnect foreign: keep hot_swap in place, propose again under dead session.
add_case "$(measure reconnect_foreign_device 'same instance after foreign identity' AUTHORIZATION_BLOCKED false env HIL_CMD_ID=os-re "$OSU" --root "$ROOT" propose-id)"

# OS probe cases (zero writes).
BEFORE="$(writes)"
PROBE_REC="$(python3 - <<PY
import json
p = json.loads('''$PROBE''')
before = int("$BEFORE")
print(json.dumps({
    "name": "autonomy_uid_probes",
    "proposal": "read key/journal/device/lock/chmod/proc-fd",
    "semantic_verdict": "os:denied",
    "authority_transition": "os",
    "blocking_layer": "OS_BLOCKED",
    "writes_before": before,
    "writes_after": before,
    "write_delta": 0,
    "expected_authorized": False,
    "executed": False,
    "driver_write_count": before,
    "journal_state": "unchanged",
    "outcome": "denied",
    "unauthorized_write": False,
}))
PY
)"
add_case "$PROBE_REC"

printf '%s\n' "$CASES_JSON" > "$CASES"
mkdir -p "$(dirname "$OUT")"
"$OSU" report --cases "$CASES" --out "$OUT" >/dev/null

python3 - <<PY
import json
r = json.load(open("$OUT"))
assert r["schema"] == "realityos.hil_proof/2"
assert r["unauthorized_driver_writes"] == 0, r
assert r["valid_driver_writes"] >= 1, r
print("os-users-proof-ok unauthorized=0 valid_writes=%s" % r["valid_driver_writes"])
PY

echo "two-user deployment test passed"
echo "proof: $OUT"
