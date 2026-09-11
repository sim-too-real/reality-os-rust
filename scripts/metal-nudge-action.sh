#!/usr/bin/env bash
# Pick ±tau_max so the certified 32-tick nudge stays inside the cage.
#
# The campaign used to hardcode --action 0.2. write_action refuses an
# outbound goal (Wizard leftover max window, or present within 32 ticks
# of 4095) instead of clamping inward. plant.act Err after ledger.prepare
# is CommandOutcome::Unknown and abort-latches the ONLINE instance, so
# the campaign cannot retry the other sign. Prefer +delta when it fits;
# only pick -delta when +delta would refuse. Keep in sync with
# realityos_metal::config::pick_inbound_nudge_action.

metal_nudge_action_from_values() {
  python3 - "$@" <<'PY'
import math, sys

def pick(present, experiment_min, experiment_max, delta_ticks, tau_max):
    if delta_ticks <= 0:
        raise SystemExit("metal_nudge_delta_ticks_not_positive:%s" % (delta_ticks,))
    if not math.isfinite(tau_max) or tau_max <= 0:
        raise SystemExit("metal_nudge_tau_max_invalid:%s" % (tau_max,))
    plus = present + delta_ticks
    if experiment_min <= plus <= experiment_max:
        print("%g" % (tau_max,))
        return
    minus = present - delta_ticks
    if experiment_min <= minus <= experiment_max:
        print("%g" % (-tau_max,))
        return
    raise SystemExit(
        "metal_nudge_no_inbound_step:present=%s:delta=%s:cage=%s..%s"
        % (present, delta_ticks, experiment_min, experiment_max)
    )

present = int(sys.argv[1])
experiment_min = int(sys.argv[2])
experiment_max = int(sys.argv[3])
delta_ticks = int(sys.argv[4])
tau_max = float(sys.argv[5])
pick(present, experiment_min, experiment_max, delta_ticks, tau_max)
PY
}

# Read bus/present, bus/position_cage.json, and metal.json (delta / tau).
# Fail closed before propose when the 32-tick step cannot stay inbound.
metal_nudge_action_from_bus() {
  local root="${1:-}"
  [[ -n "$root" ]] || return 1
  python3 - "$root" <<'PY'
import json, math, os, sys

root = sys.argv[1]
present_p = os.path.join(root, "bus", "present")
cage_p = os.path.join(root, "bus", "position_cage.json")
cfg_p = os.path.join(root, "metal.json")
try:
    present = int(open(present_p, encoding="utf-8").read().strip())
except (OSError, ValueError):
    sys.exit("metal_nudge_present_unreadable")
try:
    cage = json.load(open(cage_p, encoding="utf-8"))
except (OSError, json.JSONDecodeError):
    sys.exit("metal_nudge_cage_unreadable")
try:
    experiment_min = int(cage["experiment_min"])
    experiment_max = int(cage["experiment_max"])
except (KeyError, TypeError, ValueError):
    sys.exit("metal_nudge_cage_unreadable")
delta_ticks = 32
tau_max = 0.2
try:
    cfg = json.load(open(cfg_p, encoding="utf-8"))
    if isinstance(cfg, dict):
        if cfg.get("max_position_delta_ticks") is not None:
            delta_ticks = int(cfg["max_position_delta_ticks"])
        if cfg.get("tau_max") is not None:
            tau_max = float(cfg["tau_max"])
except (OSError, json.JSONDecodeError, TypeError, ValueError):
    pass
if delta_ticks <= 0:
    sys.exit("metal_nudge_delta_ticks_not_positive:%s" % (delta_ticks,))
if not math.isfinite(tau_max) or tau_max <= 0:
    sys.exit("metal_nudge_tau_max_invalid:%s" % (tau_max,))
plus = present + delta_ticks
if experiment_min <= plus <= experiment_max:
    action = tau_max
    step = delta_ticks
elif experiment_min <= present - delta_ticks <= experiment_max:
    action = -tau_max
    step = -delta_ticks
else:
    sys.exit(
        "metal_nudge_no_inbound_step:present=%s:delta=%s:cage=%s..%s"
        % (present, delta_ticks, experiment_min, experiment_max)
    )
# propose() re-acquires last_present. write_action does not clamp.
# The chosen sign must still fit after a hold-still hunt.
slack = 4
for p in (present, present - slack, present + slack):
    if p < experiment_min:
        p = experiment_min
    if p > experiment_max:
        p = experiment_max
    goal = p + step
    if goal < experiment_min or goal > experiment_max:
        sys.exit(
            "metal_nudge_eaten_by_present_slack:present=%s:p=%s:goal=%s:delta=%s:cage=%s..%s:slack=%s"
            % (present, p, goal, delta_ticks, experiment_min, experiment_max, slack)
        )
print("%g" % (action,))
sys.exit(0)
PY
}

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  set -euo pipefail
  tmp="$(mktemp -d)"
  trap 'rm -rf "$tmp"' EXIT
  # Mid-range cage: +32 fits. Keep the historical +0.2 campaign default.
  [[ "$(metal_nudge_action_from_values 2048 2000 2096 32 0.2)" == "0.2" ]]
  # Wizard leftover max == present (PTY AT_MAX). +32 refuses; -32 fits.
  [[ "$(metal_nudge_action_from_values 2048 2000 2048 32 0.2)" == "-0.2" ]]
  # Horn near 4095: experiment_max is the Position Mode stop.
  [[ "$(metal_nudge_action_from_values 4090 4042 4095 32 0.2)" == "-0.2" ]]
  # Horn near 0: +32 still fits the ±48 cage.
  [[ "$(metal_nudge_action_from_values 10 0 58 32 0.2)" == "0.2" ]]
  mkdir -p "$tmp/bus"
  printf '%s\n' '4090' >"$tmp/bus/present"
  printf '%s\n' '{"experiment_min":4042,"experiment_max":4095}' >"$tmp/bus/position_cage.json"
  printf '%s\n' '{"max_position_delta_ticks":32,"tau_max":0.2}' >"$tmp/metal.json"
  [[ "$(metal_nudge_action_from_bus "$tmp")" == "-0.2" ]]
  printf '%s\n' '2048' >"$tmp/bus/present"
  printf '%s\n' '{"experiment_min":2000,"experiment_max":2096}' >"$tmp/bus/position_cage.json"
  [[ "$(metal_nudge_action_from_bus "$tmp")" == "0.2" ]]
  if metal_nudge_action_from_values 2048 2048 2048 32 0.2 2>/dev/null; then
    echo "error: empty cage must not invent a nudge" >&2
    exit 1
  fi
  # Post-hold present 2044 in a 36-tick leftover max window: raw pick is
  # -0.2, but present-4 then misses and write_action abort-latches.
  printf '%s\n' '2044' >"$tmp/bus/present"
  printf '%s\n' '{"experiment_min":2012,"experiment_max":2048}' >"$tmp/bus/position_cage.json"
  if metal_nudge_action_from_bus "$tmp" 2>/dev/null; then
    echo "error: 36-tick leftover after hold must not propose -0.2" >&2
    exit 1
  fi
  echo "metal-nudge-action-ok"
fi
