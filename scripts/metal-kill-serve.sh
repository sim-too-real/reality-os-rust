#!/usr/bin/env bash
# Kill leftover realityos-metal-smoke for one metal root.
# A zombie serve rewrites journal+seal after rm -rf and then --first-online refuses.
# Matches the --root argv exactly so /tmp/realityos-metal does not kill ...-ipc.

set -euo pipefail

root="${1:-}"
if [[ -z "$root" || "$root" == "/" || "$root" == "/tmp" || "$root" == "/var" ]]; then
  echo "error: refuse to kill metal-smoke for root='$root'" >&2
  exit 2
fi

matches_root() {
  local cmdline="$1"
  local rest="${cmdline##*realityos-metal-smoke --root }"
  if [[ "$rest" == "$cmdline" ]]; then
    return 1
  fi
  [[ "$rest" == "${root} "* || "$rest" == "${root}" ]]
}

kill_matching() {
  local pid cmdline
  while read -r pid cmdline; do
    if matches_root "$cmdline"; then
      kill -9 "$pid" 2>/dev/null || true
    fi
  done < <(pgrep -af 'realityos-metal-smoke' 2>/dev/null || true)
}

kill_matching
for _ in $(seq 1 40); do
  still=0
  while read -r pid cmdline; do
    if matches_root "$cmdline"; then
      still=1
      kill -9 "$pid" 2>/dev/null || true
    fi
  done < <(pgrep -af 'realityos-metal-smoke' 2>/dev/null || true)
  if [[ "$still" -eq 0 ]]; then
    exit 0
  fi
  sleep 0.05
done
exit 0
