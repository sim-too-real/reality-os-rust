#!/usr/bin/env bash
# GNU coreutils `stat -c '%a'` prints 600 for mode 0600 (no leading 0).
# Comparing the raw string to "0600" fails after a correct chmod and
# used to abort every real USB-UART owner claim.

unix_mode_eq() {
  local got="${1:-}"
  local want="${2:-}"
  [[ -n "$got" && -n "$want" ]] || return 1
  [[ $((8#$got)) -eq $((8#$want)) ]]
}

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  set -euo pipefail
  unix_mode_eq 600 0600
  unix_mode_eq 0600 600
  unix_mode_eq 644 0644
  if unix_mode_eq 666 0600; then
    echo "error: 666 must not equal 0600" >&2
    exit 1
  fi
  if unix_mode_eq "" 0600; then
    echo "error: empty mode must not match" >&2
    exit 1
  fi
  tmp="$(mktemp)"
  chmod 0600 "$tmp"
  unix_mode_eq "$(stat -c '%a' "$tmp")" 0600
  rm -f "$tmp"
  echo "metal-unix-mode-ok"
fi
