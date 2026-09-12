#!/usr/bin/env bash
# Decide --first-online vs --restart from the durable ONLINE journal.
#
# Keep in sync with crates/metal/src/config.rs JOURNAL and
# crates/plant/src/ledger.rs seal_path_for / with_online_journal.
# first_online=true refuses first_online_but_journal_or_seal_exists.
# A DTR-RESET bind that is not yet live already created both; retrying
# --first-online aborts first contact. HIL crash-replay already flips
# to --restart when driver.jsonl exists.

METAL_ONLINE_JOURNAL_NAME="driver.jsonl"
METAL_ONLINE_SEAL_SUFFIX=".authority-seal"

metal_online_journal_exists() {
  local root="${1:-}"
  [[ -n "$root" ]] || return 1
  [[ -f "$root/$METAL_ONLINE_JOURNAL_NAME" || -f "$root/${METAL_ONLINE_JOURNAL_NAME}${METAL_ONLINE_SEAL_SUFFIX}" ]]
}

# Echo 1 for --first-online, 0 for --restart.
# requested is the campaign's intended flag for this serve; an existing
# journal/seal forces 0 so start_online can resume instead of refuse.
metal_serve_first_online_flag() {
  local requested="${1:-1}"
  local root="${2:-}"
  if metal_online_journal_exists "$root"; then
    printf '%s\n' 0
    return
  fi
  printf '%s\n' "$requested"
}

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  set -euo pipefail
  tmp="$(mktemp -d)"
  trap 'rm -rf "$tmp"' EXIT
  if metal_online_journal_exists "$tmp"; then
    echo "error: empty root must not look like an ONLINE journal" >&2
    exit 1
  fi
  if [[ "$(metal_serve_first_online_flag 1 "$tmp")" != "1" ]]; then
    echo "error: first serve without journal must keep --first-online" >&2
    exit 1
  fi
  if [[ "$(metal_serve_first_online_flag 0 "$tmp")" != "0" ]]; then
    echo "error: explicit --restart without journal must stay 0" >&2
    exit 1
  fi
  printf '%s\n' '{}' >"$tmp/$METAL_ONLINE_JOURNAL_NAME"
  if ! metal_online_journal_exists "$tmp"; then
    echo "error: driver.jsonl must count as an ONLINE journal" >&2
    exit 1
  fi
  if [[ "$(metal_serve_first_online_flag 1 "$tmp")" != "0" ]]; then
    echo "error: existing journal must force --restart (first_online_but_journal_or_seal_exists)" >&2
    exit 1
  fi
  rm -f "$tmp/$METAL_ONLINE_JOURNAL_NAME"
  printf '%s\n' '{}' >"$tmp/${METAL_ONLINE_JOURNAL_NAME}${METAL_ONLINE_SEAL_SUFFIX}"
  if ! metal_online_journal_exists "$tmp"; then
    echo "error: authority-seal alone must count (ledger refuses first_online either way)" >&2
    exit 1
  fi
  if [[ "$(metal_serve_first_online_flag 1 "$tmp")" != "0" ]]; then
    echo "error: existing seal must force --restart" >&2
    exit 1
  fi
  echo "metal-online-journal-ok"
fi
