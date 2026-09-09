#!/usr/bin/env bash
# Prepare two-UID metal directories. Does not invent OS users.
# Root is outside the threat model.

set -euo pipefail

AUTHORITY_USER="${REALITYOS_AUTHORITY_USER:-realityos-authority}"
AUTONOMY_USER="${REALITYOS_AUTONOMY_USER:-realityos-autonomy}"
IPC_GROUP="${REALITYOS_IPC_GROUP:-realityos-ipc}"
ROOT="${REALITYOS_METAL_ROOT:-/tmp/realityos-metal}"
DEVICE="${REALITYOS_METAL_DEVICE:-}"

if ! id -u "$AUTHORITY_USER" >/dev/null 2>&1 || ! id -u "$AUTONOMY_USER" >/dev/null 2>&1; then
  echo "error: OS users $AUTHORITY_USER / $AUTONOMY_USER missing (exit 2; do not fake)" >&2
  exit 2
fi
if ! getent group "$IPC_GROUP" >/dev/null 2>&1; then
  echo "error: group $IPC_GROUP missing" >&2
  exit 2
fi

install -d -m 0750 -o "$AUTHORITY_USER" -g "$AUTHORITY_USER" "$ROOT"
install -d -m 0700 -o "$AUTHORITY_USER" -g "$AUTHORITY_USER" "$ROOT/bus"

if [[ ! -f "$ROOT/signing.key" ]]; then
  dd if=/dev/urandom of="$ROOT/signing.key" bs=32 count=1 status=none
fi
chown "$AUTHORITY_USER:$AUTHORITY_USER" "$ROOT/signing.key"
chmod 0600 "$ROOT/signing.key"

if [[ -n "$DEVICE" && -e "$DEVICE" ]]; then
  chown "$AUTHORITY_USER:$AUTHORITY_USER" "$DEVICE" 2>/dev/null || true
  chmod 0600 "$DEVICE" 2>/dev/null || true
  if id -nG "$AUTHORITY_USER" | grep -qw dialout; then
    :
  else
    echo "note: add $AUTHORITY_USER to dialout if device chmod/chown is refused" >&2
  fi
fi

echo "metal-deploy root=$ROOT device=${DEVICE:-unset}"
