#!/usr/bin/env bash
# Prepare a two-user HIL topology on a local Linux host.
#
# Threat model: ordinary (non-root) processes. Root prepares, then is outside
# the tested threat model.
#
# Exit 2 if users/group are missing (do not fake success).

set -euo pipefail

AUTHORITY_USER="${REALITYOS_AUTHORITY_USER:-realityos-authority}"
AUTONOMY_USER="${REALITYOS_AUTONOMY_USER:-realityos-autonomy}"
IPC_GROUP="${REALITYOS_IPC_GROUP:-realityos-ipc}"
ROOT="${REALITYOS_HIL_ROOT:-/tmp/realityos-hil-os}"

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  cat <<EOF
Usage: sudo REALITYOS_HIL_ROOT=/var/lib/realityos-hil $0

Creates/uses two OS users and a shared IPC group:

  ${AUTHORITY_USER}  owns journal, seal, signing.key, bus/, ipc.sock
  ${AUTONOMY_USER}   may connect to ipc.sock only (group ${IPC_GROUP})
  ${IPC_GROUP}       0660 on the Unix socket; nothing else

Filesystem signing.key is not a hardware root of trust.

Then:
  sudo -u ${AUTHORITY_USER} -g ${IPC_GROUP} hil-authority --root \$ROOT --first-online --production serve
  sudo -u ${AUTONOMY_USER}  hil-untrusted --root \$ROOT --verb hold --id p1

Root is outside this threat model.
EOF
  exit 0
fi

if [[ "$(id -u)" -ne 0 ]]; then
  echo "error: this harness must run as root to switch users; root is still outside the threat model" >&2
  exit 2
fi

if ! getent group "$IPC_GROUP" >/dev/null 2>&1; then
  echo "error: missing group $IPC_GROUP (create it; do not fake multi-user success)" >&2
  exit 2
fi
if ! id -u "$AUTHORITY_USER" >/dev/null 2>&1; then
  echo "error: missing user $AUTHORITY_USER (create it; do not fake multi-user success)" >&2
  exit 2
fi
if ! id -u "$AUTONOMY_USER" >/dev/null 2>&1; then
  echo "error: missing user $AUTONOMY_USER (create it; do not fake multi-user success)" >&2
  exit 2
fi

if ! id -nG "$AUTHORITY_USER" | tr ' ' '\n' | grep -qx "$IPC_GROUP"; then
  echo "error: $AUTHORITY_USER is not in $IPC_GROUP" >&2
  exit 2
fi
if ! id -nG "$AUTONOMY_USER" | tr ' ' '\n' | grep -qx "$IPC_GROUP"; then
  echo "error: $AUTONOMY_USER is not in $IPC_GROUP" >&2
  exit 2
fi

rm -rf "$ROOT"
mkdir -p "$ROOT/bus"
install -d -m 0751 -o "$AUTHORITY_USER" -g "$AUTHORITY_USER" "$ROOT"
install -d -m 0700 -o "$AUTHORITY_USER" -g "$AUTHORITY_USER" "$ROOT/bus"

# Authority-owned signing material. Not TPM/HSM.
dd if=/dev/urandom of="$ROOT/signing.key" bs=32 count=1 status=none
chown "$AUTHORITY_USER:$AUTHORITY_USER" "$ROOT/signing.key"
chmod 0600 "$ROOT/signing.key"

echo "prepared $ROOT"
echo "authority user: $AUTHORITY_USER (owns socket, key, journal, actuator endpoint)"
echo "autonomy user:  $AUTONOMY_USER (proposal IPC only, group $IPC_GROUP)"
echo "ipc group:      $IPC_GROUP (socket 0660)"
echo "signing.key is filesystem storage, not a hardware root of trust."
echo "root is outside this threat model."
