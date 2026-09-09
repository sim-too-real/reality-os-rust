#!/usr/bin/env bash
# Deploy a two-user HIL topology on a local Linux host.
#
# Threat model: ordinary (non-root) processes. Root is outside this threat model.
#
# This script is a documented harness, not a unit-test guarantee. GitHub Actions
# cannot reliably exercise distinct UIDs. Do not treat a green unit test as
# proof of OS identity separation.
#
# Requirements:
#   * run as root (or a user who can sudo -u)
#   * users REALITYOS_AUTHORITY_USER (default: realityos-authority)
#   * users REALITYOS_AUTONOMY_USER  (default: realityos-autonomy)
#   * REALITYOS_HIL_ROOT directory
#
# Authority user owns: Unix socket, signing key, journal+seal, actuator endpoint.
# Autonomy user may connect to the proposal IPC only.

set -euo pipefail

AUTHORITY_USER="${REALITYOS_AUTHORITY_USER:-realityos-authority}"
AUTONOMY_USER="${REALITYOS_AUTONOMY_USER:-realityos-autonomy}"
ROOT="${REALITYOS_HIL_ROOT:-/tmp/realityos-hil-os}"
BIN_DIR="${REALITYOS_HIL_BIN:-}"

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  cat <<EOF
Usage: sudo REALITYOS_HIL_ROOT=/var/lib/realityos-hil $0

Creates/uses two OS users and a root directory:

  ${AUTHORITY_USER}  owns journal, seal, signing key, bus/, ipc.sock
  ${AUTONOMY_USER}   may connect to ipc.sock only

Then:
  sudo -u ${AUTHORITY_USER} hil-authority --root \$ROOT --first-online serve
  sudo -u ${AUTONOMY_USER}  hil-untrusted --root \$ROOT --verb hold --id p1

Autonomy must fail to:
  * read \$ROOT/signing.key
  * write \$ROOT/driver.jsonl
  * open \$ROOT/bus/actuator.log (mode 000)
  * chmod/chown authority files (not root)

Exit 2 if users are missing (do not fake success).
EOF
  exit 0
fi

if [[ "$(id -u)" -ne 0 ]]; then
  echo "error: this harness must run as root to switch users; root is still outside the threat model" >&2
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

mkdir -p "$ROOT/bus"
install -d -m 0750 -o "$AUTHORITY_USER" -g "$AUTHORITY_USER" "$ROOT"
install -d -m 0700 -o "$AUTHORITY_USER" -g "$AUTHORITY_USER" "$ROOT/bus"
install -m 0600 -o "$AUTHORITY_USER" -g "$AUTHORITY_USER" /dev/null "$ROOT/signing.key" || true
# IPC socket is created by hil-authority; directory must be traversable by autonomy.
chmod 0751 "$ROOT"
chown "$AUTHORITY_USER:$AUTHORITY_USER" "$ROOT"

echo "prepared $ROOT"
echo "authority user: $AUTHORITY_USER (owns socket, key, journal, actuator endpoint)"
echo "autonomy user:  $AUTONOMY_USER (proposal IPC only)"
echo "root is outside this threat model."
if [[ -n "$BIN_DIR" ]]; then
  echo "start: sudo -u $AUTHORITY_USER $BIN_DIR/hil-authority --root $ROOT --first-online serve"
fi
