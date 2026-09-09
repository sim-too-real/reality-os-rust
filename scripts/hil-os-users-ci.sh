#!/usr/bin/env bash
# Create the two HIL OS users (if missing) and run the measurable UID test.
# Intended for Linux CI / local root. Do not treat cargo unit tests as this proof.

set -euo pipefail

AUTHORITY_USER="${REALITYOS_AUTHORITY_USER:-realityos-authority}"
AUTONOMY_USER="${REALITYOS_AUTONOMY_USER:-realityos-autonomy}"
IPC_GROUP="${REALITYOS_IPC_GROUP:-realityos-ipc}"

if [[ "$(id -u)" -ne 0 ]]; then
  echo "error: $0 must run as root (sudo) so it can create users and switch UIDs" >&2
  exit 2
fi

if ! getent group "$IPC_GROUP" >/dev/null 2>&1; then
  groupadd --system "$IPC_GROUP"
fi
if ! id -u "$AUTHORITY_USER" >/dev/null 2>&1; then
  useradd --system --no-create-home --shell /bin/bash -G "$IPC_GROUP" "$AUTHORITY_USER"
fi
if ! id -u "$AUTONOMY_USER" >/dev/null 2>&1; then
  useradd --system --no-create-home --shell /bin/bash -G "$IPC_GROUP" "$AUTONOMY_USER"
fi
usermod -aG "$IPC_GROUP" "$AUTHORITY_USER"
usermod -aG "$IPC_GROUP" "$AUTONOMY_USER"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$REPO"
if [[ -z "${REALITYOS_HIL_BIN:-}" ]]; then
  if [[ "$(id -u)" -eq 0 ]]; then
    echo "error: build binaries as a non-root user, then set REALITYOS_HIL_BIN" >&2
    echo "  cargo build -p realityos-hil --bins" >&2
    echo "  sudo REALITYOS_HIL_BIN=\$PWD/target/debug $0" >&2
    exit 2
  fi
  cargo build -p realityos-hil --bins
  export REALITYOS_HIL_BIN="$REPO/target/debug"
fi
export REALITYOS_HIL_ROOT="${REALITYOS_HIL_ROOT:-/tmp/realityos-hil-os}"
export REALITYOS_OS_USERS_PROOF="${REALITYOS_OS_USERS_PROOF:-$REPO/docs/hil_os_users_proof.json}"
"$SCRIPT_DIR/hil-os-users-test.sh"
