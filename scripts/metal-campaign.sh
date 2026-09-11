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
CUTOFF_TESTED="${REALITYOS_METAL_CUTOFF_TESTED:-0}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$SCRIPT_DIR/.." && pwd)"
# shellcheck source=metal-unix-mode.sh
source "$SCRIPT_DIR/metal-unix-mode.sh"
# Authority UID cannot write the repo `docs/` tree. Resolve against the
# script's repo, not `$PWD`: `sudo ... /path/scripts/metal-campaign.sh`
# from $HOME used to install ~/docs/metal_proof.json after a live run.
OUT="${REALITYOS_METAL_PROOF:-$REPO/docs/metal_proof.json}"
if [[ "$OUT" != /* ]]; then
  OUT="$REPO/$OUT"
fi

if [[ "$(id -u)" -ne 0 ]]; then
  echo "error: run as root to switch UIDs; root is not the tested actor" >&2
  exit 2
fi
# The documented bench command does not mention HIL CI. A host that
# has the XL330 but never ran hil-os-users-ci used to exit 2 in
# metal-deploy before probe. Create the same system users/group here
# (do not invent UIDs for the proof — these are real OS accounts).
# nscd/sssd can keep a negative "user not found" after useradd.
# udev OWNER= and chown then fail; MODE stays 0660 dialout and
# authority gets EACCES at probe (same class as creating the
# accounts after first USB prepare).
flush_name_service_cache() {
  if command -v nscd >/dev/null 2>&1; then
    nscd -i passwd >/dev/null 2>&1 || true
    nscd -i group >/dev/null 2>&1 || true
  fi
  if command -v sss_cache >/dev/null 2>&1; then
    sss_cache -U >/dev/null 2>&1 || true
    sss_cache -G >/dev/null 2>&1 || true
  fi
}

ensure_metal_os_users() {
  if ! command -v groupadd >/dev/null 2>&1 || ! command -v useradd >/dev/null 2>&1 || ! command -v usermod >/dev/null 2>&1; then
    echo "error: groupadd/useradd/usermod not found; create $AUTHORITY_USER / $AUTONOMY_USER and group $IPC_GROUP" >&2
    return 1
  fi
  if ! getent group "$IPC_GROUP" >/dev/null 2>&1; then
    groupadd --system "$IPC_GROUP"
    echo "metal-campaign: created system group $IPC_GROUP"
  fi
  if ! id -u "$AUTHORITY_USER" >/dev/null 2>&1; then
    useradd --system --no-create-home --shell /bin/bash -G "$IPC_GROUP" "$AUTHORITY_USER"
    echo "metal-campaign: created system user $AUTHORITY_USER"
  fi
  if ! id -u "$AUTONOMY_USER" >/dev/null 2>&1; then
    useradd --system --no-create-home --shell /bin/bash -G "$IPC_GROUP" "$AUTONOMY_USER"
    echo "metal-campaign: created system user $AUTONOMY_USER"
  fi
  usermod -aG "$IPC_GROUP" "$AUTHORITY_USER"
  usermod -aG "$IPC_GROUP" "$AUTONOMY_USER"
  flush_name_service_cache
  if ! getent passwd "$AUTHORITY_USER" >/dev/null 2>&1 \
    || ! getent passwd "$AUTONOMY_USER" >/dev/null 2>&1 \
    || ! getent group "$IPC_GROUP" >/dev/null 2>&1; then
    echo "error: metal OS users/group are not visible to getent (nscd/sssd cache?). udev OWNER= would fail." >&2
    return 1
  fi
  local probe
  probe="$(mktemp)"
  if ! chown "$AUTHORITY_USER:$AUTHORITY_USER" "$probe" 2>/dev/null; then
    rm -f "$probe"
    echo "error: chown $AUTHORITY_USER failed; udev OWNER= cannot resolve that user" >&2
    return 1
  fi
  rm -f "$probe"
}

# USB-serial only. A silent chown miss used to leave 0660 dialout;
# authority is not in dialout, so probe open is EACCES after prepare.
# Inode uid+mode, not NSS %U — a stale nscd name can hide a successful
# chown the same way OWNER= missed a just-created user.
usb_tty_owner_mode_ok() {
  local real="$1"
  local want_uid got_uid mode
  want_uid="$(id -u "$AUTHORITY_USER")"
  got_uid="$(stat -c '%u' "$real" 2>/dev/null || true)"
  mode="$(stat -c '%a' "$real" 2>/dev/null || true)"
  [[ "$got_uid" == "$want_uid" ]] && unix_mode_eq "$mode" 0600
}

# Live /dev/ttyUSB* (or ACM/CH341) after resolving by-id / by-path.
# A dangling /dev/serial/by-id symlink used to make basename look like
# usb-FTDI_... so claim/latency/settle treated the node as PTY/GPIO and
# skipped latency_timer=1 — first hold then missed the 40 ms deadline.
usb_serial_resolved_real() {
  local dev="$1"
  local real name
  real="$(readlink -f "$dev" 2>/dev/null || true)"
  name="$(basename "${real:-}")"
  case "$name" in
    ttyUSB* | ttyACM* | ttyCH341*)
      # A dangling by-id can readlink to a vanished or recycled
      # ttyUSB0. That is not a live UART.
      if [[ -e "$real" ]]; then
        echo "$real"
        return 0
      fi
      ;;
  esac
  return 1
}

# Campaign DEVICE that must stay on the USB fail-closed path even when
# the current string is a udev symlink, not ttyUSB0.
usb_serial_must_resolve() {
  local dev="$1"
  case "$dev" in
    /dev/serial/by-id/* | /dev/serial/by-path/*) return 0 ;;
  esac
  [[ -n "${METAL_USB_SERIAL:-}" || -n "${METAL_USB_PORT:-}" ]]
}

claim_usb_tty() {
  local dev="$1"
  local real
  if ! real="$(usb_serial_resolved_real "$dev")"; then
    if usb_serial_must_resolve "$dev"; then
      echo "error: $dev did not resolve to a live USB-serial tty; refuse to skip owner/mode claim" >&2
      return 1
    fi
    return 0
  fi
  if [[ ! -e "$real" ]]; then
    echo "error: USB-UART $dev vanished before owner/mode claim" >&2
    return 1
  fi
  # A no-op chown still emits udev change on typical Ubuntu. That used
  # to wake ModemManager / reset FTDI latency_timer / rename ttyUSB0
  # immediately before probe opened the UART.
  if usb_tty_owner_mode_ok "$real"; then
    return 0
  fi
  if ! chown "$AUTHORITY_USER:$AUTHORITY_USER" "$real"; then
    echo "error: chown $AUTHORITY_USER $real failed (NSS/udev). Authority cannot open the UART." >&2
    return 1
  fi
  if ! chmod 0600 "$real"; then
    echo "error: chmod 0600 $real failed" >&2
    return 1
  fi
  if ! usb_tty_owner_mode_ok "$real"; then
    echo "error: $real is uid=$(stat -c '%u' "$real" 2>/dev/null || true) mode=$(stat -c '%a' "$real" 2>/dev/null || true) after claim (want uid=$(id -u "$AUTHORITY_USER") 0600). udev/NSS did not stick." >&2
    return 1
  fi
}
# Root-created files default to owner-only. `report` runs as the
# authority UID and must read proof_meta / cases. A hardened umask
# 0077 used to abort mint after the physical campaign had already run.
umask 0077
authority_readable() {
  local f="$1"
  chmod 0644 "$f"
  chown "$AUTHORITY_USER:$AUTHORITY_USER" "$f"
}
# Docs set REALITYOS_METAL_BIN=$PWD/target/debug. The same
# `sudo /path/scripts/metal-campaign.sh` from $HOME that used to
# mint ~/docs also points BIN at ~/target/debug. Prefer the env
# path when it has both binaries; otherwise use the script's repo.
resolve_metal_bin() {
  local candidates=()
  if [[ -n "${BIN_DIR:-}" ]]; then
    candidates+=("$BIN_DIR")
  fi
  candidates+=("$REPO/target/debug" "$REPO/target/release")
  local d
  for d in "${candidates[@]}"; do
    if [[ -x "$d/realityos-metal-smoke" && -x "$d/realityos-metal-propose" ]]; then
      if [[ -n "${BIN_DIR:-}" && "$d" != "$BIN_DIR" ]]; then
        echo "metal-campaign: $BIN_DIR has no metal binaries; using $d (script repo, not cwd)" >&2
      fi
      BIN_DIR="$d"
      return 0
    fi
  done
  return 1
}
if ! resolve_metal_bin; then
  echo "error: metal binaries not found. Set REALITYOS_METAL_BIN or: cargo build -p realityos-metal --bins" >&2
  echo "error: looked in ${BIN_DIR:-<unset>} $REPO/target/debug $REPO/target/release" >&2
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
PTY_SEQUENCE_ACTIVE=0
if [[ "$DEVICE_REAL" == /dev/pts/* ]]; then
  if [[ "${REALITYOS_METAL_PTY_SEQUENCE:-0}" != "1" ]]; then
    echo "error: refusing PTY $DEVICE_REAL; not a physical actuator. Will not write metal_proof.json." >&2
    exit 2
  fi
  echo "metal-campaign: PTY sequence only; not metal evidence; will not install docs/metal_proof.json"
  PTY_SEQUENCE_ACTIVE=1
  CUTOFF_TESTED=0
  export REALITYOS_METAL_CUTOFF_TESTED=0
  export REALITYOS_METAL_ALLOW_PTY=1
fi
# probe identifies (SRL poke only; no EEPROM, no torque). serve enables
# torque; hold/nudge write goal_position. Do not touch the servo until
# the operator has opened VIN and seen lost holding torque.
if [[ "$PTY_SEQUENCE_ACTIVE" != "1" && "$CUTOFF_TESTED" != "1" ]]; then
  echo "error: refuse to torque or command the XL330 before the independent VIN cutoff is operator-tested." >&2
  echo "error: open the VIN disconnect, confirm lost holding torque (USB/data may stay enumerated), then REALITYOS_METAL_CUTOFF_TESTED=1." >&2
  echo "error: that cutoff is not STO/SS1/PL/SIL unless the hardware's own documentation says it is." >&2
  exit 2
fi
if [[ -z "$ROOT" || "$ROOT" == "/" || "$ROOT" == "/tmp" || "$ROOT" == "/var" ]]; then
  echo "error: refusing to wipe unexpected REALITYOS_METAL_ROOT=$ROOT" >&2
  exit 2
fi
# FTDI/U2D2 defaults latency_timer to 16 ms. Two waits miss the 40 ms live
# I/O deadline and latch the software watchdog on the first real USB-UART.
usb_tty_latency_timer_path() {
  local name="$1" timer
  for timer in \
    "/sys/bus/usb-serial/devices/${name}/latency_timer" \
    "/sys/class/tty/${name}/device/latency_timer"; do
    if [[ -e "$timer" ]]; then
      echo "$timer"
      return 0
    fi
  done
  return 1
}

# CH340/ch341 often has no latency_timer (ok). FTDI/U2D2 must read 1.
# A vanished tty or dangling by-id has no timer file too — that is not
# CH340 success (the first hold then ran at the kernel 16 ms default).
usb_tty_latency_ok() {
  local dev="$1" name timer got real
  real="$(readlink -f "$dev" 2>/dev/null || true)"
  name="$(basename "${real:-}")"
  case "$name" in
    ttyUSB* | ttyACM* | ttyCH341*) ;;
    *) return 1 ;;
  esac
  [[ -e "$real" ]] || return 1
  timer="$(usb_tty_latency_timer_path "$name" || true)"
  [[ -z "$timer" ]] && return 0
  got="$(tr -d '[:space:]' <"$timer" 2>/dev/null || true)"
  [[ "$got" == "1" ]]
}

set_usb_serial_latency() {
  local dev="$1"
  local real name timer got
  if ! real="$(usb_serial_resolved_real "$dev")"; then
    if usb_serial_must_resolve "$dev"; then
      echo "error: $dev did not resolve to a live USB-serial tty; refuse to skip latency_timer=1" >&2
      return 1
    fi
    return 0
  fi
  name="$(basename "$real")"
  if usb_tty_latency_ok "$real"; then
    return 0
  fi
  timer="$(usb_tty_latency_timer_path "$name" || true)"
  if [[ -z "$timer" ]]; then
    return 0
  fi
  if echo 1 >"$timer" 2>/dev/null; then
    got="$(tr -d '[:space:]' <"$timer" 2>/dev/null || true)"
    if [[ "$got" == "1" ]]; then
      echo "metal-campaign: set $timer=1 (USB-UART default 16 ms can miss the 40 ms live deadline)"
      return 0
    fi
  fi
  # CH340/ch341 often has no latency_timer; skip. FTDI/U2D2 always has
  # the file at 16 ms — a write that does not stick used to continue
  # and miss the 40 ms live deadline on the first hold.
  echo "error: USB-UART latency_timer exists but is not 1 after write; default 16 ms misses the 40 ms live deadline" >&2
  return 1
}

# Ubuntu usbcore autosuspend is often 2 s. An idle gap between campaign
# cases then makes the next USB-UART xfer miss the 40 ms live deadline
# and look like bus_lost / a watchdog miss. PTY has no sysfs node.
# A write without read-back used to continue on `auto` (same class as
# latency_timer). Stop at the UART device (idVendor+idProduct); do not
# climb to a hub and treat that as success.
usb_tty_uart_power_node() {
  local name="$1" node
  node="$(readlink -f "/sys/class/tty/${name}/device" 2>/dev/null || true)"
  while [[ -n "$node" && "$node" != / && "$node" != /sys ]]; do
    if [[ -f "$node/idVendor" ]]; then
      if [[ -f "$node/idProduct" && -f "$node/power/control" ]]; then
        echo "$node"
        return 0
      fi
      return 1
    fi
    node="$(dirname "$node")"
  done
  return 1
}

usb_tty_power_ok() {
  local dev="$1" name uart got
  name="$(basename "$(readlink -f "$dev" 2>/dev/null || echo "$dev")")"
  uart="$(usb_tty_uart_power_node "$name" || true)"
  [[ -n "$uart" ]] || return 1
  got="$(tr -d '[:space:]' <"$uart/power/control" 2>/dev/null || true)"
  [[ "$got" == "on" ]]
}

disable_usb_autosuspend() {
  local dev="$1"
  local real name uart="" got
  if ! real="$(usb_serial_resolved_real "$dev")"; then
    if usb_serial_must_resolve "$dev"; then
      echo "error: $dev did not resolve to a live USB-serial tty; refuse to skip power/control=on" >&2
      return 1
    fi
    return 0
  fi
  name="$(basename "$real")"
  uart="$(usb_tty_uart_power_node "$name" || true)"
  if [[ -z "$uart" || ! -f "$uart/power/control" ]]; then
    echo "error: USB-UART $dev has no UART-device power/control; refuse default autosuspend" >&2
    return 1
  fi
  if usb_tty_power_ok "$real"; then
    return 0
  fi
  echo on >"$uart/power/control" 2>/dev/null || true
  if [[ -f "$uart/power/autosuspend_delay_ms" ]]; then
    echo -1 >"$uart/power/autosuspend_delay_ms" 2>/dev/null || true
  fi
  got="$(tr -d '[:space:]' <"$uart/power/control" 2>/dev/null || true)"
  if [[ "$got" != "on" ]]; then
    echo "error: USB-UART power/control is '${got:-unreadable}' after write (want on); autosuspend can miss the 40 ms live deadline" >&2
    return 1
  fi
  echo "metal-campaign: set $uart/power/control=on (USB autosuspend can miss the 40 ms live deadline)"
}

# Walk sysfs from the tty to the UART USB device (first
# idVendor+idProduct) and read one attribute from that node only.
# A CH340/CP2102 often has an empty `serial` file; that used to be
# exit 0 so wait never required dest. A node with idVendor and no
# idProduct used to climb to a parent hub and inherit that serial —
# ATTRS{serial}==<hub> then matches every tty on the hub.
usb_sysfs_value() {
  local dev="$1" key="$2"
  local name node val
  name="$(basename "$(readlink -f "$dev" 2>/dev/null || echo "$dev")")"
  node="$(readlink -f "/sys/class/tty/${name}/device" 2>/dev/null || true)"
  while [[ -n "$node" && "$node" != / && "$node" != /sys ]]; do
    if [[ -f "$node/idVendor" ]]; then
      if [[ ! -f "$node/idProduct" ]]; then
        return 1
      fi
      if [[ -f "$node/$key" ]]; then
        val="$(tr -d '\n' <"$node/$key")"
        val="${val#"${val%%[![:space:]]*}"}"
        val="${val%"${val##*[![:space:]]}"}"
        if [[ -n "$val" ]]; then
          printf '%s' "$val"
          return 0
        fi
      fi
      return 1
    fi
    node="$(dirname "$node")"
  done
  return 1
}

# busnum:devpath:idVendor:idProduct. CH340/CP2102 often have an empty
# USB serial; udev change can still keep this port key. Do not invent
# 0:nodevpath — probe would bind usb:vid:pid:nodevpath and serve would
# miss when the real dest showed up.
usb_sysfs_port_key() {
  local dev="$1"
  local bus dest vid pid
  bus="$(usb_sysfs_value "$dev" busnum || true)"
  dest="$(usb_sysfs_value "$dev" devpath || true)"
  vid="$(usb_sysfs_value "$dev" idVendor || true)"
  pid="$(usb_sysfs_value "$dev" idProduct || true)"
  if [[ -n "$bus" && -n "$dest" && -n "$vid" && -n "$pid" ]]; then
    echo "${bus}:${dest}:${vid}:${pid}"
    return 0
  fi
  return 1
}

# udev change / MM stop can re-enumerate FTDI as ttyUSB1. A KERNEL==ttyUSB0
# rule and a stale DEVICE then miss the servo. Prefer /dev/serial/by-id,
# else by-path (CH340 often has no by-id), else the tty whose USB serial
# or port key still matches.
find_tty_by_usb_serial() {
  local want="$1"
  local p real got
  [[ -n "$want" ]] || return 1
  if [[ -d /dev/serial/by-id ]]; then
    for p in /dev/serial/by-id/*; do
      [[ -e "$p" ]] || continue
      real="$(readlink -f "$p" 2>/dev/null || true)"
      [[ -n "$real" ]] || continue
      got="$(usb_sysfs_value "$real" serial || true)"
      if [[ "$got" == "$want" ]]; then
        echo "$p"
        return 0
      fi
    done
  fi
  if [[ -d /dev/serial/by-path ]]; then
    for p in /dev/serial/by-path/*; do
      [[ -e "$p" ]] || continue
      real="$(readlink -f "$p" 2>/dev/null || true)"
      [[ -n "$real" ]] || continue
      got="$(usb_sysfs_value "$real" serial || true)"
      if [[ "$got" == "$want" ]]; then
        echo "$p"
        return 0
      fi
    done
  fi
  for p in /dev/ttyUSB* /dev/ttyACM* /dev/ttyCH341*; do
    [[ -e "$p" ]] || continue
    got="$(usb_sysfs_value "$p" serial || true)"
    if [[ "$got" == "$want" ]]; then
      echo "$p"
      return 0
    fi
  done
  return 1
}

find_tty_by_usb_port() {
  local want="$1"
  local p real got
  [[ -n "$want" ]] || return 1
  if [[ -d /dev/serial/by-path ]]; then
    for p in /dev/serial/by-path/*; do
      [[ -e "$p" ]] || continue
      real="$(readlink -f "$p" 2>/dev/null || true)"
      [[ -n "$real" ]] || continue
      got="$(usb_sysfs_port_key "$real" || true)"
      if [[ "$got" == "$want" ]]; then
        echo "$p"
        return 0
      fi
    done
  fi
  if [[ -d /dev/serial/by-id ]]; then
    for p in /dev/serial/by-id/*; do
      [[ -e "$p" ]] || continue
      real="$(readlink -f "$p" 2>/dev/null || true)"
      [[ -n "$real" ]] || continue
      got="$(usb_sysfs_port_key "$real" || true)"
      if [[ "$got" == "$want" ]]; then
        echo "$p"
        return 0
      fi
    done
  fi
  for p in /dev/ttyUSB* /dev/ttyACM* /dev/ttyCH341*; do
    [[ -e "$p" ]] || continue
    got="$(usb_sysfs_port_key "$p" || true)"
    if [[ "$got" == "$want" ]]; then
      echo "$p"
      return 0
    fi
  done
  return 1
}

# Recorded USB serial/port must match this node. A living ttyUSB0 after
# crash close / CH340 re-enum can be a different adapter on the same bench.
usb_device_matches_recorded() {
  local dev="$1"
  local got
  [[ -e "$dev" ]] || return 1
  if [[ -n "${METAL_USB_SERIAL:-}" ]]; then
    got="$(usb_sysfs_value "$dev" serial || true)"
    [[ "$got" == "$METAL_USB_SERIAL" ]]
    return
  fi
  if [[ -n "${METAL_USB_PORT:-}" ]]; then
    got="$(usb_sysfs_port_key "$dev" || true)"
    [[ "$got" == "$METAL_USB_PORT" ]]
    return
  fi
  return 0
}

stable_usb_symlink_for() {
  local real="$1"
  local p
  if [[ -d /dev/serial/by-id ]]; then
    for p in /dev/serial/by-id/*; do
      [[ -e "$p" ]] || continue
      if [[ "$(readlink -f "$p" 2>/dev/null || true)" == "$real" ]]; then
        echo "$p"
        return 0
      fi
    done
  fi
  # CH340/CP2102 usually have no USB serial, so by-id is missing. by-path
  # stays on the same USB port across ttyUSB0 → ttyUSB1.
  if [[ -d /dev/serial/by-path ]]; then
    for p in /dev/serial/by-path/*; do
      [[ -e "$p" ]] || continue
      if [[ "$(readlink -f "$p" 2>/dev/null || true)" == "$real" ]]; then
        echo "$p"
        return 0
      fi
    done
  fi
  echo "$real"
}

# Echo a live path whose USB identity matches the recorded adapter, or
# fail if only a vanished / recycled name is left.
resolve_recorded_usb_tty() {
  local dev="$1"
  local real name p chosen
  real="$(readlink -f "$dev" 2>/dev/null || echo "$dev")"
  name="$(basename "$real")"
  if [[ -e "$real" ]] && usb_device_matches_recorded "$real"; then
    chosen="$(stable_usb_symlink_for "$real")"
    if [[ "$chosen" != "$dev" ]]; then
      echo "metal-campaign: using stable $chosen (udev can rename $name)" >&2
    fi
    echo "$chosen"
    return 0
  fi
  if [[ -n "${METAL_USB_SERIAL:-}" ]]; then
    p="$(find_tty_by_usb_serial "$METAL_USB_SERIAL" || true)"
    if [[ -n "$p" ]]; then
      echo "metal-campaign: $real is missing or a different adapter; continuing on $p" >&2
      echo "$p"
      return 0
    fi
  fi
  if [[ -n "${METAL_USB_PORT:-}" ]]; then
    p="$(find_tty_by_usb_port "$METAL_USB_PORT" || true)"
    if [[ -n "$p" ]]; then
      echo "metal-campaign: $real is missing or a different adapter; continuing on $p (USB port $METAL_USB_PORT)" >&2
      echo "$p"
      return 0
    fi
  fi
  return 1
}

# First contact after plug-in / udev add: the tty node can exist before
# USB attributes. idVendor+idProduct alone can still lack busnum/devpath;
# the port key used to invent 0:nodevpath and probe bound
# usb:vid:pid:nodevpath, then serve missed when dest appeared. An empty
# CH340 serial file or a parent hub serial used to count as "serial
# present" so wait returned before the UART dest existed. dest can also
# appear before iSerial; Rust prefers serial, so binding dest then
# measuring FTDI/U2D2 serial on serve misses before hold.
wait_usb_sysfs_identity() {
  local dev="$1"
  local real name i j
  real="$(readlink -f "$dev" 2>/dev/null || true)"
  name="$(basename "${real:-}")"
  case "$name" in
    ttyUSB*|ttyACM*|ttyCH341*) ;;
    *)
      if usb_serial_must_resolve "$dev"; then
        echo "error: $dev did not resolve to a live USB-serial tty before sysfs identity wait" >&2
        return 1
      fi
      return 0
      ;;
  esac
  for i in $(seq 1 40); do
    if [[ -e "$real" ]] && {
      usb_sysfs_value "$real" serial >/dev/null \
        || usb_sysfs_port_key "$real" >/dev/null
    }; then
      if usb_sysfs_value "$real" serial >/dev/null; then
        return 0
      fi
      for j in $(seq 1 10); do
        sleep 0.05
        if usb_sysfs_value "$real" serial >/dev/null; then
          return 0
        fi
      done
      if usb_sysfs_port_key "$real" >/dev/null \
        || usb_sysfs_value "$real" serial >/dev/null; then
        return 0
      fi
    fi
    sleep 0.1
  done
  echo "warning: USB sysfs serial or busnum:devpath:vid:pid never appeared on the UART device for $real" >&2
  return 1
}

stabilize_metal_device() {
  local dev="$1"
  local real name i p
  real="$(readlink -f "$dev" 2>/dev/null || echo "$dev")"
  name="$(basename "$real")"
  case "$name" in
    ttyUSB*|ttyACM*|ttyCH341*) ;;
    *)
      # FTDI/U2D2 DEVICE is usually /dev/serial/by-id. crash_if close
      # can leave that symlink dangling for 1–3 s; basename is then
      # usb-FTDI_... and the old skip returned immediately — crash-replay
      # never waited, then the caller either opened at 16 ms or failed
      # closed without rematching the live tty.
      if ! usb_serial_must_resolve "$dev"; then
        echo "$dev"
        return 0
      fi
      ;;
  esac
  if p="$(resolve_recorded_usb_tty "$dev")"; then
    echo "$p"
    return 0
  fi
  # crash_if / CH340 close can drop the USB device for 1–3 s. Five
  # immediate serve retries lose that race and reopen a recycled ttyUSB0.
  if [[ -n "${METAL_USB_SERIAL:-}" || -n "${METAL_USB_PORT:-}" ]]; then
    for i in $(seq 1 40); do
      sleep 0.1
      if p="$(resolve_recorded_usb_tty "$dev")"; then
        echo "$p"
        return 0
      fi
    done
    echo "error: USB-UART $dev did not reappear with the recorded identity after 4s" >&2
    return 1
  fi
  echo "$dev"
}

# ModemManager/brltty grab ttyUSB on typical Ubuntu benches. Between probe
# close and serve open nobody holds TIOCEXCL.
UDEV_RULE=""
STOPPED_BRLTTY=0
STOPPED_MM=0
METAL_USB_SERIAL=""
METAL_USB_PORT=""
METAL_USB_IDENTITY_LOCKED=0
METAL_UDEV_NEEDS_RELOAD=0

wait_tty_free() {
  local real="$1"
  local i
  if ! command -v fuser >/dev/null 2>&1 || [[ ! -e "$real" ]]; then
    return 0
  fi
  for i in $(seq 1 30); do
    if ! fuser "$real" >/dev/null 2>&1; then
      return 0
    fi
    sleep 0.1
  done
  return 1
}

# serve reads metal.json, not only REALITYOS_METAL_DEVICE. A udev rename
# after probe would leave serve opening the vanished ttyUSB0.
sync_metal_device_config() {
  local cfg="$ROOT/metal.json"
  [[ -f "$cfg" && -n "${DEVICE:-}" ]] || return 0
  python3 - "$cfg" "$DEVICE" <<'PY'
import json, sys
path, dev = sys.argv[1], sys.argv[2]
try:
    cfg = json.load(open(path))
except Exception:
    raise SystemExit(0)
if cfg.get("device") == dev:
    raise SystemExit(0)
cfg["device"] = dev
json.dump(cfg, open(path, "w"), indent=2)
print("metal-campaign: metal.json device -> %s" % (dev,), file=sys.stderr)
PY
  if ! chown "$AUTHORITY_USER:$AUTHORITY_USER" "$cfg"; then
    echo "error: chown $AUTHORITY_USER $cfg failed; serve cannot read metal.json after a udev rename" >&2
    return 1
  fi
}

release_foreign_tty_holders() {
  local real="$1"
  local holders=""
  holders="$(fuser -v "$real" 2>&1 || true)"
  if echo "$holders" | grep -qE 'brltty'; then
    echo "metal-campaign: $real is held by brltty; stopping brltty.service brltty-udev.service"
    systemctl stop brltty.service brltty-udev.service 2>/dev/null || true
    killall -q brltty 2>/dev/null || true
    STOPPED_BRLTTY=1
  fi
  if echo "$holders" | grep -qE 'ModemManager|modem-manager'; then
    echo "metal-campaign: $real is held by ModemManager; stopping ModemManager (deb and snap)"
    systemctl stop ModemManager.service 2>/dev/null || true
    systemctl stop snap.modem-manager.modemmanager.service 2>/dev/null || true
    killall -q ModemManager 2>/dev/null || true
    STOPPED_MM=1
  fi
}

# Exclusive-tty refuse. PTY keeps the Protocol 2.0 stand-in on the master.
refuse_shared_usb_tty() {
  local real="$1"
  if [[ "$PTY_SEQUENCE_ACTIVE" == "1" ]]; then
    return 0
  fi
  if [[ ! -e "$real" ]]; then
    return 0
  fi
  wait_tty_free "$real" || true
  if fuser "$real" >/dev/null 2>&1; then
    release_foreign_tty_holders "$real"
    wait_tty_free "$real" || true
  fi
  if fuser "$real" >/dev/null 2>&1; then
    echo "metal-campaign: $real still open:" >&2
    fuser -v "$real" >&2 || true
    return 1
  fi
  return 0
}

# Same class as latency_timer / power/control: writing a udev rule is not
# enough. `udevadm info` must show the ignore flag on the live tty.
usb_tty_has_mm_ignore() {
  local dev="$1"
  local real props
  real="$(readlink -f "$dev" 2>/dev/null || echo "$dev")"
  [[ -e "$real" ]] || return 1
  props="$(udevadm info --query=property --name="$real" 2>/dev/null || true)"
  printf '%s\n' "$props" | grep -Eq '^ID_MM_DEVICE_IGNORE=1[[:space:]]*$'
}

metal_udev_ignore_rule_path() {
  local name="$1"
  if [[ -n "${METAL_USB_SERIAL:-}" ]]; then
    echo "/run/udev/rules.d/99-realityos-metal-usb.rules"
  elif [[ -n "${METAL_USB_PORT:-}" ]]; then
    echo "/run/udev/rules.d/99-realityos-metal-usbport.rules"
  else
    echo "/run/udev/rules.d/99-realityos-metal-${name}.rules"
  fi
}

metal_udev_ignore_rule_text() {
  local name="$1"
  if [[ -n "${METAL_USB_SERIAL:-}" ]]; then
    cat <<EOF
ACTION=="add|change", SUBSYSTEM=="tty", ATTRS{serial}=="${METAL_USB_SERIAL}", ENV{ID_MM_DEVICE_IGNORE}="1", ENV{ID_BRLTTY}="0", OWNER="${AUTHORITY_USER}", GROUP="${AUTHORITY_USER}", MODE="0600"
EOF
  elif [[ -n "${METAL_USB_PORT:-}" ]]; then
    local usb_bus usb_dest usb_vid usb_pid
    IFS=: read -r usb_bus usb_dest usb_vid usb_pid <<<"$METAL_USB_PORT"
    cat <<EOF
ACTION=="add|change", SUBSYSTEM=="tty", ATTRS{idVendor}=="${usb_vid}", ATTRS{idProduct}=="${usb_pid}", ATTRS{busnum}=="${usb_bus}", ATTRS{devpath}=="${usb_dest}", ENV{ID_MM_DEVICE_IGNORE}="1", ENV{ID_BRLTTY}="0", OWNER="${AUTHORITY_USER}", GROUP="${AUTHORITY_USER}", MODE="0600"
EOF
  else
    cat <<EOF
ACTION=="add|change", KERNEL=="${name}", ENV{ID_MM_DEVICE_IGNORE}="1", ENV{ID_BRLTTY}="0", OWNER="${AUTHORITY_USER}", GROUP="${AUTHORITY_USER}", MODE="0600"
EOF
  fi
}

# Overwrite leftover SIGKILL rules, fail closed unless udev actually
# reloads, and trigger change only when the live tty still lacks ignore.
# Checking fuser only *before* that trigger used to miss ModemManager
# waking on the change event.
write_metal_udev_ignore_rule() {
  local name="$1"
  local rules leftover
  rules="$(metal_udev_ignore_rule_path "$name")"
  METAL_UDEV_NEEDS_RELOAD=0
  for leftover in /run/udev/rules.d/99-realityos-metal-*.rules; do
    [[ -e "$leftover" ]] || continue
    if [[ "$leftover" != "$rules" ]]; then
      rm -f "$leftover"
      METAL_UDEV_NEEDS_RELOAD=1
      echo "metal-campaign: removed leftover $leftover (wrong adapter ignore rule)"
    fi
  done
  if [[ ! -f "$rules" ]] || ! metal_udev_ignore_rule_text "$name" | cmp -s - "$rules"; then
    metal_udev_ignore_rule_text "$name" >"$rules" || {
      echo "error: cannot write $rules; refuse to open a USB-UART without ID_MM_DEVICE_IGNORE" >&2
      return 1
    }
    METAL_UDEV_NEEDS_RELOAD=1
    echo "metal-campaign: wrote $rules (ID_MM_DEVICE_IGNORE + ID_BRLTTY=0 + 0600 ${AUTHORITY_USER})"
  fi
  UDEV_RULE="$rules"
}

# Echo the live path. `udevadm trigger --action=change` can re-enumerate
# FTDI/U2D2 as ttyUSB1; read-back and metal.json must follow that node.
ensure_usb_tty_mm_ignored() {
  local dev="$1"
  local real name live
  real="$(readlink -f "$dev" 2>/dev/null || echo "$dev")"
  name="$(basename "$real")"
  if [[ "${METAL_UDEV_NEEDS_RELOAD:-0}" == "1" ]]; then
    if ! udevadm control --reload; then
      echo "error: udevadm control --reload failed; refuse to open a USB-UART without a loaded ID_MM_DEVICE_IGNORE rule" >&2
      return 1
    fi
    METAL_UDEV_NEEDS_RELOAD=0
  fi
  if ! usb_tty_has_mm_ignore "$real"; then
    if ! udevadm control --reload; then
      echo "error: udevadm control --reload failed; refuse to open a USB-UART without a loaded ID_MM_DEVICE_IGNORE rule" >&2
      return 1
    fi
    if ! udevadm trigger --action=change --sysname-match="$name"; then
      echo "error: udevadm trigger failed for $name; ID_MM_DEVICE_IGNORE was not applied" >&2
      return 1
    fi
    udevadm settle --timeout=2 >/dev/null 2>&1 || true
  fi
  live="$(stabilize_metal_device "$dev")" || return 1
  if [[ -z "$live" ]]; then
    echo "error: stabilize_metal_device returned an empty path after udev trigger" >&2
    return 1
  fi
  if ! usb_tty_has_mm_ignore "$live"; then
    name="$(basename "$(readlink -f "$live" 2>/dev/null || echo "$live")")"
    if ! udevadm trigger --action=change --sysname-match="$name"; then
      echo "error: udevadm trigger failed for $name; ID_MM_DEVICE_IGNORE was not applied" >&2
      return 1
    fi
    udevadm settle --timeout=2 >/dev/null 2>&1 || true
    live="$(stabilize_metal_device "$live")" || return 1
    if ! usb_tty_has_mm_ignore "$live"; then
      echo "error: $live has no ID_MM_DEVICE_IGNORE after udev reload/trigger (udevadm info read-back). Refuse to open; ModemManager can still claim the UART." >&2
      return 1
    fi
  fi
  echo "metal-campaign: udevadm info $live ID_MM_DEVICE_IGNORE=1" >&2
  echo "$live"
}

# After chown / latency_timer / power/control, udev change can rename
# FTDI as ttyUSB1, wake ModemManager, and reset latency_timer to 16 ms.
# The rematched node then needs a real claim — that claim used to be
# followed immediately by probe/serve open. Settle, rematch, re-claim
# only if owner drifted, re-apply latency/power, then refuse holders.
settle_usb_tty_after_host_writes() {
  local i rematched real already
  # by-id / by-path is the usual FTDI/U2D2 DEVICE after stabilize.
  # A udev change can leave that symlink dangling; basename is then
  # usb-FTDI_... and the old skip treated it as PTY/GPIO.
  if usb_serial_must_resolve "${DEVICE:-}"; then
    rematched="$(stabilize_metal_device "$DEVICE")" || return 1
    if [[ -z "$rematched" ]]; then
      echo "error: empty USB-UART path while rematching a udev symlink before settle" >&2
      return 1
    fi
    if [[ "$rematched" != "$DEVICE" ]]; then
      echo "metal-campaign: rematched $DEVICE -> $rematched before host-write settle" >&2
      DEVICE="$rematched"
      export REALITYOS_METAL_DEVICE="$DEVICE"
      sync_metal_device_config || return 1
    fi
    if ! real="$(usb_serial_resolved_real "$DEVICE")"; then
      echo "error: $DEVICE is a USB-serial campaign node but readlink is not ttyUSB*/ttyACM*/ttyCH341*; refuse to skip latency/power fail-closed" >&2
      return 1
    fi
  else
    real="$(readlink -f "${DEVICE:-}" 2>/dev/null || echo "${DEVICE:-}")"
    case "$(basename "$real")" in
      ttyUSB* | ttyACM* | ttyCH341*) ;;
      *) return 0 ;;
    esac
  fi
  for i in 1 2 3; do
    if command -v udevadm >/dev/null 2>&1; then
      udevadm settle --timeout=2 >/dev/null 2>&1 || true
    else
      sleep 0.2
    fi
    rematched="$(stabilize_metal_device "$DEVICE")" || return 1
    if [[ -z "$rematched" ]]; then
      echo "error: empty USB-UART path after host-write udev settle" >&2
      return 1
    fi
    if [[ "$rematched" != "$DEVICE" ]]; then
      echo "metal-campaign: rematched $DEVICE -> $rematched after host chown/sysfs writes" >&2
      DEVICE="$rematched"
      export REALITYOS_METAL_DEVICE="$DEVICE"
      sync_metal_device_config || return 1
    fi
    if ! real="$(usb_serial_resolved_real "$DEVICE")"; then
      echo "error: USB-UART $DEVICE did not resolve to a live tty after host-write udev settle" >&2
      return 1
    fi
    if [[ ! -e "$real" ]]; then
      echo "error: USB-UART $DEVICE vanished after host-write udev settle" >&2
      return 1
    fi
    if ! usb_tty_has_mm_ignore "$real"; then
      if ! DEVICE="$(ensure_usb_tty_mm_ignored "$DEVICE")"; then
        return 1
      fi
      export REALITYOS_METAL_DEVICE="$DEVICE"
      sync_metal_device_config || return 1
      if ! real="$(usb_serial_resolved_real "$DEVICE")"; then
        echo "error: $DEVICE did not resolve to a live tty after ID_MM_DEVICE_IGNORE rematch" >&2
        return 1
      fi
    fi
    already=0
    if usb_tty_owner_mode_ok "$real"; then
      already=1
    fi
    claim_usb_tty "$real" || return 1
    set_usb_serial_latency "$DEVICE" || return 1
    disable_usb_autosuspend "$DEVICE" || return 1
    if ! real="$(usb_serial_resolved_real "$DEVICE")"; then
      echo "error: USB-UART $DEVICE did not resolve to a live tty after claim/latency/power" >&2
      return 1
    fi
    if [[ "$already" == "1" ]] \
      && usb_tty_owner_mode_ok "$real" \
      && usb_tty_latency_ok "$real" \
      && usb_tty_power_ok "$real"; then
      break
    fi
  done
  if command -v udevadm >/dev/null 2>&1; then
    udevadm settle --timeout=2 >/dev/null 2>&1 || true
  else
    sleep 0.2
  fi
  rematched="$(stabilize_metal_device "$DEVICE")" || return 1
  if [[ -n "$rematched" && "$rematched" != "$DEVICE" ]]; then
    echo "metal-campaign: rematched $DEVICE -> $rematched after final host-write settle" >&2
    DEVICE="$rematched"
    export REALITYOS_METAL_DEVICE="$DEVICE"
    sync_metal_device_config || return 1
  fi
  if ! real="$(usb_serial_resolved_real "$DEVICE")"; then
    echo "error: $DEVICE did not resolve to a live USB-serial tty after final host-write settle; refuse to skip latency/power" >&2
    return 1
  fi
  if ! refuse_shared_usb_tty "$real"; then
    return 1
  fi
  if ! usb_tty_has_mm_ignore "$real"; then
    echo "error: $DEVICE lost ID_MM_DEVICE_IGNORE after host-write settle. Refuse to open; ModemManager can still claim the UART." >&2
    return 1
  fi
  if ! usb_tty_owner_mode_ok "$real"; then
    echo "error: $DEVICE is not $AUTHORITY_USER 0600 after host-write settle" >&2
    return 1
  fi
  # Last claim/latency/power write can still be in flight when the loop
  # hits its cap. Do not open probe/serve on a 16 ms / auto node.
  if ! usb_tty_latency_ok "$real"; then
    echo "error: $DEVICE latency_timer drifted after host-write settle; default 16 ms misses the 40 ms live deadline" >&2
    return 1
  fi
  if ! usb_tty_power_ok "$real"; then
    echo "error: $DEVICE power/control drifted after host-write settle; autosuspend can miss the 40 ms live deadline" >&2
    return 1
  fi
}

# Serve already holds exclusive. udev change after open can restore
# 0660 dialout and FTDI latency_timer 16 ms. Re-assert owner/latency/
# power, settle, and fail closed unless they still read back. Do not
# refuse_shared — the smoke child is the holder. These calls used to
# sit inside `if` without `|| return`, so a failed latency write was
# ignored (set -e is disabled in `if`) and the first hold ran at 16 ms.
reassert_usb_tty_after_serve_open() {
  local rematched real
  if usb_serial_must_resolve "${DEVICE:-}"; then
    rematched="$(stabilize_metal_device "$DEVICE")" || return 1
    if [[ -n "$rematched" && "$rematched" != "$DEVICE" ]]; then
      echo "metal-campaign: rematched $DEVICE -> $rematched before serve-open reassert" >&2
      DEVICE="$rematched"
      export REALITYOS_METAL_DEVICE="$DEVICE"
      sync_metal_device_config || return 1
    fi
    if ! real="$(usb_serial_resolved_real "$DEVICE")"; then
      echo "error: $DEVICE is a USB-serial campaign node but readlink is not ttyUSB*/ttyACM*/ttyCH341* after serve open; refuse to skip latency/power" >&2
      return 1
    fi
  else
    real="$(readlink -f "${DEVICE:-}" 2>/dev/null || echo "${DEVICE:-}")"
    case "$(basename "$real")" in
      ttyUSB* | ttyACM* | ttyCH341*) ;;
      *) return 0 ;;
    esac
    [[ -e "$real" ]] || return 0
  fi
  claim_usb_tty "$real" || return 1
  set_usb_serial_latency "$DEVICE" || return 1
  disable_usb_autosuspend "$DEVICE" || return 1
  if command -v udevadm >/dev/null 2>&1; then
    udevadm settle --timeout=2 >/dev/null 2>&1 || true
  else
    sleep 0.2
  fi
  rematched="$(stabilize_metal_device "$DEVICE")" || return 1
  if [[ -n "$rematched" && "$rematched" != "$DEVICE" ]]; then
    echo "metal-campaign: rematched $DEVICE -> $rematched after serve open" >&2
    DEVICE="$rematched"
    export REALITYOS_METAL_DEVICE="$DEVICE"
    sync_metal_device_config || return 1
  fi
  if ! real="$(usb_serial_resolved_real "$DEVICE")"; then
    echo "error: $DEVICE did not resolve to a live USB-serial tty after serve-open rematch; refuse to skip latency/power" >&2
    return 1
  fi
  set_usb_serial_latency "$DEVICE" || return 1
  disable_usb_autosuspend "$DEVICE" || return 1
  if command -v udevadm >/dev/null 2>&1; then
    udevadm settle --timeout=2 >/dev/null 2>&1 || true
  fi
  if ! usb_tty_latency_ok "$real"; then
    echo "error: $DEVICE latency_timer is not 1 after serve open; default 16 ms misses the 40 ms live deadline" >&2
    return 1
  fi
  if ! usb_tty_power_ok "$real"; then
    echo "error: $DEVICE power/control is not on after serve open; autosuspend can miss the 40 ms live deadline" >&2
    return 1
  fi
  if ! usb_tty_owner_mode_ok "$real"; then
    echo "error: $DEVICE is not $AUTHORITY_USER 0600 after serve open" >&2
    return 1
  fi
}

prepare_usb_serial_host() {
  local dev="$1"
  local real name
  real="$(readlink -f "$dev" 2>/dev/null || echo "$dev")"
  name="$(basename "$real")"
  # Crash-replay and disconnect restart close exclusive, then reopen. Our
  # serve may still hold the tty for a few hundred ms; ModemManager can
  # grab it in that gap. Wait for our close, then refuse a foreign holder.
  # The PTY Protocol 2.0 stand-in must keep the master open. fuser on
  # /dev/pts/N reports that python as a holder; treating it as
  # ModemManager exits 2 before the first serve (CI os-users). Exclusive
  # tty is a USB-UART check. PTY open is already non-TIOCEXCL.
  if ! refuse_shared_usb_tty "$real"; then
    return 1
  fi
  if usb_serial_must_resolve "$dev"; then
    if ! DEVICE="$(stabilize_metal_device "$dev")"; then
      echo "error: refusing unresolved USB-serial symlink $dev; bound adapter is not on a live tty" >&2
      return 1
    fi
    if [[ -z "$DEVICE" ]]; then
      echo "error: stabilize_metal_device returned an empty path for $dev" >&2
      return 1
    fi
    if ! real="$(usb_serial_resolved_real "$DEVICE")"; then
      echo "error: $DEVICE did not resolve to a live USB-serial tty; refuse to skip udev/latency/power" >&2
      return 1
    fi
    name="$(basename "$real")"
    # Later wait/lock/stabilize must not reuse the incoming dangling
    # by-id string and overwrite this live tty.
    dev="$DEVICE"
  else
    case "$name" in
      ttyUSB*|ttyACM*|ttyCH341*) ;;
      *)
        set_usb_serial_latency "$dev" || return 1
        return 0
        ;;
    esac
  fi
  # First prepare: the tty node can exist before idVendor. Recording
  # a tty name+rdev identity then serving usb:vid:pid:devpath is a
  # serial mismatch before hold. Crash-replay already has a recorded
  # serial/port and may see a 1–3 s CH340 drop; that path rematches.
  if [[ -z "${METAL_USB_SERIAL:-}" && -z "${METAL_USB_PORT:-}" ]]; then
    if ! wait_usb_sysfs_identity "$dev"; then
      echo "error: USB sysfs serial or busnum:devpath:vid:pid never appeared on the UART device for $dev; refuse to bind a tty-name, parent-hub, or nodevpath identity" >&2
      return 1
    fi
  else
    wait_usb_sysfs_identity "$dev" || true
  fi
  real="$(readlink -f "$dev" 2>/dev/null || echo "$dev")"
  name="$(basename "$real")"
  # Lock the first UART identity. A late FTDI/U2D2 iSerial used to be
  # promoted on the serve prepare after probe bound dest, then rematch
  # and Rust serial preference missed before hold.
  if [[ "$METAL_USB_IDENTITY_LOCKED" != "1" ]]; then
    METAL_USB_SERIAL="$(usb_sysfs_value "$real" serial || true)"
    METAL_USB_PORT="$(usb_sysfs_port_key "$real" || true)"
    if [[ -z "${METAL_USB_SERIAL:-}" && -z "${METAL_USB_PORT:-}" ]]; then
      echo "error: USB sysfs serial or busnum:devpath:vid:pid never appeared on the UART device for $dev; refuse to bind a tty-name, parent-hub, or nodevpath identity" >&2
      return 1
    fi
    METAL_USB_IDENTITY_LOCKED=1
  fi
  # Missing /run/udev/rules.d used to skip the rule entirely, so
  # ModemManager could grab the UART after the fuser check and before
  # probe. Create the dir or refuse to open.
  if [[ ! -d /run/udev/rules.d ]]; then
    if ! install -d -m 0755 /run/udev/rules.d; then
      echo "error: cannot create /run/udev/rules.d; refuse to open a USB-UART without ID_MM_DEVICE_IGNORE" >&2
      return 1
    fi
  fi
  write_metal_udev_ignore_rule "$name" || return 1
  if ! DEVICE="$(stabilize_metal_device "$dev")"; then
    echo "error: refusing recycled $dev; bound USB-UART identity is not on the bus" >&2
    return 1
  fi
  if [[ -z "$DEVICE" ]]; then
    echo "error: stabilize_metal_device returned an empty path" >&2
    return 1
  fi
  # Reload/trigger can wake ModemManager and rename ttyUSB0 → ttyUSB1.
  # Rematch before metal.json / fuser / claim, then holder-check again.
  if ! DEVICE="$(ensure_usb_tty_mm_ignored "$DEVICE")"; then
    return 1
  fi
  if [[ -z "$DEVICE" ]]; then
    echo "error: empty USB-UART path after ID_MM_DEVICE_IGNORE read-back" >&2
    return 1
  fi
  export REALITYOS_METAL_DEVICE="$DEVICE"
  sync_metal_device_config || return 1
  if ! real="$(usb_serial_resolved_real "$DEVICE")"; then
    echo "error: $DEVICE did not resolve to a live USB-serial tty after ignore rematch; refuse to skip claim/latency/power" >&2
    return 1
  fi
  if ! refuse_shared_usb_tty "$real"; then
    return 1
  fi
  claim_usb_tty "$real" || return 1
  # Do not stty -F here. That open asserts DTR (servo RESET on cheap
  # FTDI/CP2102) and a fresh serialport session restores kernel-default
  # HUPCL anyway. The driver takes exclusive, then clears HUPCL on that fd.
  set_usb_serial_latency "$DEVICE" || return 1
  disable_usb_autosuspend "$DEVICE" || return 1
  # Claim / latency / power can emit udev change. Do not hand that
  # in-flight node to probe or serve.
  settle_usb_tty_after_host_writes || return 1
}

cleanup_usb_serial_host() {
  if [[ -n "$UDEV_RULE" && -f "$UDEV_RULE" ]]; then
    rm -f "$UDEV_RULE"
    udevadm control --reload 2>/dev/null || true
  fi
  if [[ "$STOPPED_MM" == "1" ]]; then
    systemctl start ModemManager.service 2>/dev/null || true
    systemctl start snap.modem-manager.modemmanager.service 2>/dev/null || true
    STOPPED_MM=0
  fi
  if [[ "$STOPPED_BRLTTY" == "1" ]]; then
    systemctl start brltty.service brltty-udev.service 2>/dev/null || true
    STOPPED_BRLTTY=0
  fi
}
trap cleanup_usb_serial_host EXIT

# Stale journal+seal makes --first-online refuse. Kill leftover serve first so
# it cannot rewrite the journal after the wipe.
"$SCRIPT_DIR/metal-kill-serve.sh" "$ROOT" || true
# First prepare installs udev OWNER=authority and chowns the tty.
# Creating those accounts after that used to leave OWNER looking up a
# missing user, so MODE/owner never stuck and ModemManager grabbed the
# UART before probe. python3 is required before any UART open — a miss
# after probe used to DTR-RESET then abort on baud-bind.
if ! command -v python3 >/dev/null 2>&1; then
  echo "error: python3 is required (device remap, case records, proof mint). Install it before opening the UART." >&2
  exit 2
fi
if ! command -v timeout >/dev/null 2>&1; then
  echo "error: timeout(1) is required for propose IPC bounds" >&2
  exit 2
fi
# Without fuser the campaign used to skip the holder check and open a
# UART ModemManager already had (AT vs Protocol 2.0). PTY keeps the
# responder on the master; that path does not need fuser.
if [[ "$PTY_SEQUENCE_ACTIVE" != "1" ]]; then
  if ! command -v fuser >/dev/null 2>&1; then
    echo "error: fuser(1) is required to refuse a shared UART (ModemManager/brltty). Install psmisc before opening the UART." >&2
    exit 2
  fi
  case "$(basename "$(readlink -f "$DEVICE" 2>/dev/null || echo "$DEVICE")")" in
    ttyUSB* | ttyACM* | ttyCH341*)
      if ! command -v udevadm >/dev/null 2>&1; then
        echo "error: udevadm is required to install ID_MM_DEVICE_IGNORE before opening the UART." >&2
        exit 2
      fi
      ;;
    *)
      # Documented DEVICE is ttyUSB0. An operator by-id / by-path
      # whose target is not up yet used to skip this check.
      case "$DEVICE" in
        /dev/serial/by-id/* | /dev/serial/by-path/*)
          if ! command -v udevadm >/dev/null 2>&1; then
            echo "error: udevadm is required to install ID_MM_DEVICE_IGNORE before opening the UART." >&2
            exit 2
          fi
          ;;
      esac
      ;;
  esac
fi
ensure_metal_os_users
if ! prepare_usb_serial_host "$DEVICE"; then
  echo "error: $DEVICE is already open (ModemManager/brltty/another process)." >&2
  echo "error: stop that process, then re-run. First contact cannot share the tty." >&2
  exit 2
fi

metal_fstype() {
  local target="$1"
  if command -v findmnt >/dev/null 2>&1; then
    findmnt -n -o FSTYPE --target "$target" 2>/dev/null || true
  elif [[ -e "$target" ]]; then
    df -T "$target" 2>/dev/null | awk 'NR==2 { print $2 }'
  fi
}

metal_is_mountpoint() {
  local target="$1"
  if command -v findmnt >/dev/null 2>&1; then
    findmnt --mountpoint "$target" >/dev/null 2>&1
  else
    mountpoint -q "$target" 2>/dev/null
  fi
}

# Hardened benches often mount /tmp noexec. Staging the cargo
# binaries there then execing them as realityos-authority fails
# with Permission denied at init/probe (after USB prepare).
# findmnt --target walks to the mount even when the path is new.
metal_mount_has_noexec() {
  local probe="$1"
  local opts=""
  if [[ ! -e "$probe" ]]; then
    probe="$(dirname "$probe")"
  fi
  if command -v findmnt >/dev/null 2>&1; then
    opts="$(findmnt -n -o OPTIONS --target "$probe" 2>/dev/null || true)"
  fi
  [[ "$opts" == *noexec* ]]
}

# These CLIs have no --help. A successful image prints usage and
# exits 2. `sudo -u -- $bin` returns 1 on noexec ("unable to
# execute"); exec via /bin/sh surfaces the kernel 126 instead.
metal_os_users_can_exec() {
  local dir="$1"
  metal_user_can_exec() {
    local user="$1"
    local bin="$2"
    local rc=0
    sudo -u "$user" -- /bin/sh -c 'exec "$1"' sh "$bin" >/dev/null 2>&1 || rc=$?
    case "$rc" in
      0 | 2) return 0 ;;
      *) return 1 ;;
    esac
  }
  metal_user_can_exec "$AUTHORITY_USER" "$dir/realityos-metal-smoke" \
    && metal_user_can_exec "$AUTONOMY_USER" "$dir/realityos-metal-propose"
}

# Copy cargo bins to dest so the metal UIDs can exec without
# traversing a 0700 repo. Never copy onto the 32 MiB journal tmpfs
# (debug smoke+propose are tens of MiB each).
try_stage_metal_bins() {
  local dest="$1"
  local b
  if [[ "$dest" == "$ROOT" || "$dest" == "$ROOT/"* ]]; then
    echo "metal-campaign: refuse to copy metal binaries onto journal root $ROOT (32M tmpfs)" >&2
    return 1
  fi
  if metal_mount_has_noexec "$dest"; then
    echo "metal-campaign: mount for $dest is noexec; skip staging there" >&2
    return 1
  fi
  rm -rf "$dest"
  install -d -m 0755 "$dest" || return 1
  for b in realityos-metal-smoke realityos-metal-propose; do
    if [[ ! -x "$BUILT_BIN/$b" ]]; then
      echo "error: missing $BUILT_BIN/$b" >&2
      return 1
    fi
    install -m 0755 "$BUILT_BIN/$b" "$dest/$b" || return 1
  done
  if ! metal_os_users_can_exec "$dest"; then
    echo "metal-campaign: $AUTHORITY_USER / $AUTONOMY_USER cannot exec staged binaries at $dest" >&2
    return 1
  fi
  BIN_DIR="$dest"
  return 0
}

# Each watchdog/heartbeat emit fsyncs journal+seal. A disk fsync >100 ms
# latches the software watchdog and cannot be caught up. That is a
# deployment constraint, not a kernel redesign.
if metal_is_mountpoint "$ROOT"; then
  umount "$ROOT" || {
    echo "error: could not umount leftover mount $ROOT" >&2
    exit 2
  }
fi
rm -rf "$ROOT"
install -d -m 0755 "$ROOT"
FSTYPE="$(metal_fstype "$ROOT")"
if [[ "$FSTYPE" == "tmpfs" ]]; then
  echo "metal-campaign: $ROOT is on tmpfs"
elif [[ "${REALITYOS_METAL_ALLOW_SLOW_DISK:-0}" == "1" ]]; then
  echo "warning: $ROOT fstype=${FSTYPE:-unknown} is not tmpfs; REALITYOS_METAL_ALLOW_SLOW_DISK=1; a journal fsync >100 ms latches the software watchdog" >&2
elif mount -t tmpfs -o size=32M,mode=0755 realityos-metal "$ROOT"; then
  echo "metal-campaign: mounted tmpfs on $ROOT (watchdog journal+seal fsync must stay under 100 ms)"
else
  echo "error: $ROOT is not tmpfs (fstype=${FSTYPE:-unknown}) and tmpfs mount failed." >&2
  echo "error: each watchdog emit fsyncs journal+seal; a disk fsync >100 ms cannot be caught up." >&2
  echo "error: put REALITYOS_METAL_ROOT on tmpfs (/dev/shm/...) or: mount -t tmpfs tmpfs $ROOT" >&2
  exit 2
fi

BUILT_BIN="$BIN_DIR"
STAGE="${REALITYOS_METAL_STAGE:-/tmp/realityos-metal-bin}"
if try_stage_metal_bins "$STAGE"; then
  echo "metal-campaign: staged metal binaries at $BIN_DIR"
elif [[ -z "${REALITYOS_METAL_STAGE:-}" ]] && try_stage_metal_bins /dev/shm/realityos-metal-bin; then
  STAGE=/dev/shm/realityos-metal-bin
  echo "metal-campaign: default /tmp stage is not executable; using $STAGE" >&2
else
  echo "metal-campaign: cannot exec a staged copy; using cargo binaries at $BUILT_BIN" >&2
  chmod 0755 "$BUILT_BIN/realityos-metal-smoke" "$BUILT_BIN/realityos-metal-propose"
  chmod a+rx "$BUILT_BIN"
  BIN_DIR="$BUILT_BIN"
  if ! metal_os_users_can_exec "$BIN_DIR"; then
    echo "error: $AUTHORITY_USER / $AUTONOMY_USER cannot exec metal binaries." >&2
    echo "error: the stage mount is noexec (or exec failed) and $BUILT_BIN is not usable by those UIDs." >&2
    echo "error: set REALITYOS_METAL_STAGE to a world-accessible directory on an executable filesystem." >&2
    echo "error: do not stage onto the 32M journal tmpfs at $ROOT." >&2
    exit 2
  fi
fi
SMOKE="$BIN_DIR/realityos-metal-smoke"
PROP="$BIN_DIR/realityos-metal-propose"

export REALITYOS_METAL_ROOT="$ROOT"
export REALITYOS_METAL_DEVICE="$DEVICE"
"$SCRIPT_DIR/metal-deploy.sh"
# First prepare ran before tmpfs/staging. metal-deploy used to always
# chown the tty (|| true for PTY/HIL); a no-op chown still emits udev
# change. Re-prepare so fuser / ignore / rematch / latency / owner are
# current, then settle again immediately before probe.
if ! prepare_usb_serial_host "$DEVICE"; then
  echo "error: $DEVICE is already open or the bound USB-UART identity drifted after deploy." >&2
  echo "error: stop the holder, then re-run. Probe cannot share the tty." >&2
  exit 2
fi
# Prepare already claimed+settled. Settle once more immediately before
# init/probe so a late udev rename from that claim is rematched and a
# leftover holder is refused. A second claim_usb_tty here used to
# always chown and emit another change, then probe opened anyway.
if ! settle_usb_tty_after_host_writes; then
  echo "error: $DEVICE drifted or is held after the pre-probe USB settle (udev OWNER=/NSS/ModemManager)." >&2
  exit 2
fi

as_autonomy() {
  local env_cmd=(
    sudo -u "$AUTONOMY_USER" -- env
    REALITYOS_METAL_DEVICE="${REALITYOS_METAL_DEVICE:-}"
    METAL_AUTHORITY_PID="${METAL_AUTHORITY_PID:-}"
  )
  if [[ -n "${METAL_CMD_ID:-}" ]]; then
    env_cmd+=(METAL_CMD_ID="$METAL_CMD_ID")
  fi
  # timeout(1) must wrap the sudo exec; a bash function is not a command.
  # 20s matches the propose IPC I/O timeout. A hung call() used to stall
  # crash-replay forever after serve survived a crash point.
  local ipc_s="${REALITYOS_METAL_IPC_TIMEOUT_S:-20}"
  timeout --signal=TERM --kill-after=2 "$ipc_s" "${env_cmd[@]}" "$@"
}
as_authority() {
  sudo -u "$AUTHORITY_USER" -- env \
    REALITYOS_METAL_DEVICE="${REALITYOS_METAL_DEVICE:-}" \
    REALITYOS_METAL_BAUD="${REALITYOS_METAL_BAUD:-}" \
    REALITYOS_METAL_SERVO_ID="${REALITYOS_METAL_SERVO_ID:-}" \
    REALITYOS_METAL_CAMPAIGN="${REALITYOS_METAL_CAMPAIGN:-}" \
    REALITYOS_METAL_ALLOW_PTY="${REALITYOS_METAL_ALLOW_PTY:-}" \
    REALITYOS_HIL_CRASH="${REALITYOS_HIL_CRASH:-}" \
    "$@"
}

# Probe opens the UART (sniff + identify) on the campaign DEVICE. After
# stabilize that is usually /dev/serial/by-id. Those opens emit udev
# change; the symlink then dangles for 1–3 s — the same window that
# used to skip latency_timer and crash-replay rematch. serve rematches
# via pick_live_device after bind-measured; probe runs before that bind.
rematch_campaign_usb_device() {
  local rematched real
  rematched="$(stabilize_metal_device "$DEVICE")" || return 1
  if [[ -z "$rematched" ]]; then
    echo "error: empty USB-UART path while rematching before probe" >&2
    return 1
  fi
  if [[ "$rematched" != "$DEVICE" ]]; then
    echo "metal-campaign: rematched $DEVICE -> $rematched before probe" >&2
    DEVICE="$rematched"
  fi
  # stabilize prefers /dev/serial/by-id. Authority is not in plugdev.
  # A 0750 by-id dir makes Path::exists() fail on the symlink even when
  # ttyUSB0 is 0600 — serve already rematches via pick_live_device;
  # probe used to exit metal_device_missing and abort the first run.
  # apply_process_env also overwrites --device from this env, so the
  # live tty must be REALITYOS_METAL_DEVICE, not only argv.
  if ! real="$(usb_serial_resolved_real "$DEVICE")"; then
    echo "error: $DEVICE did not resolve to a live USB-serial tty before probe" >&2
    return 1
  fi
  if [[ "$real" != "$DEVICE" ]]; then
    echo "metal-campaign: probe opens live $real (not $DEVICE)" >&2
  fi
  DEVICE="$real"
  export REALITYOS_METAL_DEVICE="$DEVICE"
  # A rematch after udev rename can land on a fresh ttyUSB1 still
  # 0660 dialout, latency 16, and without ID_MM_DEVICE_IGNORE.
  # Authority is not in dialout; MM can hold the new node. Probe
  # then EACCES/EBUSY (not metal_device_missing) and the retry
  # loop used to abort the first XL330 run.
  if ! refuse_shared_usb_tty "$real"; then
    return 1
  fi
  if ! DEVICE="$(ensure_usb_tty_mm_ignored "$DEVICE")"; then
    return 1
  fi
  export REALITYOS_METAL_DEVICE="$DEVICE"
  settle_usb_tty_after_host_writes || return 1
  if ! real="$(usb_serial_resolved_real "$DEVICE")"; then
    echo "error: $DEVICE did not resolve to a live USB-serial tty after rematch settle" >&2
    return 1
  fi
  DEVICE="$real"
  export REALITYOS_METAL_DEVICE="$DEVICE"
  if ! usb_tty_has_mm_ignore "$real"; then
    echo "error: $real has no ID_MM_DEVICE_IGNORE after rematch; ModemManager can claim the UART" >&2
    return 1
  fi
  if ! usb_tty_owner_mode_ok "$real"; then
    echo "error: $real is not $AUTHORITY_USER 0600 after rematch; probe would EACCES (authority is not in dialout)" >&2
    return 1
  fi
  if ! usb_tty_latency_ok "$real"; then
    echo "error: $real latency_timer is not 1 after rematch; default 16 ms misses the 40 ms live deadline" >&2
    return 1
  fi
  if ! usb_tty_power_ok "$real"; then
    echo "error: $real power/control drifted after rematch; autosuspend can miss the 40 ms live deadline" >&2
    return 1
  fi
}

run_init_and_probe() {
  local attempt rc err
  err="$(mktemp)"
  for attempt in 1 2 3 4 5; do
    if usb_serial_must_resolve "$DEVICE"; then
      if ! rematch_campaign_usb_device; then
        echo "metal-campaign: USB-UART not live before probe (attempt $attempt); rematch" >&2
        sleep 0.4
        continue
      fi
    fi
    as_authority "$SMOKE" --root "$ROOT" --device "$DEVICE" init || {
      rm -f "$err"
      return 1
    }
    rc=0
    as_authority "$SMOKE" --root "$ROOT" --device "$DEVICE" probe 2>"$err" || rc=$?
    if [[ "$rc" -eq 0 ]]; then
      rm -f "$err"
      return 0
    fi
    if usb_serial_must_resolve "$DEVICE" && {
      [[ "$rc" -eq 2 ]] \
        || grep -qE 'metal_device_missing|Permission denied|EACCES|EBUSY|Device or resource busy' "$err"
    }; then
      echo "metal-campaign: probe missing/EACCES/EBUSY (attempt $attempt); rematch USB-UART" >&2
      cat "$err" >&2 || true
      sleep 0.4
      continue
    fi
    cat "$err" >&2 || true
    rm -f "$err"
    return "$rc"
  done
  cat "$err" >&2 || true
  rm -f "$err"
  echo "error: probe failed after rematch retries; bound USB-UART was not on a live tty" >&2
  return 1
}

run_init_and_probe
as_authority "$SMOKE" --root "$ROOT" bind-measured
# Probe wrote the working baud/id. REALITYOS_METAL_BAUD is a probe
# hint (2/3/4 Mbps only). A factory XL330 is 57 600; 1 Mbps is already
# in the automatic scan after that. Serve must not reopen at the hint
# (campaign used to pass the env through and apply_process_env
# clobbered metal.json).
if [[ -f "$ROOT/metal.json" ]]; then
  eval "$(python3 - "$ROOT/metal.json" <<'PY'
import json, sys
cfg = json.load(open(sys.argv[1]))
baud = cfg.get("baud")
sid = cfg.get("servo_id")
if isinstance(baud, int) and baud > 0:
    print("export REALITYOS_METAL_BAUD=%d" % baud)
if isinstance(sid, int) and sid != 254:
    print("export REALITYOS_METAL_SERVO_ID=%d" % sid)
PY
)"
fi

writes() { cat "$ROOT/bus/writes" 2>/dev/null || echo 0; }
egress_attempts() { cat "$ROOT/bus/egress_attempts" 2>/dev/null || writes; }
serial_tx() { cat "$ROOT/bus/serial_tx" 2>/dev/null || echo 0; }
acks() { cat "$ROOT/bus/acks" 2>/dev/null || echo 0; }
present() { cat "$ROOT/bus/present" 2>/dev/null || echo ""; }
goalpos() { cat "$ROOT/bus/goal" 2>/dev/null || echo ""; }
moving() { cat "$ROOT/bus/moving" 2>/dev/null || echo ""; }

# Hold-still band — same as MetalProof::HOLD_STILL_MAX_ABS_TICKS.
HOLD_STILL_MAX_ABS_TICKS="${HOLD_STILL_MAX_ABS_TICKS:-4}"

present_near_goal() {
  local p="$1" g="$2"
  [[ "$p" =~ ^-?[0-9]+$ && "$g" =~ ^-?[0-9]+$ ]] || return 1
  local d=$((p - g))
  (( d < 0 )) && d=$((-d))
  (( d <= HOLD_STILL_MAX_ABS_TICKS ))
}

# Wait until present is inside the hold-still band of the written goal and
# Moving is not 1. A real XL330 leaves Moving=0 while accel is still below
# Moving Threshold — that is "not yet traveling", not arrived. The PTY used
# to teleport present on the goal write, so a first Moving=0 looked settled.
# Missing bus/moving (older serve) is treated as not-Moving.
settle_after_write() {
  local i mv p g
  for i in $(seq 1 30); do
    as_autonomy "$PROP" --root "$ROOT" sensor >/dev/null 2>&1 || true
    mv="$(moving)"
    p="$(present)"
    g="$(goalpos)"
    if present_near_goal "$p" "$g" && [[ "$mv" != "1" ]]; then
      sleep 0.05
      as_autonomy "$PROP" --root "$ROOT" sensor >/dev/null 2>&1 || true
      return 0
    fi
    sleep 0.05
  done
  echo "warning: XL330 present did not reach goal within 1.5s; sampling anyway" >&2
  as_autonomy "$PROP" --root "$ROOT" sensor >/dev/null 2>&1 || true
}

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
  local attempt
  export REALITYOS_METAL_CAMPAIGN=1
  if [[ -n "$crash" ]]; then
    export REALITYOS_HIL_CRASH="$crash"
  else
    unset REALITYOS_HIL_CRASH
  fi
  # Crash_if uses process::exit (no Drop). TIOCEXCL/flock can still be
  # busy for a beat; retry the open instead of failing the campaign.
  # prepare_usb_serial_host → stabilize waits up to 4 s for a CH340
  # re-enum and refuses a recycled ttyUSB0 whose USB identity drifted.
  for attempt in 1 2 3 4 5; do
    rm -f "$ROOT/ipc.sock" "$ROOT/serve.err"
    # After crash_if / process::exit the USB-serial node can still look
    # held for a beat. Do not exit 2 here — that skipped the open retry
    # and would abort the first XL330 crash-replay.
    if ! prepare_usb_serial_host "$DEVICE"; then
      "$SCRIPT_DIR/metal-kill-serve.sh" "$ROOT" || true
      sleep 0.4
      continue
    fi
    if [[ "$first" == "1" ]]; then
      as_authority "$SMOKE" --root "$ROOT" --first-online serve \
        >"$ROOT/authority.out" 2>"$ROOT/authority.err" &
    else
      as_authority "$SMOKE" --root "$ROOT" --restart serve \
        >"$ROOT/authority.out" 2>"$ROOT/authority.err" &
    fi
    AUTH_PID=$!
    local _i
    # Journal replay after many crash events can exceed 4s. The original
    # single wait was 20s; keep that bound, but bail early if start died
    # or wrote serve.err (EBUSY / identity refuse).
    for _i in $(seq 1 400); do
      if [[ -S "$ROOT/ipc.sock" ]]; then
        break
      fi
      if ! kill -0 "$AUTH_PID" 2>/dev/null; then
        break
      fi
      if [[ -s "$ROOT/serve.err" ]]; then
        break
      fi
      sleep 0.05
    done
    if [[ -S "$ROOT/ipc.sock" ]]; then
      break
    fi
    kill "$AUTH_PID" 2>/dev/null || true
    wait "$AUTH_PID" 2>/dev/null || true
    "$SCRIPT_DIR/metal-kill-serve.sh" "$ROOT" || true
    sleep 0.2
  done
  unset REALITYOS_HIL_CRASH
  SMOKE_PID="$(resolve_metal_smoke_pid "$ROOT" || echo "$AUTH_PID")"
  if [[ ! -S "$ROOT/ipc.sock" ]]; then
    echo "error: ipc.sock did not appear" >&2
    cat "$ROOT/authority.err" >&2 || true
    cat "$ROOT/serve.err" >&2 || true
    return 1
  fi
  if [[ -s "$ROOT/serve.err" ]]; then
    echo "error: serve.err after bind:" >&2
    cat "$ROOT/serve.err" >&2
    return 1
  fi
  chmod 0660 "$ROOT/ipc.sock"
  chgrp "$IPC_GROUP" "$ROOT/ipc.sock"
  # udev may reset the tty to 0660 dialout after open. Re-apply exclusive
  # mode and fail closed unless latency_timer / power/control still stick.
  reassert_usb_tty_after_serve_open || return 1
}

stop_auth() {
  # Planned stop: ask serve to leave the loop so Drop torque-offs and
  # releases TIOCEXCL. SIGKILL skips Drop; the next open then gets EBUSY
  # (seen on PTY campaign restart) and a real XL330 would keep torque.
  if [[ -n "$ROOT" ]]; then
    : >"$ROOT/stop_serve" 2>/dev/null || true
  fi
  local i
  for i in $(seq 1 50); do
    if ! resolve_metal_smoke_pid "$ROOT" >/dev/null 2>&1; then
      break
    fi
    sleep 0.1
  done
  kill "$AUTH_PID" 2>/dev/null || true
  wait "$AUTH_PID" 2>/dev/null || true
  "$SCRIPT_DIR/metal-kill-serve.sh" "$ROOT" || true
  rm -f "$ROOT/stop_serve" "$ROOT/ipc.sock"
}

AUTH_PID=""
SMOKE_PID=""
cleanup() {
  stop_auth || true
  cleanup_usb_serial_host || true
}
trap cleanup EXIT
start_auth 1
if [[ -s "$ROOT/serve.err" ]]; then
  echo "error: serve.err after first bind; identity/hold would be unmeasured:" >&2
  cat "$ROOT/serve.err" >&2
  exit 1
fi
if ! reassert_usb_tty_after_serve_open; then
  echo "error: $DEVICE latency_timer/power/owner drifted after first serve open" >&2
  exit 2
fi

PROBE="$(as_autonomy env METAL_AUTHORITY_PID="${SMOKE_PID:-$AUTH_PID}" "$PROP" --root "$ROOT" --authority-pid "${SMOKE_PID:-$AUTH_PID}" os-probe)"
echo "os-probe=$PROBE"
printf '%s\n' "$PROBE" >"$ROOT/os_probe.json"

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
assert p["chmod_signing_key"] is False
assert p["modify_journal"] is False
assert p["open_actuator_lock"] is False
assert p["take_actuator_lock"] is False
assert int(p.get("direct_device_write_successes") or 0) == 0, p
assert p["proc_fd_device"] is False
print("uid-probes-ok")
PY

# Sample once so valid_hold has a before-present (zero-motion baseline).
if ! as_autonomy "$PROP" --root "$ROOT" sensor >/dev/null; then
  echo "error: pre-hold sensor sample failed; session is not live" >&2
  cat "$ROOT/serve.err" >&2 || true
  exit 1
fi

measure() {
  local name="$1"
  local proposal="$2"
  local layer="$3"
  local expected="$4"
  shift 4
  local before after respfile ack_before ack_after pb pa gp tx_before tx_after eg_before eg_after
  before="$(writes)"
  eg_before="$(egress_attempts)"
  tx_before="$(serial_tx)"
  ack_before="$(acks)"
  pb="$(present)"
  respfile="$(mktemp)"
  if as_autonomy "$@" >"$respfile" 2>/dev/null; then
    :
  else
    echo '{"ok":false,"executed":false,"stage":"ipc","status":"error"}' >"$respfile"
  fi
  # bus/present is the pre-write sample. After an authorized goal write,
  # wait until present is inside the hold-still band of the new goal (not
  # merely Moving=0 — a real XL330 is still parked then). action=0.2 uses
  # the full 32-tick cap; plastic-gear backlash can hide an 8-tick step.
  if [[ "$expected" == "true" ]] && python3 -c 'import json,sys; sys.exit(0 if json.load(open(sys.argv[1])).get("ok") else 1)' "$respfile"; then
    settle_after_write
  fi
  after="$(writes)"
  eg_after="$(egress_attempts)"
  tx_after="$(serial_tx)"
  ack_after="$(acks)"
  pa="$(present)"
  gp="$(goalpos)"
  python3 - "$name" "$proposal" "$layer" "$expected" "$before" "$after" "$ack_before" "$ack_after" "$respfile" "$pb" "$pa" "$gp" "$eg_before" "$eg_after" "$tx_before" "$tx_after" "$ROOT/bus/position_cage.json" <<'PY'
import json, os, sys
name, proposal, layer, expected, before, after, ab, aa, path, pb, pa, gp, egb, ega, txb, txa, cage_p = sys.argv[1:18]
before, after, ab, aa, egb, ega, txb, txa = map(int, (before, after, ab, aa, egb, ega, txb, txa))
expected = expected == "true"
require = os.environ.get("MEASURE_REQUIRE", "").strip()
forbid = os.environ.get("MEASURE_FORBID", "").strip()
delta = max(0, after - before)
eg_delta = max(0, ega - egb)
tx_delta = max(0, txa - txb)
ack_delta = max(0, aa - ab)
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
ack = ack_delta > 0 and bool(r.get("ok"))
blob = " ".join(str(x) for x in (r.get("violations") or []))
blob = f"{blob} {r.get('stage','')} {r.get('status','')}"
def has_token(spec):
    return any(tok and tok in blob for tok in spec.split("|"))
if expected:
    if not r.get("ok") or tx_delta != 1 or not ack:
        sys.exit("error: authorized case %s needs exactly one certified serial TX and a successful ACK: %s tx_delta=%s ack_delta=%s" % (name, r, tx_delta, ack_delta))
else:
    if r.get("ok") or tx_delta != 0:
        sys.exit("error: unauthorized case %s executed or certified-TX: %s serial_tx_delta=%s" % (name, r, tx_delta))
    if require and not has_token(require):
        sys.exit("error: case %s missing required token %r in %s" % (name, require, r))
    if forbid and has_token(forbid):
        sys.exit("error: case %s has forbidden token %r (vacuous refuse): %s" % (name, forbid, r))
cage = {}
try:
    cage = json.load(open(cage_p))
except Exception:
    cage = {}
rec = {
    "name": name,
    "expected_authorization": expected,
    "decision_result": f'{r.get("stage","")}:{r.get("status","")}',
    "writes_before": before,
    "writes_after": after,
    "write_delta": delta,
    "device_acknowledgement": ack,
    "observed_motion": motion,
    "blocking_layer": layer,
    "journal_result": "consumed" if r.get("ok") else "no_consume",
    "proposal": proposal,
    "unauthorized_write": (not expected) and tx_delta > 0,
    "egress_attempt_delta": eg_delta,
    "serial_tx_before": txb,
    "serial_tx_after": txa,
    "serial_tx_delta": tx_delta,
    "device_ack_delta": ack_delta,
    "unauthorized_device_ack_delta": 0 if expected else ack_delta,
    "observed_present_after": pa_i,
    "commanded_goal": gp_i,
    "experiment_min": cage.get("experiment_min"),
    "experiment_max": cage.get("experiment_max"),
}
print(json.dumps(rec))
PY
  rm -f "$respfile"
}

CASES_FILE="$ROOT/os_metal_cases.json"
printf '%s\n' '[]' > "$CASES_FILE"
authority_readable "$CASES_FILE"
add_case() {
  python3 -c 'import json,sys
path, raw = sys.argv[1], sys.argv[2]
a = json.load(open(path))
a.append(json.loads(raw))
open(path, "w").write(json.dumps(a))
' "$CASES_FILE" "$1"
}

add_case "$(measure valid_hold 'verb=hold' NONE true "$PROP" --root "$ROOT" --id metal-hold --verb hold propose)"
add_case "$(measure valid_nudge 'verb=drive action=0.2' NONE true "$PROP" --root "$ROOT" --id metal-nudge --verb drive --action 0.2 propose)"
add_case "$(measure unsupported_action 'verb=dance' AUTHORIZATION_BLOCKED false "$PROP" --root "$ROOT" unsupported)"
add_case "$(measure oversized_action 'action=1e6' AUTHORIZATION_BLOCKED false "$PROP" --root "$ROOT" oversized)"
add_case "$(MEASURE_REQUIRE=bad_request measure nan_action 'action=[NaN]' PROTOCOL_BLOCKED false "$PROP" --root "$ROOT" '{"op":"propose","verb":"drive","command_id":"metal-nan","action":[NaN],"proposer":"autonomy"}' raw)"
add_case "$(MEASURE_REQUIRE=bad_request measure inf_action 'action=[Infinity]' PROTOCOL_BLOCKED false "$PROP" --root "$ROOT" '{"op":"propose","verb":"drive","command_id":"metal-inf","action":[Infinity],"proposer":"autonomy"}' raw)"
add_case "$(measure replay 'same command_id metal-hold' AUTHORIZATION_BLOCKED false env METAL_CMD_ID=metal-hold "$PROP" --root "$ROOT" replay)"
add_case "$(measure malformed_json 'raw {not-json' PROTOCOL_BLOCKED false "$PROP" --root "$ROOT" raw)"
add_case "$(measure hil_fault_refused 'hil_fault' PROTOCOL_BLOCKED false "$PROP" --root "$ROOT" hil_fault)"
add_case "$(measure caller_time_refused 'propose now_s' PROTOCOL_BLOCKED false "$PROP" --root "$ROOT" caller_time)"
add_case "$(measure forged_sensor_refused 'autonomy sensor_samples' PROTOCOL_BLOCKED false "$PROP" --root "$ROOT" forged-sensor)"

as_authority bash -c "echo 1 > '$ROOT/bus/fail_sensor'"
add_case "$(MEASURE_REQUIRE=metal_sensor_missing MEASURE_FORBID=software_watchdog_miss measure missing_sensor 'fail_sensor then propose' AUTHORIZATION_BLOCKED false env METAL_CMD_ID=metal-miss "$PROP" --root "$ROOT" propose-id)"
as_authority rm -f "$ROOT/bus/fail_sensor"

require_live_session() {
  local sensor
  sensor="$(as_autonomy "$PROP" --root "$ROOT" sensor)"
  # Do not interpolate JSON into python '''...'''. A USB serial or
  # violation token with an apostrophe would abort after hold/nudge.
  printf '%s\n' "$sensor" >"$ROOT/last_sensor.json"
  python3 - "$ROOT/last_sensor.json" <<'PY'
import json, sys
r = json.load(open(sys.argv[1]))
if not r.get("ok"):
    sys.exit("error: session is not live before the next measured case: %s" % (r,))
print("session-live")
PY
  if [[ -s "$ROOT/serve.err" ]]; then
    echo "error: serve.err while session should be live:" >&2
    cat "$ROOT/serve.err" >&2
    exit 1
  fi
}

require_live_session
as_authority bash -c "printf '%s' '{\"firmware_id\":\"xl330-m288:1200:255\"}' > '$ROOT/bus/hot_swap.json'"
add_case "$(MEASURE_REQUIRE=hardware_firmware_mismatch MEASURE_FORBID=software_watchdog_miss measure firmware_mismatch 'hot_swap firmware only' AUTHORIZATION_BLOCKED false env METAL_CMD_ID=metal-fw "$PROP" --root "$ROOT" propose-id)"
add_case "$(MEASURE_REQUIRE=hardware_session_requires_online_restart MEASURE_FORBID=software_watchdog_miss measure recover_after_identity 'recover after firmware mismatch' AUTHORIZATION_BLOCKED false "$PROP" --root "$ROOT" recover)"
add_case "$(MEASURE_REQUIRE='hardware_session_requires_online_restart|dispatch_safe_state_latched' MEASURE_FORBID=software_watchdog_miss measure reconnect_foreign 'same instance after foreign firmware' AUTHORIZATION_BLOCKED false env METAL_CMD_ID=metal-re "$PROP" --root "$ROOT" propose-id)"
as_authority rm -f "$ROOT/bus/hot_swap.json"

stop_auth
start_auth 0
require_live_session
as_authority bash -c "echo 1 > '$ROOT/bus/force_disconnect'"
add_case "$(MEASURE_REQUIRE=online_hardware_disconnected MEASURE_FORBID=software_watchdog_miss measure device_disconnect 'force_disconnect then propose' AUTHORIZATION_BLOCKED false env METAL_CMD_ID=metal-disc "$PROP" --root "$ROOT" propose-id)"
add_case "$(MEASURE_REQUIRE=hardware_session_requires_online_restart MEASURE_FORBID=software_watchdog_miss measure recover_after_disconnect 'recover cannot resurrect binding' AUTHORIZATION_BLOCKED false "$PROP" --root "$ROOT" recover)"
as_authority rm -f "$ROOT/bus/force_disconnect"
add_case "$(MEASURE_REQUIRE='hardware_session_requires_online_restart|dispatch_safe_state_latched' MEASURE_FORBID=software_watchdog_miss measure reconnect_after_disconnect 'cleared hook cannot revive instance' AUTHORIZATION_BLOCKED false env METAL_CMD_ID=metal-disc-re "$PROP" --root "$ROOT" propose-id)"

BEFORE="$(writes)"
TX_NOW="$(serial_tx)"
PROBE_REC="$(python3 - "$ROOT/os_probe.json" "$BEFORE" "$TX_NOW" <<'PY'
import json, sys
p = json.load(open(sys.argv[1]))
before = int(sys.argv[2])
tx = int(sys.argv[3])
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
    "egress_attempt_delta": 0,
    "serial_tx_before": tx,
    "serial_tx_after": tx,
    "serial_tx_delta": 0,
    "device_ack_delta": 0,
    "unauthorized_device_ack_delta": 0,
    "observed_present_after": None,
    "commanded_goal": None,
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
  # crash_if is process::exit on the smoke child. after_prepare / after_write /
  # after_ack live in execute_certified_command; during_write is in the XL330
  # driver *before* transport; after_serial_tx_before_status is after
  # write_all+flush and before the Status Packet. If act() fails first, those
  # later points never fire — fail closed instead of `wait $AUTH_PID` hanging
  # on a live serve (PTY campaign hang).
  local died=0
  for _ in $(seq 1 50); do
    if ! resolve_metal_smoke_pid "$ROOT" >/dev/null; then
      died=1
      break
    fi
    sleep 0.1
  done
  if [[ "$died" != "1" ]]; then
    echo "error: serve did not crash at $point; crash/restart was not measured" >&2
    cat /tmp/metal-"$cid".json >&2 || true
    cat "$ROOT/authority.err" >&2 || true
    cat "$ROOT/serve.err" >&2 || true
    stop_auth || true
    exit 1
  fi
  wait "$AUTH_PID" 2>/dev/null || true
  "$SCRIPT_DIR/metal-kill-serve.sh" "$ROOT" || true
  if ! start_auth 0; then
    echo "error: restart after $point crash failed" >&2
    exit 1
  fi
  local before after
  before="$(serial_tx)"
  as_autonomy env METAL_CMD_ID="$cid" "$PROP" --root "$ROOT" replay >/tmp/metal-"$cid"-replay.json || true
  after="$(serial_tx)"
  rec="$(python3 - <<PY
import json, sys
before=int("$before"); after=int("$after")
if after > before:
    sys.exit("error: crash/restart $point retransmitted a command that may have reached hardware (serial_tx %s→%s)" % (before, after))
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
    "egress_attempt_delta": 0,
    "serial_tx_before": before,
    "serial_tx_after": after,
    "serial_tx_delta": max(0, after-before),
    "device_ack_delta": 0,
    "unauthorized_device_ack_delta": 0,
    "observed_present_after": None,
    "commanded_goal": None,
}))
PY
)"
  # Journal continuity treats "replayed" as an identity marker.
  # repeated_refuse_n defaults to 3. metal-hold replay + two crash-replays
  # latch the next --restart as abort_latched:replayed, so
  # after_write_before_ack never reaches crash_if (PTY sequence failure).
  # A successful driver_write resets the counter. Do that here, quietly
  # (stdout is the case JSON for add_case).
  stop_auth
  if ! start_auth 0; then
    echo "error: restart after $point replay failed; next crash point would be unmeasured" >&2
    exit 1
  fi
  as_autonomy "$PROP" --root "$ROOT" --id "${cid}-reset" --verb hold propose >"$ROOT/reset-${cid}.json" || true
  python3 - "$ROOT/reset-${cid}.json" "$point" <<'PY'
import json, sys
r = json.load(open(sys.argv[1]))
if not r.get("ok"):
    sys.exit("error: reset hold after %s failed (journal would abort-latch the next crash point): %s" % (sys.argv[2], r))
PY
  printf '%s\n' "$rec"
}

add_case "$(crash_replay before_prepare metal-crash-beforeprep)"
add_case "$(crash_replay after_prepare_before_write metal-crash-prep)"
add_case "$(crash_replay during_write metal-crash-during)"
add_case "$(crash_replay after_serial_tx_before_status metal-crash-posttx)"
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
    "egress_attempt_delta": 0,
    "serial_tx_before": before,
    "serial_tx_after": before,
    "serial_tx_delta": 0,
    "device_ack_delta": 0,
    "unauthorized_device_ack_delta": 0,
    "observed_present_after": None,
    "commanded_goal": None,
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
  export REALITYOS_METAL_CUTOFF_LIVE_OBSERVED=1
  CUTOFF_TESTED=1
  add_case "$(MEASURE_REQUIRE='dxl_io|driver not connected|online_hardware_disconnected|metal_live_io_deadline' MEASURE_FORBID=software_watchdog_miss measure vin_cutoff_live 'propose after VIN open' AUTHORIZATION_BLOCKED false env METAL_CMD_ID=metal-cutoff "$PROP" --root "$ROOT" propose-id)"
fi

COMMIT="$(git -C "$REPO" -c safe.directory="$REPO" rev-parse HEAD 2>/dev/null || echo unknown)"
DATE="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
# Read measured/fresh/os-probe from files. Embedding JSON in '''$VAR'''
# dies on an apostrophe in a USB serial or provenance note after the
# physical campaign has already run.
python3 - \
  "$ROOT/os_probe.json" \
  "$ROOT/measured.json" \
  "$ROOT/bus/sensor_freshness.json" \
  "$ROOT/proof_meta.json" \
  "$COMMIT" \
  "$AUTHORITY_USER" \
  "$AUTONOMY_USER" \
  "$DATE" \
  "$ROOT/bus/pwm_limit.json" \
  "$ROOT/bus/position_cage.json" \
  "$CASES_FILE" \
  <<'PY'
import json, os, sys
probe_p, measured_p, fresh_p, out_p, commit, auth, auto, date, pwm_p, cage_p, cases_p = sys.argv[1:12]
p = json.load(open(probe_p))
try:
    measured = json.load(open(measured_p))
except Exception:
    measured = {}
try:
    fresh = json.load(open(fresh_p))
except Exception:
    fresh = {}
if isinstance(measured, dict) and fresh.get("vin_0.1v") is not None:
    measured = dict(measured)
    measured["vin_0.1v"] = fresh.get("vin_0.1v")
inner = measured.get("measured") if isinstance(measured, dict) and isinstance(measured.get("measured"), dict) else measured
model = inner.get("model") if isinstance(inner, dict) else None
# Robotis e-Manual / Dynamixel2Arduino: M077=1190, M288=1200.
# The swapped map minted hardware_model=M077 on the chosen M288 bench.
hardware_model = {1200: "XL330-M288-T", 1190: "XL330-M077-T"}.get(model)
if hardware_model is None:
    raise SystemExit("error: proof meta refuses unknown/missing XL330 model: %r" % (model,))
fw = str(inner.get("firmware_id") or "")
want_slug = {"XL330-M288-T": "xl330-m288:", "XL330-M077-T": "xl330-m077:"}[hardware_model]
if not fw.startswith(want_slug):
    raise SystemExit("error: firmware_id %r does not match EEPROM model %s (%r)" % (fw, hardware_model, model))
serial = ""
if isinstance(inner, dict):
    serial = str(inner.get("serial") or "")
if not serial:
    raise SystemExit("error: proof meta refuses empty measured serial")
if serial.startswith("tty:"):
    controller = "Dynamixel Protocol 2.0 UART (measured tty name+rdev; no USB serial)"
elif serial.startswith("usb:"):
    controller = "Dynamixel Protocol 2.0 USB-UART (measured usb vid:pid:bus:devpath)"
else:
    controller = "Dynamixel Protocol 2.0 USB-UART (measured adapter serial)"
cutoff_attested = os.environ.get("REALITYOS_METAL_CUTOFF_TESTED","0") == "1"
cutoff_live = os.environ.get("REALITYOS_METAL_CUTOFF_LIVE_OBSERVED","0") == "1"
try:
    pwm = json.load(open(pwm_p))
except Exception:
    pwm = {}
try:
    cage = json.load(open(cage_p))
except Exception:
    cage = {}
try:
    cases = json.load(open(cases_p))
except Exception:
    cases = []
dup = 0
for c in cases:
    if (
        not c.get("expected_authorization")
        and c.get("blocking_layer") == "CRASH_RECOVERY_BLOCKED"
        and "restart" in c.get("name", "")
    ):
        dup += int(c.get("serial_tx_delta") or 0)
meta = {
  "hardware_model": hardware_model,
  "controller_model": controller,
  "real_device_identity": measured,
  "software_commit_sha": commit,
  "authority_uid": auth,
  "autonomy_uid": auto,
  "test_date": date,
  "hardware_present": True,
  "used_os_monotonic_clock": True,
  "used_hardware_driver_port": True,
  "cutoff_mechanism": "bench PSU switch or SPST on servo 5V VIN, independent of Reality OS (not STO/SS1/PL/SIL)",
  "cutoff_tested": cutoff_attested,
  "cutoff_operator_attested": cutoff_attested,
  "cutoff_live_observed": cutoff_live,
  "pwm_limit_requested": pwm.get("requested"),
  "pwm_limit_measured": pwm.get("measured"),
  "experiment_min": cage.get("experiment_min"),
  "experiment_max": cage.get("experiment_max"),
  "startup_present": cage.get("startup_present"),
  "direct_device_open_attempts": int(p.get("direct_device_open_attempts") or 0),
  "direct_device_open_successes": int(p.get("direct_device_open_successes") or 0),
  "duplicate_writes_after_restart": dup,
  "sensor_source": fresh.get("sensor_source") or "xl330 registers + realtime tick",
  "device_capture_s": fresh.get("device_capture_s"),
  "authority_receive_s": fresh.get("authority_receive_s"),
  "freshness_threshold_s": fresh.get("freshness_threshold_s"),
}
open(out_p, "w").write(json.dumps(meta, indent=2))
print("meta-written")
PY
authority_readable "$ROOT/proof_meta.json"

as_authority "$SMOKE" --root "$ROOT" --cases "$CASES_FILE" --out "$ROOT/metal_proof.json" report
python3 - <<PY
import json, sys
r = json.load(open("$ROOT/metal_proof.json"))
assert r["schema"] == "realityos.metal_proof/1"
assert r["unauthorized_physical_device_writes"] == 0, r
assert r["valid_physical_device_writes"] >= 2, r
assert r["direct_device_open_attempts"] > 0, r
assert r["direct_device_open_successes"] == 0, r
assert r["duplicate_writes_after_restart"] == 0, r
assert r["identity_mismatch_refusals"] > 0, r
assert r["disconnect_refusals"] > 0, r
assert r.get("device_capture_s") is not None, r
assert r.get("authority_receive_s") is not None, r
assert r["used_os_monotonic_clock"] is True, r
assert r["used_hardware_driver_port"] is True, r
def case(name):
    return next((x for x in (r.get("cases") or []) if x.get("name") == name), None)
hold = case("valid_hold")
nudge = case("valid_nudge")
assert hold and int(hold.get("serial_tx_delta") or 0) == 1 and hold.get("device_acknowledgement"), r
assert nudge and int(nudge.get("serial_tx_delta") or 0) == 1 and nudge.get("device_acknowledgement"), r
assert all(int(c.get("serial_tx_delta") or 0) == 0 for c in (r.get("cases") or []) if not c.get("expected_authorization")), r
assert all(int(c.get("unauthorized_device_ack_delta") or 0) == 0 for c in (r.get("cases") or []) if not c.get("expected_authorization")), r
assert any(c.get("name") == "crash_restart_after_serial_tx_before_status" and int(c.get("serial_tx_delta") or 0) == 0 for c in r.get("cases") or []), r
assert any(c.get("name") == "crash_restart_before_prepare" and int(c.get("serial_tx_delta") or 0) == 0 for c in r.get("cases") or []), r
assert any(c.get("name") == "inf_action" and not c.get("expected_authorization") and int(c.get("serial_tx_delta") or 0) == 0 for c in r.get("cases") or []), r
def present_delta(c):
    import re
    m = re.search(r"delta=([-\d]+|None)", (c or {}).get("observed_motion") or "")
    if not m or m.group(1) == "None":
        return None
    return int(m.group(1))
HOLD_STILL_MAX_ABS_TICKS = 4
hd = present_delta(hold)
nd = present_delta(nudge)
assert hold.get("observed_present_after") is not None and hd is not None and abs(hd) <= HOLD_STILL_MAX_ABS_TICKS, (
    "valid_hold must keep present inside the no-load hunt band with a post-command sample",
    hd,
    r,
)
assert nudge.get("observed_present_after") is not None and nd is not None and abs(nd) > HOLD_STILL_MAX_ABS_TICKS, (
    "valid_nudge must move present farther than no-load hunt, not only write a goal",
    nd,
    r,
)
pty_sequence = """$PTY_SEQUENCE_ACTIVE""" == "1"
if pty_sequence:
    assert r.get("cutoff_live_observed") is False, r
    assert r["experiment_status"] != "measured_success", r
    assert r.get("hardware_present") is True
    print("pty-sequence-ok status=%s serial_tx=%s (not metal)" % (r["experiment_status"], r["valid_physical_device_writes"]))
else:
    assert r["hardware_present"] is True
    if not r.get("cutoff_live_observed"):
        assert r["experiment_status"] != "measured_success", r
        raise SystemExit("error: live independent VIN cutoff was not observed; REALITYOS_METAL_CUTOFF_TESTED is operator attestation only and cannot mint measured_success")
    assert r["experiment_status"] == "measured_success", r
    print("metal-proof-ok status=%s serial_tx=%s" % (r["experiment_status"], r["valid_physical_device_writes"]))
PY
if [[ "$PTY_SEQUENCE_ACTIVE" == "1" ]]; then
  if [[ -f "$REPO/docs/metal_proof.json" || -f "$PWD/docs/metal_proof.json" ]]; then
    echo "error: PTY sequence must not install docs/metal_proof.json" >&2
    exit 1
  fi
  echo "metal PTY sequence finished (not physical evidence)"
  echo "pty-report: $ROOT/metal_proof.json"
  exit 0
fi
# Install into the repo only after every success criterion is true.
install -D -m 0644 "$ROOT/metal_proof.json" "$OUT"
REPORT_SRC="$ROOT/METAL_PROOF_REPORT.md"
REPORT_DST="$(dirname "$OUT")/METAL_PROOF_REPORT.md"
if [[ -f "$REPORT_SRC" ]]; then
  install -D -m 0644 "$REPORT_SRC" "$REPORT_DST"
fi

echo "metal campaign finished"
echo "proof: $OUT"
echo "report: $REPORT_DST"
