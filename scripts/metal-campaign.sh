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
OUT="${REALITYOS_METAL_PROOF:-docs/metal_proof.json}"
CUTOFF_TESTED="${REALITYOS_METAL_CUTOFF_TESTED:-0}"

# Authority UID cannot write the repo `docs/` tree. Resolve the install path
# now; the reporter writes into $ROOT, then root copies here.
if [[ "$OUT" != /* ]]; then
  OUT="$(pwd)/$OUT"
fi

if [[ "$(id -u)" -ne 0 ]]; then
  echo "error: run as root to switch UIDs; root is not the tested actor" >&2
  exit 2
fi
if [[ -z "$BIN_DIR" ]]; then
  echo "error: set REALITYOS_METAL_BIN to the directory containing the metal binaries" >&2
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
set_usb_serial_latency() {
  local dev="$1"
  local real name timer
  real="$(readlink -f "$dev" 2>/dev/null || echo "$dev")"
  name="$(basename "$real")"
    case "$name" in
    ttyUSB*|ttyACM*|ttyCH341*) ;;
    *) return 0 ;;
  esac
  for timer in \
    "/sys/bus/usb-serial/devices/${name}/latency_timer" \
    "/sys/class/tty/${name}/device/latency_timer"; do
    if [[ -e "$timer" ]] && echo 1 >"$timer" 2>/dev/null; then
      echo "metal-campaign: set $timer=1 (USB-UART default 16 ms can miss the 40 ms live deadline)"
      return 0
    fi
  done
}

# Ubuntu usbcore autosuspend is often 2 s. An idle gap between campaign
# cases then makes the next USB-UART xfer miss the 40 ms live deadline
# and look like bus_lost / a watchdog miss. PTY has no sysfs node.
disable_usb_autosuspend() {
  local dev="$1"
  local real name node
  real="$(readlink -f "$dev" 2>/dev/null || echo "$dev")"
  name="$(basename "$real")"
  case "$name" in
    ttyUSB*|ttyACM*|ttyCH341*) ;;
    *) return 0 ;;
  esac
  node="$(readlink -f "/sys/class/tty/${name}/device" 2>/dev/null || true)"
  while [[ -n "$node" && "$node" != / && "$node" != /sys ]]; do
    if [[ -f "$node/power/control" ]]; then
      if echo on >"$node/power/control" 2>/dev/null; then
        echo "metal-campaign: set $node/power/control=on (USB autosuspend can miss the 40 ms live deadline)"
      fi
    fi
    if [[ -f "$node/power/autosuspend_delay_ms" ]]; then
      echo -1 >"$node/power/autosuspend_delay_ms" 2>/dev/null || true
    fi
    if [[ -f "$node/idVendor" ]]; then
      break
    fi
    node="$(dirname "$node")"
  done
}

# Walk sysfs from the tty to the USB device (first idVendor) and read
# one attribute from that node only. A CH340/CP2102 with an empty serial
# must not inherit a parent hub serial — ATTRS{serial}==<hub> would match
# every tty on the hub after udev rename.
usb_sysfs_value() {
  local dev="$1" key="$2"
  local name node
  name="$(basename "$(readlink -f "$dev" 2>/dev/null || echo "$dev")")"
  node="$(readlink -f "/sys/class/tty/${name}/device" 2>/dev/null || true)"
  while [[ -n "$node" && "$node" != / && "$node" != /sys ]]; do
    if [[ -f "$node/idVendor" ]]; then
      if [[ -f "$node/$key" ]]; then
        tr -d '\n' <"$node/$key"
        return 0
      fi
      return 1
    fi
    node="$(dirname "$node")"
  done
  return 1
}

# busnum:devpath:idVendor:idProduct. CH340/CP2102 often have an empty
# USB serial; udev change can still keep this port key.
usb_sysfs_port_key() {
  local dev="$1"
  local bus dest vid pid
  bus="$(usb_sysfs_value "$dev" busnum || true)"
  dest="$(usb_sysfs_value "$dev" devpath || true)"
  vid="$(usb_sysfs_value "$dev" idVendor || true)"
  pid="$(usb_sysfs_value "$dev" idProduct || true)"
  if [[ -n "$vid" && -n "$pid" ]]; then
    echo "${bus:-0}:${dest:-nodevpath}:${vid}:${pid}"
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
# idVendor is visible. Recording an empty port then binding tty name+rdev
# makes the later usb:vid:pid:devpath serve identity miss.
wait_usb_sysfs_identity() {
  local dev="$1"
  local real name i
  real="$(readlink -f "$dev" 2>/dev/null || echo "$dev")"
  name="$(basename "$real")"
  case "$name" in
    ttyUSB*|ttyACM*|ttyCH341*) ;;
    *) return 0 ;;
  esac
  for i in $(seq 1 40); do
    if [[ -e "$real" ]] && usb_sysfs_value "$real" idVendor >/dev/null; then
      return 0
    fi
    sleep 0.1
  done
  echo "warning: USB sysfs idVendor never appeared for $real; identity may fall back to tty name+rdev" >&2
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
      echo "$dev"
      return 0
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
    echo "warning: USB-UART $dev did not reappear with the recorded identity after 4s" >&2
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
  chown "$AUTHORITY_USER:$AUTHORITY_USER" "$cfg" 2>/dev/null || true
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
  if echo "$holders" | grep -qE 'ModemManager'; then
    echo "metal-campaign: $real is held by ModemManager; stopping ModemManager.service"
    systemctl stop ModemManager.service 2>/dev/null || true
    STOPPED_MM=1
  fi
}

prepare_usb_serial_host() {
  local dev="$1"
  local real name rules
  real="$(readlink -f "$dev" 2>/dev/null || echo "$dev")"
  name="$(basename "$real")"
  # Crash-replay and disconnect restart close exclusive, then reopen. Our
  # serve may still hold the tty for a few hundred ms; ModemManager can
  # grab it in that gap. Wait for our close, then refuse a foreign holder.
  # The PTY Protocol 2.0 stand-in must keep the master open. fuser on
  # /dev/pts/N reports that python as a holder; treating it as
  # ModemManager exits 2 before the first serve (CI os-users). Exclusive
  # tty is a USB-UART check. PTY open is already non-TIOCEXCL.
  if [[ "$PTY_SEQUENCE_ACTIVE" != "1" ]] && command -v fuser >/dev/null 2>&1 && [[ -e "$real" ]]; then
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
  fi
  case "$name" in
    ttyUSB*|ttyACM*|ttyCH341*) ;;
    *)
      set_usb_serial_latency "$dev"
      return 0
      ;;
  esac
  wait_usb_sysfs_identity "$dev" || true
  real="$(readlink -f "$dev" 2>/dev/null || echo "$dev")"
  if [[ -z "${METAL_USB_SERIAL:-}" ]]; then
    METAL_USB_SERIAL="$(usb_sysfs_value "$real" serial || true)"
  fi
  if [[ -z "${METAL_USB_PORT:-}" ]]; then
    METAL_USB_PORT="$(usb_sysfs_port_key "$real" || true)"
  fi
  if [[ -d /run/udev/rules.d ]]; then
    if [[ -n "$METAL_USB_SERIAL" ]]; then
      rules="/run/udev/rules.d/99-realityos-metal-usb.rules"
    elif [[ -n "$METAL_USB_PORT" ]]; then
      rules="/run/udev/rules.d/99-realityos-metal-usbport.rules"
    else
      rules="/run/udev/rules.d/99-realityos-metal-${name}.rules"
    fi
    if [[ ! -f "$rules" ]]; then
      if [[ -n "$METAL_USB_SERIAL" ]]; then
        cat >"$rules" <<EOF
ACTION=="add|change", SUBSYSTEM=="tty", ATTRS{serial}=="${METAL_USB_SERIAL}", ENV{ID_MM_DEVICE_IGNORE}="1", ENV{ID_BRLTTY}="0", OWNER="${AUTHORITY_USER}", GROUP="${AUTHORITY_USER}", MODE="0600"
EOF
      elif [[ -n "$METAL_USB_PORT" ]]; then
        IFS=: read -r usb_bus usb_dest usb_vid usb_pid <<<"$METAL_USB_PORT"
        cat >"$rules" <<EOF
ACTION=="add|change", SUBSYSTEM=="tty", ATTRS{idVendor}=="${usb_vid}", ATTRS{idProduct}=="${usb_pid}", ATTRS{busnum}=="${usb_bus}", ATTRS{devpath}=="${usb_dest}", ENV{ID_MM_DEVICE_IGNORE}="1", ENV{ID_BRLTTY}="0", OWNER="${AUTHORITY_USER}", GROUP="${AUTHORITY_USER}", MODE="0600"
EOF
      else
        cat >"$rules" <<EOF
ACTION=="add|change", KERNEL=="${name}", ENV{ID_MM_DEVICE_IGNORE}="1", ENV{ID_BRLTTY}="0", OWNER="${AUTHORITY_USER}", GROUP="${AUTHORITY_USER}", MODE="0600"
EOF
      fi
      UDEV_RULE="$rules"
      udevadm control --reload 2>/dev/null || true
      udevadm trigger --action=change --sysname-match="$name" 2>/dev/null || true
      udevadm settle --timeout=2 2>/dev/null || true
      echo "metal-campaign: installed $rules (ID_MM_DEVICE_IGNORE + ID_BRLTTY=0 + 0600 ${AUTHORITY_USER})"
    else
      UDEV_RULE="$rules"
    fi
  fi
  DEVICE="$(stabilize_metal_device "$dev")"
  export REALITYOS_METAL_DEVICE="$DEVICE"
  sync_metal_device_config
  real="$(readlink -f "$DEVICE" 2>/dev/null || echo "$DEVICE")"
  if [[ -e "$real" ]]; then
    chown "$AUTHORITY_USER:$AUTHORITY_USER" "$real" 2>/dev/null || true
    chmod 0600 "$real" 2>/dev/null || true
  fi
  # Linux asserts DTR on first open. Cheap FTDI/CP2102 wire DTR to RESET.
  # Pre-open -hupcl is not enough: the next serialport open restores
  # kernel-default HUPCL. After TIOCEXCL, stty on the node and on
  # /proc/<pid>/fd/N is EBUSY. The driver takes exclusive, then clears
  # HUPCL on that fd via termios. Campaign still clears here so a leftover
  # holder close is less likely to DTR-RESET before serve (U2D2 has
  # no DTR-RESET).
  if [[ -e "$real" ]]; then
    /bin/stty -F "$real" -hupcl >/dev/null 2>&1 || true
  fi
  set_usb_serial_latency "$DEVICE"
  disable_usb_autosuspend "$DEVICE"
}

cleanup_usb_serial_host() {
  if [[ -n "$UDEV_RULE" && -f "$UDEV_RULE" ]]; then
    rm -f "$UDEV_RULE"
    udevadm control --reload 2>/dev/null || true
  fi
  if [[ "$STOPPED_MM" == "1" ]]; then
    systemctl start ModemManager.service 2>/dev/null || true
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
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
"$SCRIPT_DIR/metal-kill-serve.sh" "$ROOT" || true
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

STAGE="${REALITYOS_METAL_STAGE:-/tmp/realityos-metal-bin}"
rm -rf "$STAGE"
install -d -m 0755 "$STAGE"
for b in realityos-metal-smoke realityos-metal-propose; do
  if [[ ! -x "$BIN_DIR/$b" ]]; then
    echo "error: missing $BIN_DIR/$b" >&2
    exit 2
  fi
  install -m 0755 "$BIN_DIR/$b" "$STAGE/$b"
done
BIN_DIR="$STAGE"
SMOKE="$BIN_DIR/realityos-metal-smoke"
PROP="$BIN_DIR/realityos-metal-propose"

export REALITYOS_METAL_ROOT="$ROOT"
export REALITYOS_METAL_DEVICE="$DEVICE"
"$SCRIPT_DIR/metal-deploy.sh"

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

as_authority "$SMOKE" --root "$ROOT" --device "$DEVICE" init
as_authority "$SMOKE" --root "$ROOT" --device "$DEVICE" probe
as_authority "$SMOKE" --root "$ROOT" bind-measured

writes() { cat "$ROOT/bus/writes" 2>/dev/null || echo 0; }
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
  # udev may reset the tty to 0660 dialout after open. Re-apply exclusive mode.
  if [[ -e "$DEVICE" ]]; then
    chown "$AUTHORITY_USER:$AUTHORITY_USER" "$DEVICE" 2>/dev/null || true
    chmod 0600 "$DEVICE" 2>/dev/null || true
    set_usb_serial_latency "$DEVICE"
    disable_usb_autosuspend "$DEVICE"
  fi
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
if [[ -e "$DEVICE" ]]; then
  chown "$AUTHORITY_USER:$AUTHORITY_USER" "$DEVICE" 2>/dev/null || true
  chmod 0600 "$DEVICE" 2>/dev/null || true
  set_usb_serial_latency "$DEVICE"
  disable_usb_autosuspend "$DEVICE"
fi

PROBE="$(as_autonomy env METAL_AUTHORITY_PID="${SMOKE_PID:-$AUTH_PID}" "$PROP" --root "$ROOT" --authority-pid "${SMOKE_PID:-$AUTH_PID}" os-probe)"
echo "os-probe=$PROBE"

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
assert p["modify_journal"] is False
assert p["take_actuator_lock"] is False
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
  local before after respfile ack_before ack_after pb pa gp
  before="$(writes)"
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
  ack_after="$(acks)"
  pa="$(present)"
  gp="$(goalpos)"
  python3 - "$name" "$proposal" "$layer" "$expected" "$before" "$after" "$ack_before" "$ack_after" "$respfile" "$pb" "$pa" "$gp" <<'PY'
import json, os, sys
name, proposal, layer, expected, before, after, ab, aa, path, pb, pa, gp = sys.argv[1:13]
before, after, ab, aa = map(int, (before, after, ab, aa))
expected = expected == "true"
require = os.environ.get("MEASURE_REQUIRE", "").strip()
forbid = os.environ.get("MEASURE_FORBID", "").strip()
delta = max(0, after - before)
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
ack = aa > ab
if r.get("device_acks") is not None and r.get("physical_writes") is not None:
    ack = ack or bool(r.get("ok") and aa > ab)
blob = " ".join(str(x) for x in (r.get("violations") or []))
blob = f"{blob} {r.get('stage','')} {r.get('status','')}"
def has_token(spec):
    return any(tok and tok in blob for tok in spec.split("|"))
if expected:
    if not r.get("ok") or delta < 1:
        sys.exit("error: authorized case %s did not produce a physical write: %s delta=%s" % (name, r, delta))
else:
    if r.get("ok") or delta > 0:
        sys.exit("error: unauthorized case %s executed or wrote: %s delta=%s" % (name, r, delta))
    if require and not has_token(require):
        sys.exit("error: case %s missing required token %r in %s" % (name, require, r))
    if forbid and has_token(forbid):
        sys.exit("error: case %s has forbidden token %r (vacuous refuse): %s" % (name, forbid, r))
rec = {
    "name": name,
    "expected_authorization": expected,
    "decision_result": f'{r.get("stage","")}:{r.get("status","")}',
    "writes_before": before,
    "writes_after": after,
    "write_delta": delta,
    "device_acknowledgement": ack and bool(r.get("ok")),
    "observed_motion": motion,
    "blocking_layer": layer,
    "journal_result": "consumed" if r.get("ok") else "no_consume",
    "proposal": proposal,
    "unauthorized_write": (not expected) and delta > 0,
}
print(json.dumps(rec))
PY
  rm -f "$respfile"
}

CASES_FILE="$ROOT/os_metal_cases.json"
printf '%s\n' '[]' > "$CASES_FILE"
chmod 0644 "$CASES_FILE"
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
add_case "$(measure nan_action 'action=NaN' AUTHORIZATION_BLOCKED false "$PROP" --root "$ROOT" --id metal-nan --verb drive --action nan propose)"
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
  python3 - <<PY
import json, sys
r = json.loads('''$sensor''')
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
as_authority bash -c "printf '%s' '{\"firmware_id\":\"xl330-m288:1190:255\"}' > '$ROOT/bus/hot_swap.json'"
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
PROBE_REC="$(python3 - <<PY
import json
p = json.loads('''$PROBE''')
before = int("$BEFORE")
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
  # driver. If act() fails first, those later points never fire — fail closed
  # instead of `wait $AUTH_PID` hanging on a live serve (PTY campaign hang).
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
  before="$(writes)"
  as_autonomy env METAL_CMD_ID="$cid" "$PROP" --root "$ROOT" replay >/tmp/metal-"$cid"-replay.json || true
  after="$(writes)"
  rec="$(python3 - <<PY
import json, sys
before=int("$before"); after=int("$after")
if after > before:
    sys.exit("error: crash/restart $point retried a command that may have reached hardware (%s→%s)" % (before, after))
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

add_case "$(crash_replay after_prepare_before_write metal-crash-prep)"
add_case "$(crash_replay during_write metal-crash-during)"
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
  CUTOFF_TESTED=1
  add_case "$(MEASURE_REQUIRE='dxl_io|driver not connected|online_hardware_disconnected|metal_live_io_deadline' MEASURE_FORBID=software_watchdog_miss measure vin_cutoff_live 'propose after VIN open' AUTHORIZATION_BLOCKED false env METAL_CMD_ID=metal-cutoff "$PROP" --root "$ROOT" propose-id)"
fi

REPO="$(cd "$SCRIPT_DIR/.." && pwd)"
COMMIT="$(git -C "$REPO" -c safe.directory="$REPO" rev-parse HEAD 2>/dev/null || echo unknown)"
DATE="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
MEASURED="$(cat "$ROOT/measured.json" 2>/dev/null || echo '{}')"
FRESH="$(cat "$ROOT/bus/sensor_freshness.json" 2>/dev/null || echo '{}')"
python3 - <<PY
import json, os
p = json.loads('''$PROBE''')
measured = json.loads('''$MEASURED''')
fresh = json.loads('''$FRESH''') if '''$FRESH'''.strip() else {}
if isinstance(measured, dict) and fresh.get("vin_0.1v") is not None:
    measured = dict(measured)
    measured["vin_0.1v"] = fresh.get("vin_0.1v")
inner = measured.get("measured") if isinstance(measured, dict) and isinstance(measured.get("measured"), dict) else measured
model = inner.get("model") if isinstance(inner, dict) else None
hardware_model = {1190: "XL330-M288-T", 1200: "XL330-M077-T"}.get(model)
if hardware_model is None:
    raise SystemExit("error: proof meta refuses unknown/missing XL330 model: %r" % (model,))
serial = ""
if isinstance(inner, dict):
    serial = str(inner.get("serial") or "")
if not serial:
    raise SystemExit("error: proof meta refuses empty measured serial")
if serial.startswith("tty:"):
    controller = "Dynamixel Protocol 2.0 UART (measured tty name+rdev; no USB serial)"
elif serial.startswith("usb:"):
    controller = "Dynamixel Protocol 2.0 USB-UART (measured usb vid:pid:devpath)"
else:
    controller = "Dynamixel Protocol 2.0 USB-UART (measured adapter serial)"
cutoff = os.environ.get("REALITYOS_METAL_CUTOFF_TESTED","0") == "1"
meta = {
  "hardware_model": hardware_model,
  "controller_model": controller,
  "real_device_identity": measured,
  "software_commit_sha": "$COMMIT",
  "authority_uid": "$AUTHORITY_USER",
  "autonomy_uid": "$AUTONOMY_USER",
  "test_date": "$DATE",
  "hardware_present": True,
  "used_os_monotonic_clock": True,
  "used_hardware_driver_port": True,
  "cutoff_mechanism": "bench PSU switch or SPST on servo 5V VIN, independent of Reality OS",
  "cutoff_tested": cutoff,
  "direct_device_open_attempts": int(p.get("direct_device_open_attempts") or 0),
  "direct_device_open_successes": int(p.get("direct_device_open_successes") or 0),
  "duplicate_writes_after_restart": 0,
  "sensor_source": fresh.get("sensor_source") or "xl330 registers + realtime tick",
  "device_capture_s": fresh.get("device_capture_s"),
  "authority_receive_s": fresh.get("authority_receive_s"),
  "freshness_threshold_s": fresh.get("freshness_threshold_s"),
}
open("$ROOT/proof_meta.json","w").write(json.dumps(meta, indent=2))
print("meta-written")
PY

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
assert any(c.get("name") == "valid_hold" and int(c.get("write_delta") or 0) > 0 for c in r.get("cases") or []), r
assert any(c.get("name") == "valid_nudge" and int(c.get("write_delta") or 0) > 0 for c in r.get("cases") or []), r
def present_delta(name):
    c = next((x for x in (r.get("cases") or []) if x.get("name") == name), None)
    import re
    m = re.search(r"delta=([-\d]+|None)", (c or {}).get("observed_motion") or "")
    if not m or m.group(1) == "None":
        return None
    return int(m.group(1))
# Same band as crates/metal/src/proof.rs HOLD_STILL_MAX_ABS_TICKS.
HOLD_STILL_MAX_ABS_TICKS = 4
hd = present_delta("valid_hold")
nd = present_delta("valid_nudge")
assert hd is not None and abs(hd) <= HOLD_STILL_MAX_ABS_TICKS, (
    "valid_hold must keep present inside the no-load hunt band",
    hd,
    r,
)
assert nd is not None and abs(nd) > HOLD_STILL_MAX_ABS_TICKS, (
    "valid_nudge must move present farther than no-load hunt, not only write a goal",
    nd,
    r,
)
pty_sequence = """$PTY_SEQUENCE_ACTIVE""" == "1"
if pty_sequence:
    assert r["cutoff_tested"] is False, r
    assert r["experiment_status"] != "measured_success", r
    assert r.get("hardware_present") is True
    print("pty-sequence-ok status=%s writes=%s (not metal)" % (r["experiment_status"], r["valid_physical_device_writes"]))
else:
    assert r["hardware_present"] is True
    assert r["cutoff_tested"] is True, r
    assert r["experiment_status"] == "measured_success", r
    print("metal-proof-ok status=%s writes=%s" % (r["experiment_status"], r["valid_physical_device_writes"]))
PY
if [[ "$PTY_SEQUENCE_ACTIVE" == "1" ]]; then
  if [[ -f docs/metal_proof.json || -f "$PWD/docs/metal_proof.json" ]]; then
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
