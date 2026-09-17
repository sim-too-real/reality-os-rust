#!/usr/bin/env bash
# crash_replay stdout is the case JSON for add_case. USB prepare logs on
# stdout abort the first live XL330 campaign: PTY skips that path, but a
# CH340 drop / FTDI latency_timer reset after crash_if rewrites sysfs and
# used to print "metal-campaign: set …" into the capture.
#
# Not physical evidence.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CAMPAIGN="$SCRIPT_DIR/metal-campaign.sh"

if [[ "${1:-}" == "--self-test" || "${BASH_SOURCE[0]}" == "$0" ]]; then
  python3 - "$CAMPAIGN" <<'PY'
import json, pathlib, re, sys

campaign = pathlib.Path(sys.argv[1]).read_text()
log = (
    "metal-campaign: set /sys/devices/pci0000:00/0000:00:14.0/usb1/1-1/"
    "1-1:1.0/ttyUSB1/power/control=on "
    "(USB autosuspend can miss the 40 ms live deadline)"
)
case = {
    "name": "crash_restart_before_prepare",
    "expected_authorization": False,
    "serial_tx_delta": 0,
}
polluted = log + "\n" + json.dumps(case)
try:
    json.loads(polluted)
except json.JSONDecodeError:
    pass
else:
    raise SystemExit("error: add_case json.loads must refuse a prepare log prefix")

fn = re.search(r"\ncrash_replay\(\) \{\n(.*?)\n\}\n", campaign, re.S)
if not fn:
    raise SystemExit("error: crash_replay function not found")
body = fn.group(1)
if "} >&2" not in body or 'printf \'%s\\n\' "$rec"' not in body:
    raise SystemExit(
        "error: crash_replay must send the work block to stderr and print only "
        "the case JSON (add_case captures stdout; USB prepare logs are not JSON)"
    )
tail = body[body.rindex("} >&2") :]
if 'printf \'%s\\n\' "$rec"' not in tail:
    raise SystemExit("error: case JSON must be printed after the stderr work block")

missing = []
for i, line in enumerate(campaign.splitlines(), 1):
    if re.search(r'echo\s+"metal-campaign:', line) and ">&2" not in line:
        missing.append(f"{i}:{line.strip()}")
if missing:
    raise SystemExit(
        "error: metal-campaign: log lines must go to stderr "
        "(crash_replay / add_case stdout is JSON):\n  " + "\n  ".join(missing)
    )

points = [
    "before_prepare",
    "after_prepare_before_write",
    "during_write",
    "after_serial_tx_before_status",
    "after_write_before_ack",
    "after_ack",
]
for point in points:
    if not re.search(r'add_case "\$\(crash_replay %s ' % re.escape(point), campaign):
        raise SystemExit("error: missing crash_replay invocation for %s" % (point,))
if 'crash_restart_${point}' not in campaign:
    raise SystemExit(
        "error: crash_replay must still write case records named crash_restart_<point>"
    )

if "after_serial_tx_before_status" not in body:
    raise SystemExit("error: crash_replay dropped after_serial_tx_before_status")
if "if after > before:" not in body:
    raise SystemExit(
        "error: crash_replay must refuse retransmission when serial_tx increases"
    )
if '"serial_tx_delta": max(0, after-before)' not in body:
    raise SystemExit(
        "error: crash_replay must record serial_tx_delta from the measured before/after"
    )
if '"physical_writes_delta": max(0, after-before)' not in body:
    raise SystemExit(
        "error: crash_replay must record physical_writes_delta from the measured before/after"
    )

code = lambda line: line.split("#", 1)[0]
unbounded = []
for i, line in enumerate(campaign.splitlines(), 1):
    if re.search(r'\bwait\s+"\$\{?AUTH_PID', code(line)):
        unbounded.append("%s:%s" % (i, line.strip()))
if unbounded:
    raise SystemExit(
        "error: unbounded wait on AUTH_PID (wait only after the pid is known "
        "exited/reapable):\n  " + "\n  ".join(unbounded)
    )

funcs = {}
name = None
buf = []
for line in campaign.splitlines():
    start = re.match(r"^([A-Za-z_][A-Za-z0-9_]*)\(\) \{", line)
    if start:
        name = start.group(1)
        buf = [line]
        continue
    if name is not None:
        buf.append(line)
        if line == "}":
            funcs[name] = "\n".join(buf)
            name = None
            buf = []

reap_helpers = []
for fname, fbody in funcs.items():
    wait_line = None
    for line in fbody.splitlines():
        if re.search(r'\bwait\s+"\$\{?pid\}?"', code(line)):
            wait_line = line
            break
    if wait_line is None:
        continue
    before = fbody.split(wait_line, 1)[0]
    if not re.search(r"(kill -0|auth_process_alive|auth_pid_alive)", before):
        continue
    if not re.search(
        r"(kill -0|auth_process_alive|auth_pid_alive).{0,240}return 1",
        before,
        re.S,
    ):
        raise SystemExit(
            "error: %s wait is reachable while the pid may still be running" % (fname,)
        )
    reap_helpers.append(fname)
if not reap_helpers:
    raise SystemExit(
        "error: missing bounded reap helper that waits only after the pid is "
        "known exited/reapable"
    )

if "metal_authority_shutdown_timeout" not in campaign:
    raise SystemExit(
        "error: unconfirmed authority shutdown must fail with "
        "metal_authority_shutdown_timeout"
    )
code_blob = " ".join(code(line) for line in campaign.splitlines() if code(line).strip())
if re.search(r'as_authority\s+"\$SMOKE"[\s\S]{0,200}serve[\s\S]{0,120}&', code_blob):
    raise SystemExit(
        "error: do not background as_authority for serve; command substitution "
        "waits on a backgrounded function (PTY hang class)"
    )
print("metal-campaign-case-stdout-ok")
PY
  "$CAMPAIGN" --self-test-authority-lifecycle
fi
