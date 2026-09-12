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
print("metal-campaign-case-stdout-ok")
PY
fi
