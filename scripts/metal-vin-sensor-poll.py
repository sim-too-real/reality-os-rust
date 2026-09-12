#!/usr/bin/env python3
"""Autonomy-side live VIN wait. One connect per sample on ipc.sock.

Prints one sensor JSON body to stdout when a VIN token appears.
Does not write the metal root (0751 / bus 0700). UART death, truncated
IPC, a healthy ok=true sample, and a missing socket are not VIN.
"""

import json
import socket
import sys
import time

TOKENS = (
    "dxl_vin_outside_wizard_limits",
    "dxl_vin_unreadable",
)


def recv_line(sock, rec_deadline):
    buf = b""
    while time.monotonic() < rec_deadline:
        remain = rec_deadline - time.monotonic()
        if remain <= 0:
            return None
        sock.settimeout(min(0.05, remain))
        try:
            chunk = sock.recv(4096)
        except socket.timeout:
            continue
        except OSError:
            return None
        if not chunk:
            return None
        buf += chunk
        nl = buf.find(b"\n")
        if nl >= 0:
            return buf[:nl].decode("utf-8", "replace")
    return None


def one_sensor(sock_path, deadline):
    remain = deadline - time.monotonic()
    if remain <= 0:
        return None
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
        sock.settimeout(min(0.05, remain))
        sock.connect(sock_path)
        sock.settimeout(min(2.0, max(0.01, deadline - time.monotonic())))
        sock.sendall(
            json.dumps(
                {
                    "op": "sensor",
                    "verb": "hold",
                    "command_id": "metal-vin-wait",
                    "proposer": "autonomy",
                }
            ).encode()
            + b"\n"
        )
        return recv_line(sock, min(deadline, time.monotonic() + 2.0))
    except OSError:
        return None
    finally:
        sock.close()


def vin_body(raw):
    if not raw or not raw.strip():
        return None
    try:
        body = json.loads(raw)
    except json.JSONDecodeError:
        return None
    if not isinstance(body, dict) or body.get("ok") is True:
        return None
    blob = json.dumps(body).lower()
    if any(tok in blob for tok in TOKENS):
        return body
    return None


def main(argv):
    if len(argv) < 3:
        print("usage: metal-vin-sensor-poll.py SOCK SECONDS", file=sys.stderr)
        return 2
    sock_path = argv[1]
    try:
        seconds = float(argv[2])
    except ValueError:
        return 2
    if seconds <= 0:
        return 2
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        body = vin_body(one_sensor(sock_path, deadline))
        if body is None:
            continue
        json.dump(body, sys.stdout, separators=(",", ":"))
        sys.stdout.write("\n")
        sys.stdout.flush()
        return 0
    return 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
