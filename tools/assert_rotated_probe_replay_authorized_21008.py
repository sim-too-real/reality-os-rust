#!/usr/bin/env python3
"""Refuse 21008 replay unless FREEZE.json authorizes it on evidence.

Does not run the simulator. Does not weaken Governor thresholds.
Exit 0 only when replay_authorized is true and block_reason is null/absent.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_FREEZE = (
    ROOT
    / "docs/superpowers/evidence/rotated-probe-heldout-seed-21008-20261104/FREEZE.json"
)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--freeze", type=Path, default=DEFAULT_FREEZE)
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args(argv)

    if not args.freeze.is_file():
        report = {"ok": False, "error": f"missing freeze: {args.freeze}"}
        print(json.dumps(report, indent=2))
        return 2

    freeze = json.loads(args.freeze.read_text(encoding="utf-8"))
    authorized = freeze.get("replay_authorized") is True
    block = freeze.get("replay_block_reason")
    thresholds = freeze.get("preserved_motion_inference_guard") or {}
    ok = (
        authorized
        and block in (None, "")
        and thresholds.get("minimum_planar_motion_m") == 0.001
        and thresholds.get("minimum_tangential_force_lower_bound_n") == 0.5
        and thresholds.get("thresholds_changed") is False
    )
    report = {
        "ok": ok,
        "freeze": str(args.freeze),
        "status": freeze.get("status"),
        "replay_authorized": freeze.get("replay_authorized"),
        "replay_block_reason": block,
        "thresholds_unchanged": thresholds.get("thresholds_changed") is False,
        "admission_status": (freeze.get("scope_limits") or {}).get("admission_status"),
    }
    print(json.dumps(report, indent=2))
    return 0 if ok else 3


if __name__ == "__main__":
    raise SystemExit(main())
