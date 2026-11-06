#!/usr/bin/env python3
"""Fail-closed scan for independent contact_wrench trials (does not invent them)."""

from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = ROOT / "docs" / "superpowers" / "evidence"
HERE = Path(__file__).resolve().parent
CONTRACT = HERE / "HELD_OUT_CAPTURE_CONTRACT.json"
STATUS = HERE / "HELD_OUT_STATUS.json"
OUT = HERE / "HELD_OUT_CAPTURE_SCAN.json"

DEV_CONTROLS = {
    "v7:grasp_fixture:arm_gripper_lift:pinch_grasp_lift:21006:21",
    "v7:grasp_fixture:arm_gripper_lift:pinch_grasp_lift:21010:25",
}
EXCLUDE = {
    "grasp-lift-wrench-trace-20261103",
}


def has_contact_wrenches(payload: object) -> bool:
    if isinstance(payload, dict):
        if payload.get("contact_wrenches"):
            return True
        return any(has_contact_wrenches(v) for v in payload.values())
    if isinstance(payload, list):
        return any(has_contact_wrenches(v) for v in payload)
    return False


def main() -> int:
    contract = json.loads(CONTRACT.read_text(encoding="utf-8"))
    hits = []
    for path in EVIDENCE.rglob("RESULT.json"):
        rel = path.relative_to(EVIDENCE).as_posix()
        if any(part in rel for part in EXCLUDE):
            continue
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        if not has_contact_wrenches(data):
            continue
        hits.append(rel)

    independent = [h for h in hits if "grasp-lift-wrench-trace-20261103" not in h]
    min_needed = int(contract["acceptance_to_open_held_out"]["min_independent_trials"])
    opened = len(independent) >= min_needed
    status = (
        "held_out_capture_ready"
        if opened
        else "blocked_missing_independent_contact_wrench_trials"
    )
    scan = {
        "schema": "realityos.grasp_lift_moment_aware_predictor.held_out_capture_scan/v1",
        "contract_freeze_id": contract["freeze_id"],
        "result_json_with_contact_wrenches_outside_exclude": hits,
        "independent_count": len(independent),
        "min_independent_trials_required": min_needed,
        "development_controls_excluded": sorted(DEV_CONTROLS),
        "status": status,
        "opened_held_out_validation": opened,
        "authority": contract["authority"],
    }
    OUT.write_text(json.dumps(scan, indent=2) + "\n", encoding="utf-8")

    prior = json.loads(STATUS.read_text(encoding="utf-8"))
    if prior.get("status") != "blocked_missing_independent_contact_wrench_trials" and not opened:
        print("HELD_OUT_STATUS drifted unexpectedly", file=sys.stderr)
        return 1
    if opened:
        print("HELD_OUT_CAPTURE unexpected open without owner/capture work", file=sys.stderr)
        return 1
    print(f"HELD_OUT_CAPTURE_SCAN status={status} independent_count={len(independent)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
