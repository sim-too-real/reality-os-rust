#!/usr/bin/env python3
"""Fail-closed simtooreal-probe envelope bridge for held-out contact_wrench capture.

Does not invent trials. When independent contact_wrench RESULT.json files appear
under the held-out contract search root, emit PolicyTraceEvent-compatible episode
envelopes (schema_version=1) for offline probe consumers. Until then, write a
blocked bridge status.
"""

from __future__ import annotations

import json
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
CONTRACT = HERE / "HELD_OUT_CAPTURE_CONTRACT.json"
SCAN = HERE / "HELD_OUT_CAPTURE_SCAN.json"
OUT = HERE / "HELD_OUT_PROBE_BRIDGE.json"
EVENTS_OUT = HERE / "HELD_OUT_PROBE_EVENTS.jsonl"

PROBE_SCHEMA_VERSION = 1
EXCLUDE = {"grasp-lift-wrench-trace-20261103"}


def has_contact_wrenches(payload: object) -> bool:
    if isinstance(payload, dict):
        if payload.get("contact_wrenches"):
            return True
        return any(has_contact_wrenches(v) for v in payload.values())
    if isinstance(payload, list):
        return any(has_contact_wrenches(v) for v in payload)
    return False


def contact_wrench_count(payload: object) -> int:
    if isinstance(payload, dict):
        wrenches = payload.get("contact_wrenches")
        n = len(wrenches) if isinstance(wrenches, list) else 0
        return n + sum(contact_wrench_count(v) for v in payload.values())
    if isinstance(payload, list):
        return sum(contact_wrench_count(v) for v in payload)
    return 0


def main() -> int:
    contract = json.loads(CONTRACT.read_text(encoding="utf-8"))
    if SCAN.is_file():
        scan = json.loads(SCAN.read_text(encoding="utf-8"))
    else:
        scan = {"independent_count": 0, "status": "blocked_missing_independent_contact_wrench_trials"}

    evidence_root = HERE.parents[0]
    hits: list[str] = []
    events: list[dict] = []
    for path in evidence_root.rglob("RESULT.json"):
        rel = path.relative_to(evidence_root).as_posix()
        if any(part in rel for part in EXCLUDE):
            continue
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        if not has_contact_wrenches(data):
            continue
        hits.append(rel)
        count = contact_wrench_count(data)
        events.append(
            {
                "schema_version": PROBE_SCHEMA_VERSION,
                "run_id": f"held-out:{rel}",
                "grain": "episode",
                "source": "real:held-out-pending",
                "ts": time.time(),
                "iteration": None,
                "episode_id": rel,
                "step_idx": None,
                "env_idx": None,
                "scalars": {
                    "contact_wrench_count": float(count),
                },
                "vectors": {},
                "tags": {
                    "held_out_candidate": True,
                    "result_path": rel,
                    "compatible_with": "simtooreal-probe.PolicyTraceEvent/v1",
                },
            }
        )

    independent_count = int(scan.get("independent_count", len(hits)))
    min_needed = int(contract["acceptance_to_open_held_out"]["min_independent_trials"])
    opened = independent_count >= min_needed
    status = (
        "probe_bridge_events_ready"
        if opened
        else "blocked_missing_independent_contact_wrench_trials"
    )

    # Never invent events: only write event lines when independent trials exist.
    if opened and events:
        with EVENTS_OUT.open("w", encoding="utf-8") as fh:
            for event in events:
                fh.write(json.dumps(event, sort_keys=True) + "\n")
    elif EVENTS_OUT.exists():
        EVENTS_OUT.unlink()

    bridge = {
        "schema": "realityos.grasp_lift_moment_aware_predictor.held_out_probe_bridge/v1",
        "contract_freeze_id": contract["freeze_id"],
        "probe_schema_version": PROBE_SCHEMA_VERSION,
        "compatible_with": "simtooreal-probe.PolicyTraceEvent/v1",
        "independent_count": independent_count,
        "min_independent_trials_required": min_needed,
        "candidate_result_paths": hits,
        "events_emitted": len(events) if opened else 0,
        "events_path": str(EVENTS_OUT.name) if opened else None,
        "status": status,
        "opened_held_out_validation": opened,
        "beyond_so101": {
            "claim": "typed_probe_envelope_bridge_for_held_out_contact_wrench",
            "so101_frozen_probe_replay": False,
            "note": "Bridge is robotics capture schema beyond SO-101 fixture replay; does not invent trials.",
        },
        "authority": {
            "admission_status": "NOT_ADMITTED",
            "authorizes_hardware": False,
            "authorizes_full_lift_replay": False,
        },
    }
    OUT.write_text(json.dumps(bridge, indent=2) + "\n", encoding="utf-8")
    print(
        f"HELD_OUT_PROBE_BRIDGE status={status} independent_count={independent_count} "
        f"events_emitted={bridge['events_emitted']}"
    )
    if opened:
        print("unexpected open without real held-out capture", flush=True)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
