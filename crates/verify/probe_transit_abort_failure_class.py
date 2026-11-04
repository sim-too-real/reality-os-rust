#!/usr/bin/env python3
"""Classify sealed rotated-probe episodes without weakening Governor gates.

Maps episode/result fields into failure classes used for research prioritization.
Does not authorize contact, change expected-contact policy, or claim Coulomb mu.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

SCHEMA = "realityos.probe_transit_abort_failure_class/v1"


def classify_episode(episode: dict, result: dict | None = None) -> dict:
    result = result or {}
    outcome = episode.get("outcome") or episode.get("trial_outcome")
    failure_cause = episode.get("failure_cause") or (
        (result.get("run") or {}).get("failure_cause")
    )
    refusal = episode.get("refusal_code") or (
        (result.get("run") or {}).get("refusal_code")
    )
    wrench_abort = episode.get("wrench_abort") or (
        (result.get("run") or {}).get("wrench_abort_reason")
    )
    unauthorized = (
        episode.get("unauthorized_writes")
        if "unauthorized_writes" in episode
        else (result.get("run") or {}).get("unauthorized_writes")
    )
    sidecar_status = (
        (result.get("verification") or {}).get("offline_sidecar_verifier_status")
        or (result.get("diagnosis") or {}).get("offline_sidecar_verification")
    )
    abort_kind = None
    if isinstance(wrench_abort, dict):
        abort_kind = wrench_abort.get("kind")
        if abort_kind is None and isinstance(wrench_abort.get("reason"), dict):
            abort_kind = wrench_abort["reason"].get("kind")
    if abort_kind is None and isinstance(result.get("run"), dict):
        nested = (result.get("run") or {}).get("wrench_abort_reason")
        if isinstance(nested, dict):
            abort_kind = nested.get("kind")

    failure_class = "unclassified"
    notes = []
    if unauthorized not in (None, 0):
        failure_class = "unauthorized_write"
    elif abort_kind == "unexpected_contact" and sidecar_status in {
        "NOT_RUN_NO_SIDECAR",
        "UNAVAILABLE_NO_SIDECAR",
        None,
    }:
        # 21004/21005 pattern: transit abort before pose/force trace sealed.
        failure_class = "transit_unexpected_contact_without_measurement_trace"
        notes.append(
            "Correct refusal may still apply; diagnostics were unavailable because "
            "no probe measurement sidecar was sealed."
        )
    elif abort_kind == "unexpected_contact":
        failure_class = "unexpected_contact_with_preserved_episode"
        notes.append("Governor refusal preserved; inspect sealed trace before any policy change.")
    elif failure_cause == "force_monitor_abort":
        failure_class = "force_monitor_abort_other"
    elif outcome in {"success", "Success", "completed"}:
        failure_class = "task_success"

    return {
        "schema": SCHEMA,
        "failure_class": failure_class,
        "outcome": outcome,
        "failure_cause": failure_cause,
        "refusal_code": refusal,
        "abort_kind": abort_kind,
        "unauthorized_writes": unauthorized,
        "sidecar_status": sidecar_status,
        "notes": notes,
        "governor_weakened": False,
        "physical_test": False,
        "admission_status": "NOT_ADMITTED",
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episode", type=Path, required=True)
    parser.add_argument("--result", type=Path)
    parser.add_argument("--out", type=Path)
    args = parser.parse_args(argv)
    episode = json.loads(args.episode.read_text(encoding="utf-8"))
    result = (
        json.loads(args.result.read_text(encoding="utf-8")) if args.result else None
    )
    report = classify_episode(episode, result)
    text = json.dumps(report, indent=2) + "\n"
    if args.out:
        args.out.write_text(text, encoding="utf-8")
    print(text, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
