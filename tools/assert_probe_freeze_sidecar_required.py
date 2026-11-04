#!/usr/bin/env python3
"""Refuse contact-class freeze seal-complete without a non-empty probe sidecar.

Does not invent measurements. Does not weaken Governor thresholds.
Sealed historical 21004/21005 evidence remains unchanged; this gate is for
new freeze packages that claim contact/transit observation completeness.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

CONTACT_CLASS_MARKERS = (
    "unexpected_contact",
    "transit_abort",
    "contact_class",
    "require_probe_sidecar",
)


def assess_freeze_dir(evidence_dir: Path) -> dict:
    freeze_path = evidence_dir / "FREEZE.json"
    sidecar_path = evidence_dir / "PROBE_MEASUREMENT_SIDECAR.json"
    out: dict = {
        "schema": "realityos.probe_freeze_sidecar_required/v1",
        "evidence_dir": str(evidence_dir),
        "governor_weakened": False,
        "admission_status": "NOT_ADMITTED",
        "invented_measurements": False,
    }
    if not freeze_path.is_file():
        out.update(
            {
                "status": "FAIL_NO_FREEZE",
                "seal_allowed": False,
                "reason": "FREEZE.json missing",
            }
        )
        return out

    freeze = json.loads(freeze_path.read_text(encoding="utf-8"))
    blob = json.dumps(freeze, sort_keys=True).lower()
    contact_class = any(marker in blob for marker in CONTACT_CLASS_MARKERS)
    # Also treat explicit freeze flag.
    if freeze.get("require_probe_sidecar") is True:
        contact_class = True
    if freeze.get("observation_class") in {
        "contact",
        "transit_abort",
        "probe_contact",
    }:
        contact_class = True

    out["contact_class_required"] = contact_class
    if not contact_class:
        out.update(
            {
                "status": "PASS_NOT_CONTACT_CLASS",
                "seal_allowed": True,
                "reason": "freeze is not contact-class; sidecar gate not applicable",
            }
        )
        return out

    if not sidecar_path.is_file():
        out.update(
            {
                "status": "FAIL_NO_SIDECAR",
                "seal_allowed": False,
                "reason": "contact-class freeze lacks PROBE_MEASUREMENT_SIDECAR.json",
            }
        )
        return out

    size = sidecar_path.stat().st_size
    if size <= 2:
        out.update(
            {
                "status": "FAIL_EMPTY_SIDECAR",
                "seal_allowed": False,
                "reason": "sidecar present but empty",
                "sidecar_bytes": size,
            }
        )
        return out

    # Structural non-empty JSON object/array required; no invented rows.
    try:
        payload = json.loads(sidecar_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        out.update(
            {
                "status": "FAIL_SIDECAR_NOT_JSON",
                "seal_allowed": False,
                "reason": f"sidecar JSON parse failed: {exc}",
                "sidecar_bytes": size,
            }
        )
        return out

    if payload in ({}, [], None):
        out.update(
            {
                "status": "FAIL_EMPTY_SIDECAR",
                "seal_allowed": False,
                "reason": "sidecar JSON is empty object/array",
                "sidecar_bytes": size,
            }
        )
        return out

    out.update(
        {
            "status": "PASS_SIDECAR_PRESENT",
            "seal_allowed": True,
            "reason": "contact-class freeze has non-empty sidecar",
            "sidecar_bytes": size,
        }
    )
    return out


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--evidence-dir", type=Path, required=True)
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()
    result = assess_freeze_dir(args.evidence_dir)
    if args.json:
        print(json.dumps(result, indent=2))
    else:
        print(f"{result['status']}: seal_allowed={result['seal_allowed']} ({result['reason']})")
    return 0 if result.get("seal_allowed") else 3


if __name__ == "__main__":
    raise SystemExit(main())
