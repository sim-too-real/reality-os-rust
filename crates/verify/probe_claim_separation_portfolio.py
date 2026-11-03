#!/usr/bin/env python3
"""Fail-closed claim-separation portfolio over sealed rotated-probe offline analyses.

Scans docs/superpowers/evidence/rotated-probe-heldout-seed-*/OFFLINE_ANALYSIS.json
and requires the claim_separation gate. Does not run the simulator or invent seeds.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

VERIFY_DIR = Path(__file__).resolve().parent
ROOT = VERIFY_DIR.parents[1]
if str(VERIFY_DIR) not in sys.path:
    sys.path.insert(0, str(VERIFY_DIR))

from probe_contact_force_analysis import claim_separation_gate_violations  # noqa: E402

EVIDENCE = ROOT / "docs" / "superpowers" / "evidence"
DEFAULT_OUT = EVIDENCE / "rotated-probe-claim-separation-portfolio-20261104.json"
SCHEMA = "realityos.probe_claim_separation_portfolio/v1"


def discover_analyses() -> list[Path]:
    return sorted(
        path for path in EVIDENCE.glob("rotated-probe-heldout-seed-*/OFFLINE_ANALYSIS.json") if path.is_file()
    )


def evaluate() -> dict:
    rows = []
    failures = []
    for path in discover_analyses():
        rel = path.relative_to(ROOT).as_posix()
        try:
            analysis = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as error:
            row = {
                "analysis_path": rel,
                "status": "ERROR",
                "violations": [str(error)],
            }
            rows.append(row)
            failures.append(rel)
            continue
        violations = claim_separation_gate_violations(analysis)
        status = "PASS" if not violations else "FAIL"
        rows.append(
            {
                "analysis_path": rel,
                "status": status,
                "violations": violations,
                "measurement_count": len(analysis.get("measurements") or []),
            }
        )
        if status != "PASS":
            failures.append(rel)
    return {
        "schema": SCHEMA,
        "updated": "2026-11-04",
        "gate": "claim_separation_portfolio",
        "status": "PASS" if not failures else "FAIL",
        "analysis_count": len(rows),
        "failure_count": len(failures),
        "failures": failures,
        "analyses": rows,
        "note": "Offline portfolio gate only; not a new held-out simulator run; not physical evidence.",
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    args = parser.parse_args()
    report = evaluate()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(
        json.dumps(
            {
                "status": report["status"],
                "analysis_count": report["analysis_count"],
                "failure_count": report["failure_count"],
                "out": str(args.out),
            },
            indent=2,
        )
    )
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
