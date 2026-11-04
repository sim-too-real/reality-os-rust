#!/usr/bin/env python3
"""Portfolio offline failure-class labels for sealed rotated-probe evidence dirs."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

VERIFY_DIR = Path(__file__).resolve().parent
ROOT = VERIFY_DIR.parents[1]
if str(VERIFY_DIR) not in sys.path:
    sys.path.insert(0, str(VERIFY_DIR))

from probe_transit_abort_failure_class import classify_episode  # noqa: E402

EVIDENCE = ROOT / "docs" / "superpowers" / "evidence"
SCHEMA = "realityos.probe_failure_class_portfolio/v1"
DEFAULT_SEEDS = [
    "rotated-probe-heldout-seed-21004-20261104",
    "rotated-probe-heldout-seed-21005-20261104",
    "rotated-probe-heldout-seed-21006-20261104",
    "rotated-probe-heldout-seed-21007-20261104",
]


def classify_dir(path: Path) -> dict:
    episode_path = path / "EPISODE.json"
    result_path = path / "RESULT.json"
    if not episode_path.is_file():
        return {
            "seed_dir": path.name,
            "status": "SKIPPED_NO_EPISODE",
            "failure_class": None,
            "governor_weakened": False,
        }
    if not result_path.is_file():
        return {
            "seed_dir": path.name,
            "status": "SKIPPED_NO_RESULT",
            "failure_class": None,
            "governor_weakened": False,
        }
    episode = json.loads(episode_path.read_text(encoding="utf-8"))
    result = json.loads(result_path.read_text(encoding="utf-8"))
    report = classify_episode(episode, result)
    return {
        "seed_dir": path.name,
        "status": "CLASSIFIED",
        "failure_class": report["failure_class"],
        "governor_weakened": report["governor_weakened"],
        "admission_status": report["admission_status"],
        "report": report,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path)
    args = parser.parse_args(argv)
    rows = [classify_dir(EVIDENCE / name) for name in DEFAULT_SEEDS]
    portfolio = {
        "schema": SCHEMA,
        "seed_count": len(rows),
        "classified_count": sum(1 for row in rows if row["status"] == "CLASSIFIED"),
        "missing_count": sum(1 for row in rows if row["status"] != "CLASSIFIED"),
        "governor_weakened_any": any(row.get("governor_weakened") for row in rows),
        "rows": rows,
        "admission_status": "NOT_ADMITTED",
        "metal": False,
        "shop_release": False,
        "viewport_admits": False,
    }
    text = json.dumps(portfolio, indent=2) + "\n"
    out = args.out or (
        EVIDENCE / "rotated-probe-failure-class-portfolio-20261104.json"
    )
    out.write_text(text, encoding="utf-8")
    print(text, end="")
    return (
        0
        if portfolio["missing_count"] == 0 and not portfolio["governor_weakened_any"]
        else 2
    )


if __name__ == "__main__":
    raise SystemExit(main())
