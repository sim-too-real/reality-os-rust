#!/usr/bin/env python3
"""Offline friction-bound vs prior diagnosis for sealed probe sidecars.

Does not change Governor contact/force thresholds. Does not invent Updated
intervals. Used to separate first-onset-outside-prior from successive-probe
prior incompatibility.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

VERIFY = Path(__file__).resolve().parent
if str(VERIFY) not in sys.path:
    sys.path.insert(0, str(VERIFY))

from probe_interval_reference import (  # noqa: E402
    ReplayError,
    _force_interval,
    _validate_measurement,
)

SCHEMA = "realityos.probe_friction_bound_diagnosis/v1"


def _measurement_bounds(measurement: dict[str, Any]) -> list[float] | None:
    motion = measurement.get("motion") or {}
    first_motion = motion.get("first_motion_s")
    samples = measurement.get("samples")
    if first_motion is None or not isinstance(samples, list) or not samples:
        return None
    onset = float(first_motion)
    before = next(
        (sample for sample in reversed(samples) if float(sample["capture_s"]) < onset),
        None,
    )
    after = next(
        (sample for sample in samples if float(sample["capture_s"]) >= onset),
        None,
    )
    if before is None or after is None:
        return None
    pre = _force_interval(before, measurement, onset)
    post = _force_interval(after, measurement, onset)
    if pre[2] <= 0.0 or post[2] <= 0.0 or pre[3] <= 0.0 or post[1] <= 0.0:
        return None
    lower = pre[0] / pre[3]
    upper = post[1] / post[2]
    if not (math.isfinite(lower) and math.isfinite(upper)):
        return None
    return [lower, upper]


def diagnose_record(record: dict[str, Any]) -> dict[str, Any]:
    measurement = record.get("measurement")
    if not isinstance(measurement, dict):
        return {
            "diagnosis": "invalid_measurement",
            "prior": None,
            "measurement_bounds": None,
            "intersection": None,
        }
    prior, _has_contact = _validate_measurement(measurement)
    bounds = _measurement_bounds(measurement)
    if bounds is None:
        return {
            "diagnosis": "bounds_unavailable",
            "prior": prior,
            "measurement_bounds": None,
            "intersection": None,
            "runtime_contact_ambiguous": bool(
                measurement.get("runtime_contact_ambiguous", False)
            ),
            "stored_inference": record.get("inference"),
        }
    if bounds[0] > bounds[1]:
        diagnosis = "empty_measurement_bounds"
        intersection = None
    else:
        intersection = [max(prior[0], bounds[0]), min(prior[1], bounds[1])]
        if intersection[0] <= intersection[1]:
            diagnosis = "overlaps_prior"
        elif bounds[0] > prior[1]:
            diagnosis = "entirely_above_prior"
        elif bounds[1] < prior[0]:
            diagnosis = "entirely_below_prior"
        else:
            diagnosis = "measurement_bounds_do_not_overlap_prior"
    return {
        "diagnosis": diagnosis,
        "prior": prior,
        "measurement_bounds": bounds,
        "intersection": intersection,
        "runtime_contact_ambiguous": bool(
            measurement.get("runtime_contact_ambiguous", False)
        ),
        "stored_inference": record.get("inference"),
    }


def diagnose_sidecar(sidecar: dict[str, Any]) -> dict[str, Any]:
    records = sidecar.get("measurements")
    if not isinstance(records, list) or not records:
        raise ReplayError("sidecar has no measurements")
    rows = [diagnose_record(record) for record in records]
    return {
        "schema": SCHEMA,
        "trial_id": sidecar.get("trial_id"),
        "measurement_count": len(rows),
        "measurements": rows,
        "admission_status": "NOT_ADMITTED",
        "metal": False,
        "shop_release": False,
        "viewport_admits": False,
        "limitations": [
            "Research-only bound geometry vs declared prior; does not admit metal.",
            "Does not change contact/force Governor thresholds.",
        ],
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sidecar", type=Path)
    parser.add_argument("--out", type=Path)
    args = parser.parse_args(argv)
    try:
        sidecar = json.loads(args.sidecar.read_text(encoding="utf-8"))
        report = diagnose_sidecar(sidecar)
    except (OSError, json.JSONDecodeError, ReplayError, TypeError, ValueError) as error:
        payload = {"status": "FAIL", "error": str(error)}
        print(json.dumps(payload, indent=2))
        return 2
    text = json.dumps(report, indent=2) + "\n"
    if args.out:
        args.out.write_text(text, encoding="utf-8")
    print(text, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
