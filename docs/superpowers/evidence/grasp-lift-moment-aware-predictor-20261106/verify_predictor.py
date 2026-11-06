#!/usr/bin/env python3
"""Independent recount of the moment-aware predictor package."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
RESULT = HERE / "PREDICTOR_RESULT.md"
PACKAGE = HERE / "PREDICTOR_RESULT.json"
OUT = HERE / "INDEPENDENT_VERIFICATION.json"
TRACE = HERE.parent / "grasp-lift-wrench-trace-20261103"


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest().upper()


def main() -> None:
    package = json.loads(PACKAGE.read_text(encoding="utf-8"))
    checks = []

    def check(name: str, ok: bool, detail=None) -> None:
        checks.append({"name": name, "ok": bool(ok), "detail": detail})

    check("freeze_bound", package["freeze_sha256"] == sha256(TRACE / "FREEZE.json"))
    check("result_bound", package["result_sha256"] == sha256(TRACE / "RESULT.json"))
    check("two_trials", len(package["trials"]) == 2, len(package["trials"]))

    for trial in package["trials"]:
        pred = trial["prediction"]
        weight = trial["object_weight_n"]
        check(
            f"{trial['trial_id'][-8:]}_scalar_ratio",
            math.isclose(
                pred["scalar_proxy_to_weight_ratio"],
                pred["scalar_capacity_proxy_n"] / weight,
                rel_tol=0,
                abs_tol=1e-12,
            ),
        )
        expected_accept = pred["scalar_capacity_proxy_n"] >= weight
        check(
            f"{trial['trial_id'][-8:]}_scalar_flag",
            trial["scalar_proxy_would_accept"] == expected_accept,
        )
        if pred["refuse_reasons"]:
            check(
                f"{trial['trial_id'][-8:]}_refuse",
                pred["decision"] == "refuse_lift",
            )

    cap = package["capability"]
    check("catches_false_accept", cap["catches_scalar_false_accept"] is True)
    check("refuses_21006", cap["predictor_refuses_21006"] is True)
    check("refuses_21010", cap["predictor_refuses_21010"] is True)
    check(
        "authority_closed",
        package["authority"]["admission_status"] == "NOT_ADMITTED"
        and package["authority"]["authorizes_hardware"] is False,
    )

    failed = [c for c in checks if not c["ok"]]
    verification = {
        "schema": "realityos.grasp_lift_moment_aware_predictor.independent/v1",
        "status": "PASS" if not failed else "FAIL",
        "package_sha256": sha256(PACKAGE),
        "checks": checks,
        "failed": failed,
        "admission_status": "NOT_ADMITTED",
    }
    OUT.write_text(json.dumps(verification, indent=2) + "\n", encoding="utf-8", newline="\n")
    print(verification["status"], "failed", len(failed))
    if failed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
