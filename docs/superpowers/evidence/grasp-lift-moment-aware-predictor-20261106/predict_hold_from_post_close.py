#!/usr/bin/env python3
"""Offline moment/direction-aware hold predictor from post-close audits only.

Fail-closed: refuses when the scalar μ×normal proxy would accept a saturated
horizontal pinch with negligible vertical force share (the 21006 false pass).
Does not authorize lifts, change gates, or claim physical capacity.
"""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
EVIDENCE = Path(__file__).resolve().parent
TRACE = EVIDENCE.parent / "grasp-lift-wrench-trace-20261103"
FREEZE = TRACE / "FREEZE.json"
RESULT = TRACE / "RESULT.json"
MODEL = ROOT / "robots/bundles/arm_gripper_lift/model.xml"
OUT = EVIDENCE / "PREDICTOR_RESULT.json"
REPORT = EVIDENCE / "RESULT.md"

ACTUATOR_CAP_N = 1.95
VERTICAL_FRACTION_MIN = 0.15
ACTUATOR_UTIL_REFUSE = 0.98


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest().upper()


def finite(value, name: str) -> float:
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{name} non-finite")
    return number


def parse_gravity(model_path: Path) -> list[float]:
    import xml.etree.ElementTree as ET

    root = ET.fromstring(model_path.read_text(encoding="utf-8"))
    option = root.find("option")
    assert option is not None
    gravity = [float(x) for x in option.attrib["gravity"].split()]
    assert len(gravity) == 3 and gravity[2] < 0.0
    return gravity


def first_bilateral_post_close(audits: list[dict]) -> list[dict]:
    rows = [row for row in audits if row.get("phase") == "post_close"]
    if len(rows) < 2:
        raise ValueError("need at least two post_close audits")
    pair = rows[:2]
    if pair[0].get("finger_body") == pair[1].get("finger_body"):
        raise ValueError("first post_close pair is not bilateral")
    return pair


def predict_from_post_close(
    pair: list[dict],
    *,
    weight_n: float,
    mu_lower: float,
) -> dict:
    normal_sum = 0.0
    force_sum = [0.0, 0.0, 0.0]
    moment_sum = [0.0, 0.0, 0.0]
    force_mag_sum = 0.0
    actuator_peak = 0.0
    contact_count = 0
    for row in pair:
        normal_sum += finite(row["normal_force_n"], "normal_force_n")
        for axis_force in row.get("actuator_force") or []:
            actuator_peak = max(actuator_peak, abs(finite(axis_force, "actuator_force")))
        for contact in row.get("contact_wrenches") or []:
            force = contact.get("force_world_on_object_n")
            moment = contact.get("moment_world_about_object_com_nm")
            if force is None or moment is None:
                continue
            contact_count += 1
            mag = 0.0
            for axis in range(3):
                f = finite(force[axis], "force")
                m = finite(moment[axis], "moment")
                force_sum[axis] += f
                moment_sum[axis] += m
                mag += f * f
            force_mag_sum += math.sqrt(mag)

    scalar_proxy = mu_lower * normal_sum
    vertical = force_sum[2]
    vertical_fraction = abs(vertical) / max(force_mag_sum, 1.0e-12)
    actuator_util = actuator_peak / ACTUATOR_CAP_N
    moment_norm = math.sqrt(sum(component * component for component in moment_sum))

    refuse_reasons: list[str] = []
    if scalar_proxy < weight_n:
        refuse_reasons.append("scalar_proxy_below_weight")
    if vertical_fraction < VERTICAL_FRACTION_MIN and actuator_util >= ACTUATOR_UTIL_REFUSE:
        refuse_reasons.append("saturated_horizontal_pinch_negligible_vertical_share")
    if contact_count == 0:
        refuse_reasons.append("missing_post_close_contact_wrenches")

    decision = "refuse_lift" if refuse_reasons else "conditionally_allow_scalar_and_direction"
    return {
        "decision": decision,
        "refuse_reasons": refuse_reasons,
        "scalar_capacity_proxy_n": scalar_proxy,
        "scalar_proxy_to_weight_ratio": scalar_proxy / weight_n,
        "post_close_vertical_force_n": vertical,
        "post_close_force_vertical_fraction": vertical_fraction,
        "post_close_net_moment_norm_nm": moment_norm,
        "actuator_peak_n": actuator_peak,
        "actuator_utilization": actuator_util,
        "contact_wrench_count": contact_count,
        "thresholds": {
            "actuator_cap_n": ACTUATOR_CAP_N,
            "vertical_fraction_min": VERTICAL_FRACTION_MIN,
            "actuator_util_refuse": ACTUATOR_UTIL_REFUSE,
        },
    }


def main() -> None:
    freeze = json.loads(FREEZE.read_text(encoding="utf-8"))
    result = json.loads(RESULT.read_text(encoding="utf-8"))
    freeze_hash = sha256(FREEZE)
    result_hash = sha256(RESULT)
    if result.get("freeze_sha256") != freeze_hash:
        raise SystemExit("RESULT is not bound to FREEZE")
    gravity = parse_gravity(MODEL)

    trials_out = []
    for binding in freeze["primary_control_inputs"]:
        input_path = ROOT / binding["path"]
        if sha256(input_path) != binding["sha256"]:
            raise SystemExit(f"input hash mismatch {binding['trial_id']}")
        trial_bundle = json.loads(input_path.read_text(encoding="utf-8"))
        trial = trial_bundle["trial"]
        object_id = trial["scenario"]["object_id"]
        objects = [obj for obj in trial["scenario"]["objects"] if obj["name"] == object_id]
        if len(objects) != 1:
            raise SystemExit("object mass ambiguous")
        mass = finite(objects[0]["mass"], "mass")
        weight = mass * abs(gravity[2])

        matches = [
            record
            for record in result["records"]
            if record["episode"]["trial_id"] == binding["trial_id"]
        ]
        if len(matches) != 1:
            raise SystemExit(f"missing episode {binding['trial_id']}")
        episode = matches[0]["episode"]
        mu_lower = finite(episode["grasp_simulated_friction_oracle_interval"][0], "mu_lower")
        pair = first_bilateral_post_close(episode["grasp_contact_audits"])
        prediction = predict_from_post_close(pair, weight_n=weight, mu_lower=mu_lower)

        first_lift = [
            row
            for row in episode["grasp_contact_audits"]
            if row.get("phase") == "lift_waypoint_1"
        ][:2]
        first_lift_vertical = 0.0
        for row in first_lift:
            for contact in row.get("contact_wrenches") or []:
                first_lift_vertical += finite(
                    contact["force_world_on_object_n"][2], "first_lift_fz"
                )

        scalar_would_accept = prediction["scalar_capacity_proxy_n"] >= weight
        trials_out.append(
            {
                "trial_id": binding["trial_id"],
                "observed_outcome": episode.get("outcome"),
                "object_weight_n": weight,
                "prediction": prediction,
                "scalar_proxy_would_accept": scalar_would_accept,
                "first_lift_vertical_force_n": first_lift_vertical,
                "first_lift_supports_weight": first_lift_vertical >= weight,
            }
        )

    # Capability claim on this frozen cohort: catch scalar false accept on 21006.
    trial_21006 = next(t for t in trials_out if t["trial_id"].endswith(":21006:21"))
    trial_21010 = next(t for t in trials_out if t["trial_id"].endswith(":21010:25"))
    capability = {
        "scalar_false_accept_21006": trial_21006["scalar_proxy_would_accept"]
        and not trial_21006["first_lift_supports_weight"],
        "predictor_refuses_21006": trial_21006["prediction"]["decision"] == "refuse_lift",
        "predictor_refuses_21010": trial_21010["prediction"]["decision"] == "refuse_lift",
        "catches_scalar_false_accept": False,
    }
    capability["catches_scalar_false_accept"] = (
        capability["scalar_false_accept_21006"]
        and capability["predictor_refuses_21006"]
        and capability["predictor_refuses_21010"]
    )

    package = {
        "schema": "realityos.grasp_lift_moment_aware_post_close_predictor/v1",
        "classification": "OFFLINE_SIMULATION_PREDICTOR; TWO_FROZEN_DEVELOPMENT_CONTROLS; NOT_ADMITTED",
        "freeze_sha256": freeze_hash,
        "result_sha256": result_hash,
        "model_xml_sha256": sha256(MODEL),
        "source_trace": "docs/superpowers/evidence/grasp-lift-wrench-trace-20261103",
        "method": (
            "Post-close only: refuse if scalar μ×normal < weight, or if actuator "
            "utilization >= 0.98 while |Fz|/Σ|F| < 0.15 (saturated horizontal pinch)."
        ),
        "capability": capability,
        "trials": trials_out,
        "authority": {
            "admission_status": "NOT_ADMITTED",
            "metal": False,
            "shop_release": False,
            "viewport_admits": False,
            "authorizes_hardware": False,
        },
        "limits": [
            "Two frozen development controls only; not held-out validation.",
            "Simulation contact evidence only; not physical measurement.",
            "Does not change runtime grasp gates or authorize lift.",
            "Archived friction-mapping caveat from wrench-trace CHECKPOINT still applies.",
        ],
    }
    OUT.write_text(json.dumps(package, indent=2) + "\n", encoding="utf-8", newline="\n")
    REPORT.write_text(
        "\n".join(
            [
                "# Moment-aware post-close hold predictor — 2026-11-06",
                "",
                f"**Classification:** `{package['classification']}`",
                "",
                "## Capability delta",
                "",
                "Scalar μ×normal proxy **accepts** trial `21006:21` but first-lift vertical "
                "force does not support weight (known false accept).",
                "",
                "This post-close predictor **refuses** both `21006:21` and `21010:25`, "
                f"catching the false accept: `{capability['catches_scalar_false_accept']}`.",
                "",
                "## Reproduce",
                "",
                "```text",
                "py -3.10 -B docs/superpowers/evidence/grasp-lift-moment-aware-predictor-20261106/predict_hold_from_post_close.py",
                "py -3.10 -B docs/superpowers/evidence/grasp-lift-moment-aware-predictor-20261106/verify_predictor.py",
                "```",
                "",
                "## Authority",
                "",
                "`NOT_ADMITTED`; does not authorize hardware; metal/shop/viewport false.",
                "",
            ]
        ),
        encoding="utf-8",
        newline="\n",
    )
    print(
        "catches_false_accept",
        capability["catches_scalar_false_accept"],
        "21006",
        trial_21006["prediction"]["decision"],
        "21010",
        trial_21010["prediction"]["decision"],
    )
    if not capability["catches_scalar_false_accept"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
