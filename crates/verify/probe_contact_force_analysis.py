#!/usr/bin/env python3
"""Offline comparison of probe slip onset, support contacts, and tool wrenches.

This script reads a sealed sidecar only. It does not import or execute MuJoCo,
simulation, inference, or control code. Run the Reality OS sidecar verifier
before analysis when a digest-verified result is required.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any


FORMAT = "realityos.probe_contact_force_analysis"
SCHEMA_VERSION = 1
SUPPORT_BODY = "table"
TANGENTIAL_FORCE_LOWER_BOUND_N = 0.5


def _xy_displacement(sample: dict[str, Any], origin: list[float]) -> float:
    position = sample.get("position_world_m")
    if not isinstance(position, list) or len(position) < 2:
        raise ValueError("pose_position_invalid")
    return math.hypot(float(position[0]) - float(origin[0]), float(position[1]) - float(origin[1]))


def _motion_onset(measurement: dict[str, Any]) -> tuple[dict[str, Any] | None, dict[str, Any] | None, int | None]:
    trace = measurement.get("motion_trace")
    if not isinstance(trace, dict):
        return None, None, None
    samples = trace.get("samples")
    origin = trace.get("start_position_world_m")
    threshold = measurement.get("motion", {}).get("pose_resolution_m")
    if not isinstance(samples, list) or not samples or not isinstance(origin, list) or len(origin) < 2:
        return None, None, None
    if not isinstance(threshold, (int, float)) or not math.isfinite(float(threshold)) or threshold <= 0:
        raise ValueError("motion_threshold_invalid")
    previous = None
    for index, sample in enumerate(samples):
        if _xy_displacement(sample, origin) > float(threshold):
            return previous, sample, index
        previous = sample
    return previous, None, None


def _contact_force_trace_status(trace: dict[str, Any] | None) -> str:
    if not isinstance(trace, dict):
        return "absent_legacy"
    samples = trace.get("samples")
    if not isinstance(samples, list):
        return "invalid"
    force_lists = [sample.get("contact_force_samples") for sample in samples]
    if not any(value is not None for value in force_lists):
        return "absent_legacy"
    if any(not isinstance(value, list) for value in force_lists):
        return "invalid"
    for sample, force_samples in zip(samples, force_lists):
        contact_pairs = sample.get("contact_pairs")
        if not isinstance(contact_pairs, list) or len(contact_pairs) != len(force_samples):
            return "invalid"
        for pair, force_sample in zip(contact_pairs, force_samples):
            if not isinstance(force_sample, dict) or force_sample.get("contact_pair") != pair:
                return "invalid"
    return "verified"


def _support_contacts(sample: dict[str, Any] | None, object_id: str) -> list[dict[str, Any]] | None:
    if sample is None:
        return None
    force_samples = sample.get("contact_force_samples")
    if not isinstance(force_samples, list):
        return None
    contacts = []
    for force_sample in force_samples:
        pair = force_sample.get("contact_pair", {})
        body_names = {pair.get("body1"), pair.get("body2")}
        if object_id not in body_names or SUPPORT_BODY not in body_names:
            continue
        contacts.append(
            {
                "contact_pair": pair,
                "position_world_m": force_sample.get("position_world_m"),
                "frame": force_sample.get("frame"),
                "friction": force_sample.get("friction"),
                "force_local": force_sample.get("force_local"),
                "torque_local": force_sample.get("torque_local"),
                "normal_force_n": force_sample.get("normal_force_n"),
                "tangential_force_n": force_sample.get("tangential_force_n"),
                "force_available": force_sample.get("force_available"),
            }
        )
    return contacts


def _wrench_summary(sample: dict[str, Any] | None) -> dict[str, Any] | None:
    if sample is None:
        return None
    return {
        "sequence": sample.get("sequence"),
        "capture_s": sample.get("capture_s"),
        "wrench": sample.get("wrench"),
        "error_bounds": sample.get("error_bounds"),
    }


def _compare_friction_interval(
    record: dict[str, Any], contacts: list[dict[str, Any]] | None
) -> dict[str, Any]:
    inferred = record.get("posterior_support_friction_interval")
    if not isinstance(inferred, list) or len(inferred) != 2 or not contacts:
        return {
            "status": "unavailable",
            "inferred_interval": inferred,
            "simulator_sliding_coefficients": [],
        }
    lower, upper = float(inferred[0]), float(inferred[1])
    coefficients = []
    for contact in contacts:
        friction = contact.get("friction")
        if isinstance(friction, list) and len(friction) >= 2:
            pair = [float(friction[0]), float(friction[1])]
            coefficients.append(
                {
                    "contact_pair": contact.get("contact_pair"),
                    "coefficients": pair,
                    "inside_inferred_interval": [lower <= value <= upper for value in pair],
                }
            )
    if not coefficients:
        status = "unavailable"
    elif any(any(entry["inside_inferred_interval"]) for entry in coefficients):
        status = "overlap"
    else:
        status = "disjoint"
    return {
        "status": status,
        "inferred_interval": inferred,
        "simulator_sliding_coefficients": coefficients,
    }


def _analyze_measurement(record: dict[str, Any]) -> dict[str, Any]:
    measurement = record["measurement"]
    trace = measurement.get("motion_trace")
    status = _contact_force_trace_status(trace)
    previous, onset, onset_index = _motion_onset(measurement)
    result: dict[str, Any] = {
        "action_key": record.get("action_key"),
        "measurement_digest": record.get("measurement_digest"),
        "measurement_inference": record.get("inference"),
        "contact_force_trace_status": status,
        "support_body": SUPPORT_BODY,
    }
    if not isinstance(trace, dict):
        result["slip_onset"] = {"status": "absent_legacy"}
        return result
    if onset is None or onset_index is None:
        result["slip_onset"] = {"status": "not_observed_in_trace"}
        return result

    pose_samples = trace["samples"]
    previous_time = float(trace["start_s"] if previous is None else previous["capture_s"])
    onset_time = float(onset["capture_s"])
    selected_samples = []
    for index in range(max(0, onset_index - 1), min(len(pose_samples), onset_index + 2)):
        pose_sample = pose_samples[index]
        selected_samples.append(
            {
                "relative_to_onset": "before" if index < onset_index else "at" if index == onset_index else "after",
                "simulator_step": pose_sample.get("simulator_step"),
                "capture_s": pose_sample.get("capture_s"),
                "planar_displacement_m": _xy_displacement(
                    pose_sample, trace["start_position_world_m"]
                ),
                "wrench_sequence": pose_sample.get("wrench_sequence"),
                "obj_table_contacts": _support_contacts(pose_sample, trace["object_id"]),
            }
        )
    onset_contacts = _support_contacts(onset, trace["object_id"])

    wrench_samples = measurement.get("samples", [])
    wrench_by_sequence = {
        sample.get("sequence"): sample
        for sample in wrench_samples
        if isinstance(sample, dict) and sample.get("sequence") is not None
    }
    exact_wrench = wrench_by_sequence.get(onset.get("wrench_sequence"))
    at_or_before = [sample for sample in wrench_samples if float(sample["capture_s"]) <= previous_time]
    at_or_after = [sample for sample in wrench_samples if float(sample["capture_s"]) >= onset_time]
    lower_wrench = max(at_or_before, key=lambda sample: float(sample["capture_s"])) if at_or_before else None
    upper_wrench = min(at_or_after, key=lambda sample: float(sample["capture_s"])) if at_or_after else None

    result["slip_onset"] = {
        "status": "bracketed",
        "threshold_m": measurement["motion"]["pose_resolution_m"],
        "time_bracket_s": [previous_time, onset_time],
        "onset_pose_sample": {
            "simulator_step": onset.get("simulator_step"),
            "capture_s": onset_time,
            "planar_displacement_m": _xy_displacement(
                onset, trace["start_position_world_m"]
            ),
            "wrench_sequence": onset.get("wrench_sequence"),
        },
        "tool_wrench_at_onset": _wrench_summary(exact_wrench),
        "tool_wrench_interval": {
            "left": _wrench_summary(lower_wrench),
            "right": _wrench_summary(upper_wrench),
            "target_time_bracket_s": [previous_time, onset_time],
        },
        "inference_vs_simulator_friction": _compare_friction_interval(
            record, onset_contacts
        ),
        "obj_table_contact_snapshots": selected_samples,
    }
    return result


def analyze(sidecar_path: Path) -> dict[str, Any]:
    sidecar = json.loads(sidecar_path.read_text(encoding="utf-8"))
    if not isinstance(sidecar, dict) or sidecar.get("format") != "realityos.probe_measurement_sidecar":
        raise ValueError("unsupported_sidecar_format")
    records = sidecar.get("measurements")
    if not isinstance(records, list):
        raise ValueError("sidecar_measurements_invalid")
    return {
        "format": FORMAT,
        "schema_version": SCHEMA_VERSION,
        "status": "OFFLINE_RECOMPUTATION_ONLY",
        "trial_id": sidecar.get("trial_id"),
        "trial_digest": sidecar.get("trial_digest"),
        "thresholds": {
            "planar_motion_m": [record["measurement"]["motion"]["pose_resolution_m"] for record in records],
            "tool_tangential_force_lower_bound_n": TANGENTIAL_FORCE_LOWER_BOUND_N,
            "tool_contact_force_used_for_inference": False,
        },
        "scope": "Reads the supplied sidecar only; does not import simulation, inference, or control code.",
        "measurements": [_analyze_measurement(record) for record in records],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sidecar", type=Path, help="probe measurement sidecar JSON")
    parser.add_argument("--output", type=Path, help="write the analysis JSON to this path")
    args = parser.parse_args()
    try:
        result = analyze(args.sidecar)
    except (OSError, json.JSONDecodeError, KeyError, TypeError, ValueError) as error:
        print(json.dumps({"status": "ERROR", "error": str(error)}), file=sys.stderr)
        return 2
    payload = json.dumps(result, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.write_text(payload, encoding="utf-8")
    else:
        sys.stdout.write(payload)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
