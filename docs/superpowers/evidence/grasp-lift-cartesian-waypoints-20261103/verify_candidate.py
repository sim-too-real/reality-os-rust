#!/usr/bin/env python3
"""Independently verify the frozen Cartesian grasp-lift simulation record."""

from __future__ import annotations

import hashlib
import json
import math
import re
import subprocess
from datetime import datetime, timezone
from pathlib import Path


ROOT = Path(__file__).resolve().parents[4]
EVIDENCE = ROOT / "docs/superpowers/evidence/grasp-lift-cartesian-waypoints-20261103"
FREEZE_PATH = EVIDENCE / "FREEZE-v2.json"
RESULT_PATH = EVIDENCE / "CANDIDATE_PRIMARY_V2.json"
OUTPUT_PATH = EVIDENCE / "INDEPENDENT_VERIFICATION.json"
CANDIDATE_SOURCE_COMMIT = "0f14e9b073fcadf11ed69c6643cc7bbbfb36daf5"


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest().upper()


def read_json(path: Path) -> tuple[dict, bytes]:
    data = path.read_bytes()
    return json.loads(data), data


def request_mass_and_gravity(episode: dict) -> tuple[float, tuple[float, float, float]]:
    requests = episode.get("reasoning_request_audits", [])
    for request in reversed(requests):
        mass_match = re.search(r"mass_kg: Some\(([-+0-9.eE]+)\)", request)
        gravity_match = re.search(r"gravity_m_s2: \[([^\]]+)\]", request)
        if mass_match and gravity_match:
            gravity = tuple(float(value.strip()) for value in gravity_match.group(1).split(","))
            if len(gravity) == 3:
                return float(mass_match.group(1)), gravity
    raise AssertionError("frozen request omitted mass or gravity")


def verify() -> dict:
    freeze, freeze_bytes = read_json(FREEZE_PATH)
    result, _ = read_json(RESULT_PATH)
    assert freeze["status"] == "FROZEN_BEFORE_SIMULATION"
    assert result["stage"] == "candidate_primary"
    assert result["freeze_sha256"] == sha256(freeze_bytes)
    assert freeze["candidate_source"]["profile_id"] == "cartesian_vertical_max_1mm_v1"

    source_files = {
        "crates/verify/tests/manipulation_benchmark.rs": "integration_test_sha256",
        "crates/verify/src/manipulation_benchmark/execution.rs": "execution_sha256",
        "crates/semantics/src/physical_reasoner.rs": "reasoner_sha256",
    }
    for relative_path, freeze_key in source_files.items():
        source = subprocess.run(
            ["git", "show", f"{CANDIDATE_SOURCE_COMMIT}:{relative_path}"],
            cwd=ROOT,
            check=True,
            capture_output=True,
        ).stdout
        assert sha256(source) == freeze["candidate_source"][freeze_key]

    baseline_binding = freeze["baseline_result"]
    baseline_path = ROOT / baseline_binding["path"]
    baseline, baseline_bytes = read_json(baseline_path)
    assert sha256(baseline_bytes) == baseline_binding["sha256"]
    baseline_records = baseline["records"]
    baseline_successes = sum(
        record["episode"]["outcome"].lower() == "success" for record in baseline_records
    )
    assert len(baseline_records) == baseline_binding["expected_trials"] == 7
    assert baseline_successes == baseline_binding["expected_successes"] == 4

    expected = {entry["trial_id"]: entry for entry in freeze["primary_control_inputs"]}
    records = {record["trial_id"]: record for record in result["records"]}
    assert set(records) == set(expected)
    assert len(records) == 2

    metrics = []
    for trial_id, record in sorted(records.items()):
        binding = expected[trial_id]
        episode = record["episode"]
        input_path = ROOT / binding["path"]
        assert sha256(input_path.read_bytes()) == binding["sha256"]
        observed_digest = (
            record.get("trial_digest")
            or record.get("trial", {}).get("trial_digest")
            or episode.get("trial_digest")
        )
        assert observed_digest == binding["trial_digest"]
        assert episode["unauthorized_writes"] == 0
        assert episode["failure_cause"] == "runtime_divergence"
        guard = next(
            (
                item.removeprefix("grasp_lift_guard_failure=")
                for item in episode["decision_audits"]
                if item.startswith("grasp_lift_guard_failure=")
            ),
            None,
        )
        assert guard is not None and "object_tool_relative_pose_exceeded_20mm" in guard
        error_match = re.search(r"relative_pose_error_m=([0-9.]+);waypoint=(\d+)", guard)
        assert error_match

        positions = episode["grasp_phase_object_positions"]
        assert len(positions) > 1
        object_delta_z = positions[-1][2] - positions[0][2]
        assert abs(object_delta_z) < 0.001

        audits = episode["grasp_contact_audits"][-2:]
        assert len(audits) == 2
        assert {audit["finger_body"] for audit in audits} == {"finger_l", "finger_r"}
        assert all(audit["object_vertical_force_n"] is not None for audit in audits)
        net_vertical_n = sum(audit["object_vertical_force_n"] for audit in audits)
        mass_kg, gravity = request_mass_and_gravity(episode)
        weight_n = mass_kg * math.sqrt(sum(component * component for component in gravity))
        assert net_vertical_n < weight_n

        metrics.append(
            {
                "trial_id": trial_id,
                "outcome": episode["outcome"],
                "failure_cause": episode["failure_cause"],
                "guard": guard,
                "object_vertical_delta_m": object_delta_z,
                "last_sample_net_vertical_contact_force_n": net_vertical_n,
                "weight_n": weight_n,
                "vertical_contact_force_to_weight_ratio": net_vertical_n / weight_n,
                "last_sample_normal_forces_n": [audit["normal_force_n"] for audit in audits],
            }
        )

    assert freeze["fixed_guards"]["admission_status"] == "NOT_ADMITTED"
    assert freeze["fixed_guards"]["metal"] is False
    assert freeze["fixed_guards"]["shop_release"] is False
    assert freeze["fixed_guards"]["viewport_admits"] is False
    assert not (EVIDENCE / "CANDIDATE_COHORT.json").exists()

    verification = {
        "schema": "realityos.grasp_lift_cartesian_waypoints_verification/v1",
        "classification": "VALIDATED_LEARNING; CARTESIAN_PATH_HYPOTHESIS_REJECTED; SIMULATION_ONLY; NOT_ADMITTED",
        "verified_at_utc": datetime.now(timezone.utc).isoformat(),
        "freeze_sha256": sha256(freeze_bytes),
        "candidate_source_commit": CANDIDATE_SOURCE_COMMIT,
        "candidate_source_hashes_match": True,
        "primary_result_sha256": sha256(RESULT_PATH.read_bytes()),
        "baseline_result_sha256": sha256(baseline_bytes),
        "baseline_trials": len(baseline_records),
        "baseline_successes": baseline_successes,
        "candidate_primary_trials": len(records),
        "candidate_cohort_run": False,
        "authority_flags_closed": True,
        "trials": metrics,
        "interpretation": (
            "Both candidate controls hit the unchanged relative-pose guard. At the final recorded "
            "contact sample, simulated net vertical finger force remained below the request weight, "
            "while object vertical motion stayed below 1 mm. This supports a contact-load "
            "distribution mismatch with the scalar hold screen; it is simulation evidence only."
        ),
    }
    OUTPUT_PATH.write_text(json.dumps(verification, indent=2) + "\n", encoding="utf-8")
    return verification


if __name__ == "__main__":
    print(json.dumps(verify(), indent=2))
