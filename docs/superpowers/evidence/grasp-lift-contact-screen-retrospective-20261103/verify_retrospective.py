#!/usr/bin/env python3
"""Recompute a bounded, retrospective grasp contact-capacity screen."""

import hashlib
import json
import math
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
HERE = Path(__file__).resolve().parent
FREEZE_PATH = HERE / "FREEZE.json"
RESULTS_PATH = HERE / "RESULTS.json"


def sha256_bytes(data):
    return hashlib.sha256(data).hexdigest().upper()


def sha256_file(path):
    return sha256_bytes(path.read_bytes())


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8"))


def source_at_commit(commit, path):
    return subprocess.run(
        ["git", "show", f"{commit}:{path}"],
        cwd=ROOT,
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    ).stdout


def record_for(document, trial_id):
    matches = [r for r in document["records"] if r["trial_id"] == trial_id]
    assert len(matches) == 1, f"expected one record for {trial_id}; got {len(matches)}"
    return matches[0]


freeze = read_json(FREEZE_PATH)
assert freeze["schema"] == "realityos.grasp_lift_prelift_contact_screen_freeze/v1"
assert sha256_file(HERE / "verify_retrospective.py") == freeze["verifier_sha256"]

for source in freeze["current_source_files"]:
    blob = source_at_commit(freeze["current_source_commit"], source["path"])
    assert sha256_bytes(blob) == source["sha256"], source["path"]

documents = {}
for item in freeze["evidence_inputs"]:
    path = ROOT / item["path"]
    assert sha256_file(path) == item["sha256"], item["path"]
    documents[item["id"]] = read_json(path)

primary = documents["candidate_primary_v2"]
controls = []
for spec in freeze["controls"]:
    trial_id = spec["trial_id"]
    candidate = record_for(primary, trial_id)
    candidate_episode = candidate["episode"]
    trial_digest = candidate_episode["trial_digest"]

    historical_document = documents[spec["historical_evidence_id"]]
    historical = record_for(historical_document, trial_id)
    historical_episode = historical["episode"]
    assert historical_episode["trial_digest"] == trial_digest
    assert historical_document.get("candidate_worktree_dirty") is True

    post_close = [
        a for a in historical_episode["grasp_contact_audits"]
        if a["phase"] == "post_close"
    ]
    assert len(post_close) == 2
    assert {a["finger_body"] for a in post_close} == {"finger_l", "finger_r"}
    assert all(a["force_available"] for a in post_close)
    assert all(
        a["normal_force_n"] is not None and math.isfinite(a["normal_force_n"])
        for a in post_close
    )
    assert all(a["contact_friction"] and len(a["contact_friction"][0]) == 5 for a in post_close)

    scenario = candidate["trial"]["scenario"]
    target = next(o for o in scenario["objects"] if o["name"] == scenario["object_id"])
    historical_scenario = historical["trial"]["scenario"]
    historical_target = next(
        o for o in historical_scenario["objects"]
        if o["name"] == historical_scenario["object_id"]
    )
    assert target["mass"] == historical_target["mass"]
    assert target["friction"] == historical_target["friction"]

    interval = historical_episode["grasp_simulated_friction_oracle_interval"]
    assert len(interval) == 2 and all(math.isfinite(v) for v in interval)
    assert interval[0] <= interval[1]
    normal_by_finger = {a["finger_body"]: a["normal_force_n"] for a in post_close}
    weight = target["mass"] * 9.81
    capacity_proxy = interval[0] * sum(normal_by_finger.values())
    prediction = "REJECT_BEFORE_LIFT" if capacity_proxy < weight else "PASS_TO_LIFT"

    lift_audits = candidate_episode["grasp_contact_audits"]
    assert len(lift_audits) >= 2
    last_pair = lift_audits[-2:]
    assert {a["finger_body"] for a in last_pair} == {"finger_l", "finger_r"}
    last_pair_vertical = sum(a["object_vertical_force_n"] for a in last_pair)
    lift_positions = candidate_episode["grasp_phase_object_positions"]
    assert len(lift_positions) >= 2
    delta_z = lift_positions[-1][2] - lift_positions[0][2]

    controls.append({
        "trial_id": trial_id,
        "trial_digest": trial_digest,
        "candidate_outcome": candidate_episode["outcome"],
        "candidate_failure_cause": candidate_episode["failure_cause"],
        "historical_source_commit": historical_document.get("candidate_source_sha"),
        "historical_worktree_dirty": historical_document.get("candidate_worktree_dirty"),
        "historical_source_hash": historical_episode.get("source_hash"),
        "historical_pre_lift_target_height_m": spec["historical_target_height_m"],
        "mass_kg": target["mass"],
        "weight_n_at_9_81_m_s2": weight,
        "simulated_friction_oracle_interval": interval,
        "post_close_normal_force_by_finger_n": normal_by_finger,
        "capacity_proxy_n_mu_lower_times_sum_normals": capacity_proxy,
        "capacity_proxy_to_weight_ratio": capacity_proxy / weight,
        "proxy_prediction": prediction,
        "post_close_contact_friction_tuples": [a["contact_friction"][0] for a in post_close],
        "post_close_vertical_force_by_finger_n": {
            a["finger_body"]: a["object_vertical_force_n"] for a in post_close
        },
        "candidate_last_recorded_lift_vertical_contact_force_n": last_pair_vertical,
        "candidate_object_vertical_delta_m": delta_z,
        "candidate_last_lift_audit_phase": last_pair[0]["phase"],
        "interpretation": "Retrospective proxy only; post-close snapshots came from dirty-source legacy runs.",
    })

assert len(controls) == 2
assert all(
    r["candidate_outcome"] == "unexpected_refusal"
    and r["candidate_failure_cause"] == "runtime_divergence"
    for r in controls
)
assert [r["proxy_prediction"] for r in controls] == [
    "REJECT_BEFORE_LIFT", "PASS_TO_LIFT"
]
matches = sum(
    (r["proxy_prediction"] == "REJECT_BEFORE_LIFT")
    == (r["candidate_failure_cause"] == "runtime_divergence")
    for r in controls
)
result = {
    "schema": "realityos.grasp_lift_prelift_contact_screen_result/v1",
    "experiment_id": freeze["experiment_id"],
    "freeze_sha256": sha256_file(FREEZE_PATH),
    "verifier_sha256": sha256_file(HERE / "verify_retrospective.py"),
    "verified_current_source_commit": freeze["current_source_commit"],
    "verified_candidate_source_commit": freeze["candidate_source_commit"],
    "evidence_hashes_verified": {x["id"]: x["sha256"] for x in freeze["evidence_inputs"]},
    "controls": controls,
    "summary": {
        "controls": len(controls),
        "candidate_runtime_divergences": 2,
        "proxy_rejections": sum(r["proxy_prediction"] == "REJECT_BEFORE_LIFT" for r in controls),
        "proxy_passes_that_later_failed": sum(
            r["proxy_prediction"] == "PASS_TO_LIFT"
            and r["candidate_failure_cause"] == "runtime_divergence"
            for r in controls
        ),
        "agreement_with_candidate_failure": matches,
        "agreement_fraction": matches / len(controls),
        "classification": "RETROSPECTIVE_PARTIAL_DISCRIMINATION; NOT_A_VALIDATED_PREDICTOR; SIMULATION_ONLY; NOT_ADMITTED",
    },
    "authority": {
        "admission_status": "NOT_ADMITTED",
        "metal": False,
        "shop_release": False,
        "viewport_admits": False,
    },
}
RESULTS_PATH.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
print(json.dumps(result["summary"], indent=2))
for row in controls:
    print(
        row["trial_id"],
        "proxy=", row["proxy_prediction"],
        "capacity/weight=", round(row["capacity_proxy_to_weight_ratio"], 3),
        "later_support_N=", round(row["candidate_last_recorded_lift_vertical_contact_force_n"], 3),
        "delta_z_m=", row["candidate_object_vertical_delta_m"],
    )
