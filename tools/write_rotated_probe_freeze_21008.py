#!/usr/bin/env python3
"""Build and validate the prepare-only freeze for held-out seed 21008.

Does not run the simulator and does not git-commit. Replay remains blocked
until these freeze bytes are committed and FREEZE_VERIFY still PASSes.
"""
from __future__ import annotations

import hashlib
import json
import math
import re
import subprocess
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = ROOT / "docs/superpowers/evidence/rotated-probe-heldout-seed-21008-20261104"
TEMPLATE = ROOT / "docs/superpowers/evidence/rotated-probe-heldout-seed-21007-20261104/FREEZE.json"
EXPECTED_MOTION_GUARD = {
    "minimum_planar_motion_m": 0.001,
    "minimum_tangential_force_lower_bound_n": 0.5,
    "source": "crates/verify/src/manipulation_benchmark/execution.rs",
    "thresholds_changed": False,
}


def sha256_upper(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest().upper()


def git_head() -> str:
    out = subprocess.check_output(["git", "-C", str(ROOT), "rev-parse", "HEAD"], text=True)
    return out.strip()


def git_status_paths() -> list[str]:
    out = subprocess.check_output(
        ["git", "-C", str(ROOT), "status", "--porcelain=v1", "--untracked-files=all"],
        text=True,
    )
    return sorted(line[3:] for line in out.splitlines() if len(line) >= 4)


def _finite_number(value: object) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _nonnegative_int(value: object) -> bool:
    return isinstance(value, int) and not isinstance(value, bool) and value >= 0


def _read_motion_guard(path: Path) -> dict | None:
    source = path.read_text(encoding="utf-8")
    patterns = {
        "minimum_planar_motion_m": r"const\s+PROBE_SLIP_DISPLACEMENT_M:\s*f64\s*=\s*([0-9.eE+-]+)\s*;",
        "minimum_tangential_force_lower_bound_n": r"const\s+MIN_PROBE_CONTACT_FORCE_N:\s*f64\s*=\s*([0-9.eE+-]+)\s*;",
    }
    values = {}
    for name, pattern in patterns.items():
        match = re.search(pattern, source)
        if match is None:
            return None
        values[name] = float(match.group(1))
    return {
        **values,
        "source": EXPECTED_MOTION_GUARD["source"],
        "thresholds_changed": False,
    }


def main() -> int:
    trial_path = EVIDENCE / "TRIAL_INPUT.json"
    preflight_path = EVIDENCE / "PREFLIGHT.json"
    freeze_path = EVIDENCE / "FREEZE.json"
    verify_path = EVIDENCE / "FREEZE_VERIFY.json"
    if not trial_path.is_file() or not preflight_path.is_file():
        print("prepare-only TRIAL_INPUT/PREFLIGHT missing")
        return 2

    try:
        trial = json.loads(trial_path.read_text(encoding="utf-8"))
        preflight = json.loads(preflight_path.read_text(encoding="utf-8"))
        template = json.loads(TEMPLATE.read_text(encoding="utf-8"))

        source_hashes = {
            rel: sha256_upper(ROOT / rel) for rel in template["source_hashes_sha256"]
        }
        current_guard = _read_motion_guard(ROOT / EXPECTED_MOTION_GUARD["source"])
        model_hashes = {
            rel: sha256_upper(ROOT / rel) for rel in template["model_files_sha256"]
        }
        trial_input_hash = sha256_upper(trial_path)
        preflight_hash = sha256_upper(preflight_path)
        working_tree_paths = git_status_paths()
        head = git_head()
    except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError, subprocess.SubprocessError) as exc:
        print(f"cannot build 21008 freeze from required evidence: {exc}")
        return 2

    now = datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")
    trial_id = trial.get("trial_id")
    seed = trial.get("seed")
    trial_index = trial.get("index")
    expected_id_suffix = f":{seed}:{trial_index}"
    guard = template.get("preserved_motion_inference_guard")
    model_mismatches = [
        rel
        for rel, digest in model_hashes.items()
        if digest != template["model_files_sha256"].get(rel)
    ]
    source_mismatches = [
        rel
        for rel, digest in source_hashes.items()
        if digest != template["source_hashes_sha256"].get(rel)
    ]
    before_writes = preflight.get("control_writes_before")
    after_writes = preflight.get("control_writes_after")
    before_time = preflight.get("simulation_time_before_s")
    after_time = preflight.get("simulation_time_after_s")

    checks = {
        "trial_preflight_id_matches": bool(trial_id) and trial_id == preflight.get("trial_id"),
        "trial_seed_and_index_match_target": (
            seed == 21008
            and trial_index == 0
            and isinstance(trial_id, str)
            and trial_id.endswith(expected_id_suffix)
        ),
        "trial_digest_present": (
            isinstance(preflight.get("trial_digest"), str)
            and len(preflight["trial_digest"]) == 64
            and all(ch in "0123456789abcdef" for ch in preflight["trial_digest"])
        ),
        "preflight_authorized_action_matches": preflight.get("authorized_action_key") == "probe:+x",
        "preflight_zero_step": (
            _nonnegative_int(preflight.get("simulator_steps"))
            and preflight["simulator_steps"] == 0
        ),
        "control_writes_unchanged": (
            _nonnegative_int(before_writes)
            and _nonnegative_int(after_writes)
            and before_writes == after_writes
        ),
        "control_writes_zero": (
            _nonnegative_int(before_writes)
            and _nonnegative_int(after_writes)
            and before_writes == 0
            and after_writes == 0
        ),
        "simulation_time_unchanged": (
            _finite_number(before_time)
            and _finite_number(after_time)
            and before_time == after_time
        ),
        "thresholds_unchanged": (
            guard == EXPECTED_MOTION_GUARD and current_guard == EXPECTED_MOTION_GUARD
        ),
        "source_hashes_collected": (
            bool(source_hashes) and EXPECTED_MOTION_GUARD["source"] in source_hashes
        ),
        "model_hashes_collected": bool(model_hashes),
        "model_files_match_21007_freeze": not model_mismatches,
        "working_tree_provenance_collected": isinstance(working_tree_paths, list),
    }
    required_checks = list(checks)
    passed = all(checks[name] is True for name in required_checks)

    first_action_preflight = {
        "authorized_action_key": preflight.get("authorized_action_key"),
        "authorized_grant_id": preflight.get("authorized_grant_id"),
        "control_writes_before": before_writes,
        "control_writes_after": after_writes,
        "simulation_time_before_s": before_time,
        "simulation_time_after_s": after_time,
        "simulator_steps": preflight.get("simulator_steps"),
    }
    freeze = {
        "schema": "realityos.rotated_probe_diagnostic.freeze/v1",
        "experiment_id": "rotated_probe_heldout_seed_21008_iiwa14_20261104_v1",
        "frozen_at_utc": now,
        "status": (
            "FROZEN_WORKING_TREE_AWAITING_COMMIT_BEFORE_REPLAY"
            if passed
            else "INVALID_WORKING_TREE_PREFLIGHT"
        ),
        "classification": (
            "HELD_OUT_SEED_DIAGNOSTIC; TRANSIT_TRACE_CAPTURE; ABORT_PATH_SEAL; "
            "WORKING_TREE_FREEZE; SIMULATION_ONLY; NOT_ADMITTED"
        ),
        "repository": "reality-os-rust",
        "branch": "main",
        "source_commit": head,
        "working_tree_changes_at_freeze": bool(working_tree_paths),
        "working_tree_change_paths": working_tree_paths,
        "working_tree_change_purpose": (
            "Offline failure-class tooling and prepare/freeze evidence for 21008. "
            "Capture path hashes below bind the diagnostic code; replay stays blocked "
            "until this freeze is committed."
        ),
        "prior_seeds": "21004-21007 one-run budgets consumed; 21007 source-identity closed",
        "trial_id": trial_id,
        "trial_digest": preflight.get("trial_digest"),
        "trial_seed": seed,
        "trial_index": trial_index,
        "trial_expectation": trial.get("expectation"),
        "trial_input_sha256": trial_input_hash,
        "preflight_sha256": preflight_hash,
        "model_hash": trial.get("model_hash"),
        "trial_source_hash": trial.get("source_hash"),
        "first_action_preflight": first_action_preflight,
        "preserved_motion_inference_guard": guard,
        "run_budget": {
            "simulation_runs_max": 1,
            "simulation_runs_at_freeze": 0,
            "prepare_only_preflight_runs": 1,
            "full_lift_replay": False,
        },
        "source_hashes_sha256": source_hashes,
        "model_files_sha256": model_hashes,
        "command": (
            "$env:REALITYOS_ROTATED_PROBE_DIAGNOSTIC_DIR="
            "'C:\\\\projects\\\\reality-os-rust\\\\docs\\\\superpowers\\\\evidence\\\\"
            "rotated-probe-heldout-seed-21008-20261104'; "
            "$env:REALITYOS_ROTATED_PROBE_TRIAL_SEED='21008'; "
            "$env:REALITYOS_ROTATED_PROBE_TRIAL_INDEX='0'; "
            "Remove-Item Env:REALITYOS_ROTATED_PROBE_PREPARE_ONLY -ErrorAction SilentlyContinue; "
            "cargo test -p realityos-verify --test manipulation_benchmark "
            "iiwa_rotated_probe_unbracketed_motion_diagnostic_capture -- --ignored --exact --nocapture"
        ),
        "expected_result": (
            "If unexpected_contact aborts during transit, sealed probe pose/force trace should "
            "still be available when probe_trace is requested; Governor refusal correctness "
            "unchanged. 1 mm / 0.5 N unchanged."
        ),
        "scope_limits": template["scope_limits"],
        "expected_episode_outcome": "preserve_sealed_sidecar_or_typed_abort",
        "expected_wrench_inference_reason": "any",
        "held_out_from_trial_seed": 21000,
        "diagnostic_intervention": (
            "New seed after 21007: verify transit abort can seal diagnostic evidence without "
            "weakening unexpected-contact gate. Commit freeze before replay."
        ),
        "replay_authorized": False,
        "replay_block_reason": "freeze_not_committed" if passed else "freeze_checks_failed",
    }
    verify = {
        "schema": "realityos.rotated_probe_diagnostic.freeze_verify/v1",
        "experiment_id": freeze["experiment_id"],
        "verified_at_utc": now,
        "status": "PASS_WORKING_TREE" if passed else "FAIL_WORKING_TREE",
        "checks": {
            **checks,
            "required_check_failures": [name for name in required_checks if checks[name] is not True],
            "trial_seed": seed,
            "preflight_authorized_action": preflight.get("authorized_action_key"),
            "minimum_planar_motion_m": (guard or {}).get("minimum_planar_motion_m"),
            "minimum_tangential_force_lower_bound_n": (guard or {}).get(
                "minimum_tangential_force_lower_bound_n"
            ),
            "current_motion_inference_guard": current_guard,
            "source_commit": head,
            "working_tree_changes_at_freeze": bool(working_tree_paths),
            "working_tree_change_paths": working_tree_paths,
            "trial_input_sha256": trial_input_hash,
            "preflight_sha256": preflight_hash,
            "model_file_hash_mismatches": model_mismatches,
            "source_file_hash_mismatches_vs_21007_freeze": source_mismatches,
            "execution_rs_sha256": source_hashes.get(
                "crates/verify/src/manipulation_benchmark/execution.rs"
            ),
            "mujoco_worker_sha256": source_hashes.get("crates/verify/mujoco_worker.py"),
            "replay_authorized": False,
        },
        "admission_status": "NOT_ADMITTED",
        "metal": False,
        "shop_release": False,
        "viewport_admits": False,
    }

    EVIDENCE.mkdir(parents=True, exist_ok=True)
    freeze_path.write_text(json.dumps(freeze, indent=2) + "\n", encoding="utf-8")
    verify_path.write_text(json.dumps(verify, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"freeze": str(freeze_path), "verify_status": verify["status"]}, indent=2))
    return 0 if passed else 2


if __name__ == "__main__":
    raise SystemExit(main())
