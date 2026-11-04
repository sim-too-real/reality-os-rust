#!/usr/bin/env python3
"""Build FREEZE for 21003 cadence-escalation paired replay (new evidence dir)."""
from __future__ import annotations

import hashlib
import json
import subprocess
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
EV = (
    ROOT
    / "docs/superpowers/evidence/rotated-probe-heldout-seed-21003-cadence-escalation-20261104"
)
TPL = (
    ROOT
    / "docs/superpowers/evidence/rotated-probe-heldout-seed-21003-20261104/FREEZE.json"
)


def sha256_upper(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest().upper()


def main() -> int:
    trial = json.loads((EV / "TRIAL_INPUT.json").read_text(encoding="utf-8"))
    preflight = json.loads((EV / "PREFLIGHT.json").read_text(encoding="utf-8"))
    template = json.loads(TPL.read_text(encoding="utf-8"))
    source_hashes = {
        rel: sha256_upper(ROOT / rel) for rel in template["source_hashes_sha256"]
    }
    model_hashes = {
        rel: sha256_upper(ROOT / rel) for rel in template["model_files_sha256"]
    }
    head = subprocess.check_output(
        ["git", "-C", str(ROOT), "rev-parse", "HEAD"], text=True
    ).strip()
    status = subprocess.check_output(
        [
            "git",
            "-C",
            str(ROOT),
            "status",
            "--porcelain=v1",
            "--untracked-files=all",
        ],
        text=True,
    )
    paths = sorted(line[3:] for line in status.splitlines() if len(line) >= 4)
    now = (
        datetime.now(timezone.utc)
        .replace(microsecond=0)
        .isoformat()
        .replace("+00:00", "Z")
    )
    cmd = (
        "$env:REALITYOS_ROTATED_PROBE_DIAGNOSTIC_DIR="
        "'C:\\\\projects\\\\reality-os-rust\\\\docs\\\\superpowers\\\\evidence\\\\"
        "rotated-probe-heldout-seed-21003-cadence-escalation-20261104'; "
        "$env:REALITYOS_ROTATED_PROBE_TRIAL_SEED='21003'; "
        "$env:REALITYOS_ROTATED_PROBE_TRIAL_INDEX='0'; "
        "Remove-Item Env:REALITYOS_ROTATED_PROBE_PREPARE_ONLY -ErrorAction SilentlyContinue; "
        "cargo test -p realityos-verify --test manipulation_benchmark "
        "iiwa_rotated_probe_unbracketed_motion_diagnostic_capture -- --ignored --exact --nocapture"
    )
    freeze = {
        "schema": "realityos.rotated_probe_diagnostic.freeze/v1",
        "experiment_id": "rotated_probe_heldout_seed_21003_cadence_escalation_iiwa14_20261104_v1",
        "frozen_at_utc": now,
        "status": "FROZEN_WORKING_TREE_CADENCE_ESCALATION_REPLAY",
        "classification": "CADENCE_ESCALATION_PAIRED_REPLAY; SIMULATION_ONLY; NOT_ADMITTED",
        "repository": "reality-os-rust",
        "branch": "main",
        "source_commit": head,
        "working_tree_changes_at_freeze": bool(paths),
        "working_tree_change_paths": paths,
        "working_tree_change_purpose": (
            "PreferHigherLoadFamilyOnIncompatible wired; fresh 21003 paired "
            "replay without overwriting sealed 21003 evidence."
        ),
        "prior_seeds": (
            "sealed 21003 successive prior incompatibility + offline cadence "
            "policy comparison"
        ),
        "trial_id": trial["trial_id"],
        "trial_digest": preflight["trial_digest"],
        "trial_seed": trial["seed"],
        "trial_index": trial["index"],
        "trial_expectation": trial["expectation"],
        "trial_input_sha256": sha256_upper(EV / "TRIAL_INPUT.json"),
        "preflight_sha256": sha256_upper(EV / "PREFLIGHT.json"),
        "model_hash": trial["model_hash"],
        "trial_source_hash": trial["source_hash"],
        "first_action_preflight": {
            "authorized_action_key": preflight.get("authorized_action_key"),
            "authorized_grant_id": preflight.get("authorized_grant_id"),
            "control_writes_before": preflight.get("control_writes_before"),
            "control_writes_after": preflight.get("control_writes_after"),
            "simulation_time_before_s": preflight.get("simulation_time_before_s"),
            "simulation_time_after_s": preflight.get("simulation_time_after_s"),
            "simulator_steps": preflight.get("simulator_steps"),
        },
        "preserved_motion_inference_guard": template[
            "preserved_motion_inference_guard"
        ],
        "run_budget": {
            "simulation_runs_max": 1,
            "simulation_runs_at_freeze": 0,
            "prepare_only_preflight_runs": 1,
            "full_lift_replay": False,
        },
        "source_hashes_sha256": source_hashes,
        "model_files_sha256": model_hashes,
        "command": cmd,
        "expected_result": (
            "Under PreferHigherLoadFamilyOnIncompatible: episode reaches success "
            "with unauthorized_writes=0, or records escalated_higher_load_family; "
            "1 mm / 0.5 N unchanged."
        ),
        "scope_limits": template["scope_limits"],
        "expected_episode_outcome": "success",
        "expected_wrench_inference_reason": "any",
        "held_out_from_trial_seed": 21000,
        "diagnostic_intervention": (
            "Cadence escalation on same seed geometry as sealed 21003; new "
            "evidence dir; no threshold change."
        ),
        "replay_authorized": True,
        "replay_block_reason": None,
        "metal": False,
        "shop_release": False,
        "viewport_admits": False,
        "admission_status": "NOT_ADMITTED",
    }
    (EV / "FREEZE.json").write_text(
        json.dumps(freeze, indent=2) + "\n", encoding="utf-8"
    )
    print(f"wrote {EV / 'FREEZE.json'}")
    print(f"dirty_paths={len(paths)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
