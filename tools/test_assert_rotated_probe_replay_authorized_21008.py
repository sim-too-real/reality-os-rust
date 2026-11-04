#!/usr/bin/env python3
from __future__ import annotations

import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TOOL = ROOT / "tools/assert_rotated_probe_replay_authorized_21008.py"
LIVE = (
    ROOT
    / "docs/superpowers/evidence/rotated-probe-heldout-seed-21008-20261104/FREEZE.json"
)


class AssertReplayAuthorized21008Tests(unittest.TestCase):
    def test_live_freeze_is_clean_tree_authorized_without_threshold_change(self) -> None:
        result = subprocess.run(
            [sys.executable, str(TOOL), "--freeze", str(LIVE), "--json"],
            cwd=ROOT,
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        report = json.loads(result.stdout)
        self.assertTrue(report["ok"])
        self.assertTrue(report["replay_authorized"])
        self.assertTrue(report["thresholds_unchanged"])
        self.assertEqual(report["admission_status"], "NOT_ADMITTED")

    def test_authorized_freeze_fixture_passes_without_threshold_change(self) -> None:
        with tempfile.TemporaryDirectory(prefix="assert-21008-") as tmp:
            freeze_path = Path(tmp) / "FREEZE.json"
            freeze_path.write_text(
                json.dumps(
                    {
                        "status": "FROZEN_CLEAN_TREE_REPLAY_AUTHORIZED",
                        "replay_authorized": True,
                        "replay_block_reason": None,
                        "preserved_motion_inference_guard": {
                            "minimum_planar_motion_m": 0.001,
                            "minimum_tangential_force_lower_bound_n": 0.5,
                            "thresholds_changed": False,
                        },
                        "scope_limits": {"admission_status": "NOT_ADMITTED"},
                    }
                ),
                encoding="utf-8",
            )
            result = subprocess.run(
                [sys.executable, str(TOOL), "--freeze", str(freeze_path), "--json"],
                cwd=ROOT,
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertTrue(json.loads(result.stdout)["ok"])


if __name__ == "__main__":
    unittest.main()
