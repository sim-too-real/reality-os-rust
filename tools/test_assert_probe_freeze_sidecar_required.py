#!/usr/bin/env python3
from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path

TOOLS = Path(__file__).resolve().parent
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))

from assert_probe_freeze_sidecar_required import assess_freeze_dir


class TestProbeFreezeSidecarRequired(unittest.TestCase):
    def _write(self, root: Path, freeze: dict, sidecar=None) -> Path:
        root.mkdir(parents=True, exist_ok=True)
        (root / "FREEZE.json").write_text(json.dumps(freeze), encoding="utf-8")
        if sidecar is not None:
            (root / "PROBE_MEASUREMENT_SIDECAR.json").write_text(
                json.dumps(sidecar), encoding="utf-8"
            )
        return root

    def test_non_contact_class_passes(self):
        with tempfile.TemporaryDirectory() as tmp:
            ev = self._write(
                Path(tmp) / "ev",
                {"schema": "test", "observation_class": "motion_only"},
            )
            row = assess_freeze_dir(ev)
            self.assertTrue(row["seal_allowed"])
            self.assertEqual(row["status"], "PASS_NOT_CONTACT_CLASS")

    def test_contact_class_without_sidecar_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            ev = self._write(
                Path(tmp) / "ev",
                {
                    "schema": "test",
                    "observation_class": "transit_abort",
                    "require_probe_sidecar": True,
                },
            )
            row = assess_freeze_dir(ev)
            self.assertFalse(row["seal_allowed"])
            self.assertEqual(row["status"], "FAIL_NO_SIDECAR")
            self.assertFalse(row["governor_weakened"])
            self.assertFalse(row["invented_measurements"])

    def test_contact_class_with_sidecar_passes(self):
        with tempfile.TemporaryDirectory() as tmp:
            ev = self._write(
                Path(tmp) / "ev",
                {"schema": "test", "observation_class": "contact"},
                sidecar={"records": [{"t": 0.0, "force_n": 1.0}]},
            )
            row = assess_freeze_dir(ev)
            self.assertTrue(row["seal_allowed"])
            self.assertEqual(row["status"], "PASS_SIDECAR_PRESENT")

    def test_empty_sidecar_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            ev = self._write(
                Path(tmp) / "ev",
                {"schema": "test", "require_probe_sidecar": True},
                sidecar={},
            )
            row = assess_freeze_dir(ev)
            self.assertFalse(row["seal_allowed"])
            self.assertEqual(row["status"], "FAIL_EMPTY_SIDECAR")


if __name__ == "__main__":
    unittest.main()
