#!/usr/bin/env python3
from __future__ import annotations

import json
import sys
import unittest
from pathlib import Path

VERIFY = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(VERIFY))

from probe_friction_bound_diagnosis import diagnose_sidecar  # noqa: E402

ROOT = VERIFY.parents[1]
EVIDENCE = ROOT / "docs" / "superpowers" / "evidence"


class ProbeFrictionBoundDiagnosisTests(unittest.TestCase):
    def test_21008_first_onset_entirely_above_prior(self) -> None:
        path = (
            EVIDENCE
            / "rotated-probe-heldout-seed-21008-20261104"
            / "PROBE_MEASUREMENT_SIDECAR.json"
        )
        if not path.is_file():
            self.skipTest("gitignored 21008 sidecar not present locally")
        report = diagnose_sidecar(json.loads(path.read_text(encoding="utf-8")))
        self.assertEqual(report["measurement_count"], 1)
        row = report["measurements"][0]
        self.assertEqual(row["diagnosis"], "entirely_above_prior")
        self.assertGreater(row["measurement_bounds"][0], row["prior"][1])
        self.assertEqual(report["admission_status"], "NOT_ADMITTED")


if __name__ == "__main__":
    unittest.main()
