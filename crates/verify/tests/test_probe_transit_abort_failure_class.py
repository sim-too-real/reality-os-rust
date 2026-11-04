#!/usr/bin/env python3
from __future__ import annotations

import json
import runpy
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
MOD = runpy.run_path(str(ROOT / "crates/verify/probe_transit_abort_failure_class.py"))
EVIDENCE = ROOT / "docs/superpowers/evidence"


class ProbeTransitAbortFailureClassTests(unittest.TestCase):
    def test_seed_21005_classifies_transit_abort_without_sidecar(self) -> None:
        episode = json.loads(
            (EVIDENCE / "rotated-probe-heldout-seed-21005-20261104/EPISODE.json").read_text(
                encoding="utf-8"
            )
        )
        result = json.loads(
            (EVIDENCE / "rotated-probe-heldout-seed-21005-20261104/RESULT.json").read_text(
                encoding="utf-8"
            )
        )
        report = MOD["classify_episode"](episode, result)
        self.assertEqual(
            report["failure_class"],
            "transit_unexpected_contact_without_measurement_trace",
        )
        self.assertFalse(report["governor_weakened"])
        self.assertEqual(report["admission_status"], "NOT_ADMITTED")

    def test_seed_21007_is_not_empty_sidecar_transit_class(self) -> None:
        episode = json.loads(
            (EVIDENCE / "rotated-probe-heldout-seed-21007-20261104/EPISODE.json").read_text(
                encoding="utf-8"
            )
        )
        result = json.loads(
            (EVIDENCE / "rotated-probe-heldout-seed-21007-20261104/RESULT.json").read_text(
                encoding="utf-8"
            )
        )
        report = MOD["classify_episode"](episode, result)
        self.assertNotEqual(
            report["failure_class"],
            "transit_unexpected_contact_without_measurement_trace",
        )
        self.assertFalse(report["governor_weakened"])


if __name__ == "__main__":
    unittest.main()
