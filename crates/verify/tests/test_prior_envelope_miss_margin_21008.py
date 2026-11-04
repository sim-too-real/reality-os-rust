#!/usr/bin/env python3
"""Sealed 21008 prior-envelope miss margin matches offline formula."""

from __future__ import annotations

import json
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SEED = (
    ROOT
    / "docs"
    / "superpowers"
    / "evidence"
    / "rotated-probe-heldout-seed-21008-20261104"
)


class TestPriorEnvelopeMissMargin21008(unittest.TestCase):
    def test_sealed_miss_margin_matches_bound_diagnosis(self) -> None:
        diagnosis = json.loads(
            (SEED / "FRICTION_BOUND_DIAGNOSIS.json").read_text(encoding="utf-8")
        )
        miss = json.loads(
            (SEED / "PRIOR_ENVELOPE_MISS_MARGIN.json").read_text(encoding="utf-8")
        )
        row = diagnosis["measurements"][0]
        self.assertEqual(row["diagnosis"], "entirely_above_prior")
        prior = row["prior"]
        bounds = row["measurement_bounds"]
        self.assertEqual(miss["declared_prior"], prior)
        self.assertEqual(miss["measurement_bounds"], bounds)
        absolute = bounds[0] - prior[1]
        relative = absolute / prior[1]
        self.assertAlmostEqual(miss["absolute_miss"], absolute, places=12)
        self.assertAlmostEqual(miss["relative_miss"], relative, places=12)
        self.assertGreater(miss["relative_miss"], 0.05)
        self.assertFalse(miss["live_governor_path_changed"])
        self.assertEqual(miss["admission_status"], "NOT_ADMITTED")
        self.assertFalse(miss["metal"])
        self.assertFalse(miss["shop_release"])
        self.assertFalse(miss["viewport_admits"])

    def test_sidecar_portfolio_records_miss_margin_for_21008(self) -> None:
        portfolio = json.loads(
            (
                ROOT
                / "docs"
                / "superpowers"
                / "evidence"
                / "rotated-probe-sidecar-verification-portfolio-20261104.json"
            ).read_text(encoding="utf-8")
        )
        row = next(
            r
            for r in portfolio["rows"]
            if r["seed_dir"] == "rotated-probe-heldout-seed-21008-20261104"
        )
        self.assertTrue(row.get("prior_envelope_miss_margin_present"))
        self.assertEqual(
            row.get("prior_envelope_miss_margin_path"),
            "PRIOR_ENVELOPE_MISS_MARGIN.json",
        )
        self.assertFalse(portfolio["governor_weakened_any"])
        self.assertIn("PRIOR_ENVELOPE_MISS_MARGIN", portfolio["conclusion"])
        self.assertTrue(row.get("first_onset_policy_comparison_present"))
        self.assertEqual(
            row.get("first_onset_policy_comparison_path"),
            "FIRST_ONSET_POLICY_COMPARISON.json",
        )
        self.assertIn("FIRST_ONSET_POLICY_COMPARISON sealed", portfolio["conclusion"])


if __name__ == "__main__":
    unittest.main()
