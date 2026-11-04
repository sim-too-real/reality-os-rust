#!/usr/bin/env python3
"""Sealed 21008 first-onset policy comparison matches bound diagnosis numbers."""

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


class TestFirstOnsetPolicyComparison21008(unittest.TestCase):
    def test_sealed_policy_comparison_matches_diagnosis_and_miss_margin(self) -> None:
        diagnosis = json.loads(
            (SEED / "FRICTION_BOUND_DIAGNOSIS.json").read_text(encoding="utf-8")
        )
        miss = json.loads(
            (SEED / "PRIOR_ENVELOPE_MISS_MARGIN.json").read_text(encoding="utf-8")
        )
        comparison = json.loads(
            (SEED / "FIRST_ONSET_POLICY_COMPARISON.json").read_text(encoding="utf-8")
        )
        row = diagnosis["measurements"][0]
        self.assertEqual(row["diagnosis"], "entirely_above_prior")
        self.assertEqual(comparison["declared_prior"], row["prior"])
        self.assertEqual(comparison["measurement_bounds"], row["measurement_bounds"])
        self.assertEqual(comparison["bound_diagnosis"], "entirely_above_prior")
        self.assertEqual(
            comparison["prior_envelope_miss_margin_path"],
            "PRIOR_ENVELOPE_MISS_MARGIN.json",
        )
        self.assertEqual(miss["declared_prior"], comparison["declared_prior"])
        self.assertFalse(comparison["live_governor_path_changed"])
        self.assertEqual(comparison["live_policy"], "retain_declared_prior")
        self.assertEqual(comparison["governor_contact_threshold_m"], 0.001)
        self.assertEqual(comparison["governor_force_threshold_n"], 0.5)

        by_policy = {arm["policy"]: arm for arm in comparison["arms"]}
        self.assertFalse(by_policy["retain_declared_prior"]["proves_mu_required"])
        self.assertTrue(by_policy["retain_declared_prior"]["live_status_quo"])
        self.assertFalse(
            by_policy["prefer_higher_load_family_on_incompatible"]["proves_mu_required"]
        )
        self.assertFalse(
            by_policy["refuse_prior_envelope_as_not_credible"]["proves_mu_required"]
        )
        narrow = by_policy["narrow_to_measurement_on_entirely_above_prior"]
        self.assertTrue(narrow["proves_mu_required"])
        self.assertFalse(narrow["wired_live"])
        self.assertEqual(narrow["interval"], comparison["measurement_bounds"])
        self.assertGreaterEqual(narrow["interval"][0], comparison["mu_required"])
        self.assertEqual(comparison["admission_status"], "NOT_ADMITTED")
        self.assertFalse(comparison["metal"])


if __name__ == "__main__":
    unittest.main()
