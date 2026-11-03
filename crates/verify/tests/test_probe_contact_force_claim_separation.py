"""Unit tests for offline onset force-ratio vs Coulomb-parameter claim separation."""

from __future__ import annotations

import importlib.util
import unittest
from pathlib import Path


def _load_analysis_module():
    path = Path(__file__).resolve().parents[1] / "probe_contact_force_analysis.py"
    spec = importlib.util.spec_from_file_location("probe_contact_force_analysis", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class ClaimSeparationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.mod = _load_analysis_module()

    def test_separates_force_ratio_overlap_from_coulomb_miss(self) -> None:
        contacts = [
            {
                "contact_pair": {"body1": "obj0", "body2": "table"},
                "friction": [0.75, 0.75],
                "normal_force_n": 10.0,
                "tangential_force_n": 4.5,
            }
        ]
        record = {"posterior_support_friction_interval": [0.40, 0.50]}
        result = self.mod._compare_friction_interval(record, contacts)
        self.assertEqual(result["status"], "disjoint")
        separation = result["claim_separation"]
        self.assertEqual(
            separation["onset_force_ratio_vs_contact_tn"]["status"], "overlap"
        )
        self.assertEqual(
            separation["coulomb_parameter_in_inferred_interval"]["status"], "no"
        )

    def test_reports_unavailable_without_contacts(self) -> None:
        record = {"posterior_support_friction_interval": [0.40, 0.50]}
        result = self.mod._compare_friction_interval(record, None)
        self.assertEqual(result["status"], "unavailable")
        self.assertEqual(result["claim_separation"]["status"], "unavailable")


if __name__ == "__main__":
    unittest.main()
