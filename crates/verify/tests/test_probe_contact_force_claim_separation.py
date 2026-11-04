"""Unit tests for offline onset force-ratio vs Coulomb-parameter claim separation."""

from __future__ import annotations

import importlib.util
import json
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

    def test_seed_21007_offline_analysis_passes_claim_separation_gate(self) -> None:
        path = (
            Path(__file__).resolve().parents[3]
            / "docs"
            / "superpowers"
            / "evidence"
            / "rotated-probe-heldout-seed-21007-20261104"
            / "OFFLINE_ANALYSIS.json"
        )
        analysis = json.loads(path.read_text(encoding="utf-8"))
        self.assertEqual(self.mod.claim_separation_gate_violations(analysis), [])

    def test_seed_21006_offline_analysis_passes_claim_separation_gate(self) -> None:
        path = (
            Path(__file__).resolve().parents[3]
            / "docs"
            / "superpowers"
            / "evidence"
            / "rotated-probe-heldout-seed-21006-20261104"
            / "OFFLINE_ANALYSIS.json"
        )
        analysis = json.loads(path.read_text(encoding="utf-8"))
        comparison = analysis["measurements"][0]["slip_onset"]["inference_vs_simulator_friction"]
        self.assertEqual(comparison["status"], "overlap")
        self.assertIn("claim_separation", comparison)
        self.assertEqual(self.mod.claim_separation_gate_violations(analysis), [])

    def test_heldout_offline_analysis_portfolio_passes_claim_separation_gate(self) -> None:
        portfolio_path = Path(__file__).resolve().parents[1] / "probe_claim_separation_portfolio.py"
        spec = importlib.util.spec_from_file_location(
            "probe_claim_separation_portfolio", portfolio_path
        )
        module = importlib.util.module_from_spec(spec)
        assert spec.loader is not None
        spec.loader.exec_module(module)
        report = module.evaluate()
        self.assertGreaterEqual(report["analysis_count"], 4)
        self.assertEqual(report["status"], "PASS", report.get("failures"))
        self.assertEqual(report["failure_count"], 0)

    def test_incomplete_no_sidecar_analysis_passes_claim_separation_gate(self) -> None:
        analysis = {
            "status": "INCOMPLETE_NO_MEASUREMENT_SIDECAR",
            "episode": {"outcome": "action_failed"},
        }
        self.assertEqual(self.mod.claim_separation_gate_violations(analysis), [])

    def test_claim_separation_gate_fails_when_subclaims_are_stripped(self) -> None:
        analysis = {
            "measurements": [
                {
                    "slip_onset": {
                        "inference_vs_simulator_friction": {
                            "status": "disjoint",
                            "claim_separation": {"note": "incomplete"},
                        }
                    }
                }
            ]
        }
        violations = self.mod.claim_separation_gate_violations(analysis)
        self.assertIn("measurement[0]_missing_onset_force_ratio_claim", violations)
        self.assertIn("measurement[0]_missing_coulomb_parameter_claim", violations)

    def test_claim_separation_gate_rejects_force_ratio_claimed_as_coulomb_mu(self) -> None:
        analysis = {
            "measurements": [
                {
                    "slip_onset": {
                        "inference_vs_simulator_friction": {
                            "status": "disjoint",
                            "claim_separation": {
                                "onset_force_ratio_vs_contact_tn": {"status": "overlap"},
                                "coulomb_parameter_in_inferred_interval": {"status": "no"},
                                "coulomb_mu_identified": True,
                                "inferred_quantity": "onset_or_sticking_force_ratio",
                                "note": "Coulomb note present",
                            },
                        }
                    }
                }
            ]
        }
        violations = self.mod.claim_separation_gate_violations(analysis)
        self.assertIn(
            "measurement[0]_coulomb_mu_identified_true_without_interval_hit",
            violations,
        )
        self.assertIn(
            "measurement[0]_force_ratio_quantity_cannot_identify_coulomb_mu",
            violations,
        )


if __name__ == "__main__":
    unittest.main()
