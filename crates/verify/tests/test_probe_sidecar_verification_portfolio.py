#!/usr/bin/env python3
from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path

VERIFY = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(VERIFY))

from probe_sidecar_verification_portfolio import (  # noqa: E402
    build_portfolio,
    verify_evidence_dir,
)

ROOT = VERIFY.parents[1]
EVIDENCE = ROOT / "docs" / "superpowers" / "evidence"


class ProbeSidecarVerificationPortfolioTests(unittest.TestCase):
    def test_21004_fails_no_sidecar_with_diagnostic_integrity(self) -> None:
        row = verify_evidence_dir(
            EVIDENCE / "rotated-probe-heldout-seed-21004-20261104"
        )
        self.assertEqual(row["verification_status"], "FAIL_NO_SIDECAR")
        self.assertFalse(row["invented_measurements"])
        self.assertFalse(row["governor_weakened"])
        self.assertEqual(
            row["episode_diagnostic"]["status"], "INTEGRITY_PASS"
        )
        self.assertEqual(
            row["failure_class_after_verification"],
            "unexpected_contact_with_episode_diagnostic_only",
        )

    def test_21005_fails_no_sidecar(self) -> None:
        row = verify_evidence_dir(
            EVIDENCE / "rotated-probe-heldout-seed-21005-20261104"
        )
        self.assertEqual(row["verification_status"], "FAIL_NO_SIDECAR")
        self.assertEqual(
            row["failure_class_after_verification"],
            "unexpected_contact_with_episode_diagnostic_only",
        )

    def test_21004_sidecar_capture_records_pass_at_seal(self) -> None:
        path = EVIDENCE / "rotated-probe-heldout-seed-21004-sidecar-capture-20261104"
        if not (path / "PROBE_MEASUREMENT_SIDECAR.json").is_file():
            self.skipTest("gitignored 21004 capture sidecar not present locally")
        row = verify_evidence_dir(path)
        self.assertEqual(row["verification_status"], "PASS_RECORDED_AT_SEAL")
        self.assertEqual(
            row["result_recorded_sidecar_status"],
            "PASS_STRUCTURAL_AMBIGUOUS_CONTACT",
        )
        self.assertEqual(
            row["failure_class_after_verification"],
            "unexpected_contact_with_preserved_episode",
        )
        self.assertFalse(row["governor_weakened"])

    def test_21005_sidecar_capture_records_pass_at_seal(self) -> None:
        path = EVIDENCE / "rotated-probe-heldout-seed-21005-sidecar-capture-20261104"
        if not (path / "PROBE_MEASUREMENT_SIDECAR.json").is_file():
            self.skipTest("gitignored 21005 capture sidecar not present locally")
        row = verify_evidence_dir(path)
        self.assertEqual(row["verification_status"], "PASS_RECORDED_AT_SEAL")
        self.assertEqual(
            row["result_recorded_sidecar_status"],
            "PASS_STRUCTURAL_AMBIGUOUS_CONTACT",
        )
        self.assertEqual(
            row["failure_class_after_verification"],
            "unexpected_contact_with_preserved_episode",
        )
        self.assertFalse(row["governor_weakened"])

    def test_21007_records_pass_without_inventing(self) -> None:
        row = verify_evidence_dir(
            EVIDENCE / "rotated-probe-heldout-seed-21007-20261104"
        )
        self.assertTrue(str(row["verification_status"]).startswith("PASS"))
        self.assertFalse(row["invented_measurements"])
        self.assertGreater(row.get("measurement_count") or 0, 0)

    def test_empty_sidecar_file_is_fail_empty(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)
            (path / "PROBE_MEASUREMENT_SIDECAR.json").write_text(
                json.dumps(
                    {
                        "format": "realityos.probe_measurement_sidecar",
                        "schema_version": 1,
                        "trial_id": "t",
                        "trial_digest": "d",
                        "measurements": [],
                    }
                ),
                encoding="utf-8",
            )
            (path / "EPISODE.json").write_text(
                json.dumps(
                    {
                        "trial_id": "t",
                        "trial_digest": "d",
                        "outcome": "action_failed",
                        "failure_cause": "force_monitor_abort",
                        "refusal_code": "WRENCH_MONITOR_ABORTED_ACTION",
                        "unauthorized_writes": 0,
                    }
                ),
                encoding="utf-8",
            )
            (path / "RESULT.json").write_text(
                json.dumps(
                    {
                        "run": {"wrench_abort_reason": {"kind": "unexpected_contact"}},
                        "verification": {},
                    }
                ),
                encoding="utf-8",
            )
            row = verify_evidence_dir(path)
            self.assertEqual(row["verification_status"], "FAIL_EMPTY_MEASUREMENTS")

    def test_portfolio_requires_fail_and_pass_arms(self) -> None:
        portfolio = build_portfolio(
            [
                "rotated-probe-heldout-seed-21004-20261104",
                "rotated-probe-heldout-seed-21005-20261104",
                "rotated-probe-heldout-seed-21007-20261104",
            ]
        )
        self.assertGreaterEqual(portfolio["fail_no_sidecar_count"], 2)
        self.assertGreaterEqual(portfolio["pass_count"], 1)
        self.assertFalse(portfolio["governor_weakened_any"])
        self.assertFalse(portfolio["invented_measurements_any"])


if __name__ == "__main__":
    unittest.main()
