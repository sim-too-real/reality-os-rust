#!/usr/bin/env python3
from __future__ import annotations

import json
import runpy
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
MOD = runpy.run_path(str(ROOT / "crates/verify/probe_failure_class_portfolio.py"))


class ProbeFailureClassPortfolioTests(unittest.TestCase):
    def test_portfolio_classifies_sealed_seeds_without_weakening_governor(self) -> None:
        with tempfile.TemporaryDirectory(prefix="failclass-port-") as tmp:
            out = Path(tmp) / "portfolio.json"
            code = MOD["main"](["--out", str(out)])
            self.assertEqual(code, 0)
            data = json.loads(out.read_text(encoding="utf-8"))
            self.assertEqual(data["schema"], "realityos.probe_failure_class_portfolio/v1")
            self.assertGreaterEqual(data["classified_count"], 4)
            self.assertFalse(data["governor_weakened_any"])
            classes = {
                row["seed_dir"]: row["failure_class"]
                for row in data["rows"]
                if row["status"] == "CLASSIFIED"
            }
            self.assertEqual(
                classes.get("rotated-probe-heldout-seed-21005-20261104"),
                "transit_unexpected_contact_without_measurement_trace",
            )

    def test_portfolio_fails_closed_when_a_seed_is_missing(self) -> None:
        with tempfile.TemporaryDirectory(prefix="failclass-missing-") as tmp:
            evidence = Path(tmp) / "evidence"
            evidence.mkdir()
            out = Path(tmp) / "portfolio.json"
            module_globals = MOD["main"].__globals__
            previous_evidence = module_globals["EVIDENCE"]
            module_globals["EVIDENCE"] = evidence
            try:
                code = MOD["main"](["--out", str(out)])
            finally:
                module_globals["EVIDENCE"] = previous_evidence

            self.assertEqual(code, 2)
            data = json.loads(out.read_text(encoding="utf-8"))
            self.assertEqual(data["classified_count"], 0)
            self.assertEqual(data["missing_count"], 4)

    def test_portfolio_fails_closed_when_result_is_missing(self) -> None:
        with tempfile.TemporaryDirectory(prefix="failclass-no-result-") as tmp:
            evidence = Path(tmp) / "evidence"
            seed = evidence / "rotated-probe-heldout-seed-21004-20261104"
            seed.mkdir(parents=True)
            (seed / "EPISODE.json").write_text('{"outcome":"success"}\n', encoding="utf-8")
            out = Path(tmp) / "portfolio.json"
            module_globals = MOD["main"].__globals__
            previous_evidence = module_globals["EVIDENCE"]
            module_globals["EVIDENCE"] = evidence
            try:
                code = MOD["main"](["--out", str(out)])
            finally:
                module_globals["EVIDENCE"] = previous_evidence

            self.assertEqual(code, 2)
            data = json.loads(out.read_text(encoding="utf-8"))
            self.assertEqual(data["missing_count"], 4)
            self.assertEqual(data["rows"][0]["status"], "SKIPPED_NO_RESULT")


if __name__ == "__main__":
    unittest.main()
