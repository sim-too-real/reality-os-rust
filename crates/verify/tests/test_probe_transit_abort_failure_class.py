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
    def test_seed_21005_without_evidence_dir_keeps_empty_trace_class(self) -> None:
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

    def test_seed_21005_with_episode_diagnostic_refines_class(self) -> None:
        seed = EVIDENCE / "rotated-probe-heldout-seed-21005-20261104"
        episode = json.loads((seed / "EPISODE.json").read_text(encoding="utf-8"))
        result = json.loads((seed / "RESULT.json").read_text(encoding="utf-8"))
        report = MOD["classify_episode"](episode, result, evidence_dir=seed)
        self.assertEqual(
            report["failure_class"],
            "unexpected_contact_with_episode_diagnostic_only",
        )
        self.assertTrue(report["episode_diagnostic_present"])
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

    def test_seed_21003_successive_from_diagnosis_without_episode_onset_fields(
        self,
    ) -> None:
        seed = EVIDENCE / "rotated-probe-heldout-seed-21003-20261104"
        episode = json.loads((seed / "EPISODE.json").read_text(encoding="utf-8"))
        result = json.loads((seed / "RESULT.json").read_text(encoding="utf-8"))
        diagnosis = json.loads(
            (seed / "INFERENCE_BOUND_DIAGNOSIS.json").read_text(encoding="utf-8")
        )
        self.assertIsNone(episode.get("first_slip_onset_interval"))
        self.assertIsNone(episode.get("support_friction_prior_before_first_onset"))
        report = MOD["classify_episode"](
            episode, result, inference_bound_diagnosis=diagnosis
        )
        self.assertEqual(
            report["failure_class"],
            "expected_success_blocked_by_successive_prior_incompatibility",
        )
        self.assertFalse(report["governor_weakened"])

    def test_seed_21008_first_onset_outside_prior_not_successive(self) -> None:
        seed = EVIDENCE / "rotated-probe-heldout-seed-21008-20261104"
        episode = json.loads((seed / "EPISODE.json").read_text(encoding="utf-8"))
        result = json.loads((seed / "RESULT.json").read_text(encoding="utf-8"))
        diagnosis = None
        diagnosis_path = seed / "FRICTION_BOUND_DIAGNOSIS.json"
        if diagnosis_path.is_file():
            diagnosis = json.loads(diagnosis_path.read_text(encoding="utf-8"))
        report = MOD["classify_episode"](
            episode, result, inference_bound_diagnosis=diagnosis
        )
        self.assertEqual(
            report["failure_class"],
            "expected_success_blocked_by_first_onset_outside_prior",
        )
        self.assertIsNone(episode.get("first_slip_onset_interval"))
        self.assertFalse(report["governor_weakened"])


if __name__ == "__main__":
    unittest.main()
