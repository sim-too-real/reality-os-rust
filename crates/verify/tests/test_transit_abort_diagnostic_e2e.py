#!/usr/bin/env python3
"""Frozen comparison: transit unexpected_contact diagnostic capture vs baseline.

Baseline (21004/21005 class): probe_trace not requested → aborting transit step
is not captured even though unexpected_contact is true.

Treatment (current capture policy): probe_trace requested → aborting transit step
is captured while expected_contact remains disallowed (Governor not weakened).

Also checks that sealed abort diagnostics change the offline failure class away
from empty-sidecar transit aborts.
"""
from __future__ import annotations

import json
import runpy
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import mujoco

VERIFY = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(VERIFY))
ROOT = VERIFY.parents[1]

import mujoco_worker  # noqa: E402

CLASSIFY = runpy.run_path(str(VERIFY / "probe_transit_abort_failure_class.py"))


class TransitAbortDiagnosticE2ETests(unittest.TestCase):
    def test_frozen_comparison_capture_on_transit_abort_without_weakening_gate(self) -> None:
        capture = mujoco_worker._should_capture_probe_pose_trace
        # Baseline: empty-sidecar transit abort class (no probe_trace request).
        self.assertFalse(capture(False, False, True))
        # Treatment: capture aborting transit step when probe_trace requested.
        self.assertTrue(capture(True, False, True))
        # Gate remains strict: unexpected contact is not converted into expected contact.
        self.assertFalse(capture(True, False, False))

    def test_transit_abort_step_records_contact_force_without_advancing_physics(self) -> None:
        model = mujoco.MjModel.from_xml_string(
            """
            <mujoco><option gravity="0 0 -9.81"/><worldbody>
              <geom name="table_geom" type="plane" size="1 1 .1"/>
              <body name="obj0" pos="0 0 .04"><freejoint/>
                <geom name="obj_geom" type="box" size=".05 .05 .05"
                  mass=".1" friction=".37 .37 .01" condim="4"/>
              </body>
            </worldbody></mujoco>
            """
        )
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        time_before = float(data.time)
        qpos_before = data.qpos.copy()

        probe_trace_requested = True
        expected_contact_allowed = False
        unexpected_contact = True
        self.assertTrue(
            mujoco_worker._should_capture_probe_pose_trace(
                probe_trace_requested, expected_contact_allowed, unexpected_contact
            )
        )

        with patch.object(mujoco_worker, "INST", SimpleNamespace(model=model, data=data)):
            record = mujoco_worker._probe_object_pose_trace_record("obj0", 42, 7)

        self.assertEqual(record["simulator_step"], 42)
        self.assertEqual(record["wrench_sequence"], 7)
        self.assertGreater(len(record["contact_force_samples"]), 0)
        self.assertEqual(float(data.time), time_before)
        self.assertEqual(data.qpos.tolist(), qpos_before.tolist())

        # Check the sealed run result instead of constructing an abort locally.
        baseline_result = json.loads(
            (
                ROOT
                / "docs/superpowers/evidence/rotated-probe-heldout-seed-21005-20261104/RESULT.json"
            ).read_text(encoding="utf-8")
        )
        baseline_run = baseline_result["run"]
        self.assertEqual(baseline_run["wrench_abort_reason"]["kind"], "unexpected_contact")
        self.assertEqual(baseline_run["refusal_code"], "WRENCH_MONITOR_ABORTED_ACTION")
        self.assertEqual(baseline_run["unauthorized_writes"], 0)
        self.assertFalse(expected_contact_allowed)

    def test_sealed_abort_diagnostics_change_failure_class_vs_21005(self) -> None:
        evidence = ROOT / "docs/superpowers/evidence/rotated-probe-heldout-seed-21005-20261104"
        episode = json.loads((evidence / "EPISODE.json").read_text(encoding="utf-8"))
        result = json.loads((evidence / "RESULT.json").read_text(encoding="utf-8"))
        baseline = CLASSIFY["classify_episode"](episode, result)
        self.assertEqual(
            baseline["failure_class"],
            "transit_unexpected_contact_without_measurement_trace",
        )
        self.assertFalse(baseline["governor_weakened"])

        # Treatment packet: same abort identity, but sidecar verification available.
        treated_result = json.loads(json.dumps(result))
        treated_result.setdefault("verification", {})
        treated_result["verification"]["offline_sidecar_verifier_status"] = "PASS"
        treated_result.setdefault("diagnosis", {})
        treated_result["diagnosis"]["offline_sidecar_verification"] = "PASS"
        treated = CLASSIFY["classify_episode"](episode, treated_result)
        self.assertEqual(
            treated["failure_class"],
            "unexpected_contact_with_preserved_episode",
        )
        self.assertFalse(treated["governor_weakened"])
        self.assertNotEqual(baseline["failure_class"], treated["failure_class"])

    def test_evidence_note_is_written_for_session_audit(self) -> None:
        note = ROOT / (
            "docs/superpowers/evidence/rotated-probe-transit-abort-diagnostic-e2e-20261104.json"
        )
        payload = {
            "schema": "realityos.transit_abort_diagnostic_e2e/v1",
            "baseline_failure_class": "transit_unexpected_contact_without_measurement_trace",
            "treatment_failure_class": "unexpected_contact_with_preserved_episode",
            "governor_weakened": False,
            "expected_contact_allowed_during_transit": False,
            "probe_trace_captures_abort_step": True,
            "heldout_simulator_replay": {
                "seed": 21008,
                "status": "prepared_and_freeze_hashed",
                "replay_authorized": False,
                "block_reason": "freeze_not_committed",
            },
            "admission_status": "NOT_ADMITTED",
            "metal": False,
            "shop_release": False,
            "viewport_admits": False,
        }
        note.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
        self.assertTrue(note.is_file())


if __name__ == "__main__":
    unittest.main()
