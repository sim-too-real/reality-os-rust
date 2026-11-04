from __future__ import annotations

import hashlib
import json
import runpy
import tempfile
import unittest
from pathlib import Path

MODULE = runpy.run_path(str(Path(__file__).with_name("write_rotated_probe_freeze_21008.py")))


def sha256_upper(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest().upper()


class WriteRotatedProbeFreeze21008Tests(unittest.TestCase):
    def _prepare_case(self, root: Path) -> tuple[dict, dict, Path, dict]:
        evidence = root / "evidence"
        evidence.mkdir()
        source = root / "src.py"
        source.write_text("source\n", encoding="utf-8")
        motion_source_rel = MODULE["EXPECTED_MOTION_GUARD"]["source"]
        motion_source = root / motion_source_rel
        motion_source.parent.mkdir(parents=True)
        motion_source.write_text(
            "const PROBE_SLIP_DISPLACEMENT_M: f64 = 0.001;\n"
            "const MIN_PROBE_CONTACT_FORCE_N: f64 = 0.5;\n",
            encoding="utf-8",
        )
        model = root / "model.xml"
        model.write_text("model\n", encoding="utf-8")
        trial = {
            "trial_id": "v7:development:menagerie_iiwa14:push_probe_replan:21008:0",
            "seed": 21008,
            "index": 0,
            "expectation": "expected_success",
            "model_hash": "model-digest",
            "source_hash": "source-digest",
        }
        preflight = {
            "trial_id": trial["trial_id"],
            "trial_digest": "a" * 64,
            "authorized_action_key": "probe:+x",
            "authorized_grant_id": "grant-1",
            "control_writes_before": 0,
            "control_writes_after": 0,
            "simulation_time_before_s": 0.0,
            "simulation_time_after_s": 0.0,
            "simulator_steps": 0,
        }
        trial_path = evidence / "TRIAL_INPUT.json"
        trial_path.write_text(json.dumps(trial), encoding="utf-8")
        preflight_path = evidence / "PREFLIGHT.json"
        preflight_path.write_text(json.dumps(preflight), encoding="utf-8")
        template = {
            "source_hashes_sha256": {
                "src.py": sha256_upper(source),
                motion_source_rel: sha256_upper(motion_source),
            },
            "model_files_sha256": {"model.xml": sha256_upper(model)},
            "preserved_motion_inference_guard": MODULE["EXPECTED_MOTION_GUARD"],
            "scope_limits": {
                "physical_test": False,
                "physical_actions": 0,
                "purchases": 0,
                "fabrication": 0,
                "external_contact": False,
                "admission_status": "NOT_ADMITTED",
                "metal": False,
                "shop_release": False,
                "viewport_admits": False,
            },
        }
        template_path = root / "template.json"
        template_path.write_text(json.dumps(template), encoding="utf-8")
        namespace = MODULE["main"].__globals__
        previous = {
            name: namespace[name]
            for name in ("ROOT", "EVIDENCE", "TEMPLATE", "git_head", "git_status_paths")
        }
        namespace["ROOT"] = root
        namespace["EVIDENCE"] = evidence
        namespace["TEMPLATE"] = template_path
        namespace["git_head"] = lambda: "1" * 40
        namespace["git_status_paths"] = lambda: ["evidence/TRIAL_INPUT.json"]
        return namespace, preflight, evidence, previous

    def test_valid_prepare_only_evidence_writes_a_passing_freeze(self) -> None:
        with tempfile.TemporaryDirectory(prefix="freeze-21008-valid-") as tmp:
            namespace, _, evidence, previous = self._prepare_case(Path(tmp))
            try:
                code = MODULE["main"]()

                verify = json.loads((evidence / "FREEZE_VERIFY.json").read_text(encoding="utf-8"))
                freeze = json.loads((evidence / "FREEZE.json").read_text(encoding="utf-8"))
                self.assertEqual(code, 0)
                self.assertEqual(verify["status"], "PASS_WORKING_TREE")
                self.assertEqual(freeze["status"], "FROZEN_WORKING_TREE_AWAITING_COMMIT_BEFORE_REPLAY")
                self.assertTrue(verify["checks"]["working_tree_changes_at_freeze"])
                self.assertIn("evidence/TRIAL_INPUT.json", verify["checks"]["working_tree_change_paths"])
                self.assertFalse(freeze["replay_authorized"])
            finally:
                namespace.update(previous)

    def test_bad_preflight_cannot_write_a_passing_freeze_or_return_zero(self) -> None:
        with tempfile.TemporaryDirectory(prefix="freeze-21008-bad-") as tmp:
            namespace, preflight, evidence, previous = self._prepare_case(Path(tmp))
            try:
                preflight["simulator_steps"] = 1
                (evidence / "PREFLIGHT.json").write_text(json.dumps(preflight), encoding="utf-8")

                code = MODULE["main"]()

                verify = json.loads((evidence / "FREEZE_VERIFY.json").read_text(encoding="utf-8"))
                freeze = json.loads((evidence / "FREEZE.json").read_text(encoding="utf-8"))
                self.assertEqual(code, 2)
                self.assertEqual(verify["status"], "FAIL_WORKING_TREE")
                self.assertIn("preflight_zero_step", verify["checks"]["required_check_failures"])
                self.assertEqual(freeze["status"], "INVALID_WORKING_TREE_PREFLIGHT")
                self.assertFalse(freeze["replay_authorized"])
            finally:
                namespace.update(previous)

    def test_changed_motion_threshold_cannot_write_a_passing_freeze(self) -> None:
        with tempfile.TemporaryDirectory(prefix="freeze-21008-threshold-") as tmp:
            namespace, _, evidence, previous = self._prepare_case(Path(tmp))
            try:
                motion_source = Path(tmp) / MODULE["EXPECTED_MOTION_GUARD"]["source"]
                motion_source.write_text(
                    "const PROBE_SLIP_DISPLACEMENT_M: f64 = 0.002;\n"
                    "const MIN_PROBE_CONTACT_FORCE_N: f64 = 0.5;\n",
                    encoding="utf-8",
                )

                code = MODULE["main"]()

                verify = json.loads((evidence / "FREEZE_VERIFY.json").read_text(encoding="utf-8"))
                freeze = json.loads((evidence / "FREEZE.json").read_text(encoding="utf-8"))
                self.assertEqual(code, 2)
                self.assertFalse(verify["checks"]["thresholds_unchanged"])
                self.assertIn("thresholds_unchanged", verify["checks"]["required_check_failures"])
                self.assertEqual(freeze["status"], "INVALID_WORKING_TREE_PREFLIGHT")
            finally:
                namespace.update(previous)

    def test_malformed_preflight_counts_fail_closed(self) -> None:
        for field, value, expected_check in (
            ("simulator_steps", False, "preflight_zero_step"),
            ("control_writes_before", 0.0, "control_writes_unchanged"),
        ):
            with self.subTest(field=field), tempfile.TemporaryDirectory(
                prefix="freeze-21008-count-type-"
            ) as tmp:
                namespace, preflight, evidence, previous = self._prepare_case(Path(tmp))
                try:
                    preflight[field] = value
                    (evidence / "PREFLIGHT.json").write_text(
                        json.dumps(preflight), encoding="utf-8"
                    )

                    code = MODULE["main"]()

                    verify = json.loads(
                        (evidence / "FREEZE_VERIFY.json").read_text(encoding="utf-8")
                    )
                    self.assertEqual(code, 2)
                    self.assertFalse(verify["checks"][expected_check])
                    self.assertIn(
                        expected_check,
                        verify["checks"]["required_check_failures"],
                    )
                finally:
                    namespace.update(previous)


if __name__ == "__main__":
    unittest.main()
