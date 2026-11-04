import copy
import json
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from probe_interval_reference import (  # noqa: E402
    ReplayError,
    _measurement_digest,
    _pose_sample_digest,
    _trace_digest,
    verify_sidecar,
)


def sidecar_path(trial_seed: int) -> Path:
    if trial_seed == 23021:
        folder = "fresh-out-of-cohort-probe-contact-trace-v1-23021-20261103"
    else:
        folder = (
            "fresh-out-of-cohort-per-step-contact-trace-v1-"
            f"{trial_seed}-20261103"
        )
    return (
        ROOT
        / "docs/superpowers/evidence/grasp-lift-margin-20261102"
        / folder
        / "PROBE_MEASUREMENT_SIDECAR.json"
    )


def load_sidecar(trial_seed: int) -> dict:
    return json.loads(sidecar_path(trial_seed).read_text(encoding="utf-8"))


class IndependentProbeIntervalReferenceTests(unittest.TestCase):
    def test_recomputes_posthoc_archived_slip_onset_interval_23021(self):
        report = verify_sidecar(load_sidecar(23021))

        self.assertEqual(report["status"], "PASS")
        self.assertEqual(
            report["trial_id"],
            "v7:development:menagerie_ur5e:push_probe_replan:23021:17",
        )
        self.assertEqual(report["measurements"][0]["inference"], "Updated(SlipOnset)")
        self.assertAlmostEqual(
            report["measurements"][0]["interval"][0],
            0.47885574015040633,
            delta=1e-12,
        )
        self.assertAlmostEqual(
            report["measurements"][0]["interval"][1],
            0.721137085017195,
            delta=1e-12,
        )
        self.assertEqual(
            report["measurements"][0]["first_xy_motion_bracket_s"],
            [23.555999999997823, 23.557999999997822],
        )

    def test_recomputes_frozen_slip_onset_interval(self):
        report = verify_sidecar(load_sidecar(23023))

        self.assertEqual(report["status"], "PASS")
        self.assertEqual(
            report["trial_id"],
            "v7:development:menagerie_ur5e:push_probe_replan:23023:17",
        )
        self.assertEqual(report["measurements"][0]["inference"], "Updated(SlipOnset)")
        self.assertAlmostEqual(
            report["measurements"][0]["interval"][0],
            0.47116960533439084,
            delta=1e-12,
        )
        self.assertAlmostEqual(
            report["measurements"][0]["interval"][1],
            0.5484442443281078,
            delta=1e-12,
        )

    def test_recomputes_frozen_no_intended_contact_refusal(self):
        report = verify_sidecar(load_sidecar(23024))

        self.assertEqual(report["status"], "PASS")
        self.assertEqual(
            report["trial_id"],
            "v7:development:menagerie_ur5e:push_probe_replan:23024:17",
        )
        self.assertEqual(
            report["measurements"][0]["inference"],
            "NoInference(NoIntendedContact)",
        )
        self.assertEqual(report["measurements"][0]["interval"], [0.05, 0.8])

    def test_rejects_changed_raw_wrench_with_stale_measurement_digest(self):
        sidecar = load_sidecar(23023)
        mutated = copy.deepcopy(sidecar)
        mutated["measurements"][0]["measurement"]["samples"][0]["wrench"][0] += 1.0

        with self.assertRaisesRegex(ReplayError, "sample digest mismatch"):
            verify_sidecar(mutated)

    def test_rejects_tampered_recorded_inference_interval(self):
        sidecar = load_sidecar(23023)
        mutated = copy.deepcopy(sidecar)
        mutated["measurements"][0]["inference"]["Updated"]["interval"][0] += 0.01

        with self.assertRaisesRegex(ReplayError, "stored inference mismatch"):
            verify_sidecar(mutated)

    def test_rejects_unknown_stored_no_inference_reason(self):
        sidecar = load_sidecar(23024)
        mutated = copy.deepcopy(sidecar)
        mutated["measurements"][0]["inference"]["NoInference"] = "invalid_measurement"

        with self.assertRaisesRegex(ReplayError, "stored inference mismatch"):
            verify_sidecar(mutated)

    def test_rejects_unknown_schema_version(self):
        sidecar = load_sidecar(23023)
        sidecar["schema_version"] = 2

        with self.assertRaisesRegex(ReplayError, "unsupported sidecar format"):
            verify_sidecar(sidecar)

    def test_malformed_field_is_reported_as_replay_error(self):
        sidecar = load_sidecar(23023)
        del sidecar["measurements"][0]["measurement"]["samples"][0]["capture_s"]

        with self.assertRaisesRegex(ReplayError, "invalid sidecar"):
            verify_sidecar(sidecar)

    def test_rejects_non_object_pose_sample_as_replay_error(self):
        sidecar = load_sidecar(23024)
        sidecar["measurements"][0]["measurement"]["motion_trace"]["samples"][0] = None

        with self.assertRaises(ReplayError):
            verify_sidecar(sidecar)

    def test_rejects_pose_mismatch_at_trace_start(self):
        sidecar = load_sidecar(23024)
        measurement = sidecar["measurements"][0]["measurement"]
        trace = measurement["motion_trace"]
        first = copy.deepcopy(trace["samples"][0])
        first["simulator_step"] -= 1
        first["wrench_sequence"] = None
        first["capture_s"] = trace["start_s"]
        first["orientation_world_wxyz"] = [0.9999875000260416, 0.004999979166692708, 0.0, 0.0]
        first["digest"] = _pose_sample_digest(first)
        trace["samples"].insert(0, first)
        trace["digest"] = _trace_digest(trace)
        sidecar["measurements"][0]["measurement_digest"] = _measurement_digest(measurement)

        with self.assertRaisesRegex(ReplayError, "start pose mismatch"):
            verify_sidecar(sidecar)

    def test_rejects_incomplete_contact_trace(self):
        sidecar = load_sidecar(23024)
        mutated = copy.deepcopy(sidecar)
        mutated["measurements"][0]["measurement"]["motion_trace"]["samples"][0][
            "contact_pairs"
        ] = None

        with self.assertRaisesRegex(ReplayError, "incomplete contact trace"):
            verify_sidecar(mutated)

    def test_rejects_archived_unidentifiable_record_without_contact_pairs(self):
        path = (
            ROOT
            / "docs/superpowers/evidence/grasp-lift-margin-20261102"
            / "fresh-out-of-cohort-probe-replay-v1-23020-20261102"
            / "PROBE_MEASUREMENT_SIDECAR.json"
        )
        sidecar = json.loads(path.read_text(encoding="utf-8"))

        with self.assertRaisesRegex(ReplayError, "incomplete contact trace"):
            verify_sidecar(sidecar)

    def test_rejects_unpaired_pose_wrench_sequence_even_with_valid_hashes(self):
        sidecar = load_sidecar(23024)
        mutated = copy.deepcopy(sidecar)
        measurement = mutated["measurements"][0]["measurement"]
        trace = measurement["motion_trace"]
        first, second = trace["samples"][:2]
        extra = copy.deepcopy(first)
        extra["simulator_step"] = first["simulator_step"] + 1
        extra["capture_s"] = (first["capture_s"] + second["capture_s"]) / 2.0
        extra["wrench_sequence"] = 2**63
        extra["position_world_m"] = [
            (float(first["position_world_m"][axis]) + float(second["position_world_m"][axis])) / 2.0
            for axis in range(3)
        ]
        extra["digest"] = _pose_sample_digest(extra)
        for pose in trace["samples"][1:]:
            pose["simulator_step"] += 1
            pose["digest"] = _pose_sample_digest(pose)
        trace["samples"].insert(1, extra)
        trace["digest"] = _trace_digest(trace)
        mutated["measurements"][0]["measurement_digest"] = _measurement_digest(measurement)

        with self.assertRaisesRegex(ReplayError, "wrench-pose alignment mismatch"):
            verify_sidecar(mutated)

    def test_structurally_verifies_runtime_contact_ambiguous_without_interval_replay(self):
        sidecar = load_sidecar(23024)
        mutated = copy.deepcopy(sidecar)
        record = mutated["measurements"][0]
        measurement = record["measurement"]
        measurement["runtime_contact_ambiguous"] = True
        record["inference"] = {"NoInference": "unexpected_contact"}
        record["posterior_support_friction_interval"] = list(
            measurement["prior_support_friction"]
        )
        record["measurement_digest"] = _measurement_digest(measurement)

        report = verify_sidecar(mutated)

        self.assertEqual(report["status"], "PASS_STRUCTURAL_AMBIGUOUS_CONTACT")
        self.assertFalse(report["interval_replay_supported"])
        self.assertEqual(
            report["measurements"][0]["inference"],
            "NoInference(UnexpectedContact)",
        )
        self.assertIsNone(report["measurements"][0]["first_xy_motion_bracket_s"])
        self.assertTrue(report["measurements"][0]["runtime_contact_ambiguous"])

    def test_rejects_ambiguous_contact_with_invented_updated_inference(self):
        sidecar = load_sidecar(23024)
        mutated = copy.deepcopy(sidecar)
        record = mutated["measurements"][0]
        measurement = record["measurement"]
        measurement["runtime_contact_ambiguous"] = True
        record["inference"] = {
            "Updated": {"interval": [0.2, 0.3], "mode": "slip_onset"}
        }
        record["measurement_digest"] = _measurement_digest(measurement)

        with self.assertRaisesRegex(ReplayError, "stored inference mismatch"):
            verify_sidecar(mutated)

    def test_sealed_21004_ambiguous_force_trace_structurally_passes_when_present(self):
        path = (
            ROOT
            / "docs/superpowers/evidence"
            / "rotated-probe-heldout-seed-21004-sidecar-capture-20261104"
            / "PROBE_MEASUREMENT_SIDECAR.json"
        )
        if not path.is_file():
            self.skipTest("gitignored 21004 sidecar not present locally")
        report = verify_sidecar(json.loads(path.read_text(encoding="utf-8")))
        self.assertEqual(report["status"], "PASS_STRUCTURAL_AMBIGUOUS_CONTACT")
        self.assertEqual(
            report["measurements"][0]["inference"],
            "NoInference(UnexpectedContact)",
        )


if __name__ == "__main__":
    unittest.main()
