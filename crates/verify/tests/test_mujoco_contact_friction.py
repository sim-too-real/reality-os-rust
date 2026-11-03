import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import mujoco_worker


class ScenarioContactFrictionTests(unittest.TestCase):
    def test_target_finger_pair_uses_scenario_sliding_friction_only(self):
        with tempfile.TemporaryDirectory() as directory:
            model_path = Path(directory) / "friction_fixture.xml"
            model_path.write_text(
                """<mujoco><option gravity="0 0 0"/><worldbody>
                  <body name="finger" pos="0 0 0">
                    <geom name="finger_geom" type="box" size=".05 .05 .05"/>
                  </body>
                  <body name="support" pos=".08 0 0">
                    <geom name="support_geom" type="box" size=".05 .05 .05"/>
                  </body>
                </worldbody></mujoco>""",
                encoding="utf-8",
            )
            scenario_object = {
                "name": "target",
                "type": "box",
                "pos": [0.04, 0.0, 0.0],
                "size": [0.03, 0.03, 0.03],
                "friction": 0.4,
                "movable": True,
                "tool_object_friction_bodies": ["finger"],
            }
            instance = mujoco_worker.Instance()
            with patch.object(mujoco_worker, "INST", instance):
                result = mujoco_worker._load(
                    {
                        "format": "mjcf",
                        "path": str(model_path),
                        "objects": [scenario_object],
                    }
                )

                self.assertTrue(result["ok"], result)
                contacts = result["state"]["contacts"]
                target_finger = [
                    contact
                    for contact in contacts
                    if {contact["body1"], contact["body2"]} == {"target", "finger"}
                ]
                target_support = [
                    contact
                    for contact in contacts
                    if {contact["body1"], contact["body2"]} == {"target", "support"}
                ]

                self.assertTrue(target_finger, "expected target–finger contact")
                self.assertTrue(target_support, "expected target–support contact")
                for contact in target_finger:
                    self.assertEqual(contact["dim"], 3)
                    self.assertEqual(contact["friction"][:2], [0.4, 0.4])
                    self.assertAlmostEqual(contact["friction"][2], 0.01)
                for contact in target_support:
                    self.assertEqual(contact["friction"][:2], [1.0, 1.0])
                    self.assertAlmostEqual(contact["friction"][2], 0.01)

                update = mujoco_worker.handle(
                    {"cmd": "configure_body", "body": "target", "friction": 0.6}
                )
                self.assertTrue(update["ok"], update)
                updated_target_finger = [
                    contact
                    for contact in update["state"]["contacts"]
                    if {contact["body1"], contact["body2"]} == {"target", "finger"}
                ]
                self.assertTrue(updated_target_finger)
                for contact in updated_target_finger:
                    self.assertEqual(contact["friction"][:2], [0.6, 0.6])
                    self.assertAlmostEqual(contact["friction"][2], 0.01)

                finger_update = mujoco_worker.handle(
                    {"cmd": "configure_body", "body": "finger", "friction": 0.2}
                )
                self.assertTrue(finger_update["ok"], finger_update)
                still_target_friction = [
                    contact
                    for contact in finger_update["state"]["contacts"]
                    if {contact["body1"], contact["body2"]} == {"target", "finger"}
                ]
                self.assertTrue(still_target_friction)
                self.assertTrue(
                    all(contact["friction"][:2] == [0.6, 0.6] for contact in still_target_friction)
                )

    def test_missing_finger_body_rejects_the_worker_load(self):
        with tempfile.TemporaryDirectory() as directory:
            model_path = Path(directory) / "friction_fixture.xml"
            model_path.write_text(
                """<mujoco><worldbody>
                  <body name="finger"><geom type="box" size=".05 .05 .05"/></body>
                </worldbody></mujoco>""",
                encoding="utf-8",
            )
            scenario_object = {
                "name": "target",
                "friction": 0.4,
                "tool_object_friction_bodies": ["missing_finger"],
            }
            with patch.object(mujoco_worker, "INST", mujoco_worker.Instance()):
                result = mujoco_worker._load(
                    {
                        "format": "mjcf",
                        "path": str(model_path),
                        "objects": [scenario_object],
                    }
                )
            self.assertFalse(result["ok"])
            self.assertIn("scenario_tool_object_friction_unknown_finger_body", result["error"])

    def test_pair_override_preserves_a_nondefault_contact_dimension(self):
        with tempfile.TemporaryDirectory() as directory:
            model_path = Path(directory) / "friction_condim_fixture.xml"
            model_path.write_text(
                """<mujoco><option gravity="0 0 0"/><worldbody>
                  <body name="finger"><geom type="box" size=".05 .05 .05" condim="4"/></body>
                </worldbody></mujoco>""",
                encoding="utf-8",
            )
            scenario_object = {
                "name": "target",
                "type": "box",
                "pos": [0.04, 0.0, 0.0],
                "friction": 0.4,
                "tool_object_friction_bodies": ["finger"],
            }
            with patch.object(mujoco_worker, "INST", mujoco_worker.Instance()):
                result = mujoco_worker._load(
                    {
                        "format": "mjcf",
                        "path": str(model_path),
                        "objects": [scenario_object],
                    }
                )
            self.assertTrue(result["ok"], result)
            contacts = [
                contact
                for contact in result["state"]["contacts"]
                if {contact["body1"], contact["body2"]} == {"target", "finger"}
            ]
            self.assertTrue(contacts)
            self.assertTrue(all(contact["dim"] == 4 for contact in contacts))


if __name__ == "__main__":
    unittest.main()
