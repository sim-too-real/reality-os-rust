import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import mujoco

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import mujoco_worker


class ScenarioContactFrictionTests(unittest.TestCase):
    def test_pair_name_allocator_skips_pairs_already_created_in_the_same_load(self):
        spec = mujoco_worker.mujoco.MjSpec()
        self.assertEqual(
            mujoco_worker._unique_spec_name(
                spec,
                "pair",
                "scenario_tool_friction_pair",
                {"scenario_tool_friction_pair_0"},
            ),
            "scenario_tool_friction_pair_1",
        )

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

    def test_actual_two_finger_gripper_compiles_two_target_friction_pairs(self):
        repo_root = Path(__file__).resolve().parents[3]
        model_path = repo_root / "robots/bundles/arm_gripper_lift/model.xml"
        spec = mujoco_worker._load_spec(str(model_path), {})
        friction = 0.7
        worker_objects = [
            {
                "name": "target",
                "type": "box",
                "pos": [0.25, 0.0, 0.12],
                "size": [0.01, 0.01, 0.02],
                "mass": 0.2,
                "movable": True,
                "friction": friction,
                "tool_object_friction_bodies": ["finger_l", "finger_r"],
            }
        ]
        mujoco_worker._add_scenario_objects(spec, worker_objects)
        model = spec.compile()
        target_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "target")
        target_geom_ids = {
            geom_id
            for geom_id in range(model.ngeom)
            if int(model.geom_bodyid[geom_id]) == target_body
        }
        target_pair_ids = [
            pair_id
            for pair_id in range(model.npair)
            if int(model.pair_geom1[pair_id]) in target_geom_ids
            or int(model.pair_geom2[pair_id]) in target_geom_ids
        ]
        finger_names = set()
        for pair_id in target_pair_ids:
            geom_ids = [int(model.pair_geom1[pair_id]), int(model.pair_geom2[pair_id])]
            pair_bodies = {
                model.body(int(model.geom_bodyid[geom_id])).name for geom_id in geom_ids
            }
            finger_names.update(pair_bodies - {"target"})
            self.assertEqual(
                list(model.pair_friction[pair_id]),
                [friction, friction, 0.01, 0.01, 0.01],
            )

        self.assertEqual(len(target_pair_ids), 2)
        self.assertEqual(finger_names, {"finger_l", "finger_r"})


if __name__ == "__main__":
    unittest.main()
