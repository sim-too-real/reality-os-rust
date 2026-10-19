import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import mujoco

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import mujoco_worker
from mujoco_worker import (
    _actuator_force_limit_reached,
    _has_unexpected_sensor_subtree_contact,
    _sensor_subtree_actuator_saturations,
    _sensor_subtree_body_ids,
    _sensor_contact_pair_is_expected,
)


class MonitoredWrenchGateTests(unittest.TestCase):
    def test_force_limited_actuator_is_flagged_at_either_force_bound(self):
        self.assertFalse(_actuator_force_limit_reached(0.0, True, (-8.0, 8.0)))
        self.assertTrue(_actuator_force_limit_reached(-8.0, True, (-8.0, 8.0)))
        self.assertTrue(_actuator_force_limit_reached(8.0, True, (-8.0, 8.0)))
        self.assertTrue(_actuator_force_limit_reached(8.0001, True, (-8.0, 8.0)))
        self.assertFalse(_actuator_force_limit_reached(8.0, False, (-8.0, 8.0)))

    def test_only_contact_with_the_declared_target_is_admissible(self):
        tool_subtree = {3, 4, 5}
        self.assertTrue(_sensor_contact_pair_is_expected(4, 12, tool_subtree, 12))
        self.assertTrue(_sensor_contact_pair_is_expected(12, 5, tool_subtree, 12))
        self.assertFalse(_sensor_contact_pair_is_expected(4, 13, tool_subtree, 12))
        self.assertFalse(_sensor_contact_pair_is_expected(3, 4, tool_subtree, 12))
        self.assertTrue(_sensor_contact_pair_is_expected(12, 13, tool_subtree, 12))

    def test_mujoco_extra_tool_contact_is_detected_from_runtime_contacts(self):
        model = mujoco.MjModel.from_xml_string(
            """
            <mujoco><option gravity="0 0 0"/><worldbody>
              <body name="tool" pos="0 0 0"><freejoint/>
                <geom name="tool_geom" type="box" size=".05 .05 .05"/>
                <site name="ft"/>
              </body>
              <body name="target" pos=".08 0 0"><freejoint/>
                <geom name="target_geom" type="box" size=".05 .05 .05"/>
              </body>
              <body name="obstacle" pos="0 .08 0"><freejoint/>
                <geom name="obstacle_geom" type="box" size=".05 .05 .05"/>
              </body>
            </worldbody><sensor>
              <force name="f" site="ft"/><torque name="t" site="ft"/>
            </sensor></mujoco>
            """
        )
        data = mujoco.MjData(model)
        force_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "f")
        torque_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "t")
        sensor_ids = (force_id, torque_id)
        target_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "target")
        obstacle_geom_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_GEOM, "obstacle_geom"
        )

        with patch.object(mujoco_worker, "INST", SimpleNamespace(model=model, data=data)):
            subtree = _sensor_subtree_body_ids(sensor_ids)
            model.geom_contype[obstacle_geom_id] = 0
            model.geom_conaffinity[obstacle_geom_id] = 0
            mujoco.mj_forward(model, data)
            self.assertFalse(
                _has_unexpected_sensor_subtree_contact(subtree, target_id)
            )

            model.geom_contype[obstacle_geom_id] = 1
            model.geom_conaffinity[obstacle_geom_id] = 1
            mujoco.mj_forward(model, data)
            self.assertTrue(
                _has_unexpected_sensor_subtree_contact(subtree, target_id)
            )

    def test_mujoco_force_limited_actuator_saturation_is_detected(self):
        model = mujoco.MjModel.from_xml_string(
            """
            <mujoco><option gravity="0 0 0"/><worldbody>
              <body name="robot" pos="0 0 0"><joint name="hinge" type="hinge"
                axis="0 1 0" range="-2 2" limited="true"/>
                <geom type="box" size=".05 .05 .05"/><site name="ft"/>
              </body>
            </worldbody><actuator>
              <position name="motor" joint="hinge" kp="100" ctrlrange="-2 2"
                forcerange="-.1 .1"/>
            </actuator><sensor>
              <force name="f" site="ft"/><torque name="t" site="ft"/>
            </sensor></mujoco>
            """
        )
        data = mujoco.MjData(model)
        data.ctrl[0] = 1.0
        mujoco.mj_forward(model, data)
        force_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "f")
        torque_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "t")
        with patch.object(mujoco_worker, "INST", SimpleNamespace(model=model, data=data)):
            self.assertEqual(
                _sensor_subtree_actuator_saturations((force_id, torque_id)), ["motor"]
            )

    def test_force_limited_unsupported_transmission_fails_closed(self):
        model = mujoco.MjModel.from_xml_string(
            """
            <mujoco><option gravity="0 0 0"/><worldbody>
              <body name="robot"><joint name="hinge" type="hinge"/>
                <geom type="sphere" size=".03"/><site name="ft"/>
              </body>
            </worldbody><tendon><fixed name="tendon">
              <joint joint="hinge" coef="1"/>
            </fixed></tendon><actuator>
              <motor name="tendon_motor" tendon="tendon" forcelimited="true"
                forcerange="-1 1"/>
            </actuator><sensor>
              <force name="f" site="ft"/><torque name="t" site="ft"/>
            </sensor></mujoco>
            """
        )
        data = mujoco.MjData(model)
        force_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "f")
        torque_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "t")
        with patch.object(mujoco_worker, "INST", SimpleNamespace(model=model, data=data)):
            with self.assertRaisesRegex(
                ValueError, "wrench_force_limited_actuator_transmission_unsupported"
            ):
                _sensor_subtree_actuator_saturations((force_id, torque_id))


if __name__ == "__main__":
    unittest.main()
