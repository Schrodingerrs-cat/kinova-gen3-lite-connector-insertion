"""Offline regression tests. Source ros_env.sh, then python3 -m unittest discover -s ."""
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
from moveit_msgs.msg import MoveItErrorCodes

from arm import Arm
from robot_ros import JOINTS, Robot
from teleop_keys import jog_to


class TeleopTests(unittest.TestCase):
    def make_arm(self):
        robot = Mock(q=np.zeros(6))
        return Arm(robot, {"joint_speed_deg_s": 10}, np.eye(4), -np.inf, 0.81)

    def test_small_joint_motion_is_sent(self):
        arm = self.make_arm()
        result = jog_to(arm, None, np.array([0.1, 0, 0, 0, 0, 0]), False)
        arm.r.move_joints.assert_called_once()
        self.assertIn("completed", result)

    def test_identical_target_is_not_reported_as_motion(self):
        arm = self.make_arm()
        self.assertIn("no change", jog_to(arm, None, np.zeros(6), False))
        arm.r.move_joints.assert_not_called()

    def test_dry_does_not_move(self):
        arm = self.make_arm()
        self.assertIn("dry", jog_to(arm, None, np.ones(6), True))
        arm.r.move_joints.assert_not_called()

    def test_ik_reason_survives_to_ui(self):
        arm = self.make_arm()
        arm.r.ik.return_value = None
        arm.r.ik_error = "MoveIt IK: FRAME_TRANSFORM_FAILURE (-21)"
        result = jog_to(arm, np.eye(4), None, False)
        self.assertIn(arm.r.ik_error, result)
        arm.r.move_joints.assert_not_called()

    def test_controller_failure_is_not_an_ik_rejection(self):
        arm = self.make_arm()
        arm.r.move_joints.side_effect = RuntimeError("controller rejected trajectory")
        self.assertIn("EXECUTION FAILED", jog_to(arm, None, np.ones(6), False))

    def make_ik_robot(self, code):
        robot = Mock(q=np.zeros(6))
        robot.spin_until.return_value = True
        response = SimpleNamespace(error_code=SimpleNamespace(val=code), solution=SimpleNamespace(
            joint_state=SimpleNamespace(name=JOINTS, position=[0.1] * 6)))
        robot.ik_srv.call_async.return_value.result.return_value = response
        robot._pose = Robot._pose.__get__(robot)
        return robot

    def test_solver_error_code_and_retry(self):
        robot = self.make_ik_robot(MoveItErrorCodes.NO_IK_SOLUTION)
        self.assertIsNone(Robot.ik(robot, np.eye(4)))
        self.assertIn("NO_IK_SOLUTION", robot.ik_error)
        self.assertEqual(robot.ik_srv.call_async.call_count, 3)

    def test_service_timeout_cancels_pending_request(self):
        robot = self.make_ik_robot(MoveItErrorCodes.SUCCESS)
        robot.spin_until.return_value = False
        self.assertIsNone(Robot.ik(robot, np.eye(4)))
        robot.ik_srv.call_async.return_value.cancel.assert_called_once()
        self.assertIn("did not respond", robot.ik_error)

    def test_successful_ik(self):
        robot = self.make_ik_robot(MoveItErrorCodes.SUCCESS)
        np.testing.assert_allclose(Robot.ik(robot, np.eye(4)), np.degrees([0.1] * 6))
        self.assertEqual(robot.ik_error, "")


if __name__ == "__main__":
    unittest.main()
